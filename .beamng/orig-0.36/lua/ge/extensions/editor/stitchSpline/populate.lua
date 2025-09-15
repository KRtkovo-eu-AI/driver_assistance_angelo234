-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local toolPrefixStr = 'Stitch Spline' -- The global prefix for the stitch spline tool.

local anchorPrefix = 'anchorPoint_'

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module dependencies.
local geom = require('editor/toolUtilities/geom')
local util = require('editor/toolUtilities/util')

-- Module constants.
local poleScaleVec = vec3(1.0, 1.0, 1.0)
local up = vec3(0, 0, 1)
local rot = quat()
local root2Over2 = math.sqrt(2) * 0.5
local rot_q270 = quat(0, 0, -root2Over2, root2Over2)

-- Module state.
local staticMeshes = {}
local finalPosns, finalTans, finalNormals = {}, {}, {}
local anchors, anchorPointNames = {}, {}
local tmpScale, rightVec, tmpTangent = vec3(), vec3(), vec3()
local tmp1, tmp2, tmp3, tmp4 = vec3(), vec3(), vec3(), vec3()


-- Manages the pools of static meshes, per component type, to match the count of finalPosns.
local function manageMeshPools(spline, numAnchors, splineIdx)
  -- Ensure the stitch spline has a scene tree folder.
  -- [If it doesn't exist (maybe user deleted it), then create a new one.]
  local folder = scenetree.findObjectById(spline.sceneTreeFolderId)
  if not folder then
    local baseName = string.format(toolPrefixStr .. " %d", splineIdx)
    local uniqueName = util.generateUniqueName(baseName, toolPrefixStr)
    folder = createObject("SimGroup")
    folder:registerObject(string.format("%s - %s", uniqueName, spline.id))
    folder.cansave = false
    scenetree.MissionGroup:addObject(folder)
    spline.sceneTreeFolderId = folder:getId()
    table.clear(staticMeshes[spline.id])
  end

  local splineName, splineId = spline.name, spline.id
  staticMeshes[splineId] = staticMeshes[splineId] or {}
  local meshesBySpline = staticMeshes[splineId]

  -- Manage the pool of static meshes for the poles.
  local poleId = spline.poleMeshPath
  meshesBySpline[poleId] = meshesBySpline[poleId] or {}
  local poleMeshes = meshesBySpline[poleId]
  local numPolesNeeded, numPolesInPool = #finalPosns, #poleMeshes
  for i = numPolesInPool + 1, numPolesNeeded do -- Add any extra poles which may be needed.
    local meshId = string.format('%s_Pole_%d', splineName, i)
    local mesh = createObject('TSStatic')
    mesh.cansave = false
    mesh:setField('shapeName', 0, poleId)
    mesh:registerObject(meshId)
    mesh.scale = poleScaleVec
    folder:addObject(mesh.obj) -- Add the mesh to the spline's scene tree folder.
    poleMeshes[i] = mesh
  end
  for i = numPolesInPool, numPolesNeeded + 1, -1 do -- Remove any excess poles which we no longer need.
    local mesh = poleMeshes[i]
    if mesh and simObjectExists(mesh) then
      mesh:delete()
    end
    poleMeshes[i] = nil
  end

  -- Manage the pool of static meshes for the wires.
  local wireId = spline.wireMeshPath
  meshesBySpline[wireId] = meshesBySpline[wireId] or {}
  local wireMeshes = meshesBySpline[wireId]
  local numWiresNeeded, numWiresInPool = (numPolesNeeded - 1) * numAnchors, #wireMeshes
  for i = numWiresInPool + 1, numWiresNeeded do -- Add any extra wires which may be needed.
    local meshId = string.format('%s_Wire_%d', splineName, i)
    local mesh = createObject('TSStatic')
    mesh.cansave = false
    mesh:setField('shapeName', 0, wireId)
    mesh:registerObject(meshId)
    folder:addObject(mesh.obj) -- Add the mesh to the spline's scene tree folder.
    wireMeshes[i] = mesh
  end
  for i = numWiresInPool, numWiresNeeded + 1, -1 do -- Remove any excess wires which we no longer need.
    local mesh = wireMeshes[i]
    if mesh and simObjectExists(mesh) then
      mesh:delete()
    end
    wireMeshes[i] = nil
  end
end

-- Gets the anchor points of a pole component.
local function getAnchorPoints(staticMesh, anchorPointNames, pos, tgt, binormal, jitterQuat)
  local anchorsInPole = {}
  for i = 1, #anchorPointNames do
    local isExist, anchorPos, _, _, _, _ = staticMesh:getNodeTransform(anchorPointNames[i])
    if not isExist then
      break
    end
    anchorPos = jitterQuat * anchorPos -- Apply the same jitter to the anchor point position, which was applied to the pole component.
    tmp1:set(anchorPos.y * tgt.x, anchorPos.y * tgt.y, anchorPos.y * tgt.z) -- Use set methods to form the linear combination.
    tmp2:set(anchorPos.x * binormal.x, anchorPos.x * binormal.y, anchorPos.x * binormal.z)
    tmp3:set(anchorPos.z * up.x, anchorPos.z * up.y, anchorPos.z * up.z)
    tmp4:set(tmp1.x + tmp2.x + tmp3.x, tmp1.y + tmp2.y + tmp3.y, tmp1.z + tmp2.z + tmp3.z)
    anchorsInPole[i] = pos + tmp4 -- The anchor point position in the pole component's local space.
  end
  return anchorsInPole
end

-- Computes the anchor point names from the static mesh with the given path.
local function computeAnchorPointNames(path)
  -- Create a temporary mesh.
  local tmpMesh = createObject('TSStatic')
  tmpMesh.cansave = false
  tmpMesh:setField('shapeName', 0, path)
  tmpMesh:registerObject('tmp_pole')

  -- Get the anchor point names from the temporary mesh.
  table.clear(anchorPointNames)
  local ctr = 1
  while true do
    local testName = string.format('%s%d', anchorPrefix, ctr)
    local isSuccess, _, _, _, _, _ = tmpMesh:getNodeTransform(testName)
    if not isSuccess then
      break
    end
    anchorPointNames[ctr] = testName
    ctr = ctr + 1
  end

  -- Delete the temporary mesh.
  if tmpMesh and simObjectExists(tmpMesh) then
    tmpMesh:delete()
  end
end

-- Populates a stitch spline with instances of .
local function populateStitchSpline(spline, splineIdx)
  local length = spline.pole_extentsL_Center
  if not length or not spline.poleMeshPath or #spline.divPoints < 2 or #spline.nodes < 2 then
    return
  end

  -- Sample along true arc-length to get the positions, tangents, and normals for each pole component.
  geom.sampleSpline(spline.divPoints, spline.tangents, spline.normals, length + spline.spacing, finalPosns, finalTans, finalNormals)

  -- Get the anchor point names from the chosen pole mesh.
  local poleId = spline.poleMeshPath
  computeAnchorPointNames(poleId)

  -- Manage the pools of static meshes to ensure that there is the correct number of instances to populate the stitch spline.
  manageMeshPools(spline, #anchorPointNames, splineIdx)

  -- Create the static meshes for each pole component on the stitch spline.
  table.clear(anchors)
  local splineId = spline.id
  for i = 1, #finalPosns do
    -- Compute the position and Frenet frame for this pole component.
    local pos = finalPosns[i]
    local tangent, normal = finalTans[i], finalNormals[i]
    rightVec:setCross(tangent, normal)

    -- Compute a jitter quaternion for this pole component.
    local jitterQuat = geom.computeRandomJitterQuat(spline, tangent, rightVec, normal)

    -- Compute the full rotation for this pole component.
    tangent.z = 0 -- Ensure the pole is always fully upright.
    rot:setFromDir(tangent, up)
    rot = jitterQuat * rot -- Apply any requested random jittering to the pole component.

    -- Update the static mesh object for this pole component.
    local poleMesh = staticMeshes[splineId][poleId][i]
    poleMesh:setPosRot(pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w)

    -- Cache all the anchor points of this pole component.
    anchors[i] = getAnchorPoints(poleMesh, anchorPointNames, pos, tangent, rightVec, jitterQuat)
  end

  -- Create the wire mesh objects, using the anchor points of the pole components.
  local wireId, ctr = spline.wireMeshPath, 1
  local extentsLInv, extentsZInv = 1.0 / spline.wire_extentsL_Center, 1.0 / spline.wire_extentsZ_Center
  for i = 1, #finalPosns - 1 do
    local anchors1, anchors2 = anchors[i], anchors[i + 1]
    for j = 1, #anchors1 do
      -- Compute the rotation for this wire component.
      local p1, p2 = anchors1[j], anchors2[j]
      tmpTangent:set(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
      tmpTangent:normalize()
      rot:setFromDir(tmpTangent, up)
      rot = rot_q270 * rot

      -- Update the static mesh object for this wire component.
      local wireMesh = staticMeshes[splineId][wireId][ctr]
      ctr = ctr + 1
      wireMesh:setPosRot(p1.x, p1.y, p1.z, rot.x, rot.y, rot.z, rot.w)
      local xScaling = p1:distance(p2) * extentsLInv -- Scale along length so the wire fits pole-to-pole.
      local zScaling = spline.sag * extentsZInv -- Scale along height so the wire has the desired sag.
      tmpScale:set(xScaling, 1.0, zScaling)
      wireMesh.scale = tmpScale
    end
  end
end

-- Attempts to removes the static meshes of the stitch spline, from the scene.
local function tryRemove(spline)
  local splineId = spline.id
  local meshesBySpline = staticMeshes[splineId]
  if meshesBySpline then
    for _, v in pairs(meshesBySpline) do
      local numMeshesByShape = #v
      for i = 1, numMeshesByShape do
        local mesh = v[i]
        if mesh and simObjectExists(mesh) then
          mesh:delete()
        end
      end
    end
    staticMeshes[splineId] = nil
  end
end

-- Set the cansave flag for the static meshes of the given stitch spline.
local function setCansave(spline)
  for _, v in pairs(staticMeshes[spline.id]) do
    for _, mesh in ipairs(v) do
      mesh.cansave = true
    end
  end
end


-- Public interface.
M.populateStitchSpline =                                populateStitchSpline
M.tryRemove =                                           tryRemove

M.setCansave =                                          setCansave

return M