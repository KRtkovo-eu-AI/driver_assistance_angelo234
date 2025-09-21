-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- External modules used.
local util = require('editor/tech/roadArchitect/utilities')                                                 -- The Road Architect utilities module.

-- Private constants.
local ceil, min, max, abs, sqrt = math.ceil, math.min, math.max, math.abs, math.sqrt
local random, randomseed = math.random, math.randomseed

-- Module state.
local spanMeshes = {}                                                                                       -- The collection of lane-spanning meshes.
local singleMeshes = {}                                                                                     -- The collection of single unit (patch) meshes.

-- Module constants.
local scaleVec = vec3(1, 1, 1)                                                                              -- A vec3 used for representing uniform scale.
local vertical = vec3(0, 0, 1)                                                                              -- A vec3 used to represent the global world space 'up' axis.
local downVec = -vertical
local raised = vertical * 4.5

local root2Over2 = sqrt(2) * 0.5
local rot_q0 = quat(0, 0, 0, 1)                                                                             -- Some common rotations (as quaternions).
local rot_q90 = quat(0, 0, root2Over2, root2Over2)
local rot_q180 = quat(0, 0, 1, 0)
local rot_q270 = quat(0, 0, -root2Over2, root2Over2)


-- Computes the length of the given polyline.
local function computePolylineLengths(posns)
  local lens = { 0.0 }
  for i = 2, #posns do
    local iMinus1 = i - 1
    lens[i] = lens[iMinus1] + posns[iMinus1]:distance(posns[i])
  end
  return lens
end

-- Populates a road with instances of a static mesh along its length.
local function createSelectedMesh(r, roadIdx, layer, folder)
  randomseed(41225)

  local displayName = layer.matDisplay
  local meshPath, length = layer.mat, layer.extentsL
  local vertOffset = layer.vertOffset[0]
  local latOffset = layer.latOffset[0]
  local lIdx = layer.lane[0]
  local pIdx = 2
  if layer.isLeft[0] then
    pIdx = 1
  end
  local preRot = rot_q0
  local isCentered = abs(layer.boxXLeft - layer.boxXRight) < 0.1 and max(layer.boxXLeft, layer.boxXRight) > 1.0
  if layer.rot[0] == 1 then
    preRot = rot_q90
    length = layer.extentsW
    isCentered = abs(layer.boxYLeft - layer.boxYRight) < 0.1 and max(layer.boxYLeft, layer.boxYRight) > 1.0
  elseif layer.rot[0] == 2 then
    preRot = rot_q180
  elseif layer.rot[0] == 3 then
    preRot = rot_q270
    length = layer.extentsW
    isCentered = abs(layer.boxYLeft - layer.boxYRight) < 0.1 and max(layer.boxYLeft, layer.boxYRight) > 1.0
  end
  local extraSpacing = layer.spacing[0]
  local jitter = layer.jitter[0]
  local isGlobalZ = layer.useWorldZ[0]

  if not length then
    return
  end

  local rData = r.renderData
  local startDivIdx, endDivIdx = 1, #rData
  if not layer.isSpanLong[0] then
    startDivIdx = util.computeDivIndicesFromNode(layer.nMin[0], r)
    endDivIdx = util.computeDivIndicesFromNode(layer.nMax[0], r)
  end

  local posns, nmls, ctr = {}, {}, 1
  for j = startDivIdx, endDivIdx do
    local lData = rData[j][lIdx]
    local tgt = rData[min(#rData, j + 1)][lIdx][1] - rData[max(1, j - 1)][lIdx][1]
    tgt:normalize()
    local lat = lData[6]
    local nml = tgt:cross(lat)
    local orig = lData[pIdx] + raised
    orig.z = orig.z - castRayStatic(orig, downVec, 1000) + 0.02
    posns[ctr] = orig + (vertOffset * nml) + (latOffset * lat)
    nmls[ctr] = nml
    ctr = ctr + 1
  end

  -- Compute the positions and normal vectors for each mesh unit on the lane.
  local lens = computePolylineLengths(posns)
  local numMeshes = ceil(lens[#lens] / (length + extraSpacing))
  local q = 0.0
  local fPosns, fNmls = {}, {}
  local ctr = 1
  for _ = 1, numMeshes do
    local pos, nml = util.polyLerp(posns, nmls, lens, q)
    if pos then
      fPosns[ctr], fNmls[ctr] = pos, nml
      ctr = ctr + 1
    end
    q = q + length + extraSpacing
  end

  -- Create the static meshes for each mesh unit on the lane.
  spanMeshes[r.name] = spanMeshes[r.name] or {}
  local thisMesh = spanMeshes[r.name]
  local q = 0.0
  numMeshes = #fPosns
  for j = 1, numMeshes do
    local p1, p2 = fPosns[j], fPosns[j + 1]
    local posOrig = p1
    if j == numMeshes then
      p1, p2 = fPosns[j - 1], fPosns[j]
      posOrig = p2
    end
    if p1 and p2 then
      local v = p2 - p1
      local pos = nil
      if isCentered then                                                                            -- If the static mesh has its origin at the center of mesh.
        pos = posOrig + v * 0.5
      else
        pos = posOrig
      end
      local tgt = v:normalized()

      local rot = quat()
      if isGlobalZ then
        tgt.z = 0.0
        rot:setFromDir(tgt, vertical)
      else
        rot:setFromDir(tgt, fNmls[j])
      end
      rot = preRot * rot_q90 * rot

      rot.x = rot.x + (random() * 2 - 1) * jitter                                                   -- Apply any random jittering, if requested.
      rot.y = rot.y + (random() * 2 - 1) * jitter
      rot.z = rot.z + (random() * 2 - 1) * jitter

      local static = createObject('TSStatic')
      static:setField('shapeName', 0, meshPath)
      static:setField('decalType', 0, 'None')
      local meshId = displayName .. ' ' .. tostring(roadIdx) .. '-' .. tostring(lIdx) .. '-' .. tostring(j)
      static:registerObject(meshId)
      static:setPosRot(pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w)
      static.scale = scaleVec
      static.canSave = true
      folder:addObject(static.obj)
      thisMesh[meshId] = static
    end
    q = q + length + extraSpacing
  end
end

-- Create a single mesh unit on this road, using the given layer parameters.
local function createSingleMeshUnit(r, roadIdx, layer, folder, sIdx)

  -- If no material has been selected for this single mesh unit, leave immediately.
  if not layer.mat or layer.mat == '' then
    return
  end

  -- Compute the position on the road.
  local rData = r.renderData
  local lIdx = layer.lane[0]
  local lengths = util.computeRoadLength(rData)
  local pEval = layer.pos[0] * lengths[#lengths]                                                    -- The longitudinal evaluation position on the road, in meters.
  local lower, upper = util.findBounds(pEval, lengths)
  local q = (pEval - lengths[lower]) / (lengths[upper] - lengths[lower])                            -- The q in [0, 1] between div points (linear interpolation).
  local pL = nil
  if layer.isLeft[0] then
    local pL1 = rData[lower][lIdx][1]
    local pL2 = rData[upper][lIdx][1]
    pL = pL1 + q * (pL2 - pL1)
  else
    local pL1 = rData[lower][lIdx][2]
    local pL2 = rData[upper][lIdx][2]
    pL = pL1 + q * (pL2 - pL1)
  end
  if abs(layer.vertOffset[0]) < 0.001 then
    pL = pL + raised
    pL.z = pL.z - castRayStatic(pL, downVec, 1000) + 0.02
  end
  local n1 = rData[lower][lIdx][5]
  local n2 = rData[upper][lIdx][5]
  local nml = n1 + q * (n2 - n1)
  local l1 = rData[lower][lIdx][6]
  local l2 = rData[upper][lIdx][6]
  local lat = l1 + q * (l2 - l1)
  local pos = pL + nml * (layer.vertOffset[0] + 0.001) + lat * layer.latOffset[0]

  -- Compute the rotation.
  local preRot = rot_q0
  local lRot = layer.rot[0]
  if lRot == 1 then
    preRot = rot_q90
  elseif lRot == 2 then
    preRot = rot_q180
  elseif lRot == 3 then
    preRot = rot_q270
  end
  local rot = quat()
  local tgt = lat:cross(nml)
  rot:setFromDir(tgt, nml)
  rot = preRot * rot_q90 * rot

  singleMeshes[r.name] = singleMeshes[r.name] or {}
  local thisMesh = singleMeshes[r.name]
  local static = createObject('TSStatic')
  static:setField('shapeName', 0, layer.mat)
  static:setField('decalType', 0, 'None')
  local meshId = layer.matDisplay .. ' ' .. tostring(roadIdx) .. '- single -' .. tostring(sIdx)
  static:registerObject(meshId)
  static:setPosRot(pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w)
  static.scale = scaleVec
  static.canSave = true
  folder:addObject(static.obj)
  thisMesh[meshId] = static
end

-- Creates all the appropriate static meshes for a given road.
local function createStaticMeshes(r, roadIdx, folder)
  local profile = r.profile
  local layers = profile.layers
  if layers then
    for i = 1, #layers do
      local layer = layers[i]
      local layerType = layer.type[0]
      if layerType == 4 then
        createSelectedMesh(r, roadIdx, layer, folder)
      elseif layerType == 5 then
        createSingleMeshUnit(r, roadIdx, layer, folder, i)
      end
    end
  end
end

-- Attempts to removes the meshes of the road with the given name, from the scene (if it exists).
-- [This is done through road indices; the actual handling of the meshes structure should be private to this module].
local function tryRemove(roadName)
  if spanMeshes[roadName] then
    for _, v in pairs(spanMeshes[roadName]) do
      v:delete()
    end
  end
  spanMeshes[roadName] = nil

  if singleMeshes[roadName] then
    for _, v in pairs(singleMeshes[roadName]) do
      v:delete()
    end
  end
  singleMeshes[roadName] = nil
end


-- Public interface.
M.createStaticMeshes =                                    createStaticMeshes

M.tryRemove =                                             tryRemove

return M