-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local auditionHeight = 1000.0                                                                       -- The height above zero, at which the static meshes are auditioned, in metres.
local camRotInc = math.pi / 500                                                                     -- The step size of the angle when rotating the camera around the audition center.
local auditionPlanarDistFac = 1.6                                                                   -- A factor used for determining the audition camera planar distance.
local auditionElevationFac = 0.8                                                                    -- A factor used for determining the audition camera elevation.

local staticMeshPaths = {                                                                           -- The paths of the common static meshes, used for searching on init.
  'art/shapes/objects',
  'art/shapes/garage_and_dealership/Clutter' }

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- External modules used.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.

-- Private constants.
local sin, cos = math.sin, math.cos
local twoPi = math.pi * 2.0
local gView, auditionVec, auditionCamPos = vec3(0, 0), vec3(0, 0, auditionHeight), vec3(0, 0)
local scaleVec = vec3(1, 1, 1)                                                                      -- A vec3 used for representing uniform scale.
local camRotAngle = 0.0
local meshRad = 10.0                                                                                -- The default distance between the mesh and the camera, in meters.

-- Module state.
local auditionMesh = nil                                                                            -- The mesh under audition.
local isAuditionMeshLive = false                                                                    -- A flag which indicates if the audition mesh exists, or not.
local availStaticMeshes = {}                                                                        -- The collection of available static meshes.
local oldPos, oldRot = nil, nil                                                                     -- The previous camera pose, before going to profile view.
local isInMeshView = false                                                                          -- A flag which indicates if the camera is in static mesh audition view, or not.


-- Fetch the list of available static meshes.
local function fetchAvailableStaticMeshes()
  table.clear(availStaticMeshes)
  local ctr = 1
  for j = 1, #staticMeshPaths do
    local meshPaths = FS:findFiles(staticMeshPaths[j], "*.dae", -1, true, false)
    for i = 1, #meshPaths do
      availStaticMeshes[ctr] = { path = meshPaths[i], filename = util.getFilenameFromPath(meshPaths[i]) }
      ctr = ctr + 1
    end
  end
  table.sort(availStaticMeshes, function(a, b) return a.filename < b.filename end)
end

-- Removes the static mesh under audition.
local function removeAuditionMesh()
  if isAuditionMeshLive then
    auditionMesh:delete()
    isAuditionMeshLive = false
  end
  auditionMesh = nil
end

-- Updates the camera position upon changing of selected mesh.
local function updateCameraPose()
  gView:set(0.0, -meshRad * auditionPlanarDistFac, auditionHeight + meshRad * auditionElevationFac)
  local gRot = quatFromDir(auditionVec - gView)
  commands.setFreeCamera()
  core_camera.setPosRot(0, gView.x, gView.y, gView.z, gRot.x, gRot.y, gRot.z, gRot.w)
end

-- Add/replace the mesh under audition with the mesh with the given index.
local function addMeshToAudition(selectedMeshIdx, road, customIdx)
  removeAuditionMesh()
  isAuditionMeshLive = true
  auditionMesh = createObject('TSStatic')
  auditionMesh:setField('shapeName', 0, availStaticMeshes[selectedMeshIdx].path)
  auditionMesh:setField('decalType', 0, 'None')
  auditionMesh:registerObject('Mesh Under Audition [temporary]')
  scenetree.MissionGroup:addObject(auditionMesh)
  auditionMesh:setPosRot(0.0, 0.0, auditionHeight, 0, 0, 0, 1)
  auditionMesh.scale = scaleVec

  -- Update the camera distance, based on the size of the newly-selected mesh.
  local box = auditionMesh:getObjBox()
  meshRad = 1.2 * box:getLength()
  if isInMeshView then
    updateCameraPose()
  end

  -- Set the mesh box data in the layer.
  local worldBox = auditionMesh:getWorldBox()
  local center = auditionMesh:getPosition()
  local minExtents = worldBox.minExtents
  local maxExtents = worldBox.maxExtents
  local lay = road.profile.layers[customIdx]
  lay.boxXLeft = center.x - minExtents.x
  lay.boxXRight = maxExtents.x - center.x
  lay.boxYLeft = center.y - minExtents.y
  lay.boxYRight = maxExtents.y - center.y
  lay.boxZLeft = center.z - minExtents.z
  lay.boxZRight = maxExtents.z - center.z
  local extents = box:getExtents()
  lay.extentsL = extents.x
  lay.extentsW = extents.y
  lay.extentsH = extents.z
end

-- Rotate camera around the audition centroid.
local function rotateCamera(ang)
  local x, y, s, c = gView.x, gView.y, sin(ang), cos(ang)
  auditionCamPos:set(x * c - y * s, x * s + y * c, gView.z)
  local gRot = quatFromDir(auditionVec - auditionCamPos)
  core_camera.setPosRot(0, auditionCamPos.x, auditionCamPos.y, auditionCamPos.z, gRot.x, gRot.y, gRot.z, gRot.w)
end

-- Moves the camera to the mesh audition preview pose.
-- [Also adjusts the timing parameters respectively].
local function goToMeshView(timer, time)
  if not isInMeshView then
    time, isInMeshView = 0.0, true
    timer:stopAndReset()
    oldPos, oldRot = core_camera.getPosition(), core_camera.getQuat()                               -- Store the current camera position so we can return to it later.
    updateCameraPose()
  end
  return time
end

-- Manages the rotation of the audition camera.
local function manageRotateCam()
  rotateCamera(camRotAngle)
  camRotAngle = camRotAngle + camRotInc
  if camRotAngle > twoPi then
    camRotAngle = camRotAngle - twoPi
  end
end

-- Returns the camera to the stored old view.
local function goToOldView()
  if oldPos and oldRot then
    core_camera.setPosRot(0, oldPos.x, oldPos.y, oldPos.z, oldRot.x, oldRot.y, oldRot.z, oldRot.w)
  end
  isInMeshView = false
end


-- Public interface.
M.availStaticMeshes =                                     availStaticMeshes

M.fetchAvailableStaticMeshes =                            fetchAvailableStaticMeshes
M.addMeshToAudition =                                     addMeshToAudition
M.goToMeshView =                                          goToMeshView
M.manageRotateCam =                                       manageRotateCam
M.goToOldView =                                           goToOldView
M.removeAuditionMesh =                                    removeAuditionMesh
M.updateCameraPose =                                      updateCameraPose

return M