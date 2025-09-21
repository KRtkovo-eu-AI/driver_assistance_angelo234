-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local editor

--- Sets the current camera in the editor's viewport.
-- @param type the camera type
--  Allowed values:
--  - editor.CameraType_Game - the gameplay camera
--  - editor.CameraType_Free - the editor free fly-by camera
local function selectCamera(type)
  -- there is no level loaded, so no cameras, just return
  if not levelLoaded then return end
  editor.previousCameraType = editor.currentCameraType
  editor.currentCameraType = type
  if type == editor.CameraType_Game then
    commands.setGameCamera()
  elseif type == editor.CameraType_Free then
    commands.setFreeCamera()
  end
end

--- Returns true if the camera is free camera mode.
local function isFreeCamera()
  return commands.isFreeCamera()
end

--- Toggles between current in-game camera and free fly (editor) camera.
local function toggleFreeCamera()
  if not isFreeCamera() then
    editor.selectCamera(editor.CameraType_Free)
  else
    editor.selectCamera(editor.CameraType_Game)
  end
end

--- Returns the editor camera (free/fly camera).
local function getCamera()
  return commands.getCamera()
end

--- Orients the camera to look at a specific world point.
-- @param pt the point in world where the camera should look at
local function cameraLookAtPoint(pt)
  local pos = core_camera.getPosition()
  core_camera.setRotation(0, quatFromDir(pt - pos))
end

--- Sets the camera position at the center of the current object selection.
local function placeCameraAtSelection()
  if not editor.selection or tableIsEmpty(editor.selection.object) then
    return
  end
  local centroid = editor.getSelectionCentroid()
  core_camera.setPosition(0, centroid)
end

--- Sets the camera position at the current player position.
local function placeCameraAtPlayer()
  commands.dropCameraAtPlayer()
end

--- Sets the player position at the current editor camera position.
local function placePlayerAtCamera()
  local playerVehicle = getPlayerVehicle(0)
  if playerVehicle then
    local pos = core_camera.getPosition()
    playerVehicle:setPosition(pos)
  end
end

--- Fit/position the camera viewport to the bounding box of the selected objects, so one can view the entire selection.
local function fitViewToSelection(center)
  local rot = core_camera.getQuat()
  if not center then
    center = editor.objectSelectionBBox:getCenter()
    if not editor.selection.objects or #editor.selection.objects == 0 then
      center = editor.getAxisGizmoTransform():getColumn(3)
    end
  end
  local pos = center + rot * vec3(0, -15, 0)
  local rot = quatFromDir(center - pos)
  core_camera.setPosRot(0, pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w)
end

local cameraSmoothMoveJob = function(job)
  local currentStep = 0
  local currentStepPos = 0
  local camVars = job.args[1]
  local step = (camVars.camEndPos - camVars.camStartPos)/80
  while currentStep <= 20 do
    job.sleep(0.05)
    local tempTarget = camVars.camStartPos +  step * currentStepPos
    local rot = quatFromDir(camVars.center - camVars.camEndPos)
    core_camera.setPosRot(0, tempTarget.x, tempTarget.y, tempTarget.z, rot.x, rot.y, rot.z, rot.w)
    if(currentStep < 12) then
      currentStepPos = currentStepPos + 6
    else
      currentStepPos = currentStepPos + 1
    end
    currentStep = currentStep + 1
  end
end

local function fitViewToSelectionSmooth()
  if not editor_gizmoHelper.isGizmoVisible() then return end
  local bBox = editor.objectSelectionBBox
  local viewRadius = (bBox.maxExtents - bBox.minExtents):len()
  local maxViewRadius = 16000
  local minViewRadius = 20

  if viewRadius <= 0 then
    viewRadius = minViewRadius
  end

  if viewRadius > maxViewRadius then
    viewRadius = maxViewRadius
  end

  local camVars = {}
  camVars.camStartPos = core_camera.getPosition()
  local rot = core_camera.getQuat()
  local center = bBox:getCenter()
  if not editor.selection.object or (#editor.selection.object == 0) then
    center = editor.getAxisGizmoTransform():getColumn(3)
    viewRadius = minViewRadius
  end

  camVars.center = center
  camVars.camEndPos = center + rot * vec3(0, -viewRadius, 0)
  core_jobsystem.create(cameraSmoothMoveJob, 1, camVars)
end

--- Sets the speed of the editor fly camera.
-- @param speed the speed of the fly camera movement.
--    Allowed values:
--      editor.CameraSpeed_Slowest
--      editor.CameraSpeed_Slow
--      editor.CameraSpeed_Slower
--      editor.CameraSpeed_Normal
--      editor.CameraSpeed_Faster
--      editor.CameraSpeed_Fast
--      editor.CameraSpeed_Fastest
local function setCameraSpeed(speed)
  core_camera.setSpeed(speed)
end

local function getCameraSpeed(speed)
  return core_camera.getSpeed()
end

local function getCameraBookmarksGroup()
  if not scenetree.MissionGroup then return end
  local bookmarksGroup = scenetree.findObject("MissionGroup/CameraBookmarks")
  if not bookmarksGroup then
    bookmarksGroup = worldEditorCppApi.createObject("SimGroup")
    bookmarksGroup:registerObject("CameraBookmarks")
    scenetree.MissionGroup:addObject(bookmarksGroup)
  end
  return bookmarksGroup
end

--- Adds a new named camera bookmark at the current world position.
-- @param name the name of the bookmark
-- @return the id of the bookmark object
local function addCameraBookmark(name)
  local bookmarksGroup = getCameraBookmarksGroup()
  local bookmark = worldEditorCppApi.createObject("CameraBookmark")
  bookmark:setField("datablock", 0, "CameraBookmarkMarker")
  bookmark:setField("scale", 0, "5 5 5")
  bookmark:registerObject("")
  bookmark:setInternalName(name)
  bookmark:setTransform(editor.getCamera():getTransform())
  bookmarksGroup:addObject(bookmark)
  return bookmark:getID()
end

--- Deletes a camera bookmark by its name.
-- @param objectId the object id of the bookmark
local function deleteCameraBookmark(objectId)
  local obj = scenetree.findObjectById(objectId)
  if obj then
    obj:deleteObject()
  end
end

--- Returns the bookmarks SimGroup object, where the bookmarks are kept.
local function getCameraBookmarks()
  return getCameraBookmarksGroup()
end

--- Return the object location as text. Used internally by the editor.
-- @param obj the object to get position text for
local function getObjectLocationText(obj)
  obj = Sim.upcast(obj)
  local mtx = obj:getTransform()
  local pos = mtx:getColumn(3)
  local q = QuatF(0,0,0,0)
  q:setFromMatrix(mtx)
  return "["..pos.x..", "..pos.y..", "..pos.z..", "..q.x..", "..q.y..", "..q.z..", "..q.w.."]"
end

--- Copies the camera bookmark to clipboard as text.
-- @param obj the camera bookmark object to copy
local function copyCameraBookmarkToClipboard(obj)
  setClipboard(getObjectLocationText(obj))
end

--- Paste the camera bookmark from clipboard.
local function pasteCameraBookmarkFromClipboard()
  local txt = getClipboard()
  commands.setFreeCameraTransformJson(txt)
end

--- Delete all the camera bookmarks.
local function clearCameraBookmarks()
  local bookmarks = getCameraBookmarks()
  local objs = {}
  local ids = {}
  for i = 1, bookmarks:size() do
    local obj = bookmarks:at(i - 1)
    table.insert(objs, obj)
    table.insert(ids, obj:getId())
  end
  for i = 1, #objs do
    objs[i]:delete()
  end
end

--- Jump to the camera bookmark.
-- @param objectId the object id of the bookmark to jump to
local function jumpToCameraBookmark(objectId)
  local obj = Sim.findObjectById(objectId)
  if obj then
    local pos = obj:getPosition()
    local rot = obj:getRotation()
    core_camera.setPosRot(0, pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w)
  end
end

--- Set the smooth camera move value
-- @param value the newton mode value
local function setSmoothCameraMove(value)
  if not commands.isFreeCamera() then return end
  core_camera.setNewtonTranslation(0, value)
end

--- Set the smooth camera rotate value
-- @param value the newton rotate value
local function setSmoothCameraRotate(value)
  if not commands.isFreeCamera() then return end
  core_camera.setNewtonRotation(0, value)
end

--- Set the smooth camera drag value
-- @param value the drag value
local function setSmoothCameraDrag(drag)
  local cam = core_camera.getGlobalCameras().free
  if cam then
    cam.translationDrag = drag
  end
end

local function setSmoothCameraAngularDrag(drag)
  local cam = core_camera.getGlobalCameras().free
  if cam then
    cam.angularDrag = drag
  end
end

local function setSmoothCameraDragNormalized(value)
  local minDrag = 5
  local maxDrag = 20
  value = minDrag + (1 - value) * (maxDrag - minDrag)
  setSmoothCameraDrag(value)
end

local function setSmoothCameraAngularDragNormalized(value)
  local minDrag = 5
  local maxDrag = 20
  value = minDrag + (1 - value) * (maxDrag - minDrag)
  setSmoothCameraAngularDrag(value)
end

local function getSmoothCameraParams()
  local cam = core_camera.getGlobalCameras().free
  if cam then
    return {
      newtonMode = cam.newtonTranslation,
      newtonRotation = cam.newtonRotation,
      mass = cam.mass,
      drag = cam.translationDrag,
      force = cam.translationForce,
      angularDrag = cam.angularDrag,
      angularForce = cam.angularForce
    }
  else
    editor.logWarn("No camera is set, cannot get camera params")
  end
end

local function setSmoothCameraParams(params)
  local cam = core_camera.getGlobalCameras().free
  if cam then
    cam.newtonTranslation = params.newtonMode
    cam.newtonRotation = params.newtonRotation
    cam.translationDrag = params.drag
    cam.mass = params.mass
    cam.translationForce = params.force
    cam.angularDrag = params.angularDrag
    cam.angularForce = params.angularForce
    cam.angularVelocity = vec3(0,0,0)
  else
    editor.logWarn("No camera is set, cannot set camera params")
  end
end

local function initialize(editorInstance)
  editor = editorInstance

  -- constants
  editor.CameraType_Game = 1
  editor.CameraType_Free = 2
  editor.CameraSpeed_Slowest = 5
  editor.CameraSpeed_Slow = 35
  editor.CameraSpeed_Slower = 70
  editor.CameraSpeed_Normal = 100
  editor.CameraSpeed_Faster = 130
  editor.CameraSpeed_Fast = 165
  editor.CameraSpeed_Fastest = 200

  -- variables
  editor.currentCameraType = editor.CameraType_Game
  editor.previousCameraType = 0

  -- functions
  editor.selectCamera = selectCamera
  editor.toggleFreeCamera = toggleFreeCamera
  editor.placeCameraAtSelection = placeCameraAtSelection
  editor.placeCameraAtPlayer = placeCameraAtPlayer
  editor.placePlayerAtCamera = placePlayerAtCamera
  editor.getCamera = getCamera
  editor.cameraLookAtPoint = cameraLookAtPoint
  editor.fitViewToSelection = fitViewToSelection
  editor.fitViewToSelectionSmooth = fitViewToSelectionSmooth
  editor.setCameraSpeed = setCameraSpeed
  editor.getCameraSpeed = getCameraSpeed
  editor.addCameraBookmark = addCameraBookmark
  editor.deleteCameraBookmark = deleteCameraBookmark
  editor.getCameraBookmarks = getCameraBookmarks
  editor.clearCameraBookmarks = clearCameraBookmarks
  editor.jumpToCameraBookmark = jumpToCameraBookmark
  editor.copyCameraBookmarkToClipboard = copyCameraBookmarkToClipboard
  editor.pasteCameraBookmarkFromClipboard = pasteCameraBookmarkFromClipboard
  editor.getObjectLocationText = getObjectLocationText
  editor.setSmoothCameraMove = setSmoothCameraMove
  editor.setSmoothCameraRotate = setSmoothCameraRotate
  editor.setSmoothCameraDrag = setSmoothCameraDrag
  editor.setSmoothCameraAngularDrag = setSmoothCameraAngularDrag
  editor.setSmoothCameraDragNormalized = setSmoothCameraDragNormalized
  editor.setSmoothCameraAngularDragNormalized = setSmoothCameraAngularDragNormalized
  editor.getSmoothCameraParams = getSmoothCameraParams
  editor.setSmoothCameraParams = setSmoothCameraParams
end

local M = {}
M.initialize = initialize

return M
