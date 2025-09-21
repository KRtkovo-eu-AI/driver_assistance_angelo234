-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M  = {}
local logTag = "commands"

local function initCamera()
  core_camera.clearInputs()
  local cam = scenetree.findObject("gameCamera")
  if not cam then
    log("E", "", "Cannot setCamera, camera not found: "..tostring("gameCamera"))
    return
  end

  local mainView = RenderViewManagerInstance:getOrCreateView('main')
  serverConnection.onCameraHandlerSetInitial()
  extensions.hook('onCameraHandlerSet')

  mainView:setCameraObject(cam.obj)

  core_camera.requestConfig(nil)
  core_camera.setGlobalCameraByName(nil)
  return cam
end

local function isFreeCamera()
  return core_camera.getActiveCamName() == "free"
end

local function getGameCamera() return scenetree.findObject("gameCamera") end
local function setGameCamera()
  core_camera.setGlobalCameraByName(nil)
end

-- function used by C++ side, if you rename or move, you need to edit C++ side too
local function setFreeCamera()
  core_camera.setByName(0, 'free')
  core_camera.setPosition(0, core_camera.getPosition())
  core_camera.setRotation(0, core_camera.getQuat())
  core_camera.resetCamera(0)
end

-- camera modifier for faster speed (typically shift key)
local function toggleFastSpeed(enabled)
  if core_camera then core_camera.setFastSpeedModifier(enabled) end
end

-- camera modifier for normal speed (typically alt+scrollwheel)
local function changeCameraSpeed(val)
  if core_camera then core_camera.changeSpeed(val) end
end

local wasFreeCamera
local function onNodegrabStart(usingPlayerVehicle)
  wasFreeCamera = isFreeCamera()
  if usingPlayerVehicle then return end
  if not wasFreeCamera then
    setFreeCamera()
  end
end
local function onNodegrabStop(usingPlayerVehicle)
  if not wasFreeCamera and isFreeCamera() then
    setGameCamera()
  end
end

local function dropCameraAtPlayer()
  local playerVehicle = getPlayerVehicle(0)
  if not playerVehicle then return end
  setFreeCamera()
  core_camera.setPosition(0, playerVehicle:getPosition())
  core_camera.setRotation(0, quat(playerVehicle:getRotation()))
  core_camera.resetCamera(0)
end

local function dropPlayerAtCamera()
  local playerVehicle = getPlayerVehicle(0)
  if not playerVehicle then return end
  local pos = core_camera.getPosition()
  local camDir = core_camera.getForward()
  camDir.z = 0
  local camRot = quatFromDir(camDir, vec3(0,0,1))
  local rot =  quat(0, 0, 1, 0) * camRot -- vehicles' forward is inverted
  playerVehicle:setPositionRotation(pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w)
  setGameCamera()
  if core_camera.getActiveCamName(0) == "bigMap" then
    core_camera.setByName(0, "orbit", false)
  end
  core_camera.resetCamera(0)
end

local function dropPlayerAtCameraNoReset()
  local playerVehicle = getPlayerVehicle(0)
  if not playerVehicle then return end
  local pos = core_camera.getPosition()
  local camDir = core_camera.getForward()
  camDir.z = 0
  local camRot = quatFromDir(camDir, vec3(0,0,1))
  camRot = quat(0, 0, 1, 0) * camRot -- vehicles' forward is inverted

  local vehRot = quat(playerVehicle:getClusterRotationSlow(playerVehicle:getRefNodeId()))
  local diffRot = vehRot:inversed() * camRot
  playerVehicle:setClusterPosRelRot(playerVehicle:getRefNodeId(), pos.x, pos.y, pos.z, diffRot.x, diffRot.y, diffRot.z, diffRot.w)
  playerVehicle:applyClusterVelocityScaleAdd(playerVehicle:getRefNodeId(), 0, 0, 0, 0)
  setGameCamera()
  if core_camera.getActiveCamName(0) == "bigMap" then
    core_camera.setByName(0, "orbit", false)
  end
  core_camera.resetCamera(0)
  playerVehicle:setOriginalTransform(pos.x, pos.y, pos.z, camRot.x, camRot.y, camRot.z, camRot.w)
end

local function toggleCamera(player)
  player = 0 -- forcibly have multiseat users switch main camera instead of their own
  if isFreeCamera() then
    setGameCamera()
    extensions.core_camera.displayCameraNameUI(player)
    extensions.hook("onCameraToggled", {cameraType='GameCam'})
  else
    setFreeCamera()
    ui_message("ui.camera.freecam",  10, "cameramode")
    extensions.hook("onCameraToggled", {cameraType='FreeCam'})
  end
end

local function getCameraTransformJson()
  local pos = core_camera.getPosition()
  local rot = core_camera.getQuat()
  return string.format('[%0.2f, %0.2f, %0.2f, %g, %g, %g, %g]', pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w)
end

local function setFreeCameraTransformJson(json)
  setFreeCamera()

  json = jsonDecode(json, nil)
  if not json then return end

  for i=1,7 do
    if not json[i] then return end
  end

  core_camera.setPosRot(0, json[1], json[2], json[3], json[4], json[5], json[6], json[7])
end

-- global functions for backwards compatibility
local deprecationWarningDone = {}
local function deprecationWarning(oldFunction, newFunction)
  if not deprecationWarningDone[oldFunction] then
    log('W', logTag, string.format('function "%s" is deprecated. Please use function "%s" instead', oldFunction, newFunction))
    deprecationWarningDone[oldFunction] = true
  end
end

function setCameraPosRot(px, py, pz, rx, ry, rz, rw)
  deprecationWarning("setCameraPosRot", "core_camera.setPosRot")
  core_camera.setPosRot(0, px, py, pz, rx, ry, rz, rw)
end

function setCameraFovDeg(fovDeg)
  deprecationWarning("setCameraFovDeg", "core_camera.setFOV")
  if not isFreeCamera() then return end
  core_camera.setFOV(0, fovDeg)
end

local function radToDeg(r)
  return (r * 180.0) / math.pi
end

function setCameraFovRad(fovRad)
  deprecationWarning("setCameraFovRad", "core_camera.setFOV")
  if not isFreeCamera() then return end
  core_camera.setFOV(0, radToDeg(fovRad))
end

function setCameraFov(fovDeg)
  deprecationWarning("setCameraFov", "core_camera.setFOV")
  if not isFreeCamera() then return end
  core_camera.setFOV(0, fovDeg)
end

function getCameraPosition() deprecationWarning("getCameraPosition", "core_camera.getPosition") return core_camera.getPosition() end
function getCameraUp() deprecationWarning("getCameraUp", "core_camera.getUp") return core_camera.getUp() end
function getCameraRight() deprecationWarning("getCameraRight", "core_camera.getRight")  return core_camera.getRight() end
function getCameraForward() deprecationWarning("getCameraForward", "core_camera.getForward") return core_camera.getForward() end
function getCameraQuat() deprecationWarning("getCameraQuat", "core_camera.getQuat") return core_camera.getQuat() end
function getCameraFovDeg() deprecationWarning("getCameraFovDeg", "core_camera.getFovDeg") return core_camera.getFovDeg() end
function getCameraFovRad() deprecationWarning("getCameraFovRad", "core_camera.getFovRad") return core_camera.getFovRad() end
function getCameraFov() deprecationWarning("getCameraFov", "core_camera.getFovDeg") return core_camera.getFovDeg() end

M.dropCameraAtPlayer = dropCameraAtPlayer
M.dropPlayerAtCamera = dropPlayerAtCamera
M.dropPlayerAtCameraNoReset = dropPlayerAtCameraNoReset
M.getCamera = getGameCamera -- retrocompat
M.getGame = getGame -- retrocompat
M.onNodegrabStart = onNodegrabStart
M.onNodegrabStop = onNodegrabStop
M.setFreeCamera = setFreeCamera -- function used by C++ side, if you rename or move, you need to edit C++ side too
M.setGameCamera = setGameCamera
M.setCameraFree = setFreeCamera -- retrocompat
M.setCameraPlayer = setGameCamera -- retrocompat
M.changeCameraSpeed = changeCameraSpeed
M.toggleFastSpeed = toggleFastSpeed
M.toggleCamera = toggleCamera
M.isFreeCamera = isFreeCamera
M.getCameraTransformJson = getCameraTransformJson
M.setFreeCameraTransformJson = setFreeCameraTransformJson
M.onSettingsChanged = onSettingsChanged
M.initCamera = initCamera

return M
