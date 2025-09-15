-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'core_camera', 'core_settings_settings', 'core_input_bindings'}

local min = math.min
local max = math.max

M.cabinFilterStrength = 1

local lastCamPos = nil
local lastCameraForward = nil
local cameraForward = vec3()
local vecDown3F = vec3(0,0,-1)
local frameFlag = true
local camPos = vec3()
local vehVelocity = vec3()
local insideModifier = settings.getValue('AudioInsideModifier')

-- audio blur helper variables
local gameAudioBlurValue = 0
local missionMarkerInteraction = false
local interactingWithMissionUI = false
local audioBlurLog = nop
--audioBlurLog = log --// Enable this line to add logging for audio blur.

local function onPreRender(dtReal, dtSim, dtRaw)
  if Engine.Audio.getGlobalParams then
    local globalParams = Engine.Audio.getGlobalParams()
    if globalParams then

      globalParams:setParameterValue("g_GameAudioBlur", gameAudioBlurValue)

      camPos:set(core_camera.getPositionXYZ())
      cameraForward:set(core_camera.getForwardXYZ())

      if dtSim > 0 then
        globalParams:setParameterValue("g_CamSpeedMS", camPos:distance(lastCamPos or camPos) / dtSim)
        globalParams:setParameterValue("g_CamRotationSpeedMS", cameraForward:distance(lastCameraForward or cameraForward) / dtSim)
      end
      lastCamPos = lastCamPos or vec3()
      lastCameraForward = lastCameraForward or vec3()
      lastCamPos:set(camPos)
      lastCameraForward:set(cameraForward)

      if frameFlag then
        local tod = scenetree.tod
        if tod and tod.time then
          globalParams:setParameterValue("g_Tod", tod.time)
        end

        local camAngle = math.atan2(cameraForward.x, -cameraForward.y) * 180 / math.pi + 180.0
        globalParams:setParameterValue("g_CamRotationAngle", camAngle)

        local veh = getPlayerVehicle(0)
        if veh then
          vehVelocity:set(veh:getVelocityXYZ())
          globalParams:setParameterValue("g_VehicleSpeedPlayerMS", vehVelocity:length())
        end
      else
        local isCameraInside = (core_camera and core_camera.isCameraInside(0, camPos)) or 0
        globalParams:setParameterValue("g_CamOnboard", square(square(insideModifier)) * isCameraInside) -- cockpit flag, used e.g. for driver camera
        globalParams:setParameterValue("c_CabinFilterReverbStrength", clamp(M.cabinFilterStrength, 0, 1)) -- cockpit flag, used e.g. for driver camera
        local camObj = getCamera()
        camObj = (camObj and Sim.upcast(camObj)) or camObj
        globalParams:setParameterValue("g_CamFree", commands.isFreeCamera() and 1 or 0)
        local camUnderwater = (camObj and camObj:isCameraUnderwater()) and 1 or 0
        globalParams:setParameterValue("g_CamUnderwater", camUnderwater)
        globalParams:setParameterValue("g_UnderwaterDepth", camUnderwater == 0 and -1 or camObj:getCameraDepthUnderwater())
        local camHeightToGeometry = castRayStatic(camPos, vecDown3F, 200)
        globalParams:setParameterValue("g_CamHeightToGround", (camObj and camHeightToGeometry) or 0)
        globalParams:setParameterValue("g_CamHeightToSea", (camObj and camPos.z) or 0)
      end
    end

    frameFlag = not frameFlag
  end
end

local function initEngineSound(vehicleId, engineId, jsonPath, nodeIdArray, noloadVol, loadVol)
  local vehicle = scenetree.findObjectById(vehicleId)
  if vehicle then
    if type(nodeIdArray) ~= 'table' then
      nodeIdArray = {nodeIdArray}
    end
    vehicle:engineSoundInit(engineId, jsonPath, nodeIdArray, noloadVol or 1, loadVol or 1)
    vehicle:engineSoundParameterList(engineId, {wet_level = 0, dry_level = 1})
  end
end

local function initExhaustSound(vehicleId, engineId, jsonPath, nodeIdPairArray, noloadVol, loadVol)
  local vehicle = scenetree.findObjectById(vehicleId)
  if vehicle then
    if type(nodeIdPairArray) ~= 'table' then
      nodeIdPairArray = {{nodeIdPairArray, nodeIdPairArray}}
    end

    vehicle:engineSoundInit(engineId, jsonPath, nodeIdPairArray, noloadVol or 1, loadVol or 1)
    vehicle:engineSoundParameterList(engineId, {wet_level = 0, dry_level = 1})
  end
end

local function updateEngineSound(vehicleId, engineId, rpm, onLoad, engineVolume)
  local vehicle = scenetree.findObjectById(vehicleId)
  if not vehicle then return end
  vehicle:engineSoundUpdate(engineId, rpm, onLoad, engineVolume)
end

local function setEngineSoundParameter(vehicleId, engineId, paramName, paramValue)
  local vehicle = scenetree.findObjectById(vehicleId)
  if not vehicle then return end
  vehicle:engineSoundParameter(engineId, paramName, paramValue)
end

local function setEngineSoundParameterList(vehicleId, engineId, parameters)
  local vehicle = scenetree.findObjectById(vehicleId)
  if not vehicle then return end
  vehicle:engineSoundParameterList(engineId, parameters)
end

local function setExhaustSoundNodes(vehicleId, engineId, nodeIdPairArray)
  local vehicle = scenetree.findObjectById(vehicleId)
  if not vehicle then return end

  vehicle:engineSoundNodes(engineId, nodeIdPairArray)
end

local function onSettingsChanged()
  insideModifier = settings.getValue('AudioInsideModifier')
end

local function onUiChangedState(toState, fromState)
  if not missionMarkerInteraction then
    return
  end

  local old_value = gameAudioBlurValue
  if interactingWithMissionUI then
    gameAudioBlurValue = 1
    if fromState == 'play' and toState == 'blank' then
      gameAudioBlurValue = 1
    elseif fromState == 'play' and toState == 'scenario-start' then
      gameAudioBlurValue = 1
      interactingWithMissionUI = false
    elseif fromState == 'scenario-start' and toState == 'play' then
      gameAudioBlurValue = 0
      interactingWithMissionUI = false
    elseif fromState == 'play' and toState == 'scenario-end' then
      gameAudioBlurValue = 0
      interactingWithMissionUI = false
    elseif fromState == 'menu' and toState == 'play' then
      gameAudioBlurValue = 0
      interactingWithMissionUI = false
    elseif fromState == 'fadeScreen' and toState == 'play' then
      gameAudioBlurValue = 0
      interactingWithMissionUI = false
    end
  else
    gameAudioBlurValue = 0
    interactingWithMissionUI = false
  end

  audioBlurLog('I','AUDIO',string.format("ui changed: %s => %s  gameAudioBlurValue = %0.1f (old = %0.1f) (interactingWithMissionUI = %s)", tostring(fromState), tostring(toState), gameAudioBlurValue, old_value, tostring(interactingWithMissionUI)))
end

local function onMissionInfoChangedState(fromState, toState, content)
  if not missionMarkerInteraction then
    return
  end

  if toState == 'opened' then
    interactingWithMissionUI = true
  elseif fromState == 'opened' and toState == 'closed' then
    interactingWithMissionUI = false
    gameAudioBlurValue = 0
  end
  audioBlurLog('I','AUDIO',string.format("missionInfo changed: %s => %s  gameAudioBlurValue = %0.1f (interactingWithMissionUI = %s)", tostring(fromState), tostring(toState), gameAudioBlurValue, tostring(interactingWithMissionUI)))
end

local function onActivityAcceptGatherData(elemData, activityData)
  audioBlurLog('I','AUDIO',string.format("onActivityAcceptGatherData: elemData = %s, activityData = %s",dumps(elemData),(activityData)))
  missionMarkerInteraction = false
  for i,v in ipairs(elemData) do
    if v.type == "mission" then
      missionMarkerInteraction = true
    end
  end
end

local function onMissionAvailabilityChanged(data)
  if data and data.missionCount == 0 then
    missionMarkerInteraction = false
  end
end

local function setCabinFilterStrength(objId, value)
  if objId ~= getPlayerVehicle(0):getId() then return end
  M.cabinFilterStrength = value
end

local function onVehicleSwitched(oldId, newId, player)
  if player ~= 0 then return end
  local newVehicle = getObjectByID(newId)
  if not newVehicle then return end
  newVehicle:queueLuaCommand("sounds.updateCabinFilter()")
end


M.onPreRender                   = onPreRender
M.onSettingsChanged             = onSettingsChanged
M.initEngineSound               = initEngineSound
M.initExhaustSound              = initExhaustSound
M.updateEngineSound             = updateEngineSound
M.setEngineSoundParameter       = setEngineSoundParameter
M.setEngineSoundParameterList   = setEngineSoundParameterList
M.setExhaustSoundNodes          = setExhaustSoundNodes
M.onUiChangedState              = onUiChangedState
M.onMissionInfoChangedState     = onMissionInfoChangedState
M.onActivityAcceptGatherData    = onActivityAcceptGatherData
M.onMissionAvailabilityChanged  = onMissionAvailabilityChanged
M.onVehicleSwitched             = onVehicleSwitched
M.setCabinFilterStrength        = setCabinFilterStrength

M.setAudioBlur = function (value)
  audioBlurLog('I','AUDIO',string.format("gameAudioBlurValue changed manually: gameAudioBlurValue = %0.1f (old = %0.1f)", value, gameAudioBlurValue))
  gameAudioBlurValue = value
end

return M
