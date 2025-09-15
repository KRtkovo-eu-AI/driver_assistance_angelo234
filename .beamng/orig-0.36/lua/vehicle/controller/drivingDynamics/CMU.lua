-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 50

local abs = math.abs

M.sensorHub = nil
M.vehicleData = nil
M.virtualSensors = nil

M.warningLightPulse = 0

local warningLightPulseTimer = 0
local warningLightPulseTime = 0.15
local isStoppedSmoother = newTemporalSmoothing(100, 1)
local isOnRoofSmoother = newTemporalSmoothing(50, 1)

local isUDPConnected = false
local debugSendFPS = 1 / 30
local debugReceiveFPS = 1 / 10
local debugHeartbeatFPS = 1 / 1
local debugSendTimer = 0
local debugReceiveTimer = 0
local debugHeartbeatTimer = 0
local debugPackets = {}
local udpSocket

local controlParameters
local initialControlParameters

local subControllers = nil
local subControllerLookup = nil

local CMUDebugPacket = {sourceType = "CMU", isActiveVehicle = false}

local calibrationCallbacksUpdate = {}
local calibrationCallbacksUpdateFixedStep = {}
local calibrationCallbacksUpdateGFX = {}

local function updateCalibrationCallback(dt)
  for _, callback in ipairs(calibrationCallbacksUpdate) do
    callback(dt)
  end
end

local function updateFixedStepCalibrationCallback(dt)
  for _, callback in ipairs(calibrationCallbacksUpdateFixedStep) do
    callback(dt)
  end
end

local function updateGFXCalibrationCallback(dt)
  for _, callback in ipairs(calibrationCallbacksUpdateGFX) do
    callback(dt)
  end
end

local function disableSystemsForCalibration()
  local supervisors = M.getSupervisors()
  for _, sub in ipairs(supervisors) do
    sub.setParameters({isEnabled = false})
  end
end

local function registerCalibrationCallback(callback, callbackType)
  if callbackType == "update" then
    table.insert(calibrationCallbacksUpdate, callback)
  elseif callbackType == "updateFixedStep" then
    table.insert(calibrationCallbacksUpdateFixedStep, callback)
  elseif callbackType == "updateGFX" then
    table.insert(calibrationCallbacksUpdateGFX, callback)
  end

  M.updateCalibrationCallback = #calibrationCallbacksUpdate > 0 and updateCalibrationCallback or nop
  M.updateFixedStepCalibrationCallback = #calibrationCallbacksUpdateFixedStep > 0 and updateFixedStepCalibrationCallback or nop
  M.updateGFXCalibrationCallback = #calibrationCallbacksUpdateGFX > 0 and updateGFXCalibrationCallback or nop
end

local function debugPacket(packet)
  packet.vehicleID = objectId
  table.insert(debugPackets, packet)
end

local function sendDebugData()
  if isUDPConnected then
    udpSocket:send(jsonEncode(debugPackets))
    table.clear(debugPackets)
  end
end

local function sendDebugHeartbeat()
  if isUDPConnected then
    udpSocket:send("BNGDSE")
  end
end

local function sendConfigData()
  debugPacket({sourceType = "CMU", packetType = "config", config = controlParameters})

  subControllers = controller.getControllersFromPath("drivingDynamics/")

  for _, sub in ipairs(subControllers) do
    if sub.sendConfigData then
      sub.sendConfigData()
    end
  end
end

local function receiveDebugCommands()
  if not isUDPConnected then
    return
  end

  local data = udpSocket:receive()
  if data then
    --dump(data)
    local splits = split(data, "->")
    local commandType = splits[1]
    local controllerName = splits[2]
    if commandType and controllerName then
      if commandType == "RequestConfig" then
        sendConfigData()
      elseif commandType == "EnableDebugMode" then
        local receiver = subControllerLookup[controllerName]
        if receiver.setDebugMode then
          receiver.setDebugMode(true)
        end
      elseif commandType == "SetConfig" then
        local receiver = subControllerLookup[controllerName]
        local controllerConfig = splits[3]
        if receiver and receiver.setConfig and controllerConfig then
          local configTable = jsonDecode(controllerConfig, "CMU SetConfig")
          --dump(configTable)
          receiver.setConfig(configTable)
          sendConfigData()
        end
      elseif commandType == "SetProperty" then
        local receiver = subControllerLookup[controllerName]
        if receiver and receiver.setParameters then
          local param = splits[3]
          local paramTable = jsonDecode(param, "CMU SetProperty")
          --dump(paramTable)
          receiver.setParameters(paramTable)
        end
      end
    end
  end
end

local function checkForRollOverAndCrash(dt)
  local isOnRoof = M.sensorHub.accelerationZSmooth > -(M.sensorHub.gravity * 0.5)
  local isRollingOver = M.sensorHub.pitch > 0.9 or M.sensorHub.roll > 0.9
  local isOnRoofSmooth = isOnRoofSmoother:getUncapped(isOnRoof and 1 or 0, dt)
  local isCrashed = electrics.values.postCrashBrakeTriggered and electrics.values.postCrashBrakeTriggered > 0
  local isStopped = abs(M.sensorHub.accelerationXSmooth) < 2 and abs(M.sensorHub.accelerationYSmooth) < 2 and abs(M.sensorHub.yawAVSmooth) < 0.5
  local isStoppedSmooth = isStoppedSmoother:getUncapped(isStopped and 1 or 0, dt)

  if isStoppedSmooth >= 1 then
    if isOnRoofSmooth >= 1 then
      electrics.values.dseRollOverStopped = 1
    end
    if isCrashed then
      electrics.values.dseCrashStopped = 1
    end
  end

  if isRollingOver then
    electrics.values.dseRollingOver = 1
  end
end

local function updateGFX(dt)
  checkForRollOverAndCrash(dt)

  warningLightPulseTimer = warningLightPulseTimer + dt
  if warningLightPulseTimer >= warningLightPulseTime then
    M.warningLightPulse = bit.bxor(M.warningLightPulse, 1)
    warningLightPulseTimer = 0
  end
  local yawControl = M.getSupervisor("yawControl")
  local tractionControl = M.getSupervisor("tractionControl")
  local warningPulse = (((yawControl and yawControl.isActing) or (tractionControl and tractionControl.isActing)) and 1 or 0) * M.warningLightPulse
  electrics.values.dseWarningPulse = warningPulse

  M.updateGFXCalibrationCallback(dt)
end

local function updateGFXDebugNotEnabled(dt)
  updateGFX(dt)

  debugHeartbeatTimer = debugHeartbeatTimer + dt
  if debugHeartbeatTimer >= debugHeartbeatFPS then
    debugHeartbeatTimer = debugHeartbeatTimer - debugHeartbeatFPS
    receiveDebugCommands()
    sendDebugHeartbeat()
  end
end

local function updateGFXDebugEnabled(dt)
  updateGFX(dt)

  CMUDebugPacket.isActiveVehicle = playerInfo.firstPlayerSeated
  debugPacket(CMUDebugPacket)

  debugSendTimer = debugSendTimer + dt
  if debugSendTimer >= debugSendFPS then
    debugSendTimer = debugSendTimer - debugSendFPS
    sendDebugData()
  end

  debugReceiveTimer = debugReceiveTimer + dt
  if debugReceiveTimer >= debugReceiveFPS then
    debugReceiveTimer = debugReceiveTimer - debugReceiveFPS
    receiveDebugCommands()
  end
end

local function updateFixedStep(dt)
  M.updateFixedStepCalibrationCallback(dt)
end

local function update(dt)
  M.updateCalibrationCallback(dt)
end

local function shutDownAllSystems()
  subControllers = controller.getControllersFromPath("drivingDynamics/")

  for _, sub in ipairs(subControllers) do
    if sub.shutdown then
      sub.shutdown()
    end
  end
end

local function setDebugMode(debugEnabled)
  M.updateGFX = debugEnabled and updateGFXDebugEnabled or updateGFXDebugNotEnabled
  for _, c in pairs(subControllerLookup) do
    if c.typeName ~= "drivingDynamics/CMU" and c.setDebugMode then
      c.setDebugMode(debugEnabled)
    end
  end

  controller.cacheAllControllerFunctions()
end

local function reset(jbeamData)
  table.clear(debugPackets)
  electrics.values.dseCrashStopped = nil
  electrics.values.dseRollOverStopped = nil
  electrics.values.dseRollingOver = nil
  isOnRoofSmoother:reset()
  isStoppedSmoother:reset()
end

local function init(jbeamData)
  isUDPConnected = false
  local debugSettings = jbeamData.debugSettings or {}
  table.clear(debugPackets)
  local peerIP = debugSettings.peerIP or "127.0.0.1"
  local peerPort = debugSettings.peerPort or 54812

  --socket is not always available
  if socket then
    udpSocket = socket.udp()
    udpSocket:settimeout(0.00)
    local result, error = udpSocket:setpeername(peerIP, peerPort)
    if result and not error then
      isUDPConnected = true
    end
  end

  local indicateUI = jbeamData.indicateUI == nil and true or false
  controlParameters = {uiDisplayData = {simplePowertrainApp = {doUpdate = indicateUI, activeColor = "98FB00", offColor = "343434"}}}

  electrics.values.dseCrashStopped = nil
  electrics.values.dseRollOverStopped = nil
  electrics.values.dseRollingOver = nil
end

local function initSecondStage()
  subControllerLookup = {}
  subControllers = controller.getControllersFromPath("drivingDynamics/")

  for _, sub in ipairs(subControllers) do
    if sub.typeName ~= "drivingDynamics/CMU" then
      if sub.registerCMU then
        sub.registerCMU(M)
      end

      local controllerName = sub.name
      local slashPos = controllerName:find("/", -controllerName:len())
      if slashPos then
        controllerName = controllerName:sub(slashPos + 1)
      end
      subControllerLookup[controllerName] = sub
    else
      subControllerLookup[sub.name] = sub --Put the CMU in the list too
    end
  end

  M.sensorHub = M.getSensor("sensorHub")
  M.vehicleData = M.getSensor("vehicleData")
  M.virtualSensors = M.getSensor("virtualSensors")

  setDebugMode(false)

  initialControlParameters = deepcopy(controlParameters)
end

local function initLastStage()
  subControllers = controller.getControllersFromPath("drivingDynamics/")

  local allSystemsActive = true

  for _, sub in ipairs(subControllers) do
    if sub.typeName ~= "drivingDynamics/CMU" then
      if not sub.isActive then
        log("W", "CMU.initSecondStage", string.format("System %q is inactive, system startup not possible.", sub.name))
      end
      allSystemsActive = allSystemsActive and sub.isActive
    end
  end

  if not allSystemsActive then
    log("E", "CMU.initSecondStage", "Not all systems are active, aborting init and shutting down...")
    shutDownAllSystems()
  end

  electrics.values.isYCBrakeActive = 0
  electrics.values.isTCBrakeActive = 0

  --Tell the debug app that we spawned a new car
  debugPacket({sourceType = "CMU", packetType = "init"})
end

local function getSubController(folder, name)
  local path = string.format("drivingDynamics/%s/%s", folder, name)

  for _, sub in ipairs(subControllers) do
    if sub.typeName == path then
      return sub
    end
  end
end

local function getSubControllers(folder)
  local path = string.format("drivingDynamics/%s/", folder)
  local subs = {}

  for _, sub in ipairs(subControllers) do
    if sub.typeName:startswith(path) then
      table.insert(subs, sub)
    end
  end

  return subs
end

local function getSensor(sensorName)
  return getSubController("sensors", sensorName)
end

local function getSupervisor(supervisorName)
  return getSubController("supervisors", supervisorName)
end

local function getActuator(actuatorName)
  return getSubController("actuators", actuatorName)
end

local function getSupervisors()
  return getSubControllers("supervisors")
end

local function getSensors()
  return getSubControllers("sensors")
end

local function getActuators()
  return getSubControllers("actuators")
end

local function applyParameter(currentParameters, initialParameters, changedParameters, name)
  local splits = split(name, ".")
  local currentControlParam = currentParameters
  local currentDefaultControlParam = initialParameters
  for i = 1, #splits - 1 do
    currentControlParam = currentControlParam[splits[i]]
    currentDefaultControlParam = currentDefaultControlParam[splits[i]]
  end
  local value = changedParameters[name]
  if value == nil or currentControlParam == nil then
    return false
  end
  name = splits[#splits]

  if currentControlParam[name] ~= nil and currentDefaultControlParam[name] ~= nil then
    if value == "default" then
      currentControlParam[name] = deepcopy(currentDefaultControlParam[name])
    else
      currentControlParam[name] = value
    end
    return true
  else
    log("D", "CMU.applyParameter", "Can't find parameter: " .. name)
    -- dump(changedParameters)
    -- dump(currentControlParam)
    -- dump(currentDefaultControlParam)
    -- print(debug.traceback())
    return false
  end
end

local function setParameters(parameters)
  local updateActiveColor = applyParameter(controlParameters, initialControlParameters, parameters, "uiDisplayData.simplePowertrainApp.activeColor")
  local updateOffColor = applyParameter(controlParameters, initialControlParameters, parameters, "uiDisplayData.simplePowertrainApp.offColor")
  local updateDoUpdate = applyParameter(controlParameters, initialControlParameters, parameters, "uiDisplayData.simplePowertrainApp.doUpdate")
  if updateActiveColor or updateOffColor or updateDoUpdate then
    local driveModesController = controller.getController("driveModes")
    if driveModesController then
      driveModesController.setSimpleControlButton("dseBackwardsCompat", "DSE", "powertrain_esc", controlParameters.uiDisplayData.simplePowertrainApp.activeColor, controlParameters.uiDisplayData.simplePowertrainApp.offColor, "dseWarningPulse")
    end
  end
end

local function setConfig(configTable)
  controlParameters = configTable
end

local function getConfig()
  return deepcopy(controlParameters)
end

M.init = init
M.initSecondStage = initSecondStage
M.reset = reset
M.initLastStage = initLastStage
M.updateGFX = updateGFX
M.updateFixedStep = updateFixedStep
M.update = update

M.getSensor = getSensor
M.getSensors = getSensors
M.getSupervisor = getSupervisor
M.getSupervisors = getSupervisors
M.getActuator = getActuator
M.getActuators = getActuators

M.sendDebugPacket = debugPacket
M.setDebugMode = setDebugMode

M.applyParameter = applyParameter
M.setParameters = setParameters
M.setConfig = setConfig
M.getConfig = getConfig

M.disableSystemsForCalibration = disableSystemsForCalibration
M.registerCalibrationCallback = registerCalibrationCallback
M.updateCalibrationCallback = nop
M.updateFixedStepCalibrationCallback = nop
M.updateGFXCalibrationCallback = nop

return M
