-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 80
M.providerOrder = 20
M.isActive = false

local abs = math.abs
local min = math.min

local CMU = nil
local isDebugEnabled = false

local controlParameters = {isEnabled = true}
local initialControlParameters

local configPacket = {sourceType = "STMEstimate", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "STMEstimate"}

local function calculateExpectedYaw(dt)
  local frontWheelAngle = abs(CMU.vehicleData.frontWheelAngle)
  local speed = CMU.virtualSensors.trustWorthiness.virtualSpeed >= 1 and abs(CMU.virtualSensors.virtual.speed) or abs(CMU.virtualSensors.virtual.wheelSpeed)
  local invWheelBase = CMU.vehicleData.vehicleStats.invWheelBase
  local invSquaredCharacteristicSpeed = CMU.vehicleData.vehicleStats.invSquaredCharacteristicSpeed

  --calculate expected yaw rate based on steering angle
  local desiredYawRateSteering = ((frontWheelAngle * invWheelBase) * (speed / (1 + (speed * speed * invSquaredCharacteristicSpeed))))
  --calculate expected yaw rate based on Gs
  local desiredYawRateAcceleration = controlParameters.maxLateralAcceleration / (speed + 1e-30)

  --get the resulting desired yaw rate (smallest) and make sure to use the sign from the steering part (acceleration part is always positive)
  local desiredYawRate = min(desiredYawRateSteering, desiredYawRateAcceleration)

  return desiredYawRate
end

local function updateGFX(dt)
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  debugPacket.isEnabled = controlParameters.isEnabled

  CMU.sendDebugPacket(debugPacket)
end

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled

  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
end

local function registerCMU(cmu)
  CMU = cmu
end

local function reset()
end

local function init(jbeamData)
  M.isActive = true
end

local function initSecondStage(jbeamData)
  local yawControl = CMU.getSupervisor("yawControl")
  if not yawControl then
    M.isActive = false
    return
  end

  yawControl.registerYawProvider(M)

  controlParameters.isEnabled = true
  controlParameters.maxLateralAcceleration = jbeamData.maxLateralAcceleration or 12

  initialControlParameters = deepcopy(controlParameters)
end

local function initLastStage(jbeamData)
end

local function shutdown()
  M.isActive = false
  M.updateGFX = nil
  M.update = nil
end

local function setParameters(parameters)
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "isEnabled")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "maxLateralAcceleration")
end

local function setConfig(configTable)
  controlParameters = configTable
end

local function getConfig()
  return deepcopy(controlParameters)
end

local function sendConfigData()
  configPacket.config = controlParameters
  CMU.sendDebugPacket(configPacket)
end

M.init = init
M.initSecondStage = initSecondStage
M.initLastStage = initLastStage

M.reset = reset

M.updateGFX = updateGFX

M.registerCMU = registerCMU
M.setDebugMode = setDebugMode
M.shutdown = shutdown
M.setParameters = setParameters
M.setConfig = setConfig
M.getConfig = getConfig
M.sendConfigData = sendConfigData

M.calculateExpectedYaw = calculateExpectedYaw

return M
