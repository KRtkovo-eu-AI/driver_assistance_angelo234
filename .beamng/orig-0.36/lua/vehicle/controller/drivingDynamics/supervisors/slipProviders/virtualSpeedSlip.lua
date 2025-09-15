-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 70
M.providerOrder = 10
M.isActive = false

local min = math.min
local max = math.max
local abs = math.abs

local CMU = nil
local isDebugEnabled = false

local controlParameters = {tractionControl = {isEnabled = true}, absControl = {isEnabled = true}}
local initialControlParameters

local configPacket = {sourceType = "virtualSpeedSlip", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "virtualSpeedSlip"}

--called from updateFixedStep
local function calculateSlipTractionControl(wheelGroup, steeringCoef, velocityOffset, dt)
  if not controlParameters.tractionControl.isEnabled then
    for j = 1, wheelGroup.wheelCount do
      local wheelData = wheelGroup.wheels[j]
      wheelData.slip = 0
      wheelGroup.maxSlip = 0
      wheelGroup.minSlip = 0
      wheelGroup.slipRange = 0
    end
    return false
  end

  wheelGroup.maxSlip = 0
  wheelGroup.minSlip = 1
  local vehicleVelocity = CMU.virtualSensors.virtual.speed
  --we only want to record slip when we are actually powering the wheels
  local torqueCoef = linearScale(wheelGroup.motor.throttle, 0, 0.01, 0, 1)
  local speedTrustCoef = linearScale(CMU.virtualSensors.trustWorthiness.virtualSpeed, 0.5, 0.8, 0, 1)
  local slipCoef = min(torqueCoef, steeringCoef, speedTrustCoef)

  for j = 1, wheelGroup.wheelCount do
    local wheelData = wheelGroup.wheels[j]
    local wd = wheelData.wd
    local wheelSpeed = wd.wheelSpeed * (CMU.vehicleData.turningCircleSpeedRatios[wd.name] or 1)
    local wheelSlip = wheelData.slipSmoother:get(abs(min(max(((wheelSpeed + velocityOffset) / (vehicleVelocity + velocityOffset)) - 1, -0.5), 0.5)) * slipCoef)

    wheelData.slip = wheelSlip
    wheelGroup.maxSlip = max(wheelGroup.maxSlip, wheelSlip)
    wheelGroup.minSlip = min(wheelGroup.minSlip, wheelSlip)
  end

  wheelGroup.slipRange = max(wheelGroup.maxSlip - wheelGroup.minSlip, 0)

  return true
end

--called from updateFixedStep
local function calculateSlipABSControl(wheelData, velocityOffset, dt)
  if not controlParameters.absControl.isEnabled then
    wheelData.slip = 0
    return false
  end

  local vehicleVelocity = CMU.virtualSensors.virtual.speed
  local speedTrustCoef = linearScale(CMU.virtualSensors.trustWorthiness.virtualSpeed, 0.5, 0.8, 0, 1)
  local slipCoef = min(speedTrustCoef, 1)

  local wd = wheelData.wd
  local wheelSpeed = wd.wheelSpeed * (CMU.vehicleData.turningCircleSpeedRatios[wd.name] or 1)
  local wheelSlip = wheelData.slipSmoother:get(abs(min(max(((wheelSpeed + velocityOffset) / (vehicleVelocity + velocityOffset)) - 1, -0.5), 0.5)) * slipCoef)

  wheelData.slip = wheelSlip

  return true
end

local function updateGFX(dt)
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  debugPacket.isEnabledTractionControl = controlParameters.tractionControl.isEnabled
  debugPacket.isEnabledABSControl = controlParameters.absControl.isEnabled

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
  local tractionControl = CMU.getSupervisor("tractionControl")
  local absControl = CMU.getSupervisor("absControl")

  if tractionControl then
    tractionControl.registerSlipProvider(M)
  end
  if absControl then
    absControl.registerSlipProvider(M)
  end
end

local function initLastStage(jbeamData)
  initialControlParameters = deepcopy(controlParameters)
end

local function shutdown()
  M.isActive = false
  M.updateGFX = nil
  M.update = nil
end

local function setParameters(parameters)
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.isEnabled")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "absControl.isEnabled")
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

M.calculateSlipTractionControl = calculateSlipTractionControl
M.calculateSlipABSControl = calculateSlipABSControl

return M
