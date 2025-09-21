-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 70
M.componentOrderTractionControl = 1000 --this can't be used for traction control
M.componentOrderYawControl = 30
M.componentOrderABSControl = 1000 --this can't be used for ABS control
M.isActive = false
M.isActingAsTC = false
M.isActingAsYC = false
M.isActingAsABS = false

local min = math.min
local abs = math.abs

local CMU = nil
local isDebugEnabled = false

local controlParameters = {yawControl = {isEnabled = true, slipAngleThreshold = 0}}
local initialControlParameters

local configPacket = {sourceType = "aeroControl", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "aeroControl", tractionControl = {}, yawControl = {}}

local reduceOversteerSmoother = newTemporalSmoothing(1, 1000)

--returns true if component did act as traction control
--called from updateFixedStep
local function actAsTractionControl(wheelGroup, dt)
  M.isActingAsTC = false
  return M.isActingAsTC
end

--returns true if component did act as yaw control
--called from updateFixedStep
local function actAsYawControl(measuredYaw, expectedYaw, yawDifference, bodySlipAngle, dt)
  M.isActingAsYC = false
  local requestReduceOversteer = false
  if controlParameters.yawControl.isEnabled then
    local bsaControlSign = sign(measuredYaw) * sign(bodySlipAngle) --negative if oversteering
    local correctedBSA = -min(abs(bodySlipAngle) * bsaControlSign, 0)
    requestReduceOversteer = abs(correctedBSA) > controlParameters.yawControl.slipAngleThreshold
    M.isActingAsYC = requestReduceOversteer
  end

  electrics.values.yawControlRequestReduceOversteer = sign(reduceOversteerSmoother:getUncapped(requestReduceOversteer and 1 or 0, dt))
  return M.isActingAsYC
end

local function updateGFX(dt)
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  debugPacket.tractionControl.isActing = M.isActingAsTC
  debugPacket.yawControl.isActing = M.isActingAsYC
  debugPacket.yawControl.slipAngleThreshold = controlParameters.yawControl.slipAngleThreshold

  CMU.sendDebugPacket(debugPacket)
end

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled

  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
end

local function registerCMU(cmu)
  CMU = cmu
end

local function applyControlParameters()
end

local function reset()
  M.isActingAsTC = false
  M.isActingAsYC = false
  M.isActingAsABS = false
  reduceOversteerSmoother:reset()
end

local function init(jbeamData)
  M.isActingAsTC = false
  M.isActingAsYC = false
  M.isActingAsABS = false
end

local function initSecondStage(jbeamData)
  local yawControl = CMU.getSupervisor("yawControl")
  if yawControl then
    yawControl.registerComponent(M)
  end
  controlParameters.yawControl.isEnabled = true
  controlParameters.yawControl.slipAngleThreshold = jbeamData.slipAngleThreshold or 0.2

  M.actAsTractionControl = nop

  applyControlParameters()

  initialControlParameters = deepcopy(controlParameters)
  M.isActive = true
end

local function initLastStage(jbeamData)
end

local function shutdown()
  M.isActive = false
  M.isActingAsTC = false
  M.isActingAsYC = false
  M.isActingAsABS = false
  M.updateGFX = nil
  M.update = nil
end

local function setParameters(parameters)
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.isEnabled")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.slipAngleThreshold")

  applyControlParameters()
end

local function setConfig(configTable)
  controlParameters = configTable
  applyControlParameters()
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

M.actAsTractionControl = actAsTractionControl
M.actAsYawControl = actAsYawControl
M.actAsABSControl = nop

return M
