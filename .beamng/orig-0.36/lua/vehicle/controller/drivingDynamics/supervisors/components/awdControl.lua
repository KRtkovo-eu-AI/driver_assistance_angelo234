-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 70
M.componentOrderTractionControl = 10
M.componentOrderYawControl = 20
M.componentOrderABSControl = 1000 --this can't be used for ABS control
M.isActive = false
M.isActingAsTC = false
M.isActingAsYC = false
M.isActingAsABS = false

local min = math.min
local abs = math.abs

local CMU = nil
local isDebugEnabled = false

local controlParameters = {tractionControl = {isEnabled = true}, yawControl = {isEnabled = true, slipAngleThreshold = 0}, controlMode = ""}
local initialControlParameters

local configPacket = {sourceType = "awdControl", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "awdControl", tractionControl = {}, yawControl = {}}

local relevantActuator
local lockOverrideMethod

--returns true if component did act as traction control
--called from updateFixedStep
local function actAsTractionControl(wheelGroup, dt)
  M.isActingAsTC = false
  return M.isActingAsTC
end

local function yawControlRearMain(bsa)
  local overrideMin = linearScale(bsa, 0, controlParameters.yawControl.slipAngleThreshold, 0, 1)
  local overrideMax = 1 --todo change once understeer can be detected reliably
  return overrideMin, overrideMax
end

local function yawControlFrontMain(bsa)
  local overrideMin = 0 --todo change once understeer can be detected reliably
  local overrideMax = linearScale(bsa, 0, controlParameters.yawControl.slipAngleThreshold, 1, 0)
  return overrideMin, overrideMax
end

--returns true if component did act as yaw control
--called from updateFixedStep
local function actAsYawControl(measuredYaw, expectedYaw, yawDifference, bodySlipAngle, dt)
  M.isActingAsYC = false
  relevantActuator.resetOverride()

  local overrideMin = 0
  local overrideMax = 1
  if controlParameters.yawControl.isEnabled then
    if electrics.values.throttle > 0 then
      local bsaControlSign = sign(measuredYaw) * sign(bodySlipAngle) --negative if oversteering
      local correctedBSA = -min(abs(bodySlipAngle) * bsaControlSign, 0)
      overrideMin, overrideMax = lockOverrideMethod(correctedBSA)
    end
  end

  relevantActuator.overrideMin = overrideMin
  relevantActuator.overrideMax = overrideMax

  M.isActingAsYC = (overrideMin > 0.01) or (overrideMax < 0.99)
  return false -- don't let the CMU know that we are acting, otherwise the UI will be notified as well and that might be too intrusive for AWD control
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
  if controlParameters.controlMode == "frontMain" then
    lockOverrideMethod = yawControlFrontMain
  elseif controlParameters.controlMode == "rearMain" then
    lockOverrideMethod = yawControlRearMain
  elseif controlParameters.controlMode == "centerDiff" then
    lockOverrideMethod = yawControlRearMain -- for now use the same as rearMain
  end
end

local function reset()
  M.isActingAsTC = false
  M.isActingAsYC = false
  M.isActingAsABS = false
end

local function init(jbeamData)
  M.isActingAsTC = false
  M.isActingAsYC = false
  M.isActingAsABS = false
end

local function initSecondStage(jbeamData)
  local awdActuatorName = jbeamData.awdActuatorName or "electronicSplitShaftLock"
  relevantActuator = CMU.getActuator(awdActuatorName)

  if relevantActuator then
    local tractionControl = CMU.getSupervisor("tractionControl")
    if tractionControl then
      tractionControl.registerComponent(M)
    end

    local yawControl = CMU.getSupervisor("yawControl")
    if yawControl then
      yawControl.registerComponent(M)
    end
    controlParameters.controlMode = jbeamData.controlMode or "rearMain"
    controlParameters.yawControl.slipAngleThreshold = jbeamData.slipAngleThreshold or 0.2

    applyControlParameters()
  else
    M.actAsTractionControl = nop
    M.actAsYawControl = nop
  end

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
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.isEnabled")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.isEnabled")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.slipAngleThreshold")

  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "controlMode")

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
