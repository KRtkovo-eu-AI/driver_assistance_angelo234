-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 70
M.componentOrderTractionControl = 20
M.componentOrderYawControl = 10
M.componentOrderABSControl = 1000 --this can't be used for ABS control
M.isActive = false
M.isActingAsTC = false
M.isActingAsYC = false
M.isActingAsABS = false

local min = math.min
local abs = math.abs

local CMU = nil
local isDebugEnabled = false

local controlParameters = {
  tractionControl = {
    isEnabled = true
  },
  yawControl = {
    isEnabled = true,
    openFrontDiffOversteerThresholdMin = 0,
    openFrontDiffOversteerThresholdMax = 0,
    openRearDiffOversteerThresholdMin = 0,
    openRearDiffOversteerThresholdMax = 0,
    lockableFrontDiffOversteerThresholdMin = 0,
    lockableFrontDiffOversteerThresholdMax = 0,
    lockableRearDiffOversteerThresholdMin = 0,
    lockableRearDiffOversteerThresholdMax = 0
  }
}
local initialControlParameters

local configPacket = {sourceType = "diffControl", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "diffControl", tractionControl = {}, yawControl = {}}

local relevantFrontDiff
local relevantRearDiff
local yawControlFrontDiffMethod
local yawControlRearDiffMethod

--returns true if component did act as traction control
--called from updateFixedStep
local function actAsTractionControl(wheelGroup, dt)
  M.isActingAsTC = false
  return M.isActingAsTC
end

local function yawControlOpenFrontDiff(measuredYaw, expectedYaw, yawDifference, bsa)
  local frontOverrideMin = 0 --not used with this diff type
  local frontOverrideMax = 1 --TODO add when reliable understeer prediction exists

  relevantFrontDiff.overrideMin = frontOverrideMin
  relevantFrontDiff.overrideMax = frontOverrideMax

  return frontOverrideMax < 1
end

local function yawControlOpenRearDiff(measuredYaw, expectedYaw, yawDifference, bsa)
  local rearOverrideMin = 0 --not used with this diff type
  local rearOverrideMax = linearScale(bsa, controlParameters.yawControl.openRearDiffOversteerThresholdMax, controlParameters.yawControl.openRearDiffOversteerThresholdMin, 0, 1)

  relevantRearDiff.overrideMin = rearOverrideMin
  relevantRearDiff.overrideMax = rearOverrideMax

  return rearOverrideMax < 1
end

local function yawControlLockableFrontDiff(measuredYaw, expectedYaw, yawDifference, bsa)
  local frontOverrideMin = linearScale(bsa, controlParameters.yawControl.lockableFrontDiffOversteerThresholdMax, controlParameters.yawControl.lockableFrontDiffOversteerThresholdMin, 1, 0)
  local frontOverrideMax = 1 --todo add when reliable understeer prediction exists

  relevantFrontDiff.overrideMin = frontOverrideMin
  relevantFrontDiff.overrideMax = frontOverrideMax

  return (frontOverrideMin > 0) or (frontOverrideMax < 1)
end

local function yawControlLockableRearDiff(measuredYaw, expectedYaw, yawDifference, bsa)
  local rearOverrideMin = 0 --todo add when reliable understeer prediction exists
  local rearOverrideMax = linearScale(bsa, controlParameters.yawControl.lockableRearDiffOversteerThresholdMax, controlParameters.yawControl.lockableRearDiffOversteerThresholdMin, 0, 1)

  relevantRearDiff.overrideMin = rearOverrideMin
  relevantRearDiff.overrideMax = rearOverrideMax

  return (rearOverrideMin > 0) or (rearOverrideMax < 1)
end

local function yawControlBiasFrontDiff(measuredYaw, expectedYaw, yawDifference, bsa)
  --TODO
  local frontOverride = 1

  relevantFrontDiff.overrideMin = -frontOverride
  relevantFrontDiff.overrideMax = frontOverride

  return false
end

local function yawControlBiasRearDiff(measuredYaw, expectedYaw, yawDifference, bsa)
  local wheelSideIndex = 0
  if measuredYaw > 0 then
    wheelSideIndex = relevantRearDiff.wheelSides.right
  elseif measuredYaw < 0 then
    wheelSideIndex = relevantRearDiff.wheelSides.left
  end
  --TODO
  local rearOverrideMagnitude = linearScale(bsa, -0.2, -0.1, 0.2, 0)
  local rearOverrideSign = wheelSideIndex

  relevantRearDiff.overrideMin = rearOverrideMagnitude * rearOverrideSign
  relevantRearDiff.overrideMax = rearOverrideMagnitude * rearOverrideSign

  return false
end

--returns true if component did act as yaw control
--called from updateFixedStep
local function actAsYawControl(measuredYaw, expectedYaw, yawDifference, bodySlipAngle, dt)
  M.isActingAsYC = false
  if relevantFrontDiff then
    relevantFrontDiff.resetOverride()
  end
  if relevantRearDiff then
    relevantRearDiff.resetOverride()
  end

  if controlParameters.yawControl.isEnabled then
    if electrics.values.throttle > 0 then
      local bsaControlSign = sign(measuredYaw) * sign(bodySlipAngle) --negative if oversteering
      local correctedBSA = -min(abs(bodySlipAngle) * bsaControlSign, 0)
      local isActingFront = yawControlFrontDiffMethod(measuredYaw, expectedYaw, yawDifference, correctedBSA)
      local isActingRear = yawControlRearDiffMethod(measuredYaw, expectedYaw, yawDifference, correctedBSA)
      M.isActingAsYC = isActingFront or isActingRear
    end
  end
  return false -- don't let the CMU know that we are acting, otherwise the UI will be notified as well and that might be too intrusive for AWD control
end

local function updateGFX(dt)
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  debugPacket.tractionControl.isActing = M.isActingAsTC
  debugPacket.yawControl.isActing = M.isActingAsYC

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
end

local function init(jbeamData)
  M.isActingAsTC = false
  M.isActingAsYC = false
  M.isActingAsABS = false
end

local function initSecondStage(jbeamData)
  local frontDiffName = jbeamData.frontDiffName or "lockFront"
  local rearDiffName = jbeamData.rearDiffName or "lockRear"
  relevantFrontDiff = controller.getController(frontDiffName)
  relevantRearDiff = controller.getController(rearDiffName)

  if relevantFrontDiff or relevantRearDiff then
    local tractionControl = CMU.getSupervisor("tractionControl")
    if tractionControl then
      tractionControl.registerComponent(M)
    end

    local yawControl = CMU.getSupervisor("yawControl")
    if yawControl then
      yawControl.registerComponent(M)
    end

    local frontDiffTypeLookup = {
      --["drivingDynamics/actuators/activeDiffBias"] = yawControlBiasFrontDiff,
      ["drivingDynamics/actuators/activeDiffLock"] = yawControlLockableFrontDiff,
      ["drivingDynamics/actuators/electronicDiffLock"] = yawControlOpenFrontDiff
    }
    local rearDiffTypeLookup = {
      --["drivingDynamics/actuators/activeDiffBias"] = yawControlBiasRearDiff,
      ["drivingDynamics/actuators/activeDiffLock"] = yawControlLockableRearDiff,
      ["drivingDynamics/actuators/electronicDiffLock"] = yawControlOpenRearDiff
    }

    yawControlFrontDiffMethod = (relevantFrontDiff and frontDiffTypeLookup[relevantFrontDiff.typeName]) or nop
    yawControlRearDiffMethod = (relevantRearDiff and rearDiffTypeLookup[relevantRearDiff.typeName]) or nop

    controlParameters.yawControl.openFrontDiffOversteerThresholdMin = jbeamData.openFrontDiffOversteerThresholdMin or 0
    controlParameters.yawControl.openFrontDiffOversteerThresholdMax = jbeamData.openFrontDiffOversteerThresholdMax or 0
    controlParameters.yawControl.openRearDiffOversteerThresholdMin = jbeamData.openRearDiffOversteerThresholdMin or 0
    controlParameters.yawControl.openRearDiffOversteerThresholdMax = jbeamData.openRearDiffOversteerThresholdMax or 0.1

    controlParameters.yawControl.lockableFrontDiffOversteerThresholdMin = jbeamData.lockableFrontDiffOversteerThresholdMin or 0
    controlParameters.yawControl.lockableFrontDiffOversteerThresholdMax = jbeamData.lockableFrontDiffOversteerThresholdMax or 0.2
    controlParameters.yawControl.lockableRearDiffOversteerThresholdMin = jbeamData.lockableRearDiffOversteerThresholdMin or 0
    controlParameters.yawControl.lockableRearDiffOversteerThresholdMax = jbeamData.lockableRearDiffOversteerThresholdMax or 0.2

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
  --CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl")
  --CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.isEnabled")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.isEnabled")

  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.openFrontDiffOversteerThresholdMin")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.openFrontDiffOversteerThresholdMax")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.openRearDiffOversteerThresholdMin")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.openRearDiffOversteerThresholdMax")

  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.lockableFrontDiffOversteerThresholdMin")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.lockableFrontDiffOversteerThresholdMax")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.lockableRearDiffOversteerThresholdMin")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.lockableRearDiffOversteerThresholdMax")

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
