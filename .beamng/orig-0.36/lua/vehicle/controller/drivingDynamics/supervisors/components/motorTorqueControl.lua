-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 70
M.componentOrderTractionControl = 40
M.componentOrderYawControl = 40
M.componentOrderABSControl = 1000 --this can't be used for ABS control
M.isActive = false
M.isActingAsTC = false
M.isActingAsYC = false
M.isActingAsABS = false

local abs = math.abs
local min = math.min
local max = math.max

local CMU = nil
local isDebugEnabled = false

local controlParameters = {}
local initialControlParameters

local configPacket = {sourceType = "motorTorqueControl", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "motorTorqueControl", wheelGroups = {}, yawControl = {}, tractionControl = {wheelGroupControl = {}}}

local yawControlAVPID
local yawControlSlipAnglePID

local wheelGroupControlData = {}

local throttleFactors = {}
local controlledMotors = {}
local motorCount = 0

local function updateFixedStep(dt)
  for i = 1, motorCount do
    local motorName = controlledMotors[i]
    local throttleFactorData = throttleFactors[motorName]
    local motor = powertrain.getDevice(motorName)
    if motor then
      motor.throttleFactor = min(throttleFactorData.tractionControl, throttleFactorData.yawControl)
    end
  end
end

--returns true if component did act as traction control
--called from updateFixedStep
local function actAsTractionControl(wheelGroup, dt)
  M.isActingAsTC = false
  if not wheelGroupControlData[wheelGroup.motor.name] then
    return false
  end

  local motor = wheelGroup.motor
  local controlData = wheelGroupControlData[motor.name]

  if not controlParameters.tractionControl.isEnabled then
    controlData.throttleFactorTractionControl = 1
    throttleFactors[motor.name].tractionControl = 1
    return false
  end

  local output = controlData.throttleFactorSmoother:getUncapped(1 - controlData.tractionControlPID:get(-wheelGroup.maxSlip, -controlData.slipThreshold, dt), dt)
  output = max(output, controlData.minimumThrottleLimit)
  controlData.throttleFactorTractionControl = output
  throttleFactors[motor.name].tractionControl = output

  M.isActingAsTC = output < 1
  return M.isActingAsTC
end

--returns true if component did act as yaw control
--called from updateFixedStep
local function actAsYawControl(measuredYaw, expectedYaw, yawDifference, bodySlipAngle, dt)
  M.isActingAsYC = false
  if not controlParameters.yawControl.isEnabled then
    for _, motorName in ipairs(controlledMotors) do
      throttleFactors[motorName].yawControl = 1
    end
    return false
  end

  local avFactor = 1 - yawControlAVPID:get(-abs(yawDifference), -controlParameters.yawControl.yawAvThreshold, dt)
  local bsaControlSign = sign(measuredYaw) * sign(bodySlipAngle) --negative if oversteering
  local correctedBSA = min(abs(bodySlipAngle) * bsaControlSign, 0)
  local slipAngleFactor = 1 - yawControlSlipAnglePID:get(-abs(correctedBSA), -controlParameters.yawControl.slipAngleThreshold, dt)
  local throttleFactor = max(min(avFactor, slipAngleFactor), controlParameters.yawControl.minimumThrottleLimit)

  for _, motorName in ipairs(controlledMotors) do
    throttleFactors[motorName].yawControl = throttleFactor
  end

  M.isActingAsYC = throttleFactor < 1
  return M.isActingAsYC
end

local function updateGFX(dt)
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  for k, v in pairs(wheelGroupControlData) do
    debugPacket.wheelGroups[k] = debugPacket.wheelGroups[k] or {}
    debugPacket.wheelGroups[k].throttleFactorTractionControl = throttleFactors[k].tractionControl
    debugPacket.wheelGroups[k].throttleFactorYawControl = throttleFactors[k].yawControl
    debugPacket.wheelGroups[k].throttleFactor = min(throttleFactors[k].tractionControl or 1, throttleFactors[k].yawControl or 1)
    debugPacket.wheelGroups[k].slipThreshold = v.slipThreshold
  end

  if controlParameters.tractionControl then
    for k, v in pairs(controlParameters.tractionControl.wheelGroupSettings) do
      debugPacket.tractionControl.wheelGroupControl[k] = debugPacket.tractionControl.wheelGroupControl[k] or {}
      debugPacket.tractionControl.wheelGroupControl[k].slipThreshold = v.slipThreshold
    end
  end

  if controlParameters.yawControl then
    debugPacket.yawControl.yawAVThreshold = controlParameters.yawControl.yawAvThreshold
    debugPacket.yawControl.slipAngleThreshold = controlParameters.yawControl.slipAngleThreshold
  end

  debugPacket.tractionControl.isActing = M.isActingAsTC
  debugPacket.yawControl.isActing = M.isActingAsYC

  --TODO Remove?
  --debugPacket.isEnabledTractionControl = controlParameters.tractionControl.isEnabled
  --debugPacket.isEnabledYawControl = controlParameters.yawControl.isEnabled

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
  M.isActingAsTC = false
  M.isActingAsYC = false
  M.isActingAsABS = false

  if controlParameters.tractionControl then
    for k, _ in pairs(controlParameters.tractionControl.wheelGroupSettings) do
      wheelGroupControlData[k].tractionControlPID:reset()
    end
  end
end

local function init(jbeamData)
  M.isActive = true
  M.isActingAsTC = false
  M.isActingAsYC = false
  M.isActingAsABS = false
end

local function applyControlParameters()
  if controlParameters.tractionControl then
    for k, v in pairs(controlParameters.tractionControl.wheelGroupSettings) do
      wheelGroupControlData[k].slipThreshold = v.slipThreshold
      wheelGroupControlData[k].minimumThrottleLimit = v.minimumThrottleLimit
      wheelGroupControlData[k].tractionControlPID:setConfig(v.kP, v.kI, v.kD, 0, 1, v.integralInCoef, v.integralOutCoef, 0)
    end
  end
end

local function initSecondStage(jbeamData)
  local useForTractionControl = jbeamData.useForTractionControl == nil and true or jbeamData.useForTractionControl
  local useForYawControl = jbeamData.useForYawControl == nil and true or jbeamData.useForYawControl

  local relevantMotorNames = {}

  local tractionControl = CMU.getSupervisor("tractionControl")
  if tractionControl and jbeamData.tractionControl and useForTractionControl then
    tractionControl.registerComponent(M)
    controlParameters.tractionControl = {isEnabled = true, wheelGroupSettings = {}}

    wheelGroupControlData = {}
    --TODO we should make sure there's at least dummy data for all existing wheelGroups, even if not configured in jbeam to avoid expensive runtime checks
    local wheelGroupSettings = tableFromHeaderTable(jbeamData.tractionControl.wheelGroupSettings or {})
    if wheelGroupSettings then
      for _, groupSetting in ipairs(wheelGroupSettings) do
        local kP = groupSetting.kP or 0
        local kI = groupSetting.kI or 0
        local kD = groupSetting.kD or 0
        local integralInCoef = groupSetting.integralInCoef or 20
        local integralOutCoef = groupSetting.integralOutCoef or 2
        local slipThreshold = groupSetting.slipThreshold or 0.2
        local minimumThrottleLimit = groupSetting.minimumThrottleLimit or 0

        wheelGroupControlData[groupSetting.motorName] = {
          tractionControlPID = newPIDParallel(kP, kI, kD, 0, 1, integralInCoef, integralOutCoef, 0),
          slipThreshold = slipThreshold,
          minimumThrottleLimit = minimumThrottleLimit,
          throttleFactorSmoother = newTemporalSmoothing(5, 5) --TODO Expose to jbeam
        }

        controlParameters.tractionControl.wheelGroupSettings[groupSetting.motorName] = {
          slipThreshold = slipThreshold,
          minimumThrottleLimit = minimumThrottleLimit,
          kP = kP,
          kI = kI,
          kD = kD,
          integralInCoef = integralInCoef,
          integralOutCoef = integralOutCoef
        }

        relevantMotorNames[groupSetting.motorName] = true
      end
    end
  end

  local yawControl = CMU.getSupervisor("yawControl")
  if yawControl and jbeamData.yawControl and useForYawControl then
    yawControl.registerComponent(M)
    controlParameters.yawControl = {isEnabled = true, PIDSettings = {}}

    local jbeamPIDSettings
    if jbeamData.yawControl.PIDSettings then
      jbeamPIDSettings = tableFromHeaderTable(jbeamData.yawControl.PIDSettings)
    else
      jbeamPIDSettings = {
        {type = "yawAV", kP = 2, kI = 0.5, kD = 0, integralInCoef = 100, integralOutCoef = 10},
        {type = "slipAngle", kP = 2, kI = 1.0, kD = 0, integralInCoef = 10, integralOutCoef = 1}
      }
    end

    controlParameters.yawControl.PIDSettings = {}

    for _, setting in pairs(jbeamPIDSettings) do
      controlParameters.yawControl.PIDSettings[setting.type] = {
        kP = setting.kP,
        kI = setting.kI,
        kD = setting.kD,
        integralInCoef = setting.integralInCoef,
        integralOutCoef = setting.integralOutCoef
      }
    end

    local yawControlledMotors = jbeamData.yawControlledMotors or {}
    controlParameters.yawControl.controlledMotorNames = {}
    for _, v in pairs(yawControlledMotors) do
      relevantMotorNames[v] = true
    end

    --TODO safeguard against missing settings (only av or slipangle)
    local avSettings = controlParameters.yawControl.PIDSettings.yawAV
    local slipAngleSettings = controlParameters.yawControl.PIDSettings.slipAngle

    yawControlAVPID = newPIDParallel(avSettings.kP, avSettings.kI, avSettings.kD, 0, 1, avSettings.integralInCoef, avSettings.integralOutCoef, 0)
    yawControlSlipAnglePID = newPIDParallel(slipAngleSettings.kP, slipAngleSettings.kI, slipAngleSettings.kD, 0, 1, slipAngleSettings.integralInCoef, slipAngleSettings.integralOutCoef, 0)

    controlParameters.yawControl.yawAvThreshold = jbeamData.yawControl.yawAVThreshold or 0.4
    controlParameters.yawControl.slipAngleThreshold = jbeamData.yawControl.slipAngleThreshold or 0.1
    controlParameters.yawControl.minimumThrottleLimit = jbeamData.yawControl.minimumThrottleLimit or 0.0
  end

  throttleFactors = {}
  controlledMotors = {}
  motorCount = 0

  for k, _ in pairs(relevantMotorNames) do
    table.insert(controlledMotors, k)
    throttleFactors[k] = {tractionControl = 1, yawControl = 1}
  end
  motorCount = #controlledMotors

  applyControlParameters()

  initialControlParameters = deepcopy(controlParameters)
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
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings")

  for k, _ in pairs(wheelGroupControlData) do
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings." .. k .. ".kP")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings." .. k .. ".kI")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings." .. k .. ".kD")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings." .. k .. ".integralInCoef")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings." .. k .. ".integralOutCoef")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings." .. k .. ".slipThreshold")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings." .. k .. ".minimumThrottleLimit")
  end

  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.isEnabled")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.yawAvThreshold")
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
M.updateFixedStep = updateFixedStep

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
