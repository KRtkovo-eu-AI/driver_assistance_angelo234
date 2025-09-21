-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 70
M.componentOrderTractionControl = 30
M.componentOrderYawControl = 50
M.componentOrderABSControl = 50
M.isActive = false
M.isActingAsTC = false
M.isActingAsYC = false
M.isActingAsABS = false

local abs = math.abs
local min = math.min
local max = math.max

local CMU = nil
local isDebugEnabled = false

local controlParameters = {tractionControl = {isEnabled = true, wheelGroupSettings = {}}, yawControl = {isEnabled = true, PIDSettings = {}}, absControl = {wheelSettings = {}}}
local initialControlParameters

local configPacket = {sourceType = "brakeControl", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "brakeControl", tractionControl = {wheelGroupControl = {}}, yawControl = {}, absControl = {}}

local wheelControlKeys = {}
local wheelControlKeyCount = 0
local wheelControlData = {}

local wheelGroupControlData = {}

local yawControlAVBrakingPID
local yawControlSlipAngleBrakingPID

local wheelCount = 0

local absWheelData = {}
local absBrakeTorqueLimits = {}
local absBrakeTorqueLimitsCache = {}

local isYCBrakeActive = false
local isTCBrakeActive = false
local isABSBrakeActive = false

local function updateWheelsIntermediate(dt)
  for i = 0, wheelCount - 1 do
    local wheel = wheels.wheels[i]
    local controlData = wheelControlData[wheel.name]
    if controlData then
      wheel.desiredBrakingTorque = max(wheel.desiredBrakingTorque, wheel.brakeTorque * min((controlData.brakeFactorTractionControl or 0) + (controlData.brakeFactorYawControl or 0), 1))
    end
  end
  absBrakeTorqueLimits.left = absBrakeTorqueLimitsCache.left
  absBrakeTorqueLimits.right = absBrakeTorqueLimitsCache.right
  absBrakeTorqueLimits.center = absBrakeTorqueLimitsCache.center
  absBrakeTorqueLimitsCache.left = 0
  absBrakeTorqueLimitsCache.right = 0
  absBrakeTorqueLimitsCache.center = 0
end

--returns true if component did act as traction control
--called from updateFixedStep
local function actAsTractionControl(wheelGroup, dt)
  M.isActingAsTC = false
  local groupControlData = wheelGroupControlData[wheelGroup.name]
  if not groupControlData then
    return false
  end
  if not controlParameters.tractionControl.isEnabled then
    for i = 1, wheelGroup.wheelCount do
      local wheel = wheelGroup.wheels[i]
      local controlData = wheelControlData[wheel.name]

      controlData.brakeFactorTractionControl = 0
    end
    return false
  end

  local maximumSpeedCoef = linearScale(CMU.virtualSensors.virtual.speed, groupControlData.maxVelocity, groupControlData.maxVelocity + 1, 1, 0)
  local slipRangeCoef = linearScale(wheelGroup.slipRange, groupControlData.slipRangeThreshold, groupControlData.slipRangeThreshold * 2, 0, 1)

  local maxBrakeFactor = 0
  for i = 1, wheelGroup.wheelCount do
    local wheel = wheelGroup.wheels[i]
    local controlData = wheelControlData[wheel.name]
    local brakeFactor = controlData.tractionControlBrakingPID:get(-wheel.slip, -groupControlData.slipThreshold, dt) * maximumSpeedCoef * slipRangeCoef
    controlData.brakeFactorTractionControl = brakeFactor
    maxBrakeFactor = max(maxBrakeFactor, brakeFactor)
  end

  M.isActingAsTC = maxBrakeFactor > 0
  --this is used for turning the brakelights on when brakes are used by DSE
  isTCBrakeActive = maxBrakeFactor > 0.1 or isTCBrakeActive

  return M.isActingAsTC
end

--returns true if component did act as yaw control
--called from updateFixedStep
local function actAsYawControl(measuredYaw, expectedYaw, yawDifference, bodySlipAngle, dt)
  M.isActingAsYC = false
  local wheelAccess = CMU.vehicleData.wheelAccess

  for i = 1, wheelControlKeyCount do
    local wheelName = wheelControlKeys[i]
    wheelControlData[wheelName].brakeFactorYawControl = 0
  end

  if not controlParameters.yawControl.isEnabled then
    return false
  end

  local avBrakeFactor = yawControlAVBrakingPID:get(-abs(yawDifference), -controlParameters.yawControl.yawAvThreshold, dt)
  local slipAngleBrakeFactor = yawControlSlipAngleBrakingPID:get(-abs(bodySlipAngle), -controlParameters.yawControl.slipAngleThreshold, dt)

  local wheelToBrake
  if avBrakeFactor > slipAngleBrakeFactor then
    if yawDifference > 0 then --Oversteer
      if measuredYaw > 0 then
        wheelToBrake = wheelAccess.frontLeft
      else
        wheelToBrake = wheelAccess.frontRight
      end
    else --Understeer
      if measuredYaw > 0 then
        wheelToBrake = wheelAccess.rearRight
      else
        wheelToBrake = wheelAccess.rearLeft
      end
    end
  else
    local bsaControlSign = sign(measuredYaw) * sign(bodySlipAngle)
    if bsaControlSign < 0 then --Oversteer
      if measuredYaw < 0 then
        wheelToBrake = wheelAccess.frontRight
      else
        wheelToBrake = wheelAccess.frontLeft
      end
    else
      --BSA can't correctly detect understeer, so nothing to do here
      return false
    end
  end
  local wheelData = wheelControlData[wheelToBrake.name]

  local finalBrakeFactor = max(avBrakeFactor, slipAngleBrakeFactor)
  local antiLockUpFactor = linearScale(abs(wheelToBrake.angularVelocityBrakeCouple), 2, 5, 0, 1)
  wheelData.brakeFactorYawControl = finalBrakeFactor * antiLockUpFactor

  M.isActingAsYC = finalBrakeFactor > 0
  --this is used for turning the brakelights on when brakes are used by DSE
  isYCBrakeActive = wheelData.brakeFactorYawControl > 0.3 or isYCBrakeActive

  return M.isActingAsYC
end

local function updateBrakeNoABS(wd, brake, invAirspeed, airspeed, airspeedCutOff, dt)
  local brakeInputSplit = wd.brakeInputSplit
  return wd.brakeTorque * (min(brake, brakeInputSplit) + max(brake - brakeInputSplit, 0) * wd.brakeSplitCoef)
end

local function updateBrakeABS(wd, brake, invAirspeed, airspeed, airspeedCutOff, dt)
  local absData = absWheelData[wd.name]
  if not absData then
    return 0
  end
  if brake > 0 and wd.brakeTorque > 0 then
    local brakeInputSplit = wd.brakeInputSplit
    local nonABSBrakingTorque = wd.brakeTorque * (min(brake, brakeInputSplit) + max(brake - brakeInputSplit, 0) * wd.brakeSplitCoef)

    local desiredBrakingTorque = nonABSBrakingTorque * absData.absCoef
    absBrakeTorqueLimitsCache[wd.oppositeWheelSide] = max(desiredBrakingTorque * 1.05, absBrakeTorqueLimitsCache[wd.oppositeWheelSide])
    desiredBrakingTorque = min(desiredBrakingTorque, absBrakeTorqueLimits[wd.ownWheelSide])
    absData.lastDesiredBrakingTorque = desiredBrakingTorque

    return desiredBrakingTorque
  else
    absBrakeTorqueLimitsCache[wd.oppositeWheelSide] = max(absBrakeTorqueLimitsCache[wd.oppositeWheelSide], wd.brakeTorque * 2)
    absData.lastDesiredBrakingTorque = 0
    return 0
  end
end

local function actAsABSControl(wheelData, brake, vehicleVelocity, dt)
  M.isActingAsABS = false
  local absData = absWheelData[wheelData.wheelName]
  if not absData then
    return false
  end

  local airspeedCutOffSpeed = 2
  local airspeedCutOff = vehicleVelocity > airspeedCutOffSpeed

  if brake > 0 then
    absData.absTimer = absData.absTimer - dt
    if absData.absTimer <= 0 then
      local absDT = max(dt, absData.absTime) --if the ABS frequency is smaller than the physics step, we need to use the right dt here
      absData.absTimer = absData.absTimer + absData.absTime

      local absCoef = 1 - absData.absBrakingPID:get(-wheelData.slip, -absData.slipThreshold, absDT)
      local steeringCoef = linearScale(abs(CMU.sensorHub.yawAVSmooth), 0, absData.steeringCoefYawRate, 1, absData.minSteeringCoef)
      absCoef = absCoef * steeringCoef
      absData.absCoef = airspeedCutOff and absCoef or 1
      M.isActingAsABS = absData.absCoef < 0.9
    end
  else
    absData.absBrakingPID:reset()
    absData.absTimer = 0
  end

  isABSBrakeActive = false

  return M.isActingAsABS
end

local function updateGFX(dt)
  electrics.values.isYCBrakeActive = isYCBrakeActive and 1 or 0
  electrics.values.isTCBrakeActive = isTCBrakeActive and 1 or 0
  electrics.values.isABSBrakeActive = isABSBrakeActive and 1 or 0
  isYCBrakeActive = false
  isTCBrakeActive = false
  isABSBrakeActive = false
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  debugPacket.wheelData = debugPacket.wheelData or {}
  for k, v in pairs(wheelControlData) do
    debugPacket.wheelData[k] = debugPacket.wheelData[k] or {}
    debugPacket.wheelData[k].brakeFactorTractionControl = v.brakeFactorTractionControl
    debugPacket.wheelData[k].brakeFactorYawControl = v.brakeFactorYawControl
    debugPacket.wheelData[k].brakeFactor = min((v.brakeFactorTractionControl or 0) + (v.brakeFactorYawControl or 0), 1)
  end

  for k, v in pairs(controlParameters.tractionControl.wheelGroupSettings) do
    debugPacket.tractionControl.wheelGroupControl[k] = debugPacket.tractionControl.wheelGroupControl[k] or {}
    debugPacket.tractionControl.wheelGroupControl[k].slipThreshold = v.slipThreshold
    debugPacket.tractionControl.wheelGroupControl[k].slipRangeThreshold = v.slipRangeThreshold
  end

  debugPacket.yawControl.yawAVThreshold = controlParameters.yawControl.yawAvThreshold
  debugPacket.yawControl.slipAngleThreshold = controlParameters.yawControl.slipAngleThreshold

  debugPacket.absControl.wheelData = debugPacket.absControl.wheelData or {}
  for k, v in pairs(absWheelData) do
    debugPacket.absControl.wheelData[k] = debugPacket.absControl.wheelData[k] or {}
    debugPacket.absControl.wheelData[k].absCoef = v.absCoef
    debugPacket.absControl.wheelData[k].minSteeringCoef = v.minSteeringCoef
    debugPacket.absControl.wheelData[k].steeringCoefYawRate = v.steeringCoefYawRate
    debugPacket.absControl.wheelData[k].slipThreshold = v.slipThreshold
    debugPacket.absControl.wheelData[k].brakingTorque = v.lastDesiredBrakingTorque
  end
  debugPacket.absControl.brakeTorqueLimits = absBrakeTorqueLimits

  debugPacket.tractionControl.isActing = M.isActingAsTC
  debugPacket.yawControl.isActing = M.isActingAsYC
  debugPacket.absControl.isActing = M.isActingAsABS

  CMU.sendDebugPacket(debugPacket)
end

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled

  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
end

local function registerCMU(cmu)
  CMU = cmu
end

local function registerWheelBrakeUpdates()
  for wheelName, _ in pairs(absWheelData) do
    wheels.setWheelBrakeUpdate(wheelName, updateBrakeNoABS, updateBrakeABS)
  end
end

local function reset()
  M.isActingAsTC = false
  M.isActingAsYC = false
  M.isActingAsABS = false
  isYCBrakeActive = false
  isTCBrakeActive = false
  isABSBrakeActive = false

  registerWheelBrakeUpdates()
end

local function init(jbeamData)
  M.isActingAsTC = false
  M.isActingAsYC = false
  M.isActingAsABS = false
end

local function applyControlParameters()
  for k, v in pairs(controlParameters.tractionControl.wheelGroupSettings) do
    wheelGroupControlData[k].slipThreshold = v.slipThreshold
    wheelGroupControlData[k].slipRangeThreshold = v.slipRangeThreshold
    wheelGroupControlData[k].maxVelocity = v.maxVelocity
  end

  if controlParameters.tractionControl.wheelBrakingPID then
    for k, v in pairs(controlParameters.tractionControl.wheelBrakingPID) do
      if wheelControlData[k].tractionControlBrakingPID then
        wheelControlData[k].tractionControlBrakingPID:setConfig(v.kP, v.kI, v.kD, 0, 1, v.integralInCoef, v.integralOutCoef, 0)
      end
    end
  end

  if yawControlAVBrakingPID then
    local avSettings = controlParameters.yawControl.PIDSettings.yawAV
    yawControlAVBrakingPID:setConfig(avSettings.kP, avSettings.kI, avSettings.kD, 0, 1, avSettings.integralInCoef, avSettings.integralOutCoef, 0)
  end
  if yawControlSlipAngleBrakingPID then
    local slipAngleSettings = controlParameters.yawControl.PIDSettings.slipAngle
    yawControlSlipAngleBrakingPID:setConfig(slipAngleSettings.kP, slipAngleSettings.kI, slipAngleSettings.kD, 0, 1, slipAngleSettings.integralInCoef, slipAngleSettings.integralOutCoef, 0)
  end

  for k, v in pairs(controlParameters.absControl.wheelSettings) do
    absWheelData[k].slipThreshold = v.slipThreshold
    absWheelData[k].absTime = 1 / v.absFrequency
    absWheelData[k].absBrakingPID:setConfig(v.kP, v.kI, v.kD, 0, 1, v.integralInCoef, v.integralOutCoef, 0)
    absWheelData[k].minSteeringCoef = v.minSteeringCoef
    absWheelData[k].steeringCoefYawRate = v.steeringCoefYawRate
  end
end

local function initSecondStage(jbeamData)
  --todo create jbeam param to determine if component should be used as TC or ESC component

  local useForTractionControl = jbeamData.useForTractionControl == nil and true or jbeamData.useForTractionControl
  local useForYawControl = jbeamData.useForYawControl == nil and true or jbeamData.useForYawControl
  local useForABSControl = jbeamData.useForABSControl == nil and true or jbeamData.useForABSControl

  wheelControlData = {} --TC + YC

  local tractionControl = CMU.getSupervisor("tractionControl")
  if tractionControl and jbeamData.tractionControl and useForTractionControl then
    tractionControl.registerComponent(M)

    wheelGroupControlData = {}
    controlParameters.tractionControl.wheelGroupSettings = {}
    controlParameters.tractionControl.wheelBrakingPID = {}
    local wheelGroupSettings = tableFromHeaderTable(jbeamData.tractionControl.wheelGroupSettings or {})
    local wheelGroupSettingsLookup = {}
    if wheelGroupSettings then
      for _, groupSetting in ipairs(wheelGroupSettings) do
        wheelGroupSettingsLookup[groupSetting.motorName] = groupSetting
        wheelGroupControlData[groupSetting.motorName] = {
          slipThreshold = groupSetting.slipThreshold or 0.2,
          slipRangeThreshold = groupSetting.slipRangeThreshold or 0,
          maxVelocity = groupSetting.maxVelocity or 1
        }
        controlParameters.tractionControl.wheelGroupSettings[groupSetting.motorName] = deepcopy(wheelGroupControlData[groupSetting.motorName])
      end
    end

    for _, wheelData in ipairs(tractionControl.tractionControlledWheels) do
      local groupSetting = wheelGroupSettingsLookup[wheelData.wheelGroup.name]
      if groupSetting then
        local kP = groupSetting.kP or 0
        local kI = groupSetting.kI or 0
        local kD = groupSetting.kD or 0
        local integralInCoef = groupSetting.integralInCoef or 20
        local integralOutCoef = groupSetting.integralOutCoef or 2

        if not wheelControlData[wheelData.wheel.name] then
          wheelControlData[wheelData.wheel.name] = {}
          table.insert(wheelControlKeys, wheelData.wheel.name)
        end
        wheelControlData[wheelData.wheel.name].tractionControlBrakingPID = newPIDParallel(kP, kI, kD, 0, 1, integralInCoef, integralOutCoef, 0)
        wheelControlData[wheelData.wheel.name].brakeFactorTractionControl = 0

        controlParameters.tractionControl.wheelBrakingPID[wheelData.wheel.name] = {
          kP = kP,
          kI = kI,
          kD = kD,
          integralInCoef = integralInCoef,
          integralOutCoef = integralOutCoef
        }
      end
    end
  end

  local yawControl = CMU.getSupervisor("yawControl")
  if yawControl and jbeamData.yawControl and useForYawControl then
    yawControl.registerComponent(M)

    local jbeamPIDSettings = tableFromHeaderTable(jbeamData.yawControl.PIDSettings or {})

    for _, setting in pairs(jbeamPIDSettings) do
      controlParameters.yawControl.PIDSettings[setting.type] = {
        kP = setting.kP,
        kI = setting.kI,
        kD = setting.kD,
        integralInCoef = setting.integralInCoef,
        integralOutCoef = setting.integralOutCoef
      }
    end

    for _, wheel in pairs(CMU.vehicleData.wheelAccess) do
      if not wheelControlData[wheel.name] then
        wheelControlData[wheel.name] = {}
        table.insert(wheelControlKeys, wheel.name)
      end
      wheelControlData[wheel.name].brakeFactorYawControl = 0
    end

    local avSettings = controlParameters.yawControl.PIDSettings.yawAV
    local slipAngleSettings = controlParameters.yawControl.PIDSettings.slipAngle

    yawControlAVBrakingPID = newPIDParallel(avSettings.kP, avSettings.kI, avSettings.kD, 0, 1, avSettings.integralInCoef, avSettings.integralOutCoef, 0)
    yawControlSlipAngleBrakingPID = newPIDParallel(slipAngleSettings.kP, slipAngleSettings.kI, slipAngleSettings.kD, 0, 1, slipAngleSettings.integralInCoef, slipAngleSettings.integralOutCoef, 0)

    controlParameters.yawControl.yawAvThreshold = jbeamData.yawControl.yawAVThreshold or 0.4
    controlParameters.yawControl.slipAngleThreshold = jbeamData.yawControl.slipAngleThreshold or 0.1
  end

  local absControl = CMU.getSupervisor("absControl")
  if absControl and useForABSControl then
    absControl.registerComponent(M)
    controlParameters.absControl.wheelSettings = {}
    local wheelSettings = tableFromHeaderTable(jbeamData.absControl.wheelSettings or {})
    for _, wheelSetting in pairs(wheelSettings) do
      if absControl.absControlledWheels[wheelSetting.wheelName] then
        local absFrequency = wheelSetting.absFrequency or 100
        local slipThreshold = wheelSetting.slipThreshold or 0.2
        local kP = wheelSetting.kP or 5
        local kI = wheelSetting.kI or 0.5
        local kD = wheelSetting.kD or 0.1
        local integralInCoef = wheelSetting.integralInCoef or 20
        local integralOutCoef = wheelSetting.integralOutCoef or 20
        local minSteeringCoef = wheelSetting.minSteeringCoef or 0.3
        local steeringCoefYawRate = wheelSetting.steeringCoefYawRate or 0.2

        controlParameters.absControl.wheelSettings[wheelSetting.wheelName] = {
          absFrequency = absFrequency,
          slipThreshold = slipThreshold,
          kP = kP,
          kI = kI,
          kD = kD,
          integralInCoef = integralInCoef,
          integralOutCoef = integralOutCoef,
          minSteeringCoef = minSteeringCoef,
          steeringCoefYawRate = steeringCoefYawRate
        }

        absWheelData[wheelSetting.wheelName] = {
          absTimer = 0,
          absTime = 1 / absFrequency,
          slipThreshold = slipThreshold,
          absBrakingPID = newPIDParallel(kP, kI, kD, 0, 1, integralInCoef, integralOutCoef, 0),
          absCoef = 1,
          minSteeringCoef = minSteeringCoef,
          steeringCoefYawRate = steeringCoefYawRate,
          lastDesiredBrakingTorque = 0
        }
      end
    end

    absBrakeTorqueLimits.left = 999999
    absBrakeTorqueLimits.right = 999999
    absBrakeTorqueLimits.center = 999999
    absBrakeTorqueLimitsCache.left = 0
    absBrakeTorqueLimitsCache.right = 0
    absBrakeTorqueLimitsCache.center = 0
    registerWheelBrakeUpdates()
  end

  wheelControlKeyCount = #wheelControlKeys
  initialControlParameters = deepcopy(controlParameters)

  M.isActive = true
end

local function initLastStage(jbeamData)
  wheelCount = wheels.wheelCount
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
  --Traction Control
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.isEnabled")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings")

  for k, _ in pairs(wheelGroupControlData) do
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelBrakingPID." .. k .. ".kP")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelBrakingPID." .. k .. ".kI")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelBrakingPID." .. k .. ".kD")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelBrakingPID." .. k .. ".integralInCoef")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelBrakingPID." .. k .. ".integralOutCoef")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings." .. k .. ".slipThreshold")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings." .. k .. ".slipRangeThreshold")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "tractionControl.wheelGroupSettings." .. k .. ".maxVelocity")
  end

  --Yaw Control
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.isEnabled")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.slipAngleThreshold")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "yawControl.yawAvThreshold")

  --ABS Control
  for k, _ in pairs(absWheelData) do
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "absControl.wheelSettings." .. k .. ".kP")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "absControl.wheelSettings." .. k .. ".kI")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "absControl.wheelSettings." .. k .. ".kD")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "absControl.wheelSettings." .. k .. ".integralInCoef")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "absControl.wheelSettings." .. k .. ".integralOutCoef")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "absControl.wheelSettings." .. k .. ".slipThreshold")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "absControl.wheelSettings." .. k .. ".absFrequency")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "absControl.wheelSettings." .. k .. ".minSteeringCoef")
    CMU.applyParameter(controlParameters, initialControlParameters, parameters, "absControl.wheelSettings." .. k .. ".steeringCoefYawRate")
  end

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
M.updateWheelsIntermediate = updateWheelsIntermediate

M.registerCMU = registerCMU
M.setDebugMode = setDebugMode
M.shutdown = shutdown
M.setParameters = setParameters
M.setConfig = setConfig
M.getConfig = getConfig
M.sendConfigData = sendConfigData

M.actAsTractionControl = actAsTractionControl
M.actAsYawControl = actAsYawControl
M.actAsABSControl = actAsABSControl

return M
