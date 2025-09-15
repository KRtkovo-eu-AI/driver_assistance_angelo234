-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 56

M.isActive = false

local sin = math.sin
local cos = math.cos
local tan = math.tan
local arctan = math.atan
local abs = math.abs
local atan2 = math.atan2
local pi = math.pi
local twoPi = pi * 2
local halfPi = pi * 0.5
local max = math.max
local min = math.min

M.reference = {
  bodySlipAngle = 0,
  speed = 0
}

M.virtual = {
  speed = 0, --actual final calculated speed
  integratedSpeed = 0, --integration based speed component
  avgWheelSpeed = 0, --avg speed of all wheels
  avgNonPropulsedWheelSpeed = 0, --avg wheelspeed of all non propulsed wheels
  wheelSpeed = 0, --final calculated wheelspeed
  bodySlipAngle = 0, --final BSA
  bodySlipAngleIntegrated = 0, --BSA based on integratings things (for higher speeds)
  bodySlipAngleLowSpeed = 0, --Estimated BSA based on yaw rate (low speeds)
  lastBodySlipAngle = 0,
  bodySlipAngleRate = 0
  --roll = 0,
  --pitch = 0
  --forceYFrontAxle = 0
}

local resetCriteria = {
  isRollingProbability = 0, --rolling straight without turning, braking or putting power to the wheels
  isStoppedProbability = 0, --stopped, no power to the wheels, might be braking
  isDrivingStraightProbability = 0, --driving in a straight line, power or brakes possible
  isRollingProbabilitySmoother = newTemporalSmoothing(2),
  isStoppedProbabilitySmoother = newTemporalSmoothing(2),
  isDrivingStraightProbabilitySmoother = newTemporalSmoothing(2)
}

M.trustWorthiness = {
  pitch = 1,
  roll = 1,
  jerk = 1,
  bodySlipAngle = 1,
  wheelSpeed = 1,
  virtualSpeed = 1,
  needsFullReset = false
}

local smoother = {
  wheelSpeedTrust = newTemporalSmoothing(100, 5)
}

local boolToNumber = {[true] = 1, [false] = 0}

local wheelCount = 0

local CMU = nil
local isDebugEnabled = false

local debugPacket = {sourceType = "virtualSensors"}

local function updateBodySlipAngle(sinPitch, cosPitch, sinRoll, sinBodySlipAngle, cosBodySlipAngle, gravity, accelerationX, accelerationY, dt)
  local virtual = M.virtual
  local vehicleData = CMU.vehicleData
  local vehicleStats = vehicleData.vehicleStats
  local sensorHub = CMU.sensorHub
  local yawRate = sensorHub.yawAV
  local yawRateSmooth = sensorHub.yawAVSmooth

  local invSpeed = 1 / virtual.speed
  local lowSpeedCoef = linearScale(abs(virtual.speed), 0.1, 1, 0, 1)

  virtual.lastBodySlipAngle = virtual.bodySlipAngle
  local highSpeedBSAChange = -(sinBodySlipAngle * (accelerationX + sinPitch * gravity * 0) * invSpeed) + (cosBodySlipAngle * (accelerationY - sinRoll * cosPitch * gravity * 0) * invSpeed) - yawRate --(5.7)
  virtual.bodySlipAngleIntegrated = virtual.bodySlipAngleIntegrated + highSpeedBSAChange * dt * lowSpeedCoef
  if abs(virtual.bodySlipAngleIntegrated) > twoPi then
    virtual.bodySlipAngleIntegrated = virtual.bodySlipAngleIntegrated - sign(virtual.bodySlipAngleIntegrated) * twoPi
  end

  ---low speed bsa---
  local vSTM = (vehicleData.wheelAccess.frontRight.wheelSpeed + vehicleData.wheelAccess.frontLeft.wheelSpeed) * 0.5
  local vX = vSTM / (cos(vehicleData.frontWheelAngle) + sin(vehicleData.frontWheelAngle) * vehicleData.frontWheelAngle) --(5.38)
  local vY = vX * tan(vehicleData.frontWheelAngle) - yawRateSmooth * vehicleStats.distanceCOGFrontAxle --(5.38)
  virtual.bodySlipAngleLowSpeed = vX ~= 0 and (arctan(vY / vX) * lowSpeedCoef) or 0 --(5.39)
  ------------------

  if virtual.bodySlipAngleIntegrated ~= virtual.bodySlipAngleIntegrated then
    virtual.bodySlipAngleIntegrated = 0
  end

  virtual.bodySlipAngle = linearScale(abs(virtual.speed), 0, 2, virtual.bodySlipAngleLowSpeed, virtual.bodySlipAngleIntegrated)

  virtual.bodySlipAngleRate = abs((virtual.lastBodySlipAngle - virtual.bodySlipAngle)) / dt
end

local function updateIntegratedSpeed(sinPitch, cosPitch, sinRoll, sinBodySlipAngle, cosBodySlipAngle, gravity, accelerationX, accelerationY, dt)
  local virtual = M.virtual
  local dSpeed = (cosBodySlipAngle * (accelerationX + sinPitch * gravity * 0)) + (sinBodySlipAngle * (accelerationY - sinRoll * cosPitch * gravity * 0)) --(5.6)
  virtual.integratedSpeed = virtual.integratedSpeed + dSpeed * dt
end

local function updateWheelSpeed(dt)
  local trustWorthiness = M.trustWorthiness
  local avgWheelSpeed = 0
  local avgNonPropulsedWheelSpeed = 0
  local nonPropulsedWheelCount = 0
  local brakingTrustWorthiness = 1
  local brakingTrustWorthinessNonPropulsed = 1
  local propulsionTrustWorthiness = 1

  for i = 0, wheelCount - 1 do
    local wd = wheels.wheels[i]
    avgWheelSpeed = avgWheelSpeed + wd.wheelSpeed
    local brakeTorqueTrustCoef = abs((abs(wd.coreData.brakeTorqueApplied) - wd.frictionTorque) * 0.01)
    if not wd.isPropulsed then
      nonPropulsedWheelCount = nonPropulsedWheelCount + 1
      avgNonPropulsedWheelSpeed = avgNonPropulsedWheelSpeed + wd.wheelSpeed
      brakingTrustWorthinessNonPropulsed = max(brakingTrustWorthinessNonPropulsed - brakeTorqueTrustCoef, 0)
    end
    local wheelPropulsionDevice = powertrain.getPropulsionDeviceForWheel(wd.name)
    local propulsionTorque = wheelPropulsionDevice and wheelPropulsionDevice.outputTorque1 or 0
    propulsionTrustWorthiness = max(propulsionTrustWorthiness - max(propulsionTorque * 0.005, 0), 0)
    brakingTrustWorthiness = max(brakingTrustWorthiness - brakeTorqueTrustCoef, 0)
  end

  if nonPropulsedWheelCount > 0 then
    M.virtual.wheelSpeed = avgNonPropulsedWheelSpeed / nonPropulsedWheelCount
    trustWorthiness.wheelSpeed = smoother.wheelSpeedTrust:getUncapped(brakingTrustWorthinessNonPropulsed, dt)
  else
    M.virtual.wheelSpeed = avgWheelSpeed / wheels.wheelCount
    trustWorthiness.wheelSpeed = smoother.wheelSpeedTrust:getUncapped(min(brakingTrustWorthiness, propulsionTrustWorthiness), dt)
  end
end

local function updateTrustWorthiness(dt)
  local trustWorthiness = M.trustWorthiness
  local sensorHub = CMU.sensorHub
  local virtual = M.virtual

  local accNoiseX = sensorHub.accNoiseX
  local accNoiseY = sensorHub.accNoiseY
  local accNoiseZ = sensorHub.accNoiseZ
  local accNoiseSum = accNoiseX + accNoiseY + accNoiseZ

  --print(sensorHub.roll)
  trustWorthiness.pitch = linearScale(abs(sensorHub.pitch), 1.4, halfPi, 1, 0)
  trustWorthiness.roll = linearScale(abs(sensorHub.roll), 1.4, halfPi, 1, 0)
  trustWorthiness.bodySlipAngle = linearScale(abs(virtual.bodySlipAngle), 0.8, 1.0, 1, 0)
  trustWorthiness.virtualSpeed = min(trustWorthiness.pitch, trustWorthiness.roll, linearScale(abs(virtual.bodySlipAngle), 0.2, 0.5, 1, 0))

  local absAccX = abs(sensorHub.accelerationXSmooth)
  local absAccY = abs(sensorHub.accelerationYSmooth)
  local absAccZ = abs(sensorHub.accelerationZSmooth - sensorHub.gravity)

  local absPitchAV = abs(sensorHub.pitchAVSmooth)
  local absRollAV = abs(sensorHub.rollAVSmooth)
  local absYawAV = abs(sensorHub.yawAVSmooth)

  if not trustWorthiness.needsFullReset then
    local bodySlipAngleRunAway = virtual.bodySlipAngleRate > 1000 and abs(virtual.bodySlipAngle) > 1
    local bodySlipAngleRateTooHigh = virtual.bodySlipAngleRate > 5 and abs(virtual.wheelSpeed) > 5
    local bodySlipAngleTooHighLowSpeed = abs(virtual.bodySlipAngle) > 1 and abs(virtual.wheelSpeed) < 1
    local slipAngleTooHigh = abs(virtual.bodySlipAngle) > 2
    local pitchRunAway = false
    local rollRunAway = false
    local accXTooHigh = absAccX > 20
    local accYTooHigh = absAccY > 30
    local accZTooHigh = absAccZ > 40
    local accTooHigh = accXTooHigh or accYTooHigh or accZTooHigh
    local avTooHigh = max(absPitchAV, absRollAV, absYawAV) > 5
    local accNoiseTooHigh = accNoiseSum > 100

    if bodySlipAngleRunAway or bodySlipAngleRateTooHigh or bodySlipAngleTooHighLowSpeed or slipAngleTooHigh or pitchRunAway or rollRunAway or accTooHigh or avTooHigh or accNoiseTooHigh then
      trustWorthiness.needsFullReset = true
    end
  end

  local accXLow = absAccX < 1.5
  local accXVeryLow = absAccX < 1.0
  local accYLow = absAccY < 3
  local accYVeryLow = absAccY < 2
  local accZLow = absAccZ < 2

  local pitchLow = abs(sensorHub.pitch) < 0.8
  local rollLow = abs(sensorHub.roll) < 0.8

  local pitchAVLow = absPitchAV < 0.2
  local rollAVLow = absRollAV < 0.2
  local yawAVLow = absYawAV < 0.2

  local wheelSpeedTrustHigh = trustWorthiness.wheelSpeed >= 0.95
  local wheelSpeedLow = abs(virtual.wheelSpeed) < 0.2

  local electricsValues = electrics.values

  local throttleZero = electricsValues.throttle <= 0
  local throttleLow = electricsValues.throttle <= 0.2
  local brakeLow = electricsValues.brake <= 0.2
  local steeringZero = abs(electricsValues.steering_input) <= 0.05

  if trustWorthiness.needsFullReset then
    trustWorthiness.bodySlipAngle = 0
    trustWorthiness.virtualSpeed = 0
  end

  local sharedProbability = boolToNumber[accXLow] + boolToNumber[accZLow] + boolToNumber[pitchLow] + boolToNumber[rollLow] + boolToNumber[pitchAVLow] + boolToNumber[rollAVLow] + boolToNumber[yawAVLow]
  local rawIsRollingProbability = (sharedProbability + boolToNumber[accYLow] + boolToNumber[wheelSpeedTrustHigh] + boolToNumber[throttleLow] + boolToNumber[brakeLow]) * 0.0909090909 -- / 11
  local rawIsStoppedProbability = (sharedProbability + boolToNumber[accYVeryLow] + boolToNumber[wheelSpeedLow] + boolToNumber[throttleZero]) * 0.1 -- / 10
  local rawIsDrivingStraight = (boolToNumber[accXVeryLow] + boolToNumber[accZLow] + boolToNumber[pitchLow] + boolToNumber[rollLow] + boolToNumber[pitchAVLow] + boolToNumber[rollAVLow] + boolToNumber[yawAVLow] + boolToNumber[steeringZero]) * 0.125 --/ 8

  resetCriteria.isRollingProbability = resetCriteria.isRollingProbabilitySmoother:getUncapped(rawIsRollingProbability, dt)
  resetCriteria.isStoppedProbability = resetCriteria.isStoppedProbabilitySmoother:getUncapped(rawIsStoppedProbability, dt)
  resetCriteria.isDrivingStraightProbability = resetCriteria.isDrivingStraightProbabilitySmoother:getUncapped(rawIsDrivingStraight, dt)

  if resetCriteria.isStoppedProbability >= 0.99 then
    virtual.bodySlipAngle = 0
    virtual.bodySlipAngleIntegrated = 0
    virtual.integratedSpeed = 0

    trustWorthiness.virtualSpeed = 1
    trustWorthiness.bodySlipAngle = 1
    trustWorthiness.needsFullReset = false
  elseif resetCriteria.isRollingProbability >= 0.99 then
    virtual.bodySlipAngle = 0
    virtual.bodySlipAngleIntegrated = 0
    virtual.integratedSpeed = virtual.wheelSpeed

    trustWorthiness.virtualSpeed = 1
    trustWorthiness.bodySlipAngle = 1
    trustWorthiness.needsFullReset = false
  elseif resetCriteria.isDrivingStraightProbability >= 0.99 then
    virtual.bodySlipAngle = 0
    virtual.bodySlipAngleIntegrated = 0

    trustWorthiness.bodySlipAngle = 1
  end
end

local function updateFixedStep(dt)
  updateWheelSpeed(dt)
  updateTrustWorthiness(dt)
end

local function update(dt)
  local sensorHub = CMU.sensorHub
  local virtual = M.virtual
  local sinPitch = sin(sensorHub.pitch)
  local cosPitch = cos(sensorHub.pitch)
  local sinRoll = sin(sensorHub.roll)
  local sinBodySlipAngle = sin(virtual.bodySlipAngle)
  local cosBodySlipAngle = cos(virtual.bodySlipAngle)
  local gravity = sensorHub.gravity
  local accelerationX = -sensorHub.accelerationY --X from the paper is Y in the game
  local accelerationY = -sensorHub.accelerationX --Y from the paper is X in the game

  updateBodySlipAngle(sinPitch, cosPitch, sinRoll, sinBodySlipAngle, cosBodySlipAngle, gravity, accelerationX, accelerationY, dt)
  updateIntegratedSpeed(sinPitch, cosPitch, sinRoll, sinBodySlipAngle, cosBodySlipAngle, gravity, accelerationX, accelerationY, dt)

  virtual.speed = virtual.integratedSpeed
end

local function updateDebug(dt)
  update(dt)
  local reference = M.reference

  local bsa = atan2(CMU.sensorHub.vX, CMU.sensorHub.vY)
  local bsaSign = sign(bsa)
  reference.bodySlipAngle = (bsa - halfPi * bsaSign) * clamp(abs(reference.speed) * 1, 0, 1)
  reference.speed = CMU.sensorHub.vX
end

local function updateGFX(dt)
end

local function updateGFXDebug(dt)
  updateGFX(dt)
  local virtual = M.virtual
  local reference = M.reference
  local trustWorthiness = M.trustWorthiness

  debugPacket.referenceBodySlipAngle = reference.bodySlipAngle
  debugPacket.referenceSpeed = reference.speed

  debugPacket.virtualBodySlipAngle = virtual.bodySlipAngle
  debugPacket.virtualBodySlipAngleIntegrated = virtual.bodySlipAngleIntegrated
  debugPacket.virtualBodySlipAngleLowSpeed = virtual.bodySlipAngleLowSpeed
  debugPacket.virtualSpeed = virtual.speed
  debugPacket.virtualSpeedOffset = virtual.speed - reference.speed

  debugPacket.virtualSpeedOld = electrics.values.virtualAirspeed

  debugPacket.avgWheelSpeed = virtual.avgWheelSpeed
  debugPacket.avgNonPropulsedWheelspeed = virtual.avgNonPropulsedWheelSpeed
  debugPacket.wheelSpeed = virtual.wheelSpeed
  debugPacket.wheelSpeedOffset = virtual.wheelSpeed - reference.speed

  debugPacket.wheelVirtualSpeedOffset = virtual.speed - virtual.wheelSpeed

  debugPacket.pitchTrust = trustWorthiness.pitch
  debugPacket.rollTrust = trustWorthiness.roll
  debugPacket.bodySlipAngleTrust = trustWorthiness.bodySlipAngle
  debugPacket.wheelSpeedTrust = trustWorthiness.wheelSpeed
  debugPacket.virtualSpeedTrust = trustWorthiness.virtualSpeed

  debugPacket.resetIsStopped = resetCriteria.isStoppedProbability
  debugPacket.resetIsRolling = resetCriteria.isRollingProbability
  debugPacket.resetIsDrivingStraight = resetCriteria.isDrivingStraightProbability

  CMU.sendDebugPacket(debugPacket)
end

local function init(jbeamData)
  M.reference.bodySlipAngle = 0
  M.reference.speed = 0

  M.virtual.speed = 0
  M.virtual.integratedSpeed = 0
  M.virtual.wheelSpeed = 0
  M.virtual.bodySlipAngle = 0
  M.virtual.bodySlipAngleIntegrated = 0
  M.virtual.bodySlipAngleLowSpeed = 0
  M.virtual.lastBodySlipAngle = 0
  M.virtual.bodySlipAngleRate = 0

  M.trustWorthiness.pitch = 1
  M.trustWorthiness.roll = 1
  M.trustWorthiness.bodySlipAngle = 1
  M.trustWorthiness.virtualSpeed = 1
  M.trustWorthiness.needsFullReset = false

  M.isActive = true
end

local function initLastStage()
  wheelCount = wheels.wheelCount
end

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled

  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
  M.update = isDebugEnabled and updateDebug or update
end

local function registerCMU(cmu)
  CMU = cmu
end

local function shutdown()
  M.isActive = false
  M.updateGFX = nil
  M.update = nil
  M.updateWheelsIntermediate = nil
end

M.init = init
M.initLastStage = initLastStage
M.updateGFX = updateGFX
M.update = update
M.updateFixedStep = updateFixedStep

M.registerCMU = registerCMU
M.setDebugMode = setDebugMode
M.shutdown = shutdown

return M
