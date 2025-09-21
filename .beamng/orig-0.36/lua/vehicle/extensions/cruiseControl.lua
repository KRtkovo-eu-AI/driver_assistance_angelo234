-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.hasReachedTargetSpeed = false
M.minimumSpeed = 30 / 3.6

local max = math.max

local targetAcceleration = 3

local isEnabled = false
local targetSpeed = 100 / 3.6
local rampedTargetSpeed = 0
local state = {}
local disableOnReset = false
local throttleSmooth = newTemporalSmoothing(200, 200)
local speedPID = newPIDStandard(0.3, 2, 0.0, 0, 1, 1, 1, 0, 1)
--speedPID:setDebug(true)

local function onReset()
  log("D", "CruiseControl", "Cruise Control online")
  if disableOnReset then
    isEnabled = false
    electrics.values.throttleOverride = nil
  end
  M.hasReachedTargetSpeed = false
  state = {}
  throttleSmooth:reset()
end

local function updateGFX(dt)
  if not isEnabled then
    return
  end

  --check for post crash brake triggered
  if electrics.values.postCrashBrakeTriggered then
    M.setEnabled(false)
    return
  end

  if input.brake > 0 then
    --disable cruise control when braking
    M.setEnabled(false)
    return
  end

  if input.clutch > 0 or input.throttle > 0 then
    --dont't do anything if we use the clutch or if we manually input a throttle value

    electrics.values.throttleOverride = input.throttle
    return
  end

  --ramp up/down our target speed with our desired target acceleration to avoid integral wind-up
  if rampedTargetSpeed ~= targetSpeed then
    local upperLimit = targetSpeed > rampedTargetSpeed and targetSpeed or rampedTargetSpeed
    local lowerLimit = targetSpeed < rampedTargetSpeed and targetSpeed or rampedTargetSpeed
    rampedTargetSpeed = clamp(rampedTargetSpeed + sign(targetSpeed - rampedTargetSpeed) * targetAcceleration * dt, lowerLimit, upperLimit)
  end

  local currentSpeed = electrics.values.wheelspeed or 0
  local output = speedPID:get(currentSpeed, rampedTargetSpeed, dt)
  electrics.values.throttleOverride = throttleSmooth:getUncapped(output, dt)

  local currentError = currentSpeed - targetSpeed
  M.hasReachedTargetSpeed = math.abs(currentError) / targetSpeed <= 0.03
end

local function setSpeed(speed)
  isEnabled = true
  targetSpeed = max(speed, M.minimumSpeed)
  rampedTargetSpeed = electrics.values.wheelspeed or 0
  M.hasReachedTargetSpeed = false
  M.requestState()
end

local function changeSpeed(offset)
  isEnabled = true
  targetSpeed = max(targetSpeed + offset, M.minimumSpeed)
  rampedTargetSpeed = electrics.values.wheelspeed or 0
  M.hasReachedTargetSpeed = false
  M.requestState()
end

local function holdCurrentSpeed()
  local currentSpeed = electrics.values.wheelspeed or 0
  if currentSpeed > M.minimumSpeed then
    setSpeed(currentSpeed)
  end
  M.requestState()
end

local function setEnabled(enabled)
  isEnabled = enabled
  M.hasReachedTargetSpeed = false
  electrics.values.throttleOverride = nil
  rampedTargetSpeed = electrics.values.wheelspeed or 0
  throttleSmooth:reset()
  M.requestState()
end

local function setTargetAcceleration(target)
  targetAcceleration = target
end

local function requestState()
  state.targetSpeed = targetSpeed
  state.isEnabled = isEnabled

  electrics.values.cruiseControlTarget = targetSpeed
  electrics.values.cruiseControlActive = isEnabled

  if not playerInfo.firstPlayerSeated then
    return
  end
  guihooks.trigger("CruiseControlState", state)
end

local function getConfiguration()
  return {isEnabled = isEnabled, targetSpeed = targetSpeed, minimumSpeed = M.minimumSpeed, hasReachedTargetSpeed = M.hasReachedTargetSpeed}
end

-- public interface
M.onReset = onReset
M.updateGFX = updateGFX
M.setSpeed = setSpeed
M.changeSpeed = changeSpeed
M.holdCurrentSpeed = holdCurrentSpeed
M.setEnabled = setEnabled
M.requestState = requestState
M.getConfiguration = getConfiguration
M.setTargetAcceleration = setTargetAcceleration

return M
