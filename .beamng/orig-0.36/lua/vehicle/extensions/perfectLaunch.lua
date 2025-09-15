-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.launchFailed = false

local min = math.min
local max = math.max

local throttle = 1
local state = "idle"
local throttleIntegral = 0
local targetPos = nil
local yawSmoother = nil

local function onInit()
  state = "idle"
  throttle = 1
  throttleIntegral = 0
  targetPos = nil
  M.launchFailed = false
  yawSmoother = newExponentialSmoothing(30)
  if drivetrain.esc.isExisting then
    drivetrain.esc.pauseESCAction = false
  end

  controller.mainController.setGearboxMode("arcade")

  extensions.reload("straightLine")
  straightLine.onInit()
end

local function go()
  controller.mainController.setFreeze(0)
  state = "go"
end

local function prepare(target)
  state = "prepareThrottle"
  if drivetrain.esc.isExisting then
    drivetrain.esc.pauseESCAction = true
  end
  controller.mainController.setFreeze(1)

  straightLine.setTargetDirection(target, "road")
end

local function stop()
  input.event("parkingbrake", 0, 1)
  input.event("brake", 0, 1)
  input.event("throttle", 0, 1)
  controller.mainController.setFreeze(0)
  straightLine.stop()
  onInit()
end

local function updateGFX(dt)
  if state == "idle" then
  elseif state == "prepareThrottle" then
    --input.event('clutch', 0, 2)
    --input.event('parkingbrake', 1, 1)
    input.event("throttle", 1, 1)
    state = "prepareBrake"
  elseif state == "prepareBrake" then
    --input.event('parkingbrake', 1, 1)
    --input.event('brake', 1, 1)
    input.event("throttle", 1, 1)
    throttle = 1
  elseif state == "go" then
    input.event("throttle", throttle, 2)
    input.event("parkingbrake", 0, 1)
    input.event("brake", 0, 1)

    local yawRate = yawSmoother:get(-obj:getYawAngularVelocity())
    local spinning = math.max(math.abs(yawRate) - 0.07, 0)
    if yawRate > 0.5 then
      M.launchFailed = true
    end

    local velocity = obj:getVelocity()
    local directionVector = obj:getDirectionVector()
    local actualVelocity = max((directionVector:dot(velocity) / (directionVector:length() * directionVector:length()) * directionVector):length(), 0)

    local peakSlipError = 0
    for _, wheel in pairs(wheels.wheels) do
      if wheel.hasDiffAttached then
        local wheelSpeed = max(wheel.obj.getAngularVelocityBrakeCouple() * wheel.wheelDir * wheel.radius, 0)
        local expectedWheelSpeed = actualVelocity
        local slip = min(max((wheelSpeed - expectedWheelSpeed) / (wheelSpeed + 1e-30), 0), 1)
        peakSlipError = max(peakSlipError, slip)
      end
    end

    local slipError = (peakSlipError - 0.2) + spinning * 80
    if slipError < 0 then
      slipError = slipError * 2
    end
    throttleIntegral = max(min(throttleIntegral + (slipError) * dt, 0.5), 0)
    throttle = min(max(1 - (slipError * 0.5 + throttleIntegral * 3.5), 0.05), 1)
    if throttle > 0.95 then
      throttle = 1
    end

  --        if streams.willSend("genericGraphAdvanced") then
  --            gui.send('genericGraphAdvanced', {
  --                    throttle = { title = "Throttle", color = getContrastColorStringRGB(1), unit = "", value = throttle},
  --                    P = { title = "P", color = getContrastColorStringRGB(2), unit = "", value = slipError},
  --                    I = { title = "I", color = getContrastColorStringRGB(3), unit = "", value = throttleIntegral},
  --                    spinning = { title = "Spinning", color = getContrastColorStringRGB(4), unit = "", value = spinning},
  --                })
  --        end
  end

  straightLine.updateGFX(dt)
end

-- public interface
M.onInit = onInit
M.onReset = onInit
M.updateGFX = updateGFX
M.prepare = prepare
M.go = go
M.stop = stop

return M
