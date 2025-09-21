-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local min = math.min
local max = math.max

local steeringIntegral = 0
local steeringDerivative = 0
local targetPosOffset = nil
local baseTargetPos = nil
local steeringMode = nil

local function onInit()
  steeringIntegral = 0
  targetPosOffset = nil
  baseTargetPos = nil
end

local function setTargetDirection(target, mode)
  if not target then
    local direction = obj:getDirectionVector():z0()
    local pos = obj:getPosition()
    baseTargetPos = pos + direction * 20
  else
    baseTargetPos = target
  end
  targetPosOffset = vec3(0, 1, 0)
  print("Targetpos: " .. dumps(baseTargetPos))

  steeringMode = mode
  print("Using steering mode: " .. steeringMode)
end

local function stop()
  input.event("steering", 0, 0)
  onInit()
end

local function updateGFX(dt)
  if baseTargetPos then
    local myPos = obj:getPosition()
    local targetPos = baseTargetPos + vec3((myPos.x - 20) * targetPosOffset.x, (myPos.y - 20) * targetPosOffset.y, (myPos.z - 20) * targetPosOffset.z)
    local distanceVector = targetPos - myPos
    local directionVector = obj:getDirectionVector()
    local angleError = (distanceVector:dot(directionVector) / (directionVector:length() * distanceVector:length()))
    local vectorLeft = obj:getDirectionVectorUp():cross(directionVector)
    local angleSign = distanceVector:dot(vectorLeft)

    angleError = min(max((angleError - sign(angleError)), -1), 1) * sign(angleSign)
    angleError = sign(angleError) * math.sqrt(math.abs(angleError))
    steeringIntegral = max(min(steeringIntegral + angleError * dt, 0.1), -0.1)
    local steeringDerivative = angleError / dt

    local integralCoef = steeringMode == "offroad" and 0.5 or 0
    local proportionalCoef = steeringMode == "offroad" and 0.8 or 0.4
    local derivativeCoef = steeringMode == "offroad" and 0 or 0

    local steering = angleError * proportionalCoef + steeringIntegral * integralCoef + steeringDerivative * derivativeCoef
    input.event("steering", steering, 1)

  --        if streams.willSend("genericGraphAdvanced") then
  --            gui.send('genericGraphAdvanced', {
  --                    steering = { title = "Steering", color = getContrastColorStringRGB(1), unit = "ms", value = steering},
  --                    P = { title = "P", color = getContrastColorStringRGB(2), unit = "ms", value = angleError * proportionalCoef},
  --                    I = { title = "I", color = getContrastColorStringRGB(3), unit = "ms", value = steeringIntegral * integralCoef},
  --                })
  --        end
  end
end

-- public interface
M.onInit = onInit
M.onReset = onInit
M.updateGFX = updateGFX
M.setTargetDirection = setTargetDirection
M.stop = stop

return M
