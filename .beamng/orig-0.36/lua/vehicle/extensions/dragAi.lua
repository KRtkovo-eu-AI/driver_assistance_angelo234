-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local targetPosition = nil
local state = nil

local function getReady()
  state = "ready"
end

local function go()
  state = "go"
  input.event("parkingbrake", 0, 1)
end

local function setTarget(targetPos)
  targetPosition = targetPos
end

local function updateGFX(dt)
  if state == "ready" then
    input.event("throttle", 1, 1)
  elseif state == "go" then
    input.event("throttle", 1, 1)

    local targetPos = vec3(targetPosition)
    local myPos = obj:getPosition()
    local distanceVector = targetPos - myPos
    distanceVector = vec3(distanceVector.x, distanceVector.y, 0)
    local directionVector = obj:getDirectionVector()
    directionVector = vec3(directionVector.x, directionVector.y, 0)
    local angle = (distanceVector:dot(directionVector) / (directionVector:length() * distanceVector:length()))
    local vectorLeft = obj:getDirectionVectorUp():cross(directionVector)
    local angleSign = distanceVector:dot(vectorLeft)

    angle = (angle - sign(angle) * 1) * 30 * sign(angleSign)
    input.event("steering", angle, 1)

    if distanceVector:length() <= 10 then
      state = "stopping"
    end
  elseif state == "stopping" then
    input.event("brake", 1, 1)
    input.event("parkingbrake", 1, 1)
    input.event("throttle", 0, 1)
    input.event("steering", 0, 1)

    if electrics.values.airspeed < 2 then
      state = "finished"
      input.event("brake", 0, 1)
      input.event("throttle", 0, 1)
    end
  elseif state == "finished" then
    input.event("parkingbrake", 1, 1)
  end
end

local function reset()
  state = "idle"
  input.event("throttle", 0, 1)
  input.event("parkingbrake", 1, 1)
end

M.getReady = getReady
M.go = go
M.reset = reset
M.setTarget = setTarget
M.updateGFX = updateGFX

return M
