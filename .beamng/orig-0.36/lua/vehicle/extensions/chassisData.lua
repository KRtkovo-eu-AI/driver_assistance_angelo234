-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local abs = math.abs
local max = math.max
local chassisData

local function getChassisData()
  local frontLeftWheel
  local frontRightWheel
  local rearLeftWheel

  local avgWheelPos = vec3(0, 0, 0)
  local wheelCount = 0
  for _, wheel in pairs(wheels.wheels) do
    local wheelNodePos = vec3(v.data.nodes[wheel.node1].pos) --find the wheel position

    avgWheelPos = avgWheelPos + wheelNodePos --sum up all positions
    wheelCount = wheelCount + 1
  end

  avgWheelPos = avgWheelPos / wheelCount --make the average of all positions

  local vectorForward = vec3(v.data.nodes[v.data.refNodes[0].ref].pos) - vec3(v.data.nodes[v.data.refNodes[0].back].pos) -- obj:getDirectionVector() --vector facing forward
  local vectorUp = vec3(v.data.nodes[v.data.refNodes[0].up].pos) - vec3(v.data.nodes[v.data.refNodes[0].ref].pos)

  local vectorRight = vectorForward:cross(vectorUp) --vector facing to the right

  for _, wheel in pairs(wheels.wheels) do
    local wheelNodePos = vec3(v.data.nodes[wheel.node1].pos) --find the wheel position
    local wheelVector = wheelNodePos - avgWheelPos --create a vector from our "center" to the wheel
    local dotForward = vectorForward:dot(wheelVector) --calculate dot product of said vector and forward vector
    local dotLeft = vectorRight:dot(wheelVector) --calculate dot product of said vector and left vector

    if dotForward >= 0 then
      if dotLeft >= 0 then
        frontRightWheel = wheel
      else
        frontLeftWheel = wheel
      end
    else
      if dotLeft < 0 then
        rearLeftWheel = wheel
      end
    end
  end

  if not (frontLeftWheel and frontRightWheel and rearLeftWheel) then
    return nil
  end

  local FR1 = 0
  local FR2 = 0
  local FL1 = 0
  local FL2 = 0
  local RL1 = 0
  local RL2 = 0
  for _, n in pairs(v.data.nodes) do
    if n.cid == frontRightWheel.node1 then
      FR1 = n.pos
    elseif n.cid == frontRightWheel.node2 then
      FR2 = n.pos
    elseif n.cid == frontLeftWheel.node1 then
      FL1 = n.pos
    elseif n.cid == frontLeftWheel.node2 then
      FL2 = n.pos
    elseif n.cid == rearLeftWheel.node1 then
      RL1 = n.pos
    elseif n.cid == rearLeftWheel.node2 then
      RL2 = n.pos
    end
  end
  local wbFrontY
  local wbRearY
  if abs(FL1.x) > abs(FL2.x) then
    wbFrontY = FL1.y
  else
    wbFrontY = FL2.y
  end
  if abs(RL1.x) > abs(RL2.x) then
    wbRearY = RL1.y
  else
    wbRearY = RL2.y
  end

  local wheelBase = abs(wbFrontY - wbRearY) --calculate wheelbase from the distance of the front and rear wheels

  --get the outer node from each wheel so we don't accidentially end up using the inner nodes or some combination of inner and outer
  local twLeftX = max(abs(FL1.x), abs(FL2.x)) * sign(FL1.x)
  local twRightX = max(abs(FR1.x), abs(FR2.x)) * sign(FR1.x)

  --substract wheel size from trackwidth calc to get the center distance
  local trackWidth = abs(twLeftX - twRightX) - abs(FL1.x - FL2.x) * 0.5

  return {trackWidth = trackWidth, wheelBase = wheelBase, objectID = objectId, vehicleName = v.data.information.name}
end

local function sendChassisData(objID, callback, forceRefresh)
  if not chassisData or forceRefresh then
    chassisData = getChassisData()
  end

  if chassisData then
    obj:queueObjectLuaCommand(objID, string.format("%s(%q)", callback, jsonEncode(chassisData)))
  end
end

M.requestChassisData = sendChassisData

return M
