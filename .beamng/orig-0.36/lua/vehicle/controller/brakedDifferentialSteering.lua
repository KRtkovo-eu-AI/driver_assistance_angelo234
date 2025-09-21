-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local min = math.min

local leftSteeringWheel
local rightSteeringWheel

local leftBrake = 0
local rightBrake = 0
local steeringPlay = 0
local leftMaxBrakeSteeringBrakeTorque
local rightMaxBrakeSteeringBrakeTorque

local function updateWheelsIntermediate(dt)
  leftSteeringWheel.desiredBrakingTorque = min(leftSteeringWheel.desiredBrakingTorque + leftBrake * leftMaxBrakeSteeringBrakeTorque, leftSteeringWheel.brakeTorque)
  rightSteeringWheel.desiredBrakingTorque = min(rightSteeringWheel.desiredBrakingTorque + rightBrake * rightMaxBrakeSteeringBrakeTorque, rightSteeringWheel.brakeTorque)
end

local function updateGFX(dt)
  local steering = electrics.values.steering_input or 0
  local brakedSteeringSpeedCoef = linearScale(electrics.values.wheelspeed or 1, 0, 0.2, 0, 1)

  leftBrake = linearScale(-clamp(steering, -1, 0), steeringPlay, 1, 0, 1) * brakedSteeringSpeedCoef
  rightBrake = linearScale(clamp(steering, 0, 1), steeringPlay, 1, 0, 1) * brakedSteeringSpeedCoef
end

local function reset(jbeamData)
  leftBrake = 0
  rightBrake = 0
end

local function init(jbeamData)
  steeringPlay = jbeamData.steeringPlay or 0
  local brakeRName = jbeamData.rightBrakeName or "brake_R"
  local brakeLName = jbeamData.leftBrakeName or "brake_L"
  for _, wd in pairs(wheels.wheelRotators) do
    if wd.name == brakeLName then
      leftSteeringWheel = wd
    elseif wd.name == brakeRName then
      rightSteeringWheel = wd
    end
  end
  local maxSteeringBrakeTorque = jbeamData.maxSteeringBrakeTorque
  leftMaxBrakeSteeringBrakeTorque = maxSteeringBrakeTorque and min(maxSteeringBrakeTorque, leftSteeringWheel.brakeTorque) or leftSteeringWheel.brakeTorque
  rightMaxBrakeSteeringBrakeTorque = maxSteeringBrakeTorque and min(maxSteeringBrakeTorque, rightSteeringWheel.brakeTorque) or rightSteeringWheel.brakeTorque
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX
M.updateWheelsIntermediate = updateWheelsIntermediate

return M
