-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local acos = math.acos
local deg = math.deg
local abs = math.abs
local atan2 = math.atan2

local data = {}
local frontLeftWheel
local frontRightWheel
local trailerId
local hasRemoteData = false
local inputStates = {
  idle = "idle",
  accelerating = "accelerating",
  firstSteer = "firstSteer",
  secondSteer = "secondSteer",
  holdSteer = "holdSteer"
}
local inputState = inputStates.idle
local targetSpeed = 0
local steeringTarget1 = 0
local steeringTarget2 = 0
local steeringTarget3 = 0

local steeringSmoother = newTemporalSigmoidSmoothing(7, 1, 1, 7)

--  tech_trailSim.startManeuver(100, 0.05,0.02,0.01)
local function startManeuver(speedKmh, steering1, steering2, steering3, steeringSmootherRate, steeringSmootherAcc)
  inputState = inputStates.accelerating
  speedKmh = speedKmh or 100

  targetSpeed = speedKmh / 3.6
  steeringTarget1 = steering1 or 0.05
  steeringTarget2 = steering2 or 0.02
  steeringTarget3 = steering3 or 0.01
  steeringSmoother = newTemporalSigmoidSmoothing(steeringSmootherRate or 7, steeringSmootherAcc or 1, steeringSmootherAcc or 1, steeringSmootherRate or 7)
  cruiseControl.setSpeed(targetSpeed)
  input.event("steering", 0)
end

local function trailerData(dataString, id)
  local trailerData = deserialize(dataString)
  for i, d in ipairs(trailerData) do
    data[4 + i] = d
  end
  trailerId = id
  hasRemoteData = true
end

local function updateDisplay(dt)
  local objects = mapmgr.getObjects()
  local thisVehicle = objects[objectId]
  local trailerVehicle = objects[trailerId or 0]
  local trailerAngle = 0
  if thisVehicle and trailerVehicle then
    local vehicleDirection = thisVehicle.dirVec:z0():normalized()
    local trailerDirection = trailerVehicle.dirVec:z0():normalized()
    --trailerAngle = deg(acos(vehicleDirection:dot(trailerDirection)))
    trailerAngle = deg(atan2(vehicleDirection:cross(trailerDirection):dot(vec3(0, 0, 1)), vehicleDirection:dot(trailerDirection)))
  end

  local steeringSign = sign(electrics.values.steering or 0)
  local frontLeftWheelAngle = acos(obj:nodeVecPlanarCosRightForward(frontLeftWheel.node2, frontLeftWheel.node1)) * steeringSign
  local frontRightWheelAngle = acos(obj:nodeVecPlanarCosRightForward(frontRightWheel.node1, frontRightWheel.node2)) * steeringSign

  local frontWheelAngle = (frontLeftWheelAngle + frontRightWheelAngle) * 0.5

  local _, _, yawAV = obj:getRollPitchYawAngularVelocity()
  data[1][2] = electrics.values.wheelspeed or 0 --vehicle speed
  data[2][2] = -deg(frontWheelAngle) --steering wheel angle
  data[3][2] = yawAV or 0 --vehicle yaw velocity
  data[4][2] = trailerAngle or 0 -- angle between trailer and vehicle
  if hasRemoteData then
    guihooks.graph(unpack(data))
  end
end

local function updateInput(dt)
  local steeringInput
  if inputState == inputStates.idle then
  elseif inputState == inputStates.accelerating then
    if cruiseControl.hasReachedTargetSpeed then
      inputState = inputStates.firstSteer
    --cruiseControl.setEnabled(false)
    end
  elseif inputState == inputStates.firstSteer then
    local steeringTarget = steeringTarget1
    steeringInput = steeringSmoother:get(steeringTarget, dt)
    if abs(steeringInput - steeringTarget) < 0.001 then
      inputState = inputStates.secondSteer
    end
  elseif inputState == inputStates.secondSteer then
    local steeringTarget = -steeringTarget2
    steeringInput = steeringSmoother:get(steeringTarget, dt)
    if abs(steeringInput - steeringTarget) < 0.001 then
      inputState = inputStates.holdSteer
    end
  elseif inputState == inputStates.holdSteer then
    local steeringTarget = steeringTarget3
    steeringInput = steeringSmoother:get(steeringTarget, dt)
  end
  if steeringInput then
    input.event("steering", steeringInput)
  end
end

local function updateGFXTow(dt)
  updateInput(dt)
  updateDisplay(dt)
end

local function updateGFXTrailer(dt)
  local _, _, yawAV = obj:getRollPitchYawAngularVelocity()
  data[1][2] = -sensors.gx2 or 0 --trailer acceleration
  data[2][2] = yawAV or 0 -- trailer yaw velocity
  BeamEngine:queueAllObjectLua(string.format("if tech_trailSim then tech_trailSim.trailerData(%q,%d) end", serialize(data), objectId))
end

local function onReset()
  --table.clear(data)
  if cruiseControl then
    cruiseControl.setEnabled(false)
  end
  input.event("steering", 0)
end

local function onExtensionLoaded()
  local engines = powertrain.getDevicesByCategory("engine")
  if #engines > 0 then
    M.updateGFX = updateGFXTow
    data = {
      {"Vehicle Speed", 0, 40, "m/s"},
      {"Steering Angle", 0, 2, "°", true},
      {"Vehicle Yaw Speed", 0, 1, "rad/s", true},
      {"Vehicle/Trailer Angle", 0, 20, "°", true}
    }
    extensions.load("cruiseControl")
    for _, wheel in pairs(wheels.wheels) do
      if wheel.name == "FR" then
        frontRightWheel = wheel
      elseif wheel.name == "FL" then
        frontLeftWheel = wheel
      end
    end
  else
    M.updateGFX = updateGFXTrailer
    data = {
      {"Trailer Acceleration", 0, 10, "m/s²", true},
      {"Trailer Yaw Speed", 0, 1, "rad/s", true}
    }
  end
end

M.onExtensionLoaded = onExtensionLoaded
M.onReset = onReset
M.updateGFX = nop
M.trailerData = trailerData
M.startManeuver = startManeuver

return M
