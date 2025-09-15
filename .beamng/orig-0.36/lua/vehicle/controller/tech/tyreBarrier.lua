-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Control parameters.
local testDuration = 10.0                                                   -- The length of time to measure data after the test starts, in seconds.
local initialVel = 45                                                       -- The requested initial velocity, in kph.
local initalVelOffset = 0.5                                              -- An offset added on to boost the hitting of the intial velocity target.
local targetStartPos = vec3(100, 0)                                         -- The target test start position.
local distToStartTolSq = 1.0                                                -- A tolerance (m^2) used when testing how close the vehicle is to the start position.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

local abs, acos = math.abs, math.acos

-- Module state.
local isPreamble = true                                                     -- A flag which indicates if we are currently in the preamble phase of the test.
local hasDataBeenWritten = false                                            -- A flag which indicates if the test data has been written to file already.
local time = 0.0                                                            -- The test timer, in seconds.

-- Get the two RWA angles from the vehicle (FL and FR).
local function getWheelAngles()
  local steeringSign = sign(electrics.values.steering_input)
  local FLWh, FRWh = wheels.wheelRotators[wheels.wheelRotatorIDs.FL], wheels.wheelRotators[wheels.wheelRotatorIDs.FR]
  local frontLeftWheelAngle = acos(obj:nodeVecPlanarCosRightForward(FLWh.node2, FLWh.node1)) * steeringSign
  local frontRightWheelAngle = acos(obj:nodeVecPlanarCosRightForward(FRWh.node1, FRWh.node2)) * steeringSign
  return frontLeftWheelAngle, frontRightWheelAngle
end

-- Initialisation callback (called once when this controller is instantiated).
local function init(data)
  isPreamble = true
  if data.initVel ~= nil then                                               -- Set the target initial velocity, if it has been provided by the user.
    initialVel = data.initVel
  end
  if data.posX ~= nil then                                                  -- Set the target start position, if it has been provided by the user.
    targetStartPos = vec3(data.posX, data.posY, data.posZ)
  end
end

-- Reset callback (called wheh this controller is reset).
local function reset()
end

-- Update called. This is called on every physics step update, once the controller has been loaded.
local function update(dt)

  -- Ensure the steering is fixed straight ahead and the brakes are fully off, throughout the test.
  input.event("steering", 0.0)
  input.event("brake", 0.0)
  input.event("parkingBrake", 0.0)

  -- [PHASE 1]: In the first phase, we get the vehicle up to the feed initial conditions (velocity, STWA, and direction).
  if isPreamble then
    thrusters.applyVelocity(obj:getDirectionVector():normalized() * (initialVel + initalVelOffset) / 3.6, 0.5)
    local pos, vel = obj:getPosition(), obj:getVelocity()
    dump("[Preamble phase]: current vehicle velocity: " .. vel:length() * 3.6)
    if (pos - targetStartPos):squaredLength() < distToStartTolSq then
      isPreamble, time = false, 0.0
      dump("Start position reached [preamble phase complete - moving on to phase 2]...")
    end
    return
  end

  -- [PHASE 3]: If the test time has expired, leave immediately.
  if time > testDuration then
    dump("Test complete!")
    if hasDataBeenWritten == false then
      extensions.tech_tyreBarrier.triggerWriteDataToFile()
      hasDataBeenWritten = true
      extensions.tech_tyreBarrier.markTestComplete()
    end
    return
  end

  -- [PHASE 2]: In the second phase, the test has begun and we record data.
  local dat = {}

  -- The time stamp.
  dat.t = time

  -- The vehicle position, velocity, and pose.
  local pos, vel = obj:getPosition(), obj:getVelocity()
  dat.posX, dat.posY, dat.posZ, dat.velX, dat.velY, dat.velZ = pos.x, pos.y, pos.z, vel.x, vel.y, vel.z
  local roll, pitch, yaw = obj:getRollPitchYaw()
  dat.roll, dat.pitch, dat.yaw = roll, pitch, yaw

  -- The IMU readings.
  local IMUData = extensions.tech_tyreBarrier.getLatestIMUData()
  dat.IMUTime = IMUData.time
  dat.IMUMass = IMUData.mass
  dat.IMUAccelRaw0, dat.IMUAccelRaw1, dat.IMUAccelRaw2 = IMUData.accRaw[1], IMUData.accRaw[2], IMUData.accRaw[3]
  dat.IMUAccelSmooth0, dat.IMUAccelSmooth1, dat.IMUAccelSmooth2 = IMUData.accSmooth[1], IMUData.accSmooth[2],IMUData.accSmooth[3]
  dat.IMUAngVelX, dat.IMUAngVelY, dat.IMUAngVelZ = IMUData.angVel[1], IMUData.angVel[2], IMUData.angVel[3]
  dat.IMUAngVelSmoothX, dat.IMUAngVelSmoothY, dat.IMUAngVelSmoothZ = IMUData.angVelSmooth[1], IMUData.angVelSmooth[2], IMUData.angVelSmooth[3]
  dat.IMUAngAccelX, dat.IMUAngAccelY, dat.IMUAngAccelZ = IMUData.angAccel[1], IMUData.angAccel[2], IMUData.angAccel[3]
  dat.IMUPosX, dat.IMUPosY, dat.IMUPosZ = IMUData.pos[1], IMUData.pos[2], IMUData.pos[3]
  dat.IMUAxis1X, dat.IMUAxis1Y, dat.IMUAxis1Z = IMUData.dirX[1], IMUData.dirX[2], IMUData.dirX[3]
  dat.IMUAxis2X, dat.IMUAxis2Y, dat.IMUAxis2Z = IMUData.dirY[1], IMUData.dirY[2], IMUData.dirY[3]
  dat.IMUAxis3X, dat.IMUAxis3Y, dat.IMUAxis3Z = IMUData.dirZ[1], IMUData.dirZ[2], IMUData.dirZ[3]

  -- The vehicle wheel readings.
  local wh1, wh2, wh3, wh4 = wheels.wheelRotators[0], wheels.wheelRotators[1], wheels.wheelRotators[2], wheels.wheelRotators[3]
  dat.Wh1Speed, dat.Wh2Speed, dat.Wh3Speed, dat.Wh4Speed = 3.6 * wh1.wheelSpeed, 3.6 * wh2.wheelSpeed, 3.6 * wh3.wheelSpeed, 3.6 * wh4.wheelSpeed
  dat.Wh1BrkTorque = -(abs(wh1.coreData.brakeTorqueApplied) - wh1.frictionTorque)
  dat.Wh2BrkTorque = -(abs(wh2.coreData.brakeTorqueApplied) - wh2.frictionTorque)
  dat.Wh3BrkTorque = -(abs(wh3.coreData.brakeTorqueApplied) - wh3.frictionTorque)
  dat.Wh4BrkTorque = -(abs(wh4.coreData.brakeTorqueApplied) - wh4.frictionTorque)
  dat.Wh1DrvTorque = wh1.propulsionTorque * wh1.wheelDir
  dat.Wh2DrvTorque = wh2.propulsionTorque * wh2.wheelDir
  dat.Wh3DrvTorque = wh3.propulsionTorque * wh3.wheelDir
  dat.Wh4DrvTorque = wh4.propulsionTorque * wh4.wheelDir
  dat.Wh1AngVel = wh1.angularVelocity * wh1.wheelDir
  dat.Wh2AngVel = wh2.angularVelocity * wh2.wheelDir
  dat.Wh3AngVel = wh3.angularVelocity * wh3.wheelDir
  dat.Wh4AngVel = wh4.angularVelocity * wh4.wheelDir
  dat.RWA_FL, dat.RWA_FR = getWheelAngles()

  -- Send the latest data readings to the extension, to be collated.
  extensions.tech_tyreBarrier.appendLatestReading(dat)
  time = time + dt
end


-- Public interface.
M.init = init
M.reset = reset
M.update = update

return M