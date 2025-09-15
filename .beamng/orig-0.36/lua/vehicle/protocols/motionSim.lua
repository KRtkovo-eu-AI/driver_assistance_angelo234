-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- ========================================================================================================================= --
-- For information on how to implement and distribute your custom UDP protocol, please check https://go.beamng.com/protocols --
-- ========================================================================================================================= --

-- generic protocol to guide simple motion platforms
local M = {}

local function init() end
local function reset() end
local function getAddress()        return settings.getValue("protocols_motionSim_address") end        -- return "127.0.0.1"
local function getPort()           return settings.getValue("protocols_motionSim_port") end           -- return 4567
local function getMaxUpdateRate()  return settings.getValue("protocols_motionSim_maxUpdateRate") end  -- return 60

local function isPhysicsStepUsed()
  --return false-- use graphics step. performance cost is ok. the update rate could reach UP TO min(getMaxUpdateRate(), graphicsFramerate)
  return true   -- use physics step. performance cost is big. the update rate could reach UP TO min(getMaxUpdateRate(), 2000 Hz)
end

local function getStructDefinition()
  return [[
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////// IMPORTANT: if you modify this definition, also update the docs at https://go.beamng.com/protocols /////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    char format[4]; // allows to verify if packet is the expected format, fixed value of "BNG1"

    float posX, posY, posZ; // world position of the vehicle
    float velX, velY, velZ; // velocity of the vehicle
    float accX, accY, accZ; // acceleration of the vehicle, gravity not included

    float upX,  upY,  upZ;  // vector components of a vector pointing "up" relative to the vehicle

    float rollPos, pitchPos, yawPos; // angle of roll, pitch and yaw of the vehicle
    float rollVel, pitchVel, yawVel; // angular velocities of roll, pitch and yaw of the vehicle
    float rollAcc, pitchAcc, yawAcc; // angular acceleration of roll, pitch and yaw of the vehicle
  ]]
end

local function fillStruct(o, dtSim)
  o.format = "BNG1"

  o.posX, o.posY, o.posZ = protocols.posX, protocols.posY, protocols.posZ
  o.velX, o.velY, o.velZ = protocols.velXSmoothed, protocols.velYSmoothed, protocols.velZSmoothed
  o.accX, o.accY, o.accZ = protocols.accXSmoothed, protocols.accYSmoothed, protocols.accZSmoothed

  o.upX,  o.upY,  o.upZ  = protocols.upX,  protocols.upY,  protocols.upZ

  o.rollPos, o.pitchPos, o.yawPos = protocols.rollPosSmoothed, protocols.pitchPosSmoothed, protocols.yawPosSmoothed
  o.rollVel, o.pitchVel, o.yawVel = protocols.rollVelSmoothed, protocols.pitchVelSmoothed, protocols.yawVelSmoothed
  o.rollAcc, o.pitchAcc, o.yawAcc = protocols.rollAccSmoothed, protocols.pitchAccSmoothed, protocols.yawAccSmoothed
end

M.init = init
M.reset = reset
M.getAddress = getAddress
M.getPort = getPort
M.getMaxUpdateRate = getMaxUpdateRate
M.getStructDefinition = getStructDefinition
M.fillStruct = fillStruct
M.isPhysicsStepUsed = isPhysicsStepUsed

return M
