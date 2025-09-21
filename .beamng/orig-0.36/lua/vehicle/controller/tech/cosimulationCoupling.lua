-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}


local logTag = 'cosimulationCoupling'

-- External modules used.
local dat = require('tech/cosimulationNames')
local lpack = require("lpack")
local csvlib = require('csvlib')
local csvSendData
local csvReceiveData
local logReceive
local enableVSL
local enableCosim
local initialized = false
local cycleTime = 0.0                   -- The cycle time.  This is the most-recently measured time between two send operations, in seconds.
local lastTimeOfSend = os.clockhp()     -- The time at which the last message was sent from Lua to Simulink.

-- local csvPhysicsSteps
-- Module constants.
local min, max, floor, ceil = math.min, math.max, math.floor, math.ceil
local abs, sqrt, acos = math.abs, math.sqrt, math.acos
local names, groups = dat.names, dat.groups                                                         -- The common string names for each group property/signal group.

-- Module state.
local udpSendIP, udpReceiveIP = "127.0.0.1", "127.0.0.1"                                            -- The IP address for each end of the communication.
local udpSendPort, udpReceivePort = 64890, 64891                                                    -- The port numbers at each end of the communication.
local time3rdParty = 0.0005                                                                         -- The 3rd party computation time.
local pingTime = 1e-5                                                                               -- The ping round-trip time.
local udpSendSocket, udpRecvSocket = nil, nil                                                       -- The udp sockets for sending/receiving data.
local sendCtr = 0                                                                                   -- Counts how many messages have been sent.
local maxRecvId = 0                                                                                 -- The maximum Id which has been received so far.
local stepsSinceLastSend = 0                                                                        -- Counts the number of update cycles since the last message was sent.
local sendSkips = 0                                                                                 -- Number of updates to skip between sends, in the chosen window size.
local unanswered = 0                                                                                -- Number of unanswered received messages we allow in chosen window size.
local id = 0                                                                                        -- The unique id number for messages sent over the sockets.
local blockingTimeoutLength = 2.0                                                                   -- The timeout size (in seconds) when blocking on message receives.
local msgOut = {}                                                                                   -- The table used to store the outgoing message to be sent to 3rd party.
local msgIn = { 1 }                                                                                 -- The table used to store the decoded message from 3rd party.
local defaultIndex = -1                                                                             -- A default index used in the incoming message, which contains zero.
local inDMap, inWMap, outMap, frozenValues, inBools = {}, {}, {}, {}, {}                            -- The mappings structures, computed at init.
local initLength, initWidth, initHeight = 0, 0, 0                                                   -- The initial vehicle dimensions.
local wNodePos = {}                                                                                 -- An array used to store the wheel first node positions (see Kinematics group).
local master = { { {} }, { {} }, { {} }, {}, { {} } }                                               -- Arrays of ordered group values (per group), for the outgoing message.
local jumpTable = {}                                                                                -- A jump table structure, used for the outgoing message.
local num2BoolTable = { [0] = false, [1] = true }                                                   -- A jump table structure, used to convert numbers to bool, fast.
local wheelRotators, wheelIds = wheels.wheelRotators, wheels.wheelRotatorIDs                        -- Cache the wheel rotator data.
local wheelsOrder = {}                                                                              -- An array used to provide a fixed order for the vehicle wheels.
local inTorques, pTorqueKeys, bTorqueKeys, fTorqueKeys = { {}, {}, {} }, {}, {}, {}                 -- Arrays for the incoming torque values for each wheel.
local vehSensors = {}                                                                               -- An ordered array of attached sensors, referenced by the signals.


local function linspace(length)
  local out = {}
  for i=1,length,1 do
    out[i] = tostring(i)
  end
  return out
end



local function createCSV(sendNames, receiveNames)
  log('I', logTag, 'Initializing CSV logging with correct offset...')

  -- Insert "signal_number" after "time" and before the actual signal names
  local sendHeaders = { "time", "message_index", unpack(sendNames) }
  local receiveHeaders = { "time", "message_index", unpack(receiveNames) }

  -- Create CSV files with updated headers
  csvSendData = csvlib.newCSV(unpack(sendHeaders))
  csvReceiveData = csvlib.newCSV(unpack(receiveHeaders))

  if csvSendData then
    log('I', logTag, 'csvSendData successfully created with offset column.')
  else
    log('E', logTag, 'Failed to create csvSendData.')
  end

  if csvReceiveData then
    log('I', logTag, 'csvReceiveData successfully created with offset column.')
  else
    log('E', logTag, 'Failed to create csvReceiveData.')
  end
end




local function saveCSV()
  -- Get current date and time in the desired format
  local timestamp = os.date("%Y-%m-%d_%H-%M-%S")

  -- Create filenames with date and time
  local sendFileName = string.format("CoSimulationInternalLog_%s.csv", timestamp)
  csvSendData:write(sendFileName)


  if enableCosim then
    local receiveFileName = string.format("CoSimulationExternalLog_%s.csv", timestamp)
    csvReceiveData:write(receiveFileName)
  end

  log('I', logTag, 'logs saved in csv')
end


-- Gathers the Kinematics properties for the outgoing message.
local function gatherKinematicsProperties()
  local kOut = master[1][1]
  local pos, vel = obj:getPosition(), obj:getVelocity()
  kOut[1], kOut[2], kOut[3] = pos.x, pos.y, pos.z                                                   -- Position, meters.
  kOut[4], kOut[5], kOut[6] = vel.x, vel.y, vel.z                                                   -- Velocity, m/s.
  local ffiS = sensors.ffiSensors
  kOut[7], kOut[8], kOut[9] = ffiS.sensorX, ffiS.sensorY, ffiS.sensorZnonInertial                   -- Acceleration, ms^-2.
  kOut[10], kOut[11], kOut[12] = obj:getRollPitchYaw()                                              -- Roll/pitch/yaw, rad.
  kOut[13] = obj:getRollAngularVelocity()                                                           -- Roll rate, rad/s.
  kOut[14] = obj:getPitchAngularVelocity()                                                          -- Pitch rate, rad/s.
  kOut[15] = obj:getYawAngularVelocity()                                                            -- Yaw rate, rad/s.
  kOut[16], kOut[17] = obj:getGroundSpeed(), obj:getAltitude()                                      -- Ground speed, m/s. Altitude, meters.
  local fwd = obj:getDirectionVector()
  fwd:normalize()
  kOut[18], kOut[19], kOut[20] = fwd.x, fwd.y, fwd.z                                                -- Local frame unit forward vector, meters.
  local up = obj:getDirectionVectorUp()
  up:normalize()
  kOut[21], kOut[22], kOut[23] = up.x, up.y, up.z                                                   -- Local frame unit up vector, meters.
  local right = fwd:cross(up)
  kOut[24], kOut[25], kOut[26] = right.x, right.y, right.z                                          -- Local frame unit lateral vector, meters.
  kOut[27], kOut[28], kOut[29] = initLength, initWidth, initHeight                                  -- Initial length/width/height, meters.
  local cogWI = obj:calcCenterOfGravity(false)
  kOut[30], kOut[31], kOut[32] = cogWI.x, cogWI.y, cogWI.z                                          -- Center-of-Gravity (with wheels included).
  local cogWNI = obj:calcCenterOfGravity(true)
  kOut[33], kOut[34], kOut[35] = cogWNI.x, cogWNI.y, cogWNI.z                                       -- Center-of-Gravity (without wheels included).
  local mfb = obj:getFrontPosition()
  kOut[36], kOut[37], kOut[38] = mfb.x, mfb.y, mfb.z                                                -- Vehicle mid front bumper position.
  local mrb = mfb - fwd * initLength
  kOut[39], kOut[40], kOut[41] = mrb.x, mrb.y, mrb.z                                                -- Vehicle mid rear bumper position.
  local ctr = 1
  for _, wheel in pairs(wheels.wheels) do
    wNodePos[ctr] = obj:getNodePosition(wheel.node1)
    ctr = ctr + 1
  end
  local fam = pos + (wNodePos[min(ctr, 3)] + wNodePos[min(ctr, 4)]) * 0.5
  kOut[42], kOut[43], kOut[44] = fam.x, fam.y, fam.z                                                -- Vehicle front axle midpoint (if wheels are symmetric).
  local ram = pos + (wNodePos[min(ctr, 1)] + wNodePos[min(ctr, 2)]) * 0.5
  kOut[45], kOut[46], kOut[47] = ram.x, ram.y, ram.z                                                -- Vehicle rear axle midpoint (if wheels are symmetric).
end

-- Gathers the Wheels properties for the outgoing message.
local function gatherWheelsProperties()
  local wOut = master[2][1]
  local stInSign, wCtr = sign(electrics.values.steering_input), 1
  for i = 1, wheels.wheelRotatorCount do
    local wheel = wheelRotators[wheelIds[wheelsOrder[i]]]
    local wheelDir, frictionTorque = wheel.wheelDir, wheel.frictionTorque
    wOut[wCtr] = wheel.wheelSpeed                                                                   -- Wheel speed, ms^-1.
    wOut[wCtr + 1] = wheel.angularVelocityBrakeCouple * wheelDir                                    -- Angular velocity, rad/s.
    wOut[wCtr + 2] = wheel.downForce                                                                -- Downforce, Nm.
    wOut[wCtr + 3] = abs(wheel.coreData.brakeTorqueApplied) - frictionTorque                        -- Braking torque, Nm.
    wOut[wCtr + 4] = wheel.propulsionTorque * wheelDir                                              -- Propulsion torque, Nm.
    wOut[wCtr + 5] = frictionTorque                                                                 -- Friction torque, Nm.
    wOut[wCtr + 6] = acos(obj:nodeVecPlanarCosRightForward(wheel.node1, wheel.node2)) * stInSign    -- Road wheel angle, rad.
    wCtr = wCtr + 7
  end
end

-- Gathers the Electrics properties for the outgoing message.
local function gatherElectricsProperties() master[3][1] = electrics.values end

-- Gathers the Powertrain properties for the outgoing message.
local function gatherPowertrainProperties() master[4] = powertrain.getDevices() end

-- Gathers the Sensors properties for the outgoing message.
local function gatherSensorsProperties()
  local sOut, sCtr = master[5][1], 1
  for _, v in ipairs(vehSensors.IMUs) do                                                            -- Append all the IMU readings.
    local d = v.ctrl.getLatest(v.id)
    local pos = d.pos
    sOut[sCtr], sOut[sCtr + 1], sOut[sCtr + 2] = pos[1], pos[2], pos[3]
    local dir1 = d.dirX
    sOut[sCtr + 3], sOut[sCtr + 4], sOut[sCtr + 5] = dir1[1], dir1[2], dir1[3]
    local dir2 = d.dirY
    sOut[sCtr + 6], sOut[sCtr + 7], sOut[sCtr + 8] = dir2[1], dir2[2], dir2[3]
    local dir3 = d.dirZ
    sOut[sCtr + 9], sOut[sCtr + 10], sOut[sCtr + 11] = dir3[1], dir3[2], dir3[3]
    sOut[sCtr + 12] = d.mass
    local angVelRaw = d.angVel
    sOut[sCtr + 13], sOut[sCtr + 14], sOut[sCtr + 15] = angVelRaw[1], angVelRaw[2], angVelRaw[3]
    local angVelSm = d.angVelSmooth
    sOut[sCtr + 16], sOut[sCtr + 17], sOut[sCtr + 18] = angVelSm[1], angVelSm[2], angVelSm[3]
    local accelRaw = d.accRaw
    sOut[sCtr + 19], sOut[sCtr + 20], sOut[sCtr + 21] = accelRaw[1], accelRaw[2], accelRaw[3]
    local accelSm = d.accSmooth
    sOut[sCtr + 22], sOut[sCtr + 23], sOut[sCtr + 24] = accelSm[1], accelSm[2], accelSm[3]
    local angAccel = d.angAccel
    sOut[sCtr + 25], sOut[sCtr + 26], sOut[sCtr + 27] = angAccel[1], angAccel[2], angAccel[3]
    sOut[sCtr + 28] = d.time
    sCtr = sCtr + 29
  end
  for _, v in ipairs(vehSensors.GPSs) do                                                            -- Append all the GPS readings.
    local d = v.ctrl.getLatest(v.id)
    sOut[sCtr], sOut[sCtr + 1] = d.x, d.y
    sOut[sCtr + 2], sOut[sCtr + 3] = d.lon, d.lat
    sOut[sCtr + 4] = d.time
    sCtr = sCtr + 5
  end
  for _, v in ipairs(vehSensors.idealRADARs) do                                                     -- Append the Ideal RADAR readings.
    local d = v.ctrl.getLatest(v.id)
    local veh = d.closestVehicles1
    sOut[sCtr] = sqrt(veh.distToPlayerVehicleSq)
    sOut[sCtr + 1], sOut[sCtr + 2] = d.length, d.width
    local vel = veh.vel
    sOut[sCtr + 3], sOut[sCtr + 4], sOut[sCtr + 5] = vel.x, vel.y, vel.z
    local accel = veh.acc
    sOut[sCtr + 6], sOut[sCtr + 7], sOut[sCtr + 8] = accel.x, accel.y, accel.z
    sOut[sCtr + 9], sOut[sCtr + 10] = veh.relDistX, veh.relDistY
    sOut[sCtr + 11], sOut[sCtr + 12] = veh.relVelX, veh.relVelY
    sOut[sCtr + 13], sOut[sCtr + 14] = veh.relAccX, veh.relAccY
    veh = d.closestVehicles2
    sOut[sCtr + 15] = sqrt(veh.distToPlayerVehicleSq)
    sOut[sCtr + 16], sOut[sCtr + 17] = d.length, d.width
    local vel = veh.vel
    sOut[sCtr + 18], sOut[sCtr + 19], sOut[sCtr + 20] = vel.x, vel.y, vel.z
    local accel = veh.acc
    sOut[sCtr + 21], sOut[sCtr + 22], sOut[sCtr + 23] = accel.x, accel.y, accel.z
    sOut[sCtr + 24], sOut[sCtr + 25] = veh.relDistX, veh.relDistY
    sOut[sCtr + 26], sOut[sCtr + 27] = veh.relVelX, veh.relVelY
    sOut[sCtr + 28], sOut[sCtr + 29] = veh.relAccX, veh.relAccY
    veh = d.closestVehicles3
    sOut[sCtr + 30] = sqrt(veh.distToPlayerVehicleSq)
    sOut[sCtr + 31], sOut[sCtr + 32] = d.length, d.width
    local vel = veh.vel
    sOut[sCtr + 33], sOut[sCtr + 34], sOut[sCtr + 35] = vel.x, vel.y, vel.z
    local accel = veh.acc
    sOut[sCtr + 36], sOut[sCtr + 37], sOut[sCtr + 38] = accel.x, accel.y, accel.z
    sOut[sCtr + 39], sOut[sCtr + 40] = veh.relDistX, veh.relDistY
    sOut[sCtr + 41], sOut[sCtr + 42] = veh.relVelX, veh.relVelY
    sOut[sCtr + 43], sOut[sCtr + 44] = veh.relAccX, veh.relAccY
    veh = d.closestVehicles4
    sOut[sCtr + 45] = sqrt(veh.distToPlayerVehicleSq)
    sOut[sCtr + 46], sOut[sCtr + 47] = d.length, d.width
    local vel = veh.vel
    sOut[sCtr + 48], sOut[sCtr + 49], sOut[sCtr + 50] = vel.x, vel.y, vel.z
    local accel = veh.acc
    sOut[sCtr + 51], sOut[sCtr + 52], sOut[sCtr + 53] = accel.x, accel.y, accel.z
    sOut[sCtr + 54], sOut[sCtr + 55] = veh.relDistX, veh.relDistY
    sOut[sCtr + 56], sOut[sCtr + 57] = veh.relVelX, veh.relVelY
    sOut[sCtr + 58], sOut[sCtr + 59] = veh.relAccX, veh.relAccY
    sOut[sCtr + 60] = d.time
    sCtr = sCtr + 61
  end
  for _, v in ipairs(vehSensors.roads) do                                                    -- Append all the Roads Sensor readings.
    local d = v.ctrl.getLatest(v.id)
    sOut[sCtr], sOut[sCtr + 1], sOut[sCtr + 2] = d.halfWidth, d.roadRadius, d.headingAngle
    sOut[sCtr + 3], sOut[sCtr + 4], sOut[sCtr + 5] = d.dist2CL, d.dist2Left, d.dist2Right
    sOut[sCtr + 6], sOut[sCtr + 7], sOut[sCtr + 8] = d.drivability, d.speedLimit, d.flag1way
    sOut[sCtr + 9], sOut[sCtr + 10], sOut[sCtr + 11] = d.xP0onCL, d.yP0onCL, d.zP0onCL
    sOut[sCtr + 12], sOut[sCtr + 13], sOut[sCtr + 14] = d.xP1onCL, d.yP1onCL, d.zP1onCL
    sOut[sCtr + 15], sOut[sCtr + 16], sOut[sCtr + 17] = d.xP2onCL, d.yP2onCL, d.zP2onCL
    sOut[sCtr + 18], sOut[sCtr + 19], sOut[sCtr + 20] = d.xP3onCL, d.yP3onCL, d.zP3onCL
    sOut[sCtr + 21] = d.time
    sCtr = sCtr + 22
  end
end

-- Creates the message which will be sent out to 3rd party.
local function createMessage()

  -- Compute the outgoing properties for each group, using the jump table.
  -- [Only groups requested by the .csv will appear in the jump table].
  for k, v in ipairs(jumpTable) do
    v()
  end

  -- Prepare the message table, include a unique id for this message.
  -- [This is always the first value in the message].
  table.clear(msgOut)
  msgOut[1] = id
  id = id + 1
  for i, sig in ipairs(outMap) do
    msgOut[i + 1] = master[sig.mId][sig.i1][sig.i2]
  end
end

-- Handles received message. Sets the appropriate vehicle system properties.
local function handleMessageReceive()



  -- If the id of the received message is not new, then skip it. This is either redundant or a ghost message.
  if msgIn[1] <= maxRecvId then
    return false
  end
  maxRecvId = msgIn[1]

  -- Set the default message index to zero.
  -- [When channels are not used, they index this value].
  msgIn[defaultIndex] = 0.0

  -- Cast all Boolean-valued elements of the incoming message (which are numbers 0 or 1 in the socket), to Boolean type.
  for _, v in ipairs(inBools) do
    msgIn[v] = num2BoolTable[sign(abs(msgIn[v]))]
  end

  -- Set the 'Driver' group controls from the incoming message signals.
  for _, data in ipairs(inDMap) do
    local iF = data.iFreeze                                                                         -- The index to the freeze channel, if it exists (otherwise 1).
    local fFreeze = sign(abs(msgIn[iF]))                                                            -- Clamp the freeze channel value to 0 or 1.
    local i1 = data.i1
    local fR, fM, fA = data.fReplace, data.fMultiply, data.fAdd                                     -- The flags for each component (replace, multiply, add).
    local vR, vM, vA = msgIn[data.iValue], msgIn[data.iMultiply], msgIn[data.iAdd]                  -- The multiply and add signal values.
    local fMInv, fAInv = 1 - fM, 1 - fA                                                             -- The multiply and add channel flag opposite values.
    local cReplace = max(0, fMInv + fAInv - 1) * fR * vR                                            -- The masked 'replace' contribution.
    local vMvR = vM * vR
    local cMultiply = fM * fAInv * vMvR                                                             -- The masked 'multiply' contribution.
    local cAdd = fA * fMInv * (vR + vA)                                                             -- The masked 'add' contribution.
    local cBoth = max(0, fM + fA - 1) * (vMvR * fR + fA * vA)                                       -- The masked 'both' (multiply AND add) contribution.
    local evalWithMode = cReplace + cMultiply + cAdd + cBoth                                        -- Sum over all the masked contributions.
    local posPart = fFreeze * frozenValues[iF]
    local freezeOpp = 1.0 - fFreeze                                                                 -- The freeze channel flag opposite value.
    local evalWithModeAndFreeze = freezeOpp * evalWithMode + posPart                                -- Mask with the freeze channel value (0 or 1).
    input.event(i1, evalWithModeAndFreeze, FILTER_DIRECT)
    frozenValues[iF] = freezeOpp * evalWithModeAndFreeze + posPart                                  -- Update the frozen value, as required.
  end

  -- Set the 'Wheels' group controls from the incoming message signals.
  for _, data in ipairs(inWMap) do
    local iF = data.iFreeze                                                                         -- The index to the freeze channel, if it exists (otherwise 1).
    local fFreeze = sign(abs(msgIn[iF]))                                                            -- Clamp the freeze channel value to 0 or 1.
    local i1, WRI = data.i1, data.WRI
    local fR, fM, fA = data.fReplace, data.fMultiply, data.fAdd                                     -- The flags for each component (replace, multiply, add).
    local vR, vM, vA = msgIn[data.iValue], msgIn[data.iMultiply], msgIn[data.iAdd]                  -- The multiply and add signal values.
    local fMInv, fAInv = 1 - fM, 1 - fA                                                             -- The multiply and add channel flag opposite values.
    local cReplace = max(0, fMInv + fAInv - 1) * fR * vR                                            -- The masked 'replace' contribution.
    local vMvR = vM * vR
    local cMultiply = fM * fAInv * vMvR                                                             -- The masked 'multiply' contribution.
    local cAdd = fA * fMInv * (vR + vA)                                                             -- The masked 'add' contribution.
    local cBoth = max(0, fM + fA - 1) * (vMvR * fR + fA * vA)                                       -- The masked 'both' (multiply AND add) contribution.
    local evalWithMode = cReplace + cMultiply + cAdd + cBoth                                        -- Sum over all the masked contributions.
    local posPart = fFreeze * frozenValues[iF]
    local freezeOpp = 1.0 - fFreeze                                                                 -- The freeze channel flag opposite value.
    local evalWithModeAndFreeze = freezeOpp * evalWithMode + posPart                                -- Mask with the freeze channel value (0 or 1).
    inTorques[i1][WRI] = evalWithModeAndFreeze
    frozenValues[iF] = freezeOpp * evalWithModeAndFreeze + posPart                                  -- Update the frozen value, as required.
  end

  return true
end

-- Finds the index in the 1D 'Kinematics' structure, to which the given name relates.
local function kinematicsName2Id(name)
  if name == names.vehiclePositionX then return 1 end
  if name == names.vehiclePositionY then return 2 end
  if name == names.vehiclePositionZ then return 3 end
  if name == names.vehicleVelocityX then return 4 end
  if name == names.vehicleVelocityY then return 5 end
  if name == names.vehicleVelocityZ then return 6 end
  if name == names.vehicleAccelerationX then return 7 end
  if name == names.vehicleAccelerationY then return 8 end
  if name == names.vehicleAccelerationZ then return 9 end
  if name == names.vehicleRoll then return 10 end
  if name == names.vehiclePitch then return 11 end
  if name == names.vehicleYaw then return 12 end
  if name == names.vehicleRollRate then return 13 end
  if name == names.vehiclePitchRate then return 14 end
  if name == names.vehicleYawRate then return 15 end
  if name == names.vehicleGroundSpeed then return 16 end
  if name == names.vehicleAltitude then return 17 end
  if name == names.vehicleForwardX then return 18 end
  if name == names.vehicleForwardY then return 19 end
  if name == names.vehicleForwardZ then return 20 end
  if name == names.vehicleUpX then return 21 end
  if name == names.vehicleUpY then return 22 end
  if name == names.vehicleUpZ then return 23 end
  if name == names.vehicleRightX then return 24 end
  if name == names.vehicleRightY then return 25 end
  if name == names.vehicleRightZ then return 26 end
  if name == names.vehicleInitialLength then return 27 end
  if name == names.vehicleInitialWidth then return 28 end
  if name == names.vehicleInitialHeight then return 29 end
  if name == names.vehicleCOGWithGravity then return 30 end
  if name == names.vehicleCOGWithoutGravity then return 31 end
  if name == names.vehicleMidFrontBumperX then return 32 end
  if name == names.vehicleMidFrontBumperY then return 33 end
  if name == names.vehicleMidFrontBumperZ then return 34 end
  if name == names.vehicleMidRearBumperX then return 35 end
  if name == names.vehicleMidRearBumperY then return 36 end
  if name == names.vehicleMidRearBumperZ then return 37 end
  if name == names.vehicleFrontAxleMidpointX then return 38 end
  if name == names.vehicleFrontAxleMidpointY then return 39 end
  if name == names.vehicleFrontAxleMidpointZ then return 40 end
  if name == names.vehicleRearAxleMidpointX then return 41 end
  if name == names.vehicleRearAxleMidpointY then return 42 end
  if name == names.vehicleRearAxleMidpointZ then return 43 end
  log('E', logTag, 'Kinematics group target not found from given name.')
  return nil
end

-- Finds the index in the 1D 'Driver' structure, to which the given name relates.
local function driverName2Id(name)
  if name == names.throttle then return 'throttle' end
  if name == names.throttleInput then return 'throttle_input' end
  if name == names.brake then return 'brake' end
  if name == names.brakeInput then return 'brake_input' end
  if name == names.clutch then return 'clutch' end
  if name == names.clutchInput then return 'clutch_input' end
  if name == names.parkingBrake then return 'parkingbrake' end
  if name == names.parkingBrakeInput then return 'parkingbrake_input' end
  if name == names.steeringWheelPosition then return 'steering' end
  if name == names.steeringWheelPositionInput then return 'steering_input' end
  log('E', logTag, 'Drivers group target not found from given name.')
  return nil
end

-- Finds the index in the 1D 'Wheels' structure, to which the given name relates.
local function wheelName2Id(name)
  local numWheels, wCtr = wheels.wheelRotatorCount, 1
  for i = 1, numWheels do
    if string.find(name, wheelsOrder[i]) then
      if string.find(name, names.wheelSpeed) then return wCtr end
      if string.find(name, names.angularVelocity) then return wCtr + 1 end
      if string.find(name, names.downforce) then return wCtr + 2 end
      if string.find(name, names.brakingTorque) then return wCtr + 3 end
      if string.find(name, names.propulsionTorque) then return wCtr + 4 end
      if string.find(name, names.frictionTorque) then return wCtr + 5 end
      if string.find(name, names.wheelAngle) then return wCtr + 6 end
    end
    wCtr = wCtr + 7
  end
  log('E', logTag, 'Wheels group target not found from given name.')
  return nil
end

-- Finds the id in the wheel rotator table, from the given name.
local function wheelName2WheelRotatorId(name)
  local numWheels = wheels.wheelRotatorCount
  for i = 1, numWheels do
    if string.find(name, wheelsOrder[i]) then
      return wheelsOrder[i]
    end
  end
  log('E', logTag, 'Wheels rotator id target not found from given name.')
  return nil
end

-- Gets the electrics device key from a given signal name.
local function electricsName2Id(name)
  for k, _ in pairs(electrics.values) do                                                            -- First check the first-level keys (which the name already is).
    if k == name then
      return k
    end
  end
  log('E', logTag, 'Electrics group target not found from given name.')
  return nil
end

-- Gets the powertrain device keys from a given signal name.
-- [This method considers up to two levels deep in the powertrain devices structure].
local function powertrainName2Ids(name)
  for k, dev in pairs(powertrain.getDevices()) do
    if string.find(name, dev.name) then
      if dev.inputAV and string.find(name, 'inputAV') then return k, 'inputAV' end
      if dev.gearRatio and string.find(name, 'gearRatio') then return k, 'gearRatio' end
      if string.find(name, 'isBroken') then return k, 'isBroken' end
      if dev.mode and string.find(name, 'mode') then return k, 'mode' end
      if dev.outputTorque1 and string.find(name, 'outputTorque1') then return k, 'outputTorque1' end
      if dev.outputTorque2 and string.find(name, 'outputTorque2') then return k, 'outputTorque2' end
      if dev.outputAV1 and string.find(name, 'outputAV1') then return k, 'outputAV1' end
      if dev.outputAV2 and string.find(name, 'outputAV2') then return k, 'outputAV2' end
    end
  end
  log('E', logTag, 'Powertrain group target not found from given name.')
  return nil, nil
end

-- Finds the index in the 1D 'Sensors' structure, to which the given name relates.
local function sensorName2Id(nSig)
  local sCtr = 1
  for _, sensor in ipairs(vehSensors.IMUs) do                                                       -- First, search through the IMU sensors, in order.
    if nSig == names.imuPositionX then return sCtr end
    if nSig == names.imuPositionY then return sCtr + 1 end
    if nSig == names.imuPositionZ then return sCtr + 2 end
    if nSig == names.imuAxis1DirectionX then return sCtr + 3 end
    if nSig == names.imuAxis1DirectionY then return sCtr + 4 end
    if nSig == names.imuAxis1DirectionZ then return sCtr + 5 end
    if nSig == names.imuAxis2DirectionX then return sCtr + 6 end
    if nSig == names.imuAxis2DirectionY then return sCtr + 7 end
    if nSig == names.imuAxis2DirectionZ then return sCtr + 8 end
    if nSig == names.imuAxis3DirectionX then return sCtr + 9 end
    if nSig == names.imuAxis3DirectionY then return sCtr + 10 end
    if nSig == names.imuAxis3DirectionZ then return sCtr + 11 end
    if nSig == names.imuMass then return sCtr + 12 end
    if nSig == names.imuAngularVelocityRawAxis1 then return sCtr + 13 end
    if nSig == names.imuAngularVelocityRawAxis2 then return sCtr + 14 end
    if nSig == names.imuAngularVelocityRawAxis3 then return sCtr + 15 end
    if nSig == names.imuAngularVelocitySmoothedAxis1 then return sCtr + 16 end
    if nSig == names.imuAngularVelocitySmoothedAxis2 then return sCtr + 17 end
    if nSig == names.imuAngularVelocitySmoothedAxis3 then return sCtr + 18 end
    if nSig == names.imuAccelerationRawAxis1 then return sCtr + 19 end
    if nSig == names.imuAccelerationRawAxis2 then return sCtr + 20 end
    if nSig == names.imuAccelerationRawAxis3 then return sCtr + 21 end
    if nSig == names.imuAccelerationSmoothedAxis1 then return sCtr + 22 end
    if nSig == names.imuAccelerationSmoothedAxis2 then return sCtr + 23 end
    if nSig == names.imuAccelerationSmoothedAxis3 then return sCtr + 24 end
    if nSig == names.imuAngularAccelerationAxis1 then return sCtr + 25 end
    if nSig == names.imuAngularAccelerationAxis2 then return sCtr + 26 end
    if nSig == names.imuAngularAccelerationAxis3 then return sCtr + 27 end
    if nSig == names.imuReadingTimestamp then return sCtr + 28 end
    sCtr = sCtr + 29
  end
  for _, sensor in ipairs(vehSensors.GPSs) do                                                       -- Second, search through the GPS sensors, in order.
    if nSig == names.gpsXCoordinate then return sCtr end
    if nSig == names.gpsYCoordinate then return sCtr + 1 end
    if nSig == names.gpsLongitude then return sCtr + 2 end
    if nSig == names.gpsLatitude then return sCtr + 3 end
    if nSig == names.gpsReadingTimestamp then return sCtr + 4 end
    sCtr = sCtr + 5
  end
  for _, sensor in ipairs(vehSensors.idealRADARs) do                                                -- Third, search through the Ideal RADAR sensor, if it exists.
    if nSig == names.idealRADARVehicle1Distance then return sCtr end
    if nSig == names.idealRADARVehicle1Length then return sCtr + 1 end
    if nSig == names.idealRADARVehicle1Width then return sCtr + 2 end
    if nSig == names.idealRADARVehicle1VelocityX then return sCtr + 3 end
    if nSig == names.idealRADARVehicle1VelocityY then return sCtr + 4 end
    if nSig == names.idealRADARVehicle1VelocityZ then return sCtr + 5 end
    if nSig == names.idealRADARVehicle1AccelerationX then return sCtr + 6 end
    if nSig == names.idealRADARVehicle1AccelerationY then return sCtr + 7 end
    if nSig == names.idealRADARVehicle1AccelerationZ then return sCtr + 8 end
    if nSig == names.idealRADARVehicle1RelativeDistanceX then return sCtr + 9 end
    if nSig == names.idealRADARVehicle1RelativeDistanceY then return sCtr + 10 end
    if nSig == names.idealRADARVehicle1RelativeVelocityX then return sCtr + 11 end
    if nSig == names.idealRADARVehicle1RelativeVelocityY then return sCtr + 12 end
    if nSig == names.idealRADARVehicle1RelativeAccelerationX then return sCtr + 13 end
    if nSig == names.idealRADARVehicle1RelativeAccelerationY then return sCtr + 14 end
    if nSig == names.idealRADARVehicle2Distance then return sCtr + 15 end
    if nSig == names.idealRADARVehicle2Length then return sCtr + 16 end
    if nSig == names.idealRADARVehicle2Width then return sCtr + 17 end
    if nSig == names.idealRADARVehicle2VelocityX then return sCtr + 18 end
    if nSig == names.idealRADARVehicle2VelocityY then return sCtr + 19 end
    if nSig == names.idealRADARVehicle2VelocityZ then return sCtr + 20 end
    if nSig == names.idealRADARVehicle2AccelerationX then return sCtr + 21 end
    if nSig == names.idealRADARVehicle2AccelerationY then return sCtr + 22 end
    if nSig == names.idealRADARVehicle2AccelerationZ then return sCtr + 23 end
    if nSig == names.idealRADARVehicle2RelativeDistanceX then return sCtr + 24 end
    if nSig == names.idealRADARVehicle2RelativeDistanceY then return sCtr + 25 end
    if nSig == names.idealRADARVehicle2RelativeVelocityX then return sCtr + 26 end
    if nSig == names.idealRADARVehicle2RelativeVelocityY then return sCtr + 27 end
    if nSig == names.idealRADARVehicle2RelativeAccelerationX then return sCtr + 28 end
    if nSig == names.idealRADARVehicle2RelativeAccelerationY then return sCtr + 29 end
    if nSig == names.idealRADARVehicle3Distance then return sCtr + 30 end
    if nSig == names.idealRADARVehicle3Length then return sCtr + 31 end
    if nSig == names.idealRADARVehicle3Width then return sCtr + 32 end
    if nSig == names.idealRADARVehicle3VelocityX then return sCtr + 33 end
    if nSig == names.idealRADARVehicle3VelocityY then return sCtr + 34 end
    if nSig == names.idealRADARVehicle3VelocityZ then return sCtr + 35 end
    if nSig == names.idealRADARVehicle3AccelerationX then return sCtr + 36 end
    if nSig == names.idealRADARVehicle3AccelerationY then return sCtr + 37 end
    if nSig == names.idealRADARVehicle3AccelerationZ then return sCtr + 38 end
    if nSig == names.idealRADARVehicle3RelativeDistanceX then return sCtr + 39 end
    if nSig == names.idealRADARVehicle3RelativeDistanceY then return sCtr + 40 end
    if nSig == names.idealRADARVehicle3RelativeVelocityX then return sCtr + 41 end
    if nSig == names.idealRADARVehicle3RelativeVelocityY then return sCtr + 42 end
    if nSig == names.idealRADARVehicle3RelativeAccelerationX then return sCtr + 43 end
    if nSig == names.idealRADARVehicle3RelativeAccelerationY then return sCtr + 44 end
    if nSig == names.idealRADARVehicle4Distance then return sCtr + 45 end
    if nSig == names.idealRADARVehicle4Length then return sCtr + 46 end
    if nSig == names.idealRADARVehicle4Width then return sCtr + 47 end
    if nSig == names.idealRADARVehicle4VelocityX then return sCtr + 48 end
    if nSig == names.idealRADARVehicle4VelocityY then return sCtr + 49 end
    if nSig == names.idealRADARVehicle4VelocityZ then return sCtr + 50 end
    if nSig == names.idealRADARVehicle4AccelerationX then return sCtr + 51 end
    if nSig == names.idealRADARVehicle4AccelerationY then return sCtr + 52 end
    if nSig == names.idealRADARVehicle4AccelerationZ then return sCtr + 53 end
    if nSig == names.idealRADARVehicle4RelativeDistanceX then return sCtr + 54 end
    if nSig == names.idealRADARVehicle4RelativeDistanceY then return sCtr + 55 end
    if nSig == names.idealRADARVehicle4RelativeVelocityX then return sCtr + 56 end
    if nSig == names.idealRADARVehicle4RelativeVelocityY then return sCtr + 57 end
    if nSig == names.idealRADARVehicle4RelativeAccelerationX then return sCtr + 58 end
    if nSig == names.idealRADARVehicle4RelativeAccelerationY then return sCtr + 59 end
    if nSig == names.idealRADARReadingTimestamp then return sCtr + 60 end
    sCtr = sCtr + 61
  end
  for _, sensor in ipairs(vehSensors.roads) do                                                      -- Last, search through the Roads Sensor, if it exists.
    if nSig == names.roadsRoadHalfWidth then return sCtr end
    if nSig == names.roadsRoadRadius then return sCtr + 1 end
    if nSig == names.roadsRoadHeading then return sCtr + 2 end
    if nSig == names.roadsDistanceToCenterline then return sCtr + 3 end
    if nSig == names.roadsDistanceToRoadLeftEdge then return sCtr + 4 end
    if nSig == names.roadsDistanceToRoadRightEdge then return sCtr + 5 end
    if nSig == names.roadsDrivability then return sCtr + 6 end
    if nSig == names.roadsSpeedLimit then return sCtr + 7 end
    if nSig == names.roadsIsOneWay then return sCtr + 8 end
    if nSig == names.roadsClosestPointX then return sCtr + 9 end
    if nSig == names.roadsClosestPointY then return sCtr + 10 end
    if nSig == names.roadsClosestPointZ then return sCtr + 11 end
    if nSig == names.roads2ndClosestPointX then return sCtr + 12 end
    if nSig == names.roads2ndClosestPointY then return sCtr + 13 end
    if nSig == names.roads2ndClosestPointZ then return sCtr + 14 end
    if nSig == names.roads3rdClosestPointX then return sCtr + 15 end
    if nSig == names.roads3rdClosestPointY then return sCtr + 16 end
    if nSig == names.roads3rdClosestPointZ then return sCtr + 17 end
    if nSig == names.roads4thClosestPointX then return sCtr + 18 end
    if nSig == names.roads4thClosestPointY then return sCtr + 19 end
    if nSig == names.roads4thClosestPointZ then return sCtr + 20 end
    if nSig == names.roadsReadingTimestamp then return sCtr + 21 end
    sCtr = sCtr + 22
  end
  log('E', logTag, 'Sensors group target not found from given name.')
  return nil
end

-- Computes the mapping structures.
local function computeMappingStructures(signalsTo, signalsFrom, sensorMap)

  -- Compute an ordered array for the wheels.
  local wheelIds, wOrderCtr = wheelIds, 1
  for k, _ in pairs(wheelIds) do
    wheelsOrder[wOrderCtr] = k
    wOrderCtr = wOrderCtr + 1
  end

  -- Compute the outgoing signals map.
  -- [Each entry in the map contains indices for the intermediate 'master' structure].
  table.clear(outMap)
  local isK, isW, isE, isP, isS = false, false, false, false, false
  local numSigTo = #signalsTo
  for i = 1, numSigTo do
    local s = signalsTo[i]
    local name, g = s.name, s.groupName
    if g == groups.kinematics then
      outMap[i] = { mId = 1, i1 = 1, i2 = kinematicsName2Id(name) }
      isK = true
    elseif g == groups.driver then
      outMap[i] = { mId = 3, i1 = 1, i2 = driverName2Id(name) }
      isE = true
    elseif g == groups.wheels then
      outMap[i] = { mId = 2, i1 = 1, i2 = wheelName2Id(name) }
      isW = true
    elseif g == groups.electrics then
      outMap[i] = { mId = 3, i1 = 1, i2 = electricsName2Id(name) }
      isE = true
    elseif g == groups.powertrain then
      local i1, i2 = powertrainName2Ids(name)
      outMap[i] = { mId = 4, i1 = i1, i2 = i2 }
      isP = true
    elseif string.find(g, 'IMU') or string.find(g, 'GPS') or string.find(g, 'Ideal RADAR') or string.find(g, 'Roads') then
      outMap[i] = { mId = 5, i1 = 1, i2 = sensorName2Id(name) }
      isS = true
    else
      outMap[i] = nil
    end
  end

  -- Create the outgoing message jump table linkages.
  -- [This ensures that in update, we only consider groups which appear in the .csv].
  if isK then table.insert(jumpTable, gatherKinematicsProperties) end
  if isW then table.insert(jumpTable, gatherWheelsProperties) end
  if isE then table.insert(jumpTable, gatherElectricsProperties) end
  if isP then table.insert(jumpTable, gatherPowertrainProperties) end
  if isS then table.insert(jumpTable, gatherSensorsProperties) end

  -- Create hashtables for the 'Driving' and 'Wheels' incoming message groups.
  -- [We store the indices in the message, to each of the channels which appear in the incoming signals list].
  table.clear(inBools)
  local dTable, wTable, numSigFrom = {}, {}, #signalsFrom
  for i = 1, numSigFrom do
    local s = signalsFrom[i]
    local name, g, iPlus1 = s.name, s.groupName, i + 1                                              -- Increment the index, since the msg always has the id at position 1.
    if g == groups.driver then                                                                      -- The incoming 'Driver' group signals (pedals, steering).
      if not dTable[name] then
        dTable[name] = {
          i1 = driverName2Id(name),
          fReplace = 1, fMultiply = 0, fAdd = 0,
          iValue = defaultIndex, iMultiply = defaultIndex, iAdd = defaultIndex, iFreeze = defaultIndex }
      end
      if s.isValue then dTable[name].iValue = iPlus1 end                                            -- Store the index in the incoming msg, of where the channels are.
      if s.isMultiply then dTable[name].iMultiply, dTable[name].fMultiply = iPlus1, 1 end
      if s.isAdd then dTable[name].iAdd, dTable[name].fAdd = iPlus1, 1 end
      if s.isFreeze then dTable[name].iFreeze = iPlus1 end
    elseif g == groups.wheels then                                                                  -- The incoming 'Wheels' group signals (wheel torques).
      if not wTable[name] then
        local WRI = wheelIds[wheelName2WheelRotatorId(name)]
        if string.find(name, "propulsionTorque") then
          wTable[name] = {
            i1 = 1, WRI = WRI,
            fReplace = 1, fMultiply = 0, fAdd = 0,
            iValue = defaultIndex, iMultiply = defaultIndex, iAdd = defaultIndex, iFreeze = defaultIndex }
          pTorqueKeys[WRI] = true
          inTorques[1][WRI] = 0.0
        elseif string.find(name, "brakingTorque") then
          wTable[name] = { i1 = 2, WRI = WRI,
          fReplace = 1, fMultiply = 0, fAdd = 0,
          iValue = defaultIndex, iMultiply = defaultIndex, iAdd = defaultIndex, iFreeze = defaultIndex }
          bTorqueKeys[WRI] = true
          inTorques[2][WRI] = 0.0
        elseif string.find(name, "frictionTorque") then
          wTable[name] = { i1 = 3, WRI = WRI,
          fReplace = 1, fMultiply = 0, fAdd = 0,
          iValue = defaultIndex, iMultiply = defaultIndex, iAdd = defaultIndex, iFreeze = defaultIndex }
          fTorqueKeys[WRI] = true
          inTorques[3][WRI] = 0.0
        end
      end
      if s.isValue then wTable[name].iValue = iPlus1 end                                            -- Store the index in the incoming msg, of where the channels are.
      if s.isMultiply then wTable[name].iMultiply, wTable[name].fMultiply = iPlus1, 1 end
      if s.isAdd then wTable[name].iAdd, wTable[name].fAdd = iPlus1, 1 end
      if s.isFreeze then wTable[name].iFreeze = iPlus1 end
    end

    -- Store an array of all the Boolean-valued input message indices.
    -- [We do not include freeze channel values, which should remain as numbers].
    if s.type == 'boolean' and not s.isFreeze then
      table.insert(inBools, iPlus1)
    end
  end

  -- Convert the hashtables to ordered arrays, to create the final incoming maps.
  table.clear(inDMap)
  table.clear(inWMap)
  local ctr = 1
  for _, v in pairs(dTable) do
    inDMap[ctr] = v
    ctr = ctr + 1
  end
  ctr = 1
  for _, v in pairs(wTable) do
    inWMap[ctr] = v
    ctr = ctr + 1
  end

  -- Initialize the frozen values storage structure.
  -- [These are defaulted to all start at zero when the coupling starts executing].
  table.clear(frozenValues)
  for i = 1, numSigFrom do
    if signalsFrom[i].isFreeze then
      frozenValues[i + 1] = 0.0
    end
  end
  frozenValues[defaultIndex] = 0.0                                                                  -- Set a zero freeze value at the default index position.
end



local function extractSignalNames(signals)
  local names = {}
  for i, signal in ipairs(signals) do
    names[i] = signal.name -- Extract the actual signal name
  end
  return names
end


-- Initialisation callback.
local function init(dataEncoded)

  -- Decode the given setup data.
  local data = lpack.decode(dataEncoded)[1]

  -- Set the connection data.
  time3rdParty, pingTime = data.time3rdParty, data.pingTime
  udpSendIP, udpSendPort = data.udpSendIP, data.udpSendPort
  udpReceiveIP, udpReceivePort = data.udpReceiveIP, data.udpReceivePort
  enableVSL=data.enableVSL
  enableCosim=data.enableCosim
  -- Cache some properties known at init, in state.
  initLength, initWidth, initHeight = obj:getInitialLength(), obj:getInitialWidth(), obj:getInitialHeight()

  -- Set the sensors mapping and sensor controller references.
  -- [This structure maps the sensor name to the sensor id in the simulator (either ge lua or gameengine)].
  vehSensors = data.sensorMap
  local IMUs, GPSs, idealRADARs, roads = vehSensors.IMUs, vehSensors.GPSs, vehSensors.idealRADARs, vehSensors.roads

  local idOffset = 0
  for i = 1, #IMUs do
    while IMUs[i].ctrl == nil do
    IMUs[i].ctrl = controller.getController('advancedIMU' .. IMUs[i].id)
        IMUs[i].ctrl = controller.getController('advancedIMU' .. (IMUs[i].id + idOffset))
        if IMUs[i].ctrl == nil then
            idOffset = idOffset + 1
        end
    end
  end

  for i = 1, #GPSs do
    GPSs[i].ctrl = controller.getController('GPS' .. GPSs[i].id)
  end
  for i = 1, #idealRADARs do
    idealRADARs[i].ctrl = controller.getController('idealRADARSensor' .. idealRADARs[i].id)
  end
  for i = 1, #roads do
    roads[i].ctrl = controller.getController('roadsSensor' .. roads[i].id)
  end

  -- Compute the in/out mapping structures.
  computeMappingStructures(data.signalsTo, data.signalsFrom, data.sensorMap)

  -- Set up the UDP send and receive sockets.
  -- [We always start with a non-blocking receive socket (by using zero timeout)].
  if enableCosim then
    udpSendSocket = socket.udp()
    local _, error = udpSendSocket:setpeername(udpSendIP, udpSendPort)
    if error then
      log('E', logTag, 'UDP send socket could not be set up.')
    end
    udpRecvSocket = socket.udp()
    udpRecvSocket:settimeout(0.0)
    local _, error = udpRecvSocket:setsockname(udpReceiveIP, udpReceivePort)
    if error then
      log('E', logTag, 'UDP receive socket could not be set up.')
    end

    -- Compute some internal control parameters, relating to the coupling window.
    sendSkips = ceil(time3rdParty / physicsDt) - 1
    unanswered = ceil(pingTime / time3rdParty)

    log('I', logTag, 'Coupling between BeamNG and 3rd party has started.')
  end


  -- Create CSV files if VSL is enabled
  if enableVSL then
    local sendNames = extractSignalNames(data.signalsTo)
    local receiveNames = extractSignalNames(data.signalsFrom)

    if #sendNames == 0 or #receiveNames == 0 then
      log('E', logTag, 'Signal indexes are empty. CSV cannot be created.')
    else
      log('I', logTag, 'Creating CSV files for logging.')
      createCSV(sendNames, receiveNames)
    end

    -- Debug log to check if csvSendData is successfully created
    if csvSendData then
      log('I', logTag, 'csvSendData successfully initialized.')
    else
      log('E', logTag, 'Failed to initialize csvSendData.')
    end
  end


initialized = true

end

-- Callback for setting wheel torques.
local function updateWheelsIntermediate(dt)
  local propulsionTorques = inTorques[1]
  for k, _ in pairs(pTorqueKeys) do                                                                 -- Set wheel propulsion torque only if set to be controlled by 3rd party.
    local wheel = wheelRotators[k]
    wheel.propulsionTorque = propulsionTorques[k] * wheel.wheelDir
  end
  local brakeTorques = inTorques[2]
  for k, _ in pairs(bTorqueKeys) do                                                                 -- Set wheel braking torque only if set to be controlled by 3rd party.
    wheelRotators[k].desiredBrakingTorque = -brakeTorques[k]
  end
  local frictionTorques = inTorques[3]
  for k, _ in pairs(fTorqueKeys) do                                                                 -- Set wheel friction torque only if set to be controlled by 3rd party.
    wheelRotators[k].frictionTorque = frictionTorques[k]
  end
end



local function addToCSV(msgOut)
  cycleTime = os.clockhp()  - lastTimeOfSend
  lastTimeOfSend = os.clockhp()
  csvSendData:add(os.clockhp(), unpack(msgOut))
end

-- Update callback.
local function update(dt)

  if enableVSL and csvSendData == nil then
    log('W', logTag, 'csvSendData is nil. Attempting to initialize CSV again.')
    local sendIndexes = linspace(#dat.names)  -- Adjust dat.names if needed to fit your signal structure
    local receiveIndexes = linspace(#dat.names)
    createCSV(sendIndexes, receiveIndexes)
    if csvSendData == nil then
        log('E', logTag, 'Failed to initialize csvSendData in update(). Skipping CSV logging.')
        return
    else
        log('I', logTag, 'csvSendData successfully created during update().')
    end
  end

if not initialized then
  log('W', logTag, 'Update called before initialization is complete.')
  return
end


if enableCosim then
  if stepsSinceLastSend >= sendSkips then                                                           -- Determine if we should skip sending in this cycle.
    -- We will send in this cycle. Check if we have reached the window width.
    -- [This is when we have received the same number of messages as we have sent out].
    if sendCtr - maxRecvId < unanswered then
      udpRecvSocket:settimeout(0.0)                                                                 -- Not yet reached window width, so do non-blocking recv and send new msg.
      local rawMsgFrom3rdParty = udpRecvSocket:receive()
      if rawMsgFrom3rdParty ~= nil then
        table.clear(msgIn)
        lpack.decodeDoubleArray(rawMsgFrom3rdParty, msgIn)
        handleMessageReceive()
        if enableVSL then
        csvReceiveData:add(os.clockhp(), unpack(msgIn))
        end

      end
      createMessage()
      udpSendSocket:send(lpack.encodeDoubleArray(msgOut))
      sendCtr, stepsSinceLastSend = sendCtr + 1, 0
      if enableVSL then
        addToCSV(msgOut)
      end


    else
      udpRecvSocket:settimeout(blockingTimeoutLength)                                               -- Reached the window width, so do blocking receive and send a new msg.
      local rawMsgFrom3rdParty = udpRecvSocket:receive()
      if rawMsgFrom3rdParty ~= nil then
        table.clear(msgIn)
        lpack.decodeDoubleArray(rawMsgFrom3rdParty, msgIn)
        if handleMessageReceive() or maxRecvId == 0 then
          createMessage()
          udpSendSocket:send(lpack.encodeDoubleArray(msgOut))
          sendCtr, stepsSinceLastSend = sendCtr + 1, 0
          if enableVSL then
            addToCSV(msgOut)
          end
        end
      else
        sendCtr, maxRecvId = 0, 0
      end
    end
  else
      udpRecvSocket:settimeout(0.0)                                                                   -- Must skip sending in this cycle, so only perform a non-blocking recv.
      local rawMsgFrom3rdParty = udpRecvSocket:receive()
      if rawMsgFrom3rdParty ~= nil then
        table.clear(msgIn)
        lpack.decodeDoubleArray(rawMsgFrom3rdParty, msgIn)
        handleMessageReceive()
        if enableVSL then
        csvReceiveData:add(os.clockhp(), unpack(msgIn))
        end


    end
    stepsSinceLastSend = stepsSinceLastSend + 1
  end

  else

    if not csvSendData then
      log('E', logTag, 'csvSendData is not initialized. Skipping CSV logging.')
      return
    end
    cycleTime = os.clockhp() - lastTimeOfSend
    lastTimeOfSend = os.clockhp()

    -- Create message data for logging
    createMessage()

    -- Add to CSV for logging
    csvSendData:add(os.clockhp(), unpack(msgOut))
    -- log('I', logTag, 'VSL only from controller, logging data.')




end




end


-- Stops the coupling between BeamNG and the 3rd party.
local function stop()
  if enableCosim then
  udpSendSocket:close()
  udpRecvSocket:close()
  end
  if enableVSL then
    saveCSV()
    log('I', logTag, 'Coupling between BeamNG and 3rd party has terminated and CSV saved too')
  end

end




-- Public interface.
M.updateWheelsIntermediate =                              updateWheelsIntermediate
M.init =                                                  init
M.update =                                                update
M.stop =                                                  stop

return M