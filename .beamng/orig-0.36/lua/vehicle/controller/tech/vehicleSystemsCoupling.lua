-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This controller provides a generic interface between the BeamNG powertrain and a Simulink model. Powertrain values are sent to Simulink, where external computations
-- are done, then Simulink sends back control values for other properties (eg torque values).

-- The user must set the 'simulinkTime' and 'pingTime' properties near the top of this file, in order to achieve efficient coupling.

-- The 'udpIP' IP address string should also be set to match the address of the machine which is executing Simulink.

-- *** USER CONTROL PARAMETERS ***
-- These need to be set by the user before commencing BeamNG-Simulink coupling.
local udpSendIP = "127.0.0.1"           -- The IP address for the udp communication on the Simulink computer. 127.0.0.1 is the loopback port (for sending on same machine).
local udpReceiveIP = "127.0.0.1"        -- The IP address for the udp communication on the BeamNG computer. 127.0.0.1 is the loopback port (for sending on same machine).
local udpSendPort = 64890               -- The port number on the Simulink computer, that Simulink will receive data on.
local udpReceivePort = 64891            -- The port number on the BeamNG computer, that BeamNG will receive data on.
local simulinkTime = 0.005              -- The Simulink computation time. This is how long it takes Simulink to perform its computations upon receiving a message until responding.
local pingTime = 0.00001                -- The ping round-trip time. This is the time taken for a message to be sent from the BeamNG computer to Simulink and back.

----------

local lpack = require("lpack")

local M = {}
M.type = "auxiliary"

local logTag = 'vehicleSystemsCoupling'

local abs = math.abs

-- The UDP socket properties.
local udpSendSocket                     -- The UDP socket for sending data from Lua to Simulink.
local udpRecvSocket                     -- The UDP socket for sending data from Simulink to Lua.

-- Properties used to control the wheels. These are internal and should not be adjusted by the user.
local brakeTorques = {}                 -- The brake torque values for each wheel, in N-m.
local propulsionTorques = {}            -- The propulsion torque values for each wheel, in N-m.
local isUsingTorques = false            -- The flag which indicates the drive mode [false = pedals or true = torques].
local wheelsOrder = {"FL", "FR", "RL", "RR"}
local wheelMessageOffsets = {FL = 38, FR = 44, RL = 50, RR = 56}

-- BeamNG-Simulink coupling properties. These are internal and should not be adjusted by the user.
local sendCtr = 0                       -- Counts how many messages have been sent.
local maxRecvId = 0                     -- The maximum Id which has been received so far.
local stepsSinceLastSend = 0            -- Counts the number of update cycles since the last message was sent.
local sendSkips = 0                     -- The number of updates to skip between message sends, in the chosen window size.
local unanswered = 0                    -- The number of unanswered received messages we are allowed to have in the chosen window size.
local id = 0                            -- The unique Id number for messages sent over the sockets.
local blockingTimeoutLength = 2.0       -- The timeout size (in seconds) when blocking on message receives.
local messageToSimulink = {}            -- The table used to store the outgoing message to be sent to Simulink, via UDP.
local decodedMessageFromSimulink = {1}  -- The table used to store the decoded message from Simulink.

-- Test
local csvSendData
local csvReceiveData
local csvPhysicsSteps
local debugFile
local csvWriter = require('csvlib')
local sendLength = 111
local receiveLength = 64

local function linspace(length)
  local out = {}
  for i=1,length,1 do
    out[i] = tostring(i)
  end
  return out
end

local function createCSV(sendKeys, receiveKeys)
  csvSendData = csvWriter.newCSV('time', unpack(sendKeys))
  csvReceiveData = csvWriter.newCSV('time', unpack(receiveKeys))
  csvPhysicsSteps = csvWriter.newCSV('time')
end

local function saveCSV()
  -- save in AppData/Local/BeamNG/<current Version>
  csvSendData:write('simulinkSendLog.csv')
  csvReceiveData:write('simulinkReceiveLog.csv')
  csvPhysicsSteps:write('simulinkPhysicsStepsLog.csv')
end

-- Creates the message which will be sent out to Simulink via UDP. This message contains the most-recent vehicle systems properties from vlua.
local function createMessage()
  table.clear(messageToSimulink)

  -- Include a Unique Id for this message, and increment the unique Id counter.
  messageToSimulink[1] = id
  id = id + 1

  -- Bank A: Driver Controls.
  messageToSimulink[2] = electrics.values.throttle or 0             -- Throttle, [0, 1] continuous.
  messageToSimulink[3] = electrics.values.throttle_input or 0       -- Throttle input, [0, 1] continuous.
  messageToSimulink[4] = electrics.values.brake or 0                -- Brake, [0, 1] continuous.
  messageToSimulink[5] = electrics.values.brake_input or 0          -- Brake input, [0, 1] continuous.
  messageToSimulink[6] = electrics.values.clutch or 0               -- Clutch, [0, 1] continuous.
  messageToSimulink[7] = electrics.values.clutch_input or 0         -- Clutch input, [0, 1] continuous.
  messageToSimulink[8] = electrics.values.parkingbrake or 0         -- Parking brake, [0, 1] continuous.
  messageToSimulink[9] = electrics.values.parkingbrake_input or 0   -- Parking brake input, [0, 1] continuous.
  messageToSimulink[10] = electrics.values.steering or 0             -- Steering, [-1, 1] continuous.
  messageToSimulink[11] = electrics.values.steering_input or 0      -- Steering input, [-1, 1] continuous.

  -- Bank B: Body State.
  local pos = obj:getPosition()
  local vel = obj:getVelocity()
  local roll, pitch, yaw = obj:getRollPitchYaw()
  messageToSimulink[12] = pos.x                                     -- Position X, m.
  messageToSimulink[13] = pos.y                                     -- Position Y, m.
  messageToSimulink[14] = pos.z                                     -- Position Z, m.
  messageToSimulink[15] = vel.x                                     -- Velocity X, ms^-1.
  messageToSimulink[16] = vel.y                                     -- Velocity Y, ms^-1.
  messageToSimulink[17] = vel.z                                     -- Velocity Z, ms^-1.
  messageToSimulink[18] = obj:getGroundSpeed()                      -- Velocity, ms^-1.
  messageToSimulink[19] = sensors.ffiSensors.sensorX                -- Acceleration X, ms^-2.
  messageToSimulink[20] = sensors.ffiSensors.sensorY                -- Acceleration Y, ms^-2.
  messageToSimulink[21] = sensors.ffiSensors.sensorZnonInertial     -- Acceleration Z, ms^-2.
  messageToSimulink[22] = roll                                      -- Roll, rad.
  messageToSimulink[23] = pitch                                     -- Pitch, rad.
  messageToSimulink[24] = yaw                                       -- Yaw, rad.
  messageToSimulink[25] = obj:getAltitude()                         -- Altitude, m.

  -- Bank C: Status.
  messageToSimulink[26] = electrics.values.ignitionLevel or 0       -- Ignition level, [0, 1, 2, 3] integer.
  messageToSimulink[27] = electrics.values.gearIndex or 0           -- Gear, number.
  messageToSimulink[28] = electrics.values.fuel or 0                -- Fuel, [0, 1] continuous.
  messageToSimulink[29] = electrics.values.engineLoad or 0          -- Engine load, [0, 1] continuous.
  messageToSimulink[30] = electrics.values.highbeam or 0            -- High beam, [0, 1]
  messageToSimulink[31] = electrics.values.lowbeam or 0             -- Low beam, [0, 1]
  messageToSimulink[32] = electrics.values.maxrpm or 0              -- Maximum RPM, 1/min.
  messageToSimulink[33] = electrics.values.reverse or 0             -- Reverse, [0, 1]
  messageToSimulink[34] = electrics.values.rpm or 0                 -- RPM, 1/min.
  messageToSimulink[35] = electrics.values.signal_L or 0            -- Signal L, [0 or 1].
  messageToSimulink[36] = electrics.values.signal_R or 0            -- Signal R, [0 or 1].
  messageToSimulink[37] = electrics.values.wheelspeed or 0          -- Wheel speed, ms^-1.

  -- Banks D, E, F, G (for wheels FL, FR, RL, RR).
  for i = 1, wheels.wheelRotatorCount do
    local wheel =  wheels.wheelRotators[wheels.wheelRotatorIDs[wheelsOrder[i]]]
    local messageOffset = wheelMessageOffsets[wheel.name]
    if messageOffset then
      messageToSimulink[messageOffset] = wheel.angularVelocityBrakeCouple * wheel.wheelDir                  -- Wheel FL angular velocity, rad/s.
      messageToSimulink[messageOffset + 1] = wheel.wheelSpeed                                               -- Wheel FL wheel speed, ms^-1.
      messageToSimulink[messageOffset + 2] = abs(wheel.coreData.brakeTorqueApplied) - wheel.frictionTorque  -- Wheel FL braking torque, Nm.
      messageToSimulink[messageOffset + 3] = wheel.propulsionTorque * wheel.wheelDir                        -- Wheel FL propulsion torque, Nm.
      messageToSimulink[messageOffset + 4] = wheel.frictionTorque                                           -- Wheel FL friction torque, Nm.
      messageToSimulink[messageOffset + 5] = wheel.downForce                                                -- Wheel FL downforce, Nm.
    end
  end

  -- Bank H: Custom User Values.
  messageToSimulink[62] = 0.0
  messageToSimulink[63] = 0.0
  messageToSimulink[64] = 0.0
  messageToSimulink[65] = 0.0
  messageToSimulink[66] = 0.0
  messageToSimulink[67] = 0.0
  messageToSimulink[68] = 0.0
  messageToSimulink[69] = 0.0
  messageToSimulink[70] = 0.0
  messageToSimulink[71] = 0.0
  messageToSimulink[72] = 0.0
  messageToSimulink[73] = 0.0
  messageToSimulink[74] = 0.0
  messageToSimulink[75] = 0.0
  messageToSimulink[76] = 0.0
  messageToSimulink[77] = 0.0
  messageToSimulink[78] = 0.0
  messageToSimulink[79] = 0.0
  messageToSimulink[80] = 0.0
  messageToSimulink[81] = 0.0
  messageToSimulink[82] = 0.0
  messageToSimulink[83] = 0.0
  messageToSimulink[84] = 0.0
  messageToSimulink[85] = 0.0
  messageToSimulink[86] = 0.0
  messageToSimulink[87] = 0.0
  messageToSimulink[88] = 0.0
  messageToSimulink[89] = 0.0
  messageToSimulink[90] = 0.0
  messageToSimulink[91] = 0.0
  messageToSimulink[92] = 0.0
  messageToSimulink[93] = 0.0
  messageToSimulink[94] = 0.0
  messageToSimulink[95] = 0.0
  messageToSimulink[96] = 0.0
  messageToSimulink[97] = 0.0
  messageToSimulink[98] = 0.0
  messageToSimulink[99] = 0.0
  messageToSimulink[100] = 0.0
  messageToSimulink[101] = 0.0
  messageToSimulink[102] = 0.0
  messageToSimulink[103] = 0.0
  messageToSimulink[104] = 0.0
  messageToSimulink[105] = 0.0
  messageToSimulink[106] = 0.0
  messageToSimulink[107] = 0.0
  messageToSimulink[108] = 0.0
  messageToSimulink[109] = 0.0
  messageToSimulink[110] = 0.0
  messageToSimulink[111] = 0.0
end

-- Sends a message to Simulink, via the UDP send socket.
local function sendUDP()
  local serialisedMsg = lpack.encodeDoubleArray(messageToSimulink)
  udpSendSocket:send(serialisedMsg)
  sendCtr = sendCtr + 1
  stepsSinceLastSend = 0

  if debugFile then
    csvSendData:add(os.clockhp(), unpack(messageToSimulink))
  end
end

-- Attempts to receive a message from Simulink, from the UDP receive socket.
local function receiveUDP()
  return udpRecvSocket:receive()
end

-- Handles received message. Sets the appropriate vehicle system properties, using data which has arrived from Simulink.
local function handleMessageReceive()
  -- If the Id of the received message is not new, then skip it. This is either redundant or a ghost message.

  if decodedMessageFromSimulink[1] <= maxRecvId then
    return false
  end

  if debugFile then
    csvReceiveData:add(os.clockhp(), unpack(decodedMessageFromSimulink))
  end

  maxRecvId = decodedMessageFromSimulink[1]

  -- Bank A: Vehicle values.
  local throttleInput = decodedMessageFromSimulink[2]
  if throttleInput == throttleInput then
    input.event("throttle", throttleInput, FILTER_DIRECT)             -- Engine throttle.
  end

  local brakeInput = decodedMessageFromSimulink[3]
  if brakeInput == brakeInput then
    input.event("brake", brakeInput, FILTER_DIRECT)                   -- Brake pedal.
  end

  local steeringInput = decodedMessageFromSimulink[4]
  if steeringInput == steeringInput then
    input.event("steering", steeringInput, FILTER_DIRECT)             -- Steering input.
  end

  local gearValue = decodedMessageFromSimulink[5]
  if gearValue == gearValue then
    -- TODO: THIS PROPERTY NEEDS TO BE LINKED TO SIMULATOR.
  end

  local wheelFLBrakingTorque = decodedMessageFromSimulink[6]          -- Brake torque FL, in N-m.
  if wheelFLBrakingTorque == wheelFLBrakingTorque then
    brakeTorques[1] = wheelFLBrakingTorque
  end

  local wheelFRBrakingTorque = decodedMessageFromSimulink[7]          -- Brake torque FR, in N-m.
  if wheelFRBrakingTorque == wheelFRBrakingTorque then
    brakeTorques[2] = wheelFRBrakingTorque
  end

  local wheelRLBrakingTorque = decodedMessageFromSimulink[8]          -- Brake torque RL, in N-m.
  if wheelRLBrakingTorque == wheelRLBrakingTorque then
    brakeTorques[3] = wheelRLBrakingTorque
  end

  local wheelRRBrakingTorque = decodedMessageFromSimulink[9]          -- Brake torque RR, in N-m.
  if wheelRRBrakingTorque == wheelRRBrakingTorque then
    brakeTorques[4] = wheelRRBrakingTorque
  end

  local wheelFLPropulsionTorque = decodedMessageFromSimulink[10]       -- Propulsion torques FL, in N-m.
  if wheelFLPropulsionTorque == wheelFLPropulsionTorque then
    propulsionTorques[1] = wheelFLPropulsionTorque
  end

  local wheelFRPropulsionTorque = decodedMessageFromSimulink[11]      -- Propulsion torques FR, in N-m.
  if wheelFRPropulsionTorque == wheelFRPropulsionTorque then
    propulsionTorques[2] = wheelFRPropulsionTorque
  end

  local wheelRLPropulsionTorque = decodedMessageFromSimulink[12]      -- Propulsion torques RL, in N-m.
  if wheelRLPropulsionTorque == wheelRLPropulsionTorque then
    propulsionTorques[3] = wheelRLPropulsionTorque
  end

  local wheelRRPropulsionTorque = decodedMessageFromSimulink[13]      -- Propulsion torques RR, in N-m.
  if wheelRRPropulsionTorque == wheelRRPropulsionTorque then
    propulsionTorques[4] = wheelRRPropulsionTorque
  end

  local driveMode = decodedMessageFromSimulink[14]                    -- Drive mode [0.0 = pedals or 1.0 = torques].
  if driveMode == driveMode then
    if driveMode < 0.5 then
      isUsingTorques = false
    else
      isUsingTorques = true
    end
  end

  -- Bank B: Custom user values.
  local custom1 = decodedMessageFromSimulink[15]
  local custom2 = decodedMessageFromSimulink[16]
  local custom3 = decodedMessageFromSimulink[17]
  local custom4 = decodedMessageFromSimulink[18]
  local custom5 = decodedMessageFromSimulink[19]
  local custom6 = decodedMessageFromSimulink[20]
  local custom7 = decodedMessageFromSimulink[21]
  local custom8 = decodedMessageFromSimulink[22]
  local custom9 = decodedMessageFromSimulink[23]
  local custom10 = decodedMessageFromSimulink[24]
  local custom11 = decodedMessageFromSimulink[25]
  local custom12 = decodedMessageFromSimulink[26]
  local custom13 = decodedMessageFromSimulink[27]
  local custom14 = decodedMessageFromSimulink[28]
  local custom15 = decodedMessageFromSimulink[29]
  local custom16 = decodedMessageFromSimulink[30]
  local custom17 = decodedMessageFromSimulink[31]
  local custom18 = decodedMessageFromSimulink[32]
  local custom19 = decodedMessageFromSimulink[33]
  local custom20 = decodedMessageFromSimulink[34]
  local custom21 = decodedMessageFromSimulink[35]
  local custom22 = decodedMessageFromSimulink[36]
  local custom23 = decodedMessageFromSimulink[37]
  local custom24 = decodedMessageFromSimulink[38]
  local custom25 = decodedMessageFromSimulink[39]
  local custom26 = decodedMessageFromSimulink[40]
  local custom27 = decodedMessageFromSimulink[41]
  local custom28 = decodedMessageFromSimulink[42]
  local custom29 = decodedMessageFromSimulink[43]
  local custom30 = decodedMessageFromSimulink[44]
  local custom31 = decodedMessageFromSimulink[45]
  local custom32 = decodedMessageFromSimulink[46]
  local custom33 = decodedMessageFromSimulink[47]
  local custom34 = decodedMessageFromSimulink[48]
  local custom35 = decodedMessageFromSimulink[49]
  local custom36 = decodedMessageFromSimulink[50]
  local custom37 = decodedMessageFromSimulink[51]
  local custom38 = decodedMessageFromSimulink[52]
  local custom39 = decodedMessageFromSimulink[53]
  local custom40 = decodedMessageFromSimulink[54]
  local custom41 = decodedMessageFromSimulink[55]
  local custom42 = decodedMessageFromSimulink[56]
  local custom43 = decodedMessageFromSimulink[57]
  local custom44 = decodedMessageFromSimulink[58]
  local custom45 = decodedMessageFromSimulink[59]
  local custom46 = decodedMessageFromSimulink[60]
  local custom47 = decodedMessageFromSimulink[61]
  local custom48 = decodedMessageFromSimulink[62]
  local custom49 = decodedMessageFromSimulink[63]
  local custom50 = decodedMessageFromSimulink[64]

  return true
end

local function initialSetup(config)
  if config then
    -- get config overrides or keep defaults
    udpSendIP = config.udpSendIP or udpSendIP
    udpReceiveIP = config.udpReceiveIP or udpReceiveIP
    udpSendPort = config.udpSendPort or udpSendPort
    udpReceivePort = config.udpReceivePort or udpReceivePort
    simulinkTime = config.simulinkTime or simulinkTime
    pingTime = config.pingTime or pingTime
    debugFile = config.debugFile or debugFile
  end

  udpSendSocket = socket.udp()      -- The UDP socket for sending data from Lua to Simulink.
  udpRecvSocket = socket.udp()      -- The UDP socket for sending data from Simulink to Lua.
  -- Set up the UDP send socket.
  local result, error = udpSendSocket:setpeername(udpSendIP, udpSendPort)
  if error then
    log('E', logTag, 'UDP send socket could not be set up.')
  end

  -- Set up the UDP receive socket. We always start with a non-blocking socket (by using zero timeout).
  udpRecvSocket:settimeout(0.0)
  local result, error = udpRecvSocket:setsockname(udpReceiveIP, udpReceivePort)
  if error then
    log('E', logTag, 'UDP receive socket could not be set up.')
  end

  -- Set the drive mode to start with pedal input control.
  isUsingTorques = false

  -- Compute the coupling control parameters.
  sendSkips = math.ceil(simulinkTime / physicsDt) - 1
  unanswered = math.ceil(pingTime / simulinkTime)
  log('I', logTag, 'Ping time = ' .. pingTime .. ' , Simulink fixed step size = ' .. string.format("%.6g", (sendSkips + 1) * physicsDt))

  -- Create CSV file
  if debugFile then
    local sendIndexes = linspace(sendLength)
    local receiveIndexes = linspace(receiveLength)
    createCSV(sendIndexes, receiveIndexes)
  end
end

-- The controller initialisation function. This is called once when the controller is loaded, and sets up the UDP sockets for all future communication.
local function init(jbeamData)
  debugFile = jbeamData.debugFile
  if not jbeamData.loadedByExtension then
    -- we always want this the lifetime of the controller be handled by the extension,
    -- as that allows us to stop the coupling on Lua reload
    extensions.load('tech/vehicleSystemsCoupling')
    tech_vehicleSystemsCoupling.startCoupling({skipControllerLoad = true})
    initialSetup()
  end
end

local function updateWheelsIntermediate(dt)
  if isUsingTorques then
    for i = 1, wheels.wheelRotatorCount do
      local wheel = wheels.wheelRotators[wheels.wheelRotatorIDs[wheelsOrder[i]]]
      wheel.propulsionTorque = (propulsionTorques[i] or 0) * wheel.wheelDir
      wheel.desiredBrakingTorque = -(brakeTorques[i] or 0)
    end
  end
end

-- Update function. This is called on every physics step update, once the controller has been loaded.
local function update(dt)
  if debugFile then
    csvPhysicsSteps:add(os.clockhp())
  end
  -- Determine if we should skip sending in this cycle.
  if stepsSinceLastSend >= sendSkips then
    -- We will send in this cycle. Check if we have reached the window width. This is when we have received the same number of messages as we have sent out.

    if sendCtr - maxRecvId < unanswered then
      -- We have not yet reached the window width, so do a non-blocking receive and then send a new message.
      udpRecvSocket:settimeout(0.0)
      local rawMsgFromSimulink = receiveUDP()
      if rawMsgFromSimulink ~= nil then
        table.clear(decodedMessageFromSimulink)
        lpack.decodeDoubleArray(rawMsgFromSimulink, decodedMessageFromSimulink)
        handleMessageReceive()
      end
      createMessage()
      sendUDP()
    else
      -- We have reached the window width, so do a blocking receive and then send a new message.
      udpRecvSocket:settimeout(blockingTimeoutLength)
      local rawMsgFromSimulink = receiveUDP()
      if rawMsgFromSimulink ~= nil then
        table.clear(decodedMessageFromSimulink)
        lpack.decodeDoubleArray(rawMsgFromSimulink, decodedMessageFromSimulink)
        if handleMessageReceive() or maxRecvId == 0 then
          createMessage()
          sendUDP()
        end
      else
        sendCtr = 0
        maxRecvId = 0
      end
    end
  else
    -- We must skip sending in this cycle, so only perform a non-blocking receive.
    udpRecvSocket:settimeout(0.0)
    local rawMsgFromSimulink = receiveUDP()
    if rawMsgFromSimulink ~= nil then
      table.clear(decodedMessageFromSimulink)
      lpack.decodeDoubleArray(rawMsgFromSimulink, decodedMessageFromSimulink)
      handleMessageReceive()
    end
    stepsSinceLastSend = stepsSinceLastSend + 1
  end
end

local function stopCoupling()
  log('I', logTag, 'Stopped coupling.')
  if debugFile then
    saveCSV()
  end

  udpSendSocket:close()
  udpRecvSocket:close()
end

-- Public interface.
M.init = init
M.initialSetup = initialSetup
M.stopCoupling = stopCoupling

-- Functions triggered by updates.
M.updateWheelsIntermediate = updateWheelsIntermediate
M.update = update

return M