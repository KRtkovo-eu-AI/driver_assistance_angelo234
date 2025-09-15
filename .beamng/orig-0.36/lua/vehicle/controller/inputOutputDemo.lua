-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
--Keep as "auxiliary", there is also "main", but for logging purposes that would be the wrong choice
M.type = "auxiliary"
--This determines when the controller update methods are executed relative to other controllers.
--If you have no specific requirements to that order, it's not relevant
M.defaultOrder = 100000

local floor = math.floor
local random = math.random

local csvHeaders
local csvData
local timer

local udpSocket = socket.udp()

local function SaveLogToDisk()
  local fileName = "testLog.csv"
  local data = ""
  --iterating over all "lines" of saved data
  for _, line in ipairs(csvData) do
    --concat individual line entries
    local lineString = table.concat(line, ",")
    --concat line to existing data and start a new line
    data = data .. lineString .. "\r\n"
  end

  --write all csv contents to disk
  writeFile(fileName, data)
end

--A custom method that can be called internally or possibly externally if exposed.
local function saveLogToCSV()
  SaveLogToDisk()
end

local function LogInterestingData()
  --headers: "time", "throttle", "brake", "batteryCapacity", "motorPower"
  local time = floor(timer * 1000) / 1000 --Make sure time doesn't have dozen of digits
  local throttle = electrics.values.throttle --throttle input
  local brake = electrics.values.brake --brake input
  local batteryCapacity = 0
  local motorPower = 0

  --Get all electric motors (could be more than one)
  local electricMotors = powertrain.getDevicesByType("electricMotor")
  for _, motor in ipairs(electricMotors) do
    --Sum up output power of all motors ([W])
    motorPower = motorPower + motor.outputTorque1 * motor.outputAV1
  end

  --Get all energy storages
  local storages = energyStorage.getStorages()
  for _, storage in pairs(storages) do
    --filter for electric batteries
    if storage.type == "electricBattery" then
      --sum up stored energy ([J]) of all batteries
      batteryCapacity = batteryCapacity + storage.storedEnergy
    end
  end

  --create a table with all the relevant info from this logging step
  local currentData = {time, throttle, brake, batteryCapacity, motorPower}
  table.insert(csvData, currentData)
end

local function GenerateInputEvents(dt)
  local throttle = random()
  local brake = random()
  local steering = random() * 2 - 1

  --generate input events for various inputs
  input.event("throttle", throttle, FILTER_DIRECT)
  input.event("brake", brake, FILTER_DIRECT)
  input.event("steering", steering, FILTER_DIRECT)
end

local function CommunicateViaUDP(dt)
  --send some data via UDP
  udpSocket:send(jsonEncode({test = 123}))

  --receive data via UDP if available
  local data = udpSocket:receive()
  dump(data)
  --do something with the data
end

--This runs at a variable update rate matching the current FPS
--Example: The game runs at 60fps: Emthod is being called 60 times per second (and the dt variable is ~1/60 = 0.166667s)
--If you overload this method performance wise, fps will drop gracefully, so disk IO and other heavily unpredictable things are done/triggered from here (or user input)
local function updateGFX(dt)
  LogInterestingData()
  GenerateInputEvents()
  CommunicateViaUDP()
end

--This runs at a fixed 100hz. (derived from 2000hz physics step, see note about performance on update())
local function updateFixedStep(dt)
  --LogInterestingData()
end

--This runs at a fixed 2000hz. Note: Fixed 2000hz means that there is ever only 0.0005s or 0.5ms of computing time available for _all_ work.
--That includes all the raw physics, large parts of the powertrain, some parts of the safety and driving electronics, etc
--As soon as your computer takes more than the 0.5ms to process all these things, the game will not be able to compute things in time anymore
--and it will enter an automatic slow motion mode
--In practice this means that you should be very well aware of how performance sensitive your code is and avoid uunpredictable things
--like disk IO in anything that is/derives from the 2000hz update
local function update(dt)
  --increment timer with passed time since last call, depending on where this is called (update, updateFixedStep, updateGFX the resolution of time keeping changes)
  timer = timer + dt
  --LogInterestingData()
end

--This is being called once after the vehicle is created and then once everytime when the vehicle is reset
local function reset()
  csvData = {csvHeaders}
  timer = 0
end

--This is called once when a vehicle is created
local function init()
  csvHeaders = {"time", "throttle", "brake", "batteryCapacity", "motorPower"}
  csvData = {csvHeaders}
  timer = 0

  local peerIP = "127.0.0.1"
  local peerPort = 54812
  udpSocket:setpeername(peerIP, peerPort)
  udpSocket:settimeout(0.005)
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX
M.updateFixedStep = updateFixedStep
M.update = update

--If you want to expose your own methods to the outside (for example for calling upon a key press)
--you need to list them here in the following format.
--There's an external and an internal name, the external is what the outside world sees,
--the internal is what your method in this file needs to be called like. (Both names can be the same)
--If this controller code is active (ie. specified in jbeam) you can call this method like this:
--controller.getController("loggerTemplate").saveLogToCSV()
-- External name   Internal name
M.saveLogToCSV = saveLogToCSV

return M
