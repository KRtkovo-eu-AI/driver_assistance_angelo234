-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
M.type = "auxiliary"

-- Powertrain sensor core properties.
local sensorId  -- The unique Id number for powertrain sensor.
local GFXUpdateTime  -- The GFX step update time (ie how often readings data is available to the user).

local timeSinceLastPoll = 0.0 -- The time since this powertrain sensor was last polled (for graphics step).

-- Physics step parameters.
local physicsTimer  -- A timer used for the physics step, to check if an readings update is required.
local physicsUpdateTime  -- How often the physics should be updated, in seconds.

-- Powertrain readings data.
local readings = {} -- The table of raw sensor readings (since the last graphics step update).
local readingIndex = 1 -- The index in the raw readings array at which to place the next data.

-- Physics step update for this powertrain sensor instance.
local function update(dtSim)
  -- Cycle the physics update timer. If we are not ready for a physics step update, leave immediately.
  if physicsTimer < physicsUpdateTime then
    physicsTimer = physicsTimer + dtSim
    return
  end
  physicsTimer = physicsTimer - physicsUpdateTime

  -- Fetch the latest readings from the powertrain.
  local latestReading = {}
  for _, device in pairs(powertrain.getDevices()) do
    local deviceData = {inputAV = device.inputAV, gearRatio = device.gearRatio, isBroken = device.isBroken, mode = device.mode}
    if device.numberOfOutputPorts then
      for i = 1, device.numberOfOutputPorts, 1 do
        deviceData[device.outputTorqueNames[i]] = device[device.outputTorqueNames[i]]
        deviceData[device.outputAVNames[i]] = device[device.outputAVNames[i]]
      end
    else
      deviceData.outputTorque1 = device.outputTorque1
      deviceData.outputAV1 = device.outputAV1
      deviceData.outputTorque2 = device.outputTorque2
      deviceData.outputAV2 = device.outputAV2
    end
    if device.parent then
      deviceData.parentName = device.parent.name
      deviceData.parentOutputIndex = device.inputIndex
    end
    latestReading[device.name] = deviceData
    latestReading['time'] = obj:getSimTime()      -- Time-stamp the sample reading.
  end

  -- Store the latest readings for this powertrain sensor in the extension. This is used for sending back on the physics step.
  extensions.tech_powertrainSensor.cacheLatestReading(sensorId, latestReading)

  -- Add the data to the readings array, for later retrieval. This is used for sending back on the graphics step.
  readings[readingIndex] = latestReading
  readingIndex = readingIndex + 1
end

-- Initialises this powertrain sensor instance.
local function init(data)
  sensorId = data.sensorId
  GFXUpdateTime = data.GFXUpdateTime
  timeSinceLastPoll = 0.0
  readings = {}
  readingIndex = 1
  physicsTimer = 0.0
  physicsUpdateTime = data.physicsUpdateTime
end

local function reset()
  readings = {} -- empty the table of raw readings, because we have now collected them in the GFX step update.
  readingIndex = 1 -- and reset the index.
  timeSinceLastPoll = timeSinceLastPoll % math.max(GFXUpdateTime, 1e-30)
end

local function getSensorData()
  return {
    readings = readings,
    GFXUpdateTime = GFXUpdateTime,
    timeSinceLastPoll = timeSinceLastPoll
  }
end

local function incrementTimer(dtSim)
  timeSinceLastPoll = timeSinceLastPoll + dtSim
end

-- Public interface:
M.update = update
M.init = init
M.reset = reset
M.getSensorData = getSensorData
M.incrementTimer = incrementTimer

return M
