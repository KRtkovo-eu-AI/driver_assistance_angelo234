-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

-- Core properties.
local sensorId                                  -- The unique Id number for this sensor.
local GFXUpdateTime                             -- The GFX step update time (ie how often readings data is available to the sensor user).

local timeSinceLastPoll = 0.0                   -- The time since this sensor was last polled (for graphics step).

-- Physics step parameters.
local physicsTimer                              -- A timer used for the physics step, to check if an readings update is required.
local physicsUpdateTime                         -- How often the physics should be updated, in seconds.

local readings = {}                             -- The table of raw sensor readings (since the last graphics step update).
local readingIndex = 1                          -- The index in the raw readings array at which to place the next data.

-- Properties which are updated regularly.
local currentPos = vec3(0, 0, 0)
local currentDir = vec3(0, 0, 0)

-- Pre-initialize some quantities for efficiency.
local nodes = {}
local pos, force, vel

-- Physics step update for this sensor instance.
local function update(dtSim)
  -- Cycle the physics update timer. If we are not ready for a physics step update, leave immediately.
  if physicsTimer < physicsUpdateTime then
    physicsTimer = physicsTimer + dtSim
    return
  end
  physicsTimer = physicsTimer - physicsUpdateTime

  nodes = {}
  local nodesCount = obj:getNodeCount()
  for i=0, nodesCount do
    pos = obj:getNodePosition(i)
    force = obj:getNodeForceVector(i)
    vel = obj:getNodeVelocityVector(i)
    nodes[i] = {
        posX = pos.x, posY = pos.y, posZ = pos.z,
        forceX = force.x, forceY = force.y, forceZ = force.z,
        velX = vel.x, velY = vel.y, velZ = vel.z,
        mass = obj:getNodeMass(i) }
  end

  -- Timestamp the latest reading data.
  local latestReading = { time = obj:getSimTime(), nodes = nodes }

  -- Store the latest readings in the extension. This is used for sending back on the physics step.
  extensions.tech_mesh.cacheLatestReading(sensorId, latestReading)

  -- Add the data to the readings array, for later retrieval. This is used for sending back on the graphics step.
  readings[readingIndex] = latestReading
  readingIndex = readingIndex + 1
end

-- Initialises this sensor instance.
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
  readings = {}             -- empty the table of raw readings, because we have now collected them in the GFX step update.
  readingIndex = 1          -- and reset the index.
  timeSinceLastPoll = timeSinceLastPoll % math.max(GFXUpdateTime, 1e-30)
end

local function getSensorData()
  return {
    timeSinceLastPoll = timeSinceLastPoll,
    GFXUpdateTime = GFXUpdateTime,
    rawReadings = readings }
end

local function incrementTimer(dtSim)
  timeSinceLastPoll = timeSinceLastPoll + dtSim
end

-- Public interface:
M.update                                    = update
M.init                                      = init
M.reset                                     = reset
M.getSensorData                             = getSensorData
M.incrementTimer                            = incrementTimer

return M