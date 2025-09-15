-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local cos = math.cos
local max = math.max

-- GPS core properties.
local sensorId                                  -- The unique Id number for this GPS sensor.
local GFXUpdateTime                             -- The GFX step update time (ie how often readings data is available to the user).
local nodeIndex1, nodeIndex2, nodeIndex3        -- The indices of the three nodes which comprise the sensor attach triangle.
local b1, b2                                    -- The barycentric coordinates of the sensor re the triangle.
local signedProjDist                            -- The signed distance from the sensor triangle plane to the sensor.
local isVisualised = true                       -- A flag which indicates if this GPS sensor should be visualised
local refLon, refLat = 0.0, 0.0                 -- The (lon, latt) coordinates for the origin of the map.

local timeSinceLastPoll = 0.0                   -- The time since this GPS sensor was last polled (for graphics step).

-- Physics step parameters.
local physicsTimer                              -- A timer used for the physics step, to check if an readings update is required.
local physicsUpdateTime                         -- How often the physics should be updated, in seconds.

-- GPS readings data.
local readings = {}                             -- The table of raw sensor readings (since the last graphics step update).
local readingIndex = 1                          -- The index in the raw readings array at which to place the next data.
local latestReading = {
  time = 0.0,
  x = 0.0,
  y = 0.0,
  lon = 0.0,
  lat = 0.0 }

-- Properties which are updated regularly.
local pos = vec3(0, 0)

-- Converts an (x, y) Euclidean grid position in metres, to a (lon, latt) spherical surface position, using a reference point in (latt, lon).
local function xY2LonLat(x, y, refLon, refLat)
  return (x / (cos(refLat * 0.01745329251) * 111319.88888888888889)) + refLon, (y * 8.998200359928014399819e-6) + refLat
end

-- Physics step update for this GPS sensor instance.
local function update(dtSim)
  -- Cycle the physics update timer. If we are not ready for a physics step update, leave immediately.
  if physicsTimer < physicsUpdateTime then
    physicsTimer = physicsTimer + dtSim
    return
  end
  physicsTimer = physicsTimer - physicsUpdateTime

  -- Compute the current world-space position of the GPS sensor.
  local node1, node2, node3 = obj:getNodePosition(nodeIndex1), obj:getNodePosition(nodeIndex2), obj:getNodePosition(nodeIndex3)
  local edge1, edge2 = node2 - node1, node3 - node1
  local edge1Norm, edge2Norm = edge1:normalized(), edge2:normalized()
  local normal = edge1Norm:cross(edge2Norm):normalized()
  local projPos = node1 + b1 * edge2 + b2 * edge1                     -- The projection of the world-space position onto the triangle plane.
  pos = obj:getPosition() + projPos + signedProjDist * normal         -- The current world-space position of the sensor.

  -- Convert the (x, y) coordinates to (lon, latt).
  local lon, lat = xY2LonLat(pos.x, pos.y, refLon, refLat)

  -- Gather the latest reading data.
  latestReading = { time = obj:getSimTime(), x = pos.x, y = pos.y, lon = lon, lat = lat }

  -- Store the latest readings for this GPS sensor in the extension. This is used for sending back on the physics step.
  -- NOTE: this is for when polling directly through the vlua - python socket, so we get the latest reading.
  extensions.tech_GPS.cacheLatestReading(sensorId, latestReading)

  -- Add the data to the readings array, for later retrieval. This is used for sending back on the graphics step.
  readings[readingIndex] = latestReading
  readingIndex = readingIndex + 1
end

-- Initialises this GPS sensor instance.
local function init(data)
    sensorId = data.sensorId
    GFXUpdateTime = data.GFXUpdateTime
    nodeIndex1 = data.nodeIndex1
    nodeIndex2 = data.nodeIndex2
    nodeIndex3 = data.nodeIndex3
    b1 = data.u
    b2 = data.v
    signedProjDist = data.signedProjDist
    refLon = data.refLon
    refLat = data.refLat
    isVisualised = data.isVisualised
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
    isVisualised = isVisualised,
    timeSinceLastPoll = timeSinceLastPoll,
    GFXUpdateTime = GFXUpdateTime,
    pos = pos,
    rawReadings = readings }
end

local function getLatest() return latestReading end

local function setIsVisualised(value) isVisualised = value end

local function incrementTimer(dtSim) timeSinceLastPoll = timeSinceLastPoll + dtSim end


-- Public interface:
M.update                                                  = update
M.init                                                    = init
M.reset                                                   = reset
M.getSensorData                                           = getSensorData
M.getLatest                                               = getLatest
M.setIsVisualised                                         = setIsVisualised
M.incrementTimer                                          = incrementTimer

return M