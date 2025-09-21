-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

-- Advanced IMU core properties.
local sensorId                                  -- The unique Id number for this Advanced IMU sensor.
local GFXUpdateTime                             -- The GFX step update time (ie how often readings data is available to the user).
local nodeIndex1, nodeIndex2, nodeIndex3        -- The indices of the three nodes which comprise the sensor attach triangle.
local b1, b2, b3                                -- The three barycentric coordinates of the sensor re the triangle.
local w1, w2, w3                                -- Positive-valued weights for the interpolation.
local signedProjDist                            -- The signed distance from the sensor triangle plane to the sensor.
local triangleSpaceForward, triangleSpaceUp     -- Cached forward/up vectors in the local triangle space.
local isVisualised = true                       -- A flag which indicates if this Advanced IMU sensor should be visualised
local isUsingGravity = false                    -- A flag which indicates if this Advanced IMU should include gravity in its computation.

local timeSinceLastPoll = 0.0                   -- The time since this Advanced IMU sensor was last polled (for graphics step).

-- Physics step parameters.
local physicsTimer                              -- A timer used for the physics step, to check if an readings update is required.
local physicsUpdateTime                         -- How often the physics should be updated, in seconds.
local physicsSmootherAccelX = nil               -- The chosen smoothers for the acceleration readings, in each dimension.
local physicsSmootherAccelY = nil
local physicsSmootherAccelZ = nil
local physicsSmootherGyroX = nil                -- The chosen smoothers for the gyroscopic readings, in each dimension.
local physicsSmootherGyroY = nil
local physicsSmootherGyroZ = nil

-- Advanced IMU readings data.
local smoothedAccelReading                      -- The smoothed acceleration reading.
local smoothedGyroReading                       -- The smoothed gyroscopic reading.
local readings = {}                             -- The table of raw sensor readings (since the last graphics step update).
local readingIndex = 1                          -- The index in the raw readings array at which to place the next data.
local latestReading = {
  time = 0.0,
  mass = 0.0,
  accRaw = { 0, 0, 0 },
  accSmooth = { 0, 0, 0 },
  angVel = { 0, 0, 0 },
  angVelSmooth = { 0, 0, 0 },
  angAccel = { 0, 0, 0 },
  pos = { 0, 0, 0 },
  dirX = { 0, 0, 0 },
  dirY = { 0, 0, 0 },
  dirZ = { 0, 0, 0 } }

-- Properties which are updated regularly.
local currentPos = vec3(0, 0, 0)
local currentDir = vec3(0, 0, 0)

-- Physics step update for this Advanced IMU sensor instance.
local function update(dtSim)
  -- Cycle the physics update timer. If we are not ready for a physics step update, leave immediately.
  if physicsTimer < physicsUpdateTime then
    physicsTimer = physicsTimer + dtSim
    return
  end
  physicsTimer = physicsTimer - physicsUpdateTime

  -- Compute the current position of the Advanced IMU sensor.
  local node1, node2, node3 = obj:getNodePosition(nodeIndex1), obj:getNodePosition(nodeIndex2), obj:getNodePosition(nodeIndex3)
  local edge1, edge2 = node2 - node1, node3 - node1
  local edge1Norm, edge2Norm = edge1:normalized(), edge2:normalized()
  local normal = edge1Norm:cross(edge2Norm):normalized()
  local projPos = node1 + b1 * edge2 + b2 * edge1                     -- The projection of the world-space position onto the triangle plane.
  currentPos = projPos + signedProjDist * normal                      -- The current world-space position of the sensor.

  -- Get the mass at each node.
  local m1 = obj:getNodeMass(nodeIndex1)
  local m2 = obj:getNodeMass(nodeIndex1)
  local m3 = obj:getNodeMass(nodeIndex1)

  -- Compute the acceleration vectors at each node, using Newton II [a := F / m].
  local a1 = obj:getNodeForceVector(nodeIndex1) / m1
  local a2 = obj:getNodeForceVector(nodeIndex2) / m2
  local a3 = obj:getNodeForceVector(nodeIndex3) / m3

  -- Get the velocity vector at each node.
  local v1 = obj:getNodeVelocityVector(nodeIndex1)
  local v2 = obj:getNodeVelocityVector(nodeIndex2)
  local v3 = obj:getNodeVelocityVector(nodeIndex3)

  -- Compute the rotational component of each nodal acceleration vector, by subtracting the translation component.
  local translation = (a1 + a2 + a3) / 3
  local aRot1, aRot2, aRot3 = a1 - translation, a2 - translation, a3 - translation

  -- Compute the barycenter of the attachment triangle.
  local baryCenter = (node1 + node2 + node3) / 3

  -- Compute the curl and divergence at the projected point (on triangle plane).
  local r1, r2, r3 = node1 - baryCenter, node2 - baryCenter, node3 - baryCenter     -- vectors from the barycenter to each node.
  local curl = r1:cross(aRot1) * w1 + r2:cross(aRot2) * w2 + r3:cross(aRot3) * w3
  local divergence = r1:dot(aRot1) * w1 + r2:dot(aRot2) * w2 + r3:dot(aRot3) * w3

  -- Compute the total acceleration vector at the sensor position, and also the angular velocity/acceleration terms.
  local r = currentPos - baryCenter
  local invDenom = 1.0 / (r1:squaredLength() * w1 + r2:squaredLength() * w2 + r3:squaredLength() * w3 + 1e-30)
  local totalAccel = translation + (curl:cross(r) + divergence * r) * invDenom

  -- Compute the angular velocity and angular acceleration.
  local vCenter = (v1 + v2 + v3) / 3
  local angVel = (r1:cross(v1 - vCenter) * w1 + r2:cross(v2 - vCenter) * w2 + r3:cross(v3 - vCenter) * w3) * invDenom
  local angAccel = curl * invDenom

  -- Add on the acceleration due to gravity (as a vector), if requested.
  if isUsingGravity then
    totalAccel = totalAccel + obj:getGravityVector()
  end

  -- Convert the fixed triangle-space coordinate system to world space (the former was pre-computed when the sensor was created).
  local forward = triangleSpaceForward
  local up = triangleSpaceUp
  local triangleThird = edge1Norm:cross(normal):normalized()
  currentDir = (edge1Norm * forward.x + normal * forward.y + triangleThird * forward.z):normalized()
  local worldUp = (edge1Norm * up.x + normal * up.y + triangleThird * up.z):normalized()
  local worldThird = currentDir:cross(worldUp):normalized()

  -- Resolve the acceleration vector to the world-space sensor coordinate system.
  local accel = vec3(totalAccel:dot(currentDir), totalAccel:dot(worldUp), totalAccel:dot(worldThird))

  -- Compute the local mass using interpolation from the three node mass values.
  local interpolatedMass = m1 * w1 + m2 * w2 + m3 * w3

  -- Smooth the acceleration/gyroscopic vectors (based on their previous values), and store them for later retrieval during the GFX step.
  smoothedAccelReading = vec3(physicsSmootherAccelX:get(accel.x), physicsSmootherAccelY:get(accel.y), physicsSmootherAccelZ:get(accel.z))
  smoothedGyroReading = vec3(physicsSmootherGyroX:get(angVel.x), physicsSmootherGyroY:get(angVel.y), physicsSmootherGyroZ:get(angVel.z))

  -- Gather the latest reading data.
  latestReading = {
    time = obj:getSimTime(),
    mass = interpolatedMass,
    accRaw = accel:toTable(),
    accSmooth = smoothedAccelReading:toTable(),
    angVel = angVel:toTable(),
    angVelSmooth = smoothedGyroReading:toTable(),
    angAccel = angAccel:toTable(),
    pos = (currentPos + obj:getPosition()):toTable(),
    dirX = currentDir:toTable(),
    dirY = worldUp:toTable(),
    dirZ = worldThird:toTable() }

  -- Store the latest readings for this advanced IMU sensor in the extension. This is used for sending back on the physics step.
  extensions.tech_advancedIMU.cacheLatestReading(sensorId, latestReading)

  -- Add the data to the readings array, for later retrieval. This is used for sending back on the graphics step.
  readings[readingIndex] = latestReading
  readingIndex = readingIndex + 1
end

-- Initialises this Advanced IMU sensor instance.
local function init(data)
    sensorId = data.sensorId
    GFXUpdateTime = data.GFXUpdateTime
    nodeIndex1 = data.nodeIndex1
    nodeIndex2 = data.nodeIndex2
    nodeIndex3 = data.nodeIndex3
    b1 = data.u
    b2 = data.v
    b3 = 1.0 - b1 - b2
    w1 = math.max(0, b1)
    w2 = math.max(0, b2)
    w3 = math.max(0, b3)
    signedProjDist = data.signedProjDist
    triangleSpaceForward = data.triangleSpaceForward
    triangleSpaceUp = data.triangleSpaceUp
    isVisualised = data.isVisualised
    isUsingGravity = data.isUsingGravity
    timeSinceLastPoll = 0.0
    readings = {}
    smoothedAccelReading = 0.0
    smoothedGyroReading = 0.0
    readingIndex = 1
    physicsTimer = 0.0
    physicsUpdateTime = data.physicsUpdateTime    -- NOTE: we use this period as dt in the smoothers.

    -- If the user has provided a cutoff frequency instead of a window width, compute the related window width from that now.
    if data.accelCutoffFrequency ~= nil then
      local piDtF = math.pi * physicsUpdateTime * data.accelCutoffFrequency
      data.accelWindowWidth = math.max(1.0, (2 * piDtF + 1.0) / piDtF)
    end
    if data.gyroCutoffFrequency ~= nil then
      local piDtF = math.pi * physicsUpdateTime * data.gyroCutoffFrequency
      data.gyroWindowWidth = math.max(1.0, (2 * piDtF + 1.0) / piDtF)
    end

    -- Initialise the smoothers.
    physicsSmootherAccelX = newExponentialSmoothing(data.accelWindowWidth, 0.0, physicsUpdateTime)
    physicsSmootherAccelY = newExponentialSmoothing(data.accelWindowWidth, 0.0, physicsUpdateTime)
    physicsSmootherAccelZ = newExponentialSmoothing(data.accelWindowWidth, 0.0, physicsUpdateTime)
    physicsSmootherGyroX = newExponentialSmoothing(data.gyroWindowWidth, 0.0, physicsUpdateTime)
    physicsSmootherGyroY = newExponentialSmoothing(data.gyroWindowWidth, 0.0, physicsUpdateTime)
    physicsSmootherGyroZ = newExponentialSmoothing(data.gyroWindowWidth, 0.0, physicsUpdateTime)
end

local function reset()
  readings = {}             -- empty the table of raw readings, because we have now collected them in the GFX step update.
  readingIndex = 1          -- and reset the index.
  timeSinceLastPoll = timeSinceLastPoll % math.max(GFXUpdateTime, 1e-30)
end

local function getSensorData()
  return {
    isVisualised = isVisualised,
    isUsingGravity = isUsingGravity,
    timeSinceLastPoll = timeSinceLastPoll,
    GFXUpdateTime = GFXUpdateTime,
    currentPos = currentPos + obj:getPosition(),  -- convert from vehicle space (where computations are done) to world space.
    currentDir = currentDir,
    rawReadings = readings }
end

local function getLatest() return latestReading end

local function setIsUsingGravity(value) isUsingGravity = value end

local function setIsVisualised(value) isVisualised = value end

local function incrementTimer(dtSim) timeSinceLastPoll = timeSinceLastPoll + dtSim end


-- Public interface:
M.update                                                  = update
M.init                                                    = init
M.reset                                                   = reset
M.getSensorData                                           = getSensorData
M.getLatest                                               = getLatest
M.setIsUsingGravity                                       = setIsUsingGravity
M.setIsVisualised                                         = setIsVisualised
M.incrementTimer                                          = incrementTimer

return M