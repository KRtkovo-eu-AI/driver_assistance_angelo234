-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Control parameters.
local maxVehicleRangeSq = 22500.0                                          -- The maximum squared distance from the player vehicle, at which other vehicles can be detected.

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

local max, min, abs, sqrt, acos = math.max, math.min, math.abs, math.sqrt, math.acos

-- Module state.
local sensorId                                                            -- The unique Id number for ideal RADAR sensor.
local GFXUpdateTime                                                       -- The GFX step update time (ie how often readings data is available to the user).
local timeSinceLastPoll = 0.0                                             -- The time since this ideal RADAR sensor was last polled (for graphics step).
local physicsTimer                                                        -- A timer used for the physics step, to check if an readings update is required.
local physicsUpdateTime                                                   -- How often the physics should be updated, in seconds.
local readings, readingIndex = {}, 1                                      -- Container and counter to store the raw sensor readings (since the last graphics step update).

-- Player vehicle state.
local pos = vec3(0, 0)                                                    -- The player vehicle position.
local fwd, right = vec3(0, 0), vec3(0, 0)                                 -- The player vehicle orthogonal frame.
local vehFront = vec3(0, 0)                                               -- The player vehicle front/rear bumper midpoint positions.
local vel, acc = vec3(0, 0), vec3(0, 0)                                   -- The player vehicle velocity and acceleration vectors.
local lastVelPlayer = vec3(0, 0)                                          -- An initial starting value for the player vehicle velocity.
local lastDt = 0.0                                                        -- The previous time step size (used for velocity and acceleration computations).
local lastPos, lastVel = {}, {}                                           -- Tables to store the last-known position and velocity data, for all other vehicles in the simulator.

local nullReading = {
    vehicleID = 0, width = 0, length = 0,
    positionB = { x = 0, y = 0, z = 0 },
    distToPlayerVehicleSq = 0, relDistX = 0, relDistY = 0,
    velBB = { x = 0, y = 0, z = 0 }, acc = { x = 0, y = 0, z = 0 },
    relVelX = 0, relVelY = 0, relAccX = 0, relAccY = 0 }

local latestReading = {
  closestVehicles1 = nullReading,
  closestVehicles2 = nullReading,
  closestVehicles3 = nullReading,
  closestVehicles4 = nullReading }

-- Projects a vector onto another vector.
local function project(a, b) return (a:dot(b) / b:dot(b)) * b end

-- Computes the relative quantity (difference) between two vectors in the direction of a given axis.
local function getRelativeQuantity(v1, v2, axis) return project(v2, axis):length() - project(v1, axis):length() end

-- Sorts a table by instances' squared distance value.
local function getKeysSortedByDistanceSq(tbl, sortFunction)
  local keys = {}
  for key in pairs(tbl) do
    table.insert(keys, key)
  end
  table.sort(keys, function(a, b)
    return sortFunction(tbl[a], tbl[b])
  end)
  return keys
end

-- A sort function, used to sort vehicles by squared distance to player vehicle.
local function sortAscending(a, b) return a.distToPlayerVehicleSq < b.distToPlayerVehicleSq end

local function init(data)
  sensorId = data.sensorId
  GFXUpdateTime = data.GFXUpdateTime
  timeSinceLastPoll = 0.0
  readings, readingIndex = {}, 1
  physicsTimer = 0.0
  physicsUpdateTime = data.physicsUpdateTime
end

local function reset()
  readings, readingIndex = {}, 1
  timeSinceLastPoll = timeSinceLastPoll % math.max(GFXUpdateTime, 1e-30)
end

local function getSensorData() return { readings = readings, GFXUpdateTime = GFXUpdateTime, timeSinceLastPoll = timeSinceLastPoll } end

local function getLatest() return latestReading end

local function incrementTimer(dtSim) timeSinceLastPoll = timeSinceLastPoll + dtSim end

-- The ideal RADAR sensor physics step update callback.
local function update(dtSim)

  -- Cycle the physics update timer. If we are not ready for a physics step update, leave immediately.
  if physicsTimer < physicsUpdateTime then
    physicsTimer = physicsTimer + dtSim
    return
  end
  physicsTimer = physicsTimer - physicsUpdateTime

  -- Update the player vehicle properties.
  pos = obj:getPosition()
  vel = obj:getVelocity()
  fwd = obj:getForwardVector():normalized()
  right = obj:getDirectionVectorRight():normalized()
  vehFront = obj:getFrontPosition()
  local playerPosToFront = vehFront - pos
  local lastDtInv = 1.0 / max(1e-12, lastDt)
  acc = (vel - lastVelPlayer) * lastDtInv                                                                         -- Use FD once to get acceleration.
  lastVelPlayer = vel

  -- Compute the relevant properties for the other vehicles.
  local vehicles, ctr = {}, 1
  for k, _ in pairs(mapmgr.getObjects()) do
    if k ~= objectId then

      -- Compute the position, velocity and acceleration of this other vehicle.
      local posB, velB, accB = obj:getObjectCenterPosition(k), vec3(0, 0, 0), vec3(0, 0, 0)
      if lastPos[k] ~= nil then
        velB = (posB - lastPos[k]) * lastDtInv                                                                    -- Use FD once to get velocity.
      end
      if lastVel[k] ~= nil then
        accB = (velB - lastVel[k]) * lastDtInv                                                                    -- Use FD twice to get acceleration.
      end
      lastPos[k], lastVel[k] = posB, velB

      -- Store the data of all other vehicles which satisfy the following conditions:
      -- i) within a certain range of the player vehicle.
      -- ii) facing the same direction as the player vehicle.
      -- iii) in front of the player vehicle.
      local fwdB = obj:getObjectDirectionVector(k)
      fwdB:normalize()
      local distantPoint = pos + (1e12 * fwd)                                                                     -- A point on the player vehicle forward vector, far in the distance.
      if fwd:dot(fwdB) > 0.0 and (distantPoint - pos):lenSquared() > (distantPoint - posB):lenSquared() then      -- Test that other vehicle is in front hemisphere and has same dir.
        local distToPlayerVehicleSq = (pos - posB):lenSquared()                                                   -- The squared distance between the player and other vehicle.
        if distToPlayerVehicleSq < maxVehicleRangeSq then                                                         -- Only consider vehicles which are within the set range.
          local upB = obj:getObjectDirectionVectorUp(k)                                                           -- The other vehicle's frame.
          upB:normalize()
          local widthB, lengthB = obj:getObjectInitialWidth(k), obj:getObjectInitialLength(k)                     -- The other vehicle's dimensions.
          local frontB = obj:getObjectFrontPosition(k)                                                            -- The other vehicle's front/rear bumper midpoint positions.
          local rearB = frontB - (fwdB * lengthB)
          local playerPosToBRear = rearB - pos                                                                    -- The vector from the player position to the other vehicle rear.
          local relDistX = getRelativeQuantity(playerPosToFront, playerPosToBRear, fwd)                           -- The relative distance to the player vehicle front position.
          local relDistY = getRelativeQuantity(playerPosToFront, playerPosToBRear, right)
          local relVelX, relVelY = getRelativeQuantity(vel, velB, fwd), getRelativeQuantity(vel, velB, right)     -- The relative velocity wrt the player vehicle frame.
          local relAccX, relAccY = getRelativeQuantity(acc, accB, fwd), getRelativeQuantity(acc, accB, right)     -- The relative acceleration wrt the player vehicle frame.
          vehicles[ctr] = {
            vehicleID = k,
            width = widthB, length = lengthB,
            positionB = posB,
            distToPlayerVehicleSq = distToPlayerVehicleSq or 0.0,
            relDistX = relDistX, relDistY = relDistY,
            velBB = velB,
            relVelX = relVelX, relVelY = relVelY,
            acc = accB,
            relAccX = relAccX, relAccY = relAccY }
          ctr = ctr + 1
        end
      end
    end
  end

  -- Sort the candidate vehicles by their squared distance to the player vehicle, ascending.
  local vClosest1, vClosest2, vClosest3, vClosest4 = nullReading, nullReading, nullReading, nullReading
  local sortMap = getKeysSortedByDistanceSq(vehicles, sortAscending)
  if sortMap[1] ~= nil then
    vClosest1 = vehicles[sortMap[1]]
  end
  if sortMap[2] ~= nil then
    vClosest2 = vehicles[sortMap[2]]
  end
  if sortMap[3] ~= nil then
    vClosest3 = vehicles[sortMap[3]]
  end
  if sortMap[4] ~= nil then
    vClosest4 = vehicles[sortMap[4]]
  end

  -- Populate the latest reading table, and include a timestamp.
  latestReading = {
    time = obj:getSimTime(),
    closestVehicles1 = vClosest1,
    closestVehicles2 = vClosest2,
    closestVehicles3 = vClosest3,
    closestVehicles4 = vClosest4 }

  -- Store the latest readings for this ideal RADAR sensor in the extension. This is used for sending back on the physics step.
  extensions.tech_idealRADARSensor.cacheLatestReading(sensorId, latestReading)

  -- Add the data to the readings array, for later retrieval. This is used for sending back on the graphics step.
  readings[readingIndex] = latestReading
  readingIndex = readingIndex + 1

  lastDt = dtSim
end


-- Public interface:
M.init =                                                  init
M.reset =                                                 reset
M.getSensorData =                                         getSensorData
M.getLatest =                                             getLatest
M.incrementTimer =                                        incrementTimer
M.update =                                                update

return M