-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local partOrigins = {}

local meshes = {}                               -- The collection of active mesh sensors.
local latestReadings = {}                       -- The collection of latest readings for each mesh sensor

-- Core properties.
local sensorId                                  -- The unique Id number for this sensor.
local GFXUpdateTime                             -- The GFX step update time (ie how often readings data is available to the user).
local timeSinceLastPoll = 0.0                   -- The time since this sensor was last polled (for graphics step).
local nodes, beams = {}, {}
local latestReading = {}

local function updateMesh(dtSim, sensorId, isAdHocRequest, adHocRequestId)

  -- If we are not ready to poll, then increment the timer and leave the callback.
  if not isAdHocRequest and timeSinceLastPoll < GFXUpdateTime then
    timeSinceLastPoll = timeSinceLastPoll + dtSim
    return
  end

  -- Compute the latest node data.
  local nodesCount = obj:getNodeCount()
  for i=0, nodesCount do
    -- pos, force, vel, mass, partOrigin
    nodes[i] = nodes[i] or { vec3(), vec3(), vec3(), 0.0, "no part origin" }
    nodes[i][1]:set(obj:getNodePositionRelativeXYZ(i))
    nodes[i][2]:set(obj:getNodeForceVectorXYZ(i))
    nodes[i][3]:set(obj:getNodeVelocityVector(i))
    nodes[i][4] = obj:getNodeMass(i)
    nodes[i][5] = partOrigins[i]
  end

  -- Compute the latest beam data.
  local beamCount = obj:getBeamCount()
  for i=0, beamCount do
    beams[i] = obj:getBeamStress(i)
  end

  -- Update the latest reading data.
  table.clear(latestReading)
  latestReading.time = obj:getSimTime()
  latestReading.nodes = nodes
  latestReading.beams = beams

  -- Update the collection (readings for all sensors) with latest data.
  latestReadings[sensorId] = latestReading

  -- If this request is ad-hoc, then we also update the ad-hoc request in ge lua, so that this can be collected later by the user.
  if isAdHocRequest then
    local adHocData = { requestId = adHocRequestId, reading = latestReading }
    obj:queueGameEngineLua(string.format("tech_sensors.updateMeshAdHocRequest(%q)", lpack.encode(adHocData)))
  end

  timeSinceLastPoll = 0.0
end

local function create(data)
  local decodedData = lpack.decode(data)
  sensorId = decodedData.sensorId
  GFXUpdateTime = decodedData.GFXUpdateTime
  meshes[sensorId] = true

  -- Populate the part origins table (these are sent to BeamNGpy so we can isolate different parts of the mesh there).
  local nodes = v.data.nodes
  for i = 0, tableSizeC(nodes) - 1 do
    partOrigins[i] = nodes[i].partOrigin
  end
end

local function remove(sensorId)
  meshes[sensorId] = nil
end

local function setUpdateTime(sensorId, GFXUpdateTime)
  meshes[sensorId].GFXUpdateTime = GFXUpdateTime
end

local function adHocRequest(sensorId, requestId)
  updateMesh(0.0, sensorId, true, requestId)
end

local function getMeshReading(sensorId)
  return latestReadings[sensorId]
end

local function onVehicleDestroyed(vid)
  for sensorId, _ in pairs(meshes) do
    if vid == objectId then
      remove(sensorId)
      meshes[sensorId] = nil
    end
  end
end

local function updateGFX(dtSim)
  for sensorId, _ in pairs(meshes) do
    updateMesh(dtSim, sensorId, false, nil)
  end
end

-- Public interface:

-- Core API functions.
M.create                                    = create
M.remove                                    = remove
M.adHocRequest                              = adHocRequest
M.getMeshReading                            = getMeshReading

-- Property setters.
M.setUpdateTime                             = setUpdateTime

-- Functions triggered by hooks.
M.updateGFX                                 = updateGFX
M.onVehicleDestroyed                        = onVehicleDestroyed

return M