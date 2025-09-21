-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local GPSs = {}                 -- The collection of active GPS sensors.
local latestReadings = {}       -- The collection of latest readings for each GPS sensor

-- Send the GPS readings to ge lua.
local function updateGPSGFXStep(dtSim, sensorId, isAdHocRequest, adHocRequestId)

  -- Get the latest GPS data from the controller.
  local controller = GPSs[sensorId].controller
  local data = controller.getSensorData()

  -- Draw this GPS sensor, if requested.
  if data.isVisualised == true then
    obj.debugDrawProxy:drawSphere(0.05, data.pos, color(0, 255, 0, 255))
  end

  -- If we are not ready to poll this GPS, then increment the timer and leave.
  if not isAdHocRequest and data.timeSinceLastPoll < data.GFXUpdateTime then
    controller.incrementTimer(dtSim)
    return
  end

  -- Send the latest sensor readings from vlua to ge lua.
  local rawReadingsData = { sensorId = sensorId, reading = data.rawReadings }
  obj:queueGameEngineLua(string.format("tech_sensors.updateGPSLastReadings(%q)", lpack.encode(rawReadingsData)))

  -- If this request is ad-hoc, then we also update the ad-hoc request in ge lua, so that this can be collected later by the user.
  if isAdHocRequest then
    local adHocData = { requestId = adHocRequestId, reading = data.rawReadings }
    obj:queueGameEngineLua(string.format("tech_sensors.updateGPSAdHocRequest(%q)", lpack.encode(adHocData)))
  end

  -- Reset the raw readings table, now that the GFX update step has been performed.
  controller.reset()
end

local function create(data)

  -- Create a controller instance for this GPS sensor.
  local decodedData = lpack.decode(data)
  local controllerData = {
    sensorId = decodedData.sensorId,
    GFXUpdateTime = decodedData.GFXUpdateTime,
    physicsUpdateTime = decodedData.physicsUpdateTime,
    nodeIndex1 = decodedData.nodeIndex1,
    nodeIndex2 = decodedData.nodeIndex2,
    nodeIndex3 = decodedData.nodeIndex3,
    u = decodedData.u,
    v = decodedData.v,
    refLon = decodedData.refLon,
    refLat = decodedData.refLat,
    signedProjDist = decodedData.signedProjDist,
    isVisualised = decodedData.isVisualised }

  GPSs[decodedData.sensorId] = {
    data = controllerData,
    controller = controller.loadControllerExternal('tech/GPS', 'GPS' .. decodedData.sensorId, controllerData) }
end

local function remove(sensorId)
  controller.unloadControllerExternal('GPS' .. sensorId)
  GPSs[sensorId] = nil
end

local function setUpdateTime(sensorId, GFXUpdateTime) GPSs[sensorId].GFXUpdateTime = GFXUpdateTime end

local function setIsVisualised(data)
  local decodedData = lpack.decode(data)
  GPSs[decodedData.sensorId].controller.setIsVisualised(decodedData.isVisualised)
end

local function adHocRequest(sensorId, requestId) updateGPSGFXStep(0.0, sensorId, true, requestId) end

local function cacheLatestReading(sensorId, latestReading)
  if sensorId ~= nil then
    latestReadings[sensorId] = latestReading
  end
end

local function getGPSReading(sensorId) return latestReadings[sensorId] end

local function getLatest(sensorId) return GPSs[sensorId].controller.getLatest() end

local function updateGFX(dtSim)
  for sensorId, _ in pairs(GPSs) do
    updateGPSGFXStep(dtSim, sensorId, false, nil)
  end
end

local function onVehicleDestroyed(vid)
  for sensorId, _ in pairs(GPSs) do
    if vid == objectId then
      remove(sensorId)
      GPSs[sensorId] = nil
    end
  end
end


-- Public interface:
M.create                                                  = create
M.remove                                                  = remove
M.adHocRequest                                            = adHocRequest
M.cacheLatestReading                                      = cacheLatestReading
M.getGPSReading                                           = getGPSReading
M.getLatest                                               = getLatest
M.setUpdateTime                                           = setUpdateTime
M.setIsVisualised                                         = setIsVisualised
M.updateGFX                                               = updateGFX
M.onVehicleDestroyed                                      = onVehicleDestroyed

return M