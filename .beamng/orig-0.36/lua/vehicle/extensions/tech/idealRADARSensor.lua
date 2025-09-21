-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}


local idealRADARs = {}          -- The collection of active ideal radar sensors.
local latestReadings = {}       -- The collection of latest readings for each ideal radar sensor


-- Send the ideal RADAR readings to ge lua.
local function updateIdealRADARsGFXStep(dtSim, sensorId, isAdHocRequest, adHocRequestId)

  -- Get the latest ideal RADAR data from the controller.
  local controller = idealRADARs[sensorId].controller
  local data = controller.getSensorData()
  -- If we are not ready to poll this ideal RADAR sensor, then increment the timer and leave.
  if not isAdHocRequest and data.timeSinceLastPoll < data.GFXUpdateTime then
    controller.incrementTimer(dtSim)
    return
  end

  -- Send the latest sensor readings from vlua to ge lua.
  local readingsData = { sensorId = sensorId, reading = data.readings }

  obj:queueGameEngineLua(string.format("tech_sensors.updateIdealRADARLastReadings(%q)", lpack.encode(readingsData)))

  -- If this request is ad-hoc, then we also update the ad-hoc request in ge lua, so that this can be collected later by the user.
  if isAdHocRequest then
    local adHocData = { requestId = adHocRequestId, reading = data.readings }
    obj:queueGameEngineLua(string.format("tech_sensors.updateIdealRADARAdHocRequest(%q)", lpack.encode(adHocData)))
  end

  -- Reset the raw readings table, now that the GFX update step has been performed.
  controller.reset()
end

local function create(data)

  -- Create a controller instance for this ideal RADAR sensor.
  local decodedData = lpack.decode(data)
  local controllerData = {
    sensorId = decodedData.sensorId,
    GFXUpdateTime = decodedData.GFXUpdateTime,
    physicsUpdateTime = decodedData.physicsUpdateTime,
    isSendImmediately = decodedData.isSendImmediately }

  idealRADARs[decodedData.sensorId] = {
    data = controllerData,
    controller = controller.loadControllerExternal('tech/idealRADARSensor', 'idealRADARSensor' .. decodedData.sensorId, controllerData) }
end

local function remove(sensorId)
  controller.unloadControllerExternal('idealRADARSensor' .. sensorId)
  idealRADARs[sensorId] = nil
end

local function setUpdateTime(sensorId, GFXUpdateTime) idealRADARs[sensorId].GFXUpdateTime = GFXUpdateTime end

local function adHocRequest(sensorId, requestId) updateIdealRADARsGFXStep(0.0, sensorId, true, requestId) end

local function cacheLatestReading(sensorId, latestReading)
  if sensorId ~= nil then
    latestReadings[sensorId] = latestReading
  end
end

local function getIdealRADARReading(sensorId) return latestReadings[sensorId] end

local function getLatest(sensorId) return idealRADARs[sensorId].controller.getLatest() end

local function updateGFX(dtSim)
  for sensorId, _ in pairs(idealRADARs) do
    updateIdealRADARsGFXStep(dtSim, sensorId, false, nil)
  end
end

local function onVehicleDestroyed(vid)
  for sensorId, _ in pairs(idealRADARs) do
    if vid == objectId then
      remove(sensorId)
      idealRADARs[sensorId] = nil
    end
  end
end


-- Public interface:
M.create                                                  = create
M.remove                                                  = remove
M.adHocRequest                                            = adHocRequest
M.cacheLatestReading                                      = cacheLatestReading
M.getIdealRADARReading                                    = getIdealRADARReading
M.getLatest                                               = getLatest
M.setUpdateTime                                           = setUpdateTime
M.updateGFX                                               = updateGFX
M.onVehicleDestroyed                                      = onVehicleDestroyed

return M