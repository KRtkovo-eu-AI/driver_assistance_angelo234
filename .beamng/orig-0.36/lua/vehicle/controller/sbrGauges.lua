-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local htmlTexture = require("htmlTexture")

local deg = math.deg
local atan2 = math.atan2

local gaugesScreenName = nil
local htmlPath = nil

local updateTimer = 0
local updateGraphTimer = 0
local updateFPS = 30
local avgConsumptionSmoother = newExponentialSmoothing(1000)
local avgPowerSmoother = newExponentialSmoothing(1000)
local currentConsumptionSmoother = newExponentialSmoothing(50)
local currentPowerSmoother = newExponentialSmoothing(30)
local lastEnergyAvailable = 0
local avgConsumptionPer100km = 0
local avgPower = 0
local range = 0

local batteriesUsed = {}
local gaugeData = {sensors = {}}
local mapData = {}
local consumData = {}
local vecX = vec3(1, 0, 0)
local vecY = vec3(0, -1, 0)

local function updateGFX(dt)
  updateTimer = updateTimer + dt
  updateGraphTimer = updateGraphTimer + dt

  if playerInfo.anyPlayerSeated and obj:getUpdateUIflag() then
    local wheelSpeed = electrics.values.wheelspeed * 3.6
    local isMoving = wheelSpeed > 1

    local energyLeft = 0
    for k, _ in pairs(batteriesUsed) do
      local storage = energyStorage.getStorage(k)
      energyLeft = energyLeft + storage.storedEnergy
    end

    local diff = lastEnergyAvailable - energyLeft
    local currentPower = currentPowerSmoother:get(diff / updateTimer) --J/s aka W
    avgPower = avgPowerSmoother:get(isMoving and currentPower or avgPower)

    lastEnergyAvailable = energyLeft

    local timeToGo100km = isMoving and (100 / wheelSpeed) or 0
    local currentConsumptionPer100km = currentPower * timeToGo100km

    currentConsumptionPer100km = currentConsumptionSmoother:get(currentConsumptionPer100km)
    avgConsumptionPer100km = avgConsumptionSmoother:get(isMoving and currentConsumptionPer100km or avgConsumptionPer100km)

    local wheelspeed = electrics.values.wheelspeed or 0
    gaugeData.gear = electrics.values.gear

    gaugeData.sensors.gx2 = sensors.gx2
    gaugeData.sensors.gy2 = sensors.gy2

    gaugeData.temp = obj:getEnvTemperature() - 273.15
    --data.time = os.date("%H") .. ":" .. os.date("%M") -- done to prevent seconds from being sent.
    gaugeData.speed = wheelspeed
    gaugeData.electrics = electrics.values
    htmlTexture.call(gaugesScreenName, "updateData", gaugeData)

    mapData.x, mapData.y = obj:getPositionXYZ()
    local dir = obj:getDirectionVector():normalized()
    mapData.rotation = deg(atan2(dir:dot(vecX), dir:dot(vecY)))
    htmlTexture.call(gaugesScreenName, "updateMap", mapData)

    consumData.current = currentPower * 0.001
    consumData.average = avgPower * 0.001
    consumData.range = range
    htmlTexture.call(gaugesScreenName, "updateConsum", consumData)

    updateTimer = 0

    if updateGraphTimer > 5 then
      range = avgConsumptionPer100km > 0 and (energyLeft * 0.0278 / avgConsumptionPer100km) or 0
      updateGraphTimer = 0
      htmlTexture.call(gaugesScreenName, "appendGraphConsum", avgConsumptionPer100km / 1000)
    end
  end
end

local function init(jbeamData)
  log("E", "sbrGauges", "This controller is deprecated and shall not be used. It might be removed without further notice in the future! Please switch to the 'genericGauges' controller instead.")

  gaugesScreenName = jbeamData.materialName
  htmlPath = jbeamData.htmlPath
  local unitType = settings.getValue("uiUnitLength") or "metric"
  local width = 1024
  local height = 512

  if not gaugesScreenName then
    log("E", "sbrGauges", "Got no material name for the texture, can't display anything...")
    M.updateGFX = nop
  else
    if htmlPath then
      htmlTexture.create(gaugesScreenName, htmlPath, width, height, updateFPS, "automatic")
      htmlTexture.call(gaugesScreenName, "setUnits", {unitType = unitType})
      obj:queueGameEngineLua(string.format('extensions.ui_uiNavi.requestVehicleDashboardMap(%q, "initMap", %d)', gaugesScreenName, obj:getID()))
    else
      log("E", "sbrGauges", "Got no html path for the texture, can't display anything...")
      M.updateGFX = nop
    end
  end

  batteriesUsed = {}
  local motors = powertrain.getDevicesByType("electricMotor")
  for _, v in pairs(motors) do
    for _, j in pairs(v.registeredEnergyStorages) do
      batteriesUsed[j] = true
    end
  end

  lastEnergyAvailable = 0

  for k, _ in pairs(batteriesUsed) do
    local storage = energyStorage.getStorage(k)
    lastEnergyAvailable = lastEnergyAvailable + storage.storedEnergy
  end
  avgConsumptionSmoother:set(20000)
  currentConsumptionSmoother:set(0)
  currentPowerSmoother:set(0)
end

local function reset()
  htmlTexture.call(gaugesScreenName, "setUnits", {unitType = settings.getValue("uiUnitLength") or "metric"})
end

local function setUIMode(modeName, modeColor)
  htmlTexture.call(gaugesScreenName, "updateMode", {txt = modeName, col = modeColor})
end

local function setParameters(parameters)
  if parameters.modeName and parameters.modeColor then
    setUIMode(parameters.modeName, parameters.modeColor)
  end
end

M.init = init
M.reset = reset
--nop
M.updateGFX = updateGFX
M.setUIMode = setUIMode
M.setParameters = setParameters

return M
