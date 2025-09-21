-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local htmlTexture = require("htmlTexture")

local min = math.min
local max = math.max

local gaugesScreenName = nil
local htmlPath = nil

local previousFuel = 0
local fuelSmoother = newTemporalSmoothing(50, 50)
local fuelDisplaySmoother = newTemporalSmoothing(5, 3)
local avgFuelSum = 0
local avgFuelCounter = 0
local updateTimer = 0
local updateFPS = 30
local unitType
local uiIsReady = false

local function updateGFX(dt)
  if not uiIsReady then
    return
  end
  updateTimer = updateTimer + dt
  if playerInfo.anyPlayerSeated and obj:getUpdateUIflag() then
    local data = {}
    local wheelspeed = electrics.values.wheelspeed or 0
    data.gear = electrics.values.gear

    local fuelVolume = electrics.values.fuelVolume or 0
    local fuelConsumption = min(max((previousFuel - fuelVolume) / (updateTimer * wheelspeed) * 1000 * 100, 0), 100) -- l/(s*(m/s)) = l/m
    previousFuel = fuelVolume
    local fuelConsumptionSmooth = fuelSmoother:getUncapped(fuelConsumption, updateTimer)

    if wheelspeed > 1.4 then
      avgFuelSum = avgFuelSum + fuelConsumption
      avgFuelCounter = avgFuelCounter + 1
    end
    data.averageFuelConsumption = min(max(avgFuelSum / max(avgFuelCounter, 1), 0), 30)

    local fuelDisplay = min(max((3 * fuelConsumptionSmooth) / 30, 0), 3)
    if (electrics.values.engineLoad or 0) <= 0 then
      fuelDisplay = -1
    end
    if wheelspeed < 1 and (electrics.values.throttle or 0) <= 0 then
      fuelDisplay = 0
    end
    data.fuelDisplay = fuelDisplaySmoother:getUncapped(fuelDisplay, updateTimer)

    data.temp = obj:getEnvTemperature() - 273.15
    data.time = os.date("%H") .. ":" .. os.date("%M") -- done to prevent seconds from being sent.
    data.speed = wheelspeed
    --dump(data)

    htmlTexture.call(gaugesScreenName, "updateData", data)
    updateTimer = 0
  end
end

local function init(jbeamData)
  log("E", "etkGauges", "This controller is deprecated and shall not be used. It might be removed without further notice in the future! Please switch to the 'genericGauges' controller instead.")

  previousFuel = 0
  avgFuelSum = 0
  avgFuelCounter = 0
  fuelSmoother:reset()
  fuelDisplaySmoother:reset()

  gaugesScreenName = jbeamData.materialName
  htmlPath = jbeamData.htmlPath
  unitType = jbeamData.unitType or "metric"
  local width = jbeamData.textureWidth or 512
  local height = jbeamData.textureHeight or 256

  if not gaugesScreenName then
    log("E", "etkGauges", "Got no material name for the texture, can't display anything...")
    M.updateGFX = nop
  else
    if htmlPath then
      htmlTexture.create(gaugesScreenName, htmlPath, width, height, updateFPS, "automatic")
    else
      log("E", "etkGauges", "Got no html path for the texture, can't display anything...")
      M.updateGFX = nop
    end
  end
end

local function uiReady()
  htmlTexture.call(gaugesScreenName, "setUnits", {unitType = unitType})
  uiIsReady = true
end

M.init = init
M.updateGFX = updateGFX
M.uiReady = uiReady

return M
