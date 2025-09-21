-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local htmlTexture = require("htmlTexture")

local gaugesScreenName = nil
local htmlPath = nil

local min = math.min
local max = math.max

local updateTimer = 0
local updateFPS = 8
local gaugeData = {electrics = {}}

local previousFuel = 0
local avgFuelSum = 0
local avgFuelCounter = 0

local function updateGFX(dt)
  updateTimer = updateTimer + dt

  if playerInfo.anyPlayerSeated and obj:getUpdateUIflag() and updateTimer > 0.1 then
    -- gaugeData.temp = obj:getEnvTemperature() - 273.15
    gaugeData.electrics.signal_L = electrics.values.signal_L
    gaugeData.electrics.signal_R = electrics.values.signal_R
    gaugeData.electrics.lights = electrics.values.lights
    gaugeData.electrics.highbeam = electrics.values.highbeam
    gaugeData.electrics.lowfuel = electrics.values.lowfuel
    gaugeData.electrics.parkingbrake = electrics.values.parkingbrake
    gaugeData.electrics.checkengine = electrics.values.checkengine
    gaugeData.electrics.hazard = electrics.values.hazard
    gaugeData.electrics.oil = electrics.values.oil
    gaugeData.electrics.oiltemp = electrics.values.oiltemp
    gaugeData.electrics.gear = electrics.values.gear
    gaugeData.electrics.rpmTacho = electrics.values.rpmTacho
    gaugeData.electrics.fuel = electrics.values.fuel
    gaugeData.electrics.watertemp = electrics.values.watertemp
    gaugeData.electrics.engineRunning = electrics.values.engineRunning
    gaugeData.electrics.wheelspeed = electrics.values.wheelspeed
    gaugeData.electrics.abs = electrics.values.abs
    gaugeData.electrics.absActive = electrics.values.absActive

    local wheelspeed = electrics.values.wheelspeed or 0
    local fuelVolume = electrics.values.fuelVolume or 0
    local fuelConsumption = min(max((previousFuel - fuelVolume) / (updateTimer * wheelspeed) * 1000 * 100, 0), 100) -- l/(s*(m/s)) = l/m
    previousFuel = fuelVolume

    if wheelspeed > 1.4 then
      avgFuelSum = avgFuelSum + fuelConsumption
      avgFuelCounter = avgFuelCounter + 1
    end
    gaugeData.averageFuelConsumption = min(max(avgFuelSum / max(avgFuelCounter, 1), 0), 50)

    htmlTexture.call(gaugesScreenName, "updateData", gaugeData)

    updateTimer = 0
  end
end

local function init(jbeamData)
  log("E", "wendoverGauges", "This controller is deprecated and shall not be used. It might be removed without further notice in the future! Please switch to the 'genericGauges' controller instead.")

  gaugesScreenName = jbeamData.materialName
  htmlPath = jbeamData.htmlPath
  local width = 1024
  local height = jbeamData.height or 161

  if not gaugesScreenName then
    log("E", "wendoverGauges", "Got no material name for the texture, can't display anything...")
    M.updateGFX = nop
  else
    if htmlPath then
      htmlTexture.create(gaugesScreenName, htmlPath, width, height, updateFPS, "automatic")
    else
      log("E", "wendoverGauges", "Got no html path for the texture, can't display anything...")
      M.updateGFX = nop
    end
  end

  gaugeData.gearboxType = "none"
  local gearboxes = powertrain.getDevicesByCategory("gearbox")
  if #gearboxes > 0 then
    gaugeData.gearboxType = gearboxes[1].type
  end
end

local function reset()
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

return M
