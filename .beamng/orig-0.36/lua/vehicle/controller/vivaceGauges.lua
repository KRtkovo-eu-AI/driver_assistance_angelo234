-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local htmlTexture = require("htmlTexture")

local gaugesScreenName = nil
local htmlPath = nil

local updateTimer = 0
local updateFPS = 15
local gaugeData = {electrics = {}}
local motors = {}

local function updateGFX(dt)
  updateTimer = updateTimer + dt

  if playerInfo.anyPlayerSeated and obj:getUpdateUIflag() then
    updateTimer = 0
    local powerDisplay = 0
    local motorCount = 0
    for _, motor in ipairs(motors) do
      powerDisplay = powerDisplay + (motor.throttle or 0)
      powerDisplay = powerDisplay - (motor.regenThrottle or 0)
      motorCount = motorCount + 1
    end
    powerDisplay = motorCount > 0 and (powerDisplay / motorCount * (electrics.values.wheelspeed > 0.5 and 1 or 0)) or 0

    gaugeData.temp = obj:getEnvTemperature() - 273.15
    gaugeData.electrics.signal_L = electrics.values.signal_L
    gaugeData.electrics.signal_R = electrics.values.signal_R
    gaugeData.electrics.lights = electrics.values.lights
    gaugeData.electrics.highbeam = electrics.values.highbeam
    gaugeData.electrics.fog = 0 --no fog lights on vivace
    gaugeData.electrics.lowpressure = electrics.values.lowpressure
    gaugeData.electrics.lowfuel = electrics.values.lowfuel
    gaugeData.electrics.parkingbrake = electrics.values.parkingbrake
    gaugeData.electrics.checkengine = electrics.values.checkengine
    gaugeData.electrics.hazard = electrics.values.hazard
    gaugeData.electrics.oil = electrics.values.oil
    gaugeData.electrics.cruiseControlActive = electrics.values.cruiseControlActive
    gaugeData.electrics.gear = electrics.values.gear
    gaugeData.electrics.rpmTacho = electrics.values.rpmTacho
    gaugeData.electrics.fuel = electrics.values.fuel
    gaugeData.electrics.watertemp = electrics.values.watertemp
    gaugeData.electrics.engineRunning = electrics.values.engineRunning
    gaugeData.electrics.wheelspeed = electrics.values.wheelspeed
    gaugeData.electrics.esc = electrics.values.esc
    gaugeData.electrics.escActive = electrics.values.escActive
    gaugeData.electrics.tcs = electrics.values.tcs
    gaugeData.electrics.tcsActive = electrics.values.tcsActive
    gaugeData.electrics.pwr = powerDisplay

    htmlTexture.call(gaugesScreenName, "updateData", gaugeData)
  end
end

local function init(jbeamData)
  log("E", "vivaceGauges", "This controller is deprecated and shall not be used. It might be removed without further notice in the future! Please switch to the 'genericGauges' controller instead.")

  gaugesScreenName = jbeamData.materialName
  htmlPath = jbeamData.htmlPath
  local width = 1024
  local height = 256

  if not gaugesScreenName then
    log("E", "vivaceGauges", "Got no material name for the texture, can't display anything...")
    M.updateGFX = nop
  else
    if htmlPath then
      htmlTexture.create(gaugesScreenName, htmlPath, width, height, updateFPS, "automatic")
    else
      log("E", "vivaceGauges", "Got no html path for the texture, can't display anything...")
      M.updateGFX = nop
    end
  end

  local config = {unit = settings.getValue("uiUnitLength") or "metric"}
  config = tableMergeRecursive(jbeamData, config)

  motors = powertrain.getDevicesByType("electricMotor")

  htmlTexture.call(gaugesScreenName, "setup", config)
end

local function reset()
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

M.setParameters = setParameters

return M
