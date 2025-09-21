-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local htmlTexture = require("htmlTexture")

local gaugesScreenName = nil
local htmlPath = nil
local gaugeHTMLTexture

local updateTimer = 0
local gaugeData = {electrics = {}}

local oldtrip = -999999
local oldOdometer = -999999
local updateDistance = 1

local abs = math.abs

local function updateGFX(dt)
  updateTimer = updateTimer + dt

  local trip = electrics.values.trip or 0
  local currentOdometerValue = electrics.values.odometer or 0

  local hasChanged = (abs(trip - oldtrip) > updateDistance) or (abs(currentOdometerValue - oldOdometer) > updateDistance)

  if hasChanged and playerInfo.anyPlayerSeated and obj:getUpdateUIflag() then
    --gcprobe()
    gaugeData.electrics.trip = extensions.odometer.getRelativeRecording()
    gaugeData.electrics.odometer = currentOdometerValue
    --gcprobe()
    oldtrip = trip
    oldOdometer = currentOdometerValue

    gaugeHTMLTexture:streamJS("updateData", "updateData", gaugeData)
    updateTimer = 0
  end
end

local function reset()
  oldtrip = -999999
end

local function initSecondStage(jbeamData)
  --merge config data from multiple parts so that some things can be defined in sub-parts. section name needs to be "configuration_xyz"
  local configData = jbeamData.configuration or {}
  --dump(configData)
  for k, v in pairs(jbeamData) do
    if k:sub(1, #"configuration_") == "configuration_" then
      tableMergeRecursive(configData, v)
    end
  end
  --dump(configData)

  if not configData then
    log("E", "analogOdometer.initSecondStage", "Can't find config data...")
    return
  end

  gaugesScreenName = configData.materialName
  htmlPath = configData.htmlPath
  local width = configData.displayWidth
  local height = configData.displayHeight

  if configData.unit == "imperial" then
    updateDistance = 1609.34 --1 mile
  else
    updateDistance = 1000 --1km
  end
  if configData.odometerHasDecimalSeparator or configData.tripHasDecimalSeparator then
    updateDistance = updateDistance * 0.1 --if we have a decimal place, update 10 times as often as without
  end

  if not gaugesScreenName then
    log("E", "analogOdometer.initSecondStage", "Got no material name for the texture, can't display anything...")
    return
  else
    if htmlPath then
      gaugeHTMLTexture = htmlTexture.new(gaugesScreenName, htmlPath, width, height, 1, "manual")
    else
      log("E", "analogOdometer.initSecondStage", "Got no html path for the texture, can't display anything...")
      return
    end
  end

  gaugeHTMLTexture:callJS("setup", configData)
end

local function setParameters(parameters)
end

M.init = nop
M.initSecondStage = initSecondStage
M.reset = reset
--nop
M.updateGFX = updateGFX

M.setParameters = setParameters

return M
