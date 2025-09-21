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
local updateTimerOdo = 99
local updateFPS = 60
local gaugeData = {electrics = {}, powertrain = {}, customModules = {}}
local electricsConfig
local powertrainConfig
local customModuleConfig

local electricsUpdate = nop
local powertrainUpdate = nop
local customModuleUpdate = nop

local function updateElectricsData(dt)
  for _, v in ipairs(electricsConfig) do
    gaugeData.electrics[v] = electrics.values[v] or 0
  end
end

local function updatePowertrainData(dt)
  for _, v in ipairs(powertrainConfig) do
    for _, n in ipairs(v.properties) do
      gaugeData.powertrain[v.device.name][n] = v.device[n] or 0
    end
  end
end

local function updateCustomModuleData(dt)
  for _, module in ipairs(customModuleConfig) do
    module.controller.updateGaugeData(gaugeData.customModules[module.name], dt)
  end
end

local function updateGFX(dt)
  updateTimer = updateTimer + dt
  updateTimerOdo = updateTimerOdo + dt

  if playerInfo.anyPlayerSeated and obj:getUpdateUIflag() then
    --gcprobe()
    electricsUpdate(updateTimer)
    powertrainUpdate(updateTimer)
    customModuleUpdate(updateTimer)
    --gcprobe()
    --dump(gaugeData)

    gaugeHTMLTexture:streamJS("updateData", "updateData", gaugeData)
    updateTimer = 0
  end
end

local function setupElectricsData(config)
  if not config then
    return
  end
  electricsConfig = {}
  for _, v in pairs(config) do
    table.insert(electricsConfig, v)
  end
  electricsUpdate = updateElectricsData
end

local function setupPowertrainData(config)
  if not config then
    return
  end
  local mergedConfig = {}
  for _, v in pairs(tableFromHeaderTable(config)) do
    mergedConfig[v.deviceName] = mergedConfig[v.deviceName] or {}
    table.insert(mergedConfig[v.deviceName], v.property)
  end

  powertrainConfig = {}
  for k, v in pairs(mergedConfig) do
    local device = powertrain.getDevice(k)
    if device then
      table.insert(powertrainConfig, {device = device, properties = v})
      gaugeData.powertrain[k] = {}
    end
  end

  powertrainUpdate = updatePowertrainData
end

local function setupCustomModuleData(config)
  if not config then
    return
  end

  local mergedConfig = {}
  for _, v in pairs(tableFromHeaderTable(config)) do
    mergedConfig[v.moduleName] = mergedConfig[v.moduleName] or {}
    if v.property then
      mergedConfig[v.moduleName][v.property] = true
    end
  end
  --dump(mergedConfig)

  customModuleConfig = {}
  local controllerPath = "gauges/customModules/"
  for k, v in pairs(mergedConfig) do
    local c = controller.getController(controllerPath .. k)
    if c and c.setupGaugeData and c.updateGaugeData then
      c.setupGaugeData(v, gaugeHTMLTexture)
      table.insert(customModuleConfig, {controller = c, name = k, properties = v})
      gaugeData.customModules[k] = {}
    else
      log("E", "genericGauges.setupCustomModuleData", "Can't find controller: " .. k)
    end
  end

  customModuleUpdate = updateCustomModuleData
end

local function reset()
end

local function initSecondStage(jbeamData)
  local displayData = jbeamData.displayData

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
    log("E", "genericGauges.initSecondStage", "Can't find config data...")
    return
  end

  gaugesScreenName = configData.materialName
  htmlPath = configData.htmlPath
  local width = configData.displayWidth
  local height = configData.displayHeight

  if not gaugesScreenName then
    log("E", "genericGauges.initSecondStage", "Got no material name for the texture, can't display anything...")
    return
  else
    if htmlPath then
      --htmlTexture.create(gaugesScreenName, htmlPath, width, height, updateFPS, "automatic")
      gaugeHTMLTexture = htmlTexture.new(gaugesScreenName, htmlPath, width, height, updateFPS)
    else
      log("E", "genericGauges.initSecondStage", "Got no html path for the texture, can't display anything...")
      return
    end
  end

  setupElectricsData(displayData.electrics)
  setupPowertrainData(displayData.powertrain)
  setupCustomModuleData(displayData.customModules)

  --settingskeys to be found here: lua\ge\extensions\core\settings\settings.lua
  local config = {
    uiUnitLength = settings.getValue("uiUnitLength") or "metric",
    uiUnitTemperature = settings.getValue("uiUnitTemperature") or "c",
    uiUnitWeight = settings.getValue("uiUnitWeight") or "kg",
    uiUnitTorque = settings.getValue("uiUnitTorque") or "metric",
    uiUnitPower = settings.getValue("uiUnitPower") or "hp",
    uiUnitEnergy = settings.getValue("uiUnitEnergy") or "metric",
    uiUnitConsumptionRate = settings.getValue("uiUnitConsumptionRate") or "metric",
    uiUnitVolume = settings.getValue("uiUnitVolume") or "l",
    uiUnitPressure = settings.getValue("uiUnitPressure") or "bar",
    uiUnitDate = settings.getValue("uiUnitDate") or "ger"
  }
  config = tableMerge(config, configData)
  --dump(config)

  gaugeHTMLTexture:callJS("setup", config)
end

local function setUIMode(parameters)
  gaugeHTMLTexture:callJS("updateMode", parameters)
end

local function setParameters(parameters)
  if parameters.modeName and parameters.modeColor then
    setUIMode(parameters)
  end
end

local function setPartCondition(odometer, integrity, visual)
  odometerOffset = odometer
end

local function getPartCondition(storage)
  local integrityState = {
    odometer = odometerOffset
  }
  local integrityValue = 1
  return integrityValue, integrityState
end

M.init = nop
M.initSecondStage = initSecondStage
M.reset = reset
--nop
M.updateGFX = updateGFX

M.setParameters = setParameters

return M
