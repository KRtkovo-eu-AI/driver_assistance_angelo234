-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local max = math.max
local clamp = clamp

local actuatorsControllerMain
local actuatorsControllerLift
local liftAirbagsGroupName
local mainAirbagsGroupName
local currentMode
local modes = {lift = "lift", drop = "drop"}
local minMaximumSupplyPressure
local maxMaximumSupplyPressure
local maximumSupplyPressure
local maximumSupplyPressureChangeRate
local mainAirbagsPressureEletricsName
local liftAirbagsPressureEletricsName
local liftAxleControlModeEletricsName

local function updateGFX(dt)
  if currentMode == modes.drop then
    actuatorsControllerMain.setBeamGroupMaximumSupplyPressure(mainAirbagsGroupName, maximumSupplyPressure)
  elseif currentMode == modes.lift then
  end

  maximumSupplyPressure = clamp(maximumSupplyPressure + maximumSupplyPressureChangeRate * dt, max(powertrain.currentEnvPressure, minMaximumSupplyPressure), maxMaximumSupplyPressure)
  --print(maximumSupplyPressure)
  electrics.values[mainAirbagsPressureEletricsName] = actuatorsControllerMain.getAveragePressure(mainAirbagsGroupName) - powertrain.currentEnvPressure
  electrics.values[liftAirbagsPressureEletricsName] = actuatorsControllerLift.getAveragePressure(liftAirbagsGroupName) - powertrain.currentEnvPressure
  electrics.values[liftAxleControlModeEletricsName] = currentMode == modes.drop and 1 or 0
  --print(electrics.values[mainAirbagsPressureEletricsName])
end

local function setMode(mode)
  currentMode = mode
  if currentMode == modes.drop then
    actuatorsControllerMain.setBeamGroupValveState(mainAirbagsGroupName, 1)
    actuatorsControllerLift.setBeamGroupValveState(liftAirbagsGroupName, -1)
  elseif currentMode == modes.lift then
    actuatorsControllerLift.setBeamGroupValveState(liftAirbagsGroupName, 1)
    actuatorsControllerMain.setBeamGroupValveState(mainAirbagsGroupName, -1)
  end
end

local function toggleMode()
  if currentMode == modes.drop then
    setMode(modes.lift)
  else
    setMode(modes.drop)
  end
end

local function setMaximumSupplyPressure(maxSupplyPressure)
  maximumSupplyPressure = maxSupplyPressure
end

local function setMaximumSupplyPressureChangeRate(maxSupplyPressureChangeRate)
  maximumSupplyPressureChangeRate = maxSupplyPressureChangeRate
end

local function reset(jbeamData)
  maximumSupplyPressureChangeRate = 0
end

local function init(jbeamData)
  maximumSupplyPressure = jbeamData.defaultMaximumSupplyPressure or 0
  maximumSupplyPressureChangeRate = 0
  liftAirbagsGroupName = jbeamData.liftAirbagsGroupName
  mainAirbagsGroupName = jbeamData.mainAirbagsGroupName
  minMaximumSupplyPressure = jbeamData.minimumSupplyPressure or 0
  maxMaximumSupplyPressure = jbeamData.maximumSupplyPressure or 1000000
  maximumSupplyPressureChangeRate = jbeamData.maximumSupplyPressureChangeRate or 1000
  mainAirbagsPressureEletricsName = jbeamData.mainAirbagsPressureEletricsName or (M.name .. "_" .. "main_airbags_pressure_avg")
  liftAirbagsPressureEletricsName = jbeamData.liftAirbagsPressureEletricsName or (M.name .. "_" .. "lift_airbags_pressure_avg")
  liftAxleControlModeEletricsName = jbeamData.liftAxleControlModeEletricsName or (M.name .. "_" .. "lift_axle_mode")
end

local function initLastStage(jbeamData)
  local actuatorsControllerNameLift = jbeamData.actuatorsControllerNameLift
  actuatorsControllerLift = controller.getController(actuatorsControllerNameLift)
  if not actuatorsControllerLift then
    log("E", "liftAxleControl.init", "Can't find specified lift actuators controller: " .. actuatorsControllerNameLift)
    M.updateGFX = nop
    return
  end

  local actuatorsControllerNameMain = jbeamData.actuatorsControllerNameMain
  actuatorsControllerMain = controller.getController(actuatorsControllerNameMain)
  if not actuatorsControllerMain then
    log("E", "liftAxleControl.init", "Can't find specified main actuators controller: " .. actuatorsControllerNameMain)
    M.updateGFX = nop
    return
  end

  setMode(modes.lift)
end

M.init = init
--M.initSecondStage = initSecondStage
M.initLastStage = initLastStage

M.reset = reset

M.updateGFX = updateGFX

M.setMode = setMode
M.toggleMode = toggleMode
M.setMaximumSupplyPressure = setMaximumSupplyPressure
M.setMaximumSupplyPressureChangeRate = setMaximumSupplyPressureChangeRate

return M
