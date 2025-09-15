-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local controlledEngine
local clutchDisableElectricsName
local compressionBrakeCoef
local electricsNameActual
local electricsNameSetting
local electricsNameIsEnabled
local electricsNameLevelIndex
local lastCompressionBrakeCoef = -1
local compressionBrakeLevels
local compressionBrakeLevelActiveIndex
local didSetDrivingAggressionOverride

local function setCompressionBrakeCoef(coef)
  compressionBrakeCoef = clamp(coef, 0, 1)
  guihooks.message(string.format("Compression Brake: %d%%", compressionBrakeCoef * 100), 5, "vehicle.compressionBrake." .. controlledEngine.name)
end

local function changeCompressionBrakeLevel(indexChange)
  compressionBrakeLevelActiveIndex = clamp(compressionBrakeLevelActiveIndex + indexChange, 1, #compressionBrakeLevels)
  setCompressionBrakeCoef(compressionBrakeLevels[compressionBrakeLevelActiveIndex])
end

local function setCompressionBrakeLevel(level)
  compressionBrakeLevelActiveIndex = clamp(level, 1, #compressionBrakeLevels)
  setCompressionBrakeCoef(compressionBrakeLevels[compressionBrakeLevelActiveIndex])
end

local function toggleCompressionBrakeState()
  if compressionBrakeCoef > 0 then
    setCompressionBrakeCoef(0)
  else
    setCompressionBrakeCoef(compressionBrakeLevels[compressionBrakeLevelActiveIndex])
  end
end

local function updateGFX(dt)
  local compressionBrakeCoefActual = compressionBrakeCoef * (1 - sign(controlledEngine.requestedThrottle))
  if electrics.values[clutchDisableElectricsName] and electrics.values[clutchDisableElectricsName] < 1 then
    compressionBrakeCoefActual = 0
  end
  if compressionBrakeCoefActual ~= lastCompressionBrakeCoef then
    controlledEngine:setCompressionBrakeCoef(compressionBrakeCoefActual)
    lastCompressionBrakeCoef = compressionBrakeCoefActual
  end

  if compressionBrakeCoefActual > 0 then --always override the aggression (this can impact other systems potentially)
    controller.mainController.setAggressionOverride(1)
    didSetDrivingAggressionOverride = true
  elseif didSetDrivingAggressionOverride then --but only unset the override once IF we did set it before to at least give other systems a chance of using it as well
    controller.mainController.setAggressionOverride(nil)
    didSetDrivingAggressionOverride = false
  end
  electrics.values[electricsNameSetting] = compressionBrakeCoef
  electrics.values[electricsNameActual] = compressionBrakeCoefActual
  electrics.values[electricsNameIsEnabled] = compressionBrakeCoef > 0
  electrics.values[electricsNameLevelIndex] = compressionBrakeLevelActiveIndex
end

local function reset(jbeamData)
  --compressionBrakeCoef = 0
  lastCompressionBrakeCoef = -1
end

local function init(jbeamData)
  local engineName = jbeamData.controlledEngine or "mainEngine"
  controlledEngine = powertrain.getDevice(engineName)
  if not controlledEngine then
    --TODO fix
    log("E", "compressionBrake.init", string.format("Can't find requested engine with name: %q, compression brake controls won't work!", engineName))
    M.updateGFX = nop
    return
  end
  electricsNameActual = jbeamData.electricsNameActual or (engineName .. "_compressionBrake_actual")
  electricsNameSetting = jbeamData.electricsNameSetting or (engineName .. "_compressionBrake_setting")
  electricsNameIsEnabled = jbeamData.electricsNameIsEnabled or (engineName .. "_compressionBrake_isEnabled")
  electricsNameLevelIndex = jbeamData.electricsNameLevelIndex or (engineName .. "_compressionBrake_levelIndex")

  compressionBrakeLevels = jbeamData.compressionBrakeLevels or {0.33, 0.66, 1.0}
  compressionBrakeLevelActiveIndex = #compressionBrakeLevels

  clutchDisableElectricsName = jbeamData.clutchDisableElectricsName or "clutchRatio"

  compressionBrakeCoef = 0
  didSetDrivingAggressionOverride = false
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

M.setCompressionBrakeCoef = setCompressionBrakeCoef
M.setCompressionBrakeLevel = setCompressionBrakeLevel
M.changeCompressionBrakeLevel = changeCompressionBrakeLevel
M.toggleCompressionBrakeState = toggleCompressionBrakeState

return M
