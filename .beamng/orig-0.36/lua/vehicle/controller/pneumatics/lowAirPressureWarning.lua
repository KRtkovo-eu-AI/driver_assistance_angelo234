-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local lowAirWarningElectric
local airTooLowPressure
local airOkPressure
local relevantPressureTank
local lowAirWarningSoundEvent
local lowAirWarningSoundVolume
local isPlayingLowPressureSound

local function updateGFX(dt)
  local pressure = relevantPressureTank and relevantPressureTank.currentPressure or 0
  if pressure <= airTooLowPressure and electrics.values.ignitionLevel > 0 then
    electrics.values[lowAirWarningElectric] = 1
    if lowAirWarningSoundEvent and not isPlayingLowPressureSound then
      obj:playSFX(lowAirWarningSoundEvent)
      guihooks.message("vehicle.pneumatics.lowAirPressureWarning", 3, "lowAirWarning")
      isPlayingLowPressureSound = true
    end
  elseif pressure > airOkPressure or electrics.values.ignitionLevel == 0 then
    electrics.values[lowAirWarningElectric] = 0
    if lowAirWarningSoundEvent and isPlayingLowPressureSound then
      obj:stopSFX(lowAirWarningSoundEvent)
      isPlayingLowPressureSound = false
    end
  end
end

local function resetSounds(jbeamData)
  if lowAirWarningSoundEvent then
    obj:stopSFX(lowAirWarningSoundEvent)
  end
  isPlayingLowPressureSound = false
end

local function reset(jbeamData)
  electrics.values[lowAirWarningElectric] = 0
end

local function initSounds(jbeamData)
  local warningSoundEventName = jbeamData.lowAirWarningSoundEvent
  local warningSoundNode = jbeamData.lowAirWarningSoundNode and beamstate.nodeNameMap[jbeamData.lowAirWarningSoundNode] or 0
  lowAirWarningSoundVolume = jbeamData.lowAirWarningSoundVolume or 0.5

  if warningSoundEventName then
    lowAirWarningSoundEvent = obj:createSFXSource2(warningSoundEventName, "AudioDefaultLoop3D", "lowAirWarning", warningSoundNode, 0)
    obj:setVolume(lowAirWarningSoundEvent, lowAirWarningSoundVolume)
  end
  isPlayingLowPressureSound = false
  bdebug.setNodeDebugText("Misc", warningSoundNode, M.name .. " - Low Air Pressure Warning: " .. (warningSoundEventName or "no event"))
end

local function init(jbeamData)
  lowAirWarningElectric = jbeamData.lowAirWarningElectric or "lowAirPressure"
  airTooLowPressure = (jbeamData.airTooLowPressure or 600000) + powertrain.currentEnvPressure
  airOkPressure = (jbeamData.airOkPressure or 650000) + powertrain.currentEnvPressure
  local pressureTankName = jbeamData.relevantPressureTankName or "mainAirTank"
  relevantPressureTank = energyStorage.getStorage(pressureTankName)
end

M.init = init
M.initSounds = initSounds

M.reset = reset
M.resetSounds = resetSounds

M.updateGFX = updateGFX

return M
