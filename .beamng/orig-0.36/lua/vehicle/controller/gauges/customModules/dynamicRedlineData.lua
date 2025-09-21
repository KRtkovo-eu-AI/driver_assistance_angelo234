-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local floor = math.floor

local minOutput
local maxOutput
local minOutputRPM
local maxOutputRPM
local minOilTemp
local maxOilTemp
local yellowOutputOffset
local coldOilRPMCoef

local function updateGFX(dt)
end

local function updateGaugeData(moduleData, dt)
  local currentTempRedlineCoef = linearScale(electrics.values.oiltemp or 0, minOilTemp, maxOilTemp, coldOilRPMCoef, 1)
  local maxRedline = electrics.values.maxrpm or 0
  local currentRPM = electrics.values.rpm or 0
  local adjustedRedline = maxRedline * currentTempRedlineCoef
  local redlineOutput = floor(linearScale(adjustedRedline, minOutputRPM, maxOutputRPM, minOutput, maxOutput))
  local maxGearReached = (electrics.values.gearIndex or 0) >= (electrics.values.maxGearIndex or 0)
  moduleData.yellow = redlineOutput + yellowOutputOffset
  moduleData.red = redlineOutput
  moduleData.shiftLight = (currentRPM >= adjustedRedline * 0.95) and not maxGearReached
end

local function setupGaugeData(properties)
end

local function reset()
end

local function init(jbeamData)
  minOutput = jbeamData.minOutput or 0
  maxOutput = jbeamData.maxOutput or 20
  minOutputRPM = jbeamData.minOutputRPM or 0
  maxOutputRPM = jbeamData.maxOutputRPM or 10000
  minOilTemp = jbeamData.minOilTemp or 50
  maxOilTemp = jbeamData.maxOilTemp or 90
  yellowOutputOffset = jbeamData.yellowOutputOffset or -2
  coldOilRPMCoef = jbeamData.coldOilRPMCoef or 0.5
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

M.setupGaugeData = setupGaugeData
M.updateGaugeData = updateGaugeData

return M
