-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local function updateGFX(dt)
end

local function updateGaugeData(moduleData, dt)
  moduleData.temperatureEnv = obj:getEnvTemperature() - 273.15
  moduleData.time = os.date("%H") .. ":" .. os.date("%M")
end

local function setupGaugeData(properties)
end

local function reset()
end

local function init(jbeamData)
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

M.setupGaugeData = setupGaugeData
M.updateGaugeData = updateGaugeData

return M
