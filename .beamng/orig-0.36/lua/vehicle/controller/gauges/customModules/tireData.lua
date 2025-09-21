-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local max = math.max

local updateTemperatures = false
local updatePressures = false

local function updateGFX(dt)
end

local function updateGaugeData(moduleData, dt)
  if updateTemperatures then
    moduleData.temperatures = moduleData.temperatures or {}
    for _, wd in pairs(wheels.wheels) do
      moduleData.temperatures[wd.name] = "#ffffff"
    end
  end
  if updatePressures then
    moduleData.pressures = moduleData.pressures or {}
    for _, wd in pairs(wheels.wheels) do
      local hasPressure = wd.pressureGroup and v.data.pressureGroups and v.data.pressureGroups[wd.pressureGroup]
      local pressure = 0
      if hasPressure then
        pressure = max((obj:getGroupPressure(v.data.pressureGroups[wd.pressureGroup]) - obj:getEnvPressure()) * 0.001, 0)
      end
      moduleData.pressures[wd.name] = pressure
    end
  end
end

local function setupGaugeData(properties)
  updateTemperatures = properties.temperatures or false
  updatePressures = properties.pressures or false
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
