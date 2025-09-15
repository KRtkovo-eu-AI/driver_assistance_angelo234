-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local updateAcceleration = false
local updateAccelerationSmooth = false

local function updateGFX(dt)
end

local function updateGaugeData(moduleData, dt)
  if updateAcceleration then
    moduleData.x = sensors.gx
    moduleData.y = sensors.gy
    moduleData.z = sensors.gz
  end
  if updateAccelerationSmooth then
    moduleData.xSmooth = sensors.gx2
    moduleData.ySmooth = sensors.gy2
    moduleData.zSmooth = sensors.gz2
  end
end

local function setupGaugeData(properties)
  updateAcceleration = properties.acceleration or false
  updateAccelerationSmooth = properties.accelerationSmooth or false
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
