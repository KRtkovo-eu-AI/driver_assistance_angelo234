-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local abs = math.abs

local electricsNames
local motorThrottleElectric

local function updateGFX(dt)
  local motorThrottle = 0
  for _, electric in ipairs(electricsNames) do
    if type(electrics.values[electric]) == "number" and abs(electrics.values[electric]) > 0 then
      motorThrottle = 1
    end
  end
  electrics.values[motorThrottleElectric] = motorThrottle
end

local function reset(jbeamData)
end

local function init(jbeamData)
  electricsNames = type(jbeamData.controlElectricsName) ~= "table" and {jbeamData.controlElectricsName} or jbeamData.controlElectricsName
  motorThrottleElectric = jbeamData.motorThrottleElectricsName
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

return M
