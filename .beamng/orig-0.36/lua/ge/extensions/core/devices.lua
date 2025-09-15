-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local function onVehicleSwitched(oldId, newId, player)
  if Device then Device.resetRGB(0) end
end

M.onVehicleSwitched = onVehicleSwitched

return M
