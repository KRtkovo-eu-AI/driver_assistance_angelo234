-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

-- get available photomode overlay files
local function getPhotomodeOverlays()
  local result = {}
  for i,file in ipairs(FS:findFiles('ui/modules/photomode/ui-overlays', '*.png', 0, false, false)) do
    table.insert(result, {filename=file})
  end
  return result
end

M.getPhotomodeOverlays = getPhotomodeOverlays
return M