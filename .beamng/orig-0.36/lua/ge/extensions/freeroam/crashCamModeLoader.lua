-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local function onSettingsChanged()
  if settings.getValue('enableCrashCam') then
    extensions.load('freeroam_crashCamMode')
  elseif freeroam_crashCamMode then
    extensions.unload('freeroam_crashCamMode')
  end
end

local function onExtensionLoaded()
  if settings.getValue('enableCrashCam') then
    extensions.load('freeroam_crashCamMode')
  end
end

M.onSettingsChanged = onSettingsChanged
M.onExtensionLoaded = onExtensionLoaded

return M
