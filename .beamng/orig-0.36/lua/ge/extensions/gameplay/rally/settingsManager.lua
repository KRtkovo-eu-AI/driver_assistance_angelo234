-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local MissionSettings = require('/lua/ge/extensions/gameplay/rally/notebook/missionSettings')

local M = {}

local missionSettings = nil

local function loadMissionSettingsForMissionDir(missionDir)
  local missionSettings = MissionSettings(rallyUtil.getMissionSettingsFile(missionDir))
  if missionSettings:load() then
    return missionSettings, nil
  end
  return nil, "error loading mission settings"
end

local function getMissionSettings()
  return missionSettings
end

-- local function ensureMissionSettingsFile(notebook)
--   local md = notebook:getMissionDir()
--   if not md then
--     log('E', logTag, 'ensureMissionSettingsFile has nil missionDir')
--     return
--   end

--   local settingsFname = rallyUtil.getMissionSettingsFile(md)
--   if not FS:fileExists(settingsFname) then
--     log('I', logTag, 'creating mission.settings.json at '..settingsFname)
--     local missionSettings = MissionSettings(settingsFname)
--     missionSettings.notebook.filename = notebook:basename()
--     local assumedCodriverName = notebook.codrivers.sorted[1].name
--     if assumedCodriverName then
--       missionSettings.notebook.codriver = assumedCodriverName
--     else
--       missionSettings.notebook.codriver = nil
--       log('W', logTag, 'ensureMissionSettingsFile has no codrivers')
--     end
--     jsonWriteFile(settingsFname, missionSettings:onSerialize(), true)
--   end
-- end

-- The notebook determines the language via the selected codriver
--
-- The MissionSettings tracks the selected codriver.
local function load(notebook)
  local missionSettingsPath = rallyUtil.getMissionSettingsFile(notebook:getMissionDir())
  missionSettings = MissionSettings(missionSettingsPath)
  missionSettings:load()
end

M.getMissionSettings = getMissionSettings
M.ensureMissionSettingsFile = ensureMissionSettingsFile
M.reset = reset
M.load = load
M.loadMissionSettingsForMissionDir = loadMissionSettingsForMissionDir

return M
