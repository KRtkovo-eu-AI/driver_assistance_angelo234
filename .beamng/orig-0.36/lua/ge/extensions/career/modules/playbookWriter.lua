-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'career_career'}

local moduleVersion = 42

local playbook = {}
local fileName = "playbook.json"


local function addMissionPlayedEntry(missionId, stars)
  local e = {
    missionId = missionId,
    stars = stars
  }
  table.insert(playbook, e)
end

local function loadDataFromFile()
  local saveSlot, savePath = career_saveSystem.getCurrentSaveSlot()
  if not saveSlot then return end

  local saveInfo = savePath and jsonReadFile(savePath .. "/info.json")
  local outdated = not saveInfo or saveInfo.version < moduleVersion

  playbook = (not outdated and savePath and jsonReadFile(savePath .. "/career/"..fileName)) or {}
end

local function onExtensionLoaded()
  if not career_career.isActive() then return false end
  loadDataFromFile()
end

local function onCareerActive(active)
  loadDataFromFile()
end

-- this should only be loaded when the career is active
local function onSaveCurrentSaveSlot(currentSavePath)
  career_saveSystem.jsonWriteFileSafe(currentSavePath .. "/career/"..fileName, playbook, true)
end

M.onExtensionLoaded = onExtensionLoaded
M.onCareerActive = onCareerActive
M.onSaveCurrentSaveSlot = onSaveCurrentSaveSlot
M.onClientStartMission = onClientStartMission
M.addMissionPlayedEntry = addMissionPlayedEntry

return M