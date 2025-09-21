-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logFileName = "career.log"
local logList = {}

M.dependencies = {"career_saveSystem"}

local function addLog(message, origin, severity)
  table.insert(logList, string.format("%d|%s|%s|%s", os.time(), severity or "I", origin or "", message))
end

local function onSaveCurrentSaveSlot(currentSavePath)
  local saveRoot = career_saveSystem.getSaveRootDirectory()
  local saveSlot, _ = career_saveSystem.getCurrentSaveSlot()
  if not saveSlot then return end

  addLog(string.format("Save game to %s", currentSavePath), "log")

  local f = io.open(saveRoot .. saveSlot .. "/" .. logFileName, "a")
  if not f then return end

  f:write(table.concat(logList, "\n") .. "\n")
  f:close()
  logList = {}
end

local function onCareerModulesActivated()
  logList = {}
  local saveSlot, savePath = career_saveSystem.getCurrentSaveSlot()
  addLog(string.format("Loaded game %s", savePath), "log")
end

M.addLog = addLog

M.onSaveCurrentSaveSlot = onSaveCurrentSaveSlot
M.onCareerModulesActivated = onCareerModulesActivated

return M