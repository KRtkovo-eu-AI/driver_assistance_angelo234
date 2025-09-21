-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local lastUsedFolder = "/"

local function loadPresets()
  local callback = function(data)
    local filepath = data.filepath
    if filepath == nil or filepath == "" then return end
    log('I','postfx','load preset callback.....'..dumps(data))
    lastUsedFolder = data.path
    postFxModule.loadPresetFile(data.filepath)
  end

  editor_fileDialog.openFile(callback, {{"Post Effect Settings", ".postfx"}}, false, lastUsedFolder)
end

local function savePresets()
  local callback = function(data)
    local filepath = data.filepath
    if filepath == nil or filepath == "" then return end
    log('I','postfx','save preset callback.....'..dumps(data))
    lastUsedFolder = data.path
    postFxModule.savePresetFile(data.filepath)
  end

  editor_fileDialog.saveFile(callback, {{"Post Effect Settings", ".postfx"}}, false, lastUsedFolder)
end

M.loadPresets = loadPresets
M.savePresets = savePresets

return M
