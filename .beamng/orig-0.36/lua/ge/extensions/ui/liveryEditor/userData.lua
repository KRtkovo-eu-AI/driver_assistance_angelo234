-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- should contain implementation for retrieving save files,
-- saved skins, and recent data (e.g. recently used decals, recently updated save file)
local M = {}

local api = extensions.editor_api_dynamicDecals

local saveDir = 'settings/dynamicDecals/'
local FILENAME_PATTERN = "^[a-zA-Z0-9_-]+$"
local dynDecalsExtension = '.dynDecals.json'

local saveFiles = {}

local getFilename = function(file)
  local _, fn, e = path.split(file)
  return fn:sub(1, #fn - #dynDecalsExtension)
end

local getFileDirectory = function(file)
  local _, fn, e = path.split(file)
end

M.requestUpdatedData = function()
  guihooks.trigger("LiverySaveFilesUpdated", M.getSaveFiles())
end

M.saveFileExists = function(filename)
  local filePath = saveDir .. filename .. dynDecalsExtension
  return FS:fileExists(filePath)
end

M.getSaveFiles = function()
  local queriedFiles = {}
  local files = FS:findFiles(saveDir, '*' .. dynDecalsExtension, -1, false, false)
  for i, file in ipairs(files) do
    local stat = FS:stat(file)

    queriedFiles[i] = {
      name = getFilename(file),
      location = file,
      created = stat.created,
      modified = stat.modtime,
      fileSize = stat.filesize
    }
  end
  return queriedFiles
end

M.createSaveFile = function(filename)
  if not string.match(filename, FILENAME_PATTERN) then
    log("W", "", "Cannot create invalid filename: " .. filename)
    return false
  end

  local path = saveDir .. filename .. dynDecalsExtension
  api.saveLayerStackToFile(path)
  return path
end

M.renameFile = function(filename, newFilename)
  log("D", "", "Renaming file from " .. filename .. " to " .. newFilename)

  local filePath = saveDir .. filename .. dynDecalsExtension

  if not FS:fileExists(filePath) then
    log("W", "", "File " .. (filename or "nil") .. " not found")
    return false
  end

  local newFilePath = saveDir .. newFilename .. dynDecalsExtension
  if FS:renameFile(filePath, newFilePath) == 0 then
    guihooks.trigger("LiverySaveFilesUpdated", M.getSaveFiles())
    return true
  else
    log("W", "", "Unable to rename " .. filename .. " to " .. newFilename)
    return false
  end
end

M.deleteSaveFile = function(filename)
  log("D", "", "Deleting file from " .. filename)

  local filePath = saveDir .. filename .. dynDecalsExtension

  if not FS:fileExists(filePath) then
    log("W", "", "File " .. (filename or "nil") .. " not found")
    return false
  end

  FS:removeFile(filePath)
  guihooks.trigger("LiverySaveFilesUpdated", M.getSaveFiles())
  return true
end

M.getFilename = getFilename

return M
