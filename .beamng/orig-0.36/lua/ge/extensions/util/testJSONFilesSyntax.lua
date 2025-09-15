-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local jsonDebug = require('jsonDebug')

local function jsonDebugDecode(content, context)
  local state, data, warnings = xpcall(function() return jsonDebug.decode(content, context) end, debug.traceback)
  if state == false then
    log('E', "jsonDecode", "unable to decode JSON: "..tostring(context))
    log('E', "jsonDecode", "JSON decoding error: "..tostring(data))
    return nil
  end
  for _, warning in ipairs(warnings) do
    log('W', 'jsonDecode', warning)
  end
  return data
end

local function onExtensionLoaded()
  local filePaths = FS:findFiles('/', "*.jbeam\t*.pc\t*.json", -1, true, false)
  for _, filePath in ipairs(filePaths) do
    local dir, fileName, _ = path.splitWithoutExt(filePath)
    local fileText = readFile(filePath)
    local data = jsonDebugDecode(fileText, filePath)
  end
end

M.onExtensionLoaded = onExtensionLoaded

return M