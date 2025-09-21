-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactFire"
M.moduleActions = {}
M.moduleLookups = {}

local function interactFire(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, { "string" })
  if not dataTypeCheck then
    return { failReason = dataTypeError }
  end
  local fireMethod = params[1]
  if not fire[fireMethod] then
    return { failReason = "can't find requested fire method" }
  end
  fire[fireMethod]()
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleActions.interactFire = interactFire
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
