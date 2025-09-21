-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactRecovery"
M.moduleActions = {}
M.moduleLookups = {}

local function interactRecovery(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, { "string" })
  if not dataTypeCheck then
    return { failReason = dataTypeError }
  end
  local recoveryMethod = params[1]
  if not recovery[recoveryMethod] then
    return { failReason = "can't find requested recovery method" }
  end
  recovery[recoveryMethod]()
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleActions.interactRecovery = interactRecovery
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
