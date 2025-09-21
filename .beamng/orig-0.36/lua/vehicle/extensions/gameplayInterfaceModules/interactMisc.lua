-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactMisc"
M.moduleActions = {}
M.moduleLookups = {}

local function ping(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  return {ping = true}
end

local function latchesOpen()
  for k, v in pairs(controller.getControllersByType("advancedCouplerControl")) do
    v.detachGroup()
  end
end

local function latchesClose()
  for k, v in pairs(controller.getControllersByType("advancedCouplerControl")) do
    v.tryAttachGroupImpulse()
  end
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleActions.latchesOpen = latchesOpen
  M.moduleActions.latchesClose = latchesClose
  M.moduleLookups.ping = ping
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
