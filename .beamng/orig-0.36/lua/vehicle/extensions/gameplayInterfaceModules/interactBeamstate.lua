-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactBeamstate"
M.moduleActions = {}
M.moduleLookups = {}

local function interactBeamstate(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local beamstateMethod = params[1]
  if not beamstate[beamstateMethod] then
    return {failReason = "can't find requested beamstate method"}
  end
  beamstate[beamstateMethod]()
end

local function getBeamstateCouplerOffset(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local couplerTag = params[1]
  local couplerOffset = beamstate.getCouplerOffset(couplerTag)
  return {result = couplerOffset}
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleActions.interactBeamstate = interactBeamstate
  M.moduleLookups.couplerOffset = getBeamstateCouplerOffset
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
