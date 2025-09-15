-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactElectrics"
M.moduleActions = {}
M.moduleLookups = {}

local function setIgnitionLevel(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"number"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local ignitionLevel = params[1]
  electrics.setIgnitionLevel(ignitionLevel)
end

local function setLightbarMode(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"number"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local mode = params[1]
  electrics.set_lightbar_signal(mode)
end

local function setLightMode(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"number"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local mode = params[1]
  electrics.setLightsState(mode)
end

local function getElectricsData(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end

  local key = params[1]
  return {result = electrics.values[key]}
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleActions.setIgnitionLevel = setIgnitionLevel
  M.moduleActions.setLightbarMode = setLightbarMode
  M.moduleActions.setLightMode = setLightMode
  M.moduleLookups.electricsValue = getElectricsData
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
