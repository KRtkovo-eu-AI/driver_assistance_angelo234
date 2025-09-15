-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactController"
M.moduleActions = {}
M.moduleLookups = {}

local function controllerGameplayEvent(params)
  local eventName = params[1]
  local eventParams = {unpack(params, 2)}
  controller.onGameplayEvent(eventName, eventParams)
end

local function setFreeze(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"boolean"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local freeze = params[1]
  controller.setFreeze(freeze and 1 or 0)
end

local function setGearboxMode(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local mode = params[1]
  controller.mainController.setGearboxMode(mode)
end

local function shiftToGearIndex(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"number"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local gearIndex = params[1]
  controller.mainController.shiftToGearIndex(gearIndex)
end

local function getMainControllerData(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local dataKey = params[1]
  if dataKey == "gearboxMode" then
    return {result = electrics.values.gearboxMode}
  elseif dataKey == "freezeState" then
    return {result = electrics.values.freezeState}
  end
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleActions.controllerGameplayEvent = controllerGameplayEvent
  M.moduleActions.setFreeze = setFreeze
  M.moduleActions.setGearboxMode = setGearboxMode
  M.moduleActions.shiftToGearIndex = shiftToGearIndex
  M.moduleLookups.mainController = getMainControllerData
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
