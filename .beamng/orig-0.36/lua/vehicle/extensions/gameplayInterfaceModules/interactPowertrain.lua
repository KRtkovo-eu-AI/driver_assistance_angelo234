-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactPowertrain"
M.moduleActions = {}
M.moduleLookups = {}

local function getPowertrainDeviceData(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, { "string", "string" })
  if not dataTypeCheck then
    return { failReason = dataTypeError }
  end
  local deviceName = params[1]
  local deviceProperty = params[2]

  local device = powertrain.getDevice(deviceName)
  return { result = device[deviceProperty] }
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleLookups.powertrainDevice = getPowertrainDeviceData
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
