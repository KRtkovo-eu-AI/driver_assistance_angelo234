-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactVehiclePerformanceData"
M.moduleActions = {}
M.moduleLookups = {}

local function startRecording(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string", "optional:string", "optional:string", "optional:string", "optional:string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local recordingTypes = {params[1], params[2], params[3], params[4], params[5]}
  extensions.vehiclePerformanceData.startRecording(recordingTypes)
end

local function stopRecording(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string", "optional:string", "optional:string", "optional:string", "optional:string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local recordingTypes = {params[1], params[2], params[3], params[4], params[5]}
  extensions.vehiclePerformanceData.stopRecording(recordingTypes)
end

local function getRecordingData(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string", "optional:string", "optional:string", "optional:string", "optional:string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local recordingTypes = {params[1], params[2], params[3], params[4], params[5]}
  return extensions.vehiclePerformanceData.getRecordingData(recordingTypes)
end

local function getStaticData(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  return extensions.vehiclePerformanceData.getStaticData()
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleActions.startRecording = startRecording
  M.moduleActions.stopRecording = stopRecording
  M.moduleLookups.getRecordingData = getRecordingData
  M.moduleLookups.getStaticData = getStaticData
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
