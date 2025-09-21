-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactPartCondition"
M.moduleActions = {}
M.moduleLookups = {}

local function initConditions(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"optional:table", "optional:number", "optional:number", "optional:number", "optional:table"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local conditions = params[1]
  local fallbackOdometer = params[2]
  local fallbackIntegrityValue = params[3]
  local fallbackVisualValue = params[4]
  local defaultPaints = params[5]
  partCondition.initConditions(conditions, fallbackOdometer, fallbackIntegrityValue, fallbackVisualValue, defaultPaints)
end

local function ensureConditionsInit(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"number", "number", "number"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local fallbackOdometer = params[1]
  local fallbackIntegrityValue = params[2]
  local fallbackVisualValue = params[3]
  partCondition.ensureConditionsInit(fallbackOdometer, fallbackIntegrityValue, fallbackVisualValue)
end

local function getConditions(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  return {result = partCondition.getConditions()}
end

local function createConditionSnapshot(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local snapshotKey = params[1]
  partCondition.createConditionSnapshot(snapshotKey)
end

local function applyConditionSnapshot(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local snapshotKey = params[1]
  partCondition.applyConditionSnapshot(snapshotKey)
end

local function deleteConditionSnapshots(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  partCondition.deleteConditionSnapshots()
end

local function setResetSnapshotKey(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local snapshotKey = params[1]
  partCondition.setResetSnapshotKey(snapshotKey)
end

local function createAndSetPartConditionResetSnapshotKey(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  local snapshotKey = params[1]
  partCondition.createConditionSnapshot(snapshotKey)
  partCondition.setResetSnapshotKey(snapshotKey)
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleLookups.getPartConditions = getConditions
  M.moduleActions.initPartConditions = initConditions
  M.moduleActions.ensurePartConditionsInit = ensureConditionsInit
  M.moduleActions.createPartConditionSnapshot = createConditionSnapshot
  M.moduleActions.applyPartConditionSnapshot = applyConditionSnapshot
  M.moduleActions.deletePartConditionSnapshots = deleteConditionSnapshots
  M.moduleActions.setPartConditionResetSnapshotKey = setResetSnapshotKey
  M.moduleActions.createAndSetPartConditionResetSnapshotKey = createAndSetPartConditionResetSnapshotKey
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
