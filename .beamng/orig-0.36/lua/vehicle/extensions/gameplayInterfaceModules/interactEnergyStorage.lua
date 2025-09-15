-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactEnergyStorage"
M.moduleActions = {}
M.moduleLookups = {}

local function setEnergyStorageEnergy(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, { "string", "number" })
  if not dataTypeCheck then
    return { failReason = dataTypeError }
  end
  local storageName = params[1]
  local energy = params[2]
  local storage = energyStorage.getStorage(storageName)
  if not storage then
    return { failReason = "can't find requested storage" }
  end
  storage.storedEnergy = energy
end

local function getEnergyStorageData(params)
  local storages = energyStorage.getStorages()
  local result = {}
  for name, storage in pairs(storages) do
    table.insert(result, {
      name = name,
      storageType = storage.type,
      energyType = storage.energyType,
      currentEnergy = storage.storedEnergy,
      maxEnergy = storage.energyCapacity
    })
  end

  return { result }
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleActions.setEnergyStorageEnergy = setEnergyStorageEnergy
  M.moduleLookups.energyStorage = getEnergyStorageData
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
