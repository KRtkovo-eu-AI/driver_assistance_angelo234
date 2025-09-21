-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local tableSize = tableSize
local log = log
local min = math.min

M.nilStorage = {}
local storageFactories = nil
local availableStorageFactories = nil
local energyStorages = {} --keeps track of all available energy storages, also used as LUT
local orderedStorages = {}
local storageCount = 0

local breakTriggerBeams = {} --shaft break beam cache

local function updateGFX(dt)
  for i = 1, storageCount, 1 do
    local storage = orderedStorages[i]
    storage.remainingRatio = storage.initialStoredEnergy > 0 and storage.storedEnergy / storage.initialStoredEnergy or 0
    if storage.updateGFX then
      storage:updateGFX(dt)
    end
  end
end

local function init()
  M.updateGFX = nop

  orderedStorages = {}
  energyStorages = {}
  breakTriggerBeams = {}

  storageCount = 0
  storageFactories = {}

  if not availableStorageFactories then
    availableStorageFactories = {}
    local directory = "lua/vehicle/energyStorage"
    local files = FS:findFiles(directory, "*.lua", -1, true, false)
    if files then
      for _, file in ipairs(files) do
        local _, fileNameWithExt, _ = path.split(file)
        local fileName = fileNameWithExt:sub(1, -5)
        local storageFactoryPath = "energyStorage/" .. fileName
        availableStorageFactories[fileName] = storageFactoryPath
      end
    else
      log("E", "energyStorage.init", "Can't find energyStorage directory: " .. tostring(directory))
    end
  end

  --dump(availableStorageFactories)

  for _, jbeamData in pairs(deepcopy(v.data.energyStorage or {})) do
    tableMergeRecursive(jbeamData, v.data[jbeamData.name] or {})

    if availableStorageFactories[jbeamData.type] and not storageFactories[jbeamData.type] then
      local deviceFactory = require(availableStorageFactories[jbeamData.type])
      storageFactories[jbeamData.type] = deviceFactory
    end

    --load our actual storage via the storage factory
    if storageFactories[jbeamData.type] then
      local storage = storageFactories[jbeamData.type].new(jbeamData)
      energyStorages[storage.name] = storage
    else
      log("E", "energyStorage.init", "Found unknown energyStorage type: " .. jbeamData.type)
      log("E", "energyStorage.init", "EnergyStorage will not init correctly!")
      return
    end
  end

  --dump(storageFactories)
  --dump(energyStorages)

  for _, device in pairs(powertrain.getDevices()) do
    if device.energyStorage then
      if device.energyStorage and type(device.energyStorage) ~= "table" then
        device.energyStorage = {device.energyStorage}
      end
      for _, s in pairs(device.energyStorage) do
        local storage = energyStorages[s]
        if storage then
          device:registerStorage(storage.name)
        end
      end
    end
  end

  for _, storage in pairs(energyStorages) do
    table.insert(orderedStorages, storage)
  end

  storageCount = tableSize(energyStorages)

  local beamTriggers = {}
  for _, storage in pairs(energyStorages) do
    if storage.breakTriggerBeam then
      if type(storage.breakTriggerBeam) ~= "table" then
        storage.breakTriggerBeam = {storage.breakTriggerBeam}
      end
      for _, name in ipairs(storage.breakTriggerBeam) do
        beamTriggers[name] = storage
      end
    end
    damageTracker.setDamage("energyStorage", storage.name, false)
  end

  --dump(beamTriggers)

  for _, v in pairs(v.data.beams or {}) do
    if v.name and v.name ~= "" and beamTriggers[v.name] then
      breakTriggerBeams[v.cid] = beamTriggers[v.name]
    end
  end

  if storageCount > 0 then
    M.updateGFX = updateGFX
  end

  --dump(breakTriggerBeams)

  --Here we create a bit of special magic to deal with fire and forget storage access
  --for example: energyStorage.getStorage("abc").doSomething()
  --in this case an error is thrown if "abc" is not a valid storage.
  --Using energyStorage.getStorageSafe() instead returns a magic table that
  --happily accepts all indexes and can be called without throwing errors (if no real storage is found)
  M.nilStorage = {}
  local mt = {
    __index = function(t, _)
      return t
    end, --return self when indexing
    __call = function(t, ...)
      return t
    end, --return self when being called
    __newindex = function(_, _, _)
    end, --prevent any write access
    __metatable = false --hide metatable to prevent any changes to it
  }
  setmetatable(M.nilStorage, mt)
end

local function reset()
  for _, v in pairs(orderedStorages) do
    if v.reset then
      v:reset()
    end
    damageTracker.setDamage("energyStorage", v.name, false)
  end

  for _, device in pairs(powertrain.getDevices()) do
    if device.energyStorage then
      if device.energyStorage and type(device.energyStorage) ~= "table" then
        device.energyStorage = {device.energyStorage}
      end
      for _, s in pairs(device.energyStorage) do
        local storage = energyStorages[s]
        if storage then
          device:registerStorage(storage.name)
        end
      end
    end
  end
end

local function beamBroke(id)
  if not breakTriggerBeams[id] then
    return
  end

  local storage = breakTriggerBeams[id]
  if storage.onBreak then
    storage:onBreak()
  end

  damageTracker.setDamage("energyStorage", storage.name, true)
end

local function onCouplerFound(nodeId, obj2id, obj2nodeId, nodeDist)
  for i = 1, storageCount, 1 do
    local storage = orderedStorages[i]
    if storage.onCouplerFound then
      storage:onCouplerFound(nodeId, obj2id, obj2nodeId, nodeDist)
    end
  end
end

local function onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  for i = 1, storageCount, 1 do
    local storage = orderedStorages[i]
    if storage.onCouplerAttached then
      storage:onCouplerAttached(nodeId, obj2id, obj2nodeId, attachEnergy)
    end
  end
end

local function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  for i = 1, storageCount, 1 do
    local storage = orderedStorages[i]
    if storage.onCouplerDetached then
      storage:onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
    end
  end
end

local function getStorages()
  return energyStorages
end

local function getStorage(name)
  return energyStorages[name]
end

local function getStorageSafe(name)
  local storage = energyStorages[name]
  if storage then
    return storage
  else
    log("D", "energyStorage.getStorageSafe", string.format("Didn't find storage '%s', returning nilStorage.", name))
    --log("D", "energyStorage.getStorageSafe", debug.traceback())
    --return our nilStorage that accepts all indexes and can be called without errors
    return M.nilStorage
  end
end

local function getPartRelevantStorages(partTypeData)
  local relevantStorages = {}

  for _, partType in ipairs(partTypeData or {}) do
    local split = split(partType, ":")
    if split[1] == "energyStorage" then
      local storageName = split[2]
      table.insert(relevantStorages, storageName)
    end
  end
  return relevantStorages
end

local function setPartCondition(partTypeData, odometer, integrity, visual)
  local storageIntegrity = integrity

  local relevantStorages = getPartRelevantStorages(partTypeData)
  for _, relevantStorage in ipairs(relevantStorages) do
    local storage = M.getStorage(relevantStorage)
    if storage and storage.setPartCondition then
      if type(integrity) == "table" and integrity.energyStorage then
        storageIntegrity = integrity.energyStorage[storage.name]
      end
      storage:setPartCondition(odometer, storageIntegrity, visual)
    end
  end
end

local function getPartCondition(partTypeData)
  local relevantStorages = getPartRelevantStorages(partTypeData)
  local canProvideIntegrityCondition = false
  local canProvideVisualCondition = false
  local energyStorageCondition = {integrityValue = 1, integrityState = {}, visualValue = 1, visualState = {}}

  for _, relevantStorage in ipairs(relevantStorages) do
    local storage = M.getStorage(relevantStorage)
    if storage and storage.getPartCondition then
      local storageIntegrityValue, storageIntegrityState = storage:getPartCondition()
      energyStorageCondition.integrityState[relevantStorage] = storageIntegrityState
      energyStorageCondition.integrityValue = min(energyStorageCondition.integrityValue, storageIntegrityValue)

      canProvideIntegrityCondition = true
    end
  end

  return energyStorageCondition, canProvideIntegrityCondition, canProvideVisualCondition
end

local function onDeserialize(data)
  --  if not data or type(data) ~= "table" then
  --    return
  --  end
  --  for name, storageData in pairs(data) do
  --    if name and energyStorages[name] then
  --      energyStorages[name]:deserialize(storageData)
  --    end
  --  end
end

local function onSerialize()
  local data = {}
  for _, storage in pairs(energyStorages) do
    if storage.serialize then
      data[storage.name] = storage:serialize()
    end
  end
  return data
end

M.init = init
M.reset = reset
M.updateGFX = nop

M.onDeserialize = onDeserialize
M.onSerialize = onSerialize

M.beamBroke = beamBroke

M.onCouplerFound = onCouplerFound
M.onCouplerAttached = onCouplerAttached
M.onCouplerDetached = onCouplerDetached

M.getStorages = getStorages
M.getStorage = getStorage
M.getStorageSafe = getStorageSafe

M.getPartCondition = getPartCondition
M.setPartCondition = setPartCondition

return M
