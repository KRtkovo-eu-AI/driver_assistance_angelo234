-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local clamp = clamp

local function setStoredEnergy(storage, energy)
  storage.storedEnergy = energy
  storage.remainingVolume = storage.energyCapacity > 0 and storage.storedEnergy / 3600000 or 0 -- storedEnergy in kWh
  storage.remainingRatio = storage.energyCapacity > 0 and storage.storedEnergy / storage.energyCapacity or 0
end

local function setRemainingRatio(storage, ratio)
  storage.storedEnergy = storage.energyCapacity * clamp(ratio, 0, 1)
end

local function updateGFX(storage, dt)
  storage:setStoredEnergy(storage.storedEnergy)
end

local function registerDevice(storage, device)
  storage.assignedDevices[device.name] = device
end

local function setPartCondition(storage, odometer, integrity, visual)
  local integrityState = integrity
  if type(integrity) == "number" then
    local integrityValue = integrity
    integrityState = {
      storedEnergy = storage.energyCapacity * integrityValue
    }
  end

  storage:setStoredEnergy(integrityState.storedEnergy)
end

local function getPartCondition(storage)
  local integrityState = {
    storedEnergy = storage.storedEnergy
  }
  local integrityValue = 1

  return integrityValue, integrityState
end

local function reset(storage)
  storage:setStoredEnergy(storage.startingCapacity * 3600000) --kWh to J
end

local function new(jbeamData)
  local storage = {
    name = jbeamData.name,
    type = jbeamData.type,
    energyType = "electricEnergy",
    assignedDevices = {},
    remainingRatio = 1,
    reset = reset,
    updateGFX = updateGFX,
    registerDevice = registerDevice,
    setStoredEnergy = setStoredEnergy,
    setRemainingRatio = setRemainingRatio,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition
  }

  storage.capacity = jbeamData.batteryCapacity or 0 --kWh
  storage.startingCapacity = jbeamData.startingCapacity or storage.capacity
  --storage.storedEnergy = storage.startingCapacity * 3600000 --kWh to J
  storage.energyCapacity = storage.capacity * 3600000 --kWh to J
  --storage.remainingVolume = storage.capacity

  storage.initialStoredEnergy = storage.startingCapacity * 3600000 --kWh to J
  --storage.remainingRatio = storage.energyCapacity > 0 and storage.storedEnergy / storage.energyCapacity or 0

  storage.jbeamData = jbeamData

  storage:setStoredEnergy(storage.startingCapacity * 3600000) --kWh to J

  return storage
end

M.new = new

return M
