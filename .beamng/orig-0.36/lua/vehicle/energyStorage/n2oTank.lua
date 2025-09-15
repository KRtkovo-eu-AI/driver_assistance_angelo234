-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local clamp = clamp
local min = math.min

local function updateGFX(storage, dt)
  storage.remainingMass = storage.storedEnergy / storage.energyDensity
  storage.remainingRatio = storage.energyCapacity > 0 and storage.storedEnergy / storage.energyCapacity or 0

  for k, v in pairs(storage.nodes) do
    obj:setNodeMass(k, v + storage.storedEnergy * storage.nodeMassCoef)
  end

  if storage.currentLeakRate > 0 then
    storage:setRemainingMass(storage.remainingMass - storage.currentLeakRate * dt)
  end
end

local function setRemainingMass(storage, mass)
  storage.storedEnergy = clamp(mass, 0, storage.capacity) * storage.energyDensity
end

local function setRemainingRatio(storage, ratio)
  storage.storedEnergy = storage.energyCapacity * clamp(ratio, 0, 1)
end

local function onBreak(storage)
  storage.currentLeakRate = storage.brokenLeakRate
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

  storage.storedEnergy = integrityState.storedEnergy
  storage.remainingMass = storage.storedEnergy / storage.energyDensity
  storage.remainingRatio = storage.energyCapacity > 0 and storage.storedEnergy / storage.energyCapacity or 0
end

local function getPartCondition(storage)
  local integrityState = {
    storedEnergy = storage.storedEnergy
  }
  local integrityValue = 1

  return integrityValue, integrityState
end

local function reset(storage)
  storage.currentLeakRate = 0
  storage.storedEnergy = storage.startingCapacity * storage.energyDensity
  storage.remainingRatio = storage.energyCapacity > 0 and storage.storedEnergy / storage.energyCapacity or 0

  --apply final weight as soon as possible
  for k, v in pairs(storage.nodes) do
    obj:setNodeMass(k, v + storage.storedEnergy * storage.nodeMassCoef)
  end
end

local function deserialize(storage, data)
  if data.remainingMass then
    storage:setRemainingMass(data.remainingMass)
  end
end

local function serialize(storage)
  return {remainingMass = storage.remainingMass}
end

local function new(jbeamData)
  local storage = {
    name = jbeamData.name,
    type = jbeamData.type,
    assignedDevices = {},
    remainingRatio = 1,
    remainingMass = 1,
    currentLeakRate = 0,
    energyType = "n2o",
    breakTriggerBeam = jbeamData.breakTriggerBeam,
    jbeamData = jbeamData,
    updateGFX = updateGFX,
    setRemainingMass = setRemainingMass,
    setRemainingRatio = setRemainingRatio,
    deserialize = deserialize,
    serialize = serialize,
    registerDevice = registerDevice,
    onBreak = onBreak,
    reset = reset,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition
  }

  storage.energyDensity = 41.5 * 1000000 / 5 --MJ/kg, we use a fuel to N2O ratio of 1:5
  --since we "drain" equal amounts of "energy" from both fuel and N2O tanks, we need have the energy density at one fifth of actual fuel

  storage.energyDensity = jbeamData.energyDensity or storage.energyDensity or 0 --MJ/kg

  storage.capacity = jbeamData.capacity or 0
  storage.startingCapacity = min(jbeamData.startingCapacity or storage.capacity, storage.capacity)
  storage.storedEnergy = storage.startingCapacity * storage.energyDensity
  storage.energyCapacity = storage.capacity * storage.energyDensity

  storage.brokenLeakRate = storage.capacity / 60 --drain tank in 60 seconds, no matter how much is in there

  storage.nodeMassCoef = 0
  storage.nodes = {}
  local nodeCount = 0
  if jbeamData.nodes and jbeamData.nodes._engineGroup_nodes then
    for _, n in pairs(jbeamData.nodes._engineGroup_nodes) do
      storage.nodes[n] = v.data.nodes[n].nodeWeight --save initial mass as the offset for the node weights
      nodeCount = nodeCount + 1
    end
    if nodeCount > 0 and storage.energyDensity > 0 then
      storage.nodeMassCoef = 1 / (storage.energyDensity * nodeCount) --calculate weight per energy left
    end
  end

  --apply final weight as soon as possible
  for k, v in pairs(storage.nodes) do
    obj:setNodeMass(k, v + storage.storedEnergy * storage.nodeMassCoef)
  end

  storage.initialStoredEnergy = storage.startingCapacity * storage.energyDensity
  storage.remainingRatio = storage.energyCapacity > 0 and storage.storedEnergy / storage.energyCapacity or 0

  storage.jbeamData = jbeamData

  return storage
end

M.new = new

return M
