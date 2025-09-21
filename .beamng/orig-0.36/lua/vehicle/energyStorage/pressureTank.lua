-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local constants = {
  gasConstant = 8.314 -- J/(mol * K)
}

local abs = math.abs
local max = math.max
local min = math.min
local sqrt = math.sqrt

local function updateTankSync(storage, dt)
  --consumer side---
  if storage.pneumaticPTOSupplyPressureElectricsName and storage.pneumaticPTOSupplyFlowElectricsName then
    --print("consumer")
    local supplyPressure = electrics.values[storage.pneumaticPTOSupplyPressureElectricsName] or 0 --supply pressure from our supply (which can be another consumer, but we don't care)

    storage.supplyPressure = supplyPressure
    local energyFlowRate = 0
    --only sync if we have an actual usable pressure, if it's 0 then something is either wrong or or existing supply pressure is isolated from its tank
    if supplyPressure > 1 then
      local pressureDiff = supplyPressure - storage.currentPressure
      local ptoFlowRateIn = pressureDiff * storage.consumerPressureDiffFlowRateCoef --adjusts how much flowrate results from a given pressure diff, this affects simulation stability
      local airVolumeIn = ptoFlowRateIn * dt
      local airEnergyIn = abs(pressureDiff) * airVolumeIn
      --print(string.format("Supply: %.2f, local: %.2f, diff: %.2f", supplyPressure, storage.currentPressure, pressureDiff))
      storage.storedEnergy = max(0, storage.storedEnergy + airEnergyIn)
      energyFlowRate = airEnergyIn / dt
    end

    electrics.values[storage.pneumaticPTOSupplyFlowElectricsName] = energyFlowRate --report flow to the relevant supply
  end
  ------

  if storage.isPrimarySupply then
    storage.supplyPressure = storage.currentPressure
  end

  ---supply side---
  --send our own consumer pressure to further potential consumers of our own down the line
  --don't send more than our own supply pressure so that if a tank loses supply, this will propagate to the next consumer
  -- for the initial supply with a compressor, current and supply are the same, so it doesn't matter there

  local consumerPressure = min(storage.currentPressure, storage.supplyPressure) * storage.consumerPressureCoef
  electrics.values[storage.pneumaticPTOConsumerPressureElectricsName] = consumerPressure --this is for the actual sync with our consumer (it includes the vehicle specific electrics bit)
  electrics.values[storage.pressureConsumerElectricName] = consumerPressure --this is for exposure in our own vehicle for props and such (actual pressure sent to cosnumers)
  electrics.values[storage.pressureConsumerCoefElectricName] = storage.consumerPressureCoef --this is for exposure in our own vehicle for props and such (pressure COEF)
  local ptoEnergyFlowRateOut = electrics.values[storage.pneumaticPTOConsumerFlowElectricsName] or 0
  local airEnergyOut = ptoEnergyFlowRateOut * dt
  storage.storedEnergy = max(0, storage.storedEnergy - airEnergyOut)
  ------
end

local function updateGFX(storage, dt)
  -- calculate pressure from stored energy and capacity: PV = nRT, e = nRT, therefore PV = e and P = e / V
  storage.currentPressure = storage.storedEnergy * storage.invCapacity

  --depending on what role this tank has, update the relevant coupler electronics for the pneumatic PTO
  storage:updateTankSync(dt)

  if storage.damageDeformGroup then
    local currentTankDeformDamage = beamstate.deformGroupDamage[storage.damageDeformGroup] and beamstate.deformGroupDamage[storage.damageDeformGroup].damage or 0
    storage.currentLeakRate = clamp(linearScale(currentTankDeformDamage, 0, 1, 0, storage.maxLeakRate), storage.currentLeakRate, storage.maxLeakRate)
  end

  if storage.currentLeakRate > storage.previousLeakRate then
    storage.previousLeakRate = storage.currentLeakRate
    damageTracker.setDamage("pressureTank", "deformationLeak", true, true)
  end

  --only leak if something is actually broken and if our current pressure is > ambient
  if storage.currentLeakRate > 0 and storage.currentPressure > powertrain.currentEnvPressure then
    -- Leak rate limited by speed of sound when absolute pressure is greater than twice env pressure (i.e. relative pressure is greater than env pressure)
    local leakRateCoef = min(1, storage.currentPressure * powertrain.invCurrentEnvPressure)
    local effectiveLeakRate = storage.currentLeakRate * sqrt(leakRateCoef)

    local volumeLeaked = effectiveLeakRate * dt
    local energyLeaked = storage.currentPressure * volumeLeaked

    storage.storedEnergy = max(0, storage.storedEnergy - energyLeaked)
  end

  storage:updateEnergy()

  for k, v in pairs(storage.nodes) do
    obj:setNodeMass(k, v + storage.remainingMass * storage.nodeMassCoef)
  end

  electrics.values[storage.pressureElectricName] = storage.currentPressure - powertrain.currentEnvPressure

  if storage.showDebugGraph then
    local supplyPressure = storage.pneumaticPTOSupplyPressureElectricsName and electrics.values[storage.pneumaticPTOSupplyPressureElectricsName] or 0
    guihooks.graph({"Tank Pressure", storage.currentPressure, 1000000, ""}, {"Supply Pressure", supplyPressure, 1000000, ""})
  end
end

local function updateEnergy(storage, molarQuantity)
  if not molarQuantity then
    -- if molar quantity is not provided, compute the amount of gas in the tank as per the last update (using previous env temperature)
    -- moles (n) = PV / RT; PV = energy, therefore n = e / RT
    molarQuantity = storage.storedEnergy / (constants.gasConstant * storage.prevEnvTemperature)
  end

  -- update new energy from mass and CURRENT env temperature: e = n * R * T
  storage.storedEnergy = molarQuantity * constants.gasConstant * powertrain.currentEnvTemperature
  storage.remainingMass = molarQuantity * storage.gasMolarMass
  storage.remainingRatio = storage.energyCapacity > 0 and storage.storedEnergy / storage.energyCapacity or 0
  storage.prevEnvPressure = powertrain.currentEnvPressure
  storage.prevEnvTemperature = powertrain.currentEnvTemperature
end

local function onCouplerAttached(storage, nodeId, obj2id, obj2nodeId, attachForce)
  --if the attached node is our supply node (ie the one where WE connect to OUR supply) and we act as consumer
  if storage.pneumaticPTOSupplyCouplerNodeLookup[nodeId] then
    --print("we are consumer")
    --we now need to create the correct electrics names to sync with our supply consiting of the agreed upon base name and the supply's objId
    storage.pneumaticPTOSupplyPressureElectricsName = storage.pneumaticPTOPressureBaseElectricsName .. "_" .. obj2id
    storage.pneumaticPTOSupplyFlowElectricsName = storage.pneumaticPTOFlowBaseElectricsName .. "_" .. obj2id

    --initialize both pressure and flow to 0, flow will then be updated by us and pressure by the supply
    electrics.values[storage.pneumaticPTOSupplyPressureElectricsName] = 0
    electrics.values[storage.pneumaticPTOSupplyFlowElectricsName] = 0
    --print(storage.pneumaticPTOSupplyPressureElectricsName)
    --print(storage.pneumaticPTOSupplyFlowElectricsName)
    beamstate.sendExportCouplerData(obj2id, obj2nodeId, {electrics = {storage.pneumaticPTOSupplyPressureElectricsName}})
  end

  --if we just attached to our own consumer (ie we act as supply)
  if storage.pneumaticPTOConsumerCouplerNodeLookup[nodeId] then
    --print("we are supply")
    --initialize the reported pressure and flow to 0, flow is then constantly updated from the consumer via the coupler and pressure by us
    electrics.values[storage.pneumaticPTOConsumerPressureElectricsName] = 0
    electrics.values[storage.pneumaticPTOConsumerFlowElectricsName] = 0
    beamstate.sendExportCouplerData(obj2id, obj2nodeId, {electrics = {storage.pneumaticPTOConsumerFlowElectricsName}})
  end
end

local function onCouplerDetached(storage, nodeId, obj2id, obj2nodeId, breakForce)
  --if the decoupled node is our supply node (ie the one where WE connect to OUR supply)
  if storage.pneumaticPTOSupplyCouplerNodeLookup[nodeId] then
    storage.supplyPressure = 0
    --if we had any stored supply electric names
    if storage.pneumaticPTOSupplyPressureElectricsName then
      --kill the value of the electric
      electrics.values[storage.pneumaticPTOSupplyPressureElectricsName] = nil
      --and kill the supply specific name
      storage.pneumaticPTOSupplyPressureElectricsName = nil
    end
    if storage.pneumaticPTOSupplyFlowElectricsName then
      electrics.values[storage.pneumaticPTOSupplyFlowElectricsName] = nil
      storage.pneumaticPTOSupplyFlowElectricsName = nil
    end
  end

  --if we just lost our own consumer
  if storage.pneumaticPTOConsumerCouplerNodeLookup[nodeId] then
    --kill the flow value from it in our system
    electrics.values[storage.pneumaticPTOConsumerFlowElectricsName] = nil
  end
end

local function getRemainingMass(storage)
  return storage.remainingMass
end

local function setRemainingMass(storage, mass)
  -- convert mass to moles and then moles to energy
  -- moles = mass / molarMass
  -- e = nRT
  local molarQuantity = mass * storage.invGasMolarMass

  storage:updateEnergy(molarQuantity)
end

local function setRemainingRatio(storage, ratio)
  storage.remainingRatio = min(max(ratio, 0), 1)
  storage.storedEnergy = storage.energyCapacity * storage.remainingRatio
  storage:updateEnergy()
end

local function setConsumerPressureCoef(storage, coef)
  storage.consumerPressureCoef = clamp(coef, 0, 1)
  guihooks.message("vehicle.pneumatics.trailerAir." .. (coef > 0 and "enabled" or "disabled"), 3, "vehicle.pneumatics.trailerAir")
end

local function toggleConsumerPressureCoef(storage)
  storage:setConsumerPressureCoef(storage.consumerPressureCoef > 0 and 0 or 1)
end

local function onBreak(storage)
  storage.currentLeakRate = storage.maxLeakRate
  damageTracker.setDamage("pressureTank", "breakLeak", true, true)
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

  storage.storedEnergy = integrityState.storedEnergy or storage.storedEnergy
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
  storage.previousLeakRate = 0
  storage.consumerPressureCoef = 1

  storage.storedEnergy = storage.initialStoredEnergy
  storage:updateEnergy()

  storage.currentPressure = storage.storedEnergy * storage.invCapacity
  storage.supplyPressure = 0

  --apply final weight as soon as possible
  for k, v in pairs(storage.nodes) do
    obj:setNodeMass(k, v + storage.remainingMass * storage.nodeMassCoef)
  end

  electrics.values[storage.pressureElectricName] = storage.currentPressure

  --reset all the supply/consumer electrics values
  electrics.values[storage.pneumaticPTOConsumerPressureElectricsName] = 0
  electrics.values[storage.pneumaticPTOConsumerFlowElectricsName] = 0
  if storage.pneumaticPTOSupplyPressureElectricsName then
    electrics.values[storage.pneumaticPTOSupplyPressureElectricsName] = nil
    storage.pneumaticPTOSupplyPressureElectricsName = nil
  end
  if storage.pneumaticPTOSupplyFlowElectricsName then
    electrics.values[storage.pneumaticPTOSupplyFlowElectricsName] = nil
    storage.pneumaticPTOSupplyFlowElectricsName = nil
  end

  damageTracker.setDamage("pressureTank", "breakLeak", false, false)
  damageTracker.setDamage("pressureTank", "deformationLeak", false, false)
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
    energyType = jbeamData.energyType or "air",
    gasMolarMass = jbeamData.gasMolarMass or 28.9647, -- g/mol (default value is the approximate molar mass of air)
    assignedDevices = {},
    remainingRatio = 1,
    remainingMass = 1,
    currentLeakRate = 0,
    consumerPressureCoef = 1,
    breakTriggerBeam = jbeamData.breakTriggerBeam,
    showDebugGraph = jbeamData.showDebugGraph or false,
    jbeamData = jbeamData,
    updateGFX = updateGFX,
    updateEnergy = updateEnergy,
    updateTankSync = updateTankSync,
    getRemainingMass = getRemainingMass,
    setRemainingMass = setRemainingMass,
    setRemainingRatio = setRemainingRatio,
    setConsumerPressureCoef = setConsumerPressureCoef,
    toggleConsumerPressureCoef = toggleConsumerPressureCoef,
    deserialize = deserialize,
    serialize = serialize,
    registerDevice = registerDevice,
    onBreak = onBreak,
    onCouplerAttached = onCouplerAttached,
    onCouplerDetached = onCouplerDetached,
    reset = reset,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition
  }

  storage.capacity = jbeamData.capacity or 0 -- volume, m^3
  storage.invCapacity = storage.capacity > 0 and 1 / storage.capacity or 0
  storage.invGasMolarMass = storage.gasMolarMass > 0 and 1 / storage.gasMolarMass or 0
  storage.maxWorkingPressure = jbeamData.maxPressure or 1000000 -- 1,000 kPa or roughly 145 psi
  storage.startingPressure = min((jbeamData.startingPressure or 0) + powertrain.currentEnvPressure, storage.maxWorkingPressure)
  storage.prevEnvPressure = powertrain.currentEnvPressure
  storage.prevEnvTemperature = powertrain.currentEnvTemperature

  -- calculate stored energy: E = P * V
  storage.energyCapacity = storage.maxWorkingPressure * storage.capacity
  storage.storedEnergy = storage.startingPressure * storage.capacity
  storage.initialStoredEnergy = storage.storedEnergy
  storage.remainingRatio = storage.energyCapacity > 0 and storage.storedEnergy / storage.energyCapacity or 0
  storage.currentPressure = storage.storedEnergy * storage.invCapacity
  storage.supplyPressure = 0
  storage.isPrimarySupply = jbeamData.isPrimarySupply or false

  -- calculate mass from energy and temperature: m = (P * V * Ma) / (R * T) ; E = P * V therefore m = (E * Ma) / (R * T)
  local invTempCoefficient = 1 / (constants.gasConstant * powertrain.currentEnvTemperature)
  storage.remainingMass = storage.storedEnergy * storage.gasMolarMass * invTempCoefficient
  storage.maxStoredMass = storage.energyCapacity * storage.gasMolarMass * invTempCoefficient

  storage.maxLeakRate = storage.capacity / 3 --drain tank in 5 seconds, no matter how much is in there

  storage.nodeMassCoef = 0
  storage.nodes = {}
  local nodeCount = 0
  if jbeamData.nodes and jbeamData.nodes._engineGroup_nodes then
    for _, n in pairs(jbeamData.nodes._engineGroup_nodes) do
      storage.nodes[n] = v.data.nodes[n].nodeWeight --save initial mass as the offset for the node weights
      nodeCount = nodeCount + 1
    end
    if nodeCount > 0 then
      storage.nodeMassCoef = 1 / nodeCount
    end
  end

  --apply final weight as soon as possible
  for k, v in pairs(storage.nodes) do
    obj:setNodeMass(k, v + storage.storedEnergy * storage.nodeMassCoef)
  end

  --base electrics name that needs to match between vehicles for a connection to work
  --one could make multiple pressure systems using different names
  storage.pneumaticPTOPressureBaseElectricsName = jbeamData.pneumaticPTOPressureBaseSyncName or "pneumaticPTOPressure"
  storage.pneumaticPTOFlowBaseElectricsName = jbeamData.pneumaticPTOFlowBaseSyncName or "pneumaticPTOFlow"
  --these are the electrics names for communication with our CONSUMER
  --TODO this only supports a single consumer, to fix this, the electrics name needs to consist of both our id and the consumer id...
  storage.pneumaticPTOConsumerPressureElectricsName = storage.pneumaticPTOPressureBaseElectricsName .. "_" .. objectId
  storage.pneumaticPTOConsumerFlowElectricsName = storage.pneumaticPTOFlowBaseElectricsName .. "_" .. objectId

  --these are the electrics names for communication with our SUPPLY, they are generated upon coupling
  --TODO this only supports a single consumer for our supply, to fix this, the electrics name needs to consist of both the supply id and our consumer id...
  storage.pneumaticPTOSupplyPressureElectricsName = nil --received via electrics from supply vehicle
  storage.pneumaticPTOSupplyFlowElectricsName = nil --received via electrics from supply vehicle

  --used for looking up if a given coupler belongs to us
  storage.pneumaticPTOSupplyCouplerNodeLookup = {}
  local ptoSupplyNodeNames = jbeamData.pneumaticPTOSupplyCouplerNodeNames or {}
  for _, nodeName in pairs(ptoSupplyNodeNames) do
    if beamstate.nodeNameMap[nodeName] then
      storage.pneumaticPTOSupplyCouplerNodeLookup[beamstate.nodeNameMap[nodeName]] = true
    end
  end
  --used for looking up if a given coupler belongs to us
  storage.pneumaticPTOConsumerCouplerNodeLookup = {}
  local ptoConsumerNodeNames = jbeamData.pneumaticPTOConsumerCouplerNodeNames or {}
  for _, nodeName in pairs(ptoConsumerNodeNames) do
    if beamstate.nodeNameMap[nodeName] then
      storage.pneumaticPTOConsumerCouplerNodeLookup[beamstate.nodeNameMap[nodeName]] = true
    end
  end

  --how quickly a pressure difference between local and supply pressure is equalized, this has stability implications as it a P factor
  storage.consumerPressureDiffFlowRateCoef = jbeamData.consumerPressureDiffFlowRateCoef or 0.000001

  storage.pressureElectricName = storage.name .. "_pressureRelative"
  storage.pressureConsumerElectricName = storage.name .. "_pressureConsumer"
  storage.pressureConsumerCoefElectricName = storage.name .. "_pressureConsumerCoef"
  storage.damageDeformGroup = jbeamData.tankDamageDeformGroup
  storage.previousLeakRate = 0

  electrics.values[storage.pressureElectricName] = storage.currentPressure

  electrics.values[storage.pneumaticPTOConsumerPressureElectricsName] = 0
  electrics.values[storage.pneumaticPTOConsumerFlowElectricsName] = 0

  damageTracker.setDamage("pressureTank", "breakLeak", false, false)
  damageTracker.setDamage("pressureTank", "deformationLeak", false, false)

  storage.jbeamData = jbeamData

  return storage
end

M.new = new

return M
