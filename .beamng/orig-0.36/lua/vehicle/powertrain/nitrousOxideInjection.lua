-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local floor = math.floor
local max = math.max
local min = math.min
local abs = math.abs

local rpmToAV = 0.104719755
local avToRPM = 9.5493

M.isExisting = true
M.isArmed = false
M.isActive = false

local assignedEngine = nil
local nitrousOxideTorqueLookup = nil
local nitrousOxideOverrideTorqueLookup = nil
local noArmName = nil
local noOverrideName = nil
local noActiveName = nil
local minimumGear = nil
local volumeCoef = nil

local isArmed = false
local manualOverride = false
local cutInRPM = nil

local purgeActiveTime = 0
local purgeParticleTick = 0
local purgeValveNodes = nil
local purgeSounds = {}
local purgeSoundActive
local purgeEvent = "event:>Vehicle>Nitrous_Purging"
local purgeVolume = 1

local storageWithEnergyCounter = 0
local registeredEnergyStorages = {}
local previousEnergyLevels = {}
local hasLiquid = true
local energyStorageRatios = {}

local function updateSounds(dt)
end

local function purgeLines(purgeTime)
  purgeActiveTime = (#purgeValveNodes > 0 and hasLiquid) and purgeTime or 0
end

local function getTankRatio()
  local ratio = 0
  local counter = 0
  for _, s in pairs(registeredEnergyStorages) do
    local storage = energyStorage.getStorage(s)
    if storage then
      ratio = ratio + storage.remainingRatio
      counter = counter + 1
    end
  end
  ratio = counter > 0 and ratio / counter or 0
  return ratio
end

local function updateEnergyStorageRatios()
  for _, s in pairs(registeredEnergyStorages) do
    local storage = energyStorage.getStorage(s)
    if storage then
      if storage.storedEnergy > 0 then
        energyStorageRatios[storage.name] = 1 / storageWithEnergyCounter
      else
        energyStorageRatios[storage.name] = 0
      end
    end
  end
end

local function updateFuelUsage()
  if not isArmed and not manualOverride then
    return
  end

  local hasLiquidTmp = false
  local previousTankCount = storageWithEnergyCounter
  for _, s in pairs(registeredEnergyStorages) do
    local storage = energyStorage.getStorage(s)
    if storage then
      local previous = previousEnergyLevels[storage.name]
      storage.storedEnergy = max(storage.storedEnergy - (assignedEngine.spentEnergyNitrousOxide * energyStorageRatios[storage.name]), 0)
      if previous > 0 and storage.storedEnergy <= 0 then
        storageWithEnergyCounter = storageWithEnergyCounter - 1
      elseif previous <= 0 and storage.storedEnergy > 0 then
        storageWithEnergyCounter = storageWithEnergyCounter + 1
      end
      previousEnergyLevels[storage.name] = storage.storedEnergy
    end

    hasLiquidTmp = hasLiquidTmp or (storage and storage.storedEnergy > 0 or false)
  end
  if previousTankCount ~= storageWithEnergyCounter then
    updateEnergyStorageRatios()
  end

  hasLiquid = hasLiquidTmp
end

local function updateGFX(dt)
  if assignedEngine.engineDisabled then
    M.updateGFX = nop
    M.isArmed = false
    M.isActive = false
    return
  end

  updateFuelUsage()

  local purgeActive = purgeActiveTime > 0
  if purgeActive then
    purgeParticleTick = purgeParticleTick + dt
    if purgeParticleTick > 0.02 then
      for _, v in ipairs(purgeValveNodes) do
        obj:addParticleByNodesRelative(v.cid1, v.cid2, -2, 70, 0, 1)
        obj:addParticleByNodesRelative(v.cid1, v.cid2, -4, 71, 0, 1)
        obj:addParticleByNodesRelative(v.cid1, v.cid2, -8, 72, 0, 1)
      end
      purgeParticleTick = 0
    end
    if not purgeSoundActive then
      for k, v in ipairs(purgeValveNodes) do
        purgeSounds[k] = purgeSounds[k] or obj:createSFXSource2(purgeEvent, "AudioDefaultLoop3D", "nitrousPurge", v.cid1, 0)
        obj:setVolume(purgeSounds[k], purgeVolume)
        obj:cutSFX(purgeSounds[k])
        obj:playSFX(purgeSounds[k])
      end
      purgeSoundActive = true
    end
    purgeActiveTime = purgeActiveTime - dt
  elseif purgeSoundActive then
    for k, _ in ipairs(purgeValveNodes) do
      obj:stopSFX(purgeSounds[k] or -1)
    end
    purgeSoundActive = false
  end

  manualOverride = (electrics.values[noOverrideName] or 0) >= 1
  isArmed = (electrics.values[noArmName] or 0) >= 1
  local engineRPM = floor(assignedEngine.outputAV1 * avToRPM)
  local rpmHighEnough = engineRPM >= cutInRPM
  local hasEnoughThrottle = assignedEngine.throttle >= 1
  local isGearHighEnough = abs(electrics.values.gearIndex) >= minimumGear
  local clutchNotUsed = (electrics.values.clutch or 0) == 0
  local shouldUseN2o = (isArmed and hasEnoughThrottle and rpmHighEnough and isGearHighEnough and clutchNotUsed) or manualOverride
  local n2oActive = shouldUseN2o and hasLiquid and not purgeActive
  local torqueLookup = manualOverride and nitrousOxideOverrideTorqueLookup or nitrousOxideTorqueLookup
  local noTorque = n2oActive and torqueLookup[engineRPM] or 0
  M.isArmed = isArmed
  M.isActive = n2oActive

  electrics.values[noActiveName] = n2oActive

  --assignedEngine.continuousAfterFireFuel = assignedEngine.continuousAfterFireFuel + (n2oActive and 100 * dt or 0)
  assignedEngine.nitrousOxideTorque = assignedEngine.nitrousOxideTorque + noTorque
  assignedEngine.engineVolumeCoef = assignedEngine.engineVolumeCoef * (noTorque > 0 and volumeCoef or 1)
  assignedEngine.invBurnEfficiencyCoef = assignedEngine.invBurnEfficiencyCoef * (n2oActive and 2 or 1)
end

local function registerStorage(storageName)
  local storage = energyStorage.getStorage(storageName)
  if storage and storage.storedEnergy > 0 then
    storageWithEnergyCounter = storageWithEnergyCounter + 1
    table.insert(registeredEnergyStorages, storageName)
    updateEnergyStorageRatios()
  end
  hasLiquid = true
  previousEnergyLevels[storageName] = storage.storedEnergy
end

local function reset()
  M.isArmed = false
  M.isActive = false

  isArmed = false
  manualOverride = false

  purgeActiveTime = 0

  storageWithEnergyCounter = 0
  registeredEnergyStorages = {}
  previousEnergyLevels = {}
  hasLiquid = true
  energyStorageRatios = {}
end

local function init(device, data)
  M.isArmed = false
  M.isActive = false

  if data == nil then
    M.updateGFX = nop
    return
  end

  assignedEngine = device

  nitrousOxideTorqueLookup = {}
  nitrousOxideOverrideTorqueLookup = {}
  local addedPower = (tonumber(data.addedPower or 0)) * 1000
  cutInRPM = min(max(tonumber(data.cutInRPM) or assignedEngine.idleRPM, 1), assignedEngine.maxRPM * 0.9)
  local cutInRange = data.cutInRange or 50
  local invCutInRange = 1 / cutInRange
  local cutInStart = cutInRPM - cutInRange
  for i = 1, assignedEngine.maxRPM * 2, 1 do
    local adjustedAddedPower = min(max(addedPower * (i - cutInStart) * invCutInRange, 0), addedPower)
    nitrousOxideTorqueLookup[i + 1] = adjustedAddedPower / (i * rpmToAV)
    nitrousOxideOverrideTorqueLookup[i + 1] = addedPower / (i * rpmToAV)
  end

  noArmName = data.electricsArmName or "nitrousOxideArm"
  noOverrideName = data.electricsOverrideName or "nitrousOxideOverride"
  noActiveName = data.electricsActiveName or "nitrousOxideActive"
  minimumGear = tonumber(data.minimumGear) or 0
  volumeCoef = data.volumeCoef or 1.5

  isArmed = false
  manualOverride = false

  purgeActiveTime = 0
  purgeValveNodes = {}
  local valveNodes = data.purgeValves_nodes or {}
  local valveCount = #valveNodes - (#valveNodes % 2)
  for i = 1, valveCount, 2 do
    local cid1 = valveNodes[i]
    local cid2 = valveNodes[i + 1]
    if type(cid1) == "number" and type(cid2) == "number" then
      table.insert(purgeValveNodes, {cid1 = cid1, cid2 = cid2})
    end
  end

  storageWithEnergyCounter = 0
  registeredEnergyStorages = {}
  previousEnergyLevels = {}
  hasLiquid = true
  energyStorageRatios = {}

  M.updateGFX = updateGFX
  M.updateSounds = updateSounds
end

local function initSounds(data)
  purgeEvent = data.purgeSoundEvent or "event:>Vehicle>Nitrous_Purging"
  purgeVolume = data.purgeSoundVolume or 1
end

local function resetSounds()
  for k, _ in ipairs(purgeValveNodes) do
    obj:stopSFX(purgeSounds[k] or -1)
  end
end

local function getAddedTorque()
  local addedTorque = {}
  for k, _ in pairs(assignedEngine.torqueCurve) do
    if type(k) == "number" and k < assignedEngine.maxRPM then
      local rpm = floor(k)
      addedTorque[k + 1] = nitrousOxideTorqueLookup[rpm] or 0
    end
  end
  return addedTorque
end

-- public interface
M.init = init
M.initSounds = initSounds
M.updateSounds = nop
M.reset = reset
M.resetSounds = resetSounds
M.updateGFX = nop
M.getAddedTorque = getAddedTorque
M.registerStorage = registerStorage
M.getTankRatio = getTankRatio
M.purgeLines = purgeLines

return M
