-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {}
M.deviceCategories = {clutchlike = true, clutch = true, pneumaticPowerSource = true, torqueConsumer = true}

local max = math.max
local abs = math.abs

local twoPi = math.pi * 2
local invTwoPi = 1 / twoPi
local avToRPM = 9.549296596425384

local function updateVelocity(device, dt)
  device.inputAV = device.parent[device.parentOutputAVName]
  device.parent[device.parentOutputAVName] = device.inputAV

  -- volume of air being moved per second
  local pumpingCoef = device.pumpEfficiency * (1 - device.deloadCoef)
  local pumpFlow = max(0, device.inputAV * invTwoPi * device.pumpDisplacement * pumpingCoef)
  -- TODO: calculate lost energy

  -- in order to determine how much "energy" is added to the tank, we need to compute the volume of air being moved
  -- at this point in time using the airflow calculated above. both sides of the Ideal Gas Law (PV = nRT) represent
  -- energy, so we can use the pressure of the air as it entered the pump (environmental pressure) along with the
  -- volume this update to calculate the energy moved this update.
  local pumpedVolume = pumpFlow * dt
  local pumpedEnergy = pumpedVolume * powertrain.currentEnvPressure

  --streams.drawGraph("pumpedAirFlow", { value = pumpFlow })
  --streams.drawGraph("pumpedAirVolume", { value = pumpedVolume })
  --streams.drawGraph("pumpedEnergy", { value = pumpedEnergy })

  device.pumpedAirFlow = pumpFlow
  device.pumpedEnergy = pumpedEnergy

  local pressureTank = device.pressureTank

  if pressureTank then
    pressureTank.storedEnergy = pressureTank.storedEnergy + pumpedEnergy

    -- handle deloading
    local relTankPressure = pressureTank.currentPressure - powertrain.currentEnvPressure

    if relTankPressure > device.deloadStartPressure then
      if device.engaged then
        --log("D", "compressor.updateVelocity", ("playing %q at %d"):format(device.purgeSoundEvent, device.purgeSoundNodeId))
        obj:playSFXOnce(device.purgeSoundEvent, device.purgeSoundNodeId, device.purgeSoundVolume, 1)
      end

      device.engaged = false
    elseif relTankPressure < device.deloadEndPressure then
      device.engaged = true
    end

    --streams.drawGraph("tankPressure", { value = tankPressure / 1000, min = 0, max = pressureTank.maxWorkingPressure / 1000 })

    device.pumpPressure = (1 - device.deloadCoef) * relTankPressure
  else
    device.engaged = false
    device.pumpPressure = 0
  end

  device.deloadCoef = (device.engaged or not device.deloadWhenDisengaged) and 0 or 1
end

local function updateTorque(device, dt)
  local torqueDiff = device.inputAV > 0 and (device.pumpPressure * device.pumpDisplacement * invTwoPi) or 0

  --streams.drawGraph("torqueDiff", torqueDiff)

  device.torqueDiff = torqueDiff
end

local function updateGFX(device, dt)
  if streams.willSend("pneumaticsData") then
    local tankPressure = nil
    if device.pressureTank then
      tankPressure = (device.pressureTank.currentPressure - powertrain.currentEnvPressure) * 0.001
    end
    gui.send(
      "pneumaticsData",
      {
        tankPressure = tankPressure,
        compressorState = device.engaged and "Engaged" or "Disengaged"
      }
    )
  end
end

local function updateSounds(device, dt)
  local inputAV = device.inputAV
  local volumeCoef = linearScale(inputAV, 10, 20, 0, 1)
  local volume = device.compressorSoundVolume * volumeCoef
  local pitch = device.pitchSmoothing:get(inputAV * avToRPM, dt)
  local color = 1 - device.deloadCoef
  obj:setVolumePitchCT(device.compressorSound, volume, pitch, color, 0)
  --guihooks.graph({"Pressure", device.accumulatorPressure, 55000000, ""}, {"Flow", device.pumpedAirFlow, 0.005, ""}, {"Volume Coef", volumeFlowCoef, 1, ""}, {"Volume", volume, 1, ""}, {"Pitch", pitch, 1, ""})
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  device.torqueUpdate = updateTorque

  if device.isBroken then
  --TODO
  end
end

local function applyDeformGroupDamage(device, damageAmount)
end

local function setPartCondition(device, subSystem, odometer, integrity, visual)
  local integrityState = integrity
  if type(integrity) == "number" then
    local integrityValue = integrity
    integrityState = {isBroken = false}
  end

  if integrityState.isBroken then
    device:onBreak()
  end
end

local function getPartCondition(device)
  local integrityState = {isBroken = device.isBroken}
  local integrityValue = 1
  if device.isBroken then
    integrityValue = 0
  end
  return integrityValue, integrityState
end

local function validate(device)
  return true
end

local function onBreak(device)
  device.isBroken = true
  selectUpdates(device)
end

local function registerStorage(device, storageName)
  local storage = energyStorage.getStorage(storageName)
  if not storage then
    return
  end
  if storage.type == "pressureTank" then
    if storage.energyType ~= device.requiredEnergyType then
      log("E", "compressor.registerStorage", ("provided energyStorage for compressor %q has wrong energyType (compressor wants %q, storage is %q)"):format(device.name, device.requiredEnergyType, storage.energyType))
      return
    end

    device.pressureTank = storage
  end
end

local function calculateInertia(device)
  local outputInertia
  local cumulativeGearRatio = 1
  local maxCumulativeGearRatio = 1
  --the pump only has virtual inertia
  outputInertia = device.virtualInertia --some default inertia

  device.cumulativeInertia = outputInertia / device.gearRatio / device.gearRatio
  device.invCumulativeInertia = device.cumulativeInertia > 0 and 1 / device.cumulativeInertia or 0
  device.cumulativeGearRatio = cumulativeGearRatio * device.gearRatio
  device.maxCumulativeGearRatio = maxCumulativeGearRatio * device.gearRatio
end

local function resetSounds(device, jbeamData)
end

local function initSounds(device, jbeamData)
  local purgeSoundNode = jbeamData.purgeSoundNode and beamstate.nodeNameMap[jbeamData.purgeSoundNode]
  device.purgeSoundNodeId = purgeSoundNode or device.parent.engineNodeID or 0
  --log("D", "compressor.initSounds", ("purgeSoundNode: %q | purgeSoundNodeId: %d"):format(purgeSoundNode, device.purgeSoundNodeId))
  device.purgeSoundEvent = jbeamData.pumpPurgeEvent or "event:>Vehicle>Pneumatics>Air_Dryer_Purge"
  device.purgeSoundVolume = jbeamData.purgeSoundVolume or 0.5
  local compressorSoundNode = jbeamData.compressorSoundNode and beamstate.nodeNameMap[jbeamData.compressorSoundNode]
  compressorSoundNode = compressorSoundNode or device.parent.engineNodeID or 0
  local compressorLoopEvent = jbeamData.compressorLoopEvent or "event:>Vehicle>Pneumatics>Air_Compressor"
  device.compressorSound = obj:createSFXSource2(compressorLoopEvent, "AudioDefaultLoop3D", "compressorSound", compressorSoundNode, 1)
  device.compressorSoundVolume = jbeamData.compressorSoundVolume or 0.5
  obj:playSFX(device.compressorSound)
  obj:setVolumePitchCT(device.compressorSound, 0, 0, 0, 0)

  --device.volumeSmoothing = newTemporalSigmoidSmoothing(5, 2, 2, 5)
  device.pitchSmoothing = newTemporalSigmoidSmoothing(2000, 4000, 4000, 2000)
end

local function reset(device, jbeamData)
  device.gearRatio = jbeamData.gearRatio or 1
  device.friction = jbeamData.friction or 0
  device.cumulativeInertia = 1
  device.invCumulativeInertia = 1
  device.cumulativeGearRatio = 1
  device.maxCumulativeGearRatio = 1

  device.inputAV = 0
  device.lastInputAV = 0
  device.visualShaftAngle = 0
  device.virtualMassAV = 0

  device.isBroken = false
  device.wearFrictionCoef = 1
  device.damageFrictionCoef = 1

  device.pumpPressure = 0
  device.pumpedAirFlow = 0
  device.pumpedEnergy = 0
  device.deloadCoef = 1

  device[device.outputTorqueName] = 0
  device[device.outputAVName] = 0

  --device.volumeSmoothing:reset()
  device.pitchSmoothing:reset()

  selectUpdates(device)

  return device
end

local function new(jbeamData)
  local device = {
    deviceCategories = shallowcopy(M.deviceCategories),
    requiredExternalInertiaOutputs = shallowcopy(M.requiredExternalInertiaOutputs),
    outputPorts = shallowcopy(M.outputPorts),
    name = jbeamData.name,
    type = jbeamData.type,
    inputName = jbeamData.inputName,
    inputIndex = jbeamData.inputIndex,
    gearRatio = jbeamData.gearRatio or 1,
    friction = jbeamData.friction or 0,
    dynamicFriction = jbeamData.dynamicFriction or 0,
    wearFrictionCoef = 1,
    damageFrictionCoef = 1,
    cumulativeInertia = 1,
    invCumulativeInertia = 1,
    virtualInertia = 1,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    electricsName = jbeamData.electricsName,
    visualShaftAVName = jbeamData.visualShaftAVName,
    inputAV = 0,
    lastInputAV = 0,
    visualShaftAngle = 0,
    virtualMassAV = 0,
    isBroken = false,
    nodeCid = jbeamData.node,
    reset = reset,
    onBreak = onBreak,
    registerStorage = registerStorage,
    validate = validate,
    calculateInertia = calculateInertia,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition,
    updateGFX = updateGFX,
    initSounds = initSounds,
    resetSounds = resetSounds,
    updateSounds = updateSounds,
    torqueDiff = 0
  }

  device.pressureTank = nil
  device.requiredEnergyType = jbeamData.requiredEnergyType or "air"
  device.energyStorage = jbeamData.energyStorage

  device.pumpDisplacement = jbeamData.pumpDisplacement or 0.00035

  -- TODO: implement max pressure and use catmullrom spline for efficiency curve
  device.pumpEfficiency = jbeamData.pumpEfficiency or 0.8 -- portion of the pump's displacement that actually makes it into the air tank on each stroke

  device.deloadStartPressure = jbeamData.cutOutPressure or 900000 -- Pascals, *relative* pressure (i.e. pressure above environmental)
  device.deloadEndPressure = jbeamData.cutInPressure or device.deloadStartPressure * 0.8
  device.deloadCoef = 1
  device.engaged = false

  device.pumpPressure = 0
  device.pumpedAirFlow = 0
  device.pumpedEnergy = 0

  device.outputTorqueName = "outputTorque1"
  device.outputAVName = "outputAV1"
  device[device.outputTorqueName] = 0
  device[device.outputAVName] = 0

  device.motorThrottleElectricsName = jbeamData.motorThrottleElectricsName or (device.name .. "Throttle")
  device.deloadWhenDisengaged = jbeamData.deloadWhenDisengaged == nil and true or jbeamData.deloadWhenDisengaged

  device.mode = "connected"

  device.breakTriggerBeam = jbeamData.breakTriggerBeam
  if device.breakTriggerBeam and device.breakTriggerBeam == "" then
    --get rid of the break beam if it's just an empty string (cancellation)
    device.breakTriggerBeam = nil
  end

  selectUpdates(device)

  return device
end

M.new = new

return M
