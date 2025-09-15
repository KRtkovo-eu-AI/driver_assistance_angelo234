-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true}
M.deviceCategories = {clutchlike = true, clutch = true}
M.requiredExternalInertiaOutputs = {1}

local max = math.max
local min = math.min
local abs = math.abs
local sqrt = math.sqrt
local clamp = clamp

local kelvinToCelsius = -273.15
local rpmToAV = 0.104719755

local function updateGFX(device, dt)
  local kClutchToHousing = 20
  local tEnv = obj:getEnvTemperature() + kelvinToCelsius

  local energyToClutch = device.frictionLossPerUpdate * device.clutchThermalsCoef * device.clutchThermalsEnabledCoef
  local energyClutchToBellHousing = (device.clutchTemperature - tEnv) * kClutchToHousing * device.clutchCoolingCoef * dt

  device.clutchTemperature = clamp(device.clutchTemperature + (energyToClutch - energyClutchToBellHousing) * device.clutchEnergyCoef, tEnv, device.clutchPermanentDamageTempThreshold)
  local thermalEfficiency = clamp(-0.5 * (device.clutchTemperature - device.clutchMaxSafeTemp) * device.clutchInvOverheatRange + 1, 0.5, 1)
  device.thermalEfficiency = device.clutchPermanentlyDamaged and 0.25 or thermalEfficiency

  device.clutchSmokeTimer = device.clutchSmokeTimer > 1 and 0 or device.clutchSmokeTimer + dt * (1 - thermalEfficiency) * 50
  if device.clutchSmokeTimer >= 1 and device.children[1].transmissionNodeID then
    obj:addParticleByNodesRelative(device.children[1].transmissionNodeID, device.children[1].transmissionNodeID, 1, 35, 0, 1)
  end

  local clutchMessage = nil
  local messageTime = 1

  if device.clutchTemperature >= device.clutchWarningTemp and not device.clutchPermanentlyDamaged then
    clutchMessage = "High clutch temperature..."
  end
  if device.thermalEfficiency < 1 and not device.clutchPermanentlyDamaged then
    clutchMessage = "Clutch overheating..."
  end

  if device.clutchTemperature >= device.clutchPermanentDamageTempThreshold and not device.clutchPermanentlyDamaged then
    clutchMessage = "Clutch permanently damaged!"
    messageTime = 3
    damageTracker.setDamage("powertrain", device.name, true)
  end

  if clutchMessage then
    device.clutchThermalsMessageTimer = device.clutchThermalsMessageTimer - dt
    if device.clutchThermalsMessageTimer <= 0 then
      guihooks.message({txt = clutchMessage}, messageTime, "vehicle.clutchThermals")
      device.clutchThermalsMessageTimer = 0.9
    end
  end

  device.clutchPermanentlyDamaged = device.clutchPermanentlyDamaged or device.clutchTemperature >= device.clutchPermanentDamageTempThreshold

  device.frictionLossPerUpdate = 0

  if streams.willSend("clutchThermalData") then
    gui.send(
      "clutchThermalData",
      {
        clutchTemperature = device.clutchTemperature,
        thermalEfficiency = device.thermalEfficiency,
        energyToClutch = energyToClutch,
        energyClutchToBellHousing = energyClutchToBellHousing,
        maxSafeTemp = device.clutchMaxSafeTemp,
        efficiencyScaleEnd = device.clutchEfficiencyScaleEnd,
        permanentDamageTemp = device.clutchPermanentDamageTempThreshold
      }
    )
  end
end

local function updateVelocity(device, dt)
  device.inputAV = device.parent.outputAV1
end

local function updateTorque(device, dt)
  local clutchRatio = linearScale(device.inputAV, device.engageAVStart, device.engageAVEnd, 0, 1)

  local avDiff = device.inputAV - device.outputAV1
  local lockDampAV = avDiff

  local maxClutchAngle = device.maxClutchAngle
  local clutchFreePlay = device.clutchFreePlay * device.damageClutchFreePlayCoef * device.wearClutchFreePlayCoef
  local clutchAngle = clamp(device.clutchAngle + avDiff * dt * device.clutchStiffness, -maxClutchAngle * clutchRatio - clutchFreePlay, maxClutchAngle * clutchRatio + clutchFreePlay)

  clutchAngle = device.clutchAngleSmoother:get(clutchAngle)

  local absFreeClutchAngle = max(abs(clutchAngle) - clutchFreePlay, 0)
  local lockTorque = device.lockTorque * device.thermalEfficiency * device.damageLockTorqueCoef * device.wearLockTorqueCoef

  lockDampAV = device.lockDampAVSmoother:get(lockDampAV)

  --linear clutch
  --local torqueDiff = (clamp(absFreeClutchAngle * sign(clutchAngle) * device.lockSpring + lockDampAV * device.lockDamp * clutchRatio, -lockTorque, lockTorque))
  --squared to linear clutch
  local torqueDiff = (clamp(min(1, absFreeClutchAngle) * absFreeClutchAngle * sign(clutchAngle) * device.lockSpring + lockDampAV * device.lockDamp * clutchRatio, -lockTorque, lockTorque))

  device.clutchAngle = clutchAngle
  device.torqueDiff = torqueDiff
  device.outputTorque1 = torqueDiff
  device.clutchRatio = clutchRatio

  device.frictionLossPerUpdate = device.frictionLossPerUpdate + torqueDiff * avDiff * dt
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  device.torqueUpdate = updateTorque
end

local function setLock(device, enabled)
  device.clutchThermalsCoef = enabled and 0 or 1
end

local function applyDeformGroupDamage(device, damageAmount)
  device.damageClutchFreePlayCoef = device.damageClutchFreePlayCoef + linearScale(damageAmount, 0, 0.01, 0, 0.01)
  device.damageLockTorqueCoef = max(device.damageLockTorqueCoef - linearScale(damageAmount, 0, 0.01, 0, 0.1), 0.2)
  device:calculateInertia()
end

local function setPartCondition(device, subSystem, odometer, integrity, visual)
  device.wearLockTorqueCoef = linearScale(odometer, 30000000, 500000000, 1, 0.7)
  device.wearClutchFreePlayCoef = linearScale(odometer, 30000000, 500000000, 1, 10)
  local integrityState = integrity
  if type(integrity) == "number" then
    local integrityValue = integrity
    integrityState = {
      damageClutchFreePlayCoef = linearScale(integrityValue, 1, 0, 1, 20),
      damageLockTorqueCoef = linearScale(integrityValue, 1, 0, 1, 0.5),
      clutchPermanentlyDamaged = false
    }
  end

  device.damageClutchFreePlayCoef = integrityState.damageClutchFreePlayCoef or 1
  device.damageLockTorqueCoef = integrityState.damageLockTorqueCoef or 1
  device.clutchPermanentlyDamaged = integrityState.clutchPermanentlyDamaged or false

  device:calculateInertia()
end

local function getPartCondition(device)
  local integrityState = {
    damageClutchFreePlayCoef = device.damageClutchFreePlayCoef,
    damageLockTorqueCoef = device.damageLockTorqueCoef,
    clutchPermanentlyDamaged = device.clutchPermanentlyDamaged
  }
  local integrityValueFreePlay = linearScale(device.damageClutchFreePlayCoef, 1, 20, 1, 0)
  local integrityValueLockTorque = linearScale(device.damageLockTorqueCoef, 1, 0.5, 1, 0)
  local integrityValue = min(integrityValueFreePlay, integrityValueLockTorque)

  if device.clutchPermanentlyDamaged then
    integrityValue = 0
  end

  return integrityValue, integrityState
end

local function validate(device)
  if not device.parent.deviceCategories.engine then
    log("E", "frictionClutch.validate", "Parent device is not an engine device...")
    log("E", "frictionClutch.validate", "Actual parent:")
    log("E", "frictionClutch.validate", powertrain.dumpsDeviceData(device.parent))
    return false
  end

  device.lockTorque = device.lockTorque or (device.parent.torqueData.maxTorque * 1.25 + device.parent.maxRPM * device.parent.inertia * math.pi / 30)
  return true
end

local function calculateInertia(device)
  local outputInertia = 0
  local cumulativeGearRatio = 1
  local maxCumulativeGearRatio = 1
  if device.children and #device.children > 0 then
    local child = device.children[1]
    outputInertia = child.cumulativeInertia
    cumulativeGearRatio = child.cumulativeGearRatio
    maxCumulativeGearRatio = child.maxCumulativeGearRatio
  end

  device.cumulativeInertia = min(outputInertia, device.parent.inertia * 0.5)
  device.lockSpring = device.lockSpringBase or (powertrain.stabilityCoef * powertrain.stabilityCoef * device.cumulativeInertia * device.lockSpringCoef) --Nm/rad
  device.lockDamp = device.lockDampRatio * sqrt(device.lockSpring * device.cumulativeInertia)

  --^2 spring but linear spring after 1 rad
  device.maxClutchAngle = sqrt(device.lockTorque / device.lockSpring) + max(device.lockTorque / device.lockSpring - 1, 0)
  --linear spring
  --device.maxClutchAngle = device.lockTorque / device.lockSpring

  device.cumulativeGearRatio = cumulativeGearRatio
  device.maxCumulativeGearRatio = maxCumulativeGearRatio
end

local function reset(device, jbeamData)
  device.cumulativeInertia = 1
  device.cumulativeGearRatio = 1
  device.maxCumulativeGearRatio = 1

  device.outputAV1 = 0
  device.inputAV = 0
  device.outputTorque1 = 0
  device.clutchAngle = 0
  device.clutchRatio = 1
  device.torqueDiff = 0
  device.lockDampAVSmoother:reset()
  device.clutchAngleSmoother:reset()

  device.thermalEfficiency = 1
  device.frictionLossPerUpdate = 0
  device.clutchTemperature = obj:getEnvTemperature() + kelvinToCelsius
  device.clutchPermanentlyDamaged = false
  device.clutchSmokeTimer = 0
  device.clutchThermalsCoef = 1
  device.clutchThermalsMessageTimer = 0

  device.damageClutchFreePlayCoef = 1
  device.damageLockTorqueCoef = 1
  device.wearClutchFreePlayCoef = 1
  device.wearLockTorqueCoef = 1

  selectUpdates(device)
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
    gearRatio = 1,
    additionalEngineInertia = jbeamData.additionalEngineInertia or 0,
    cumulativeInertia = 1,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    outputAV1 = 0,
    inputAV = 0,
    outputTorque1 = 0,
    clutchAngle = 0,
    clutchRatio = 1,
    torqueDiff = 0,
    engageAVStart = (jbeamData.engageRPMStart or 800) * rpmToAV,
    engageAVEnd = (jbeamData.engageRPMEnd or 1100) * rpmToAV,
    thermalEfficiency = 1,
    damageClutchFreePlayCoef = 1,
    damageLockTorqueCoef = 1,
    wearClutchFreePlayCoef = 1,
    wearLockTorqueCoef = 1,
    frictionLossPerUpdate = 0,
    clutchTemperature = obj:getEnvTemperature() + kelvinToCelsius,
    clutchPermanentlyDamaged = false,
    clutchSmokeTimer = 0,
    clutchThermalsCoef = 1,
    clutchThermalsEnabledCoef = 1,
    clutchThermalsMessageTimer = 0,
    electricsClutchRatioName = jbeamData.electricsClutchRatioName or "clutchRatio",
    lockDampRatio = jbeamData.lockDampRatio or 0.15, --1 is critically damped
    lockDampAVSmoother = newExponentialSmoothing(jbeamData.lockDampSmoothing or 0),
    clutchAngleSmoother = newExponentialSmoothing(jbeamData.clutchAngleSmoothing or 0),
    clutchStiffness = jbeamData.clutchStiffness or 1,
    clutchFreePlay = jbeamData.clutchFreePlay or 0.125,
    lockSpringCoef = jbeamData.lockSpringCoef or 1,
    lockTorque = jbeamData.lockTorque,
    lockSpringBase = jbeamData.lockSpring,
    reset = reset,
    validate = validate,
    calculateInertia = calculateInertia,
    setLock = setLock,
    updateGFX = updateGFX,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition
  }

  local thermalsEnabled = jbeamData.thermalsEnabled == nil and true or jbeamData.thermalsEnabled
  device.clutchThermalsEnabledCoef = thermalsEnabled and 1 or 0
  device.clutchCoolingCoef = jbeamData.coolingCoef or 0.8
  device.clutchPermanentDamageTempThreshold = jbeamData.maxClutchTemp or 300
  device.clutchWarningTemp = jbeamData.warningTemp or 100
  device.clutchMaxSafeTemp = jbeamData.maxSafeClutchTemp or 150
  device.clutchInvOverheatRange = 1 / (jbeamData.clutchOverheatRange or 100)
  device.clutchEfficiencyScaleEnd = device.clutchMaxSafeTemp + 1 / device.clutchInvOverheatRange
  local mass = jbeamData.clutchMass or 1
  local specificHeat = jbeamData.clutchSpecificHeat or 490
  device.clutchEnergyCoef = 1 / (mass * specificHeat)

  selectUpdates(device)

  return device
end

M.new = new

return M
