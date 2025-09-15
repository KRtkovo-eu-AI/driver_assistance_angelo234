-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true, [2] = true}
M.deviceCategories = {shaft = true, differential = true}
M.requiredExternalInertiaOutputs = {1, 2}

local primaryOutputID = 1
local secondaryOutputID = 2

local max = math.max
local min = math.min
local sqrt = math.sqrt

local function updateVelocity(device, dt)
  device.inputAV = device[device.primaryOutputAVName] * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function lockedUpdateTorque(device, dt)
  local avDiff = device[device.primaryOutputAVName] - device[device.secondaryOutputAVName]
  local clutchRatio = device.clutchRatio
  local maxShaftAngle = device.maxShaftAngle
  local shaftAngle = clamp(device.shaftAngle + avDiff * dt * device.clutchStiffness, -maxShaftAngle * clutchRatio, maxShaftAngle * clutchRatio)
  device.shaftAngle = shaftAngle
  local lockTorque = device.lockTorque * device.wearLockTorqueCoef * device.damageLockTorqueCoef
  local secondaryTorque = clamp(shaftAngle * shaftAngle * device.lockSpring * sign(shaftAngle) + device.lockDamp * avDiff * clutchRatio, -lockTorque, lockTorque)

  device[device.primaryOutputTorqueName] = (device.parent[device.parentOutputTorqueName] - (device.friction * clamp(device.inputAV, -1, 1) + device.dynamicFriction * device.inputAV + device.torqueLossCoef * device.parent[device.parentOutputTorqueName]) * device.wearFrictionCoef * device.damageFrictionCoef) * device.gearRatio - secondaryTorque
  device[device.secondaryOutputTorqueName] = secondaryTorque
end

local function viscousUpdateTorque(device)
  local avDiff = device[device.primaryOutputAVName] - device[device.secondaryOutputAVName]
  local viscousTorque = device.viscousTorque * device.wearViscousTorqueCoef * device.damageViscousTorqueCoef
  local secondaryTorque = device.torqueSmoother:get(clamp(device.viscousCoef * math.abs(avDiff) ^ device.viscousExponent * sign(avDiff), -viscousTorque, viscousTorque))

  device[device.primaryOutputTorqueName] = (device.parent[device.parentOutputTorqueName] - (device.friction * clamp(device.inputAV, -1, 1) + device.dynamicFriction * device.inputAV + device.torqueLossCoef * device.parent[device.parentOutputTorqueName]) * device.wearFrictionCoef * device.damageFrictionCoef) * device.gearRatio - secondaryTorque
  device[device.secondaryOutputTorqueName] = secondaryTorque
end

local function disconnectedUpdateTorque(device)
  device[device.primaryOutputTorqueName] = (device.parent[device.parentOutputTorqueName] - (device.friction * clamp(device.inputAV, -1, 1) + device.dynamicFriction * device.inputAV) * device.wearFrictionCoef * device.damageFrictionCoef) * device.gearRatio
  device[device.secondaryOutputTorqueName] = 0
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  if device.splitType == "viscous" then
    device.torqueUpdate = viscousUpdateTorque
  elseif device.splitType == "locked" then
    device.torqueUpdate = lockedUpdateTorque
  end

  if device.isBroken or device.mode == "disconnected" then
    device.torqueUpdate = disconnectedUpdateTorque
  end
end

local function applyDeformGroupDamage(device, damageAmount)
  device.damageFrictionCoef = device.damageFrictionCoef + linearScale(damageAmount, 0, 0.01, 0, 0.1)
  device.damageLockTorqueCoef = device.damageLockTorqueCoef + linearScale(damageAmount, 0, 0.01, 0, 0.05)
  device.damageViscousTorqueCoef = device.damageViscousTorqueCoef + linearScale(damageAmount, 0, 0.01, 0, 0.05)
end

local function setPartCondition(device, subSystem, odometer, integrity, visual)
  device.wearFrictionCoef = linearScale(odometer, 30000000, 1000000000, 1, 2)
  device.wearLockTorqueCoef = linearScale(odometer, 30000000, 500000000, 1, 0.7)
  device.wearViscousTorqueCoef = linearScale(odometer, 30000000, 500000000, 1, 0.7)
  local integrityState = integrity
  if type(integrity) == "number" then
    local integrityValue = integrity
    integrityState = {
      damageFrictionCoef = linearScale(integrityValue, 1, 0, 1, 10),
      damageLockTorqueCoef = linearScale(integrityValue, 1, 0, 1, 0.1),
      damageViscousTorqueCoef = linearScale(integrityValue, 1, 0, 1, 0.1)
    }
  end

  device.damageFrictionCoef = integrityState.damageFrictionCoef or 1
  device.damageLockTorqueCoef = integrityState.damageLockTorqueCoef or 1
  device.damageViscousTorqueCoef = integrityState.damageViscousTorqueCoef or 1

  if integrityState.isBroken then
    device:onBreak()
  end
end

local function getPartCondition(device)
  local integrityState = {
    damageFrictionCoef = device.damageFrictionCoef,
    damageLockTorqueCoef = device.damageLockTorqueCoef,
    damageViscousTorqueCoef = device.damageViscousTorqueCoef,
    isBroken = device.isBroken
  }
  local frictionIntegrityValue = linearScale(device.damageFrictionCoef, 1, 10, 1, 0)
  local lockTorqueIntegrityValue = linearScale(device.damageFrictionCoef, 1, 0.1, 1, 0)
  local viscousTorqueIntegrityValue = linearScale(device.damageFrictionCoef, 1, 0.1, 1, 0)
  local integrityValue = min(frictionIntegrityValue, lockTorqueIntegrityValue, viscousTorqueIntegrityValue)
  if device.isBroken then
    integrityValue = 0
  end
  return integrityValue, integrityState
end

local function setMode(device, mode)
  device.mode = mode
  selectUpdates(device)
end

local function validate(device)
  if device.isPhysicallyDisconnected then
    device.mode = "disconnected"
    selectUpdates(device)
  end

  return true
end

local function onBreak(device)
  device.isBroken = true
  device.virtualMassAV = device.outputAV1

  selectUpdates(device)
end

local function calculateInertia(device)
  local outputInertia = 0
  local secondaryOutputInertia = 0
  local cumulativeGearRatio = 1
  local maxCumulativeGearRatio = 1
  if device.children then
    if device.children[primaryOutputID] then
      outputInertia = device.children[primaryOutputID].cumulativeInertia
      cumulativeGearRatio = device.children[primaryOutputID].cumulativeGearRatio
      maxCumulativeGearRatio = device.children[primaryOutputID].maxCumulativeGearRatio
    end
    if device.children[secondaryOutputID] then
      secondaryOutputInertia = device.children[secondaryOutputID].cumulativeInertia
    end
  end

  device.cumulativeInertia = outputInertia / device.gearRatio / device.gearRatio
  device.cumulativeGearRatio = cumulativeGearRatio * device.gearRatio
  device.maxCumulativeGearRatio = maxCumulativeGearRatio * device.gearRatio

  device.lockSpring = device.lockSpringBase or (powertrain.stabilityCoef * powertrain.stabilityCoef * secondaryOutputInertia * device.lockSpringCoef) --Nm/rad
  device.lockDamp = device.lockDampRatio * sqrt(device.lockSpring * secondaryOutputInertia)
  device.maxShaftAngle = math.sqrt(device.lockTorque / device.lockSpring)
  --print(device.lockSpring)
  --print(device.maxShaftAngle)
end

local function reset(device, jbeamData)
  device.gearRatio = jbeamData.gearRatio or 1
  device.friction = jbeamData.friction or 0
  device.cumulativeInertia = 1
  device.cumulativeGearRatio = 1
  device.maxCumulativeGearRatio = 1

  device.outputAV1 = 0
  device.outputAV2 = 0
  device.inputAV = 0
  device.outputTorque1 = 0
  device.outputTorque2 = 0
  device.visualShaftAngle = 0
  device.isBroken = false

  device.viscousCoef = jbeamData.viscousCoef or 10
  device.clutchRatio = jbeamData.defaultClutchRatio or 1

  device.wearFrictionCoef = 1
  device.damageFrictionCoef = 1
  device.wearLockTorqueCoef = 1
  device.damageLockTorqueCoef = 1
  device.wearViscousTorqueCoef = 1
  device.damageViscousTorqueCoef = 1

  if jbeamData.canDisconnect then
    device.mode = jbeamData.isDisconnected and "disconnected" or "connected"
  else
    device.mode = "connected"
  end

  --locked specific
  device.shaftAngle = 0

  --viscous specific
  device.torqueSmoother:reset()

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
    gearRatio = jbeamData.gearRatio or 1,
    friction = jbeamData.friction or 0,
    dynamicFriction = jbeamData.dynamicFriction or 0,
    torqueLossCoef = jbeamData.torqueLossCoef or 0,
    wearFrictionCoef = 1,
    damageFrictionCoef = 1,
    wearLockTorqueCoef = 1,
    damageLockTorqueCoef = 1,
    wearViscousTorqueCoef = 1,
    damageViscousTorqueCoef = 1,
    cumulativeInertia = 1,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    defaultVirtualInertia = jbeamData.defaultVirtualInertia or nil, --meant to be nil if not specified manually
    outputAV1 = 0,
    outputAV2 = 0,
    inputAV = 0,
    outputTorque1 = 0,
    outputTorque2 = 0,
    visualShaftAngle = 0,
    isBroken = false,
    splitType = jbeamData.splitType or "viscous",
    reset = reset,
    onBreak = onBreak,
    setMode = setMode,
    validate = validate,
    calculateInertia = calculateInertia,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition
  }

  device.torqueLossCoef = clamp(device.torqueLossCoef, 0, 1)

  primaryOutputID = min(max(jbeamData.primaryOutputID or 1, 1), 2) --must be either 1 or 2
  secondaryOutputID = math.abs(primaryOutputID * 3 - 5) --converts 1 -> 2 and 2 -> 1

  device.primaryOutputTorqueName = "outputTorque" .. tostring(primaryOutputID)
  device.primaryOutputAVName = "outputAV" .. tostring(primaryOutputID)
  device.secondaryOutputTorqueName = "outputTorque" .. tostring(secondaryOutputID)
  device.secondaryOutputAVName = "outputAV" .. tostring(secondaryOutputID)

  if jbeamData.canDisconnect then
    device.availableModes = {"connected", "disconnected"}
    device.mode = jbeamData.isDisconnected and "disconnected" or "connected"
  else
    device.availableModes = {"connected"}
    device.mode = "connected"
  end

  --locked specific
  device.shaftAngle = 0
  device.lockTorque = jbeamData.lockTorque or 500
  device.clutchRatio = jbeamData.defaultClutchRatio or 1
  device.lockDampRatio = jbeamData.lockDampRatio or 0.15 --1 is critically damped
  device.clutchStiffness = jbeamData.clutchStiffness or 1
  device.lockSpringCoef = jbeamData.lockSpringCoef or 1
  device.lockSpringBase = jbeamData.lockSpring

  --viscous specific
  device.viscousCoef = jbeamData.viscousCoef or 10
  device.viscousTorque = jbeamData.viscousTorque or device.viscousCoef * 10
  device.viscousExponent = jbeamData.viscousExponent or 1
  device.torqueSmoother = newExponentialSmoothing(jbeamData.viscousSmoothing or 25)

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
