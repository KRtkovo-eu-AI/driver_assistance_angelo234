-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true}
M.deviceCategories = {winch = true, torqueConsumer = true}
M.requiredExternalInertiaOutputs = {1}

local max = math.max
local min = math.min
local abs = math.abs

local function updateVelocity(device, dt)
  device.inputAV = device.outputAV1 * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function updateTorque(device)
  device.outputTorque1 = 0
  --actuateBeam(int outId, float force, float speedLimit, float slipForce, float frictionForce, float slipSpeedLimit, float minExtend, float maxExtend)
  local force = -electrics.values.ignitionLevel * 500000
  local slipForce = 0
  local frictionForce = 0
  obj:actuateBeam(beamstate.tagBeamMap.ropeTest[1], force, 10, slipForce, frictionForce, 10, 0, 100)
end

local function applyDeformGroupDamage(device, damageAmount)
  device.damageFrictionCoef = device.damageFrictionCoef + linearScale(damageAmount, 0, 0.01, 0, 0.1)
end

local function setPartCondition(device, subSystem, odometer, integrity, visual)
  device.wearFrictionCoef = linearScale(odometer, 30000000, 1000000000, 1, 2)
  local integrityState = integrity
  if type(integrity) == "number" then
    local integrityValue = integrity
    integrityState = {damageFrictionCoef = linearScale(integrityValue, 1, 0, 1, 50), isBroken = false}
  end

  device.damageFrictionCoef = integrityState.damageFrictionCoef or 1

  if integrityState.isBroken then
    device:onBreak()
  end
end

local function getPartCondition(device)
  local integrityState = {damageFrictionCoef = device.damageFrictionCoef, isBroken = device.isBroken}
  local integrityValue = linearScale(device.damageFrictionCoef, 1, 50, 1, 0)
  if device.isBroken then
    integrityValue = 0
  end
  return integrityValue, integrityState
end

local function calculateInertia(device)
  local outputInertia = 0

  local gearRatio = device.gearRatio
  device.cumulativeInertia = outputInertia / gearRatio / gearRatio
  device.cumulativeGearRatio = device.gearRatio
  device.maxCumulativeGearRatio = device.maxGearRatio
end

local function reset(device, jbeamData)
  device.friction = jbeamData.friction or 0
  device.cumulativeInertia = 1
  device.cumulativeGearRatio = 1
  device.maxCumulativeGearRatio = 1

  device.outputAV1 = 0
  device.inputAV = 0
  device.outputTorque1 = 0
  device.virtualMassAV = 0
  device.isBroken = false

  device.wearFrictionCoef = 1
  device.damageFrictionCoef = 1
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
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    outputAV1 = 0,
    inputAV = 0,
    outputTorque1 = 0,
    virtualMassAV = 0,
    isBroken = false,
    reset = reset,
    torqueUpdate = updateTorque,
    velocityUpdate = updateVelocity,
    calculateInertia = calculateInertia,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition
  }

  return device
end

M.new = new

return M
