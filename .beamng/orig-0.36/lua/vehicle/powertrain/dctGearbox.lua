-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true}
M.deviceCategories = {clutchlike = true, gearbox = true}
M.requiredExternalInertiaOutputs = {1}

local max = math.max
local min = math.min
local abs = math.abs
local sqrt = math.sqrt

local rpmToAV = 0.104719755
local avToRPM = 9.549296596425384

local function updateSounds(device, dt)
  local gearWhineCoefInput = device.gearWhineCoefsInput[device.gearIndex] or 0
  local gearWhineCoefOutput = device.gearWhineCoefsOutput[device.gearIndex] or 0

  local gearWhineDynamicsCoef = 0.05
  local fixedVolumePartOutput = device.gearWhineOutputAV * device.invMaxExpectedOutputAV --normalized AV
  local powerVolumePartOutput = device.gearWhineOutputAV * device.gearWhineOutputTorque * device.invMaxExpectedPower --normalized power
  local volumeOutput = clamp(gearWhineCoefOutput + ((abs(fixedVolumePartOutput) + abs(powerVolumePartOutput)) * gearWhineDynamicsCoef), 0, 10)

  local fixedVolumePartInput = device.gearWhineInputAV * device.invMaxExpectedInputAV --normalized AV
  local powerVolumePartInput = device.gearWhineInputAV * device.gearWhineInputTorque * device.invMaxExpectedPower --normalized power
  local volumeInput = clamp(gearWhineCoefInput + ((abs(fixedVolumePartInput) + abs(powerVolumePartInput)) * gearWhineDynamicsCoef), 0, 10)

  local inputPitchCoef = device.gearRatio >= 0 and device.forwardInputPitchCoef or device.reverseInputPitchCoef
  local outputPitchCoef = device.gearRatio >= 0 and device.forwardOutputPitchCoef or device.reverseOutputPitchCoef
  local pitchInput = clamp(abs(device.gearWhineInputAV) * avToRPM * inputPitchCoef, 0, 10000000)
  local pitchOutput = clamp(abs(device.gearWhineOutputAV) * avToRPM * outputPitchCoef, 0, 10000000)

  local inputLoad = device.gearWhineInputTorque * device.invMaxExpectedInputTorque
  local outputLoad = device.gearWhineOutputTorque * device.invMaxExpectedOutputTorque
  local outputRPMSign = sign(device.gearWhineOutputAV)

  device.gearWhineOutputLoop:setVolumePitch(volumeOutput, pitchOutput, outputLoad, outputRPMSign)
  device.gearWhineInputLoop:setVolumePitch(volumeInput, pitchInput, inputLoad, outputRPMSign)

  -- print(string.format("volIn - %0.2f / volOut - %0.2f / ptchIn - %0.2f / ptchOut - %0.2f / inLoad - %0.2f / outLoad - %0.2f", volumeInput, volumeOutput, pitchInput, pitchOutput, inputLoad, outputLoad))
end

local function updateVelocity(device, dt)
  device.inputAV = device.parent.outputAV1
  device.clutchAV1 = device.outputAV1 * device.gearRatio1 * device.lockCoef
  device.clutchAV2 = device.outputAV1 * device.gearRatio2 * device.lockCoef
end

local function updateTorque(device, dt)
  local inputAV = device.inputAV
  device.clutchRatio1 = electrics.values[device.electricsClutchRatio1Name] or 0
  device.clutchRatio2 = electrics.values[device.electricsClutchRatio2Name] or 0

  if inputAV < (device.parent.idleAV or inputAV) * 0.5 then
    device.clutchRatio1 = 0
    device.clutchRatio2 = 0
  end

  local avDiff1 = inputAV - device.clutchAV1
  local avDiff2 = inputAV - device.clutchAV2
  local clutchRatio1 = device.clutchRatio1
  local clutchRatio2 = device.clutchRatio2
  local maxClutchAngle1 = device.maxClutchAngle1
  local maxClutchAngle2 = device.maxClutchAngle2

  device.clutchAngle1 = min(max(device.clutchAngle1 + avDiff1 * dt * device.clutchStiffness, -maxClutchAngle1 * clutchRatio1), maxClutchAngle1 * clutchRatio1)
  device.clutchAngle2 = min(max(device.clutchAngle2 + avDiff2 * dt * device.clutchStiffness, -maxClutchAngle2 * clutchRatio2), maxClutchAngle2 * clutchRatio2)

  local lockTorque = device.lockTorque * device.wearLockTorqueCoef * device.damageLockTorqueCoef
  device.torqueDiff1 = (min(max(device.clutchAngle1 * device.lockSpring1 + device.lockDamp1 * avDiff1 * clutchRatio1, -lockTorque), lockTorque))
  device.torqueDiff2 = (min(max(device.clutchAngle2 * device.lockSpring2 + device.lockDamp2 * avDiff2 * clutchRatio2, -lockTorque), lockTorque))

  device.torqueDiff = device.torqueDiff1 + device.torqueDiff2

  device.outputTorque1 = ((device.torqueDiff1 - device.friction * min(max(device.clutchAV1, -1), 1)) * device.gearRatio1 + (device.torqueDiff2 - device.friction * min(max(device.clutchAV2, -1), 1)) * device.gearRatio2) * device.lockCoef
  device.clutchRatio = max(device.clutchRatio1, device.clutchRatio2)

  device.gearWhineInputTorque = device.gearWhineInputTorqueSmoother:get(device.torqueDiff)
  device.gearWhineOutputTorque = device.gearWhineOutputTorqueSmoother:get(device.outputTorque1)
  device.gearWhineInputAV = device.gearWhineInputAVSmoother:get(inputAV)
  device.gearWhineOutputAV = device.gearWhineOutputAVSmoother:get(device.outputAV1)
end

local function neutralUpdateVelocity(device, dt)
  device.inputAV = device.parent.outputAV1
  device.clutchAV1 = device.outputAV1 * device.gearRatio1 * device.lockCoef
  device.clutchAV2 = device.outputAV1 * device.gearRatio2 * device.lockCoef
end

local function neutralUpdateTorque(device, dt)
  device.torqueDiff = 0
  device.outputTorque1 = 0

  device.gearWhineInputTorque = device.gearWhineInputTorqueSmoother:get(device.torqueDiff)
  device.gearWhineOutputTorque = device.gearWhineOutputTorqueSmoother:get(device.outputTorque1)
  --device.gearWhineInputAV = device.gearWhineInputAVSmoother:get(device.inputAV)
  device.gearWhineInputAV = device.gearWhineInputAVSmoother:get(max(device.clutchAV1, device.clutchAV2))
  device.gearWhineOutputAV = device.gearWhineOutputAVSmoother:get(device.outputAV1)
end

local function parkUpdateVelocity(device, dt)
  device.inputAV = device.parent.outputAV1
  device.clutchAV1 = device.outputAV1 * device.gearRatio1 * device.lockCoef
  device.clutchAV2 = device.outputAV1 * device.gearRatio2 * device.lockCoef
end

local function parkUpdateTorque(device, dt)
  device.torqueDiff = 0
  local outputAV1 = device.outputAV1

  if abs(outputAV1) < 100 then
    device.parkEngaged = 1
  end

  device.parkClutchAngle = min(max(device.parkClutchAngle + outputAV1 * dt, -device.maxParkClutchAngle), device.maxParkClutchAngle)
  device.outputTorque1 = -(device.parkClutchAngle * device.parkLockSpring + device.parkLockDamp * outputAV1) * device.parkEngaged

  device.gearWhineInputTorque = device.gearWhineInputTorqueSmoother:get(device.torqueDiff)
  device.gearWhineOutputTorque = device.gearWhineOutputTorqueSmoother:get(device.outputTorque1)
  --device.gearWhineInputAV = device.gearWhineInputAVSmoother:get(device.inputAV)
  device.gearWhineInputAV = device.gearWhineInputAVSmoother:get(max(device.clutchAV1, device.clutchAV2))
  device.gearWhineOutputAV = device.gearWhineOutputAVSmoother:get(device.outputAV1)
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  device.torqueUpdate = updateTorque

  if device.mode == "neutral" then
    device.velocityUpdate = neutralUpdateVelocity
    device.torqueUpdate = neutralUpdateTorque
  end

  if device.mode == "park" then
    device.velocityUpdate = parkUpdateVelocity
    device.torqueUpdate = parkUpdateTorque
    device.parkEngaged = 0
  end
end

local function applyDeformGroupDamage(device, damageAmount)
  device.damageLockTorqueCoef = max(device.damageLockTorqueCoef - linearScale(damageAmount, 0, 0.01, 0, 0.1), 0.2)
end

local function setPartCondition(device, subSystem, odometer, integrity, visual)
  device.wearLockTorqueCoef = linearScale(odometer, 30000000, 500000000, 1, 0.7)
  local integrityState = integrity
  if type(integrity) == "number" then
    local integrityValue = integrity
    integrityState = {
      damageLockTorqueCoef = linearScale(integrityValue, 1, 0, 1, 0.5)
    }
  end

  device.damageLockTorqueCoef = integrityState.damageLockTorqueCoef or 1
end

local function getPartCondition(device)
  local integrityState = {
    damageLockTorqueCoef = device.damageLockTorqueCoef
  }
  local integrityValueLockTorque = linearScale(device.damageLockTorqueCoef, 1, 0.5, 1, 0)
  local integrityValue = min(integrityValueLockTorque)

  return integrityValue, integrityState
end

local function validate(device)
  if not device.parent.deviceCategories.engine then
    log("E", "dctGearbox.validate", "Parent device is not an engine device...")
    log("E", "dctGearbox.validate", "Actual parent:")
    log("E", "dctGearbox.validate", powertrain.dumpsDeviceData(device.parent))
    return false
  end

  device.lockTorque = device.lockTorque or (device.parent.torqueData.maxTorque * 1.25 + device.parent.maxRPM * device.parent.inertia * math.pi / 30)

  local maxEngineTorque
  local maxEngineAV

  if device.parent and device.parent.deviceCategories.engine then
    local engine = device.parent
    local torqueData = engine:getTorqueData()
    maxEngineTorque = torqueData.maxTorque
    maxEngineAV = engine.maxAV
  else
    maxEngineTorque = 100
    maxEngineAV = 6000 * rpmToAV
  end

  device.maxExpectedInputTorque = maxEngineTorque
  device.invMaxExpectedInputTorque = 1 / maxEngineTorque
  device.invMaxExpectedOutputTorque = 0
  device.maxExpectedPower = maxEngineAV * device.maxExpectedInputTorque
  device.invMaxExpectedPower = 1 / device.maxExpectedPower
  device.maxExpectedOutputAV = maxEngineAV / device.minGearRatio
  device.invMaxExpectedOutputAV = 1 / device.maxExpectedOutputAV
  device.invMaxExpectedInputAV = 1 / maxEngineAV
  return true
end

local function setMode(device, mode)
  device.mode = mode
  selectUpdates(device)
end

local function setGearIndex1(device, index)
  device.gearIndex1 = min(max(index, device.minGearIndex), device.maxGearIndex)
  device.gearRatio1 = device.gearRatios[device.gearIndex1]

  powertrain.calculateTreeInertia()

  selectUpdates(device)
end

local function setGearIndex2(device, index)
  device.gearIndex2 = min(max(index, device.minGearIndex), device.maxGearIndex)
  device.gearRatio2 = device.gearRatios[device.gearIndex2]

  powertrain.calculateTreeInertia()

  selectUpdates(device)
end

local function setLock(device, enabled)
  device.lockCoef = enabled and 0 or 1
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

  local gearRatio1 = device.gearRatio1
  local gearRatio2 = device.gearRatio2
  local divisionSafeGearRatio1 = gearRatio1 ~= 0 and abs(gearRatio1) or (device.maxGearRatio * 2)
  local divisionSafeGearRatio2 = gearRatio2 ~= 0 and abs(gearRatio2) or (device.maxGearRatio * 2)

  device.cumulativeInertia1 = min(outputInertia / divisionSafeGearRatio1 / divisionSafeGearRatio1, device.parent.inertia * 0.5)
  device.cumulativeInertia2 = min(outputInertia / divisionSafeGearRatio2 / divisionSafeGearRatio2, device.parent.inertia * 0.5)

  device.lockSpring1 = device.lockSpringBase or (powertrain.stabilityCoef * powertrain.stabilityCoef * device.cumulativeInertia1) --Nm/rad
  device.lockSpring2 = device.lockSpringBase or (powertrain.stabilityCoef * powertrain.stabilityCoef * device.cumulativeInertia2) --Nm/rad

  device.lockDamp1 = device.lockDampRatio1 * sqrt(device.lockSpring1 * device.cumulativeInertia1)
  device.lockDamp2 = device.lockDampRatio2 * sqrt(device.lockSpring2 * device.cumulativeInertia2)

  device.maxClutchAngle1 = device.lockTorque / device.lockSpring1 --rad
  device.maxClutchAngle2 = device.lockTorque / device.lockSpring2 --rad

  device.parkLockSpring = device.parkLockSpringBase or (powertrain.stabilityCoef * powertrain.stabilityCoef * outputInertia * 0.5) --Nm/rad
  device.parkLockDamp = device.parkLockDampRatio * sqrt(device.parkLockSpring * outputInertia)
  device.maxParkClutchAngle = device.parkLockTorque / device.parkLockSpring

  device.cumulativeGearRatio = cumulativeGearRatio * (device.clutchRatio1 > device.clutchRatio2 and gearRatio1 or gearRatio2)
  device.maxCumulativeGearRatio = maxCumulativeGearRatio * device.maxGearRatio
end

local function resetSounds(device)
  device.gearWhineInputTorqueSmoother:reset()
  device.gearWhineOutputTorqueSmoother:reset()
  device.gearWhineInputAVSmoother:reset()
  device.gearWhineOutputAVSmoother:reset()

  device.gearWhineInputAV = 0
  device.gearWhineOutputAV = 0
  device.gearWhineInputTorque = 0
  device.gearWhineOutputTorque = 0
end

local function initSounds(device, jbeamData)
  local gearWhineOutputSample = jbeamData.gearWhineOutputEvent or "event:>Vehicle>Transmission>helical_01>twine_out"
  device.gearWhineOutputLoop = sounds.createSoundObj(gearWhineOutputSample, "AudioDefaultLoop3D", "GearWhineOut", device.transmissionNodeID or sounds.engineNode)

  local gearWhineInputSample = jbeamData.gearWhineInputEvent or "event:>Vehicle>Transmission>helical_01>twine_in"
  device.gearWhineInputLoop = sounds.createSoundObj(gearWhineInputSample, "AudioDefaultLoop3D", "GearWhineIn", device.transmissionNodeID or sounds.engineNode)

  bdebug.setNodeDebugText("Powertrain", device.transmissionNodeID or sounds.engineNode, device.name .. ": " .. gearWhineOutputSample)
  bdebug.setNodeDebugText("Powertrain", device.transmissionNodeID or sounds.engineNode, device.name .. ": " .. gearWhineInputSample)

  device.forwardInputPitchCoef = jbeamData.forwardInputPitchCoef or 1
  device.forwardOutputPitchCoef = jbeamData.forwardOutputPitchCoef or 1
  device.reverseInputPitchCoef = jbeamData.reverseInputPitchCoef or 0.7
  device.reverseOutputPitchCoef = jbeamData.reverseOutputPitchCoef or 0.7

  local inputAVSmoothing = jbeamData.gearWhineInputPitchCoefSmoothing or 50
  local outputAVSmoothing = jbeamData.gearWhineOutputPitchCoefSmoothing or 50
  local inputTorqueSmoothing = jbeamData.gearWhineInputVolumeCoefSmoothing or 10
  local outputTorqueSmoothing = jbeamData.gearWhineOutputVolumeCoefSmoothing or 10

  device.gearWhineInputTorqueSmoother = newExponentialSmoothing(inputTorqueSmoothing)
  device.gearWhineOutputTorqueSmoother = newExponentialSmoothing(outputTorqueSmoothing)
  device.gearWhineInputAVSmoother = newExponentialSmoothing(inputAVSmoothing)
  device.gearWhineOutputAVSmoother = newExponentialSmoothing(outputAVSmoothing)

  device.gearWhineInputAV = 0
  device.gearWhineOutputAV = 0
  device.gearWhineInputTorque = 0
  device.gearWhineOutputTorque = 0

  device.gearWhineOutputLoop:setParameter("c_gearboxMaxPower", device.maxExpectedPower * 0.001)
  device.gearWhineInputLoop:setParameter("c_gearboxMaxPower", device.maxExpectedPower * 0.001)
end

local function reset(device, jbeamData)
  device.gearRatio = 0
  device.friction = jbeamData.friction or 0
  device.cumulativeGearRatio = 1
  device.maxCumulativeGearRatio = 1

  device.outputAV1 = 0
  device.inputAV = 0
  device.outputTorque1 = 0
  device.isBroken = false

  device.lockCoef = 1

  device.gearRatio1 = 0
  device.gearRatio2 = 0

  device.clutchAngle1 = 0
  device.clutchAngle2 = 0
  device.clutchRatio1 = 1
  device.clutchRatio2 = 1
  device.clutchRatio = 1 --just used as a "max" of the two actual clutches for display purposes
  device.torqueDiff1 = 0
  device.torqueDiff2 = 0
  device.torqueDiff = 0

  device.parkClutchAngle = 0

  device.damageLockTorqueCoef = 1
  device.wearLockTorqueCoef = 1

  device:setGearIndex1(1)
  device:setGearIndex2(2)

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
    gearRatio = 0,
    friction = jbeamData.friction or 0,
    dynamicFriction = jbeamData.dynamicFriction or 0,
    torqueLossCoef = jbeamData.torqueLossCoef or 0,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    outputAV1 = 0,
    inputAV = 0,
    outputTorque1 = 0,
    isBroken = false,
    lockCoef = 1,
    electricsClutchRatio1Name = jbeamData.electricsClutchRatio1Name or "clutchRatio1",
    electricsClutchRatio2Name = jbeamData.electricsClutchRatio2Name or "clutchRatio2",
    gearRatios = {},
    gearRatio1 = 0,
    gearRatio2 = 0,
    clutchAngle1 = 0,
    clutchAngle2 = 0,
    clutchRatio1 = 1,
    clutchRatio2 = 1,
    clutchRatio = 1, --just used as a "max" of the two actual clutches for display purposes
    torqueDiff1 = 0,
    torqueDiff2 = 0,
    torqueDiff = 0,
    damageLockTorqueCoef = 1,
    wearLockTorqueCoef = 1,
    additionalEngineInertia = jbeamData.additionalEngineInertia or 0,
    reset = reset,
    initSounds = initSounds,
    resetSounds = resetSounds,
    updateSounds = updateSounds,
    setMode = setMode,
    validate = validate,
    setLock = setLock,
    calculateInertia = calculateInertia,
    setGearIndex1 = setGearIndex1,
    setGearIndex2 = setGearIndex2,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition
  }

  device.torqueLossCoef = clamp(device.torqueLossCoef, 0, 1)

  device.clutchStiffness = jbeamData.clutchStiffness or 1
  device.lockDampRatio1 = jbeamData.lockDampRatio1 or 0.15 --1 is critically damped
  device.lockDampRatio2 = jbeamData.lockDampRatio2 or 0.15 --1 is critically damped
  device.lockTorque = jbeamData.lockTorque
  device.lockSpringBase = jbeamData.lockSpring

  --gearbox park locking clutch
  device.parkClutchAngle = 0
  device.parkLockTorque = jbeamData.parkLockTorque or 1000 --Nm
  device.parkLockDampRatio = jbeamData.parkLockDampRatio or 0.4 --1 is critically damped
  device.parkLockSpringBase = jbeamData.parkLockSpring

  local forwardGears = {}
  local reverseGears = {}
  for _, v in pairs(jbeamData.gearRatios) do
    table.insert(v >= 0 and forwardGears or reverseGears, v)
  end

  device.maxGearIndex = 0
  device.minGearIndex = 0
  device.maxGearRatio = 0
  device.minGearRatio = 999999
  for i = 0, tableSize(forwardGears) - 1, 1 do
    device.gearRatios[i] = forwardGears[i + 1]
    device.maxGearIndex = max(device.maxGearIndex, i)
    device.maxGearRatio = max(device.maxGearRatio, abs(device.gearRatios[i]))
    if device.gearRatios[i] ~= 0 then
      device.minGearRatio = min(device.minGearRatio, abs(device.gearRatios[i]))
    end
  end
  local reverseGearCount = tableSize(reverseGears)
  for i = -reverseGearCount, -1, 1 do
    local index = -reverseGearCount - i - 1
    device.gearRatios[i] = reverseGears[abs(index)]
    device.minGearIndex = min(device.minGearIndex, index)
    device.maxGearRatio = max(device.maxGearRatio, abs(device.gearRatios[i]))
    if device.gearRatios[i] ~= 0 then
      device.minGearRatio = min(device.minGearRatio, abs(device.gearRatios[i]))
    end
  end
  device.gearCount = abs(device.maxGearIndex) + abs(device.minGearIndex)

  device.gearWhineCoefsOutput = {}
  local gearWhineCoefsOutput = jbeamData.gearWhineCoefsOutput or jbeamData.gearWhineCoefs
  if gearWhineCoefsOutput and type(gearWhineCoefsOutput) == "table" then
    local gearIndex = device.minGearIndex
    for _, v in pairs(gearWhineCoefsOutput) do
      device.gearWhineCoefsOutput[gearIndex] = v
      gearIndex = gearIndex + 1
    end
  else
    for i = device.minGearIndex, device.maxGearIndex, 1 do
      device.gearWhineCoefsOutput[i] = 0
    end
  end

  device.gearWhineCoefsInput = {}
  local gearWhineCoefsInput = jbeamData.gearWhineCoefsInput or jbeamData.gearWhineCoefs
  if gearWhineCoefsInput and type(gearWhineCoefsInput) == "table" then
    local gearIndex = device.minGearIndex
    for _, v in pairs(gearWhineCoefsInput) do
      device.gearWhineCoefsInput[gearIndex] = v
      gearIndex = gearIndex + 1
    end
  else
    for i = device.minGearIndex, device.maxGearIndex, 1 do
      device.gearWhineCoefsInput[i] = i < 0 and 0.3 or 0
    end
  end

  if jbeamData.gearboxNode_nodes and type(jbeamData.gearboxNode_nodes) == "table" then
    device.transmissionNodeID = jbeamData.gearboxNode_nodes[1]
  end

  if type(device.transmissionNodeID) ~= "number" then
    device.transmissionNodeID = nil
  end

  device:setGearIndex1(1)
  device:setGearIndex2(2)

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
