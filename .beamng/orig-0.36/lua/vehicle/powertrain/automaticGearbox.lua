-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true}
M.deviceCategories = {gearbox = true}
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

  local fixedVolumePartOutput = device.gearWhineOutputAV * device.invMaxExpectedOutputAV * device.gearWhineFixedCoefOutput --normalized AV
  local powerVolumePartOutput = device.gearWhineOutputAV * device.gearWhineOutputTorque * device.invMaxExpectedPower * device.gearWhinePowerCoefOutput --normalized power
  local volumeOutput = clamp(abs(fixedVolumePartOutput) + abs(powerVolumePartOutput), 0, 10) * gearWhineCoefOutput

  local fixedVolumePartInput = device.gearWhineInputAV * device.gearWhineFixedCoefInput * device.invMaxExpectedInputAV --normalized AV
  local powerVolumePartInput = device.gearWhineInputAV * device.gearWhineInputTorque * device.invMaxExpectedPower * device.gearWhinePowerCoefInput --normalized power
  local volumeInput = clamp(abs(fixedVolumePartInput) + abs(powerVolumePartInput), 0, 10) * gearWhineCoefInput

  local inputPitchCoef = device.gearRatio >= 0 and device.forwardInputPitchCoef or device.reverseInputPitchCoef
  local outputPitchCoef = device.gearRatio >= 0 and device.forwardOutputPitchCoef or device.reverseOutputPitchCoef
  local pitchInput = clamp(abs(device.gearWhineInputAV) * avToRPM * inputPitchCoef, 0, 10000000)
  local pitchOutput = clamp(abs(device.gearWhineOutputAV) * avToRPM * outputPitchCoef, 0, 10000000)

  local inputLoad = device.gearWhineInputTorque * device.invMaxExpectedInputTorque
  local outputLoad = device.gearWhineOutputTorque * device.invMaxExpectedOutputTorque
  local outputRPMSign = sign(device.gearWhineOutputAV)

  device.gearWhineOutputLoop:setVolumePitch(volumeOutput, pitchOutput, outputLoad, outputRPMSign)
  device.gearWhineInputLoop:setVolumePitch(volumeInput, pitchInput, inputLoad, outputRPMSign)

  --print (string.format(" INPUT  volumeInput =%0.4f : pitchInput =%6.0d : inputLoad =%0.4f : outputRPMSign=%0.4f", volumeInput, pitchInput, inputLoad, outputRPMSign))
  --print (string.format(" OUTPUT volumeOutput=%0.4f : pitchOutput=%6.0d : outputLoad=%0.4f : outputRPMSign=%0.4f", volumeOutput, pitchOutput, outputLoad, outputRPMSign))
end

local function updateVelocity(device, dt)
  device.inputAV = device.outputAV1 * device.gearRatio * device.lockCoef
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function updateTorque(device, dt)
  local inputTorque = device.parent[device.parentOutputTorqueName]
  local inputAV = device.inputAV
  local outputAV1 = device.outputAV1
  local signGearRatio = sign(device.gearRatio)

  local oneWayTorque = device.oneWayTorqueSmoother:get(min(max(device.oneWayViscousCoef * outputAV1, -device.oneWayViscousTorque), device.oneWayViscousTorque))
  device.oneWayTorqueSmoother:set(outputAV1 * signGearRatio < 0 and oneWayTorque or 0)
  oneWayTorque = device.oneWayTorqueSmoother:value() * signGearRatio

  --reused for transbrake
  device.parkClutchAngle = min(max(device.parkClutchAngle + outputAV1 * dt, -device.maxParkClutchAngle), device.maxParkClutchAngle)
  local friction = (device.friction * clamp(inputAV, -1, 1) + device.dynamicFriction * inputAV + device.torqueLossCoef * inputTorque) * device.wearFrictionCoef * device.damageFrictionCoef
  local outputTorque = ((inputTorque * device.shiftLossCoef - friction) * device.gearRatio - oneWayTorque * signGearRatio) * device.lockCoef - (device.parkClutchAngle * device.parkLockSpring + device.parkLockDamp * outputAV1) * (1 - device.lockCoef)
  device.outputTorque1 = outputTorque

  device.gearWhineInputTorque = device.gearWhineInputTorqueSmoother:get(inputTorque)
  device.gearWhineOutputTorque = device.gearWhineOutputTorqueSmoother:get(outputTorque)
  device.gearWhineInputAV = device.gearWhineInputAVSmoother:get(inputAV)
  device.gearWhineOutputAV = device.gearWhineOutputAVSmoother:get(outputAV1)
end

local function neutralUpdateVelocity(device, dt)
  device.inputAV = device.virtualMassAV
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function neutralUpdateTorque(device, dt)
  local inputAV = device.inputAV
  local outputAV1 = device.outputAV1
  local outputTorque = device.parent[device.parentOutputTorqueName] - (device.friction * clamp(inputAV, -1, 1) + device.dynamicFriction * inputAV) * device.wearFrictionCoef * device.damageFrictionCoef
  device.virtualMassAV = device.virtualMassAV + outputTorque * device.invCumulativeInertia * dt

  --reused for transbrake
  device.parkClutchAngle = min(max(device.parkClutchAngle + outputAV1 * dt, -device.maxParkClutchAngle), device.maxParkClutchAngle)
  device.outputTorque1 = -(device.parkClutchAngle * device.parkLockSpring + device.parkLockDamp * outputAV1) * (1 - device.lockCoef)

  device.gearWhineInputTorque = device.gearWhineInputTorqueSmoother:get(device.parent[device.parentOutputTorqueName])
  device.gearWhineOutputTorque = device.gearWhineOutputTorqueSmoother:get(0)
  device.gearWhineInputAV = device.gearWhineInputAVSmoother:get(inputAV)
  device.gearWhineOutputAV = device.gearWhineOutputAVSmoother:get(outputAV1)
end

local function parkUpdateVelocity(device, dt)
  device.inputAV = device.virtualMassAV
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function parkUpdateTorque(device, dt)
  local inputAV = device.inputAV
  local outputAV1 = device.outputAV1
  local outputTorque = device.parent[device.parentOutputTorqueName] - (device.friction * clamp(inputAV, -1, 1) + device.dynamicFriction * inputAV) * device.wearFrictionCoef * device.damageFrictionCoef
  device.virtualMassAV = device.virtualMassAV + outputTorque * device.invCumulativeInertia * dt

  device.parkClutchAngle = min(max(device.parkClutchAngle + outputAV1 * dt, -device.maxParkClutchAngle), device.maxParkClutchAngle)
  device.outputTorque1 = -(device.parkClutchAngle * device.parkLockSpring + device.parkLockDamp * outputAV1) * device.parkEngaged

  device.gearWhineInputTorque = device.gearWhineInputTorqueSmoother:get(device.parent[device.parentOutputTorqueName])
  device.gearWhineOutputTorque = device.gearWhineOutputTorqueSmoother:get(0)
  device.gearWhineInputAV = device.gearWhineInputAVSmoother:get(inputAV)
  device.gearWhineOutputAV = device.gearWhineOutputAVSmoother:get(outputAV1)
end

local function updateGFX(device, dt)
  --interpolate gear ratio to simulate the opening/closing clutches of the auto gearbox
  if device.gearRatio ~= device.desiredGearRatio then
    local difference = device.desiredGearRatio - device.gearRatio
    local gearRatioChangeRateCoef = device.damageGearRatioChangeRateCoef * device.wearGearRatioChangeRateCoef
    local change = min(device.gearRatioChangeRate * dt * gearRatioChangeRateCoef, abs(difference))
    device.gearRatio = device.gearRatio + change * sign(difference)
    if device.gearRatio == device.desiredGearRatio then
      local maxExpectedOutputTorque = device.maxExpectedInputTorque * device.gearRatio
      device.invMaxExpectedOutputTorque = maxExpectedOutputTorque ~= 0 and 1 / maxExpectedOutputTorque or 0
      powertrain.calculateTreeInertia()
      device.shiftLossCoef = 1
    end
  --print(string.format("Gearratio: %.3f / %.3f", device.gearRatio, device.desiredGearRatio))
  end

  device.isShiftingUp = device.gearRatio > device.desiredGearRatio
  device.isShiftingDown = device.gearRatio < device.desiredGearRatio
  device.isShifting = device.isShiftingUp or device.isShiftingDown

  if device.mode == "park" and abs(device.outputAV1) < 100 then
    device.parkEngaged = 1
  end
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  device.torqueUpdate = updateTorque
  device.parkClutchAngle = 0

  if device.mode == "park" then
    device.velocityUpdate = parkUpdateVelocity
    device.torqueUpdate = parkUpdateTorque
    device.parkEngaged = 0
    --make sure the virtual mass has the right AV
    device.virtualMassAV = device.inputAV
  end

  if device.mode == "neutral" then
    device.velocityUpdate = neutralUpdateVelocity
    device.torqueUpdate = neutralUpdateTorque
    --make sure the virtual mass has the right AV
    device.virtualMassAV = device.inputAV
  end
end

local function applyDeformGroupDamage(device, damageAmount)
  device.damageFrictionCoef = device.damageFrictionCoef + linearScale(damageAmount, 0, 0.01, 0, 0.1)
  device.damageGearRatioChangeRateCoef = max(device.damageGearRatioChangeRateCoef - linearScale(damageAmount, 0, 0.01, 0, 0.1), 0.2)
end

local function setPartCondition(device, subSystem, odometer, integrity, visual)
  device.wearFrictionCoef = linearScale(odometer, 30000000, 1000000000, 1, 2)
  device.wearGearRatioChangeRateCoef = linearScale(odometer, 30000000, 500000000, 1, 0.2)

  local integrityState = integrity
  if type(integrity) == "number" then
    local integrityValue = integrity
    integrityState = {
      damageFrictionCoef = linearScale(integrityValue, 1, 0, 1, 50),
      damageGearRatioChangeRateCoef = linearScale(integrityValue, 1, 0, 1, 0.2),
      isBroken = false
    }
  end

  device.damageFrictionCoef = integrityState.damageFrictionCoef or 1
  device.damageGearRatioChangeRateCoef = integrityState.damageGearRatioChangeRateCoef or 1

  if integrityState.isBroken then
    device:onBreak()
  end
end

local function getPartCondition(device)
  local integrityState = {
    damageFrictionCoef = device.damageFrictionCoef,
    damageGearRatioChangeRateCoef = device.damageGearRatioChangeRateCoef,
    isBroken = device.isBroken
  }
  local integrityValueFriction = linearScale(device.damageFrictionCoef, 1, 50, 1, 0)
  local integrityValueGearRatioChange = linearScale(device.damageGearRatioChangeRateCoef, 1, 0.2, 1, 0)

  local integrityValue = min(integrityValueFriction, integrityValueGearRatioChange)
  if device.isBroken then
    integrityValue = 0
  end
  return integrityValue, integrityState
end

local function validate(device)
  if not device.parent.deviceCategories.viscouscoupling then
    log("E", "automaticGearbox.validate", "Parent device is not a viscous coupling device...")
    log("E", "automaticGearbox.validate", "Actual parent:")
    log("E", "automaticGearbox.validate", powertrain.dumpsDeviceData(device.parent))
    return false
  end

  local maxEngineTorque
  local maxEngineAV

  if device.parent.parent and device.parent.parent.deviceCategories.engine then
    local engine = device.parent.parent
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
  device.invMaxExpectedPower = 1 / (maxEngineAV * device.maxExpectedInputTorque)
  device.maxExpectedOutputAV = maxEngineAV / device.minGearRatio
  device.invMaxExpectedOutputAV = 1 / device.maxExpectedOutputAV
  device.invMaxExpectedInputAV = 1 / maxEngineAV

  return true
end

local function setMode(device, mode)
  device.mode = mode
  selectUpdates(device)
end

local function setGearIndex(device, index, gearChangeTime)
  device.gearIndex = min(max(index, device.minGearIndex), device.maxGearIndex)
  device.desiredGearRatio = device.gearRatios[device.gearIndex]
  device.gearRatioChangeRate = abs((device.desiredGearRatio - device.gearRatio) / (max(device.minimumGearChangeTime, gearChangeTime or 0)))
  if abs(device.gearRatio - device.desiredGearRatio) > 0.01 then
    device.shiftLossCoef = device.shiftEfficiency
  end

  selectUpdates(device)
end

local function setLock(device, enabled)
  device.lockCoef = enabled and 0 or 1
  device.parkClutchAngle = 0
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

  local gearRatio = device.gearRatio ~= 0 and abs(device.gearRatio) or (device.maxGearRatio * 2)
  device.cumulativeInertia = outputInertia / gearRatio / gearRatio
  device.invCumulativeInertia = 1 / device.cumulativeInertia

  device.parkLockSpring = device.parkLockSpringBase or (powertrain.stabilityCoef * powertrain.stabilityCoef * outputInertia * 0.5) --Nm/rad
  device.parkLockDamp = device.parkLockDampRatio * sqrt(device.parkLockSpring * outputInertia)
  device.maxParkClutchAngle = device.parkLockTorque / device.parkLockSpring --rad

  device.cumulativeGearRatio = cumulativeGearRatio * device.gearRatio
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

  device.gearWhineFixedCoefOutput = jbeamData.gearWhineFixedCoefOutput or 0.7
  device.gearWhinePowerCoefOutput = 1 - device.gearWhineFixedCoefOutput
  device.gearWhineFixedCoefInput = jbeamData.gearWhineFixedCoefInput or 0.4
  device.gearWhinePowerCoefInput = 1 - device.gearWhineFixedCoefInput
end

local function reset(device, jbeamData)
  device.gearRatio = jbeamData.gearRatio or 1
  device.friction = jbeamData.friction or 0
  device.cumulativeInertia = 1
  device.cumulativeGearRatio = 1
  device.maxCumulativeGearRatio = 1

  device.outputAV1 = 0
  device.inputAV = 0
  device.outputTorque1 = 0
  device.virtualMassAV = 0
  device.isBroken = false

  device.lockCoef = 1

  device.shiftLossCoef = 1
  device.damageGearRatioChangeRateCoef = 1
  device.wearGearRatioChangeRateCoef = 1

  device.desiredGearRatio = 0
  device.isShifting = false
  device.isShiftingUp = false
  device.isShiftingDown = false
  device.mode = "drive"

  --gearbox park locking clutch
  device.parkClutchAngle = 0

  device.wearFrictionCoef = 1
  device.damageFrictionCoef = 1

  --one way viscous coupling (prevents rolling backwards)
  device.oneWayTorqueSmoother:reset()
  device:setGearIndex(0)

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
    cumulativeInertia = 1,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    outputAV1 = 0,
    inputAV = 0,
    outputTorque1 = 0,
    virtualMassAV = 0,
    isBroken = false,
    lockCoef = 1,
    shiftEfficiency = jbeamData.shiftEfficiency or 0.5,
    shiftLossCoef = 1,
    damageGearRatioChangeRateCoef = 1,
    wearGearRatioChangeRateCoef = 1,
    parkLockSpringBase = jbeamData.parkLockSpring,
    gearRatios = {},
    desiredGearRatio = 0,
    isShifting = false,
    isShiftingUp = false,
    isShiftingDown = false,
    minimumGearChangeTime = jbeamData.gearChangeTime or 0.5, --time in s it takes to interpolate from one to another gear ratio when shifting (simulates clutches inside the auto transmission)
    mode = "drive",
    reset = reset,
    setMode = setMode,
    validate = validate,
    calculateInertia = calculateInertia,
    setGearIndex = setGearIndex,
    updateGFX = updateGFX,
    initSounds = initSounds,
    resetSounds = resetSounds,
    updateSounds = updateSounds,
    setLock = setLock,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition
  }

  device.torqueLossCoef = clamp(device.torqueLossCoef, 0, 1)

  local forwardGears = {}
  local reverseGears = {}
  for k, v in pairs(jbeamData.gearRatios) do
    if type(k) == "number" then
      table.insert(v >= 0 and forwardGears or reverseGears, v)
    end
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
      device.gearWhineCoefsInput[i] = 0
    end
  end

  --gearbox park locking clutch
  device.parkClutchAngle = 0
  device.parkLockTorque = jbeamData.parkLockTorque or 1000 --Nm
  device.parkLockDampRatio = jbeamData.parkLockDampRatio or 0.4 --1 is critically damped

  --one way viscous coupling (prevents rolling backwards)
  device.oneWayViscousCoef = jbeamData.oneWayViscousCoef or 5
  device.oneWayViscousTorque = jbeamData.oneWayViscousTorque or device.oneWayViscousCoef * 25
  device.oneWayTorqueSmoother = newExponentialSmoothing(jbeamData.oneWayViscousSmoothing or 50)

  if jbeamData.gearboxNode_nodes and type(jbeamData.gearboxNode_nodes) == "table" then
    device.transmissionNodeID = jbeamData.gearboxNode_nodes[1]
  end

  if type(device.transmissionNodeID) ~= "number" then
    device.transmissionNodeID = nil
  end

  device:setGearIndex(0)

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
