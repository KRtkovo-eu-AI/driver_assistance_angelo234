-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {}
M.deviceCategories = {clutchlike = true, clutch = true, hydraulicPowerSource = true, torqueConsumer = true}

local max = math.max
local abs = math.abs
local sqrt = math.sqrt

local twoPi = math.pi * 2
local invTwoPi = 1 / twoPi
local avToRPM = 9.549296596425384

local function updateVelocity(device, dt)
  ---------------- PUMP TYPES ------------------------
  if device.pumpType == "variableDisplacement" then
    --pump type: variable displacement
    --investigate if we need this here or if we can move to the actual hydraulic control logic
    local stallProtection = 1 - clamp((50 - device.inputAV) * 0.02, 0, 1) --0.02 == 1 / 50
    --baseline pressure for dynamic displacement scale
    local baselinePressure = device.pumpWorkingPressure - device.pumpRegulationRange
    device.currentDisplacement = device.pumpMaxDisplacement * device.pumpSmoother:get(linearScale(device.accumulatorPressure, baselinePressure, device.pumpWorkingPressure, 1, 0)) * stallProtection
  elseif device.pumpType == "fixedDisplacement" then
    --pump type: fixed displacement
    device.currentDisplacement = device.pumpMaxDisplacement
  end
  ----------------------------------------------------

  device.inputAV = device.parent.outputAV1 * device.gearRatio * device.isConnectedCoef

  ------------- Accumulator input -----------------------------
  device.pumpFlowRate = max(0, device.inputAV * invTwoPi * device.currentDisplacement)
  --------------------------------------------------------------

  device.accumulatorOilVolume = clamp(device.accumulatorOilVolume + (device.pumpFlowRate - device.accumulatorOutFlow) * dt, 0, device.accumulatorMaxVolume * 2)
end

local function updateTorque(device, dt)
  --accumulator
  device.accumulatorPressure = device.pumpWorkingPressure * device.accumulatorOilVolume * device.invAccumulatorMaxVolume

  device.torqueDiff = device.inputAV > 0 and (device.accumulatorPressure * device.currentDisplacement * invTwoPi * device.invGearRatio) or 0
  device.accumulatorOutFlow = 0

  for _, consumer in ipairs(device.connectedConsumers) do
    local consumerFlow, tankFlow = consumer:update(device.accumulatorPressure, dt)
    device.accumulatorOutFlow = device.accumulatorOutFlow + consumerFlow + tankFlow
  end

  device.reliefFullyOpenPressure = device.reliefOpeningPressure + device.reliefPressureRange --pre-compute TODO

  --gradually open relief flow as pressure builds, otherwise we get unstable pressure limit
  local reliefPressureScale = linearScale(device.accumulatorPressure, device.reliefOpeningPressure, device.reliefFullyOpenPressure, 0, 1)
  device.reliefFlow = 0.6 * device.reliefValveArea * reliefPressureScale * sqrt(2 * abs(device.accumulatorPressure) * 0.0012) -- 0.6: efficiency, 0.0012: oil density
end

local function updateGFX(device, dt)
  ---supply side---
  --send our own consumer pressure to further potential consumers of our own down the line
  --don't send more than our own supply pressure so that if a tank loses supply, this will propagate to the next consumer
  -- for the initial supply with a compressor, current and supply are the same, so it doesn't matter there
  electrics.values[device.hydraulicPTOConsumerPressureElectricsName] = device.accumulatorPressure
  electrics.values[device.hydraulicPTOConsumerMaxFlowRateElectricsName] = device.pumpFlowRate
  local ptoFlowOut = electrics.values[device.hydraulicPTOConsumerFlowElectricsName] or 0 --TODO this only supports a single remote acc
  device.accumulatorOilVolume = clamp(device.accumulatorOilVolume - ptoFlowOut * dt, 0, device.accumulatorMaxVolume * 2)
  ------

  if device.showDebugGraph then
    guihooks.graph({"Pressure", device.accumulatorPressure, 25000000, ""}, {"Pump Flow", device.pumpFlowRate, 0.005}, {"RPM", device.inputAV * avToRPM, 200}, {"Relief Flow", device.reliefFlow, 0.01}, {"PTO Flow Out", ptoFlowOut, 0.005})
  end
end

local function updateSounds(device, dt)
  local volumeFlowCoef = linearScale(abs(device.pumpFlowRate), device.pumpLoopVolumeFlowCoefMinFlow, device.pumpLoopVolumeFlowCoefMaxFlow, 0, 1)
  local volumeRaw = linearScale(device.accumulatorPressure, device.pumpLoopVolumeMinPressure, device.pumpLoopVolumeMaxPressure, device.pumpLoopVolumeMin, device.pumpLoopVolumeMax) * volumeFlowCoef
  local volume = device.volumeSmoothing:get(volumeRaw, dt)
  local pitchRaw = linearScale(device.pumpFlowRate, device.pumpLoopPitchMinFlow, device.pumpLoopPitchMaxFlow, 0, 1)
  local pitch = device.pitchSmoothing:get(pitchRaw, dt)
  local normalizedPressure = linearScale(device.accumulatorPressure, 0, device.pumpWorkingPressure, 0, 1)
  device.pumpLoopStartStopEnabled = device.pumpLoopStartStopEnabled or device.pumpFlowRate >= 0.000005
  local isLowFlow = device.pumpFlowRate < 0.000005
  obj:setVolumePitchCT(device.pumpSound, volume, pitch, normalizedPressure, device.pumpLoopStartStopEnabled and (isLowFlow and 0 or 1) or 0.5)

  for _, consumer in ipairs(device.connectedConsumers) do
    consumer:updateSounds(dt)
  end

  if device.showDebugGraphSound then
    guihooks.graph({"Pressure", device.accumulatorPressure, 55000000, ""}, {"Flow", device.pumpFlowRate, 0.005, ""}, {"Volume Coef", volumeFlowCoef, 1, ""}, {"Volume", volume, 1, ""}, {"Volume Raw", volumeRaw, 1, ""}, {"Pitch", pitch, 1, ""}, {"Pitch Raw", pitchRaw, 1, ""}, {"Input AV", device.inputAV, 400, ""})
  end
end

local function onCouplerAttached(device, nodeId, obj2id, obj2nodeId, attachForce)
  --if we just attached to our own consumer (ie we act as supply)
  if device.hydraulicPTOConsumerCouplerNodeLookup[nodeId] then
    --print("we are supply")
    --initialize the reported pressure and flow to 0, flow is then constantly updated from the consumer via the coupler and pressure by us
    electrics.values[device.hydraulicPTOConsumerPressureElectricsName] = 0
    electrics.values[device.hydraulicPTOConsumerFlowElectricsName] = 0
    electrics.values[device.hydraulicPTOConsumerMaxFlowRateElectricsName] = 0
    beamstate.sendExportCouplerData(obj2id, obj2nodeId, {electrics = {device.hydraulicPTOConsumerFlowElectricsName}})
  end
end

local function onCouplerDetached(device, nodeId, obj2id, obj2nodeId, breakForce)
  --if we just lost our own consumer
  if device.hydraulicPTOConsumerCouplerNodeLookup[nodeId] then
    --kill the flow value from it in our system
    electrics.values[device.hydraulicPTOConsumerFlowElectricsName] = nil
  end
end

local function setConnected(device, isConnected)
  device.isConnectedCoef = isConnected and 1 or 0
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  device.torqueUpdate = updateTorque
  device.updateGFX = updateGFX

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

local function initSounds(device, jbeamData)
  local pumpLoopEvent = jbeamData.pumpLoopEvent or "event:>Vehicle>Hydraulics>Pump_Big"
  local pumpLoopNode = jbeamData.pumpLoopNode and beamstate.nodeNameMap[jbeamData.pumpLoopNode]
  local pumpLoopNodeId = pumpLoopNode or device.parent.engineNodeID or 0
  device.pumpSound = obj:createSFXSource2(pumpLoopEvent, "AudioDefaultLoop3D", "pumpSound", pumpLoopNodeId, 1)
  obj:setVolumePitchCT(device.pumpSound, 0, 0, 0, 0)
  obj:playSFX(device.pumpSound)
  device.pumpLoopStartStopEnabled = false

  device.pumpLoopVolumeFlowCoefMinFlow = jbeamData.pumpLoopVolumeFlowCoefMinFlow or 0.00001
  device.pumpLoopVolumeFlowCoefMaxFlow = jbeamData.pumpLoopVolumeFlowCoefMaxFlow or 0.002
  device.pumpLoopVolumeMinPressure = jbeamData.pumpLoopVolumeMinPressure or 1000
  device.pumpLoopVolumeMaxPressure = jbeamData.pumpLoopVolumeMaxPressure or 20000000
  device.pumpLoopVolumeMin = jbeamData.pumpLoopVolumeMin or 0.5
  device.pumpLoopVolumeMax = jbeamData.pumpLoopVolumeMax or 1
  device.pumpLoopPitchMinFlow = jbeamData.pumpLoopPitchMinFlow or 0
  device.pumpLoopPitchMaxFlow = jbeamData.pumpLoopPitchMaxFlow or 0.00008

  local volumeSmoothingInRate = jbeamData.pumpLoopVolumeSmoothingInRate or 5
  local volumeSmoothingStartAccel = jbeamData.pumpLoopVolumeSmoothingStartAccel or 2
  local volumeSmoothingStopAccel = jbeamData.pumpLoopVolumeSmoothingStopAccel or 2
  local volumeSmoothingOutRate = jbeamData.pumpLoopVolumeSmoothingOutRate or 5

  local pitchSmoothingInRate = jbeamData.pumpLoopPitchSmoothingInRate or 5
  local pitchSmoothingStartAccel = jbeamData.pumpLoopPitchSmoothingStartAccel or 2
  local pitchSmoothingStopAccel = jbeamData.pumpLoopPitchSmoothingStopAccel or 2
  local pitchSmoothingOutRate = jbeamData.pumpLoopPitchSmoothingOutRate or 5

  device.volumeSmoothing = newTemporalSigmoidSmoothing(volumeSmoothingInRate, volumeSmoothingStartAccel, volumeSmoothingStopAccel, volumeSmoothingOutRate)
  device.pitchSmoothing = newTemporalSigmoidSmoothing(pitchSmoothingInRate, pitchSmoothingStartAccel, pitchSmoothingStopAccel, pitchSmoothingOutRate)

  for _, consumer in ipairs(device.connectedConsumers) do
    if consumer.initSounds then
      consumer:initSounds(device.consumerJbeamData[consumer.name])
    end
  end

  bdebug.setNodeDebugText("Hydraulics", pumpLoopNodeId, device.name .. " - Pump Loop: " .. (pumpLoopEvent or "no event"))
end

local function resetSounds(device, jbeamData)
  device.volumeSmoothing:reset()
  device.pitchSmoothing:reset()
  device.pumpLoopStartStopEnabled = false

  for _, consumer in ipairs(device.connectedConsumers) do
    if consumer.resetSounds then
      consumer:resetSounds(device.consumerJbeamData[consumer.name])
    end
  end
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
  device.isConnectedCoef = jbeamData.isConnected and 1 or 0

  device.isBroken = false
  device.wearFrictionCoef = 1
  device.damageFrictionCoef = 1

  device.pumpFlowRate = 0
  device.accumulatorOutFlow = 0

  device[device.outputTorqueName] = 0
  device[device.outputAVName] = 0
  device.accumulatorOilVolume = device.initialAccumulatorOilVolume

  device.pumpSmoother:reset()
  device.currentDisplacement = 0

  electrics.values[device.hydraulicPTOConsumerPressureElectricsName] = 0
  electrics.values[device.hydraulicPTOConsumerFlowElectricsName] = 0
  electrics.values[device.hydraulicPTOConsumerMaxFlowRateElectricsName] = 0

  selectUpdates(device)

  for _, consumer in ipairs(device.connectedConsumers) do
    consumer:reset(device.consumerJbeamData[consumer.name])
  end

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
    isConnectedCoef = jbeamData.isConnected and 1 or 0,
    visualShaftAngle = 0,
    virtualMassAV = 0,
    isBroken = false,
    nodeCid = jbeamData.node,
    showDebugGraph = jbeamData.showDebugGraph or false,
    showDebugGraphSound = jbeamData.showDebugGraphSound or false,
    reset = reset,
    onBreak = onBreak,
    validate = validate,
    calculateInertia = calculateInertia,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition,
    initSounds = initSounds,
    resetSounds = resetSounds,
    updateSounds = updateSounds,
    setConnected = setConnected,
    onCouplerAttached = onCouplerAttached,
    onCouplerDetached = onCouplerDetached,
    torqueDiff = 0
  }

  device.invGearRatio = 1 / device.gearRatio

  device.connectedConsumers = {}

  device.pumpMaxDisplacement = jbeamData.pumpMaxDisplacement or 0.0002
  device.pumpType = jbeamData.pumpType or "variableDisplacement" -- "fixedDisplacement"
  device.pumpRegulationRange = jbeamData.pumpRegulationRange or 10000000
  device.pumpWorkingPressure = jbeamData.pumpWorkingPressure or 25000000
  device.pumpSmoother = newExponentialSmoothing(5)

  device.accumulatorMaxVolume = jbeamData.accumulatorMaxVolume or 0.001
  device.invAccumulatorMaxVolume = 1 / device.accumulatorMaxVolume
  device.initialAccumulatorOilVolume = jbeamData.initialAccumulatorOilVolume or device.accumulatorMaxVolume * 0.9
  device.initialAccumulatorPressure = jbeamData.initialAccumulatorPressure or device.pumpWorkingPressure * 0.9

  device.reliefOpeningPressure = jbeamData.reliefOpeningPressure or device.pumpWorkingPressure * 1.05
  device.reliefPressureRange = jbeamData.reliefPressureRange or 2000000
  device.reliefValveArea = jbeamData.reliefValveArea or 0.00001

  device.currentDisplacement = device.pumpMaxDisplacement
  device.accumulatorPressure = device.initialAccumulatorPressure
  device.accumulatorOilVolume = device.initialAccumulatorOilVolume
  device.pumpFlowRate = 0
  device.accumulatorOutFlow = 0

  device.hydraulicPTOPressureElectricsName = jbeamData.hydraulicPTOPressureElectricsName or "hydraulicPTOPressure"
  device.hydraulicPTOMaxFlowRateElectricsName = jbeamData.hydraulicPTOMaxFlowRateElectricsName or "hydraulicPTOMaxFlowRate"
  device.hydraulicPTOFlowElectricsName = jbeamData.hydraulicPTOFlowElectricsName or "hydraulicPTOFlow"

  local hydraulicConsumerFactories = {}
  hydraulicConsumerFactories.hydraulicCylinder = require("powertrain/hydraulicCylinder")
  --hydraulicConsumerFactories.hydraulicRotator = require("powertrain/hydraulicRotator")

  device.consumerJbeamData = {}
  for _, ph in pairs(v.data.powertrainHydros or {}) do
    if ph.connectedPump == device.name then
      local consumerJbeamData = deepcopy(tableMerge(ph, v.data[ph.name] or {}))

      local consumerType = consumerJbeamData.type
      local factory = hydraulicConsumerFactories[consumerType]
      local consumer = factory.new(consumerJbeamData, device)

      table.insert(device.connectedConsumers, consumer)
      device.consumerJbeamData[consumerJbeamData.name] = consumerJbeamData
    end
  end

  device.outputTorqueName = "outputTorque1"
  device.outputAVName = "outputAV1"
  device[device.outputTorqueName] = 0
  device[device.outputAVName] = 0

  device.mode = "connected"

  device.breakTriggerBeam = jbeamData.breakTriggerBeam
  if device.breakTriggerBeam and device.breakTriggerBeam == "" then
    --get rid of the break beam if it's just an empty string (cancellation)
    device.breakTriggerBeam = nil
  end

  --base electrics name that needs to match between vehicles for a connection to work
  --one could make multiple pressure systems using different names
  device.hydraulicPTOPressureBaseElectricsName = jbeamData.hydraulicPTOPressureBaseSyncName or "hydraulicPTOPressure"
  device.hydraulicPTOFlowBaseElectricsName = jbeamData.hydraulicPTOFlowBaseSyncName or "hydraulicPTOFlow"
  device.hydraulicPTOMaxFlowRateBaseElectricsName = jbeamData.hydraulicPTOMaxFlowRateBaseSyncName or "hydraulicPTOMaxFlowRate"
  --these are the electrics names for communication with our CONSUMER
  --TODO this only supports a single consumer, to fix this, the electrics name needs to consist of both our id and the consumer id...
  device.hydraulicPTOConsumerPressureElectricsName = device.hydraulicPTOPressureBaseElectricsName .. "_" .. objectId
  device.hydraulicPTOConsumerFlowElectricsName = device.hydraulicPTOFlowBaseElectricsName .. "_" .. objectId
  device.hydraulicPTOConsumerMaxFlowRateElectricsName = device.hydraulicPTOMaxFlowRateBaseElectricsName .. "_" .. objectId

  electrics.values[device.hydraulicPTOConsumerPressureElectricsName] = 0
  electrics.values[device.hydraulicPTOConsumerFlowElectricsName] = 0
  electrics.values[device.hydraulicPTOConsumerMaxFlowRateElectricsName] = 0

  --used for looking up if a given coupler belongs to us
  device.hydraulicPTOConsumerCouplerNodeLookup = {}
  local ptoConsumerNodeNames = jbeamData.hydraulicPTOConsumerCouplerNodeNames or {}
  for _, nodeName in pairs(ptoConsumerNodeNames) do
    if beamstate.nodeNameMap[nodeName] then
      device.hydraulicPTOConsumerCouplerNodeLookup[beamstate.nodeNameMap[nodeName]] = true
    end
  end

  selectUpdates(device)

  return device
end

M.new = new

return M
