-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true}
M.deviceCategories = {engine = true}

local min = math.min
local abs = math.abs

local avToRPM = 9.549296596425384

local function getTorqueData(device)
  return {maxRPM = 0, curves = {}, maxTorque = 0, maxPower = 0, maxTorqueRPM = 0, maxPowerRPM = 0, finalCurveName = 1, deviceName = device.name, vehicleID = obj:getId()}
end

local function sendTorqueData(device, data)
  if not data then
    data = device:getTorqueData()
  end
  guihooks.trigger("TorqueCurveChanged", data)
end

local function scaleFriction(device, friction)
  device.friction = device.friction * friction
end

local function scaleOutputTorque(device, state)
  device.outputTorqueState = device.outputTorqueState * state
end

local function disable(device)
  device.outputTorqueState = 0
  device.isDisabled = true
end

local function enable(device)
  device.outputTorqueState = 1
  device.isDisabled = false
end

local function lockUp(device)
  device.outputTorqueState = 0
  device.isDisabled = true
end

local function updateGFX(device, dt)
  device.outputRPM = device.outputAV1 * avToRPM
end

--velocity update is always nopped for engines

local function updateTorque(device, dt)
  local outputAV = device.outputAV1

  local friction = device.friction
  local dynamicFriction = device.dynamicFriction

  local actualTorque = device.desiredOutputTorque
  actualTorque = actualTorque * device.outputTorqueState
  local avSign = sign(outputAV)

  local frictionTorque = abs(friction * avSign + dynamicFriction * outputAV)
  --friction torque is limited for stability
  frictionTorque = min(frictionTorque, abs(outputAV) * device.inertia * 2000) * avSign

  device.outputTorque1 = actualTorque - frictionTorque
end

local function selectUpdates(device)
  device.velocityUpdate = nop
  device.torqueUpdate = updateTorque
end

local function validate(device)
  if not device.children or #device.children < 1 then
    device.clutchChild = {torqueDiff = 0}
  end

  table.insert(powertrain.engineData, {maxRPM = 0, torqueReactionNodes = device.torqueReactionNodes})

  selectUpdates(device)
  return true
end

local function onBreak(device)
  device:lockUp()
end

local function calculateInertia(device)
  local outputInertia = 0
  local cumulativeGearRatio = 1

  if device.children and #device.children > 0 then
    local child = device.children[1]
    outputInertia = child.cumulativeInertia
    cumulativeGearRatio = child.cumulativeGearRatio
  end

  device.cumulativeInertia = outputInertia
  device.cumulativeGearRatio = cumulativeGearRatio
end

local function reset(device, jbeamData)
  device.friction = jbeamData.friction or 0

  device.outputAV1 = 0
  device.outputTorque1 = 0

  device.dynamicFriction = jbeamData.dynamicFriction or 0

  device.desiredOutputTorque = 0

  device.inertia = jbeamData.inertia or 0.1

  device.outputTorqueState = 1
  device.isDisabled = false

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
    gearRatio = jbeamData.gearRatio,
    friction = jbeamData.friction or 0,
    cumulativeGearRatio = jbeamData.cumulativeGearRatio,
    isPhysicallyDisconnected = true,
    isPropulsed = true,
    outputAV1 = 0,
    inputAV = 0,
    outputTorque1 = 0,
    virtualMassAV = 0,
    isBroken = false,
    dynamicFriction = jbeamData.dynamicFriction or 0,
    inertia = jbeamData.inertia or 0.1,
    outputTorqueState = 1,
    desiredOutputTorque = 0,
    isDisabled = false,
    reset = reset,
    initSounds = nop,
    resetSounds = nop,
    updateSounds = nop,
    onBreak = nop,
    validate = validate,
    calculateInertia = calculateInertia,
    updateGFX = updateGFX,
    scaleFriction = scaleFriction,
    scaleOutputTorque = scaleOutputTorque,
    activateStarter = nop,
    deactivateStarter = nop,
    setIgnition = nop,
    cutIgnition = nop,
    sendTorqueData = sendTorqueData,
    getTorqueData = getTorqueData,
    lockUp = lockUp,
    disable = disable,
    enable = enable,
    initEngineSound = nop,
    setEngineSoundParameterList = nop,
    getSoundConfiguration = nop
  }

  device.torqueReactionNodes = jbeamData["torqueReactionNodes_nodes"]

  if device.torqueReactionNodes and #device.torqueReactionNodes == 3 then
    local pos1 = vec3(v.data.nodes[device.torqueReactionNodes[1]].pos)
    local pos2 = vec3(v.data.nodes[device.torqueReactionNodes[2]].pos)
    local pos3 = vec3(v.data.nodes[device.torqueReactionNodes[3]].pos)
    local avgPos = (((pos1 + pos2) / 2) + pos3) / 2
    device.visualPosition = {x = avgPos.x, y = avgPos.y, z = avgPos.z}
  end

  --dump(jbeamData)

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
