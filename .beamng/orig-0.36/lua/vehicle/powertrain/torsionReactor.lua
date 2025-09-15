-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true}
M.deviceCategories = {shaft = true}

local function updateVelocity(device, dt)
  device.inputAV = device.outputAV1 * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function updateTorque(device)
  device.outputTorque1 = device.parent[device.parentOutputTorqueName] * device.gearRatio
end

local function updateTorqueDisabled(device)
  device.outputTorque1 = device.parent[device.parentOutputTorqueName] * device.gearRatio
end

local function validate(device)
  if not (device.parent and not device.parent.isFake) then
    log("W", "torsionReactor.validate", "Can't find parent device...")
  end

  if not (device.torqueReactionNodes and #device.torqueReactionNodes == 3) then
    log("W", "torsionReactor.validate", "Wrong torque reaction node setup, node data:")
    log("W", "torsionReactor.validate", dumps(device.torqueReactionNodes))
    device.torqueUpdate = updateTorqueDisabled
  end

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

  device.cumulativeInertia = outputInertia / device.gearRatio / device.gearRatio
  device.invCumulativeInertia = device.cumulativeInertia > 0 and 1 / device.cumulativeInertia or 0
  device.cumulativeGearRatio = cumulativeGearRatio * device.gearRatio
  device.maxCumulativeGearRatio = maxCumulativeGearRatio * device.gearRatio
end

local function reset(device, jbeamData)
  device.gearRatio = jbeamData.gearRatio or 1
  device.cumulativeInertia = 1
  device.invCumulativeInertia = 1
  device.cumulativeGearRatio = 1
  device.maxCumulativeGearRatio = 1

  device.inputAV = 0
  device.outputTorque1 = 0
  device.outputAV1 = 0
  device.isBroken = false
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
    friction = 0,
    cumulativeInertia = 1,
    invCumulativeInertia = 1,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    electricsName = jbeamData.electricsName,
    visualShaftAVName = jbeamData.visualShaftAVName,
    inputAV = 0,
    outputTorque1 = 0,
    outputAV1 = 0,
    isBroken = false,
    velocityUpdate = updateVelocity,
    torqueUpdate = updateTorque,
    reset = reset,
    onBreak = nop,
    validate = validate,
    calculateInertia = calculateInertia
  }

  device.torqueReactionNodes = {}
  for _, v in pairs(jbeamData.torqueReactionNodes_nodes or {}) do
    if type(v) == "number" then
      table.insert(device.torqueReactionNodes, v)
    end
  end
  if #device.torqueReactionNodes <= 0 then
    device.torqueReactionNodes = nil
  end

  device.breakTriggerBeam = nil

  return device
end

M.new = new

return M
