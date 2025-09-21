-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true}
M.deviceCategories = {shaft = true, differential = true}

local function updateVelocity(device, dt)
  local outputAV = 0
  --find average velocity, like a differential
  for i = 1, device.numberOfOutputPorts do
    outputAV = outputAV + device[device.outputAVNames[i]]
  end
  device.inputAV = outputAV * device.gearRatio / device.numberOfOutputPorts
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function updateTorque(device, dt)
  local inputTorque = device.parent[device.parentOutputTorqueName] * device.gearRatio
  local wheelSideInputAV = device.inputAV / device.gearRatio
  local wheelSideInputTorque = inputTorque / device.numberOfOutputPorts
  local viscousCoef = device.viscousCoef

  for i = 1, device.numberOfOutputPorts do
    device.torqueDiff[i] = device.torqueDiffSmoother[i]:get((device[device.outputAVNames[i]] - wheelSideInputAV) * viscousCoef)
    device[device.outputTorqueNames[i]] = wheelSideInputTorque - device.torqueDiff[i]
  end
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  device.torqueUpdate = updateTorque
end

local function validate(device)
  return true
end

local function calculateInertia(device)
  local outputInertia = 0
  local cumulativeGearRatio = nil
  local maxCumulativeGearRatio = nil
  if device.children and #device.children > 0 then
    for i = 1, device.numberOfOutputPorts, 1 do
      outputInertia = outputInertia + device.children[i].cumulativeInertia
      if (cumulativeGearRatio and cumulativeGearRatio ~= device.children[i].cumulativeGearRatio) or (maxCumulativeGearRatio and maxCumulativeGearRatio ~= device.children[i].maxCumulativeGearRatio) then
        log("W", "multiShaft.calculateInertia", string.format("Found non-matching gear ratios for multishaft outputs: A: '%.4f', B: '%.4f', A(max): '%.4f', B(max): '%.4f'", cumulativeGearRatio, device.children[i].cumulativeGearRatio, maxCumulativeGearRatio, device.children[i].maxCumulativeGearRatio))
      else
        cumulativeGearRatio = device.children[i].cumulativeGearRatio
        maxCumulativeGearRatio = device.children[i].maxCumulativeGearRatio
      end
    end
  end

  device.cumulativeInertia = outputInertia / device.gearRatio / device.gearRatio
  device.invCumulativeInertia = 1 / device.cumulativeInertia
  device.viscousCoef = device.viscousCoefBase or (100 * outputInertia)
  device.cumulativeGearRatio = cumulativeGearRatio * device.gearRatio
  device.maxCumulativeGearRatio = maxCumulativeGearRatio * device.gearRatio
end

local function reset(device, jbeamData)
  device.gearRatio = jbeamData.gearRatio or 1
  device.friction = jbeamData.friction or 0
  device.cumulativeInertia = 1
  device.invCumulativeInertia = 1
  device.cumulativeGearRatio = 1
  device.maxCumulativeGearRatio = 1

  device.inputAV = 0
  device.virtualMassAV = 0
  device.isBroken = false
  device.mode = "connected"

  for i = 1, device.numberOfOutputPorts, 1 do
    device.torqueDiff[i] = 0
    device.torqueDiffSmoother[i]:reset()
    device[device.outputTorqueNames[i]] = 0
    device[device.outputAVNames[i]] = 0
  end

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
    cumulativeInertia = 1,
    invCumulativeInertia = 1,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    electricsName = jbeamData.electricsName,
    visualShaftAVName = jbeamData.visualShaftAVName,
    inputAV = 0,
    virtualMassAV = 0,
    viscousCoefBase = jbeamData.viscousCoef,
    isBroken = false,
    availableModes = {"connected"},
    mode = "connected",
    defaultVirtualInertia = jbeamData.defaultVirtualInertia or nil, --meant to be nil if not specified manually
    reset = reset,
    validate = validate,
    calculateInertia = calculateInertia
  }

  device.numberOfOutputPorts = jbeamData.numberOfOutputPorts or 0
  device.outputPorts = {}
  device.torqueDiff = {}
  device.torqueDiffSmoother = {}
  device.outputTorqueNames = {}
  device.outputAVNames = {}
  device.requiredExternalInertiaOutputs = {}
  for i = 1, device.numberOfOutputPorts, 1 do
    device.torqueDiff[i] = 0
    device.torqueDiffSmoother[i] = newExponentialSmoothing(jbeamData.viscousSmoothing or 25)
    device.outputPorts[i] = true
    device.outputTorqueNames[i] = "outputTorque" .. tostring(i)
    device.outputAVNames[i] = "outputAV" .. tostring(i)
    device[device.outputTorqueNames[i]] = 0
    device[device.outputAVNames[i]] = 0
    table.insert(device.requiredExternalInertiaOutputs, i)
  end

  selectUpdates(device)

  return device
end

M.new = new

return M
