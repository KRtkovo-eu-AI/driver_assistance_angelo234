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

local function updateVelocity(device, dt)
  device.inputAV = device.outputAV1 * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function updateTorque(device)
  device.outputTorque1 = (device.parent[device.parentOutputTorqueName] - (device.friction * clamp(device.inputAV, -1, 1) + device.dynamicFriction * device.inputAV + device.torqueLossCoef * device.parent[device.parentOutputTorqueName]) * device.wearFrictionCoef * device.damageFrictionCoef) * device.gearRatio
end

local function setGearIndex(device, index, omitInertiaCaculation)
  device.gearIndex = min(max(index, device.minGearIndex), device.maxGearIndex)
  device.gearRatio = device.gearRatios[device.gearIndex]

  if not omitInertiaCaculation then
    powertrain.calculateTreeInertia()
  end
end

local function updateSimpleControlButtons(device)
  if #device.availableModes > 1 and device.uiSimpleModeControl then
    local modeIconLookup = {
      high = "powertrain_rangebox_high",
      low = "powertrain_rangebox_low"
    }
    extensions.ui_simplePowertrainControl.setButton("powertrain_device_mode_shortcut_" .. device.name, device.uiName, modeIconLookup[device.mode], nil, nil, string.format("powertrain.toggleDeviceMode(%q)", device.name))
  end
end

local function setMode(device, mode)
  device.mode = mode
  if mode == "high" then
    device:setGearIndex(device.highGearIndex)
  elseif mode == "low" then
    device:setGearIndex(device.lowGearIndex)
  end
  device:updateSimpleControlButtons()
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
  local cumulativeGearRatio = 1
  local maxCumulativeGearRatio = 1
  if device.children and #device.children > 0 then
    local child = device.children[1]
    outputInertia = child.cumulativeInertia
    cumulativeGearRatio = child.cumulativeGearRatio
    maxCumulativeGearRatio = child.maxCumulativeGearRatio
  end

  local gearRatio = device.gearRatio ~= 0 and abs(device.gearRatio) or device.maxGearRatio
  device.cumulativeInertia = outputInertia / gearRatio / gearRatio
  device.cumulativeGearRatio = cumulativeGearRatio * device.gearRatio
  device.maxCumulativeGearRatio = maxCumulativeGearRatio * device.maxGearRatio
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
    gearIndex = 1,
    gearRatios = {},
    reset = reset,
    torqueUpdate = updateTorque,
    velocityUpdate = updateVelocity,
    setMode = setMode,
    calculateInertia = calculateInertia,
    setGearIndex = setGearIndex,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition,
    updateSimpleControlButtons = updateSimpleControlButtons
  }

  device.torqueLossCoef = clamp(device.torqueLossCoef, 0, 1)

  local forwardGears = {}
  local reverseGears = {}
  for _, v in pairs(jbeamData.gearRatios) do
    table.insert(v >= 0 and forwardGears or reverseGears, v)
  end

  device.maxGearIndex = 0
  device.minGearIndex = 0
  device.minGearRatio = 1000
  device.maxGearRatio = 0
  for i = 0, tableSize(forwardGears) - 1, 1 do
    device.gearRatios[i] = forwardGears[i + 1]
    if device.gearRatios[i] > device.maxGearRatio then
      device.lowGearIndex = i
    end
    if device.gearRatios[i] < device.minGearRatio then
      device.highGearIndex = i
    end
    device.maxGearIndex = max(device.maxGearIndex, i)
    device.maxGearRatio = max(device.maxGearRatio, abs(device.gearRatios[i]))
    device.minGearRatio = min(device.minGearRatio, abs(device.gearRatios[i]))
  end
  for i = -1, -tableSize(reverseGears), -1 do
    device.gearRatios[i] = reverseGears[abs(i)]
    if device.gearRatios[i] > device.maxGearRatio then
      device.lowGearIndex = i
    end
    if device.gearRatios[i] < device.minGearRatio then
      device.highGearIndex = i
    end
    device.minGearIndex = min(device.minGearIndex, i)
    device.maxGearRatio = max(device.maxGearRatio, abs(device.gearRatios[i]))
    device.minGearRatio = min(device.minGearRatio, abs(device.gearRatios[i]))
  end
  device.gearCount = abs(device.maxGearIndex) + abs(device.minGearIndex) + 1

  device:setGearIndex(0, true)

  device.availableModes = {"high", "low"}
  device.mode = device.lowGearIndex == 0 and "low" or "high"

  return device
end

M.new = new

return M
