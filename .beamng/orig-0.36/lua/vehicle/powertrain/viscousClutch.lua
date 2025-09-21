-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true}
M.deviceCategories = {clutchlike = true, viscouscoupling = true}
M.requiredExternalInertiaOutputs = {1}

local rpmToAV = 0.104719755

local max = math.max
local min = math.min

local function updateVelocity(device, dt)
  device.inputAV = device.parent.outputAV1
end

local function updateTorque(device, dt)
  local avDiff = device.inputAV - device.outputAV1
  local clutchRatio = device.clutchRatio

  if device.cutInAV then
    local internalClutchRatio = min(max((device.inputAV - device.cutInAV) / (device.stallAV), 0), 1)
    clutchRatio = min(clutchRatio, internalClutchRatio)
  end

  device.torqueDiff = device.torqueDiffSmoother:get(min(max((device.viscousCoef * avDiff), -device.viscousTorque), device.viscousTorque) * clutchRatio)
  device.outputTorque1 = device.torqueDiff
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  device.torqueUpdate = updateTorque
end

local function validate(device)
  if not device.parent.deviceCategories.engine then
    log("E", "viscousClutch.validate", "Parent device is not an engine device...")
    log("E", "viscousClutch.validate", "Actual parent:")
    log("E", "viscousClutch.validate", powertrain.dumpsDeviceData(device.parent))
    return false
  end

  device.viscousTorque = device.viscousTorque or (device.parent.torqueData.maxTorque * 1.25 + device.parent.maxRPM * device.parent.inertia * math.pi / 30)
  return true
end

local function setMode(device, mode)
  device.mode = mode
  selectUpdates(device)
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

  device.cumulativeInertia = outputInertia
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
  device.isBroken = false
  device.clutchRatio = 1
  device.torqueDiff = 0

  device.viscousCoef = jbeamData.viscousCoef or 10 --Nm/rad/s

  device.torqueDiffSmoother:reset()

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
    isBroken = false,
    clutchRatio = 1,
    torqueDiff = 0,
    reset = reset,
    setMode = setMode,
    validate = validate,
    calculateInertia = calculateInertia
  }

  device.torqueDiffSmoother = newExponentialSmoothing(jbeamData.viscousSmoothing or 25)
  device.viscousCoef = jbeamData.viscousCoef or 10 --Nm/rad/s
  device.viscousTorque = jbeamData.viscousTorque

  device.cutInAV = (jbeamData.cutInRPM * rpmToAV) or nil
  device.stallAV = (jbeamData.stallRPM * rpmToAV) or 1

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
