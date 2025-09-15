-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true, [2] = true}
M.deviceCategories = {differential = true}
M.requiredExternalInertiaOutputs = {1, 2}

local max = math.max
local min = math.min
local abs = math.abs
local sign = sign
local sqrt = math.sqrt

local function updateVelocity(device)
  --calculate input AV based on the two differential output AVs (weighted by base torque split as the split is created by different spider gear ratios on each output)
  --inputAV is the carrier AV * gear ratio
  device.inputAV = (device.outputAV1 * device.diffTorqueSplitA + device.outputAV2 * device.diffTorqueSplitB) * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function openUpdateTorque(device)
  local inputAV = device.inputAV
  --divide out the gear ratio to get the carrier AV
  local outputAV1diff = device.outputAV1 - inputAV * device.invGearRatio
  local outputAV2diff = device.outputAV2 - inputAV * device.invGearRatio
  local outputAVdiff = device.outputAV1 - device.outputAV2
  local absMaxOutputAVdiff = max(abs(outputAV1diff), abs(outputAV2diff))

  local friction = (device.friction * clamp(inputAV, -1, 1) + device.dynamicFriction * inputAV + device.torqueLossCoef * device.parent[device.parentOutputTorqueName]) * device.wearFrictionCoef * device.damageFrictionCoef
  local inputTorque = (device.parent[device.parentOutputTorqueName] - friction) * (1 - min(device.speedLimitCoef * absMaxOutputAVdiff * absMaxOutputAVdiff * absMaxOutputAVdiff, 1)) * device.gearRatio

  --some small locking torque due to friction effects
  local openTorque = 0.034 * abs(inputTorque)
  device.inputTorque = inputTorque
  device.outputTorque1 = inputTorque * device.diffTorqueSplitA - openTorque * clamp(outputAVdiff, -0.1, 0.1)
  device.outputTorque2 = inputTorque * device.diffTorqueSplitB - openTorque * clamp(outputAVdiff, -0.1, 0.1)
end

local function LSDUpdateTorque(device, dt)
  local inputAV = device.inputAV
  --divide out the gear ratio to get the carrier AV
  local outputAV1diff = device.outputAV1 - inputAV * device.invGearRatio
  local outputAV2diff = device.outputAV2 - inputAV * device.invGearRatio
  local outputAVdiff = device.outputAV1 - device.outputAV2
  local absMaxOutputAVdiff = max(abs(outputAV1diff), abs(outputAV2diff))

  local friction = (device.friction * clamp(inputAV, -1, 1) + device.dynamicFriction * inputAV + device.torqueLossCoef * device.parent[device.parentOutputTorqueName]) * device.wearFrictionCoef * device.damageFrictionCoef
  local inputTorque = (device.parent[device.parentOutputTorqueName] - friction) * (1 - min(device.speedLimitCoef * absMaxOutputAVdiff * absMaxOutputAVdiff * absMaxOutputAVdiff, 1)) * device.gearRatio

  --lsd works with an initial preload torque + input torque sensing locking ability
  local torqueSign = sign(inputTorque)
  local lsdLockCoef = min(max(torqueSign, 0) * device.lsdLockCoef - min(torqueSign, 0) * device.lsdRevLockCoef, 1)
  --active locking support added but not active unless appropriate controller is used
  local lsdTorque = device.lsdPreload + lsdLockCoef * abs(inputTorque) + device.activeLockTorque * device.activeLockCoef

  device.maxDiffAngle = sqrt(lsdTorque / device.lockSpring)
  device.diffAngle = clamp(device.diffAngle + outputAVdiff * dt, -device.maxDiffAngle, device.maxDiffAngle)
  local lockTorque = clamp(device.diffAngle * device.diffAngle * device.lockSpring * sign(device.diffAngle), -device.lockTorque, device.lockTorque)

  device.inputTorque = inputTorque
  device.outputTorque1 = inputTorque * device.diffTorqueSplitA - lockTorque
  device.outputTorque2 = inputTorque * device.diffTorqueSplitB + lockTorque
end

local function viscousLSDUpdateTorque(device)
  local inputAV = device.inputAV
  --divide out the gear ratio to get the carrier AV
  local outputAV1diff = device.outputAV1 - inputAV * device.invGearRatio
  local outputAV2diff = device.outputAV2 - inputAV * device.invGearRatio
  local outputAVdiff = device.outputAV1 - device.outputAV2
  local absMaxOutputAVdiff = max(abs(outputAV1diff), abs(outputAV2diff))

  local friction = (device.friction * clamp(inputAV, -1, 1) + device.dynamicFriction * inputAV + device.torqueLossCoef * device.parent[device.parentOutputTorqueName]) * device.wearFrictionCoef * device.damageFrictionCoef
  local inputTorque = (device.parent[device.parentOutputTorqueName] - friction) * (1 - min(device.speedLimitCoef * absMaxOutputAVdiff * absMaxOutputAVdiff * absMaxOutputAVdiff, 1)) * device.gearRatio

  --vlsd works with speed sensitive locking torque
  local viscousTorque = device.viscousTorque1Smoother:get(clamp(device.viscousCoef * outputAVdiff, -device.viscousTorque, device.viscousTorque))
  device.inputTorque = inputTorque
  device.outputTorque1 = inputTorque * device.diffTorqueSplitA - viscousTorque
  device.outputTorque2 = inputTorque * device.diffTorqueSplitB + viscousTorque
end

local function lockedUpdateTorque(device, dt)
  local inputAV = device.inputAV
  local outputAVdiff = device.outputAV1 - device.outputAV2
  local absOutputAVdiff = abs(outputAVdiff)

  local friction = (device.friction * clamp(inputAV, -1, 1) + device.dynamicFriction * inputAV + device.torqueLossCoef * device.parent[device.parentOutputTorqueName]) * device.wearFrictionCoef * device.damageFrictionCoef
  local inputTorque = (device.parent[device.parentOutputTorqueName] - friction) * (1 - min(device.speedLimitCoef * absOutputAVdiff * absOutputAVdiff * absOutputAVdiff, 1)) * device.gearRatio

  --integrate a position difference for the locking spring to act on, but constrain it to deform if too much torque
  device.diffAngle = clamp(device.diffAngle + outputAVdiff * dt, -device.maxDiffAngle, device.maxDiffAngle)
  local lockTorque = clamp(device.diffAngle * device.diffAngle * device.lockSpring * sign(device.diffAngle) + device.lockDamp * outputAVdiff, -device.lockTorque, device.lockTorque)
  device.inputTorque = inputTorque
  device.outputTorque1 = inputTorque * 0.5 - lockTorque
  device.outputTorque2 = inputTorque * 0.5 + lockTorque
end

local function activeLockUpdateTorque(device, dt)
  local inputAV = device.inputAV
  --divide out the gear ratio to get the carrier AV
  local outputAV1diff = device.outputAV1 - inputAV * device.invGearRatio
  local outputAV2diff = device.outputAV2 - inputAV * device.invGearRatio
  local outputAVdiff = device.outputAV1 - device.outputAV2

  local absMaxOutputAVdiff = max(abs(outputAV1diff), abs(outputAV2diff))
  local friction = (device.friction * clamp(inputAV, -1, 1) + device.dynamicFriction * inputAV + device.torqueLossCoef * device.parent[device.parentOutputTorqueName]) * device.wearFrictionCoef * device.damageFrictionCoef
  local inputTorque = (device.parent[device.parentOutputTorqueName] - friction) * (1 - min(device.speedLimitCoef * absMaxOutputAVdiff * absMaxOutputAVdiff * absMaxOutputAVdiff, 1)) * device.gearRatio

  --integrate a position difference for the locking spring to act on, but constrain it to deform if too much torque
  local maxClutchLockTorque = device.activeLockTorque * device.activeLockCoef
  device.maxDiffAngle = sqrt(maxClutchLockTorque / device.lockSpring)
  device.diffAngle = clamp(device.diffAngle + outputAVdiff * dt, -device.maxDiffAngle, device.maxDiffAngle)

  local lockTorque = clamp(device.diffAngle * device.diffAngle * device.lockSpring * sign(device.diffAngle) + device.lockDamp * outputAVdiff, -maxClutchLockTorque, maxClutchLockTorque)
  device.inputTorque = inputTorque
  device.outputTorque1 = inputTorque * device.diffTorqueSplitA - lockTorque
  device.outputTorque2 = inputTorque * device.diffTorqueSplitB + lockTorque
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  if device.mode == "open" or device.mode == "torqueVectoring" then
    device.torqueUpdate = openUpdateTorque
  elseif device.mode == "lsd" then
    device.torqueUpdate = LSDUpdateTorque
  elseif device.mode == "viscous" then
    device.torqueUpdate = viscousLSDUpdateTorque
  elseif device.mode == "locked" or device.mode == "dually" then --duallies use locked diffs as well, but we need a specific type to be able to detect these from code
    device.torqueUpdate = lockedUpdateTorque
  elseif device.mode == "activeLock" then
    device.torqueUpdate = activeLockUpdateTorque
  else
    log("E", "differential.selectDeviceUpdates", "Found unknown differential type: '" .. device.mode .. "'")
  end
end

local function setPartCondition(device, subSystem, odometer, integrity, visual)
  --TODO: -lsdLockCoef
  --      -viscousCoef
  --      -viscousTorque

  device.wearFrictionCoef = linearScale(odometer, 30000000, 1000000000, 1, 2)
  local integrityState = integrity
  if type(integrity) == "number" then
    local integrityValue = integrity
    integrityState = {
      damageFrictionCoef = linearScale(integrityValue, 1, 0, 1, 10),
      isBroken = false
    }
  end

  device.damageFrictionCoef = integrityState.damageFrictionCoef or 1

  if integrityState.isBroken then
    device:onBreak()
  end
end

local function getPartCondition(device)
  local integrityState = {
    damageFrictionCoef = device.damageFrictionCoef,
    isBroken = device.isBroken
  }
  local integrityValue = linearScale(device.damageFrictionCoef, 1, 10, 1, 0)
  if device.isBroken then
    integrityValue = 0
  end
  return integrityValue, integrityState
end

local function updateSimpleControlButtons(device)
  if #device.availableModes > 1 and device.uiSimpleModeControl then
    local modeIconLookup = {
      open = "powertrain_differential_open",
      torqueVectoring = "powertrain_differential_open",
      lsd = "powertrain_differential_lsd",
      viscous = "powertrain_differential_lsd",
      locked = "powertrain_differential_closed",
      dually = "powertrain_differential_open",
      activeLock = "powertrain_differential_lsd"
    }
    extensions.ui_simplePowertrainControl.setButton("powertrain_device_mode_shortcut_" .. device.name, device.uiName, modeIconLookup[device.mode], nil, nil, string.format("powertrain.toggleDeviceMode(%q)", device.name))
  end
end

local function setMode(device, mode)
  local isValidMode = false
  for _, availableMode in ipairs(device.availableModes) do
    if mode == availableMode then
      isValidMode = true
    end
  end
  if not isValidMode then
    return
  end
  device.mode = mode
  selectUpdates(device)
  device:updateSimpleControlButtons()
end

local function calculateInertia(device)
  local outputInertia = 0
  local cumulativeGearRatio = 1
  local maxCumulativeGearRatio = 1
  if device.children then
    local grA = 0
    local grB = 0
    local maxGRA = 0
    local maxGRB = 0
    if device.children[1] then
      outputInertia = outputInertia + device.children[1].cumulativeInertia * device.diffTorqueSplitB
      grB = device.children[1].cumulativeGearRatio
      maxGRB = device.children[1].maxCumulativeGearRatio
    end
    if device.children[2] then
      outputInertia = outputInertia + device.children[2].cumulativeInertia * device.diffTorqueSplitA
      grA = device.children[2].cumulativeGearRatio
      maxGRA = device.children[2].maxCumulativeGearRatio
    end

    if grA ~= grB or maxGRA ~= maxGRB then
      --guihooks.message("Caution: Mismatched final drive ratios!  ".. grA.. "  vs  ".. grB, 5)
      log("D", "differential.calculateInertia", string.format("%s: Found non-matching gear ratios for differential outputs: A: '%.4f', B: '%.4f', A(max): '%.4f', B(max): '%.4f'", device.name, grA, grB, maxGRA, maxGRB))
    else
      cumulativeGearRatio = grA
      maxCumulativeGearRatio = maxGRA
    end
    outputInertia = outputInertia * 2
  end

  if device.lockSpringAutoCalc then
    device.lockSpring = powertrain.stabilityCoef * powertrain.stabilityCoef * min(device.children[1].cumulativeInertia, device.children[2].cumulativeInertia)
    device.lockTorque = device.lockSpring
  end

  device.lockDamp = device.lockDampRatio * sqrt(device.lockSpring * min(device.children[1].cumulativeInertia, device.children[2].cumulativeInertia))
  device.maxDiffAngle = sqrt(device.lockTorque / device.lockSpring)

  device.cumulativeInertia = outputInertia / device.gearRatio / device.gearRatio
  device.cumulativeGearRatio = cumulativeGearRatio * device.gearRatio
  device.maxCumulativeGearRatio = maxCumulativeGearRatio * device.gearRatio
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

  device.invGearRatio = 1 / device.gearRatio

  --viscous specific
  device.viscousTorque1Smoother:reset()
  device.viscousTorque2Smoother:reset()

  --locked specific
  device.diffAngle = 0

  --active locking specific
  device.activeLockCoef = 0

  device.wearFrictionCoef = 1
  device.damageFrictionCoef = 1

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
    defaultVirtualInertia = jbeamData.defaultVirtualInertia or nil, --meant to be nil if not specified manually
    speedLimitCoef = (jbeamData.speedLimitCoef or 1) * 0.0000002,
    outputAV1 = 0,
    outputAV2 = 0,
    inputAV = 0,
    outputTorque1 = 0,
    outputTorque2 = 0,
    reset = reset,
    setMode = setMode,
    calculateInertia = calculateInertia,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition,
    updateSimpleControlButtons = updateSimpleControlButtons
  }

  device.torqueLossCoef = clamp(device.torqueLossCoef, 0, 1)

  local diffTorqueSplit = jbeamData.diffTorqueSplit or 0.5
  device.diffTorqueSplitA = diffTorqueSplit
  device.diffTorqueSplitB = 1 - device.diffTorqueSplitA

  if type(jbeamData.diffType) == "table" then
    device.availableModes = shallowcopy(jbeamData.diffType)
    device.mode = jbeamData.diffType[1] or "open"
    device.defaultToggle = jbeamData.defaultToggle == nil and true or jbeamData.defaultToggle
  else
    device.mode = jbeamData.diffType or "open"
    device.availableModes = {device.mode}
  end

  device.visualType = "differential_" .. device.mode

  device.invGearRatio = 1 / device.gearRatio

  --lsd specific
  device.lsdPreload = jbeamData.lsdPreload or 50
  device.lsdLockCoef = jbeamData.lsdLockCoef or 0.2
  device.lsdRevLockCoef = jbeamData.lsdRevLockCoef or device.lsdLockCoef

  --viscous specific
  device.viscousCoef = jbeamData.viscousCoef or 5
  device.viscousTorque = jbeamData.viscousTorque or device.viscousCoef * 10
  device.viscousExponent = jbeamData.viscousExponent or 1
  device.viscousTorque1Smoother = newExponentialSmoothing(jbeamData.viscousSmoothing or 25)
  device.viscousTorque2Smoother = newExponentialSmoothing(jbeamData.viscousSmoothing or 25)

  --locked specific
  device.diffAngle = 0
  device.lockTorque = jbeamData.lockTorque or 500
  device.lockSpring = jbeamData.lockSpring or device.lockTorque
  device.lockDampRatio = jbeamData.lockDampRatio or 0.1 --1 is critically damped

  --active locking (affects activeLock and lsd with appropriate controller)
  device.activeLockCoef = 0
  device.activeLockTorque = jbeamData.activeLockTorque or device.lockTorque
  device.lockSpringAutoCalc = jbeamData.lockSpring == nil and jbeamData.lockTorque == nil

  selectUpdates(device)

  return device
end

M.new = new

return M
