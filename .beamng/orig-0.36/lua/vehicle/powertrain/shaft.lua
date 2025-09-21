-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {[1] = true}
M.deviceCategories = {shaft = true}

local abs = math.abs

local function updateVelocity(device, dt)
  device.inputAV = device[device.outputAVName] * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function updateTorque(device)
  device[device.outputTorqueName] = (device.parent[device.parentOutputTorqueName] - (device.friction * clamp(device.inputAV, -1, 1) + device.dynamicFriction * device.inputAV) * device.wearFrictionCoef * device.damageFrictionCoef) * device.gearRatio
end

local function disconnectedUpdateVelocity(device, dt)
  --use the speed of the virtual mass, not the drivetrain on other side if the shaft is broken or disconnected, otherwise, pass through
  device.inputAV = device.virtualMassAV * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function disconnectedUpdateTorque(device, dt)
  local outputTorque = (device.parent[device.parentOutputTorqueName] - (device.friction * clamp(device.inputAV, -1, 1) + device.dynamicFriction * device.inputAV) * device.wearFrictionCoef * device.damageFrictionCoef) * device.gearRatio
  --accelerate a virtual mass with the output torque if the shaft is disconnected or broken
  device.virtualMassAV = device.virtualMassAV + outputTorque * device.invCumulativeInertia * dt
  device[device.outputTorqueName] = 0 --set to 0 to stop children receiving torque
end

local function wheelShaftUpdateVelocity(device, dt)
  device[device.outputAVName] = device.wheel.angularVelocity * device.wheelDirection
  device.inputAV = device[device.outputAVName] * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function wheelShaftUpdateTorque(device)
  local outputTorque = device.parent[device.parentOutputTorqueName] * device.gearRatio
  local wheel = device.wheel
  wheel.propulsionTorque = outputTorque * device.wheelDirection
  wheel.frictionTorque = (device.friction + device.dynamicFriction * device.inputAV) * device.wearFrictionCoef * device.damageFrictionCoef
  device[device.outputTorqueName] = outputTorque
  local trIdx = wheel.torsionReactorIdx
  powertrain.torqueReactionCoefs[trIdx] = powertrain.torqueReactionCoefs[trIdx] + abs(outputTorque)
end

local function wheelShaftDisconnectedUpdateVelocity(device, dt)
  device[device.outputAVName] = device.wheel.angularVelocity * device.wheelDirection
  device.inputAV = device.virtualMassAV * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function wheelShaftDisconnectedUpdateTorque(device, dt)
  local outputTorque = (device.parent[device.parentOutputTorqueName] - (device.friction * clamp(device.inputAV, -1, 1) + device.dynamicFriction * device.inputAV) * device.wearFrictionCoef * device.damageFrictionCoef) * device.gearRatio
  --accelerate a virtual mass with the output torque if the shaft is disconnected or broken
  device.virtualMassAV = device.virtualMassAV + outputTorque * device.invCumulativeInertia * dt
  device[device.outputTorqueName] = 0 --set to 0 to stop children receiving torque
  device.wheel.propulsionTorque = 0
  device.wheel.frictionTorque = device.friction
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  device.torqueUpdate = updateTorque

  if device.connectedWheel then
    device.velocityUpdate = wheelShaftUpdateVelocity
    device.torqueUpdate = wheelShaftUpdateTorque
  end

  if device.isBroken or device.mode == "disconnected" then
    device.velocityUpdate = disconnectedUpdateVelocity
    device.torqueUpdate = disconnectedUpdateTorque
    if device.connectedWheel then
      device.velocityUpdate = wheelShaftDisconnectedUpdateVelocity
      device.torqueUpdate = wheelShaftDisconnectedUpdateTorque
    end
    --make sure the virtual mass has the right AV
    device.virtualMassAV = device.inputAV
  end
end

local function applyDeformGroupDamage(device, damageAmount)
  device.damageFrictionCoef = device.damageFrictionCoef + linearScale(damageAmount, 0, 0.01, 0, 0.1)
end

local function setPartCondition(device, subSystem, odometer, integrity, visual)
  device.wearFrictionCoef = linearScale(odometer, 30000000, 1000000000, 1, 1.5)
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

local function validate(device)
  if device.isPhysicallyDisconnected then
    device.mode = "disconnected"
    selectUpdates(device)
  end

  if (not device.connectedWheel) and (not device.children or #device.children <= 0) then
    --print(device.name)
    local parentDiff = device.parent
    while parentDiff.parent and not parentDiff.deviceCategories.differential do
      parentDiff = parentDiff.parent
      --print(parentDiff and parentDiff.name or "nil")
    end

    if parentDiff and parentDiff.deviceCategories.differential and parentDiff.defaultVirtualInertia then
      --print("Found parent diff, using its default virtual inertia: "..parentDiff.defaultVirtualInertia)
      device.virtualInertia = parentDiff.defaultVirtualInertia
    end
  end

  if device.connectedWheel and device.parent then
    --print(device.connectedWheel)
    --print(device.name)
    local torsionReactor = device.parent
    while torsionReactor.parent and torsionReactor.type ~= "torsionReactor" do
      torsionReactor = torsionReactor.parent
      --print(torsionReactor and torsionReactor.name or "nil")
    end

    if torsionReactor and torsionReactor.type == "torsionReactor" and torsionReactor.torqueReactionNodes then
      local wheel = powertrain.wheels[device.connectedWheel]
      local reactionNodes = torsionReactor.torqueReactionNodes
      wheel.obj:setEngineAxisCoupleNodes(reactionNodes[1], reactionNodes[2], reactionNodes[3])
      device.torsionReactor = torsionReactor
      wheel.torsionReactor = torsionReactor
    end
  end

  return true
end

local function updateSimpleControlButtons(device)
  if #device.availableModes > 1 and device.uiSimpleModeControl then
    local modeIconLookup
    if device.connectedWheel then
      modeIconLookup = {connected = "powertrain_wheel_connected", disconnected = "powertrain_wheel_disconnected"}
    else
      modeIconLookup = {connected = "powertrain_shaft_connected", disconnected = "powertrain_shaft_disconnected"}
    end
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
  --prevent mode changes to physically disconnected devices, they lock up if in any other mode than "disconnected"
  if device.isPhysicallyDisconnected then
    device.mode = "disconnected"
  end
  selectUpdates(device)
  device:updateSimpleControlButtons()
end

local function onBreak(device)
  device.isBroken = true
  --obj:breakBeam(device.breakTriggerBeam)
  selectUpdates(device)
end

local function calculateInertia(device)
  local outputInertia
  local cumulativeGearRatio = 1
  local maxCumulativeGearRatio = 1
  if device.children and #device.children > 0 then
    local child = device.children[1]
    outputInertia = child.cumulativeInertia
    cumulativeGearRatio = child.cumulativeGearRatio
    maxCumulativeGearRatio = child.maxCumulativeGearRatio
  elseif device.connectedWheel then
    local axisInertia = 0
    local wheel = powertrain.wheels[device.connectedWheel]
    local hubNode1 = vec3(v.data.nodes[wheel.node1].pos)
    local hubNode2 = vec3(v.data.nodes[wheel.node2].pos)

    for _, nid in pairs(wheel.nodes) do
      local n = v.data.nodes[nid]
      local distanceToAxis = vec3(n.pos):distanceToLine(hubNode1, hubNode2)
      axisInertia = axisInertia + (n.nodeWeight * (distanceToAxis * distanceToAxis))
    end

    --print(device.connectedWheel .. " Hub-Axis Inertia: " .. axisInertia .. " kgm^2")
    outputInertia = axisInertia
  else
    --Nothing connected to this shaft :(
    outputInertia = device.virtualInertia --some default inertia
  end

  device.cumulativeInertia = outputInertia / device.gearRatio / device.gearRatio
  device.invCumulativeInertia = device.cumulativeInertia > 0 and 1 / device.cumulativeInertia or 0
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
  device.visualShaftAngle = 0
  device.virtualMassAV = 0

  device.isBroken = false
  device.wearFrictionCoef = 1
  device.damageFrictionCoef = 1

  device[device.outputTorqueName] = 0
  device[device.outputAVName] = 0

  selectUpdates(device)

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
    virtualInertia = 2,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    electricsName = jbeamData.electricsName,
    visualShaftAVName = jbeamData.visualShaftAVName,
    inputAV = 0,
    visualShaftAngle = 0,
    virtualMassAV = 0,
    isBroken = false,
    torsionReactor = nil,
    nodeCid = jbeamData.node,
    reset = reset,
    onBreak = onBreak,
    setMode = setMode,
    validate = validate,
    calculateInertia = calculateInertia,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition,
    updateSimpleControlButtons = updateSimpleControlButtons
  }

  if jbeamData.connectedWheel and powertrain.wheels[jbeamData.connectedWheel] then
    device.connectedWheel = jbeamData.connectedWheel
    device.wheel = powertrain.wheels[device.connectedWheel]
    device.wheelDirection = powertrain.wheels[device.connectedWheel].wheelDir

    device.cumulativeInertia = 1

    local pos = v.data.nodes[device.wheel.node1].pos
    device.visualPosition = pos
    device.visualType = "wheel"
  end

  local outputPortIndex = 1
  if jbeamData.outputPortOverride then
    device.outputPorts = {}
    for _, v in pairs(jbeamData.outputPortOverride) do
      device.outputPorts[v] = true
      outputPortIndex = v
    end
  end

  device.outputTorqueName = "outputTorque" .. tostring(outputPortIndex)
  device.outputAVName = "outputAV" .. tostring(outputPortIndex)
  device[device.outputTorqueName] = 0
  device[device.outputAVName] = 0

  if jbeamData.canDisconnect then
    device.availableModes = {"connected", "disconnected"}
    device.mode = jbeamData.isDisconnected and "disconnected" or "connected"
  else
    device.availableModes = {"connected"}
    device.mode = "connected"
  end

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
