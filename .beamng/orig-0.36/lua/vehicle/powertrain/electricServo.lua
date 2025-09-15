-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.outputPorts = {}
M.deviceCategories = {engine = true}

local min = math.min
local abs = math.abs

local rpmToAV = 0.104719755
local pi = math.pi
local twoPi = pi * 2

--velocity update is always nopped for engines

local function servoUpdateTorque(device, dt)
  local wheelAV = device.wheel.angularVelocity
  local angle = device.currentAngle + wheelAV * dt
  device.currentAngle = device.rotationMode == "absolute" and angle or (abs(angle) % twoPi) * sign(angle)

  local angleError = device.targetAngle - device.currentAngle
  local throttle = min(abs(angleError) * device.angularSpring, 1)
  device.torqueDirection = sign(angleError) * device.wheelDirection

  local wheelAVSign = sign(wheelAV)
  local backdrivenCoef = (device.torqueDirection * wheelAVSign < 0) and 1 or 1 - min(abs(wheelAV) / device.maxAV, 1)
  local torque = device.maxTorque * device.torqueDirection * backdrivenCoef * throttle
  device.outputTorque1 = torque - device.friction * wheelAVSign - device.dynamicFriction * wheelAV

  device.wheel.propulsionTorque = device.outputTorque1 * device.wheelDirection
end

local function servoDisconnectedUpdateVelocity(device)
end

local function servoDisconnectedUpdateTorque(device, dt)
  device.outputTorque1 = 0
  device.wheel.propulsionTorque = 0
end

--do not try to catch specific events, just always sanitize the input and current angle
--always convert to 0-360, no matter what comes in and fix the currentAngle to 0-360

local function setTargetAngle(device, angle)
  if device.rotationMode == "absolute" then
    device.targetAngle = angle
  elseif device.rotationMode == "shortest" then
    device.targetAngle = (abs(angle) % twoPi) * sign(angle)
    if device.minAngle and device.maxAngle then
      device.targetAngle = clamp(device.targetAngle, device.minAngle, device.maxAngle)
    end

    local angleDiff = abs(device.currentAngle - device.targetAngle)
    if angleDiff > pi then
      device.currentAngle = device.currentAngle - twoPi * sign(device.currentAngle)
    end
  end
  --print(string.format("Wanted: %.2f, used %.2f instead, current angle: %.2f", math.deg(angle), math.deg(device.targetAngle), math.deg(device.currentAngle)))
end

local function setTargetAngleRelative(device, angle)
  device:setTargetAngle(device.targetAngle + angle)
end

local function setMinMaxAngles(device, minAngle, maxAngle)
  device.minAngle = minAngle
  device.maxAngle = maxAngle
end

local function updateGFX(device, dt)
  --note: this method is only executed if any electrics name for the target angle is set
  --if logic other than for the above needs to be in here, updateGFX needs to become a regular update method

  local angle = electrics.values[device.electricsTargetAngleName] or 0
  device:setTargetAngle(angle)
end

local function selectUpdates(device)
  device.velocityUpdate = nop
  device.torqueUpdate = servoUpdateTorque

  if device.isBroken or device.mode == "disconnected" then
    device.velocityUpdate = servoDisconnectedUpdateVelocity
    device.torqueUpdate = servoDisconnectedUpdateTorque
    --make sure the virtual mass has the right AV
    device.virtualMassAV = device.inputAV
  end

  --we only need GFX update if we want to set the position via an electrics value, otherwise leave it at nil
  if device.electricsTargetAngleName then
    device.updateGFX = updateGFX
  end
end

local function validate(device)
  return true
end

local function setMode(device, mode)
  device.mode = mode
  selectUpdates(device)
end

local function onBreak(device)
  device.isBroken = true
  device.outputTorque1 = 0
  device.wheel.propulsionTorque = 0
  --  if device.connectedRotator then
  --    device.wheelObj.torque = 0
  --  end

  selectUpdates(device)
end

local function calculateInertia(device)
  local outputInertia = 0
  if device.connectedRotator then
    local axisInertia = 0
    local wheel = powertrain.wheels[device.connectedRotator]
    local hubNode1 = vec3(v.data.nodes[wheel.node1].pos)
    local hubNode2 = vec3(v.data.nodes[wheel.node2].pos)

    for _, nid in pairs(wheel.nodes) do
      local n = v.data.nodes[nid]
      local distanceToAxis = vec3(n.pos):distanceToLine(hubNode1, hubNode2)
      axisInertia = axisInertia + (n.nodeWeight * (distanceToAxis * distanceToAxis))
    end

    --print(device.connectedWheel.." Hub-Axis: "..axisInertia.." kgm²")
    outputInertia = axisInertia
  end
  device.cumulativeInertia = outputInertia / device.gearRatio / device.gearRatio
  device.invCumulativeInertia = device.cumulativeInertia > 0 and 1 / device.cumulativeInertia or 0
  device.maxCumulativeGearRatio = device.gearRatio
end

local function reset(device, jbeamData)
  device.outputTorque1 = 0
  device.outputAV1 = 0
  device.targetAngle = 0
  device.currentAngle = 0
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
    angularSpring = jbeamData.angularSpring or 1,
    cumulativeInertia = 1,
    invCumulativeInertia = 1,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    inputAV = 0,
    virtualMassAV = 0,
    isBroken = false,
    rotationMode = jbeamData.rotationMode or "absolute", --also: "shortest"
    electricsTargetAngleName = jbeamData.electricsTargetAngleName, --no default here, if no value is specified, servo is controlled via dedicated API
    minAngle = nil,
    maxAngle = nil,
    reset = reset,
    onBreak = onBreak,
    setMode = setMode,
    validate = validate,
    calculateInertia = calculateInertia,
    targetAngle = 0,
    setTargetAngle = setTargetAngle,
    setTargetAngleRelative = setTargetAngleRelative,
    setMinMaxAngles = setMinMaxAngles,
    currentAngle = 0,
    torqueDirection = 1
  }

  device.maxTorque = jbeamData.stallTorque or 1000
  device.maxRPM = jbeamData.maxRPM
  device.maxAV = device.maxRPM * rpmToAV

  if jbeamData.connectedRotator and powertrain.wheels[jbeamData.connectedRotator] then
    device.connectedRotator = jbeamData.connectedRotator
    device.wheel = powertrain.wheels[device.connectedRotator]
    device.wheelObj = powertrain.wheels[device.connectedRotator].obj
    device.wheelDirection = powertrain.wheels[device.connectedRotator].wheelDir
    wheels.setWheelRotatorType(device.wheel.wheelID, "rotator")

    device.cumulativeInertia = 1

    local pos = v.data.nodes[device.wheel.node1].pos
    device.visualPosition = pos
    device.visualType = "servo"
  else
    --can't find connected rotator
  end

  device.outputTorque1 = 0
  device.outputAV1 = 0

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
