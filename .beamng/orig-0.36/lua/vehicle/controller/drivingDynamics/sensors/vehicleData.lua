-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 54

local abs = math.abs
local min = math.min
local acos = math.acos
local sqrt = math.sqrt

M.isActive = false

M.frontLeftWheelAngle = 0
M.frontRightWheelAngle = 0
M.frontWheelAngle = 0

M.rearLeftWheelAngle = 0
M.rearRightWheelAngle = 0
M.rearWheelAngle = 0

M.turningCircleSpeedRatios = {}

M.vehicleStats = {
  wheelBase = 0,
  invWheelBase = 0,
  distanceCOGFrontAxle = 0,
  distanceCOGRearAxle = 0,
  trackWidth = 0,
  trackWidthRefLeft = 0,
  trackWidthRefRight = 0,
  mass = 0,
  characteristicSpeed = 0,
  invSquaredCharacteristicSpeed = 0,
  skewStiffnessFront = 0,
  skewStiffnessRear = 0,
  inertiaZ = 0
}

M.wheelAccess = {
  frontRight = nil,
  frontLeft = nil,
  rearRight = nil,
  rearLeft = nil
}

local smoothers = {
  turningCircleAcc = newTemporalSmoothing(100, 100)
}

local turningCircle = {
  centerSTM = 0,
  centerAcc = 0,
  centerFinal = 0,
  innerFinal = 0,
  outerFinal = 0
}

local leftWheels = {}
local rightWheels = {}

local wheelCount = 0

local CMU = nil
local isDebugEnabled = false

local debugPacket = {sourceType = "vehicleData"}

local function getTurningCircleSTM()
  local wheelAccess = M.wheelAccess
  local leftDist = obj:wheelTurnRadius(wheelAccess.frontLeft.wheelID, wheelAccess.rearLeft.wheelID)
  local rightDist = obj:wheelTurnRadius(wheelAccess.frontRight.wheelID, wheelAccess.rearRight.wheelID)

  return (leftDist + rightDist) * 0.5 --turningRadiusSTM
end

local function getTurningCircleAcc(speed, dt)
  if speed < 0.1 then
    return 0
  end
  local refNodeAcc = smoothers.turningCircleAcc:getUncapped(abs(CMU.sensorHub.accelerationXSmooth), dt)
  local turningCircleAcc = (speed * speed) / refNodeAcc
  return turningCircleAcc
end

local function guardInfNaN(number)
  return isnaninf(number) and 0 or number
end

local function update(dt)
  local wheelAccess = M.wheelAccess
  local virtualSensors = CMU.virtualSensors
  local sensorHub = CMU.sensorHub

  local frontLeftWheel = wheelAccess.frontLeft
  local frontRightWheel = wheelAccess.frontRight
  local rearLeftWheel = wheelAccess.rearLeft
  local rearRightWheel = wheelAccess.rearRight

  local steeringInput = sensorHub.steeringInput
  local steeringSign = sign(steeringInput)

  M.frontLeftWheelAngle = acos(obj:nodeVecPlanarCosRightForward(frontLeftWheel.node2, frontLeftWheel.node1)) * steeringSign
  M.frontRightWheelAngle = acos(obj:nodeVecPlanarCosRightForward(frontRightWheel.node1, frontRightWheel.node2)) * steeringSign
  M.rearLeftWheelAngle = acos(obj:nodeVecPlanarCosRightForward(rearLeftWheel.node2, rearLeftWheel.node1))
  M.rearRightWheelAngle = acos(obj:nodeVecPlanarCosRightForward(rearRightWheel.node1, rearRightWheel.node2))

  M.frontWheelAngle = (M.frontLeftWheelAngle + M.frontRightWheelAngle) * 0.5
  M.rearWheelAngle = (M.rearLeftWheelAngle + M.rearRightWheelAngle) * 0.5

  local virtualSpeed = virtualSensors.virtual.speed
  local wheelSpeed = virtualSensors.virtual.wheelSpeed
  local speed = virtualSensors.trustWorthiness.virtualSpeed >= 0.5 and virtualSpeed or wheelSpeed

  turningCircle.centerAcc = getTurningCircleAcc(speed, dt)
  turningCircle.centerSTM = min(getTurningCircleSTM(), 1e10)

  turningCircle.centerFinal = linearScale(speed, 3, 5, turningCircle.centerSTM, turningCircle.centerAcc)

  local offsetInner = steeringInput < 0 and M.vehicleStats.trackWidthRefRight or M.vehicleStats.trackWidthRefLeft
  local offsetOuter = steeringInput < 0 and M.vehicleStats.trackWidthRefLeft or M.vehicleStats.trackWidthRefRight
  turningCircle.innerFinal = turningCircle.centerFinal - offsetInner
  turningCircle.outerFinal = turningCircle.centerFinal + offsetOuter
  local turningCircleRatioOuter = guardInfNaN(turningCircle.outerFinal / turningCircle.innerFinal)

  local leftRatio = M.frontWheelAngle > 0 and 1 or turningCircleRatioOuter
  local rightRatio = M.frontWheelAngle < 0 and 1 or turningCircleRatioOuter
  for i = 0, wheelCount - 1 do
    local wheelName = wheels.wheels[i].name
    M.turningCircleSpeedRatios[wheelName] = leftWheels[wheelName] and leftRatio or rightRatio
  end
end

local function updateDebug(dt)
  update(dt)
end

local function updateGFX(dt)
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  debugPacket.frontLeftWheelAngle = M.frontLeftWheelAngle
  debugPacket.frontRightWheelAngle = M.frontRightWheelAngle
  debugPacket.rearLeftWheelAngle = M.rearLeftWheelAngle
  debugPacket.rearRightWheelAngle = M.rearRightWheelAngle

  debugPacket.frontWheelAngle = M.frontWheelAngle
  debugPacket.rearWheelAngle = M.rearWheelAngle

  debugPacket.turningCircleSTM = guardInfNaN(turningCircle.centerSTM)
  debugPacket.turningCircleAcc = guardInfNaN(turningCircle.centerAcc)
  debugPacket.turningCircleFinal = guardInfNaN(turningCircle.centerFinal)
  debugPacket.turningCircleInner = guardInfNaN(turningCircle.innerFinal)
  debugPacket.turningCircleOuter = guardInfNaN(turningCircle.outerFinal)

  debugPacket.turningCircleRatios = M.turningCircleSpeedRatios

  CMU.sendDebugPacket(debugPacket)
end

local function calculateCharacteristicSpeed()
  local eg = (M.vehicleStats.mass * (M.vehicleStats.skewStiffnessRear * M.vehicleStats.distanceCOGRearAxle - M.vehicleStats.skewStiffnessFront * M.vehicleStats.distanceCOGFrontAxle)) / (M.vehicleStats.skewStiffnessFront * M.vehicleStats.skewStiffnessRear * M.vehicleStats.wheelBase)
  M.vehicleStats.characteristicSpeed = sqrt(M.vehicleStats.wheelBase / abs(eg + 1e-30)) --guard against infinity
  if isDebugEnabled then
    log("D", "vehicleData.calculateCharacteristicSpeed", string.format("Calculated EG: %.6f", eg))
    log("D", "vehicleData.calculateCharacteristicSpeed", string.format("Calculated characteristic speed: %.2f m/s", M.vehicleStats.characteristicSpeed))
    if eg < -0.01 then --don't check for exactly 0, 0 is fine, so check for something slightly smaller
      log("W", "vehicleData.calculateCharacteristicSpeed", string.format("Calculated EG (%.6f) is lower than 0 (oversteery car setup)!", eg))
    end
  end
end

local function calculateAxleDistances()
  M.vehicleStats.wheelBase = obj:nodeLength(M.wheelAccess.frontRight.node1, M.wheelAccess.rearRight.node1) --calculate wheelbase from the distance of the front and rear wheels
  M.vehicleStats.invWheelBase = 1 / M.vehicleStats.wheelBase
  M.vehicleStats.cogWithoutWheels = obj:calcCenterOfGravityRel(true)

  local frontAxlePos = 0
  local rearAxlePos = 0
  local twLeft = 0
  local twRight = 0

  local totalMass = 0
  for _, n in pairs(v.data.nodes) do
    totalMass = totalMass + n.nodeWeight

    --Find the positions of the front and rear axle
    if n.cid == M.wheelAccess.frontRight.node1 then
      frontAxlePos = n.pos.y
      twRight = n.pos.x
    elseif n.cid == M.wheelAccess.rearRight.node1 then
      rearAxlePos = n.pos.y
    elseif n.cid == M.wheelAccess.frontLeft.node1 then
      twLeft = n.pos.x
    end
  end
  M.vehicleStats.mass = totalMass

  M.vehicleStats.distanceCOGFrontAxle = abs(M.vehicleStats.cogWithoutWheels.y - frontAxlePos)
  M.vehicleStats.distanceCOGRearAxle = abs(M.vehicleStats.cogWithoutWheels.y - rearAxlePos)

  M.vehicleStats.trackWidth = abs(twLeft - twRight)
  local refNodeX = v.data.nodes[v.data.refNodes[0].ref].pos.x
  M.vehicleStats.trackWidthRefLeft = abs(twLeft - refNodeX)
  M.vehicleStats.trackWidthRefRight = abs(twRight - refNodeX)

  if isDebugEnabled then
    log("D", "vehicleData.calculateAxleDistances", string.format("Distance COG to Rearaxle: %.3fm", M.vehicleStats.distanceCOGRearAxle))
    log("D", "vehicleData.calculateAxleDistances", string.format("Distance COG to Frontaxle: %.3fm", M.vehicleStats.distanceCOGFrontAxle))
    log("D", "vehicleData.calculateAxleDistances", string.format("Wheelbase: %.3fm", M.vehicleStats.wheelBase))
    log("D", "vehicleData.calculateAxleDistances", string.format("Track width: %.3fm", M.vehicleStats.trackWidth))
  end
end

local function calculateInertiaZ()
  local cogZAxis = vec3(M.vehicleStats.cogWithoutWheels.x, M.vehicleStats.cogWithoutWheels.y, 0)
  local inertiaZ = 0
  for _, v in pairs(v.data.nodes) do
    local posNoZ = vec3(v.pos.x, v.pos.y, 0)
    local distanceToCOG = (cogZAxis - posNoZ):length()
    local mass = v.nodeWeight
    inertiaZ = inertiaZ + mass * distanceToCOG * distanceToCOG
  end
  M.vehicleStats.inertiaZ = inertiaZ
end

local function reset()
  smoothers.turningCircleAcc:reset()
end

local function init(jbeamData)
end

local function initSecondStage(jbeamData)
  M.isActive = false

  M.vehicleStats.skewStiffnessFront = jbeamData.skewStiffnessFront or 0
  M.vehicleStats.skewStiffnessRear = jbeamData.skewStiffnessRear or 0

  local cornerWheelData = jbeamData.cornerWheels or {"FR", "FL", "RR", "RL"}

  if v.config.partConfigFilename then
    local _, configName = path.splitWithoutExt(v.config.partConfigFilename)
    local configFilePath = string.format("%sdrivingDynamics/%s.stat.json", vehiclePath, configName)
    local configContent = jsonReadFile(configFilePath)
    if configContent then
      --print("found model config data:")
      --print(configFilePath)
      --dump(configContent)
      if configContent.vehicleData then
        M.vehicleStats.skewStiffnessFront = configContent.vehicleData.skewStiffnessFront or 0
        M.vehicleStats.skewStiffnessRear = configContent.vehicleData.skewStiffnessRear or 0
        cornerWheelData = configContent.vehicleData.cornerWheelData or cornerWheelData
      end
    end
  end

  local cornerWheels = {}
  for _, wheelName in pairs(cornerWheelData) do
    cornerWheels[wheelName] = true
  end

  local avgWheelPos = vec3(0, 0, 0)

  --calculate avg wheel position for later being able to determine where a given wheel is
  for _, wheel in pairs(wheels.wheels) do
    if cornerWheels[wheel.name] then
      local wheelNodePos = v.data.nodes[wheel.node1].pos
      avgWheelPos = avgWheelPos + wheelNodePos
    end
  end

  avgWheelPos = avgWheelPos / #wheels.wheels --make the average of all positions

  local refNodes = v.data.refNodes[0]
  local vectorForward = vec3(v.data.nodes[refNodes.ref].pos) - vec3(v.data.nodes[refNodes.back].pos)
  local vectorUp = vec3(v.data.nodes[refNodes.up].pos) - vec3(v.data.nodes[refNodes.ref].pos)
  local vectorRight = vectorForward:cross(vectorUp)

  local foundWheelsCount = 0

  for _, wheel in pairs(wheels.wheels) do
    if cornerWheels[wheel.name] then
      local wheelNodePos = vec3(v.data.nodes[wheel.node1].pos)
      local wheelVector = wheelNodePos - avgWheelPos
      local dotForward = vectorForward:dot(wheelVector)
      local dotLeft = vectorRight:dot(wheelVector)
      wheelCount = wheelCount + 1

      if dotLeft >= 0 then
        if dotForward >= 0 then
          M.wheelAccess.frontRight = wheel
          foundWheelsCount = foundWheelsCount + 1
        else
          M.wheelAccess.rearRight = wheel
          foundWheelsCount = foundWheelsCount + 1
        end
        rightWheels[wheel.name] = true
      else
        if dotForward >= 0 then
          M.wheelAccess.frontLeft = wheel
          foundWheelsCount = foundWheelsCount + 1
        else
          M.wheelAccess.rearLeft = wheel
          foundWheelsCount = foundWheelsCount + 1
        end
        leftWheels[wheel.name] = true
      end
      M.turningCircleSpeedRatios[wheel.name] = 1
    end
  end

  if foundWheelsCount ~= 4 then
    log("E", "vehicleData.init", "Can't find correct wheels, aborting...")
    return
  end

  calculateAxleDistances()
  calculateCharacteristicSpeed()
  calculateInertiaZ()

  M.isActive = true
end

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled

  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
  M.update = isDebugEnabled and update or updateDebug
end

local function registerCMU(cmu)
  CMU = cmu
end

local function shutdown()
  M.isActive = false
  M.updateGFX = nil
  M.update = nil
end

M.init = init
M.initSecondStage = initSecondStage

M.reset = reset

M.updateGFX = updateGFX
M.update = update

M.registerCMU = registerCMU
M.setDebugMode = setDebugMode
M.shutdown = shutdown

return M
