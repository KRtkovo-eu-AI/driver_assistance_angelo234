-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 52

local max = math.max
local min = math.min
local abs = math.abs

M.isActive = false

M.steeringInput = 0

M.roll = 0
M.pitch = 0

M.rollAV = 0
M.pitchAV = 0
M.yawAV = 0

M.rollAVSmooth = 0
M.pitchAVSmooth = 0
M.yawAVSmooth = 0

M.vX = 0
M.vY = 0

M.yawAcceleration = 0

M.gravity = 0

M.accelerationX = 0
M.accelerationY = 0
M.accelerationZ = 0

M.accelerationXSmooth = 0
M.accelerationYSmooth = 0
M.accelerationZSmooth = 0

M.accNoiseX = 0
M.accNoiseY = 0
M.accNoiseZ = 0

local rollAVSmoother = newTemporalSmoothingNonLinear(20)
local pitchAVSmoother = newTemporalSmoothingNonLinear(20)
local yawAVSmoother = newTemporalSmoothingNonLinear(20)

local accelerationXSmoother = newTemporalSmoothingNonLinear(15)
local accelerationYSmoother = newTemporalSmoothingNonLinear(15)
local accelerationZSmoother = newTemporalSmoothingNonLinear(20)

local accXNoiseUpperSmoother = newTemporalSmoothing(5, 100000)
local accXNoiseLowerSmoother = newTemporalSmoothing(5, 100000)
local accYNoiseUpperSmoother = newTemporalSmoothing(5, 100000)
local accYNoiseLowerSmoother = newTemporalSmoothing(5, 100000)
local accZNoiseUpperSmoother = newTemporalSmoothing(5, 100000)
local accZNoiseLowerSmoother = newTemporalSmoothing(5, 100000)

local CMU = nil
local isDebugEnabled = false

local lastYawAV = 0

local debugPacket = {sourceType = "sensorHub"}

local function update(dt)
  M.rollAV, M.pitchAV, M.yawAV = obj:getRollPitchYawAngularVelocity()

  M.rollAVSmooth = rollAVSmoother:get(M.rollAV, dt)
  M.pitchAVSmooth = pitchAVSmoother:get(M.pitchAV, dt)
  M.yawAVSmooth = yawAVSmoother:get(M.yawAV, dt)

  local dYawAV = M.yawAV - lastYawAV
  M.yawAcceleration = dYawAV / dt
  M.steeringInput = electrics.values.steering_input

  M.roll, M.pitch = obj:getRollPitchYawRad()

  local ffisensors = sensors.ffiSensors
  M.accelerationX = ffisensors.sensorX
  M.accelerationY = ffisensors.sensorY
  M.accelerationZ = ffisensors.sensorZnonInertial

  M.accelerationXSmooth = accelerationXSmoother:get(M.accelerationX, dt)
  M.accelerationYSmooth = accelerationYSmoother:get(M.accelerationY, dt)
  M.accelerationZSmooth = accelerationZSmoother:get(M.accelerationZ, dt)

  local accXNoiseUpper = accXNoiseUpperSmoother:getUncapped(max(M.accelerationX, 0), dt)
  local accXNoiseLower = accXNoiseLowerSmoother:getUncapped(min(M.accelerationX, 0), dt)
  M.accNoiseX = min(abs(accXNoiseUpper - accXNoiseLower), 150)

  local accYNoiseUpper = accYNoiseUpperSmoother:getUncapped(max(M.accelerationY, 0), dt)
  local accYNoiseLower = accYNoiseLowerSmoother:getUncapped(min(M.accelerationY, 0), dt)
  M.accNoiseY = min(abs(accYNoiseUpper - accYNoiseLower), 150)

  local accZNoiseUpper = accZNoiseUpperSmoother:getUncapped(max(M.accelerationZ - M.gravity, 0), dt)
  local accZNoiseLower = accZNoiseLowerSmoother:getUncapped(min(M.accelerationZ - M.gravity, 0), dt)
  M.accNoiseZ = min(abs(accZNoiseUpper - accZNoiseLower), 150)
end

local function updateDebug(dt)
  update(dt)

  --Reference stuff, do not use for actual logic---
  M.worldVelocity = obj:getVelocity()
  M.directionVector = obj:getDirectionVector()
  M.directionVectorUp = obj:getDirectionVectorUp()
  M.directionVectorLeft = M.directionVectorUp:cross(M.directionVector)
  M.vX = M.worldVelocity:dot(M.directionVector)
  M.vY = M.worldVelocity:dot(M.directionVectorLeft)
  -------

  debugPacket.accelerationX = M.accelerationX
  debugPacket.accelerationY = M.accelerationY
  debugPacket.accelerationZ = M.accelerationZ - M.gravity

  debugPacket.accelerationXSmooth = M.accelerationXSmooth
  debugPacket.accelerationYSmooth = M.accelerationYSmooth
  debugPacket.accelerationZSmooth = M.accelerationZSmooth - M.gravity

  debugPacket.roll = M.roll
  debugPacket.pitch = M.pitch

  debugPacket.rollAV = M.rollAV
  debugPacket.pitchAV = M.pitchAV
  debugPacket.yawAV = M.yawAV

  debugPacket.rollAVSmooth = M.rollAVSmooth
  debugPacket.pitchAVSmooth = M.pitchAVSmooth
  debugPacket.yawAVSmooth = M.yawAVSmooth

  debugPacket.accNoiseX = M.accNoiseX
  debugPacket.accNoiseY = M.accNoiseY
  debugPacket.accNoiseZ = M.accNoiseZ
end

local function updateGFX(dt)
  --print(M.accNoiseX + M.accNoiseY + M.accNoiseZ)
  M.gravity = obj:getGravity()
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  CMU.sendDebugPacket(debugPacket)
end

local function reset()
  rollAVSmoother:reset()
  pitchAVSmoother:reset()
  yawAVSmoother:reset()

  accelerationXSmoother:reset()
  accelerationYSmoother:reset()
  accelerationZSmoother:reset()

  accXNoiseUpperSmoother:reset()
  accXNoiseLowerSmoother:reset()
  accYNoiseUpperSmoother:reset()
  accYNoiseLowerSmoother:reset()
  accZNoiseUpperSmoother:reset()
  accZNoiseLowerSmoother:reset()
end

local function init(jbeamData)
  M.gravity = obj:getGravity()
  lastYawAV = 0
  M.isActive = true
end

local function initLastStage()
end

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled

  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
  M.update = isDebugEnabled and updateDebug or update
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
M.initLastStage = initLastStage

M.reset = reset

M.updateGFX = updateGFX
M.update = update

M.registerCMU = registerCMU
M.setDebugMode = setDebugMode
M.shutdown = shutdown

return M
