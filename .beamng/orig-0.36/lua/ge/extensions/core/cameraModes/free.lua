-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
C.__index = C

local manualzoom = require('core/cameraModes/manualzoom')

local rotEulerTemp = quat()
local function setRotateEuler(x, y, z, qSource, qDest)
  rotEulerTemp:setFromEuler(0, z, 0)
  qDest:setMul2(rotEulerTemp, qSource)
  rotEulerTemp:setFromEuler(0, 0, x)
  qDest:setMul2(rotEulerTemp, qDest)
  rotEulerTemp:setFromEuler(y, 0, 0)
  qDest:setMul2(rotEulerTemp, qDest)
  return qDest
end

function C:reset()
  self.angularVelocity = vec3(0,0,0)
  self.velocity = vec3(0,0,0)
  self.manualzoom = manualzoom()
  self.manualzoom:init(65)
  self.rot.z = 0 -- reset roll
end

function C:setSmoothedCam(smoothed)
  if smoothed then
    self.angularForce = 150
    self.angularDrag = 2.5
    self.mass = 10
    self.translationForce = 600
    self.translationDrag = 2
    self.angularVelocity = vec3(0,0,0)
  else
    self.angularForce = 400
    self.angularDrag = 16
    self.mass = 1
    self.translationForce = 250
    self.translationDrag = 17
  end
end

function C:init()
  self.isGlobal = true
  self.hidden = true

  self:setSmoothedCam(false)

  self.pos = vec3(0,0,0)
  self.rot = vec3(0,0,0)
  self.newtonTranslation = true
  self.newtonRotation = true
  self:reset()
end

function C:setPosition(position)
  self.pos = position
end

function C:setFOV(fovDeg)
  self.manualzoom:init(fovDeg)
end

function C:setNewtonRotation(enabled)
  self.newtonRotation = enabled
end

function C:setNewtonTranslation(enabled)
  self.newtonTranslation = enabled
end

function C:setRotation(rotation)
  local eulerYXZ = rotation:toEulerYXZ()
  eulerYXZ.y = -eulerYXZ.y
  self.rot:set(eulerYXZ)
end

local inputVec, acc, forceVec, tempVec = vec3(), vec3(), vec3(), vec3()
local qdir, qdirLook = quat(), quat()
function C:update(data)
  -- Rotation
  local dtFactor = data.dt * 200
  inputVec:set(
    MoveManager.yawRelative / dtFactor + (MoveManager.yawRight - MoveManager.yawLeft) * 0.07,
    MoveManager.pitchRelative / dtFactor + (MoveManager.pitchUp - MoveManager.pitchDown) * 0.07,
    MoveManager.rollRelative / dtFactor + (MoveManager.rollLeft - MoveManager.rollRight) * 0.07
  )
  if self.newtonRotation then
    acc:set(0,0,0)
    if inputVec:squaredLength() > 0 then
      acc:setScaled2(inputVec, self.angularForce / self.mass)
    end
    forceVec:setScaled2(acc, data.dt) -- Acceleration
    forceVec:set(push3(forceVec) - push3(self.angularVelocity) * math.min(self.angularDrag * data.dt, 1)) -- Drag
    self.angularVelocity:setAdd(forceVec)
  else
    self.angularVelocity:setScaled2(inputVec, 30)
  end

  self.rot:set(push3(self.rot) + push3(self.angularVelocity) * data.dt) -- Rotate
  self.rot.y = clamp(self.rot.y, -1.5706, 1.5706)

  qdir:set(0,0,0,1)
  setRotateEuler(self.rot.x, -self.rot.y, 0, qdir, qdirLook)
  setRotateEuler(0, 0, self.rot.z, qdirLook, qdir)

  -- Translation
  acc:set(0,0,0)
  inputVec:set(
    MoveManager.right - MoveManager.left + MoveManager.absXAxis,
    MoveManager.forward - MoveManager.backward + MoveManager.absYAxis,
    MoveManager.up - MoveManager.down + MoveManager.absZAxis
  )

  local modifiedSpeed = data.fastSpeedModifier and data.speed * 3 or data.speed
  local adjustedSpeed = ((modifiedSpeed^2)/30) / 40
  if self.newtonTranslation then
    local force = self.translationForce * adjustedSpeed
    if inputVec:squaredLength() > 0 then
      acc:setScaled2(inputVec, force / self.mass)
    end
    forceVec:setScaled2(acc, data.dt) -- Acceleration
    forceVec:set(push3(forceVec) - push3(self.velocity) * math.min(self.translationDrag * data.dt, 1)) -- Drag
    self.velocity:setAdd(forceVec)
  else
    self.velocity:setScaled2(inputVec, adjustedSpeed * 15)
  end
  tempVec:setRotate(qdir, self.velocity)
  self.pos:set(push3(self.pos) + push3(tempVec) * data.dt) -- Move

  data.res.rot = qdir
  data.res.pos:set(self.pos)

  self.manualzoom:update(data)
  return true
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
  local o = ... or {}
  setmetatable(o, C)
  o:init()
  return o
end
