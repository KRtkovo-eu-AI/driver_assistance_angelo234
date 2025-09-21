-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- PACENOTE ORBIT CAMERA

local C = {}
C.__index = C

function C:init()
  -- print('pacenoteOrbit cam init')
  self.isGlobal = true
  self.fixedTargetPos = vec3(0, 0, 0)
  self.fov = 65 -- deg
  -- self.fov = 1.13446 -- rad

  self:setupStuff()
  self:onSettingsChanged()
  self:reset()
end

function C:setupStuff()
  if self.defaultRotation == nil then
    self.defaultRotation = vec3(0, -17, 0)
  end
  self.defaultRotation = vec3(self.defaultRotation)
  if not self.camRot then self.camRot = vec3(self.defaultRotation) end
  self.camMinDist = 10
  self.camMaxDist = 1500
  local default
  self.defaultDistance = 300
  self.camDist = self.distance or self.defaultDistance
end

function C:onSettingsChanged()
  core_camera.clearInputs() --TODO is this really necessary?
  self.fovModifier = settings.getValue('cameraOrbitFovModifier')
  self.relaxation = settings.getValue('cameraOrbitRelaxation') or 3
  self.maxDynamicFov = settings.getValue('cameraOrbitMaxDynamicFov') or 35
  self.maxDynamicPitch = math.rad(settings.getValue('cameraOrbitMaxDynamicPitch') or 0)
  self.maxDynamicOffset = settings.getValue('cameraOrbitMaxDynamicOffset') or 0
  self.smoothingEnabled = settings.getValue('cameraOrbitSmoothing', true)
end

function C:reset()
  if self.cameraResetted == 0 then
    self.preResetPos = vec3(self.camLastTargetPos2)
    self.cameraResetted = 3
  end
end

function C:setRef(center, left, back)
  self.fixedTargetPos = center
end

function C:setDefaultDistance(d)
  self.defaultDistance = d
end

function C:update(data)
  data.res.collisionCompatible = true

  local targetPos = self.fixedTargetPos

  local yawDif = 0.1 * (MoveManager.yawRight - MoveManager.yawLeft)
  local pitchDif = 0.1 * (MoveManager.pitchDown - MoveManager.pitchUp)

  -- Camera rotation around the fixed point based on user input
  local maxRot = 180 -- rotation speed
  local dtfactor = data.dt * 1000
  local mouseYaw = sign(MoveManager.yawRelative) * math.min(math.abs(MoveManager.yawRelative * 10), maxRot * data.dt) + yawDif * dtfactor
  local mousePitch = sign(-MoveManager.pitchRelative) * math.min(math.abs(MoveManager.pitchRelative * 10), maxRot * data.dt) + pitchDif * dtfactor

  -- Keyboard input for rotation
  local maxRotKeyboard = 80 -- rotation speed
  local keyboardYaw = (MoveManager.left - MoveManager.right) * maxRotKeyboard * data.dt
  local keyboardPitch = (MoveManager.forward - MoveManager.backward) * maxRotKeyboard * data.dt

  if mouseYaw ~= 0 or mousePitch ~= 0 then
    self.camRot.x = self.camRot.x - mouseYaw
    self.camRot.y = self.camRot.y - mousePitch
  end

  if keyboardYaw ~= 0 or keyboardPitch ~= 0 then
    self.camRot.x = self.camRot.x - keyboardYaw
    self.camRot.y = self.camRot.y - keyboardPitch
  end

  self.camRot.y = math.min(math.max(self.camRot.y, -85), 85)

  -- Ensure the rotation is within bounds
  self.camRot.x = (self.camRot.x + 360) % 360
  if self.camRot.x > 180 then
    self.camRot.x = self.camRot.x - 360
  end

  -- Zoom control
  local zoomChange = MoveManager.zoomIn - MoveManager.zoomOut

  MoveManager.zoomIn = 0
  MoveManager.zoomOut = 0

  local zoomSpeed = 3.0
  self.camDist = clamp(self.camDist + zoomChange * dtfactor * zoomSpeed, self.camMinDist, self.camMaxDist)

  -- Calculate the new camera position based on rotation and distance
  local rot = vec3(math.rad(self.camRot.x), math.rad(self.camRot.y), 0)
  local calculatedCamPos = vec3(
    math.sin(rot.x) * math.cos(rot.y),
    -math.cos(rot.x) * math.cos(rot.y),
    -math.sin(rot.y)
  )
  calculatedCamPos:normalize()
  calculatedCamPos = calculatedCamPos * self.camDist
  local camPos = targetPos + calculatedCamPos


  -- Get the terrain height at the new camera position
  -- Note: Assuming newPos should be camPos in your context and considering BeamNG's Z-up coordinate system
  local terrainHeight = core_terrain.getTerrainHeight(vec3(camPos.x, camPos.y, 0))

  -- Adjust camPos.z to ensure it's not below the terrain height
  -- Add some offset if needed to prevent clipping with the terrain surface
  local offsetAboveTerrain = 1.0 -- Adjust this value as needed
  if terrainHeight and camPos.z < terrainHeight + offsetAboveTerrain then
    camPos.z = terrainHeight + offsetAboveTerrain
  end

  -- Apply the calculated camera position and orientation
  data.res.pos = camPos
  data.res.rot = quatFromDir(targetPos - camPos)
  data.res.fov = self.fov

  return true
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
  local o = ... or {}
  setmetatable(o, C)
  o:init()
  return o
end