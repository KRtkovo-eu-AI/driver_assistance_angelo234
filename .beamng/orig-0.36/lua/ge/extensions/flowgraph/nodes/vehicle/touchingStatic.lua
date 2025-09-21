-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Static Object Touch'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle

C.description = 'Flows if a vehicle is intersecting with static objects or terrain in the world, by using raycasts.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'Id of vehicle to check collision for.' },
  { dir = 'in', type = 'number', name = 'widthOffset', hidden = true, description = 'Width offset, relative to vehicle.' },
  { dir = 'in', type = 'number', name = 'lengthOffset', hidden = true, description = 'Length offset, relative to vehicle.' },
  { dir = 'in', type = 'number', name = 'heightOffset', hidden = true, default = 0.5, description = 'Height offset from ground.' },
  { dir = 'in', type = 'bool', name = 'debugMode', hidden = true, description = 'Display debug drawing for the collision checks.' },

  { dir = 'out', type = 'flow', name = 'touching', description = 'Outflow for this node when touching.' },
  { dir = 'out', type = 'flow', name = 'touchFL', hidden = true, description = 'Front-left collision.' },
  { dir = 'out', type = 'flow', name = 'touchF', hidden = true, description = 'Front collision.' },
  { dir = 'out', type = 'flow', name = 'touchFR', hidden = true, description = 'Front-right collision.' },
  { dir = 'out', type = 'flow', name = 'touchR', hidden = true, description = 'Right collision.' },
  { dir = 'out', type = 'flow', name = 'touchBR', hidden = true, description = 'Back-right collision.' },
  { dir = 'out', type = 'flow', name = 'touchB', hidden = true, description = 'Back collision.' },
  { dir = 'out', type = 'flow', name = 'touchBL', hidden = true, description = 'Back-left collision.' },
  { dir = 'out', type = 'flow', name = 'touchL', hidden = true, description = 'Left collision.' },
  { dir = 'out', type = 'bool', name = 'isTouching', hidden = true, description = 'True while touching is active.' }
}
C.tags = {'collision', 'collide', 'hit', 'static'}

-- TODO: make this a generic function outside of the flowgraph environment

function C:init()
  self:reset()
end

function C:_executionStarted()
  self.points = {FL = {pos = vec3()}, FR = {pos = vec3()}, BR = {pos = vec3()}, BL = {pos = vec3()}, F = {pos = vec3()}, R = {pos = vec3()}, B = {pos = vec3()}, L = {pos = vec3()}}
  self.center, self.xOffset, self.yOffset, self.zOffset = vec3(), vec3(), vec3(), vec3()
end

function C:_executionStopped()
  self.points, self.center, self.xOffset, self.yOffset, self.zOffset = nil, nil, nil, nil, nil
  self:reset()
end

function C:reset()
  for _, pin in pairs(self.pinOut) do
    pin.value = false
  end
  self.active = false
end

function C:work()
  if not self.pinIn.vehId.value or not getObjectByID(self.pinIn.vehId.value) then
    self:reset()
    return
  end

  self.active = true

  local veh = getObjectByID(self.pinIn.vehId.value)
  local oobb = veh:getSpawnWorldOOBB()

  self.xOffset:set(veh:getDirectionVector():cross(veh:getDirectionVectorUp()) * (self.pinIn.widthOffset.value or 0))
  self.yOffset:set(veh:getDirectionVector() * (self.pinIn.lengthOffset.value or 0))
  self.zOffset:set(veh:getDirectionVectorUp() * (self.pinIn.heightOffset.value or 0.5))

  self.points.FL.pos:set(oobb:getPoint(0) - self.xOffset + self.yOffset + self.zOffset)
  self.points.FR.pos:set(oobb:getPoint(3) + self.xOffset + self.yOffset + self.zOffset)
  self.points.BR.pos:set(oobb:getPoint(7) + self.xOffset - self.yOffset + self.zOffset)
  self.points.BL.pos:set(oobb:getPoint(4) - self.xOffset - self.yOffset + self.zOffset)

  self.points.F.pos:set(linePointFromXnorm(self.points.FL.pos, self.points.FR.pos, 0.5))
  self.points.R.pos:set(linePointFromXnorm(self.points.FR.pos, self.points.BR.pos, 0.5))
  self.points.B.pos:set(linePointFromXnorm(self.points.BR.pos, self.points.BL.pos, 0.5))
  self.points.L.pos:set(linePointFromXnorm(self.points.BL.pos, self.points.FL.pos, 0.5))

  self.center:set(linePointFromXnorm(self.points.F.pos, self.points.B.pos, 0.5))

  -- consider spreading out the frames for the static ray casts
  self.pinOut.touching.value = false
  self.pinOut.isTouching.value = false
  for k, v in pairs(self.points) do
    local dist = v.pos:distance(self.center)
    local hitDist = castRayStatic(self.center, v.pos - self.center, dist + 1e-6)
    v.hit = hitDist <= dist
    self.pinOut["touch"..k].value = v.hit
    if v.hit then
      self.pinOut.touching.value = true
      self.pinOut.isTouching.value = true
    end
  end
end

function C:onPreRender()
  if self.active and self.pinIn.debugMode.value then
    for k, v in pairs(self.points) do
      local color = v.hit and ColorF(1, 0, 0, 1) or ColorF(0, 1, 0, 0.5)
      debugDrawer:drawSphere(v.pos, 0.1, color)
    end
  end
end

return _flowgraph_createNode(C)
