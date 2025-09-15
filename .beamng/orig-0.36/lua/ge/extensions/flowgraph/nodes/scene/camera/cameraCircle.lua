-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Camera Auto Circle'
C.description = 'Makes the camera move in a circle around a vehicle or point.'
C.category = 'once_p_duration'
C.color = ui_flowgraph_editor.nodeColors.camera
C.icon = ui_flowgraph_editor.nodeIcons.camera

C.pinSchema = {
  {dir = 'in', type = 'number', name = 'vehId', description = 'Vehicle id to focus on; if nil, uses player vehicle.'},
  {dir = 'in', type = 'vec3', name = 'centerPos', hidden = true, description = '(Optional) If given, camera targets this position instead.'},
  {dir = 'in', type = 'number', name = 'startAngle', hidden = true, description = '(Optional) Start angle of circle (in degrees).'},
  {dir = 'in', type = 'number', name = 'radius', description = 'Base radius from center point.'},
  {dir = 'in', type = 'number', name = 'height', description = 'Height from center point.'},
  {dir = 'in', type = 'bool', name = 'snapHeight', hidden = true, description = 'If true, height is relative to current position over terrain.'},
  {dir = 'in', type = 'number', name = 'speed', description = 'Camera speed for clockwise movement; use negative values to go counterclockwise.'}
}

C.tags = {'camera', 'path', 'radial', 'orbit'}

local vecUp = vec3(0, 0, 1)
function C:_executionStarted()
  self:onNodeReset()
end

function C:onNodeReset()
  self.centerPos = vec3()
  self.offset = vec3()
  self.camPos = vec3()
  self.angle = 0
end

function C:workOnce()
  if not commands.isFreeCamera() then
    commands.setFreeCamera()
  end
  self.angle = self.pinIn.startAngle.value or self.angle
  self.angle = self.angle + 180
  if self.pinIn.vehId.value then -- if vehicle id is given, the start angle is relative to the vehicle orientation
    local veh = getObjectByID(self.pinIn.vehId.value)
    if veh then
      local dirVec = veh:getDirectionVector()
      self.angle = self.angle + math.deg(math.atan2(dirVec.x, dirVec.y))
    end
  end
end

function C:work()
  if self.pinIn.centerPos.value then
    self.centerPos:setFromTable(self.pinIn.centerPos.value)
  else
    local veh = getObjectByID(self.pinIn.vehId.value or be:getPlayerVehicleID(0))
    if veh then
      self.centerPos:set(veh:getSpawnWorldOOBB():getCenter())
    end
  end

  local radius = self.pinIn.radius.value or 5
  local height = self.pinIn.height.value or 0
  local speed = self.pinIn.speed.value or 10
  local aRad = math.rad(self.angle)
  local prevZ = self.camPos.z == 0 and self.centerPos.z or self.camPos.z

  self.offset:set(math.sin(aRad), math.cos(aRad), 0)

  self.camPos:set(self.centerPos)
  self.camPos:setAdd(self.offset * radius)
  self.camPos.z = self.camPos.z + height
  if self.pinIn.snapHeight.value then
    local nextZ = be:getSurfaceHeightBelow(self.camPos) + height
    if nextZ < self.centerPos.z + math.min(-1, height - 1) then -- if surface height is too low
      nextZ = self.camPos.z
    end
    self.camPos.z = lerp(prevZ, nextZ, 0.05) -- smoothing, to prevent rough steps
  end

  local q = quatFromDir(self.centerPos - self.camPos, vecUp)
  core_camera.setPosRot(0, self.camPos.x, self.camPos.y, self.camPos.z, q.x, q.y, q.z, q.w)

  self.angle = self.angle + self.mgr.dtReal * speed
end

return _flowgraph_createNode(C)
