-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Random Quaternion'
C.tags = {'quat', 'quaternion', 'rotation', 'random'}
C.icon = "casino"
C.description = "Provides a random quaternion."
C.category = 'repeat_instant'

C.pinSchema = {
  {dir = 'in', type = 'number', name = 'xMinAngle', default = 0, hardcoded = true, description = "Minimum angle for the X axis, in degrees."},
  {dir = 'in', type = 'number', name = 'xMaxAngle', default = 0, hardcoded = true, description = "Maximum angle for the X axis, in degrees."},
  {dir = 'in', type = 'number', name = 'yMinAngle', default = 0, hardcoded = true, description = "Minimum angle for the Y axis, in degrees."},
  {dir = 'in', type = 'number', name = 'yMaxAngle', default = 0, hardcoded = true, description = "Maximum angle for the Y axis, in degrees."},
  {dir = 'in', type = 'number', name = 'zMinAngle', default = 0, hardcoded = true, description = "Minimum angle for the Z axis, in degrees."},
  {dir = 'in', type = 'number', name = 'zMaxAngle', default = 360, hardcoded = true, description = "Maximum angle for the Z axis, in degrees."},
  {dir = 'out', type = 'quat', name = 'quaternion', description = "The quaternion value."}
}

function C:work()
  local x = lerp(math.rad(self.pinIn.xMinAngle.value or 0), math.rad(self.pinIn.xMaxAngle.value or 0), math.random())
  local y = lerp(math.rad(self.pinIn.yMinAngle.value or 0), math.rad(self.pinIn.yMaxAngle.value or 0), math.random())
  local z = lerp(math.rad(self.pinIn.zMinAngle.value or 0), math.rad(self.pinIn.zMaxAngle.value or 0), math.random())

  local q = quat(0, 0, 0, 1)
  q = quatFromEuler(x, 0, 0) * q
  q = quatFromEuler(0, y, 0) * q
  q = quatFromEuler(0, 0, z) * q
  self.pinOut.quaternion.value = q:toTable()
end

return _flowgraph_createNode(C)
