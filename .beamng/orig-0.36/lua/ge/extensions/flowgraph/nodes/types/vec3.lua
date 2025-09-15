-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Vec3'
C.description = "Provides a vector3."
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'x', description = 'The x value.' },
  { dir = 'in', type = 'number', name = 'y', description = 'The y value.' },
  { dir = 'in', type = 'number', name = 'z', description = 'The z value.' },
  { dir = 'in', type = 'quat', name = 'rot', hidden = true, description = '(Optional) Converts from a quaternion instead.' },
  { dir = 'out', type = 'vec3', name = 'value', description = 'The vector3 value.' }
}

C.tags = {'vec3', 'vector', 'direction'}

local tempQuat = quat()
local vecOut = vec3()
local vecY = vec3(0, 1, 0)
function C:work()
  if self.pinIn.rot.value then
    local rot = self.pinIn.rot.value
    tempQuat:set(rot[1], rot[2], rot[3], rot[4])
    vecOut:set(vecY:rotated(tempQuat))
    self.pinOut.value.value = {vecOut.x, vecOut.y, vecOut.z}
  else
    self.pinOut.value.value = {self.pinIn.x.value or 0, self.pinIn.y.value or 0, self.pinIn.z.value or 0}
  end
end

function C:drawMiddle(builder, style)
  builder:Middle()
  im.Text(tostring(vec3(self.pinIn.x.value or 0, self.pinIn.y.value or 0, self.pinIn.z.value or 0)))
end

function C:drawProperties()
end

function C:_onSerialize(res)
end

function C:_onDeserialized(nodeData)
end

return _flowgraph_createNode(C)
