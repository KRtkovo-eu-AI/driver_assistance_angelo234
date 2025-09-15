-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Random Vector'
C.tags = {'vec3', 'vector', 'random'}
C.icon = "casino"
C.description = "Provides a random vector (not normalized)."
C.category = 'repeat_instant'

C.pinSchema = {
  {dir = 'in', type = 'number', name = 'xMin', default = -1, hardcoded = true, description = "Minimum value for the X axis."},
  {dir = 'in', type = 'number', name = 'xMax', default = 1, hardcoded = true, description = "Maximum value for the X axis."},
  {dir = 'in', type = 'number', name = 'yMin', default = -1, hardcoded = true, description = "Minimum value for the Y axis."},
  {dir = 'in', type = 'number', name = 'yMax', default = 1, hardcoded = true, description = "Maximum value for the Y axis."},
  {dir = 'in', type = 'number', name = 'zMin', default = -1, hardcoded = true, description = "Minimum value for the Z axis."},
  {dir = 'in', type = 'number', name = 'zMax', default = 1, hardcoded = true, description = "Maximum value for the Z axis."},
  {dir = 'out', type = 'vec3', name = 'vector', description = "The vector value."}
}

function C:work()
  local x = lerp(self.pinIn.xMin.value or 0, self.pinIn.xMax.value or 0, math.random())
  local y = lerp(self.pinIn.yMin.value or 0, self.pinIn.yMax.value or 0, math.random())
  local z = lerp(self.pinIn.zMin.value or 0, self.pinIn.zMax.value or 0, math.random())
  self.pinOut.vector.value = {x, y, z}
end

return _flowgraph_createNode(C)
