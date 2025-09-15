-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Line Point From Xnorm'
C.description = 'Returns the position that is between two line points, given the xnorm (scalar ratio).'
C.category = 'repeat_instant'
C.color = ui_flowgraph_editor.nodeColors.default
C.pinSchema = {
  { dir = 'in', type = 'vec3', name = 'posA', description = "Line start position." },
  { dir = 'in', type = 'vec3', name = 'posB', description = "Line end position." },
  { dir = 'in', type = 'number', name = 'xnorm', description = "Xnorm value (usually between 0 and 1); if none given, assumes midpoint (0.5)." },
  { dir = 'out', type = 'vec3', name = 'pos', description = "Calculated position." }
}

C.tags = {'vec3', 'vector', 'line'}

local posA = vec3()
local posB = vec3()
function C:work()
  if self.pinIn.posA.value and self.pinIn.posB.value then
    posA:setFromTable(self.pinIn.posA.value)
    posB:setFromTable(self.pinIn.posB.value)
    self.pinOut.pos.value = linePointFromXnorm(posA, posB, self.pinIn.xnorm.value or 0.5):toTable()
  end
end

return _flowgraph_createNode(C)
