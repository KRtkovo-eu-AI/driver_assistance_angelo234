-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Get First Element of Table'

C.description = 'Get the first element of a table.'
C.category = 'repeat_instant'
C.tags = {"Table", "Get", "First", "Element", "Index"}
C.pinSchema = {
  { dir = 'in', type = 'table', name = 'table', description = "The table to get the first element of." },
  { dir = 'out', type = 'any', name = 'firstElement', description = "The first element of the table." },
}

C.color = ui_flowgraph_editor.nodeColors.default

function C:init(mgr)
end

function C:_executionStarted()
  self.oldPos = nil
end

function C:getFirstElement(table)
  if self.pinIn.table.value then
    return self.pinIn.table.value[1]
  end
end

function C:work()
  if not self.pinIn.table.value then return end
  self.pinOut.firstElement.value = self:getFirstElement(self.pinIn.table.value)
end

return _flowgraph_createNode(C)
