-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'To String'
C.description = 'Attempts to create a string out of the input.'
C.category = 'simple'

C.pinSchema = {
  { dir = 'in', type = 'any', name = 'value', description = 'Value to attempt to turn into a string.' },
  { dir = 'out', type = 'string', name = 'value', description = 'Result string of transformed value.' },
}

C.tags = {'string', 'tostring'}

function C:work()
  self.pinOut.value.value = tostring(self.pinIn.value.value)
end

return _flowgraph_createNode(C)
