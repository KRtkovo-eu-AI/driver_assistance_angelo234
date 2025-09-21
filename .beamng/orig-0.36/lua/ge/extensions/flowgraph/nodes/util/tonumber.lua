-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'To Number'
C.description = 'Attempts to create a number out of the input.'
C.todo = 'Warning, this may return nil.'
C.category = 'simple'

C.pinSchema = {
  { dir = 'in', type = 'any', name = 'value', description = 'Value to attempt to turn into a number.' },
  { dir = 'out', type = 'number', name = 'value', description = 'Result number of transformed value.' },
}

C.tags = {'number', 'tonumber'}

function C:work()
  self.pinOut.value.value = tonumber(self.pinIn.value.value)
end

return _flowgraph_createNode(C)
