-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Chainflow Branch'
C.icon = "fg_sideways"
C.description = 'Lets the flow through either out pin depending on a condition (chain flow only).'
C.category = 'logic'
C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'in', type = 'bool', name = 'condition', description = '' },
  { dir = 'out', type = 'flow', name = 'True', description = '', chainFlow = true },
  { dir = 'out', type = 'flow', name = 'False', description = '', chainFlow = true },
}

C.tags = { 'if', 'decision', 'true', 'false'}

function C:work()
  if self.pinIn.flow.value then
    if self.pinIn.condition.value then
      self.pinOut.True.value = true
      self.pinOut.False.value = false
    else
      self.pinOut.True.value = false
      self.pinOut.False.value = true
    end
  else
    self.pinOut.True.value = false
    self.pinOut.False.value = false
  end
end

return _flowgraph_createNode(C)
