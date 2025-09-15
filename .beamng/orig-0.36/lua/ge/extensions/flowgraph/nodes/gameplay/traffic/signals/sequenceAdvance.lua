-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Advance Sequence'
C.description = 'Advances the signal sequence to the next step.'
C.color = ui_flowgraph_editor.nodeColors.signals
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'once_instant'


C.pinSchema = {
  {dir = 'in', type = 'table', name = 'sequenceData', tableType = 'signalSequenceData', description = 'Signal sequence data.'},
}

C.tags = {'traffic', 'signals'}

function C:workOnce()
  local sequence = self.pinIn.sequenceData.value
  if sequence then
    self.pinIn.sequenceData.value:advance()
  end
end

return _flowgraph_createNode(C)