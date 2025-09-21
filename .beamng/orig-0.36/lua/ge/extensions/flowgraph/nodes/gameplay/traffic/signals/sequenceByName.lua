-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Sequence by Name'
C.description = 'Finds a signal sequence (signal group / phase logic) by name.'
C.color = ui_flowgraph_editor.nodeColors.signals
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'once_instant'
C.pinSchema = {
  {dir = 'in', type = 'table', name = 'signalsData', tableType = 'signalsData', description = 'Table of traffic signals data; use the File Traffic Signals node.'},
  {dir = 'in', type = 'string', name = 'name', description = 'Name of the signal.'},
  {dir = 'out', type = 'bool', name = 'exists', hidden = true, description = 'True if data exists.'},
  {dir = 'out', type = 'table', name = 'sequenceData', tableType = 'signalSequenceData', description = 'Signal sequence data.'}
}

C.tags = {'traffic', 'signals'}

function C:onNodeReset()
  self.pinOut.sequenceData.value = nil
end

function C:init()
  self:onNodeReset()
end

function C:_executionStopped()
  self:onNodeReset()
end

function C:work(args)
  if not self.pinOut.sequenceData.value then
    if self.pinIn.signalsData.value and self.pinIn.name.value then
      for _, sequence in ipairs(self.pinIn.signalsData.value.sequences) do
        if sequence.name == self.pinIn.name.value then
          self.pinOut.sequenceData.value = sequence
          break
        end
      end
    end
  end

  self.pinOut.exists.value = self.pinOut.sequenceData.value and true or false
end

return _flowgraph_createNode(C)