-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Signal Sequence Settings'
C.description = 'Sets properties of a signal sequence.'
C.color = ui_flowgraph_editor.nodeColors.signals
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'once_instant'

C.pinSchema = {
  {dir = 'in', type = 'table', name = 'sequenceData', tableType = 'signalSequenceData', description = 'Signal sequence data.'},
  {dir = 'in', type = 'bool', name = 'active', description = 'Enables or disables the sequence.'},
  {dir = 'in', type = 'bool', name = 'timed', description = 'Enables or disables the sequence timer.'},
  {dir = 'in', type = 'number', name = 'stepIndex', description = 'Force sets the step to jump to in the sequence (has priority over phase index).'},
  {dir = 'in', type = 'number', name = 'phaseIndex', description = 'Force sets the phase to jump to in the sequence.'}
}

C.tags = {'traffic', 'signals'}

function C:workOnce()
  local sequence = self.pinIn.sequenceData.value
  if sequence then
    if self.pinIn.active.value ~= nil then
      sequence:setActive(self.pinIn.active.value)
    end
    if self.pinIn.timed.value ~= nil then
      sequence:enableTimer(self.pinIn.timed.value)
    end
    if self.pinIn.stepIndex.value ~= nil then
      sequence:setStep(self.pinIn.stepIndex.value)
    elseif self.pinIn.phaseIndex.value ~= nil then
      sequence:setPhase(self.pinIn.phaseIndex.value)
    end
  end
end

return _flowgraph_createNode(C)