-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Get Signal Sequence'
C.description = 'Gets properties of a signal sequence.'
C.color = ui_flowgraph_editor.nodeColors.signals
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'repeat_instant'

C.pinSchema = {
  {dir = 'in', type = 'table', name = 'sequenceData', tableType = 'signalSequenceData', description = 'Signal sequence data.'},
  {dir = 'out', type = 'number', name = 'step', description = 'Current step index.'},
  {dir = 'out', type = 'number', name = 'maxSteps', description = 'Total number of steps in the sequence.'},
  {dir = 'out', type = 'number', name = 'phase', description = 'Current phase index.'},
  {dir = 'out', type = 'bool', name = 'active', hidden = true, description = 'True while the sequence is active.'},
  {dir = 'out', type = 'bool', name = 'timed', hidden = true, description = 'True while the sequence is using the timer.'}
}

C.tags = {'traffic', 'signals'}

function C:work(args)
  local sequence = self.pinIn.sequenceData.value
  if sequence then
    self.pinOut.step.value = sequence.currStep
    self.pinOut.maxSteps.value = #sequence.sequenceTimings
    self.pinOut.phase.value = sequence.currPhase
    self.pinOut.active.value = sequence.active
    self.pinOut.timed.value = not sequence.ignoreTimer
  end
end

return _flowgraph_createNode(C)