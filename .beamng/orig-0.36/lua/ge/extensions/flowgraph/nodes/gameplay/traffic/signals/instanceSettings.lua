-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Signal Instance Settings'
C.description = 'Sets properties of a signal instance.'
C.color = ui_flowgraph_editor.nodeColors.signals
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'once_instant'

C.pinSchema = {
  {dir = 'in', type = 'table', name = 'instanceData', tableType = 'signalInstanceData', description = 'Signal instance data.'},
  {dir = 'in', type = 'bool', name = 'active', description = 'Enables or disables the signal.'},
  {dir = 'in', type = 'number', name = 'stateIndex', description = 'Force sets the signal state, by controller state index (use 0 to reset).'},
  {dir = 'in', type = 'table', name = 'sequence', tableType = 'signalSequenceData', hidden = true, description = 'Changes the sequence to use (refreshes state if active).'},
  {dir = 'in', type = 'table', name = 'controller', tableType = 'signalControllerData', hidden = true, description = 'Changes the controller to use (refreshes state if active).'}
}

function C:workOnce()
  local instance = self.pinIn.instanceData.value
  if instance then
    if self.pinIn.sequence.value ~= nil then
      instance:setSequence(self.pinIn.sequence.value.id)
    end
    if self.pinIn.controller.value ~= nil then
      instance:setController(self.pinIn.controller.value.id)
    end
    if self.pinIn.active.value ~= nil then
      instance:setActive(self.pinIn.active.value)
    end
    if self.pinIn.stateIndex.value ~= nil then
      instance:setStrictState(self.pinIn.stateIndex.value)
    end
  end
end

return _flowgraph_createNode(C)