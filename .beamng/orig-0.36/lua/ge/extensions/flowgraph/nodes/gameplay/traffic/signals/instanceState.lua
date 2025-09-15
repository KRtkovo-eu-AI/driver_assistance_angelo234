-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Predict Signal State'
C.description = 'Predicts the future state of a signal from the given seconds.'
C.color = ui_flowgraph_editor.nodeColors.signals
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'once_instant'

C.pinSchema = {
  {dir = 'in', type = 'table', name = 'instanceData', tableType = 'signalInstanceData', description = 'Signal instance data.'},
  {dir = 'in', type = 'number', name = 'seconds', description = 'Future time, in seconds; only positive values are supported.'},
  {dir = 'out', type = 'string', name = 'stateName', description = 'Predicted signal state name.'},
  {dir = 'out', type = 'string', name = 'stateAction', description = 'Predicted signal state action.'}
}

C.tags = {'traffic', 'signals'}

function C:workOnce()
  local instance = self.pinIn.instanceData.value
  if instance then
    local stateName, stateData = instance:getStateAfterTime(self.pinIn.seconds.value)
    self.pinOut.stateName.value = stateName
    self.pinOut.stateAction.value = stateData.action
  end
end

return _flowgraph_createNode(C)