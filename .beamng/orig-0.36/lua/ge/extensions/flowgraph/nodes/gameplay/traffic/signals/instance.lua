-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Get Signal Instance'
C.description = 'Gets properties of a signal instance.'
C.color = ui_flowgraph_editor.nodeColors.signals
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'repeat_instant'

C.pinSchema = {
  {dir = 'in', type = 'table', name = 'instanceData', tableType = 'signalInstanceData', description = 'Signal instance data.'},
  {dir = 'out', type = 'vec3', name = 'pos', description = 'Signal stop position.'},
  {dir = 'out', type = 'vec3', name = 'dirVec', description = 'Signal direction.'},
  {dir = 'out', type = 'vec3', name = 'targetPos', description = 'Signal target position (e.g. intersection).'},
  {dir = 'out', type = 'number', name = 'radius', hidden = true, description = 'Signal range distance.'},
  {dir = 'out', type = 'string', name = 'stateName', description = 'Signal state name.'},
  {dir = 'out', type = 'string', name = 'stateAction', description = 'Signal state action.'},
  {dir = 'out', type = 'bool', name = 'active', hidden = true, description = 'True while the signal is active.'},
  {dir = 'out', type = 'table', name = 'sequenceData', tableType = 'signalSequenceData', description = 'Signal sequence data, to use with other signal nodes.'},
  {dir = 'out', type = 'table', name = 'controllerData', tableType = 'signalControllerData', description = 'Signal controller data, to use with other signal nodes.'}
}

C.tags = {'traffic', 'signals'}

function C:work(args)
  local instance = self.pinIn.instanceData.value
  if instance then
    self.pinOut.pos.value = instance.pos
    self.pinOut.dirVec.value = instance.dirVec
    self.pinOut.targetPos.value = instance.targetPos or instance.pos -- get target pos?
    self.pinOut.radius.value = instance.radius

    local stateName, stateData = instance:getState()
    self.pinOut.stateName.value = stateName
    self.pinOut.stateAction.value = stateData.action
    self.pinOut.active.value = instance.active

    self.pinOut.sequenceData.value = instance:getSequence() or {}
    self.pinOut.controllerData.value = instance:getController() or {}
  end
end

return _flowgraph_createNode(C)