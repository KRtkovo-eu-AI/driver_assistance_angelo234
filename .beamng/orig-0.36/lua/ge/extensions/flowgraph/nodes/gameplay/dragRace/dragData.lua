-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Drag Race Data'

C.description = 'Get Drag Data from the drag system'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'reset', description = 'Reset the node.', impulse = true },
  { dir = 'in', type = 'flow', name = 'start', description = 'Start the activity.', impulse = true },
  { dir = 'out', type = 'flow', name = 'flow', description = 'Outflow for this node.'},

  { dir = 'out', type = 'flow', name = 'started', description = 'The activity has started.', impulse = true },
  { dir = 'out', type = 'flow', name = 'completed', description = 'The activity has ended', impulse = true },


  { dir = 'out', type = 'string', name = 'dragType', description = ''},
  { dir = 'out', type = 'string', name = 'context', description = ''},
}

C.tags = {'gameplay', 'utils'}

function C:_executionStarted()
end

function C:work()
  if self.pinIn.reset.value then
    self.data = gameplay_drag_general.getData()
    self.pinOut.started.value = false
    self.pinOut.completed.value = false
    return
  end

  if self.pinIn.start.value and not self.data.isStarted then
    gameplay_drag_general.startDragRaceActivity()
  end

  self.pinOut.started.value = self.data.isStarted
  self.pinOut.completed.value = self.data.isCompleted
  self.pinOut.dragType.value = self.data.dragType
  self.pinOut.context.value = self.data.context

end

return _flowgraph_createNode(C)