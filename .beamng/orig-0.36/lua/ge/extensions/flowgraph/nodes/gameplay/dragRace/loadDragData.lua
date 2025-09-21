-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Load Drag Race Data'

C.description = 'Initialize all the Drag System'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'string', name = 'filepath', description = "File path of the dragData"},
  { dir = 'out', type = 'flow', name = 'flow', description = 'Impulse out flow when data is loaded', impulse = true },
}

C.tags = {'gameplay', 'utils'}

function C:work()
  if not self.pinIn.filepath.value then return end

  if not self.pinOut.flow.value then
    gameplay_drag_general.loadDragDataForMission(self.pinIn.filepath.value)
    self.pinOut.flow.value = true
  else
    self.pinOut.flow.value = false
  end
end

return _flowgraph_createNode(C)