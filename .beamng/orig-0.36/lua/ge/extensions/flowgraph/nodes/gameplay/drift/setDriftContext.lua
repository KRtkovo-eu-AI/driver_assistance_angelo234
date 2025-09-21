-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Set Drift Context'

C.description = 'A drift context sets a certain amount of drift rules/display.'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'string', name = 'driftContext', description = "stopped/inChallenge/inFreeroam"},
}

C.tags = {'gameplay', 'utils'}

function C:work()
  gameplay_drift_general.setContext(self.pinIn.driftContext.value)
end

return _flowgraph_createNode(C)