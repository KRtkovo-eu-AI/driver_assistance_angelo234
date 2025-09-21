-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Set Drift Challenge Mode'

C.description = 'Sets the drift challenge mode.'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'string', name = 'driftChallengeMode', description = "A to B/A to B with stunt zone/Gymkhana"},
}

C.tags = {'gameplay', 'utils'}

function C:work()
  gameplay_drift_general.setChallengeMode(self.pinIn.driftChallengeMode.value)
end

return _flowgraph_createNode(C)