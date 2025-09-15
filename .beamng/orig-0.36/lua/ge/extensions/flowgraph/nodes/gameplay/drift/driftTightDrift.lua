-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Drift Through Detected'

C.description = "Detects tight drift zones."
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', impulse = true, name = 'tightDrift', description = "Will fire when a tight drift is detected"},
  { dir = 'out', type = 'number', name = 'tightDriftScore', description = "The score obtained from the tight drift"},
}

C.tags = {'gameplay', 'utils', 'drift'}

local tightInfo
function C:work()
  tightInfo = self.mgr.modules.drift:getCallBacks().tight

  self.pinOut.tightDrift.value = false
  if tightInfo.ttl > 0 then
    self.pinOut.tightDrift.value = true
    self.pinOut.tightDriftScore.value = tightInfo.data.score
  end
end

return _flowgraph_createNode(C)