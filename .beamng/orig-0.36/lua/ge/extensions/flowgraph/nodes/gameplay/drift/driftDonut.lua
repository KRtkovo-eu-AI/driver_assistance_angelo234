-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Drift Donut Detected'

C.description = "Detects donut zones."
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', impulse = true, name = 'donutDrift', description = "Will fire when a donut is detected"},
  { dir = 'out', type = 'number', name = 'donutScore', description = "The score obtained from the donut"},
}

C.tags = {'gameplay', 'utils', 'drift'}

local donutInfo
function C:work()
  donutInfo = self.mgr.modules.drift:getCallBacks().donut

  self.pinOut.donutDrift.value = false
  if donutInfo.ttl > 0 then
    self.pinOut.donutDrift.value = true
    self.pinOut.donutScore.value = donutInfo.data.score
  end

end

return _flowgraph_createNode(C)