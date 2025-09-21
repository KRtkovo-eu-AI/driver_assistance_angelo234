-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'EndScreen Drift Stats'
C.color = ui_flowgraph_editor.nodeColors.ui
C.description = 'Creates a drift stats panel for the end screen.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'out', type = 'flow', name = 'flow', description = '', chainFlow = true },
}

C.tags = { 'end', 'finish', 'screen', 'outro', 'ui' }

function C:work()
  self.pinOut.flow.value = self.pinIn.flow.value
  self.mgr.modules.ui:addDriftStats(
    {
      perfStats = gameplay_drift_scoreboard.getPerformanceStats(),
      tiersStats = gameplay_drift_scoreboard.getTiersStats(),
      driftEvents = gameplay_drift_scoreboard.getDriftEventStats(),
    })
end

return _flowgraph_createNode(C)
