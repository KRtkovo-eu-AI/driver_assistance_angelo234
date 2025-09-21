-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Start Screen Crash Analysis Step Details'
C.color = ui_flowgraph_editor.nodeColors.ui
C.description = 'Creates a crash step details panel for the simple start screen'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'in', type = 'table', name = 'stepScoreData', description = 'Step score data'},
  { dir = 'out', type = 'flow', name = 'flow', description = '', chainFlow = true },
}

C.tags = { 'start', 'finish', 'screen', 'outro', 'ui', "crash", "analysis", "step", "details" }

function C:work()
  self.pinOut.flow.value = self.pinIn.flow.value
  self.mgr.modules.ui:addCrashAnalysisStepDetails(
    {
      stepScoreData = self.pinIn.stepScoreData.value
    })
end

return _flowgraph_createNode(C)
