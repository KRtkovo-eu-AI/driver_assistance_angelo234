-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Set Tutorial Step'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career

C.description = 'Completes the current Linear Tutorial Step and starts a new one.'
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'nextStep', hidden=true, description = 'The next step. If none provided, will use one more than the previous one.' },
}
C.tags = {}

function C:init()

end

function C:workOnce()
  if career_modules_linearTutorial then
    career_modules_linearTutorial.completeLinearStep(self.pinIn.nextStep.value)
  else
    self:__setNodeError('work', "Career module career_modules_linearTutorial not loaded!")
  end
  self.pinOut.flow.value = self.pinIn.flow.value
end

return _flowgraph_createNode(C)