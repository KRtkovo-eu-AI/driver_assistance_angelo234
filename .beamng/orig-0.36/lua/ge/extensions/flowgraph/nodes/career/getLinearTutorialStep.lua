-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Get Tutorial Step'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career

C.description = 'Gets the current Linear Tutorial Step'
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'out', type = 'number', name = 'step', hidden=true, description = 'The step of the tutorial.' },
}
C.tags = {}

function C:init()

end

function C:workOnce()
  if career_modules_linearTutorial then
    self.pinOut.step.value = career_modules_linearTutorial.getLinearStep()
  else
    self:__setNodeError('work', "Career module career_modules_linearTutorial not loaded!")
  end
  self.pinOut.flow.value = self.pinIn.flow.value
end

return _flowgraph_createNode(C)