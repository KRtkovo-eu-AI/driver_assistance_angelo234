-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Get Tutorial Flag'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career

C.description = 'Gets a specific Tutorial Flag'
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'string', name = 'name', description = 'The name of the flag.' },
  { dir = 'out', type = 'bool', name = 'value', description = 'If the flag is set or not'},
}
C.tags = {}

function C:init()

end

function C:workOnce()
  self.pinOut.value.value = nil
  if career_modules_linearTutorial then
    self.pinOut.value.value = career_modules_linearTutorial.getTutorialFlag(self.pinIn.name.value)
  else
    self:__setNodeError('work', "Career module career_modules_linearTutorial not loaded!")
  end
  self.pinOut.flow.value = self.pinIn.flow.value
end

return _flowgraph_createNode(C)