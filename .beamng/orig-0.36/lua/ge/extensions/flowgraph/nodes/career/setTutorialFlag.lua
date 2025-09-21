-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Set Tutorial Flag'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career

C.description = 'Sets a specific Tutorial Flag to be set or not set.'
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'string', name = 'name', description = 'The name of the flag.' },
  { dir = 'in', type = 'bool', name = 'value', description = 'If the flag should be set or not', hardcoded = true },
}
C.tags = {}

function C:init()

end

function C:workOnce()
  if career_modules_linearTutorial then
    career_modules_linearTutorial.setTutorialFlag(self.pinIn.name.value, self.pinIn.value.value or false)
  else
    self:__setNodeError('work', "Career module career_modules_linearTutorial not loaded!")
  end
  self.pinOut.flow.value = self.pinIn.flow.value
end

return _flowgraph_createNode(C)