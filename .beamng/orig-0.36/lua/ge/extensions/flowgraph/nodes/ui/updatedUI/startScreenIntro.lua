-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'StartScreen Intro'
C.color = ui_flowgraph_editor.nodeColors.ui
C.description = 'Combo node for intro, objectives, and ratings.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'in', type = {'string', 'table'}, name = 'text', description = 'Contents of the panel.', default="Text" },
  { dir = 'in', type = 'bool', name = 'includeObjectives', description = 'If true, automatically adds objectives after the panel.' },
  { dir = 'in', type = 'bool', name = 'includeRatings', description = 'If true, automatically adds ratings after the panel.' },
  { dir = 'out', type = 'flow', name = 'flow', description = '', chainFlow = true },
}

C.tags = { 'start', 'screen', 'intro', 'ui' }

function C:work()
  self.pinOut.flow.value = self.pinIn.flow.value
  local text = self.pinIn.text.value
  if (type(text) == 'table' and text.txt == '') or text == '' then
    text = nil
  end
  if text then
    self.mgr.modules.ui:addUIElement({type = 'textPanel', header = "Intro", text = text, pages = {
      main = true,
    }})
  end

  if self.pinIn.includeObjectives.value then
    self.mgr.modules.ui:addObjectives()
  end
  if self.pinIn.includeRatings.value then
    self.mgr.modules.ui:addRatings()
  end
end

return _flowgraph_createNode(C)
