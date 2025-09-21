-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'EndScreen Results'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.description = "Shows the mission results on the end screen."
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'out', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'in', type = {'string','table'},  name = 'text', description = 'Subtext of the menu.' },
  { dir = 'in', type = 'table', name = 'change', description = 'Change from the attempt. Use the aggregate attempt node.' },
  { dir = 'in', type = 'bool', name = 'includeObjectives', description = 'if true, automatically adds objectives after the panel.' },
  { dir = 'in', type = 'bool', name = 'includeRatings', description = 'if true, automatically adds ratings after the panel.' },
}

C.tags = { 'end', 'finish', 'screen', 'outro', 'ui' }

function C:init()
end

function C:work()
  self.pinOut.flow.value = self.pinIn.flow.value
  local header = "Results"
  local text = self.pinIn.text.value
  if (type(text) == 'table' and text.txt == '') or text == '' then
    text = nil
  end
  if self.pinIn.change.value and self.pinIn.change.value.formattedAttempt  then
    self.mgr.modules.ui:addUIElement({
      type = 'textPanel',
      header = header,
      attempt = self.pinIn.change.value.formattedAttempt,
      pages = {
        main = true,
      }})
    header = nil
  end
  if self.pinIn.text.value then
    self.mgr.modules.ui:addUIElement({
      type = 'textPanel',
      header = header,
      text = self.pinIn.text.value,
      pages = {
        main = true,
    }})
    header = nil
  end
  if self.pinIn.includeObjectives.value then
    self.mgr.modules.ui:addObjectives(self.pinIn.change.value)
  end
  if self.pinIn.includeRatings.value then
    self.mgr.modules.ui:addRatings(self.pinIn.change.value)
  end
end

return _flowgraph_createNode(C)
