-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'StartScreen Text Panel'
C.color = ui_flowgraph_editor.nodeColors.ui
C.description = 'Assigns header and content text to the start screen.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'in', type = 'string', name = 'header', description = 'Header of the panel.', default="missions.missions.general.rules" },
  { dir = 'in', type = {'string', 'table'}, name = 'text', description = 'Contents of the panel.', default="Text" },
  { dir = 'in', type = 'bool', name = 'experimental', description = 'If this panel should have "experimental" visuals.', hidden=true},
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
    local pages = {
      main = true,
    }
    if self.pinIn.header.value == 'missions.missions.general.rules' or self.pinIn.header.value == 'Rules' then
      pages.main = false
      pages.rules = true
    end
    self.mgr.modules.ui:addUIElement({
      type = 'textPanel',
      header = self.pinIn.header.value,
      text = text,
      experimental = self.pinIn.experimental.value,
      pages = pages,
    })
  end
end

return _flowgraph_createNode(C)
