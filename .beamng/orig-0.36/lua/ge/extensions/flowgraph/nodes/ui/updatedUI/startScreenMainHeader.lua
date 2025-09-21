-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Screen Main Header'
C.color = ui_flowgraph_editor.nodeColors.ui
C.description = 'Used for a large header, usually the mission name.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'in', type = 'string', name = 'header', description = 'Header of the panel, like the mission name', default="Header", },
  { dir = 'in', type = 'string', name = 'preHeader', description = 'Preheader, like the mission type', default="Header", },
  { dir = 'out', type = 'flow', name = 'flow', description = '', chainFlow = true },
}

C.tags = { 'start', 'screen', 'intro', 'ui' }

function C:work()
  self.pinOut.flow.value = self.pinIn.flow.value
  self.mgr.modules.ui:addHeader({header = self.pinIn.header.value, preHeader = self.pinIn.preHeader.value})
end

return _flowgraph_createNode(C)
