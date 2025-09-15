-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Screen Finish'
C.color = ui_flowgraph_editor.nodeColors.ui
C.description = 'Finishes the screen setup and sends it to UI.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'out', type = 'flow', name = 'flow', description = '' },
}

C.tags = { 'start', 'end', 'finish', 'screen', 'intro', 'outro', 'ui' }

function C:work()
  self.mgr.modules.ui:finishUIBuilding()
end

return _flowgraph_createNode(C)
