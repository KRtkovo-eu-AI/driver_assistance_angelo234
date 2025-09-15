-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Get Gamestate'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.description = "Gets the ui state"
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'string', name = 'state', description = 'State' },
}

C.tags = {'string','util'}

function C:init()
end

function C:work()
  self.pinOut.state.value = core_gamestate.state.state
end

return _flowgraph_createNode(C)
