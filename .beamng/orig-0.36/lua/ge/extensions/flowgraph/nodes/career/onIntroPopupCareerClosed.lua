-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'On introPopupCareerClosed'
C.description = 'Detects when the intropopup is closed..'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'closed', description = "Outflow once when the intropopup is closed", impulse = true },
}
C.dependencies = {}


function C:init()
  self.flags = {}
end


function C:work(args)
  self.pinOut.closed.value = false
  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end
  table.clear(self.flags)
end

function C:onIntroPopupCareerClosed(data)
  self.flags.closed = true
end


function C:_afterTrigger()
  table.clear(self.flags)
end





return _flowgraph_createNode(C)
