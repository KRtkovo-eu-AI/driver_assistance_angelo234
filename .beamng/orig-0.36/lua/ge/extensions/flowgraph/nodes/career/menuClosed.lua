-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Menu Closed'
C.description = 'Util Helper for Garage tutorial'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'menuClosed', description = "Outflow once if any menu was closed and the player is back in playstate", impulse = true },
}
C.dependencies = {}

function C:init()
  self.flags = {}
end

function C:work(args)
  self.pinOut.menuClosed.value = false
  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end
  table.clear(self.flags)
end

function C:onUIPlayStateChanged(enteredPlay)
  if enteredPlay then
    self.flags.menuClosed = true
  end
end


return _flowgraph_createNode(C)