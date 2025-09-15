-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'On Bigmap State Change'
C.description = 'Detects then the map screen is opened, closed.'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
    { dir = 'out', type = 'flow', name = 'enteredStart', description = "Outflow once when bigmap is entered, at start of transition.", impulse = true },
    { dir = 'out', type = 'flow', name = 'enteredEnd', description = "Outflow once when bigmap is entered, at end of transition (or instant).", impulse = true },
    { dir = 'out', type = 'flow', name = 'exitedStart', description = "Outflow once when bigmap is exited, at start of transition.", impulse = true },
    { dir = 'out', type = 'flow', name = 'exitedEnd', description = "Outflow once when bigmap is exited, at end of transition (or instant)", impulse = true },
}
C.dependencies = {}


function C:init()
  self.flags = {}
end


function C:work(args)
  self.pinOut.enteredStart.value = false
  self.pinOut.enteredEnd.value = false
  self.pinOut.exitedStart.value = false
  self.pinOut.exitedEnd.value = false
  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end
  table.clear(self.flags)
end

function C:onBigmapStartTransition(activate, esc)
  self.flags.enteredStart = activate
  self.flags.exitedStart = not activate
end

function C:onActivateBigMapCallback()
  self.flags.enteredEnd = true
end
function C:onDeactivateBigMapCallback()
  self.flags.exitedEnd = true
end


return _flowgraph_createNode(C)
