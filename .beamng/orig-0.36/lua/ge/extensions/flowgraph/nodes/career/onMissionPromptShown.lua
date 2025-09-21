-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'On Poi Prompt Shown'
C.description = 'Detects when the player stopped at a Poi and the prompt to view missions, dealership etc is shown.'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'shownPrompt', description = "Outflow once when the prompt is shown. This also includes the garage and refueling prompts.", impulse = true },
  { dir = 'out', type = 'flow', name = 'openedMenu', description = "Outflow once when the prompt is shown.", impulse = true },
}
C.dependencies = {}


function C:init()
  self.flags = {}
end


function C:work(args)
  self.pinOut.shownPrompt.value = false
  self.pinOut.openedMenu.value = false
  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end
  table.clear(self.flags)
end

function C:onActivityAcceptUpdate(data)
  self.flags.shownPrompt = true
end

function C:onAvailableMissionsSentToUi(data)
  self.flags.openedMenu = true
end



return _flowgraph_createNode(C)
