-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'On Custom Tow'
C.description = 'Detects when the player uses a custom towing action in the career tutorial'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'tow', description = "Outflow the player is being towed. Make sure to fade from black after you do something", impulse = true },
}
C.dependencies = {}


function C:init()
  self.flags = {}
end

function C:work(args)
  self.pinOut.tow.value = false
  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end
  table.clear(self.flags)
end
--[[
function C:_executionStarted()
  if career_career.isActive() and career_modules_linearTutorial then
    career_modules_linearTutorial.setTutorialFlag("customTowHookEnabled", true)
  end
end

function C:_executionStopped()
  if career_career.isActive() and career_modules_linearTutorial then
    career_modules_linearTutorial.setTutorialFlag("customTowHookEnabled", false)
  end
end
]]

function C:onCareerCustomTowHook(poiId)
  self.flags.tow = true
end

return _flowgraph_createNode(C)
