-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'On Mission Start/Stop'
C.description = 'Detects when the player started or stopped a mission.'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'started', description = "Outflow once when a mission is started.", impulse = true },
  { dir = 'out', type = 'flow', name = 'stopped', description = "Outflow once when a mission is stopped (from the flowgraph.", impulse = true },
  --{ dir = 'out', type = 'flow', name = 'abandoned', description = "Outflow once when a mission is stopped (from the flowgraph.", impulse = true },
}
C.dependencies = {}

function C:init()
  self.flags = {}
end


function C:work(args)
  self.pinOut.started.value = false
  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end
  table.clear(self.flags)
end

function C:onMissionStartWithFade(data)
  self.flags.started = true
end

function C:onAnyMissionChanged(newState, mission)
  if newState == "stopped" then
    self.flags.stopped = true
  end
end

return _flowgraph_createNode(C)
