-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Start Auto Replay'
C.description = 'Will stop the current recording and start a new one'

C.pinSchema = {
  {dir = 'in', type = 'flow', impulse = true, name = 'startNewRec', description = 'Will stop the current recording and start a new one'},
}

C.tags = {}

function C:work()
  self.mgr.modules.missionReplay:startNewRec()
end


return _flowgraph_createNode(C)
