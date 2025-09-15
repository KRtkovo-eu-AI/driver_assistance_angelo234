-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Stop Auto Replay'
C.description = 'Will stop the current recording if currently recording'

C.pinSchema = {
  {dir = 'in', type = 'flow', impulse = true, name = 'stopRec', description = 'Will stop the current recording if currently recording'},
  {dir = 'in', type = 'bool', name = 'attemptNotCompleted', description = 'Needs to be true if the mission attempt is not finished'},
}

C.tags = {}

function C:work()
  if self.pinIn.attemptNotCompleted.value then
    self.mgr.modules.missionReplay:saveMetaInfoWithoutAttempt()
  end

  self.mgr.modules.missionReplay:stopIfRec()
end


return _flowgraph_createNode(C)
