-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Get Drag Race Winner Data'

C.description = 'Get a list of the winners'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'bool', name = 'playerWin', description = ""},
  { dir = 'out', type = 'number', name = 'playerPos', description = ""},
}

C.tags = {'gameplay', 'utils'}


function C:_executionStarted()
  self.data = {}
end

function C:work()
  self.data = gameplay_drag_general.getWinnersData()
  if self.pinOut.playerWin:isUsed() then
    self.pinOut.playerWin.value =  self.data[1].isPlayable
  end
  if self.pinOut.playerPos:isUsed() then
    for k, v in pairs(self.data) do
      if v.isPlayable then
        self.pinOut.playerPos.value = k
        break
      end
    end
  end
end

return _flowgraph_createNode(C)