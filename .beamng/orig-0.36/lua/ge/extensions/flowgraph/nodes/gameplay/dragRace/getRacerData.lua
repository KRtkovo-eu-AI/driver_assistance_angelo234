-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Get Drag Racer Data'

C.description = 'get all the data for this vehId'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = "ID of the vehicle"},
  { dir = 'out', type = 'bool', name = 'isPlayable', description = ""},
  { dir = 'out', type = 'number', name = 'lane', description = ""},
  { dir = 'out', type = 'bool', name = 'isDesqualified', description = ""},
  { dir = 'out', type = 'string', name = 'desqualifiedReason', description = ""},
  { dir = 'out', type = 'bool', name = 'isFinished', description = ""},
  { dir = 'out', type = 'vec3', name = 'spawnPos', description = ""},
  { dir = 'out', type = 'quat', name = 'spawnRot', description = ""},
}

C.tags = {'gameplay', 'utils'}


function C:_executionStarted()
  self.data = {}
  self.dragData = {}
end

function C:work()
  self.data = gameplay_drag_general.getRacerData(self.pinIn.vehId.value)
  self.dragData = gameplay_drag_general.getData()

  if self.pinOut.isPlayable:isUsed() then
    self.pinOut.isPlayable.value = self.data.isPlayable
  end
  if self.pinOut.lane:isUsed() then
    self.pinOut.lane.value = self.data.lane
  end
  if self.pinOut.isDesqualified:isUsed() then
    self.pinOut.isDesqualified.value = self.data.isDesqualified
  end
  if self.pinOut.desqualifiedReason:isUsed() then
    self.pinOut.desqualifiedReason.value = self.data.desqualifiedReason
  end
  if self.pinOut.isFinished:isUsed() then
    self.pinOut.isFinished.value = self.data.isFinished
  end

  if self.pinOut.spawnPos:isUsed() then
    self.pinOut.spawnPos.value = self.dragData.strip.lanes[self.data.lane].waypoints.spawn.transform.pos:toTable()
  end

  if self.pinOut.spawnRot:isUsed() then
    self.pinOut.spawnRot.value = self.dragData.strip.lanes[self.data.lane].waypoints.spawn.transform.rot:toTable()
  end

end

return _flowgraph_createNode(C)