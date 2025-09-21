-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Mission Cleanup'
C.color = im.ImVec4(0.13, 0.3, 0.64, 0.75)
C.description = "Cleans up the world state before a mission, with parameters that override the mission manager processing."
C.category = 'once_p_duration'

C.pinSchema = {
  {dir = 'in', type = 'bool', name = 'keepPlayer', description = '(Optional) If true, the player vehicle will stay active and will not be stashed.'},
  {dir = 'in', type = 'bool', name = 'keepTraffic', description = '(Optional) If true, the current traffic will stay active and will not be stashed.'},
  {dir = 'out', type = 'number', name = 'vehId', description = 'Original player vehicle id.'}
}
C.tags = { 'activity' }

function C:workOnce()
  self.mgr.modules.mission:processVehicles({keepPlayer = self.pinIn.keepPlayer.value, keepTraffic = self.pinIn.keepTraffic.value})
  self.pinOut.vehId.value = self.mgr.modules.mission:getOriginalPlayerId() or be:getPlayerVehicleID(0)
end

return _flowgraph_createNode(C)