-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Drift Race Path'

C.description = 'Uses a race path to set checkpoints, destination, starting pos ...'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'string', name = 'racePath', description = "Race path"},
  { dir = 'in', type = 'bool', name = 'disableWrongWayAndDist', description = "Disabled the wrong way message and the distance remaining message"},
  {dir = 'out', type = 'table', name = 'pathData', tableType = 'pathData', description = 'Data from the path for other nodes to process.'},
  {dir = 'out', type = 'table', name = 'waypoints', description = "Waypoints from the race files", tableType = 'generic'},
}

C.tags = {'gameplay', 'utils', 'drift'}

function C:work()
  local file, valid = self.mgr:getRelativeAbsolutePath({self.pinIn.racePath.value}, true)
  if valid then
   gameplay_drift_destination.setRacePath({filePath = file, disableWrongWayAndDist = self.pinIn.disableWrongWayAndDist.value})
   self.pinOut.waypoints.value = gameplay_drift_destination.getWaypoints()
   self.pinOut.pathData.value = gameplay_drift_destination.getPathData(file)
  end
end

return _flowgraph_createNode(C)