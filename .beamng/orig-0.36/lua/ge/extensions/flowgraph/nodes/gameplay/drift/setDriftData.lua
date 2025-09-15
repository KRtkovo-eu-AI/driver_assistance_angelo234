-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Set Drift Data'

C.description = 'Set the drift data such as stunt zones.'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'string', name = 'driftDataFile', description = "Drift data file"},
}

C.tags = {'gameplay', 'utils'}

function C:work()
  local file, valid = self.mgr:getRelativeAbsolutePath({self.pinIn.driftDataFile.value}, true)
  if valid then
   gameplay_drift_saveLoad.loadDriftData(file)
  end
end

return _flowgraph_createNode(C)