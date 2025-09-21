-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Drift Bounds'

C.description = 'Uses a sites file to create the bounds for the drift mission.'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'string', name = 'sitesPath', description = "The path of the site file"},
}

C.tags = {'gameplay', 'utils', 'drift'}

function C:work()
  local file, valid = self.mgr:getRelativeAbsolutePath({self.pinIn.sitesPath.value}, true)
  if valid then
   gameplay_drift_bounds.setBounds(file)
  end
end

return _flowgraph_createNode(C)