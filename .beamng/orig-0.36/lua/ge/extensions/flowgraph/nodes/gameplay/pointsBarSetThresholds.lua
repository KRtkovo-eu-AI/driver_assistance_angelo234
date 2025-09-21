-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'PointsBar Thresholds'
C.description = 'Sets the thresholds for the points bar.'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui

C.tags = { 'Points', 'Bar', 'Thresholds' }
C.category = 'repeat_instant'
C.pinSchema = {
    { dir = 'in', type = 'number', name = 'bronze', description = "Bronze threshold for the points bar."},
    { dir = 'in', type = 'number', name = 'silver', description = "Silver threshold for the points bar."},
    { dir = 'in', type = 'number', name = 'gold', description = "Gold threshold for the points bar."},
}

function C:work()
  ui_apps_pointsBar.setThresholds({self.pinIn.bronze.value, self.pinIn.silver.value, self.pinIn.gold.value})
  self.mgr.modules.ui.pointsBarChanged = true
end

return _flowgraph_createNode(C)
