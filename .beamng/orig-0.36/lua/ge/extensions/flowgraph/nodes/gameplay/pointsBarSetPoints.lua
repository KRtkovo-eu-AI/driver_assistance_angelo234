-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'PointsBar Points'
C.description = 'Sets the points for the points bar.'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui

C.tags = { 'Points', 'Bar', 'Set', 'set bar points'}
C.category = 'repeat_instant'
C.pinSchema = {
    { dir = 'in', type = 'number', name = 'points', description = "Points to set for the points bar. This will be used for how long the points bar will be filled."},
    { dir = 'in', type = 'string', name = 'pointsLabel', description = "Points label to set for the points bar. This will be the text displayed on the points bar."},
}

function C:work()
  ui_apps_pointsBar.setPoints(self.pinIn.points.value, self.pinIn.pointsLabel.value)
  self.mgr.modules.ui.pointsBarChanged = true
end

return _flowgraph_createNode(C)
