-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Scatter Traffic'
C.description = 'Teleports all traffic vehicles away from the player or camera position.'
C.color = ui_flowgraph_editor.nodeColors.traffic
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'once_instant'
C.tags = {'traffic', 'ai', 'respawn', 'teleport'}

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'minDist', description = '(Optional) Minimum distance away from the player to teleport vehicles.' },
  { dir = 'in', type = 'number', name = 'maxDist', description = '(Optional) Maximum distance away from the player to teleport vehicles.' }
}

function C:workOnce()
  gameplay_traffic.scatterTraffic(nil, self.pinIn.minDist.value, self.pinIn.maxDist.value)
end

return _flowgraph_createNode(C)