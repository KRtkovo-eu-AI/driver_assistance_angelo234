-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

C.name = 'Remove Prefab'
C.color = ui_flowgraph_editor.nodeColors.scene
C.icon = ui_flowgraph_editor.nodeIcons.svene
C.description = 'Removes a prefab.'
C.category = 'repeat_instant'

C.pinSchema = {
  {dir = 'in', type = 'flow', name = 'flow', description = "Inflow for this node."},
  { dir = 'in', type = 'number', name = 'id', default = 0, description = "Prefab ID. If not present" },
}



function C:work()
  self.mgr.modules.prefab:deletePrefab(self.pinIn.id.value, true)
end

function C:_executionStopped()

end


return _flowgraph_createNode(C)