-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

C.name = 'Track Prefab'
C.color = ui_flowgraph_editor.nodeColors.scene
C.icon = ui_flowgraph_editor.nodeIcons.svene
C.description = 'Allow Flowgraph to track the Prefab spawned outside Flowgraph.'
C.category = 'once_instant'

C.pinSchema = {
  {dir = 'in', type = 'flow', name = 'flow', description = "Inflow for this node."},
  { dir = 'in', type = 'number', name = 'id', default = 0, description = "Prefab ID. If not present, player Prefab will be used." },
  { dir = 'in', type = 'bool', name = 'dontDelete', hidden = true, default = true, description = 'If true, the Prefab will not be deleted when you stop the project.'}
}

function C:workOnce()
  self.mgr.modules.prefab:addPrefab(self.pinIn.id.value, {dontDelete = self.pinIn.dontDelete.value, skipNavgraphReload = true, skipCollisionReload = true})
end

function C:_executionStopped()

end

return _flowgraph_createNode(C)