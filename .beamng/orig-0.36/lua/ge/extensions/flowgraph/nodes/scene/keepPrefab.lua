-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Keep Prefab'
C.color = ui_flowgraph_editor.nodeColors.scene
C.icon = ui_flowgraph_editor.nodeIcons.scene

C.description = 'Marks a prefab to be kept after the flowgraph stops. By default, prefab spawned during runtime will be removed at the end.'
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'id', description = 'The Id of the prefab to be affected.' },
  { dir = 'in', type = 'bool', name = 'keep', description = 'If the prefab should be kept after stopping.' },
}

C.tags = {}

function C:init()

end

function C:workOnce()
  if self.pinIn.id.value then
    self.mgr.modules.prefab:setKeepPrefab(self.pinIn.id.value, self.pinIn.keep.value)
  end
  self.pinOut.flow.value = self.pinIn.flow.value
end


return _flowgraph_createNode(C)
