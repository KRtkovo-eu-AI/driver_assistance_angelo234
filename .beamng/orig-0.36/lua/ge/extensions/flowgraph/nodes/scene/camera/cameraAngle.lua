-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Camera Transform'
C.description = 'Gives information about the camera transform'
C.category = 'repeat_instant'
C.color = ui_flowgraph_editor.nodeColors.camera
C.icon = ui_flowgraph_editor.nodeIcons.camera
C.pinSchema = {
    { dir = 'out', type = 'vec3', name = 'pos', description = 'Position of the camera.' },
    { dir = 'out', type = 'quat', name = 'rot', description = 'Rotation of the camera.' },
    { dir = 'out', type = 'number', name = 'fov', description = 'Field of view of the camera.' }
}

C.tags = {}

function C:init()

end

function C:work()
  self.pinOut.fov.value = core_camera.getFovDeg()
  if self.pinOut.pos:isUsed() then
    self.pinOut.pos.value = core_camera.getPosition():toTable()
  end
  if self.pinOut.rot:isUsed() then
    self.pinOut.rot.value = core_camera.getQuat():toTable()
  end
end

return _flowgraph_createNode(C)
