-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Camera Look at Position'
C.description = 'Sets the camera rotation to always look at the given position.'
C.category = 'repeat_instant'
C.color = ui_flowgraph_editor.nodeColors.camera
C.icon = ui_flowgraph_editor.nodeIcons.camera
C.pinSchema = {
  {dir = 'in', type = 'vec3', name = 'position', description = 'Target position.'}
}

local vecUp = vec3(0, 0, 1)
function C:work()
  if not commands.isFreeCamera() then
    commands.setFreeCamera()
  end

  local camPos = core_camera.getPosition()
  local camRot = quatFromDir(vec3(self.pinIn.position.value) - camPos, vecUp)
  core_camera.setPosRot(0, camPos.x, camPos.y, camPos.z, camRot.x, camRot.y, camRot.z, camRot.w)
end

return _flowgraph_createNode(C)