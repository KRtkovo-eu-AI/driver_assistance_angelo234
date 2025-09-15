-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Flip Upright'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle

C.description = "Flips the vehicle upright without repairing it."
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'Defines the id of the vehicle to move.' },
}
C.legacyPins = {
}
C.tags = {'rotation', 'position', 'recover', 'move'}

function C:init()
  --self.data.useWheelCenter = false
end

function C:work()
  self:resetVehicle()
  self.pinOut.flow.value = self.pinIn.flow.value
end

function C:resetVehicle()
  local veh
  if self.pinIn.vehId.value then
    veh = scenetree.findObjectById(self.pinIn.vehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  if not veh then
    return
  end
  if veh then
    spawn.safeTeleport(veh, veh:getPosition(), quatFromDir(veh:getDirectionVector()), nil, nil, nil, nil, false )
  end
end

return _flowgraph_createNode(C)
