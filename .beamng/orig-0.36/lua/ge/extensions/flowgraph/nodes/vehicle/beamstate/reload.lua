-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Reload Vehicle'
C.description = "Reloads a vehicle. Uses player vehicle if no ID is given."
C.category = 'repeat_p_duration'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'Defines the id of the vehicle to reload.' },
}
C.legacyPins = {
  _in = {
    vehID = 'vehId'
  }
}
C.tags = {'reset'}

function C:work()
  local veh
  if self.pinIn.vehId.value then
    veh = scenetree.findObjectById(self.pinIn.vehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  if not veh then
    return
  end
  veh:reload()
end

return _flowgraph_createNode(C)
