-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Shift to Gear Index'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle

C.description = 'Shifts to a specific gear for vehicles with a manual gearbox. Undefined behaviour for vehicle using automatic gearbox or using a gear that is not within range.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'Id of the vehicle to affect.' },
  { dir = 'in', type = 'number', name = 'gear', description = 'Gear index to shift. 0 = Neutral, -1 = Reverse, 1 = First Gear etc.' },
}

C.tags = {}


function C:work()
  local veh
  if self.pinIn.vehId.value then
    veh = scenetree.findObjectById(self.pinIn.vehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  if veh then
    core_vehicleBridge.executeAction(veh,'shiftToGearIndex', self.pinIn.gear.value)
  end
end


return _flowgraph_createNode(C)
