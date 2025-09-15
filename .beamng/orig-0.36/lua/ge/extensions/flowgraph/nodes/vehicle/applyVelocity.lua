-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Apply Velocity to Vehicle'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.description = 'Instantly sets a velocity for a vehicle.'
C.category = 'repeat_p_duration'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'Vehicle id; if not set, uses the player vehicle.' },
  { dir = 'in', type = 'vec3', name = 'dirVec', description = 'Direction vector; if not set, uses the current vehicle direction vector.' },
  { dir = 'in', type = 'number', name = 'coefficient', default = 1, description = 'Direction vector multiplier.' }
}
-- hint: to prevent triggering emergency systems of some vehicles, raise the coefficient gradually over time.

C.tags = {'boost', 'thrust', 'move'}

function C:work()
  local veh = self.pinIn.vehId.value and scenetree.findObjectById(self.pinIn.vehId.value) or getPlayerVehicle(0)
  if not veh then return end

  local dirVec = self.pinIn.dirVec.value and vec3(self.pinIn.dirVec.value) or veh:getDirectionVector()
  dirVec:setScaled(self.pinIn.coefficient.value or 1)
  veh:queueLuaCommand("thrusters.applyVelocity("..serialize(dirVec)..")")
end

return _flowgraph_createNode(C)
