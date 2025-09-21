-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Boost Vehicle'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.description = 'Boost your vehicle.'
C.category = 'repeat_p_duration'

C.todo = "Dont know if this actually works. Invokes the core_booster extension."
C.obsolete = "Unknown working status; try the Apply Velocity node instead."
C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'Defines the id of the vehicle to boost.' },
  { dir = 'in', type = 'number', name = 'power', description = 'Defines the power of the boost.' }
}
C.legacyPins = {
  _in = {
    vehID = 'vehId'
  }
}

C.tags = {'boost', 'thrust'}

function C:init()
  self.loaded = false
end

function C:_executionStopped()
  self.loaded = false
end

function C:work()
  local veh
  if self.pinIn.vehId.value then
    veh = scenetree.findObjectById(self.pinIn.vehId.value)
  else
    veh = getPlayerVehicle(0)
  end

  if not self.loaded then
    veh:queueLuaCommand('extensions.load("core_booster")')
    self.loaded = true
  end
  veh:queueLuaCommand('core_booster.boost({'..(self.pinIn.power.value or 0) .. ',0,0},' .. (self.mgr.dtSim)..')')
end


return _flowgraph_createNode(C)
