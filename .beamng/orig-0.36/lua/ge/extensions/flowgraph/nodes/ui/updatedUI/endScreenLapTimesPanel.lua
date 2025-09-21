-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'EndScreen Lap Times'
C.color = ui_flowgraph_editor.nodeColors.ui
C.description = 'Displays the vehicle lap times in the end screen.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'out', type = 'flow', name = 'flow', description = '', chainFlow = true },

  { dir = 'in', type = 'table', name = 'race', description = 'Race Data.'},
  { dir = 'in', type = 'number', name = 'vehId', description = 'Vehicle id to use.'},
}

C.tags = { 'end', 'finish', 'screen', 'outro', 'ui' }

function C:work()
  self.pinOut.flow.value = self.pinIn.flow.value
  local veh
  if self.pinIn.vehId.value then
    veh = scenetree.findObjectById(self.pinIn.vehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  if veh and self.pinIn.race.value then
    local state = self.pinIn.race.value.states[veh:getID()]
    self.mgr.modules.ui:addLaptimesForVehicle(state)
  end
end

return _flowgraph_createNode(C)
