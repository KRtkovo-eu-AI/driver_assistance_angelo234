-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

C.name = 'Setup Pursuit Gameplay'
C.description = 'Uses the traffic system to create pursuit gameplay, with a suspect and police vehicles.'
C.color = ui_flowgraph_editor.nodeColors.traffic
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'once_instant'
C.tags = {'police', 'cops', 'pursuit', 'chase', 'traffic', 'ai'}

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'suspectId', description = 'Suspect vehicle id.' },
  { dir = 'in', type = 'number', hidden = true, name = 'mainPoliceId', description = '(Optional) Use this to provide one police vehicle id.' },
  { dir = 'in', type = 'table', tableType = 'vehicleIds', name = 'policeIds', description = 'Array of police vehicle ids; if none given, attempts to use existing police vehicles.' },
  { dir = 'in', type = 'number', name = 'pursuitMode', description = 'Initial pursuit mode (from 1 to 3)' },
  { dir = 'in', type = 'bool', name = 'preventAutoStart', description = 'Set this to true if you want to manually control the pursuit activation.' },

  { dir = 'out', type = 'flow', name = 'success', description = 'Flows if setup was successful.' },
  { dir = 'out', type = 'flow', name = 'fail', description = 'Flows if setup failed for some reason.' }
}

function C:workOnce()
  local suspectId = self.pinIn.suspectId.value or be:getPlayerVehicleID(0)
  local policeIds = self.pinIn.mainPoliceId.value and {self.pinIn.mainPoliceId.value} or self.pinIn.policeIds.value
  local options = {pursuitMode = self.pinIn.pursuitMode.value, preventAutoStart = self.pinIn.preventAutoStart.value}
  self.pinOut.success.value = gameplay_police.setupPursuitGameplay(suspectId, policeIds, options)
  self.pinOut.fail.value = not self.pinOut.success.value

  if self.pinOut.success.value then -- prevents the scatter traffic function from teleporting these vehicles
    local traffic = gameplay_traffic.getTrafficData()
    traffic[suspectId].ignoreForceTeleport = true
    for _, id in ipairs(policeIds) do
      traffic[id].ignoreForceTeleport = true
    end
  end
end

return _flowgraph_createNode(C)