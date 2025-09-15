-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'AI Chase'
C.color = ui_flowgraph_editor.nodeColors.ai
C.icon = ui_flowgraph_editor.nodeIcons.ai
C.description = 'Sets a vehicle to chase another vehicle, with intentional contact enabled.'
C.category = 'once_p_duration'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'aiVehId', description = 'Vehicle id to apply AI mode: Chase.' },
  { dir = 'in', type = 'number', name = 'targetId', description = 'Vehicle id of the target vehicle that will be chased.' }
}

C.tags = {}

function C:workOnce()
  local veh
  if self.pinIn.aiVehId.value then
    veh = getObjectByID(self.pinIn.aiVehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  if not veh then return end

  local targetId = self.pinIn.targetId.value or be:getPlayerVehicleID(0)
  veh:queueLuaCommand('ai.setMode("chase")')
  veh:queueLuaCommand('ai.setTargetObjectID('..targetId..')')
end

return _flowgraph_createNode(C)
