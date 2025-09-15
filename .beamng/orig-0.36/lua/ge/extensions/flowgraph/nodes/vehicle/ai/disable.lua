-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'AI Disable'
C.color = ui_flowgraph_editor.nodeColors.ai
C.icon = ui_flowgraph_editor.nodeIcons.ai
C.description = 'Disables AI mode for a vehicle.'
C.category = 'repeat_p_duration'
C.pinSchema = {
  { dir = 'in', type = 'number', name = 'aiVehId', description = 'Vehicle id to disable AI mode for.' }
}

C.tags = {}

function C:init()
  self.data.useScriptStop = false
  self.data.handBrakeWhenFinished = false
  self.data.straightenWheelsWhenFinished = false
end

function C:work()
  local veh
  if self.pinIn.aiVehId.value then
    veh = getObjectByID(self.pinIn.aiVehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  if not veh then return end

  if self.data.useScriptStop then
    veh:queueLuaCommand('ai:scriptStop('..tostring(self.data.handBrakeWhenFinished)..','..tostring(self.data.straightenWheelsWhenFinished)..')')
  else
    veh:queueLuaCommand('ai.setState({mode = "disabled"})')
  end
end


return _flowgraph_createNode(C)
