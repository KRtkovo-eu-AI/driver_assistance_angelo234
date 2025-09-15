-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'AI Stop'
C.color = ui_flowgraph_editor.nodeColors.ai
C.icon = ui_flowgraph_editor.nodeIcons.ai
C.description = 'Lets the AI stop the vehicle.'
C.category = 'once_p_duration'
C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'stopped', description = 'Outflow when the vehicle is stopped.' },
  { dir = 'in', type = 'number', name = 'aiVehId', description = 'Vehicle id to stop.' },
  { dir = 'in', type = 'number', name = 'checkVelocity', hidden = true, default = 0.01, hardcoded = true, description = 'If given, vehicle has to be slower than this to be considered stopped.' }
}

C.tags = {'halt', 'disable'}

function C:init()
  self.complete = false
end

function C:onNodeReset()
  self.complete = false
end

function C:_executionStarted()
  self:onNodeReset()
end

function C:getVeh()
  local veh
  if self.pinIn.aiVehId.value then
    veh = getObjectByID(self.pinIn.aiVehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  return veh
end

function C:workOnce()
  local veh = self:getVeh()
  if not veh then return end

  veh:queueLuaCommand('ai.setState({mode = "stop"})')
end

function C:work()
  if self.complete then
    self.pinOut.stopped.value = true
    return
  end

  local vehId = self.pinIn.aiVehId.value or be:getPlayerVehicleID(0)
  local vData = map.objects[vehId]
  if vData then
    self.complete = vData.vel:length() < (self.pinIn.checkVelocity.value or 0.01)
  end
end

return _flowgraph_createNode(C)
