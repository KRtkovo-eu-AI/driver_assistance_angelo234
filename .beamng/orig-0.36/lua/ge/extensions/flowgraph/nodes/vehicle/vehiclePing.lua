-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Vehicle Ping'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle

C.description = 'Pings the vehicle - this can be used to make sure that previously sent instructions to vehicles have arrived before continuing..'
C.category = 'once_f_duration'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'Id of the vehicle to ping.' },
}

C.tags = {}

function C:init()
  self:onNodeReset()
end
function C:_executionStarted()
  self:onNodeReset()
end

function C:onNodeReset()
  self.receivedInfo = nil
  self:setDurationState('inactive')
end

function C:workOnce()
  local veh
  if self.pinIn.vehId.value then
    veh = scenetree.findObjectById(self.pinIn.vehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  if veh then
    core_vehicleBridge.requestValue(veh, function(val) self.receivedInfo = true end,'ping')
    self:setDurationState('started')
  end
end

function C:work()
  if self.receivedInfo then
    self.receivedInfo = nil
    self:setDurationState('finished')
  end
end




return _flowgraph_createNode(C)
