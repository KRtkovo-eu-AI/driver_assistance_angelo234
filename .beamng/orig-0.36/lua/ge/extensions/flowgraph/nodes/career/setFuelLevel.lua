-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Set Fuel Level'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career

C.description = 'Sets the fuel level for a car. Uses the same percentage across all tanks.'
C.category = 'once_f_duration'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'The Id of the vehicle to be affected.' },
  { dir = 'in', type = 'number', name = 'mult', description = 'Fuel multiplcator (percentage of max fuel) 0-1' },
}
C.tags = {}


function C:init()
  self:onNodeReset()
end
function C:_executionStarted()
  self:onNodeReset()
end

function C:_executionStarted()

  self:onNodeReset()
end

function C:onNodeReset()
  self.done = false
  self.fuelData = nil
  self.receivedData = nil
  self.sentCommand = false
  self:setDurationState('inactive')
end

function C:workOnce()
  local veh
  if self.pinIn.vehId.value then
    veh = scenetree.findObjectById(self.pinIn.vehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  if not veh then return end
  core_vehicleBridge.requestValue(veh,function(ret) self.fuelData = ret[1] self.receivedData = true end, 'energyStorage')
  self:setDurationState('started')
end
function C:work()
  if self.receivedData then
    local veh
    if self.pinIn.vehId.value then
      veh = scenetree.findObjectById(self.pinIn.vehId.value)
    else
      veh = getPlayerVehicle(0)
    end
    local fuelType = nil
    local allowedTypes = {gasoline = true, diesel = true, electricEnergy = true}
    for _, tank in ipairs(self.fuelData) do
      core_vehicleBridge.executeAction(veh,'setEnergyStorageEnergy', tank.name, tank.maxEnergy * self.pinIn.mult.value)
    end
    self:setDurationState('finished')
  end
end

return _flowgraph_createNode(C)