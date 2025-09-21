-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Get Electrics Value'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle

C.description = 'Gets an Electrics value. If no ID is given, the current player vehicle is used.'
C.category = 'once_f_duration'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'Id of the vehicle to affect.' },
  { dir = 'in', type = 'string', name = 'key', description = 'Key for the Electrics value' },
  { dir = 'out', type = 'any', name = 'value', description = 'The returned value.' },
}

C.tags = {}

function C:init()
  self:onNodeReset()
end
function C:postInit()
  self.pinInLocal.key.hardTemplates = {
    {value = 'throttle'},
    {value = 'throttle_input'},
    {value = 'rpm'},
    {value = 'brake'},
    {value = 'brake_input'},
    {value = 'parkingbrake'},
    {value = 'parkingbrake_input'},
    {value = 'gear'},
    {value = 'clutch'},
  }
end
function C:_executionStarted()
  self:onNodeReset()
end
function C:_executionStopped()
  self:onNodeReset()
end

function C:clearNotification()
  --dump(self._setupData)
  if self._setupData then
    local veh = scenetree.findObjectById(self._setupData.vehId)
    if veh then
      core_vehicleBridge.unregisterValueChangeNotification(veh, self._setupData.key)
    end
  end
  self._setupData = nil
end

function C:onNodeReset()
  self:setDurationState('inactive')
  self:clearNotification()
end

function C:workOnce()
  local veh
  if self.pinIn.vehId.value then
    veh = scenetree.findObjectById(self.pinIn.vehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  if veh then
    core_vehicleBridge.registerValueChangeNotification(veh, self.pinIn.key.value)
    self._setupData = {vehId = veh:getId(), key = self.pinIn.key.value}
    self:setDurationState('started')
  end
end

function C:work()
  if self._setupData then
    local val = core_vehicleBridge.getCachedVehicleData(self._setupData.vehId, self._setupData.key)
    if val ~= nil then
      self.pinOut.value.value = val
      self:setDurationState('finished')
    end
  end
end




return _flowgraph_createNode(C)
