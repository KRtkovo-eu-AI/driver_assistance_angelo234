-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Get Mission Setup Data'
C.description = "Provides values stored in the mission setup modules."
C.category = 'provider'

C.pinSchema = {
  { dir = 'out', type = 'bool', name = 'useOriginalPlayerVeh', description = 'Is true if the previous player vehicle was selected as the mission vehicle.' },
  { dir = 'out', type = 'string', name = 'playerModel', hidden = true, description = 'The provided player vehicle model.' },
  { dir = 'out', type = 'string', name = 'playerConfig', hidden = true, description = 'The provided player vehicle config.' },
  { dir = 'out', type = 'string', name = 'playerConfigPath', hidden = true, description = 'The provided player vehicle full config path.' },
  { dir = 'out', type = 'bool', name = 'useTraffic', description = 'Is true if traffic is active for the mission.' },
  { dir = 'out', type = 'bool', name = 'usePrevTraffic', hidden = true, description = 'Is true if the traffic system is using pre-mission traffic.' },
  { dir = 'out', type = 'number', name = 'trafficAmount', description = 'The defined traffic amount.' },
  { dir = 'out', type = 'number', name = 'trafficActiveAmount', description = 'The defined traffic active amount.' },
  { dir = 'out', type = 'number', name = 'trafficRespawnRate', description = 'The defined traffic respawn rate.' }
}

function C:work()
  if not self.mgr.activity then return end

  local data = self.mgr.activity.setupModules
  if not data then return end

  -- default values are provided if the data is nil
  self.pinOut.useOriginalPlayerVeh.value = data.vehicles.usePlayerVehicle and true or false
  self.pinOut.playerModel.value = data.vehicles.playerModel or ''
  self.pinOut.playerConfig.value = data.vehicles.playerConfig or ''
  self.pinOut.playerConfigPath.value = data.vehicles.playerConfigPath or ''
  self.pinOut.useTraffic.value =  data.traffic.useTraffic and true or false
  self.pinOut.usePrevTraffic.value =  data.traffic.usePrevTraffic and true or false
  self.pinOut.trafficAmount.value = data.traffic.amount or 0
  self.pinOut.trafficActiveAmount.value = data.traffic.activeAmount or 0
  self.pinOut.trafficRespawnRate.value = data.traffic.respawnRate or 0
end

return _flowgraph_createNode(C)