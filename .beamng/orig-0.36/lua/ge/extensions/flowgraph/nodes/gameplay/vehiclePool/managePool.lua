-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Manage Vehicle Pool'
C.description = 'Sets and gets properties of a vehicle pool object.'
C.color = ui_flowgraph_editor.nodeColors.traffic
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'table', name = 'vehPool', tableType = 'vehiclePool', description = 'Vehicle pool object; use the Create Pool node.' },
  { dir = 'in', type = 'number', name = 'maxActive', description = 'Maximum amount of active vehicles in the pool; set to -1 to use max amount. Tip: Use the Activate All Vehicles node with this.' },
  { dir = 'in', type = 'bool', name = 'globalPoolingMode', description = 'If true, the pool will limit the amount of active vehicles in the entire scene.' },

  { dir = 'out', type = 'table', name = 'vehicleIds', tableType = 'vehicleIds', description = 'Table of vehicle ids.' },
  { dir = 'out', type = 'table', name = 'activeVehIds', tableType = 'vehicleIds', hidden = true, description = 'Table of active vehicle ids.' },
  { dir = 'out', type = 'table', name = 'inactiveVehIds', tableType = 'vehicleIds', hidden = true, description = 'Table of inactive vehicle ids.' },
  { dir = 'out', type = 'number', name = 'activeAmount', description = 'Amount of active vehicles in this pool.' },
  { dir = 'out', type = 'number', name = 'inactiveAmount', description = 'Amount of inactive vehicles in this pool.' }
}

C.dependencies = {'core_vehicleActivePooling'}
C.tags = {'traffic', 'budget', 'pooling'}

function C:workOnce()
  local pool = self.pinIn.vehPool.value

  if pool then
    local amount = self.pinIn.maxActive.value
    pool.poolingMode = self.pinIn.globalPoolingMode.value and 'global' or 'default'
    if amount and amount >= 0 then
      pool:setMaxActiveAmount(amount)
    else
      pool:setMaxActiveAmount(math.huge)
    end

    self.pinOut.vehicleIds.value = pool:getVehs()
    self.pinOut.activeVehIds.value = pool.activeVehs
    self.pinOut.inactiveVehIds.value = pool.inactiveVehs
    self.pinOut.activeAmount.value = #pool.activeVehs
    self.pinOut.inactiveAmount.value = #pool.inactiveVehs
  end
end

return _flowgraph_createNode(C)