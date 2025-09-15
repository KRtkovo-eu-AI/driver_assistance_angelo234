-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Garage Tutorial Util'
C.description = 'Util Helper for Garage tutorial'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'bool', name = 'parkedInGarage', description = "if a vehicle is in the garage" },
  { dir = 'out', type = 'bool', name = 'isRepaired', description = "if the vehicle in the garage is repaired" },
}
C.dependencies = {}

function C:init()
  self.timer = 0
end

function C:_executionStarted()
  self.timer = 0
end

function C:work(args)
  local vehInventoryId = career_modules_inventory.getInventoryIdsInClosestGarage(true)
  self.pinOut.parkedInGarage.value = false
  if vehInventoryId then
    local objId = career_modules_inventory.getVehicleIdFromInventoryId(vehInventoryId)
    local vehicleData = map.objects[objId]
    if vehicleData.vel:length() < 0.25 then
      self.pinOut.parkedInGarage.value = true
    end
  end

  self.timer = self.timer + self.mgr.dtSim
  if self.timer > 1 then
    self.timer = self.timer -1
    if not gameplay_walk.isWalking() then
      career_modules_inventory.updatePartConditions(nil, career_modules_inventory.getInventoryIdFromVehicleId(be:getPlayerVehicleID(0)))
    end
  end

  self.pinOut.isRepaired.value = not career_modules_insurance.inventoryVehNeedsRepair(career_modules_inventory.getInventoryIdFromVehicleId(be:getPlayerVehicleID(0)))

end



return _flowgraph_createNode(C)