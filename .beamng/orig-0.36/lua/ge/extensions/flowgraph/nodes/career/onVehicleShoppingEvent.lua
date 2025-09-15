-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'On Vehicle Shopping Event'
C.description = 'Various Vehicle Shopping Events'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'openedMenu', description = "Outflow once when the menu opens.", impulse = true },
  { dir = 'out', type = 'flow', name = 'closedMenu', description = "Outflow once when the menu is closed.", impulse = true },
  { dir = 'out', type = 'flow', name = 'vehicleSpawned', description = "Outflow once when the vehicle is spawned for inspection.", impulse = true },
  { dir = 'out', type = 'flow', name = 'openedDetail', description = "Outflow once when the Detailled Purchase Menu is opened", impulse = true },
  { dir = 'out', type = 'flow', name = 'vehicleBought', description = "Outflow once when the vehicle buyign is complete.", impulse = true },
  { dir = 'out', type = 'number', name = 'vehId', description = "Outflow once when the vehicle buyign is complete."},
  { dir = 'out', type = 'bool', name = 'testDriveActive', description = "If the user is currently doing a testdrive"},

}
C.dependencies = {}

function C:init()
  self.flags = {}
end

function C:work(args)
  self.pinOut.openedMenu.value = false
  self.pinOut.closedMenu.value = false
  self.pinOut.openedDetail.value = false
  self.pinOut.vehicleBought.value = false
  self.pinOut.vehicleSpawned.value = false
  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end
  table.clear(self.flags)
  self.pinOut.testDriveActive.value = career_modules_testDrive.isActive()
end

function C:onVehicleShoppingMenuOpened(data)
  self.flags.openedMenu = true
end

function C:onVehicleShoppingVehicleShown(data)
  self.flags.vehicleSpawned = true
end

function C:onVehicleShoppingPurchaseMenuOpened(data)
  self.flags.openedDetail = true
end

function C:onVehicleAddedToInventory(data)
  self.flags.vehicleBought = true
  self.pinOut.vehId.value = career_modules_inventory.getVehicleIdFromInventoryId(data.inventoryId)
end

function C:onVehicleShoppingMenuClosed()
  self.flags.closedMenu = true
end

function C:onUIPlayStateChanged(enteredPlay)
  if enteredPlay then
    self.flags.closedMenu = true
  end
end


return _flowgraph_createNode(C)