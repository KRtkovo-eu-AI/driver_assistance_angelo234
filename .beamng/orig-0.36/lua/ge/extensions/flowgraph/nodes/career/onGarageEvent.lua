-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'On Garage Event'
C.description = 'Various Garage related Events'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'beforeGarageInventoryOpened', description = "Outflow once when the Inventory Menu for before garage opens.", impulse = true },
  { dir = 'out', type = 'flow', name = 'beforeGarageVehicleSelected', description = "Outflow once when the vehicle is selected before garage mode.", impulse = true },
  { dir = 'out', type = 'flow', name = 'endMode', description = "Outflow once when Garage mode is ended", impulse = true },

}
C.dependencies = {}

function C:init()
  self.flags = {}
end

function C:work(args)
  self.pinOut.beforeGarageInventoryOpened.value = false
  self.pinOut.beforeGarageVehicleSelected.value = false
  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end
  table.clear(self.flags)
end

function C:onBeforeGarageVehicleSelectionMenuOpened(data)
  self.flags.beforeGarageInventoryOpened = true
end

function C:onBeforeGarageVehicleSelected(data)
  self.flags.beforeGarageVehicleSelected = true
end

function C:onEndGarageMode(data)
  self.flags.endMode = true
end


return _flowgraph_createNode(C)