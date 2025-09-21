-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Add To Inventory'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career

C.description = 'Adds a vehicle to the player Inventory.'
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'The Id of the vehicle to be affected.' },
}
C.tags = {}

function C:init()

end

function C:workOnce()
  if self.pinIn.vehId.value then
    if career_modules_inventory then
      career_modules_inventory.addVehicle(self.pinIn.vehId.value)
    else
      self:__setNodeError('work', "Career module career_modules_inventory not loaded!")
    end
  else
    self:__setNodeError('work', "No Vehicle ID given.")
  end
  self.pinOut.flow.value = self.pinIn.flow.value
end

return _flowgraph_createNode(C)