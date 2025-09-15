-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Has Coupler Tag'
C.description = 'Detect whether or not a vehicle has a specific coupler tag.'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = "link"
C.category = 'repeat_instant'

local couplerTags = {
  "tow_hitch",
  "fifthwheel",
  "gooseneck_hitch",
  "fifthwheel_v2",
  "pintle",
}

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehA', description = 'Id of vehicle A.' },
}
for _, tag in ipairs(couplerTags) do
  table.insert(C.pinSchema, { dir = 'out', type = 'bool', name = tag, description = 'True if the vehicle has the ' .. tag .. ' coupler tag.' })
end

C.tags = {'event','attach','detach'}

function C:_executionStarted()
  for _, tag in ipairs(couplerTags) do
    self.pinOut[tag].value = false
  end
end

function C:work(args)
  local vehA = self.mgr.modules.vehicle:getVehicle(self.pinIn.vehA.value)
  for _, tag in pairs(couplerTags) do
    self.pinOut[tag].value = vehA.couplerTags[tag] or false
  end
end

return _flowgraph_createNode(C)
