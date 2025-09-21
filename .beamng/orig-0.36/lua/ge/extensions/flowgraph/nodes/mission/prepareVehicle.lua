-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Prepare Vehicle'
C.description = "Automatically sets vehicle electrics and values with regards to the gameplay environment."
C.color = im.ImVec4(0.13, 0.3, 0.64, 0.75)
C.category = 'once_instant'

C.pinSchema = {
  {dir = 'in', type = 'number', name = 'vehId', hidden = true, description = '(Optional) Vehicle id.'}
}

function C:workOnce()
  self.mgr.modules.mission:prepareVehicle(self.pinIn.vehId.value or be:getPlayerVehicleID(0))
end

return _flowgraph_createNode(C)