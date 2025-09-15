-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Get Player Vehicle ID'
C.description = 'Gets the player vehicles id.'
C.color = ui_flowgraph_editor.nodeColors.scene
C.icon = ui_flowgraph_editor.nodeIcons.scene
C.category = 'provider'

C.pinSchema = {
  { dir = 'out', type = 'number', name = 'vehId', description = 'Id of the vehicle that the player currently controls.' },
  { dir = 'out', type = 'bool', name = 'exist', description = 'If the object exists.', hidden=true, },
}
C.legacyPins = {
  out = {
    objID = 'vehId'
  }
}

C.tags = {}

function C:init()
end

function C:work()
  self.pinOut.vehId.value = be:getPlayerVehicleID(0) or -1
  self.pinOut.exist.value = self.pinOut.vehId.value ~= -1
end


return _flowgraph_createNode(C)
