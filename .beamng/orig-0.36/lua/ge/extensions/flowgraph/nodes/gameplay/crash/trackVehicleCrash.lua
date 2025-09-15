-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Track Vehicle Crash'

C.description = 'Add a vehicle to track for crashes.'
C.color = ui_flowgraph_editor.nodeColors.crash
C.category = 'repeat_instant'
C.tags = {"crash", "vehicle", "track", "crashDetection"}
C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = "The vehicle id that should be tracked." },
}


function C:work()
  gameplay_util_crashDetection.addTrackedVehicleById(self.pinIn.vehId.value);
end

return _flowgraph_createNode(C)
