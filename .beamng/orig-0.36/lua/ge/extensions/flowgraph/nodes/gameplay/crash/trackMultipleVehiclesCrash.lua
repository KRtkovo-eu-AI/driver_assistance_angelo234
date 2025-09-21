-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Track Multiple Vehicles Crash'

C.description = 'Add multiple vehicles to track for crashes.'
C.color = ui_flowgraph_editor.nodeColors.crash
C.category = 'repeat_instant'
C.tags = {"crash", "vehicle", "track", "crashDetection"}
C.pinSchema = {
  { dir = 'in', type = 'table', name = 'vehIds', description = "The vehicle ids that should be tracked." },
}


function C:work()
  for _, vehId in ipairs(self.pinIn.vehIds.value) do
    gameplay_util_crashDetection.addTrackedVehicleById(vehId);
  end
end

return _flowgraph_createNode(C)
