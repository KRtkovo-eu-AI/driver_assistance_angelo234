-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'EndScreen Drag Time Slip'
C.color = ui_flowgraph_editor.nodeColors.ui
C.description = 'Displays the drag race time slip with detailed info.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'out', type = 'flow', name = 'flow', description = '', chainFlow = true },
}

C.tags = { 'end', 'finish', 'screen', 'outro', 'ui' }

function C:init()
  self.panel = {
    type = "dragTimeSlip",
    header = "Timeslip",
    timeslip = {},
    pages = {
      timeslip = true,
    }
  }
end

function C:work()
  self.pinOut.flow.value = self.pinIn.flow.value
  -- add it to the layout
  local slipData = gameplay_drag_general.createTimeslipData()
  --dump(dragData)
  if not slipData then
    self.mgr.modules.ui:addUIElement(self.panel)
    log("W", logTag, "No slip data found!")
    return
  end
  self.panel.timeslip = slipData
  --dumpz(self.panel, 4)
  self.mgr.modules.ui:addUIElement(self.panel)
end

return _flowgraph_createNode(C)
