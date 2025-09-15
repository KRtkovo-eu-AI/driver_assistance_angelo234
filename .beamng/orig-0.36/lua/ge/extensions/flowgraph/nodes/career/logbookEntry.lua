-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Logbook Entry'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career
C.behaviour = { duration = true }
C.description = "Adds an entry to the logbook."
C.category = 'once_instant'


C.pinSchema = {
  {dir = 'in', type = 'string', name = 'title', description = "The Title"},
  {dir = 'in', type = 'string', name = 'text', description = "The Description Text"},
}



function C:workOnce()
  if career_career.isActive() and career_modules_logbook then
    career_modules_logbook.genericInfoUnlocked(self.pinIn.title.value, self.pinIn.text.value)
  end
  self.pinOut.flow.value = self.pinIn.flow.value
end

return _flowgraph_createNode(C)
