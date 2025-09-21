-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Tasklist Header'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.behaviour = { duration = true }
C.description = "Sets the header and subtext for the career tutorial tasklist."
C.category = 'once_instant'

C.pinSchema = {
  {dir = 'in', type = 'any', name = 'label', description = "The header text"},
  {dir = 'in', type = 'any', name = 'sub', description = "The header subtext"},
}

C.tags = {'goal','goals'}

function C:workOnce()

  guihooks.trigger("SetTasklistHeader", {
    label = self.pinIn.label.value,
    subtext = self.pinIn.sub.value
  })
  self.pinOut.flow.value = self.pinIn.flow.value
end

return _flowgraph_createNode(C)
