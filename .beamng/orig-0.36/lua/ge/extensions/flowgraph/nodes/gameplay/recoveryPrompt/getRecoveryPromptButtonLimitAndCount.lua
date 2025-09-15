-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Get Recovery Prompt Button Limit and Count'
C.description = 'Activates or deactivates a button the recovery prompt. If a button is active, it will be clickable. Otherwise it will be grayed out and not clickable, but still visible.'
C.color = ui_flowgraph_editor.nodeColors.recoveryPrompt
C.icon = ui_flowgraph_editor.nodeIcons.recoveryPrompt
C.category = 'once_instant'

C.pinSchema = {
  {dir = 'in', type = 'string', name = 'id', description = "ID for the button"},
  {dir = 'out', type = 'number', name = 'limit', description = "Limit for this button"},
  {dir = 'out', type = 'number', name = 'count', description = "Count for this button"},
}
C.dependencies = {'gameplay_walk'}

function C:postInit()
  self.pinInLocal.id.hardTemplates = {
    {value = 'flipMission'},
    {value = 'recoverMission'},
    {value = 'submitMission'},
    {value = 'restartMission'},
  }
end
function C:workOnce(args)
  local data = core_recoveryPrompt.getButtonLimitsAndCounts()
  self.pinOut.limit.value = nil
  self.pinOut.count.value = nil
  if data[self.pinIn.id.value] then
    self.pinOut.limit.value = data[self.pinIn.id.value].limit or -1
    self.pinOut.count.value = data[self.pinIn.id.value].count or -1
  end
end

function C:drawMiddle(builder, style)
  builder:Middle()
end

return _flowgraph_createNode(C)
