-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Set Recovery Prompt Enabled'
C.description = 'Enables or Disables buttons the recovery prompt. If a button is enabled, it will be clickable. Otherwise it will be grayed out and not clickable, but still visible.'
C.color = ui_flowgraph_editor.nodeColors.recoveryPrompt
C.icon = ui_flowgraph_editor.nodeIcons.recoveryPrompt
C.category = 'once_instant'

C.pinSchema = {
  {dir = 'in', type = 'bool', name = 'flipMission', default = true, hidden=false, description = "If 'Flip upright' should be enabled. Use this for missions."},
  {dir = 'in', type = 'bool', name = 'recoverMission', default = true, hidden=false, description = "If 'Recover' should be enabled. Use this for missions."},
  {dir = 'in', type = 'bool', name = 'submitMission', default = false, hidden=true, description = "If 'Submit Score' should be enabled. Use this for missions."},
  {dir = 'in', type = 'bool', name = 'restartMission', default = false, hidden=true, description = "If 'Restart Mission' should be enabled. Use this for missions."},

}
C.dependencies = {'gameplay_walk'}



function C:workOnce(args)
  for _, o in pairs({'flipMission','recoverMission','submitMission','restartMission'}) do
    if self.pinIn[o].value ~= nil then
      core_recoveryPrompt.setButtonEnabledById(o, self.pinIn[o].value)
    end
  end
end

function C:drawMiddle(builder, style)
  builder:Middle()
end

return _flowgraph_createNode(C)
