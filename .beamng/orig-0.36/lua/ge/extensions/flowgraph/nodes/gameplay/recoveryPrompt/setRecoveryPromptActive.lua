-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Set Recovery Prompt Active'
C.description = 'Activates or Deactivates the recovery prompt. If it is active, the default "Controls Reset" node will not work.'
C.color = ui_flowgraph_editor.nodeColors.recoveryPrompt
C.icon = ui_flowgraph_editor.nodeIcons.recoveryPrompt
C.category = 'once_instant'

C.pinSchema = {
  {dir = 'in', type = 'bool', name = 'active', default = true, hardcoded = true, description = "If this system should be active or not."},
  {dir = 'in', type = 'bool', name = 'enableAll', default = true, hardcoded = true, description = "If set to a non-nil value, will also set enable to this pins value, for all buttons affected by this node."},
  {dir = 'in', type = 'bool', name = 'flipMission', default = true, hardcoded = true, hidden=false, description = "If 'Flip upright' should be active or not. Use this for missions. Has no fade."},
  {dir = 'in', type = 'bool', name = 'recoverMission', default = true, hardcoded = true, hidden=false, description = "If 'Recover' should be active or not. Use this for missions. Has fade."},
  {dir = 'in', type = 'bool', name = 'submitMission', default = false, hardcoded = true, hidden=false, description = "If 'Submit Score' should be active or not. Use this for missions. Has no fade."},
  {dir = 'in', type = 'bool', name = 'restartMission', default = false, hardcoded = true, hidden=false, description = "If 'Restart Mission' should be active or not. Use this for missions. Has no fade."},
}
C.dependencies = {'gameplay_walk'}
C.blocksOnResetGameplay = true


function C:workOnce(args)
  if self.pinIn.active.value then
    core_recoveryPrompt.setActive(true)
    core_recoveryPrompt.deactivateAllButtons()
    for _, o in pairs({'flipMission','recoverMission','submitMission','restartMission'}) do
      if self.pinIn[o].value ~= nil then
        core_recoveryPrompt.setButtonActiveById(o, self.pinIn[o].value)
        --if self.pinIn.enableAll.value ~= nil and  then
        --  core_recoveryPrompt.setButtonEnabledById(o, self.pinIn.enableAll.value and self.pinIn[o].value)
        --end
      end
    end
  else
    core_recoveryPrompt.setActive(false)
  end
end

function C:drawMiddle(builder, style)
  builder:Middle()
end

return _flowgraph_createNode(C)
