-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Set Recovery Prompt Limits'
C.description = 'Sets a limit to the amount a button can be used in the recovery prompt. If no uses are left, the button still shows (if it is active), but cannot be pressed anymore and is grayed out. Use -1 for unlimited.'
C.color = ui_flowgraph_editor.nodeColors.recoveryPrompt
C.icon = ui_flowgraph_editor.nodeIcons.recoveryPrompt
C.category = 'once_instant'

C.pinSchema = {
  {dir = 'in', type = 'bool', name = 'resetCounts', default = true, hardcoded = true, hidden=false, description = "If set, will also set all used-counts to 0.", fixed = true},
  {dir = 'in', type = 'number', name = 'flipMission', default = 5,  hidden=false, description = "If 'Flip upright' should be limited. Use this for missions.", fixed = true},
  {dir = 'in', type = 'number', name = 'recoverMission', default = 5, hidden=false, description = "If 'Recover' should be limited. Use this for missions.", fixed = true},
  {dir = 'in', type = 'number', name = 'submitMission', default = -1, hidden=true, description = "If 'Submit Score' should be limited. Use this for missions.", fixed = true},
  {dir = 'in', type = 'number', name = 'restartMission', default = -1, hidden=true, description = "If 'Restart Mission' should be limited. Use this for missions.", fixed = true},

}
C.dependencies = {'gameplay_walk'}
C.allowCustomInPins = true
C.allowedManualPinTypes = {
  number = true,
}
function C:init()
  self.savePins = true
end

function C:workOnce(args)
  local limits = {}
  local counts = {}
  for name, pin in pairs(self.pinInLocal) do
    if self.pinIn[name].value ~= nil and pin.type == 'number' then
      limits[name] = self.pinIn[name].value
      counts[name] = true
    end
  end
  core_recoveryPrompt.setButtonLimits(limits)
  core_recoveryPrompt.resetButtonLimitCounters(counts)
end
function C:drawMiddle(builder, style)
  builder:Middle()
end
return _flowgraph_createNode(C)
