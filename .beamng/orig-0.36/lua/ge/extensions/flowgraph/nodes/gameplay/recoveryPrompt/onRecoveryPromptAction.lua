-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'On Recovery Prompt Action'
C.description = 'Lets flow through if any button of the recoveryPrompt has been pressed'
C.color = ui_flowgraph_editor.nodeColors.recoveryPrompt
C.icon = ui_flowgraph_editor.nodeIcons.recoveryPrompt
C.category = 'logic'

C.pinSchema = {
  {dir = 'in', type = 'flow', name = 'flow', description = "Inflow for this node.", fixed = true},
  {dir = 'out', type = 'flow', name = 'flow', description = "Outflow for this node.", fixed = true},
  {dir = 'out', type = 'flow', name = 'flipMission', description = "Outflow once when 'Flip upright'(Mission) has been pressed.", fixed = true},
  {dir = 'out', type = 'flow', name = 'recoverMission', description = "Outflow once when 'Recover'(Mission) has been pressed.", fixed = true},
  {dir = 'out', type = 'flow', name = 'submitMission', description = "Outflow once when 'Commit Attempt'(Mission) has been pressed.", fixed = true},
  {dir = 'out', type = 'flow', name = 'restartMission', description = "Outflow once when 'Restart Mission'(Mission) has been pressed.", fixed = true},
}
C.allowCustomOutPins = true
C.allowedManualPinTypes = {
  flow = true,
}
function C:init()
  self.savePins = true
end
function C:work(args)
  for _, pin in pairs(self.pinOut) do
    pin.value = false
  end
  self.pinOut.flow.value = self.pinIn.flow.value
  for key, val in pairs(self.flags) do
    if self.pinOut[key] then
      self.pinOut[key].value = val
    end
  end
  table.clear(self.flags)
end

function C:_afterTrigger()
  table.clear(self.flags)
end

function C:drawMiddle(builder, style)
  builder:Middle()
end

function C:_executionStarted()
  self.flags = {}
end

function C:onRecoveryPromptButtonPressed(id)
  self.flags[id] = true
end

return _flowgraph_createNode(C)
