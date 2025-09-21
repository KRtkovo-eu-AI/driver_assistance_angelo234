-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
C.name = 'Set Recovery Prompt Button Enabled'
C.description = "Enables or disables recovery prompt buttons. If a button is inactive when it should be enabled, it will be activated too."
C.icon = ui_flowgraph_editor.nodeIcons.recoveryPrompt
C.color = ui_flowgraph_editor.nodeColors.recoveryPrompt
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'string', name = 'id', description = 'ID of the button.' },
  { dir = 'in', type = 'bool', name = 'enabled', description = 'If the button should be enabled or disabled.' },
}

function C:workOnce()
  core_recoveryPrompt.setButtonEnabledById(self.pinIn.id.value, self.pinIn.enabled.value or false)
end

function C:postInit()
  self.pinInLocal.id.hardTemplates = {
    {value = 'flipMission'},
    {value = 'recoverMission'},
    {value = 'submitMission'},
    {value = 'restartMission'},
  }
end

return _flowgraph_createNode(C)
