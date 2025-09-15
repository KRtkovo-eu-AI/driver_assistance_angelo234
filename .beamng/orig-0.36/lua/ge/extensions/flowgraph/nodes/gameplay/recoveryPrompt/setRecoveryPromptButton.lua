-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
C.name = 'Set Recovery Prompt Button'
C.description = "Creates or sets recovery prompt buttons."
C.icon = ui_flowgraph_editor.nodeIcons.recoveryPrompt
C.color = ui_flowgraph_editor.nodeColors.recoveryPrompt
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'string', name = 'buttonId', description = 'ID of the button.' },
  { dir = 'in', type = 'string', name = 'label', default = "Button", description = 'Displayed text for the button.' },
  { dir = 'in', type = 'number', name = 'order', description = 'This buttons order in the button list. Leave empty for automatic order (last entry).' },
  { dir = 'in', type = 'bool', name = 'fadeActive', hidden = true, default = true, hardcoded = true, description = 'If fadeToBack effect should happen' },
  { dir = 'in', type = 'string', name = 'icon', description = 'id of the icon to use for the button' },
}

function C:workOnce()

  core_recoveryPrompt.addButton(self.pinIn.buttonId.value, self.pinIn.label.value, nop, self.pinIn.order.value, nil, true, true, self.pinIn.fadeActive.value, self.pinIn.icon.value)
end

return _flowgraph_createNode(C)
