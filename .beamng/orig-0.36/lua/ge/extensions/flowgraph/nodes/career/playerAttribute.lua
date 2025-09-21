-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui


local C = {}

C.name = 'Add Or Remove Player Attribute'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career

C.description = 'Add or remove a certain amount of a certain player attribute of the current save slot'
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'bool', default = true, hardcoded = true, name = 'add', description = 'True will add money, false will remove it' },
  { dir = 'in', type = 'number', name = 'amount', default = 0, hardcoded = true, description = 'The amount of money to take away' },
  { dir = 'in', type = 'string', name = 'playerAttribute', default = "money", hardcoded = true, description = "The attribute's name" },
  { dir = 'in', type = 'string', name = 'reason', default = "", description = "Reason for change" },
}
C.tags = {}


function C:onNodeReset()
  self.flag = false
end

function C:_executionStarted()
  self:onNodeReset()
end

function C:work()
  if career_career.isActive() and not self.flag then
    career_modules_playerAttributes.addAttributes({[self.pinIn.playerAttribute.value]=self.pinIn.amount.value * ((self.pinIn.add.value == true) and 1 or -1)}, {label = self.pinIn.reason.value or "Unknown Reason (Flowgraph)"})
    self.flag = true
  end

  if self.pinIn.reset.value then
    self:onNodeReset()
  end
end

return _flowgraph_createNode(C)