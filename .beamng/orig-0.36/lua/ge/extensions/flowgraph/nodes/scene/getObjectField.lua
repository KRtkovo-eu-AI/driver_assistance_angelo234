-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Get Object Field'
C.description = 'Gets a field value from an object.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'objectId', description = 'Defines the id of the object to use.' },
  { dir = 'in', type = 'string', name = 'fieldName', description = 'Defines the name of the field to use.' },
  { dir = 'in', type = 'number', name = 'fieldArrayNum', default = 0, hardcoded = true, hidden = true, description = '(Optional) Field array number to read from.' },

  { dir = 'out', type = 'any', name = 'value', description = 'Result value.' }
}
C.color = ui_flowgraph_editor.nodeColors.scene
C.icon = ui_flowgraph_editor.nodeIcons.scene
C.tags = {}

function C:work()
  if not self.pinIn.objectId.value or not self.pinIn.fieldName.value then return end
  local obj = scenetree.findObjectById(self.pinIn.objectId.value)
  if not obj then return end

  if obj:getFields()[self.pinIn.fieldName.value] then
    self.pinOut.value.value = obj:getField(self.pinIn.fieldName.value, self.pinIn.fieldArrayNum.value or 0)
  else
    self.pinOut.value.value = obj:getDynDataFieldbyName(self.pinIn.fieldName.value, self.pinIn.fieldArrayNum.value or 0)
  end
end

return _flowgraph_createNode(C)
