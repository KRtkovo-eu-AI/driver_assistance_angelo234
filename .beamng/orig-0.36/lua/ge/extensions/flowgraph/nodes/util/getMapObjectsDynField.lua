-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Get Map Objects IDs by DynField'

C.description = 'Returns a list of IDs of map objects containing a dynamic field.'
C.category = 'repeat_instant'
C.tags = {"map", "dynamic field", "objects", "vehicle", "get", "find"}
C.pinSchema = {
  { dir = 'in', type = 'string', name = 'dynamicFieldName', description = "The name of the dynamic field to search for." },
  { dir = 'out', type = 'table', name = 'objects', description = "Table of objects containing the dynamic field." },
}

C.color = ui_flowgraph_editor.nodeColors.default

function C:init(mgr)

end

function C:_executionStarted()
  self.oldPos = nil
end

function C:findObjectsWithDynField(dynamicFieldName)
  local objects = {}
  for objId, _ in pairs(map.objects) do
    local object = getObjectByID(objId)
    for _, name in ipairs(object:getDynamicFields()) do
      if name == dynamicFieldName then
        table.insert(objects, object:getId())
      end
    end
  end
  return objects
end

function C:work()
  if not self.pinIn.dynamicFieldName.value then return end
  self.pinOut.objects.value = self:findObjectsWithDynField(self.pinIn.dynamicFieldName.value)
end

return _flowgraph_createNode(C)
