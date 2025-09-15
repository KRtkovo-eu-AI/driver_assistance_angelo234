-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

local pinName = 'idToIgnore_'

C.name = 'Remove Other Vehicles'
C.description = 'Will remove every vehicles except from the one(s) specified'
C.category = 'once_instant'

C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.pinSchema = {
  {dir = 'in', type = 'number', name = pinName..'1', description = "Vehicle id that won't be removed"},
}

function C:init()
  self.count = 1
  self.onlyDrivableVehs = false
  self.onlyFlowgraphVehs = false
end

function C:drawCustomProperties()
  local reason = nil
  im.PushID1("LAYOUT_COLUMNS")
  im.Columns(2, "layoutColumns")
  im.Text("Count")
  im.NextColumn()
  local ptr = im.IntPtr(self.count)
  if im.InputInt('##count'..self.id, ptr) then
    if ptr[0] < 1 then ptr[0] = 1 end
    self:updatePins(self.count, ptr[0])
    reason = "Changed Value count to " .. tostring(ptr[0])
  end

  im.NextColumn()
  im.Text("Only Drivable Vehicles")
  im.NextColumn()
  ptr = im.BoolPtr(self.onlyDrivableVehs)
  if im.Checkbox('##onlyDrivableVehs'..self.id, ptr) then
    self.onlyDrivableVehs = ptr[0]
    reason = "Changed Value onlyDrivableVehs to " .. tostring(ptr[0])
  end

  im.NextColumn()
  im.Text("Only Flowgraph Vehicles")
  im.NextColumn()
  ptr = im.BoolPtr(self.onlyFlowgraphVehs)
  if im.Checkbox('##onlyFlowgraphVehs'..self.id, ptr) then
    self.onlyFlowgraphVehs = ptr[0]
    reason = "Changed Value onlyFlowgraphVehs to " .. tostring(ptr[0])
  end
  im.Columns(1)
  im.PopID()
  return reason
end

function C:updatePins(old, new)
  if new < old then
    for i = old, new+1, -1 do
      for _, lnk in pairs(self.graph.links) do
        if lnk.targetPin == self.pinInLocal[pinName..i] then
          self.graph:deleteLink(lnk)
        end
      end
      self:removePin(self.pinInLocal[pinName..i])
    end
  else
    for i = old+1, new do
      --direction, type, name, default, description, autoNumber
      self:createPin('in', 'number', pinName..i)
    end
  end
  self.count = new
end

function C:workOnce()
  local vehIds = {}
  if self.onlyDrivableVehs then
    for _, v in ipairs(getAllVehiclesByType()) do
      table.insert(vehIds, v:getId())
    end
  else
    for _, v in ipairs(getAllVehicles()) do
      table.insert(vehIds, v:getId())
    end
  end

  if self.onlyFlowgraphVehs then
    local tempIds = {}
    local idKeys = tableValuesAsLookupDict(vehIds)
    for _, v in ipairs(self.mgr.modules.vehicle:getSpawnedVehicles()) do
      if idKeys[v] then
        table.insert(tempIds, v)
      end
    end
    vehIds = tempIds
  end

  for _, id in ipairs(vehIds) do
    local delete = true

    for i = 1, self.count do
      if self.pinIn[pinName..i] and self.pinIn[pinName..i].value == id then
        delete = false
        break
      end
    end

    if delete then
      local obj = scenetree.findObjectById(id)

      if obj then
        if editor and editor.onRemoveSceneTreeObjects then
          editor.onRemoveSceneTreeObjects({obj:getId()})
        end
        obj:delete()
      end
    end
  end
end

function C:_onSerialize(res)
  res.count = self.count
  res.onlyDrivableVehs = self.onlyDrivableVehs
  res.onlyFlowgraphVehs = self.onlyFlowgraphVehs
end

function C:_onDeserialized(data)
  self:updatePins(self.count, data.count)
  self.count = data.count
  self.onlyDrivableVehs = data.onlyDrivableVehs
  self.onlyFlowgraphVehs = data.onlyFlowgraphVehs
end

return _flowgraph_createNode(C)
