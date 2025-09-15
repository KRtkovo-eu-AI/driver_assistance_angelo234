-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Set Drag Vehicles'

C.description = "Set all the drag vehicles into the race system."
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId_1', description = 'VehId 1 that will be set to the lane 1 of the dragRace.' },
  { dir = 'in', type = 'bool', name = 'isPlayable_1', description = '' },
  { dir = 'in', type = 'number', name = 'dial_1', description = '' },
  { dir = 'in', type = 'number', name = 'lane_1', description = '' },
  { dir = 'out', type = 'flow', name = 'flow', description = 'Impulse out flow for when all vehicles are into the dragRace system.', impulse = true },
}

C.tags = {'gameplay', 'utils'}

function C:init()
  self.count = 1
  self.mode = 'first'
end

function C:drawMiddle(builder, style)
  builder:Middle()
  im.TextUnformatted(self.mode)
end

function C:_executionStarted()

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
    reason = "Changed Value count to " .. ptr[0]
  end
  im.NextColumn()
  im.TextUnformatted("Merge Functions")
  im.NextColumn()
  if im.BeginCombo("##", self.mode, 0) then
    for _,fun in ipairs(fg_utils.sortedMergeFuns.any) do
      if fun.name ~= 'readOnly' then
        if im.Selectable1(fun.name, fun.name == self.mode) then
          self.mode = fun.name
          reason = "Changed merge function to " .. fun.name
        end
        ui_flowgraph_editor.tooltip(fun.desc)
      end
    end
    im.EndCombo()
  end
  im.Columns(1)
  im.PopID()
  return reason
end

function C:updatePins(old, new)
  if new < old then

    for i = old, new+1, -1 do
      for _, lnk in pairs(self.graph.links) do
        if lnk.sourcePin == self.pinInLocal['vehId_'..i] then
          self.graph:deleteLink(lnk)
        end
        if lnk.sourcePin == self.pinInLocal['isPlayable_'..i] then
          self.graph:deleteLink(lnk)
        end
        if lnk.sourcePin == self.pinInLocal['dial_'..i] then
          self.graph:deleteLink(lnk)
        end
        if lnk.sourcePin == self.pinInLocal['lane_'..i] then
          self.graph:deleteLink(lnk)
        end
      end
      self:removePin(self.pinInLocal['vehId_'..i])
      self:removePin(self.pinInLocal['isPlayable_'..i])
      self:removePin(self.pinInLocal['dial_'..i])
      self:removePin(self.pinInLocal['lane_'..i])
    end

  else
    for i = old+1, new do
      --direction, type, name, default, description, autoNumber
      self:createPin('in', 'number', 'vehId_' .. i, nil, 'Vehicle in lane ' .. i .. ' that will be set.')
      self:createPin('in', 'number', 'dial_' .. i, nil, 'dial in lane ' .. i .. ' that will be set.')
      self:createPin('in', 'bool', 'isPlayable_' .. i, nil, '')
      self:createPin('in', 'number', 'lane_' .. i, nil, '')
    end
  end
  self.count = new
end

function C:workOnce()
  local vehicleList = {}
  for i=1,self.count do
    local vehId = self.pinIn['vehId_'..i].value
    local isPlayable = self.pinIn['isPlayable_'..i].value or false
    local dial = self.pinIn['dial_' .. i].value or -1
    local lane = self.pinIn['lane_' .. i].value or 1
    if vehId and vehId > 0 then
      table.insert(vehicleList, {id = vehId, isPlayable = isPlayable, dial = dial, lane = lane})
    end
  end

  if #vehicleList == 0 then return end
  gameplay_drag_general.setVehicles(vehicleList)
  self.pinOut.flow.value = true
end


function C:_onSerialize(res)
  res.mode = self.mode
  res.count = self.count
end

function C:_onDeserialized(res)
  self.mode = res.mode or 'first'
  self.count = res.count or 1
  self:updatePins(1, self.count)
end

return _flowgraph_createNode(C)