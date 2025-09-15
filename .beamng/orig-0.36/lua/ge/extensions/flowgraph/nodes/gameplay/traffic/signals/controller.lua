-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Get Signal Controller'
C.description = 'Gets properties of a signal controller.'
C.color = ui_flowgraph_editor.nodeColors.signals
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'repeat_instant'

C.pinSchema = {
  {dir = 'in', type = 'table', name = 'controllerData', tableType = 'signalControllerData', description = 'Signal controller data.'},
  {dir = 'out', type = 'string', name = 'signalType', description = 'Controller type.'},
  {dir = 'out', type = 'number', name = 'stateCount', description = 'Number of states.'},
  {dir = 'out', type = 'number', name = 'totalDuration', description = 'Complete duration of all states.'}
}

C.tags = {'traffic', 'signals'}

function C:init()
  self.count = 1
end

function C:postInit()
  self:updatePins(0, self.count)
end

function C:drawCustomProperties()
  local reason
  im.PushID1("LAYOUT_COLUMNS")
  im.Columns(2, "layoutColumns")
  im.Text("State Count")
  im.NextColumn()
  local count = im.IntPtr(self.count)
  if im.InputInt('##count'..self.id, count) then
    count[0] = math.max(0, count[0])
    self:updatePins(self.count, count[0])
    reason = "Changed Value count to " .. count[0]
  end

  im.Columns(1)
  im.PopID()
  return reason
end

function C:updatePins(old, new)
  if new < old then
    for i = old, new + 1, -1 do
      for _, lnk in pairs(self.graph.links) do
        if lnk.sourcePin == self.pinOut['stateName_'..i] or lnk.sourcePin == self.pinOut['stateDuration_'..i] then
          self.graph:deleteLink(lnk)
        end
      end
      self:removePin(self.pinOut['stateName_'..i])
      self:removePin(self.pinOut['stateDuration_'..i])
    end
  else
    for i = old + 1, new do
      self:createPin('out', 'string', 'stateName_'..i)
      self:createPin('out', 'number', 'stateDuration_'..i)
    end
  end
  self.count = new
end

function C:_onSerialize(res)
  res.count = self.count
end

function C:_onDeserialized(res)
  self.count = res.count or 1
  self:updatePins(1, self.count)
end

function C:work(args)
  local ctrl = self.pinIn.controllerData.value
  if ctrl then
    self.pinOut.signalType.value = ctrl.type
    self.pinOut.stateCount.value = tableSize(ctrl.states)
    if not self.pinOut.totalDuration.value then
      self.pinOut.totalDuration.value = 0
      for _, state in ipairs(ctrl.states) do
        self.pinOut.totalDuration.value = self.pinOut.totalDuration.value + state.duration
      end
    end

    for i = 1, self.count do
      local state = ctrl.states[i]
      self.pinOut['stateName_'..i].value = state and state.state or 'none'
      self.pinOut['stateDuration_'..i].value = state and state.duration or 0
    end
  end
end

return _flowgraph_createNode(C)