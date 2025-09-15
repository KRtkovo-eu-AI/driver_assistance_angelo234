-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Multi Graph'
C.color = ui_flowgraph_editor.nodeColors.debug
C.icon = ui_flowgraph_editor.nodeIcons.debug
C.description = "Draws multiple lines in the same graph."
C.category = 'repeat_instant'

C.pinSchema = {}

C.tags = {'util', 'draw', 'table', 'data'}

local graphSize = im.ImVec2(400, 300)

function C:init()
  self.inputLabels = {}
  self.inputColors = {}
  self.graphData = {}
  self.graphDataCount = 400
  self.valueCount = 0
  self.data.count = 1
  self.data.scaleMin = 0
  self.data.scaleMax = 1
end

function C:work()
  local i = 1
  for l, pin in pairs(self.pinIn) do
    if pin.value and type(pin.value) == "number" then
      if not self.graphData[i] then self.graphData[i] = {} end
      table.insert(self.graphData[i], pin.value)
      self.data.scaleMax = math.max(pin.value, self.data.scaleMax)
      self.data.scaleMin = math.min(pin.value, self.data.scaleMin)
      if #self.graphData[i] >= self.graphDataCount then
        table.remove(self.graphData[i], 1)
      end
      i = i + 1
    end
  end
end

function C:resetGraphData()
  self.inputLabels = {}
  self.inputColors = {}
  self.graphData = {}
  local i = 0
  for l, pin in pairs(self.pinInLocal) do
    if pin.type == "number" and self.pinIn[l].value then
      table.insert(self.inputLabels, pin.name)
      local c = rainbowColor(self.data.count, i, 255)
      table.insert(self.inputColors, im.ImColorByRGB(c[1], c[2], c[3], 255))
      table.insert(self.graphData, {})
      i = i + 1
    end
  end
  self.valueCount = i
end

function C:updatePins(old, new)
  new = math.max(0, math.ceil(new))
  self.data.count = new

  if new < old then
    for i = old, new + 1, -1 do
      for _, link in pairs(self.graph.links) do
        if link.targetPin == self.pinInLocal['value'..i]then
          self.graph:deleteLink(link)
        end
      end
      self:removePin(self.pinInLocal['value'..i])
    end
  else
    for i = old + 1, new do
      -- direction, type, name, default, description, autoNumber
      self:createPin('in', 'number', 'value'..i, 0, 'Data value #'..i)
    end
  end

  self:resetGraphData()
end

function C:onLinkDeleted()
  self:resetGraphData()
end

function C:onLink()
  self:resetGraphData()
end

function C:onUnlink()
  self:resetGraphData()
end

function C:_onSerialize(res)
end

function C:_onDeserialized(data)
  self:updatePins(0, self.data.count)
end

function C:drawMiddle(builder, style)
  if not self.updatePins then return end

  local pinCount = tableSize(self.pinInLocal) - 1
  if pinCount ~= self.data.count then
    self:updatePins(pinCount, self.data.count)
  end

  builder:Middle()
  if #self.graphData > 0 then
    -- TODO: fix invalid C type error
    im.PlotMultiLines("##plotMultiLines"..self.id, #self.graphData, self.inputLabels, self.inputColors, self.graphData, self.graphDataCount, "", self.data.scaleMin, self.data.scaleMax, graphSize)
  end
end

return _flowgraph_createNode(C)
