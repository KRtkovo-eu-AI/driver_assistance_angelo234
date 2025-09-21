-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'im Vehicle Selector'
C.description = 'Vehicle selector ui made in imgui.'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'selected', description = 'Flows after the Select button was pressed.' },
  { dir = 'out', type = 'string', name = 'model', description = 'The model of the selected vehicle.' },
  { dir = 'out', type = 'string', name = 'config', description = 'The config of the selected vehicle.' },
  { dir = 'out', type = 'string', name = 'configPath', description = 'The full config path of the selected vehicle.' },
  { dir = 'out', type = 'string', name = 'paintName1', hidden = true, description = 'The first paint layer of the selected vehicle.' },
  { dir = 'out', type = 'string', name = 'paintName2', hidden = true, description = 'The second paint layer of the selected vehicle.' },
  { dir = 'out', type = 'string', name = 'paintName3', hidden = true, description = 'The third paint layer of the selected vehicle.' }
}

C.tags = {'imgui', 'vehicle', 'config'}

function C:init()
  self.data.enableConfigs = true
  self.data.enablePaints = false
  self.data.enableCustomConfig = false
end

function C:_executionStarted()
  self.util = require("/lua/ge/extensions/editor/util/vehicleSelectUtil")("FG Vehicle Selector##"..self.id)
  self.util.enableConfigs = self.data.enableConfigs
  self.util.enablePaints = self.data.enablePaints
  self.util.enableCustomConfig = self.data.enableCustomConfig

  self.open = false
  self.done = false
  self.refresh = false
end

function C:drawMiddle(builder, style)
  builder:Middle()
end

function C:displayWindow()
  im.SetNextWindowSize(im.ImVec2(480, 300))
  im.Begin("Vehicle Select##"..self.id, im.BoolPtr(true))

  if self.util:widget() then
    self.refresh = true
  end
  if self.refresh then
    self.pinOut.model.value = self.util.model
    self.pinOut.config.value = self.util.config
    self.pinOut.configPath.value = self.util.configPath
    self.pinOut.paintName1.value = self.util.paintName
    self.pinOut.paintName2.value = self.util.paintName2
    self.pinOut.paintName3.value = self.util.paintName3

    self.refresh = false
  end

  if im.Button("Select") then
    self.done = true
    self.open = false
    self.pinOut.selected.value = true
  end
  if im.Button("Reset") then
    self.util:resetSelections()
    self.refresh = true
  end

  im.End()
end

function C:onNodeReset()
  self.open = false
  self.done = false
  self.pinOut.selected.value = false
end

function C:workOnce()
  self.open = true
  self.refresh = true
end

function C:work()
  if self.open then
    self:displayWindow()
  end
end

return _flowgraph_createNode(C)
