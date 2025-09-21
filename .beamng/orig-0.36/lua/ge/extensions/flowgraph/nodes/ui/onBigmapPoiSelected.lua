-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'On Bigmap Poi'
C.description = 'Detects then a poi is selected or navigated to on the bigmap.'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'selected', description = "Outflow once when a Poi is selected", impulse = true },
  { dir = 'out', type = 'flow', name = 'deselected', description = "Outflow once when a Poi is deselected", impulse = true },
  { dir = 'out', type = 'string', name = 'poiId', description = "ID of the selected Poi"},
  { dir = 'out', type = 'flow', name = 'navigated', description = "Outflow once when a location is navigated to.", impulse = true },
  { dir = 'out', type = 'flow', name = 'denavigated', description = "Outflow once when the navigation has been reset.", impulse = true },
  { dir = 'out', type = 'vec3', name = 'pos', description = "Position that has been navigated to."},
}
C.dependencies = {}


function C:init()
  self.flags = {}
  self.navPos = vec3()
end

function C:work(args)
  self.pinOut.selected.value = false
  self.pinOut.deselected.value = false
  self.pinOut.navigated.value = false
  self.pinOut.denavigated.value = false
  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end
  self.pinOut.poiId.value = self.poiId
  self.pinOut.pos.value = self.navPos:toTable()
  --dump(self.flags)
  table.clear(self.flags)
end

function C:onPoiSelectedFromBigmap(poiId)
  if poiId then
    self.flags.selected = true
  else
    self.flags.deselected = true
  end
  self.poiId = poiId
end

function C:onSetBigmapNavFocus(pos)
  if pos then
    self.flags.navigated = true
  else
    self.flags.denavigated = true
  end
  self.navPos = pos or vec3()
end

return _flowgraph_createNode(C)
