-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'im Color'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.description = "Displays a color selector in an imgui window."
C.category = 'repeat_instant'

C.pinSchema = {
    { dir = 'in', type = 'color', name = 'colorIn', hidden = true, description = 'Initial color value.' },
    { dir = 'in', type = 'any', name = 'text', description = 'Text to display.' },
    { dir = 'out', type = 'color', name = 'colorOut', description = 'The color value.' },
}

C.tags = {'imgui'}

function C:_executionStarted()
  for _, p in pairs(self.pinOut) do
    p.value = false
  end

  self.imColor = im.ArrayFloatByTbl(self.pinIn.colorIn.value or {1, 1, 1, 1})
end

function C:work()
  im.ColorEdit4((self.pinIn.text.value or "Color")..'##colorPickerFG'..self.id, self.imColor, im.flags(im.ColorEditFlags_NoInputs, im.ColorEditFlags_AlphaBar))
  self.pinOut.colorOut.value = {self.imColor[0], self.imColor[1], self.imColor[2], self.imColor[3]}
end

return _flowgraph_createNode(C)
