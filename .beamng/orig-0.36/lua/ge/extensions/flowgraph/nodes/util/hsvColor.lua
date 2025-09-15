-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Color HSV'
C.description = "Creates a color using the HSV color scheme. All input values are between 0 and 1"
C.category = 'simple'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'h', description = 'Hue' },
  { dir = 'in', type = 'number', name = 's', description = 'Saturation' },
  { dir = 'in', type = 'number', name = 'v', description = 'Value' },
  { dir = 'in', type = 'number', name = 'a', description = 'Alpha, controls paint chrominess in vehicles' },
  { dir = 'out', type = 'color', name = 'color', description = 'Output as color type' },
}

C.tags = {'variable'}

function C:init()

end

function C:HSVtoRGB(h,s,v)
  h = h - math.floor(h)
  return {HSVtoRGB(math.max(0, math.min(1, h)), math.max(0, math.min(1, s)), math.max(0, math.min(1, v)))}
end

function C:work()
  local rgb = self:HSVtoRGB(self.pinIn.h.value or 0, self.pinIn.s.value or 1, self.pinIn.v.value or 1)
  self.pinOut.color.value = {rgb[1],rgb[2],rgb[3], self.pinIn.a.value or 1}
end

return _flowgraph_createNode(C)
