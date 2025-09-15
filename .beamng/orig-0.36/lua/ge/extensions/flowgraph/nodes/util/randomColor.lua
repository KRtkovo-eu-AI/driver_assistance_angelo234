-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Random Color'
C.tags = {'random', 'color', 'colour'}
C.icon = "casino"
C.description = "Provides a random color."
C.category = 'repeat_instant'

C.pinSchema = {
  {dir = 'in', type = 'bool', name = 'cuteColor', description = "If true, only generates highly saturated colors."},
  {dir = 'in', type = 'bool', name = 'randomAlpha', description = "If true, also randomizes the alpha value."},
  {dir = 'out', type = 'color', name = 'color', description = "The color value."}
}

function C:work()
  local a = self.pinIn.randomAlpha.value and math.random() or 1
  if self.pinIn.cuteColor.value then
    local r, g, b = HSVtoRGB(math.random(), lerp(1, 0.7, square(math.random())), lerp(1, 0.7, square(math.random())))
    self.pinOut.color.value = {r, g, b, a}
  else
    self.pinOut.color.value = {math.random(), math.random(), math.random(), a}
  end
end

return _flowgraph_createNode(C)
