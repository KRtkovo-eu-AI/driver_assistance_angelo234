-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

local defaultFGColor = {51 / 255, 135 / 255, 255 / 255, 200 / 255}
local defaultBGColor = {165 / 255, 203 / 255, 255 / 255, 200 / 255}
local defaultDecalPath = "art/shapes/interface/parkDecalStripes.png"
local defaultDecalScale = {1, 1, 1}

C.name = 'Decal Single'
C.color = ui_flowgraph_editor.nodeColors.scene
C.icon = ui_flowgraph_editor.nodeIcons.scene
C.description = "Draws a single decal in the world."
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'vec3', name = 'pos', description = 'Decal position.' },
  { dir = 'in', type = 'quat', name = 'rot', description = 'Decal rotation.' },
  { dir = 'in', type = 'vec3', name = 'scl', description = "Decal scale.", default = deepcopy(defaultDecalScale), hardcoded = true },
  { dir = 'in', type = 'string', name = 'decalPath', description = 'The path of the decal to be used.', default = defaultDecalPath, hardcoded = true },
  { dir = 'in', type = 'color', name = 'clrA', hidden = true, hardcoded = true, default = deepcopy(defaultFGColor), description = 'Primary color of the decal.' },
  { dir = 'in', type = 'color', name = 'clrB', hidden = true, hardcoded = true, default = deepcopy(defaultBGColor), description = 'Secondary color of the decal.' },
  { dir = 'in', type = 'number', name = 'clrFrequency', hidden = true, hardcoded = true, default = 2, description = 'Frequency of the color change.' }
}

C.tags = {'util', 'draw'}

function C:init()
end

local pingpong = function(t, max)
  local v = (t % (2 * max))
  if v > max then
    return max - (v - max)
  else
    return v
  end
end

function C:work()
  local clr = {}
  for i = 1, 4 do
    clr[i] = lerp(self.pinIn.clrA.value[i], self.pinIn.clrB.value[i], pingpong(os.clockhp() * self.pinIn.clrFrequency.value, 1))
  end
  local decal = {{
    texture = self.pinIn.decalPath.value,
    position = vec3(self.pinIn.pos.value),
    forwardVec = quat(self.pinIn.rot.value) * vec3(0, 1, 0),
    color = ColorF(unpack(clr)),
    scale = vec3(self.pinIn.scl.value or defaultDecalScale),
    fadeStart = 200,
    fadeEnd = 250
  }}
  Engine.Render.DynamicDecalMgr.addDecals(decal, 1)
end

return _flowgraph_createNode(C)
