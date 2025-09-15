-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

local defaultFGColor = {0, 1, 0, 1} -- green
local defaultFGColor1 = {1, 0, 0, 1} -- red
local defaultFGColor2 = {0, 0, 1, 1} -- blue
local defaultBGColor = {1, 1, 1, 1} -- white
local defaultAmount = 32
local defaultDecalPath = "art/shapes/arrows/t_arrow_opaque_d.color.png"
local defaultDecalScale = {1, 1, 3}
local defaultInverted = false

C.name = 'Decal Line'
C.color = ui_flowgraph_editor.nodeColors.scene
C.icon = ui_flowgraph_editor.nodeIcons.scene
C.description = "Draws a simple decal multiple times from point A to point B. Works like a loading bar."
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'vec3', name = 'posA', description = 'Start of the line.' },
  { dir = 'in', type = 'vec3', name = 'posB', description = 'End of the line.' },
  { dir = 'in', type = 'vec3', name = 'decalScale', description = 'Decal scale.', default = deepcopy(defaultDecalScale), hardcoded = true },
  { dir = 'in', type = 'number', name = 'amount', description = "How many decals will be used to draw the line.", default = defaultAmount, hardcoded = true },
  { dir = 'in', type = 'bool', name = 'inverted', hidden = true, hardcoded = true, default = defaultInverted, description = "Whether the line will be filling from the bottom instead of the top." },
  { dir = 'in', type = 'string', name = 'decalPath', description = "The path to the decal to be used.", default = defaultDecalPath, hardcoded = true },
  { dir = 'in', type = 'number', name = 'filling1', description = "How much filled (0 - 100) will the first filling be (Can display progress).", default = 100, hardcoded = true },
  { dir = 'in', type = 'number', name = 'filling2', description = "How much filled (0 - 100) will be the second filling (Can display a cooldown).", default = 0, hardcoded = true },
  { dir = 'in', type = 'color', name = 'filledColor', hidden = true, hardcoded = true, default = deepcopy(defaultFGColor), description = 'Primary color.' },
  { dir = 'in', type = 'color', name = 'fillingColor1', hidden = true, hardcoded = true, default = deepcopy(defaultFGColor1), description = 'First filling color (progress).' },
  { dir = 'in', type = 'color', name = 'fillingColor2', hidden = true, hardcoded = true, default = deepcopy(defaultFGColor2), description = 'Second filling color (cooldown).' },
  { dir = 'in', type = 'color', name = 'backgroundColor', hidden = true, hardcoded = true, default = deepcopy(defaultBGColor), description = 'Secondary color (opposite of filling).' }
}

C.tags = {'util', 'draw'}

function C:init()
  self.data.snapToTerrain = false
end

local function getNewDecal()
  -- create decals
  return {
    texture = 'art/shapes/arrows/t_arrow_opaque_d.color.png',
    position = vec3(0, 0, 0),
    forwardVec = vec3(0, 0, 0),
    color = ColorF(1, 0, 0, 1 ),
    scale = vec3(1, 1, 1),
    fadeStart = 200,
    fadeEnd = 250
  }
end

local decals, count
function C:_executionStarted()
  decals = {}
  count = 0
  self.colorCache = {}
  self.colorFCache = {}
end

local function increaseDecalPool(max)
  while count < max do
    count = count + 1
    table.insert(decals, getNewDecal())
  end
end

local fwd = vec3()
local t, data, a, b
function C:work()
  for k, pin in pairs(self.pinInLocal) do
    if pin.type == 'color' then
      if not self.colorCache[k] or (self.pinIn[k].value and not setEqual(self.colorCache[k], self.pinIn[k].value)) then
        self.colorCache[k] = deepcopy(self.pinIn[k].value or pin.default)
        self.colorFCache[k] = ColorF(unpack(self.colorCache[k]))
      end
    end
  end

  local amount = self.pinIn.amount.value or defaultAmount
  local invAmount = 1 / amount
  increaseDecalPool(amount+1)

  a, b = vec3(self.pinIn.posA.value) or vec3(0, 0, 0), vec3(self.pinIn.posB.value) or vec3(0, 0, 0)

  fwd:set((b - a):normalized())
  for i = 0, self.pinIn.amount.value or defaultAmount do
    t = i * invAmount
    data = decals[i + 1]

    if (self.pinIn.filling1.value or 100) >= 100 then
      if (self.pinIn.filling2.value or 0) > 0 then
        if (not self.pinIn.inverted.value and (t * 100) or (100 - t * 100)) <= (self.pinIn.filling2.value or 0) then
          data.color = self.colorFCache.fillingColor2
        else
          data.color = self.colorFCache.filledColor
        end
      else
        data.color = self.colorFCache.filledColor
      end
    else
      if (not self.pinIn.inverted.value and (t * 100) or (100 - t * 100)) <= (100 - self.pinIn.filling1.value or 0) then
        data.color = self.colorFCache.fillingColor1
      else
        data.color = self.colorFCache.backgroundColor
      end
    end

    data.position = lerp(a, b, t)
    data.forwardVec = fwd
    data.texture = self.pinIn.decalPath.value or defaultDecalPath
    data.scale:set(unpack(self.pinIn.decalScale.value or defaultDecalScale))

    -- if self.data.snapToTerrain and core_terrain and core_terrain.getTerrain() then
    --   data.position.z = core_terrain.getTerrainHeight(data.position)
    -- end
  end
  Engine.Render.DynamicDecalMgr.addDecals(decals, amount)
end

return _flowgraph_createNode(C)
