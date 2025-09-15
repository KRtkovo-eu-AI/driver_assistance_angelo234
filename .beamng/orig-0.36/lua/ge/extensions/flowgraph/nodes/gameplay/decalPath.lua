-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

local defaultDecalColor = {40 / 255, 120 / 255, 250 / 255, 1}

C.name = 'Decal Path'
C.color = ui_flowgraph_editor.nodeColors.scene
C.icon = ui_flowgraph_editor.nodeIcons.scene
C.description = "Draws an animated decal from point A to point B, along a route."
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'vec3', name = 'posA', description = 'Start position of the route.' },
  { dir = 'in', type = 'vec3', name = 'posB', description = 'End position of the route.' },
  { dir = 'in', type = 'number', name = 'speed', description = 'Animation speed of the decal.', default = 10, hardcoded = true },
  { dir = 'in', type = 'number', name = 'spacing', description = 'Spacing between decal instances.', default = 15, hardcoded = true },
  { dir = 'in', type = 'color', name = 'decalColor', hidden = true, description = 'Decal color.' }
}

C.tags = {'util', 'draw', 'route'}

function C:init()
end

function C:getNewDecal()
  -- create decals
  return {
    texture = "art/shapes/arrows/arrow_groundmarkers_1.png",
    position = vec3(0, 0, 0),
    forwardVec = vec3(0, 0, 0),
    color = ColorF(unpack(self.pinIn.decalColor.value or defaultDecalColor)),
    scale = vec3(8, 12, 4),
    fadeStart = 400,
    fadeEnd = 500
  }
end

local decals, count
function C:_executionStarted()
  decals = {}
  count = 0
end

function C:increaseDecalPool(max)
  while count < max do
    count = count + 1
    table.insert(decals, self:getNewDecal())
  end
end
local route = require('/lua/ge/extensions/gameplay/route/route')()
local fwd = vec3()
local t, data, a, b
function C:work()
  route:setupPathMulti({vec3(self.pinIn.posA.value), vec3(self.pinIn.posB.value)})
  local path = route.path
  local totalPathLength = path[1].distToTarget

  local speed = self.pinIn.speed.value or 10
  local spacing = self.pinIn.spacing.value or 15
  local distance = 0
  distance = os.clock() * speed % spacing
  local pathCount = #path
  for _, wp in ipairs(path) do
    wp.distanceFromStart = totalPathLength - wp.distToTarget
  end

  local markers = {}
  for i = 1, pathCount-1 do
    local cur, nex = path[i], path[i+1]
    local segmentLength = cur.distToTarget - nex.distToTarget
    while distance < nex.distanceFromStart do
      local fwd = (nex.pos - cur.pos):normalized()
      table.insert(markers, {pos = cur.pos + fwd * (distance-cur.distanceFromStart), fwd = fwd, alpha = 1, dist = distance})
      if #markers == 1 then
        markers[1].alpha = distance / spacing
      end
      distance = distance + spacing
    end
  end

  if #markers > 1 then
    markers[#markers].alpha = ((totalPathLength-markers[#markers].dist) / spacing)
  end

  self:increaseDecalPool(#markers)

  for i, m in ipairs(markers) do
    data = decals[i]
    data.position = m.pos
    data.forwardVec = m.fwd
    data.color.a = m.alpha
  end

  Engine.Render.DynamicDecalMgr.addDecals(decals, #markers)
end

return _flowgraph_createNode(C)
