-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
local basePrefix = "base_marker_"
local sidesPrefix = "cylinder_marker_"
local distantPrefix = "distant_marker_"
local baseShape = "art/shapes/interface/sideMarker/checkpoint_curve_base.dae"
local sideShape =  "art/shapes/interface/single_faded_column.dae"

local base, left, right

local modeInfos = {
  default = {
    color = {1, 0.07, 0},
    baseColor = {1, 1, 1},
  },
  next = {
    color = {0.0, 0.0, 0.0},
    baseColor = {0, 0, 0},
  },
  start = {
    color = {0.4, 1, 0.2},
    baseColor = {1, 1, 1},
  },
  lap = {
    color = {0.4, 1, 0.2},
    baseColor = {1, 1, 1},
  },
  recovery = {
    color = {1, 0.85, 0},
    baseColor = {1, 1, 1},
  },
  final = {
    color = {0.1, 0.3, 1},
    baseColor = {1, 1, 1},
  },
  branch = {
    color = {1, 0.6, 0},
    baseColor = {1, 1, 1},
  },
  hidden = {
    color = {0, 0, 0},
    baseColor = {0, 0, 0},
  }
}

local zVec = vec3(0,0,1)

local fadeNear = 1
local fadeFar = 50

local function inverseLerp(min, max, value)
 if math.abs(max - min) < 1e-30 then return min end
 return (value - min) / (max - min)
end

-- todo: replace this by a HSV-lerp if blending with non-gray colors
local lerpedColor = vec3()
local function lerpColor(a,b,t)
  lerpedColor:set(lerp(a[1],b[1],t), lerp(a[2],b[2],t), lerp(a[3],b[3],t))
  return lerpedColor
end

-- called when this object is created. initialize variables here (but dont spawn objects)
function C:init(id)
  self.id = id
  self.visible = false

  self.pos = nil
  self.radius = nil
  self.color = nil
  self.currentColor = ColorF(1,1,1,1):asLinear4F()
  self.colorBase = ColorF(1,1,1,1):asLinear4F()

  self.colorTimer = 0
  self.colorLerpDuration = 0.3
  self.minAlpha = 0.25

  self.fadeNear = fadeNear
  self.fadeFar = fadeFar

  self.modeInfos = deepcopy(modeInfos)

  self:clearMarkers()
  self.distant = nil
  self.normal = nil

  self.mode = 'hidden'
  self.oldMode = 'hidden'
end

-- called every frame to update the visuals.
local playerPosition = vec3(0,0,0)
local scale = vec3()
local markerOffset = vec3(0,0,-10)
function C:update(dt, dtSim)
  self.colorTimer = self.colorTimer + dt
  if self.colorTimer >= self.colorLerpDuration then
    if self.mode == 'hidden' then
      self:hide()
    end
  end
  if not self.visible then return end

  playerPosition:set(core_camera.getPosition())

  local distanceFromMarker = self.pos:distance(playerPosition)

  local t = clamp(self.colorTimer / self.colorLerpDuration,0,1)
  local color = lerpColor(self.modeInfos[self.oldMode or 'default'].color, self.modeInfos[self.mode or 'default'].color, t)
  self.currentColor.x = color.x
  self.currentColor.y = color.y
  self.currentColor.z = color.z
  self.currentColor.w = clamp(inverseLerp(self.fadeNear,self.fadeFar,distanceFromMarker),0,0.75) + clamp(self.minAlpha, 0, 0.25)

  local normal = self.normal and self.normal or (playerPosition-self.pos):normalized()
  local rot = quatFromDir(normal:z0()):toTorqueQuat()
  if distanceFromMarker > self.radius*1.5 then
    self.side = normal:cross(zVec)
  end
  local baseHeight = clamp(inverseLerp(10,40,distanceFromMarker),self.radius,self.radius*3)
  base = scenetree.findObjectById(self.baseId)
  if base then
    local color = lerpColor(self.modeInfos[self.oldMode or 'default'].baseColor, self.modeInfos[self.mode or 'default'].baseColor, t)
    self.colorBase.x = color.x
    self.colorBase.y = color.y
    self.colorBase.z = color.z
    self.colorBase.w = self.currentColor.w * 0.5

    base.instanceColor = self.colorBase
    if distanceFromMarker > self.radius*1.5 then
      base:setField('rotation', 0, rot.x .. ' ' .. rot.y .. ' ' .. rot.z .. ' ' .. rot.w)
    end
    base:setScale(vec3(self.radius, self.radius, baseHeight))
    base:updateInstanceRenderData()
  end
  local sideRadius = math.max(0.125, distanceFromMarker*0.03)
  local sideHeight = clamp(inverseLerp(60,180,distanceFromMarker),0,20)+1 +clamp(inverseLerp(1800,2040,distanceFromMarker),0,20)
  --debugDrawer:drawTextAdvanced(self.pos, String(string.format("%0.2f -> %0.2f / %0.2f / %0.2f", distanceFromMarker, sideRadius, sideHeight, baseHeight)), ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
  left = scenetree.findObjectById(self.leftId)
  if left then
    left.instanceColor = self.currentColor
    left:setPosition(self.pos - self.side * self.radius + markerOffset)
    left:updateInstanceRenderData()
    scale:set(sideRadius, sideRadius, sideHeight)
    left:setScale(scale)
  end
  right = scenetree.findObjectById(self.rightId)
  if right then
    right.instanceColor = self.currentColor
    right:setPosition(self.pos + self.side * self.radius + markerOffset)
    right:updateInstanceRenderData()
    scale:set(sideRadius, sideRadius, sideHeight)
    right:setScale(scale)
  end
end

-- setting it to represent checkpoints. mode can be:
-- default (red, "normal" checkpoint)
-- branch (yellow, for branching paths)
-- next (black, the one after the current checkpoint)
-- lap (green, last cp in non-last lap)
-- finish (blue, last cp in last lap)
-- start (green, first cp when using rolling start)
function C:setToCheckpoint(wp)
  self.pos = vec3(wp.pos)
  self.radius = wp.radius
  self.normal = wp.normal and vec3(wp.normal) or nil
  self.side = (self.normal or vec3(1,0,0)):cross(vec3(0,0,1))

  self.fadeNear = wp.fadeNear or self.fadeNear
  self.fadeFar = wp.fadeFar or self.fadeFar
  self.minAlpha = wp.minAlpha or self.minAlpha
  base = scenetree.findObjectById(self.baseId)
  if base then
    base:setPosition(vec3(self.pos))
    base:setScale(vec3(self.radius, self.radius, self.radius))
  end
  left = scenetree.findObjectById(self.leftId)
  if left then
    left:setPosition(vec3(self.pos - self.side * self.radius))
    left:setScale(vec3(1,1,1))
  end
  right = scenetree.findObjectById(self.rightId)
  if right then
    right:setPosition(vec3(self.pos + self.side * self.radius))
    right:setScale(vec3(1,1,1))
  end
end

function C:setMode(mode)
  if mode ~= 'hidden' then
    self:show()
  end
  self.oldMode = self.mode
  self.mode = mode
  self.colorTimer = 0

  self:update(0,0)
end

-- visibility management
function C:setVisibility(v)
  self.visible = v
  base = scenetree.findObjectById(self.baseId)
  if base then
    base.hidden = not v
  end
  left = scenetree.findObjectById(self.leftId)
  if left then
    left.hidden = not v
  end
  right = scenetree.findObjectById(self.rightId)
  if right then
    right.hidden = not v
  end
end

function C:hide() self:setVisibility(false) end
function C:show() self:setVisibility(true)  end

-- marker management
function C:createObject(shapeName, objectName)
  local marker =  createObject('TSStatic')
  marker:setField('shapeName', 0, shapeName)
  marker:setPosition(vec3(0, 0, 0))
  marker.scale = vec3(1, 1, 1)
  marker:setField('rotation', 0, '1 0 0 0')
  marker.useInstanceRenderData = true
  marker:setField('instanceColor', 0, '1 1 1 1')
  marker:setInternalName('marker')
  marker.canSave = false
  marker.hidden = true
  marker:registerObject(objectName)

  local scenarioObjectsGroup = scenetree.ScenarioObjectsGroup
  if scenarioObjectsGroup then
    scenarioObjectsGroup:addObject(marker)
  end

  return marker
end

-- creates neccesary objects
function C:createMarkers()
  self:clearMarkers()
  self.baseId  = self:createObject(baseShape,basePrefix..self.id):getId()
  self.leftId  = self:createObject(sideShape,sidesPrefix.."left"..self.id):getId()
  self.rightId = self:createObject(sideShape,sidesPrefix.."right"..self.id):getId()
end

function C:clearMarker(id)
  if id then
    local obj = scenetree.findObjectById(id)
    if obj then obj:delete() end
  end
end

-- destorys/cleans up all objects created by this
function C:clearMarkers()
  self:clearMarker(self.baseId)
  self:clearMarker(self.leftId)
  self:clearMarker(self.rightId)
  self.baseId = nil
  self.leftId = nil
  self.rightId = nil
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end