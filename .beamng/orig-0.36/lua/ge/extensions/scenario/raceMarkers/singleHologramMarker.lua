-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
local columnPrefix = "column_marker_"
local columnShape = "art/shapes/interface/s_single_faded_column_rect.dae"
local centerColumn

local modeInfos = {
  default = {
    columnColor = {0, 0.4, 1},
    alpha = 1,
    scalarThin = 1,
  },
  next = {
    columnColor = {0.0, 0.0, 0.0},
    alpha = 1,
    scalarThin = 0.75,
  },
  start = {
    columnColor = {0.4, 1, 0.2},
    alpha = 1,
    scalarThin = 1,
  },
  lap = {
    columnColor = {0.4, 1, 0.2},
    alpha = 1,
    scalarThin = 1,
  },
  recovery = {
    columnColor = {1, 0.85, 0},
    alpha = 1,
    scalarThin = 1,
  },
  final = {
    columnColor = {0.1, 0.3, 1},
    alpha = 1,
    scalarThin = 1,
  },
  branch = {
    columnColor = {1, 0.6, 0},
    alpha = 1,
    scalarThin = 1,
  },
  hidden = {
    columnColor = {1, 1, 1},
    alpha = 1,
    scalarThin = 0,
  }
}

local zVec = vec3(0,0,1)
local fadeNear = 50
local fadeFar = 80
local bobAmplitude = 0.05
local bobFrequency = 0.333
local bobOffset = vec3(0,0,0)
local playerPosition = vec3(0,0,0)
local scale = vec3()
local columnOffset = vec3(0,0,7)

local function inverseLerp(min, max, value)
 if math.abs(max - min) < 1e-30 then return min end
 return (value - min) / (max - min)
end

local lerpedColor = vec3()
local function lerpColor(a,b,t)
  lerpedColor:set(lerp(a[1],b[1],t), lerp(a[2],b[2],t), lerp(a[3],b[3],t))
  return lerpedColor
end

function C:init(id)
  self.id = id
  self.visible = false
  self.pos = nil
  self.radius = nil
  self.columnColor = ColorF(1,1,1,1):asLinear4F()
  self.colorTimer = 0
  self.colorLerpDuration = 0.8
  self.minAlpha = 0.25
  self.fadeNear = fadeNear
  self.fadeFar = fadeFar
  self.modeInfos = deepcopy(modeInfos)
  self:clearMarkers()
  self.distant = nil
  self.normal = nil
  self.mode = 'hidden'
  self.oldMode = 'hidden'
  self.centerPos = nil
end

function C:update(dt, dtSim)
  self.colorTimer = self.colorTimer + dt
  if self.colorTimer >= self.colorLerpDuration then
    if self.mode == 'hidden' then
      self:hide()
    end
  end
  if not self.visible then return end

  local bobTime = self.colorTimer * bobFrequency * math.pi * 2
  bobOffset:set(0, 0, math.sin(bobTime) * bobAmplitude)

  playerPosition:set(core_camera.getPosition())
  local distanceFromMarker = self.pos:distance(playerPosition)

  local t = clamp(self.colorTimer / self.colorLerpDuration,0,1)
  local alpha = lerp(self.modeInfos[self.oldMode or 'default'].alpha, self.modeInfos[self.mode or 'default'].alpha, t)
  local color = lerpColor(self.modeInfos[self.oldMode or 'default'].columnColor, self.modeInfos[self.mode or 'default'].columnColor, t)

  self.columnColor.x = color.x
  self.columnColor.y = color.y
  self.columnColor.z = color.z
  self.columnColor.w = clamp(inverseLerp(60,120,distanceFromMarker),0.15,1) * alpha

  local scalarThin = lerp(self.modeInfos[self.oldMode or 'default'].scalarThin, self.modeInfos[self.mode or 'default'].scalarThin, t)
  local sideHeight = clamp(inverseLerp(60,180,distanceFromMarker),0,20)+1 +clamp(inverseLerp(1800,2040,distanceFromMarker),0,20)

  centerColumn = scenetree.findObjectById(self.columnId)
  if centerColumn then
    centerColumn.instanceColor = self.columnColor
    centerColumn:setPosition(self.centerPos + columnOffset + bobOffset)
    scale:set(scalarThin, scalarThin, sideHeight)
    centerColumn:setScale(scale)
    centerColumn:updateInstanceRenderData()
  end
end

function C:setToCheckpoint(wp)
  self.pos = vec3(wp.pos)
  self.radius = wp.radius
  self.normal = wp.normal and vec3(wp.normal) or vec3(1,1,0):normalized()
  self.side = (self.normal or vec3(1,0,0)):cross(vec3(0,0,1))

  self.fadeNear = wp.fadeNear or self.fadeNear
  self.fadeFar = wp.fadeFar or self.fadeFar
  self.minAlpha = wp.minAlpha or self.minAlpha

  local rot = quatFromDir(self.normal:z0())
  local up = vec3(0,0,1)
  local rayLengthUp, rayLengthDown = 6, -25

  if core_forest.getForestObject() then core_forest.getForestObject():disableCollision() end
  local hit = Engine.castRay(self.pos+up*rayLengthUp, self.pos+up*rayLengthDown, true, false)
  if core_forest.getForestObject() then core_forest.getForestObject():enableCollision() end

  self.centerPos = hit and vec3(hit.pt) or self.pos
  if self.pos.z - self.centerPos.z > 1 then self.centerPos.z = self.pos.z end

  local rotQuat = rot:toTorqueQuat()
  local rotStr = rotQuat.x .. ' ' .. rotQuat.y .. ' ' .. rotQuat.z .. ' ' .. rotQuat.w

  centerColumn = scenetree.findObjectById(self.columnId)
  if centerColumn then
    centerColumn:setPosition(self.centerPos)
    centerColumn:setField('rotation', 0, rotStr)
    centerColumn:setScale(vec3(1,1,1))
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

function C:setVisibility(v)
  self.visible = v
  centerColumn = scenetree.findObjectById(self.columnId)
  if centerColumn then
    centerColumn.hidden = not v
  end
end

function C:hide() self:setVisibility(false) end
function C:show() self:setVisibility(true)  end

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

function C:createMarkers()
  self:clearMarkers()
  self.columnId = self:createObject(columnShape, columnPrefix..self.id):getId()
end

function C:clearMarker(id)
  if id then
    local obj = scenetree.findObjectById(id)
    if obj then obj:delete() end
  end
end

function C:clearMarkers()
  self:clearMarker(self.columnId)
  self.columnId = nil
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end