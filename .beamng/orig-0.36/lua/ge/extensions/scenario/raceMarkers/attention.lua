-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
local basePrefix = "base_marker_"
local sidesPrefix = "cylinder_marker_"
local distantPrefix = "distant_marker_"
local sideShape =  "/art/shapes/interface/s_mm_arrow_ribbon_down.dae"

-- used to offset the height of the marker's position when the player, in walking mode, gets close to the marker
local markerPosHeightOffset = 0
local markerMaxPosHeightOffset = 1
local plDistToMarkerOffset = 4
local tmpVec = vec3()

local function inverseLerp(min, max, value)
 if math.abs(max - min) < 1e-30 then return min end
 return (value - min) / (max - min)
end

-- todo: replace this by a HSV-lerp if blending with non-gray colors
local function lerpColor(a,b,t)
  return {lerp(a[1],b[1],t),lerp(a[2],b[2],t),lerp(a[3],b[3],t)}
end

-- called when this object is created. initialize variables here (but dont spawn objects)
function C:init(id)
  self.id = id
  self.visible = false

  self.pos = nil

  self.colorTimer = 0
  self.colorLerpDuration = 0.3

  self.base = nil
  self.sides = nil
  self.distant = nil
  self.normal = nil

  self.mode = 'hidden'
  self.oldMode = 'hidden'

  self.iconPositionSmoother = newTemporalSmoothingNonLinear(10,10)
  self.iconPositionSmoother:set(5)
end


-- called every frame to update the visuals.
function C:update(dt, dtSim)
  self.colorTimer = self.colorTimer + dt
  if self.colorTimer >= self.colorLerpDuration then
    if self.mode == 'hidden' then
      self:hide()
    end
  end
  if not self.visible then return end

  local playerPosition = vec3(0,0,0)
  playerPosition:set(core_camera.getPosition())

  local toMarker2d = (self.pos-playerPosition):z0():normalized()
  local toRight = vec3(toMarker2d.y,-toMarker2d.x)

  local distanceFromMarker = self.pos:distance(playerPosition)

  if gameplay_walk.isWalking() then
    if distanceFromMarker > plDistToMarkerOffset then
      markerPosHeightOffset = self.iconPositionSmoother:get(0, dt)
    else
      markerPosHeightOffset = self.iconPositionSmoother:get(markerMaxPosHeightOffset, dt)
    end
    tmpVec:set(0, 0, markerPosHeightOffset)
  end

  local t = clamp(self.colorTimer / self.colorLerpDuration,0,1)
  local color =  {0, 0.4, 1}
  self.currentColor = ColorF(color[1],color[2],color[3],color[4] or 1)
  self.currentColor.a = 1

  if self.left then
    local fwd = (playerPosition-self.pos)
    local rot = (quatFromDir(fwd:z0())):toTorqueQuat()
    self.left:setField('rotation', 0, rot.x .. ' ' .. rot.y .. ' ' .. rot.z .. ' ' .. rot.w)
    self.left.instanceColor = self.currentColor:asLinear4F()
    self.left.instanceColor1 = ColorF(0,0,0,self.currentColor.a):asLinear4F()
    self.left:setPosition( vec3(0,0,math.sin(os.clock()*3)*(0.2*self.scale.x/1.5))+self.pos+tmpVec)
--      self.left:setField('instanceColor', 1, ""..self.currentColor.r.." "..self.currentColor.g.." "..self.currentColor.b.." "..self.currentColor.a)
--    self.left:setField('instanceColor1', 1, ""..self.currentColor.r.." "..self.currentColor.g.." "..self.currentColor.b.." "..self.currentColor.a)
    self.left:updateInstanceRenderData()
    self.left:setScale(self.scale)
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
  self.scale = vec3(1,1,1) * (wp.radius or 1.5)
  if self.left then
    self.left:setPosition(vec3(0,0,1.75)+self.pos)
    self.left:setScale(vec3(1,1,1))
  end
end

function C:setMode(mode)
  if mode ~= 'hidden' then
    self:show()
  else
    self:hide()
  end
  self.oldMode = self.mode
  self.mode = mode
  self.colorTimer = 0

  self:update(0,0)

end

-- visibility management
function C:setVisibility(v)
  self.visible = v

  if self.left then
    self.left.hidden = not v
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
  self._ids = {}
  if not self.left then
    self.left = self:createObject(sideShape,sidesPrefix.."left"..self.id)
    table.insert(self._ids, self.left:getId())
  end
end

-- destorys/cleans up all objects created by this
function C:clearMarkers()
  for _, id in ipairs(self._ids or {}) do
    local obj = scenetree.findObjectById(id)
    if obj then
      obj:delete()
    end
  end
  self._ids = nil
  self.left = nil
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end