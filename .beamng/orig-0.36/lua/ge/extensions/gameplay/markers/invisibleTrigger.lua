-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

-- called when this object is created. initialize variables here (but dont spawn objects)
function C:init() end

-- called every frame to update the visuals.
function C:update(data)
--[[
  debugDrawer:drawSphere(self.pos, self.radius, ColorF(1,0,0,0.25))
  debugDrawer:drawLine(self.pos, data.vehPos, ColorF(0.91,0.05,0.48,0.5))
  simpleDebugText3d(String(string.format("%0.2fm", data.vehPos:distance(self.pos))), (self.pos + data.vehPos)*0.5)
  ]]
end

-- Interactivity
function C:interactInPlayMode(interactData, interactableElements) end
function C:interactWhileMoving(interactData)
  if interactData.isWalking then return end
  if interactData.vehPos:squaredDistance(self.pos) <= self.radius*self.radius then
    if not self._inside then
      if self.onEnter then
        self.onEnter(interactData)
      end
    end
    if self.onInside then
      self.onInside(interactData)
    end
    self._inside = true
  else
    if self._inside then
      if self.onExit then
        self.onExit(interactData)
      end
    end
    self._inside = false
  end
end

function C:setup(cluster)
  self.cluster = cluster
  self.pos = cluster.pos
  self.radius = cluster.radius
  self.onEnter = cluster.onEnter
  self.onInside = cluster.onInside
end

-- creates neccesary objects
function C:createObjects() end
function C:setHidden(value) end
function C:show() end
function C:hide()
  self._inside = false
end

-- destorys/cleans up all objects created by this
function C:clearObjects() end

function C:instantFade(visible) end

local function create(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end

-- zoneMarkers are not grouped/merged - each poi will be one cluster.
local function cluster(pois, allClusters)
  for _, poi in ipairs(pois) do
    local cluster = {
      id = 'invisibleTrigger#'..poi.id,

      pos = poi.markerInfo.invisibleTrigger.pos,
      radius = poi.markerInfo.invisibleTrigger.radius,
      onEnter = poi.markerInfo.invisibleTrigger.onEnter,
      onInside = poi.markerInfo.invisibleTrigger.onInside,

      visibilityPos = poi.markerInfo.invisibleTrigger.pos,
      visibilityRadius = 0,
      create = create,
    }
    table.insert(allClusters, cluster)
  end
end

return {
  create = create,
  cluster = cluster
}