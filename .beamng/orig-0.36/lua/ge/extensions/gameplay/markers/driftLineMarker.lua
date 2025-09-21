-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

local idCounter = 0

function C:init()
  self.id = idCounter
  idCounter = idCounter + 1
end

function C:createObjects()

end

function C:clearObjects()
  for _, name in ipairs(self.cluster.markerObjects or {}) do
    local obj = scenetree.findObject(name)
    if obj then
      obj:setHidden(true)
      --log("I","","Hidden: " .. name)
    end
  end
end

function C:hide()
end

function C:show()

end

function C:update()
end
--[[
local colorOut, colorIn = ColorF(0.91,0.05,0.48,0.2), ColorF(0.09,0.90,0.48,0.2)
local finishColor = ColorF(0,1,0,0.5)
local columnObj, iconRendererObj
local missionColorI = ColorI(255,255,255,255)
local tmpVec = vec3()
function C:update(data)
  debugDrawer:drawSphere(self.pos, self.radius, self._inside and colorIn or colorOut)
  if self.mode == "endLine" then
    debugDrawer:drawSphere(self.pos, self.radius/2, finishColor)
  debugDrawer:drawLine(self.pos, data.vehPos, ColorF(0.91,0.05,0.48,0.5))
  simpleDebugText3d(String(string.format("%0.2fm", data.vehPos:distance(self.pos))), (self.pos + data.vehPos)*0.5)
  end
end
]]


function C:setup(cluster)
  self.cluster = cluster
  self.pos = cluster.pos
  self.radius = cluster.radius
  self.lineId = cluster.lineId
  self._inside = nil
  for _, name in ipairs(self.cluster.markerObjects or {}) do
    local obj = scenetree.findObject(name)
    if obj then
      obj:setHidden(false)
      --log("I","","Show: " .. name)
    end
  end
end

-- Interactivity
function C:interactWhileMoving(interactData)
  self._inside = false
  if interactData.isWalking then return end
  if interactData.vehPos:squaredDistance(self.pos) <= self.radius*self.radius then
    self._inside = true
  end

  if self._inside then
    local drift = gameplay_drift_freeroam_driftSpots
    drift.detectStart(self.lineId)
  end
end


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
      id = 'driftLineMarker#'..poi.id,

      isDriftLineMarker = true,

      pos = poi.markerInfo.driftLineMarker.pos,
      radius = poi.markerInfo.driftLineMarker.radius,
      lineId = poi.lineId,
      startDir = poi.markerInfo.driftLineMarker.startDir,
      markerObjects = poi.markerInfo.driftLineMarker.markerObjects,

      visibilityPos = poi.markerInfo.driftLineMarker.pos,
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
