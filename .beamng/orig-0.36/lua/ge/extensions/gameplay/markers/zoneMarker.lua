-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

function C:init() end

function C:update(data)
end

function C:setup(cluster)
  self.zones = cluster.zones
  self.cluster = cluster
  self.pos2d = vec3(cluster.visibilityPos.x, cluster.visibilityPos.y, 0)
  self.radius = cluster.visibilityRadius
end

function C:setHidden(value) end
function C:createObjects() end
function C:hide() end
function C:show() end
function C:instantFade(visible) end
function C:clearObjects() end

-- Interactivity
function C:interactInPlayMode(interactData, interactableElements)
  if interactData.isWalking then return end
  local inside = false
  if interactData.vehPos2d:distance(self.pos2d) <= self.radius+2 then
    for _, z in ipairs(self.zones) do
      if (  z:containsPoint2D(interactData.bbPoints[1])
        and z:containsPoint2D(interactData.bbPoints[4])
        and z:containsPoint2D(interactData.bbPoints[5])
        and z:containsPoint2D(interactData.bbPoints[8])) then
        inside = true
      end
    end
  end

  if interactData.canInteract and inside then
    for _, elem in ipairs(self.cluster.elemData) do
      table.insert(interactableElements, elem)
    end
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
      id = 'zoneMarker#'..poi.id,
      --containedIds = {poi.id},
      zones = poi.markerInfo.zoneMarker.zones,
      visibilityPos = poi.markerInfo.zoneMarker.pos,
      visibilityRadius = poi.markerInfo.zoneMarker.radius,
      elemData = {poi.data},
      create = create,
    }
    table.insert(allClusters, cluster)
  end
end

return {
  create = create,
  cluster = cluster
}
