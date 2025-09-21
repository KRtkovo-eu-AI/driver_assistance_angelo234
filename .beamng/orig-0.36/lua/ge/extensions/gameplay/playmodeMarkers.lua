-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {"gameplay_rawPois"}
local playmodeClusters = nil
local markersByClusterId = {}
local playmodeQt = nil

-- this list should be built dynamicly
local playmodeMarkerTypeNames = {
  missionMarker = true,
  parkingMarker = true,
  zoneMarker = true,
  walkingMarker = true,
  gasStationMarker = true,
  driftLineMarker = true,
  invisibleTrigger = true,
}

local function idSort(a,b) return a.id<b.id end

local function clearPlaymodeClusters()
  if playmodeClusters or playmodeQt or next(markersByClusterId) then
    log("D","","Playmode clusters and markers cleared")
  end
  playmodeClusters = nil
  playmodeQt = nil
  for _, marker in pairs(markersByClusterId) do
    marker:clearObjects()
  end
  table.clear(markersByClusterId)
end

local function sanitizeCluster(cluster)
  for _, key in ipairs({"id","visibilityPos","visibilityRadius"}) do
    if not cluster[key] then log("E","","No ".. key .. " for cluster " .. (cluster.id or dumps(cluster)))
    end
  end
end

local function clusterPlaymodePois(pois)
  -- first, sort/duplicate all pois by their markerTypes
  local poisByMarkerType = {}
  for markerType, _ in pairs(playmodeMarkerTypeNames) do
    poisByMarkerType[markerType] = {}
    for _, poi in ipairs(pois) do
      if poi.markerInfo[markerType] then
        table.insert(poisByMarkerType[markerType], poi)
      end
    end
  end
  local allClusters = {}
  -- let each markertype cluster their own markers in their own way
  for markerType, _ in pairs(playmodeMarkerTypeNames) do
    local factory = require('lua/ge/extensions/gameplay/markers/'..markerType)
    factory.cluster(poisByMarkerType[markerType], allClusters)
  end

  table.sort(allClusters, idSort)

  -- check all clusters if they have required fields
  for _, cluster in ipairs(allClusters) do
    sanitizeCluster(cluster)
  end
  return allClusters
end

local clusterGeneration = -1
local function checkGeneration()
  if clusterGeneration < gameplay_rawPois.getRawPoiGeneration() then
    clearPlaymodeClusters()
    clusterGeneration = gameplay_rawPois.getRawPoiGeneration()
    log("D","","Playmode markers/clusters cleared. New Generation: " .. clusterGeneration)
  end
end
local function getPlaymodeClusters()
  checkGeneration()
  if not playmodeClusters then
    local pois, rawPoiGeneration = gameplay_rawPois.getRawPoiListByLevel(getCurrentLevelIdentifier())

    playmodeClusters = clusterPlaymodePois(pois)
  end
  return playmodeClusters
end

local quadtree = require('quadtree') -- change to KD Tree?
local function getPlaymodeClustersAsQuadtree()
  checkGeneration()
  if not playmodeQt then
    playmodeQt = quadtree.newQuadtree()
    for _, cluster in ipairs(getPlaymodeClusters()) do
      playmodeQt:preLoad(cluster.id, quadtree.pointBBox(cluster.visibilityPos.x, cluster.visibilityPos.y, cluster.visibilityRadius))
    end
    playmodeQt:build()
  end
  return playmodeQt
end
M.getPlaymodeClusters = getPlaymodeClusters
M.getPlaymodeClustersAsQuadtree = getPlaymodeClustersAsQuadtree

local function getMarkerForCluster(cluster)
  checkGeneration()
  if not markersByClusterId[cluster.id] then
    local marker = cluster.create()
    marker:createObjects()
    marker:setup(cluster)
    markersByClusterId[cluster.id] = marker
  end
  return markersByClusterId[cluster.id]
end
M.getMarkerForCluster = getMarkerForCluster

M.onClientEndMission = clearPlaymodeClusters
M.onSerialize = clearPlaymodeClusters
M.clear = clearPlaymodeClusters

return M
