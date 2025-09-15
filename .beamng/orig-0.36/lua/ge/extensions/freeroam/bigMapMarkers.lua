-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}


local updateData = {}
local decals = {}
local visibleIds, visibleIdsSorted = {}, {}

local flatPoiList, poisById, poiIdList = nil, nil, nil
local clusterSettingsCounter = 0
local clusterSettingsById = {}
local currentClusterSettingsId = nil
local markersByClusterId = {}

local radiusStep = 5
local clusterSettingsById = {}
local function setupFilter(validIds, radius)
  local elementsInFilter = {}

  if radius then
    radius = round(radius/radiusStep) * radiusStep
  else
    radius = 20
  end

  local settingsId = radius.."#-"
  local validIdsLookup = {}
  table.sort(validIds)
  for _, e in ipairs(validIds) do
    settingsId = settingsId .. e
    validIdsLookup[e] = true
  end
  clusterSettingsCounter = clusterSettingsCounter + 1
  local clusterSettings = {radius = radius, id = settingsId, validIdsLookup = validIdsLookup, markerPrefix = clusterSettingsCounter}
  clusterSettingsById[clusterSettings.id] = clusterSettings
  currentClusterSettingsId = clusterSettings.id
end

local nextMarkerFullAlpha = false
local function displayBigMapMarkers(dtReal)
  profilerPopEvent("BigMapMarkers parkingSpeedFactor")
  -- put reference for icon manager in
  updateData.dt = dtReal
  updateData.bigmapTransitionActive = freeroam_bigMapMode.isTransitionActive()
  updateData.camPos = core_camera.getPosition()

  local clusterSettingsIdsSorted = tableKeysSorted(clusterSettingsById)
  --print("Begin Update")
  for _, csId in ipairs(clusterSettingsIdsSorted) do
    --print(csId)
    local isActiveSettings = csId == currentClusterSettingsId
    local clusters = M.getAllClustersBySettings(csId)
    for _, cluster in ipairs(clusters) do
      local marker = M.getClusterMarker(cluster)
      if marker then
        -- Check if the marker should be visible
        if isActiveSettings then
          if nextMarkerFullAlpha then
            marker:setFullAlphaInstant()
          end
          marker:show()
        else
          marker:hide()
        end
        --print("updating: " .. marker.id .. " ("..marker.cluster.id..")")
        marker:update(updateData)
      end
    end
  end
  nextMarkerFullAlpha = false
end


local yVector = vec3(0,1,0)
local function pointRayDistance(point, rayPos, rayDir)
  return (point - rayPos):cross(rayDir):length() / rayDir:length()
end

local function handleMouse(camMode, uiPopupOpen, mouseMoved, poiIsSelected)

  -- disable hovering when in "controller mode" and there is already a POI selected
  if not mouseMoved and poiIsSelected then return end

  local clusterIconRenderer = scenetree.findObject("markerIconRenderer")
  if not clusterIconRenderer then return end
  local ray
  if mouseMoved then
    ray = getCameraMouseRay()
  else
    local camDir = core_camera.getQuat() * yVector
    ray = {pos = core_camera.getPosition(), dir = camDir}
  end
  for i, cluster in ipairs(M.getAllClustersBySettings(currentClusterSettingsId)) do
    local iconInfo = clusterIconRenderer:getIconByName(cluster.id .. "bigMap")

    if iconInfo then
      local iconPos = iconInfo.worldPosition
      local sphereRadius = iconPos:distance(core_camera.getPosition()) * 0.0006 * camMode.manualzoom.fov
      if not uiPopupOpen and pointRayDistance(iconPos, ray.pos, ray.dir) <= sphereRadius then
        return cluster.containedIds[1]
        --local marker = M.getClusterMarker(cluster)
        --if marker.visibleInBigmap then
        --  marker.hovered = true
        --return marker
        --end
      end
    end
  end
end

local function getIdsFromHoveredPoiId(id)
  for i, cluster in ipairs(M.getAllClustersBySettings(currentClusterSettingsId)) do
    if cluster.containedIdsLookup[id] then
      return cluster.containedIds
    end
  end
end

local bigmapMarkerFactory = require('lua/ge/extensions/gameplay/markers/bigmapMarker')
local function getClusterMarker(cluster)
  if not markersByClusterId[cluster.id] then
    local marker = bigmapMarkerFactory.createMarker()
    marker:setup(cluster)
    markersByClusterId[cluster.id] = marker
  end
  return markersByClusterId[cluster.id]
end
M.displayBigMapMarkers = displayBigMapMarkers
M.handleMouse = handleMouse
M.getIdsFromHoveredPoiId = getIdsFromHoveredPoiId
M.getClusterMarker = getClusterMarker
M.setupFilter = setupFilter

M.buildPoiList = buildPoiList

-- custom clustering
local clustersBySettings = {}

local function hideMarkers()
  for _, marker in pairs(markersByClusterId) do
    marker:hide()
  end
end
M.hideMarkers = hideMarkers
local function clearMarkers()
  for _, marker in pairs(markersByClusterId) do
    marker:clearObjects()
  end
  table.clear(markersByClusterId)
end
M.clearMarkers = clearMarkers

local quadtree = require('quadtree') -- change to KD Tree?
local function idSort(a,b) return a.id < b.id end
local function clusterBySettings(elements, settingsId)
  clustersBySettings[settingsId] = {}
  local settings = clusterSettingsById[settingsId]

  -- filter all pois that have a bigmapMarker representation and are in validIds
  local filteredPois = {}
  --dump(settings.validIdsLookup)
  --local validCopy = deepcopy(settings.validIdsLookup)
  for _, poi in ipairs(elements) do
    if settings.validIdsLookup[poi.id] and poi.markerInfo.bigmapMarker then
      --validCopy[poi.id] = "INCLUDED"
      table.insert(filteredPois, poi)
      --print("ok " .. poi.id)
    else
     --print("No ok " .. poi.id)
    end
    --if settings.validIdsLookup[poi.id] and not poi.markerInfo.bigmapMarker then
    --  validCopy[poi.id] = "Missing bigmapMarker"
    --end

  end
  --dump(validCopy)
  table.sort(filteredPois, idSort)

  -- preload all elements into a qt for quick clustering
  local qt = quadtree.newQuadtree()
  local count = 0
  for i, poi in ipairs(filteredPois) do
    qt:preLoad(i, quadtree.pointBBox(poi.markerInfo.bigmapMarker.pos.x, poi.markerInfo.bigmapMarker.pos.y, settings.radius))
    count = i
  end
  qt:build()

  --go through the list and check for closeness to cluster
  for index = 1, count do
    local cur = filteredPois[index]
    if cur then
      local cluster = {}
      local bmi = cur.markerInfo.bigmapMarker

      -- find all the list that potentially overlap with cur, and get all the ones that actually overlap into cluster list
      for id in qt:query(quadtree.pointBBox(bmi.pos.x, bmi.pos.y, settings.radius)) do
        local candidate = filteredPois[id]

        candidate._qtId = id

        if not bmi.cluster and cur == candidate then
          table.insert(cluster, candidate)
        elseif bmi.cluster and candidate.markerInfo.bigmapMarker.cluster then
          if bmi.pos:squaredDistance(candidate.markerInfo.bigmapMarker.pos) < square(settings.radius) then
            table.insert(cluster, candidate)
          end
        end
      end

      -- remove all the elements in the cluster from the qt and the locations list
      for _, c in ipairs(cluster) do
        qt:remove(c._qtId, filteredPois[c._qtId].markerInfo.bigmapMarker.pos.x, filteredPois[c._qtId].markerInfo.bigmapMarker.pos.y)
        filteredPois[c._qtId] = false
      end

      table.sort(cluster, idSort)
      local cl = bigmapMarkerFactory.merge(cluster, settings.markerPrefix)
      table.insert(clustersBySettings[settingsId], cl)
    end
  end

end

local clusterGeneration = -1
local function getAllClustersBySettings(settingsId)
  local elements, rawPoiGeneration = gameplay_rawPois.getRawPoiListByLevel(getCurrentLevelIdentifier())
  if clusterGeneration < rawPoiGeneration then

    clustersBySettings = {}
    clusterGeneration = rawPoiGeneration
    log("D","","Bigmap markers cleared. New Generation: " .. clusterGeneration)
  end
  if not clustersBySettings[settingsId] then
    clusterBySettings(elements, settingsId)
  end
  return clustersBySettings[settingsId]
end
M.getAllClustersBySettings = getAllClustersBySettings

M.setNextMarkersFullAlphaInstant = function()
  nextMarkerFullAlpha = true
end

return M
