-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local logTag = 'editor_aiViz'
local im = ui_imgui

local upVector = vec3(0, 0, 1)

local colLightGrey = ColorF(0.9, 0.9, 0.9, 0.3)
local linkBaseColor = ColorF(0, 0, 0, 0.7)
local arrowBaseColor = ColorF(0, 0, 0, 1)
local arrowAltColor = ColorF(0.5, 0, 0, 1)
local arrowSize1 = Point2F(1, 1)
local arrowSize2 = Point2F(1, 0)

local laneColor1 = ColorF(1, 0.2, 0.2, 1)
local laneColor2 = ColorF(0.2, 0.4, 1, 1)
local laneSize = Point2F(1, 0.2)

local linkLineColor = ColorF(0, 1, 0, 1)
local linkLineOffset = vec3(0, 0, 0.5)

local maxConnectionRenderDistance = 100

local camPos
local mapNodes = {}
local hoveredNode
local hoveredLink

local quadtree = require('quadtree')
local qtNodes
local updateQt

local drawMode = nil
local drawModes = {
  drivability = false,
  type = false,
  speedLimit = false
}

-- gets the color depending on the datas drivability, from red to green.
local drivabilityColorCache = {}
local function getDrivabilityColor(data)
  if data.drivability == nil then return colLightGrey end
  if data.gatedRoad then return ColorF(1, 0.5, 0.5, 0.5) end
  if not drivabilityColorCache[data.drivability] then
    local rainbow = rainbowColor(50, clamp(data.drivability,0,1)*15, 1)
    drivabilityColorCache[data.drivability] = ColorF(rainbow[1], rainbow[2], rainbow[3],0.5)
  end
  return drivabilityColorCache[data.drivability]
end

-- gets the color depending on the datas type.
local typeColors = {
  public = ColorF(0, 0, 1, 0.5),
  private = ColorF(1, 0, 0, 0.5)
}
local function getTypeColor(data)
  if data.type == nil then return colLightGrey end
  return typeColors[data.type or 'public']
end

-- gets the color depending on the datas speedLimit, as a gradient from red over green to blue.
local speedLimitCache = {}
local function getSpeedLimitColor(data)
  if not speedLimitCache[data.speedLimit] then
    local rainbow = rainbowColor(50, clamp(data.speedLimit, 0, 36), 1)
    local clr = ColorF(rainbow[1], rainbow[2], rainbow[3],0.5)
    speedLimitCache[data.speedLimit] = clr
  end
  return speedLimitCache[data.speedLimit]
end

-- selects the appropriate color-selecting function for a link.
local drawFunctions = {
  drivability = getDrivabilityColor,
  type = getTypeColor,
  speedLimit = getSpeedLimitColor
}
local function getLinkColor(data)
  if drawMode and drawFunctions[drawMode] then return drawFunctions[drawMode](data) or linkBaseColor end
  return linkBaseColor
end

-- returns the link-text that should be displayed up-close.
local linkTextFunctions = {
  drivability = function(data) if data.drivability == nil then return "-" else return string.format("%g", data.drivability) end end,
  type = function(data) return data.type or "-" end,
  speedLimit = function(data) if data.speedLimit == nil then return "-" else return string.format("%g m/s", data.speedLimit) end end
}

local function getLinkText(data)
  if drawMode then
    return linkTextFunctions[drawMode] and linkTextFunctions[drawMode](data) or "-"
  else
    return ""
  end
end

local distances = {}
local distancesOfLinks = {}

local function getNodeWithSmallestDist(nodes)
  local min = math.huge
  local res
  for nid, _ in pairs(nodes) do
    if distances[nid] < min or not res then
      res = nid
      min = distances[nid]
    end
  end
  return res
end

local function findDistanceFromNode(nid)
  -- using Dijkstra
  -- init
  table.clear(distances)
  table.clear(distancesOfLinks)
  if not nid then return end
  local node = mapNodes[nid]
  local nodesToCheck = deepcopy(mapNodes)

  -- remove all nodes that are too far away
  for otherNid, data in pairs(mapNodes) do
    if data.pos:squaredDistance(node.pos) > square(maxConnectionRenderDistance) then
      -- remove the links
      for otherNid2, _ in pairs(data.links) do
        if nodesToCheck[otherNid2] then
          nodesToCheck[otherNid2].links[otherNid] = nil
        end
      end
      -- remove the node
      nodesToCheck[otherNid] = nil
    else
      distances[otherNid] = math.huge
    end
  end
  distances[nid] = 0

  -- actual algo
  while not tableIsEmpty(nodesToCheck) do
    local nextNid = getNodeWithSmallestDist(nodesToCheck)
    local nextNodeData = nodesToCheck[nextNid]
    nodesToCheck[nextNid] = nil

    for neighbor, otherLinkData in pairs(nextNodeData.links) do
      if nodesToCheck[neighbor] and otherLinkData.inNode == nextNid then
        local distOfPathThroughNextNode = distances[nextNid] + nextNodeData.pos:distance(nodesToCheck[neighbor].pos)
        if distOfPathThroughNextNode < distances[neighbor] then
          distances[neighbor] = distOfPathThroughNextNode
          if not distancesOfLinks[neighbor] then
            distancesOfLinks[neighbor] = {}
          end
          if not distancesOfLinks[nextNid] then
            distancesOfLinks[nextNid] = {}
          end
          distancesOfLinks[neighbor][nextNid] = distOfPathThroughNextNode
          distancesOfLinks[nextNid][neighbor] = distOfPathThroughNextNode
        end
      end
    end
  end

  -- Add distances to the links that are close enough, but not on any shortest route from any node
  for otherNid, data in pairs(mapNodes) do
    if distancesOfLinks[otherNid] then
      for neighbor, linkData in pairs(data.links) do
        if distancesOfLinks[neighbor] and not distancesOfLinks[otherNid][neighbor] then
          local newDist = (distances[otherNid] + distances[neighbor]) / 2 + data.pos:distance(mapNodes[neighbor].pos)
          distancesOfLinks[otherNid][neighbor] = newDist
          distancesOfLinks[neighbor][otherNid] = newDist
        end
      end
    end
  end
end

local function shouldLineBeDrawn(nid1, nid2)
  return distancesOfLinks[nid1] and distancesOfLinks[nid1][nid2] and distancesOfLinks[nid1][nid2] < maxConnectionRenderDistance
end

-- returns the displacement value of the lane (negative = left, positive = right)
local function getLaneOffset(nid1, nid2, width, lane, laneCount)
  local link = mapNodes[nid1].links[nid2] or mapNodes[nid2].links[nid1]
  if link.inNode == nid2 then
    nid1, nid2 = nid2, nid1
  end

  return (lane - laneCount / 2 - 0.5) * (width / laneCount)
end

local function drawNode(nid, n)
  local color
  if hoveredNode == nid then
    color = ColorF(1, 1, 0, 1)
  elseif distances[nid] and distances[nid] < maxConnectionRenderDistance then
    color = linkLineColor
  else
    color = ColorF(0.9, 0.9, 0.9, 0.3)
  end
  debugDrawer:drawSphere(n.pos, n.radius, color)

  local camNodeSqDist = camPos:squaredDistance(n.pos)
  if camNodeSqDist < square(clamp(editor.getPreference("gizmos.visualization.visualizationDrawDistance") * 0.5, 50, 250)) then
    debugDrawer:drawText(n.pos, String(tostring(nid)), linkBaseColor)
  end

  -- draw edges
  -- map.nodes is single sided i.e. edge between nodes a, b is either in map.nodes[a].links[b] or map.nodes[b].links[a] but not both
  for lid, data in pairs(n.links) do
    if mapNodes[lid] then
      local lidPos = mapNodes[lid].pos

      local linkColor = getLinkColor(data)
      if hoveredLink and hoveredLink.nid1 == nid and hoveredLink.nid2 == lid then
        linkColor = ColorF(1, 1, 0, 1)
      end
      debugDrawer:drawSquarePrism(n.pos, lidPos, Point2F(0.6, n.radius*2), Point2F(0.6, mapNodes[lid].radius*2), linkColor)
      if shouldLineBeDrawn(nid, lid) then
        debugDrawer:drawCylinder(n.pos + linkLineOffset, lidPos + linkLineOffset, 0.3, linkLineColor)
      end
      local inNodePos = mapNodes[data.inNode].pos
      local outNodePos
      if data.outNode then
        outNodePos = mapNodes[data.outNode].pos
      else
        outNodePos = data.inNode == nid and mapNodes[lid].pos or n.pos
      end
      local edgeDirVec = outNodePos - inNodePos
      local edgeLength = edgeDirVec:length()
      edgeDirVec:setScaled(1 / (edgeLength + 1e-30))

      if data.lanes then -- if lane data is available
        local strLen = 1 -- number of characters representing a single lane in the lane string
        local laneCount = #data.lanes / strLen
        local inNodeRad, outNodeRad = mapNodes[data.inNode].radius, (data.outNode and mapNodes[data.outNode].radius or (data.inNode == nid and mapNodes[lid].radius or n.radius))

        -- calculate arrow spacing to draw lane direction indicator arrows
        local arrowLength = 2
        local usableEdgeLength = edgeLength - arrowLength
        local k = math.max(1, math.floor(usableEdgeLength / 30) - 1) -- number of arrows per lane (30m between arrows). skip first and last.
        local dispVec = (usableEdgeLength / (k + 1)) * edgeDirVec
        local arrowLengthVec = arrowLength * edgeDirVec

        local right1 = edgeDirVec:cross(mapNodes[data.inNode].normal)

        for i = 1, laneCount do -- draw lanes
          -- Draw arrows indicating lane direction
          local offset1 = getLaneOffset(data.inNode, data.outNode or (data.inNode == nid and lid or nid), math.min(inNodeRad, outNodeRad) * 2, i, laneCount)
          local laneDir = data.lanes:byte(i) == 43 --> ascii code for '+'
          color = laneDir and laneColor2 or laneColor1
          local tailPos = inNodePos + right1 * offset1
          local tipPos  = vec3()
          for j = 1, k do
            tailPos:setAdd(dispVec)
            tipPos:setAdd2(tailPos, arrowLengthVec)
            debugDrawer:drawSquarePrism(tailPos, tipPos, laneDir and arrowSize1 or arrowSize2, laneDir and arrowSize2 or arrowSize1, color)
          end
        end
      else
        if data.oneWay then
          local edgeProgress = 0.5
          while edgeProgress <= edgeLength do
            color = core_camera.getForward():dot(edgeDirVec) >= 0 and arrowBaseColor or arrowAltColor
            debugDrawer:drawSquarePrism((inNodePos + edgeProgress * edgeDirVec), (inNodePos + (edgeProgress + 2) * edgeDirVec), arrowSize1, arrowSize2, color)
            edgeProgress = edgeProgress + 15
          end
        end
      end
    end
  end
end

local function staticRayCast()
  if core_forest.getForestObject() then core_forest.getForestObject():disableCollision() end
  local rayCast = cameraMouseRayCast()
  if core_forest.getForestObject() then core_forest.getForestObject():enableCollision() end

  return rayCast
end

local function onNavgraphReloaded()
  mapNodes = {}
  updateQt = true
end

local function onEditorGui()
  local drawEnabled = drawMode ~= nil
  if drawEnabled then
    if tableIsEmpty(mapNodes) then
      mapNodes = map.getMap().nodes or {}
    end

    if not qtNodes or updateQt then
      qtNodes = quadtree.newQuadtree()
      if mapNodes then
        for nid, n in pairs(mapNodes) do
          local nPos = n.pos
          local radius = n.radius
          n.normal = n.normal or map.surfaceNormal(nPos, radius * 0.5)
          qtNodes:preLoad(nid, quadtree.pointBBox(nPos.x, nPos.y, radius))
        end
      end
      qtNodes:build()
      updateQt = false
    end
    camPos = core_camera.getPosition()
  end

  if drawEnabled then
    local rayCast = staticRayCast()

    -- Get hovered node/link for highlighting
    hoveredNode = nil
    hoveredLink = nil
    if rayCast and not (im.IsAnyItemHovered() or im.IsWindowHovered(im.HoveredFlags_AnyWindow)) then
      local minHitDist = rayCast.distance
      local ray = getCameraMouseRay()
      local rayDir = ray.dir
      for nid in qtNodes:query(quadtree.pointBBox(camPos.x, camPos.y, 200)) do
        local node = mapNodes[nid]
        local minSphereHitDist, _ = intersectsRay_Sphere(ray.pos, rayDir, node.pos, node.radius)
        if minSphereHitDist and minSphereHitDist < minHitDist then
          hoveredNode = nid
          hoveredLink = nil
          minHitDist = minSphereHitDist
        end

        for otherNid, link in pairs(node.links) do
          if link.inNode == nid then
            local linkDir = mapNodes[otherNid].pos - node.pos
            local perpendicularDir = linkDir:normalized():cross(upVector)
            local p1 = node.pos + perpendicularDir * node.radius + vec3(0, 0, 0.5)
            local p2 = node.pos + -perpendicularDir * node.radius + vec3(0, 0, 0.5)
            local p3 = mapNodes[otherNid].pos + -perpendicularDir * mapNodes[otherNid].radius + vec3(0, 0, 0.5)
            local p4 = mapNodes[otherNid].pos + perpendicularDir * mapNodes[otherNid].radius + vec3(0, 0, 0.5)
            local hitDist1 = intersectsRay_Triangle(camPos, rayDir, p1, p2, p3)
            local hitDist2 = intersectsRay_Triangle(camPos, rayDir, p1, p3, p4)
            local hitDist = math.min(hitDist1, hitDist2)
            if hitDist < minHitDist then
              minHitDist = hitDist
              hoveredLink = {}
              hoveredLink["nid1"] = nid
              hoveredLink["nid2"] = otherNid
              hoveredNode = nil
            end
          end
        end
      end
    end

    -- Draw nodes
    if qtNodes then
      for nid in qtNodes:query(quadtree.pointBBox(camPos.x, camPos.y, editor.getPreference("gizmos.visualization.visualizationDrawDistance"))) do
        local node = mapNodes[nid]
        if node then
          drawNode(nid, node)
        end
      end
    end
  end
end

local function computeDrawMode()
  drawMode = nil
  for k, v in pairs(drawModes) do
    if v then drawMode = k end
  end
end

local function enableDrawMode(mode, enabled)
  if drawModes[mode] == nil then log('E','',"Drawmode " .. dumps(mode) .. " does not exist for aiViz!") return end
  -- disable all other drawmodes
  if enabled then
    for k, v in pairs(drawModes) do
      drawModes[k] = false
    end
  end
  drawModes[mode] = enabled
  computeDrawMode()
end

local function getDrawMode()
  return drawMode
end

M.onEditorGui = onEditorGui
M.onNavgraphReloaded = onNavgraphReloaded
M.enableDrawMode = enableDrawMode
M.getDrawMode = getDrawMode

return M