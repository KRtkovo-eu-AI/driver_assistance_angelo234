-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local logTag = 'editor_slotTrafficEditor'
local actionMapName = "SlotTrafficEditor"
local editModeName = "Edit Slot Traffic"
local ffi = require('ffi')
local roadRiverGui = extensions.editor_roadRiverGui
local im = ui_imgui

local upVector = vec3(0, 0, 1)
local downVector = vec3(0, 0, -1)

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
local roads = {} -- New table to store roads
local navgraphDirty = false
local selectedOnMouseClick

local graphpath = require('graphpath')
local quadtree = require('quadtree')
local gp
local qtNodes
local updateQt

local onlySelectedNode
local hoveredNode
local selectedLink
local hoveredLink
local linkToSnapTo
local heldNode
local selectedRoad

local nodesToLinkTo = {}
local focusPoint
local mouseButtonHeldOnNode = false
local dragMouseStartPos = vec3(0, 0, 0)
local dragStartPosition
local nodeOldPositions = {}
local oldNodeWidth
local tempNodes = {}
local temporaryLink
local addNodeMode

local toolWindowName = 'Slot Traffic Editor'
local roadFilterText = ""

local function setDirty()
  navgraphDirty = true
  editor.setDirty()
end

local function isSelected(item)
  if type(item) == "table" then
    -- item is a link
    local nid1Links = editor.selection.stLink and editor.selection.stLink[item.nid1]
    return nid1Links and nid1Links[item.nid2]
  else
    -- item is a node
    return editor.selection.stNode and editor.selection.stNode[item]
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

local function selectNode(id, addToSelection)
  if id == nil then
    if editor.selection then
      editor.selection.stNode = nil
    end
    table.clear(nodesToLinkTo)
    onlySelectedNode = nil
    findDistanceFromNode(nil)
    return
  end

  if not editor.selection.stNode then editor.selection.stNode = {} end
  if editor.keyModifiers.ctrl or addToSelection then
    if editor.selection.stNode[id] then
      editor.selection.stNode[id] = nil
      table.remove(nodesToLinkTo, tableFindKey(nodesToLinkTo, id))
    else
      editor.selection.stNode[id] = true
    end
  else
    editor.selection.stNode = {}
    table.clear(nodesToLinkTo)
    editor.selection.stNode[id] = true
  end

  if tableSize(editor.selection.stNode) == 1 then
    onlySelectedNode = id
    findDistanceFromNode(nil)
  else
    onlySelectedNode = nil
    findDistanceFromNode(nil)
  end

  if editor.selection.stNode[id] then
    table.insert(nodesToLinkTo, id)
  end
  if tableIsEmpty(editor.selection.stNode) then editor.selection.stNode = nil end
end

local function selectNodes(ids)
  editor.selection.stNode = {}
  table.clear(nodesToLinkTo)
  for _, id in ipairs(ids) do
    selectNode(id, true)
  end
end

local function getOnlySelectedLink()
  local result
  for nid1, nodeLinks in pairs(editor.selection.stLink) do
    for nid2, _ in pairs(nodeLinks) do
      if result then return nil end
      result = {}
      result["nid1"] = nid1
      result["nid2"] = nid2
    end
  end
  return result
end

local function isLinkSelectionEmpty()
  for nid1, nodeLinks in pairs(editor.selection.stLink) do
    for nid2, _ in pairs(nodeLinks) do
      return false
    end
  end
  return true
end

local function selectLink(link)
  if link == nil then
    if editor.selection then
      editor.selection.stLink = nil
    end
    selectedLink = nil
    selectedRoad = nil
    return
  end
  if not editor.selection.stLink then editor.selection.stLink = {} end
  if not editor.selection.stLink[link.nid1] then
    editor.selection.stLink[link.nid1] = {}
  end
  if editor.keyModifiers.ctrl then
    if editor.selection.stLink[link.nid1][link.nid2] then
      editor.selection.stLink[link.nid1][link.nid2] = nil
    else
      editor.selection.stLink[link.nid1][link.nid2] = true
    end
  else
    editor.selection.stLink = {}
    editor.selection.stLink[link.nid1] = {}
    editor.selection.stLink[link.nid1][link.nid2] = true
  end

  selectedLink = getOnlySelectedLink()
  if isLinkSelectionEmpty() then
    editor.selection.stLink = nil
    selectedRoad = nil
  else
    -- Find which road this link belongs to
    for roadId, road in pairs(roads) do
      local controlPoints = road.controlPoints
      for i = 1, #controlPoints - 1 do
        if (controlPoints[i] == link.nid1 and controlPoints[i+1] == link.nid2) or
           (controlPoints[i] == link.nid2 and controlPoints[i+1] == link.nid1) then
          selectedRoad = roadId
          break
        end
      end
      if selectedRoad then break end
    end
  end
end

local function changeLinkDirection(link)
  local linkData = mapNodes[link.nid1].links[link.nid2]
  if linkData.inNode == link.nid1 then
    linkData.inNode = link.nid2
  else
    linkData.inNode = link.nid1
  end
end

-- Change Link Direction
local function changeLinkDirectionActionUndo(actionData)
  changeLinkDirection(actionData.link)
end

local changeLinkDirectionActionRedo = changeLinkDirectionActionUndo

local function setLinkField(link, fieldName, value)
  local linkData = mapNodes[link.nid1].links[link.nid2]
  if linkData then
    linkData[fieldName] = value
  end
end

-- Change Link Field
local function changeLinkFieldActionUndo(actionData)
  setLinkField(actionData.link, actionData.fieldName, actionData.oldValue)
end

local function changeLinkFieldActionRedo(actionData)
  setLinkField(actionData.link, actionData.fieldName, actionData.newValue)
end


local function setNodeField(nid, fieldName, value)
  local node = mapNodes[nid]
  if node then
    node[fieldName] = value
  end
end

-- Change Node Field
local function changeNodeFieldActionUndo(actionData)
  for nid, _ in pairs(actionData.nids) do
    setNodeField(nid, actionData.fieldName, actionData.oldValues[nid])
  end
end

local function changeNodeFieldActionRedo(actionData)
  for nid, _ in pairs(actionData.nids) do
    setNodeField(nid, actionData.fieldName, actionData.newValues[nid])
  end
end

local function setNodePosition(nid, position, safeStartPos)
  if safeStartPos then
    dragStartPosition = dragStartPosition or mapNodes[nid].pos
  end
  mapNodes[nid].pos = position
end

local function getConnectedLinks(nids)
  local result = {}
  for nid, _ in pairs(nids) do
    result[nid] = mapNodes[nid].links
  end
  return result
end

local function areNodesConnected(nid1, nid2)
  return mapNodes[nid1].links[nid2] or mapNodes[nid2].links[nid1]
end

local function getNewNodeName(prefix, idx)
  local nodeName = prefix..idx
  if mapNodes[nodeName] then
    nodeName = nodeName.."_"
    local postfix = 1
    while mapNodes[nodeName..postfix] do
      postfix = postfix + 1
    end
    nodeName = nodeName..postfix
  end
  return nodeName
end

-- TODO
local function addNode(pos, radius, nid)
  nid = nid or getNewNodeName("manual", "")
  mapNodes[nid] = {pos = vec3(pos), radius = radius, normal = map.surfaceNormal(pos, radius * 0.5), links = {}}
  updateQt = true
  return nid
end

local function addLink(nid1, nid2, drivability, speedLimit, lanes)
  if not nid1 or not nid2 then return end
  drivability = drivability or 1
  speedLimit = speedLimit or 50
  local linkInfo = {inNode = nid1, drivability = drivability, speedLimit = speedLimit, lanes = lanes}
  mapNodes[nid2].links[nid1] = linkInfo
  mapNodes[nid1].links[nid2] = linkInfo
  updateQt = true
end

local function deleteLink(nid1, nid2)
  mapNodes[nid2].links[nid1] = nil
  mapNodes[nid1].links[nid2] = nil
  updateQt = true
end

-- TODO
local function deleteNode(nid)
  local nids = {}
  nids[nid] = true
  for nid1, links in pairs(getConnectedLinks(nids)) do
    for nid2, link in pairs(links) do
      deleteLink(nid1, nid2)
    end
  end
  mapNodes[nid] = nil

  updateQt = true
end

-- Add Node
local function addNodeLinkActionUndo(actionData)
  if actionData.linkInfos then
    for nid1, links in pairs(actionData.linkInfos) do
      for nid2, linkInfo in pairs(links) do
        deleteLink(nid1, nid2)
      end
    end
  end
  if actionData.nodeInfos then
    for _, nodeInfo in ipairs(actionData.nodeInfos) do
      deleteNode(nodeInfo.nid)
    end
  end

  selectLink(nil)
  selectNode(nil)
  updateQt = true
end

local function addNodeLinkActionRedo(actionData)
  if actionData.nodeInfos then
    for _, nodeInfo in ipairs(actionData.nodeInfos) do
      addNode(nodeInfo.pos, nodeInfo.radius, nodeInfo.nid)
      mapNodes[nodeInfo.nid].links = nodeInfo.links
    end
  end

  if actionData.linkInfos then
    for nid1, links in pairs(actionData.linkInfos) do
      for nid2, linkInfo in pairs(links) do
        mapNodes[nid1].links[nid2] = linkInfo
        mapNodes[nid2].links[nid1] = linkInfo
      end
    end
  end
  updateQt = true
end

-- Delete Node
local deleteNodeLinkActionUndo = addNodeLinkActionRedo
local deleteNodeLinkActionRedo = addNodeLinkActionUndo

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
    color = roadRiverGui.highlightColors.hoveredNode
  elseif editor.selection.stNode and editor.selection.stNode[nid] then
    color = roadRiverGui.highlightColors.selectedNode
  elseif distances[nid] and distances[nid] < maxConnectionRenderDistance then
    color = linkLineColor
  else
    color = roadRiverGui.highlightColors.nodeTransparent
  end
  debugDrawer:drawSphere(n.pos, n.radius, color)

  local camNodeSqDist = camPos:squaredDistance(n.pos)
  if camNodeSqDist < square(clamp(editor.getPreference("gizmos.visualization.visualizationDrawDistance") * 0.5, 50, 250)) then
    debugDrawer:drawText(n.pos, String(tostring(nid)), linkBaseColor)
  end

  -- Draw road segments connected to this node
  for roadId, road in pairs(roads) do
    local controlPoints = road.controlPoints
    for i = 1, #controlPoints - 1 do
      local cp1 = controlPoints[i]
      local cp2 = controlPoints[i+1]

      -- Only draw segments connected to this node
      if cp1 == nid or cp2 == nid then
        local n1 = mapNodes[cp1]
        local n2 = mapNodes[cp2]

        if n1 and n2 then
          local linkColor = road.color or linkBaseColor

          -- Check if this segment is hovered or selected
          if hoveredLink and
            ((hoveredLink.nid1 == cp1 and hoveredLink.nid2 == cp2) or
             (hoveredLink.nid1 == cp2 and hoveredLink.nid2 == cp1)) then
            linkColor = roadRiverGui.highlightColors.hoveredNode
          end

          if editor.selection.stLink and
             ((editor.selection.stLink[cp1] and editor.selection.stLink[cp1][cp2]) or
              (editor.selection.stLink[cp2] and editor.selection.stLink[cp2][cp1])) then
            linkColor = roadRiverGui.highlightColors.selectedNode
          end

          debugDrawer:drawSquarePrism(n1.pos, n2.pos, Point2F(0.6, n1.radius*2), Point2F(0.6, n2.radius*2), linkColor)

          if shouldLineBeDrawn(cp1, cp2) then
            debugDrawer:drawCylinder(n1.pos + linkLineOffset, n2.pos + linkLineOffset, 0.3, linkLineColor)
          end

          -- Draw direction arrows if this road is one-way
          if road.properties.oneWay then
            local inNodePos = n1.pos
            local outNodePos = n2.pos
            if road.properties.inNode == cp2 then
              inNodePos, outNodePos = outNodePos, inNodePos
            end

            local edgeDirVec = outNodePos - inNodePos
            local edgeLength = edgeDirVec:length()
            edgeDirVec:setScaled(1 / (edgeLength + 1e-30))

            -- Draw one-way arrows
            local edgeProgress = 0.5
            while edgeProgress <= edgeLength do
              color = core_camera.getForward():dot(edgeDirVec) >= 0 and arrowBaseColor or arrowAltColor
              debugDrawer:drawSquarePrism((inNodePos + edgeProgress * edgeDirVec),
                                        (inNodePos + (edgeProgress + 2) * edgeDirVec),
                                        arrowSize1, arrowSize2, color)
              edgeProgress = edgeProgress + 15
            end
          end

          -- Draw lane markers if this road has lane information
          if road.properties.lanes then
            local strLen = 1
            local laneCount = #road.properties.lanes / strLen
            local inNodeRad, outNodeRad = n1.radius, n2.radius

            -- calculate arrow spacing to draw lane direction indicator arrows
            local edgeDirVec = n2.pos - n1.pos
            local edgeLength = edgeDirVec:length()
            edgeDirVec:setScaled(1 / (edgeLength + 1e-30))

            local arrowLength = 2
            local usableEdgeLength = edgeLength - arrowLength
            local k = math.max(1, math.floor(usableEdgeLength / 30) - 1)
            local dispVec = (usableEdgeLength / (k + 1)) * edgeDirVec
            local arrowLengthVec = arrowLength * edgeDirVec

            local right1 = edgeDirVec:cross(n1.normal)

            -- Draw lane markers
            for laneIdx = 1, laneCount do
              local offset1 = getLaneOffset(cp1, cp2, math.min(inNodeRad, outNodeRad) * 2, laneIdx, laneCount)
              local laneDir = road.properties.lanes:byte(laneIdx) == 43 -- '+'
              color = laneDir and laneColor2 or laneColor1
              local tailPos = n1.pos + right1 * offset1
              local tipPos = vec3()
              for j = 1, k do
                tailPos:setAdd(dispVec)
                tipPos:setAdd2(tailPos, arrowLengthVec)
                debugDrawer:drawSquarePrism(tailPos, tipPos,
                                          laneDir and arrowSize1 or arrowSize2,
                                          laneDir and arrowSize2 or arrowSize1, color)
              end
            end
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

local function importDecalroads()
  selectNode(nil)
  selectLink(nil)

  local rawNodes = map.getMap().nodes
  -- First, make a copy of all nodes
  mapNodes = {}
  for nid, node in pairs(rawNodes) do
    mapNodes[nid] = {
      pos = vec3(node.pos),
      radius = node.radius,
      normal = vec3(node.normal or upVector),
      links = {} -- Keep links temporarily for road tracing
    }
  end

  -- Add links temporarily for road tracing
  for nid1, node in pairs(rawNodes) do
    for nid2, link in pairs(node.links) do
      -- Create two-way links for easier traversal
      mapNodes[nid1].links[nid2] = deepcopy(link)
      if not mapNodes[nid2].links[nid1] then
        mapNodes[nid2].links[nid1] = deepcopy(link)
      end
    end
  end

  -- Clear roads table
  roads = {}

  -- Track processed links to avoid duplicates
  local processedLinks = {}
  local roadId = 1

  -- Function to check if two links have the same properties
  local function linkPropertiesMatch(link1, link2)
    if not link1 or not link2 then return false end
    return link1.drivability == link2.drivability and
           link1.speedLimit == link2.speedLimit and
           (link1.lanes == link2.lanes)
  end

  -- Function to get next node in a continuous road segment
  local function getNextNode(nodeId, prevNodeId)
    local node = mapNodes[nodeId]
    local matchingLinks = {}

    -- Find links with matching properties that haven't been processed
    local baseLink = prevNodeId and mapNodes[nodeId].links[prevNodeId]

    for linkedNodeId, linkData in pairs(node.links) do
      if linkedNodeId ~= prevNodeId and
         not processedLinks[nodeId .. "-" .. linkedNodeId] and
         not processedLinks[linkedNodeId .. "-" .. nodeId] and
         (not baseLink or linkPropertiesMatch(baseLink, linkData)) then
        table.insert(matchingLinks, linkedNodeId)
      end
    end

    -- If exactly one matching link, it's a continuous segment
    if #matchingLinks == 1 then
      return matchingLinks[1]
    end

    -- Otherwise, it's a junction or end of road
    return nil
  end

  -- Function to trace a road starting from a node
  local function traceRoad(startNodeId, initialPrevNodeId)
    local road = {
      id = "road_" .. roadId,
      controlPoints = {}, -- Store node IDs to use as control points
      properties = {}
    }

    local currentNodeId = startNodeId
    local prevNodeId = initialPrevNodeId

    -- If we're starting at a junction, find any unprocessed link
    if not prevNodeId then
      for linkedNodeId, _ in pairs(mapNodes[currentNodeId].links) do
        if not processedLinks[currentNodeId .. "-" .. linkedNodeId] and
           not processedLinks[linkedNodeId .. "-" .. currentNodeId] then
          prevNodeId = currentNodeId
          currentNodeId = linkedNodeId
          break
        end
      end

      -- If no unprocessed link was found, return nil
      if not prevNodeId then return nil end
    end

    -- Initialize road properties from first link
    local firstLink = mapNodes[prevNodeId].links[currentNodeId]
    if firstLink then
      road.properties = {
        drivability = firstLink.drivability,
        speedLimit = firstLink.speedLimit,
        lanes = firstLink.lanes,
        oneWay = firstLink.oneWay or true
      }
    end

    -- Mark first link as processed
    processedLinks[prevNodeId .. "-" .. currentNodeId] = true
    processedLinks[currentNodeId .. "-" .. prevNodeId] = true

    -- Add the first two nodes as control points
    table.insert(road.controlPoints, prevNodeId)
    table.insert(road.controlPoints, currentNodeId)

    -- Continue tracing until no more continuous segments
    local nextNodeId = getNextNode(currentNodeId, prevNodeId)
    while nextNodeId do
      -- Mark segment as processed
      processedLinks[currentNodeId .. "-" .. nextNodeId] = true
      processedLinks[nextNodeId .. "-" .. currentNodeId] = true

      -- Add the next node as control point
      table.insert(road.controlPoints, nextNodeId)

      -- Move to next segment
      prevNodeId = currentNodeId
      currentNodeId = nextNodeId
      nextNodeId = getNextNode(currentNodeId, prevNodeId)
    end

    -- Assign a random color to this road
    local r = math.random()
    local g = math.random()
    local b = math.random()
    road.color = ColorF(r, g, b, 0.7)

    roadId = roadId + 1
    return road
  end

  -- Seed the random number generator to ensure reproducible colors
  math.randomseed(os.time())

  -- Process all nodes and links to create roads
  for nodeId, node in pairs(mapNodes) do
    -- Find unprocessed links from this node
    for linkedNodeId, _ in pairs(node.links) do
      if not processedLinks[nodeId .. "-" .. linkedNodeId] and
         not processedLinks[linkedNodeId .. "-" .. nodeId] then
        local road = traceRoad(nodeId, nil)
        if road and #road.controlPoints >= 2 then
          roads[road.id] = road
        end
      end
    end
  end

  -- Clear temporary links from mapNodes
  for nid, node in pairs(mapNodes) do
    node.links = {}
  end

  log('I', logTag, "Imported " .. tableSize(roads) .. " roads with " .. tableSize(mapNodes) .. " nodes")
  updateQt = true
end

local function loadMapNodes()
  log('I', logTag, "Trying to load navgraph from file navgraph.json")
  local levelDir = path.split(getMissionFilename())
  if not levelDir then return end

  local loadedNavgraph = jsonReadFile(levelDir .. "navgraph.json")
  if loadedNavgraph then
    mapNodes = loadedNavgraph
    for nid1, node in pairs(mapNodes) do
      for nid2, link in pairs(node.links) do
        mapNodes[nid2].links[nid1] = link
      end
      node.pos = vec3(node.pos)
      node.normal = vec3(node.normal)
    end

    log('I', logTag, "Loaded navgraph from file")

    -- Load roads if available
    local loadedRoads = jsonReadFile(levelDir .. "roads.json")
    if loadedRoads then
      roads = loadedRoads
      log('I', logTag, "Loaded roads from file")
    else
      -- If no roads file exists, generate roads from the navgraph
      importDecalroads()
    end

    selectLink(nil)
    selectNode(nil)
  end
end

-- Calculate the length of a road from its control points
local function calculateRoadLength(road)
  local length = 0
  local controlPoints = road.controlPoints

  for i = 1, #controlPoints - 1 do
    local node1 = mapNodes[controlPoints[i]]
    local node2 = mapNodes[controlPoints[i + 1]]

    if node1 and node2 then
      length = length + node1.pos:distance(node2.pos)
    end
  end

  return length
end

-- Add a function to select a road
local function selectRoad(roadId)
  local road = roads[roadId]
  if not road then return end

  -- Clear any existing selection
  selectNode(nil)
  selectLink(nil)

  -- Set the selected road
  selectedRoad = roadId

  -- Select first segment of the road
  if #road.controlPoints >= 2 then
    local link = {
      nid1 = road.controlPoints[1],
      nid2 = road.controlPoints[2]
    }
    selectLink(link)
  end
end

-- Add a function to delete a road and its nodes
local function deleteRoad(roadId)
  local road = roads[roadId]
  if not road then return end

  -- Deselect this road if it's selected
  if selectedRoad == roadId then
    selectLink(nil)
    selectedRoad = nil
  end

  -- Delete the road
  roads[roadId] = nil
  setDirty()
end

-- Add a function to focus the camera on a road
local function focusOnRoad(roadId)
  local road = roads[roadId]
  if not road or #road.controlPoints < 1 then return end

  -- Find the center of the road
  local centerPos = vec3(0, 0, 0)
  local count = 0

  for _, cpId in ipairs(road.controlPoints) do
    local node = mapNodes[cpId]
    if node then
      centerPos = centerPos + node.pos
      count = count + 1
    end
  end

  if count > 0 then
    centerPos = centerPos / count
    -- Set camera position to look at the road center
    editor.setCameraPositionRotation(
      centerPos + vec3(10, 10, 10), -- Position slightly offset from the center
      Quat(0, 0, 0, 1)  -- Default rotation
    )
  end
end

-- Function for update callback in edit mode
local function updateEdit()
  camPos = core_camera.getPosition()
  if qtNodes then
    for nid in qtNodes:query(quadtree.pointBBox(camPos.x, camPos.y, editor.getPreference("gizmos.visualization.visualizationDrawDistance"))) do
      local node = mapNodes[nid]
      if node then
        drawNode(nid, node)
      end
    end
  end
end

local function onEditorGui()
  local editModeOpen = (editor.editMode and (editor.editMode.displayName == editModeName))
  if editModeOpen then
    if editor.beginWindow(toolWindowName, "Slot Traffic") then
      if im.Button("Import all decal roads") then
        importDecalroads()
        setDirty()
      end

      im.Separator()

      -- Display road statistics
      im.Text("Roads: " .. tableSize(roads))
      im.Text("Nodes: " .. tableSize(mapNodes))

      im.Separator()

      -- Show roads data table with virtual scrolling
      if tableSize(roads) > 0 then
        -- Add filter text box
        local filterFlags = im.InputTextFlags_AutoSelectAll
        im.Text("Filter roads:")
        im.SameLine()
        local roadFilterChanged = im.InputText("##RoadFilter", im.ArrayChar(128, roadFilterText), filterFlags)
        if roadFilterChanged then
          roadFilterText = string.lower(ffi.string(im.ArrayChar(128, roadFilterText)))
        end

        -- Table flags
        local tableFlags = im.TableFlags_ScrollY +
                          im.TableFlags_RowBg +
                          im.TableFlags_BordersOuter +
                          im.TableFlags_BordersV +
                          im.TableFlags_Resizable +
                          im.TableFlags_Reorderable

        -- Begin the table
        if im.BeginTable("##RoadsTable", 5, tableFlags, im.ImVec2(0, 300)) then
          -- Setup columns
          im.TableSetupColumn("Road ID", im.TableColumnFlags_DefaultSort, 0, 0)
          im.TableSetupColumn("Length (m)", im.TableColumnFlags_WidthFixed, 80, 1)
          im.TableSetupColumn("Control Points", im.TableColumnFlags_WidthFixed, 100, 2)
          im.TableSetupColumn("Properties", im.TableColumnFlags_WidthStretch, 0, 3)
          im.TableSetupColumn("Actions", im.TableColumnFlags_WidthFixed, 120, 4)
          im.TableHeadersRow()

          -- Clipper for virtual scrolling
          local clipper = im.ImGuiListClipper()
          local roadsArray = {}
          for roadId, road in pairs(roads) do
            -- Apply filter if any
            if roadFilterText == "" or string.find(string.lower(roadId), roadFilterText) then
              table.insert(roadsArray, roadId)
            end
          end
          table.sort(roadsArray)

          if #roadsArray == 0 then
            im.Text("No roads match the filter criteria.")
          else
            im.ImGuiListClipper_Begin(clipper, #roadsArray)
            while im.ImGuiListClipper_Step(clipper) do
              for row = clipper.DisplayStart + 1, clipper.DisplayEnd do
                local roadId = roadsArray[row]
                local road = roads[roadId]

                if road then
                  im.TableNextRow()

                  -- Road ID column
                  im.TableSetColumnIndex(0)
                  local roadColor = road.color or linkBaseColor
                  im.PushStyleColor2(im.Col_Text, im.ImVec4(roadColor.r, roadColor.g, roadColor.b, 1))
                  if im.Selectable1(roadId, roadId == selectedRoad, im.SelectableFlags_SpanAllColumns) then
                    selectRoad(roadId)
                  end
                  im.PopStyleColor()

                  -- Length column
                  im.TableSetColumnIndex(1)
                  local length = calculateRoadLength(road)
                  im.Text(string.format("%.2f", length))

                  -- Control points column
                  im.TableSetColumnIndex(2)
                  im.Text(tostring(#road.controlPoints))

                  -- Properties column
                  im.TableSetColumnIndex(3)
                  local propsText = ""
                  if road.properties then
                    if road.properties.speedLimit then
                      propsText = propsText .. "Speed: " .. road.properties.speedLimit .. " "
                    end
                    if road.properties.lanes then
                      propsText = propsText .. "Lanes: " .. #road.properties.lanes .. " "
                    end
                    if road.properties.oneWay then
                      propsText = propsText .. "OneWay "
                    end
                    if road.properties.drivability then
                      propsText = propsText .. "Drv: " .. string.format("%.1f", road.properties.drivability)
                    end
                  end
                  im.Text(propsText)

                  -- Actions column
                  im.TableSetColumnIndex(4)
                  im.PushID1(roadId)
                  if im.Button("Focus##" .. roadId) then
                    focusOnRoad(roadId)
                  end
                  im.SameLine()
                  im.PushStyleColor2(im.Col_Button, im.ImVec4(0.7, 0.2, 0.2, 0.7))
                  im.PushStyleColor2(im.Col_ButtonHovered, im.ImVec4(0.9, 0.3, 0.3, 0.7))
                  if im.Button("Delete##" .. roadId) then
                    deleteRoad(roadId)
                  end
                  im.PopStyleColor(2)
                  im.PopID()
                end
              end
            end

            -- Clean up
            --im.ImGuiListClipper_Destroy(clipper)
          end

          im.EndTable()
        end
      end

      im.Separator()

      -- Show road information if a link is selected
      if selectedLink then
        -- Find which road this link belongs to
        local foundRoad = nil
        local segmentIndex = nil

        for roadId, road in pairs(roads) do
          local controlPoints = road.controlPoints
          for i = 1, #controlPoints - 1 do
            if (controlPoints[i] == selectedLink.nid1 and controlPoints[i+1] == selectedLink.nid2) or
               (controlPoints[i] == selectedLink.nid2 and controlPoints[i+1] == selectedLink.nid1) then
              foundRoad = road
              segmentIndex = i
              break
            end
          end
          if foundRoad then break end
        end

        if foundRoad then
          im.Text("Selected segment is part of road: " .. foundRoad.id)
          im.Text("Segment #" .. segmentIndex .. " of " .. (#foundRoad.controlPoints - 1))

          local props = foundRoad.properties
          if props then
            im.Text("Road Properties:")
            im.Indent()
            if props.drivability then
              im.Text("Drivability: " .. props.drivability)
            end
            if props.speedLimit then
              im.Text("Speed Limit: " .. props.speedLimit)
            end
            if props.lanes then
              im.Text("Lanes: " .. props.lanes)
            end
            if props.oneWay ~= nil then
              im.Text("One Way: " .. tostring(props.oneWay))
            end
            im.Unindent()
          end
        else
          im.Text("Selected segment is not part of any road")
        end
      end
    end
    editor.endWindow()
  end

  if editModeOpen then
    -- TODO: need to do updateQt less often!
    if not qtNodes or updateQt then
      qtNodes = quadtree.newQuadtree()
      gp = graphpath.newGraphpath()
      if mapNodes then
        -- First add all nodes to the quadtree
        for nid, n in pairs(mapNodes) do
          local nPos = n.pos
          local radius = n.radius
          n.normal = map.surfaceNormal(nPos, radius * 0.5)
          qtNodes:preLoad(nid, quadtree.pointBBox(nPos.x, nPos.y, radius))
          gp:setPointPositionRadius(nid, nPos, radius)
        end

        -- Then add edges from roads
        for roadId, road in pairs(roads) do
          local controlPoints = road.controlPoints
          local properties = road.properties or {}

          for i = 1, #controlPoints - 1 do
            local nid1 = controlPoints[i]
            local nid2 = controlPoints[i+1]
            local n1 = mapNodes[nid1]
            local n2 = mapNodes[nid2]

            if n1 and n2 then
              local nPos = n1.pos
              local lPos = n2.pos

              -- Calculate drivability
              local nDrivability = be:getTerrainDrivability(n1.pos, n1.radius)
              if nDrivability <= 0 then nDrivability = 1 end
              local lDrivability = be:getTerrainDrivability(n2.pos, n2.radius)
              if lDrivability <= 0 then lDrivability = 1 end

              local drivability = properties.drivability or 1
              local edgeDrivability = math.min(1, math.max(1e-30, (nDrivability + lDrivability) * 0.5 * drivability))
              local speedLimit = properties.speedLimit or 50
              local lanes = properties.lanes
              local oneWay = properties.oneWay or false

              -- Define direction for one-way roads
              local inNode, outNode
              if oneWay and properties.inNode then
                inNode = properties.inNode
                outNode = inNode == nid1 and nid2 or nid1
              else
                inNode = nid1
                outNode = nid2
              end

              -- Add edges to graph
              if oneWay then
                gp:uniEdge(inNode, outNode, nPos:distance(lPos) / edgeDrivability, drivability, speedLimit, lanes, oneWay)
              else
                gp:bidiEdge(nid1, nid2, nPos:distance(lPos) / edgeDrivability, drivability, speedLimit, lanes, oneWay)
              end

              -- Also ensure there's an entry in links for rendering
              if not n1.links[nid2] then
                n1.links[nid2] = {inNode = inNode, drivability = drivability, speedLimit = speedLimit, lanes = lanes, oneWay = oneWay}
              end
              if not n2.links[nid1] then
                n2.links[nid1] = {inNode = inNode, drivability = drivability, speedLimit = speedLimit, lanes = lanes, oneWay = oneWay}
              end
            end
          end
        end
        qtNodes:build()
        updateQt = false
      end
      camPos = core_camera.getPosition()
    end
  end

  if editModeOpen then
    local rayCast = staticRayCast()
    focusPoint = rayCast and rayCast.pos

    -- TODO: undo/redo history

    -- Get hovered node
    hoveredNode = nil
    hoveredLink = nil
    if rayCast and not (im.IsAnyItemHovered() or im.IsWindowHovered(im.HoveredFlags_AnyWindow)) then
      local minHitDist = rayCast.distance
      local ray = getCameraMouseRay()
      local rayDir = ray.dir
      for nid in qtNodes:query(quadtree.pointBBox(camPos.x, camPos.y, 200)) do
        local node = mapNodes[nid]
        if not tableContains(tempNodes, nid) then
          local minSphereHitDist, _ = intersectsRay_Sphere(ray.pos, rayDir, node.pos, node.radius)
          if minSphereHitDist and minSphereHitDist < minHitDist then
            hoveredNode = nid
            hoveredLink = nil
            minHitDist = minSphereHitDist
          end
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

    if focusPoint then
      -- Hovers on the map
      if editor.keyModifiers.alt then
        addNodeMode = true

        if not hoveredNode and not tempNodes[1] then
          if temporaryLink then
            deleteLink(temporaryLink.nid1, temporaryLink.nid2)
            temporaryLink = nil
          end

          -- Add Node
          table.insert(tempNodes, addNode(focusPoint, onlySelectedNode and mapNodes[onlySelectedNode].radius or 1))
          addLink(nodesToLinkTo[#tempNodes], tempNodes[#tempNodes])
        end

        if not mouseButtonHeldOnNode then
          if hoveredNode then
            -- delete the temp node
            if not tableIsEmpty(tempNodes) then
              for _, id in ipairs(tempNodes) do
                deleteNode(id)
              end
              table.clear(tempNodes)
            end

            -- add a link to the hovered node
            if onlySelectedNode and not temporaryLink and not areNodesConnected(hoveredNode, onlySelectedNode) then
              addLink(onlySelectedNode, hoveredNode)
              temporaryLink = {nid1 = onlySelectedNode, nid2 = hoveredNode}
            end

          elseif hoveredLink and selectedLink and hoveredLink.nid1 == selectedLink.nid1 and hoveredLink.nid2 == selectedLink.nid2 then
            -- snap the temp node to the hovered link
            local n1 = mapNodes[hoveredLink.nid1]
            local n2 = mapNodes[hoveredLink.nid2]
            local linkVec = n2.pos - n1.pos
            local tempVec = focusPoint - n1.pos
            local dotProduct = linkVec:dot(tempVec)
            linkToSnapTo = selectedLink
            setNodePosition(tempNodes[1], n1.pos + (linkVec:normalized() * dotProduct / linkVec:length()))
            mapNodes[tempNodes[1]].radius = (n1.radius + n2.radius) / 2
          elseif tempNodes[1] then
            setNodePosition(tempNodes[1], focusPoint)
            mapNodes[tempNodes[1]].radius = (onlySelectedNode and mapNodes[onlySelectedNode].radius) or 1
          end
        end
      end

      -- Mouse click on map
      if im.IsMouseClicked(0) and not (im.IsAnyItemHovered() or im.IsWindowHovered(im.HoveredFlags_AnyWindow)) then
        if editor.keyModifiers.alt then
          -- Clicked while in create mode
          if addNodeMode and focusPoint then
            mouseButtonHeldOnNode = true
            if tempNodes[1] then
              oldNodeWidth = mapNodes[tempNodes[1]].radius
            end
          end
        end
      end

      -- User let go of alt
      if addNodeMode and not editor.keyModifiers.alt then
        if temporaryLink then
          deleteLink(temporaryLink.nid1, temporaryLink.nid2)
          temporaryLink = nil
        end
        if not tableIsEmpty(tempNodes) then
          for _, id in ipairs(tempNodes) do
            deleteNode(id)
          end
          table.clear(tempNodes)
        end
        addNodeMode = false
        mouseButtonHeldOnNode = false
      end
    end

    -- Handle mouse click
    if im.IsMouseClicked(0) and not (im.IsAnyItemHovered() or im.IsWindowHovered(im.HoveredFlags_AnyWindow)) then
      dragMouseStartPos = vec3(im.GetMousePos().x, im.GetMousePos().y, 0)
      if not editor.keyModifiers.alt then
        if not (hoveredLink and isSelected(hoveredLink)) then
          selectLink(hoveredLink)
          if hoveredLink then
            selectedOnMouseClick = true
          end
        end
        if not (hoveredNode and isSelected(hoveredNode)) then
          selectNode(hoveredNode)
          if hoveredNode then
            selectedOnMouseClick = true
          end
        end
        heldNode = hoveredNode
        if hoveredNode then
          mouseButtonHeldOnNode = true
          if not tableIsEmpty(editor.selection.stNode) then
            for nodeId, _ in pairs(editor.selection.stNode) do
              nodeOldPositions[nodeId] = mapNodes[nodeId].pos
            end
          end
        end
      end
    end

    if im.IsMouseReleased(0) then

      -- User released LMB after holding it down on a node
      if mouseButtonHeldOnNode then
        if editor.keyModifiers.alt then
          -- Place new node permanently
          addNodeMode = false

          if tempNodes[1] then
            -- Undo action for placed node
            if linkToSnapTo then
              -- Snap node to an existing link
              if mapNodes[linkToSnapTo.nid1].links[linkToSnapTo.nid2].inNode == linkToSnapTo.nid1 then
                addLink(linkToSnapTo.nid1, tempNodes[1])
                addLink(tempNodes[1], linkToSnapTo.nid2)
              else
                addLink(linkToSnapTo.nid2, tempNodes[1])
                addLink(tempNodes[1], linkToSnapTo.nid1)
              end
              selectLink(nil)
              editor.history:beginTransaction("InsertstNode")
              local linkInfos = {}
              linkInfos[linkToSnapTo.nid1] = {}
              linkInfos[linkToSnapTo.nid1][linkToSnapTo.nid2] = mapNodes[linkToSnapTo.nid1].links[linkToSnapTo.nid2]
              editor.history:commitAction("DeletestLink", {linkInfos = linkInfos}, deleteNodeLinkActionUndo, deleteNodeLinkActionRedo)
            end

            -- Add the new node
            local nodeIds = {}
            local nodeInfos = {}
            for i, nodeId in ipairs(tempNodes) do
              table.insert(nodeInfos, {nid = nodeId, pos = vec3(mapNodes[nodeId].pos), radius = mapNodes[nodeId].radius, links = deepcopy(mapNodes[nodeId].links)})
              nodeIds[nodeId] = true
            end

            editor.history:commitAction("AddstNode", {nodeInfos = nodeInfos, linkInfos = deepcopy(getConnectedLinks(nodeIds))}, addNodeLinkActionUndo, addNodeLinkActionRedo, true)

            if linkToSnapTo then
              editor.history:endTransaction()
              deleteLink(linkToSnapTo.nid1, linkToSnapTo.nid2)
              linkToSnapTo = nil
            end
            selectNodes(tempNodes)
          elseif temporaryLink then
            -- Add only a new link
            local linkInfos = {}
            linkInfos[temporaryLink.nid1] = {}
            linkInfos[temporaryLink.nid1][temporaryLink.nid2] = mapNodes[temporaryLink.nid1].links[temporaryLink.nid2]
            editor.history:commitAction("AddstLink", {linkInfos = linkInfos}, addNodeLinkActionUndo, addNodeLinkActionRedo, true)
            selectNode(hoveredNode)
          end
          setDirty()

          --editor.setPreference("slotTrafficEditor.general.defaultRadius", nodeInfo.radius)
          table.clear(tempNodes)
          temporaryLink = nil
        elseif (not dragMouseStartPos) and not tableIsEmpty(editor.selection.stNode) then
          local newValues = {}
          for nid, _ in pairs(editor.selection.stNode) do
            newValues[nid] = vec3(mapNodes[nid].pos)
          end
          editor.history:commitAction("PositionstNode", {nids = deepcopy(editor.selection.stNode), fieldName = "pos", oldValues = deepcopy(nodeOldPositions), newValues = newValues}, changeNodeFieldActionUndo, changeNodeFieldActionRedo, true)
          setDirty()
        elseif not selectedOnMouseClick then
          if hoveredNode then
            selectNode(hoveredNode)
          elseif hoveredLink then
            selectLink(hoveredLink)
          end
        end

        mouseButtonHeldOnNode = false
        dragMouseStartPos = nil
        dragStartPosition = nil
      end
      selectedOnMouseClick = false
    end

    -- The mouse button is down
    if mouseButtonHeldOnNode and im.IsMouseDown(0) then
      local cursorPosImVec = im.GetMousePos()
      local cursorPos = vec3(cursorPosImVec.x, cursorPosImVec.y, 0)

      -- Set the width of the node by dragging
      if editor.keyModifiers.alt then
        if tempNodes[1] and editor.getPreference('slotTrafficEditor.general.dragWidth') then
          local width = math.max(oldNodeWidth + (cursorPos.x - dragMouseStartPos.x) / 10.0, 0)
          mapNodes[tempNodes[1]].radius = width
        end

      -- Put the grabbed node on the position of the cursor
      else
        if tableIsEmpty(editor.selection.stNode) then
          mouseButtonHeldOnNode = false
          dragMouseStartPos = nil
          dragStartPosition = nil
        elseif dragMouseStartPos and (dragMouseStartPos - cursorPos):length() <= 5 then
          -- Snap the node to the old position, if it is close enough
          --setNodePosition(onlySelectedNode, nodeOldPositions, true)
        else
          if focusPoint then
            -- Move all nodes by the offset and project them to the ground
            local nodeOffset = focusPoint - nodeOldPositions[heldNode]
            if core_forest.getForestObject() then core_forest.getForestObject():disableCollision() end
            for nodeId, _ in pairs(editor.selection.stNode) do
              local rayDist = castRayStatic(nodeOldPositions[nodeId] + nodeOffset + upVector, downVector, 10)
              local newPos = nodeOldPositions[nodeId] + nodeOffset + vec3(0, 0, 1 - math.min(rayDist, 10))
              setNodePosition(nodeId, newPos, true)
            end
            if core_forest.getForestObject() then core_forest.getForestObject():enableCollision() end
          end
          dragMouseStartPos = nil
        end
      end
    end
  end

  if editModeOpen then
    if not editModeOpen and tableIsEmpty(mapNodes) then
      -- import the decal roads when the edit mode is not open
      importDecalroads()
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

local function onActivate()
  editor.clearObjectSelection()
  --editor.hideAllSceneTreeInstances()
  editor.showWindow(toolWindowName)
end

local function onDeactivate()
  --editor.showAllSceneTreeInstances()
  editor.hideWindow(toolWindowName)
end

-- These methods are for the action map to call
local function selectAllNodes()
  editor.selection.stNode = {}
  table.clear(nodesToLinkTo)

  for nid, _ in pairs(mapNodes) do
    editor.selection.stNode[nid] = true
    table.insert(nodesToLinkTo, nid)
  end
end

local function onDuplicate()
  if not editor.isViewportFocused() then return end
end

local function onEditorObjectSelectionChanged()
  if not editor.editMode or (editor.editMode.displayName ~= editModeName) then
    return
  end
end

local function stNodeInspectorGui(inspectorInfo)
  if onlySelectedNode then
    local nid = next(editor.selection.stNode)
    local node = mapNodes[nid]
    local editEnded = im.BoolPtr(false)

    -- name
    im.Text("Node: " .. nid)

    -- position
    if node.pos then
      local posArray = im.ArrayFloat(3)
      posArray[0] = im.Float(node.pos.x)
      posArray[1] = im.Float(node.pos.y)
      posArray[2] = im.Float(node.pos.z)
      editor.uiInputFloat3("Position", posArray, nil, nil, editEnded)
      if editEnded[0] then
        local oldValues = {}
        local newValues = {}
        oldValues[nid] = node.pos
        newValues[nid] = vec3(posArray[0], posArray[1], posArray[2])
        editor.history:commitAction("PositionstNode", {nids = deepcopy(editor.selection.stNode), fieldName = "pos", oldValues = oldValues, newValues = newValues}, changeNodeFieldActionUndo, changeNodeFieldActionRedo)
      end
    end

    -- radius
    local radPtr = im.FloatPtr(node.radius)
    editor.uiInputFloat("Radius", radPtr, 0.1, 0.5, nil, nil, editEnded)
    if editEnded[0] then
      local oldValues = {}
      local newValues = {}
      oldValues[nid] = node.radius
      newValues[nid] = radPtr[0]
      editor.history:commitAction("ChangestNodeRadius", {nids = deepcopy(editor.selection.stNode), fieldName = "radius", oldValues = oldValues, newValues = newValues}, changeNodeFieldActionUndo, changeNodeFieldActionRedo)
    end
  end
end

local function stLinkInspectorGui(inspectorInfo)
  if selectedLink then
    local linkData = mapNodes[selectedLink.nid1].links[selectedLink.nid2]
    local editEnded = im.BoolPtr(false)

    -- drivability
    local drivabilityPtr = im.FloatPtr(linkData.drivability)
    editor.uiInputFloat("Drivability", drivabilityPtr, 0.1, 0.5, nil, nil, editEnded)
    if editEnded[0] then
      editor.history:commitAction("ChangestLinkDrivability", {link = selectedLink, fieldName = "drivability", oldValue = linkData.drivability, newValue = drivabilityPtr[0]}, changeLinkFieldActionUndo, changeLinkFieldActionRedo)
    end

    -- speed limit
    local speedLimitPtr = im.FloatPtr(linkData.speedLimit)
    editor.uiInputFloat("Speed Limit", speedLimitPtr, 0.1, 0.5, nil, nil, editEnded)
    if editEnded[0] then
      editor.history:commitAction("ChangestLinkSpeedLimit", {link = selectedLink, fieldName = "speedLimit", oldValue = linkData.speedLimit, newValue = speedLimitPtr[0]}, changeLinkFieldActionUndo, changeLinkFieldActionRedo)
    end

    -- direction
    if im.Button("Change Direction") then
      editor.history:commitAction("ChangestLinkDirection", {link = selectedLink}, changeLinkDirectionActionUndo, changeLinkDirectionActionRedo)
    end
  end
end

--local function onWindowMenuItem()
--  editor.selectEditMode(editor.editModes[editModeName])
--end

local function onDeleteSelection()
  if editor.selection.stNode and not tableIsEmpty(editor.selection.stNode) then
    local nodeInfos = {}
    for nid, _ in pairs(editor.selection.stNode) do
      local nodeInfo = {nid = nid, pos = vec3(mapNodes[nid].pos), radius = mapNodes[nid].radius, links = deepcopy(mapNodes[nid].links)}
      table.insert(nodeInfos, nodeInfo)
    end

    local nids = editor.selection.stNode
    editor.history:commitAction("DeletestNodes", {nodeInfos = nodeInfos, linkInfos = deepcopy(getConnectedLinks(nids))}, deleteNodeLinkActionUndo, deleteNodeLinkActionRedo)
  elseif editor.selection.stLink and not isLinkSelectionEmpty() then
    local linkInfos = {}
    for nid1, nodeLinks in pairs(editor.selection.stLink) do
      for nid2, _ in pairs(nodeLinks) do
        if not linkInfos[nid1] then linkInfos[nid1] = {} end
        linkInfos[nid1][nid2] = mapNodes[nid1].links[nid2]
      end
    end
    editor.history:commitAction("DeletestLinks", {linkInfos = linkInfos}, deleteNodeLinkActionUndo, deleteNodeLinkActionRedo)
  end
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, im.ImVec2(600, 200))
  editor.editModes[editModeName] =
  {
    displayName = editModeName,
    onUpdate = updateEdit,
    onActivate = onActivate,
    onDeactivate = onDeactivate,
    onDeleteSelection = onDeleteSelection,
    actionMap = actionMapName,
    onDuplicate = onDuplicate,
    iconTooltip = "SlotTraffic Editor",
    auxShortcuts = {},
    hideObjectIcons = true,
    sortOrder = 100
  }

  --editor.editModes[editModeName].icon = editor.icons.directions_bike

  editor.registerInspectorTypeHandler("stNode", stNodeInspectorGui)
  editor.registerInspectorTypeHandler("stLink", stLinkInspectorGui)

  loadMapNodes()
end

local function onEditorRegisterPreferences(prefsRegistry)
  --[[prefsRegistry:registerCategory("aiEditor")
  prefsRegistry:registerSubCategory("aiEditor", "general", nil,
  {
    -- {name = {type, default value, desc, label (nil for auto Sentence Case), min, max, hidden, advanced, customUiFunc, enumLabels}}
    {dragWidth = {"bool", false, "Drag Width", nil, nil, nil, false}}
  })]]
end

local function onEditorPreferenceValueChanged(path, value)

end

local function onNavgraphReloaded()
  qtNodes = nil
  mapNodes = {}
  loadMapNodes()
end

local function onEditorAfterSaveLevel()
  if navgraphDirty then
    local mapNodesCopy = deepcopy(mapNodes)
    for nid1, node in pairs(mapNodes) do
      for nid2, link in pairs(node.links) do
        if link.inNode ~= nid1 then
          mapNodesCopy[nid1].links[nid2] = nil
        end
      end
      mapNodesCopy[nid1].pos = node.pos:toDict()
      mapNodesCopy[nid1].normal = node.normal:toDict()
    end

    -- Also save roads data
    local roadsCopy = deepcopy(roads)

    local levelDir = path.split(getMissionFilename())
    jsonWriteFile(levelDir .. "navgraph.json", mapNodesCopy, true)
    jsonWriteFile(levelDir .. "roads.json", roadsCopy, true)
    navgraphDirty = false
    log('I', logTag, "Saved navgraph and roads to file")
  end
end

local function onSerialize()
  local data = {
    navgraphDirty = navgraphDirty,
    roads = roads
  }
  return data
end

local function onDeserialized(data)
  if data then
    navgraphDirty = data.navgraphDirty
    roads = data.roads or {}
  end
end

M.onEditorGui = onEditorGui
M.onEditorInitialized = onEditorInitialized
M.onEditorObjectSelectionChanged = onEditorObjectSelectionChanged
M.onEditorRegisterPreferences = onEditorRegisterPreferences
M.onEditorPreferenceValueChanged = onEditorPreferenceValueChanged
M.onEditorAfterSaveLevel = onEditorAfterSaveLevel
M.onNavgraphReloaded = onNavgraphReloaded

M.selectAllNodes = selectAllNodes

return M