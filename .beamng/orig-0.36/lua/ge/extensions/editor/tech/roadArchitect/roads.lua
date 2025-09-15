-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local nodeHeightLimit = 1000.0                                                                      -- The height limit for all road nodes, in metres.
local nodeRotLimit = 50.0                                                                           -- The lateral rotation limit for all road nodes, in degrees.
local splitRepelDist = 1.0                                                                          -- The distance by which to repel nodes after a split, in meters.
local auditionHeight = 1000.0                                                                       -- The height above zero, at which the prefab groups are auditioned, in metres.
local camRotInc = 0.0062831853                                                                      -- The step size of the angle when rotating the camera around the audition center.
local reparameteriseNodeDist = 25                                                                   -- When importing decal roads to road architect, the max dist between nodes for fitting.
local interDistTol = 3.5                                                                            -- When placing intermediate nodes with mouse, the max dist to centerline.

local tempRoadName = 'temp'                                                                         -- The name of the temporary road which is used for auditioning profiles.

local defaultOverlayMaterial = 'm_tread_marks_clean'                                                -- The default material used for overlays.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- External modules used.
local treeMgr = require('quadtree')                                                                 -- A module for managing a 2D quadtree, used for fast edit visualisation culling.
local profileMgr = require('editor/tech/roadArchitect/profiles')                                    -- Manages the profiles structure/handles profile calculations.
local geom = require('editor/tech/roadArchitect/geometry')                                          -- A module for performing geometric calculations.
local roadMeshMgr = require('editor/tech/roadArchitect/roadMesh')                                   -- A module for managing procedural road meshes.
local staticMeshMgr = require('editor/tech/roadArchitect/staticMesh')                               -- A module for managing static meshes.
local decMgr = require('editor/tech/roadArchitect/decals')                                          -- A module for managing road decals.
local tMesh = require('editor/tech/roadArchitect/tunnelMesh')                                       -- Manages the auto tunnel meshes.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.

-- Private constants.
local im = ui_imgui
local min, max = math.min, math.max
local abs, sin, cos, tan = math.abs, math.sin, math.cos, math.tan
local twoPi = math.pi * 2.0
local tmp0, tmp1, tmp2, tmp3 = vec3(0, 0), vec3(0, 0), vec3(0, 0), vec3(0, 0)
local temp_P1, temp_P2 = vec3(-10, 0, auditionHeight), vec3(10, 0, auditionHeight)
local gView, auditionVec, auditionCamPos = vec3(0, 0), vec3(0, 0, auditionHeight), vec3(0, 0)
local camRotAngle = 0.0
local interDistTolSq = interDistTol * interDistTol

-- Public module state.
local roads = {}                                                                                    -- The collection of roads currently present in the scene.
local roadMap = {}                                                                                  -- A hash table which maps road names to index in the roads array.
local currentMap = nil                                                                              -- The currently stored map.
local multi = {}                                                                                    -- A container for storing a multi-selection.
local tree = nil
local isAuditionProfileDirty = false                                                                -- A flag indicating if changes have been made to the profile under audition.


-- Gets the currently-loaded map.
local function getCurrentMap() return currentMap end

-- Sets the currently-loaded map.
local function setCurrentMap(cMap) currentMap = cMap end

-- Creates a new road from a given lateral profile.
local function createRoadFromProfile(profile)
  profileMgr.updateLaneFlags(profile)                                                               -- Update the lane type flags (used for displaying static mesh parameters).
  local laneKeys, leftKeys, rightKeys = profileMgr.computeLaneKeys(profile)
  return {
    displayName = im.ArrayChar(32, 'New Road'),                                                     -- The display name for this road.

    isVis = im.BoolPtr(true),                                                                       -- A flag which indicates if road will appear in edit visualisation or not.
    isHidden = false,                                                                               -- A flag which indicates if this road is hidden from the user (eg temp roads).
    isJctRoad = false,                                                                              -- A flag which indicates if this road appears in a currently-editable junction.
    treatAsInvisibleInEdit = false,                                                                 -- A flag which indicates if this road should be invisible in edit visualisation.

    isOverObject = im.BoolPtr(true),                                                                -- A flag which indicates if the overObjects flag will be set for this road's decals.

    aabb = nil,                                                                                     -- The current 2D AABB of the road.

    isDirty = true,                                                                                 -- Indicates if a change has been made to this road, requiring updates.
    isDrivable = true,                                                                              -- Indicates if this road should be driveable (wrt the navigation graph).
    isOverlay = false,                                                                              -- A flag which indicates if this road is an overlay, or not.

    isArc = false,                                                                                  -- Indicates if this road is an arc road (or a spline road).
    isBridge = false,                                                                               -- A flag which indicates if this is a bridge (rather than a road).

    groupIdx = {},                                                                                  -- A container for the indices of all groups which this road belongs to.

    renderData = nil,                                                                               -- A table containing all the world-space positions used for rendering.
    laneKeys = laneKeys, leftKeys = leftKeys, rightKeys = rightKeys,                                -- A table containg all the lane keys associated to this road.

    granFactor = im.IntPtr(1),                                                                      -- The granularity factor for this road.

    name = Engine.generateUUID(),                                                                   -- The unique id of the road.

    nodes = {},                                                                                     -- The collection of reference nodes for this road.

    profile = profile,                                                                              -- The lateral road profile associated with this road.

    overlayMat = defaultOverlayMaterial,                                                            -- The overlay material (if this is an overlay rather than a road).

    isConformRoadToTerrain = im.BoolPtr(false),                                                     -- Indicates if road should conform to the local terrain (inherit height).
    isDisplayRoadSurface = im.BoolPtr(true),                                                        -- Indicates if the road surface should be visualised (debugDraw)
    isDisplayRoadOutline = im.BoolPtr(true),                                                        -- Indicates if the road outline should be visualised (debugDraw).
    isDisplayNodeSpheres = im.BoolPtr(true),                                                        -- Indicates if the node spheres should be displayed at nodes (debugDraw).
    isDisplayNodeNumbers = im.BoolPtr(false),                                                       -- Indicates if the node number markups should be displayed (debugDraw).
    isDisplayLaneInfo = im.BoolPtr(true),                                                           -- Indicates if the lane markups should be displayed (debugDraw).
    isDisplayRefLine = im.BoolPtr(true),                                                            -- Indicates if the road reference line should be displayed (debugDraw).

    isRigidTranslation = im.BoolPtr(false),                                                         -- Indicates if fully-rigid translations are used when moving road nodes.

    forceField = im.FloatPtr(1.0),                                                                  -- The value of the force field (when using non-rigid translation).
    isCivilEngRoads = im.BoolPtr(false),                                                            -- Indicates if this road is using civil engineering style or spline style.

    bridgeWidth = im.FloatPtr(5.5),                                                                 -- Bridges: The half-width of the bridge (width of each of the two lanes), in meters.
    bridgeDepth = im.FloatPtr(4.0),                                                                 -- Bridges: The depth of the bridge, in meters.
    bridgeArch = im.FloatPtr(-6.0),                                                                 -- Bridges: The amount of arching, in meters.

    isAllowTunnels = im.BoolPtr(false),                                                             -- Tunnels: Indicates if auto tunnels are allowed on this road.
    tunnels = {},                                                                                   -- Tunnels: The collection of auto tunnel sections belonging to this road.
    radGran = im.IntPtr(15),                                                                        -- Tunnels: The radial granularity.
    radOffset = im.FloatPtr(0.0),                                                                   -- Tunnels: The radial offset.
    thickness = im.FloatPtr(1.0),                                                                   -- Tunnels: The wall thickness.
    zOffsetFromRoad = im.FloatPtr(0.0),                                                             -- Tunnels: The vertical offset, of the road inside the tunnel.
    protrudeS = im.FloatPtr(0.0),                                                                   -- Tunnels: The amount of protrusion along the tangent, at the start pos.
    protrudeE = im.FloatPtr(0.0),                                                                   -- Tunnels: The amount of protrusion along the tangent, at the end pos.
    extraS = im.IntPtr(2),                                                                          -- Tunnels: The start road position (the div point index).
    extraE = im.IntPtr(2)                                                                           -- Tunnels: The end road position (the div point index).
  }
end

-- Creates a new road from a given lateral profile template name.
local function createRoadFromTemplate(profileTemplateName)
  return createRoadFromProfile(profileMgr.createProfileFromTemplate(
    profileTemplateName,
    Engine.generateUUID()))
end

-- Computes a 2D Axis-Aligned Bounding Box which represents the given road.
-- [This is used only with the acceleration tree.]
local function computeRoadAABB_2D(r)
  local xMin, xMax, yMin, yMax = 1e99, -1e99, 1e99, -1e99
  local nodes = r.nodes
  for i = 1, #nodes do
    local n = nodes[i].p
    local x, y = n.x, n.y
    xMin, xMax, yMin, yMax = min(xMin, x), max(xMax, x), min(yMin, y), max(yMax, y)
  end
  return xMin, xMax, yMin, yMax
end

-- Handles the case when a road needs updated.
local function setDirty(r)
  if r then
    r.isDirty = true
    profileMgr.updateCondition(r)

    local xMin, xMax, yMin, yMax = computeRoadAABB_2D(r)

    if not tree then                                                                                -- If there is no tree yet, build one.
      tree = treeMgr.newQuadtree()
      local extents = { x = 5000, y = 5000 }
      local tb = extensions.editor_terrainEditor.getTerrainBlock()
      if tb then
        extents = tb:getObjectBox():getExtents()
      end
      tree:preLoad('initial_entry', -extents.x, -extents.y, extents.x, extents.y)                   -- Create the tree with this road.
      tree:build()
    end
    if r.aabb then
      tree:remove(r.name, r.aabb.x, r.aabb.y)
    end
    tree:insert(r.name, xMin, yMin, xMax, yMax)                                                     -- Update the tree.
    r.aabb = vec3((xMin + xMax) * 0.5, (yMin + yMax) * 0.5)                                         -- Set the latest AABB on the road, for later tree-removal usage.
  end
end

-- Sets all the roads dirty.
local function setAllDirty()
  for i = 1, #roads do
    setDirty(roads[i])
  end
end

-- Gets the tree.
local function getTree() return tree end

-- Removes the tree.
local function removeTree() tree = nil end

-- Sets the flag which indicates if the audition profiles needs updating, to true.
local function setAuditionProfileDirty() isAuditionProfileDirty = true end

-- Adds a new node at the end of the road with the given index.
local function addNodeToRoad(rIdx, pos)
  local r = roads[rIdx]
  if r then
    local widths, heightsL, heightsR = profileMgr.getWAndHByKey(r.profile)
    r.nodes[#r.nodes + 1] = {
      p = vec3(pos.x, pos.y, pos.z),                                                                -- The node world-space position.
      isLocked = false,                                                                             -- Indicates if node is locked or unlocked (ie if can be moved by the user).
      rot = im.FloatPtr(0.0),                                                                       -- The lateral rotation angle of the normal at this node (sets road camber).
      widths = widths,                                                                              -- The widths of each lane, by lane key.
      heightsL = heightsL,                                                                          -- The left lane height of each lane, by lane key.
      heightsR = heightsR,                                                                          -- The right lane height of each lane, by lane key.
      incircleRad = im.FloatPtr(1.0),                                                               -- The radius of the incircle, in [0.1, 2] (used for civil eng style roads).
      isAutoBanked = r.profile.isAutoBanking[0],                                                    -- A flag which indicates if auto-banking is being performed on this node, or not.
      offset = 0.0 }                                                                                -- The lateral lane offset at this node.
    setDirty(r)
  end
end

-- Adds a new node at the given position, at the given index of the road.
local function addNodeToRoadAtIdx(rIdx, pos, idx)
  local r = roads[rIdx]
  if r then
    local widths, heightsL, heightsR = profileMgr.getWAndHByKey(r.profile)
    local newNode =  {
      p = vec3(pos.x, pos.y, pos.z),                                                                -- The node world-space position.
      isLocked = false,                                                                             -- Indicates if node is locked or unlocked (ie if can be moved by the user).
      rot = im.FloatPtr(0.0),                                                                       -- The lateral rotation angle of the normal at this node (sets road camber).
      widths = widths,                                                                              -- The widths of each lane, by lane key.
      heightsL = heightsL,                                                                          -- The left lane height of each lane, by lane key.
      heightsR = heightsR,                                                                          -- The right lane height of each lane, by lane key.
      incircleRad = im.FloatPtr(1.0),                                                               -- The radius of the incircle, in [0.1, 2] (used for civil eng style roads).
      isAutoBanked = r.profile.isAutoBanking[0],                                                    -- A flag which indicates if auto-banking is being performed on this node, or not.
      offset = 0.0 }                                                                                -- The lateral lane offset at this node.
    table.insert(r.nodes, idx, newNode)
    setDirty(r)
  end
end

-- Determines if the mouse position is at an intermediate position along the road (between nodes).
local function isMouseAtIntermediatePos(rIdx, mousePos)
  local isInter, interIdx, q = false, nil, 0.5
  local r = roads[rIdx]
  if r then
    local nodes = r.nodes
    local dBest = 1e99
    for i = 2, #nodes do
      local p1, p2 = nodes[i - 1].p, nodes[i].p
      tmp1:set(p1.x, p1.y, 0.0)                                                                     -- Project the point to the XY-plane (ie 3D -> 2D).
      tmp2:set(p2.x, p2.y, 0.0)
      local dSq = mousePos:squaredDistanceToLineSegment(tmp1, tmp2)
      if dSq < min(dBest, interDistTolSq) then
        local proj = util.projectPointToLine(mousePos, tmp1, tmp2)                                  -- Project the mouse position to the line going through the line segment.
        if proj:squaredDistanceToLineSegment(tmp1, tmp2) < 1e-2 then                                -- Ensure that the projected point is inside the line segment.
          dBest = dSq
          interIdx = i
          q = proj:distance(tmp1) / tmp1:distance(tmp2)
          isInter = true
        end
      end
    end
    if isInter then
      local interPos = nodes[interIdx - 1].p + q * (nodes[interIdx].p - nodes[interIdx - 1].p)
      return isInter, interPos, interIdx
    end
  end
  return false, nil, nil
end

-- Sets the width at a given node, by dividing the given width into all the lanes.
local function setLocalWidth(rIdx, nIdx, offset)
  local r = roads[rIdx]
  if r then
    local n = r.nodes[nIdx]
    if n then
      local numLanes = 0
      local startWidth = nil
      for k, v in pairs(n.widths) do
        if r.profile[k].type == 'road_lane' then
          numLanes = numLanes + 1
          startWidth = v[0]
        end
      end
      local laneWidth = max(0.5, min(10.0, startWidth + offset))
      for k, _ in pairs(n.widths) do
        if r.profile[k].type == 'road_lane' then
          n.widths[k] = im.FloatPtr(laneWidth)
        end
      end
    end
  end
  setDirty(r)
end

-- Deep copies a node.
local function copyNode(n)
  local wC, hLC, hRC, w, hL, hR = {}, {}, {}, n.widths, n.heightsL, n.heightsR
  for i = -20, 20 do
    if w[i] then
      wC[i], hLC[i], hRC[i] = im.FloatPtr(w[i][0]), im.FloatPtr(hL[i][0]), im.FloatPtr(hR[i][0])
    end
  end
  local pos = n.p
  return {
    p = vec3(pos.x, pos.y, pos.z),
    isLocked = n.isLocked,
    rot = im.FloatPtr(n.rot[0]),
    widths = wC, heightsL = hLC, heightsR = hRC,
    incircleRad = im.FloatPtr(n.incircleRad[0]),
    isAutoBanked = n.isAutoBanked,
    offset = n.offset }
end

-- Updates the width/relative height offset for the given road (in all nodes), after changing the lateral road profile.
local function updateWAndHToNewProfile(road)
  local nodes = road.nodes
  local numNodes = #nodes
  for i = 1, numNodes do
    nodes[i].widths, nodes[i].heightsL, nodes[i].heightsR = profileMgr.getWAndHByKey(road.profile)
  end
end

-- Computes the 2D axis-aligned bounding box of the road with the given id.
-- [The profile width is also included in the computation, rather than just the nodes themselves].
local function computeAABB2D(rIdx)
  local road = roads[rIdx]
  local nodes = road.nodes
  if not road.isBridge and not road.isOverlay then
    local xMin, xMax, yMin, yMax = 1e24, -1e24, 1e24, -1e24
    for i = 1, #nodes do
      local p = nodes[i].p
      local px, py = p.x, p.y
      xMin, xMax, yMin, yMax = min(xMin, px), max(xMax, px), min(yMin, py), max(yMax, py)
    end
    return { xMin = xMin, xMax = xMax, yMin = yMin, yMax = yMax }
  end
end

-- Gets the subset of roads from the given group.
local function getRoadsFromGroup(group)
  local rOut, mark, ctr = {}, {}, 1
  local groupList = group.list
  for i = 1, #groupList do
    local gL = groupList[i]
    local rCandName = gL.r
    if not mark[rCandName] then
      rOut[ctr] = roads[roadMap[rCandName]]
      mark[rCandName] = true
      ctr = ctr + 1
    end
  end
  return rOut
end

-- Re-computes the map (hash-table) between road names and index in the roads array.
local function recomputeMap()
  table.clear(roadMap)
  local numRoads = #roads
  for i = 1, numRoads do
    roadMap[roads[i].name] = i
  end
end

-- Removes the road with the given id from the scene.
local function removeRoad(roadName)
  local idx = roadMap[roadName]
  local road = roads[idx]
  if road then
    roadMeshMgr.tryRemove(roadName)                                                                 -- Remove the road mesh from the scene, and the roads array entry.
    staticMeshMgr.tryRemove(roadName)                                                               -- If any static mesh was created for this road on finalise, remove them now.
    if road.aabb and tree then
      tree:remove(road.name, road.aabb.x, road.aabb.y)                                              -- Remove this road from the tree.
    end
    if road.isBridge then                                                                           -- If this is a bridge, remove the folder from the scene tree.
      roadMeshMgr.tryRemoveBridge(road.name)
      local folder = scenetree.findObject("Road Architect - Bridge " .. tostring(road.name))
      if folder then
        folder:delete()
      end
    end
    table.remove(roads, idx)
  end
  recomputeMap()
end

-- Removes the node with the given node id, from the road with the given road id.
local function removeNode(rIdx, nIdx) table.remove(roads[rIdx].nodes, nIdx) end

-- Produces a table of average widths, from two width tables.
local function averageWidths(w1, w2, hL1, hL2, hR1, hR2)
  local wAvg, hLAvg, hRAvg = {}, {}, {}
  for i = -20, 20 do
    local wLane1, wLane2 = w1[i], w2[i]
    local hLLane1, hLLane2, hRLane1, hRLane2 = hL1[i], hL2[i], hR1[i], hR2[i]
    if wLane1 then
      wAvg[i] = im.FloatPtr((wLane1[0] + wLane2[0]) * 0.5)
      hLAvg[i] = im.FloatPtr((hLLane1[0] + hLLane2[0]) * 0.5)
      hRAvg[i] = im.FloatPtr((hRLane1[0] + hRLane2[0]) * 0.5)
    end
  end
  return wAvg, hLAvg, hRAvg
end

-- Adds a node between the currently-selected node and the next node.
-- [The added node is computed as the midpoint of the two nodes].
local function addIntermediateNode(rIdx, nIdx, lie)
  local road = roads[rIdx]
  local nodes = road.nodes

  if lie == 'above' then
    if nIdx == 1 then                                                                               -- Special case for 'above'.
      local n1 = nodes[nIdx]
      local newNode = copyNode(n1)
      newNode.p = n1.p + (n1.p - nodes[nIdx + 1].p):normalized() * 3
      table.insert(nodes, nIdx, newNode)
    else                                                                                            -- General case for 'above'.
      local nLast = nIdx - 1
      local n1, n2 = nodes[nIdx], nodes[nLast]
      local w, hL, hR = averageWidths(n1.widths, n2.widths, n1.heightsL, n2.heightsL, n1.heightsR, n2.heightsR)
      local newNode = {
        p = (n1.p + n2.p) * 0.5,                                                                    -- The new point is the midpoint between the selected node and the next node.
        isLocked = false,                                                                           -- The new node will be unlocked by default.
        rot = im.FloatPtr((n1.rot[0] + n2.rot[0]) * 0.5),                                           -- The rest of the values are averaged between the selected node and next node.
        widths = w, heightsL = hL, heightsR = hR,
        incircleRad = im.FloatPtr((n1.incircleRad[0] + n2.incircleRad[0]) * 0.5),
        isAutoBanked = n1.isAutoBanked or n2.isAutoBanked,
        offset = (n1.offset + n2.offset) * 0.5 }
      table.insert(nodes, nIdx, newNode)
    end
  elseif lie == 'below' then
    if nIdx == #nodes then                                                                          -- Special case for 'below'.
      local n1 = nodes[nIdx]
      local newNode = copyNode(n1)
      newNode.p = n1.p + (n1.p - nodes[nIdx - 1].p):normalized() * 3
      table.insert(nodes, nIdx + 1, newNode)
    else                                                                                            -- General case for 'below'.
      local nNext = nIdx + 1
      local n1, n2 = nodes[nIdx], nodes[nNext]
      local w, hL, hR = averageWidths(n1.widths, n2.widths, n1.heightsL, n2.heightsL, n1.heightsR, n2.heightsR)
      local newNode = {
        p = (n1.p + n2.p) * 0.5,                                                                    -- The new point is the midpoint between the selected node and the next node.
        isLocked = false,                                                                           -- The new node will be unlocked by default.
        rot = im.FloatPtr((n1.rot[0] + n2.rot[0]) * 0.5),                                           -- The rest of the values are averaged between the selected node and next node.
        widths = w, heightsL = hL, heightsR = hR,
        incircleRad = im.FloatPtr((n1.incircleRad[0] + n2.incircleRad[0]) * 0.5),
        isAutoBanked = n1.isAutoBanked or n2.isAutoBanked,
        offset = (n1.offset + n2.offset) * 0.5 }
      table.insert(nodes, nNext, newNode)
    end
  end
end

-- Removes all road from the scene.
local function clearAllRoads()
  for i = #roads, 1, -1 do
    local r = roads[i]
    if not r.isJctRoad then
      removeRoad(r.name)
    end
  end
end

-- Removes all hidden roads from the roads container.
local function removeHiddenRoads()
  local numRoads = #roads
  for i = numRoads, 1, -1 do
    local r = roads[i]
    if r.isHidden then
      roadMeshMgr.tryRemove(r.name)
      staticMeshMgr.tryRemove(r.name)                                                               -- If any static mesh was created for this road on finalise, remove them now.
      table.remove(roads, i)
    end
  end

  -- Re-compute the road map.
  recomputeMap()
end

-- Moves the camera to directly above the road with the given index.
local function goToRoad(rIdx)

  -- Compute the 2D axis-aligned bounding box of the given road.
  -- [We also compute the largest height value].
  local xMin, xMax, yMin, yMax, zMax = 1e24, -1e24, 1e24, -1e24, -1e24
  local road = roads[rIdx]
  local nodes = road.nodes
  local numNodes = #nodes
  for i = 1, numNodes do
    local p = nodes[i].p
    local px, py = p.x, p.y
    xMin, xMax = min(xMin, px), max(xMax, px)
    yMin, yMax = min(yMin, py), max(yMax, py)
    zMax = max(zMax, p.z)
  end

  -- If road is too small (eg one node), do nothing.
  if abs(xMax - xMin) < 0.1 and abs(yMax - yMin) < 0.1 then
    return
  end

  -- Determine the required distance which the camera should be at.
  local midX, midY = (xMin + xMax) * 0.5, (yMin + yMax) * 0.5                                       -- Midpoint of axis-aligned bounding box.
  local tmp0 = vec3(midX, midY, 0.0)
  local tmp1 = vec3(xMax, yMax, 0.0)
  local groundDist = tmp0:distance(tmp1)                                                            -- The largest distance from the center of the box to the outside.
  local halfFov = core_camera.getFovRad() * 0.5                                                     -- Half the camera field of view (in radians).
  local height = groundDist / tan(halfFov) + zMax + 5.0                                             -- The height that the camera should be to fit all the trajectory in view.
  local rot = quatFromDir(vec3(0, 0, -1))

  -- Move the camera to the appropriate pose.
  commands.setFreeCamera()
  core_camera.setPosRot(0, midX, midY, height, rot.x, rot.y, rot.z, rot.w)
end

-- Performs a translation upon the road with the given index.
-- [The manner by which the road is translated depends on various road/node properties].
local function moveRoad(road, nodeIdx, mouseNow, mouseLast)

  -- Split into cases: [Rigid Translation] or [Force-Field Translation].
  local nodes, v = road.nodes, mouseNow - mouseLast
  local numNodes = #nodes
  if road.isRigidTranslation[0] then
    for i = 1, numNodes do                                                                          -- CASE A: [Rigid Translation].
      if not nodes[i].isLocked then
        nodes[i].p = nodes[i].p + v
      end
    end
  else
    -- First, move the central node to the mouse position.
    local cNode, fieldInv = nodes[nodeIdx], 1.0 / max(1e-7, road.forceField[0])                     -- CASE B: [Force-Field Translation].
    cNode.p = mouseNow

    -- Iterate from just below the central node, to the start of the polyline.
    for i = max(1, nodeIdx - 1), 1, -1 do
      local n = nodes[i]
      if n.isLocked then                                                                            -- Do not go beyond any locked node in the -ve direction.
        break
      end
      n.p = n.p + v * min(1.0, max(0.0, 1.0 - cNode.p:distance(n.p) * fieldInv))                    -- Move this node by a distance-based ratio (based on the force field).
    end

    -- Iterate from just above the central node, to the end of the polyline.
    for i = min(numNodes, nodeIdx + 1), numNodes do
      local n = nodes[i]
      if n.isLocked then                                                                            -- Do not go beyond and locked node in the +ve direction.
        break
      end
      n.p = n.p + v * min(1.0, max(0.0, 1.0 - cNode.p:distance(n.p) * fieldInv))                    -- Move this node by a distance-based ratio (based on the force field).
    end
  end

  setDirty(road)
end

-- Performs a height adjustment to the road with the given index.
-- [The manner by which the road height is adjusted depends on various road/node properties].
local function adjustHeight(hNew, hOld, nIdx, rIdx)

  local road = roads[rIdx]
  local nodes, dz = road.nodes, hNew - hOld
  local numNodes = #nodes

  -- First, adjust the central node to the mouse position.
  local cNode, fieldInv = nodes[nIdx], 1.0 / max(1e-7, road.forceField[0])                          -- CASE A: [Force-Field Translation].
  cNode.p.z = hNew
  cNode.p.z = min(nodeHeightLimit, max(-nodeHeightLimit, cNode.p.z))

  -- Now adjust the rest of the nodes, as appropriate.
  if nodeHeightLimit - abs(cNode.p.z) > 1e-3 then                                                   -- If the central node is maxed out, do not move the other nodes.
    if road.isRigidTranslation[0] then
      for i = 1, numNodes do                                                                        -- CASE B: [Rigid Translation].
        if i ~= nIdx then
          nodes[i].p.z = nodes[i].p.z + dz
          nodes[i].p.z = min(nodeHeightLimit, max(-nodeHeightLimit, nodes[i].p.z))
        end
      end
    else
      -- Iterate from just below the central node, to the start of the polyline.
      for i = nIdx - 1, 1, -1 do
        local n = nodes[i]
        if n.isLocked then                                                                          -- Do not go beyond any locked node in the -ve direction.
          break
        end
        n.p.z = n.p.z + dz * min(1.0, max(0.0, 1.0 - cNode.p:distance(n.p) * fieldInv))             -- Move this node by a distance-based ratio (based on the force field).
        n.p.z = min(nodeHeightLimit, max(-nodeHeightLimit, n.p.z))
      end

      -- Iterate from just above the central node, to the end of the polyline.
      for i = nIdx + 1, numNodes do
        local n = nodes[i]
        if n.isLocked then                                                                          -- Do not go beyond and locked node in the +ve direction.
          break
        end
        n.p.z = n.p.z + dz * min(1.0, max(0.0, 1.0 - cNode.p:distance(n.p) * fieldInv))             -- Move this node by a distance-based ratio (based on the force field).
        n.p.z = min(nodeHeightLimit, max(-nodeHeightLimit, n.p.z))
      end
    end
  end
  setDirty(road)
end

-- Performs a lateral rotation upon the road with the given index.
-- [The manner by which the road is rotated depends on various road/node properties].
local function adjustLateralRotation(rotNew, rotOld, nIdx, rIdx)

  local road = roads[rIdx]
  if not road.profile.isAutoBanking[0] then
    local nodes, dRot = road.nodes, rotNew - rotOld
    local numNodes = #nodes

    -- First, rotate the central node to the mouse position.
    local cNode, fieldInv = nodes[nIdx], 1.0 / max(1e-7, road.forceField[0])                        -- CASE: [Force-Field Translation].
    cNode.rot = im.FloatPtr(cNode.rot[0] + dRot)
    cNode.rot = im.FloatPtr(min(nodeRotLimit, max(-nodeRotLimit, cNode.rot[0])))

    -- Now rotate the rest of the nodes, as appropriate.
    if nodeRotLimit - abs(cNode.rot[0]) > 1e-3 then                                                 -- If the central node is maxed out, do not move the other nodes.

      if road.isRigidTranslation[0] then
        for i = 1, numNodes do                                                                      -- CASE: [Rigid Translation].
          if i ~= nIdx then
            nodes[i].rot = im.FloatPtr(nodes[i].rot[0] + dRot)
            nodes[i].rot = im.FloatPtr(min(nodeRotLimit, max(-nodeRotLimit, nodes[i].rot[0])))
          end
        end
      else

        -- Iterate from just below the central node, to the start of the polyline.
        for i = nIdx - 1, 1, -1 do
          local n = nodes[i]
          if n.isLocked then                                                                        -- Do not go beyond any locked node in the -ve direction.
            break
          end
          local rat = min(1.0, max(0.0, 1.0 - cNode.p:distance(n.p) * fieldInv))
          n.rot = im.FloatPtr(n.rot[0] + dRot * rat)                                                -- Move this node by a distance-based ratio (based on the force field).
          n.rot = im.FloatPtr(min(nodeRotLimit, max(-nodeRotLimit, n.rot[0])))
        end

        -- Iterate from just above the central node, to the end of the polyline.
        for i = nIdx + 1, numNodes do
          local n = nodes[i]
          if n.isLocked then                                                                        -- Do not go beyond and locked node in the +ve direction.
            break
          end
          local rat = min(1.0, max(0.0, 1.0 - cNode.p:distance(n.p) * fieldInv))
          n.rot = im.FloatPtr(n.rot[0] + dRot * rat)                                                -- Move this node by a distance-based ratio (based on the force field).
          n.rot = im.FloatPtr(min(nodeRotLimit, max(-nodeRotLimit, n.rot[0])))
        end
      end
    end
    setDirty(road)
  end
end

-- Copies the group indices table.
local function copyGroupIdx(gp)
  local gOut = {}
  for i = 1, #gp do
    gOut[i] = gp[i]
  end
  return gOut
end

-- Deep copies a road.
local function copyRoad(r)

  -- Deep copy the nodes.
  local nodesCopy, nodes = {}, r.nodes
  local numNodes = #nodes
  for i = 1, numNodes do
    nodesCopy[i] = copyNode(nodes[i])
  end

  local aabb = nil
  if r.aabb then
    aabb = vec3(r.aabb.x, r.aabb.y)
  end

  -- Populate the deep copy.
  local rCopy = {}
  rCopy.name = r.name
  rCopy.displayName = im.ArrayChar(32, ffi.string(r.displayName))
  rCopy.profile = profileMgr.copyProfile(r.profile)

  rCopy.nodes = nodesCopy

  rCopy.isVis = im.BoolPtr(r.isVis[0])
  rCopy.isHidden = r.isHidden
  rCopy.isJctRoad = r.isJctRoad
  rCopy.treatAsInvisibleInEdit = r.treatAsInvisibleInEdit
  rCopy.aabb = aabb
  rCopy.isDrivable = r.isDrivable
  rCopy.isOverlay = r.isOverlay
  rCopy.groupIdx = copyGroupIdx(r.groupIdx or {})
  rCopy.granFactor = im.IntPtr(r.granFactor[0])
  rCopy.isDirty = true
  rCopy.laneKeys, rCopy.leftKeys, rCopy.rightKeys = profileMgr.computeLaneKeys(r.profile)
  rCopy.renderData = nil
  rCopy.isArc = r.isArc
  rCopy.isBridge = r.isBridge

  rCopy.overlayMat = r.overlayMat or defaultOverlayMaterial

  rCopy.isOverObject = im.BoolPtr(r.isOverObject[0])

  rCopy.isConformRoadToTerrain = im.BoolPtr(r.isConformRoadToTerrain[0])

  rCopy.isDisplayRoadSurface = im.BoolPtr(r.isDisplayRoadSurface[0])
  rCopy.isDisplayRoadOutline = im.BoolPtr(r.isDisplayRoadOutline[0])
  rCopy.isDisplayNodeSpheres = im.BoolPtr(r.isDisplayNodeSpheres[0])
  rCopy.isDisplayNodeNumbers = im.BoolPtr(r.isDisplayNodeNumbers[0])
  rCopy.isDisplayLaneInfo = im.BoolPtr(r.isDisplayLaneInfo[0])
  rCopy.isDisplayRefLine = im.BoolPtr(r.isDisplayRefLine[0])

  rCopy.isRigidTranslation = im.BoolPtr(r.isRigidTranslation[0])
  rCopy.forceField = im.FloatPtr(r.forceField[0])
  rCopy.isCivilEngRoads = im.BoolPtr(r.isCivilEngRoads[0])

  rCopy.bridgeWidth = im.FloatPtr(r.bridgeWidth[0])
  rCopy.bridgeDepth = im.FloatPtr(r.bridgeDepth[0])
  rCopy.bridgeArch = im.FloatPtr(r.bridgeArch[0])

  rCopy.isAllowTunnels = im.BoolPtr(r.isAllowTunnels[0])
  rCopy.tunnels = {}
  rCopy.radGran = im.IntPtr(r.radGran[0])
  rCopy.radOffset = im.FloatPtr(r.radOffset[0])
  rCopy.thickness = im.FloatPtr(r.thickness[0])
  rCopy.zOffsetFromRoad = im.FloatPtr(r.zOffsetFromRoad[0])
  rCopy.protrudeS = im.FloatPtr(r.protrudeS[0])
  rCopy.protrudeE = im.FloatPtr(r.protrudeE[0])
  rCopy.extraS = im.IntPtr(r.extraS[0])
  rCopy.extraE = im.IntPtr(r.extraE[0])

  return rCopy
end

-- Splits the road with the given index, at the given node position.
-- [This can be used for creating junctions].
local function splitRoad(rIdx, nIdx)

  -- Cache the old road data.
  local r = roads[rIdx]
  local oldName, nodes, profile = r.name, r.nodes, r.profile
  local isConformRoadToTerrain = r.isConformRoadToTerrain[0]
  local isAllowTunnels = r.isAllowTunnels[0]
  local isRigidTranslation = r.isRigidTranslation[0]
  local forceField = r.forceField[0]
  local isCivilEngRoads = r.isCivilEngRoads[0]

  -- Create the first new road (road A).
  local copiedNodesA = {}
  for i = 1, nIdx do
    copiedNodesA[i] = copyNode(nodes[i])
  end
  local roadA = createRoadFromProfile(profile)
  roadA.displayName = im.ArrayChar(32, ffi.string(r.displayName) .. ' [A]')
  roadA.nodes = copiedNodesA

  roadA.isDrivable = r.isDrivable
  roadA.isOverlay = r.isOverlay
  roadA.groupIdx = {}
  roadA.isArc = r.isArc
  roadA.isBridge = r.isBridge

  roadA.granFactor = im.IntPtr(r.granFactor[0])
  roadA.laneKeys, roadA.leftKeys, roadA.rightKeys = profileMgr.computeLaneKeys(r.profile)

  roadA.overlayMat = r.overlayMat or defaultOverlayMaterial

  roadA.isOverObject = im.BoolPtr(r.isOverObject[0])

  roadA.isConformRoadToTerrain = im.BoolPtr(isConformRoadToTerrain)

  roadA.isDisplayRoadSurface = im.BoolPtr(r.isDisplayRoadSurface[0])
  roadA.isDisplayRoadOutline = im.BoolPtr(r.isDisplayRoadOutline[0])
  roadA.isDisplayNodeSpheres = im.BoolPtr(r.isDisplayNodeSpheres[0])
  roadA.isDisplayNodeNumbers = im.BoolPtr(r.isDisplayNodeNumbers[0])
  roadA.isDisplayLaneInfo = im.BoolPtr(r.isDisplayLaneInfo[0])
  roadA.isDisplayRefLine = im.BoolPtr(r.isDisplayRefLine[0])

  roadA.isRigidTranslation = im.BoolPtr(isRigidTranslation)
  roadA.forceField = im.FloatPtr(forceField)
  roadA.isCivilEngRoads = im.BoolPtr(isCivilEngRoads)

  roadA.bridgeWidth = im.FloatPtr(r.bridgeWidth[0])
  roadA.bridgeDepth = im.FloatPtr(r.bridgeDepth[0])
  roadA.bridgeArch = im.FloatPtr(r.bridgeArch[0])

  roadA.isAllowTunnels = im.BoolPtr(isAllowTunnels)
  roadA.tunnels = {}
  roadA.radGran = im.IntPtr(r.radGran[0])
  roadA.radOffset = im.FloatPtr(r.radOffset[0])
  roadA.thickness = im.FloatPtr(r.thickness[0])
  roadA.zOffsetFromRoad = im.FloatPtr(r.zOffsetFromRoad[0])
  roadA.protrudeS = im.FloatPtr(r.protrudeS[0])
  roadA.protrudeE = im.FloatPtr(r.protrudeE[0])
  roadA.extraS = im.IntPtr(r.extraS[0])
  roadA.extraE = im.IntPtr(r.extraE[0])

  -- Create the second new road (road B).
  local copiedNodesB, numNodes, ctr = {}, #nodes, 1
  for i = nIdx, numNodes do
    copiedNodesB[ctr] = copyNode(nodes[i])
    ctr = ctr + 1
  end
  local roadB = createRoadFromProfile(profileMgr.copyProfile(profile))
  roadB.displayName = im.ArrayChar(32, ffi.string(r.displayName) .. ' [B]')
  roadB.nodes = copiedNodesB

  roadB.isDrivable = r.isDrivable
  roadB.isOverlay = r.isOverlay
  roadB.groupIdx = {}
  roadB.isArc = r.isArc
  roadB.isBridge = r.isBridge

  roadB.granFactor = im.IntPtr(r.granFactor[0])
  roadB.laneKeys, roadB.leftKeys, roadB.rightKeys = profileMgr.computeLaneKeys(r.profile)

  roadB.overlayMat = r.overlayMat or defaultOverlayMaterial

  roadB.isOverObject = im.BoolPtr(r.isOverObject[0])

  roadB.isConformRoadToTerrain = im.BoolPtr(isConformRoadToTerrain)

  roadB.isDisplayRoadSurface = im.BoolPtr(r.isDisplayRoadSurface[0])
  roadB.isDisplayRoadOutline = im.BoolPtr(r.isDisplayRoadOutline[0])
  roadB.isDisplayNodeSpheres = im.BoolPtr(r.isDisplayNodeSpheres[0])
  roadB.isDisplayNodeNumbers = im.BoolPtr(r.isDisplayNodeNumbers[0])
  roadB.isDisplayLaneInfo = im.BoolPtr(r.isDisplayLaneInfo[0])
  roadB.isDisplayRefLine = im.BoolPtr(r.isDisplayRefLine[0])

  roadB.isRigidTranslation = im.BoolPtr(isRigidTranslation)
  roadB.forceField = im.FloatPtr(forceField)
  roadB.isCivilEngRoads = im.BoolPtr(isCivilEngRoads)

  roadB.bridgeWidth = im.FloatPtr(r.bridgeWidth[0])
  roadB.bridgeDepth = im.FloatPtr(r.bridgeDepth[0])
  roadB.bridgeArch = im.FloatPtr(r.bridgeArch[0])

  roadB.isAllowTunnels = im.BoolPtr(isAllowTunnels)
  roadB.tunnels = {}
  roadB.radGran = im.IntPtr(r.radGran[0])
  roadB.radOffset = im.FloatPtr(r.radOffset[0])
  roadB.thickness = im.FloatPtr(r.thickness[0])
  roadB.zOffsetFromRoad = im.FloatPtr(r.zOffsetFromRoad[0])
  roadB.protrudeS = im.FloatPtr(r.protrudeS[0])
  roadB.protrudeE = im.FloatPtr(r.protrudeE[0])
  roadB.extraS = im.IntPtr(r.extraS[0])
  roadB.extraE = im.IntPtr(r.extraE[0])

  -- Repel the two split endpoints slightly.
  local np = nodes[nIdx].p
  local tgtA, tgtB = nodes[nIdx - 1].p - np, nodes[nIdx + 1].p - np
  tgtA:normalize()
  tgtB:normalize()
  roadA.nodes[nIdx].p = roadA.nodes[nIdx].p + tgtA * splitRepelDist
  roadB.nodes[1].p = roadB.nodes[1].p + tgtB * splitRepelDist

  -- Add the two new roads and remove the old road.
  local numRoads = #roads
  local rIdxA, rIdxB = numRoads + 1, numRoads + 2
  roads[rIdxA], roads[rIdxB] = roadA, roadB
  roadMap[roadA.name], roadMap[roadB.name] = rIdxA, rIdxB
  recomputeMap()

  roadMeshMgr.tryRemove(oldName)
  staticMeshMgr.tryRemove(oldName)                                                                  -- If any static mesh was created for this road on finalise, remove them now.
  removeRoad(oldName)
  setDirty(roadA)
  setDirty(roadB)
end

-- Flips the direction of the road with the given index.
-- [This will change the lateral position of the road reference line, and may be useful for fixing import errors].
local function flipRoad(rIdx)
  local r = roads[rIdx]
  local nodes, poly, ctr = r.nodes, {}, 1
  for i = #nodes, 1, -1 do
    poly[ctr] = nodes[i]
    ctr = ctr + 1
  end
  if r.renderData then
    table.clear(r.renderData)
  end
  r.nodes = poly
  geom.computeRoadRenderData(r)
end

-- Manages the updating of roads.
-- [Updates the render data for all roads which require it].
local function updateRoads()
  for i = 1, #roads do
    local r = roads[i]
    if r.isDirty and #r.nodes > 1 then                                                              -- If the road has been marked as requiring updating.
      geom.computeRoadRenderData(r)                                                                 -- Compute the relevant geometric data needed for rendering the road.
      r.isDirty = false                                                                             -- The road has been updated, so we mark this in the road instance.
      profileMgr.updateCondition(r)
      if r.isBridge then
        local folder = scenetree.findObject("Road Architect - Bridge " .. tostring(r.name))
        if not folder then
          folder = createObject("SimGroup")
          folder:registerObject("Road Architect - Bridge " .. tostring(r.name))
          scenetree.MissionGroup:addObject(folder)
        end
        roadMeshMgr.updateBridge(r, folder)                                                         -- If this is a bridge, update the bridge mesh.
      end
    end
  end
end

-- Clears the bridges structure.
local function clearBridges() roadMeshMgr.clearBridges() end

-- Removes all road meshes from the scene and clears the roads structure.
local function removeAll()
  decMgr.tryRemoveAll()                                                                             -- First, remove all decals, if any exist.

  for i = 1, #roads do
    local road = roads[i]
    local roadName = road.name
    roadMeshMgr.tryRemove(roadName)                                                                 -- Remove all meshes, if any exist.
    staticMeshMgr.tryRemove(roadName)                                                               -- If any static mesh was created for this road on finalise, remove them now.

    if road.isBridge then                                                                           -- If this is a bridge, remove the folder from the scene tree.
      roadMeshMgr.tryRemoveBridge(road.name)
      local folder = scenetree.findObject("Road Architect - Bridge " .. tostring(road.name))
      if folder then
        folder:delete()
      end
    end

    local tunnels = road.tunnels
    for j = 1, #tunnels do                                                                          -- If any tunnel meshes were created for this road on finalise, remove them now.
      local t = tunnels[j]
      tMesh.tryRemove(i, t.name)
    end
  end
  clearBridges()

  table.clear(roads)                                                                                -- Clear the roads container.
  recomputeMap()                                                                                    -- Re-compute the roads map.
end

-- Updates the camera position upon changing of audition group.
local function updateCameraPose()
  gView:set(0.0, 30.0, auditionHeight + 15.0)
  local gRot = quatFromDir(auditionVec - gView)
  commands.setFreeCamera()
  core_camera.setPosRot(0, gView.x, gView.y, gView.z, gRot.x, gRot.y, gRot.z, gRot.w)
end

-- Rotate the audition camera around the audition centroid.
local function rotateAuditionCamera(ang)
  local x, y, s, c = gView.x, gView.y, sin(ang), cos(ang)
  auditionCamPos:set(x * c - y * s, x * s + y * c, gView.z)
  local gRot = quatFromDir(auditionVec - auditionCamPos)
  core_camera.setPosRot(0, auditionCamPos.x, auditionCamPos.y, auditionCamPos.z, gRot.x, gRot.y, gRot.z, gRot.w)
end

-- Creates/updates a temporary linear road section, used for displaying and editing profiles.
local function manageTempRoadSection(pIdx)

  -- Fetch the selected lateral road profile.
  local profile = profileMgr.profiles[pIdx]

  -- Fetch the index of the temp road.
  local rIdx = roadMap[tempRoadName]
  if not rIdx or isAuditionProfileDirty then
    isAuditionProfileDirty = false
    local widths, heightsL, heightsR = profileMgr.getWAndHByKey(profile)
    local n1 = {
      p = temp_P1, isLocked = true, rot = im.FloatPtr(0.0),
      widths = widths, heightsL = heightsL, heightsR = heightsR,
      incircleRad = im.FloatPtr(1.0), isAutoBanked = false, offset = 0.0 }
    local n2 = {
      p = temp_P2, isLocked = true, rot = im.FloatPtr(0.0),
      widths = widths, heightsL = heightsL, heightsR = heightsR,
      incircleRad = im.FloatPtr(1.0), isAutoBanked = false, offset = 0.0 }
    local laneKeys, leftKeys, rightKeys = profileMgr.computeLaneKeys(profile)
    if not rIdx then
      rIdx = #roads + 1
    end
    roads[rIdx] = {
      displayName = im.ArrayChar(32, 'temp'),
      isVis = im.BoolPtr(true),
      isHidden = true,
      isJctRoad = false,
      treatAsInvisibleInEdit = false,
      aabb = vec3((n1.p.x + n2.p.x) * 0.5, (n1.p.y + n2.p.y) * 0.5),
      isDrivable = false,
      isOverlay = false,
      groupIdx = {},
      granFactor = im.IntPtr(1),
      renderData = nil,
      laneKeys = laneKeys, leftKeys = leftKeys, rightKeys = rightKeys,
      isDirty = true,

      isArc = false,
      isBridge = false,

      type = nil,
      name = tempRoadName,
      nodes = { n1, n2 },

      profile = profile,

      overlayMat = defaultOverlayMaterial,

      isOverObject = im.BoolPtr(true),

      isConformRoadToTerrain = im.BoolPtr(false),

      isDisplayRoadSurface = im.BoolPtr(true),
      isDisplayRoadOutline = im.BoolPtr(true),
      isDisplayNodeSpheres = im.BoolPtr(false),
      isDisplayRefLine = im.BoolPtr(true),
      isDisplayLaneInfo = im.BoolPtr(true),
      isDisplayNodeNumbers = im.BoolPtr(false),

      isRigidTranslation = im.BoolPtr(false),
      forceField = im.FloatPtr(1.0),
      isCivilEngRoads = im.BoolPtr(false),

      bridgeWidth = im.FloatPtr(5.0),
      bridgeDepth = im.FloatPtr(5.0),
      bridgeArch = im.FloatPtr(1.0),

      isAllowTunnels = im.BoolPtr(false),
      tunnels = {},
      radGran = im.IntPtr(15),
      radOffset = im.FloatPtr(0.0),
      thickness = im.FloatPtr(1.0),
      zOffsetFromRoad = im.FloatPtr(0.0),
      protrudeS = im.FloatPtr(0.0),
      protrudeE = im.FloatPtr(0.0),
      extraS = im.IntPtr(2),
      extraE = im.IntPtr(2)
    }

    setDirty(roads[rIdx])

    -- Recompute the road map.
    recomputeMap()

    updateCameraPose()
  end

  rotateAuditionCamera(camRotAngle)
  camRotAngle = camRotAngle + camRotInc
  if camRotAngle > twoPi then
    camRotAngle = camRotAngle - twoPi
  end
end

-- Update the bridge parameters after a change (width and depth).
local function updateBridgeParameters(road)
  local nodes = road.nodes
  if nodes[1] then
    nodes[1].widths[-1] = im.FloatPtr(road.bridgeWidth)
    nodes[1].widths[1] = im.FloatPtr(road.bridgeWidth)
    nodes[1].heightsL[-1] = im.FloatPtr(road.bridgeDepth)
    nodes[1].heightsR[-1] = im.FloatPtr(road.bridgeDepth)
    nodes[1].heightsL[1] = im.FloatPtr(road.bridgeDepth)
    nodes[1].heightsR[1] = im.FloatPtr(road.bridgeDepth)
  end
  if nodes[2] then
    nodes[2].widths[-1] = im.FloatPtr(road.bridgeWidth)
    nodes[2].widths[1] = im.FloatPtr(road.bridgeWidth)
    nodes[2].heightsL[-1] = im.FloatPtr(road.bridgeDepth)
    nodes[2].heightsR[-1] = im.FloatPtr(road.bridgeDepth)
    nodes[2].heightsL[1] = im.FloatPtr(road.bridgeDepth)
    nodes[2].heightsR[1] = im.FloatPtr(road.bridgeDepth)
  end
end

-- Create a multi-select (from given polygon) and allow it to be manipulated.
local function createMultiSelect(gPolygon)

  -- Convert and copy the polygon to 2D.
  local poly2D = {}
  for i = 1, #gPolygon do
    local p = gPolygon[i]
    poly2D[i] = vec3(p.x, p.y, 0.0)
  end

  -- Collect all nodes (from all roads) which are inside the polygon.
  -- [This is stored in this module, as a table with road and node indices for each node].
  table.clear(multi)
  local ctr = 1
  for i = 1, #roads do
    local nodes = roads[i].nodes
    for j = 1, #nodes do
      if nodes[j].p:inPolygon(poly2D) then
        multi[ctr] = { r = i, n = j }
        ctr = ctr + 1
      end
    end
  end
end

-- Gets the centroid of the multi-selection.
local function getMultiSelectionCentroid()
  local xMin, xMax, yMin, yMax, zMin, zMax = 1e99, -1e99, 1e99, -1e99, 1e99, -1e99
  for i = 1, #multi do
    local mp = multi[i]
    local rIdx, nIdx = mp.r, mp.n
    if roads[rIdx] and roads[rIdx].nodes[nIdx] then
      local p = roads[rIdx].nodes[nIdx].p
      local x, y, z = p.x, p.y, p.z
      xMin, xMax, yMin, yMax, zMin, zMax = min(xMin, x), max(xMax, x), min(yMin, y), max(yMax, y), min(zMin, z), max(zMax, z)
    end
  end
  return vec3((xMin + xMax) * 0.5, (yMin + yMax) * 0.5, (zMin + zMax) * 0.5)
end

-- Re-computes the road render data, for all roads, from fresh.
local function computeAllRoadRenderData()
  local numRoads = #roads
  for i = 1, numRoads do
    local r = roads[i]
    if #r.nodes > 1 then
      geom.computeRoadRenderData(r)
    end
  end
end

-- Computes the road render data for a single road.
local function computeRoadRenderDataSingle(i) geom.computeRoadRenderData(roads[i]) end

-- Removes a group index from the given road.
local function removeGroupFromRoad(r, groupIdx)
  local gI = r.groupIdx
  for i = #gI, 1, -1 do
    if gI[i] == groupIdx then
      table.remove(gI, i)
    end
  end
end

-- Fits nodes/widths (using C-R centripetal) to the given decal road (uses standard C-R).
local function reparameteriseDecalRoad(nodes, widths)
  local distSqTol = reparameteriseNodeDist * reparameteriseNodeDist
  local isError = true
  local iter = 0
  local maxIter = 10
  while isError or iter > maxIter do
    local nOut, wOut, ctr = { nodes[1] }, { widths[1] }, 2
    isError = false
    for i = 2, #nodes do
      local p1, p2 = nodes[i - 1], nodes[i]
      local dSq = p1:squaredDistance(p2)
      if dSq > distSqTol then
        isError = true
        local pp1, pp2, pp3, pp4 = nodes[max(1, i - 2)], p1, p2, nodes[min(#nodes, i + 1)]
        local p = catmullRom(pp1, pp2, pp3, pp4, 0.5)
        tmp0:set(pp1.x, pp1.y, widths[max(1, i - 2)])
        tmp1:set(pp1.x, pp1.y, widths[i - 1])
        tmp2:set(pp1.x, pp1.y, widths[i])
        tmp3:set(pp1.x, pp1.y, widths[min(#nodes, i + 1)])
        local w = catmullRom(tmp0, tmp1, tmp2, tmp3, 0.5).z
        nOut[ctr], wOut[ctr] = p, w
        ctr = ctr + 1
      end
      nOut[ctr], wOut[ctr] = p2, widths[i]
      ctr = ctr + 1
    end
    nodes, widths = nOut, wOut
    iter = iter + 1
  end
  return nodes, widths
end

-- Set the widths for each lane, at a node.
local function setWidthsAtNode(hWidth, numLeftLanes, numRightLanes, leftKeys, rightKeys)
  local leftLaneWidth = hWidth / numLeftLanes
  local rightLaneWidth = hWidth / numRightLanes
  local widths = {}
  for i = 1, #leftKeys do
    local lIdx = leftKeys[i]
    widths[lIdx] = im.FloatPtr(leftLaneWidth)
  end
  for i = 1, #rightKeys do
    local lIdx = rightKeys[i]
    widths[lIdx] = im.FloatPtr(rightLaneWidth)
  end
  return widths
end

-- Decrease the group indices stored in the roads, after the removal of a group.
local function updateRoadsAfterRemovingGroup(placedGroups)
  for i = 1, #roads do
    table.clear(roads[i].groupIdx)
  end
  for i = 1, #placedGroups do
    local gList = placedGroups[i].list
    for j = 1, #gList do
      local r = roads[roadMap[gList[j].r]]
      util.tryAddGroupIdxToRoad(r, i)
    end
  end
end

-- Updates the multi-selection container, after a road/group of roads was removed.
local function updateMultiAfterRemove()
  for i = #multi, 1, -1 do
    local m = multi[i]
    if not roads[m.r] or not roads[m.r].nodes[m.n] then
      table.remove(multi, i)
    end
  end
end

-- Converts all decal roads (from scene tree) to Road Architect style roads.
-- [Note: this removes the original decal roads from the scene tree.]
local function convertDecalRoads2RoadArchitect()
  for i = 1, tableSize(editor.selection.object) do                                                  -- Iterate over all the selected objects in the scenetree.
    local selObj = editor.selection.object[i]
		local sel = scenetree.findObjectById(selObj)
		if sel and sel:getClassName() == "DecalRoad" then                                               -- Filter into decal roads only.

      local nodes, widths, ctr = {}, {}, 1                                                          -- Grab all the nodes and widths of this decal road.
			for _, node in ipairs(editor.getNodes(sel)) do
        nodes[ctr], widths[ctr] = node.pos, node.width
        ctr = ctr + 1
      end

      nodes, widths = reparameteriseDecalRoad(nodes, widths)                                        -- Fit the road architect nodes to the selected decal road.

      local numLeftLanes = sel:getField('lanesLeft', 0) or 1
      local numRightLanes = sel:getField('lanesRight', 0) or 1
      local profile = profileMgr.createProfileFromDecalData(numLeftLanes, numRightLanes)            -- Create the road from a profile based on the decal road metadata.
      local newRoad = createRoadFromProfile(profile)

      local rIdx = #roads + 1                                                                       -- Add the newly-created road to the collections.
      roads[rIdx] = newRoad
      roadMap[newRoad.name] = rIdx

      local _, leftKeys, rightKeys = profileMgr.computeLaneKeys(profile)
      for j = 1, #nodes do
        addNodeToRoad(rIdx, nodes[j])
      end
      for j = 1, #nodes do
        roads[rIdx].nodes[j].widths = setWidthsAtNode(widths[j] * 0.5, numLeftLanes, numRightLanes, leftKeys, rightKeys)
      end
      --sel:delete()                                                                                -- Remove the original decal road from the scene tree. THIS HAS BEEN SWITCHED OFF.
		end
	end
end

-- Vertically-offsets all roads to the terrain.
local function offsetRoads2Terrain()
  for _, road in ipairs(roads) do
    road.isConformRoadToTerrain = im.BoolPtr(true)
    setDirty(road)
  end
end

-- Vertically-offsets all roads to a custom value.
local function offsetByValue(offset)
  for _, road in ipairs(roads) do
    local nodes = road.nodes
    local numNodes = #nodes
    for i = 1, numNodes do
      nodes[i].p.z = nodes[i].p.z + offset
    end
    setDirty(road)
  end
end

-- Switches to the 'finalised' state.
-- [All requested procedural meshes are created, the collision mesh is re-computed, then all requested decals are laid].
local function finalise()

  computeAllRoadRenderData()

  -- Create a group (folder) in the scenetree to store every asset related to this road.
  local rGroups = {}
  for i = 1, #roads do
    if not roads[i].isBridge then
      rGroups[i] = createObject("SimGroup")
      rGroups[i]:registerObject("Road Architect - Road " .. tostring(i))
      scenetree.MissionGroup:addObject(rGroups[i])
    end
  end

  -- Do the bridges first.
  for i = 1, #roads do
    local r = roads[i]
    if r.isBridge then
      local nodes = r.nodes
      if #nodes > 1 then
        roadMeshMgr.createRoad(r, i, rGroups[i])
      end
    end
  end

  -- Now do the roads.
  for i = 1, #roads do
    local r = roads[i]
    if not r.isOverlay and not r.isBridge then

      -- Create any procedural/static meshes.
      local nodes = r.nodes
      if #nodes > 1 then
        roadMeshMgr.createRoad(r, i, rGroups[i])
        staticMeshMgr.createStaticMeshes(r, i, rGroups[i])

        -- Create the procedural meshes for any auto tunnels.
        if #r.tunnels > 0 then
          for j = 1, #r.tunnels do
            local t = r.tunnels[j]
            tMesh.createTunnel(i, t.name, r.renderData, t, rGroups[i])
          end
        end

      end
    end
  end

  -- Now that the meshes exist in the scene, re-compute the collision mesh to take them into account.
  be:reloadCollision()

  -- Lastly, now that the collision meshes has been updated, lay down all the requested decals.
  for i = 1, #roads do
    local r = roads[i]
    if #r.nodes > 1 and not r.isBridge then
      decMgr.createDecal(r, rGroups[i])
    end
  end

  -- Reload the navgraph.
  map.reset()
end

-- Leaves the 'finalised' state and returns to the 'edit' state.
local function unfinalise()
  for i = 1, #roads do
    local road = roads[i]
    if not road.isBridge then
      local roadName = road.name
      decMgr.tryRemove(roadName)                                                                    -- If any decal was created for this road on finalise, remove it now.

      roadMeshMgr.tryRemove(roadName)                                                               -- If any mesh was created for this road on finalise, remove it now.
      staticMeshMgr.tryRemove(roadName)                                                             -- If any static mesh was created for this road on finalise, remove them now.

      -- If any auto tunnel meshes were created, remove them now.
      local tunnels = road.tunnels
      for j = 1, #tunnels do
        local t = tunnels[j]
        tMesh.tryRemove(i, t.name)
      end

      setDirty(road)
    end
  end

  decMgr.removeTemplates()

  -- Remove any folders.
  for i = 1, #roads do
    if not roads[i].isBridge then
      local folder = scenetree.findObject("Road Architect - Road " .. tostring(i))
      if folder then
        folder:delete()
      end
    end
  end

  be:reloadCollision()                                                                              -- Re-load the collision mesh now that the meshes have been removed.
  map.reset({})                                                                                     -- Clear the navgraph.
end

-- Serialises a node.
local function serialiseNode(n)
  local widths, heightsL, heightsR = n.widths, n.heightsL, n.heightsR                               -- Serialise the widths/heights structures first.
  local serWidths, serHeightsL, serHeightsR = {}, {}, {}
  for i = -20, 20 do
    local w = widths[i]
    if w then
      local iS = tostring(i)
      serWidths[iS], serHeightsL[iS], serHeightsR[iS] = w[0], heightsL[i][0], heightsR[i][0]
    end
  end
  local p = n.p
  return {                                                                                          -- Now serialise the node container.
    posX = p.x, posY = p.y, posZ = p.z,
    isLocked = n.isLocked,
    rot = n.rot[0],
    widths = serWidths, heightsL = serHeightsL, heightsR = serHeightsR,
    incircleRad = n.incircleRad[0],
    isAutoBanked = n.isAutoBanked,
    offset = n.offset }
end

-- Deserialises a node.
local function deserialiseNode(nSer)
  local serWidths, serHeightsL, serHeightsR = nSer.widths, nSer.heightsL, nSer.heightsR             -- De-serialise the widths/heights structures first.
  local widths, heightsL, heightsR = {}, {}, {}
  for i = -20, 20 do
    local iS = tostring(i)
    local w = serWidths[iS]
    if w then
      widths[i] = im.FloatPtr(w)
      heightsL[i], heightsR[i] = im.FloatPtr(serHeightsL[iS]), im.FloatPtr(serHeightsR[iS])
    end
  end
  return {                                                                                          -- Now de-serialise the node container.
    p = vec3(nSer.posX, nSer.posY, nSer.posZ),
    isLocked = nSer.isLocked or false,
    rot = im.FloatPtr(nSer.rot),
    widths = widths, heightsL = heightsL, heightsR = heightsR,
    incircleRad = im.FloatPtr(nSer.incircleRad or 0.0),
    isAutoBanked = nSer.isAutoBanked or false,
    offset = nSer.offset or 0.0 }
end

-- Sets the visibility of all roads (master switch).
local function setRoadsVisibilityMaster(isShow)
  for i = 1, #roads do
    local r = roads[i]
    if not r.isBridge and not r.isOverlay then
      r.isVis = im.BoolPtr(isShow)
    end
  end
end

-- Sets the visibility of all bridges (master switch).
local function setBridgesVisibilityMaster(isShow)
  for i = 1, #roads do
    local r = roads[i]
    if r.isBridge then
      r.isVis = im.BoolPtr(isShow)
    end
  end
end

-- Sets the visibility of all overlays (master switch).
local function setOverlaysVisibilityMaster(isShow)
  for i = 1, #roads do
    local r = roads[i]
    if r.isOverlay then
      r.isVis = im.BoolPtr(isShow)
    end
  end
end

-- Serializes a road.
local function serialiseRoad(r)
  local serNodes, nodes = {}, r.nodes
  local numNodes = #nodes
  for i = 1, numNodes do
    serNodes[i] = serialiseNode(nodes[i])
  end
  return {
    displayName = ffi.string(r.displayName),
    isVis = r.isVis[0],
    isHidden = r.isHidden,
    isJctRoad = r.isJctRoad,
    treatAsInvisibleInEdit = r.treatAsInvisibleInEdit,
    groupIdx = copyGroupIdx(r.groupIdx),
    granFactor = r.granFactor[0],

    isDrivable = r.isDrivable,
    isOverlay = r.isOverlay,
    isArc = r.isArc,
    isBridge = r.isBridge,

    name = r.name,
    nodes = serNodes,

    profile = profileMgr.serialiseProfile(r.profile),

    overlayMat = r.overlayMat,

    isOverObject = r.isOverObject[0],

    isConformRoadToTerrain = r.isConformRoadToTerrain[0],

    isDisplayRoadSurface = r.isDisplayRoadSurface[0],
    isDisplayRoadOutline = r.isDisplayRoadOutline[0],
    isDisplayNodeSpheres = r.isDisplayNodeSpheres[0],
    isDisplayNodeNumbers = r.isDisplayNodeNumbers[0],
    isDisplayRefLine = r.isDisplayRefLine[0],

    isDisplayLaneInfo = r.isDisplayLaneInfo[0],
    isRigidTranslation = r.isRigidTranslation[0],
    forceField = r.forceField[0],
    isCivilEngRoads = r.isCivilEngRoads[0],

    bridgeWidth = r.bridgeWidth[0],
    bridgeDepth = r.bridgeDepth[0],
    bridgeArch = r.bridgeArch[0],

    isAllowTunnels = r.isAllowTunnels[0],
    radGran = r.radGran[0],
    radOffset = r.radOffset[0],
    thickness = r.thickness[0],
    zOffsetFromRoad = r.zOffsetFromRoad[0],
    protrudeS = r.protrudeS[0],
    protrudeE = r.protrudeE[0],
    extraS = r.extraS[0],
    extraE = r.extraE[0]
  }
end

-- Deserializes a road.
local function deserialiseRoad(rSer)
  local nodes, serNodes = {}, rSer.nodes
  local numNodes = #serNodes
  for i = 1, numNodes do
    nodes[i] = deserialiseNode(serNodes[i])
  end
  local profile = profileMgr.deserialiseProfile(rSer.profile)
  local laneKeys, leftKeys, rightKeys = profileMgr.computeLaneKeys(profile)
  if not rSer.isVis then
    rSer.isVis = true
  end
  return {
    displayName = im.ArrayChar(32, rSer.displayName or 'New Road'),
    isDirty = true,
    isVis = im.BoolPtr(rSer.isVis),
    isHidden = rSer.isHidden,
    isJctRoad = rSer.isJctRoad,
    treatAsInvisibleInEdit = rSer.treatAsInvisibleInEdit,
    groupIdx = rSer.groupIdx,
    granFactor = im.IntPtr(rSer.granFactor or 1),

    isDrivable = rSer.isDrivable,
    isOverlay = rSer.isOverlay,
    isArc = rSer.isArc or false,
    isBridge = rSer.isBridge or false,

    renderData = nil,
    laneKeys = laneKeys, leftKeys = leftKeys, rightKeys = rightKeys,

    name = rSer.name,
    nodes = nodes,

    profile = profile,

    overlayMat = rSer.overlayMat or defaultOverlayMaterial,

    isOverObject = im.BoolPtr(rSer.isOverObject or false),

    isConformRoadToTerrain = im.BoolPtr(rSer.isConformRoadToTerrain),

    isDisplayRoadSurface = im.BoolPtr(rSer.isDisplayRoadSurface),
    isDisplayRoadOutline = im.BoolPtr(rSer.isDisplayRoadOutline),
    isDisplayNodeSpheres = im.BoolPtr(rSer.isDisplayNodeSpheres),
    isDisplayNodeNumbers = im.BoolPtr(rSer.isDisplayNodeNumbers),
    isDisplayRefLine = im.BoolPtr(rSer.isDisplayRefLine),

    isDisplayLaneInfo = im.BoolPtr(rSer.isDisplayLaneInfo or false),
    isRigidTranslation = im.BoolPtr(rSer.isRigidTranslation or false),
    forceField = im.FloatPtr(rSer.forceField or 0.1),
    isCivilEngRoads = im.BoolPtr(rSer.isCivilEngRoads or false),

    bridgeWidth = im.FloatPtr(rSer.bridgeWidth or 5.0),
    bridgeDepth = im.FloatPtr(rSer.bridgeDepth or 0.5),
    bridgeArch = im.FloatPtr(rSer.bridgeArch or 1.0),

    isAllowTunnels = im.BoolPtr(rSer.isAllowTunnels or false),
    tunnels = {},
    radGran = im.IntPtr(rSer.radGran or 15),
    radOffset = im.FloatPtr(rSer.radOffset or 0.0),
    thickness = im.FloatPtr(rSer.thickness or 1.0),
    zOffsetFromRoad = im.FloatPtr(rSer.zOffsetFromRoad or 0.0),
    protrudeS = im.FloatPtr(rSer.protrudeS or 0.0),
    protrudeE = im.FloatPtr(rSer.protrudeE or 0.0),
    extraS = im.IntPtr(rSer.extraS or 2),
    extraE = im.IntPtr(rSer.extraE or 2),
    tOffset = im.FloatPtr(rSer.lampPostVertOffset or 1.0)
  }
end


-- Public interface.
M.roads =                                                 roads
M.map =                                                   roadMap
M.multi =                                                 multi

M.getCurrentMap =                                         getCurrentMap
M.setCurrentMap =                                         setCurrentMap

M.isAuditionProfileDirty =                                isAuditionProfileDirty

M.getTree =                                               getTree
M.removeTree =                                            removeTree

M.createRoadFromProfile =                                 createRoadFromProfile
M.createRoadFromTemplate =                                createRoadFromTemplate
M.setDirty =                                              setDirty
M.setAllDirty =                                           setAllDirty
M.setAuditionProfileDirty =                               setAuditionProfileDirty
M.addNodeToRoad =                                         addNodeToRoad
M.addNodeToRoadAtIdx =                                    addNodeToRoadAtIdx
M.isMouseAtIntermediatePos =                              isMouseAtIntermediatePos
M.setLocalWidth =                                         setLocalWidth
M.copyNode =                                              copyNode
M.updateWAndHToNewProfile =                               updateWAndHToNewProfile

M.computeRoadAABB_2D =                                    computeRoadAABB_2D
M.computeAABB2D =                                         computeAABB2D
M.getRoadsFromGroup =                                     getRoadsFromGroup

M.offsetRoads2Terrain =                                   offsetRoads2Terrain
M.offsetByValue =                                         offsetByValue
M.recomputeMap =                                          recomputeMap
M.removeRoad =                                            removeRoad
M.removeNode =                                            removeNode
M.addIntermediateNode =                                   addIntermediateNode
M.clearAllRoads =                                         clearAllRoads
M.removeHiddenRoads =                                     removeHiddenRoads
M.goToRoad =                                              goToRoad
M.moveRoad =                                              moveRoad
M.adjustHeight =                                          adjustHeight
M.adjustLateralRotation =                                 adjustLateralRotation
M.copyRoad =                                              copyRoad
M.splitRoad =                                             splitRoad
M.flipRoad =                                              flipRoad
M.updateRoads =                                           updateRoads
M.clearBridges =                                          clearBridges
M.removeAll =                                             removeAll
M.manageTempRoadSection =                                 manageTempRoadSection
M.updateBridgeParameters =                                updateBridgeParameters
M.createMultiSelect =                                     createMultiSelect
M.getMultiSelectionCentroid =                             getMultiSelectionCentroid
M.removeGroupFromRoad =                                   removeGroupFromRoad
M.updateRoadsAfterRemovingGroup =                         updateRoadsAfterRemovingGroup
M.updateMultiAfterRemove =                                updateMultiAfterRemove

M.convertDecalRoads2RoadArchitect =                       convertDecalRoads2RoadArchitect

M.finalise =                                              finalise
M.unfinalise =                                            unfinalise

M.computeAllRoadRenderData =                              computeAllRoadRenderData
M.computeRoadRenderDataSingle =                           computeRoadRenderDataSingle

M.setRoadsVisibilityMaster =                              setRoadsVisibilityMaster
M.setBridgesVisibilityMaster =                            setBridgesVisibilityMaster
M.setOverlaysVisibilityMaster =                           setOverlaysVisibilityMaster

M.serialiseRoad =                                         serialiseRoad
M.deserialiseRoad =                                       deserialiseRoad

return M