-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local crossroadArcGran = 2                                                                                          -- The granularity of sidewalk bevelled arc corner sections.

local defaultPedXMaterial = 'crossing_white'                                                                        -- The default material for pedestrian crossings.
local defaultEdgeMaterial = 'm_line_white'                                                                          -- The default material for edge lines.
local defaultEdgeBlendMaterial = 'm_road_asphalt_edge'                                                              -- The default edge blending material.
local defaultLaneArrowMaterial = 'm_decal_roadmarkings_01'                                                          -- The default material for lane arrow decals.
local defaultSeperatorMaterial = 'italy_road_markings_zebra_diagonal'                                               -- The default material for separator lanes.

local defaultCrashBarrierPath = '/art/shapes/objects/italy_guardrails_basic.dae'                                    -- The default static mesh for crash barrier post + plate sections.

local trafficBoomMeshPath = '/art/shapes/objects/s_trafficlight_boom_sn.dae'                                        -- The path to the traffic light boom mesh.

local defaultPedXSign = '/art/shapes/garage_and_dealership/Clutter/italy_traf_sign_warn_zebra_warning.dae'          -- The default static mesh for 'Pedestrian Crossing' signs.
local defaultCrossroadsSign = '/art/shapes/garage_and_dealership/Clutter/italy_traf_sign_crossroad.dae'             -- The default static mesh for 'Crossroads' signs.
local defaultTJunctionSignL = '/art/shapes/garage_and_dealership/Clutter/italy_traf_sign_junction_left.dae'         -- The default static mesh for 'T-Junction Left' signs.
local defaultTJunctionSignR = '/art/shapes/garage_and_dealership/Clutter/italy_traf_sign_junction_right.dae'        -- The default static mesh for 'T-Junction Right' signs.
local defaultLeftOrRightSign = '/art/shapes/garage_and_dealership/Clutter/italy_traf_sign_turn_leftorright.dae'     -- The default static mesh for 'Left Or Right' signs.
local defaultRoundaboutSign = '/art/shapes/garage_and_dealership/Clutter/italy_traf_sign_roundabout.dae'            -- The default static mesh for 'Roundabout' signs.
local defaultKeepRightSign = '/art/shapes/garage_and_dealership/Clutter/italy_traf_sign_keep_right.dae'             -- The default static mesh for 'Keep Left' signs.
local defaultLaneMergeSign = '/art/shapes/garage_and_dealership/Clutter/italy_traf_sign_junction_merge_left.dae'    -- The default static mesh for 'Merge Left' signs.
local defaultRoadNarrowsSign = '/art/shapes/garage_and_dealership/Clutter/italy_traf_sign_warn_road_narrows.dae'    -- The default static mesh for 'Road Narrows' signs.
local defaultPassEitherSideSign = '/art/shapes/garage_and_dealership/Clutter/italy_traf_sign_pass_either_side.dae'  -- The default static mesh for 'Pass Either Side' signs.

local jctFlattenRad = 20.0                                                                                          -- The flattening radius around junctions, in meters (L-System).

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Private constants.
local im = ui_imgui
local floor, ceil, abs, min, max = math.floor, math.ceil, math.abs, math.min, math.max
local sqrt, sin, cos, tan, pi = math.sqrt, math.sin, math.cos, math.tan, math.pi
local deg2Rad, rad2Deg = math.rad, math.deg

-- External modules used.
local roadMgr = require('editor/tech/roadArchitect/roads')                                          -- A module for managing the road structure/handling road calculations.
local profileMgr = require('editor/tech/roadArchitect/profiles')                                    -- A module for managing road profiles.
local overlayUtils = require('editor/tech/roadArchitect/overlays')                                  -- A utilities module for creating overlays for junctions.
local terra = require('editor/tech/roadArchitect/terraform')                                        -- A module containing functions for performing terraforming operations.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A general utilities module for the Road Architect tool.

-- Module state.
local junctions = {}                                                                                -- The collection of junctions currently in the editor session.

local oneOverRoot2 = 0.7071067811865475
local downRight = vec3(oneOverRoot2, -oneOverRoot2, 0)
local downLeft = vec3(-oneOverRoot2, -oneOverRoot2, 0)
local upRight = vec3(oneOverRoot2, oneOverRoot2, 0)
local upLeft = vec3(-oneOverRoot2, oneOverRoot2, 0)
local xAxis = vec3(1, 0, 0)
local vertical = vec3(0, 0, 1)
local halfPi = pi * 0.5
local jctFlattenRadSq = jctFlattenRad * jctFlattenRad


-- Compute the centroid of the junction with the given index. Filters by case.
local function getJunctionCentroid(jIdx)
  local jct = junctions[jIdx]
  local type = jct.type
  if type == 'crossing' or type == 'crossroads' or type == 't-junction' or type == 'y-junction' or type == 'roundabout' or
    type == 'urban_merge' or type == 'highway_merge' or type == 'highway_slip' then
    local r1 = roadMgr.roads[roadMgr.map[jct.roads[1]]]
    local r2 = roadMgr.roads[roadMgr.map[jct.roads[2]]]
    local n1, n2 = r1.nodes[2].p, r2.nodes[2].p
    return n1 + (n2 - n1) * 0.5
  elseif type == 'highway_urban_transition' or type == 'rural_urban_transition' then
    local r3 = roadMgr.roads[roadMgr.map[jct.roads[3]]]
    local n1, n2 = r3.nodes[1].p, r3.nodes[#r3.nodes].p
    return n1 + (n2 - n1) * 0.5
  elseif type == 'urban_separator' or type == 'highway_separator' then
    local r1 = roadMgr.roads[roadMgr.map[jct.roads[1]]]
    return r1.nodes[#r1.nodes].p
  elseif type == 'shoulder_fade' then
    if not jct.isY1Outwards[0] then
      local r1 = roadMgr.roads[roadMgr.map[jct.roads[1]]]
      local r2 = roadMgr.roads[roadMgr.map[jct.roads[2]]]
      local n1, n2 = r1.nodes[1].p, r2.nodes[#r2.nodes].p
      return n1 + (n2 - n1) * 0.5
    else
      local r1 = roadMgr.roads[roadMgr.map[jct.roads[1]]]
      local r2 = roadMgr.roads[roadMgr.map[jct.roads[2]]]
      local n1, n2 = r1.nodes[#r1.nodes].p, r2.nodes[1].p
      return n1 + (n2 - n1) * 0.5
    end
  end
end

-- Computes the rotation angle (around Z-axis) at junction place. Filters by case.
local function computeInitRot(jIdx)
  local jct = junctions[jIdx]
  local type = jct.type
  if type == 'crossing' or type == 'crossroads' or type == 't-junction' or type == 'y-junction' or type == 'roundabout' or
    type == 'urban_merge' or type == 'highway_merge' or type == 'shoulder_fade' or type == 'highway_slip' then
    local jRoads = jct.roads
    local road1 = roadMgr.roads[roadMgr.map[jRoads[1]]]
    local road2 = roadMgr.roads[roadMgr.map[jRoads[2]]]
    local v1 = road2.nodes[1].p - road1.nodes[1].p
    v1:normalize()
    return util.getRotationBetweenVecs(xAxis, v1)
  elseif type == 'rural_urban_transition' then
    local jRoads = junctions[jIdx].roads
    local r3 = roadMgr.roads[roadMgr.map[jRoads[3]]]
    local v1 = r3.nodes[#r3.nodes].p - r3.nodes[1].p
    if jct.isYOneWay[0] and jct.isY1Outwards[0] then
      v1 = r3.nodes[1].p - r3.nodes[#r3.nodes].p
    end
    v1:normalize()
    return util.getRotationBetweenVecs(xAxis, v1)
  elseif type == 'highway_urban_transition' then
    local jRoads = junctions[jIdx].roads
    local r3 = roadMgr.roads[roadMgr.map[jRoads[3]]]
    local v1 = r3.nodes[#r3.nodes].p - r3.nodes[1].p
    v1:normalize()
    return util.getRotationBetweenVecs(xAxis, v1)
  elseif type == 'urban_separator' then
    local jRoads = jct.roads
    local road1 = roadMgr.roads[roadMgr.map[jRoads[1]]]
    local v1 = road1.nodes[#road1.nodes].p - road1.nodes[1].p
    v1:normalize()
    return util.getRotationBetweenVecs(xAxis, v1)
  elseif type == 'highway_separator' then
    local jRoads = jct.roads
    local road1 = roadMgr.roads[roadMgr.map[jRoads[1]]]
    local v1 = road1.nodes[2].p - road1.nodes[1].p
    v1:normalize()
    return util.getRotationBetweenVecs(xAxis, v1)
  end
end

-- Rotates the junction with the given index, by a given quaternion.
local function rotateJctByQuat(jIdx, rot)
  local jct = junctions[jIdx]
  local jRoads = jct.roads
  for i = 1, #jRoads do
    local r = roadMgr.roads[roadMgr.map[jRoads[i]]]
    local nodes = r.nodes
    for j = 1, #nodes do
      nodes[j].p = util.rotateVecByQuaternion(nodes[j].p, rot)
    end
  end
end

-- Updates the condition of the given junction.
local function updateJunctionCondition(jct)
  local jRoads = jct.roads
  local ctr = 0
  for i = 1, #jRoads do
    local r = roadMgr.roads[roadMgr.map[jRoads[i]]]
    if not r.isOverlay then
      local profile = r.profile
      profile.condition = im.FloatPtr(jct.condition[0])
      profile.conditionSeed = im.IntPtr(jct.conditionSeed[0] + ctr)
      profile.numPatches = im.IntPtr(ceil(jct.numPatches[0] * 0.15))
      profile.numPotholes = im.IntPtr(ceil(jct.numPotholes[0] * 0.15))
      ctr = ctr + 1
      profileMgr.updateCondition(r)
    end
  end
end

-- Updates an urban crossing junction.
local function updateCrossing(jIdx, jct, isMesh)
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local isPedX1 = jct.isPedX1[0]
  local pedXWidth = jct.pedXWidth[0]
  local isSidewalk = jct.isSidewalk[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local isLowerSWAtPedX = jct.isLowerSWAtPedX[0]
  local capLength = jct.capLength[0]

  local boxX = jct.pedXDist[0]
  local boxXHalf = boxX * 0.5

  -- Create the two exit road profiles.
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileX2_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)

  -- Create the two profiles for the center cross roads.
  local profileCR_X = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  profileCR_X.layers = {}
  if not isSidewalk then
    profileMgr.autoEdgeBlending(profileCR_X, true, true, jct.edgeBlendMat)
  end

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  local isEdgeBlend = not isSidewalk

  -- Create the inner roads.
  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'jct road 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX1_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX1_I.conditionEndStopS = im.BoolPtr(false)
  profileX1_I.conditionEndStopE = im.BoolPtr(true)
  profileX1_I.isStopDecalS = im.BoolPtr(false)
  profileX1_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadX1_I.nodes[1].isLocked = false
  roadX1_I.nodes[2].isLocked = true
  roadX1_I.nodes[3].isLocked = true
  roadX1_I.nodes[4].isLocked = true

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'jct road 2')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX2_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX2_I.conditionEndStopS = im.BoolPtr(false)
  profileX2_I.conditionEndStopE = im.BoolPtr(true)
  profileX2_I.isStopDecalS = im.BoolPtr(false)
  profileX2_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadX2_I.nodes[1].isLocked = false
  roadX2_I.nodes[2].isLocked = true
  roadX2_I.nodes[3].isLocked = true
  roadX2_I.nodes[4].isLocked = true

  -- Create the cross roads in the center.
  local roadCR_X = roadMgr.createRoadFromProfile(profileCR_X)
  roadCR_X.displayName = im.ArrayChar(32, 'jct cross X')
  roadCR_X.isJctRoad = true
  profileCR_X.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileCR_X.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileCR_X.conditionCenterline = im.BoolPtr(false)
  profileCR_X.conditionEdgesL = im.BoolPtr(false)
  profileCR_X.conditionEdgesR = im.BoolPtr(false)
  profileCR_X.conditionLaneMarkings = im.BoolPtr(false)
  profileCR_X.conditionEndStopS = im.BoolPtr(false)
  profileCR_X.conditionEndStopE = im.BoolPtr(false)
  profileCR_X.isStopDecalS = im.BoolPtr(false)
  profileCR_X.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadCR_X
  roadMgr.map[roadCR_X.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf * 0.5, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf * 0.5, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadCR_X.nodes[1].isLocked = true
  roadCR_X.nodes[2].isLocked = true
  roadCR_X.nodes[3].isLocked = true
  roadCR_X.nodes[4].isLocked = true

  -- Apply the dipped kerb corners to the sidewalk profiles, if requested.
  if isLowerSWAtPedX then
    roadCR_X.nodes[2].heightsL[numLanesX + 1] = im.FloatPtr(0.01)
    roadCR_X.nodes[3].heightsL[numLanesX + 1] = im.FloatPtr(0.01)
    roadCR_X.nodes[2].heightsL[-numLanesX - 1] = im.FloatPtr(0.01)
    roadCR_X.nodes[3].heightsL[-numLanesX - 1] = im.FloatPtr(0.01)
  end

  jct.roads = { roadX1_I.name, roadX2_I.name, roadCR_X.name }

  -- Create the traffic light booms, if requested.
  if jct.isTLights[0] then
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX1_I)
    profileX1_I.layers[#profileX1_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom A'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(1.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX2_I)
    profileX2_I.layers[#profileX2_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom B'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(1.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }
  end

  -- Create the pedestrian crossing decals, if requested.
  if isPedX1 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesX), laneMax = im.IntPtr(numLanesX),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr((boxXHalf - 0.5 * pedXWidth) / boxX),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileCR_X.layers, 1, pedX)
  end

  -- Create the road signs (on poles), if requested.
  if jct.isSigns[0] then
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX1_I)
    profileX1_I.layers[#profileX1_I.layers + 1] = {
      name = im.ArrayChar(32, 'Ped X Sign L'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.05), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_warn_zebra_warning.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX2_I)
    profileX2_I.layers[#profileX2_I.layers + 1] = {
      name = im.ArrayChar(32, 'Ped X Sign R'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.05), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_warn_zebra_warning.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end
end

-- Updates an urban crossroads junction.
local function updateCrossroads(jIdx, jct, isMesh)
  local numLanesX, numLanesY = jct.numLanesX[0], jct.numLanesY[0]
  local laneWidthX, laneWidthY = jct.laneWidthX[0], jct.laneWidthY[0]
  local isYOneWay, isY1Outwards, isY2Outwards = jct.isYOneWay[0], jct.isY1Outwards[0], jct.isY2Outwards[0]
  local isPedX1, isPedX2, isPedX3, isPedX4 = jct.isPedX1[0], jct.isPedX2[0], jct.isPedX3[0], jct.isPedX4[0]
  local pedXWidth = jct.pedXWidth[0]
  local isSidewalk = jct.isSidewalk[0]
  local bevel = jct.bevel[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local isLowerSWAtPedX = jct.isLowerSWAtPedX[0]
  local capLength = jct.capLength[0]

  local boxX = nil
  if isYOneWay then
    boxX = numLanesY * laneWidthY
  else
    boxX = numLanesY * 2 * laneWidthY
  end
  local boxY = numLanesX * 2 * laneWidthX
  local boxXHalf, boxYHalf = boxX * 0.5, boxY * 0.5

  -- Create the four inner road profiles.
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  local profileX2_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  local profileY1_I, profileY2_I = nil, nil
  if isYOneWay then
    profileY1_I = profileMgr.createProfileForJctRoad1Way(numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
    profileY2_I = profileMgr.createProfileForJctRoad1Way(numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  else
    profileY1_I = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
    profileY2_I = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  end
  profileX1_I.layers = {}
  profileX2_I.layers = {}
  profileY1_I.layers = {}
  profileY2_I.layers = {}
  if not isSidewalk then
    profileMgr.autoEdgeBlending(profileX1_I, true, true, jct.edgeBlendMat)
    profileMgr.autoEdgeBlending(profileX2_I, true, true, jct.edgeBlendMat)
    profileMgr.autoEdgeBlending(profileY1_I, true, true, jct.edgeBlendMat)
    profileMgr.autoEdgeBlending(profileY2_I, true, true, jct.edgeBlendMat)
  end

  -- Create the four outer road profiles (with sidewalks, if requested).
  local profileX1_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileX2_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileY1_O, profileY2_O = nil, nil
  if isYOneWay then
    profileY1_O = profileMgr.createProfileForJctRoad1Way(numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
    profileY2_O = profileMgr.createProfileForJctRoad1Way(numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  else
    profileY1_O = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
    profileY2_O = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  end

  -- Create the two profiles for the center cross roads.
  local profileCR_X = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  local profileCR_Y = nil
  if isYOneWay then
    profileCR_Y = profileMgr.createProfileForJctRoad1Way(numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  else
    profileCR_Y = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  end
  profileCR_X.layers = {}
  profileCR_Y.layers = {}

  -- Create the four sidewalk-only profiles at each junction corner.
  local profileTL_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)
  local profileTR_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)
  local profileBL_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)
  local profileBR_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)

  -- Apply the dipped kerb corners to the sidewalk profiles, if requested.
  if isLowerSWAtPedX then
    profileTL_S[1].heightL = im.FloatPtr(0.01)
    profileTR_S[1].heightL = im.FloatPtr(0.01)
    profileBL_S[1].heightL = im.FloatPtr(0.01)
    profileBR_S[1].heightL = im.FloatPtr(0.01)
  end

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  local isEdgeBlend = not isSidewalk

  -- Create the inner roads.
  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'jct road 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX1_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX1_I.conditionCenterline = im.BoolPtr(false)
  profileX1_I.conditionEdgesL = im.BoolPtr(false)
  profileX1_I.conditionEdgesR = im.BoolPtr(false)
  profileX1_I.conditionLaneMarkings = im.BoolPtr(false)
  profileX1_I.conditionEndStopS = im.BoolPtr(false)
  profileX1_I.conditionEndStopE = im.BoolPtr(false)
  profileX1_I.isStopDecalS = im.BoolPtr(false)
  profileX1_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadX1_I.nodes[1].isLocked = true
  roadX1_I.nodes[2].isLocked = true

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'jct road 2')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX2_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX2_I.conditionCenterline = im.BoolPtr(false)
  profileX2_I.conditionEdgesL = im.BoolPtr(false)
  profileX2_I.conditionEdgesR = im.BoolPtr(false)
  profileX2_I.conditionLaneMarkings = im.BoolPtr(false)
  profileX2_I.conditionEndStopS = im.BoolPtr(false)
  profileX2_I.conditionEndStopE = im.BoolPtr(false)
  profileX2_I.isStopDecalS = im.BoolPtr(false)
  profileX2_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadX2_I.nodes[1].isLocked = true
  roadX2_I.nodes[2].isLocked = true

  local roadY1_I = roadMgr.createRoadFromProfile(profileY1_I)
  roadY1_I.displayName = im.ArrayChar(32, 'jct road 3')
  roadY1_I.isJctRoad = true
  profileY1_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY1_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY1_I.conditionCenterline = im.BoolPtr(false)
  profileY1_I.conditionEdgesL = im.BoolPtr(false)
  profileY1_I.conditionEdgesR = im.BoolPtr(false)
  profileY1_I.conditionLaneMarkings = im.BoolPtr(false)
  profileY1_I.conditionEndStopS = im.BoolPtr(false)
  profileY1_I.conditionEndStopE = im.BoolPtr(false)
  profileY1_I.isStopDecalS = im.BoolPtr(false)
  profileY1_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY1_I
  roadMgr.map[roadY1_I.name] = rIdx
  local p1 = vec3(0, -boxYHalf - bevel, 0)
  local p2 = vec3(0, -boxYHalf, 0)
  if isYOneWay then
    if isY1Outwards then
      p1.x = boxXHalf
      p2.x = boxXHalf
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p1, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p2, rot) + cen)
      roadY1_I.nodes[1].isLocked = true
      roadY1_I.nodes[2].isLocked = true
    else
      p1.x = -boxXHalf
      p2.x = -boxXHalf
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p2, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p1, rot) + cen)
      roadY1_I.nodes[1].isLocked = true
      roadY1_I.nodes[2].isLocked = true
    end
  else
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p1, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p2, rot) + cen)
    roadY1_I.nodes[1].isLocked = true
    roadY1_I.nodes[2].isLocked = true
  end

  local roadY2_I = roadMgr.createRoadFromProfile(profileY2_I)
  roadY2_I.displayName = im.ArrayChar(32, 'jct road 4')
  roadY2_I.isJctRoad = true
  profileY2_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY2_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY2_I.conditionCenterline = im.BoolPtr(false)
  profileY2_I.conditionEdgesL = im.BoolPtr(false)
  profileY2_I.conditionEdgesR = im.BoolPtr(false)
  profileY2_I.conditionLaneMarkings = im.BoolPtr(false)
  profileY2_I.conditionEndStopS = im.BoolPtr(false)
  profileY2_I.conditionEndStopE = im.BoolPtr(false)
  profileY2_I.isStopDecalS = im.BoolPtr(false)
  profileY2_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY2_I
  roadMgr.map[roadY2_I.name] = rIdx
  local p1 = vec3(0, boxYHalf + bevel, 0)
  local p2 = vec3(0, boxYHalf, 0)
  if isYOneWay then
    if isY2Outwards then
      p1.x = -boxXHalf
      p2.x = -boxXHalf
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p1, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p2, rot) + cen)
      roadY2_I.nodes[1].isLocked = true
      roadY2_I.nodes[2].isLocked = true
    else
      p1.x = boxXHalf
      p2.x = boxXHalf
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p2, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p1, rot) + cen)
      roadY2_I.nodes[1].isLocked = true
      roadY2_I.nodes[2].isLocked = true
    end
  else
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p1, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p2, rot) + cen)
    roadY2_I.nodes[1].isLocked = true
    roadY2_I.nodes[2].isLocked = true
  end

  -- Create the two cross roads in the center.
  local roadCR_X = roadMgr.createRoadFromProfile(profileCR_X)
  roadCR_X.displayName = im.ArrayChar(32, 'jct cross X')
  roadCR_X.isJctRoad = true
  profileCR_X.isEdgeBlendL = im.BoolPtr(false)
  profileCR_X.isEdgeBlendR = im.BoolPtr(false)
  profileCR_X.conditionCenterline = im.BoolPtr(false)
  profileCR_X.conditionEdgesL = im.BoolPtr(false)
  profileCR_X.conditionEdgesR = im.BoolPtr(false)
  profileCR_X.conditionLaneMarkings = im.BoolPtr(false)
  profileCR_X.conditionEndStopS = im.BoolPtr(false)
  profileCR_X.conditionEndStopE = im.BoolPtr(false)
  profileCR_X.isStopDecalS = im.BoolPtr(false)
  profileCR_X.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadCR_X
  roadMgr.map[roadCR_X.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadCR_X.nodes[1].isLocked = true
  roadCR_X.nodes[2].isLocked = true

  local roadCR_Y = roadMgr.createRoadFromProfile(profileCR_Y)
  roadCR_Y.displayName = im.ArrayChar(32, 'jct cross Y')
  roadCR_Y.isJctRoad = true
  profileCR_Y.isEdgeBlendL = im.BoolPtr(false)
  profileCR_Y.isEdgeBlendR = im.BoolPtr(false)
  profileCR_Y.conditionCenterline = im.BoolPtr(false)
  profileCR_Y.conditionEdgesL = im.BoolPtr(false)
  profileCR_Y.conditionEdgesR = im.BoolPtr(false)
  profileCR_Y.conditionLaneMarkings = im.BoolPtr(false)
  profileCR_Y.conditionEndStopS = im.BoolPtr(false)
  profileCR_Y.conditionEndStopE = im.BoolPtr(false)
  profileCR_Y.isStopDecalS = im.BoolPtr(false)
  profileCR_Y.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadCR_Y
  roadMgr.map[roadCR_Y.name] = rIdx
  local q1 = vec3(0, -boxYHalf, 0)
  local q2 = vec3(0, boxYHalf, 0)
  if isYOneWay then
    q1.x, q2.x = -boxXHalf, -boxXHalf
  end
  if not isY2Outwards then
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q2, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q1, rot) + cen)
    roadCR_Y.nodes[1].isLocked = true
    roadCR_Y.nodes[2].isLocked = true
  else
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q1, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q2, rot) + cen)
    roadCR_Y.nodes[1].isLocked = true
    roadCR_Y.nodes[2].isLocked = true
  end

  -- Create the outer roads.
  local roadX1_O = roadMgr.createRoadFromProfile(profileX1_O)
  roadX1_O.displayName = im.ArrayChar(32, 'jct road 5')
  roadX1_O.isJctRoad = true
  profileX1_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX1_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX1_O.isStopDecalS = im.BoolPtr(false)
  profileX1_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_O
  roadMgr.map[roadX1_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength, 0, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, 0, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength * 2, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength * 1.5, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p3)
  roadX1_O.nodes[1].isLocked = false
  roadX1_O.nodes[2].isLocked = true
  roadX1_O.nodes[3].isLocked = true
  roadX1_O.nodes[4].isLocked = true
  roadX1_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadX2_O = roadMgr.createRoadFromProfile(profileX2_O)
  roadX2_O.displayName = im.ArrayChar(32, 'jct road 6')
  roadX2_O.isJctRoad = true
  profileX2_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX2_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX2_O.isStopDecalS = im.BoolPtr(false)
  profileX2_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_O
  roadMgr.map[roadX2_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength, 0, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(boxXHalf + bevel, 0, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength * 2, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength * 1.5, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p3)
  roadX2_O.nodes[1].isLocked = false
  roadX2_O.nodes[2].isLocked = true
  roadX2_O.nodes[3].isLocked = true
  roadX2_O.nodes[4].isLocked = true
  roadX2_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadY1_O = roadMgr.createRoadFromProfile(profileY1_O)
  roadY1_O.displayName = im.ArrayChar(32, 'jct road 7')
  roadY1_O.isJctRoad = true
  profileY1_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY1_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY1_O.isStopDecalS = im.BoolPtr(false)
  profileY1_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY1_O
  roadMgr.map[roadY1_O.name] = rIdx
  local q1 = vec3(0, -boxYHalf - bevel - capLength, 0)
  local q2 = vec3(0, -boxYHalf - bevel, 0)
  local q3 = vec3(0, -boxYHalf - bevel - capLength * 2, 0)
  local q4 = vec3(0, -boxYHalf - bevel - capLength * 1.5, 0)
  if isYOneWay then
    if isY1Outwards then
      q1.x, q2.x, q3.x, q4.x = boxXHalf, boxXHalf, boxXHalf, boxXHalf
      local p1 = util.rotateVecByQuaternion(q1, rot) + cen
      local p3 = util.rotateVecByQuaternion(q2, rot) + cen
      local p2 = p1 + (p3 - p1) * 0.5
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q3, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q4, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, p1)
      roadMgr.addNodeToRoad(rIdx, p2)
      roadMgr.addNodeToRoad(rIdx, p3)
      roadY1_O.nodes[1].isLocked = false
      roadY1_O.nodes[2].isLocked = true
      roadY1_O.nodes[3].isLocked = true
      roadY1_O.nodes[4].isLocked = true
      roadY1_O.nodes[5].isLocked = true
    else
      q1.x, q2.x, q3.x, q4.x = -boxXHalf, -boxXHalf, -boxXHalf, -boxXHalf
      local p1 = util.rotateVecByQuaternion(q1, rot) + cen
      local p3 = util.rotateVecByQuaternion(q2, rot) + cen
      local p2 = p1 + (p3 - p1) * 0.5
      roadMgr.addNodeToRoad(rIdx, p3)
      roadMgr.addNodeToRoad(rIdx, p2)
      roadMgr.addNodeToRoad(rIdx, p1)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q4, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q3, rot) + cen)
      roadY1_O.nodes[1].isLocked = true
      roadY1_O.nodes[2].isLocked = true
      roadY1_O.nodes[3].isLocked = true
      roadY1_O.nodes[4].isLocked = true
      roadY1_O.nodes[5].isLocked = false
    end
  else
    local p1 = util.rotateVecByQuaternion(q1, rot) + cen
    local p3 = util.rotateVecByQuaternion(q2, rot) + cen
    local p2 = p1 + (p3 - p1) * 0.5
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q3, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q4, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, p1)
    roadMgr.addNodeToRoad(rIdx, p2)
    roadMgr.addNodeToRoad(rIdx, p3)
    roadY1_O.nodes[1].isLocked = false
    roadY1_O.nodes[2].isLocked = true
    roadY1_O.nodes[3].isLocked = true
    roadY1_O.nodes[4].isLocked = true
    roadY1_O.nodes[5].isLocked = true
  end
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadY2_O = roadMgr.createRoadFromProfile(profileY2_O)
  roadY2_O.displayName = im.ArrayChar(32, 'jct road 8')
  roadY2_O.isJctRoad = true
  profileY2_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY2_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY2_O.isStopDecalS = im.BoolPtr(false)
  profileY2_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY2_O
  roadMgr.map[roadY2_O.name] = rIdx
  local q1 = vec3(0, boxYHalf + bevel + capLength, 0)
  local q2 = vec3(0, boxYHalf + bevel, 0)
  local q3 = vec3(0, boxYHalf + bevel + capLength * 2, 0)
  local q4 = vec3(0, boxYHalf + bevel + capLength * 1.5, 0)
  if isYOneWay then
    if isY2Outwards then
      q1.x, q2.x, q3.x, q4.x = -boxXHalf, -boxXHalf, -boxXHalf, -boxXHalf
      local p1 = util.rotateVecByQuaternion(q1, rot) + cen
      local p3 = util.rotateVecByQuaternion(q2, rot) + cen
      local p2 = p1 + (p3 - p1) * 0.5
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q3, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q4, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, p1)
      roadMgr.addNodeToRoad(rIdx, p2)
      roadMgr.addNodeToRoad(rIdx, p3)
      roadY2_O.nodes[1].isLocked = false
      roadY2_O.nodes[2].isLocked = true
      roadY2_O.nodes[3].isLocked = true
      roadY2_O.nodes[4].isLocked = true
      roadY2_O.nodes[5].isLocked = true
    else
      q1.x, q2.x, q3.x, q4.x = boxXHalf, boxXHalf, boxXHalf, boxXHalf
      local p1 = util.rotateVecByQuaternion(q1, rot) + cen
      local p3 = util.rotateVecByQuaternion(q2, rot) + cen
      local p2 = p1 + (p3 - p1) * 0.5
      roadMgr.addNodeToRoad(rIdx, p3)
      roadMgr.addNodeToRoad(rIdx, p2)
      roadMgr.addNodeToRoad(rIdx, p1)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q4, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q3, rot) + cen)
      roadY2_O.nodes[1].isLocked = true
      roadY2_O.nodes[2].isLocked = true
      roadY2_O.nodes[3].isLocked = true
      roadY2_O.nodes[4].isLocked = true
      roadY2_O.nodes[5].isLocked = false
    end
  else
    local p1 = util.rotateVecByQuaternion(q1, rot) + cen
    local p3 = util.rotateVecByQuaternion(q2, rot) + cen
    local p2 = p1 + (p3 - p1) * 0.5
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q3, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q4, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, p1)
    roadMgr.addNodeToRoad(rIdx, p2)
    roadMgr.addNodeToRoad(rIdx, p3)
    roadY2_O.nodes[1].isLocked = false
    roadY2_O.nodes[2].isLocked = true
    roadY2_O.nodes[3].isLocked = true
    roadY2_O.nodes[4].isLocked = true
    roadY2_O.nodes[5].isLocked = true
  end
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  -- Apply the dipped kerb corners to the sidewalk profiles, if requested.
  if isLowerSWAtPedX then
    roadX1_O.nodes[5].heightsL[numLanesX + 1] = im.FloatPtr(0.01)
    roadX1_O.nodes[5].heightsL[-numLanesX - 1] = im.FloatPtr(0.01)
    roadX2_O.nodes[5].heightsL[numLanesX + 1] = im.FloatPtr(0.01)
    roadX2_O.nodes[5].heightsL[-numLanesX - 1] = im.FloatPtr(0.01)
    if isYOneWay then
      if isY1Outwards then
        roadY1_O.nodes[5].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
        roadY1_O.nodes[5].heightsL[-1] = im.FloatPtr(0.01)
      else
        roadY1_O.nodes[1].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
        roadY1_O.nodes[1].heightsL[-1] = im.FloatPtr(0.01)
      end
      if isY2Outwards then
        roadY2_O.nodes[5].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
        roadY2_O.nodes[5].heightsL[-1] = im.FloatPtr(0.01)
      else
        roadY2_O.nodes[1].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
        roadY2_O.nodes[1].heightsL[-1] = im.FloatPtr(0.01)
      end
    else
      roadY1_O.nodes[5].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
      roadY1_O.nodes[5].heightsL[-numLanesY - 1] = im.FloatPtr(0.01)
      roadY2_O.nodes[5].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
      roadY2_O.nodes[5].heightsL[-numLanesY - 1] = im.FloatPtr(0.01)
    end
  end

  -- Create the corner bevelled sidewalk roads.
  if isSidewalk then
    local roadTL_S = roadMgr.createRoadFromProfile(profileTL_S)
    roadTL_S.displayName = im.ArrayChar(32, 'jct s-walk TL')
    roadTL_S.isDrivable = false
    roadTL_S.isJctRoad = true
    profileTL_S.isEdgeBlendL = im.BoolPtr(false)
    profileTL_S.isEdgeBlendR = im.BoolPtr(false)
    profileTL_S.isStopDecalS = im.BoolPtr(false)
    profileTL_S.isStopDecalE = im.BoolPtr(false)
    roadTL_S.isArc = true
    roadTL_S.granFactor = im.IntPtr(crossroadArcGran)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadTL_S
    roadMgr.map[roadTL_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, boxYHalf, 0), rot) + cen)
    local pCen = vec3(-boxXHalf - bevel, boxYHalf + bevel, 0)
    local pMid = pCen + downRight * bevel
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(pMid, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, boxYHalf + bevel, 0), rot) + cen)
    roadTL_S.nodes[1].isLocked = true
    roadTL_S.nodes[2].isLocked = true
    roadTL_S.nodes[3].isLocked = true

    local roadTR_S = roadMgr.createRoadFromProfile(profileTR_S)
    roadTR_S.displayName = im.ArrayChar(32, 'jct s-walk TR')
    roadTR_S.isDrivable = false
    roadTR_S.isJctRoad = true
    profileTR_S.isEdgeBlendL = im.BoolPtr(false)
    profileTR_S.isEdgeBlendR = im.BoolPtr(false)
    profileTR_S.isStopDecalS = im.BoolPtr(false)
    profileTR_S.isStopDecalE = im.BoolPtr(false)
    roadTR_S.isArc = true
    roadTR_S.granFactor = im.IntPtr(crossroadArcGran)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadTR_S
    roadMgr.map[roadTR_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, boxYHalf + bevel, 0), rot) + cen)
    local pCen = vec3(boxXHalf + bevel, boxYHalf + bevel, 0)
    local pMid = pCen + downLeft * bevel
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(pMid, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel, boxYHalf, 0), rot) + cen)
    roadTR_S.nodes[1].isLocked = true
    roadTR_S.nodes[2].isLocked = true
    roadTR_S.nodes[3].isLocked = true

    local roadBL_S = roadMgr.createRoadFromProfile(profileBL_S)
    roadBL_S.displayName = im.ArrayChar(32, 'jct s-walk BL')
    roadBL_S.isDrivable = false
    roadBL_S.isJctRoad = true
    profileBL_S.isEdgeBlendL = im.BoolPtr(false)
    profileBL_S.isEdgeBlendR = im.BoolPtr(false)
    profileBL_S.isStopDecalS = im.BoolPtr(false)
    profileBL_S.isStopDecalE = im.BoolPtr(false)
    roadBL_S.isArc = true
    roadBL_S.granFactor = im.IntPtr(crossroadArcGran)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadBL_S
    roadMgr.map[roadBL_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, -boxYHalf - bevel, 0), rot) + cen)
    local pCen = vec3(-boxXHalf - bevel, -boxYHalf - bevel, 0)
    local pMid = pCen + upRight * bevel
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(pMid, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, -boxYHalf, 0), rot) + cen)
    roadBL_S.nodes[1].isLocked = true
    roadBL_S.nodes[2].isLocked = true
    roadBL_S.nodes[3].isLocked = true

    local roadBR_S = roadMgr.createRoadFromProfile(profileBR_S)
    roadBR_S.displayName = im.ArrayChar(32, 'jct s-walk BR')
    roadBR_S.isDrivable = false
    roadBR_S.isJctRoad = true
    profileBR_S.isEdgeBlendL = im.BoolPtr(false)
    profileBR_S.isEdgeBlendR = im.BoolPtr(false)
    profileBR_S.isStopDecalS = im.BoolPtr(false)
    profileBR_S.isStopDecalE = im.BoolPtr(false)
    roadBR_S.isArc = true
    roadBR_S.granFactor = im.IntPtr(crossroadArcGran)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadBR_S
    roadMgr.map[roadBR_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel, -boxYHalf, 0), rot) + cen)
    local pCen = vec3(boxXHalf + bevel, -boxYHalf - bevel, 0)
    local pMid = pCen + upLeft * bevel
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(pMid, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, -boxYHalf - bevel, 0), rot) + cen)
    roadBR_S.nodes[1].isLocked = true
    roadBR_S.nodes[2].isLocked = true
    roadBR_S.nodes[3].isLocked = true

    jct.roads = {
      roadX1_I.name, roadX2_I.name, roadY1_I.name, roadY2_I.name,
      roadX1_O.name, roadX2_O.name, roadY1_O.name, roadY2_O.name,
      roadTL_S.name, roadTR_S.name, roadBL_S.name, roadBR_S.name,
      roadCR_X.name, roadCR_Y.name, }
  else

    jct.roads = {
      roadX1_I.name, roadX2_I.name, roadY1_I.name, roadY2_I.name,
      roadX1_O.name, roadX2_O.name, roadY1_O.name, roadY2_O.name,
      roadCR_X.name, roadCR_Y.name, }
  end

  -- Create the traffic light booms, if requested.
  if jct.isTLights[0] then
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX1_I)
    profileX1_I.layers[#profileX1_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom A'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(0.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }

    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX2_I)
    profileX2_I.layers[#profileX2_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom B'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(0.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }

    local lMin, lMax = profileMgr.getMinMaxLaneKeys(profileY1_I)
    local sRot = 3
    local laneIdx = lMin
    local sPos = 0.0
    local offsetSign = 1.0
    local sIsLeft = true
    if isYOneWay and not isY1Outwards then
      sRot = 1
      laneIdx = lMax
      sPos = 1.0
      offsetSign = -1.0
      sIsLeft = false
    end
    profileY1_I.layers[#profileY1_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom C'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(laneIdx), laneMax = im.IntPtr(laneIdx),
        lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(sIsLeft), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(sRot),
        pos = im.FloatPtr(sPos), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0] * offsetSign),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }

    local lMin, lMax = profileMgr.getMinMaxLaneKeys(profileY2_I)
    local sRot = 3
    local laneIdx = lMin
    local sPos = 0.0
    local offsetSign = 1.0
    local sIsLeft = true
    if isYOneWay and not isY2Outwards then
      sRot = 1
      laneIdx = lMax
      sPos = 1.0
      offsetSign = -1.0
      sIsLeft = false
    end
    profileY2_I.layers[#profileY2_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom D'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(laneIdx), laneMax = im.IntPtr(laneIdx),
        lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(sIsLeft), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(sRot),
        pos = im.FloatPtr(sPos), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0] * offsetSign),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }
  end

  -- Add the arrow decals, if requested.
  if jct.isArrow[0] then
    if numLanesX > 1 then
      for i = 1, numLanesX do
        local frame = 2
        if i == 1 then
          frame = 0
        elseif i == numLanesX then
          frame = 1
        end
        if numLanesX == 2 and i == 1 then
          frame = 2
        end
        if numLanesX == 1 then
          frame = 2
        end
        local aPos = (capLength - jct.arrowFrontDistFromEnd[0]) / capLength
        if aPos < 0.0 or aPos > 1.0 then
          aPos = 0.5
        end
        profileX1_O.layers[#profileX1_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow X1 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        profileX2_O.layers[#profileX2_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow X2 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        if jct.isDoubleArrows[0] and capLength > jct.arrowBackDistFromEnd[0] + jct.arrowSize[0] + 0.1 then
          profileX1_O.layers[#profileX1_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow X1 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
          profileX2_O.layers[#profileX2_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow X2 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        end
      end
    end
    if numLanesY > 1 then
      for i = 1, numLanesY do
        local frame = 2
        if i == 1 then
          frame = 0
        elseif i == numLanesY then
          frame = 1
        end
        if numLanesY == 2 and i == 1 then
          frame = 2
        end
        if numLanesY == 1 then
          frame = 2
        end
        local aPos = (capLength - jct.arrowFrontDistFromEnd[0]) / capLength
        if aPos < 0.0 or aPos > 1.0 then
          aPos = 0.5
        end
        local laneIdx = -i
        if not isY1Outwards then
          local sRot = 3
          if isYOneWay and not isY1Outwards then
            sRot = 1
            laneIdx = i
          end
          profileY1_O.layers[#profileY1_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow Y1 F' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
            lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(sRot),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        end
        local laneIdx = -i
        if not isY2Outwards then
          local sRot = 3
          if isYOneWay and not isY2Outwards then
            sRot = 1
            laneIdx = i
          end
          profileY2_O.layers[#profileY2_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow Y2 F' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
            lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(sRot),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        end
        if jct.isDoubleArrows[0] and capLength > jct.arrowBackDistFromEnd[0] + jct.arrowSize[0] + 0.1 then
          local laneIdx = -i
          if not isY1Outwards then
            local sRot = 3
            if isYOneWay and not isY1Outwards then
              sRot = 1
              laneIdx = i
            end
            profileY1_O.layers[#profileY1_O.layers + 1] = {
              name = im.ArrayChar(32, 'Arrow Y1 B' .. tostring(i)),
              isHidden = false,
              doNotDelete = im.BoolPtr(true),
              isReverse = im.BoolPtr(false),
              isPaint = im.BoolPtr(false),
              isDisplay = im.BoolPtr(false),
              type = im.IntPtr(3),
              laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
              lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
              width = im.FloatPtr(pedXWidth),
              isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
              texLen = im.FloatPtr(5),
              fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
              mat = jct.arrowMat,
              rot = im.IntPtr(sRot),
              pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
              numRows = im.IntPtr(4), numCols = im.IntPtr(4),
              frame = im.IntPtr(frame),
              vertOffset = im.FloatPtr(0.0),
              latOffset = im.FloatPtr(0.0),
              spacing = im.FloatPtr(5.0),
              jitter = im.FloatPtr(0.0),
              useWorldZ = im.BoolPtr(false),
              matDisplay = '[None]',
              extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
              boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
          end
          local laneIdx = -i
          if not isY2Outwards then
            local sRot = 3
            if isYOneWay and not isY2Outwards then
              sRot = 1
              laneIdx = i
            end
            profileY2_O.layers[#profileY2_O.layers + 1] = {
              name = im.ArrayChar(32, 'Arrow Y2 B' .. tostring(i)),
              isHidden = false,
              doNotDelete = im.BoolPtr(true),
              isReverse = im.BoolPtr(false),
              isPaint = im.BoolPtr(false),
              isDisplay = im.BoolPtr(false),
              type = im.IntPtr(3),
              laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
              lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
              width = im.FloatPtr(pedXWidth),
              isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
              texLen = im.FloatPtr(5),
              fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
              mat = jct.arrowMat,
              rot = im.IntPtr(sRot),
              pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
              numRows = im.IntPtr(4), numCols = im.IntPtr(4),
              frame = im.IntPtr(frame),
              vertOffset = im.FloatPtr(0.0),
              latOffset = im.FloatPtr(0.0),
              spacing = im.FloatPtr(5.0),
              jitter = im.FloatPtr(0.0),
              useWorldZ = im.BoolPtr(false),
              matDisplay = '[None]',
              extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
              boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
          end
        end
      end
    end
  end

  -- Add road signs (poles).
  if jct.isSigns[0] then
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX1_O)
    profileX1_O.layers[#profileX1_O.layers + 1] = {
      name = im.ArrayChar(32, 'Crossroads Sign X1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrossroadsSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.5), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_crossroad.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX2_O)
    profileX2_O.layers[#profileX2_O.layers + 1] = {
      name = im.ArrayChar(32, 'Crossroads Sign X2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrossroadsSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.5), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_crossroad.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, lMax = profileMgr.getMinMaxLaneKeys(profileY1_O)
    local sRot = 3
    local laneIdx = lMin
    local sIsLeft = true
    local latOffSign = -1.0
    if isYOneWay and not isY1Outwards then
      sRot = 1
      laneIdx = lMax
      sIsLeft = false
      latOffSign = 1.0
    end
    profileY1_O.layers[#profileY1_O.layers + 1] = {
      name = im.ArrayChar(32, 'Crossroads Sign Y1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(sIsLeft), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrossroadsSign,
      rot = im.IntPtr(sRot),
      pos = im.FloatPtr(0.5), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(latOffSign * 0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_crossroad.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, lMax = profileMgr.getMinMaxLaneKeys(profileY2_O)
    local sRot = 3
    local laneIdx = lMin
    local sIsLeft = true
    local latOffSign = -1.0
    if isYOneWay and not isY2Outwards then
      sRot = 1
      laneIdx = lMax
      sIsLeft = false
      latOffSign = 1.0
    end
    profileY2_O.layers[#profileY2_O.layers + 1] = {
      name = im.ArrayChar(32, 'Crossroads Sign Y2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(sIsLeft), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrossroadsSign,
      rot = im.IntPtr(sRot),
      pos = im.FloatPtr(0.5), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(latOffSign * 0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_crossroad.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end

  -- Create the pedestrian crossing decals, if requested.
  if isPedX1 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesX), laneMax = im.IntPtr(numLanesX),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileX1_I.layers, 1, pedX)
  end
  if isPedX2 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesX), laneMax = im.IntPtr(numLanesX),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileX2_I.layers, 1, pedX)
  end
  if isPedX3 then
    local lMin, lMax = -numLanesY, numLanesY
    if isYOneWay then
      lMin, lMax = 1, numLanesY
    end
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R3'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMax),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileY1_I.layers, 1, pedX)
  end
  if isPedX4 then
    local lMin, lMax = -numLanesY, numLanesY
    if isYOneWay then
      lMin, lMax = 1, numLanesY
    end
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R4'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMax),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileY2_I.layers, 1, pedX)
  end

  if jct.isCrossings[0] then
    overlayUtils.addCrossroadsOverlays(jct)
  end
end

-- Updates an urban T-junction.
local function updateTJunction(jIdx, jct, isMesh)
  local numLanesX, numLanesY = jct.numLanesX[0], jct.numLanesY[0]
  local laneWidthX, laneWidthY = jct.laneWidthX[0], jct.laneWidthY[0]
  local isYOneWay, isY2Outwards = jct.isYOneWay[0], jct.isY2Outwards[0]
  local isPedX1, isPedX2, isPedX3 = jct.isPedX1[0], jct.isPedX2[0], jct.isPedX3[0]
  local pedXWidth = jct.pedXWidth[0]
  local isSidewalk = jct.isSidewalk[0]
  local bevel = jct.bevel[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local isLowerSWAtPedX = jct.isLowerSWAtPedX[0]
  local capLength = jct.capLength[0]

  local boxX = nil
  if isYOneWay then
    boxX = numLanesY * laneWidthY
  else
    boxX = numLanesY * 2 * laneWidthY
  end
  local boxY = numLanesX * 2 * laneWidthX
  local boxXHalf, boxYHalf = boxX * 0.5, boxY * 0.5

  -- Create the three inner road profiles.
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  local profileX2_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  local profileY2_I = nil
  if isYOneWay then
    profileY2_I = profileMgr.createProfileForJctRoad1Way(numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  else
    profileY2_I = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  end
  profileX1_I.layers = {}
  profileX2_I.layers = {}
  profileY2_I.layers = {}
  if not isSidewalk then
    profileMgr.autoEdgeBlending(profileX1_I, true, true, jct.edgeBlendMat)
    profileMgr.autoEdgeBlending(profileX2_I, true, true, jct.edgeBlendMat)
    profileMgr.autoEdgeBlending(profileY2_I, true, true, jct.edgeBlendMat)
  end

  -- Create the profile for the center cross roads.
  local profileCR_X = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  profileCR_X.layers = {}
  profileMgr.addEdgeLines(profileCR_X, 0.2, 0.2, true, false, true)
  if not isSidewalk then
    profileMgr.autoEdgeBlending(profileCR_X, true, false, jct.edgeBlendMat)
  end

  -- Create the three outer road profiles (with sidewalks, if requested).
  local profileX1_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileX2_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileY2_O = nil
  if isYOneWay then
    profileY2_O = profileMgr.createProfileForJctRoad1Way(numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  else
    profileY2_O = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  end

  -- Create the three sidewalk-only profiles at each junction corner.
  local profileT_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)
  local profileBL_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)
  local profileBR_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)

  -- Apply the dipped kerb corners to the sidewalk profiles, if requested.
  if isLowerSWAtPedX then
    profileBL_S[1].heightL = im.FloatPtr(0.01)
    profileBR_S[1].heightL = im.FloatPtr(0.01)
  end

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  local isEdgeBlend = not isSidewalk

  -- Create the inner roads.
  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'jct road 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX1_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX1_I.conditionCenterline = im.BoolPtr(false)
  profileX1_I.conditionEdgesL = im.BoolPtr(false)
  profileX1_I.conditionEdgesR = im.BoolPtr(false)
  profileX1_I.conditionLaneMarkings = im.BoolPtr(false)
  profileX1_I.conditionEndStopS = im.BoolPtr(false)
  profileX1_I.conditionEndStopE = im.BoolPtr(false)
  profileX1_I.isStopDecalS = im.BoolPtr(false)
  profileX1_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadX1_I.nodes[1].isLocked = true
  roadX1_I.nodes[2].isLocked = true

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'jct road 2')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX2_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX2_I.conditionCenterline = im.BoolPtr(false)
  profileX2_I.conditionEdgesL = im.BoolPtr(false)
  profileX2_I.conditionEdgesR = im.BoolPtr(false)
  profileX2_I.conditionLaneMarkings = im.BoolPtr(false)
  profileX2_I.conditionEndStopS = im.BoolPtr(false)
  profileX2_I.conditionEndStopE = im.BoolPtr(false)
  profileX2_I.isStopDecalS = im.BoolPtr(false)
  profileX2_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadX2_I.nodes[1].isLocked = true
  roadX2_I.nodes[2].isLocked = true

  local roadY2_I = roadMgr.createRoadFromProfile(profileY2_I)
  roadY2_I.displayName = im.ArrayChar(32, 'jct road 3')
  roadY2_I.isJctRoad = true
  profileY2_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY2_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY2_I.conditionCenterline = im.BoolPtr(false)
  profileY2_I.conditionEdgesL = im.BoolPtr(false)
  profileY2_I.conditionEdgesR = im.BoolPtr(false)
  profileY2_I.conditionLaneMarkings = im.BoolPtr(false)
  profileY2_I.conditionEndStopS = im.BoolPtr(false)
  profileY2_I.conditionEndStopE = im.BoolPtr(false)
  profileY2_I.isStopDecalS = im.BoolPtr(false)
  profileY2_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY2_I
  roadMgr.map[roadY2_I.name] = rIdx
  local p1 = vec3(0, boxYHalf + bevel, 0)
  local p2 = vec3(0, boxYHalf, 0)
  if isYOneWay then
    if isY2Outwards then
      p1.x = -boxXHalf
      p2.x = -boxXHalf
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p1, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p2, rot) + cen)
      roadY2_I.nodes[1].isLocked = true
      roadY2_I.nodes[2].isLocked = true
    else
      p1.x = boxXHalf
      p2.x = boxXHalf
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p2, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p1, rot) + cen)
      roadY2_I.nodes[1].isLocked = true
      roadY2_I.nodes[2].isLocked = true
    end
  else
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p1, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(p2, rot) + cen)
    roadY2_I.nodes[1].isLocked = true
    roadY2_I.nodes[2].isLocked = true
  end

  -- Create the cross road in the center.
  local roadCR_X = roadMgr.createRoadFromProfile(profileCR_X)
  roadCR_X.displayName = im.ArrayChar(32, 'jct cross X')
  roadCR_X.isJctRoad = true
  profileCR_X.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileCR_X.isEdgeBlendR = im.BoolPtr(false)
  profileCR_X.conditionCenterline = im.BoolPtr(false)
  profileCR_X.conditionEdgesL = im.BoolPtr(true)
  profileCR_X.conditionEdgesR = im.BoolPtr(false)
  profileCR_X.conditionLaneMarkings = im.BoolPtr(false)
  profileCR_X.conditionEndStopS = im.BoolPtr(false)
  profileCR_X.conditionEndStopE = im.BoolPtr(false)
  profileCR_X.isStopDecalS = im.BoolPtr(false)
  profileCR_X.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadCR_X
  roadMgr.map[roadCR_X.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadCR_X.nodes[1].isLocked = true
  roadCR_X.nodes[2].isLocked = true

  -- Create the outer roads.
  local roadX1_O = roadMgr.createRoadFromProfile(profileX1_O)
  roadX1_O.displayName = im.ArrayChar(32, 'jct road 4')
  roadX1_O.isJctRoad = true
  profileX1_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX1_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX1_O.isStopDecalS = im.BoolPtr(false)
  profileX1_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_O
  roadMgr.map[roadX1_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength, 0, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, 0, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength * 2, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength * 1.5, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p3)
  roadX1_O.nodes[1].isLocked = false
  roadX1_O.nodes[2].isLocked = true
  roadX1_O.nodes[3].isLocked = true
  roadX1_O.nodes[4].isLocked = true
  roadX1_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadX2_O = roadMgr.createRoadFromProfile(profileX2_O)
  roadX2_O.displayName = im.ArrayChar(32, 'jct road 5')
  roadX2_O.isJctRoad = true
  profileX2_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX2_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX2_O.isStopDecalS = im.BoolPtr(false)
  profileX2_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_O
  roadMgr.map[roadX2_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength, 0, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(boxXHalf + bevel, 0, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength * 2, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength * 1.5, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p3)
  roadX2_O.nodes[1].isLocked = false
  roadX2_O.nodes[2].isLocked = true
  roadX2_O.nodes[3].isLocked = true
  roadX2_O.nodes[4].isLocked = true
  roadX2_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadY2_O = roadMgr.createRoadFromProfile(profileY2_O)
  roadY2_O.displayName = im.ArrayChar(32, 'jct road 6')
  roadY2_O.isJctRoad = true
  profileY2_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY2_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY2_O.isStopDecalS = im.BoolPtr(false)
  profileY2_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY2_O
  roadMgr.map[roadY2_O.name] = rIdx
  local q1 = vec3(0, boxYHalf + bevel + capLength, 0)
  local q2 = vec3(0, boxYHalf + bevel, 0)
  local q3 = vec3(0, boxYHalf + bevel + capLength * 2, 0)
  local q4 = vec3(0, boxYHalf + bevel + capLength * 1.5, 0)
  if isYOneWay then
    if isY2Outwards then
      q1.x, q2.x, q3.x, q4.x = -boxXHalf, -boxXHalf, -boxXHalf, -boxXHalf
      local p1 = util.rotateVecByQuaternion(q1, rot) + cen
      local p3 = util.rotateVecByQuaternion(q2, rot) + cen
      local p2 = p1 + (p3 - p1) * 0.5
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q3, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q4, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, p1)
      roadMgr.addNodeToRoad(rIdx, p2)
      roadMgr.addNodeToRoad(rIdx, p3)
      roadY2_O.nodes[1].isLocked = false
      roadY2_O.nodes[2].isLocked = true
      roadY2_O.nodes[3].isLocked = true
      roadY2_O.nodes[4].isLocked = true
      roadY2_O.nodes[5].isLocked = true
    else
      q1.x, q2.x, q3.x, q4.x = boxXHalf, boxXHalf, boxXHalf, boxXHalf
      local p1 = util.rotateVecByQuaternion(q1, rot) + cen
      local p3 = util.rotateVecByQuaternion(q2, rot) + cen
      local p2 = p1 + (p3 - p1) * 0.5
      roadMgr.addNodeToRoad(rIdx, p3)
      roadMgr.addNodeToRoad(rIdx, p2)
      roadMgr.addNodeToRoad(rIdx, p1)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q4, rot) + cen)
      roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q3, rot) + cen)
      roadY2_O.nodes[1].isLocked = true
      roadY2_O.nodes[2].isLocked = true
      roadY2_O.nodes[3].isLocked = true
      roadY2_O.nodes[4].isLocked = true
      roadY2_O.nodes[5].isLocked = false
    end
  else
    local p1 = util.rotateVecByQuaternion(q1, rot) + cen
    local p3 = util.rotateVecByQuaternion(q2, rot) + cen
    local p2 = p1 + (p3 - p1) * 0.5
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q3, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(q4, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, p1)
    roadMgr.addNodeToRoad(rIdx, p2)
    roadMgr.addNodeToRoad(rIdx, p3)
    roadY2_O.nodes[1].isLocked = false
    roadY2_O.nodes[2].isLocked = true
    roadY2_O.nodes[3].isLocked = true
    roadY2_O.nodes[4].isLocked = true
    roadY2_O.nodes[5].isLocked = true
  end
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  -- Create the corner bevelled sidewalk roads.
  if isSidewalk then
    local roadT_S = roadMgr.createRoadFromProfile(profileT_S)
    roadT_S.displayName = im.ArrayChar(32, 'jct s-walk top')
    roadT_S.isDrivable = false
    roadT_S.isJctRoad = true
    profileT_S.isEdgeBlendL = im.BoolPtr(false)
    profileT_S.isEdgeBlendR = im.BoolPtr(false)
    roadT_S.granFactor = im.IntPtr(crossroadArcGran)
    profileT_S.isStopDecalS = im.BoolPtr(false)
    profileT_S.isStopDecalE = im.BoolPtr(false)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadT_S
    roadMgr.map[roadT_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel, -boxYHalf, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, -boxYHalf, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf * 0.5, -boxYHalf, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf * 0.5, -boxYHalf, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, -boxYHalf, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, -boxYHalf, 0), rot) + cen)
    roadT_S.nodes[1].isLocked = true
    roadT_S.nodes[2].isLocked = true
    roadT_S.nodes[3].isLocked = true
    roadT_S.nodes[4].isLocked = true
    roadT_S.nodes[5].isLocked = true
    roadT_S.nodes[6].isLocked = true

    local roadBL_S = roadMgr.createRoadFromProfile(profileBL_S)
    roadBL_S.displayName = im.ArrayChar(32, 'jct s-walk 2')
    roadBL_S.isDrivable = false
    roadBL_S.isJctRoad = true
    profileBL_S.isEdgeBlendL = im.BoolPtr(false)
    profileBL_S.isEdgeBlendR = im.BoolPtr(false)
    roadBL_S.isArc = true
    roadBL_S.granFactor = im.IntPtr(crossroadArcGran)
    profileBL_S.isStopDecalS = im.BoolPtr(false)
    profileBL_S.isStopDecalE = im.BoolPtr(false)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadBL_S
    roadMgr.map[roadBL_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, boxYHalf, 0), rot) + cen)
    local pCen = vec3(-boxXHalf - bevel, boxYHalf + bevel, 0)
    local pMid = pCen + downRight * bevel
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(pMid, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, boxYHalf + bevel, 0), rot) + cen)
    roadBL_S.nodes[1].isLocked = true
    roadBL_S.nodes[2].isLocked = true
    roadBL_S.nodes[3].isLocked = true

    local roadBR_S = roadMgr.createRoadFromProfile(profileBR_S)
    roadBR_S.displayName = im.ArrayChar(32, 'jct s-walk BR')
    roadBR_S.isDrivable = false
    roadBR_S.isJctRoad = true
    profileBR_S.isEdgeBlendL = im.BoolPtr(false)
    profileBR_S.isEdgeBlendR = im.BoolPtr(false)
    roadBR_S.isArc = true
    roadBR_S.granFactor = im.IntPtr(crossroadArcGran)
    profileBR_S.isStopDecalS = im.BoolPtr(false)
    profileBR_S.isStopDecalE = im.BoolPtr(false)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadBR_S
    roadMgr.map[roadBR_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, boxYHalf + bevel, 0), rot) + cen)
    local pCen = vec3(boxXHalf + bevel, boxYHalf + bevel, 0)
    local pMid = pCen + downLeft * bevel
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(pMid, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel, boxYHalf, 0), rot) + cen)
    roadBR_S.nodes[1].isLocked = true
    roadBR_S.nodes[2].isLocked = true
    roadBR_S.nodes[3].isLocked = true

    -- Apply the dipped kerb corners to the sidewalk profiles, if requested.
    if isLowerSWAtPedX then
      roadX1_O.nodes[5].heightsL[numLanesX + 1] = im.FloatPtr(0.01)
      roadX1_O.nodes[5].heightsL[-numLanesX - 1] = im.FloatPtr(0.01)
      roadX2_O.nodes[5].heightsL[numLanesX + 1] = im.FloatPtr(0.01)
      roadX2_O.nodes[5].heightsL[-numLanesX - 1] = im.FloatPtr(0.01)
      if isYOneWay then
        if isY2Outwards then
          roadY2_O.nodes[5].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
          roadY2_O.nodes[5].heightsL[-1] = im.FloatPtr(0.01)
        else
          roadY2_O.nodes[1].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
          roadY2_O.nodes[1].heightsL[-1] = im.FloatPtr(0.01)
        end
      else
        roadY2_O.nodes[5].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
        roadY2_O.nodes[5].heightsL[-numLanesY - 1] = im.FloatPtr(0.01)
      end
      roadT_S.nodes[1].heightsL[1] = im.FloatPtr(0.01)
      roadT_S.nodes[2].heightsL[1] = im.FloatPtr(0.01)
      roadT_S.nodes[5].heightsL[1] = im.FloatPtr(0.01)
      roadT_S.nodes[6].heightsL[1] = im.FloatPtr(0.01)
    end

    jct.roads = {
      roadX1_I.name, roadX2_I.name, roadY2_I.name,
      roadX1_O.name, roadX2_O.name, roadY2_O.name,
      roadT_S.name, roadBL_S.name, roadBR_S.name,
      roadCR_X.name }
  else
    jct.roads = {
      roadX1_I.name, roadX2_I.name, roadY2_I.name,
      roadX1_O.name, roadX2_O.name, roadY2_O.name,
      roadCR_X.name }
  end

  -- Create the traffic light booms, if requested.
  if jct.isTLights[0] then
    local lMin, lMax = profileMgr.getMinMaxLaneKeys(profileY2_I)
    local sRot = 3
    local laneIdx = lMin
    local sPos = 0.0
    local offsetSign = 1.0
    local sIsLeft = true
    if isYOneWay and not isY2Outwards then
      sRot = 1
      laneIdx = lMax
      sPos = 1.0
      offsetSign = -1.0
      sIsLeft = false
    end
    profileY2_I.layers[#profileY2_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom A'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(laneIdx), laneMax = im.IntPtr(laneIdx),
        lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(sIsLeft), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(sRot),
        pos = im.FloatPtr(sPos), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0] * offsetSign),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }

    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX1_I)
    profileX1_I.layers[#profileX1_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom B'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(0.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }

    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX2_I)
    profileX2_I.layers[#profileX2_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom C'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(0.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }
  end

  -- Add the arrow decals, if requested.
  if jct.isArrow[0] then
    if numLanesX > 1 then
      for i = 1, numLanesX do
        local frame1, frame2 = 2, 2
        if i == 1 then
          frame1, frame2 = 0, 2
        elseif i == numLanesX then
          frame1, frame2 = 2, 1
        end
        if numLanesX == 2 and i == 1 then
          frame1, frame2 = 0, 2
        end
        if numLanesX == 1 then
          frame1, frame2 = 2, 2
        end
        local aPos = (capLength - jct.arrowFrontDistFromEnd[0]) / capLength
        if aPos < 0.0 or aPos > 1.0 then
          aPos = 0.5
        end
        profileX1_O.layers[#profileX1_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow X1 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame1),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        profileX2_O.layers[#profileX2_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow X2 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame2),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        if jct.isDoubleArrows[0] and capLength > jct.arrowBackDistFromEnd[0] + jct.arrowSize[0] + 0.1 then
          profileX1_O.layers[#profileX1_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow X1 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame1),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
          profileX2_O.layers[#profileX2_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow X2 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame2),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        end
      end
    end
    if not isY2Outwards and numLanesY > 1 then
      for i = 1, numLanesY do
        local frame = 2
        if i == 1 then
          frame = 0
        elseif i == numLanesY then
          frame = 1
        end
        if numLanesY == 2 and i == 1 then
          frame = 2
        end
        if numLanesY == 1 then
          frame = 2
        end
        local aPos = (capLength - jct.arrowFrontDistFromEnd[0]) / capLength
        if aPos < 0.0 or aPos > 1.0 then
          aPos = 0.5
        end
        local sRot = 3
        local laneIdx = -i
        if isYOneWay and not isY2Outwards then
          sRot = 1
          laneIdx = i
        end
        profileY2_O.layers[#profileY2_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow Y2 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
          lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(sRot),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        if jct.isDoubleArrows[0] and capLength > jct.arrowBackDistFromEnd[0] + jct.arrowSize[0] + 0.1 then
          profileY2_O.layers[#profileY2_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow Y2 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
            lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(sRot),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        end
      end
    end
  end

  -- Add road signs (poles).
  if jct.isSigns[0] then
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX1_O)
    profileX1_O.layers[#profileX1_O.layers + 1] = {
      name = im.ArrayChar(32, 'T-Junction Sign X1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultTJunctionSignL,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.5), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_junction_left.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX2_O)
    profileX2_O.layers[#profileX2_O.layers + 1] = {
      name = im.ArrayChar(32, 'T-Junction Sign X2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultTJunctionSignR,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.5), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_junction_left.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, lMax = profileMgr.getMinMaxLaneKeys(profileY2_O)
    local sRot = 3
    local laneIdx = lMin
    local sIsLeft = true
    local latOffSign = -1.0
    if isYOneWay and not isY2Outwards then
      sRot = 1
      laneIdx = lMax
      sIsLeft = false
      latOffSign = 1.0
    end
    profileY2_O.layers[#profileY2_O.layers + 1] = {
      name = im.ArrayChar(32, 'T-Junction Sign Y2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(laneIdx), isLeft = im.BoolPtr(sIsLeft), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultLeftOrRightSign,
      rot = im.IntPtr(sRot),
      pos = im.FloatPtr(0.5), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(latOffSign * 0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_turn_leftorright.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end

    -- Create the pedestrian crossing decals, if requested.
  if isPedX1 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesX), laneMax = im.IntPtr(numLanesX),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileX1_I.layers, 1, pedX)
  end
  if isPedX2 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesX), laneMax = im.IntPtr(numLanesX),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileX2_I.layers, 1, pedX)
  end
  if isPedX3 then
    local lMin, lMax = -numLanesY, numLanesY
    if isYOneWay then
      lMin, lMax = 1, numLanesY
    end
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R4'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMax),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileY2_I.layers, 1, pedX)
  end

  if jct.isCrossings[0] then
    overlayUtils.addTJunctionOverlays(jct)
  end
end

-- Updates an urban Y-junction.
local function updateYJunction(jIdx, jct, isMesh)
  local numLanesX, numLanesY = jct.numLanesX[0], jct.numLanesY[0]
  local laneWidthX, laneWidthY = jct.laneWidthX[0], jct.laneWidthY[0]
  local isPedX1, isPedX2, isPedX3 = jct.isPedX1[0], jct.isPedX2[0], jct.isPedX3[0]
  local pedXWidth = jct.pedXWidth[0]
  local bevel = jct.bevel[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local capLength = jct.capLength[0]
  local isSidewalk = jct.isSidewalk[0]

  local boxX, boxY = numLanesY * 2 * laneWidthY, numLanesX * 2 * laneWidthX
  local boxXHalf, boxYHalf = boxX * 0.5, boxY * 0.5

  -- Create the profile for the center cross roads.
  local profileCR_X = nil
  if isSidewalk then
    profileCR_X = profileMgr.createProfileForJctRoad_SW(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)
  else
    profileCR_X = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)
  end
  profileCR_X.layers = {}
  profileMgr.addEdgeLines(profileCR_X, 0.2, 0.2, false, true)
  if not isSidewalk then
    profileMgr.autoEdgeBlending(profileCR_X, false, true, jct.edgeBlendMat)
  end

  -- Create the other profiles.
  local profileX1_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileX2_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileY2_O = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileY2_I = nil
  if isSidewalk then
    if jct.theta[0] >= 0.0 then
      profileY2_I = profileMgr.createProfileForJctRoad_SW(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
    else
      profileY2_I = profileMgr.createProfileForJctRoad_SW(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, true, jct.edgeBlendMat)
    end
  else
    profileY2_I = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  end
  profileY2_I.layers = {}

  local profileB_S = profileMgr.createProfileForJctRoadYSpecial(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, false, jct.edgeBlendMat)
  profileB_S.layers = {}

  if jct.theta[0] >= 0.0 then
    profileMgr.addEdgeLines(profileY2_I, 0.2, 0.2, false, true)
    profileMgr.addEdgeLines(profileB_S, 0.2, 0.2, false, true)
    if not isSidewalk then
      profileMgr.autoEdgeBlending(profileY2_I, false, true, jct.edgeBlendMat)
      profileMgr.autoEdgeBlending(profileB_S, false, true, jct.edgeBlendMat)
    end
  else
    profileMgr.addEdgeLines(profileY2_I, 0.2, 0.2, true, false)
    profileMgr.addEdgeLines(profileB_S, 0.2, 0.2, false, true)
    if not isSidewalk then
      profileMgr.autoEdgeBlending(profileY2_I, true, false, jct.edgeBlendMat)
      profileMgr.autoEdgeBlending(profileB_S, false, true, jct.edgeBlendMat)
    end
  end

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  local isEdgeBlend = not isSidewalk

  -- Create the cross road in the center.
  local roadCR_X = roadMgr.createRoadFromProfile(profileCR_X)
  roadCR_X.displayName = im.ArrayChar(32, 'jct cross X')
  roadCR_X.isJctRoad = true
  profileCR_X.conditionCenterline = im.BoolPtr(false)
  profileCR_X.conditionLaneMarkings = im.BoolPtr(false)
  profileCR_X.isEdgeBlendL = im.BoolPtr(false)
  profileCR_X.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileCR_X.conditionEndStopS = im.BoolPtr(false)
  profileCR_X.conditionEndStopE = im.BoolPtr(false)
  profileCR_X.conditionEdgesL = im.BoolPtr(false)
  profileCR_X.conditionEdgesR = im.BoolPtr(true)
  profileCR_X.isStopDecalS = im.BoolPtr(false)
  profileCR_X.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadCR_X
  roadMgr.map[roadCR_X.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, 0, 0), rot) + cen)
  roadCR_X.nodes[1].isLocked = true
  roadCR_X.nodes[2].isLocked = true

  local innerShift = 0.2

  -- Create the outer roads.
  local roadX1_O = roadMgr.createRoadFromProfile(profileX1_O)
  roadX1_O.displayName = im.ArrayChar(32, 'jct road 4')
  roadX1_O.isJctRoad = true
  profileX1_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX1_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX1_O.isStopDecalS = im.BoolPtr(false)
  profileX1_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_O
  roadMgr.map[roadX1_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength, 0, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(-boxXHalf - bevel + innerShift, 0, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength * 2, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength * 1.5, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p3)
  roadX1_O.nodes[1].isLocked = false
  roadX1_O.nodes[2].isLocked = true
  roadX1_O.nodes[3].isLocked = true
  roadX1_O.nodes[4].isLocked = true
  roadX1_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadX2_O = roadMgr.createRoadFromProfile(profileX2_O)
  roadX2_O.displayName = im.ArrayChar(32, 'jct road 5')
  roadX2_O.isJctRoad = true
  profileX2_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX2_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX2_O.isStopDecalS = im.BoolPtr(false)
  profileX2_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_O
  roadMgr.map[roadX2_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength, 0, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(boxXHalf + bevel - innerShift, 0, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength * 2, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength * 1.5, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p3)
  roadX2_O.nodes[1].isLocked = false
  roadX2_O.nodes[2].isLocked = true
  roadX2_O.nodes[3].isLocked = true
  roadX2_O.nodes[4].isLocked = true
  roadX2_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  -- Rotate the y-road by the given angle.
  local angRad = deg2Rad(jct.theta[0])
  local s, c = sin(angRad), cos(angRad)
  local p1 = vec3(0, boxYHalf + bevel + capLength - 0.2, 0)
  local x, y = p1.x, p1.y
  p1:set(x * c - y * s, x * s + y * c, 0.0)
  local p2 = vec3(0, boxYHalf + bevel - 0.2 + innerShift, 0)
  local x, y = p2.x, p2.y
  p2:set(x * c - y * s, x * s + y * c, 0.0)

  local roadY2_O = roadMgr.createRoadFromProfile(profileY2_O)
  roadY2_O.displayName = im.ArrayChar(32, 'jct road 6')
  roadY2_O.isJctRoad = true
  profileY2_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY2_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY2_O.isStopDecalS = im.BoolPtr(false)
  profileY2_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY2_O
  roadMgr.map[roadY2_O.name] = rIdx
  local np1 = util.rotateVecByQuaternion(p1, rot) + cen
  local np3 = util.rotateVecByQuaternion(p2, rot) + cen
  local np2 = np1 + (np3 - np1) * 0.5
  roadMgr.addNodeToRoad(rIdx, np1 + (np1 - np3):normalized() * capLength)
  roadMgr.addNodeToRoad(rIdx, np1 + (np1 - np3):normalized() * capLength * 0.5)
  roadMgr.addNodeToRoad(rIdx, np1)
  roadMgr.addNodeToRoad(rIdx, np2)
  roadMgr.addNodeToRoad(rIdx, np3)
  roadY2_O.nodes[1].isLocked = false
  roadY2_O.nodes[2].isLocked = true
  roadY2_O.nodes[3].isLocked = true
  roadY2_O.nodes[4].isLocked = true
  roadY2_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadY2_I1 = roadMgr.createRoadFromProfile(profileY2_I)
  roadY2_I1.displayName = im.ArrayChar(32, 'jct road 7')
  roadY2_I1.granFactor = im.IntPtr(2)
  roadY2_I1.isArc = true
  roadY2_I1.isJctRoad = true
  profileY2_I.conditionCenterline = im.BoolPtr(false)
  profileY2_I.conditionLaneMarkings = im.BoolPtr(false)
  profileY2_I.conditionEndStopS = im.BoolPtr(false)
  profileY2_I.conditionEndStopE = im.BoolPtr(false)
  profileY2_I.isStopDecalS = im.BoolPtr(false)
  profileY2_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY2_I1
  roadMgr.map[roadY2_I1.name] = rIdx
  local ap1, ap2, ap3 = nil, nil, nil
  if jct.theta[0] >= 0.0 then
    profileY2_I.isEdgeBlendL = im.BoolPtr(false)
    profileY2_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
    profileY2_I.conditionEdgesL = im.BoolPtr(false)
    profileY2_I.conditionEdgesR = im.BoolPtr(true)
    ap1 = util.rotateVecByQuaternion(vec3(boxXHalf + bevel, 0, 0), rot) + cen
    local xLat = roadX2_O.renderData[#roadX2_O.renderData][1][6]
    ap3 = vec3(np3.x, np3.y, np3.z)
    local aCtr = util.intersection2Lines(ap3, ap3 + roadY2_O.renderData[#roadY2_O.renderData][1][6], ap1, ap1 + xLat)
    local ang = util.angleBetweenVecs(ap3 - aCtr, ap1 - aCtr) * 0.5
    ap2 = aCtr + util.rotateVecAroundAxis(ap3 - aCtr, vertical, ang)
  else
    profileY2_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
    profileY2_I.isEdgeBlendR = im.BoolPtr(false)
    profileY2_I.conditionEdgesL = im.BoolPtr(true)
    profileY2_I.conditionEdgesR = im.BoolPtr(false)
    ap1 = util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, 0, 0), rot) + cen
    local xLat = roadX1_O.renderData[#roadX1_O.renderData][1][6]
    ap3 = vec3(np3.x, np3.y, np3.z)
    local aCtr = util.intersection2Lines(ap3, ap3 + roadY2_O.renderData[#roadY2_O.renderData][1][6], ap1, ap1 + xLat)
    local ang = util.angleBetweenVecs(ap3 - aCtr, ap1 - aCtr) * 0.5
    ap2 = aCtr + util.rotateVecAroundAxis(ap3 - aCtr, vertical, -ang)
  end
  roadMgr.addNodeToRoad(rIdx, ap3)
  roadMgr.addNodeToRoad(rIdx, ap2)
  roadMgr.addNodeToRoad(rIdx, ap1)
  roadY2_I1.nodes[1].isLocked = true
  roadY2_I1.nodes[2].isLocked = true
  roadY2_I1.nodes[3].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  -- Create the inner corner road.
  local roadB_S = roadMgr.createRoadFromProfile(profileB_S)
  roadB_S.displayName = im.ArrayChar(32, 'jct s-walk btm')
  roadB_S.isDrivable = false
  roadB_S.isArc = true
  roadB_S.isJctRoad = true
  roadB_S.granFactor = im.IntPtr(3)
  profileB_S.conditionEndStopS = im.BoolPtr(false)
  profileB_S.conditionEndStopE = im.BoolPtr(false)
  profileB_S.isStopDecalS = im.BoolPtr(false)
  profileB_S.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadB_S
  roadMgr.map[roadB_S.name] = rIdx

  local ap1, ap2, ap3 = nil, nil, nil
  if jct.theta[0] < 0.0 then
    profileB_S.conditionEdgesL = im.BoolPtr(false)
    profileB_S.conditionEdgesR = im.BoolPtr(true)
    profileB_S.isEdgeBlendL = im.BoolPtr(false)
    profileB_S.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
    ap1 = util.rotateVecByQuaternion(vec3(boxXHalf + bevel, 0, 0), rot) + cen
    local xLat = roadX2_O.renderData[#roadX2_O.renderData][1][6]
    ap3 = vec3(np3.x, np3.y, np3.z)
    local aCtr = util.intersection2Lines(ap3, ap3 + roadY2_O.renderData[#roadY2_O.renderData][1][6], ap1, ap1 + xLat)
    local ang = util.angleBetweenVecs(ap3 - aCtr, ap1 - aCtr) * 0.5
    ap2 = aCtr + util.rotateVecAroundAxis(ap3 - aCtr, vertical, ang)
    roadMgr.addNodeToRoad(rIdx, ap3)
    roadMgr.addNodeToRoad(rIdx, ap2)
    roadMgr.addNodeToRoad(rIdx, ap1)
  else
    profileB_S.conditionEdgesL = im.BoolPtr(false)
    profileB_S.conditionEdgesR = im.BoolPtr(true)
    profileB_S.isEdgeBlendL = im.BoolPtr(false)
    profileB_S.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
    ap1 = util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, 0, 0), rot) + cen
    local xLat = roadX1_O.renderData[#roadX1_O.renderData][1][6]
    ap3 = vec3(np3.x, np3.y, np3.z)
    local aCtr = util.intersection2Lines(ap3, ap3 + roadY2_O.renderData[#roadY2_O.renderData][1][6], ap1, ap1 + xLat)
    local ang = util.angleBetweenVecs(ap3 - aCtr, ap1 - aCtr) * 0.5
    ap2 = aCtr + util.rotateVecAroundAxis(ap3 - aCtr, vertical, -ang)
    roadMgr.addNodeToRoad(rIdx, ap1)
    roadMgr.addNodeToRoad(rIdx, ap2)
    roadMgr.addNodeToRoad(rIdx, ap3)
  end
  roadB_S.nodes[1].isLocked = true
  roadB_S.nodes[2].isLocked = true
  roadB_S.nodes[3].isLocked = true

  jct.roads = {
    roadX1_O.name, roadX2_O.name,
    roadY2_O.name, roadY2_I1.name,
    roadCR_X.name,
    roadB_S.name }

  -- Create the traffic light booms, if requested.
  if jct.isTLights[0] then
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX1_O)
    profileX1_O.layers[#profileX1_O.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom A'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(1.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }

    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileY2_O)
    profileY2_O.layers[#profileY2_O.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom B'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(1.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }

    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX2_O)
    profileX2_O.layers[#profileX2_O.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom C'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(1.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }
  end

  -- Add the arrow decals, if requested.
  if jct.isArrow[0] then
    if numLanesX > 1 then
      for i = 1, numLanesX do
        local frame1, frame2 = 2, 2
        if i == 1 then
          frame1, frame2 = 0, 2
        elseif i == numLanesX then
          frame1, frame2 = 2, 1
        end
        if numLanesX == 2 and i == 1 then
          frame1, frame2 = 0, 2
        end
        if numLanesX == 1 then
          frame1, frame2 = 2, 2
        end
        local aPos = (capLength - jct.arrowFrontDistFromEnd[0]) / capLength
        if aPos < 0.0 or aPos > 1.0 then
          aPos = 0.5
        end
        profileX1_O.layers[#profileX1_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow X1 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame1),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        profileX2_O.layers[#profileX2_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow X2 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame2),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        if jct.isDoubleArrows[0] and capLength > jct.arrowBackDistFromEnd[0] + jct.arrowSize[0] + 0.1 then
          profileX1_O.layers[#profileX1_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow X1 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame1),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
          profileX2_O.layers[#profileX2_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow X2 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame2),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        end
      end
    end
    if numLanesY > 1 then
      for i = 1, numLanesY do
        local frame = 2
        if i == 1 then
          frame = 0
        elseif i == numLanesY then
          frame = 1
        end
        if numLanesY == 2 and i == 1 then
          frame = 2
        end
        if numLanesY == 1 then
          frame = 2
        end
        local aPos = (capLength - jct.arrowFrontDistFromEnd[0]) / capLength
        if aPos < 0.0 or aPos > 1.0 then
          aPos = 0.5
        end
        profileY2_O.layers[#profileY2_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow Y2 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        if jct.isDoubleArrows[0] and capLength > jct.arrowBackDistFromEnd[0] + jct.arrowSize[0] + 0.1 then
          profileY2_O.layers[#profileY2_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow Y2 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        end
      end
    end
  end

  -- Add road signs (poles).
  if jct.isSigns[0] then
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX1_O)
    profileX1_O.layers[#profileX1_O.layers + 1] = {
      name = im.ArrayChar(32, 'T-Junction Sign X1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultTJunctionSignL,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.1), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_junction_left.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX2_O)
    profileX2_O.layers[#profileX2_O.layers + 1] = {
      name = im.ArrayChar(32, 'T-Junction Sign X2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultTJunctionSignR,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.1), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_junction_left.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileY2_O)
    profileY2_O.layers[#profileY2_O.layers + 1] = {
      name = im.ArrayChar(32, 'T-Junction Sign Y2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultLeftOrRightSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.1), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_turn_leftorright.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end

    -- Create the pedestrian crossing decals, if requested.
  if isPedX1 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesX), laneMax = im.IntPtr(numLanesX),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(1.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileX1_O.layers, 1, pedX)
  end
  if isPedX2 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesX), laneMax = im.IntPtr(numLanesX),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(1.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileX2_O.layers, 1, pedX)
  end
  if isPedX3 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R4'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesY), laneMax = im.IntPtr(numLanesY),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(1.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileY2_O.layers, 1, pedX)
  end

  if jct.isCrossings[0] then
    overlayUtils.addYJunctionOverlays(jct)
  end
end

-- Updates an urban roundabout junction.
local function updateRoundabout(jIdx, jct, isMesh)
  local numLanesX, numLanesY = jct.numLanesX[0], jct.numLanesY[0]
  local laneWidthX, laneWidthY = jct.laneWidthX[0], jct.laneWidthY[0]
  local isPedX1, isPedX2, isPedX3, isPedX4 = jct.isPedX1[0], jct.isPedX2[0], jct.isPedX3[0], jct.isPedX4[0]
  local pedXWidth = jct.pedXWidth[0]
  local isSidewalk = jct.isSidewalk[0]
  local bevel = jct.bevel[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local isLowerSWAtPedX = jct.isLowerSWAtPedX[0]
  local capLength = jct.capLength[0]
  local numRBLanes, laneWidthRB = jct.numRBLanes[0], jct.laneWidthRB[0]

  local boxX, boxY = numLanesY * 2 * laneWidthY, numLanesX * 2 * laneWidthX
  local boxXHalf, boxYHalf = boxX * 0.5, boxY * 0.5

  -- Create the four inner road profiles.
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  local profileX2_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  local profileY1_I = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  local profileY2_I = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  profileX1_I.layers = {}
  profileX2_I.layers = {}
  profileY1_I.layers = {}
  profileY2_I.layers = {}
  if not isSidewalk then
    profileMgr.autoEdgeBlending(profileX1_I, true, true, jct.edgeBlendMat)
    profileMgr.autoEdgeBlending(profileX2_I, true, true, jct.edgeBlendMat)
    profileMgr.autoEdgeBlending(profileY1_I, true, true, jct.edgeBlendMat)
    profileMgr.autoEdgeBlending(profileY2_I, true, true, jct.edgeBlendMat)
  end

  -- Create the four outer road profiles (with sidewalks, if requested).
  local profileX1_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileX2_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileY1_O = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileY2_O = profileMgr.createProfileForJctRoad(numLanesY, numLanesY, laneWidthY, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)

  -- Create the four sidewalk-only profiles at each junction corner.
  local profileTL_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)
  local profileTR_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)
  local profileBL_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)
  local profileBR_S = profileMgr.createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight)

  -- Create the circular roundabout profiles.
  local profileRB_1 = profileMgr.createRoundaboutProfile(numRBLanes, laneWidthRB, jct.edgeBlendMat)
  local profileRB_2 = profileMgr.createRoundaboutProfile(numRBLanes, laneWidthRB, jct.edgeBlendMat)
  profileMgr.addEdgeLines(profileRB_1, 0.2, 0.2, false, true, false)
  profileMgr.addEdgeLines(profileRB_2, 0.2, 0.2, false, true, false)
  profileMgr.autoEdgeBlending(profileRB_1, false, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileRB_2, false, true, jct.edgeBlendMat)

  if isLowerSWAtPedX then
    profileTL_S[1].heightL = im.FloatPtr(0.01)
    profileTR_S[1].heightL = im.FloatPtr(0.01)
    profileBL_S[1].heightL = im.FloatPtr(0.01)
    profileBR_S[1].heightL = im.FloatPtr(0.01)
  end

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  local isEdgeBlend = not isSidewalk

  -- Create the inner roads.
  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'jct road 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX1_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX1_I.conditionCenterline = im.BoolPtr(false)
  profileX1_I.conditionEdgesL = im.BoolPtr(false)
  profileX1_I.conditionEdgesR = im.BoolPtr(false)
  profileX1_I.conditionLaneMarkings = im.BoolPtr(false)
  profileX1_I.conditionEndStopS = im.BoolPtr(false)
  profileX1_I.conditionEndStopE = im.BoolPtr(false)
  profileX1_I.isStopDecalS = im.BoolPtr(false)
  profileX1_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel * 0.5, 0, 0), rot) + cen)
  roadX1_I.nodes[1].isLocked = true
  roadX1_I.nodes[2].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'jct road 2')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX2_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX2_I.conditionCenterline = im.BoolPtr(false)
  profileX2_I.conditionEdgesL = im.BoolPtr(false)
  profileX2_I.conditionEdgesR = im.BoolPtr(false)
  profileX2_I.conditionLaneMarkings = im.BoolPtr(false)
  profileX2_I.conditionEndStopS = im.BoolPtr(false)
  profileX2_I.conditionEndStopE = im.BoolPtr(false)
  profileX2_I.isStopDecalS = im.BoolPtr(false)
  profileX2_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel * 0.5, 0, 0), rot) + cen)
  roadX2_I.nodes[1].isLocked = true
  roadX2_I.nodes[2].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadY1_I = roadMgr.createRoadFromProfile(profileY1_I)
  roadY1_I.displayName = im.ArrayChar(32, 'jct road 3')
  roadY1_I.isJctRoad = true
  profileY1_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY1_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY1_I.conditionCenterline = im.BoolPtr(false)
  profileY1_I.conditionEdgesL = im.BoolPtr(false)
  profileY1_I.conditionEdgesR = im.BoolPtr(false)
  profileY1_I.conditionLaneMarkings = im.BoolPtr(false)
  profileY1_I.conditionEndStopS = im.BoolPtr(false)
  profileY1_I.conditionEndStopE = im.BoolPtr(false)
  profileY1_I.isStopDecalS = im.BoolPtr(false)
  profileY1_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY1_I
  roadMgr.map[roadY1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, -boxYHalf - bevel, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, -boxYHalf - bevel * 0.5, 0), rot) + cen)
  roadY1_I.nodes[1].isLocked = true
  roadY1_I.nodes[2].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadY2_I = roadMgr.createRoadFromProfile(profileY2_I)
  roadY2_I.displayName = im.ArrayChar(32, 'jct road 4')
  roadY2_I.isJctRoad = true
  profileY2_I.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY2_I.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY2_I.conditionCenterline = im.BoolPtr(false)
  profileY2_I.conditionEdgesL = im.BoolPtr(false)
  profileY2_I.conditionEdgesR = im.BoolPtr(false)
  profileY2_I.conditionLaneMarkings = im.BoolPtr(false)
  profileY2_I.conditionEndStopS = im.BoolPtr(false)
  profileY2_I.conditionEndStopE = im.BoolPtr(false)
  profileY2_I.isStopDecalS = im.BoolPtr(false)
  profileY2_I.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY2_I
  roadMgr.map[roadY2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, boxYHalf + bevel, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, boxYHalf + bevel * 0.5, 0), rot) + cen)
  roadY2_I.nodes[1].isLocked = true
  roadY2_I.nodes[2].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  -- Create the outer roads.
  local roadX1_O = roadMgr.createRoadFromProfile(profileX1_O)
  roadX1_O.displayName = im.ArrayChar(32, 'jct road 5')
  roadX1_O.isJctRoad = true
  profileX1_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX1_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX1_O.isStopDecalS = im.BoolPtr(false)
  profileX1_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_O
  roadMgr.map[roadX1_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength, 0, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, 0, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength * 2, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel - capLength * 1.5, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p3)
  roadX1_O.nodes[1].isLocked = false
  roadX1_O.nodes[2].isLocked = true
  roadX1_O.nodes[3].isLocked = true
  roadX1_O.nodes[4].isLocked = true
  roadX1_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadX2_O = roadMgr.createRoadFromProfile(profileX2_O)
  roadX2_O.displayName = im.ArrayChar(32, 'jct road 6')
  roadX2_O.isJctRoad = true
  profileX2_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileX2_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileX2_O.isStopDecalS = im.BoolPtr(false)
  profileX2_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_O
  roadMgr.map[roadX2_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength, 0, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(boxXHalf + bevel, 0, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength * 2, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel + capLength * 1.5, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p3)
  roadX2_O.nodes[1].isLocked = false
  roadX2_O.nodes[2].isLocked = true
  roadX2_O.nodes[3].isLocked = true
  roadX2_O.nodes[4].isLocked = true
  roadX2_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadY1_O = roadMgr.createRoadFromProfile(profileY1_O)
  roadY1_O.displayName = im.ArrayChar(32, 'jct road 7')
  roadY1_O.isJctRoad = true
  profileY1_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY1_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY1_O.isStopDecalS = im.BoolPtr(false)
  profileY1_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY1_O
  roadMgr.map[roadY1_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(0, -boxYHalf - bevel - capLength, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(0, -boxYHalf - bevel, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, -boxYHalf - bevel - capLength * 2, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, -boxYHalf - bevel - capLength * 1.5, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p3)
  roadY1_O.nodes[1].isLocked = false
  roadY1_O.nodes[2].isLocked = true
  roadY1_O.nodes[3].isLocked = true
  roadY1_O.nodes[4].isLocked = true
  roadY1_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadY2_O = roadMgr.createRoadFromProfile(profileY2_O)
  roadY2_O.displayName = im.ArrayChar(32, 'jct road 8')
  roadY2_O.isJctRoad = true
  profileY2_O.isEdgeBlendL = im.BoolPtr(isEdgeBlend)
  profileY2_O.isEdgeBlendR = im.BoolPtr(isEdgeBlend)
  profileY2_O.isStopDecalS = im.BoolPtr(false)
  profileY2_O.isStopDecalE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadY2_O
  roadMgr.map[roadY2_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(0, boxYHalf + bevel + capLength, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(0, boxYHalf + bevel, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, boxYHalf + bevel + capLength * 2, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, boxYHalf + bevel + capLength * 1.5, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p3)
  roadY2_O.nodes[1].isLocked = false
  roadY2_O.nodes[2].isLocked = true
  roadY2_O.nodes[3].isLocked = true
  roadY2_O.nodes[4].isLocked = true
  roadY2_O.nodes[5].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  -- Apply the dipped kerb corners to the sidewalk in the outer profiles, if requested.
  if isLowerSWAtPedX then
    roadX1_O.nodes[5].heightsL[numLanesX + 1] = im.FloatPtr(0.01)
    roadX1_O.nodes[5].heightsL[-numLanesX - 1] = im.FloatPtr(0.01)
    roadX2_O.nodes[5].heightsL[numLanesX + 1] = im.FloatPtr(0.01)
    roadX2_O.nodes[5].heightsL[-numLanesX - 1] = im.FloatPtr(0.01)
    roadY1_O.nodes[5].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
    roadY1_O.nodes[5].heightsL[-numLanesY - 1] = im.FloatPtr(0.01)
    roadY2_O.nodes[5].heightsL[numLanesY + 1] = im.FloatPtr(0.01)
    roadY2_O.nodes[5].heightsL[-numLanesY - 1] = im.FloatPtr(0.01)
  end

  -- Create the two roundabout semi-circles.
  local roadRB_1 = roadMgr.createRoadFromProfile(profileRB_1)
  roadRB_1.displayName = im.ArrayChar(32, 'circular 1')
  roadRB_1.isJctRoad = true
  profileRB_1.isEdgeBlendL = im.BoolPtr(false)
  profileRB_1.isEdgeBlendR = im.BoolPtr(true)
  profileRB_1.isStopDecalS = im.BoolPtr(false)
  profileRB_1.isStopDecalE = im.BoolPtr(false)
  roadRB_1.isArc = true
  roadRB_1.granFactor = im.IntPtr(crossroadArcGran)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadRB_1
  roadMgr.map[roadRB_1.name] = rIdx
  local rp1 = vec3(0, boxYHalf + bevel * 0.65 - numRBLanes * laneWidthRB + jct.extraRadRB[0], 0)
  local rp2 = util.rotateVecAroundAxis(rp1, vertical, halfPi)
  rp2.z = 0.0
  local rp3 = util.rotateVecAroundAxis(rp1, vertical, pi + halfPi)
  rp3.z = 0.0
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(rp1, rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(rp2, rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(rp3, rot) + cen)
  roadRB_1.nodes[1].isLocked = true
  roadRB_1.nodes[2].isLocked = true
  roadRB_1.nodes[3].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadRB_2 = roadMgr.createRoadFromProfile(profileRB_2)
  roadRB_2.displayName = im.ArrayChar(32, 'circular 2')
  roadRB_2.isJctRoad = true
  profileRB_2.isEdgeBlendL = im.BoolPtr(false)
  profileRB_2.isEdgeBlendR = im.BoolPtr(true)
  profileRB_2.isStopDecalS = im.BoolPtr(false)
  profileRB_2.isStopDecalE = im.BoolPtr(false)
  roadRB_2.isArc = true
  roadRB_2.granFactor = im.IntPtr(crossroadArcGran)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadRB_2
  roadMgr.map[roadRB_2.name] = rIdx
  local rp1 = vec3(0, -boxYHalf - bevel * 0.65 + numRBLanes * laneWidthRB - jct.extraRadRB[0], 0)
  local rp2 = util.rotateVecAroundAxis(rp1, vertical, halfPi)
  rp2.z = 0.0
  local rp3 = util.rotateVecAroundAxis(rp1, vertical, pi + halfPi)
  rp3.z = 0.0
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(rp1, rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(rp2, rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(rp3, rot) + cen)
  roadRB_2.nodes[1].isLocked = true
  roadRB_2.nodes[2].isLocked = true
  roadRB_2.nodes[3].isLocked = true
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  -- Create the corner bevelled sidewalk roads.
  if isSidewalk then
    local roadTL_S = roadMgr.createRoadFromProfile(profileTL_S)
    roadTL_S.displayName = im.ArrayChar(32, 'jct s-walk TL')
    roadTL_S.isDrivable = false
    roadTL_S.isJctRoad = true
    profileTL_S.isEdgeBlendL = im.BoolPtr(false)
    profileTL_S.isEdgeBlendR = im.BoolPtr(false)
    profileTL_S.isStopDecalS = im.BoolPtr(false)
    profileTL_S.isStopDecalE = im.BoolPtr(false)
    roadTL_S.isArc = true
    roadTL_S.granFactor = im.IntPtr(crossroadArcGran)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadTL_S
    roadMgr.map[roadTL_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, boxYHalf, 0), rot) + cen)
    local pCen = vec3(-boxXHalf - bevel, boxYHalf + bevel, 0)
    local pMid = pCen + downRight * bevel
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(pMid, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, boxYHalf + bevel, 0), rot) + cen)
    roadTL_S.nodes[1].isLocked = true
    roadTL_S.nodes[2].isLocked = true
    roadTL_S.nodes[3].isLocked = true

    local roadTR_S = roadMgr.createRoadFromProfile(profileTR_S)
    roadTR_S.displayName = im.ArrayChar(32, 'jct s-walk TR')
    roadTR_S.isDrivable = false
    roadTR_S.isJctRoad = true
    profileTR_S.isEdgeBlendL = im.BoolPtr(false)
    profileTR_S.isEdgeBlendR = im.BoolPtr(false)
    profileTR_S.isStopDecalS = im.BoolPtr(false)
    profileTR_S.isStopDecalE = im.BoolPtr(false)
    roadTR_S.isArc = true
    roadTR_S.granFactor = im.IntPtr(crossroadArcGran)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadTR_S
    roadMgr.map[roadTR_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, boxYHalf + bevel, 0), rot) + cen)
    local pCen = vec3(boxXHalf + bevel, boxYHalf + bevel, 0)
    local pMid = pCen + downLeft * bevel
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(pMid, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel, boxYHalf, 0), rot) + cen)
    roadTR_S.nodes[1].isLocked = true
    roadTR_S.nodes[2].isLocked = true
    roadTR_S.nodes[3].isLocked = true

    local roadBL_S = roadMgr.createRoadFromProfile(profileBL_S)
    roadBL_S.displayName = im.ArrayChar(32, 'jct s-walk BL')
    roadBL_S.isDrivable = false
    roadBL_S.isJctRoad = true
    profileBL_S.isEdgeBlendL = im.BoolPtr(false)
    profileBL_S.isEdgeBlendR = im.BoolPtr(false)
    profileBL_S.isStopDecalS = im.BoolPtr(false)
    profileBL_S.isStopDecalE = im.BoolPtr(false)
    roadBL_S.isArc = true
    roadBL_S.granFactor = im.IntPtr(crossroadArcGran)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadBL_S
    roadMgr.map[roadBL_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, -boxYHalf - bevel, 0), rot) + cen)
    local pCen = vec3(-boxXHalf - bevel, -boxYHalf - bevel, 0)
    local pMid = pCen + upRight * bevel
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(pMid, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - bevel, -boxYHalf, 0), rot) + cen)
    roadBL_S.nodes[1].isLocked = true
    roadBL_S.nodes[2].isLocked = true
    roadBL_S.nodes[3].isLocked = true

    local roadBR_S = roadMgr.createRoadFromProfile(profileBR_S)
    roadBR_S.displayName = im.ArrayChar(32, 'jct s-walk BR')
    roadBR_S.isDrivable = false
    roadBR_S.isJctRoad = true
    profileBR_S.isEdgeBlendL = im.BoolPtr(false)
    profileBR_S.isEdgeBlendR = im.BoolPtr(false)
    profileBR_S.isStopDecalS = im.BoolPtr(false)
    profileBR_S.isStopDecalE = im.BoolPtr(false)
    roadBR_S.isArc = true
    roadBR_S.granFactor = im.IntPtr(1)
    local rIdx = #roadMgr.roads + 1
    roadMgr.roads[rIdx] = roadBR_S
    roadMgr.map[roadBR_S.name] = rIdx
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + bevel, -boxYHalf, 0), rot) + cen)
    local pCen = vec3(boxXHalf + bevel, -boxYHalf - bevel, 0)
    local pMid = pCen + upLeft * bevel
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(pMid, rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, -boxYHalf - bevel, 0), rot) + cen)
    roadBR_S.nodes[1].isLocked = true
    roadBR_S.nodes[2].isLocked = true
    roadBR_S.nodes[3].isLocked = true

    jct.roads = {
      roadX1_I.name, roadX2_I.name, roadY1_I.name, roadY2_I.name,
      roadX1_O.name, roadX2_O.name, roadY1_O.name, roadY2_O.name,
      roadRB_1.name, roadRB_2.name,
      roadTL_S.name, roadTR_S.name, roadBL_S.name, roadBR_S.name }
  else

    jct.roads = {
      roadX1_I.name, roadX2_I.name, roadY1_I.name, roadY2_I.name,
      roadX1_O.name, roadX2_O.name, roadY1_O.name, roadY2_O.name,
      roadRB_1.name, roadRB_2.name }
  end

  -- Create the traffic light booms, if requested.
  if jct.isTLights[0] then
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX1_I)
    profileX1_I.layers[#profileX1_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom A'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(0.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }

    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX2_I)
    profileX2_I.layers[#profileX2_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom B'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(0.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }

    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileY1_I)
    profileY1_I.layers[#profileY1_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom C'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(0.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }

    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileY2_I)
    profileY2_I.layers[#profileY2_I.layers + 1] =
      {
        name = im.ArrayChar(32, 'traffic boom D'),
        isHidden = false,
        doNotDelete = im.BoolPtr(true),
        isReverse = im.BoolPtr(false),
        isPaint = im.BoolPtr(false),
        isDisplay = im.BoolPtr(true),
        type = im.IntPtr(5),
        laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
        lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
        width = im.FloatPtr(1.0),
        isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
        texLen = im.FloatPtr(5),
        fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
        mat = trafficBoomMeshPath,
        rot = im.IntPtr(3),
        pos = im.FloatPtr(0.0), size = im.FloatPtr(3.0),
        numRows = im.IntPtr(1), numCols = im.IntPtr(1),
        frame = im.IntPtr(0),
        vertOffset = im.FloatPtr(0.0),
        latOffset = im.FloatPtr(jct.trafficLatOff[0]),
        spacing = im.FloatPtr(5.0),
        jitter = im.FloatPtr(0.0),
        useWorldZ = im.BoolPtr(false),
        matDisplay = 's_trafficlight_boom_ns.dae',
        extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
        boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
      }
  end

  -- Add road signs (poles).
  if jct.isSigns[0] then
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX1_O)
    profileX1_O.layers[#profileX1_O.layers + 1] = {
      name = im.ArrayChar(32, 'Roundabout Sign X1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultRoundaboutSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.1), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_roundabout.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileX2_O)
    profileX2_O.layers[#profileX2_O.layers + 1] = {
      name = im.ArrayChar(32, 'Roundabout Sign X2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultRoundaboutSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.1), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_roundabout.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileY1_O)
    profileY1_O.layers[#profileY1_O.layers + 1] = {
      name = im.ArrayChar(32, 'Roundabout Sign Y1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultRoundaboutSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.1), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_roundabout.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    local lMin, _ = profileMgr.getMinMaxLaneKeys(profileY2_O)
    profileY2_O.layers[#profileY2_O.layers + 1] = {
      name = im.ArrayChar(32, 'Roundabout Sign Y2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultRoundaboutSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.1), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_roundabout.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end

  -- Add the arrow decals, if requested.
  if jct.isArrow[0] then
    if numLanesX > 1 then
      for i = 1, numLanesX do
        local frame = 2
        if i == 1 then
          frame = 0
        elseif i == numLanesX then
          frame = 1
        end
        if numLanesX > 1 and i == 1 then
          frame = 2
        end
        if numLanesX == 1 then
          frame = 2
        end
        local aPos = (capLength - jct.arrowFrontDistFromEnd[0]) / capLength
        if aPos < 0.0 or aPos > 1.0 then
          aPos = 0.5
        end
        profileX1_O.layers[#profileX1_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow X1 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        profileX2_O.layers[#profileX2_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow X2 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        if jct.isDoubleArrows[0] and capLength > jct.arrowBackDistFromEnd[0] + jct.arrowSize[0] + 0.1 then
          profileX1_O.layers[#profileX1_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow X1 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
          profileX2_O.layers[#profileX2_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow X2 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        end
      end
    end
    if numLanesY > 1 then
      for i = 1, numLanesY do
        local frame = 2
        if i == 1 then
          frame = 0
        elseif i == numLanesY then
          frame = 1
        end
        if numLanesY > 2 and i == 1 then
          frame = 2
        end
        if numLanesY == 1 then
          frame = 2
        end
        local aPos = (capLength - jct.arrowFrontDistFromEnd[0]) / capLength
        if aPos < 0.0 or aPos > 1.0 then
          aPos = 0.5
        end
        profileY1_O.layers[#profileY1_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow Y1 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        profileY2_O.layers[#profileY2_O.layers + 1] = {
          name = im.ArrayChar(32, 'Arrow Y2 F' .. tostring(i)),
          isHidden = false,
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(3),
          laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
          lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr(aPos),
          width = im.FloatPtr(pedXWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = jct.arrowMat,
          rot = im.IntPtr(3),
          pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
          numRows = im.IntPtr(4), numCols = im.IntPtr(4),
          frame = im.IntPtr(frame),
          vertOffset = im.FloatPtr(0.0),
          latOffset = im.FloatPtr(0.0),
          spacing = im.FloatPtr(5.0),
          jitter = im.FloatPtr(0.0),
          useWorldZ = im.BoolPtr(false),
          matDisplay = '[None]',
          extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        if jct.isDoubleArrows[0] and capLength > jct.arrowBackDistFromEnd[0] + jct.arrowSize[0] + 0.1 then
          profileY1_O.layers[#profileY1_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow Y1 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
          profileY2_O.layers[#profileY2_O.layers + 1] = {
            name = im.ArrayChar(32, 'Arrow Y2 B' .. tostring(i)),
            isHidden = false,
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(false),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(3),
            laneMin = im.IntPtr(-i), laneMax = im.IntPtr(-i),
            lane = im.IntPtr(-i), isLeft = im.BoolPtr(true), off = im.FloatPtr((capLength - jct.arrowBackDistFromEnd[0]) / capLength),
            width = im.FloatPtr(pedXWidth),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = jct.arrowMat,
            rot = im.IntPtr(3),
            pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
            numRows = im.IntPtr(4), numCols = im.IntPtr(4),
            frame = im.IntPtr(frame),
            vertOffset = im.FloatPtr(0.0),
            latOffset = im.FloatPtr(0.0),
            spacing = im.FloatPtr(5.0),
            jitter = im.FloatPtr(0.0),
            useWorldZ = im.BoolPtr(false),
            matDisplay = '[None]',
            extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
        end
      end
    end
  end

  -- Create the pedestrian crossing decals, if requested.
  if isPedX1 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesX), laneMax = im.IntPtr(numLanesX),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileX1_I.layers, 1, pedX)
  end
  if isPedX2 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesX), laneMax = im.IntPtr(numLanesX),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileX2_I.layers, 1, pedX)
  end
  if isPedX3 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R3'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesY), laneMax = im.IntPtr(numLanesY),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileY1_I.layers, 1, pedX)
  end
  if isPedX4 then
    local pedX = {
      name = im.ArrayChar(32, 'Ped X - R4'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(2),
      laneMin = im.IntPtr(-numLanesY), laneMax = im.IntPtr(numLanesY),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(pedXWidth),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPedXMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileY2_I.layers, 1, pedX)
  end

  if jct.isCrossings[0] then
    overlayUtils.addRoundaboutOverlays(jct, cen)
  end
end

-- Updates an rural/urban transition junction.
local function updateRuralUrbanTransition(jIdx, jct, isMesh)
  local numLanesX = jct.numLanesX[0]
  local laneWidthX= jct.laneWidthX[0]
  local isOneWay = jct.isYOneWay[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local capLength = jct.capLength[0]

  -- Create the outer road profiles (with sidewalks, if requested).
  local profileX1_O, profileX2_O, profileX3_O = nil, nil, nil
  if isOneWay then
    profileX1_O = profileMgr.createProfileForJctRoad1Way(numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, true, jct.edgeBlendMat)
    profileX2_O = profileMgr.createProfileForJctRoad1Way(numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
    profileX3_O = profileMgr.createProfileForJctRoad1Way(numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  else
    profileX1_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, true, jct.edgeBlendMat)
    profileX2_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
    profileX3_O = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  end
  profileX1_O.layers = {}
  profileMgr.addCenterline(profileX1_O, true)
  profileMgr.addEdgeLines(profileX1_O, 0.2, 0.2, true, true, true)
  profileMgr.addLaneDivisionLines(profileX1_O, true)
  profileX2_O.layers = {}
  profileMgr.addCenterline(profileX2_O, true)
  profileMgr.addEdgeLines(profileX2_O, 0.2, 0.2, true, true, true)
  profileMgr.addLaneDivisionLines(profileX2_O, true)
  profileMgr.autoEdgeBlending(profileX2_O, true, true, jct.edgeBlendMat)
  profileX3_O.layers = {}
  profileMgr.addCenterline(profileX3_O, true)
  profileMgr.addEdgeLines(profileX3_O, 0.2, 0.2, true, true, true)
  profileMgr.addLaneDivisionLines(profileX3_O, true)
  profileMgr.autoEdgeBlending(profileX3_O, true, true, jct.edgeBlendMat)

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  -- Create the roads.
  local roadX1_O = roadMgr.createRoadFromProfile(profileX1_O)
  roadX1_O.displayName = im.ArrayChar(32, 'jct road 1')
  roadX1_O.isJctRoad = true
  profileX1_O.isEdgeBlendL = im.BoolPtr(false)
  profileX1_O.isEdgeBlendR = im.BoolPtr(false)
  profileX1_O.isStopDecalS = im.BoolPtr(false)
  profileX1_O.isStopDecalE = im.BoolPtr(false)
  profileX1_O.conditionEndStopS = im.BoolPtr(false)
  profileX1_O.conditionEndStopE = im.BoolPtr(false)
  profileX1_O.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_O
  roadMgr.map[roadX1_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(-capLength, 0, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(-capLength * 0.5, 0, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  if isOneWay and jct.isY1Outwards[0] then
    roadMgr.addNodeToRoad(rIdx, p3)
    roadMgr.addNodeToRoad(rIdx, p2)
    roadMgr.addNodeToRoad(rIdx, p1)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength * 1.5, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength * 2, 0, 0), rot) + cen)
    roadX1_O.nodes[1].isLocked = true
    roadX1_O.nodes[2].isLocked = true
    roadX1_O.nodes[3].isLocked = true
    roadX1_O.nodes[4].isLocked = true
    roadX1_O.nodes[5].isLocked = false
  else
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength * 2, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength * 1.5, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, p1)
    roadMgr.addNodeToRoad(rIdx, p2)
    roadMgr.addNodeToRoad(rIdx, p3)
    roadX1_O.nodes[1].isLocked = false
    roadX1_O.nodes[2].isLocked = true
    roadX1_O.nodes[3].isLocked = true
    roadX1_O.nodes[4].isLocked = true
    roadX1_O.nodes[5].isLocked = true
  end
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadX2_O = roadMgr.createRoadFromProfile(profileX2_O)
  roadX2_O.displayName = im.ArrayChar(32, 'jct road 2')
  roadX2_O.isJctRoad = true
  profileX2_O.isEdgeBlendL = im.BoolPtr(true)
  profileX2_O.isEdgeBlendR = im.BoolPtr(true)
  profileX2_O.isStopDecalS = im.BoolPtr(false)
  profileX2_O.isStopDecalE = im.BoolPtr(false)
  profileX2_O.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_O
  roadMgr.map[roadX2_O.name] = rIdx
  local p1 = util.rotateVecByQuaternion(vec3(capLength, 0, 0), rot) + cen
  local p3 = util.rotateVecByQuaternion(vec3(capLength * 0.5, 0, 0), rot) + cen
  local p2 = p1 + (p3 - p1) * 0.5
  if isOneWay and not jct.isY1Outwards[0] then
    roadMgr.addNodeToRoad(rIdx, p3)
    roadMgr.addNodeToRoad(rIdx, p2)
    roadMgr.addNodeToRoad(rIdx, p1)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength * 1.5, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength * 2, 0, 0), rot) + cen)
    roadX2_O.nodes[1].isLocked = true
    roadX2_O.nodes[2].isLocked = true
    roadX2_O.nodes[3].isLocked = true
    roadX2_O.nodes[4].isLocked = true
    roadX2_O.nodes[5].isLocked = false
    profileX2_O.conditionEndStopS = im.BoolPtr(false)
    profileX2_O.conditionEndStopE = im.BoolPtr(false)
  else
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength * 2, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength * 1.5, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, p1)
    roadMgr.addNodeToRoad(rIdx, p2)
    roadMgr.addNodeToRoad(rIdx, p3)
    roadX2_O.nodes[1].isLocked = false
    roadX2_O.nodes[2].isLocked = true
    roadX2_O.nodes[3].isLocked = true
    roadX2_O.nodes[4].isLocked = true
    roadX2_O.nodes[5].isLocked = true
    profileX2_O.conditionEndStopS = im.BoolPtr(false)
    profileX2_O.conditionEndStopE = im.BoolPtr(false)
  end
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  local roadX3_O = roadMgr.createRoadFromProfile(profileX3_O)
  roadX3_O.displayName = im.ArrayChar(32, 'jct road 3')
  roadX3_O.isJctRoad = true
  profileX3_O.isEdgeBlendL = im.BoolPtr(true)
  profileX3_O.isEdgeBlendR = im.BoolPtr(true)
  profileX3_O.isStopDecalS = im.BoolPtr(false)
  profileX3_O.isStopDecalE = im.BoolPtr(false)
  profileX3_O.conditionEndStopS = im.BoolPtr(false)
  profileX3_O.conditionEndStopE = im.BoolPtr(false)
  profileX3_O.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX3_O
  roadMgr.map[roadX3_O.name] = rIdx
  if isOneWay and jct.isY1Outwards[0] then
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength * 0.5, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength * 0.5, 0, 0), rot) + cen)
    roadX3_O.nodes[1].isLocked = true
    roadX3_O.nodes[2].isLocked = true
  else
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength * 0.5, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength * 0.5, 0, 0), rot) + cen)
    roadX3_O.nodes[1].isLocked = true
    roadX3_O.nodes[2].isLocked = true
  end
  roadMgr.computeRoadRenderDataSingle(#roadMgr.roads)

  -- Taper the height in section 3.
  if isOneWay then
    if jct.isY1Outwards[0] then
      roadX1_O.nodes[1].heightsL[-1] = im.FloatPtr(0.001)
      roadX1_O.nodes[1].heightsR[-1] = im.FloatPtr(0.001)
      roadX1_O.nodes[1].heightsL[numLanesX + 1] = im.FloatPtr(0.001)
      roadX1_O.nodes[1].heightsR[numLanesX + 1] = im.FloatPtr(0.001)
    else
      roadX1_O.nodes[5].heightsL[-1] = im.FloatPtr(0.001)
      roadX1_O.nodes[5].heightsR[-1] = im.FloatPtr(0.001)
      roadX1_O.nodes[5].heightsL[numLanesX + 1] = im.FloatPtr(0.001)
      roadX1_O.nodes[5].heightsR[numLanesX + 1] = im.FloatPtr(0.001)
    end
  else
    roadX1_O.nodes[5].heightsL[-numLanesX - 1] = im.FloatPtr(0.001)
    roadX1_O.nodes[5].heightsR[-numLanesX - 1] = im.FloatPtr(0.001)
    roadX1_O.nodes[5].heightsL[numLanesX + 1] = im.FloatPtr(0.001)
    roadX1_O.nodes[5].heightsR[numLanesX + 1] = im.FloatPtr(0.001)
  end

  jct.roads = { roadX1_O.name, roadX2_O.name, roadX3_O.name }
end

-- Updates an urban merge junction.
local function updateUrbanMerge(jIdx, jct, isMesh)
  -- Create the road profiles.
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local isSidewalk = jct.isSidewalk[0]
  local sidewalkWidth, sidewalkHeight = jct.sidewalkWidth[0], jct.sidewalkHeight[0]
  local profileS1 = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileS3 = profileMgr.createProfileForJctRoad(numLanesX + 1, numLanesX + 1, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  local profileS2 = profileMgr.createProfileForJctRoad(numLanesX + 1, numLanesX + 1, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  profileS1.layers = {}
  profileMgr.addCenterline(profileS1, true)
  profileMgr.addEdgeLines(profileS1, 0.1, 0.1, true, true, true)
  profileMgr.addLaneDivisionLines(profileS1, true)
  profileS2.layers = {}
  profileMgr.addCenterline(profileS2, true)
  profileMgr.addEdgeLines(profileS2, 0.1, 0.1, true, true, true)
  profileMgr.addLaneDivisionLines(profileS2, false)
  profileS3.layers = {}
  profileMgr.addCenterline(profileS3, true)
  profileMgr.addEdgeLines(profileS3, 0.1, 0.1, true, true, true)
  profileMgr.addLaneDivisionLines(profileS3, true)
  if not isSidewalk then
    profileMgr.autoEdgeBlending(profileS1, true, true, jct.edgeBlendMat)
    profileMgr.autoEdgeBlending(profileS2, true, true, jct.edgeBlendMat)
    profileMgr.autoEdgeBlending(profileS3, true, true, jct.edgeBlendMat)
  end

  -- Create the two defining inner roads.
  local boxX = jct.s2Length[0]
  local boxXHalf = boxX * 0.5
  local capLength = jct.capLength[0]

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  -- Make the roads for the two cap sections (S1 and S3).
  local roadS1 = roadMgr.createRoadFromProfile(profileS1)
  roadS1.displayName = im.ArrayChar(32, 'jct - section1')
  roadS1.isJctRoad = true
  profileS1.isEdgeBlendL = im.BoolPtr(true)
  profileS1.isEdgeBlendR = im.BoolPtr(true)
  profileS1.conditionEdgesL = im.BoolPtr(false)
  profileS1.conditionEdgesR = im.BoolPtr(false)
  profileS1.conditionEndStopS = im.BoolPtr(true)
  profileS1.conditionEndStopE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS1
  roadMgr.map[roadS1.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadS1.nodes[1].isLocked = false
  roadS1.nodes[2].isLocked = true
  roadS1.nodes[3].isLocked = true
  roadS1.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  local roadS3 = roadMgr.createRoadFromProfile(profileS3)
  roadS3.displayName = im.ArrayChar(32, 'jct - section3')
  roadS3.isJctRoad = true
  profileS3.isEdgeBlendL = im.BoolPtr(true)
  profileS3.isEdgeBlendR = im.BoolPtr(true)
  profileS3.conditionEdgesL = im.BoolPtr(false)
  profileS3.conditionEdgesR = im.BoolPtr(false)
  profileS3.conditionEndStopS = im.BoolPtr(true)
  profileS3.conditionEndStopE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS3
  roadMgr.map[roadS3.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadS3.nodes[1].isLocked = false
  roadS3.nodes[2].isLocked = true
  roadS3.nodes[3].isLocked = true
  roadS3.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Make the road for the tapered section (S2).
  local roadS2 = roadMgr.createRoadFromProfile(profileS2)
  roadS2.displayName = im.ArrayChar(32, 'jct - section2')
  roadS2.granFactor = im.IntPtr(2)
  roadS2.isJctRoad = true
  profileS2.isEdgeBlendL = im.BoolPtr(true)
  profileS2.isEdgeBlendR = im.BoolPtr(true)
  profileS2.conditionEdgesL = im.BoolPtr(false)
  profileS2.conditionEdgesR = im.BoolPtr(false)
  profileS2.conditionEndStopS = im.BoolPtr(false)
  profileS2.conditionEndStopE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS2
  roadMgr.map[roadS2.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf + boxX / 3, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf - boxX / 3, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadS2.nodes[1].isLocked = true
  roadS2.nodes[2].isLocked = true
  roadS2.nodes[3].isLocked = true
  roadS2.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Apply the lane taper to section 2.
  local tapIdx = numLanesX + 1
  roadS2.nodes[1].widths[-tapIdx] = im.FloatPtr(0.0)
  roadS2.nodes[1].widths[tapIdx] = im.FloatPtr(0.0)
  roadS2.nodes[2].widths[-tapIdx] = im.FloatPtr(jct.sepWidthI[0])
  roadS2.nodes[2].widths[tapIdx] = im.FloatPtr(jct.sepWidthI[0])
  roadS2.nodes[3].widths[-tapIdx] = im.FloatPtr(jct.sepWidthO[0])
  roadS2.nodes[3].widths[tapIdx] = im.FloatPtr(jct.sepWidthO[0])
  roadS2.nodes[4].widths[-tapIdx] = im.FloatPtr(laneWidthX)
  roadS2.nodes[4].widths[tapIdx] = im.FloatPtr(laneWidthX)

  -- Recompute the road map after the road changes.
  roadMgr.recomputeMap()

  jct.roads = { roadS1.name, roadS3.name, roadS2.name }
end

-- Updates a highway merge junction.
local function updateHighwayMerge(jIdx, jct, isMesh)
  -- Create the road profiles.
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local cResWidth, hardWidth = jct.cResWidth[0], jct.hardWidth[0]
  local profileS1 = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, cResWidth, hardWidth, jct.edgeBlendMat)
  local profileS3 = profileMgr.createProfileForJctRoadHwyCap(numLanesX + 1, laneWidthX, cResWidth, hardWidth, jct.edgeBlendMat)
  local profileS2 = profileMgr.createProfileForJctRoadHwyCap(numLanesX + 1, laneWidthX, cResWidth, hardWidth, jct.edgeBlendMat)
  profileS1.layers = {}
  profileMgr.addCenterline(profileS1, true)
  profileMgr.addLaneDivisionLines(profileS1, true)
  profileS2.layers = {}
  profileMgr.addCenterline(profileS2, true)
  profileMgr.addLaneDivisionLines(profileS2, true)
  profileS3.layers = {}
  profileMgr.addCenterline(profileS3, true)
  profileMgr.addLaneDivisionLines(profileS3, true)
  profileMgr.autoEdgeBlending(profileS1, true, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileS2, true, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileS3, true, true, jct.edgeBlendMat)

  -- Create the two defining inner roads.
  local boxX = jct.s2Length[0]
  local boxXHalf = boxX * 0.5
  local capLength = jct.capLength[0]

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  -- Make the roads for the two cap sections (S1 and S4).
  local roadS1 = roadMgr.createRoadFromProfile(profileS1)
  roadS1.displayName = im.ArrayChar(32, 'jct - section1')
  roadS1.isJctRoad = true
  profileS1.isEdgeBlendL = im.BoolPtr(true)
  profileS1.isEdgeBlendR = im.BoolPtr(true)
  profileS1.conditionEdgesL = im.BoolPtr(false)
  profileS1.conditionEdgesR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS1
  roadMgr.map[roadS1.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadS1.nodes[1].isLocked = false
  roadS1.nodes[2].isLocked = true
  roadS1.nodes[3].isLocked = true
  roadS1.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  local roadS3 = roadMgr.createRoadFromProfile(profileS3)
  roadS3.displayName = im.ArrayChar(32, 'jct - section3')
  roadS3.isJctRoad = true
  profileS3.isEdgeBlendL = im.BoolPtr(true)
  profileS3.isEdgeBlendR = im.BoolPtr(true)
  profileS3.conditionEdgesL = im.BoolPtr(false)
  profileS3.conditionEdgesR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS3
  roadMgr.map[roadS3.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadS3.nodes[1].isLocked = false
  roadS3.nodes[2].isLocked = true
  roadS3.nodes[3].isLocked = true
  roadS3.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Make the road for the tapered section (S2).
  local roadS2 = roadMgr.createRoadFromProfile(profileS2)
  roadS2.displayName = im.ArrayChar(32, 'jct - section2')
  roadS2.granFactor = im.IntPtr(2)
  roadS2.isJctRoad = true
  profileS2.isEdgeBlendL = im.BoolPtr(true)
  profileS2.isEdgeBlendR = im.BoolPtr(true)
  profileS2.conditionEdgesL = im.BoolPtr(false)
  profileS2.conditionEdgesR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS2
  roadMgr.map[roadS2.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf + boxX / 3, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf - boxX / 3, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadS2.nodes[1].isLocked = true
  roadS2.nodes[2].isLocked = true
  roadS2.nodes[3].isLocked = true
  roadS2.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Apply the lane taper to section 2.
  local tapIdx = numLanesX + 2
  roadS2.nodes[1].widths[-tapIdx] = im.FloatPtr(0.0)
  roadS2.nodes[1].widths[tapIdx] = im.FloatPtr(0.0)
  roadS2.nodes[2].widths[-tapIdx] = im.FloatPtr(jct.sepWidthI[0])
  roadS2.nodes[2].widths[tapIdx] = im.FloatPtr(jct.sepWidthI[0])
  roadS2.nodes[3].widths[-tapIdx] = im.FloatPtr(jct.sepWidthO[0])
  roadS2.nodes[3].widths[tapIdx] = im.FloatPtr(jct.sepWidthO[0])
  roadS2.nodes[4].widths[-tapIdx] = im.FloatPtr(laneWidthX)
  roadS2.nodes[4].widths[tapIdx] = im.FloatPtr(laneWidthX)

  -- Recompute the road map after the road changes.
  roadMgr.recomputeMap()

  jct.roads = { roadS1.name, roadS3.name, roadS2.name }

  -- Add the edge lines.
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S1_1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-numLanesX - 1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S1_2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(numLanesX + 1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S1_3'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S1_4'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }

  profileS2.layers[#profileS2.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S2_1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-numLanesX - 2), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS2.layers[#profileS2.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S2_2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(numLanesX + 2), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS2.layers[#profileS2.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S2_3'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS2.layers[#profileS2.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S2_4'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }

  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-numLanesX - 2), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(numLanesX + 2), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_3'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_4'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }

  -- Crash barriers.
  if jct.isBarriersI[0] then
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end
  if jct.isBarriersO[0] then
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-numLanesX - 2), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(numLanesX + 2), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-numLanesX - 3), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(numLanesX + 3), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-numLanesX - 3), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(numLanesX + 3), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end

  -- Add road signs (poles).
  if jct.isSigns[0] then
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Merge Sign'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultLaneMergeSign,
      rot = im.IntPtr(1),
      pos = im.FloatPtr(0.33), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_junction_merge_left.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Merge Sign R'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultLaneMergeSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.1), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-0.2),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_junction_merge_left.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end

  if jct.isCrossings[0] then
    overlayUtils.addHighwayMergeOverlays(jct)
  end
end

-- Updates a highway<->urban transition junction.
local function updateHighwayUrbanTransition(jIdx, jct, isMesh)
  -- Create the road profiles.
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local cResWidth, hardWidth = jct.cResWidth[0], jct.hardWidth[0]
  local isOneWay = jct.isYOneWay[0]
  local profileS1, profileS2, profileS3 = nil, nil, nil
  if isOneWay then
    profileS1 = profileMgr.createProfileForJctRoadHwyCap1W(numLanesX, laneWidthX, hardWidth, jct.edgeBlendMat)
    profileS3 = profileMgr.createProfileForJctRoadHwyCapUrban(numLanesX, laneWidthX, jct.isSidewalk[0], jct.sidewalkWidth[0], jct.sidewalkHeight[0], true, jct.edgeBlendMat)
    profileS2 = profileMgr.createProfileForJctRoadHwyUrbanTrans(numLanesX, laneWidthX, cResWidth, hardWidth, true, jct.edgeBlendMat)
  else
    profileS1 = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, cResWidth, hardWidth, jct.edgeBlendMat)
    profileS3 = profileMgr.createProfileForJctRoadHwyCapUrban(numLanesX, laneWidthX, jct.isSidewalk[0], jct.sidewalkWidth[0], jct.sidewalkHeight[0], false, jct.edgeBlendMat)
    profileS2 = profileMgr.createProfileForJctRoadHwyUrbanTrans(numLanesX, laneWidthX, cResWidth, hardWidth, false, jct.edgeBlendMat)
  end
  profileS1.layers = {}
  profileMgr.addCenterline(profileS1, true)
  profileMgr.addEdgeLines(profileS1, 0.2, 0.2, true, true, true)
  profileMgr.addLaneDivisionLines(profileS1, true)
  profileS2.layers = {}
  profileMgr.addCenterline(profileS2, true)
  profileMgr.addEdgeLines(profileS2, 0.2, 0.2, true, true, true)
  profileMgr.addLaneDivisionLines(profileS2, true)
  profileS3.layers = {}
  profileMgr.addCenterline(profileS3, true)
  profileMgr.addEdgeLines(profileS3, 0.2, 0.2, true, true, true)
  profileMgr.addLaneDivisionLines(profileS3, true)
  profileMgr.autoEdgeBlending(profileS1, true, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileS2, true, true, jct.edgeBlendMat)

  -- Create the two defining inner roads.
  local boxX = jct.s2Length[0]
  local boxXHalf = boxX * 0.5
  local capLength = jct.capLength[0]

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  -- Make the roads for the two cap sections (S1 and S4).
  local roadS1 = roadMgr.createRoadFromProfile(profileS1)
  roadS1.displayName = im.ArrayChar(32, 'jct - section1')
  roadS1.isJctRoad = true
  profileS1.isEdgeBlendL = im.BoolPtr(true)
  profileS1.isEdgeBlendR = im.BoolPtr(true)
  profileS1.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS1
  roadMgr.map[roadS1.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadS1.nodes[1].isLocked = false
  roadS1.nodes[2].isLocked = true
  roadS1.nodes[3].isLocked = true
  roadS1.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  local roadS3 = roadMgr.createRoadFromProfile(profileS3)
  roadS3.displayName = im.ArrayChar(32, 'jct - section3')
  roadS3.isJctRoad = true
  if jct.isSidewalk[0] then
    profileS3.isEdgeBlendL = im.BoolPtr(false)
    profileS3.isEdgeBlendR = im.BoolPtr(false)
  else
    profileS3.isEdgeBlendL = im.BoolPtr(true)
    profileS3.isEdgeBlendR = im.BoolPtr(true)
  end
  profileS3.conditionCenterline = im.BoolPtr(true)
  profileS3.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS3
  roadMgr.map[roadS3.name] = rIdx
  if isOneWay then
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength / 3.0, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength * (2.0 / 3.0), 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength, 0, 0), rot) + cen)
  else
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength * (2.0 / 3.0), 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength / 3.0, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  end
  roadS3.nodes[1].isLocked = false
  roadS3.nodes[2].isLocked = true
  roadS3.nodes[3].isLocked = true
  roadS3.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Make the road for the tapered section (S2).
  local roadS2 = roadMgr.createRoadFromProfile(profileS2)
  roadS2.displayName = im.ArrayChar(32, 'jct - section2')
  roadS2.granFactor = im.IntPtr(2)
  roadS2.isJctRoad = true
  profileS2.isEdgeBlendL = im.BoolPtr(true)
  profileS2.isEdgeBlendR = im.BoolPtr(true)
  profileS2.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS2
  roadMgr.map[roadS2.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf + (boxX / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf - (boxX / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadS2.nodes[1].isLocked = true
  roadS2.nodes[2].isLocked = true
  roadS2.nodes[3].isLocked = true
  roadS2.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Apply the lane taper to section 2 (central reservation).
  if not isOneWay then
    local tapIdx = 1
    roadS2.nodes[1].widths[-tapIdx] = im.FloatPtr(cResWidth * 0.5)
    roadS2.nodes[1].widths[tapIdx] = im.FloatPtr(cResWidth * 0.5)
    roadS2.nodes[2].widths[-tapIdx] = im.FloatPtr(jct.sepWidthI[0])
    roadS2.nodes[2].widths[tapIdx] = im.FloatPtr(jct.sepWidthI[0])
    roadS2.nodes[3].widths[-tapIdx] = im.FloatPtr(jct.sepWidthO[0])
    roadS2.nodes[3].widths[tapIdx] = im.FloatPtr(jct.sepWidthO[0])
    roadS2.nodes[4].widths[-tapIdx] = im.FloatPtr(0.0)
    roadS2.nodes[4].widths[tapIdx] = im.FloatPtr(0.0)
  end

  -- Taper the hard shoulder (taper amount depends on whether sidewalks are on the other side or not).
  local tapIdx = numLanesX + 2
  local tapIdx2 = tapIdx
  if isOneWay then
    tapIdx2 = tapIdx - 1
  end
  if jct.isSidewalk[0] then
    local dW = hardWidth - jct.sidewalkWidth[0]
    roadS2.nodes[1].widths[tapIdx2] = im.FloatPtr(hardWidth)
    roadS2.nodes[2].widths[tapIdx2] = im.FloatPtr(hardWidth - (dW * 1.0 / 3.0))
    roadS2.nodes[3].widths[tapIdx2] = im.FloatPtr(hardWidth - (dW * 2.0 / 3.0))
    roadS2.nodes[4].widths[tapIdx2] = im.FloatPtr(jct.sidewalkWidth[0])
    if not isOneWay then
      roadS2.nodes[1].widths[-tapIdx] = im.FloatPtr(hardWidth)
      roadS2.nodes[2].widths[-tapIdx] = im.FloatPtr(hardWidth - (dW * 1.0 / 3.0))
      roadS2.nodes[3].widths[-tapIdx] = im.FloatPtr(hardWidth - (dW * 2.0 / 3.0))
      roadS2.nodes[4].widths[-tapIdx] = im.FloatPtr(jct.sidewalkWidth[0])
    end
  else
    roadS2.nodes[1].widths[tapIdx2] = im.FloatPtr(hardWidth)
    roadS2.nodes[2].widths[tapIdx2] = im.FloatPtr(hardWidth * 2.0 / 3.0)
    roadS2.nodes[3].widths[tapIdx2] = im.FloatPtr(hardWidth / 3.0)
    roadS2.nodes[4].widths[tapIdx2] = im.FloatPtr(0.0)
    if not isOneWay then
      roadS2.nodes[1].widths[-tapIdx] = im.FloatPtr(hardWidth)
      roadS2.nodes[2].widths[-tapIdx] = im.FloatPtr(hardWidth * 2.0 / 3.0)
      roadS2.nodes[3].widths[-tapIdx] = im.FloatPtr(hardWidth / 3.0)
      roadS2.nodes[4].widths[-tapIdx] = im.FloatPtr(0.0)
    end
  end

  -- If sidewalks are being used, taper the height in section 3.
  if jct.isSidewalk[0] then
    if isOneWay then
      roadS3.nodes[1].heightsL[-1] = im.FloatPtr(0.001)
      roadS3.nodes[1].heightsR[-1] = im.FloatPtr(0.001)
      roadS3.nodes[1].heightsL[numLanesX + 1] = im.FloatPtr(0.001)
      roadS3.nodes[1].heightsR[numLanesX + 1] = im.FloatPtr(0.001)
    else
      roadS3.nodes[4].heightsL[-numLanesX - 1] = im.FloatPtr(0.001)
      roadS3.nodes[4].heightsR[-numLanesX - 1] = im.FloatPtr(0.001)
      roadS3.nodes[4].heightsL[numLanesX + 1] = im.FloatPtr(0.001)
      roadS3.nodes[4].heightsR[numLanesX + 1] = im.FloatPtr(0.001)
    end
  end

  -- Recompute the road map after the road changes.
  roadMgr.recomputeMap()

  jct.roads = { roadS1.name, roadS3.name, roadS2.name }

  -- Add the separator layer in the central reservation of section 2.
  if not isOneWay then
    local newLayer = {
      name = im.ArrayChar(32, 'Separator'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(true),
      type = im.IntPtr(0),
      laneMin = im.IntPtr(-1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(1.0),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultSeperatorMaterial,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    table.insert(profileS2.layers, 1, newLayer)
  end

-- Crash barriers.
if not isOneWay and jct.isBarriersI[0] then
  local leftIdx, isLeft = -1, true
  if isOneWay then
    leftIdx, isLeft = 1, false
  end
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Barrier L2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(4),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(leftIdx), isLeft = im.BoolPtr(isLeft), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultCrashBarrierPath,
    rot = im.IntPtr(2),
    pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(-0.15),
    latOffset = im.FloatPtr(0.15),
    spacing = im.FloatPtr(-0.17),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = 'italy_guardrails_basic.dae',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Barrier R2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(4),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultCrashBarrierPath,
    rot = im.IntPtr(0),
    pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(-0.15),
    latOffset = im.FloatPtr(-0.15),
    spacing = im.FloatPtr(-0.17),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = 'italy_guardrails_basic.dae',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
end
if jct.isBarriersO[0] then
  local lIdx, rIdx = profileMgr.getMinMaxLaneKeys(profileS1)
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Barrier L1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(4),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(lIdx), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultCrashBarrierPath,
    rot = im.IntPtr(0),
    pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(-0.15),
    latOffset = im.FloatPtr(-0.15),
    spacing = im.FloatPtr(-0.17),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = 'italy_guardrails_basic.dae',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Barrier R1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(4),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(rIdx), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultCrashBarrierPath,
    rot = im.IntPtr(2),
    pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(-0.15),
    latOffset = im.FloatPtr(0.15),
    spacing = im.FloatPtr(-0.17),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = 'italy_guardrails_basic.dae',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS2.layers[#profileS2.layers + 1] = {
    name = im.ArrayChar(32, 'Barrier L1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(4),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(lIdx), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultCrashBarrierPath,
    rot = im.IntPtr(0),
    pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(-0.15),
    latOffset = im.FloatPtr(-0.15),
    spacing = im.FloatPtr(-0.17),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = 'italy_guardrails_basic.dae',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS2.layers[#profileS2.layers + 1] = {
    name = im.ArrayChar(32, 'Barrier R1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(4),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(rIdx), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultCrashBarrierPath,
    rot = im.IntPtr(2),
    pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(-0.15),
    latOffset = im.FloatPtr(0.15),
    spacing = im.FloatPtr(-0.17),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = 'italy_guardrails_basic.dae',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end

  if not isOneWay and jct.isSigns[0] then
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Keep Right Sign F'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultKeepRightSign,
      rot = im.IntPtr(1),
      pos = im.FloatPtr(0.95), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_keep_right.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Keep Right Sign B'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultKeepRightSign,
      rot = im.IntPtr(1),
      pos = im.FloatPtr(0.05), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_keep_right.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Road Narrows Sign F'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultRoadNarrowsSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.89), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_warn_road_narrows.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Road Narrows Sign B'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultRoadNarrowsSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_warn_road_narrows.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end

  if jct.isCrossings[0] then
    overlayUtils.addHighwayTransOverlays(jct)
  end
end

-- Updates a highway separator junction.
local function updateHighwaySeparator(jIdx, jct, isMesh)
  -- Create the road profiles.
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local cResWidth, hardWidth = jct.cResWidth[0], jct.hardWidth[0]
  local profileS1 = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, cResWidth, hardWidth, jct.edgeBlendMat)
  local profileS2A = profileMgr.createProfileForJctRoadHwyCap1W(numLanesX, laneWidthX, hardWidth, jct.edgeBlendMat)
  local profileS2B = profileMgr.createProfileForJctRoadHwyCap1W(numLanesX, laneWidthX, hardWidth, jct.edgeBlendMat)
  profileS1.layers = {}
  profileMgr.addCenterline(profileS1, true)
  profileMgr.addEdgeLines(profileS1, 0.2, 0.2, true, true, true)
  profileMgr.addLaneDivisionLines(profileS1, true)
  profileS2A.layers = {}
  profileMgr.addCenterline(profileS2A, true)
  profileMgr.addEdgeLines(profileS2A, 0.2, 0.2, true, true, true)
  profileMgr.addLaneDivisionLines(profileS2A, true)
  profileS2B.layers = {}
  profileMgr.addCenterline(profileS2B, true)
  profileMgr.addEdgeLines(profileS2B, 0.2, 0.2, true, true, true)
  profileMgr.addLaneDivisionLines(profileS2B, true)
  profileMgr.autoEdgeBlending(profileS1, true, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileS2A, true, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileS2B, true, true, jct.edgeBlendMat)

  local capLength = jct.capLength[0]

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  -- Make the roads for the two-way section.
  local roadS1 = roadMgr.createRoadFromProfile(profileS1)
  roadS1.displayName = im.ArrayChar(32, 'jct - section1')
  roadS1.isJctRoad = true
  profileS1.isEdgeBlendL = im.BoolPtr(true)
  profileS1.isEdgeBlendR = im.BoolPtr(true)
  profileS1.conditionEdgesL = im.BoolPtr(true)
  profileS1.conditionEdgesR = im.BoolPtr(true)
  profileS1.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS1
  roadMgr.map[roadS1.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, 0, 0), rot) + cen)
  roadS1.nodes[1].isLocked = false
  roadS1.nodes[2].isLocked = true
  roadS1.nodes[3].isLocked = true
  roadS1.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  local rLast = roadS1.renderData[#roadS1.renderData]
  local p1A, p1B = rLast[1][2], rLast[-1][1]
  local tgt = roadS1.nodes[#roadS1.nodes].p - roadS1.nodes[1].p
  tgt:normalize()

  local roadS2A = roadMgr.createRoadFromProfile(profileS2A)
  roadS2A.displayName = im.ArrayChar(32, 'jct - section2A')
  roadS2A.isJctRoad = true
  profileS2A.isEdgeBlendL = im.BoolPtr(true)
  profileS2A.isEdgeBlendR = im.BoolPtr(true)
  profileS2A.conditionEdgesL = im.BoolPtr(true)
  profileS2A.conditionEdgesR = im.BoolPtr(true)
  profileS2A.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS2A
  roadMgr.map[roadS2A.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, p1A)
  roadMgr.addNodeToRoad(rIdx, p1A + tgt * capLength / 3.0)
  roadMgr.addNodeToRoad(rIdx, p1A + tgt * capLength * (2.0 / 3.0))
  roadMgr.addNodeToRoad(rIdx, p1A + tgt * capLength)
  roadS2A.nodes[1].isLocked = true
  roadS2A.nodes[2].isLocked = true
  roadS2A.nodes[3].isLocked = true
  roadS2A.nodes[4].isLocked = false
  roadMgr.computeRoadRenderDataSingle(rIdx)

  local roadS2B = roadMgr.createRoadFromProfile(profileS2B)
  roadS2B.displayName = im.ArrayChar(32, 'jct - section2B')
  roadS2B.isJctRoad = true
  profileS2B.isEdgeBlendL = im.BoolPtr(true)
  profileS2B.isEdgeBlendR = im.BoolPtr(true)
  profileS2B.conditionEdgesL = im.BoolPtr(true)
  profileS2B.conditionEdgesR = im.BoolPtr(true)
  profileS2B.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS2B
  roadMgr.map[roadS2B.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, p1B + tgt * capLength)
  roadMgr.addNodeToRoad(rIdx, p1B + tgt * capLength * (2.0 / 3.0))
  roadMgr.addNodeToRoad(rIdx, p1B + tgt * capLength / 3.0)
  roadMgr.addNodeToRoad(rIdx, p1B)
  roadS2B.nodes[1].isLocked = false
  roadS2B.nodes[2].isLocked = true
  roadS2B.nodes[3].isLocked = true
  roadS2B.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Recompute the road map after the road changes.
  roadMgr.recomputeMap()

  jct.roads = { roadS1.name, roadS2A.name, roadS2B.name }

  -- Crash barriers.
  if jct.isBarriersI[0] then
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2A.layers[#profileS2A.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2B.layers[#profileS2B.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end
  if jct.isBarriersO[0] then
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-numLanesX - 2), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(numLanesX + 2), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2A.layers[#profileS2A.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(numLanesX + 1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2B.layers[#profileS2B.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(numLanesX + 1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end
end

-- Updates an shoulder fade junction.
local function updateShoulderFade(jIdx, jct, isMesh)
  -- Create the road profiles.
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local isSidewalk = jct.isSidewalk[0]
  local sidewalkWidth, sidewalkHeight = jct.sidewalkWidth[0], jct.sidewalkHeight[0]
  local hardWidth = jct.hardWidth[0]
  local capLength = jct.capLength[0]
  local profileS1 = profileMgr.createProfileForJctRoadHwyCap1W(numLanesX, laneWidthX, hardWidth, jct.edgeBlendMat)
  local profileS2 = profileMgr.createProfileForJctRoad1Way(numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, isSidewalk, jct.edgeBlendMat)
  profileS1.layers = {}
  profileMgr.addCenterline(profileS1, true)
  profileMgr.addEdgeLines(profileS1, 0.1, 0.1, true, true, true)
  profileMgr.addLaneDivisionLines(profileS1, true)
  profileS2.layers = {}
  profileMgr.addCenterline(profileS2, true)
  profileMgr.addEdgeLines(profileS2, 0.1, 0.1, true, true, true)
  profileMgr.addLaneDivisionLines(profileS2, false)
  profileMgr.autoEdgeBlending(profileS1, true, true, jct.edgeBlendMat)
  if not isSidewalk then
    profileMgr.autoEdgeBlending(profileS2, true, true, jct.edgeBlendMat)
  end

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  -- Make the roads for the two cap sections (S1 and S3).
  local roadS1 = roadMgr.createRoadFromProfile(profileS1)
  roadS1.displayName = im.ArrayChar(32, 'jct - section1')
  roadS1.isJctRoad = true
  profileS1.isEdgeBlendL = im.BoolPtr(true)
  profileS1.isEdgeBlendR = im.BoolPtr(true)
  profileS1.conditionEdgesL = im.BoolPtr(false)
  profileS1.conditionEdgesR = im.BoolPtr(false)
  profileS1.conditionEndStopS = im.BoolPtr(true)
  profileS1.conditionEndStopE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS1
  roadMgr.map[roadS1.name] = rIdx
  if jct.isY1Outwards[0] then
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength * (2.0 / 3.0), 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength / 3.0, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, 0, 0), rot) + cen)
    roadS1.nodes[1].isLocked = false
    roadS1.nodes[2].isLocked = true
    roadS1.nodes[3].isLocked = true
    roadS1.nodes[4].isLocked = true
  else
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength / 3.0, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength * (2.0 / 3.0), 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength, 0, 0), rot) + cen)
    roadS1.nodes[1].isLocked = true
    roadS1.nodes[2].isLocked = true
    roadS1.nodes[3].isLocked = true
    roadS1.nodes[4].isLocked = false
  end
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Make the road for the tapered section (S2).
  local roadS2 = roadMgr.createRoadFromProfile(profileS2)
  roadS2.displayName = im.ArrayChar(32, 'jct - section2')
  roadS2.granFactor = im.IntPtr(2)
  roadS2.isJctRoad = true
  profileS2.isEdgeBlendL = im.BoolPtr(true)
  profileS2.isEdgeBlendR = im.BoolPtr(true)
  profileS2.conditionEdgesL = im.BoolPtr(false)
  profileS2.conditionEdgesR = im.BoolPtr(false)
  profileS2.conditionEndStopS = im.BoolPtr(false)
  profileS2.conditionEndStopE = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS2
  roadMgr.map[roadS2.name] = rIdx
  if jct.isY1Outwards[0] then
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength / 3.0, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength * (2.0 / 3.0), 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength, 0, 0), rot) + cen)
    roadS2.nodes[1].isLocked = true
    roadS2.nodes[2].isLocked = true
    roadS2.nodes[3].isLocked = true
    roadS2.nodes[4].isLocked = false
  else
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength * (2.0 / 3.0), 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(capLength / 3.0, 0, 0), rot) + cen)
    roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, 0, 0), rot) + cen)
    roadS2.nodes[1].isLocked = false
    roadS2.nodes[2].isLocked = true
    roadS2.nodes[3].isLocked = true
    roadS2.nodes[4].isLocked = true
  end
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Apply the shoulder taper to section 1.
  local tapIdx = numLanesX + 1
  if jct.isY1Outwards[0] then
    if isSidewalk then
      roadS1.nodes[1].widths[tapIdx] = im.FloatPtr(hardWidth)
      roadS1.nodes[2].widths[tapIdx] = im.FloatPtr(sidewalkWidth + (hardWidth - sidewalkWidth) * (2.0 / 3.0))
      roadS1.nodes[3].widths[tapIdx] = im.FloatPtr(sidewalkWidth + (hardWidth - sidewalkWidth) / 3.0)
      roadS1.nodes[4].widths[tapIdx] = im.FloatPtr(sidewalkWidth)
    else
      roadS1.nodes[1].widths[tapIdx] = im.FloatPtr(hardWidth)
      roadS1.nodes[2].widths[tapIdx] = im.FloatPtr(hardWidth * (2.0 / 3.0))
      roadS1.nodes[3].widths[tapIdx] = im.FloatPtr(hardWidth / 3.0)
      roadS1.nodes[4].widths[tapIdx] = im.FloatPtr(0.0)
    end
  else
    if isSidewalk then
      roadS1.nodes[4].widths[tapIdx] = im.FloatPtr(hardWidth)
      roadS1.nodes[3].widths[tapIdx] = im.FloatPtr(sidewalkWidth + (hardWidth - sidewalkWidth) * (2.0 / 3.0))
      roadS1.nodes[2].widths[tapIdx] = im.FloatPtr(sidewalkWidth + (hardWidth - sidewalkWidth) / 3.0)
      roadS1.nodes[1].widths[tapIdx] = im.FloatPtr(sidewalkWidth)
    else
      roadS1.nodes[4].widths[tapIdx] = im.FloatPtr(hardWidth)
      roadS1.nodes[3].widths[tapIdx] = im.FloatPtr(hardWidth * (2.0 / 3.0))
      roadS1.nodes[2].widths[tapIdx] = im.FloatPtr(hardWidth / 3.0)
      roadS1.nodes[1].widths[tapIdx] = im.FloatPtr(0.0)
    end
  end

  -- Lower the sidewalk heights towards the junction center.
  if isSidewalk then
    if not jct.isY1Outwards[0] then
      roadS2.nodes[4].heightsL[-1] = im.FloatPtr(0.01)
      roadS2.nodes[4].heightsR[-1] = im.FloatPtr(0.01)
      roadS2.nodes[4].heightsL[numLanesX + 1] = im.FloatPtr(0.01)
      roadS2.nodes[4].heightsR[numLanesX + 1] = im.FloatPtr(0.01)
    else
      roadS2.nodes[1].heightsL[-1] = im.FloatPtr(0.01)
      roadS2.nodes[1].heightsR[-1] = im.FloatPtr(0.01)
      roadS2.nodes[1].heightsL[numLanesX + 1] = im.FloatPtr(0.01)
      roadS2.nodes[1].heightsR[numLanesX + 1] = im.FloatPtr(0.01)
    end
  end

  -- Recompute the road map after the road changes.
  roadMgr.recomputeMap()

  jct.roads = { roadS1.name, roadS2.name }
end

-- Updates an urban separator junction.
local function updateUrbanSeparator(jIdx, jct, isMesh)
  -- Create the road profiles.
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local capLength = jct.capLength[0]
  local profileS1 = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  local profileS2A = profileMgr.createProfileForJctRoad1Way(numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  local profileS2B = profileMgr.createProfileForJctRoad1Way(numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false, jct.edgeBlendMat)
  profileS1.layers = {}
  profileMgr.addCenterline(profileS1, true)
  profileMgr.addEdgeLines(profileS1, 0.15, 0.15, true, true, true)
  profileMgr.addLaneDivisionLines(profileS1, true)
  profileS2A.layers = {}
  profileMgr.addCenterline(profileS2A, true)
  profileMgr.addEdgeLines(profileS2A, 0.15, 0.15, true, true, true)
  profileMgr.addLaneDivisionLines(profileS2A, true)
  profileS2B.layers = {}
  profileMgr.addCenterline(profileS2B, true)
  profileMgr.addEdgeLines(profileS2B, 0.15, 0.15, true, true, true)
  profileMgr.addLaneDivisionLines(profileS2B, true)
  profileMgr.autoEdgeBlending(profileS1, true, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileS2A, false, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileS2B, false, true, jct.edgeBlendMat)

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  -- Make the roads for the two-way section.
  local roadS1 = roadMgr.createRoadFromProfile(profileS1)
  roadS1.displayName = im.ArrayChar(32, 'jct - section1')
  roadS1.isJctRoad = true
  profileS1.isEdgeBlendL = im.BoolPtr(true)
  profileS1.isEdgeBlendR = im.BoolPtr(true)
  profileS1.conditionEdgesL = im.BoolPtr(true)
  profileS1.conditionEdgesR = im.BoolPtr(true)
  profileS1.conditionEndStopS = im.BoolPtr(true)
  profileS1.conditionEndStopE = im.BoolPtr(false)
  profileS1.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS1
  roadMgr.map[roadS1.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(0, 0, 0), rot) + cen)
  roadS1.nodes[1].isLocked = false
  roadS1.nodes[2].isLocked = true
  roadS1.nodes[3].isLocked = true
  roadS1.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  local rLast = roadS1.renderData[#roadS1.renderData]
  local p1A, p1B = rLast[1][1], rLast[-1][2]
  local tgt = roadS1.nodes[#roadS1.nodes].p - roadS1.nodes[1].p
  tgt:normalize()

  local roadS2A = roadMgr.createRoadFromProfile(profileS2A)
  roadS2A.displayName = im.ArrayChar(32, 'jct - section2A')
  roadS2A.isJctRoad = true
  profileS2A.isEdgeBlendL = im.BoolPtr(true)
  profileS2A.isEdgeBlendR = im.BoolPtr(true)
  profileS2A.conditionEdgesL = im.BoolPtr(true)
  profileS2A.conditionEdgesR = im.BoolPtr(true)
  profileS2A.conditionEndStopS = im.BoolPtr(false)
  profileS2A.conditionEndStopE = im.BoolPtr(true)
  profileS2A.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS2A
  roadMgr.map[roadS2A.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, p1A)
  roadMgr.addNodeToRoad(rIdx, p1A + tgt * capLength * (1.0 / 3.0))
  roadMgr.addNodeToRoad(rIdx, p1A + tgt * capLength * (2.0 / 3.0))
  roadMgr.addNodeToRoad(rIdx, p1A + tgt * capLength)
  roadS2A.nodes[1].isLocked = true
  roadS2A.nodes[2].isLocked = true
  roadS2A.nodes[3].isLocked = true
  roadS2A.nodes[4].isLocked = false
  roadMgr.computeRoadRenderDataSingle(rIdx)

  local roadS2B = roadMgr.createRoadFromProfile(profileS2B)
  roadS2B.displayName = im.ArrayChar(32, 'jct - section2B')
  roadS2B.isJctRoad = true
  profileS2B.isEdgeBlendL = im.BoolPtr(true)
  profileS2B.isEdgeBlendR = im.BoolPtr(true)
  profileS2B.conditionEdgesL = im.BoolPtr(true)
  profileS2B.conditionEdgesR = im.BoolPtr(true)
  profileS2B.conditionEndStopS = im.BoolPtr(true)
  profileS2B.conditionEndStopE = im.BoolPtr(false)
  profileS2B.continueLinesToEnd = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS2B
  roadMgr.map[roadS2B.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, p1B + tgt * capLength)
  roadMgr.addNodeToRoad(rIdx, p1B + tgt * capLength * (2.0 / 3.0))
  roadMgr.addNodeToRoad(rIdx, p1B + tgt * capLength * (1.0 / 3.0))
  roadMgr.addNodeToRoad(rIdx, p1B)
  roadS2B.nodes[1].isLocked = false
  roadS2B.nodes[2].isLocked = true
  roadS2B.nodes[3].isLocked = true
  roadS2B.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Recompute the road map after the road changes.
  roadMgr.recomputeMap()

  jct.roads = { roadS1.name, roadS2A.name, roadS2B.name }
end

-- Updates a highway slip junction.
local function updateHighwaySlip(jIdx, jct, isMesh)
  -- Create the road profiles.
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local cResWidth, hardWidth = jct.cResWidth[0], jct.hardWidth[0]
  local s2Length, s3Length = jct.s2Length[0], jct.s3Length[0]
  local profileS1 = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, cResWidth, hardWidth, jct.edgeBlendMat)
  local profileS4 = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, cResWidth, hardWidth, jct.edgeBlendMat)
  local profileS2 = profileMgr.createProfileForJctRoadHwyS2(numLanesX, laneWidthX, cResWidth, hardWidth, jct.edgeBlendMat)
  local profileS3 = profileMgr.createProfileForJctRoadHwyS3(numLanesX, laneWidthX, cResWidth, hardWidth, jct.edgeBlendMat)
  local profileEL = profileMgr.createProfileForJctRoadHwyExit(laneWidthX, hardWidth, jct.edgeBlendMat)
  local profileER = profileMgr.createProfileForJctRoadHwyExit(laneWidthX, hardWidth, jct.edgeBlendMat)
  profileS1.layers = {}
  profileMgr.addCenterline(profileS1, true)
  profileMgr.addLaneDivisionLines(profileS1, true)
  profileS2.layers = {}
  profileMgr.addCenterline(profileS2, true)
  profileMgr.addLaneDivisionLines(profileS2, true)
  profileS3.layers = {}
  profileMgr.addCenterline(profileS3, true)
  profileMgr.addLaneDivisionLines(profileS3, true)
  profileS4.layers = {}
  profileMgr.addCenterline(profileS4, true)
  profileMgr.addLaneDivisionLines(profileS4, true)
  profileEL.layers = {}
  profileMgr.addCenterline(profileEL, true)
  profileMgr.addLaneDivisionLines(profileEL, true)
  profileER.layers = {}
  profileMgr.addCenterline(profileER, true)
  profileMgr.addLaneDivisionLines(profileER, true)
  profileMgr.autoEdgeBlending(profileS1, true, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileS2, true, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileS4, true, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileEL, true, true, jct.edgeBlendMat)
  profileMgr.autoEdgeBlending(profileER, true, true, jct.edgeBlendMat)

  -- Add limited edge blending to section 3.
  profileS3.layers[#profileS3.layers + 1] =
    {
      name = im.ArrayChar(32, 'Edge Blend Inner L'),
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(1),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.5),
      width = im.FloatPtr(2.0),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(18),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = jct.edgeBlendMat or defaultEdgeBlendMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
    }
  profileS3.layers[#profileS3.layers + 1] =
    {
      name = im.ArrayChar(32, 'Edge Blend Inner R'),
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(true),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(1),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(-0.5),
      width = im.FloatPtr(2.0),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(18),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = jct.edgeBlendMat or defaultEdgeBlendMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
    }
  local lMin, lMax = profileMgr.getMinMaxLaneKeys(profileS3)
  profileS3.layers[#profileS3.layers + 1] =
    {
      name = im.ArrayChar(32, 'Edge Blend Outer L'),
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(true),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(1),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(true), off = im.FloatPtr(-0.5),
      width = im.FloatPtr(2.0),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(18),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = jct.edgeBlendMat or defaultEdgeBlendMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
    }
  profileS3.layers[#profileS3.layers + 1] =
    {
      name = im.ArrayChar(32, 'Edge Blend Outer R'),
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(1),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lMax), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.5),
      width = im.FloatPtr(2.0),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(18),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = jct.edgeBlendMat or defaultEdgeBlendMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(0.0),
      numRows = im.IntPtr(0), numCols = im.IntPtr(0),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
    }

  -- Create the two defining inner roads.
  local boxX = jct.s2Length[0] + jct.s3Length[0]
  local boxXHalf = boxX * 0.5
  local capLength = jct.capLength[0]

  local cen = getJunctionCentroid(jIdx)
  local rot = computeInitRot(jIdx)

  -- Before creating new roads, remove all existing junction roads.
  local jRoads = jct.roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end

  -- Make the roads for the two cap sections (S1 and S4).
  local roadS1 = roadMgr.createRoadFromProfile(profileS1)
  roadS1.displayName = im.ArrayChar(32, 'jct - section1')
  roadS1.isJctRoad = true
  profileS1.isEdgeBlendL = im.BoolPtr(true)
  profileS1.isEdgeBlendR = im.BoolPtr(true)
  profileS1.conditionEdgesL = im.BoolPtr(false)
  profileS1.conditionEdgesR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS1
  roadMgr.map[roadS1.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf - capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadS1.nodes[1].isLocked = false
  roadS1.nodes[2].isLocked = true
  roadS1.nodes[3].isLocked = true
  roadS1.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  local roadS4 = roadMgr.createRoadFromProfile(profileS4)
  roadS4.displayName = im.ArrayChar(32, 'jct - section4')
  roadS4.isJctRoad = true
  profileS4.isEdgeBlendL = im.BoolPtr(true)
  profileS4.isEdgeBlendR = im.BoolPtr(true)
  profileS4.conditionEdgesL = im.BoolPtr(false)
  profileS4.conditionEdgesR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS4
  roadMgr.map[roadS4.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength * (2.0 / 3.0), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf + capLength / 3.0, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadS4.nodes[1].isLocked = false
  roadS4.nodes[2].isLocked = true
  roadS4.nodes[3].isLocked = true
  roadS4.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Make the road for the tapered section (S2).
  local roadS2 = roadMgr.createRoadFromProfile(profileS2)
  roadS2.displayName = im.ArrayChar(32, 'jct - section2')
  roadS2.isJctRoad = true
  profileS2.isEdgeBlendL = im.BoolPtr(true)
  profileS2.isEdgeBlendR = im.BoolPtr(true)
  profileS2.conditionEdgesL = im.BoolPtr(false)
  profileS2.conditionEdgesR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS2
  roadMgr.map[roadS2.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf + s2Length, 0, 0), rot) + cen)
  roadS2.nodes[1].isLocked = true
  roadS2.nodes[2].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Make the road for the split section (S3).
  local roadS3 = roadMgr.createRoadFromProfile(profileS3)
  roadS3.displayName = im.ArrayChar(32, 'jct - section3')
  roadS3.granFactor = im.IntPtr(3)
  roadS3.isJctRoad = true
  profileS3.isEdgeBlendL = im.BoolPtr(true)
  profileS3.isEdgeBlendR = im.BoolPtr(true)
  profileS3.conditionEdgesL = im.BoolPtr(false)
  profileS3.conditionEdgesR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  local s3Idx = rIdx
  roadMgr.roads[rIdx] = roadS3
  roadMgr.map[roadS3.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(-boxXHalf + s2Length, 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf - (s3Length * 0.5), 0, 0), rot) + cen)
  roadMgr.addNodeToRoad(rIdx, util.rotateVecByQuaternion(vec3(boxXHalf, 0, 0), rot) + cen)
  roadS3.nodes[1].isLocked = true
  roadS3.nodes[2].isLocked = true
  roadS3.nodes[3].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Apply the lane taper to section 2.
  local tapIdx = numLanesX + 2
  roadS2.nodes[1].widths[-tapIdx] = im.FloatPtr(0.0)
  roadS2.nodes[1].widths[tapIdx] = im.FloatPtr(0.0)

  -- Apply the split taper to section 3.
  local tapIdx = numLanesX + 2
  roadS3.nodes[1].widths[-tapIdx] = im.FloatPtr(0.0)
  roadS3.nodes[1].widths[tapIdx] = im.FloatPtr(0.0)
  roadS3.nodes[2].widths[-tapIdx] = im.FloatPtr(jct.sepWidthI[0])
  roadS3.nodes[2].widths[tapIdx] = im.FloatPtr(jct.sepWidthI[0])
  roadS3.nodes[3].widths[-tapIdx] = im.FloatPtr(jct.sepWidthO[0])
  roadS3.nodes[3].widths[tapIdx] = im.FloatPtr(jct.sepWidthO[0])

  -- Taper the hard shoulder on section 4.
  local tapIdx = numLanesX + 2
  roadS4.nodes[4].widths[tapIdx] = im.FloatPtr(0.0)
  roadS4.nodes[4].widths[-tapIdx] = im.FloatPtr(0.0)
  roadS4.nodes[3].widths[tapIdx] = im.FloatPtr(hardWidth * 0.3333333333)
  roadS4.nodes[3].widths[-tapIdx] = im.FloatPtr(hardWidth * 0.3333333333)
  roadS4.nodes[2].widths[tapIdx] = im.FloatPtr(hardWidth * 0.66666666667)
  roadS4.nodes[2].widths[-tapIdx] = im.FloatPtr(hardWidth * 0.66666666667)

  roadMgr.computeRoadRenderDataSingle(s3Idx)

  -- Make the two one-way exit roads.
  local roadEL = roadMgr.createRoadFromProfile(profileEL)
  roadEL.displayName = im.ArrayChar(32, 'jct - exit 1')
  roadEL.granFactor = im.IntPtr(3)
  roadEL.isJctRoad = true
  profileEL.isEdgeBlendL = im.BoolPtr(true)
  profileEL.isEdgeBlendR = im.BoolPtr(true)
  profileEL.conditionEdgesL = im.BoolPtr(false)
  profileEL.conditionEdgesR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadEL
  roadMgr.map[roadEL.name] = rIdx
  local p1 = roadS3.renderData[#roadS3.renderData][-numLanesX - 3][2]
  local tgt = p1 - roadS3.renderData[1][-numLanesX - 3][2]
  tgt:normalize()
  local p2 = p1 + tgt * capLength
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, p1 + (p2 - p1) * (2.0 / 3.0))
  roadMgr.addNodeToRoad(rIdx, p1 + (p2 - p1) / 3.0)
  roadMgr.addNodeToRoad(rIdx, p1)
  roadEL.nodes[1].isLocked = false
  roadEL.nodes[2].isLocked = true
  roadEL.nodes[3].isLocked = true
  roadEL.nodes[4].isLocked = true
  roadMgr.computeRoadRenderDataSingle(rIdx)

  local roadER = roadMgr.createRoadFromProfile(profileER)
  roadER.displayName = im.ArrayChar(32, 'jct - exit 2')
  roadER.granFactor = im.IntPtr(3)
  roadER.isJctRoad = true
  profileER.isEdgeBlendL = im.BoolPtr(true)
  profileER.isEdgeBlendR = im.BoolPtr(true)
  profileER.conditionEdgesL = im.BoolPtr(false)
  profileER.conditionEdgesR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadER
  roadMgr.map[roadER.name] = rIdx
  local p1 = roadS3.renderData[#roadS3.renderData][numLanesX + 3][1]
  local tgt = p1 - roadS3.renderData[1][numLanesX + 3][1]
  tgt:normalize()
  local p2 = p1 + tgt * capLength
  roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p1 + (p2 - p1) / 3.0)
  roadMgr.addNodeToRoad(rIdx, p1 + (p2 - p1) * (2.0 / 3.0))
  roadMgr.addNodeToRoad(rIdx, p2)
  roadER.nodes[1].isLocked = true
  roadER.nodes[2].isLocked = true
  roadER.nodes[3].isLocked = true
  roadER.nodes[4].isLocked = false
  roadMgr.computeRoadRenderDataSingle(rIdx)

  -- Recompute the road map after the road changes.
  roadMgr.recomputeMap()

  jct.roads = { roadS1.name, roadS4.name, roadS2.name, roadS3.name, roadEL.name, roadER.name }

  -- Add the separation lane layers.
  local newLayer = {
    name = im.ArrayChar(32, 'Separator L'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(0),
    laneMin = im.IntPtr(-numLanesX - 2), laneMax = im.IntPtr(-numLanesX - 2),
    lane = im.IntPtr(-numLanesX - 2), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(1.0),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultSeperatorMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  table.insert(profileS3.layers, 1, newLayer)
  local newLayer = {
    name = im.ArrayChar(32, 'Separator R'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(0),
    laneMin = im.IntPtr(numLanesX + 2), laneMax = im.IntPtr(numLanesX + 2),
    lane = im.IntPtr(numLanesX + 2), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(1.0),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultSeperatorMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  table.insert(profileS3.layers, 1, newLayer)

  -- Add the edge lines.
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S1_1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-numLanesX - 1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S1_2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(numLanesX + 1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S1_3'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS1.layers[#profileS1.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S1_4'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }

  profileS4.layers[#profileS4.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S4_1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-numLanesX - 1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS4.layers[#profileS4.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S4_2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(numLanesX + 1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS4.layers[#profileS4.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S4_3'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS4.layers[#profileS4.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S4_4'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }

  profileS2.layers[#profileS2.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S2_1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-numLanesX - 2), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS2.layers[#profileS2.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S2_2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(numLanesX + 2), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS2.layers[#profileS2.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S2_3'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS2.layers[#profileS2.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S2_4'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }

  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-numLanesX - 3), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(numLanesX + 3), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_3'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-numLanesX - 3), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_4'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(numLanesX + 3), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_5'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-numLanesX - 1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_6'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(numLanesX + 1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_7'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileS3.layers[#profileS3.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line S3_8'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }

  profileEL.layers[#profileEL.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line EL_1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileEL.layers[#profileEL.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line EL_2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }

  profileER.layers[#profileER.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line ER_1'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  profileER.layers[#profileER.layers + 1] = {
    name = im.ArrayChar(32, 'Edge line ER_2'),
    isHidden = false,
    doNotDelete = im.BoolPtr(true),
    isReverse = im.BoolPtr(false),
    isPaint = im.BoolPtr(false),
    isDisplay = im.BoolPtr(false),
    type = im.IntPtr(1),
    laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
    lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
    width = im.FloatPtr(0.25),
    isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
    texLen = im.FloatPtr(5),
    fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
    mat = defaultEdgeMaterial,
    rot = im.IntPtr(3),
    pos = im.FloatPtr(laneWidthX * 0.5), size = im.FloatPtr(jct.arrowSize[0]),
    numRows = im.IntPtr(4), numCols = im.IntPtr(4),
    frame = im.IntPtr(0),
    vertOffset = im.FloatPtr(0.0),
    latOffset = im.FloatPtr(0.0),
    spacing = im.FloatPtr(5.0),
    jitter = im.FloatPtr(0.0),
    useWorldZ = im.BoolPtr(false),
    matDisplay = '[None]',
    extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
    boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }

  -- Crash barriers.
  if jct.isBarriersI[0] then
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS4.layers[#profileS4.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS4.layers[#profileS4.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R2'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end
  if jct.isBarriersO[0] then
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-numLanesX - 2), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS1.layers[#profileS1.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(numLanesX + 2), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-numLanesX - 3), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS2.layers[#profileS2.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(numLanesX + 3), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier L1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-numLanesX - 4), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(-0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Barrier R1'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(4),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(numLanesX + 4), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultCrashBarrierPath,
      rot = im.IntPtr(2),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(-0.15),
      latOffset = im.FloatPtr(0.15),
      spacing = im.FloatPtr(-0.17),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_guardrails_basic.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end

  -- Add road signs (poles).
  if jct.isSigns[0] then
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Pass Either Side Sign R'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(-numLanesX - 1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultPassEitherSideSign,
      rot = im.IntPtr(3),
      pos = im.FloatPtr(1.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(-jct.sepWidthO[0] * 0.5),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_pass_either_side.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
    profileS3.layers[#profileS3.layers + 1] = {
      name = im.ArrayChar(32, 'Merge Sign R'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(5),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(numLanesX + 1), isLeft = im.BoolPtr(false), off = im.FloatPtr(0.0),
      width = im.FloatPtr(0.25),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = defaultLaneMergeSign,
      rot = im.IntPtr(1),
      pos = im.FloatPtr(1.0), size = im.FloatPtr(5),
      numRows = im.IntPtr(4), numCols = im.IntPtr(4),
      frame = im.IntPtr(0),
      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(jct.sepWidthO[0] * 0.5),
      spacing = im.FloatPtr(1.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = 'italy_traf_sign_junction_merge_left.dae',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0 }
  end

  if jct.isCrossings[0] then
    overlayUtils.addHighwaySlipOverlays(jct)
  end
end

-- Update the junction with the given id, after a parameter change.
local function updateJunctionAfterChange(jIdx)
  table.clear(roadMgr.multi)
  local jct = junctions[jIdx]
  if jct.type == 'crossing' then
    updateCrossing(jIdx, jct)
  elseif jct.type == 'crossroads' then
    updateCrossroads(jIdx, jct)
  elseif jct.type == 't-junction' then
    updateTJunction(jIdx, jct)
  elseif jct.type == 'y-junction' then
    updateYJunction(jIdx, jct)
  elseif jct.type == 'roundabout' then
    updateRoundabout(jIdx, jct)
  elseif jct.type == 'rural_urban_transition' then
    updateRuralUrbanTransition(jIdx, jct)
  elseif jct.type == 'urban_merge' then
    updateUrbanMerge(jIdx, jct)
  elseif jct.type == 'urban_separator' then
    updateUrbanSeparator(jIdx, jct)
  elseif jct.type == 'highway_merge' then
    updateHighwayMerge(jIdx, jct)
  elseif jct.type == 'highway_urban_transition' then
    updateHighwayUrbanTransition(jIdx, jct)
  elseif jct.type == 'highway_separator' then
    updateHighwaySeparator(jIdx, jct)
  elseif jct.type == 'shoulder_fade' then
    updateShoulderFade(jIdx, jct)
  elseif jct.type == 'highway_slip' then
    updateHighwaySlip(jIdx, jct)
  else
    -- TODO:  other junction types.
  end
  updateJunctionCondition(jct)
end

-- Adds a 2-way pedestrian crossing junction (ped x + traffic lights only).
local function addPedXJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Crossing'),
      type = 'crossing',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(10.0),
      numLanesX = im.IntPtr(1),
      numLanesY = im.IntPtr(1),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(1.0),
      s3Length = im.FloatPtr(1.0),
      cResWidth = im.FloatPtr(1.0),
      sepWidthI = im.FloatPtr(1.0),
      sepWidthO = im.FloatPtr(1.0),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(1.0),
      isBarriersI = im.BoolPtr(false),
      isBarriersO = im.BoolPtr(false),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(3.5),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-0.5),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(7),
      seed = im.IntPtr(41226) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX, numLanesY = jct.numLanesX[0], jct.numLanesY[0]
  local laneWidthX, laneWidthY = jct.laneWidthX[0], jct.laneWidthY[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)
  local profileX2_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)

  -- Create the two defining inner roads.
  local boxX = numLanesY * 2 * laneWidthY
  local boxXHalf = boxX * 0.5
  local bevel = jct.bevel[0]

  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(false)
  profileX1_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf - bevel, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf, 0, 0))

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(false)
  profileX2_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf + bevel, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf, 0, 0))

  jct.roads = { roadX1_I.name, roadX2_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds a 4-way crossroads junction.
local function addCrossroads(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Crossroads'),
      type = 'crossroads',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(4.0),
      numLanesX = im.IntPtr(1),
      numLanesY = im.IntPtr(1),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(false),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(1.0),
      s3Length = im.FloatPtr(1.0),
      cResWidth = im.FloatPtr(1.0),
      sepWidthI = im.FloatPtr(1.0),
      sepWidthO = im.FloatPtr(1.0),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(1.0),
      isBarriersI = im.BoolPtr(false),
      isBarriersO = im.BoolPtr(false),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(4),
      seed = im.IntPtr(41230) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX, numLanesY = jct.numLanesX[0], jct.numLanesY[0]
  local laneWidthX, laneWidthY = jct.laneWidthX[0], jct.laneWidthY[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)
  local profileX2_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)

  -- Create the two defining inner roads.
  local boxX = numLanesY * 2 * laneWidthY
  local boxXHalf = boxX * 0.5
  local bevel = jct.bevel[0]

  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(false)
  profileX1_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf - bevel, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf, 0, 0))

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(false)
  profileX2_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf + bevel, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf, 0, 0))

  jct.roads = { roadX1_I.name, roadX2_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds a 3-way T-junction.
local function addTJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New T-Junction'),
      type = 't-junction',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(4.0),
      numLanesX = im.IntPtr(1),
      numLanesY = im.IntPtr(1),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(1.0),
      s3Length = im.FloatPtr(1.0),
      cResWidth = im.FloatPtr(1.0),
      sepWidthI = im.FloatPtr(1.0),
      sepWidthO = im.FloatPtr(1.0),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(1.0),
      isBarriersI = im.BoolPtr(false),
      isBarriersO = im.BoolPtr(false),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(false),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(4),
      seed = im.IntPtr(41235) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX, numLanesY = jct.numLanesX[0], jct.numLanesY[0]
  local laneWidthX, laneWidthY = jct.laneWidthX[0], jct.laneWidthY[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)
  local profileX2_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)

  -- Create the two defining inner roads.
  local boxX = numLanesY * 2 * laneWidthY
  local boxXHalf = boxX * 0.5
  local bevel = jct.bevel[0]

  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'jct road 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(false)
  profileX1_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf - bevel, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf, 0, 0))

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'jct road 2')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(false)
  profileX2_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf + bevel, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf, 0, 0))

  jct.roads = { roadX1_I.name, roadX2_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds a 3-way angled Y-junction.
local function addYJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Y-Junction'),
      type = 'y-junction',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(9.0),
      numLanesX = im.IntPtr(1),
      numLanesY = im.IntPtr(1),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(1.0),
      s3Length = im.FloatPtr(1.0),
      cResWidth = im.FloatPtr(1.0),
      sepWidthI = im.FloatPtr(1.0),
      sepWidthO = im.FloatPtr(1.0),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(1.0),
      isBarriersI = im.BoolPtr(false),
      isBarriersO = im.BoolPtr(false),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(false),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(10.0),
      theta = im.FloatPtr(20.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-0.5),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(3),
      seed = im.IntPtr(41234) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX, numLanesY = jct.numLanesX[0], jct.numLanesY[0]
  local laneWidthX, laneWidthY = jct.laneWidthX[0], jct.laneWidthY[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)
  local profileX2_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)

  -- Create the two defining inner roads.
  local boxX = numLanesY * 2 * laneWidthY
  local boxXHalf = boxX * 0.5
  local bevel = jct.bevel[0]

  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'jct road 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(true)
  profileX1_I.isEdgeBlendR = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf - bevel, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf, 0, 0))

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'jct road 2')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(true)
  profileX2_I.isEdgeBlendR = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf + bevel, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf, 0, 0))

  jct.roads = { roadX1_I.name, roadX2_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds a 4-way roundabout junction.
local function addRoundaboutJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Roundabout'),
      type = 'roundabout',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(9.0),
      numLanesX = im.IntPtr(1),
      numLanesY = im.IntPtr(1),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(1),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(-1.2),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(1.0),
      s3Length = im.FloatPtr(1.0),
      cResWidth = im.FloatPtr(1.0),
      sepWidthI = im.FloatPtr(1.0),
      sepWidthO = im.FloatPtr(1.0),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(1.0),
      isBarriersI = im.BoolPtr(false),
      isBarriersO = im.BoolPtr(false),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(11.0),
      theta = im.FloatPtr(20.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(false),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(2),
      seed = im.IntPtr(41246) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX, numLanesY = jct.numLanesX[0], jct.numLanesY[0]
  local laneWidthX, laneWidthY = jct.laneWidthX[0], jct.laneWidthY[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)
  local profileX2_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)

  -- Create the two defining inner roads.
  local boxX = numLanesY * 2 * laneWidthY
  local boxXHalf = boxX * 0.5
  local bevel = jct.bevel[0]

  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'jct road 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(true)
  profileX1_I.isEdgeBlendR = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf - bevel, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf, 0, 0))

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'jct road 2')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(true)
  profileX2_I.isEdgeBlendR = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf + bevel, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf, 0, 0))

  jct.roads = { roadX1_I.name, roadX2_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds an urban sidewalk/non-sidewalk transition junction.
local function addRuralUrbanTransJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Sidewalk Urban Transition Junction'),
      type = 'rural_urban_transition',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(4.0),
      numLanesX = im.IntPtr(1),
      numLanesY = im.IntPtr(1),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(1.0),
      s3Length = im.FloatPtr(1.0),
      cResWidth = im.FloatPtr(1.0),
      sepWidthI = im.FloatPtr(1.0),
      sepWidthO = im.FloatPtr(1.0),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(1.0),
      isBarriersI = im.BoolPtr(false),
      isBarriersO = im.BoolPtr(false),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(7),
      seed = im.IntPtr(41226) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local profileX3_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)

  local roadX3_I = roadMgr.createRoadFromProfile(profileX3_I)
  roadX3_I.displayName = im.ArrayChar(32, 'temp - jct construct 3')
  roadX3_I.isJctRoad = true
  profileX3_I.isEdgeBlendL = im.BoolPtr(false)
  profileX3_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX3_I
  roadMgr.map[roadX3_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-jct.capLength[0] * 0.5, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(jct.capLength[0] * 0.5, 0, 0))

  jct.roads = { {}, {}, roadX3_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds an urban merge junction (merges eg 2 lanes -> 1 lane, 3 lanes -> lanes, etc).
local function addUrbanMergeJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Urban Merge'),
      type = 'urban_merge',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(5.0),
      numLanesX = im.IntPtr(2),
      numLanesY = im.IntPtr(2),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(20.0),
      s3Length = im.FloatPtr(10.0),
      cResWidth = im.FloatPtr(3.5),
      sepWidthI = im.FloatPtr(1.2),
      sepWidthO = im.FloatPtr(2.4),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(3.0),
      isBarriersI = im.BoolPtr(true),
      isBarriersO = im.BoolPtr(true),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(7),
      seed = im.IntPtr(41226) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)
  local profileX2_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, false)

  -- Create the two defining inner roads.
  local boxX = jct.s2Length[0] + jct.s3Length[0]
  local boxXHalf = boxX * 0.5
  local capLength = jct.capLength[0]

  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(false)
  profileX1_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf - capLength, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf, 0, 0))

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(false)
  profileX2_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf + capLength, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf, 0, 0))

  jct.roads = { roadX1_I.name, roadX2_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds an urban separator junction.
local function addUrbanSeparatorJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Urban Separator'),
      type = 'urban_separator',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(10.0),
      numLanesX = im.IntPtr(2),
      numLanesY = im.IntPtr(2),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(40.0),
      s3Length = im.FloatPtr(10.0),
      cResWidth = im.FloatPtr(3.5),
      sepWidthI = im.FloatPtr(0.9),
      sepWidthO = im.FloatPtr(2.4),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(3.0),
      isBarriersI = im.BoolPtr(true),
      isBarriersO = im.BoolPtr(true),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(7),
      seed = im.IntPtr(41226) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local sidewalkWidth = jct.sidewalkWidth[0]
  local sidewalkHeight = jct.sidewalkHeight[0]
  local profileX1_I = profileMgr.createProfileForJctRoad(numLanesX, numLanesX, laneWidthX, sidewalkWidth, sidewalkHeight, true)

  -- Create the two defining inner roads.
  local capLength = jct.capLength[0]
  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(false)
  profileX1_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-capLength, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(0, 0, 0))

  jct.roads = { roadX1_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds a highway merge junction (merges eg 2 lanes -> 1 lane, 3 lanes -> lanes, etc).
local function addHighwayMergeJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Highway Merge'),
      type = 'highway_merge',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(10.0),
      numLanesX = im.IntPtr(2),
      numLanesY = im.IntPtr(2),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(40.0),
      s3Length = im.FloatPtr(10.0),
      cResWidth = im.FloatPtr(3.5),
      sepWidthI = im.FloatPtr(0.9),
      sepWidthO = im.FloatPtr(2.4),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(3.0),
      isBarriersI = im.BoolPtr(true),
      isBarriersO = im.BoolPtr(true),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(7),
      seed = im.IntPtr(41226) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local profileX1_I = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, jct.cResWidth[0], jct.hardWidth[0])
  local profileX2_I = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, jct.cResWidth[0], jct.hardWidth[0])

  -- Create the two defining inner roads.
  local boxX = jct.s2Length[0] + jct.s3Length[0]
  local boxXHalf = boxX * 0.5
  local capLength = jct.capLength[0]

  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(false)
  profileX1_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf - capLength, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf, 0, 0))

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(false)
  profileX2_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf + capLength, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf, 0, 0))

  jct.roads = { roadX1_I.name, roadX2_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds a highway <-> urban transition junction (tapers the central reservation down to a centerline).
local function addHighwayUrbanTransJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Hwy/Urban Transition'),
      type = 'highway_urban_transition',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(5.0),
      numLanesX = im.IntPtr(2),
      numLanesY = im.IntPtr(2),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(10.0),
      s3Length = im.FloatPtr(10.0),
      cResWidth = im.FloatPtr(3.5),
      sepWidthI = im.FloatPtr(1.17),
      sepWidthO = im.FloatPtr(0.58),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(3.0),
      isBarriersI = im.BoolPtr(true),
      isBarriersO = im.BoolPtr(true),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(6),
      seed = im.IntPtr(41273) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]

  -- Create the two defining inner roads.
  local boxX = jct.s2Length[0]
  local boxXHalf = boxX * 0.5

  local profileS2 = profileMgr.createProfileForJctRoadHwyUrbanTrans(numLanesX, laneWidthX, 1.0, 1.0, false)

  -- Make the road for the tapered section (S2).
  local roadS2 = roadMgr.createRoadFromProfile(profileS2)
  roadS2.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadS2.granFactor = im.IntPtr(2)
  roadS2.isJctRoad = true
  profileS2.isEdgeBlendL = im.BoolPtr(true)
  profileS2.isEdgeBlendR = im.BoolPtr(true)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadS2
  roadMgr.map[roadS2.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf + (boxX / 3.0), 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf - (boxX / 3.0), 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf, 0, 0))

  jct.roads = { {}, {}, roadS2.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds a highway separator junction (splits a two-way highway into two one-way sections, for separate linking).
local function addHighwaySeparatorJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Highway Separator'),
      type = 'highway_separator',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(10.0),
      numLanesX = im.IntPtr(2),
      numLanesY = im.IntPtr(2),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(40.0),
      s3Length = im.FloatPtr(10.0),
      cResWidth = im.FloatPtr(3.5),
      sepWidthI = im.FloatPtr(0.9),
      sepWidthO = im.FloatPtr(2.4),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(3.0),
      isBarriersI = im.BoolPtr(true),
      isBarriersO = im.BoolPtr(true),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(7),
      seed = im.IntPtr(41226) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local profileX1_I = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, jct.cResWidth[0], jct.hardWidth[0])

  -- Create the two defining inner roads.
  local capLength = jct.capLength[0]
  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(false)
  profileX1_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-capLength, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(0, 0, 0))

  jct.roads = { roadX1_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds a shoulder fade junction (tapers out the hard shoulder lane of a 1-way highway, to become a 1-way rural road).
local function addShoulderFadeJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Shoulder Fade'),
      type = 'shoulder_fade',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(5.0),
      numLanesX = im.IntPtr(2),
      numLanesY = im.IntPtr(2),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(40.0),
      s3Length = im.FloatPtr(10.0),
      cResWidth = im.FloatPtr(3.5),
      sepWidthI = im.FloatPtr(0.9),
      sepWidthO = im.FloatPtr(2.4),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(3.0),
      isBarriersI = im.BoolPtr(true),
      isBarriersO = im.BoolPtr(true),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(7),
      seed = im.IntPtr(41226) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local profileX1_I = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, jct.cResWidth[0], jct.hardWidth[0])
  local profileX2_I = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, jct.cResWidth[0], jct.hardWidth[0])

  -- Create the two defining inner roads.
  local boxX = jct.s2Length[0] + jct.s3Length[0]
  local boxXHalf = boxX * 0.5
  local capLength = jct.capLength[0]

  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(false)
  profileX1_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf - capLength, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf, 0, 0))

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(false)
  profileX2_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf + capLength, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf, 0, 0))

  jct.roads = { roadX1_I.name, roadX2_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Adds a highway slip junction.
local function addHighwaySlipJunction(isNew)
  if isNew then
    junctions[#junctions + 1] = {
      name = im.ArrayChar(32, 'New Highway Slip'),
      type = 'highway_slip',
      roads = {},
      condition = im.FloatPtr(0.2),
      conditionSeed = im.IntPtr(41226),
      numPatches = im.IntPtr(10),
      numPotholes = im.IntPtr(0),
      capLength = im.FloatPtr(14.0),
      numLanesX = im.IntPtr(2),
      numLanesY = im.IntPtr(2),
      laneWidthX = im.FloatPtr(3.5),
      laneWidthY = im.FloatPtr(3.5),
      numRBLanes = im.IntPtr(2),
      laneWidthRB = im.FloatPtr(3.5),
      extraRadRB = im.FloatPtr(0.0),
      isYOneWay = im.BoolPtr(false),
      isY1Outwards = im.BoolPtr(true),
      isY2Outwards = im.BoolPtr(false),
      s2Length = im.FloatPtr(20.0),
      s3Length = im.FloatPtr(20.0),
      cResWidth = im.FloatPtr(3.5),
      sepWidthI = im.FloatPtr(1.5),
      sepWidthO = im.FloatPtr(3.0),
      sepMat = defaultSeperatorMaterial,
      hardWidth = im.FloatPtr(3.0),
      isBarriersI = im.BoolPtr(true),
      isBarriersO = im.BoolPtr(true),
      isSigns = im.BoolPtr(true),
      isPedX1 = im.BoolPtr(true),
      isPedX2 = im.BoolPtr(true),
      isPedX3 = im.BoolPtr(true),
      isPedX4 = im.BoolPtr(true),
      pedXDist = im.FloatPtr(1.0),
      pedXWidth = im.FloatPtr(2.0),
      isSidewalk = im.BoolPtr(true),
      bevel = im.FloatPtr(2.5),
      theta = im.FloatPtr(0.0),
      sidewalkWidth = im.FloatPtr(2.0),
      sidewalkHeight = im.FloatPtr(0.12),
      isLowerSWAtPedX = im.BoolPtr(true),
      isTLights = im.BoolPtr(true),
      trafficLatOff = im.FloatPtr(-2.6),
      isCrossings = im.BoolPtr(true),
      displayCrossings = im.BoolPtr(false),
      edgeBlendMat = defaultEdgeBlendMaterial,
      isArrow = im.BoolPtr(true),
      isDoubleArrows = im.BoolPtr(true),
      arrowSize = im.FloatPtr(1.5),
      arrowFrontDistFromEnd = im.FloatPtr(2.5),
      arrowBackDistFromEnd = im.FloatPtr(12.0),
      arrowMat = defaultLaneArrowMaterial,
      numCrossings = im.IntPtr(8),
      seed = im.IntPtr(41239) }
  end

  -- Create the two defining road profiles.
  local jct = junctions[#junctions]
  local numLanesX = jct.numLanesX[0]
  local laneWidthX = jct.laneWidthX[0]
  local profileX1_I = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, jct.cResWidth[0], jct.hardWidth[0])
  local profileX2_I = profileMgr.createProfileForJctRoadHwyCap(numLanesX, laneWidthX, jct.cResWidth[0], jct.hardWidth[0])

  -- Create the two defining inner roads.
  local boxX = jct.s2Length[0] + jct.s3Length[0]
  local boxXHalf = boxX * 0.5
  local capLength = jct.capLength[0]

  local roadX1_I = roadMgr.createRoadFromProfile(profileX1_I)
  roadX1_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX1_I.isJctRoad = true
  profileX1_I.isEdgeBlendL = im.BoolPtr(false)
  profileX1_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX1_I
  roadMgr.map[roadX1_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf - capLength, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(-boxXHalf, 0, 0))

  local roadX2_I = roadMgr.createRoadFromProfile(profileX2_I)
  roadX2_I.displayName = im.ArrayChar(32, 'temp - jct construct 1')
  roadX2_I.isJctRoad = true
  profileX2_I.isEdgeBlendL = im.BoolPtr(false)
  profileX2_I.isEdgeBlendR = im.BoolPtr(false)
  local rIdx = #roadMgr.roads + 1
  roadMgr.roads[rIdx] = roadX2_I
  roadMgr.map[roadX2_I.name] = rIdx
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf + capLength, 0, 0))
  roadMgr.addNodeToRoad(rIdx, vec3(boxXHalf, 0, 0))

  jct.roads = { roadX1_I.name, roadX2_I.name }

  updateJunctionAfterChange(#junctions)
  roadMgr.recomputeMap()
end

-- Removes the junction with the given index.
local function removeJunction(jIdx)
  local jRoads = junctions[jIdx].roads
  for i = 1, #jRoads do
    roadMgr.removeRoad(jRoads[i])
  end
  table.remove(junctions, jIdx)
end

-- Removes all junctions.
local function clearAllJunctions()
  for i = 1, #junctions do
    removeJunction(i)
  end
end

-- Translates the junction with the given index, by the given vector.
local function translateJunction(jIdx, offset)
  local jRoads = junctions[jIdx].roads
  for i = 1, #jRoads do
    local road = roadMgr.roads[roadMgr.map[jRoads[i]]]
    local nodes = road.nodes
    for j = 1, #nodes do
      nodes[j].p = nodes[j].p + offset
    end
    roadMgr.setDirty(road)
  end
end

-- Rotates the junction with the given index, by the given angle.
local function rotateJunction(jIdx, cen, theta)
  local s, c = sin(theta), cos(theta)
  local jRoads = junctions[jIdx].roads
  for i = 1, #jRoads do
    local road = roadMgr.roads[roadMgr.map[jRoads[i]]]
    local nodes = road.nodes
    for j = 1, #nodes do
      local n = nodes[j]
      local zOld = n.p.z
      local p = n.p - cen
      local x, y = p.x, p.y
      n.p:set(x * c - y * s + cen.x, x * s + y * c + cen.y, zOld)
    end
    roadMgr.setDirty(road)
  end
end

-- Rotates the junction with the given index, by the given quaternion.
local function rotateJunctionQuat(jIdx, cen, q)
  local jRoads = junctions[jIdx].roads
  for i = 1, #jRoads do
    local road = roadMgr.roads[roadMgr.map[jRoads[i]]]
    local nodes = road.nodes
    for j = 1, #nodes do
      nodes[j].p = util.rotateVecByQuaternion(nodes[j].p - cen, q) + cen
    end
    roadMgr.setDirty(road)
  end
end

-- Moves the camera above the junction with the given index, to highlight it to the user.
local function goToJunction(jIdx)
  -- Compute the 2D axis-aligned bounding box of the given junction.
  -- [We also compute the largest height value].
  local xMin, xMax, yMin, yMax, zMax = 1e24, -1e24, 1e24, -1e24, -1e24
  local roads = roadMgr.roads
  local roadsList = junctions[jIdx].roads
  for j = 1, #roadsList do
    local road = roads[roadMgr.map[roadsList[j]]]
    if road then
      local nodes = road.nodes
      local numNodes = #nodes
      for i = 1, numNodes do
        local p = nodes[i].p
        local px, py = p.x, p.y
        xMin, xMax = min(xMin, px), max(xMax, px)
        yMin, yMax = min(yMin, py), max(yMax, py)
        zMax = max(zMax, p.z)
      end
    end
  end

  -- If box is too small (eg one node), do nothing.
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

-- Sets all the roads in the junction with the given id, to use meshes/not use meshes.
local function setAllMeshProperty(jIdx)
  local rNames = junctions[jIdx].roads
  for i = 1, #rNames do
    local r = roadMgr.roads[roadMgr.map[rNames[i]]]
  end
end

-- Updates the junctions list (called after a road has been removed).
local function updateJunctionsAfterRoadRemove()
  for i = #junctions, 1, -1 do
    local jct = junctions[i]
    for j = #jct.roads, 1, -1 do
      local road = roadMgr.roads[roadMgr.map[jct.roads[j]]]
      if not road then
        table.remove(junctions, i)
        break
      end
    end
  end
end

-- Finalises the junction with the given index.
local function finaliseJunction(jIdx)
  local jct = junctions[jIdx]
  if jct then
    local jctName = ffi.string(jct.name)
    local jRoads = jct.roads
    for i = 1, #jRoads do
      local jR = roadMgr.roads[roadMgr.map[jRoads[i]]]
      if jR then
        if jR.isOverlay then
          jR.displayName = im.ArrayChar(32, 'Overlay ' .. tostring(i))
        else
          jR.displayName = im.ArrayChar(32, jctName .. ' R' .. tostring(i))
        end
        jR.isJctRoad = false
      end
    end
  end
  table.remove(junctions, jIdx)
end

-- Deep copies the given junction.
local function copyJunction(jct)
  return {
    name = im.ArrayChar(32, ffi.string(jct.name)),
    type = jct.type,
    roads = deepcopy(jct.roads),
    condition = im.FloatPtr(jct.condition[0]),
    conditionSeed = im.IntPtr(jct.conditionSeed[0]),
    numPatches = im.IntPtr(jct.numPatches[0]),
    numPotholes = im.IntPtr(jct.numPotholes[0]),
    capLength = im.FloatPtr(jct.capLength[0]),
    numLanesX = im.IntPtr(jct.numLanesX[0]),
    numLanesY = im.IntPtr(jct.numLanesY[0]),
    numRBLanes = im.IntPtr(jct.numRBLanes[0]),
    laneWidthX = im.FloatPtr(jct.laneWidthX[0]),
    laneWidthY = im.FloatPtr(jct.laneWidthY[0]),
    laneWidthRB = im.FloatPtr(jct.laneWidthRB[0]),
    extraRadRB = im.FloatPtr(jct.extraRadRB[0]),
    isYOneWay = im.BoolPtr(jct.isYOneWay[0]),
    isY1Outwards = im.BoolPtr(jct.isY1Outwards[0]),
    isY2Outwards = im.BoolPtr(jct.isY2Outwards[0]),
    s2Length = im.FloatPtr(jct.s2Length[0]),
    s3Length = im.FloatPtr(jct.s3Length[0]),
    cResWidth = im.FloatPtr(jct.cResWidth[0]),
    sepWidthI = im.FloatPtr(jct.sepWidthI[0]),
    sepWidthO = im.FloatPtr(jct.sepWidthO[0]),
    sepMat = jct.sepMat,
    hardWidth = im.FloatPtr(jct.hardWidth[0]),
    isBarriersI = im.BoolPtr(jct.isBarriersI[0]),
    isBarriersO = im.BoolPtr(jct.isBarriersO[0]),
    isSigns = im.BoolPtr(jct.isSigns[0]),
    isPedX1 = im.BoolPtr(jct.isPedX1[0]),
    isPedX2 = im.BoolPtr(jct.isPedX2[0]),
    isPedX3 = im.BoolPtr(jct.isPedX3[0]),
    isPedX4 = im.BoolPtr(jct.isPedX4[0]),
    pedXDist = im.FloatPtr(jct.pedXDist[0]),
    pedXWidth = im.FloatPtr(jct.pedXWidth[0]),
    isSidewalk = im.BoolPtr(jct.isSidewalk[0]),
    bevel = im.FloatPtr(jct.bevel[0]),
    theta = im.FloatPtr(jct.theta[0]),
    sidewalkWidth = im.FloatPtr(jct.sidewalkWidth[0]),
    sidewalkHeight = im.FloatPtr(jct.sidewalkHeight[0]),
    isLowerSWAtPedX = im.BoolPtr(jct.isLowerSWAtPedX[0]),
    isTLights = im.BoolPtr(jct.isTLights[0]),
    trafficLatOff = im.FloatPtr(jct.trafficLatOff[0]),
    isCrossings = im.BoolPtr(true),
    displayCrossings = im.BoolPtr(jct.displayCrossings[0]),
    edgeBlendMat = jct.edgeBlendMat or defaultEdgeBlendMaterial,
    isArrow = im.BoolPtr(jct.isArrow[0]),
	  isDoubleArrows = im.BoolPtr(jct.isDoubleArrows[0]),
	  arrowSize = im.FloatPtr(jct.arrowSize[0]),
	  arrowFrontDistFromEnd = im.FloatPtr(jct.arrowFrontDistFromEnd[0]),
	  arrowBackDistFromEnd = im.FloatPtr(jct.arrowBackDistFromEnd[0]),
	  arrowMat = jct.arrowMat,
    numCrossings = im.IntPtr(jct.numCrossings[0]),
    seed = im.IntPtr(jct.seed[0]) }
end

-- Serialises the given junction.
local function serialiseJct(jct)
  return {
    name = ffi.string(jct.name),
    type = jct.type,
    roads = jct.roads,
    condition = jct.condition[0],
    conditionSeed = jct.conditionSeed[0],
    numPatches = jct.numPatches[0],
    numPotholes = jct.numPotholes[0],
    capLength = jct.capLength[0],
    numLanesX = jct.numLanesX[0],
    numLanesY = jct.numLanesY[0],
    numRBLanes = jct.numRBLanes[0],
    laneWidthX = jct.laneWidthX[0],
    laneWidthY = jct.laneWidthY[0],
    laneWidthRB = jct.laneWidthRB[0],
    extraRadRB = jct.extraRadRB[0],
    isYOneWay = jct.isYOneWay[0],
    isY1Outwards = jct.isY1Outwards[0],
    isY2Outwards = jct.isY2Outwards[0],
    s2Length = jct.s2Length[0],
    s3Length = jct.s3Length[0],
    cResWidth = jct.cResWidth[0],
    sepWidthI = jct.sepWidthI[0],
    sepWidthO = jct.sepWidthO[0],
    sepMat = jct.sepMat,
    hardWidth = jct.hardWidth[0],
    isBarriersI = jct.isBarriersI[0],
    isBarriersO = jct.isBarriersO[0],
    isSigns = jct.isSigns[0],
    isPedX1 = jct.isPedX1[0],
    isPedX2 = jct.isPedX2[0],
    isPedX3 = jct.isPedX3[0],
    isPedX4 = jct.isPedX4[0],
    pedXDist = jct.pedXDist[0],
    pedXWidth = jct.pedXWidth[0],
    isSidewalk = jct.isSidewalk[0],
    bevel = jct.bevel[0],
    theta = jct.theta[0],
    sidewalkWidth = jct.sidewalkWidth[0],
    sidewalkHeight = jct.sidewalkHeight[0],
    isLowerSWAtPedX = jct.isLowerSWAtPedX[0],
    isTLights = jct.isTLights[0],
    trafficLatOff = jct.trafficLatOff[0],
    isCrossings = jct.isCrossings[0],
    displayCrossings = jct.displayCrossings[0],
    edgeBlendMat = jct.edgeBlendMat,
    isArrow = jct.isArrow[0],
	  isDoubleArrows = jct.isDoubleArrows[0],
	  arrowSize = jct.arrowSize[0],
	  arrowFrontDistFromEnd = jct.arrowFrontDistFromEnd[0],
	  arrowBackDistFromEnd = jct.arrowBackDistFromEnd[0],
	  arrowMat = jct.arrowMat,
    numCrossings = jct.numCrossings[0],
    seed = jct.seed[0] }
end

-- Deserialises the given junction.
local function deserialiseJct(jSer)
  return {
    name = im.ArrayChar(32, jSer.name),
    type = jSer.type,
    roads = jSer.roads,
    condition = im.FloatPtr(jSer.condition),
    conditionSeed = im.IntPtr(jSer.conditionSeed),
    numPatches = im.IntPtr(jSer.numPatches),
    numPotholes = im.IntPtr(jSer.numPotholes),
    capLength = im.FloatPtr(jSer.capLength),
    numLanesX = im.IntPtr(jSer.numLanesX),
    numLanesY = im.IntPtr(jSer.numLanesY),
    numRBLanes = im.IntPtr(jSer.numRBLanes),
    laneWidthX = im.FloatPtr(jSer.laneWidthX),
    laneWidthY = im.FloatPtr(jSer.laneWidthY),
    laneWidthRB = im.FloatPtr(jSer.laneWidthRB),
    extraRadRB = im.FloatPtr(jSer.extraRadRB),
    isYOneWay = im.BoolPtr(jSer.isYOneWay or false),
    isY1Outwards = im.BoolPtr(jSer.isY1Outwards or false),
    isY2Outwards = im.BoolPtr(jSer.isY2Outwards or false),
    s2Length = im.FloatPtr(jSer.s2Length),
    s3Length = im.FloatPtr(jSer.s3Length),
    cResWidth = im.FloatPtr(jSer.cResWidth),
    sepWidthI = im.FloatPtr(jSer.sepWidthI),
    sepWidthO = im.FloatPtr(jSer.sepWidthO),
    sepMat = jSer.sepMat,
    hardWidth = im.FloatPtr(jSer.hardWidth),
    isBarriersI = im.BoolPtr(jSer.isBarriersI or false),
    isBarriersO = im.BoolPtr(jSer.isBarriersO or false),
    isSigns = im.BoolPtr(jSer.isSigns or false),
    isPedX1 = im.BoolPtr(jSer.isPedX1 or false),
    isPedX2 = im.BoolPtr(jSer.isPedX2 or false),
    isPedX3 = im.BoolPtr(jSer.isPedX3 or false),
    isPedX4 = im.BoolPtr(jSer.isPedX4 or false),
    pedXDist = im.FloatPtr(jSer.pedXDist),
    pedXWidth = im.FloatPtr(jSer.pedXWidth),
    isSidewalk = im.BoolPtr(jSer.isSidewalk or false),
    bevel = im.FloatPtr(jSer.bevel),
    theta = im.FloatPtr(jSer.theta),
    sidewalkWidth = im.FloatPtr(jSer.sidewalkWidth),
    sidewalkHeight = im.FloatPtr(jSer.sidewalkHeight),
    isLowerSWAtPedX = im.BoolPtr(jSer.isLowerSWAtPedX or false),
    isTLights = im.BoolPtr(jSer.isTLights or false),
    trafficLatOff = im.FloatPtr(jSer.trafficLatOff),
    isCrossings = im.BoolPtr(jSer.isCrossings or false),
    displayCrossings = im.BoolPtr(jSer.displayCrossings or false),
    edgeBlendMat = jSer.edgeBlendMat or defaultEdgeBlendMaterial,
    isArrow = im.BoolPtr(jSer.isArrow or false),
	  isDoubleArrows = im.BoolPtr(jSer.isDoubleArrows or false),
	  arrowSize = im.FloatPtr(jSer.arrowSize or 1.5),
	  arrowFrontDistFromEnd = im.FloatPtr(jSer.arrowFrontDistFromEnd or 2.5),
	  arrowBackDistFromEnd = im.FloatPtr(jSer.arrowBackDistFromEnd or 12.0),
	  arrowMat = jSer.arrowMat or defaultLaneArrowMaterial,
    numCrossings = im.IntPtr(jSer.numCrossings),
    seed = im.IntPtr(jSer.seed) }
end

-- Saves the junction with the given id to disk.
local function saveJunction(jIdx)
  extensions.editor_fileDialog.saveFile(
    function(data)
      local jSer = serialiseJct(junctions[jIdx])
      jSer.name = util.getFilenameFromPath(data.filepath)
      local encodedData = { data = { junction = jSer, pos = getJunctionCentroid(jIdx), rot = computeInitRot(jIdx) } }
      jsonWriteFile(data.filepath, encodedData, true)
    end,
    {{"JSON",".json"}},
    false,
    "/",
    "File already exists.\nDo you want to overwrite the file?")
end

-- Loads a junction from disk.
local function loadJunction()
  extensions.editor_fileDialog.openFile(
    function(data)
      local loadedJson = jsonReadFile(data.filepath)
      local serJunction, pos, rot = loadedJson.junction, loadedJson.pos, loadedJson.rot
      local jIdx = #junctions + 1
      junctions[jIdx] = deserialiseJct(serJunction)
      local type = junctions[jIdx].type
      if type == 'crossing' then
        addPedXJunction(false)
      elseif type == 'crossroads' then
        addCrossroads(false)
      elseif type == 't-junction' then
        addTJunction(false)
      elseif type == 'y-junction' then
        addYJunction(false)
      elseif type == 'roundabout' then
        addRoundaboutJunction(false)
      elseif type == 'rural_urban_transition' then
        addRuralUrbanTransJunction(false)
      elseif type == 'urban_merge' then
        addUrbanMergeJunction(false)
      elseif type == 'urban_separator' then
        addUrbanSeparatorJunction(false)
      elseif type == 'highway_merge' then
        addHighwayMergeJunction(false)
      elseif type == 'highway_urban_transition' then
        addHighwayUrbanTransJunction(false)
      elseif type == 'highway_separator' then
        addHighwaySeparatorJunction(false)
      elseif type == 'shoulder_fade' then
        addShoulderFadeJunction(false)
      elseif type == 'highway_slip' then
        addHighwaySlipJunction(false)
      else
        -- TODO - other junction types.
      end
      rotateJctByQuat(jIdx, rot)
      translateJunction(jIdx, pos)
      table.clear(roadMgr.multi)
    end,
    {{"JSON",".json"}},
    false,
    "/")
end

-- Joins two perfectly-matched roads together, to create one single road.
local function createPerfectJoin(r1, r2, lie1, lie2)
  local newRoad = roadMgr.createRoadFromProfile(profileMgr.copyProfile(r1.profile))
  newRoad.displayName = im.ArrayChar(32, 'Joined Road')
  local nodes = {}
  local isFlipDir = false
  if lie1 == 'start' then
    for i = #r1.nodes, 2, -1 do
      nodes[#nodes + 1] = r1.nodes[i]
    end
    if lie2 == 'start' then                                                                         -- CASE: start -> start.
      for i = 1, #r2.nodes do
        nodes[#nodes + 1] = r2.nodes[i]
      end
    else                                                                                            -- CASE: start -> end.
      for i = #r2.nodes, 1, -1 do
        nodes[#nodes + 1] = r2.nodes[i]
      end
    end
    isFlipDir = true
  else
    for i = 1, #r1.nodes - 1 do
      nodes[#nodes + 1] = r1.nodes[i]
    end
    if lie2 == 'start' then                                                                         -- CASE: end -> start.
      for i = 1, #r2.nodes do
        nodes[#nodes + 1] = r2.nodes[i]
      end
    else                                                                                            -- CASE: end -> end.
      for i = #r2.nodes, 1, -1 do
        nodes[#nodes + 1] = r2.nodes[i]
      end
    end
  end
  newRoad.nodes = nodes
  newRoad.profile.layers = r1.profile.layers

  -- Update the roads list and map.
  roadMgr.removeRoad(r1.name)
  roadMgr.removeRoad(r2.name)
  local roads = roadMgr.roads
  roads[#roads + 1] = newRoad
  roadMgr.setDirty(newRoad)
  roadMgr.recomputeMap()

  -- Flip the new road, if required (used for one-way roads, to get direction correct).
  if isFlipDir then
    roadMgr.flipRoad(#roads)
  end
end

local function hashRoad(lRoad)
  local nodes = lRoad.nodes
  local sum = 0.0
  for i = 1, #nodes do
    local n = nodes[i]
    sum = sum + n.x + n.y + n.z
  end
  return floor(sum)
end

local function createJunctionAndJoin(r1, r1Lie, rOthers, forbidden)
  local roads, map = roadMgr.roads, roadMgr.map
  local numOthers = #rOthers
  if numOthers == 1 then                                                                            -- There are two roads, so just join them into one road (no junction needed).
    createPerfectJoin(r1, rOthers[1].r, r1Lie, rOthers[1].lie)
  elseif numOthers == 2 then                                                                        -- There are three roads, so fit a T-junction.
    local jCenter = nil
    if r1Lie == 'start' then
      jCenter = r1.nodes[1].p
    else
      jCenter = r1.nodes[#r1.nodes].p
    end
    addTJunction(true)
    local jIdx = #junctions
    local jct = junctions[jIdx]
    translateJunction(jIdx, jCenter)
    local jVec = roads[map[jct.roads[4]]].nodes[1].p - roads[map[jct.roads[5]]].nodes[1].p
    local jP1, jP2 = nil, nil
    if rOthers[1].lie == 'start' then
      if rOthers[2].lie == 'start' then
        jP2, jP1 = rOthers[2].r.nodes[2].p, rOthers[1].r.nodes[2].p
      else
        jP2, jP1 = rOthers[2].r.nodes[2].p, rOthers[1].r.nodes[#rOthers[1].r.nodes - 1].p
      end
    else
      if rOthers[2].lie == 'start' then
        jP2, jP1 = rOthers[2].r.nodes[#rOthers[2].r.nodes].p, rOthers[1].r.nodes[2].p
      else
        jP2, jP1 = rOthers[2].r.nodes[#rOthers[2].r.nodes].p, rOthers[1].r.nodes[#rOthers[1].r.nodes - 1].p
      end
    end
    local jCurr = jP2 - jP1
    local r1JP = nil
    if r1Lie == 'start' then
      r1JP = r1.nodes[2].p
    else
      r1JP = r1.nodes[#r1.nodes - 1].p
    end
    if util.isPointOnLeftSideOfLine(r1JP, jP1, jP2) then
      jCurr = -jCurr
    end
    local rot = util.getRotationBetweenVecs(jVec, jCurr)
    rotateJunctionQuat(jIdx, jCenter, rot)
    jct.isSidewalk = im.BoolPtr(false)
    jct.numLanesX = im.IntPtr(1)
    jct.numLanesY = im.IntPtr(1)
    jct.laneWidthX = im.FloatPtr(rOthers[1].r.nodes[1].widths[1][0])
    jct.laneWidthY = im.FloatPtr(rOthers[1].r.nodes[1].widths[1][0])
    updateJunctionAfterChange(jIdx)
    for i = 1, 3 do
      forbidden[jct.roads[i]] = true
    end
    for i = 7, #jct.roads do
      forbidden[jct.roads[i]] = true
    end
    createPerfectJoin(rOthers[1].r, roads[map[jct.roads[4]]], rOthers[1].lie, 'start')
    createPerfectJoin(rOthers[2].r, roads[map[jct.roads[5]]], rOthers[2].lie, 'start')
    createPerfectJoin(r1, roads[map[jct.roads[6]]], r1Lie, 'start')
    finaliseJunction(jIdx)
  elseif numOthers == 3 then                                                                        -- There are four roads, so fit a crossroads junction.
    local jCenter = nil
    if r1Lie == 'start' then
      jCenter = r1.nodes[1].p
    else
      jCenter = r1.nodes[#r1.nodes].p
    end
    addCrossroads(true)
    local jIdx = #junctions
    local jct = junctions[jIdx]
    translateJunction(jIdx, jCenter)
    local jVec = roads[map[jct.roads[5]]].nodes[1].p - roads[map[jct.roads[6]]].nodes[1].p
    local jCurr = nil
    if rOthers[1].lie == 'start' then
      if rOthers[2].lie == 'start' then
        jCurr = rOthers[2].r.nodes[2].p - rOthers[1].r.nodes[2].p
      else
        jCurr = rOthers[2].r.nodes[2].p - rOthers[1].r.nodes[#rOthers[1].r.nodes - 1].p
      end
    else
      if rOthers[2].lie == 'start' then
        jCurr = rOthers[2].r.nodes[#rOthers[2].r.nodes].p - rOthers[1].r.nodes[2].p
      else
        jCurr = rOthers[2].r.nodes[#rOthers[2].r.nodes].p - rOthers[1].r.nodes[#rOthers[1].r.nodes - 1].p
      end
    end
    local rot = util.getRotationBetweenVecs(jVec, jCurr)
    rotateJunctionQuat(jIdx, jCenter, rot)
    jct.isSidewalk = im.BoolPtr(false)
    jct.numLanesX = im.IntPtr(1)
    jct.numLanesY = im.IntPtr(1)
    jct.laneWidthX = im.FloatPtr(rOthers[1].r.nodes[1].widths[1][0])
    jct.laneWidthY = im.FloatPtr(rOthers[1].r.nodes[1].widths[1][0])
    updateJunctionAfterChange(jIdx)
    for i = 1, 4 do
      forbidden[jct.roads[i]] = true
    end
    for i = 9, #jct.roads do
      forbidden[jct.roads[i]] = true
    end
    createPerfectJoin(rOthers[1].r, roads[map[jct.roads[5]]], rOthers[1].lie, 'start')
    createPerfectJoin(rOthers[2].r, roads[map[jct.roads[6]]], rOthers[2].lie, 'start')
    createPerfectJoin(rOthers[3].r, roads[map[jct.roads[7]]], rOthers[3].lie, 'start')
    createPerfectJoin(r1, roads[map[jct.roads[8]]], r1Lie, 'start')
    finaliseJunction(jIdx)
  end
end

-- Chops all the given roads up to the given sphere (removes any nodes inside the sphere, and projects the corresponding start/end node onto the sphere).
local function chopRoadsToSphere(jNodes, jCen, jRad)
  local roads, map = roadMgr.roads, roadMgr.map
  for i = 1, #jNodes do
    local jNode = jNodes[i]
    local rName, rLie = jNode.rName, jNode.lie
    local road = roads[map[rName]]
    for j = 2, #road.nodes do
      local p1, p2 = road.nodes[j - 1].p, road.nodes[j].p
      local intersections = util.intersectionLineSegmentCircle(p1.x, p1.y, p2.x, p2.y, jCen.x, jCen.y, jRad)
      if intersections then
        if rLie == 'start' then
          for k = j - 2, 1, -1 do                                                                   -- Remove every node up to the sphere, from the start of the road.
            table.remove(road.nodes, k)
          end
          road.nodes[1].p.x = intersections[1][1]                                                   -- Project the first node onto the sphere.
          road.nodes[1].p.y = intersections[1][2]
          road.nodes[1].p.z = jCen.z
        else
          for k = #road.nodes, j + 1, -1 do                                                         -- Remove every node up to the sphere, from the end of the road.
            table.remove(road.nodes, k)
          end
          road.nodes[#road.nodes].p.x = intersections[1][1]                                         -- Project the last node onto the sphere.
          road.nodes[#road.nodes].p.y = intersections[1][2]
          road.nodes[#road.nodes].p.z = jCen.z
        end
        roadMgr.setDirty(road)                                                                      -- Update the road.
        break                                                                                       -- Do nothing more with this road.
      end
    end
    for j = #road.nodes - 1, 2, -1 do
      local p1, p2, p3 = road.nodes[j].p, road.nodes[j - 1].p, road.nodes[j + 1].p
      local d1, d2 = p1:squaredDistance(p2), p1:squaredDistance(p3)
      if min(d1, d2) < 25.0 and #road.nodes > 1 then
        table.remove(road.nodes, j)
      end
    end

    local startNode = road.nodes[1]
    local endNode = road.nodes[#road.nodes]

    for j = #road.nodes-1, 2, -1 do
      local p1= road.nodes[j].p
      local d1,d2 = p1:distance(startNode.p), p1:distance(endNode.p)
      if min(d1,d2) < 50.0 then
        table.remove(road.nodes, j)
      end
    end
  end
end

local function createAutoJctOverlay(p1, p2, p3, p4, laneWidth)
  local roads = roadMgr.roads
  local cProf = profileMgr.createOverlayProfile(laneWidth)
  local cRoad = roadMgr.createRoadFromProfile(cProf)
  cRoad.isOverlay = true
  cRoad.isJctRoad = false
  cRoad.isDrivable = false
  cRoad.displayName = im.ArrayChar(32, 'Jct Overlay')
  local rIdx = #roads + 1
  roads[rIdx] = cRoad
  --roadMgr.addNodeToRoad(rIdx, p1)
  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.25, 0.75))
  roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.5, 0.75))
  roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.75, 0.75))
  roadMgr.addNodeToRoad(rIdx, p3)
  --roadMgr.addNodeToRoad(rIdx, p4)
  roadMgr.setDirty(roads[rIdx])
end

local function createAutoJctOverlays(jNodes, laneWidth, jctCen)
  local jZ = jctCen.z
  local roads, map = roadMgr.roads, roadMgr.map
  local numJRoads = #jNodes
  for i = 1, numJRoads do
    local jNodeI = jNodes[i]
    local roadI, lieI = roads[map[jNodeI.rName]], jNodeI.lie
    local rDataI = roadI.renderData
    for j = i + 1, numJRoads do
      local jNodeJ = jNodes[j]
      local roadJ, lieJ = roads[map[jNodeJ.rName]], jNodeJ.lie
      local rDataJ = roadJ.renderData
      if lieI == 'start' then
        if lieJ == 'start' then                                                                     -- CASE:  start -> start.
          local p1, p2 = rDataI[min(#rDataI, 2)][1][7], rDataI[min(#rDataI, 1)][1][7]
          local p3, p4 = rDataJ[min(#rDataJ, 1)][-1][7], rDataJ[min(#rDataJ, 2)][-1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createAutoJctOverlay(p1, p2, p3, p4, laneWidth)
          local p1, p2 = rDataI[min(#rDataI, 2)][-1][7], rDataI[min(#rDataI, 1)][-1][7]
          local p3, p4 = rDataJ[min(#rDataJ, 1)][1][7], rDataJ[min(#rDataJ, 2)][1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createAutoJctOverlay(p1, p2, p3, p4, laneWidth)
        else                                                                                        -- CASE:  start -> end.
          local p1, p2 = rDataI[min(#rDataI, 2)][1][7], rDataI[min(#rDataI, 1)][1][7]
          local p3, p4 = rDataJ[max(1, #rDataJ)][1][7], rDataJ[max(1, #rDataJ - 1)][1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createAutoJctOverlay(p1, p2, p3, p4, laneWidth)
          local p1, p2 = rDataI[min(#rDataI, 2)][-1][7], rDataI[min(#rDataI, 1)][-1][7]
          local p3, p4 = rDataJ[max(1, #rDataJ)][-1][7], rDataJ[max(1, #rDataJ - 1)][-1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createAutoJctOverlay(p1, p2, p3, p4, laneWidth)
        end
      else
        if lieJ == 'start' then                                                                     -- CASE:  end -> start.
          local p1, p2 = rDataI[max(1, #rDataI - 1)][1][7], rDataI[max(1, #rDataI)][1][7]
          local p3, p4 = rDataJ[min(#rDataJ, 1)][1][7], rDataJ[min(#rDataJ, 2)][1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createAutoJctOverlay(p1, p2, p3, p4, laneWidth)
          local p1, p2 = rDataI[max(1, #rDataI - 1)][-1][7], rDataI[max(1, #rDataI)][-1][7]
          local p3, p4 = rDataJ[min(#rDataJ, 1)][-1][7], rDataJ[min(#rDataJ, 2)][-1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createAutoJctOverlay(p1, p2, p3, p4, laneWidth)
        else                                                                                        -- CASE:  end -> end.
          local p1, p2 = rDataI[max(1, #rDataI - 1)][1][7], rDataI[max(1, #rDataI)][1][7]
          local p3, p4 = rDataJ[max(1, #rDataJ)][-1][7], rDataJ[max(1, #rDataJ - 1)][-1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createAutoJctOverlay(p1, p2, p3, p4, laneWidth)
          local p1, p2 = rDataI[max(1, #rDataI - 1)][-1][7], rDataI[max(1, #rDataI)][-1][7]
          local p3, p4 = rDataJ[max(1, #rDataJ)][1][7], rDataJ[max(1, #rDataJ - 1)][1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createAutoJctOverlay(p1, p2, p3, p4, laneWidth)
        end
      end
    end
  end
  roadMgr.recomputeMap()
end

local function createLSystemJctOverlay(p1, p2, p3, p4, startWidth, endWidth)
  startWidth = startWidth/2
  endWidth = endWidth/2

  local roads = roadMgr.roads
  local cProf = profileMgr.createOverlayProfile(1)
  cProf[-1] = {
    type = 'road_lane',
    width = im.FloatPtr(1), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(false), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  local cRoad = roadMgr.createRoadFromProfile(cProf)
  cRoad.isOverlay = true
  cRoad.isJctRoad = false
  cRoad.isDrivable = false
  cRoad.displayName = im.ArrayChar(32, 'Jct Overlay')
  local rIdx = #roads + 1
  roads[rIdx] = cRoad

  roadMgr.addNodeToRoad(rIdx, p2)
  roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.25, 0.75))
  roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.5, 0.75))
  roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.75, 0.75))
  roadMgr.addNodeToRoad(rIdx, p3)

  local inc = (endWidth - startWidth) / (#roads[rIdx].nodes - 1)
  for i = 1, #roads[rIdx].nodes do
    local node = roads[rIdx].nodes[i]
    node.widths[1][0] = startWidth + inc * (i - 1)
    node.widths[-1][0] = startWidth + inc * (i - 1)
  end

  roadMgr.setDirty(roads[rIdx])
end


local function createLSystemJctOverlays(jNodes, jctCen)
  local jZ = jctCen.z
  local roads, map = roadMgr.roads, roadMgr.map
  local numJRoads = #jNodes
  for i = 1, numJRoads do
    local jNodeI = jNodes[i]
    local roadI, lieI = roads[map[jNodeI.rName]], jNodeI.lie
    local rDataI = roadI.renderData
    for j = i + 1, numJRoads do
      local jNodeJ = jNodes[j]
      local roadJ, lieJ = roads[map[jNodeJ.rName]], jNodeJ.lie
      local rDataJ = roadJ.renderData
      if lieI == 'start' then
        if lieJ == 'start' then                                                                     -- CASE:  start -> start.
          local nodeI = roadI.nodes[1]
          local nodeJ = roadJ.nodes[1]
          local laneWidthI = nodeI.widths[1][0]
          local laneWidthJ = nodeJ.widths[-1][0]
          local p1, p2 = rDataI[min(#rDataI, 2)][1][7], rDataI[min(#rDataI, 1)][1][7]
          local p3, p4 = rDataJ[min(#rDataJ, 1)][-1][7], rDataJ[min(#rDataJ, 2)][-1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createLSystemJctOverlay(p1, p2, p3, p4, laneWidthI, laneWidthJ)
          local laneWidthI = nodeI.widths[-1][0]
          local laneWidthJ = nodeJ.widths[1][0]
          local p1, p2 = rDataI[min(#rDataI, 2)][-1][7], rDataI[min(#rDataI, 1)][-1][7]
          local p3, p4 = rDataJ[min(#rDataJ, 1)][1][7], rDataJ[min(#rDataJ, 2)][1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createLSystemJctOverlay(p1, p2, p3, p4, laneWidthI, laneWidthJ)
        else                                                                                        -- CASE:  start -> end.
          local nodeI = roadI.nodes[1]
          local nodeJ = roadJ.nodes[#roadJ.nodes]
          local laneWidthI = nodeI.widths[1][0]
          local laneWidthJ = nodeJ.widths[1][0]
          local p1, p2 = rDataI[min(#rDataI, 2)][1][7], rDataI[min(#rDataI, 1)][1][7]
          local p3, p4 = rDataJ[max(1, #rDataJ)][1][7], rDataJ[max(1, #rDataJ - 1)][1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createLSystemJctOverlay(p1, p2, p3, p4, laneWidthI, laneWidthJ)
          local laneWidthI = nodeI.widths[-1][0]
          local laneWidthJ = nodeJ.widths[-1][0]
          local p1, p2 = rDataI[min(#rDataI, 2)][-1][7], rDataI[min(#rDataI, 1)][-1][7]
          local p3, p4 = rDataJ[max(1, #rDataJ)][-1][7], rDataJ[max(1, #rDataJ - 1)][-1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createLSystemJctOverlay(p1, p2, p3, p4, laneWidthI, laneWidthJ)
        end
      else
        if lieJ == 'start' then                                                                     -- CASE:  end -> start.
          local nodeI = roadI.nodes[#roadI.nodes]
          local nodeJ = roadJ.nodes[1]
          local laneWidthI = nodeI.widths[1][0]
          local laneWidthJ = nodeJ.widths[1][0]
          local p1, p2 = rDataI[max(1, #rDataI - 1)][1][7], rDataI[max(1, #rDataI)][1][7]
          local p3, p4 = rDataJ[min(#rDataJ, 1)][1][7], rDataJ[min(#rDataJ, 2)][1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createLSystemJctOverlay(p1, p2, p3, p4, laneWidthI, laneWidthJ)
          local laneWidthI = nodeI.widths[-1][0]*2
          local laneWidthJ = nodeJ.widths[-1][0]*2
          local p1, p2 = rDataI[max(1, #rDataI - 1)][-1][7], rDataI[max(1, #rDataI)][-1][7]
          local p3, p4 = rDataJ[min(#rDataJ, 1)][-1][7], rDataJ[min(#rDataJ, 2)][-1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createLSystemJctOverlay(p1, p2, p3, p4, laneWidthI, laneWidthJ)
        else                                                                                        -- CASE:  end -> end.
          local nodeI = roadI.nodes[#roadI.nodes]
          local nodeJ = roadJ.nodes[#roadJ.nodes]
          local laneWidthI = nodeI.widths[1][0]
          local laneWidthJ = nodeJ.widths[-1][0]
          local p1, p2 = rDataI[max(1, #rDataI - 1)][1][7], rDataI[max(1, #rDataI)][1][7]
          local p3, p4 = rDataJ[max(1, #rDataJ)][-1][7], rDataJ[max(1, #rDataJ - 1)][-1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createLSystemJctOverlay(p1, p2, p3, p4, laneWidthI, laneWidthJ)
          local laneWidthI = nodeI.widths[-1][0]
          local laneWidthJ = nodeJ.widths[1][0]
          local p1, p2 = rDataI[max(1, #rDataI - 1)][-1][7], rDataI[max(1, #rDataI)][-1][7]
          local p3, p4 = rDataJ[max(1, #rDataJ)][1][7], rDataJ[max(1, #rDataJ - 1)][1][7]
          p1.z, p2.z, p3.z, p4.z = jZ, jZ, jZ, jZ
          createLSystemJctOverlay(p1, p2, p3, p4, laneWidthI, laneWidthJ)
        end
      end
    end
  end
  roadMgr.recomputeMap()
end

local function distance2D(a, b)
  local x1, y1, x2, y2 = a.x, a.y, b.x, b.y
  return sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2)
end

-- Imports a road network from the L-System tool.
local function importRoadsFromLSystem(roadNetwork, intersections, DOI, margin, doTerraform)
  roadMgr.computeAllRoadRenderData()
  local roads, map = roadMgr.roads, roadMgr.map

  -- Create the Road Architect roads from each unique road in the hash table.
  for k, uniqueRoad in pairs(roadNetwork) do
    local nodesIn, roadType = uniqueRoad.splineNodes, uniqueRoad.type                                     -- TODO: road_type is not used currently.
      local nodes, widths = {}, {}
      local roadWidth = uniqueRoad.width
      for j = 1, #nodesIn do
        local n = nodesIn[j].pos
        nodes[j] = n
        widths[j] = roadWidth / 2
      end
      local profile = profileMgr.createProfileFromDecalData(1,1)                                   -- TODO: Currently assumes two-way, one lane per side.
      local newRoad = roadMgr.createRoadFromProfile(profile)
      newRoad.name = uniqueRoad.id
      local rIdx = #roads + 1
      roads[rIdx] = newRoad
      map[newRoad.name] = rIdx
      for j = 1, #nodes do
        roadMgr.addNodeToRoad(rIdx, nodes[j])
        newRoad.nodes[#newRoad.nodes].widths[-1] = im.FloatPtr(widths[j])                           -- TODO: widths are assumed as half widths, roads symmetric around center.
        newRoad.nodes[#newRoad.nodes].widths[1] = im.FloatPtr(widths[j])
      end
  end

  roadMgr.computeAllRoadRenderData()

  for nodeId, intersectionData in pairs(intersections) do
    local roadIds = intersectionData.connectedRoads
    local radius = intersectionData.radius
    local pos = intersectionData.pos
    local jNodes = {}
    for _,roadId in ipairs(roadIds) do
      local road = roadNetwork[roadId]
      local nS,nE = road.nodes[1],road.nodes[#road.nodes]
      if nodeId == nS.id then
        jNodes[#jNodes + 1] = { rName = roadId, lie = 'start' }
      elseif nodeId == nE.id then
        jNodes[#jNodes + 1] = { rName = roadId, lie = 'end' }
      end
    end

    chopRoadsToSphere(jNodes, pos, radius)
    roadMgr.computeAllRoadRenderData()
    createLSystemJctOverlays(jNodes, pos)
  end

  if doTerraform then
    -- Terraform to the refined L-System road network.
    local allRoadsGroup = { list = {} }
    local ctr = 1
    for i = 1, #roadMgr.roads do
      local tR = roadMgr.roads[i]
      for j = 1, #tR.nodes do
        allRoadsGroup.list[ctr] = { r = tR.name, n = j }
        ctr = ctr + 1
      end
    end
    terra.terraformMultiRoads(DOI, margin, allRoadsGroup)
  end
end

-- Public interface.
M.junctions =                                             junctions

M.addPedXJunction =                                       addPedXJunction
M.addCrossroads =                                         addCrossroads
M.addTJunction =                                          addTJunction
M.addYJunction =                                          addYJunction
M.addRoundaboutJunction =                                 addRoundaboutJunction
M.addRuralUrbanTransJunction =                            addRuralUrbanTransJunction
M.addUrbanMergeJunction =                                 addUrbanMergeJunction
M.addUrbanSeparatorJunction =                             addUrbanSeparatorJunction
M.addHighwayMergeJunction =                               addHighwayMergeJunction
M.addHighwayUrbanTransJunction =                          addHighwayUrbanTransJunction
M.addHighwaySeparatorJunction =                           addHighwaySeparatorJunction
M.addShoulderFadeJunction =                               addShoulderFadeJunction
M.addHighwaySlipJunction =                                addHighwaySlipJunction

M.updateJunctionAfterChange =                             updateJunctionAfterChange
M.removeJunction =                                        removeJunction
M.clearAllJunctions =                                     clearAllJunctions
M.getJunctionCentroid =                                   getJunctionCentroid
M.computeInitRot =                                        computeInitRot
M.translateJunction =                                     translateJunction
M.rotateJunction =                                        rotateJunction
M.rotateJunctionQuat =                                    rotateJunctionQuat
M.goToJunction =                                          goToJunction
M.setAllMeshProperty =                                    setAllMeshProperty
M.updateJunctionsAfterRoadRemove =                        updateJunctionsAfterRoadRemove

M.updateJunctionCondition =                               updateJunctionCondition

M.finaliseJunction =                                      finaliseJunction

M.copyJunction =                                          copyJunction

M.serialiseJct =                                          serialiseJct
M.deserialiseJct =                                        deserialiseJct

M.saveJunction =                                          saveJunction
M.loadJunction =                                          loadJunction

M.chopRoadsToSphere =                                     chopRoadsToSphere
M.createAutoJctOverlays =                                 createAutoJctOverlays

M.importRoadsFromLSystem =                                importRoadsFromLSystem

return M