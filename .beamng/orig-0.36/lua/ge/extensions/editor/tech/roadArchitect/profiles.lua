-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local defaultProfileClass = 'urban'                                                                 -- The default profile class/type.

local lightTreadMaterial = 'm_tread_marks_clean'                                                    -- The 'light tread/wear' material.
local heavyTreadMaterial = 'road_rubber_double'                                                     -- The 'heavy tread/wear' material.

local damagedAsphaltMaterial1 = 'm_asphalt_damaged_01'                                              -- The 'damaged asphalt' material
local damagedAsphaltMaterial2 = 'm_asphalt_damaged_02'                                              -- The 'damaged asphalt' material

local crackMaterial = 'm_asphalt_cracks_02'                                                         -- The 'cracks' material (decal road).
local crackMaterialDecalRoad1 = 'repair1'                                                           -- The crack patch materials (decal road type).
local crackMaterialDecalRoad2 = 'repair2'
local decalRoadPatchMaterial = 'road_patches1'                                                      -- Patches (decal road type).
local dirtTrackMaterial = 'm_dirt_variation_04'                                                     -- The base dirt road overlay (decal road type).

local decalPatchMaterial = 'asphalt_patches'                                                        -- The tiled decal containing road repair patches (decal type).
local potholePatchMaterial = 'AsphaltRoad_damage_sml_decal_01'                                      -- Pothole patches (decal type).

local defaultCenterlineMaterial = 'm_line_yellow_double_discontinue'                                -- Default materials for each type of road paint line.
local defaultEdgeMaterial = 'm_line_white'
local defaultLaneMarkingsMaterial = 'm_line_yellow_discontinue'
local defaultEndStopMat = 'm_line_white'

local edgeBlendWidth = 2.0
local edgeBlendLatOffset = 0.5

local defaultEdgeBlendMaterial = 'm_road_asphalt_edge'                                              -- The edge blending material for asphalt roads.

local defaultGutterMat = 'gutter1'                                                                  -- The default material used for road gutters.

local templateFilepaths =                                                                           -- A table of paths to the prefab road template data files.
  {
    'roadArchitect/profiles/Rural_2W_1L.json',
    'roadArchitect/profiles/Rural_2W_2L.json',
    'roadArchitect/profiles/Rural_1W_1L.json',
    'roadArchitect/profiles/Rural_1W_2L.json',

    'roadArchitect/profiles/Urban_2W_1L.json',
    'roadArchitect/profiles/Urban_2W_2L.json',
    'roadArchitect/profiles/Urban_1W_1L.json',
    'roadArchitect/profiles/Urban_2W_2L_Lamps.json',

    'roadArchitect/profiles/Hwy_2W_1L.json',
    'roadArchitect/profiles/Hwy_2W_2L.json',
    'roadArchitect/profiles/Hwy_1W_1L.json',
    'roadArchitect/profiles/Hwy_1W_2L.json',
    'roadArchitect/profiles/Hwy_2W_2L_Lamps+Barriers.json',

    'roadArchitect/profiles/Tracks_Type1.json',
    'roadArchitect/profiles/Tracks_Type2.json',
    'roadArchitect/profiles/Tracks_Type3.json',
    'roadArchitect/profiles/Tracks_Type4.json',
    'roadArchitect/profiles/Tracks_Type5.json',

    'roadArchitect/profiles/Barrier_Only.json',
    'roadArchitect/profiles/Lamps_Only.json'
  }

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- External modules used.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.

-- Private constants.
local im = ui_imgui
local min, max, floor, random = math.min, math.max, math.floor, math.random
local pView = vec3(-24.02901173, 0.8800626756, 1007.733115)
local pRot = quat(0.1090148618, -0.114031973, 0.7137778829, 0.6823735069)
local numLoadedProfiles = 1

-- Module state.
local profiles = {}                                                                                 -- The array of profiles currently present in the editor.
local isCustom = false
local isSidewalk = false
local oldPos, oldRot = nil, nil                                                                     -- The previous camera pose, before going to profile view.
local isInProfileView = false                                                                       -- A flag which indicates if the camera is in profile view, or not.


-- Gets the minimum and maximum lane key values, from a given collection of lanes.
local function getMinMaxLaneKeys(profile)
  local l, u = 100, -100
  for i = -20, 20 do
    if profile[i] then
      l, u = min(l, i), max(u, i)
    end
  end
  return l, u
end

-- Compute the road sections from a given road.
-- [This is an array of contiguous sections of road lanes, with start and end lane indices].
local function computeSectionsByType(profile, laneType1, laneType2)
  local laneType2 = laneType2 or nil
  local sections, sCtr, s = {}, 1, nil
  for i = -20, 20 do                                                                                -- Iterate over all possible lanes, from left to right.
    if i ~= 0 then
      local lane = profile[i]
      if lane and (lane.type == laneType1 or (laneType2 and lane.type == laneType2)) then
        if not s then
          s = i                                                                                     -- If we find a lane of the given type, then start a section.
        end
      else
        if s then
          local e = i - 1                                                                           -- If we have started, and find a road not of the given type, then end the section.
          if e == 0 then
            e = -1
          end
          sections[sCtr] = {
            s = s, e = e,
            isOneWay = s * e > 0,                                                                   -- Determine if this section is a one-way road or not.
            lanesL = min(0, e) - min(0, s),                                                         -- Compute the number of left and right lanes in this section.
            lanesR = max(0, e) - max(0, s) }
          sCtr = sCtr + 1
          s = nil
        end
      end
    end
  end
  return sections
end

-- Removes the layer with the given index.
local function removeLayer(p, i) table.remove(p.layers, i) end

-- Adds the road centerline to the given road, which is based on the stored styling parameters.
local function addCenterline(p, isPaint)
  local rSections = computeSectionsByType(p, 'road_lane')
  for i = 1, #rSections do
    if not rSections[i].isOneWay then
      p.layers[#p.layers + 1] =
        {
          name = im.ArrayChar(32, 'Centerline'),
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(not isPaint),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(1),
          laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
          lane = im.IntPtr(1), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
          width = im.FloatPtr(0.4),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = p.centerlineMat,
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
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
        }
    end
  end
end

-- Adds the road edge lines to the given road, which are based on the stored styling parameters.
local function addEdgeLines(p, latOffL, latOffR, isLeft, isRight, isPaint)
  local dummyLaneIdx = -1
  if p[1] then dummyLaneIdx = 1 end

  local rSections = computeSectionsByType(p, 'road_lane')

  -- Create all the left-edge layers.
  if isLeft then
    for i = 1, #rSections do
      local lIdx = rSections[i].s
      p.layers[#p.layers + 1] =
        {
          name = im.ArrayChar(32, 'Edge L ' .. tostring(i)),
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(not isPaint),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(1),
          laneMin = im.IntPtr(dummyLaneIdx), laneMax = im.IntPtr(dummyLaneIdx),
          lane = im.IntPtr(lIdx), isLeft = im.BoolPtr(true), off = im.FloatPtr(latOffL),
          width = im.FloatPtr(0.25),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = p.edgeMatL,
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
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
        }
    end
  end

  -- Create all the right-edge layers.
  if isRight then
    for i = 1, #rSections do
      local lIdx = rSections[i].e
      p.layers[#p.layers + 1] =
        {
          name = im.ArrayChar(32, 'Edge R ' .. tostring(i)),
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(not isPaint),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(1),
          laneMin = im.IntPtr(dummyLaneIdx), laneMax = im.IntPtr(dummyLaneIdx),
          lane = im.IntPtr(lIdx), isLeft = im.BoolPtr(false), off = im.FloatPtr(-latOffR),
          width = im.FloatPtr(0.25),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(5),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = p.edgeMatR,
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
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
        }
    end
  end
end

-- Adds the lane division lines to the given road, which are based on the stored styling parameters.
local function addLaneDivisionLines(p, isPaint)
  local dummyLaneIdx = -1
  if p[1] then dummyLaneIdx = 1 end

  local rSections = computeSectionsByType(p, 'road_lane')
  for i = 1, #rSections do
    for j = rSections[i].s + 1, rSections[i].e do

      -- Create the layer.
      if j ~= 0 and j ~= 1 then
        p.layers[#p.layers + 1] =
          {
            name = im.ArrayChar(32, 'Lane div ' .. tostring(i) .. '-' .. tostring(j)),
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(not isPaint),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(1),
            laneMin = im.IntPtr(dummyLaneIdx), laneMax = im.IntPtr(dummyLaneIdx),
            lane = im.IntPtr(j), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
            width = im.FloatPtr(0.2),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = p.laneMarkingsMat,
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
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
          }
      end
    end
  end
end

-- Adds the lane end/stop lines to the given road, which is based on the stored styling parameters.
local function addEndLine(p, isStart, isEnd)
  local rSections = computeSectionsByType(p, 'road_lane')

  -- Create all the left-edge layers.
  for i = 1, #rSections do
    local s, e = rSections[i].s, rSections[i].e

    if isEnd then
      if s < 0 then
        p.layers[#p.layers + 1] =
          {
            name = im.ArrayChar(32, 'Stop line A' .. tostring(i)),
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(true),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(2),
            laneMin = im.IntPtr(s), laneMax = im.IntPtr(min(-1, e)),
            lane = im.IntPtr(rSections[i].s), isLeft = im.BoolPtr(true), off = im.FloatPtr(1.0),
            width = im.FloatPtr(0.4),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = p.endStopMatE,
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
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
          }
      end
    end
    if isStart then
      if e > 0 then
        p.layers[#p.layers + 1] =
          {
            name = im.ArrayChar(32, 'Stop line B' .. tostring(i)),
            doNotDelete = im.BoolPtr(true),
            isReverse = im.BoolPtr(false),
            isPaint = im.BoolPtr(true),
            isDisplay = im.BoolPtr(false),
            type = im.IntPtr(2),
            laneMin = im.IntPtr(max(1, s)), laneMax = im.IntPtr(e),
            lane = im.IntPtr(rSections[i].s), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
            width = im.FloatPtr(0.4),
            isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
            texLen = im.FloatPtr(5),
            fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
            mat = p.endStopMatS,
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
            boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
          }
      end
    end
  end
end

-- Automatically adds paint line marking layers to the given profile (used for junctions).
local function autoPaintLines(prof)
  prof.layers = {}
  addCenterline(prof, prof.continueLinesToEnd[0])
  addEdgeLines(prof, prof.edgeLineGapL[0], prof.edgeLineGapR[0], prof.conditionEdgesL[0], prof.conditionEdgesR[0], prof.continueLinesToEnd[0])
  addEndLine(prof, false, prof.conditionEndStopE[0])
  addLaneDivisionLines(prof, prof.continueLinesToEnd[0])
end

-- Automatically adds edge blending layers to the given profile (used for junctions).
local function autoEdgeBlending(prof, isLeft, isRight, mat)
  local rSections = computeSectionsByType(prof, 'road_lane', 'shoulder')

  -- Create all the left-edge layers.
  for i = 1, #rSections do
    local s, e = rSections[i].s, rSections[i].e
    local isLeftTaken = prof[s - 1] and prof[s - 1].type ~= 'island'
    local isRightTaken = prof[e + 1] and prof[e + 1].type ~= 'island'
    if isLeft and not isLeftTaken then
      prof.layers[#prof.layers + 1] =
        {
          name = im.ArrayChar(32, 'Edge Blend L ' .. tostring(i)),
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(true),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(1),
          laneMin = im.IntPtr(s), laneMax = im.IntPtr(s),
          lane = im.IntPtr(s), isLeft = im.BoolPtr(true), off = im.FloatPtr(-edgeBlendLatOffset),
          width = im.FloatPtr(edgeBlendWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(18),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = mat or defaultEdgeBlendMaterial,
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
    end
    if isRight and not isRightTaken then
      prof.layers[#prof.layers + 1] =
        {
          name = im.ArrayChar(32, 'Edge Blend R ' .. tostring(i)),
          doNotDelete = im.BoolPtr(true),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(1),
          laneMin = im.IntPtr(e), laneMax = im.IntPtr(e),
          lane = im.IntPtr(e), isLeft = im.BoolPtr(false), off = im.FloatPtr(edgeBlendLatOffset),
          width = im.FloatPtr(edgeBlendWidth),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(18),
          fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
          mat = mat or defaultEdgeBlendMaterial,
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
    end
  end
end

-- Adds light tread to every road lane in the profile, across the full length of the road.
local function addLightTreadToLanes(p, fadeS, fadeE, lMin, lMax)
  for i = lMin, lMax do
    if p[i] and (p[i].type == 'road_lane' or p[i].type == 'shoulder') then
      p.layers[#p.layers + 1] =
        {
          name = im.ArrayChar(32, 'Light tread ' .. tostring(i)),
          isHidden = true,
          doNotDelete = im.BoolPtr(false),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(0),
          laneMin = im.IntPtr(i), laneMax = im.IntPtr(i),
          lane = im.IntPtr(i), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
          width = im.FloatPtr(3.5),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(36),
          fadeS = im.FloatPtr(fadeS), fadeE = im.FloatPtr(fadeE),
          mat = lightTreadMaterial,
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
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
        }
    end
  end
end

-- Adds heavy tread to every road lane in the profile.
local function addHeavyTreadToLanes(p, fadeS, fadeE, lMin, lMax)
  for i = lMin, lMax do
    if p[i] and (p[i].type == 'road_lane' or p[i].type == 'shoulder') then
      p.layers[#p.layers + 1] =
        {
          name = im.ArrayChar(32, 'Heavy tread ' .. tostring(i)),
          isHidden = true,
          doNotDelete = im.BoolPtr(false),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(0),
          laneMin = im.IntPtr(i), laneMax = im.IntPtr(i),
          lane = im.IntPtr(i), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
          width = im.FloatPtr(3.5),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(36),
          fadeS = im.FloatPtr(fadeS), fadeE = im.FloatPtr(fadeE),
          mat = heavyTreadMaterial,
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
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
        }
    end
  end
end

-- Adds dirt-road tyre tracks to every lane in the profile, across the full length of the road.
local function addDirtTracksToLanes(p, fadeS, fadeE)
  for i = -20, 20 do
    if p[i] and (p[i].type == 'road_lane' or p[i].type == 'shoulder') then
      p.layers[#p.layers + 1] =
        {
          name = im.ArrayChar(32, 'Dirt track ' .. tostring(i)),
          isHidden = true,
          doNotDelete = im.BoolPtr(false),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(0),
          laneMin = im.IntPtr(i), laneMax = im.IntPtr(i),
          lane = im.IntPtr(i), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
          width = im.FloatPtr(0.0),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(36),
          fadeS = im.FloatPtr(fadeS), fadeE = im.FloatPtr(fadeE),
          mat = p.dirtMat,
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
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
        }
    end
  end
end

-- Adds damaged asphalt to the given lane of the given profile, across the full road.
local function addDamagedLanesFull(p, fadeS, fadeE, lMin, lMax)
  local mat = damagedAsphaltMaterial1
  for i = lMin, lMax do
    if p[i] and (p[i].type == 'road_lane' or p[i].type == 'shoulder') then
      p.layers[#p.layers + 1] =
        {
          name = im.ArrayChar(32, 'Damage overlay ' .. tostring(i)),
          isHidden = true,
          doNotDelete = im.BoolPtr(false),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(0),
          laneMin = im.IntPtr(i), laneMax = im.IntPtr(i),
          lane = im.IntPtr(i), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
          width = im.FloatPtr(0.0),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(26),
          fadeS = im.FloatPtr(fadeS), fadeE = im.FloatPtr(fadeE),
          mat = mat,
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
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
        }
    end

    -- Cycle the material.
    if mat == damagedAsphaltMaterial1 then
      mat = damagedAsphaltMaterial2
    elseif mat == damagedAsphaltMaterial2 then
      mat = damagedAsphaltMaterial1
    end
  end
end

-- Adds damaged asphalt to the given lane of the given profile, in some [n1, n2] node interval.
local function addDamagedLanesPart(p, n1, n2, lIdx, fadeS, fadeE)
  p.layers[#p.layers + 1] =
    {
      name = im.ArrayChar(32, 'Damage overlay'),
      isHidden = true,
      doNotDelete = im.BoolPtr(false),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(0),
      laneMin = im.IntPtr(lIdx), laneMax = im.IntPtr(lIdx),
      lane = im.IntPtr(lIdx), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
      width = im.FloatPtr(8.0),
      isSpanLong = im.BoolPtr(false), nMin = im.IntPtr(n1), nMax = im.IntPtr(n2),
      texLen = im.FloatPtr(26),
      fadeS = im.FloatPtr(fadeS), fadeE = im.FloatPtr(fadeE),
      mat = decalRoadPatchMaterial,
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
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
    }
end

-- Adds crack overlays to the given lane of the given profile, in some [n1, n2] node interval.
local function addCrackedLanesPart(p, n1, n2, lIdx, isLeft, latOff, fadeS, fadeE, type)
  local tile = crackMaterialDecalRoad1
  if type > 1 then
    tile = crackMaterialDecalRoad2
  end
  p.layers[#p.layers + 1] =
    {
      name = im.ArrayChar(32, 'Crack part overlay'),
      isHidden = true,
      doNotDelete = im.BoolPtr(false),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(1),
      laneMin = im.IntPtr(1), laneMax = im.IntPtr(1),
      lane = im.IntPtr(lIdx), isLeft = im.BoolPtr(isLeft), off = im.FloatPtr(latOff),
      width = im.FloatPtr(4.0),
      isSpanLong = im.BoolPtr(false), nMin = im.IntPtr(n1), nMax = im.IntPtr(n2),
      texLen = im.FloatPtr(32),
      fadeS = im.FloatPtr(fadeS), fadeE = im.FloatPtr(fadeE),
      mat = tile,
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
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
    }
end

-- Adds a single crack patch overlay to the given lane of the given profile.
local function addSingleDecalPatch(p, q, lMin, isLeft, latOff, size)
  p.layers[#p.layers + 1] =
    {
      name = im.ArrayChar(32, 'Damage Patch'),
      isHidden = true,
      doNotDelete = im.BoolPtr(false),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(3),
      laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(isLeft), off = im.FloatPtr(q),
      width = im.FloatPtr(5.0),
      isSpanLong = im.BoolPtr(false), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0), fadeE = im.FloatPtr(0),
      mat = decalPatchMaterial,
      rot = im.IntPtr(util.randomInRange(1, 4) - 1),
      pos = im.FloatPtr(latOff), size = im.FloatPtr(size),
      numRows = im.IntPtr(4), numCols = im.IntPtr(2),
      frame = im.IntPtr(util.randomInRange(1, 8) - 1),

      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
    }
end

-- Adds a single crack patch overlay to the given lane of the given profile.
local function addSingleDecalPatchPothole(p, q, lMin, isLeft, latOff, size)
  p.layers[#p.layers + 1] =
    {
      name = im.ArrayChar(32, 'Pothole Patch'),
      isHidden = true,
      doNotDelete = im.BoolPtr(false),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(3),
      laneMin = im.IntPtr(lMin), laneMax = im.IntPtr(lMin),
      lane = im.IntPtr(lMin), isLeft = im.BoolPtr(isLeft), off = im.FloatPtr(q),
      width = im.FloatPtr(5.0),
      isSpanLong = im.BoolPtr(false), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0), fadeE = im.FloatPtr(0),
      mat = potholePatchMaterial,
      rot = im.IntPtr(util.randomInRange(1, 4) - 1),
      pos = im.FloatPtr(latOff), size = im.FloatPtr(size),
      numRows = im.IntPtr(2), numCols = im.IntPtr(2),
      frame = im.IntPtr(util.randomInRange(1, 4) - 1),

      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
    }
end

-- Adds a full crack decal to every road lane in the profile.
local function addCracksLanes(p, fadeS, fadeE, lMin, lMax)
  for i = lMin, lMax do
    if p[i] and p[i].type == 'road_lane' then
      p.layers[#p.layers + 1] =
        {
          name = im.ArrayChar(32, 'Crack overlay ' .. tostring(i)),
          isHidden = true,
          doNotDelete = im.BoolPtr(false),
          isReverse = im.BoolPtr(false),
          isPaint = im.BoolPtr(false),
          isDisplay = im.BoolPtr(false),
          type = im.IntPtr(0),
          laneMin = im.IntPtr(i), laneMax = im.IntPtr(i),
          lane = im.IntPtr(i), isLeft = im.BoolPtr(true), off = im.FloatPtr(0.0),
          width = im.FloatPtr(0.0),
          isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
          texLen = im.FloatPtr(36),
          fadeS = im.FloatPtr(fadeS), fadeE = im.FloatPtr(fadeE),
          mat = crackMaterial,
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
          boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
        }
    end
  end
end

-- Deep copies the layer information from a profile.
local function copyLayers(prof)
  local lNew, layers = {}, prof.layers
  for i = 1, #layers do
    local layer = layers[i]
    lNew[i] =
      {
        name = im.ArrayChar(32, ffi.string(layer.name)),
        isHidden = layer.isHidden,
        doNotDelete = im.BoolPtr(layer.doNotDelete[0]),
        isReverse = im.BoolPtr(layer.isReverse[0]),
        isPaint = im.BoolPtr(layer.isPaint[0]),
        isDisplay = im.BoolPtr(layer.isDisplay[0]),
        type = im.IntPtr(layer.type[0]),
        laneMin = im.IntPtr(layer.laneMin[0]), laneMax = im.IntPtr(layer.laneMax[0]),
        lane = im.IntPtr(layer.lane[0]), isLeft = im.BoolPtr(layer.isLeft[0]), off = im.FloatPtr(layer.off[0]),
        width = im.FloatPtr(layer.width[0]),
        isSpanLong = im.BoolPtr(layer.isSpanLong[0]), nMin = im.IntPtr(layer.nMin[0]), nMax = im.IntPtr(layer.nMax[0]),
        texLen = im.FloatPtr(layer.texLen[0]),
        fadeS = im.FloatPtr(layer.fadeS[0]), fadeE = im.FloatPtr(layer.fadeE[0]),
        mat = tostring(layer.mat),
        rot = im.IntPtr(layer.rot[0]),
        pos = im.FloatPtr(layer.pos[0]), size = im.FloatPtr(layer.size[0]),
        numRows = im.IntPtr(layer.numRows[0]), numCols = im.IntPtr(layer.numCols[0]),
        frame = im.IntPtr(layer.frame[0]),

        vertOffset = im.FloatPtr(layer.vertOffset or 0.0),
        latOffset = im.FloatPtr(layer.latOffset or 0.0),
        spacing = im.FloatPtr(layer.spacing or 5.0),
        jitter = im.FloatPtr(layer.jitter or 0.0),
        useWorldZ = im.BoolPtr(layer.useWorldZ or false),
        matDisplay = layer.matDisplay,
        extentsL = layer.extentsL, extentsW = layer.extentsW, extentsH = layer.extentsH,
        boxXLeft = layer.boxXLeft, boxXRight = layer.boxXRight,
        boxYLeft = layer.boxYLeft, boxYRight = layer.boxYRight,
        boxZLeft = layer.boxZLeft, boxZRight = layer.boxZRight
      }
  end
  return lNew
end

-- Gets the current profile index from a given profile name.
local function getIdFromName(name)
  for i = 1, #profiles do
    if name == profiles[i].name then return i end
  end
  return nil
end

-- Gets the profile with the given name.
local function getProfileFromName(name) return profiles[getIdFromName(name)] end

-- Updates the module lane flags (used for UI - when displaying which static meshes are present).
local function updateLaneFlags(p)
  isCustom = false
  isSidewalk = false
  for i = -20, 20 do
    local lane = p[i]
    if lane then
      local t = lane.type
      isCustom = isCustom or t == 'mesh'
      isSidewalk = isSidewalk or t == 'sidewalk'
    end
  end
end

-- Copies a profile template to an existing profile.
local function updateToNewTemplate(road, templateName)
  local template, profile = getProfileFromName(templateName), road.profile
  local originalName = profile.name
  table.clear(profile)

  profile.name = originalName
  profile.isDeletable = true
  profile.class = template.class

  profile.styleType = im.IntPtr(template.styleType[0] or 0)
  profile.condition = im.FloatPtr(template.condition[0] or 0.2)
  profile.conditionSeed = im.IntPtr(template.conditionSeed[0] or 41226)
  profile.numPatches = im.IntPtr(template.numPatches[0] or 10)
  profile.numPotholes = im.IntPtr(template.numPotholes[0] or 0)
  profile.conditionCenterline = im.BoolPtr(template.conditionCenterline[0])
  profile.conditionEdgesL = im.BoolPtr(template.conditionEdgesL[0])
  profile.conditionEdgesR = im.BoolPtr(template.conditionEdgesR[0])
  profile.conditionLaneMarkings = im.BoolPtr(template.conditionLaneMarkings[0])
  profile.conditionEndStopS = im.BoolPtr(template.conditionEndStopS[0])
  profile.conditionEndStopE = im.BoolPtr(template.conditionEndStopE[0])
  profile.edgeLineGapL = im.FloatPtr(template.edgeLineGapL[0] or 0.25)
  profile.edgeLineGapR = im.FloatPtr(template.edgeLineGapR[0] or 0.25)
  profile.centerlineMat = template.centerlineMat or defaultCenterlineMaterial
  profile.edgeMatL = template.edgeMatL or defaultEdgeMaterial
  profile.edgeMatR = template.edgeMatR or defaultEdgeMaterial
  profile.laneMarkingsMat = template.laneMarkingsMat or defaultLaneMarkingsMaterial
  profile.endStopMatL = template.endStopMatL or defaultEndStopMat
  profile.endStopMatR = template.endStopMatR or defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(template.isEdgeBlendL[0])
  profile.isEdgeBlendR = im.BoolPtr(template.isEdgeBlendR[0])
  profile.isShowEdgeBlend = im.BoolPtr(template.isShowEdgeBlend[0])
  profile.blendLeftMat = template.blendLeftMat or defaultEdgeBlendMaterial
  profile.blendRightMat = template.blendRightMat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(template.blendLeftWidth[0] or 1.0)
  profile.blendRightWidth = im.FloatPtr(template.blendRightWidth[0] or 1.0)

  profile.isStopDecalS = im.BoolPtr(template.isStopDecalS[0])
  profile.isStopDecalE = im.BoolPtr(template.isStopDecalE[0])
  profile.stopGapS = im.FloatPtr(template.stopGapS[0])
  profile.stopGapE = im.FloatPtr(template.stopGapE[0])
  profile.continueLinesToEnd = im.BoolPtr(template.continueLinesToEnd[0])

  profile.dirtMat = template.dirtMat or dirtTrackMaterial

  profile.isGutter = im.BoolPtr(template.isGutter[0])
  profile.gutterMat = template.gutterMat or defaultGutterMat
  profile.gutterMargin = im.FloatPtr(template.gutterMargin[0] or 0.02)
  profile.gutterWidth = im.FloatPtr(template.gutterWidth[0] or 0.2)
  profile.isGutterShow = im.BoolPtr(template.isGutterShow[0])

  profile.fadeS = im.FloatPtr(template.fadeS[0] or 3.0)
  profile.fadeE = im.FloatPtr(template.fadeE[0] or 3.0)

  profile.isAutoBanking = im.BoolPtr(template.isAutoBanking[0])
  profile.autoBankingFactor = im.FloatPtr(template.autoBankingFactor[0] or 1.0)
  profile.isExtraWidth = im.BoolPtr(template.isExtraWidth[0] or false)

  for i = -20, 20 do
    local lane = template[i]
    if lane then
      profile[i] = {
        type = lane.type,
        width = im.FloatPtr(lane.width[0]), heightL = im.FloatPtr(lane.heightL[0]), heightR = im.FloatPtr(lane.heightR[0]),
        isLeftSide = im.BoolPtr(lane.isLeftSide[0]), cornerDrop = im.FloatPtr(lane.cornerDrop[0]), vStart = im.IntPtr(lane.vStart[0]),
        kerbWidth = im.FloatPtr(lane.kerbWidth[0]), cornerLatOff = im.FloatPtr(lane.cornerLatOff[0]) }
    end
  end

  profile.layers = copyLayers(template)

  updateLaneFlags(profile)
end

-- Removes any layers which no longer relate to existing lanes.
-- [This is called after a profile lane is removed].
local function removeInvalidLayers(p)
  local layers = p.layers
  for i = #layers, 1, -1 do
    local lay = layers[i]
    if not p[lay.lane[0]] then
      table.remove(layers, i)
    end
  end
end

-- Raise/lower the priority of the given layer, in the given profile.
local function layerChangePriority(p, i, dir)
  local j = i + 1
  if dir == 'raise' then
    j = i - 1
  end
  local layer = p.layers[i]
  local iOld =
    {
      name = im.ArrayChar(32, ffi.string(layer.name)),
      isHidden = layer.isHidden,
      doNotDelete = im.BoolPtr(layer.doNotDelete[0]),
      isReverse = im.BoolPtr(layer.isReverse[0]),
      isPaint = im.BoolPtr(layer.isPaint[0]),
      isDisplay = im.BoolPtr(layer.isDisplay[0]),
      type = im.IntPtr(layer.type[0]),
      laneMin = im.IntPtr(layer.laneMin[0]), laneMax = im.IntPtr(layer.laneMax[0]),
      lane = im.IntPtr(layer.lane[0]), isLeft = im.BoolPtr(layer.isLeft[0]), off = im.FloatPtr(layer.off[0]),
      width = im.FloatPtr(layer.width[0]),
      isSpanLong = im.BoolPtr(layer.isSpanLong[0]), nMin = im.IntPtr(layer.nMin[0]), nMax = im.IntPtr(layer.nMax[0]),
      texLen = im.FloatPtr(layer.texLen[0]),
      fadeS = im.FloatPtr(layer.fadeS[0]), fadeE = im.FloatPtr(layer.fadeE[0]),
      mat = tostring(layer.mat),
      rot = im.IntPtr(layer.rot[0]),
      pos = im.FloatPtr(layer.pos[0]), size = im.FloatPtr(layer.size[0]),
      numRows = im.IntPtr(layer.numRows[0]), numCols = im.IntPtr(layer.numCols[0]),
      frame = im.IntPtr(layer.frame[0]),

      vertOffset = im.FloatPtr(layer.vertOffset or 0.0),
      latOffset = im.FloatPtr(layer.latOffset or 0.0),
      spacing = im.FloatPtr(layer.spacing or 5.0),
      jitter = im.FloatPtr(layer.jitter or 0.0),
      useWorldZ = im.BoolPtr(layer.useWorldZ or false),
      matDisplay = layer.matDisplay,
      extentsL = layer.extentsL, extentsW = layer.extentsW, extentsH = layer.extentsH,
      boxXLeft = layer.boxXLeft, boxXRight = layer.boxXRight,
      boxYLeft = layer.boxYLeft, boxYRight = layer.boxYRight,
      boxZLeft = layer.boxZLeft, boxZRight = layer.boxZRight
    }

  p.layers[i] = p.layers[j]
  p.layers[j] = iOld
end

-- Adds a new layer above/below the given layer index, in the given profile.
local function addLayer(p, i, dir)
  local cenLaneIdx, isLeft = -1, false
  if p[1] then
    cenLaneIdx, isLeft = 1, true
  end
  local lNew =
    {
      name = im.ArrayChar(32, 'new layer'),
      isHidden = false,
      doNotDelete = im.BoolPtr(true),
      isReverse = im.BoolPtr(false),
      isPaint = im.BoolPtr(false),
      isDisplay = im.BoolPtr(false),
      type = im.IntPtr(0),
      laneMin = im.IntPtr(cenLaneIdx), laneMax = im.IntPtr(cenLaneIdx),
      lane = im.IntPtr(cenLaneIdx), isLeft = im.BoolPtr(isLeft), off = im.FloatPtr(0.0),
      width = im.FloatPtr(1.0),
      isSpanLong = im.BoolPtr(true), nMin = im.IntPtr(1), nMax = im.IntPtr(1),
      texLen = im.FloatPtr(5),
      fadeS = im.FloatPtr(0.0), fadeE = im.FloatPtr(0.0),
      mat = lightTreadMaterial,
      rot = im.IntPtr(0),
      pos = im.FloatPtr(0.0), size = im.FloatPtr(3.0),
      numRows = im.IntPtr(1), numCols = im.IntPtr(1),
      frame = im.IntPtr(0),

      vertOffset = im.FloatPtr(0.0),
      latOffset = im.FloatPtr(0.0),
      spacing = im.FloatPtr(5.0),
      jitter = im.FloatPtr(0.0),
      useWorldZ = im.BoolPtr(false),
      matDisplay = '[None]',
      extentsL = 1.0, extentsW = 1.0, extentsH = 1.0,
      boxXLeft = 1.0, boxXRight = 1.0, boxYLeft = 1.0, boxYRight = 1.0, boxZLeft = 1.0, boxZRight = 1.0
    }
  if dir == 'above' then
    table.insert(p.layers, i, lNew)
  else
    table.insert(p.layers, i + 1, lNew)
  end
end

-- Cycles forward through the supported lane types.
local function cycleLaneType(t)
  if t == 'road_lane' then return 'sidewalk' end
  if t == 'sidewalk' then return 'shoulder' end
  if t == 'shoulder' then return 'island' end
  if t == 'island' then return 'road_lane' end
  return t
end

-- Cycles back through the supported lane types.
local function cycleLaneTypeBack(t)
  if t == 'road_lane' then return 'island' end
  if t == 'island' then return 'shoulder' end
  if t == 'shoulder' then return 'sidewalk' end
  if t == 'sidewalk' then return 'road_lane' end
  return t
end

-- Computes an array of valid lane keys for a road, ordered from smallest to largest.
local function computeLaneKeys(profile)
  local laneKeys, leftKeys, rightKeys, ctr, ctrL, ctrR = {}, {}, {}, 1, 1, 1
  for i = -20, -1 do                                                                                -- Do the left lanes separately.
    if profile[i] then
      laneKeys[ctr], leftKeys[ctrL] = i, i
      ctr, ctrL = ctr + 1, ctrL + 1
    end
  end
  for i = 1, 20 do                                                                                  -- Do the right lanes separately.
    if profile[i] then
      laneKeys[ctr], rightKeys[ctrR] = i, i
      ctr, ctrR = ctr + 1, ctrR + 1
    end
  end
  return laneKeys, leftKeys, rightKeys
end

-- Computes an ordered array of lane indices, from smallest to largest.
-- [Also returns the total number of lanes].
local function getOrderedLanes(profile)
  local lanes, ctr = {}, 1
  for i = -20, 20 do
    if profile[i] then
      lanes[ctr] = i
      ctr = ctr + 1
    end
  end
  return lanes, ctr - 1
end

-- Computes the number of left and right lanes (all types of lane).
local function getNumLanesLR(profile)
  local numLeft, numRight = 0, 0
  for i = -20, -1 do
    if profile[i] then
      numLeft = numLeft + 1
    end
  end
  for i = 1, 20 do
    if profile[i] then
      numRight = numRight + 1
    end
  end
  return numLeft, numRight
end

-- Computes the number of left and right (road lanes only).
local function getNumRoadLanesLR(profile)
  local numLeft, numRight = 0, 0
  for i = -20, -1 do
    if profile[i] and (profile[i].type == 'road_lane' or profile[i].type == 'shoulder') then
      numLeft = numLeft + 1
    end
  end
  for i = 1, 20 do
    if profile[i] and (profile[i].type == 'road_lane' or profile[i].type == 'shoulder') then
      numRight = numRight + 1
    end
  end
  return numLeft, numRight
end

-- Determines if the given profile is valid for creating an percectly-matched auto junction (when dragging road start/end to middle of other road).
local function isProfileValidForMidJctPerfect(profile)
  local numLeft, numRight = getNumLanesLR(profile)
  if numLeft ~= numRight then                                                                       -- Ensure the road is symmetric (num left lanes = num right lanes).
    return
  end
  local numSidewalks = 0
  for i = -20, 20 do
    if profile[i] then
      local pType = profile[i].type
      if pType == 'island' then                                                                     -- Ensure there are no types other than road_lane and sidewalk in the profile.
        return false
      end
      if pType == 'sidewalk' then
        if i < 0 then
          if profile[i - 1] then                                                                    -- Ensure there are no lanes to the left of left-side sidewalks.
            return false
          end
          if not profile[i + 1] or profile[i + 1].type ~= 'road_lane' then                          -- Ensure there is a road lane to the right of this left-side sidewalk.
            return false
          end
        end
        if i > 0 then
          if profile[i + 1] then                                                                    -- Ensure there are no lanes to the right of right-side sidewalks.
            return false
          end
          if not profile[i - 1] or profile[i - 1].type ~= 'road_lane' then                          -- Ensure there is a road lane to the left of this right-side sidewalk.
            return false
          end 
        end
        numSidewalks = numSidewalks + 1
      end
    end
  end
  if not (numSidewalks == 0 or numSidewalks == 2) then                                              -- Ensure there are either 0 or 2 sidewalk lanes.
    return false
  end
  return true
end

-- Determines if the given profile contains sidewalks, or not.
local function areSidewalksPresent(p)
  for i = -20, 20 do
    if p[i] and p[i].type == 'sidewalk' then
      return true
    end
  end
  return false
end

-- Computes tables of profile widths and heights (left and right), by lane key.
-- [Data is returned in imgui float ptr format].
local function getWAndHByKey(profile)
  local widths, heightsL, heightsR = {}, {}, {}
  for i = -20, 20 do
    local p = profile[i]
    if p then
      widths[i], heightsL[i], heightsR[i] = im.FloatPtr(p.width[0]), im.FloatPtr(p.heightL[0]), im.FloatPtr(p.heightR[0])
    end
  end
  return widths, heightsL, heightsR
end

-- Computes the width of the given profile.
local function getWidth(profile)
  local width = 0.0
  for i = -20, 20 do
    local p = profile[i]
    if p then
      width = width + p.width[0]
    end
  end
  return width
end

-- Converts an OpenDRIVE lane type to a supported native type.
local function convertOpenDRIVEType2Native(t)
  if t == 'shoulder' then return 'road_lane' end
  if t == 'border' then return 'road_lane' end
  if t == 'driving' then return 'road_lane' end
  if t == 'stop' then return 'road_lane' end
  if t == 'restricted' then return 'island' end
  if t == 'parking' then return 'road_lane' end
  if t == 'median' then return 'road_lane' end
  if t == 'walking' then return 'sidewalk' end
  if t == 'entry' then return 'road_lane' end
  if t == 'exit' then return 'road_lane' end
  if t == 'onRamp' then return 'road_lane' end
  if t == 'offRamp' then return 'road_lane' end
  if t == 'connectingRamp' then return 'road_lane' end
  if t == 'slipLane' then return 'road_lane' end
  if t == 'concrete' then return 'road_lane' end                                                    -- Deprecated in OpenDRIVE.
  if t == 'bidirectional' then return 'road_lane' end                                               -- Deprecated in OpenDRIVE.
  if t == 'none' then return 'island' end
  return 'road_lane'                                                                                -- Default.
end

-- Returns the current state of the lane flags.
local function getLaneFlags()
  return {
    isCustom = isCustom,
    isSidewalk = isSidewalk }
end

-- Creates a custom profile for use when importing from OpenDRIVE.
local function createCustomImportProfile(lanes)

  -- Create a table of lane keys.
  local lNew = {}
  for j = -20, 20 do
    local lane = lanes[j]
    if lane then
      lNew[j] = convertOpenDRIVEType2Native(lane.type)
    end
  end

  -- Create the new profile.
  -- [Lane widths and heights are given default values - they will be imported and added later, at each node].
  local profile = {}
  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(true)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(true)
  profile.conditionEndStopE = im.BoolPtr(true)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = defaultEdgeBlendMaterial
  profile.blendRightMat = defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)

  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  for k, nativeLaneType in pairs(lNew) do
    profile[k] = {
      type = nativeLaneType,
      width = im.FloatPtr(3.5), heightL = im.FloatPtr(0.1), heightR = im.FloatPtr(0.1),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  autoPaintLines(profile)

  return profile, lNew
end

-- Copies a profile template to a new profile.
local function createProfileFromTemplate(templateName, newName)
  local template = getProfileFromName(templateName)

  local profile = {}
  profile.name = newName or im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = template.class

  profile.styleType = im.IntPtr(template.styleType[0] or 0)
  profile.condition = im.FloatPtr(template.condition[0] or 0.2)
  profile.conditionSeed = im.IntPtr(template.conditionSeed[0] or 41226)
  profile.numPatches = im.IntPtr(template.numPatches[0] or 10)
  profile.numPotholes = im.IntPtr(template.numPotholes[0] or 0)
  profile.conditionCenterline = im.BoolPtr(template.conditionCenterline[0])
  profile.conditionEdgesL = im.BoolPtr(template.conditionEdgesL[0])
  profile.conditionEdgesR = im.BoolPtr(template.conditionEdgesR[0])
  profile.conditionLaneMarkings = im.BoolPtr(template.conditionLaneMarkings[0])
  profile.conditionEndStopS = im.BoolPtr(template.conditionEndStopS[0])
  profile.conditionEndStopE = im.BoolPtr(template.conditionEndStopE[0])
  profile.edgeLineGapL = im.FloatPtr(template.edgeLineGapL[0] or 0.25)
  profile.edgeLineGapR = im.FloatPtr(template.edgeLineGapR[0] or 0.25)
  profile.centerlineMat = template.centerlineMat or defaultCenterlineMaterial
  profile.edgeMatL = template.edgeMatL or defaultEdgeMaterial
  profile.edgeMatR = template.edgeMatR or defaultEdgeMaterial
  profile.laneMarkingsMat = template.laneMarkingsMat or defaultLaneMarkingsMaterial
  profile.endStopMatS = template.endStopMatS or defaultEndStopMat
  profile.endStopMatE = template.endStopMatE or defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(template.isEdgeBlendL[0])
  profile.isEdgeBlendR = im.BoolPtr(template.isEdgeBlendR[0])
  profile.isShowEdgeBlend = im.BoolPtr(template.isShowEdgeBlend[0])
  profile.blendLeftMat = template.blendLeftMat or defaultEdgeBlendMaterial
  profile.blendRightMat = template.blendRightMat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(template.blendLeftWidth[0] or 1.0)
  profile.blendRightWidth = im.FloatPtr(template.blendRightWidth[0] or 1.0)
  profile.isStopDecalS = im.BoolPtr(template.isStopDecalS[0])
  profile.isStopDecalE = im.BoolPtr(template.isStopDecalE[0])
  profile.stopGapS = im.FloatPtr(template.stopGapS[0])
  profile.stopGapE = im.FloatPtr(template.stopGapE[0])
  profile.continueLinesToEnd = im.BoolPtr(template.continueLinesToEnd[0] or false)

  profile.dirtMat = template.dirtMat or dirtTrackMaterial

  profile.isGutter = im.BoolPtr(template.isGutter[0])
  profile.gutterMat = template.gutterMat or defaultGutterMat
  profile.gutterMargin = im.FloatPtr(template.gutterMargin[0] or 0.02)
  profile.gutterWidth = im.FloatPtr(template.gutterWidth[0] or 0.2)
  profile.isGutterShow = im.BoolPtr(template.isGutterShow[0])

  profile.fadeS = im.FloatPtr(template.fadeS[0] or 3.0)
  profile.fadeE = im.FloatPtr(template.fadeE[0] or 3.0)

  profile.isAutoBanking = im.BoolPtr(template.isAutoBanking[0])
  profile.autoBankingFactor = im.FloatPtr(template.autoBankingFactor[0] or 1.0)
  profile.isExtraWidth = im.BoolPtr(template.isExtraWidth[0] or false)

  for i = -20, 20 do
    local lane = template[i]
    if lane then
      profile[i] = {
        type = lane.type,
        width = im.FloatPtr(lane.width[0]), heightL = im.FloatPtr(lane.heightL[0]), heightR = im.FloatPtr(lane.heightR[0]),
        isLeftSide = im.BoolPtr(lane.isLeftSide[0]), cornerDrop = im.FloatPtr(lane.cornerDrop[0]), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(lane.kerbWidth[0]), cornerLatOff = im.FloatPtr(lane.cornerLatOff[0]) }
    end
  end

  profile.layers = copyLayers(template)

  return profile
end

-- Creates a profile from decal road metadata (eg from decal roads in the scene tree, made with other tools).
local function createProfileFromDecalData(numLeftLanes, numRightLanes)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(true)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(true)
  profile.conditionEndStopE = im.BoolPtr(true)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = defaultEdgeBlendMaterial
  profile.blendRightMat = defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  for i = -numLeftLanes, -1 do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(3.5), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
  for i = 1, numRightLanes do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(3.5), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  autoPaintLines(profile)

  return profile
end

-- Creates a profile to be used for overlay roads.
local function createOverlayProfile(width)
  local profile = {}

  profile.name = im.ArrayChar(32, 'Overlay Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.0)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(0)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(true)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(true)
  profile.conditionEndStopE = im.BoolPtr(true)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = defaultEdgeBlendMaterial
  profile.blendRightMat = defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  profile[1] = {
    type = 'road_lane',
    width = im.FloatPtr(width), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  profile.layers = {}

  return profile
end

-- Creates a profile to be used for simple bridges.
local function createBridgeProfile(width, depth)
  local profile = {}

  profile.name = im.ArrayChar(32, 'Bridge Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.0)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(0)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(true)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(true)
  profile.conditionEndStopE = im.BoolPtr(true)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = defaultEdgeBlendMaterial
  profile.blendRightMat = defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  profile[-1] = {
    type = 'road_lane',
    width = im.FloatPtr(width), heightL = im.FloatPtr(depth), heightR = im.FloatPtr(depth),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  profile[1] = {
    type = 'road_lane',
    width = im.FloatPtr(width), heightL = im.FloatPtr(depth), heightR = im.FloatPtr(depth),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  profile.layers = {}

  return profile
end

-- Creates a profile from number of lanes and sidewalk data. For symmetric, two-way roads.
local function createProfileForJctRoad(numLeftLanes, numRightLanes, width, sidewalkWidth, sidewalkHeight, isSidewalk, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(true)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(true)
  profile.conditionEndStopE = im.BoolPtr(true)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  for i = -numLeftLanes, -1 do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(width), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
  for i = 1, numRightLanes do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(width), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
  if isSidewalk then
    profile[-numLeftLanes - 1] = {
      type = 'sidewalk',
      width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    profile[numRightLanes + 1] = {
      type = 'sidewalk',
      width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
      isLeftSide = im.BoolPtr(false), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  autoPaintLines(profile)
  autoEdgeBlending(profile, not isSidewalk, not isSidewalk, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Creates a profile for the tight join road used in Y-junctions.
local function createProfileForJctRoadYSpecial(numLeftLanes, numRightLanes, width, sidewalkWidth, sidewalkHeight, isSidewalk, isFlipKerb, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(true)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(true)
  profile.conditionEndStopE = im.BoolPtr(true)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  for i = -numLeftLanes, -1 do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(width), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
  for i = 1, numRightLanes do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(width), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
  if isSidewalk then
    profile[numRightLanes + 1] = {
      type = 'sidewalk',
      width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
      isLeftSide = im.BoolPtr(isFlipKerb), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  autoPaintLines(profile)
  autoEdgeBlending(profile, not isSidewalk, not isSidewalk, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Creates a profile from number of lanes and sidewalk data. For one-way roads.
local function createProfileForJctRoad1Way(numLanes, width, sidewalkWidth, sidewalkHeight, isSidewalk, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(true)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(true)
  profile.conditionEndStopE = im.BoolPtr(true)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  for i = 1, numLanes do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(width), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
  if isSidewalk then
    profile[-1] = {
      type = 'sidewalk',
      width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    profile[numLanes + 1] = {
      type = 'sidewalk',
      width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
      isLeftSide = im.BoolPtr(false), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  autoPaintLines(profile)
  autoEdgeBlending(profile, not isSidewalk, not isSidewalk, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Creates a profile from number of lanes. Contains a left-sided sidewalk lane.
local function createProfileForJctRoad_SW(numLeftLanes, numRightLanes, width, sidewalkWidth, sidewalkHeight, isLeftSide, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(true)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(true)
  profile.conditionEndStopE = im.BoolPtr(true)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  for i = -numLeftLanes, -1 do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(width), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
  for i = 1, numRightLanes do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(width), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  -- Add the single-sided sidewalk lane.
  local lIdx = nil
  if isLeftSide then
    lIdx = -numLeftLanes - 1
  else
    lIdx = numRightLanes + 1
  end
  profile[lIdx] = {
    type = 'sidewalk',
    width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
    isLeftSide = im.BoolPtr(isLeftSide), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  autoPaintLines(profile)
  autoEdgeBlending(profile, true, true, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Creates a profile with only one sidewalk lane in position 1.
local function createSidewalkOnlyProfile(sidewalkWidth, sidewalkHeight, isFlipKerb)
  if not isFlipKerb then
    isFlipKerb = false
  end

  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(0)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(true)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(true)
  profile.conditionEndStopE = im.BoolPtr(true)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = defaultEdgeBlendMaterial
  profile.blendRightMat = defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  profile[1] = {
    type = 'sidewalk',
    width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
    isLeftSide = im.BoolPtr(isFlipKerb), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  profile.layers = {}

  return profile
end

-- Create a profile for roundabout circular roads.
local function createRoundaboutProfile(numRBLanes, laneWidthRB, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(0)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(false)
  profile.conditionEdgesL = im.BoolPtr(false)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(false)
  profile.conditionEndStopE = im.BoolPtr(false)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(false)
  profile.isEdgeBlendR = im.BoolPtr(false)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  for i = 1, numRBLanes do
    profile[-i] = {
      type = 'road_lane',
      width = im.FloatPtr(laneWidthRB), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(false), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  profile.layers = {}

  return profile
end

-- Creates a highway junction profile (start and end cap sections).
local function createProfileForJctRoadHwyCap(numLanes, laneWidth, cResWidth, hardWidth, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'highway'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(false)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(false)
  profile.conditionEndStopE = im.BoolPtr(false)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  -- Set the main road lanes.
  for i = -numLanes - 1, -2 do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
  for i = 2, numLanes + 1 do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  -- Set the hard shoulder lanes.
  profile[-numLanes - 2] = {
    type = 'shoulder',
    width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  profile[numLanes + 2] = {
    type = 'shoulder',
    width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  -- Set the central reservation lanes.
  profile[-1] = {
    type = 'island',
    width = im.FloatPtr(cResWidth * 0.5), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  profile[1] = {
    type = 'island',
    width = im.FloatPtr(cResWidth * 0.5), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  autoPaintLines(profile)
  autoEdgeBlending(profile, true, true, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Creates a highway-urban transition junction profile (urban section).
local function createProfileForJctRoadHwyCapUrban(numLanes, laneWidth, isSidewalk, sidewalkWidth, sidewalkHeight, isOneWay, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'urban'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(false)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(false)
  profile.conditionEndStopE = im.BoolPtr(false)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  -- Set the main road lanes.
  if isOneWay then
    for i = 1, numLanes do
      profile[i] = {
        type = 'road_lane',
        width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
        isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    end
    if isSidewalk then
      profile[-1] = {
        type = 'sidewalk',
        width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
        isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
      profile[numLanes + 1] = {
        type = 'sidewalk',
        width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
        isLeftSide = im.BoolPtr(false), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
      end
  else
    for i = -numLanes, -1 do
      profile[i] = {
        type = 'road_lane',
        width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
        isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    end
    for i = 1, numLanes do
      profile[i] = {
        type = 'road_lane',
        width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
        isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    end

    if isSidewalk then
      profile[-numLanes - 1] = {
        type = 'sidewalk',
        width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
        isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
      profile[numLanes + 1] = {
        type = 'sidewalk',
        width = im.FloatPtr(sidewalkWidth), heightL = im.FloatPtr(sidewalkHeight), heightR = im.FloatPtr(sidewalkHeight),
        isLeftSide = im.BoolPtr(false), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    end
  end

  autoPaintLines(profile)
  autoEdgeBlending(profile, true, true, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Creates a highway-urban transition junction profile (transition/taper sections).
local function createProfileForJctRoadHwyUrbanTrans(numLanes, laneWidth, cResWidth, hardWidth, isOneWay, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'highway'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(false)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(false)
  profile.conditionEndStopE = im.BoolPtr(false)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  -- Set the main road lanes.
  if isOneWay then
    for i = 1, numLanes do
      profile[i] = {
        type = 'road_lane',
        width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
        isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    end
    profile[numLanes + 1] = {
      type = 'shoulder',
      width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  else
    for i = -numLanes - 1, -2 do
      profile[i] = {
        type = 'road_lane',
        width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
        isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    end
    for i = 2, numLanes + 1 do
      profile[i] = {
        type = 'road_lane',
        width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
        isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
        kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    end

    -- Set the hard shoulder lanes.
    profile[-numLanes - 2] = {
      type = 'shoulder',
      width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    profile[numLanes + 2] = {
      type = 'shoulder',
      width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

    -- Set the central reservation lanes.
    profile[-1] = {
      type = 'island',
      width = im.FloatPtr(cResWidth * 0.5), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    profile[1] = {
      type = 'island',
      width = im.FloatPtr(cResWidth * 0.5), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  autoPaintLines(profile)
  autoEdgeBlending(profile, true, true, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Creates a one-way highway junction profile.
-- [Does not have a central reservation lane].
local function createProfileForJctRoadHwyCap1W(numLanes, laneWidth, hardWidth, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'highway'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(false)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(false)
  profile.conditionEndStopE = im.BoolPtr(false)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  -- Set the main road lanes.
  for i = 1, numLanes do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  -- Set the hard shoulder lane.
  profile[numLanes + 1] = {
    type = 'shoulder',
    width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  autoPaintLines(profile)
  autoEdgeBlending(profile, true, true, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Creates a highway junction profile (section 2).
local function createProfileForJctRoadHwyS2(numLanes, laneWidth, cResWidth, hardWidth, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'highway'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(false)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(false)
  profile.conditionEndStopE = im.BoolPtr(false)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  -- Set the main road lanes.
  for i = -numLanes - 2, -2 do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
  for i = 2, numLanes + 2 do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  -- Set the hard shoulder lanes.
  profile[-numLanes - 3] = {
    type = 'shoulder',
    width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  profile[numLanes + 3] = {
    type = 'shoulder',
    width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  -- Set the central reservation lanes.
  profile[-1] = {
    type = 'island',
    width = im.FloatPtr(cResWidth * 0.5), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  profile[1] = {
    type = 'island',
    width = im.FloatPtr(cResWidth * 0.5), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  autoPaintLines(profile)
  autoEdgeBlending(profile, true, true, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Creates a highway junction profile (section 3).
local function createProfileForJctRoadHwyS3(numLanes, laneWidth, cResWidth, hardWidth, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'highway'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(false)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(false)
  profile.conditionEndStopE = im.BoolPtr(false)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  -- Set the main road lanes.
  for i = -numLanes - 1, -2 do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
  for i = 2, numLanes + 1 do
    profile[i] = {
      type = 'road_lane',
      width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end

  -- Set the split separator lanes.
  profile[-numLanes - 2] = {
    type = 'island',
    width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  profile[numLanes + 2] = {
    type = 'island',
    width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  -- Set the outer split road lanes.
  profile[-numLanes - 3] = {
    type = 'road_lane',
    width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  profile[numLanes + 3] = {
    type = 'road_lane',
    width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  -- Set the hard shoulder lanes.
  profile[-numLanes - 4] = {
    type = 'shoulder',
    width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  profile[numLanes + 4] = {
    type = 'shoulder',
    width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  -- Set the central reservation lanes.
  profile[-1] = {
    type = 'island',
    width = im.FloatPtr(cResWidth * 0.5), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  profile[1] = {
    type = 'island',
    width = im.FloatPtr(cResWidth * 0.5), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  autoPaintLines(profile)
  autoEdgeBlending(profile, true, true, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Creates a highway junction profile (for the one-way exit roads).
local function createProfileForJctRoadHwyExit(laneWidth, hardWidth, mat)
  local profile = {}

  profile.name = im.ArrayChar(32, 'New Profile')
  profile.isDeletable = true
  profile.class = 'highway'

  profile.styleType = im.IntPtr(0)
  profile.condition = im.FloatPtr(0.2)
  profile.conditionSeed = im.IntPtr(41226)
  profile.numPatches = im.IntPtr(10)
  profile.numPotholes = im.IntPtr(0)
  profile.conditionCenterline = im.BoolPtr(false)
  profile.conditionEdgesL = im.BoolPtr(true)
  profile.conditionEdgesR = im.BoolPtr(true)
  profile.conditionLaneMarkings = im.BoolPtr(true)
  profile.conditionEndStopS = im.BoolPtr(false)
  profile.conditionEndStopE = im.BoolPtr(false)
  profile.edgeLineGapL = im.FloatPtr(0.25)
  profile.edgeLineGapR = im.FloatPtr(0.25)
  profile.centerlineMat = defaultCenterlineMaterial
  profile.edgeMatL = defaultEdgeMaterial
  profile.edgeMatR = defaultEdgeMaterial
  profile.laneMarkingsMat = defaultLaneMarkingsMaterial
  profile.endStopMatS = defaultEndStopMat
  profile.endStopMatE = defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(true)
  profile.isEdgeBlendR = im.BoolPtr(true)
  profile.isShowEdgeBlend = im.BoolPtr(true)
  profile.blendLeftMat = mat or defaultEdgeBlendMaterial
  profile.blendRightMat = mat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(1.0)
  profile.blendRightWidth = im.FloatPtr(1.0)
  profile.isStopDecalS = im.BoolPtr(false)
  profile.isStopDecalE = im.BoolPtr(false)
  profile.stopGapS = im.FloatPtr(0.2)
  profile.stopGapE = im.FloatPtr(0.2)
  profile.continueLinesToEnd = im.BoolPtr(false)

  profile.dirtMat = dirtTrackMaterial

  profile.isGutter = im.BoolPtr(false)
  profile.gutterMat = defaultGutterMat
  profile.gutterMargin = im.FloatPtr(0.02)
  profile.gutterWidth = im.FloatPtr(0.2)
  profile.isGutterShow = im.BoolPtr(false)

  profile.fadeS = im.FloatPtr(3.0)
  profile.fadeE = im.FloatPtr(3.0)

  profile.isAutoBanking = im.BoolPtr(false)
  profile.autoBankingFactor = im.FloatPtr(1.0)
  profile.isExtraWidth = im.BoolPtr(false)

  -- Road lane.
  profile[1] = {
    type = 'road_lane',
    width = im.FloatPtr(laneWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  -- Hard shoulder.
  profile[2] = {
    type = 'shoulder',
    width = im.FloatPtr(hardWidth), heightL = im.FloatPtr(0.01), heightR = im.FloatPtr(0.01),
    isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
    kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }

  autoPaintLines(profile)
  autoEdgeBlending(profile, true, true, mat or defaultEdgeBlendMaterial)

  return profile
end

-- Deep copies a profile.
local function copyProfile(template)
  local profile = {}

  profile.name = template.name
  profile.isDeletable = true
  profile.class = template.class

  profile.styleType = im.IntPtr(template.styleType[0] or 0)
  profile.condition = im.FloatPtr(template.condition[0] or 0.2)
  profile.conditionSeed = im.IntPtr(template.conditionSeed[0] or 41226)
  profile.numPatches = im.IntPtr(template.numPatches[0] or 10)
  profile.numPotholes = im.IntPtr(template.numPotholes[0] or 0)
  profile.conditionCenterline = im.BoolPtr(template.conditionCenterline[0])
  profile.conditionEdgesL = im.BoolPtr(template.conditionEdgesL[0])
  profile.conditionEdgesR = im.BoolPtr(template.conditionEdgesR[0])
  profile.conditionLaneMarkings = im.BoolPtr(template.conditionLaneMarkings[0])
  profile.conditionEndStopS = im.BoolPtr(template.conditionEndStopS[0])
  profile.conditionEndStopE = im.BoolPtr(template.conditionEndStopE[0])
  profile.edgeLineGapL = im.FloatPtr(template.edgeLineGapL[0] or 0.25)
  profile.edgeLineGapR = im.FloatPtr(template.edgeLineGapR[0] or 0.25)
  profile.centerlineMat = template.centerlineMat or defaultCenterlineMaterial
  profile.edgeMatL = template.edgeMatL or defaultEdgeMaterial
  profile.edgeMatR = template.edgeMatR or defaultEdgeMaterial
  profile.laneMarkingsMat = template.laneMarkingsMat or defaultLaneMarkingsMaterial
  profile.endStopMatS = template.endStopMatS or defaultEndStopMat
  profile.endStopMatE = template.endStopMatE or defaultEndStopMat
  profile.isEdgeBlendL = im.BoolPtr(template.isEdgeBlendL[0])
  profile.isEdgeBlendR = im.BoolPtr(template.isEdgeBlendR[0])
  profile.isShowEdgeBlend = im.BoolPtr(template.isShowEdgeBlend[0])
  profile.blendLeftMat = template.blendLeftMat or defaultEdgeBlendMaterial
  profile.blendRightMat = template.blendRightMat or defaultEdgeBlendMaterial
  profile.blendLeftWidth = im.FloatPtr(template.blendLeftWidth[0] or 1.0)
  profile.blendRightWidth = im.FloatPtr(template.blendRightWidth[0] or 1.0)
  profile.isStopDecalS = im.BoolPtr(template.isStopDecalS[0])
  profile.isStopDecalE = im.BoolPtr(template.isStopDecalE[0])
  profile.stopGapS = im.FloatPtr(template.stopGapS[0])
  profile.stopGapE = im.FloatPtr(template.stopGapE[0])
  profile.continueLinesToEnd = im.BoolPtr(template.continueLinesToEnd[0])

  profile.dirtMat = template.dirtMat or dirtTrackMaterial

  profile.isGutter = im.BoolPtr(template.isGutter[0])
  profile.gutterMat = template.gutterMat or defaultGutterMat
  profile.gutterMargin = im.FloatPtr(template.gutterMargin[0] or 0.02)
  profile.gutterWidth = im.FloatPtr(template.gutterWidth[0] or 0.2)
  profile.isGutterShow = im.BoolPtr(template.isGutterShow[0])

  profile.fadeS = im.FloatPtr(template.fadeS[0] or 3.0)
  profile.fadeE = im.FloatPtr(template.fadeE[0] or 3.0)

  profile.isAutoBanking = im.BoolPtr(template.isAutoBanking[0])
  profile.autoBankingFactor = im.FloatPtr(template.autoBankingFactor[0] or 1.0)
  profile.isExtraWidth = im.BoolPtr(template.isExtraWidth[0] or false)

  for i = -20, 20 do
    local lane = template[i]
    if lane then
      profile[i] = {
        type = lane.type,
        width = im.FloatPtr(lane.width[0]), heightL = im.FloatPtr(lane.heightL[0]), heightR = im.FloatPtr(lane.heightR[0]),
        isLeftSide = im.BoolPtr(lane.isLeftSide[0]), cornerDrop = im.FloatPtr(lane.cornerDrop[0]), vStart = im.IntPtr(lane.vStart[0]),
        kerbWidth = im.FloatPtr(lane.kerbWidth[0]), cornerLatOff = im.FloatPtr(lane.cornerLatOff[0])  }
    end
  end

  profile.layers = copyLayers(template)

  return profile
end

-- Moves the camera to the profile edit pose.
-- [Also adjusts the timing parameters respectively].
local function goToProfileView(timer, time)
  if not isInProfileView then
    timer:stopAndReset()
    oldPos, oldRot = core_camera.getPosition(), core_camera.getQuat()                               -- Store the current camera position so we can return to it later.
    commands.setFreeCamera()
    core_camera.setPosRot(0, pView.x, pView.y, pView.z, pRot.x, pRot.y, pRot.z, pRot.w)             -- Move the camera to the profile pose.
    isInProfileView, time = true, 0.0
  end
  return time
end

-- Returns the camera to the stored old view.
local function goToOldView()
  if oldPos and oldRot then
    core_camera.setPosRot(0, oldPos.x, oldPos.y, oldPos.z, oldRot.x, oldRot.y, oldRot.z, oldRot.w)
  end
  isInProfileView = false
end

-- Gets some relevant numerical lanes data from the given profile.
-- [Number of left/right lanes and min/max lane indices].
local function getLanesData(p)
  local lNum, lMin, lMax = 0, 100, -100
  for i = -20, -1 do
    if p[i] then
      lMin, lMax, lNum = min(lMin, i), max(lMax, i), lNum + 1
    end
  end
  local rNum, rMin, rMax = 0, 100, -100
  for i = 1, 20 do
    if p[i] then
      rMin, rMax, rNum = min(rMin, i), max(rMax, i), rNum + 1
    end
  end
  return lNum, rNum, lMin, lMax, rMin, rMax
end

-- Removes the selected lane from the selected profile.
local function removeLane(selectedProfileIdx, selectedLaneIdx, side)
  if side == 'left' then
    local p = profiles[selectedProfileIdx]
    p[selectedLaneIdx] = nil
    for i = -1, -20, -1 do
      if not p[i] then
        p[i], p[i - 1] = p[i - 1], nil
      end
    end
  elseif side == 'right' then
    local p = profiles[selectedProfileIdx]
    p[selectedLaneIdx] = nil
    for i = 1, 20 do
      if not p[i] then
        p[i], p[i + 1] = p[i + 1], nil
      end
    end
  end

  -- Remove any layers which relate to lanes which no longer exist.
  local p = profiles[selectedProfileIdx]
  removeInvalidLayers(p)
end

-- Adds a lane to the selected profile, at the chosen position.
local function addLane(selectedProfileIdx, selectedLaneIdx, side, rel)
  local p = profiles[selectedProfileIdx]
  local laneHeight = 0.01
  local lNum, rNum, _, _, _, _ = getLanesData(p)
  if lNum < 20 and side == 'left' then
    if rel == 'above' then
      selectedLaneIdx = selectedLaneIdx - 1
    end
    for i = -20, selectedLaneIdx do
      p[i] = p[i + 1]
    end
    p[selectedLaneIdx] = {
      type = 'road_lane',
      width = im.FloatPtr(3.5), heightL = im.FloatPtr(laneHeight), heightR = im.FloatPtr(laneHeight),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
    return
  end
  if rNum < 20 and side == 'right' then
    if rel == 'below' then
      selectedLaneIdx = selectedLaneIdx + 1
    end
    for i = 20, selectedLaneIdx, -1 do
      p[i] = p[i - 1]
    end
    p[selectedLaneIdx] = {
      type = 'road_lane',
      width = im.FloatPtr(3.5), heightL = im.FloatPtr(laneHeight), heightR = im.FloatPtr(laneHeight),
      isLeftSide = im.BoolPtr(true), cornerDrop = im.FloatPtr(0.0), vStart = im.IntPtr(0),
      kerbWidth = im.FloatPtr(0.12), cornerLatOff = im.FloatPtr(0.0) }
  end
end

-- Sets all road lanes to the given master width.
-- [This over-writes the data in the lateral road profile].
local function applyMasterWidth(profile, mWidth)
  for i = -20, 20 do
    local lane = profile[i]
    if lane and lane.type == 'road_lane' then
      lane.width = im.FloatPtr(mWidth)
    end
  end
end

-- Updates the road condition details for the given road, based on the stored parameters.
local function updateCondition(r)
  if not r or #r.nodes < 2 or not r.renderData or #r.renderData < 2 then
    return
  end

  -- Ensure there is at least one road lane in this profile.  If not, leave immediately.
  local prof = r.profile
  local lMin, lMax = getMinMaxLaneKeys(prof)
  local isRoadLane = false
  for i = lMin, lMax do
    if prof[i] and prof[i].type == 'road_lane' then
      isRoadLane = true
      break
    end
  end

  -- Remove all layers from this profile, which are not marked as persistant.
  for i = #prof.layers, 1, -1 do
    local lay = prof.layers[i]
    if not lay.doNotDelete[0] then
      table.remove(prof.layers, i)
    end
  end

  -- If we do not have any road lanes, leave immediately.  This must be done after the layer delete phase.
  if not isRoadLane then
    return
  end

  math.randomseed(prof.conditionSeed[0])

  local rSections = computeSectionsByType(prof, 'road_lane', 'shoulder')

  for sec = 1, #rSections do
    local lMin, lMax = rSections[sec].s, rSections[sec].e

    local rLengths = util.computeRoadLength(r.renderData)
    local rLength = rLengths[#rLengths]

    -- Filter by style type.
    if prof.class == 'urban' or prof.class == 'highway' then                                        -- ASPHALT STYLE.

      -- Add the appropriate amount of damaged decal layer.
      if prof.condition[0] > 0.5 then
        local fade = (1.0 - ((prof.condition[0] * 2) - 1.0)) * rLength
        addDamagedLanesFull(prof, fade, fade, lMin, lMax)
      end

      -- Add full cracks on every lane.
      if prof.condition[0] > 0.9 then
        local fade = (1.0 - prof.condition[0]) * rLength
        addCracksLanes(prof, fade, fade, lMin, lMax)
      end

      -- Add the appropriate tread wear marks, based on the condition slider value.
      local fadeS, fadeE = prof.fadeS[0], prof.fadeE[0]
        if r.nodes[1].isLocked then
          fadeS = 0.0
        end
        if r.nodes[#r.nodes].isLocked then
          fadeE = 0.0
        end
      if prof.condition[0] > 0.01 and prof.condition[0] < 0.95 then
        addLightTreadToLanes(prof, fadeS, fadeE, lMin, lMax)
      elseif prof.condition[0] >= 0.95 then
        addHeavyTreadToLanes(prof, fadeS, fadeE, lMin, lMax)
      end

      -- Add the patches decalroads across parts of the road.
      for _ = 1, floor(prof.condition[0] * 5 * #r.nodes * 0.5) do
        local pStart = max(1, util.randomInRange(1, #r.nodes - 1))
        local pEnd = pStart + 1
        local dist = r.nodes[pStart].p:distance(r.nodes[pEnd].p) * 0.65
        local lIdx, lType = 0, 'none'
        while lIdx == 0 or (lType ~= 'road_lane' and lType ~= 'shoulder') do
          lIdx = util.randomInRange(lMin, lMax)
          if prof[lIdx] then
            lType = prof[lIdx].type
          end
        end
        addDamagedLanesPart(prof, pStart, pEnd, lIdx, dist, dist)
      end

      -- Add the crack decalroads across parts of the road.
      if prof.condition[0] > 0.5 then
        for _ = 1, floor((prof.condition[0] - 0.5) * 10) do
          local pStart = max(1, util.randomInRange(1, #r.nodes - 1))
          local pEnd = pStart + 1
          local dist = r.nodes[pStart].p:distance(r.nodes[pEnd].p) * 0.4
          local lIdx, lType = 0, 'none'
          while lIdx == 0 or lType ~= 'road_lane' do
            lIdx = util.randomInRange(lMin, lMax)
            if prof[lIdx] then
              lType = prof[lIdx].type
            end
          end
          local lLast, lNext = lIdx - 1, lIdx + 1
          if lLast == 0 then
            lLast = lLast - 1
          end
          if lNext == 0 then
            lNext = lNext + 1
          end
          local isLeft, latOff = true, 0.0
          if lIdx == lMin then
            isLeft = false
          end
          if not (prof[lLast] and prof[lLast].type == 'road_lane') and (prof[lNext] and prof[lNext].type == 'road_lane') then
            isLeft = false
            latOff = random() * 2
          elseif (prof[lLast] and prof[lLast].type == 'road_lane') and not (prof[lNext] and prof[lNext].type == 'road_lane') then
            isLeft = true
            latOff = -random() * 2
          end
          addCrackedLanesPart(prof, pStart, pEnd, lIdx, isLeft, latOff, dist, dist, util.randomInRange(1, 2))
        end
      end

      -- Add damage decal patches across the road, using 1D blue noise for spacing.
      if prof.condition[0] > 0.25 then
        local lastPos = 0.5
        for _ = 1, floor(prof.numPatches[0]) do
          local lIdx, lType = 0, 'none'
          while lIdx == 0 and lType ~= 'road_lane' do
            lIdx = util.randomInRange(lMin, lMax)
            lType = prof.type
          end
          local isLeft, latOff = true, -prof[lIdx].width[0] * 0.5                                   -- Default (eg 1-way roads) should be in middle of lane.
          if lIdx == lMin then
            isLeft = false
          end
          local lLast, lNext = lIdx - 1, lIdx + 1
          if lLast == 0 then
            lLast = lLast - 1
          end
          if lNext == 0 then
            lNext = lNext + 1
          end
          if not (prof[lLast] and prof[lLast].type == 'road_lane') and (prof[lNext] and prof[lNext].type == 'road_lane') then
            isLeft = false
            latOff = random() * 2
          elseif (prof[lLast] and prof[lLast].type == 'road_lane') and not (prof[lNext] and prof[lNext].type == 'road_lane') then
            isLeft = true
            latOff = -random() * 2
          end
          local size = 0.5 + random() * 1.5
          local pos = getBlueNoise1d(lastPos)
          local limMin, limMax = size / rLength, 1 - (size / rLength)
          addSingleDecalPatch(prof, max(limMin, min(limMax, pos)), lIdx, isLeft, latOff, size)
          lastPos = pos
        end

        -- Add pothole decal patches across the road, using 1D blue noise for spacing.
        for _ = 1, floor(prof.numPotholes[0]) do
          local lIdx, lType = 0, 'none'
          while lIdx == 0 and lType ~= 'road_lane' do
            lIdx = util.randomInRange(lMin, lMax)
            lType = prof.type
          end
          local isLeft, latOff = true, -prof[lIdx].width[0] * 0.5                                   -- Default (eg 1-way roads) should be in middle of lane.
          if lIdx == lMin then
            isLeft = false
          end
          local lLast, lNext = lIdx - 1, lIdx + 1
          if lLast == 0 then
            lLast = lLast - 1
          end
          if lNext == 0 then
            lNext = lNext + 1
          end
          if not (prof[lLast] and prof[lLast].type == 'road_lane') and (prof[lNext] and prof[lNext].type == 'road_lane') then
            isLeft = false
            latOff = random() * 2
          elseif (prof[lLast] and prof[lLast].type == 'road_lane') and not (prof[lNext] and prof[lNext].type == 'road_lane') then
            isLeft = true
            latOff = -random() * 2
          end
          local size = 0.5 + random() * 1.5
          local pos = getBlueNoise1d(lastPos)
          local limMin, limMax = size / rLength, 1 - (size / rLength)
          addSingleDecalPatchPothole(prof, max(limMin, min(limMax, pos)), lIdx, isLeft, latOff, size)
          lastPos = pos
        end
      end

    elseif prof.class == 'dirt' then                                                                -- DIRT ROAD STYLE.

      addDirtTracksToLanes(prof, 0, 0)

    end
  end
end

-- Serialises a profile.
local function serialiseProfile(p)
  local pSer = {
    name = ffi.string(p.name),
    isDeletable = p.isDeletable,
    class = p.class,

    styleType = p.styleType[0],
    condition = p.condition[0],
    conditionSeed = p.conditionSeed[0],
    numPatches = p.numPatches[0],
    numPotholes = p.numPotholes[0],
    conditionCenterline = p.conditionCenterline[0],
    conditionEdgesL = p.conditionEdgesL[0],
    conditionEdgesR = p.conditionEdgesR[0],
    conditionLaneMarkings = p.conditionLaneMarkings[0],
    conditionEndStopS = p.conditionEndStopS[0],
    conditionEndStopE = p.conditionEndStopE[0],
    edgeLineGapL = p.edgeLineGapL[0],
    edgeLineGapR = p.edgeLineGapR[0],
    centerlineMat = p.centerlineMat,
    edgeMatL = p.edgeMatL,
    edgeMatR = p.edgeMatR,
    laneMarkingsMat = p.laneMarkingsMat,
    endStopMatS = p.endStopMatS,
    endStopMatE = p.endStopMatE,
    isEdgeBlendL = p.isEdgeBlendL[0],
    isEdgeBlendR = p.isEdgeBlendR[0],
    isShowEdgeBlend = p.isShowEdgeBlend[0],
    blendLeftMat = p.blendLeftMat,
    blendRightMat = p.blendRightMat,
    blendLeftWidth = p.blendLeftWidth[0],
    blendRightWidth = p.blendRightWidth[0],
    isStopDecalS = p.isStopDecalS[0],
    isStopDecalE = p.isStopDecalE[0],
    stopGapS = p.stopGapS[0],
    stopGapE = p.stopGapE[0],
    continueLinesToEnd = p.continueLinesToEnd[0],

    dirtMat = p.dirtMat,

    isGutter = p.isGutter[0],
    gutterMat = p.gutterMat,
    gutterMargin = p.gutterMargin[0],
    gutterWidth = p.gutterWidth[0],
    isGutterShow = p.isGutterShow[0],

    fadeS = p.fadeS[0],
    fadeE = p.fadeE[0],

    isAutoBanking = p.isAutoBanking[0],
    autoBankingFactor = p.autoBankingFactor[0],
    isExtraWidth = p.isExtraWidth[0] }

  for i = -20, 20 do
    local l = p[i]
    if l then
      pSer[tostring(i)] = {
        type = l.type,
        width = l.width[0], heightL = l.heightL[0], heightR = l.heightR[0],
        isLeftSide = l.isLeftSide[0], cornerDrop = l.cornerDrop[0], vStart = l.vStart[0],
        kerbWidth = l.kerbWidth[0], cornerLatOff = l.cornerLatOff[0] }
    end
  end

  pSer.layers = {}
  for i = 1, #p.layers do
    local lay = p.layers[i]
    pSer.layers[i] =
      {
        name = ffi.string(lay.name),
        isHidden = lay.isHidden,
        doNotDelete = lay.doNotDelete[0],
        isReverse = lay.isReverse[0],
        isPaint = lay.isPaint[0],
        isDisplay = lay.isDisplay[0],
        type = lay.type[0],
        laneMin = lay.laneMin[0], laneMax = lay.laneMax[0],
        lane = lay.lane[0], isLeft = lay.isLeft[0], off = lay.off[0],
        width = lay.width[0],
        isSpanLong = lay.isSpanLong[0], nMin = lay.nMin[0], nMax = lay.nMax[0],
        texLen = lay.texLen[0],
        fadeS = lay.fadeS[0], fadeE = lay.fadeE[0],
        mat = tostring(lay.mat),
        rot = lay.rot[0],
        pos = lay.pos[0], size = lay.size[0],
        numRows = lay.numRows[0], numCols = lay.numCols[0],
        frame = lay.frame[0],

        vertOffset = lay.vertOffset[0],
        latOffset = lay.latOffset[0],
        spacing = lay.spacing[0],
        jitter = lay.jitter[0],
        useWorldZ = lay.useWorldZ[0],
        matDisplay = lay.matDisplay,
        extentsL = lay.extentsL, extentsW = lay.extentsW, extentsH = lay.extentsH,
        boxXLeft = lay.boxXLeft, boxXRight = lay.boxXRight,
        boxYLeft = lay.boxYLeft, boxYRight = lay.boxYRight,
        boxZLeft = lay.boxZLeft, boxZRight = lay.boxZRight
      }
  end

  return pSer
end

-- De-serialises a profile.
local function deserialiseProfile(pSer)
  if not pSer then
    return
  end

  local p = {
    name = im.ArrayChar(32, pSer.name or 'New Profile'),
    isDeletable = pSer.isDeletable or false,
    class = pSer.class or defaultProfileClass,

    styleType = im.IntPtr(pSer.styleType or 0),
    condition = im.FloatPtr(pSer.condition or 0.2),
    conditionSeed = im.IntPtr(pSer.conditionSeed or 41226),
    numPatches = im.IntPtr(pSer.numPatches or 10),
    numPotholes = im.IntPtr(pSer.numPotholes or 0),
    conditionCenterline = im.BoolPtr(pSer.conditionCenterline),
    conditionEdgesL = im.BoolPtr(pSer.conditionEdgesL or false),
    conditionEdgesR = im.BoolPtr(pSer.conditionEdgesR or false),
    conditionLaneMarkings = im.BoolPtr(pSer.conditionLaneMarkings),
    conditionEndStopS = im.BoolPtr(pSer.conditionEndStopS or false),
    conditionEndStopE = im.BoolPtr(pSer.conditionEndStopE or false),
    edgeLineGapL = im.FloatPtr(pSer.edgeLineGapL or 0.25),
    edgeLineGapR = im.FloatPtr(pSer.edgeLineGapR or 0.25),
    centerlineMat = pSer.centerlineMat or defaultCenterlineMaterial,
    edgeMatL = pSer.edgeMatL or defaultEdgeMaterial,
    edgeMatR = pSer.edgeMatR or defaultEdgeMaterial,
    laneMarkingsMat = pSer.laneMarkingsMat or defaultLaneMarkingsMaterial,
    endStopMatS = pSer.endStopMatS or defaultEndStopMat,
    endStopMatE = pSer.endStopMatE or defaultEndStopMat,
    isEdgeBlendL = im.BoolPtr(pSer.isEdgeBlendL or false),
    isEdgeBlendR = im.BoolPtr(pSer.isEdgeBlendR or false),
    isShowEdgeBlend = im.BoolPtr(pSer.isShowEdgeBlend),
    blendLeftMat = pSer.blendLeftMat or defaultEdgeBlendMaterial,
    blendRightMat = pSer.blendRightMat or defaultEdgeBlendMaterial,
    blendLeftWidth = im.FloatPtr(pSer.blendLeftWidth or 1.0),
    blendRightWidth = im.FloatPtr(pSer.blendRightWidth or 1.0),
    isStopDecalS = im.BoolPtr(pSer.isStopDecalS or false),
    isStopDecalE = im.BoolPtr(pSer.isStopDecalE or false),
    stopGapS = im.FloatPtr(pSer.stopGapS or false),
    stopGapE = im.FloatPtr(pSer.stopGapE or false),
    continueLinesToEnd = im.BoolPtr(pSer.continueLinesToEnd or false),

    dirtMat = pSer.dirtMat or dirtTrackMaterial,

    isGutter = im.BoolPtr(pSer.isGutter),
    gutterMat = pSer.gutterMat or defaultGutterMat,
    gutterMargin = im.FloatPtr(pSer.gutterMargin or 0.02),
    gutterWidth = im.FloatPtr(pSer.gutterWidth or 0.2),
    isGutterShow = im.BoolPtr(pSer.isGutterShow),

    fadeS = im.FloatPtr(pSer.fadeS or 0.0),
    fadeE = im.FloatPtr(pSer.fadeE or 0.0),

    isAutoBanking = im.BoolPtr(pSer.isAutoBanking),
    autoBankingFactor = im.FloatPtr(pSer.autoBankingFactor or 1.0),
    isExtraWidth = im.BoolPtr(pSer.isExtraWidth or false) }

  for i = -20, 20 do
    local l = pSer[tostring(i)]
    if l then
      p[i] = {
        type = l.type,
        width = im.FloatPtr(l.width or 3.5), heightL = im.FloatPtr(l.heightL or 0.01), heightR = im.FloatPtr(l.heightR or 0.01),
        isLeftSide = im.BoolPtr(l.isLeftSide or false), cornerDrop = im.FloatPtr(l.cornerDrop or 0.0), vStart = im.IntPtr(l.vStart or 0.0),
        kerbWidth = im.FloatPtr(l.kerbWidth or 0.12), cornerLatOff = im.FloatPtr(l.cornerLatOff or 0.0) }
    end
  end

  p.layers = {}
  if pSer.layers then
    for i = 1, #pSer.layers do
      local lay = pSer.layers[i]
      p.layers[i] =
        {
          name = im.ArrayChar(32, lay.name or 'new layer'),
          isHidden = lay.isHidden or false,
          doNotDelete = im.BoolPtr(lay.doNotDelete or false),
          isReverse = im.BoolPtr(lay.isReverse or false),
          isPaint = im.BoolPtr(lay.isPaint or false),
          isDisplay = im.BoolPtr(lay.isDisplay or false),
          type = im.IntPtr(lay.type or 0),
          laneMin = im.IntPtr(lay.laneMin or 1), laneMax = im.IntPtr(lay.laneMax or 1),
          lane = im.IntPtr(lay.lane or 1), isLeft = im.BoolPtr(lay.isLeft or false), off = im.FloatPtr(lay.off or 0.0),
          width = im.FloatPtr(lay.width or 3.5),
          isSpanLong = im.BoolPtr(lay.isSpanLong), nMin = im.IntPtr(lay.nMin or 1), nMax = im.IntPtr(lay.nMax or 1),
          texLen = im.FloatPtr(lay.texLen or 5.0),
          fadeS = im.FloatPtr(lay.fadeS or 0.0), fadeE = im.FloatPtr(lay.fadeE or 0.0),
          mat = lay.mat,
          rot = im.IntPtr(lay.rot or 0),
          pos = im.FloatPtr(lay.pos or 0.0), size = im.FloatPtr(lay.size or 0),
          numRows = im.IntPtr(lay.numRows or 0), numCols = im.IntPtr(lay.numCols or 0),
          frame = im.IntPtr(lay.frame or 0),

          vertOffset = im.FloatPtr(lay.vertOffset or 0.0),
          latOffset = im.FloatPtr(lay.latOffset or 0.0),
          spacing = im.FloatPtr(lay.spacing or 5.0),
          jitter = im.FloatPtr(lay.jitter or 0.0),
          useWorldZ = im.BoolPtr(lay.useWorldZ or false),
          matDisplay = lay.matDisplay,
          extentsL = lay.extentsL or 1.0, extentsW = lay.extentsW or 1.0, extentsH = lay.extentsH or 1.0,
          boxXLeft = lay.boxXLeft or 1.0, boxXRight = lay.boxXRight or 1.0,
          boxYLeft = lay.boxYLeft or 1.0, boxYRight = lay.boxYRight or 1.0,
          boxZLeft = lay.boxZLeft or 1.0, boxZRight = lay.boxZRight or 1.0
        }
    end
  end

  return p
end

-- Saves the given profile to disk.
local function save(p)
  extensions.editor_fileDialog.saveFile(
    function(data)
      local pSer = serialiseProfile(p)
      pSer.name = util.getFilenameFromPath(data.filepath)
      local encodedData = { profile = pSer }
      jsonWriteFile(data.filepath, encodedData, true)
    end,
    {{"JSON",".json"}},
    false,
    "/",
    "File already exists.\nDo you want to overwrite the file?")
end

-- Loads a profile from disk.
local function load()
  extensions.editor_fileDialog.openFile(
    function(data)
      local loadedJson = jsonReadFile(data.filepath)
      local serProfile = loadedJson.profile
      numLoadedProfiles = numLoadedProfiles + 1
      profiles[#profiles + 1] = deserialiseProfile(serProfile)
    end,
    {{"JSON",".json"}},
    false,
    "/")
end

-- Removes all the temporary 'Current Profile' profiles from the profiles table.
local function removeAllTempCurrentProfiles()
  for i = #profiles, 1, -1 do
    if ffi.string(profiles[i].name) == 'Current Profile' then
      table.remove (profiles, i)
    end
  end
end

-- Creates a new template upon user request.
local function createTemplateOnRequest(p)
  local pCopy = copyProfile(p)
  pCopy.name = im.ArrayChar(32, 'New Template')
  profiles[#profiles + 1] = pCopy
end

-- Loads the profile at the given file path.
local function loadPrefabProfile(filepath)
  local loadedJson = jsonReadFile(filepath)
  local serProfile = loadedJson.profile
  numLoadedProfiles = numLoadedProfiles + 1
  local p = deserialiseProfile(serProfile)
  p.name = im.ArrayChar(32, util.removeExtension(ffi.string(p.name)))
  return p
end

-- Gets the collection of default lateral road profile templates.
local function populateProfileTemplates()
  if #profiles > 0 then
    return                                                                                          -- If the profiles already exist, do not re-create them.
  end
  for i = 1, #templateFilepaths do
    profiles[i] = loadPrefabProfile(templateFilepaths[i])
    profiles[i].isDeletable = false
  end
end

-- Reset all profile templates.
local function resetTemplates()
  table.clear(profiles)
  populateProfileTemplates()
end


-- Public interface.
M.profiles =                                              profiles

M.getMinMaxLaneKeys =                                     getMinMaxLaneKeys
M.updateLaneFlags =                                       updateLaneFlags
M.getLaneFlags =                                          getLaneFlags
M.removeLayer =                                           removeLayer
M.layerChangePriority =                                   layerChangePriority
M.addLayer =                                              addLayer
M.getIdFromName =                                         getIdFromName
M.getProfileFromName =                                    getProfileFromName
M.cycleLaneType =                                         cycleLaneType
M.cycleLaneTypeBack =                                     cycleLaneTypeBack
M.computeLaneKeys =                                       computeLaneKeys
M.getOrderedLanes =                                       getOrderedLanes
M.getNumLanesLR =                                         getNumLanesLR
M.getNumRoadLanesLR =                                     getNumRoadLanesLR
M.isProfileValidForMidJctPerfect =                        isProfileValidForMidJctPerfect
M.areSidewalksPresent =                                   areSidewalksPresent
M.getWAndHByKey =                                         getWAndHByKey
M.getWidth =                                              getWidth

M.removeAllTempCurrentProfiles =                          removeAllTempCurrentProfiles

M.autoPaintLines =                                        autoPaintLines
M.autoEdgeBlending =                                      autoEdgeBlending
M.addCenterline =                                         addCenterline
M.addEdgeLines =                                          addEdgeLines
M.addEndLine =                                            addEndLine
M.addLaneDivisionLines =                                  addLaneDivisionLines
M.addLightTreadToLanes =                                  addLightTreadToLanes

M.createTemplateOnRequest =                               createTemplateOnRequest

M.computeSectionsByType =                                 computeSectionsByType

M.createCustomImportProfile =                             createCustomImportProfile
M.createProfileFromTemplate =                             createProfileFromTemplate
M.createProfileFromDecalData =                            createProfileFromDecalData

M.createOverlayProfile =                                  createOverlayProfile
M.createBridgeProfile =                                   createBridgeProfile
M.createProfileForJctRoad =                               createProfileForJctRoad
M.createProfileForJctRoadYSpecial =                       createProfileForJctRoadYSpecial
M.createProfileForJctRoad1Way =                           createProfileForJctRoad1Way
M.createProfileForJctRoad_SW =                            createProfileForJctRoad_SW
M.createSidewalkOnlyProfile =                             createSidewalkOnlyProfile
M.createRoundaboutProfile =                               createRoundaboutProfile
M.createProfileForJctRoadHwyCap =                         createProfileForJctRoadHwyCap
M.createProfileForJctRoadHwyCapUrban =                    createProfileForJctRoadHwyCapUrban
M.createProfileForJctRoadHwyUrbanTrans =                  createProfileForJctRoadHwyUrbanTrans
M.createProfileForJctRoadHwyCap1W =                       createProfileForJctRoadHwyCap1W
M.createProfileForJctRoadHwyS2 =                          createProfileForJctRoadHwyS2
M.createProfileForJctRoadHwyS3 =                          createProfileForJctRoadHwyS3
M.createProfileForJctRoadHwyExit =                        createProfileForJctRoadHwyExit

M.updateToNewTemplate =                                   updateToNewTemplate
M.copyProfile =                                           copyProfile
M.goToProfileView =                                       goToProfileView
M.goToOldView =                                           goToOldView
M.removeLane =                                            removeLane
M.addLane =                                               addLane

M.applyMasterWidth =                                      applyMasterWidth
M.populateProfileTemplates =                              populateProfileTemplates
M.resetTemplates =                                        resetTemplates

M.updateCondition =                                       updateCondition

M.serialiseProfile =                                      serialiseProfile
M.deserialiseProfile =                                    deserialiseProfile

M.save =                                                  save
M.load =                                                  load

return M