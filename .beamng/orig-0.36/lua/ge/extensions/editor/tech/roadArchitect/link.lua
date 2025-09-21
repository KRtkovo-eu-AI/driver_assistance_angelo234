-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local defaultPedXMaterial = 'crossing_white'                                                        -- The default material for pedestrian crossings.

local trafficBoomMeshPath = '/art/shapes/objects/s_trafficlight_boom_sn.dae'                        -- The path to the traffic light boom mesh.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

local roadMgr = require('editor/tech/roadArchitect/roads')                                          -- A module for managing roads.
local profileMgr = require('editor/tech/roadArchitect/profiles')                                    -- A module for managing road profiles.
local jctMgr = require('editor/tech/roadArchitect/junctions')                                       -- A module for managing junctions.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.

local im = ui_imgui
local min, max = math.min, math.max
local abs, pi, sqrt, rad2Deg = math.abs, math.pi, math.sqrt, math.deg

local vertical = vec3(0, 0, 1)


-- Checks for a match between two overlays.
local function isOverlayMatch(r1, r2) return r1.isOverlay and r2.isOverlay end

-- Checks for a perfect join match.
local function isPerfectMatch(r1, r2, lie1, lie2, numLanes1, numLanes2, lanes1, lanes2)
  if numLanes2 == numLanes1 then
    local p1, p2 = r1.profile, r2.profile
    local left1, right1 = profileMgr.getNumRoadLanesLR(p1)
    local left2, right2 = profileMgr.getNumRoadLanesLR(p2)
    local is1OneWay = left1 == 0 or right1 == 0
    local is2OneWay = left2 == 0 or right2 == 0
    if is1OneWay ~= is2OneWay then                                                                  -- Do not allow 2W -> 1L road joining.
      return false
    end
    for j = 1, numLanes1 do
      local type1 = p1[lanes1[j]].type
      if (not lie1 and lie2) or (lie1 and not lie2) then                                            -- Same direction: [End -> Start or Start -> End].
      local type2 = p2[lanes2[j]].type
        if type1 ~= type2 then
          return false
        end
      else                                                                                          -- Opposing direction: [End -> End or Start -> Start].
        local type2 = p2[lanes2[numLanes2 - j + 1]].type
        if type1 ~= type2 then
          return false
        end
        local left1, right1 = profileMgr.getNumRoadLanesLR(p1)
        local isOneWay1 = left1 == 0 and right1 > 0
        local left2, right2 = profileMgr.getNumRoadLanesLR(p2)
        local isOneWay2 = left2 == 0 and right2 > 0
        if isOneWay1 and isOneWay2 then                                                             -- If both roads are one way and oppose, we can't link them.
          return false
        end
      end
    end
    return true
  end
  return false
end

-- Checks for a 1-way highway-to-rural/urban match.
local function is1WHwyToRural(r1, r2, lie1, lie2, numLanes1, numLanes2, lanes1, lanes2)
  local p1, p2 = r1.profile, r2.profile
  local left1, right1 = profileMgr.getNumRoadLanesLR(p1)
  local isOneWay1 = left1 == 0 and right1 > 0
  local left2, right2 = profileMgr.getNumRoadLanesLR(p2)
  local isOneWay2 = left2 == 0 and right2 > 0
  if not (isOneWay1 and isOneWay2) then                                                             -- If both roads are not one way, we won't link them.
    return false
  end
  if lie1 == lie2 then                                                                              -- Ensure that the 1-way direction is consistant across the roads.
    return false
  end
  if not numLanes1 == numLanes2 + 1 then                                                            -- The highway road must have an extra (shoulder) lane, compared to rural road.
    return false
  end
  local isShoulderPresent = false
  for i = 1, #lanes1 do
    local p1Type = p1[lanes1[i]].type
    if p1Type == 'shoulder' then                                                                    -- Ensure that profile1 contains a shoulder lane (this means its a highway road).
      isShoulderPresent = true
    end
  end
  if not isShoulderPresent then
    return false
  end
  for i = 1, #lanes2 do
    local p2Type = p2[lanes2[i]].type
    if p2Type == 'shoulder' or p2Type == 'island' then                                              -- Ensure that road2 is an rural or urban road.
      return false
    end
  end
  return true
end

-- Checks for a sidewalk to no-sidewalk match.
local function isSidewalkToNoSidewalk(r1, r2, numLanes1, numLanes2, lanes1, lanes2)
  local p1, p2 = r1.profile, r2.profile
  local left1, right1 = profileMgr.getNumRoadLanesLR(p1)
  local isOneWay1 = left1 == 0 and right1 > 0
  local left2, right2 = profileMgr.getNumRoadLanesLR(p2)
  local isOneWay2 = left2 == 0 and right2 > 0
  if isOneWay1 and isOneWay2 then                                                                   -- If both roads are one way, we won't link them.
    return false
  end
  if numLanes2 == numLanes1 - 2 then
    if p1[lanes1[1]].type == 'sidewalk' and p1[lanes1[#lanes1]].type == 'sidewalk' then             -- Ensure that profile1 is bounded by sidewalk lanes.
      if p2[lanes2[1]].type == 'road_lane' and p2[lanes2[#lanes2]].type == 'road_lane' then         -- Ensure that profile2 is bounded by road lanes.
        local numSidewalks = 0
        for i = 1, #lanes1 do
          if not p1[lanes1[i]] then
            return false
          end
          local p1Type = p1[lanes1[i]].type
          if p1Type == 'island' or p1Type == 'shoulder' then                                        -- Ensure that profile1 contains only road lanes and sidewalks.
            return false
          end
          if p1Type == 'sidewalk' then                                                              -- Ensure that sidewalks only exist at the profile edge.
            if p1[lanes1[i - 1]] and p1[lanes1[i + 1]] then
              return false
            end
            numSidewalks = numSidewalks + 1
          end
        end
        if numSidewalks ~= 2 then                                                                   -- Ensure there are exactly two sidewalk lanes present in road1.
          return false
        end
        for i = 1, #lanes2 do
          if not p2[lanes2[i]] then
            return false
          end
          local p2Type = p2[lanes2[i]].type
          if p2Type == 'island' or p2Type == 'shoulder' or p2Type == 'sidewalk' then                -- Ensure that profile2 contains only road lanes.
            return false
          end
        end
        return true
      end
    end
  end
  return false
end

-- Checks for an urban taper-up match.
local function isUrbanTaperUp(r1, r2, numLanes1, numLanes2, lanes1, lanes2)
  local p1, p2 = r1.profile, r2.profile
  local left1, right1 = profileMgr.getNumRoadLanesLR(p1)
  local isOneWay1 = left1 == 0 and right1 > 0
  local left2, right2 = profileMgr.getNumRoadLanesLR(p2)
  local isOneWay2 = left2 == 0 and right2 > 0
  if isOneWay1 and isOneWay2 then                                                                   -- If both roads are one way, we won't link them
    return false
  end
  if numLanes2 == numLanes1 + 2 then
    for i = 1, #lanes1 do
      if not p1[lanes1[i]] then
        return false
      end
      local p1Type = p1[lanes1[i]].type
      if p1Type == 'island' or p1Type == 'shoulder' then                                            -- Ensure that profile1 contains only road lanes and sidewalks.
        return false
      end
      if p1Type == 'sidewalk' then                                                                  -- Ensure that sidewalks only exist at the profile edge.
        if p1[lanes1[i - 1]] and p1[lanes1[i + 1]] then
          return false
        end
      end
    end
    for i = 1, #lanes2 do
      if not p2[lanes2[i]] then
        return false
      end
      local p2Type = p2[lanes2[i]].type
      if p2Type == 'island' or p2Type == 'shoulder' then                                            -- Ensure that profile2 contains only road lanes and sidewalks.
        return false
      end
      if p2Type == 'sidewalk' then                                                                  -- Ensure that sidewalks only exist at the profile edge.
        if p2[lanes2[i - 1]] and p2[lanes2[i + 1]] then
          return false
        end
      end
    end
    return true
  end
  return false
end

-- Checks for an urban one-way to two-way match.
local function isUrban1WTo2W(r1, r2, lie1, lie2, lanes1, lanes2)
  local p1, p2 = r1.profile, r2.profile
  local left1, right1 = profileMgr.getNumRoadLanesLR(p1)
  if left1 == 0 and right1 > 0 then                                                                 -- Ensure that road1 is one-way.
    local left2, right2 = profileMgr.getNumRoadLanesLR(p2)
    if left2 < 1 or right2 < 1 then                                                                 -- Ensure that road2 is two-way.
      return false
    end
    if lie1 == lie2 then
      if right1 ~= left2 then                                                                       -- Ensure that the number of lanes match, if road ends are opposing.
        return false
      end
    else
      if right1 ~= right2 then                                                                      -- Ensure that the number of lanes match, if road ends are in-line.
        return false
      end
    end
    for i = 1, #lanes1 do
      if not p1[lanes1[i]] then
        return false
      end
      local p1Type = p1[lanes1[i]].type
      if p1Type == 'island' or p1Type == 'shoulder' then                                            -- Ensure that profile1 contains only road lanes and sidewalks.
        return false
      end
      if p1Type == 'sidewalk' then                                                                  -- Ensure that sidewalks only exist at the profile edge.
        return false
      end
    end
    for i = 1, #lanes2 do
      if not p2[lanes2[i]] then
        return false
      end
      local p2Type = p2[lanes2[i]].type
      if p2Type == 'island' or p2Type == 'shoulder' then                                            -- Ensure that profile2 contains only road lanes and sidewalks.
        return false
      end
      if p2Type == 'sidewalk' then                                                                  -- Ensure that sidewalks only exist at the profile edge.
        return false
      end
    end
    return true
  end
  return false
end

-- Checks for an urban to highway match.
local function isUrbanToHwy(r1, r2, lanes1, lanes2)
  local p1, p2 = r1.profile, r2.profile
  local left1, right1 = profileMgr.getNumRoadLanesLR(p1)
  local left2, right2 = profileMgr.getNumRoadLanesLR(p2)
  if left1 < 1 or right1 < 1 or left2 < 1 or right2 < 1 then                                        -- Ensure that both roads are two-way.
    return false
  end
  if left1 + right1 ~= left2 + right2 - 2 then                                                      -- Ensure that the lane numbers match (includes shoulders as road lanes here).
    return false
  end
  for i = 1, #lanes1 do
    if not p1[lanes1[i]] then
      return false
    end
    local p1Type = p1[lanes1[i]].type
    if p1Type == 'island' or p1Type == 'shoulder' then                                              -- Ensure that profile1 contains only road lanes and sidewalks (ie is urban).
      return false
    end
    if p1Type == 'sidewalk' then                                                                    -- Ensure that sidewalks only exist at the profile edge.
      if p1[lanes1[i - 1]] and p1[lanes1[i + 1]] then
        return false
      end
    end
  end
  local isShoulderFound = false
  for i = 1, #lanes2 do
    if not p2[lanes2[i]] then
      return false
    end
    local p2Type = p2[lanes2[i]].type
    if p2Type == 'shoulder' then
      isShoulderFound = true
    end
    if p2Type == 'sidewalk' then                                                                    -- Ensure that sidewalks are not present on road2 (ie not a highway).
      return false
    end
  end
  if not isShoulderFound then                                                                       -- Ensure profile2 contains a shoulder lane (ie is highway).
    return false
  end
  return true
end

-- Checks for a highway taper-up match.
local function isHwyTaperUp(r1, r2, lanes1, lanes2)
  local p1, p2 = r1.profile, r2.profile
  local left1, right1 = profileMgr.getNumRoadLanesLR(p1)
  local left2, right2 = profileMgr.getNumRoadLanesLR(p2)
  if left2 ~= left1 + 1 or right2 ~= right1 + 1 then                                                -- Ensure that we have the correct 'taper up' by counting road lanes and shoulders.
    return false
  end
  local isShoulderFound = false
  for i = 1, #lanes1 do
    if not p1[lanes1[i]] then
      return false
    end
    local p1Type = p1[lanes1[i]].type
    if p1Type == 'shoulder' then
      isShoulderFound = true
    end
  end
  if not isShoulderFound then                                                                       -- Ensure that road1 is a valid highway (checks if shoulder is present).
    return false
  end
  for i = 1, #lanes2 do
    if not p2[lanes2[i]] then
      return false
    end
    local p2Type = p2[lanes2[i]].type
    if p2Type == 'shoulder' then
      isShoulderFound = true
    end
  end
  if not isShoulderFound then                                                                       -- Ensure that road2 is a valid highway (checks if shoulder is present).
    return false
  end
  return true
end

-- Checks for a highway one-way to two-way match.
local function isHwy1WTo2W(r1, r2, lie1, lie2, lanes1, lanes2)
  local p1, p2 = r1.profile, r2.profile
  local left1, right1 = profileMgr.getNumRoadLanesLR(p1)
  if left1 == 0 and right1 > 0 then                                                                 -- Ensure that road1 is one-way.
    local left2, right2 = profileMgr.getNumRoadLanesLR(p2)
    if left2 < 1 or right2 < 1 then                                                                 -- Ensure that road2 is two-way.
      return false
    end
    if lie1 == lie2 then
      if right1 ~= left2 then                                                                       -- Ensure that the number of lanes match, if road ends are opposing.
        return false
      end
    else
      if right1 ~= right2 then                                                                      -- Ensure that the number of lanes match, if road ends are in-line.
        return false
      end
    end
    local isShoulderFound = false
    for i = 1, #lanes1 do
      if not p1[lanes1[i]] then
        return false
      end
      local p1Type = p1[lanes1[i]].type
      if p1Type == 'shoulder' then
        isShoulderFound = true
      end
    end
    if not isShoulderFound then                                                                     -- Ensure that road1 is a valid highway (checks if shoulder is present).
      return false
    end
    for i = 1, #lanes2 do
      if not p2[lanes2[i]] then
        return false
      end
      local p2Type = p2[lanes2[i]].type
      if p2Type == 'shoulder' then
        isShoulderFound = true
      end
    end
    if not isShoulderFound then                                                                     -- Ensure that road2 is a valid highway (checks if shoulder is present).
      return false
    end
    return true
  end
  return false
end

-- Computes a collection of possible linkages between two given roads.
local function computePossibleLinks(rIdx1, nIdx1, rIdxs, nIdxs)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local r1 = roads[rIdx1]
  local lMin1, lMax1 = profileMgr.getMinMaxLaneKeys(r1.profile)                                     -- Get the lane limits for each road.
  local numLanes1 = 0                                                                               -- Count the number of lanes in each road.
  local lanes1, ctr = {}, 1
  for i = lMin1, lMax1 do
    if i ~= 0 then
      numLanes1 = numLanes1 + 1
      lanes1[ctr] = i
      ctr = ctr + 1
    end
  end

  local links = {}
  for rr = 1, #rIdxs do
    local r2 = roads[rIdxs[rr]]
    if not r2.isJctRoad then
      local lMin2, lMax2 = profileMgr.getMinMaxLaneKeys(r2.profile)
      local numLanes2 = 0
      local lanes2, ctr = {}, 1
      for i = lMin2, lMax2 do
        if i ~= 0 then
          numLanes2 = numLanes2 + 1
          lanes2[ctr] = i
          ctr = ctr + 1
        end
      end

      -- Filter by case, and find linkable matches between the test and trial candidate roads.
      local isMatch, joinClass = false, nil
      local lie1, lie2 = nIdx1 == 1, nIdxs[rr] == 1
      if (not r1.isBridge and not r2.isBridge) and r1.isOverlay == r2.isOverlay then
        if isOverlayMatch(r1, r2) then
          isMatch, joinClass = true, 'Overlay -> Overlay'
        elseif isPerfectMatch(r1, r2, lie1, lie2, numLanes1, numLanes2, lanes1, lanes2) then
          isMatch, joinClass = true, 'Perfect'
        elseif is1WHwyToRural(r1, r2, lie1, lie2, numLanes1, numLanes2, lanes1, lanes2) then
          isMatch, joinClass = true, '1Way Hwy -> Rural'
        elseif is1WHwyToRural(r2, r1, lie1, lie2, numLanes2, numLanes1, lanes2, lanes1) then
          isMatch, joinClass = true, '1Way Rural -> Hwy'
        elseif isSidewalkToNoSidewalk(r1, r2, numLanes1, numLanes2, lanes1, lanes2) then
          isMatch, joinClass = true, 'Sidewalk -> No Sidewalk'
        elseif isSidewalkToNoSidewalk(r2, r1, numLanes2, numLanes1, lanes2, lanes1) then
          isMatch, joinClass = true, 'No Sidewalk -> Sidewalk'
        elseif isUrban1WTo2W(r1, r2, lie1, lie2, lanes1, lanes2) then
          isMatch, joinClass = true, 'Urban 1Way -> 2Way'
        elseif isUrban1WTo2W(r2, r1, lie2, lie1, lanes2, lanes1) then
          isMatch, joinClass = true, 'Urban 2Way -> 1Way'
        elseif isUrbanTaperUp(r1, r2, numLanes1, numLanes2, lanes1, lanes2) then
          isMatch, joinClass = true, 'Urban Taper Up'
        elseif isUrbanTaperUp(r2, r1, numLanes2, numLanes1, lanes2, lanes1) then
          isMatch, joinClass = true, 'Urban Taper Down'
        elseif isUrbanToHwy(r1, r2, lanes1, lanes2) then
          isMatch, joinClass = true, 'Urban -> Highway'
        elseif isUrbanToHwy(r2, r1, lanes2, lanes1) then
          isMatch, joinClass = true, 'Highway -> Urban'
        elseif isHwyTaperUp(r1, r2, lanes1, lanes2) then
          isMatch, joinClass = true, 'Highway Taper Up'
        elseif isHwyTaperUp(r2, r1, lanes2, lanes1) then
          isMatch, joinClass = true, 'Highway Taper Down'
        elseif isHwy1WTo2W(r1, r2, lie1, lie2, lanes1, lanes2) then
          isMatch, joinClass = true, 'Highway 1Way -> 2Way'
        elseif isHwy1WTo2W(r2, r1, lie2, lie1, lanes2, lanes1) then
          isMatch, joinClass = true, 'Highway 2Way -> 1Way'
        end
      end

      if isMatch then
        local r1Lie, r2Lie = 'end', 'end'
        if lie1 then r1Lie = 'start' end
        if lie2 then r2Lie = 'start' end
        links[#links + 1] = { r1Name = r1.name, r1Lie = r1Lie, r2Name = r2.name, r2Lie = r2Lie, class = joinClass }
      end
    end
  end

  -- Determine the closest candidate, and return that.
  local dBestSq, bestIdx = 1e99, 1
  for i = 2, #links do
    local link = links[i]
    local r1, r2 = roads[roadMap[link.r1Name]], roads[roadMap[link.r2Name]]
    local r1Lie, r2Lie = link.r1Lie, link.r2Lie
    local distSq = 1e99
    if r1Lie == 'start' and r2Lie == 'start' then
      distSq = r1.nodes[1].p:squaredDistance(r2.nodes[1].p)
    elseif r1Lie == 'start' and r2Lie == 'end' then
      distSq = r1.nodes[1].p:squaredDistance(r2.nodes[#r2.nodes].p)
    elseif r1Lie == 'end' and r2Lie == 'start' then
      distSq = r1.nodes[#r1.nodes].p:squaredDistance(r2.nodes[1].p)
    elseif r1Lie == 'end' and r2Lie == 'end' then
      distSq = r1.nodes[#r1.nodes].p:squaredDistance(r2.nodes[#r2.nodes].p)
    end
    if distSq < dBestSq then
      dBestSq, bestIdx = distSq, i
    end
  end

  return links[bestIdx]
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

-- Joins two overlays together.
local function createOverlayJoin(r1, r2, lie1, lie2)
  local nodes = {}
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
  r1.nodes = nodes

  r1.displayName = im.ArrayChar(32, 'Joined Overlay')

  -- Update the roads list and map.
  roadMgr.removeRoad(r2.name)
  roadMgr.setDirty(r1)
  roadMgr.recomputeMap()
end

-- Joins a 1-way highway road to a 1-way rural/urban road.
local function create1WayHwy2Rural(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = jPoint - pPen
  tgt:normalize()
  local jCenter = jPoint + tgt * 10.0
  jctMgr.addShoulderFadeJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(r2.profile))
  local _, numRight = profileMgr.getNumRoadLanesLR(r2.profile)
  jct.numLanesX = im.IntPtr(numRight)
  if lie1 == 'start' then
    local jctRoad1 = roads[roadMap[jct.roads[1]]]
    local jVec = -(jctRoad1.nodes[#jctRoad1.nodes].p - jctRoad1.nodes[1].p)
    local rot = util.getRotationBetweenVecs(jVec, tgt)
    jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
    jct.isY1Outwards = im.BoolPtr(false)
  else
    local jctRoad1 = roads[roadMap[jct.roads[1]]]
    local jVec = -(jctRoad1.nodes[#jctRoad1.nodes].p - jctRoad1.nodes[1].p)
    local rot = util.getRotationBetweenVecs(jVec, tgt)
    jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
    jct.isY1Outwards = im.BoolPtr(true)
  end
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  if jct.isY1Outwards[0] then
    createPerfectJoin(r1, roads[roadMap[jct.roads[1]]], lie1, 'start')
    createPerfectJoin(r2, roads[roadMap[jct.roads[2]]], lie2, 'end')
  else
    createPerfectJoin(r1, roads[roadMap[jct.roads[1]]], lie1, 'end')
    createPerfectJoin(r2, roads[roadMap[jct.roads[2]]], lie2, 'start')
  end
end

-- Joins a 1-way rural/urban road to a 1-way highway road.
local function create1WayRural2Hwy(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = jPoint - pPen
  tgt:normalize()
  local jCenter = jPoint + tgt * 10.0
  jctMgr.addShoulderFadeJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(r1.profile))
  local _, numRight = profileMgr.getNumRoadLanesLR(r1.profile)
  jct.numLanesX = im.IntPtr(numRight)
  if lie1 == 'start' then
    local jctRoad1 = roads[roadMap[jct.roads[1]]]
    local jVec = jctRoad1.nodes[#jctRoad1.nodes].p - jctRoad1.nodes[1].p
    local rot = util.getRotationBetweenVecs(jVec, tgt)
    jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
    jct.isY1Outwards = im.BoolPtr(true)
  else
    local jctRoad1 = roads[roadMap[jct.roads[1]]]
    local jVec = jctRoad1.nodes[#jctRoad1.nodes].p - jctRoad1.nodes[1].p
    local rot = util.getRotationBetweenVecs(jVec, tgt)
    jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
    jct.isY1Outwards = im.BoolPtr(false)
  end
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  if jct.isY1Outwards[0] then
    createPerfectJoin(r1, roads[roadMap[jct.roads[2]]], lie1, 'end')
    createPerfectJoin(r2, roads[roadMap[jct.roads[1]]], lie2, 'start')
  else
    createPerfectJoin(r1, roads[roadMap[jct.roads[2]]], lie1, 'start')
    createPerfectJoin(r2, roads[roadMap[jct.roads[1]]], lie2, 'end')
  end
end

-- Joins an urban road with sidewalks to an urban road without sidewalks, using a transition junction.
local function createSidewalk2NoSidewalk(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = jPoint - pPen
  tgt:normalize()
  local jCenter = jPoint + tgt * 20.0
  jctMgr.addRuralUrbanTransJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.isSidewalk = im.BoolPtr(true)
  local _, numRight = profileMgr.getNumRoadLanesLR(r1.profile)
  jct.numLanesX = im.IntPtr(numRight)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  createPerfectJoin(r1, roads[roadMap[jct.roads[1]]], lie1, 'start')
  createPerfectJoin(r2, roads[roadMap[jct.roads[2]]], lie2, 'start')
end

-- Joins an urban road without sidewalks to an urban road with sidewalks, using a transition junction.
local function createNoSidewalk2Sidewalk(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = pPen - jPoint
  tgt:normalize()
  local jCenter = jPoint - tgt * 20.0
  jctMgr.addRuralUrbanTransJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.isSidewalk = im.BoolPtr(true)
  local _, numRight = profileMgr.getNumRoadLanesLR(r1.profile)
  jct.numLanesX = im.IntPtr(numRight)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  createPerfectJoin(r1, roads[roadMap[jct.roads[2]]], lie1, 'start')
  createPerfectJoin(r2, roads[roadMap[jct.roads[1]]], lie2, 'start')
end

-- Joins a thinner urban road with a wider urban road, using a transition junction.
local function createUrbanTaperUp(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = jPoint - pPen
  tgt:normalize()
  local jCenter = jPoint + tgt * 20.0
  jctMgr.addUrbanMergeJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(r1.profile))
  local _, numRight = profileMgr.getNumRoadLanesLR(r1.profile)
  jct.numLanesX = im.IntPtr(numRight)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  createPerfectJoin(r1, roads[roadMap[jct.roads[1]]], lie1, 'start')
  createPerfectJoin(r2, roads[roadMap[jct.roads[2]]], lie2, 'start')
end

-- Joins a wider urban road to a thinner urban road, using a transition junction.
local function createUrbanTaperDown(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = pPen - jPoint
  tgt:normalize()
  local jCenter = jPoint - tgt * 20.0
  jctMgr.addUrbanMergeJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(r1.profile))
  local _, numRight = profileMgr.getNumRoadLanesLR(r2.profile)
  jct.numLanesX = im.IntPtr(numRight)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  createPerfectJoin(r1, roads[roadMap[jct.roads[2]]], lie1, 'start')
  createPerfectJoin(r2, roads[roadMap[jct.roads[1]]], lie2, 'start')
end

-- Joins an urban one-way road to an urban two-way road, using a transition junction.
local function createUrban1WTo2W(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = pPen - jPoint
  tgt:normalize()
  local jCenter = jPoint - tgt * 10.0
  jctMgr.addUrbanSeparatorJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(r1.profile))
  local _, numRight = profileMgr.getNumRoadLanesLR(r1.profile)
  jct.numLanesX = im.IntPtr(numRight)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  if lie1 == 'start' then
    createPerfectJoin(r1, roads[roadMap[jct.roads[2]]], lie1, 'end')
  else
    createPerfectJoin(r1, roads[roadMap[jct.roads[3]]], lie1, 'start')
  end
  createPerfectJoin(r2, roads[roadMap[jct.roads[1]]], lie2, 'start')
end

-- Joins an urban two-way road to an urban one-way road, using a transition junction.
local function createUrban2WTo1W(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = jPoint - pPen
  tgt:normalize()
  local jCenter = jPoint + tgt * 10.0
  jctMgr.addUrbanSeparatorJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(r1.profile))
  local _, numRight = profileMgr.getNumRoadLanesLR(r2.profile)
  jct.numLanesX = im.IntPtr(numRight)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  createPerfectJoin(r1, roads[roadMap[jct.roads[1]]], lie1, 'start')
  if lie2 == 'start' then
    createPerfectJoin(r2, roads[roadMap[jct.roads[2]]], lie2, 'end')
  else
    createPerfectJoin(r2, roads[roadMap[jct.roads[3]]], lie2, 'start')
  end
end

-- Joins an urban road to a highway road, using a transition junction.
local function createUrban2Hwy(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = pPen - jPoint
  tgt:normalize()
  local jCenter = jPoint - tgt * 10.0
  jctMgr.addHighwayUrbanTransJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.l2Length = im.FloatPtr(20.0)
  jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(r1.profile))
  local _, numRight = profileMgr.getNumRoadLanesLR(r1.profile)
  jct.numLanesX = im.IntPtr(numRight)
  jct.isYOneWay = im.BoolPtr(false)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  createPerfectJoin(r1, roads[roadMap[jct.roads[2]]], lie1, 'start')
  createPerfectJoin(r2, roads[roadMap[jct.roads[1]]], lie2, 'start')
end

-- Joins a highway road to an urban road, using a transition junction.
local function createHwy2Urban(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = jPoint - pPen
  tgt:normalize()
  local jCenter = jPoint + tgt * 20.0
  jctMgr.addHighwayUrbanTransJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.l2Length = im.FloatPtr(20.0)
  jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(r2.profile))
  local _, numRight = profileMgr.getNumRoadLanesLR(r2.profile)
  jct.numLanesX = im.IntPtr(numRight)
  jct.isYOneWay = im.BoolPtr(false)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  createPerfectJoin(r1, roads[roadMap[jct.roads[1]]], lie1, 'start')
  createPerfectJoin(r2, roads[roadMap[jct.roads[2]]], lie2, 'start')
end

-- Joins a thinner highway road to a wider highway road, using a transition junction.
local function createHwyTaperUp(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = jPoint - pPen
  tgt:normalize()
  local jCenter = jPoint - tgt * 10.0
  jctMgr.addHighwayMergeJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.l2Length = im.FloatPtr(10.0)
  jct.isSidewalk = im.BoolPtr(false)
  local _, numRight = profileMgr.getNumRoadLanesLR(r1.profile)
  jct.numLanesX = im.IntPtr(numRight - 1)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  createPerfectJoin(r1, roads[roadMap[jct.roads[1]]], lie1, 'start')
  createPerfectJoin(r2, roads[roadMap[jct.roads[2]]], lie2, 'start')
end

-- Joins a wider highway road to a thinner highway road, using a transition junction.
local function createHwyTaperDown(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = pPen - jPoint
  tgt:normalize()
  local jCenter = jPoint - tgt * 10.0
  jctMgr.addHighwayMergeJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.l2Length = im.FloatPtr(10.0)
  jct.isSidewalk = im.BoolPtr(false)
  local _, numRight = profileMgr.getNumRoadLanesLR(r2.profile)
  jct.numLanesX = im.IntPtr(numRight - 1)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  createPerfectJoin(r1, roads[roadMap[jct.roads[2]]], lie1, 'start')
  createPerfectJoin(r2, roads[roadMap[jct.roads[1]]], lie2, 'start')
end

-- Joins a one-way highway road to a two-way highway road, using a transition junction.
local function createHwy1WTo2W(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = pPen - jPoint
  tgt:normalize()
  local jCenter = jPoint - tgt * 10.0
  jctMgr.addHighwaySeparatorJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(r1.profile))
  local _, numRight = profileMgr.getNumRoadLanesLR(r1.profile)
  jct.numLanesX = im.IntPtr(numRight - 1)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  if lie1 == 'start' then
    createPerfectJoin(r1, roads[roadMap[jct.roads[2]]], lie1, 'end')
  else
    createPerfectJoin(r1, roads[roadMap[jct.roads[3]]], lie1, 'start')
  end
  createPerfectJoin(r2, roads[roadMap[jct.roads[1]]], lie2, 'start')
end

-- Joins a two-way highway road to a one-way highway road, using a transition junction.
local function createHwy2WTo1W(r1, r2, lie1, lie2)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local jPoint, pPen = nil, nil
  if lie2 == 'start' then
    jPoint, pPen = r2.nodes[1].p, r2.nodes[2].p
  else
    jPoint, pPen = r2.nodes[#r2.nodes].p, r2.nodes[#r2.nodes - 1].p
  end
  local tgt = jPoint - pPen
  tgt:normalize()
  local jCenter = jPoint - tgt * 10.0
  jctMgr.addHighwaySeparatorJunction(true)
  local jIdx = #jctMgr.junctions
  local jct = jctMgr.junctions[jIdx]
  jctMgr.translateJunction(jIdx, jCenter)
  local jctRoad1 = roads[roadMap[jct.roads[1]]]
  local jVec = jctRoad1.nodes[1].p - jctRoad1.nodes[#jctRoad1.nodes].p
  local rot = util.getRotationBetweenVecs(jVec, tgt)
  jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
  jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(r1.profile))
  local _, numRight = profileMgr.getNumRoadLanesLR(r2.profile)
  jct.numLanesX = im.IntPtr(numRight - 1)
  jctMgr.updateJunctionAfterChange(jIdx)
  jctMgr.finaliseJunction(jIdx)
  createPerfectJoin(r1, roads[roadMap[jct.roads[1]]], lie1, 'start')
  if lie2 == 'start' then
    createPerfectJoin(r2, roads[roadMap[jct.roads[2]]], lie2, 'end')
  else
    createPerfectJoin(r2, roads[roadMap[jct.roads[3]]], lie2, 'start')
  end
end

-- Join the roads described by the given linkage data.
local function joinRoads(link)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local r1, r2 = roads[roadMap[link.r1Name]], roads[roadMap[link.r2Name]]
  local lie1, lie2 = link.r1Lie, link.r2Lie
  local class = link.class

  if class == 'Perfect' then
    createPerfectJoin(r1, r2, lie1, lie2)
  elseif class == 'Overlay -> Overlay' then
    createOverlayJoin(r1, r2, lie1, lie2)
  elseif class == '1Way Hwy -> Rural' then
    create1WayHwy2Rural(r1, r2, lie1, lie2)
  elseif class == '1Way Rural -> Hwy' then
    create1WayRural2Hwy(r1, r2, lie1, lie2)
  elseif class == 'Sidewalk -> No Sidewalk' then
    createSidewalk2NoSidewalk(r1, r2, lie1, lie2)
  elseif class == 'No Sidewalk -> Sidewalk' then
    createNoSidewalk2Sidewalk(r1, r2, lie1, lie2)
  elseif class == 'Urban Taper Up' then
    createUrbanTaperUp(r1, r2, lie1, lie2)
  elseif class == 'Urban Taper Down' then
    createUrbanTaperDown(r1, r2, lie1, lie2)
  elseif class == 'Urban 1Way -> 2Way' then
    createUrban1WTo2W(r1, r2, lie1, lie2)
  elseif class == 'Urban 2Way -> 1Way' then
    createUrban2WTo1W(r1, r2, lie1, lie2)
  elseif class == 'Urban -> Highway' then
    createUrban2Hwy(r1, r2, lie1, lie2)
  elseif class == 'Highway -> Urban' then
    createHwy2Urban(r1, r2, lie1, lie2)
  elseif class == 'Highway Taper Up' then
    createHwyTaperUp(r1, r2, lie1, lie2)
  elseif class == 'Highway Taper Down' then
    createHwyTaperDown(r1, r2, lie1, lie2)
  elseif class == 'Highway 1Way -> 2Way' then
    createHwy1WTo2W(r1, r2, lie1, lie2)
  elseif class == 'Highway 2Way -> 1Way' then
    createHwy2WTo1W(r1, r2, lie1, lie2)
  end
end

-- Deep copies the state of all roads (used for undo/redo support).
local function copyRoadState()
  local cRoads = {}
  for i = 1, #roadMgr.roads do
    cRoads[i] = roadMgr.copyRoad(roadMgr.roads[i])
  end
  return cRoads
end

-- Computes a structure contain the data for a candidate junction.
local function computePossibleJct(rIdx, nIdx, rIdxs, nIdxs)
  local roads = roadMgr.roads
  local road1 = roads[rIdx]
  if road1.isBridge or road1.isOverlay then                                                                                     -- Bridges and overlays are not included in jcts.
    return nil
  end
  local p1 = road1.nodes[nIdx].p
  local isR1Sidewalks = profileMgr.areSidewalksPresent(road1.profile)
  local bestDist, bestIdx = 1e99, 1
  local bestType, class, isFound = 'Y', nil, false
  for i = 1, #rIdxs do
    local rOther = roads[rIdxs[i]]
    if not rOther.isBridge and not rOther.isOverlay then
      local isR2Sidewalks = profileMgr.areSidewalksPresent(rOther.profile)
      if isR1Sidewalks == isR2Sidewalks then                                                                                    -- Ensure all roads have or dont have sidewalks (must match).
        local numLeft1, numRight1 = profileMgr.getNumLanesLR(roads[rIdx].profile)
        local numLeft2, numRight2 = profileMgr.getNumLanesLR(rOther.profile)

        if numLeft1 == numLeft2 and numRight1 == numRight2 and profileMgr.isProfileValidForMidJctPerfect(rOther.profile) then   -- Test for a 'perfect' symmetric/profile-matched junction fit.

          local p2 = rOther.nodes[nIdxs[i]].p
          local dSq = util.sqDist2D(p1, p2)
          if dSq < bestDist then
            bestDist, bestIdx = dSq, i
            local vT = rOther.nodes[min(#rOther.nodes, nIdxs[i] + 1)].p - rOther.nodes[max(1, nIdxs[i] - 1)].p
            vT:normalize()
            local pPen = nil
            if nIdx == 1 then
              pPen = 2
            else
              pPen = #roads[rIdx].nodes - 1
            end
            local vJ = roads[rIdx].nodes[pPen].p - p1
            vJ:normalize()
            bestType = 'Y'
            class = '2W -> 2W Perfect. Y-Junction'
            if abs(vT:dot(vJ)) < 0.4 then
              bestType = 'T'
              class = '2W -> 2W Perfect. T-Junction'
            end
            isFound = true
          end

        else                                                                                                                    -- Test for non-perfect and one-way connections.

          local r1NumLeftRoad1, _ = profileMgr.getNumRoadLanesLR(road1.profile)
          local r2NumLeftRoad1, _ = profileMgr.getNumRoadLanesLR(rOther.profile)
          if r1NumLeftRoad1 == 0 then
            if r2NumLeftRoad1 > 0 then                                                                                          -- r1 is 1-way, r2 is 1-way.                                                                                                               -- r1 is 1-way, r2 is 2-way.
              local p2 = rOther.nodes[nIdxs[i]].p
              local dSq = util.sqDist2D(p1, p2)
              if dSq < bestDist then
                bestDist, bestIdx = dSq, i
                local vT = rOther.nodes[min(#rOther.nodes, nIdxs[i] + 1)].p - rOther.nodes[max(1, nIdxs[i] - 1)].p
                vT:normalize()
                local pPen = nil
                if nIdx == 1 then
                  pPen = 2
                else
                  pPen = #roads[rIdx].nodes - 1
                end
                local vJ = roads[rIdx].nodes[pPen].p - p1
                vJ:normalize()
                if abs(vT:dot(vJ)) < 0.4 then
                  bestType = 'T'
                  if nIdx == 1 then
                    class = '1W [in] -> 2W. T-Junction'
                  else
                    class = '1W [out] -> 2W. T-Junction'
                  end
                  isFound = true
                end
              end
            end
          else
            if numLeft1 == numRight1 and numLeft2 == numRight2 then                                                             -- r1 is 2-way, r2 is 2-way, symmetric, but not perfect.
              local p2 = rOther.nodes[nIdxs[i]].p
              local dSq = util.sqDist2D(p1, p2)
              if dSq < bestDist then
                bestDist, bestIdx = dSq, i
                local vT = rOther.nodes[min(#rOther.nodes, nIdxs[i] + 1)].p - rOther.nodes[max(1, nIdxs[i] - 1)].p
                vT:normalize()
                local pPen = nil
                if nIdx == 1 then
                  pPen = 2
                else
                  pPen = #roads[rIdx].nodes - 1
                end
                local vJ = roads[rIdx].nodes[pPen].p - p1
                vJ:normalize()
                if abs(vT:dot(vJ)) < 0.4 then
                  bestType = 'T'
                  class = '2W -> 2W Lane Change. T-Junction'
                  isFound = true
                end
              end
            end
          end
        end
      end
    end
  end
  if isFound then
    return { jName = roads[rIdx].name, jNode = nIdx, tName = roads[rIdxs[bestIdx]].name, tNode = nIdxs[bestIdx], type = bestType, class = class }
  end
  return nil
end

-- Creates the proposed junction which will split an existing road, place a T-junction in, and join to the new trunk and second road.
local function createSplitJunction(candJct)
  local roads, roadMap = roadMgr.roads, roadMgr.map
  local tName, tNode, jName, jNode = candJct.tName, candJct.tNode, candJct.jName, candJct.jNode
  local tRoadIdx, jRoadIdx = roadMap[tName], roadMap[jName]
  local tRoad, jRoad = roads[tRoadIdx], roads[jRoadIdx]
  local jLie = 'start'
  if jNode ~= 1 then
    jLie = 'end'
  end

  -- Before making any changes, copy the road structure state in case we need to revert.
  local preRoadState = copyRoadState()

  -- Split the trunk road and get references to the two new roads.
  roadMgr.splitRoad(tRoadIdx, tNode)
  local rNewT1Idx, rNewT2Idx = #roads - 1, #roads
  local rNewT1, rNewT2 = roads[rNewT1Idx], roads[rNewT2Idx]

  -- Compute the center and radius of the junction sphere [with some padding].
  local jCenter = tRoad.nodes[tNode].p
  local numLanesT, _ = profileMgr.getNumLanesLR(rNewT1.profile)
  local numLanesJ, _ = profileMgr.getNumLanesLR(jRoad.profile)
  local jRad = max(numLanesT, numLanesJ) * 4 + 15
  local jRadSq = jRad * jRad

  -- Clear any nearby (up to 3) trunk road nodes which are inside the junction sphere.
  for i = #rNewT1.nodes - 1, max(1, #rNewT1.nodes - 4), -1 do
    if rNewT1.nodes[i].p:squaredDistance(jCenter) < jRadSq then
      roadMgr.removeNode(rNewT1Idx, i)
    end
  end
  for i = min(#rNewT2.nodes, 4), 2, -1 do
    if rNewT2.nodes[i].p:squaredDistance(jCenter) < jRadSq then
      roadMgr.removeNode(rNewT2Idx, i)
    end
  end

  -- If there is a road with less than two nodes, do nothing and leave immediately.
  if #rNewT1.nodes < 2 or #rNewT2.nodes < 2 then
    table.clear(roadMgr.roads)
    for i = 1, #preRoadState do                                                                     -- Recover the road state before attempting to fit the junction.
      roadMgr.roads[i] = preRoadState[i]
    end
    roadMgr.recomputeMap()
    return
  end

  -- Repel the trunk road join points to the perimeter of the junction sphere, and add stiffening nodes to help match the tangent.
  local tgt = rNewT1.nodes[#rNewT1.nodes - 1].p - rNewT2.nodes[1].p
  tgt:normalize()
  local dSq = rNewT1.nodes[#rNewT1.nodes].p:squaredDistance(jCenter)
  if dSq < jRadSq then
    local pOld = rNewT1.nodes[#rNewT1.nodes].p
    rNewT1.nodes[#rNewT1.nodes].p = pOld + tgt * (jRad - sqrt(dSq))
    roadMgr.addIntermediateNode(rNewT1Idx, #rNewT1.nodes, 'below')                                  -- Add a new node to help with tangent matching on the trunk road.
    rNewT1.nodes[#rNewT1.nodes - 1].p = pOld + tgt * (jRad - sqrt(dSq) + 2)
  end
  local dSq = rNewT2.nodes[1].p:squaredDistance(jCenter)
  if dSq < jRadSq then
    local pOld = rNewT2.nodes[1].p
    rNewT2.nodes[1].p = pOld - tgt * (jRad - sqrt(dSq))
    roadMgr.addIntermediateNode(rNewT2Idx, 1, 'above')                                              -- Add a new node to help with tangent matching on the trunk road.
    rNewT2.nodes[2].p = pOld - tgt * (jRad - sqrt(dSq) + 2)
  end

  -- Repel the join road to the perimeter of the junction sphere, and at a stiffening node to help match the tangent.
  local lat = vertical:cross(tgt)
  local dSq = jRoad.nodes[jNode].p:squaredDistance(jCenter)
  jRoadIdx = roadMap[jName]
  if dSq < jRadSq then
    if jNode == 1 then
      local pOld = jRoad.nodes[1].p
      jRoad.nodes[1].p = pOld - lat * (jRad - sqrt(dSq))
      roadMgr.addIntermediateNode(jRoadIdx, 1, 'above')                                             -- Add a new node to help with tangent matching on the trunk road.
      jRoad.nodes[2].p = pOld - lat * (jRad - sqrt(dSq) + 2)
    else
      local pOld = jRoad.nodes[#jRoad.nodes].p
      jRoad.nodes[#jRoad.nodes].p = pOld - lat * (jRad - sqrt(dSq))
      roadMgr.addIntermediateNode(jRoadIdx, #jRoad.nodes, 'below')                                  -- Add a new node to help with tangent matching on the trunk road.
      jRoad.nodes[#jRoad.nodes - 1].p = pOld - lat * (jRad - sqrt(dSq) + 2)
    end
  end

  -- Update the render data for each exit road.
  roadMgr.computeRoadRenderDataSingle(roadMgr.map[jRoad.name])
  roadMgr.computeRoadRenderDataSingle(roadMgr.map[rNewT1.name])
  roadMgr.computeRoadRenderDataSingle(roadMgr.map[rNewT2.name])

  -- Filter by junction class and join type.
  local junctions = jctMgr.junctions
  local jIdx = nil
  if candJct.type == 'T' then

    -- Create a T-junction.
    jctMgr.addTJunction(true)
    jIdx = #junctions
    local jct = junctions[jIdx]
    jctMgr.translateJunction(jIdx, jCenter)
    local jVec = roads[roadMap[jct.roads[4]]].nodes[1].p - roads[roadMap[jct.roads[5]]].nodes[1].p
    local jCurr = rNewT2.nodes[1].p - rNewT1.nodes[#rNewT1.nodes].p
    local rot = util.getRotationBetweenVecs(jVec, jCurr)
    jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
    local jYOuter = roads[roadMap[jct.roads[6]]].nodes[1].p
    local test1 = vec3(jYOuter.x, jYOuter.y, jYOuter.z)                                               -- Sample the end Y point in configuration #1.
    jctMgr.rotateJunction(jIdx, jCenter, pi)
    local jYOuter = roads[roadMap[jct.roads[6]]].nodes[1].p
    local test2 = vec3(jYOuter.x, jYOuter.y, jYOuter.z)                                               -- Sample the end Y point in configuration #2 (rotated by pi).

    -- Add the appropriate junction properties.
    jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(rNewT1.profile) or profileMgr.areSidewalksPresent(jRoad.profile))
    local _, numTRight = profileMgr.getNumRoadLanesLR(rNewT1.profile)
    jct.numLanesX = im.IntPtr(numTRight)
    local numJLeft, numJRight = profileMgr.getNumRoadLanesLR(jRoad.profile)
    jct.numLanesY = im.IntPtr(numJRight)
    jct.isYOneWay = im.BoolPtr(numJLeft == 0)
    if candJct.class == '1W [in] -> 2W. T-Junction' then
      jct.isY2Outwards = im.BoolPtr(false)
    elseif candJct.class == '1W [out] -> 2W. T-Junction' then
      jct.isY2Outwards = im.BoolPtr(true)
    end

    jctMgr.updateJunctionAfterChange(jIdx)

    -- Orient the junction to the correct configuration (based on shortest distance to link).
    local jPenult = nil
    if jNode == 1 then
      jPenult = jRoad.nodes[3].p
    else
      jPenult = jRoad.nodes[#jRoad.nodes - 2].p
    end
    local dist1, dist2 = jPenult:squaredDistance(test1), jPenult:squaredDistance(test2)
    local orient = 1
    if dist1 < dist2 then
      jctMgr.rotateJunction(jIdx, jCenter, pi)
      orient = 2
    end

    -- Move linking points from each of the three roads to the junction ends, and link them.
    local jX1Outer = roads[roadMap[jct.roads[4]]].nodes[1].p
    local jX2Outer = roads[roadMap[jct.roads[5]]].nodes[1].p
    local road4 = roads[roadMap[jct.roads[4]]]
    local road5 = roads[roadMap[jct.roads[5]]]
    local road6 = roads[roadMap[jct.roads[6]]]
    local pp = road6.nodes[1].p
    local pPen = road6.nodes[2].p
    if jct.isYOneWay[0] and not jct.isY2Outwards[0] then
      pp = road6.nodes[#road6.nodes].p
      pPen = road6.nodes[#road6.nodes - 1].p
    end
    if jLie == 'start' then
      jRoad.nodes[1].p = vec3(pp.x, pp.y, pp.z)
      jRoad.nodes[2].p = jRoad.nodes[1].p + (pp - pPen):normalized() * 5
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[jRoad.name])
      if jct.isYOneWay[0] and not jct.isY2Outwards[0] then
        joinRoads({ r1Name = jRoad.name, r1Lie = 'start', r2Name = road6.name, r2Lie = 'end', class = 'Perfect' })
      else
        joinRoads({ r1Name = jRoad.name, r1Lie = 'start', r2Name = road6.name, r2Lie = 'start', class = 'Perfect' })
      end
    else
      jRoad.nodes[#jRoad.nodes].p = vec3(pp.x, pp.y, pp.z)
      jRoad.nodes[#jRoad.nodes - 1].p = jRoad.nodes[#jRoad.nodes].p + (pp - pPen):normalized() * 5
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[jRoad.name])
      if jct.isYOneWay[0] and not jct.isY2Outwards[0] then
        joinRoads({ r1Name = jRoad.name, r1Lie = 'end', r2Name = road6.name, r2Lie = 'end', class = 'Perfect' })
      else
        joinRoads({ r1Name = jRoad.name, r1Lie = 'end', r2Name = road6.name, r2Lie = 'start', class = 'Perfect' })
      end
    end
    if orient == 1 then
      rNewT1.nodes[#rNewT1.nodes].p = vec3(jX1Outer.x, jX1Outer.y, jX1Outer.z)
      rNewT1.nodes[#rNewT1.nodes - 1].p = rNewT1.nodes[#rNewT1.nodes].p + (road4.nodes[1].p - road4.nodes[2].p):normalized() * 5
      rNewT2.nodes[1].p = vec3(jX2Outer.x, jX2Outer.y, jX2Outer.z)
      rNewT2.nodes[2].p = rNewT2.nodes[1].p + (road5.nodes[1].p - road5.nodes[2].p):normalized() * 5
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[rNewT1.name])
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[rNewT2.name])
      joinRoads({ r1Name = rNewT1.name, r1Lie = 'end', r2Name = road4.name, r2Lie = 'start', class = 'Perfect' })
      joinRoads({ r1Name = rNewT2.name, r1Lie = 'start', r2Name = road5.name, r2Lie = 'start', class = 'Perfect' })
    else
      rNewT1.nodes[#rNewT1.nodes].p = vec3(jX2Outer.x, jX2Outer.y, jX2Outer.z)
      rNewT1.nodes[#rNewT1.nodes - 1].p = rNewT1.nodes[#rNewT1.nodes].p + (road5.nodes[1].p - road5.nodes[2].p):normalized() * 5
      rNewT2.nodes[1].p = vec3(jX1Outer.x, jX1Outer.y, jX1Outer.z)
      rNewT2.nodes[2].p = rNewT2.nodes[1].p + (road4.nodes[1].p - road4.nodes[2].p):normalized() * 5
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[rNewT1.name])
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[rNewT2.name])
      joinRoads({ r1Name = rNewT1.name, r1Lie = 'end', r2Name = road5.name, r2Lie = 'start', class = 'Perfect' })
      joinRoads({ r1Name = rNewT2.name, r1Lie = 'start', r2Name = road4.name, r2Lie = 'start', class = 'Perfect' })
    end

  else

    -- Create a Y-junction.
    jctMgr.addYJunction(true)
    jIdx = #junctions
    local jct = junctions[jIdx]
    jctMgr.translateJunction(jIdx, jCenter)
    local jVec = roads[roadMap[jct.roads[1]]].nodes[1].p - roads[roadMap[jct.roads[2]]].nodes[1].p
    local jCurr = rNewT2.nodes[1].p - rNewT1.nodes[#rNewT1.nodes].p
    local rot = util.getRotationBetweenVecs(jVec, jCurr)
    jctMgr.rotateJunctionQuat(jIdx, jCenter, rot)
    local jYOuter = roads[roadMap[jct.roads[3]]].nodes[1].p
    local test1 = vec3(jYOuter.x, jYOuter.y, jYOuter.z)                                               -- Sample the end Y point in configuration #1.
    jctMgr.rotateJunction(jIdx, jCenter, pi)
    local jYOuter = roads[roadMap[jct.roads[3]]].nodes[1].p
    local test2 = vec3(jYOuter.x, jYOuter.y, jYOuter.z)                                               -- Sample the end Y point in configuration #2 (rotated by pi).

    -- Add the appropriate junction properties.
    jct.isSidewalk = im.BoolPtr(profileMgr.areSidewalksPresent(rNewT1.profile) or profileMgr.areSidewalksPresent(jRoad.profile))
    local numT, _ = profileMgr.getNumRoadLanesLR(rNewT1.profile)
    jct.numLanesX = im.IntPtr(numT)
    local numJ, _ = profileMgr.getNumRoadLanesLR(jRoad.profile)
    jct.numLanesY = im.IntPtr(numJ)

    local vT = rNewT1.nodes[#rNewT1.nodes].p - rNewT2.nodes[1].p
    vT:normalize()
    local pPen = nil
    if jNode == 1 then
      pPen = 2
    else
      pPen = #jRoad.nodes - 1
    end
    local vJ = jRoad.nodes[pPen].p - jRoad.nodes[jNode].p
    vJ:normalize()
    local jctAngle = max(-30.0, min(30.0, rad2Deg(util.angleBetweenVecs(vT, vJ))))
    local dist1 = jRoad.nodes[jNode].p:squaredDistance(rNewT1.nodes[#rNewT1.nodes].p)
    local dist2 = jRoad.nodes[jNode].p:squaredDistance(rNewT2.nodes[1].p)
    if dist2 > dist1 then
      jctAngle = -jctAngle
    end
    local jPenult = nil
    if jNode == 1 then
      jPenult = jRoad.nodes[3].p
    else
      jPenult = jRoad.nodes[#jRoad.nodes - 2].p
    end
    local dist1, dist2 = jPenult:squaredDistance(test1), jPenult:squaredDistance(test2)
    local orient = 2
    if dist1 > dist2 then
      jctAngle = -jctAngle
      orient = 1
    end
    jct.theta = im.FloatPtr(jctAngle)

    jctMgr.updateJunctionAfterChange(jIdx)

    -- Orient the junction to the correct configuration (based on shortest distance to link).
    if orient == 2 then
      jctMgr.rotateJunction(jIdx, jCenter, pi)
    end

    -- Move linking points from each of the three roads to the junction ends, and link them.
    local jX1Outer = roads[roadMap[jct.roads[1]]].nodes[1].p
    local jX2Outer = roads[roadMap[jct.roads[2]]].nodes[1].p
    if jLie == 'start' then
      local pp = roadMgr.roads[roadMap[jct.roads[3]]].nodes[1].p
      jRoad.nodes[1].p = vec3(pp.x, pp.y, pp.z)
      jRoad.nodes[2].p = jRoad.nodes[1].p + (roads[roadMap[jct.roads[3]]].nodes[1].p - roads[roadMap[jct.roads[3]]].nodes[2].p):normalized() * 5
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[jRoad.name])
      joinRoads({ r1Name = roads[roadMap[jct.roads[3]]].name, r1Lie = 'start', r2Name = jRoad.name, r2Lie = 'start', class = 'Perfect' })
    else
      local pp = roads[roadMap[jct.roads[3]]].nodes[1].p
      jRoad.nodes[#jRoad.nodes].p = vec3(pp.x, pp.y, pp.z)
      jRoad.nodes[#jRoad.nodes - 1].p = jRoad.nodes[#jRoad.nodes].p + (roads[roadMap[jct.roads[3]]].nodes[1].p - roads[roadMap[jct.roads[3]]].nodes[2].p):normalized() * 5
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[jRoad.name])
      joinRoads({ r1Name = roads[roadMap[jct.roads[3]]].name, r1Lie = 'start', r2Name = jRoad.name, r2Lie = 'end', class = 'Perfect' })
    end
    if orient == 1 then
      rNewT1.nodes[#rNewT1.nodes].p = vec3(jX1Outer.x, jX1Outer.y, jX1Outer.z)
      rNewT1.nodes[#rNewT1.nodes - 1].p = rNewT1.nodes[#rNewT1.nodes].p + (roads[roadMap[jct.roads[1]]].nodes[1].p - roads[roadMap[jct.roads[1]]].nodes[2].p):normalized() * 5
      rNewT2.nodes[1].p = vec3(jX2Outer.x, jX2Outer.y, jX2Outer.z)
      rNewT2.nodes[2].p = rNewT2.nodes[1].p + (roads[roadMap[jct.roads[2]]].nodes[1].p - roads[roadMap[jct.roads[2]]].nodes[2].p):normalized() * 5
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[rNewT1.name])
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[rNewT2.name])
      joinRoads({ r1Name = roads[roadMap[jct.roads[1]]].name, r1Lie = 'start', r2Name = rNewT1.name, r2Lie = 'end', class = 'Perfect' })
      joinRoads({ r1Name = roads[roadMap[jct.roads[2]]].name, r1Lie = 'start', r2Name = rNewT2.name, r2Lie = 'start', class = 'Perfect' })
    else
      rNewT1.nodes[#rNewT1.nodes].p = vec3(jX2Outer.x, jX2Outer.y, jX2Outer.z)
      rNewT1.nodes[#rNewT1.nodes - 1].p = rNewT1.nodes[#rNewT1.nodes].p + (roads[roadMap[jct.roads[2]]].nodes[1].p - roads[roadMap[jct.roads[2]]].nodes[2].p):normalized() * 5
      rNewT2.nodes[1].p = vec3(jX1Outer.x, jX1Outer.y, jX1Outer.z)
      rNewT2.nodes[2].p = rNewT2.nodes[1].p + (roads[roadMap[jct.roads[1]]].nodes[1].p - roads[roadMap[jct.roads[1]]].nodes[2].p):normalized() * 5
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[rNewT1.name])
      roadMgr.computeRoadRenderDataSingle(roadMgr.map[rNewT2.name])
      joinRoads({ r1Name = roads[roadMap[jct.roads[2]]].name, r1Lie = 'start', r2Name = rNewT1.name, r2Lie = 'end', class = 'Perfect' })
      joinRoads({ r1Name = roads[roadMap[jct.roads[1]]].name, r1Lie = 'start', r2Name = rNewT2.name, r2Lie = 'start', class = 'Perfect' })
    end
  end

  jctMgr.finaliseJunction(jIdx)
  roadMgr.recomputeMap()
end


-- Public interface.
M.computePossibleLinks =                                computePossibleLinks
M.joinRoads =                                           joinRoads

M.computePossibleJct =                                  computePossibleJct
M.createSplitJunction =                                 createSplitJunction

return M