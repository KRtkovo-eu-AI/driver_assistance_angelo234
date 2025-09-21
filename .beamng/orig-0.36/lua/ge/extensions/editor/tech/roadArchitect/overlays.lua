-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local roadMgr = require('editor/tech/roadArchitect/roads')                                          -- A module for managing the road structure/handling road calculations.
local profileMgr = require('editor/tech/roadArchitect/profiles')                                    -- A module for managing road profiles.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.

-- Module constants.
local deg45 = 0.785398163
local vertical = vec3(0, 0, 1)


-- Adds overlay roads to a crossroads junction.
local function addCrossroadsOverlays(jct)
  local roads, map = roadMgr.roads, roadMgr.map
  math.randomseed(jct.seed[0])
  for _ = 1, jct.numCrossings[0] do
    local isGood = false
    while not isGood do
      local exitS, exitE = util.randomInRange(1, 4), util.randomInRange(1, 4)
      if exitS ~= exitE then
        local rS = nil
        if exitS == 1 then
          rS = roads[map[jct.roads[5]]]
        elseif exitS == 2 then
          rS = roads[map[jct.roads[6]]]
        elseif exitS == 3 then
          rS = roads[map[jct.roads[7]]]
        else
          rS = roads[map[jct.roads[8]]]
        end
        local rE = nil
        if exitE == 1 then
          rE = roads[map[jct.roads[5]]]
        elseif exitE == 2 then
          rE = roads[map[jct.roads[6]]]
        elseif exitE == 3 then
          rE = roads[map[jct.roads[7]]]
        else
          rE = roads[map[jct.roads[8]]]
        end

        local profS, profE = rS.profile, rE.profile
        local _, rightS = profileMgr.getNumRoadLanesLR(profS)
        local leftE, _ = profileMgr.getNumRoadLanesLR(profE)
        if rightS > 0 and leftE > 0 then
          local laneS, laneE = util.randomInRange(1, rightS), util.randomInRange(-leftE, -1)
          local rDataS, rDataE = rS.renderData, rE.renderData

          local p1, p2 = nil, nil
          if exitS == 1 or exitS == 2 or not jct.isYOneWay[0] then
            p1 = rDataS[1][laneS][7]
            p2 = rDataS[#rDataS][laneS][7]
          elseif exitS == 3 and jct.isYOneWay[0] then
            if jct.isY1Outwards[0] then
              p1 = rDataS[1][laneS][7]
              p2 = rDataS[#rDataS][laneS][7]
            else
              p1 = rDataS[#rDataS][laneS][7]
              p2 = rDataS[1][laneS][7]
            end
          elseif exitS == 4 and jct.isYOneWay[0] then
            if jct.isY2Outwards[0] then
              p1 = rDataS[1][laneS][7]
              p2 = rDataS[#rDataS][laneS][7]
            else
              p1 = rDataS[#rDataS][laneS][7]
              p2 = rDataS[1][laneS][7]
            end
          end

          local p3, p4 = nil, nil
          if exitE == 1 or exitE == 2 or not jct.isYOneWay[0] then
            p3 = rDataE[#rDataE][laneE][7]
            p4 = rDataE[1][laneE][7]
          elseif exitE == 3 and jct.isYOneWay[0] then
            if jct.isY1Outwards[0] then
              p3 = rDataE[#rDataE][laneE][7]
              p4 = rDataE[1][laneE][7]
            else
              p3 = rDataE[1][laneE][7]
              p4 = rDataE[#rDataE][laneE][7]
            end
          elseif exitE == 4 and jct.isYOneWay[0] then
            if jct.isY2Outwards[0] then
              p3 = rDataE[#rDataE][laneE][7]
              p4 = rDataE[1][laneE][7]
            else
              p3 = rDataE[1][laneE][7]
              p4 = rDataE[#rDataE][laneE][7]
            end
          end

          local cProf = profileMgr.createOverlayProfile(jct.laneWidthX[0])
          local cRoad = roadMgr.createRoadFromProfile(cProf)
          cRoad.isOverlay = true
          cRoad.isJctRoad = true
          cRoad.isDrivable = false
          jct.roads[#jct.roads + 1] = cRoad.name
          local rIdx = #roads + 1
          roads[rIdx] = cRoad
          roadMgr.addNodeToRoad(rIdx, p1)
          roadMgr.addNodeToRoad(rIdx, p2)
          roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.25, 0.75))
          roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.5, 0.75))
          roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.75, 0.75))
          roadMgr.addNodeToRoad(rIdx, p3)
          roadMgr.addNodeToRoad(rIdx, p4)
          isGood = true
        end
      end
    end
  end
  roadMgr.recomputeMap()
end

-- Adds overlay roads to a T-junction.
local function addTJunctionOverlays(jct)
  local roads, map = roadMgr.roads, roadMgr.map
  math.randomseed(jct.seed[0])
  for _ = 1, jct.numCrossings[0] do
    local isGood = false
    while not isGood do
      local exitS, exitE = util.randomInRange(1, 3), util.randomInRange(1, 3)
      if exitS ~= exitE then
        local rS = nil
        if exitS == 1 then
          rS = roads[map[jct.roads[4]]]
        elseif exitS == 2 then
          rS = roads[map[jct.roads[5]]]
        else
          rS = roads[map[jct.roads[6]]]
        end
        local rE = nil
        if exitE == 1 then
          rE = roads[map[jct.roads[4]]]
        elseif exitE == 2 then
          rE = roads[map[jct.roads[5]]]
        else
          rE = roads[map[jct.roads[6]]]
        end

        local profS, profE = rS.profile, rE.profile
        local _, rightS = profileMgr.getNumRoadLanesLR(profS)
        local leftE, _ = profileMgr.getNumRoadLanesLR(profE)
        if rightS > 0 and leftE > 0 then
          local laneS, laneE = util.randomInRange(1, rightS), util.randomInRange(-leftE, -1)
          local rDataS, rDataE = rS.renderData, rE.renderData

          local p1, p2 = nil, nil
          if exitS == 1 or exitS == 2 or not jct.isYOneWay[0] then
            p1 = rDataS[1][laneS][7]
            p2 = rDataS[#rDataS][laneS][7]
          elseif exitS == 3 and jct.isYOneWay[0] then
            if jct.isY2Outwards[0] then
              p1 = rDataS[1][laneS][7]
              p2 = rDataS[#rDataS][laneS][7]
            else
              p1 = rDataS[#rDataS][laneS][7]
              p2 = rDataS[1][laneS][7]
            end
          end

          local p3, p4 = nil, nil
          if exitE == 1 or exitE == 2 or not jct.isYOneWay[0] then
            p3 = rDataE[#rDataE][laneE][7]
            p4 = rDataE[1][laneE][7]
          elseif exitE == 3 and jct.isYOneWay[0] then
            if jct.isY2Outwards[0] then
              p3 = rDataE[#rDataE][laneE][7]
              p4 = rDataE[1][laneE][7]
            else
              p3 = rDataE[1][laneE][7]
              p4 = rDataE[#rDataE][laneE][7]
            end
          end

          local cProf = profileMgr.createOverlayProfile(jct.laneWidthX[0])
          local cRoad = roadMgr.createRoadFromProfile(cProf)
          cRoad.isOverlay = true
          cRoad.isJctRoad = true
          cRoad.isDrivable = false
          jct.roads[#jct.roads + 1] = cRoad.name
          local rIdx = #roads + 1
          roads[rIdx] = cRoad
          roadMgr.addNodeToRoad(rIdx, p1)
          roadMgr.addNodeToRoad(rIdx, p2)
          roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.25, 0.75))
          roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.5, 0.75))
          roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.75, 0.75))
          roadMgr.addNodeToRoad(rIdx, p3)
          roadMgr.addNodeToRoad(rIdx, p4)
          isGood = true
        end
      end
    end
  end
  roadMgr.recomputeMap()
end

-- Adds overlay roads to a Y-junction.
local function addYJunctionOverlays(jct)
  local roads, map = roadMgr.roads, roadMgr.map
  math.randomseed(jct.seed[0])
  for _ = 1, jct.numCrossings[0] do
    local isGood = false
    while not isGood do
      local exitS, exitE = util.randomInRange(1, 3), util.randomInRange(1, 3)
      if exitS ~= exitE then
        local rS = nil
        if exitS == 1 then
          rS = roads[map[jct.roads[1]]]
        elseif exitS == 2 then
          rS = roads[map[jct.roads[2]]]
        else
          rS = roads[map[jct.roads[3]]]
        end
        local rE = nil
        if exitE == 1 then
          rE = roads[map[jct.roads[1]]]
        elseif exitE == 2 then
          rE = roads[map[jct.roads[2]]]
        else
          rE = roads[map[jct.roads[3]]]
        end

        local profS, profE = rS.profile, rE.profile
        local _, rightS = profileMgr.getNumRoadLanesLR(profS)
        local leftE, _ = profileMgr.getNumRoadLanesLR(profE)
        if rightS > 0 and leftE > 0 then
          local laneS, laneE = util.randomInRange(1, rightS), util.randomInRange(-leftE, -1)
          local rDataS, rDataE = rS.renderData, rE.renderData
          local p1 = rDataS[1][laneS][7]
          local p2 = rDataS[#rDataS][laneS][7]
          local p3 = rDataE[#rDataE][laneE][7]
          local p4 = rDataE[1][laneE][7]

          local cProf = profileMgr.createOverlayProfile(jct.laneWidthX[0])
          local cRoad = roadMgr.createRoadFromProfile(cProf)
          cRoad.isOverlay = true
          cRoad.isJctRoad = true
          cRoad.isDrivable = false
          jct.roads[#jct.roads + 1] = cRoad.name
          local rIdx = #roads + 1
          roads[rIdx] = cRoad
          roadMgr.addNodeToRoad(rIdx, p1)
          roadMgr.addNodeToRoad(rIdx, p2)
          roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.25, 0.75))
          roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.5, 0.75))
          roadMgr.addNodeToRoad(rIdx, catmullRomChordal(p1, p2, p3, p4, 0.75, 0.75))
          roadMgr.addNodeToRoad(rIdx, p3)
          roadMgr.addNodeToRoad(rIdx, p4)
          isGood = true
        end
      end
    end
  end
  roadMgr.recomputeMap()
end

-- Adds overlay roads to a roundabout junction.
local function addRoundaboutOverlays(jct, cen)
  local roads, map = roadMgr.roads, roadMgr.map
  math.randomseed(jct.seed[0])
  for _ = 1, jct.numCrossings[0] do
    local isGood = false
    while not isGood do
      local exitS, exitE = util.randomInRange(1, 4), util.randomInRange(1, 4)
      if exitS ~= exitE then
        local rS = nil
        if exitS == 1 then
          rS = roads[map[jct.roads[5]]]
        elseif exitS == 2 then
          rS = roads[map[jct.roads[6]]]
        elseif exitS == 3 then
          rS = roads[map[jct.roads[7]]]
        else
          rS = roads[map[jct.roads[8]]]
        end
        local rE = nil
        if exitE == 1 then
          rE = roads[map[jct.roads[5]]]
        elseif exitE == 2 then
          rE = roads[map[jct.roads[6]]]
        elseif exitE == 3 then
          rE = roads[map[jct.roads[7]]]
        else
          rE = roads[map[jct.roads[8]]]
        end

        local pts = {}
        local rr = roads[map[jct.roads[9]]]
        for j = 1, jct.numRBLanes[0] do
          local lPts = {}
          local rBVec = rr.renderData[1][-j][7] - cen
          for k = 0, 16 do
            lPts[k + 1] = cen + util.rotateVecAroundAxis(rBVec, vertical, -deg45 * k)
          end
          pts[-j] = lPts
        end

        local profS, profE = rS.profile, rE.profile
        local _, rightS = profileMgr.getNumRoadLanesLR(profS)
        local leftE, _ = profileMgr.getNumRoadLanesLR(profE)
        if rightS > 0 and leftE > 0 then
          local laneS = util.randomInRange(1, rightS)
          local laneE = -laneS
          local rDataS, rDataE = rS.renderData, rE.renderData

          local p1 = rDataS[1][laneS][7]
          local p2 = rDataS[#rDataS][laneS][7]
          local p3 = rDataE[#rDataE][laneE][7]
          local p4 = rDataE[1][laneE][7]

          local cProf = profileMgr.createOverlayProfile(jct.laneWidthX[0])
          local cRoad = roadMgr.createRoadFromProfile(cProf)
          cRoad.isOverlay = true
          cRoad.isJctRoad = true
          cRoad.isDrivable = false
          jct.roads[#jct.roads + 1] = cRoad.name
          local rIdx = #roads + 1
          roads[rIdx] = cRoad
          roadMgr.addNodeToRoad(rIdx, p1)
          roadMgr.addNodeToRoad(rIdx, p2)
          local seqLane = -laneS
          local seqS, seqE = nil, nil
          if exitS == 1 then
            seqS = 8
          elseif exitS == 2 then
            seqS = 4
          elseif exitS == 3 then
            seqS = 6
          else
            seqS = 2
          end
          if exitE == 1 then
            seqE = 6
          elseif exitE == 2 then
            seqE = 2
          elseif exitE == 3 then
            seqE = 4
          else
            seqE = 8
          end
          if seqE < seqS then
            seqE = seqE + 8
          end
          if seqE > seqS + 2 then
            for j = seqS, seqE do
              roadMgr.addNodeToRoad(rIdx, pts[seqLane][j])
            end
            roadMgr.addNodeToRoad(rIdx, p3)
            roadMgr.addNodeToRoad(rIdx, p4)
            isGood = true
          end
        end
      end
    end
  end
  roadMgr.recomputeMap()
end

-- Adds overlay roads to a highway merge junction.
local function addHighwayMergeOverlays(jct)
  local roads, map = roadMgr.roads, roadMgr.map
  math.randomseed(jct.seed[0])
  for _ = 1, jct.numCrossings[0] do
    local numLanes = jct.numLanesX[0]
    local r1 = roads[map[jct.roads[1]]]
    local r2 = roads[map[jct.roads[3]]]
    local r3 = roads[map[jct.roads[2]]]

    local cProf = profileMgr.createOverlayProfile(jct.laneWidthX[0])
    local cRoad = roadMgr.createRoadFromProfile(cProf)
    cRoad.isOverlay = true
    cRoad.isJctRoad = true
    cRoad.isDrivable = false
    jct.roads[#jct.roads + 1] = cRoad.name
    local rIdx = #roads + 1
    roads[rIdx] = cRoad

    local laneSign = 1.0
    if math.random() < 0.5 then
      laneSign = -1.0
    end
    local lIdx = util.randomInRange(2, numLanes + 1) * laneSign
    roadMgr.addNodeToRoad(rIdx, r1.renderData[1][lIdx][7])
    roadMgr.addNodeToRoad(rIdx, r1.renderData[#r1.renderData][lIdx][7])

    local tra1 = math.random()
    if tra1 < 0.25 and r2.profile[lIdx - 1] and r2.profile[lIdx - 1].type == 'road_lane' then
      lIdx = lIdx - 1
    elseif tra1 > 0.75 and r2.profile[lIdx + 1] and r2.profile[lIdx + 1].type == 'road_lane' then
      lIdx = lIdx + 1
    end
    roadMgr.addNodeToRoad(rIdx, r2.renderData[#r2.renderData][lIdx][7])

    if lIdx < 0 and r3.profile[lIdx].type ~= 'road_lane' then
      lIdx = lIdx - 1
    elseif lIdx > 0 and r3.profile[lIdx].type ~= 'road_lane' then
      lIdx = lIdx + 1
    end
    roadMgr.addNodeToRoad(rIdx, r3.renderData[1][-lIdx][7])
  end
  roadMgr.recomputeMap()
end

-- Adds overlay roads to a highway-urban transition junction.
local function addHighwayTransOverlays(jct)
  local roads, map = roadMgr.roads, roadMgr.map
  math.randomseed(jct.seed[0])
  for _ = 1, jct.numCrossings[0] do
    local numLanes = jct.numLanesX[0]
    local r1 = roads[map[jct.roads[1]]]
    local r2 = roads[map[jct.roads[3]]]
    local r3 = roads[map[jct.roads[2]]]

    local cProf = profileMgr.createOverlayProfile(jct.laneWidthX[0])
    local cRoad = roadMgr.createRoadFromProfile(cProf)
    cRoad.isOverlay = true
    cRoad.isJctRoad = true
    cRoad.isDrivable = false
    jct.roads[#jct.roads + 1] = cRoad.name
    local rIdx = #roads + 1
    roads[rIdx] = cRoad

    local laneSign = 1.0
    if math.random() < 0.5 then
      laneSign = -1.0
    end
    local lIdx = util.randomInRange(2, numLanes + 1) * laneSign
    roadMgr.addNodeToRoad(rIdx, r1.renderData[1][lIdx][7])
    roadMgr.addNodeToRoad(rIdx, r1.renderData[#r1.renderData][lIdx][7])

    local tra1 = math.random()
    if tra1 < 0.25 and r2.profile[lIdx - 1] and r2.profile[lIdx - 1].type == 'road_lane' then
      lIdx = lIdx - 1
    elseif tra1 > 0.75 and r2.profile[lIdx + 1] and r2.profile[lIdx + 1].type == 'road_lane' then
      lIdx = lIdx + 1
    end

    if lIdx > 0 then
      lIdx = lIdx - 1
    else
      lIdx = lIdx + 1
    end
    roadMgr.addNodeToRoad(rIdx, r3.renderData[#r3.renderData][-lIdx][7])
    roadMgr.addNodeToRoad(rIdx, r3.renderData[1][-lIdx][7])
  end
  roadMgr.recomputeMap()
end

-- Adds overlay roads to a highway slip junction.
local function addHighwaySlipOverlays(jct)
  local roads, map = roadMgr.roads, roadMgr.map
  math.randomseed(jct.seed[0])
  for _ = 1, jct.numCrossings[0] do
    local numLanes = jct.numLanesX[0]
    local r1 = roads[map[jct.roads[1]]]
    local r2 = roads[map[jct.roads[3]]]
    local r3 = roads[map[jct.roads[4]]]
    local r4 = roads[map[jct.roads[2]]]
    local rEL = roads[map[jct.roads[5]]]
    local rER = roads[map[jct.roads[6]]]

    local cProf = profileMgr.createOverlayProfile(jct.laneWidthX[0])
    local cRoad = roadMgr.createRoadFromProfile(cProf)
    cRoad.isOverlay = true
    cRoad.isJctRoad = true
    cRoad.isDrivable = false
    jct.roads[#jct.roads + 1] = cRoad.name
    local rIdx = #roads + 1
    roads[rIdx] = cRoad

    local laneSign = 1.0
    if math.random() < 0.5 then
      laneSign = -1.0
    end
    local lIdx = util.randomInRange(2, numLanes + 1) * laneSign
    roadMgr.addNodeToRoad(rIdx, r1.renderData[1][lIdx][7])
    roadMgr.addNodeToRoad(rIdx, r1.renderData[#r1.renderData][lIdx][7])

    local tra1 = math.random()
    if tra1 < 0.25 and r2.profile[lIdx - 1] and r2.profile[lIdx - 1].type == 'road_lane' then
      lIdx = lIdx - 1
    elseif tra1 > 0.75 and r2.profile[lIdx + 1] and r2.profile[lIdx + 1].type == 'road_lane' then
      lIdx = lIdx + 1
    end
    roadMgr.addNodeToRoad(rIdx, r2.renderData[#r2.renderData][lIdx][7])

    if lIdx < 0 and r3.profile[lIdx].type ~= 'road_lane' then
      lIdx = lIdx - 1
    elseif lIdx > 0 and r3.profile[lIdx].type ~= 'road_lane' then
      lIdx = lIdx + 1
    end
    roadMgr.addNodeToRoad(rIdx, r3.renderData[#r3.renderData][lIdx][7])

    local lMin4, lMax4 = profileMgr.getMinMaxLaneKeys(r3.profile)
    if lIdx == lMin4 + 1 then
      roadMgr.addNodeToRoad(rIdx, rEL.renderData[1][1][7])
    elseif lIdx == lMax4 - 1 then
      roadMgr.addNodeToRoad(rIdx, rER.renderData[#rER.renderData][1][7])
    else
      roadMgr.addNodeToRoad(rIdx, r4.renderData[1][-lIdx][7])
    end
  end
  roadMgr.recomputeMap()
end


-- Public interface.
M.addCrossroadsOverlays =                                 addCrossroadsOverlays
M.addTJunctionOverlays =                                  addTJunctionOverlays
M.addYJunctionOverlays =                                  addYJunctionOverlays
M.addRoundaboutOverlays =                                 addRoundaboutOverlays

M.addHighwayMergeOverlays =                               addHighwayMergeOverlays
M.addHighwayTransOverlays =                               addHighwayTransOverlays
M.addHighwaySlipOverlays =                                addHighwaySlipOverlays

return M