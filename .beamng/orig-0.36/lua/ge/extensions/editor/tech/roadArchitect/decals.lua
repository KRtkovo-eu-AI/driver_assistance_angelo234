-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local extraGapSpace = 0.45                                                                          -- The gap at the start/end of roads, by which to lay spanning layers short.

local lightTreadMaterial = 'm_tread_marks_clean'                                                    -- The 'light tread/wear' material.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}


-- External modules used.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A utilities module.

-- Private constants.
local abs, min, max, sin, cos, deg2Rad = math.abs, math.min, math.max, math.sin, math.cos, math.rad
local vertical = vec3(0, 0, 1)
local raised = vec3(0, 0, 4.2)

-- Module state.
local asphalts = {}
local decals = {}
local templates = {}
local templateMap = {}


-- Compute the road sections from a given road.
-- [This is an array of contiguous sections of road lanes, with start and end lane indices].
local function computeSectionsByType(road, laneType)
  local sections, sCtr, s = {}, 1, nil
  local profile = road.profile
  for i = -20, 11 do                                                                                -- Iterate over all possible lanes, from left to right.
    if i ~= 0 then
      local lane = profile[i]
      if lane and lane.type == laneType then
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

-- Compute the nodes/widths for the asphalt of a given section of a given road.
-- [The nodes are on the section centerline and the widths respect the section width].
local function computeSectionGeom(road, sStart, sEnd)
  local rData = road.renderData
  local nodes, widths, numDivs = {}, {}, #rData
  for i = 1, numDivs do
    local rD = rData[i]
    local pL, pR = rD[sStart][1], rD[sEnd][2]
    nodes[i], widths[i] = (pL + pR) * 0.5, pR:distance(pL)                                          -- Use the midpoint for this node.
  end
  return nodes, widths
end

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

-- Creates a decal set for the given road.
-- [A decal set contains the main road decal, the centerline decal, road edge decals, lane division decals, and start/end junction line decals].
-- [The user can control the combination of these which will be rendered for the road, using the appropriate Imgui buttons].
local function createDecal(road, folder)

  -- The decal does not exist, so create a new decal per section.
  local name = road.name
  local renderPrKeyRoad = #road.profile.layers + 3
  asphalts[name] = computeSectionsByType(road, 'road_lane')
  local numSections = #asphalts[name]
  for s = 1, numSections do

    -- Create the navigation graph road, if desired [eg not if road is just an overlay road].
    if road.isDrivable then                                                                         -- Render standard, drivable road (not an overlay).
      local sStart, sEnd = asphalts[name][s].s, asphalts[name][s].e
      local section = asphalts[name][s]
      asphalts[name][s].road = createObject("DecalRoad")
      local dRoad = asphalts[name][s].road
      dRoad:setField("improvedSpline", 0, "true")
      if road.isOverObject[0] then
        dRoad:setField("overObjects", 0, "true")
      end
      dRoad:setField("textureLength", 0, 36)
      dRoad:setField("renderPriority", 0, renderPrKeyRoad)
      dRoad:setField("autoLanes", 0, "false")
      dRoad:setField("oneWay", 0, tostring(section.isOneWay))
      dRoad:setField("lanesLeft", 0, section.lanesL)
      dRoad:setField("lanesRight", 0, section.lanesR)
      dRoad:setField("drivability", 0, 1.0)
      dRoad:setField("material", 0, "road_invisible")
      dRoad:registerObject("")
      folder:addObject(dRoad)
      local nodes, sectionWidths = computeSectionGeom(road, sStart, sEnd)
      local numNodes, decalId = #nodes, dRoad:getID()
      local last = vec3(0, 0, 1e24)
      if sStart > 0 and sEnd > 0 then
        local ctr = 0
        for i = 1, numNodes do                                                                      -- For +ve only sections, we need to flip the node order.
          local idx = numNodes - i + 1
          local pos = nodes[idx] + raised
          if pos:squaredDistance(last) > 1.0 then
            editor.addRoadNode(decalId, { pos = pos, width = sectionWidths[idx], index = ctr })
            ctr = ctr + 1
            last = pos
          end
        end
      else
        local ctr = 0
        for i = 1, numNodes do
          local pos = nodes[i] + raised
          if pos:squaredDistance(last) > 1.0 then
            editor.addRoadNode(decalId, { pos = pos, width = sectionWidths[i], index = ctr })
            ctr = ctr + 1
            last = pos
          end
        end
      end

    elseif road.isOverlay then                                                                      -- Render overlay road (not a drivable road).

      asphalts[name][s].road = createObject("DecalRoad")
      local dRoad = asphalts[name][s].road
      dRoad:setField("improvedSpline", 0, "true")
      if road.isOverObject[0] then
        dRoad:setField("overObjects", 0, "true")
      end
      dRoad:setField("textureLength", 0, 36)
      dRoad:setField("renderPriority", 0, renderPrKeyRoad)
      dRoad:setField("drivability", 0, -1.0)
      dRoad:setField("material", 0, road.overlayMat or lightTreadMaterial)
      dRoad:setField('startEndFade', 0, string.format("%f %f", 5.0, 5.0))
      dRoad:registerObject("")
      folder:addObject(dRoad)
      local decalId = dRoad:getID()
      local last = vec3(0, 0, 1e24)
      local ctr = 0
      for i = 1, #road.nodes do
        local pos = road.nodes[i].p + raised
        if pos:squaredDistance(last) > 1.0 then
          editor.addRoadNode(decalId, { pos = pos, width = road.nodes[i].widths[1][0], index = ctr })
          ctr = ctr + 1
          last = pos
        end
      end
    end
  end

  math.randomseed(road.profile.conditionSeed[0])

  -- Create the decals for each layer stored in the profile.
  local profile = road.profile
  local layers = profile.layers
  local rData = road.renderData
  decals[name] = { layers = {}, templates = {}, instances = {} }
  local ctrL, ctrD = 1, 1
  for i = 1, #layers do
    local layer = layers[i]
    local layerType = layer.type[0]

    if layerType == 0 then                                                                          -- TYPE: [SPAN LANE].

      decals[name].layers[ctrL] = createObject("DecalRoad")
      local layerDecal = decals[name].layers[ctrL]
      ctrL = ctrL + 1
      layerDecal:setField("improvedSpline", 0, "true")
      if road.isOverObject[0] then
        layerDecal:setField("overObjects", 0, "true")
      end
      layerDecal:setField("renderPriority", 0, i)
      layerDecal:setField("textureLength", 0, layer.texLen[0])
      layerDecal:setField('startEndFade', 0, string.format("%f %f", layer.fadeS[0], layer.fadeE[0]))
      layerDecal:setField("material", 0, layer.mat)
      layerDecal:setField("drivability", 0, -1.0)
      layerDecal:setField("hidden", 0, "false")
      layerDecal:registerObject("")
      folder:addObject(layerDecal)
      local decalId = layerDecal:getID()

      local laneL, laneR, off = layer.laneMin[0], layer.laneMax[0], layer.off[0]
      local startDivIdx, endDivIdx = 1, #rData
      if not layer.isSpanLong[0] then                                                               -- If the user has limited the longitudinal span, only iterate in the chosen interval.
        startDivIdx = util.computeDivIndicesFromNode(layer.nMin[0], road)
        endDivIdx = util.computeDivIndicesFromNode(layer.nMax[0], road)
      end
      local dNodes, dWidths, latOffs, ctr = {}, {}, {}, 1
      for j = startDivIdx, endDivIdx do
        local rDL, rDR = rData[j][laneL], rData[j][laneR]
        local width = 0.0
        for k = laneL, laneR do
          if k ~= 0 then
            width = width + rData[j][k][9]
          end
        end
        dNodes[ctr] = (rDL[1] + rDR[2]) * 0.5
        dWidths[ctr] = width
        latOffs[ctr] = rDL[6] * off
        ctr = ctr + 1
      end

      -- If the road is a paint line, then lay it short (except in the case of linkage points).
      if layer.isPaint[0] then
        dNodes[1] = dNodes[1] + (dNodes[2] - dNodes[1]):normalized() * extraGapSpace
        dNodes[#dNodes] = dNodes[#dNodes] + (dNodes[#dNodes - 1] - dNodes[#dNodes]):normalized() * extraGapSpace
      end

      local last = vec3(0, 0, 1e24)
      if layer.isReverse[0] then
        local ctr = 0
        for j = #dNodes, 1, -1 do
          local pos = dNodes[j] + latOffs[j] + raised
          if pos:squaredDistance(last) > 1.0 then
            editor.addRoadNode(decalId, { pos = pos, width = dWidths[j], index = ctr })
            ctr = ctr + 1
            last = pos
          end
        end
      else
        local ctr = 0
        for j = 1, #dNodes do
          local pos = dNodes[j] + latOffs[j] + raised
          if pos:squaredDistance(last) > 1.0 then
            editor.addRoadNode(decalId, { pos = pos, width = dWidths[j], index = ctr })
            ctr = ctr + 1
            last = pos
          end
        end
      end

    elseif layerType == 1 then                                                                      -- TYPE: [OFFSET FROM LANE].

      decals[name].layers[ctrL] = createObject("DecalRoad")
      local layerDecal = decals[name].layers[ctrL]
      ctrL = ctrL + 1
      layerDecal:setField("improvedSpline", 0, "true")
      if road.isOverObject[0] then
        layerDecal:setField("overObjects", 0, "true")
      end
      layerDecal:setField("renderPriority", 0, i)
      layerDecal:setField("textureLength", 0, layer.texLen[0])
      layerDecal:setField('startEndFade', 0, string.format("%f %f", layer.fadeS[0], layer.fadeE[0]))
      layerDecal:setField("material", 0, layer.mat)
      layerDecal:setField("drivability", 0, -1.0)
      layerDecal:setField("hidden", 0, "false")
      layerDecal:registerObject("")
      folder:addObject(layerDecal)
      local decalId = layerDecal:getID()

      local sideIdx = 2
      if layer.isLeft[0] then
        sideIdx = 1
      end
      local startDivIdx, endDivIdx = 1, #rData
      if not layer.isSpanLong[0] then                                                               -- If the user has limited the longitudinal span, only iterate in chosen interval.
        startDivIdx = util.computeDivIndicesFromNode(layer.nMin[0], road)
        endDivIdx = util.computeDivIndicesFromNode(layer.nMax[0], road)
      end
      local lane, off, fixedWidth = layer.lane[0], layer.off[0], layer.width[0]

      local dNodes, latOffs, ctr = {}, {}, 1
      for j = startDivIdx, endDivIdx do
        dNodes[ctr] = rData[j][lane][sideIdx]
        local rD = rData[j][lane]
        latOffs[ctr] = rD[6] * off
        ctr = ctr + 1
      end

      -- If the road is a paint line, then lay it short (except in the case of linkage points).
      if layer.isPaint[0] then
        dNodes[1] = dNodes[1] + (dNodes[2] - dNodes[1]):normalized() * extraGapSpace
        dNodes[#dNodes] = dNodes[#dNodes] + (dNodes[#dNodes - 1] - dNodes[#dNodes]):normalized() * extraGapSpace
      end

      local last = vec3(0, 0, 1e24)
      if layer.isReverse[0] then
        local ctr = 0
        for j = #dNodes, 1, -1 do
          local pos = dNodes[j] + latOffs[j] + raised
          if pos:squaredDistance(last) > 1.0 then
            editor.addRoadNode(decalId, { pos = pos, width = fixedWidth, index = ctr })
            ctr = ctr + 1
            last = pos
          end
        end
      else
        local ctr = 0
        for j = 1, #dNodes do
          local pos = dNodes[j] + latOffs[j] + raised
          if pos:squaredDistance(last) > 1.0 then
            editor.addRoadNode(decalId, { pos = pos, width = fixedWidth, index = ctr })
            ctr = ctr + 1
            last = pos
          end
        end
      end

    elseif layerType == 2 then                                                                      -- TYPE: [UNIQUE LATERAL PATCH (NON-DECAL)].

      decals[name].layers[ctrL] = createObject("DecalRoad")
      local layerDecal = decals[name].layers[ctrL]
      ctrL = ctrL + 1
      layerDecal:setField("improvedSpline", 0, "true")
      if road.isOverObject[0] then
        layerDecal:setField("overObjects", 0, "true")
      end
      layerDecal:setField("renderPriority", 0, i)
      layerDecal:setField("textureLength", 0, layer.texLen[0])
      layerDecal:setField("material", 0, layer.mat)
      layerDecal:setField("drivability", 0, -1.0)
      layerDecal:setField("hidden", 0, "false")
      layerDecal:registerObject("")
      folder:addObject(layerDecal)
      local decalId = layerDecal:getID()

      -- Compute the relevant points, using linear interpolation.
      local lMin, lMax = layer.laneMin[0], layer.laneMax[0]
      local qq = layer.off[0]
      local lengths = util.computeRoadLength(rData)
      local pEval = qq * lengths[#lengths]                                                          -- The longitudinal evaluation position on the road, in meters.
      local lower, upper = util.findBounds(pEval, lengths)
      local q = (pEval - lengths[lower]) / (lengths[upper] - lengths[lower])                        -- The q in [0, 1] between div points (linear interpolation).
      local pL1 = rData[lower][lMin][1]
      local pL2 = rData[upper][lMin][1]
      local pL = pL1 + q * (pL2 - pL1)                                                              -- The evaluation on the left edge.
      local pR1 = rData[lower][lMax][2]
      local pR2 = rData[upper][lMax][2]
      local pR = pR1 + q * (pR2 - pR1)                                                              -- The evaluation on the right edge.

      -- Compute the local tangent at the road centerline, along which to protrude the patch.
      local cenIdx1, cenIdx2 = 1, 1
      if rData[1][-1] then
        cenIdx1, cenIdx2 = -1, 2
      end
      local tgt = rData[upper][cenIdx1][cenIdx2] - rData[lower][cenIdx1][cenIdx2]
      tgt:normalize()

      -- Compute the four quadrilateral vertices of the patch.
      local tgtVec = tgt * layer.width[0] * 0.5
      local f1, f2 = pL + tgtVec, pR + tgtVec

      if layer.isPaint[0] then
        if layer.off[0] > 0.99 then
          f1 = f1 - tgt * extraGapSpace
          f2 = f2 - tgt * extraGapSpace
        elseif layer.off[0] < 0.01 then
          f1, f2 = pL - tgtVec, pR - tgtVec
          f1 = f1 + tgt * extraGapSpace
          f2 = f2 + tgt * extraGapSpace
        end
      end

      editor.addRoadNode(decalId, { pos = f1 + raised, width = layer.width[0], index = 0 })
      editor.addRoadNode(decalId, { pos = f2 + raised, width = layer.width[0], index = 1 })

    elseif layerType == 3 then                                                                      -- TYPE: [UNIQUE DECAL PATCH].

      local lIdx = layer.lane[0]
      local lengths = util.computeRoadLength(rData)
      local pEval = layer.off[0] * lengths[#lengths]                                                -- The longitudinal evaluation position on the road, in meters.
      local lower, upper = util.findBounds(pEval, lengths)
      local q = (pEval - lengths[lower]) / (lengths[upper] - lengths[lower])                        -- The q in [0, 1] between div points (linear interpolation).
      local pL = nil
      if layer.isLeft[0] then
        local pL1 = rData[lower][lIdx][1]
        local pL2 = rData[upper][lIdx][1]
        pL = pL1 + q * (pL2 - pL1)
      else
        local pL1 = rData[lower][lIdx][2]
        local pL2 = rData[upper][lIdx][2]
        pL = pL1 + q * (pL2 - pL1)
      end
      local n1 = rData[lower][lIdx][5]
      local n2 = rData[upper][lIdx][5]
      local nml = n1 + q * (n2 - n1)
      nml:normalize()
      local l1 = rData[lower][lIdx][6]
      local l2 = rData[upper][lIdx][6]
      local lat = l1 + q * (l2 - l1)
      lat:normalize()
      local tgt = nml:cross(lat)
      local pos = pL + lat * layer.pos[0]

      -- Compute the local frame for this decal patch.
      if layer.rot[0] == 1 then
        local x, y, s, c = tgt.x, tgt.y, sin(1.5707963267948966), cos(1.5707963267948966)
        tgt:set(x * c - y * s, x * s + y * c, 0)
      elseif layer.rot[0] == 2 then
        local x, y, s, c = tgt.x, tgt.y, sin(3.141592653589793), cos(3.141592653589793)
        tgt:set(x * c - y * s, x * s + y * c, 0)
      elseif layer.rot[0] == 3 then
        local x, y, s, c = tgt.x, tgt.y, sin(-1.5707963267948966), cos(-1.5707963267948966)
        tgt:set(x * c - y * s, x * s + y * c, 0)
      end

      -- Attempt to see if this material already has a template, and use that.  Otherwise, create a new template.
      local template = nil
      if templateMap[layer.mat] then
        template = templates[templateMap[layer.mat]]
      else
        local templateIdx = #templates + 1
        templates[templateIdx] = createObject("DecalData")
        templates[templateIdx]:setField("renderPriority", 0, i)
        templates[templateIdx]:setField("textureLength", 0, layer.size[0])
        templates[templateIdx]:setField("texRows", 0, layer.numRows[0])
        templates[templateIdx]:setField("texCols", 0, layer.numCols[0])
        templates[templateIdx]:setField("size", 0, layer.size[0])
        templates[templateIdx]:setField("material", 0, layer.mat)
        templates[templateIdx]:registerObject("")
        templateMap[layer.mat] = templateIdx
        folder:addObject(templates[templateIdx])
        template = templates[templateIdx]
      end

		  decals[name].instances[ctrD] = editor.addDecalInstanceWithTan(pos, nml, tgt, template, layer.size[0], layer.frame[0], 0, 1)
      ctrD = ctrD + 1
    end
  end
end

-- Attempts to removes the decal set with the given name, from the scene (if it exists).
local function tryRemove(name)
  local decal = asphalts[name]
  if decal then
    local numDecals = #decal
    for s = 1, numDecals do
      local d = decal[s]
      if d.road then                                                                                -- Main decalRoads.
        d.road:delete()
      end
    end
    table.clear(decal)
  end

  local decal = decals[name]
  if decal then
    if decal.layers then                                                                            -- Remove all DecalRoad layers.
      for s = 1, #decal.layers do
        decal.layers[s]:delete()
      end
    end
    if decal.instances then                                                                         -- Remove all decal instances (must be before template removal).
      for s = 1, #decal.instances do
        editor.deleteDecalInstance(decal.instances[s])
      end
    end
    table.clear(decal)
  end
end

-- Remove all the templates (all instances should be removed first).
local function removeTemplates()
  for i = 1, #templates do
    if templates[i] then
      templates[i]:delete()
    end
  end
  table.clear(templates)
  table.clear(templateMap)
end

-- Attemps to remove all decal sets from the scene.
local function tryRemoveAll()
  for k, _ in pairs(asphalts) do
    tryRemove(k)
  end
  table.clear(asphalts)
  for k, _ in pairs(decals) do
    tryRemove(k)
  end
  table.clear(decals)
  removeTemplates()
end


-- Public interface.
M.createDecal =                                           createDecal

M.tryRemove =                                             tryRemove
M.removeTemplates =                                       removeTemplates
M.tryRemoveAll =                                          tryRemoveAll

return M