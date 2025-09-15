-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- The raw road network data structures.
local graph = {}
local coords2d = {}
local coords3d = {}
local normals = {}
local widths = {}

-- The post-processed road network data structures.
local pathSegments = {}
local iterMap = {}
local cachedTangents = {}
local roads = {}
local junctionMap = {}
local junctions = {}

local max, min, abs, sqrt = math.max, math.min, math.abs, math.sqrt

local function compute2dCoords()
  coords2d = {}
  for k, p in pairs(coords3d) do
    coords2d[k] = vec3(p.x, p.y, 0.0)
  end
end

local function getChildren(table)
  local children = {}
  local ctr = 0
  for k, v in pairs(table) do
    if type(v) == 'table' then
      children[k] = v
      ctr = ctr + 1
    end
  end
  return { children = children, count = ctr }
end

local function doesCollectionContainSegment(collection, testSeg)
  for k, trialSeg in pairs(collection) do
    local matches = 0
    for k1, v1 in pairs(trialSeg) do
      for k2, v2 in pairs(testSeg) do
        if v1 == v2 then
          matches = matches + 1
          break
        end
      end
    end
    if matches == #testSeg then
      return true
    end
  end
  return false
end

local function doesSegmentContainKey(currentPath, nextSuccessorKey)
  for k, v in pairs(currentPath) do
    if v == nextSuccessorKey then
      return true
    end
  end
  return false
end

local function getKeysSortedByValue(tbl, sortFunction)
  local keys = {}
  for key in pairs(tbl) do
    table.insert(keys, key)
  end
  table.sort(keys, function(a, b)
    return sortFunction(tbl[a], tbl[b])
  end)
  return keys
end

local function sortDescending(a, b)
  return a > b
end


local function computePathSegments()
  -- Trace all the path segments from the road network graph.
  pathSegments = {}
  local ctr = 1
  for headKey, v1 in pairs(graph) do
    local firstChildren = getChildren(graph[headKey])
    local successors = firstChildren['children']
    -- Remove condition that filtered out nodes with 2 children
    for childKey, v2 in pairs(successors) do
      local currentPath = {}
      currentPath[1] = headKey
      local ctr2 = 2
      local nextKey = childKey
      while true do
        currentPath[ctr2] = nextKey
        ctr2 = ctr2 + 1
        local nextChildren = getChildren(graph[nextKey])
        local nextSuccessors = nextChildren['children']
        -- Check if we've reached a junction or if all possible next nodes have been visited
        local allSuccessorsVisited = true
        for nextSuccessorKey, _ in pairs(nextSuccessors) do
          if not doesSegmentContainKey(currentPath, nextSuccessorKey) then
            allSuccessorsVisited = false
            break
          end
        end

        if nextChildren['count'] ~= 2 or allSuccessorsVisited then
          if doesCollectionContainSegment(pathSegments, currentPath) == false then
            pathSegments[ctr] = currentPath
            ctr = ctr + 1
          end
          break
        end

        local didFind = false
        for nextSuccessorKey, v3 in pairs(nextSuccessors) do
          if doesSegmentContainKey(currentPath, nextSuccessorKey) == false then
            nextKey = nextSuccessorKey
            didFind = true
            break
          end
        end
        if didFind == false then
          if doesCollectionContainSegment(pathSegments, currentPath) == false then
            pathSegments[ctr] = currentPath
            ctr = ctr + 1
          end
          break
        end
      end
    end
  end

  -- ... rest of the function remains unchanged ...
  -- Compute the average widths for each path segment.
  local avgWidths = {}
  for k, seg in pairs(pathSegments) do
    local subTotal = 0.0
    local ctr = 0
    for i, key in pairs(seg) do
      subTotal = subTotal + widths[key]
      ctr = ctr + 1
    end
    avgWidths[k] = subTotal / ctr
  end

  -- Compute the map from the standard path segments iteration order, to the sorted iteration order.
  iterMap = getKeysSortedByValue(avgWidths, sortDescending)
end

local function computeTangents(pn0, pn1, pn2, pn3)
  local d1, d2, d3 = max(sqrt(pn0:distance(pn1)), 1e-12), sqrt(pn1:distance(pn2)), max(sqrt(pn2:distance(pn3)), 1e-12)
  local m = (pn1 - pn0) / d1 + (pn0 - pn2) / (d1 + d2)
  local n = (pn1 - pn3) / (d2 + d3) + (pn3 - pn2) / d3
  local pn12 = pn2 - pn1
  local t1 = (d2 * m) + pn12
  if t1:length() < 1e-5 then
    t1 = pn12 * 0.5
  end
  local t2 = (d2 * n) + pn12
  if t2:length() < 1e-5 then
    t2 = pn12 * 0.5
  end
  return { t1 = t1, t2 = t2 }
end

local function findTrunkRoadTangent(key, stdTangent)
  local mag = stdTangent:length()
  if cachedTangents[key] ~= nil then
    local stdTanNorm = stdTangent:normalized()
    local maxSoFar = -1.0
    local bestTan = nil
    local bestDot = 0.0
    local isFound = false
    for i, testTan in ipairs(cachedTangents[key]) do
      local testTanNorm = testTan:normalized()
      local shortAngle = stdTanNorm:dot(testTanNorm)
      if abs(shortAngle) > max(0.707, maxSoFar) then
        maxSoFar = abs(shortAngle)
        bestTan = testTanNorm
        bestDot = shortAngle
        isFound = true
      end
    end
    if isFound == true then
      if bestDot < 0.0 then
        return -mag * bestTan
      end
      return mag * bestTan
    else
      cachedTangents[key][#cachedTangents[key] + 1] = stdTangent
    end
  else
    cachedTangents[key] = {}
    cachedTangents[key][1] = stdTangent
  end
  return stdTangent
end

local function fitCubic(p0, p1, p2, p3, length)
  local tangents = computeTangents(p0 - p1, vec3(0.0, 0.0, 0.0), p2 - p1, p3 - p1)
  local t1, t2 = tangents['t1'], tangents['t2']
  local dz = p2.z - p1.z
  local lengthSq = length * length
  return {
    a = p1.z,
    b = t1.z / length,
    c = ((-2.0 * t1.z) - t2.z + (3.0 * dz)) / lengthSq,
    d = (t1.z + t2.z - (2.0 * dz)) / (lengthSq * length) }
end

local function computeRoads()
  roads = {}
  cachedTangents = {}
  local ctr = 1
  for i, v in pairs(pathSegments) do
    -- Use the sorted iteration map to choose the segment processing order here.
    local seg = pathSegments[iterMap[i]]
    for i1 = 1, #seg - 1 do

      -- Compute the reference line cubic equations (parametric).
      local i0, i2, i3 = max(i1 - 1, 1), i1 + 1, min(i1 + 2, #seg)
      local seg0, seg1, seg2, seg3 = seg[i0], seg[i1], seg[i2], seg[i3]
      local p1_2d = coords2d[seg1]
      local pn0_2d, pn1_2d, pn2_2d, pn3_2d = coords2d[seg0] - p1_2d, vec3(0.0, 0.0, 0.0), coords2d[seg2] - p1_2d, coords2d[seg3] - p1_2d
      local geodesicLength2d = pn1_2d:distance(pn2_2d)
      local tangents = computeTangents(pn0_2d, pn1_2d, pn2_2d, pn3_2d)
      local t1, t2 = tangents['t1'], tangents['t2']
      if i1 == 1 then
        t1 = findTrunkRoadTangent(seg1, t1)
      end
      if i1 == #seg - 1 then
        t2 = findTrunkRoadTangent(seg2, t2)
      end
      local coeffC, coeffD = (-2.0 * t1) - t2 + (3.0 * pn2_2d), t1 + t2 - (2.0 * pn2_2d)
      local refLineCubic = {
        uA = pn1_2d.x,
        uB = t1.x,
        uC = coeffC.x,
        uD = coeffD.x,
        vA = pn1_2d.y,
        vB = t1.y,
        vC = coeffC.y,
        vD = coeffD.y }

      -- Compute the elevation cubic equation (explicit).
      local elevCubic = fitCubic(coords3d[seg0], coords3d[seg1], coords3d[seg2], coords3d[seg3], geodesicLength2d)

      -- Compute the width cubic equation (explicit).
      local widthCubic = fitCubic(
        vec3(coords3d[seg0].x, coords3d[seg0].y, widths[seg0]),
        vec3(coords3d[seg1].x, coords3d[seg1].y, widths[seg1]),
        vec3(coords3d[seg2].x, coords3d[seg2].y, widths[seg2]),
        vec3(coords3d[seg3].x, coords3d[seg3].y, widths[seg3]),
        geodesicLength2d)

      -- Create the road section.
      roads[ctr] = {
        start = seg1,
        finish = seg2,
        pos = coords3d[seg1],
        length = geodesicLength2d,
        refLineCubic = refLineCubic,
        elevCubic = elevCubic,
        widthCubic = widthCubic }
      ctr = ctr + 1
    end
  end
end

local function graphToJunctionMap()
  junctionMap = {}
  local ctr = 0
  for i, seg in pairs(pathSegments) do
    -- Test if the path segment's first node is a true junction (no dead ends). Only add if we haven't already found it previously.
    local key1 = seg[1]
    if getChildren(graph[key1])['count'] > 2 and junctionMap[key1] == nil then
      junctionMap[key1] = ctr
      ctr = ctr + 1
    end

    -- Test if the path segment's last node is a true junction (no dead ends). Only add if we haven't already found it previously.
    local key2 = seg[#seg]
    if getChildren(graph[key2])['count'] > 2 and junctionMap[key2] == nil then
      junctionMap[key2] = ctr
      ctr = ctr + 1
    end
  end
end

local function updateConnectivityData()
  for k, r in pairs(roads) do

    -- Compute the predecessor road to this road, if it exists.
    local predecessor = 'none'
    for j, r2 in pairs(roads) do
      if k ~= j and r['start'] == r2['finish'] then
        predecessor = j
        break
      end
    end
    roads[k]['predecessor'] = predecessor

    -- Compute the successor road to this road, if it exists.
    local successor = 'none'
    for j, r2 in pairs(roads) do
      if k ~= j and r['finish'] == r2['start'] then
        successor = j
        break
      end
    end
    roads[k]['successor'] = successor

    -- Compute the junction and contact point data for this road, if they exist.
    local junction = -1
    local contactPoint = 'none'
    if junctionMap[r['start']] ~= nil then
      junction = junctionMap[r['start']]
      contactPoint = 'start'
    elseif junctionMap[r['finish']] ~= nil then
      junction = junctionMap[r['finish']]
      contactPoint = 'end'
    end
    roads[k]['junction'] = junction
    roads[k]['contactPoint'] = contactPoint
  end
end

local function computeJunctions()
  junctions = {}
  local jCtr = 1
  for key, id in pairs(junctionMap) do
    local connectionRoads = {}
    local ctr = 1
    for rid, r in pairs(roads) do
      if key == r['start'] then
        connectionRoads[ctr] = { id = rid, contactPoint = 'start'}
        ctr = ctr + 1
      end
      if key == r['finish'] then
        connectionRoads[ctr] = { id = rid, contactPoint = 'end'}
        ctr = ctr + 1
      end
    end
    if #connectionRoads > 0 then
      junctions[jCtr] = { id = id, connectionRoads = connectionRoads }
      jCtr = jCtr + 1
    end
  end
end

local function writeXodr(filename)
  local fullFileName = filename .. '.xodr'
  local f = io.open(fullFileName, "w")

  -- Preamble.
  f:write('<?xml version="1.0" standalone="yes"?>\n')
  f:write('<OpenDRIVE>\n')
  f:write('<header revMajor="1" revMinor="7" name="" version="1.00" date="' .. os.date("%Y%m%d%H%M%S") .. '" north="0.0" south="0.0" east="0.0" west="0.0">\n')
  f:write('</header>\n')

  -- Write the road data.
  for rid, r in pairs(roads) do

    -- Road header data.
    f:write('<road rule="RHT" length="' .. tostring(r['length']) .. '" id="' .. tostring(rid) .. '" junction="' .. tostring(r['junction']) .. '" >\n')

    -- Road connectivity data.
    f:write('<link>\n')
    if r['predecessor'] ~= 'none' then
      f:write('<predecessor elementType="' .. 'road' .. '" elementId="' .. tostring(r['predecessor']) .. '" contactPoint="' .. tostring(r['contactPoint']) .. '" />\n')
    end
    if r['successor'] ~= 'none' then
      f:write('<successor elementType="' .. 'road' .. '" elementId="' .. tostring(r['successor']) .. '" contactPoint="' .. tostring(r['contactPoint']) .. '" />\n')
    end
    f:write('</link>\n')

    -- Geometry data.
    local Au, Bu, Cu, Du = tostring(r['refLineCubic']['uA']), tostring(r['refLineCubic']['uB']), tostring(r['refLineCubic']['uC']), tostring(r['refLineCubic']['uD'])
    local Av, Bv, Cv, Dv = tostring(r['refLineCubic']['vA']), tostring(r['refLineCubic']['vB']), tostring(r['refLineCubic']['vC']), tostring(r['refLineCubic']['vD'])
    f:write('<type s="0.0000000000000000e+00" type="town" country="DE"/>\n')
    f:write('<planView>\n')
    f:write('<geometry s="0.0000000000000000e+00" x="' .. r['pos'].x .. '" y="' .. r['pos'].y .. '" hdg="' .. '0.0000000000000000e+00' .. '" length="' .. tostring(r['length']) .. '">\n')
    f:write('<paramPoly3 aU="' .. Au .. '" bU="' .. Bu .. '" cU="' .. Cu .. '" dU="' .. Du .. '" aV="' .. Av .. '" bV="' .. Bv .. '" cV="' .. Cv .. '" dV="' .. Dv .. '"/>\n')
    f:write('</geometry>\n')
    f:write('</planView>\n')

    -- Elevation data.
    local Ae, Be, Ce, De = tostring(r['elevCubic']['a']), tostring(r['elevCubic']['b']), tostring(r['elevCubic']['c']), tostring(r['elevCubic']['d'])
    f:write('<elevationProfile>\n')
    f:write('<elevation s="0.0000000000000000e+00" a="' .. Ae .. '" b="' .. Be .. '" c="' .. Ce .. '" d="' .. De .. '"/>\n')
    f:write('</elevationProfile>\n')
    f:write('<lateralProfile>\n')
    f:write('</lateralProfile>\n')

    -- Road lane data.
    local Aw, Bw, Cw, Dw = tostring(r['widthCubic']['a']), tostring(r['widthCubic']['b']), tostring(r['widthCubic']['c']), tostring(r['widthCubic']['d'])
    f:write('<lanes>\n')
    f:write('<laneSection s="0.0000000000000000e+00">\n')
    f:write('<left>\n')
    f:write('<lane id="1" type="driving" level="false">\n')
    f:write('<link>\n')
    f:write('</link>\n')
    f:write('<width sOffset="0.0000000000000000e+00" a="' .. Aw .. '" b="' .. Bw .. '" c="' .. Cw .. '" d="' .. Dw .. '"/>\n')
    f:write('</lane>\n')
    f:write('</left>\n')
    f:write('<center>\n')
    f:write('<lane id="0" type="driving" level="false">\n')
    f:write('<link>\n')
    f:write('</link>\n')
    f:write('<roadMark sOffset="0.0000000000000000e+00" type="broken" weight="standard" color="standard" width="0.12" laneChange="both" height="0.02">\n')
    f:write('<type name="broken" width="0.12">\n')
    f:write('<line length="3.0" space="6.0" tOffset="0.0" sOffset="0.0" rule="caution" width="0.12"/>\n')
    f:write('</type>\n')
    f:write('</roadMark>\n')
    f:write('</lane>\n')
    f:write('</center>\n')
    f:write('<right>\n')
    f:write('<lane id="-1" type="driving" level="false">\n')
    f:write('<link>\n')
    f:write('</link>\n')
    f:write('<width sOffset="0.0" a="' .. Aw .. '" b="' .. Bw .. '" c="' .. Cw .. '" d="' .. Dw .. '"/>\n')
    f:write('</lane>\n')
    f:write('</right>\n')
    f:write('</laneSection>\n')
    f:write('</lanes>\n')

    -- Unused tags.
    f:write('<objects>\n')
    f:write('</objects>\n')
    f:write('<signals>\n')
    f:write('</signals>\n')
    f:write('<surface>\n')
    f:write('</surface>\n')

    f:write('</road>\n')
  end

  -- Write the junction data, in order.
  for jid, j in pairs(junctions) do
    f:write('<junction name="" id="' .. tostring(jid) .. '" type="default">\n')
    local ctr = 0
    for i1, ra in pairs(j['connectionRoads']) do
      for i2, rb in pairs(j['connectionRoads']) do
        if i1 ~= i2 then
          f:write('<connection id="' .. tostring(ctr) .. '" incomingRoad="' .. tostring(ra['id']) .. '" connectingRoad="' .. tostring(rb['id']) .. '" contactPoint="' .. tostring(ra['contactPoint']) .. '">\n')
          if ra['contactPoint'] == 'start' and rb['contactPoint'] == 'start' then
            f:write('<laneLink from="1" to="-1"/>\n')
          elseif ra['contactPoint'] == 'start' and rb['contactPoint'] == 'finish' then
            f:write('<laneLink from="1" to="1"/>\n')
          elseif ra['contactPoint'] == 'finish' and rb['contactPoint'] == 'start' then
            f:write('<laneLink from="-1" to="-1"/>\n')
          elseif ra['contactPoint'] == 'finish' and rb['contactPoint'] == 'finish' then
            f:write('<laneLink from="-1" to="1"/>\n')
          end
          f:write('</connection>\n')
          ctr = ctr + 1
        end
      end
    end
    f:write('</junction>\n')
  end

  f:write('</OpenDRIVE>\n')
  f:close()
end

-- Exports the road network from the currently-loaded map to OpenDRIVE (.xodr) format.
local function export(filename)

  -- Get the raw road network data from the currently-loaded map.
  local graphPath = map.getGraphpath()
  graph = graphPath['graph']
  coords3d = graphPath.positions
  widths = graphPath.radius
  normals = map.getMap().nodes

  -- Process the raw road network data into a collection of roads and junctions, which are amenable for export.
  compute2dCoords()
  computePathSegments()
  computeRoads()
  graphToJunctionMap()
  updateConnectivityData()
  computeJunctions()

  -- Export the processed road network data to OpenDRIVE (.xodr) format.
  writeXodr(filename)
end

-- Public interface.
M.export = export

return M