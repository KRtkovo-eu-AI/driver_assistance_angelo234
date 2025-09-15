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
local keysToNodeMap = {}

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

  return pathSegments
end

local function createNodesData()
  -- Create the nodes data: A unique list of nodes, a map from graph keys to unique node id, and bounds info.
  local nodes = {}
  keysToNodeMap = {}
  local minlat = 1e99
  local minlon = 1e99
  local maxlat = -1e99
  local maxlon = -1e99
  local ctr = 0
  local scaleFactor = 1.0 / 1e7  -- to convert metres into reasonable latitude/longitude values.

  for k, v in pairs(coords3d) do
    keysToNodeMap[k] = ctr
    local coord = vec3(v.x * scaleFactor + 45.0, v.y * scaleFactor + 45.0, v.z)
    nodes[ctr] = coord
    minlat = min(minlat, coord.x)
    minlon = min(minlon, coord.y)
    maxlat = max(maxlat, coord.x)
    maxlon = max(maxlon, coord.y)
    ctr = ctr + 1
  end

  return {
    nodes = nodes,
    minlat = minlat,
    minlon = minlon,
    maxlat = maxlat,
    maxlon = maxlon
  }
end

local function createWaysData(pathSegments)
  -- Create the unique list of OpenStreetMap 'ways'.
  local ways = {}
  for _, seg in pairs(pathSegments) do
    local n = {}
    for i = 1, #seg do
      table.insert(n, keysToNodeMap[seg[i]])
    end
    table.insert(ways, n)
  end

  return ways
end

local function formatDateTime()
  -- Format current date time in a specific format
  local time = os.date("*t")
  return string.format("%02d/%02d/%04d %02d:%02d:%02d",
    time.day, time.month, time.year,
    time.hour, time.min, time.sec)
end

local function writeOsmFile(fileName, nodesData, ways)
  -- Write the road network data to .osm format (xml).
  local fullFileName = fileName .. ".osm"
  local dateTime = formatDateTime()
  local f = io.open(fullFileName, "w")

  f:write('<?xml version="1.0" encoding="UTF-8"?>\n')
  f:write('<osm version="0.6" generator="BeamNG">\n')
  f:write(string.format('\t<bounds minlat="%s" minlon="%s" maxlat="%s" maxlon="%s"/>\n',
    tostring(nodesData.minlat), tostring(nodesData.minlon),
    tostring(nodesData.maxlat), tostring(nodesData.maxlon)))

  -- Write nodes
  for i = 0, #nodesData.nodes - 1 do
    local nodeId = i + 1
    local lat = tostring(nodesData.nodes[i].x)
    local lon = tostring(nodesData.nodes[i].y)
    local ele = tostring(nodesData.nodes[i].z)

    f:write(string.format('\t<node id="%s" lat="%s" lon="%s" ele="%s" user="BeamNG" uid="1" visible="true" version="1" changeset="1" timestamp="%s"/>\n',
      nodeId, lat, lon, ele, dateTime))
  end

  -- Write ways
  for i = 1, #ways do
    local wayId = i  -- the OpenStreetMap Id numbers start at 1 not 0.
    f:write(string.format('\t<way id="%s" user="BeamNG" uid="1" visible="true" version="1" changeset="1">\n', wayId))

    local seg = ways[i]
    for j = 1, #seg do
      local nodeId = seg[j] + 1
      f:write(string.format('\t\t<nd ref="%s"/>\n', nodeId))
    end

    f:write('\t</way>\n')
  end

  f:write('</osm>\n')
  f:close()

  log('I', 'OpenStreetMapExporter', 'Successfully exported to ' .. fullFileName)
end

-- Exports the road network from the currently-loaded map to OpenStreetMap (.osm) format.
local function export(fileName)
  -- Get the raw road network data from the currently-loaded map.
  local graphPath = map.getGraphpath()
  graph = graphPath['graph']
  coords3d = graphPath.positions
  widths = graphPath.radius
  normals = map.getMap().nodes

  -- Process the raw road network data
  compute2dCoords()
  local segments = computePathSegments()
  local nodesData = createNodesData()
  local ways = createWaysData(segments)

  -- Export the processed road network data to OpenStreetMap (.osm) format.
  writeOsmFile(fileName, nodesData, ways)
end

-- Public interface.
M.export = export

return M