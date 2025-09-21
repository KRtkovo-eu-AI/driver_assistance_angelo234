-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local graph = {}
local coords3d = {}
local pathSegments = {}
local keysToNodeMap = {}

local NUM_LANES = "2"
local PRIORITY = "2"
local SPEED = "14.0"

local function getChildren(node)
  local children = {}
  for k, v in pairs(node) do
    if type(v) == 'table' then
      children[k] = v
    end
  end
  return children
end

local function doesCollectionContainSegment(collection, testSeg)
  for _, trialSeg in pairs(collection) do
    local matches = 0
    for _, v1 in pairs(trialSeg) do
      for _, v2 in pairs(testSeg) do
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

local function doesSegmentContainKey(path, key)
  for _, v in ipairs(path) do
    if v == key then return true end
  end
  return false
end

local function computePathSegments()
  local segments = {}
  local ctr = 1

  for headKey, _ in pairs(graph) do
    local successors = getChildren(graph[headKey])
    for childKey, _ in pairs(successors) do
      local currentPath = { headKey }
      local nextKey = childKey
      while true do
        table.insert(currentPath, nextKey)
        local nextSuccessors = getChildren(graph[nextKey])
        local allVisited = true
        for k, _ in pairs(nextSuccessors) do
          if not doesSegmentContainKey(currentPath, k) then
            allVisited = false
            break
          end
        end
        if tableSize(nextSuccessors) ~= 2 or allVisited then
          if not doesCollectionContainSegment(segments, currentPath) then
            segments[ctr] = currentPath
            ctr = ctr + 1
          end
          break
        end
        local found = false
        for k, _ in pairs(nextSuccessors) do
          if not doesSegmentContainKey(currentPath, k) then
            nextKey = k
            found = true
            break
          end
        end
        if not found then
          if not doesCollectionContainSegment(segments, currentPath) then
            segments[ctr] = currentPath
            ctr = ctr + 1
          end
          break
        end
      end
    end
  end

  return segments
end

local function createNodes()
  local nodes = {}
  local id = 0
  for k, v in pairs(coords3d) do
    keysToNodeMap[k] = k
    nodes[#nodes + 1] = { id = k, x = v.x, y = v.y, z = v.z }
    id = id + 1
  end
  return nodes
end

local function createEdges(pathSegments)
  local edges = {}
  for _, path in ipairs(pathSegments) do
    for i = 1, #path - 1 do
      local fromID = path[i]
      local toID = path[i + 1]
      table.insert(edges, { from = fromID, to = toID })
    end
  end
  return edges
end

local function writeNodeFile(fileName, nodes)
  local file = io.open(fileName, "w")
  file:write("<nodes>\n")
  for _, node in ipairs(nodes) do
    file:write(string.format('\t<node id="%s" x="%f" y="%f" z="%f"/>\n',
      tostring(node.id), node.x, node.y, node.z))
  end
  file:write("</nodes>\n")
  file:close()
end

local function writeEdgeFile(fileName, edges)
  local file = io.open(fileName, "w")
  file:write("<edges>\n")
  local ctr = 1
  for _, edge in ipairs(edges) do
    local id1 = "e" .. tostring(ctr)
    file:write(string.format('\t<edge id="%s" from="%s" to="%s" priority="%s" numLanes="%s" speed="%s"/>\n',
      id1, edge.from, edge.to, PRIORITY, NUM_LANES, SPEED))
    ctr = ctr + 1

    -- Add reverse edge (bi-directional)
    local id2 = "e" .. tostring(ctr)
    file:write(string.format('\t<edge id="%s" from="%s" to="%s" priority="%s" numLanes="%s" speed="%s"/>\n',
      id2, edge.to, edge.from, PRIORITY, NUM_LANES, SPEED))
    ctr = ctr + 1
  end
  file:write("</edges>\n")
  file:close()
end

local function export(name)
  -- Load road network from BeamNG map
  local graphPath = map.getGraphpath()
  graph = graphPath.graph
  coords3d = graphPath.positions

  -- Process paths and geometry
  local pathSegments = computePathSegments()
  local nodes = createNodes()
  local edges = createEdges(pathSegments)

  -- Write to SUMO files
  writeNodeFile(name .. ".nod.xml", nodes)
  writeEdgeFile(name .. ".edg.xml", edges)
end

M.export = export
return M
