--[[
Copyright (c) 2012 Hello!Game, 2015 BeamNG GmbH

Permission is hereby granted, free of charge, to any person obtaining a copy
of newinst software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

The above copyright notice and newinst permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
]]

----------------------------------------------------------------
-- example :
--[[
gp = newGraphpath()
gp:edge("a", "b", 7)
gp:edge("a", "c", 9)
gp:edge("a", "f", 14)
gp:edge("b", "d", 15)
gp:edge("b", "c", 10)
gp:edge("c", "d", 11)
gp:edge("c", "f", 2)
gp:edge("d", "e", 6)
gp:edge("e", "f", 9)

print( table.concat( gp:getPath("a","e"), "->") )
]]

local bit = require("bit")

local tableInsert, min, max, random, rshift = table.insert, math.min, math.max, math.random, bit.rshift

local M = {}

local minheap = {}
minheap.__index = minheap

local function newMinheap()
  return setmetatable({length = 0, vals = {}}, minheap)
end

function minheap:peekKey()
  return self[min(self.length, 1)]
end

function minheap:peekVal()
  return self.vals[min(self.length, 1)]
end

function minheap:empty()
  return self.length == 0
end

function minheap:clear()
  table.clear(self.vals)
  self.length = 0
end

function minheap:insert(k, v)
  local vals = self.vals
  -- float the new key up from the bottom of the heap
  local child_index = self.length + 1 -- array index of the new child node to be added to heap
  self.length = child_index -- update the central heap length record

  while child_index > 1 do
    local parent_index = rshift(child_index, 1)
    local parent_key = self[parent_index]
    if k >= parent_key then
      break
    else
      self[child_index], vals[child_index] = parent_key, vals[parent_index]
      child_index = parent_index
    end
  end

  self[child_index], vals[child_index] = k, v
end

function minheap:pop()
  if self.length <= 0 then return end
  local vals = self.vals
  local result_key, result_val = self[1], vals[1]  -- get top value
  local heapLength = self.length
  local last_key, last_val = self[heapLength], vals[heapLength]
  heapLength = heapLength - 1
  local child_index = 2
  local parent_index = 1

  while child_index <= heapLength do
    local next_child = child_index + 1
    if next_child <= heapLength and self[next_child] < self[child_index] then
      child_index = next_child
    end
    local child_key = self[child_index]
    if last_key < child_key then
      break
    else
      self[parent_index], vals[parent_index] = child_key, vals[child_index]
      parent_index = child_index
      child_index = child_index + child_index
    end
  end

  self.length = heapLength
  self[parent_index], vals[parent_index] = last_key, last_val
  return result_key, result_val
end

-----------------------------------------------------------------

local Graphpath = {}
Graphpath.__index = Graphpath

local function newGraphpath()
  return setmetatable({graph = {}, positions = {}, radius = {}}, Graphpath)
end

function Graphpath:export(edgeCount)
  local i, edgeData = 0, table.new((edgeCount or 0) * 3, 0)
  for node1, links in pairs(self.graph) do
    for node2, linkData in pairs(links) do
      if node1 > node2 then
        i = i + 3
        edgeData[i-2], edgeData[i-1], edgeData[i] = node1, node2, linkData
      end
    end
  end
  return {edges = edgeData, positions = self.positions, radius = self.radius}
end

function Graphpath:import(graphData)
  local graph = self.graph
  local edges = graphData.edges

  for i = 1, #edges, 3 do
    if graph[edges[i]] == nil then graph[edges[i]] = {} end
    graph[edges[i]][edges[i+1]] = edges[i+2]

    if graph[edges[i+1]] == nil then graph[edges[i+1]] = {} end
    graph[edges[i+1]][edges[i]] = edges[i+2]
  end

  self.positions = graphData.positions
  self.radius = graphData.radius
end

function Graphpath:clear()
  self.graph = {}
end

function Graphpath:edge(sp, ep, dist)
  if self.graph[sp] == nil then
    self.graph[sp] = {}
  end

  self.graph[sp][ep] = {len = dist or 1}

  if self.graph[ep] == nil then
    self.graph[ep] = {}
  end
end

function Graphpath:uniEdge(inNode, outNode, dist, drivability, speedLimit, lanes, oneWay, gated, inPos, inRad, outPos, outRad)
  dist = dist or 1
  if self.graph[inNode] == nil then
    self.graph[inNode] = {}
  end

  local data = {
    len = dist,
    drivability = drivability,
    inNode = inNode,
    outNode = outNode,
    speedLimit = speedLimit,
    lanes = lanes,
    oneWay = oneWay,
    gated = gated,
    inPos = inPos,
    inRadius = inRad,
    outPos = outPos,
    outRadius = outRad} -- sp is the inNode of the edge

  self.graph[inNode][outNode] = data

  if self.graph[outNode] == nil then
    self.graph[outNode] = {}
  end

  self.graph[outNode][inNode] = data
end

function Graphpath:bidiEdge(inNode, outNode, dist, drivability, speedLimit, lanes, oneWay, gated, inPos, inRad, outPos, outRad)
  dist = dist or 1
  if self.graph[inNode] == nil then
    self.graph[inNode] = {}
  end

  local data = {
    len = dist,
    drivability = drivability,
    inNode = inNode,
    outNode = outNode,
    speedLimit = speedLimit,
    lanes = lanes,
    oneWay = oneWay,
    gated = gated,
    inPos = inPos,
    inRadius = inRad,
    outPos = outPos,
    outRadius = outRad} -- sp is the inNode of the edge

  self.graph[inNode][outNode] = data

  if self.graph[outNode] == nil then
    self.graph[outNode] = {}
  end

  self.graph[outNode][inNode] = data
end

function Graphpath:getEdgePositions(n1id, n2id)
  local edgeData = self.graph[n1id][n2id]
  return edgeData[n1id == edgeData.inNode and 'inPos' or 'outPos'] or self.positions[n1id],
         edgeData[n2id == edgeData.inNode and 'inPos' or 'outPos'] or self.positions[n2id]
end

function Graphpath:getEdgeRadii(n1id, n2id)
  local edgeData = self.graph[n1id][n2id]
  return edgeData[n1id == edgeData.inNode and 'inRadius' or 'outRadius'] or self.radius[n1id],
         edgeData[n2id == edgeData.inNode and 'inRadius' or 'outRadius'] or self.radius[n2id]
end

function Graphpath:setPointPosition(p, pos)
  self.positions[p] = pos
end

function Graphpath:setPointPositionRadius(p, pos, radius)
  self.positions[p] = pos
  self.radius[p] = radius
end

function Graphpath:setNodeRadius(node, radius)
  self.radius[node] = radius
end

local function invertPath(goal, road)
  local path = table.new(20, 0)
  local e = 0
  while goal do -- unroll path from goal to source
    e = e + 1
    path[e] = goal
    goal = road[goal]
  end

  for s = 1, e * 0.5 do -- reverse order to get source to goal
    path[s], path[e] = path[e], path[s]
    e = e - 1
  end

  return path
end

do
  local graph, index, S, nodeData, allSCC

  local function strongConnect(node)
    -- Set the depth index for node to the smallest unused index
    index = index + 1
    nodeData[node] = {index = index, lowlink = index, onStack = true}
    tableInsert(S, node)

    -- Consider succesors of node
    for adjNode, value in pairs(graph[node]) do
      if value.drivability == 1 then
        if nodeData[adjNode] == nil then -- adjNode is a descendant of 'node' in the search tree
          strongConnect(adjNode)
          nodeData[node].lowlink = min(nodeData[node].lowlink, nodeData[adjNode].lowlink)
        elseif nodeData[adjNode].onStack then -- adjNode is not a descendant of 'node' in the search tree
          nodeData[node].lowlink = min(nodeData[node].lowlink, nodeData[adjNode].index)
        end
      end
    end

    -- generate an scc (smallest possible scc is one node), i.e. in a directed accyclic graph each node constitutes an scc
    if nodeData[node].lowlink == nodeData[node].index then
      local currentSCC = {}
      local currentSCCLen = 0
      repeat
        local w = table.remove(S)
        nodeData[w].onStack = false
        currentSCC[w] = true
        currentSCCLen = currentSCCLen + 1
      until node == w
      currentSCC[0] = currentSCCLen
      tableInsert(allSCC, currentSCC)
    end
  end

  function Graphpath:scc(v)
    --[[ https://en.wikipedia.org/wiki/Tarjan%27s_strongly_connected_components_algorithm
    calculates the strongly connected components (scc) of the map graph.
    If v is provided, it only calculates the scc containing / is reachable from v.
    Returns an array of dicts ('allSCC') --]]

    graph = self.graph
    if v and graph[v] == nil then return {} end

    index, S, nodeData, allSCC = 0, {}, {}, {}

    if v then -- get only the scc containing/reachable from v
      strongConnect(v)
    else -- get all scc of the map graph
      for node, _ in pairs(graph) do
        if nodeData[node] == nil then
          strongConnect(node)
        end
      end
    end
    return allSCC
  end
end

function Graphpath:graphMinor()
  -- https://mathworld.wolfram.com/GraphMinor.html
  -- Performs vertex contraction of vertices which have exactly 2 neighbours (degree 2)
  -- The result is a "simplified" graph in which each node has degree 1 ("dead end") or is a junction (degree > 2).
  -- Every node in the output graph is also a node in the input graph.
  -- Every edge in the output graph stores the set (array) of input graph nodes that connect the output graph edge nodes (end nodes included)
  -- ex. edge a <-> b i.e. minorGraph[a][b] = {a, x, y, z, b} -> node a is conected to node b through nodes x, y, z
  -- the graph is bidirectional i.e. minorGraph[b][a] = {a, x, y, z, b} also exists.
  -- for more see edge/vertex contraction and Contraction hierarchies

  local graph = self.graph

  -- identify junction nodes
  local isJunctionNode = {}
  for nid, n in pairs(graph) do
    if tableSize(n) ~= 2 then
      isJunctionNode[nid] = true
    end
  end

  local graphMinor, visited = {}, {}
  for nid in pairs(graph) do
    if isJunctionNode[nid] then
      -- explore the edges of this junction/end node
      -- if there exist an edge to another junction/end node, add it to the minor graph
      for n2id in pairs(graph[nid]) do
        -- check if both n1id and n2id are junction nodes and also avoid including an edge twice
        if nid < n2id and isJunctionNode[n2id] then
          -- Create the edges between the end nodes of the path if they don't already exist and store the path
          if not graphMinor[nid] then
            graphMinor[nid] = {}
          end
          graphMinor[nid][n2id] = {nid, n2id}

          if not graphMinor[n2id] then
            graphMinor[n2id] = {}
          end
          graphMinor[n2id][nid] = graphMinor[nid][n2id]
        end
      end
    elseif not visited[nid] then
      local pathCount = 1
      local path = {nid}
      visited[nid] = true

      -- explore the path nid belongs to
      while true do
        -- get a neighbour of nid that is not already in the path
        local next_nid = next(graph[path[pathCount]], path[pathCount-1]) or next(graph[path[pathCount]])

        pathCount = pathCount + 1
        path[pathCount] = next_nid
        visited[next_nid] = true

        -- check if path has reached an end/junction
        if isJunctionNode[next_nid] then
          -- check if the first node in the path is also an end/junction
          if isJunctionNode[path[1]] then
            -- both ends of the path are junctions/ends i.e. the path has reached its boundaries
            -- terminate this path exploration
            break
          else
            -- reverse the already explored part of the path (nid will become the last node in the path)
            -- and continue exploring the other side of nid
            local n = pathCount
            for i = 1, n * 0.5 do
              path[i], path[n] = path[n], path[i]
              n = n - 1
            end
          end
        elseif next_nid == path[1] then
          -- the path has looped back on itself without encountering a junction node (it is an isolated self loop)
          -- the entire path has been discovered so terminate this path exploration
          break
        end
      end

      -- Create the edges between the end nodes of the path if they don't already exist and store the path
      if not graphMinor[path[1]] then
        graphMinor[path[1]] = {}
      end
      graphMinor[path[1]][path[pathCount]] = path

      if not graphMinor[path[pathCount]] then
        graphMinor[path[pathCount]] = {}
      end
      graphMinor[path[pathCount]][path[1]] = path
    end
  end

  return graphMinor
end

function Graphpath:getPath(start, goal, dirMult)
  local graph = self.graph
  if graph[start] == nil or graph[goal] == nil then return {} end

  local dirCoeff = {[true] = dirMult or 1, [false] = 1}

  local cost, node = 0, start
  local minParent = {[node] = false}
  local queued = {}
  local road = {} -- predecessor subgraph

  local q = newMinheap()
  repeat
    if road[node] == nil then
      road[node] = minParent[node]
      if node == goal then break end
      for child, data in pairs(graph[node]) do
        if road[child] == nil then -- if the shortest path to child has not already been found
          local currentChildCost = queued[child] -- lowest value with which child has entered the que
          local newChildCost = cost + data.len * dirCoeff[(data.oneWay and data.inNode == child) or false] -- For graph that misses data.oneWay in edge data
          if not currentChildCost or newChildCost < currentChildCost then
            q:insert(newChildCost, child)
            minParent[child] = node
            queued[child] = newChildCost
          end
        end
      end
    end
    cost, node = q:pop()
  until not cost

  return invertPath(goal, road)
end

local function splitID(id, delim)
  delim = delim or '\0'
  local i = string.find(id, delim)
  return string.sub(id, 1, i-1), string.sub(id, i+1, #id)
end

local function numOfLanesInDirection(lanes, dir)
  -- lanes: a lane string
  -- dir: '+' or '-'
  dir = (not dir or dir == '+') and 43 or dir == '-' and 45
  local lanesN = 0
  for i = 1, #lanes, 1 do
    if lanes:byte(i) == dir then
      lanesN = lanesN + 1
    end
  end
  return lanesN
end

function Graphpath:edgeLanesInDirection(fromNode, toNode)
  if self.graph[fromNode][toNode].inNode == fromNode then
    return numOfLanesInDirection(self.graph[fromNode][toNode].lanes, '+')
  else
    return numOfLanesInDirection(self.graph[fromNode][toNode].lanes, '-')
  end
end

function Graphpath:getEdgePath(startEdge, goal, dirMult)
  local graph = self.graph
  local bNode, fNode = splitID(startEdge)
  if not graph[bNode][fNode] or not graph[goal] then return end

  local dirCoeff = {[true] = dirMult or 1, [false] = 1}

  local cost, edge = 0, startEdge
  local minParent = {[startEdge] = false}
  local queued = {}
  local road = {} -- predecessor subgraph
  local goalEdge
  local edgeId = {'','\0',''}

  local q = newMinheap()
  repeat
    if road[edge] == nil then
      road[edge] = minParent[edge]
      bNode, fNode = splitID(edge)
      if fNode == goal then
        goalEdge = edge
        break
      end
      edgeId[1] = fNode
      for child, data in pairs(graph[fNode]) do
        if child ~= bNode then
          edgeId[3] = child
          local childEdge = table.concat(edgeId)
          if road[childEdge] == nil then -- if the shortest path to child has not already been found
            local currentChildCost = queued[childEdge] -- lowest value with which child has entered the que
            local newChildCost = cost + data.len * dirCoeff[self:edgeLanesInDirection(fNode, child) == 0]
            if not currentChildCost or newChildCost < currentChildCost then
              q:insert(newChildCost, childEdge)
              minParent[childEdge] = edge
              queued[childEdge] = newChildCost
              dump('push que', childEdge, newChildCost, edge)
            end
          end
        end
      end
      edgeId[1] = bNode
      for child, data in pairs(graph[bNode]) do
        if child ~= fNode then
          edgeId[3] = child
          local childEdge = table.concat(edgeId)
          if road[childEdge] == nil then -- if the shortest path to child has not already been found
            local currentChildCost = queued[childEdge] -- lowest value with which child has entered the que
            local newChildCost = cost + data.len * dirCoeff[self:edgeLanesInDirection(bNode, child) == 0] + 1000
            if not currentChildCost or newChildCost < currentChildCost then
              q:insert(newChildCost, childEdge)
              minParent[childEdge] = edge
              queued[childEdge] = newChildCost
              dump('push que', childEdge, newChildCost, edge)
            end
          end
        end
      end
    end
    cost, edge = q:pop()
    dump('pop que', cost, edge)
  until not cost

  local edgePath = invertPath(goalEdge, road)

  local path, _ = {}
  for i = 1, #edgePath do
    _, path[i] = splitID(edgePath[i])
  end

  return path, edgePath, road
end

function Graphpath:getPointNodePath(start, target, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff, wZ)
  -- Shortest path between a point and a node or vice versa.
  -- start/target: either start or target should be a node name, the other a vec3 point
  -- cutOffDrivability: penalize roads with drivability <= cutOffDrivability
  -- dirMult: penalty to be applied to an edge designated oneWay if it is traversed in opposite direction (should be larger than 1 typically >= 1e3-1e4).
  --          If equal to nil or 1 then it means no penalty.
  -- penaltyAboveCutoff: penalty multiplier for roads above the drivability cutoff
  -- penaltyBelowCutoff: penalty multiplier for roads below the drivability cutoff
  -- wZ: number. When higher than 1 distance minimization is biased to minimizing z diamension more so than x, y.

  local graph = self.graph
  local invert
  if start.x then
    start, target = target, start
    invert = true
  end
  if graph[start] == nil or target == nil then return {} end

  wZ = wZ or 4
  cutOffDrivability = cutOffDrivability or 0
  penaltyAboveCutoff = penaltyAboveCutoff or 1
  penaltyBelowCutoff = penaltyBelowCutoff or 10000

  local dirCoeff = {[true] = dirMult or 1, [false] = 1}
  local drivCoeff = {[true] = penaltyAboveCutoff, [false] = penaltyBelowCutoff}

  local positions = self.positions
  local cost, node = 0, start
  local minParent = {[node] = false}
  local minCost = {[node] = cost}
  local road = {}

  local targetMinCost = math.huge
  local targetMinCostLink
  local tmpVec = vec3()
  local nodeToTargetVec = vec3()

  local q = newMinheap()
  repeat
    if road[node] == nil then
      road[node] = minParent[node] -- t[2] is the predecessor of node in the shortest path to node
      if node == target then break end

      local p1 = positions[node]
      nodeToTargetVec:setSub2(target, p1)
      local pathCost = cost + square(square(nodeToTargetVec.x) + square(nodeToTargetVec.y) + square(wZ * nodeToTargetVec.z))
      if pathCost < targetMinCost then
        q:insert(pathCost, target)
        targetMinCost = pathCost
        minParent[target] = node
        targetMinCostLink = nil
      end

      local parent = road[node]
      for child, data in pairs(graph[node]) do
        local edgeCost
        local outNode = invert and node or child
        if road[child] == nil then -- if the shortest path to child has not already been found
          edgeCost = data.len * dirCoeff[data.oneWay and data.inNode == outNode] * drivCoeff[data.drivability > cutOffDrivability]
          pathCost = cost + edgeCost
          local childMinCost = minCost[child]
          if not childMinCost or pathCost < childMinCost then
            q:insert(pathCost, child)
            minCost[child] = pathCost
            minParent[child] = node
          end
        end
        if cost < targetMinCost and child ~= parent then
          tmpVec:setSub2(positions[child], p1) -- edgeVec
          local xnorm = min(1, max(0, tmpVec:dot(nodeToTargetVec) / (tmpVec:squaredLength() + 1e-30)))
          if xnorm > 0 and xnorm < 1 then
            tmpVec:setScaled(-xnorm)
            tmpVec:setAdd(nodeToTargetVec) -- distToEdgeVec
            pathCost = cost + (edgeCost or data.len * dirCoeff[data.oneWay and data.inNode == outNode] * drivCoeff[data.drivability > cutOffDrivability]) * xnorm +
                      square(square(tmpVec.x) + square(tmpVec.y) + square(wZ * tmpVec.z))
            if pathCost < targetMinCost then
              q:insert(pathCost, target)
              targetMinCost = pathCost
              minParent[target] = node
              targetMinCostLink = child
            end
          end
        end
      end
    end

    cost, node = q:pop()
  until not cost

  local path = {targetMinCostLink} -- last path node has to be added ad hoc
  local e = #path
  target = road[node] -- if all is well, node here should be the targetPos
  while target do
    e = e + 1
    path[e] = target
    target = road[target]
  end

  if not invert then
    for i = 1, e * 0.5 do -- reverse order to get source to target
      path[i], path[e] = path[e], path[i]
      e = e - 1
    end
  end

  return path
end

function Graphpath:getPointToPointPath(sourcePos, iter, targetPos, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff, wZ)
  -- sourcePos: path source position
  -- startPosLinks: graph nodes closest (by some measure) to sourcePos to be used as links to it
  -- targetPos: target position (vec3)
  -- cutOffDrivability: penalize roads with drivability <= cutOffDrivability
  -- dirMult: penalty to be applied to an edge designated oneWay if it is traversed in opposite direction (should be larger than 1 typically >= 1e3-1e4).
  --          If equal to nil or 1 then it means no penalty.
  -- penaltyAboveCutoff: penalty multiplier for roads above the drivability cutoff
  -- penaltyBelowCutoff: penalty multiplier for roads below the drivability cutoff
  -- wZ: number (typically >= 1). When higher than 1 the destination node of optimum path will be biased towards minimizing height difference to targetPos.
  if sourcePos == nil or targetPos == nil or sourcePos == targetPos then return {} end

  local sourceNode, sourceCost, sourceXnorm = iter() -- get the closest neighboor to sourcePos
  if sourceNode == nil then return {} end

  local minCost = table.new(0, 32)
  minCost[sourceNode] = sourceCost
  local xnorms = table.new(0, 32)
  xnorms[sourceNode] = sourceXnorm
  local minParent = table.new(0, 32)
  minParent[sourceNode] = false

  local node, cost = sourceNode, sourceCost
  sourceNode, sourceCost, sourceXnorm = nil, nil, nil

  local graph = self.graph
  local positions = self.positions

  wZ = wZ or 1
  cutOffDrivability = cutOffDrivability or 0
  penaltyAboveCutoff = penaltyAboveCutoff or 1
  penaltyBelowCutoff = penaltyBelowCutoff or 10000

  local dirCoeff = {[true] = dirMult or 1, [false] = 1}
  local drivCoeff = {[true] = penaltyAboveCutoff, [false] = penaltyBelowCutoff}

  local road = table.new(0, 32) -- initialize shortest paths linked list
  local targetMinCost = square(square(sourcePos.x-targetPos.x) + square(sourcePos.y-targetPos.y) + square(wZ * (sourcePos.z-targetPos.z))) -- upper bound estimate for the path cost
  local targetMinCostLink = nil
  local nodePos, tmpVec, nodeToTargetVec = vec3(), vec3(), vec3()

  local tmpNode = table.new(0, 2)
  local tmpEdge1Data = table.new(0, 4)
  local tmpEdge2Data = table.new(0, 4)

  local q = newMinheap() -- initialize que

  repeat
    if road[node] == nil then -- if the shortest path to this node has not already been found
      road[node] = minParent[node] -- set predessesor of node in shortest path to node
      if node == targetPos then break end

      local nodeLinks
      if graph[node] then
        nodeLinks = graph[node]
        nodePos:set(positions[node])
      else
        local n1id, n2id = node[1], node[2]
        local edgeData = graph[n1id][n2id]
        local dist, driv, inNode, oneWay = edgeData.len, edgeData.drivability, edgeData.inNode, edgeData.oneWay
        local xnorm = xnorms[node]

        table.clear(tmpNode)

        tmpEdge1Data.len = dist * xnorm
        tmpEdge1Data.drivability = driv
        tmpEdge1Data.inNode = (inNode == n2id and node) or inNode
        tmpEdge1Data.oneWay = oneWay
        tmpNode[n1id] = tmpEdge1Data

        tmpEdge2Data.len = dist * (1 - xnorm)
        tmpEdge2Data.drivability = driv
        tmpEdge2Data.inNode = (inNode == n1id and node) or inNode
        tmpEdge2Data.oneWay = oneWay
        tmpNode[n2id] = tmpEdge2Data

        nodeLinks = tmpNode
        nodePos:setLerp(positions[n1id], positions[n2id], xnorm)
      end

      nodeToTargetVec:setSub2(targetPos, nodePos)
      -- Cost of path if we were to go straight from this node to the target position
      local pathCost = cost + square(square(nodeToTargetVec.x) + square(nodeToTargetVec.y) + square(wZ * nodeToTargetVec.z))
      -- if pathCost is lower than the current upper bound estimate insert the targetPos in que with this cost
      if pathCost < targetMinCost then
        q:insert(pathCost, targetPos)
        targetMinCost = pathCost -- update upper bound estimate
        minParent[targetPos] = node -- set node as the tentative predessesor of targetPos
        targetMinCostLink = nil
      end

      local parent = road[node]
      for child, edgeData in pairs(nodeLinks) do
        local edgeCost
        if road[child] == nil then -- if the shortest path to child has not already been found
          edgeCost = edgeData.len * dirCoeff[edgeData.oneWay and edgeData.inNode == child] * drivCoeff[edgeData.drivability > cutOffDrivability]
          local pathToChildCost = cost + edgeCost
          if pathToChildCost < (minCost[child] or math.huge) then
            q:insert(pathToChildCost, child)
            minCost[child] = pathToChildCost
            minParent[child] = node
          end
        end

        -- Update targetMinCost if part of the edge between node and child can be used to reach targetPos.
        if cost < targetMinCost and child ~= parent then
          tmpVec:setSub2(positions[child], nodePos)
          local xnorm = tmpVec:dot(nodeToTargetVec) / (tmpVec:squaredLength() + 1e-30)
          if xnorm > 0 and xnorm < 1 then
            tmpVec:setScaled(-xnorm)
            tmpVec:setAdd(nodeToTargetVec) -- distance vector from targetPos to edge between node and child
            pathCost = cost +
                      (edgeCost or edgeData.len * dirCoeff[edgeData.oneWay and edgeData.inNode == child] * drivCoeff[edgeData.drivability > cutOffDrivability]) * xnorm +
                      square(square(tmpVec.x) + square(tmpVec.y) + square(wZ * tmpVec.z))
            if pathCost < targetMinCost then
              q:insert(pathCost, targetPos)
              targetMinCost = pathCost
              minParent[targetPos] = node
              targetMinCostLink = child
            end
          end
        end
      end
    end

    if not sourceNode then
      sourceNode, sourceCost, sourceXnorm = iter()
    end

    if (q:peekKey() or math.huge) <= (sourceCost or math.huge) then
      cost, node = q:pop()
    else
      minCost[sourceNode] = sourceCost
      xnorms[sourceNode] = sourceXnorm
      minParent[sourceNode] = false
      node, cost = sourceNode, sourceCost
      sourceNode, sourceCost, sourceXnorm = nil, nil, nil
    end
  until not node

  local path = {targetMinCostLink} -- last path node has to be added ad hoc
  local e = #path
  local target = road[node] -- if all is well, node here should be the targetPos
  while target do
    e = e + 1
    path[e] = target
    target = road[target]
  end

  if e == 0 then return {} end -- shortest path does not use the road network

  -- add the starNode link to the path if it is not there
  if graph[path[e]] == nil then
    local tmp1 = path[e][1]
    local tmp2 = path[e][2]
    path[e] = nil
    e = e - 1
    if path[e] == tmp1 and path[e-1] ~= tmp2 then
      e = e + 1
      path[e] = tmp2
    elseif path[e] == tmp2 and path[e-1] ~= tmp1 then
      e = e + 1
      path[e] = tmp1
    end
  end

  for i = 1, e * 0.5 do -- reverse order to get source to target
    path[i], path[e] = path[e], path[i]
    e = e - 1
  end

  return path
end

local function numOfLanesFromRadius(rad1, rad2)
  return max(1, math.floor(min(rad1, rad2 or math.huge) * 2 / 3.45 + 0.5))
end

-- return num of total lanes and lanes in the inNode to outNode dir
function Graphpath:numOfEdgeLanes(inNode, outNode)
  local lanesN, inDirLanesN
  local edge = self.graph[inNode][outNode]
  if edge.lanes then
    lanesN, inDirLanesN = #edge.lanes, 0
    for i = 1, lanesN, 1 do
      if edge.lanes:byte(i) == 43 then inDirLanesN = inDirLanesN + 1 end
    end
  else -- make up some lane data in case they don't exist
    if edge.oneWay then
      lanesN = numOfLanesFromRadius(self.radius[inNode], self.radius[outNode])
      inDirLanesN = lanesN
    else
      inDirLanesN = max(1, math.floor(numOfLanesFromRadius(self.radius[inNode], self.radius[outNode]) * 0.5))
      lanesN = 2 * inDirLanesN -- same number of lanes in both directions
    end
  end

  inDirLanesN = edge.inNode == inNode and inDirLanesN or lanesN - inDirLanesN

  return lanesN, inDirLanesN
end

function Graphpath:getPathT(start, mePos, pathLenLim, illegalDirPenalty, initDir)
  local graph = self.graph
  local radius = self.radius
  if graph[start] == nil then return {} end

  pathLenLim = square(pathLenLim)

  local positions = self.positions
  local cost, node = -1, start
  local minParent = {[node] = false}
  local queued = {}
  local road = {} -- predessesor of node in the shortest path to node
  local curSegDir = vec3(initDir.x, initDir.y, 0)
  local inFwdFlow, inBackFlow, inDrivability, inGated = radius[start], 0, 1, false
  local candidates = table.new(10, 0)
  local cN, sum = 0, 0

  local nextSegDir = vec3()

  local q = newMinheap()
  repeat
    if road[node] == nil then
      local parent = minParent[node]
      road[node] = parent
      local nodePos = positions[node]
      if parent then
        if mePos:squaredDistance(nodePos) >= pathLenLim then
          cN = cN + 1
          candidates[cN] = node
          sum = sum + cost
          if cN == 10 then break end
          goto continue
        end
        curSegDir:setSub2(nodePos, positions[parent]); curSegDir.z = 0; curSegDir:normalize()
        local totalLanes, numOfLanesInDir = self:numOfEdgeLanes(parent, node)
        inDrivability = graph[parent][node].drivability
        local laneFlow =  radius[node] / totalLanes
        inFwdFlow = laneFlow * numOfLanesInDir + 1e-8
        inBackFlow = laneFlow * (totalLanes - numOfLanesInDir)
        inGated = graph[parent][node].gated ~= 0
      end
      local linkCount = max(1, tableSize(graph[node]) - (minParent[node] and 1 or 0))
      for child, edgeData in pairs(graph[node]) do
        if road[child] == nil then
          local newChildCost
          if linkCount > 1 then
            nextSegDir:setSub2(positions[child], nodePos); nextSegDir.z = 0; nextSegDir:normalize()
            local cDirCoef = 0.5 * max(0, 1 + nextSegDir:dot(curSegDir))
            local t = square(square(min(1, graph[node][child].drivability / inDrivability))) * square(cDirCoef)
            local totalLanes, numOfLanesInDir = self:numOfEdgeLanes(node, child)
            local laneFlow = radius[child] / totalLanes
            local outFwdFlow = laneFlow * numOfLanesInDir + 1e-11
            local outBackFlow = laneFlow * (totalLanes - numOfLanesInDir)
            local outGated = inGated and 1 or 1 / max(graph[node][child].gated, 1)
            newChildCost = cost * t * min(inFwdFlow, outFwdFlow) * (1 + min(inBackFlow, outBackFlow)) / (inFwdFlow * (1 + inBackFlow)) * outGated
          else
            newChildCost = cost
          end

          if (queued[child] or 0) > newChildCost then
            q:insert(newChildCost, child)
            minParent[child] = node
            queued[child] = newChildCost
          end
        end
      end
      ::continue::
    end

    local newNode
    cost, newNode = q:pop()
    node = newNode or node -- just in case newNode is nil
  until not cost -- que is empty

  if candidates[1] then
    local i, res = 1, math.random() * sum
    sum = queued[candidates[i]]
    while sum > res do
      i = i + 1
      sum = sum + queued[candidates[i]]
    end
    node = candidates[i]
  end

  -- Data for Visual debug (also uncomment return value)
  -- sum = 0
  -- local candidatePaths = {winner = node}
  -- for i = 1, #candidates do sum = sum + queued[candidates[i]] end
  -- for i = 1, #candidates do
  --   table.insert(candidatePaths, invertPath(candidates[i], road))
  --   candidatePaths[i].score = queued[candidates[i]] / sum
  -- end

  local path = invertPath(node, road)

  return path--, candidatePaths
end

local function getUnexploredNeighboors(graph, node, road, road1, tab)
  -- gets all neighboors of node that have not been explored
  if not tab then tab = {} end
  table.clear(tab)
  for k, _ in pairs(graph[node]) do
    if road[k] == nil and road1[k] == nil then tab[k] = true end
  end
  return tab
end

function Graphpath:getFlows(inNode, outNode)
  local totalLanes, numOfLanesInDir = self:numOfEdgeLanes(inNode, outNode)
  local laneFlow = self.radius[outNode] / totalLanes
  local fwdFlow = laneFlow * numOfLanesInDir
  local backFlow = laneFlow * (totalLanes - numOfLanesInDir)

  return fwdFlow, backFlow
end

function Graphpath:getPathTWithState(start, mePos, pathLenLim, state)
  -- start: node id
  -- mePos: position from which the path terminating distance condition is measured (ex. position of vehicle or of the start node)
  -- pathLenLim: minimum length of path
  -- state: 1) an initial vec3 direction (ex. vehicle direction vector) or
  --        2) the previous path (a sequence of nodes). Last node of previous path must be the same as the start node.

  local graph = self.graph
  if graph[start] == nil then return {} end
  local radius = self.radius
  local positions = self.positions

  mePos = mePos or positions[start]
  pathLenLim = pathLenLim or 150

  --pathLenLim = square(pathLenLim * (1 + (math.random() - 0.5) * 0.4))
  pathLenLim = square(pathLenLim)

  local distLim, startMinParent, startNodeScore, prevPathInitDir, prevPath, initDir
  local minParent1, queued1, road1, prevPathSet = {}, {}, {}, {} -- only road1 needs to be initialized here
  local curSegDir, nextSegDir = vec3(), vec3()
  local q1 = newMinheap()
  if not state or type(state) == 'cdata' then -- state is an initial direction vector
    initDir = state or curSegDir -- state is not mandatory
    startMinParent = false
    startNodeScore = -1
  else -- state is a path
    prevPath = state
    local prevPathCount = #prevPath
    prevPathInitDir = positions[prevPath[2]] - positions[prevPath[1]]
    distLim = positions[start]:squaredDistance(positions[prevPath[1]])

    q1:insert(-1, prevPath[1])
    minParent1[prevPath[1]] = false -- needs to be false because of the parent check
    queued1[prevPath[1]] = -1
    prevPathSet[prevPath[1]] = true

    local nodeLinkCount = tableSize(graph[prevPath[1]])
    local inFwdFlow, inBackFlow, inDrivability, inGated = radius[prevPath[1]], 0, 1, false
    curSegDir:set(prevPathInitDir.x, prevPathInitDir.y, 0); curSegDir:normalize()

    for i = 2, prevPathCount do
      -- each iteration computes a "score" for the "child" node
      local prevNode, node, child = prevPath[i-2], prevPath[i-1], prevPath[i]
      prevPathSet[child] = true

      local newChildCost
      if nodeLinkCount > (prevNode and 2 or 1) then -- if the predecessor of "child" is a junction
        if prevNode then
          curSegDir:setSub2(positions[node], positions[prevNode]); curSegDir.z = 0; curSegDir:normalize()
          inDrivability = graph[prevNode][node].drivability
          inFwdFlow, inBackFlow = self:getFlows(prevNode, node)
          inFwdFlow = inFwdFlow + 1e-8
          inGated = graph[prevNode][node].gated ~= 0
        end

        nextSegDir:setSub2(positions[child], positions[node]); nextSegDir.z = 0; nextSegDir:normalize()
        local t = square(min(1, graph[node][child].drivability / inDrivability)) * square(0.5 * max(0, 1 + nextSegDir:dot(curSegDir)))
        local outFwdFlow, outBackFlow = self:getFlows(node, child)
        outFwdFlow = outFwdFlow + 1e-11
        local outGated = inGated and 1 or 1 / max(graph[node][child].gated, 1)
        newChildCost = queued1[node] * t * min(inFwdFlow, outFwdFlow) * (1 + min(inBackFlow, outBackFlow)) / (inFwdFlow * (1 + inBackFlow)) * outGated
      else -- child has the same "score" as its predecessor
        newChildCost = queued1[node]
      end

      nodeLinkCount = tableSize(graph[child])
      if nodeLinkCount > 2 or i == prevPathCount then -- child node is a junction (has more than two links) or is the start node of the next path
        q1:insert(newChildCost, child)
      else -- does not need to be explored
        road1[child] = node
      end

      minParent1[child] = node
      queued1[child] = newChildCost
    end

    startMinParent = prevPath[prevPathCount-1]
    startNodeScore = queued1[start]
  end

  local q = newMinheap()
  q:insert(startNodeScore, start)
  local queued = {[start] = startNodeScore}
  local minParent = {[start] = false}
  local road = {} -- predessesor of node in the shortest path to node
  local childNodeTab = getUnexploredNeighboors(graph, start, road, road1)
  local cN, sum, candidates = 0, 0, table.new(6, 0)

  repeat
    if q1.length == 0 or not next(childNodeTab) then -- q1 is empty or neighboors of top node in q has been explored by prev path
      local cost, node = q:pop()
      if road[node] == nil then -- this is possible not needed because of the que cleanup at the end of this branch
        local parent = minParent[node]
        road[node] = parent
        local nodePos = positions[node]
        if parent then
          if mePos:squaredDistance(nodePos) >= pathLenLim then
            cN = cN + 1
            candidates[cN] = node
            sum = sum + cost
            if cN == 4 or cost > sum * 2e-4 then break end -- less than 1 in 5000 of being selected (both cost and sum are negative so use ">")
            goto continue
          end
        end
        local inFwdFlow, inBackFlow, inDrivability, inGated
        if parent or startMinParent then
          local prevNode = parent or startMinParent
          curSegDir:setSub2(nodePos, positions[prevNode]); curSegDir.z = 0; curSegDir:normalize()
          inDrivability = graph[prevNode][node].drivability
          inFwdFlow, inBackFlow = self:getFlows(prevNode, node)
          inFwdFlow = inFwdFlow + 1e-8
          inGated = graph[prevNode][node].gated ~= 0
        else
          curSegDir:set(initDir.x, initDir.y, 0); curSegDir:normalize()
          inFwdFlow, inBackFlow, inDrivability, inGated = radius[start], 0, 1, false
        end
        local gnode = graph[node]
        local linkCountGT1 = next(gnode, next(gnode)) -- node has more than 1 neighboors
        local linkCountCheck = next(gnode, linkCountGT1) or (not minParent[node] and not startMinParent and linkCountGT1) -- node has more than 2 neighboors or is start node with more than 1 neighboors (the only node without a minParent at this point is the start node)
        for child, edgeData in pairs(graph[node]) do
          if road[child] == nil and child ~= startMinParent then -- TODO: child ~= startMinParent -> in the case of a dead end it will not be able to turn back
            local newChildCost
            if linkCountCheck then
              nextSegDir:setSub2(positions[child], nodePos); nextSegDir.z = 0; nextSegDir:normalize()
              local t = square(min(1, graph[node][child].drivability / inDrivability)) * square(0.5 * max(0, 1 + nextSegDir:dot(curSegDir)))
              local outFwdFlow, outBackFlow = self:getFlows(node, child)
              outFwdFlow = outFwdFlow + 1e-11
              local outGated = inGated and 1 or 1 / max(graph[node][child].gated, 1)
              newChildCost = cost * t * min(inFwdFlow, outFwdFlow) * (1 + min(inBackFlow, outBackFlow)) / (inFwdFlow * (1 + inBackFlow)) * outGated
            else
              newChildCost = cost
            end

            --if not (minParent1[child] == node and queued1[node] == cost) then newChildCost = cost end

            if (queued1[child] or 0) < newChildCost then
              newChildCost = newChildCost * 0.1
            end

            if (queued[child] or 0) > newChildCost then
              q:insert(newChildCost, child)
              minParent[child] = node
              queued[child] = newChildCost
            end
          end
        end
        ::continue::
      end
      while q.length > 0 and road[q.vals[1]] ~= nil do q:pop() end
      getUnexploredNeighboors(graph, q.vals[1], road, road1, childNodeTab) -- populate child node table with children of new top node
    else
      local cost, node = q1:pop()
      childNodeTab[node] = nil -- remove node from childNodeTab if it is there
      if road1[node] == nil and road[node] == nil then
        local parent = minParent1[node]
        road1[node] = parent
        local nodePos = positions[node]
        local inFwdFlow, inBackFlow, inDrivability, inGated
        if parent then
          if mePos:squaredDistance(nodePos) > max(distLim, pathLenLim) then -- not prevPathScores[node] and
            goto continue
          end
          curSegDir:setSub2(nodePos, positions[parent]); curSegDir.z = 0; curSegDir:normalize()
          inDrivability = graph[parent][node].drivability
          inFwdFlow, inBackFlow = self:getFlows(parent, node)
          inFwdFlow = inFwdFlow + 1e-8
          inGated = graph[parent][node].gated ~= 0
        else
          curSegDir:set(prevPathInitDir.x, prevPathInitDir.y, 0); curSegDir:normalize()
          inFwdFlow, inBackFlow, inDrivability, inGated = radius[prevPath[1]], 0, 1, false
        end
        local gnode = graph[node]
        local linkCountGT1 = next(gnode, next(gnode)) -- node has more than 1 neighboors
        local linkCountCheck = next(gnode, linkCountGT1) or (not minParent1[node] and linkCountGT1) -- node has more than 2 neighboors or is start node with more than 1 neighboors
        for child, edgeData in pairs(graph[node]) do
          if not prevPathSet[child] and road1[child] == nil and road[child] == nil then
            local newChildCost
            if linkCountCheck then
              nextSegDir:setSub2(positions[child], nodePos); nextSegDir.z = 0; nextSegDir:normalize()
              local t = square(min(1, graph[node][child].drivability / inDrivability)) * square(0.5 * max(0, 1 + nextSegDir:dot(curSegDir)))
              local outFwdFlow, outBackFlow = self:getFlows(node, child)
              outFwdFlow = outFwdFlow + 1e-11
              local outGated = inGated and 1 or 1 / max(graph[node][child].gated, 1)
              newChildCost = cost * t * min(inFwdFlow, outFwdFlow) * (1 + min(inBackFlow, outBackFlow)) / (inFwdFlow * (1 + inBackFlow)) * outGated
            else
              newChildCost = cost
            end

            if (queued1[child] or 0) > newChildCost then
              q1:insert(newChildCost, child)
              minParent1[child] = node
              queued1[child] = newChildCost
            end
          end
        end
        ::continue::
      end
    end
  until q.length <= 0

  local i, res = 1, math.random() * sum
  sum = queued[candidates[1]]
  if sum == nil then return {} end
  while sum > res do
    i = i + 1
    sum = sum + queued[candidates[i]]
  end

  -- Data for Visual debug
  -- sum = 0
  -- local candidatePaths = {winner = candidates[i]}
  -- for i = 1, #candidates do sum = sum + queued[candidates[i]] end
  -- for i = 1, #candidates do
  --   table.insert(candidatePaths, invertPath(candidates[i], road))
  --   candidatePaths[i].score = queued[candidates[i]] / sum
  -- end

  return invertPath(candidates[i], road) --, candidatePaths
end

function Graphpath:getFilteredPath(start, goal, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff)
  local graph = self.graph
  if graph[start] == nil or graph[goal] == nil then return {} end

  cutOffDrivability = cutOffDrivability or 0
  penaltyAboveCutoff = penaltyAboveCutoff or 1
  penaltyBelowCutoff = penaltyBelowCutoff or 10000

  local drivCoeff = {[true] = penaltyAboveCutoff, [false] = penaltyBelowCutoff}
  local dirCoeff = {[true] = dirMult or 1, [false] = 1}

  local cost, node = 0, start
  local minParent = {[node] = false}
  local road = {} -- predecessor subgraph
  local queued = {}

  local q = newMinheap()
  repeat
    if road[node] == nil then
      road[node] = minParent[node]
      if node == goal then break end
      for child, data in pairs(graph[node]) do
        if road[child] == nil then
          local currentChildCost = queued[child]
          local newChildCost = cost + data.len * dirCoeff[data.oneWay and data.inNode == child] * drivCoeff[data.drivability > cutOffDrivability]
          if currentChildCost == nil or currentChildCost > newChildCost then
            q:insert(newChildCost, child)
            minParent[child] = node
            queued[child] = newChildCost
          end
        end
      end
    end
    cost, node = q:pop()
  until not cost

  return invertPath(goal, road)
end

function Graphpath:spanMap(source, nodeBehind, target, edgeDict, dirMult)
  local graph = self.graph
  if graph[source] == nil or graph[target] == nil then return {} end

  dirMult = dirMult or 1
  local dirCoeff = {[true] = dirMult, [false] = 1}

  local q = newMinheap()
  local cost, t = 0, {source, false}
  local road = {} -- predecessor subgraph
  local queued = {}

  repeat
    local node = t[1]
    if road[node] == nil then
      road[node] = t[2]
      if node == target then break end
      for child, data in pairs(graph[node]) do
        if road[child] == nil then
          local currentChildCost = queued[child]
          local newChildCost = cost + data.len * dirCoeff[data.oneWay and data.inNode == child] * (edgeDict[node..'\0'..child] or 1e20) * ((node == source and child == nodeBehind and 300) or 1)
          if currentChildCost == nil or currentChildCost > newChildCost then
            q:insert(newChildCost, {child, node})
            queued[child] = newChildCost
          end
        end
      end
    end
    cost, t = q:pop()
  until not cost

  return invertPath(target, road)
end

function Graphpath:getPathAwayFrom(start, goal, mePos, stayAwayPos, dirMult)
  local graph = self.graph
  if graph[start] == nil or graph[goal] == nil then return {} end

  dirMult = dirMult or 1
  local dirCoeff = {[true] = dirMult, [false] = 1}

  local positions = self.positions
  local q = newMinheap()
  local cost, t = 0, {start, false}
  local road = {} -- predecessor subgraph
  local queued = {}

  repeat
    local node = t[1]
    if road[node] == nil then
      road[node] = t[2]
      if node == goal then break end
      for child, data in pairs(graph[node]) do
        if road[child] == nil then
          local currentChildCost = queued[child]
          local childPos = positions[child]
          local newChildCost = cost + data.len * dirCoeff[data.oneWay and data.inNode == child] * mePos:squaredDistance(childPos) / (stayAwayPos:squaredDistance(childPos) + 1e-30)
          if currentChildCost == nil or currentChildCost > newChildCost then
            q:insert(newChildCost, {child, node})
            queued[child] = newChildCost
          end
        end
      end
    end
    cost, t = q:pop()
  until not cost

  return invertPath(goal, road)
end

function Graphpath:getMaxNodeAround(start, radius, dir)
  local graph = self.graph
  if graph[start] == nil then return nil end

  local graphpos = self.positions
  local startpos = graphpos[start]
  local stackP = 1
  local stack = {start}
  local visited = {}
  local maxFoundNode = start
  local maxFoundScore = 0

  repeat
    local node = stack[stackP]
    stack[stackP] = nil
    stackP = stackP - 1

    local nodeStartVec = graphpos[node] - startpos
    local posNodeDist = nodeStartVec:squaredLength()
    local posNodeScore = dir and nodeStartVec:dot(dir) or posNodeDist

    if posNodeScore > maxFoundScore then
      maxFoundScore = posNodeScore
      maxFoundNode = node
    end

    if posNodeDist < radius * radius then
      for child, _ in pairs(graph[node]) do
        if visited[child] == nil then
          visited[child] = 1
          stackP = stackP + 1
          stack[stackP] = child
        end
      end
    end
  until stackP <= 0

  return maxFoundNode
end

-- using breadth-first search, returns a list of intersection nodes and their respective links
function Graphpath:getBranchNodesAround(start, maxRadius)
  local graph = self.graph
  if graph[start] == nil then return nil end

  maxRadius = maxRadius or 1000

  local posDict = self.positions
  local startPos = posDict[start]
  local stackP = 1
  local stack = {start}
  local visited = {}
  local branches = {}

  repeat
    local node = stack[stackP]
    stack[stackP] = nil
    stackP = stackP - 1

    local posNodeDist = posDict[node]:squaredDistance(startPos)

    if posNodeDist < maxRadius * maxRadius then
      local childCount = 0
      for child, _ in pairs(graph[node]) do
        if visited[child] == nil then
          visited[child] = 1
          stackP = stackP + 1
          stack[stackP] = child
          childCount = childCount + 1
        end
      end

      if childCount >= 2 then
        table.insert(branches, {node = node, links = tableKeys(graph[node]), sqDist = posNodeDist})
      end
    end
  until stackP <= 0

  return branches
end

function Graphpath:getChasePath(nodeBehind, nodeAhead, targetNodeBehind, targetNodeAhead, mePos, meVel, targetPos, targetVel, dirMult) -- smart chase path processing
  local graphpos = self.positions

  local wp1pos, wp2pos = graphpos[nodeBehind], graphpos[nodeAhead]
  local twp1pos, twp2pos = graphpos[targetNodeBehind] - targetVel, graphpos[targetNodeAhead] + targetVel -- positions with extra velocity based distance
  -- the extra distance is used to determine if the target has crossed into a parallel segment
  local meToTarget = (targetPos + targetVel:normalized()) - mePos -- target point is slightly ahead of original pos
  local meDotTarget = meToTarget:dot(targetVel)
  local wpAhead = meToTarget:dot(wp1pos - mePos) > meToTarget:dot(wp2pos - mePos) and nodeBehind or nodeAhead -- best wp that goes to target wp
  local twpAhead = wpAhead == targetNodeBehind and targetNodeBehind or targetNodeAhead -- check if best wp matches target wp
  if meDotTarget > 0 and targetNodeAhead == nodeBehind then targetNodeAhead = nodeAhead end

  local path = self:getPath(wpAhead, twpAhead, dirMult)

  if meVel:squaredLength() >= 9 and meDotTarget >= 9 and meVel:dot(graphpos[path[1]] - mePos) < 0 then -- simply pick waypoint ahead if driving same as player
    path = {nodeAhead}
  end

  return path
end

local fleeDirScoreCoeff = {[false] = 1, [true] = 0.8}
function Graphpath:getFleePath(startNode, initialDir, chasePos, pathLenLimit, rndDirCoef, rndDistCoef)
  local graph = self.graph
  if graph[startNode] == nil then return nil end

  pathLenLimit = pathLenLimit or 100
  rndDirCoef = rndDirCoef or 0
  rndDistCoef = min(rndDistCoef or 0.05, 1)
  local graphpos = self.positions
  local visited = {startNode = 0.2}
  local path = {startNode}
  local pathLen = 0

  local prevNode = startNode
  local prevDir = vec3(initialDir)
  local rnd2 = rndDirCoef * 2
  local chaseAIdist = graphpos[prevNode]:squaredDistance(chasePos) * 0.1

  repeat
    local maxScore = -math.huge
    local maxNode = -1
    local maxVec
    local maxLen

    local rDistCoef = min(1, pathLen * rndDistCoef)

    -- randomize dir
    prevDir:set(
      prevDir.x + (random() * rnd2 - rndDirCoef) * rDistCoef,
      prevDir.y + (random() * rnd2 - rndDirCoef) * rDistCoef,
      0)

    local prevPos = graphpos[prevNode]
    local chaseCoef = min(0.5, rndDistCoef * chaseAIdist)

    for child, link in pairs(graph[prevNode]) do
      local childPos = graphpos[child]
      local pathVec = childPos - prevPos
      local pathVecLen = pathVec:length()
      local driveability = link.drivability
      local vis = visited[child] or 1
      local posNodeScore = vis * fleeDirScoreCoeff[link.oneWay and link.inNode == child] * driveability * (3 + pathVec:dot(prevDir) / max(pathVecLen, 1)) * max(0, 3 + (chasePos - childPos):normalized():dot(prevDir) * chaseCoef)
      visited[child] = vis * 0.2
      if posNodeScore >= maxScore then
        maxNode = child
        maxScore = posNodeScore
        maxVec = pathVec
        maxLen = pathVecLen
      end
    end

    if maxNode == -1 then
      break
    end

    prevNode = maxNode
    prevDir = maxVec / (maxLen + 1e-30)
    pathLen = pathLen + maxLen
    table.insert(path, maxNode)

  until pathLen > pathLenLimit

  return path
end

local dirScoreCoeff = {[true] = 0.1, [false] = 1}
function Graphpath:getRandomPathG(startNode, initialDir, pathLenLimit, rndDirCoef, rndDistCoef, oneway)
  local graph = self.graph
  if graph[startNode] == nil then return nil end

  pathLenLimit = pathLenLimit or 100
  rndDirCoef = rndDirCoef or 0
  rndDistCoef = min(rndDistCoef or 0.05, 1e30)
  local graphpos = self.positions
  local visited = {startNode = 0.2}
  local path = {startNode}
  local pathLen = 0

  if oneway == nil then oneway = true end

  local prevNode = startNode
  local ropePos = graphpos[prevNode] - initialDir * 15

  dirScoreCoeff[true] = oneway == false and 1 or 0.1

  repeat
    local maxScore = -math.huge
    local maxNode = -1
    local maxVec
    local maxLen

    local curPos = graphpos[prevNode]
    local prevDir = curPos - ropePos
    local prevDirLen = prevDir:length()
    prevDir = prevDir / (prevDirLen + 1e-30) -- prevDir gets normalized here, to use with next randomization of prevDir
    ropePos = curPos - prevDir * min(prevDirLen, 15)

    -- randomize dir
    local rDistDirCoef = min(1, pathLen * rndDistCoef) * rndDirCoef
    prevDir:set(
      prevDir.x + (random() * 2 - 1) * rDistDirCoef,
      prevDir.y + (random() * 2 - 1) * rDistDirCoef,
      0)

    prevDir:normalize()

    for child, link in pairs(graph[prevNode]) do
      local pathVec = graphpos[child] - curPos
      local pathVecLen = pathVec:length()
      local vis = visited[child] or 1
      local posNodeScore = vis * dirScoreCoeff[link.oneWay and link.inNode == child] * link.drivability * (2 + pathVec:dot(prevDir) / max(pathVecLen, 1))
      visited[child] = vis * 0.2
      if posNodeScore >= maxScore then
        maxNode = child
        maxScore = posNodeScore
        maxLen = pathVecLen
        maxVec = pathVec
      end
    end

    if maxNode == -1 then
      break
    end

    if maxVec:dot(prevDir) <= 0 then
      ropePos = curPos
    end
    prevNode = maxNode
    pathLen = pathLen + maxLen
    table.insert(path, maxNode)

  until pathLen > pathLenLimit

  return path
end

-- produces a random path with a bias towards edge coliniarity
function Graphpath:getRandomPath(nodeAhead, nodeBehind, dirMult)
  local graph = self.graph
  if graph[nodeAhead] == nil or graph[nodeBehind] == nil then return {} end

  dirMult = dirMult or 1
  local dirCoeff = {[true] = dirMult, [false] = 1}

  local positions = self.positions

  local q = newMinheap()
  local cost, t = 0, {nodeAhead, false}
  local road = {} -- predecessor subgraph
  local queued = {}
  local node
  local choiceSet = {}
  local costSum = 0
  local pathLength = {[nodeBehind] = 0}

  repeat
    if road[t[1]] == nil then
      node = t[1]
      local parent = t[2] or nodeBehind
      road[node] = t[2]
      pathLength[node] = pathLength[parent] + (positions[node] - positions[parent]):length()
      if pathLength[node] <= 300 or not t[2] then
        local nodePos = positions[node]
        local edgeDirVec = (positions[parent] - nodePos):normalized()
        for child, data in pairs(graph[node]) do
          if road[child] == nil then
            local childCurrCost = queued[child]
            local penalty = 1 + 10 * square(max(0, edgeDirVec:dot((positions[child] - nodePos):normalized()) - 0.2))
            local childNewCost = cost + penalty * data.len * dirCoeff[data.oneWay and data.inNode == child] * ((node == nodeAhead and child == nodeBehind) and 1e4 or 1)
            if childCurrCost == nil or childCurrCost > childNewCost then
              queued[child] = childNewCost
              q:insert(childNewCost, {child, node})
            end
          end
        end
      else
        tableInsert(choiceSet, {node, square(1/cost)})
        costSum = costSum + square(1/cost)
        if #choiceSet == 5 then
          break
        end
      end
    end
    cost, t = q:pop()
  until not cost

  local randNum = costSum * math.random()
  local runningSum = 0

  for i = 1, #choiceSet do
    local newRunningSum = choiceSet[i][2] + runningSum
    if runningSum <= randNum and randNum <= newRunningSum then
      node = choiceSet[i][1]
      break
    end
    runningSum = newRunningSum
  end

  return invertPath(node, road)
end

-- public interface
M.newMinheap = newMinheap
M.newGraphpath = newGraphpath
return M
