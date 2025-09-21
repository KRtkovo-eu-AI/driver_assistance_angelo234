--[[
This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
If a copy of the bCDDL was not distributed with this
file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
This module contains a set of functions which manipulate behaviours of vehicles.
]]

local M = {}

local byte = string.byte
local t_insert, t_clear, t_sort = table.insert, table.clear, table.sort
local rad, deg, min, max = math.rad, math.deg, math.min, math.max

M.ignoreSections = {maxIDs=true, options=true}

-- these are defined in C, do not change the values
local NORMALTYPE = 0
local BEAM_LBEAM = 4

local function increaseMax(vehicle, name)
  local res = vehicle.maxIDs[name] or 0
  vehicle.maxIDs[name] = res + 1
  return res
end

local function addNodeWithOptions(vehicle, pos, ntype, options)
  local n
  if type(options) == 'table' then
    n = deepcopy(options)
  else
    n = {}
  end

  local nextID = increaseMax(vehicle, 'nodes')
  n.cid     = nextID
  n.pos     = pos
  n.ntype   = ntype

  --log('D', "jbeam.addNodeWithOptions","adding node "..(nextID)..".")
  t_insert(vehicle.nodes, n)
  return nextID
end

local function addNode(vehicle, pos, ntype)
  return addNodeWithOptions(vehicle, pos, ntype, vehicle.options)
end

local function addBeamWithOptions(vehicle, id1, id2, beamType, options, id3)
  id1 = id1 or options.id1
  id2 = id2 or options.id2

  -- check if nodes are valid
  local node1 = vehicle.nodes[id1]
  local node2 = vehicle.nodes[id2]
  if node1 == nil or node2 == nil then
    if node1 == nil then
      log('W', "jbeam.addBeamWithOptions","invalid node "..tostring(id1).." for new beam between "..tostring(id1).."->"..tostring(id2))
      return
    end
    if node2 == nil then
      log('W', "jbeam.addBeamWithOptions","invalid node "..tostring(id2).." for new beam between "..tostring(id1).."->"..tostring(id2))
      return
    end
  end

  -- increase counters
  local nextID = increaseMax(vehicle, 'beams')

  local b
  if type(options) == 'table' then
    b = deepcopy(options)
  else
    b = {}
  end

  if id3 ~= nil then
    local node3 = vehicle.nodes[id3]
    if node3 == nil then
      log('W', "jbeam.addBeamWithOptions","invalid node "..tostring(id3).." for new beam between "..tostring(id1).."->"..tostring(id2))
      return
    else
      beamType = BEAM_LBEAM
    end
    b.id3 = node3.cid
  end

  b.cid      = nextID
  b.id1      = node1.cid
  b.id2      = node2.cid
  b.beamType = beamType

  -- add the beam
  t_insert(vehicle.beams, b)
  return b
end

local function addBeam(vehicle, id1, id2)
  return addBeamWithOptions(vehicle, id1, id2, NORMALTYPE, vehicle.options)
end

local function addRotator(vehicle, wheelKey, wheel)
  wheel.frictionCoef = wheel.frictionCoef or 1

  local nodes = {}
  if wheel._group_nodes ~= nil then
    for k, v in ipairs(wheel._group_nodes) do
      t_insert(nodes, vehicle.nodes[v])
    end
  end

  if wheel._rotatorGroup_nodes ~= nil then
    for k, v in ipairs(wheel._rotatorGroup_nodes) do
      t_insert(nodes, vehicle.nodes[v])
    end
  end

  if next(nodes) ~= nil then
    wheel.nodes = nodes
  end
end

local MatrixF = MatrixF or nop -- allows bananabench to run if MatrixF is not defined

local cacheOpsMat, cacheNodeRots, cacheNodeOffsets, cacheNodeMoves, cachePosXSign, cacheOpCount = MatrixF(true), {}, {}, {}, 0, 0
local cachePosX, cachePosY, cachePosZ = 0, 0, 0
local cacheUsingMatrixMath = false
local nodeRots, nodeOffsets, nodeMoves, opMin, opMax, opCount = {}, {}, {}, math.huge, -math.huge, 0
local tmpMat1, tmpMat2 = MatrixF(true), MatrixF(true)
local tmpVec1 = vec3()

local function getNodeRotOffsetMoves(jbeamData)
  table.clear(nodeRots)
  table.clear(nodeOffsets)
  table.clear(nodeMoves)
  opMin, opMax, opCount = math.huge, -math.huge, 0

  for nodeKey, data in pairs(jbeamData) do
    if type(nodeKey) == 'string' and type(data) == 'table' and type(data.x) == 'number' and type(data.y) == 'number' and type(data.z) == 'number'
    and byte(nodeKey,4) == 101 and byte(nodeKey,3) == 100 and byte(nodeKey,2) == 111 and byte(nodeKey,1) == 110 then
      -- check for "nodeRotate##"
      if byte(nodeKey,10)==101 and byte(nodeKey,9)==116 and byte(nodeKey,8)==97 and byte(nodeKey,7)==116 and byte(nodeKey,6)==111 and byte(nodeKey,5)==82 then
        local idx1, idx2 = byte(nodeKey,11), byte(nodeKey,12)
        idx1 = idx1 ~= nil and idx1 - 48 or 0
        if idx2 ~= nil then idx1 = idx1 * 10 + idx2 - 48 end
        nodeRots[idx1] = data
        opMin, opMax, opCount = min(opMin, idx1), max(opMax, idx1), opCount + 1

      -- check for "nodeOffset##", ignoring offset if jbeamData.ignoreNodeOffset
      elseif not jbeamData.ignoreNodeOffset and byte(nodeKey,10)==116 and byte(nodeKey,9)==101 and byte(nodeKey,8)==115 and byte(nodeKey,7)==102 and byte(nodeKey,6)==102 and byte(nodeKey,5)==79 then
        local idx1, idx2 = byte(nodeKey,11), byte(nodeKey,12)
        idx1 = idx1 ~= nil and idx1 - 48 or 0
        if idx2 ~= nil then idx1 = idx1 * 10 + idx2 - 48 end
        nodeOffsets[idx1] = data
        opMin, opMax, opCount = min(opMin, idx1), max(opMax, idx1), opCount + 1

      -- check for "nodeMove##"
      elseif byte(nodeKey,8)==101 and byte(nodeKey,7)==118 and byte(nodeKey,6)==111 and byte(nodeKey,5)==77 then
        local idx1, idx2 = byte(nodeKey,9), byte(nodeKey,10)
        idx1 = idx1 ~= nil and idx1 - 48 or 0
        if idx2 ~= nil then idx1 = idx1 * 10 + idx2 - 48 end
        nodeMoves[idx1] = data
        opMin, opMax, opCount = min(opMin, idx1), max(opMax, idx1), opCount + 1
      end
    end
  end
end

local function updateNodeTransformCache(posX)
  local useCache = true

  if opCount ~= cacheOpCount then
    useCache = false
  else
    for i = opMin, opMax do
      local cacheNodeRot, nodeRot = cacheNodeRots[i], nodeRots[i]
      if cacheNodeRot or nodeRot then
        if (not cacheNodeRot and nodeRot or cacheNodeRot and not nodeRot) or cacheNodeRot.x ~= nodeRot.x or cacheNodeRot.y ~= nodeRot.y or cacheNodeRot.z ~= nodeRot.z
        or cacheNodeRot.px ~= nodeRot.px or cacheNodeRot.py ~= nodeRot.py or cacheNodeRot.pz ~= nodeRot.pz then
          useCache = false
          break
        end
      end
      local cacheNodeOffset, nodeOffset = cacheNodeOffsets[i], nodeOffsets[i]
      if cacheNodeOffset or nodeOffset then
        if (not cacheNodeOffset and nodeOffset or cacheNodeOffset and not nodeOffset) or cachePosXSign ~= sign(posX) or cacheNodeOffset.x ~= nodeOffset.x or cacheNodeOffset.y ~= nodeOffset.y or cacheNodeOffset.z ~= nodeOffset.z then
          useCache = false
          break
        end
      end
      local cacheNodeMove, nodeMove = cacheNodeMoves[i], nodeMoves[i]
      if cacheNodeMove or nodeMove then
        if (not cacheNodeMove and nodeMove or cacheNodeMove and not nodeMove) or cacheNodeMove.x ~= nodeMove.x or cacheNodeMove.y ~= nodeMove.y or cacheNodeMove.z ~= nodeMove.z then
          useCache = false
          break
        end
      end
    end
  end

  if useCache then return end

  table.clear(cacheNodeRots)
  table.clear(cacheNodeOffsets)
  table.clear(cacheNodeMoves)
  cachePosXSign, cacheOpCount = sign(posX), opCount
  cacheUsingMatrixMath = next(nodeRots) ~= nil

  if cacheUsingMatrixMath then
    cacheOpsMat:identity()
    for i = opMin, opMax do
      -- First apply rotation (if any)
      if nodeRots[i] then
        local op = nodeRots[i]
        if op.px or op.py or op.pz then
          local px = (op.px and type(op.px)=='number') and op.px or 0
          local py = (op.py and type(op.py)=='number') and op.py or 0
          local pz = (op.pz and type(op.pz)=='number') and op.pz or 0

          tmpMat1:identity()
          tmpMat2:identity()
          tmpVec1:set(px, py, pz)
          tmpMat1:setPosition(tmpVec1)

          tmpVec1:set(rad(-op.x), rad(-op.y), rad(-op.z))
          tmpMat2:setFromEuler(tmpVec1)
          tmpMat1:mul(tmpMat2)

          tmpMat2:identity()
          tmpVec1:set(-px, -py, -pz)
          tmpMat2:setPosition(tmpVec1)
          tmpMat1:mul(tmpMat2)
        else
          tmpVec1:set(rad(-op.x), rad(-op.y), rad(-op.z))
          tmpMat1:setFromEuler(tmpVec1)
        end
        cacheOpsMat:mul(tmpMat1)
        cacheNodeRots[i] = nodeRots[i]
      end

      -- Then apply offset (if any)
      if nodeOffsets[i] then
        local op = nodeOffsets[i]
        tmpMat1:identity()
        tmpVec1:set(cachePosXSign * op.x, op.y, op.z)
        tmpMat1:setPosition(tmpVec1)
        cacheOpsMat:mul(tmpMat1)
        cacheNodeOffsets[i] = nodeOffsets[i]
      end

      -- Finally apply move (if any)
      if nodeMoves[i] then
        local op = nodeMoves[i]
        tmpMat1:identity()
        tmpVec1:set(op.x, op.y, op.z)
        tmpMat1:setPosition(tmpVec1)
        cacheOpsMat:mul(tmpMat1)
        cacheNodeMoves[i] = nodeMoves[i]
      end
    end
  else
    cachePosX, cachePosY, cachePosZ = 0, 0, 0

    for i = opMin, opMax do
      -- Apply offset (if any)
      if nodeOffsets[i] then
        local op = nodeOffsets[i]
        cachePosX, cachePosY, cachePosZ = cachePosX + cachePosXSign * op.x, cachePosY + op.y, cachePosZ + op.z
        cacheNodeOffsets[i] = nodeOffsets[i]
      end

      -- Apply move (if any)
      if nodeMoves[i] then
        local op = nodeMoves[i]
        cachePosX, cachePosY, cachePosZ = cachePosX + op.x, cachePosY + op.y, cachePosZ + op.z
        cacheNodeMoves[i] = nodeMoves[i]
      end
    end
  end
end

local function getPosAfterNodeRotateOffsetMove(jbeamData, x, y, z)
  getNodeRotOffsetMoves(jbeamData)
  if opCount == 0 then return x, y, z end

  updateNodeTransformCache(x)

  -- cacheUsingMatrixMath is true if there are any node rotations
  if cacheUsingMatrixMath then
    tmpVec1:set(x, y, z)
    tmpVec1:set(cacheOpsMat:mulP3F(tmpVec1))
    return tmpVec1.x, tmpVec1.y, tmpVec1.z
  else
    return cachePosX + x, cachePosY + y, cachePosZ + z
  end
end

local function getFlexbodyPosRotAfterNodeRotateOffsetMove(jbeamData, x, y, z, rx, ry, rz)
  getNodeRotOffsetMoves(jbeamData)
  if opCount == 0 then return end

  updateNodeTransformCache(x)

  -- cacheUsingMatrixMath is true if there are any node rotations
  if cacheUsingMatrixMath then
    tmpVec1:set(rad(-rx), rad(-ry), rad(-rz))
    tmpMat1:setFromEuler(tmpVec1)
    tmpMat1:mul(cacheOpsMat)
    tmpVec1:set(x, y, z)
    tmpVec1:set(cacheOpsMat:mulP3F(tmpVec1))
    local eulerRot = tmpMat1:toEuler()
    return tmpVec1.x, tmpVec1.y, tmpVec1.z, deg(-eulerRot.x), deg(-eulerRot.y), deg(-eulerRot.z)
  else
    return cachePosX + x, cachePosY + y, cachePosZ + z
  end
end

--
local function getPosRotBeforeNodeRotateOffsetMove(jbeamData, x, y, z, rx, ry, rz)
  getNodeRotOffsetMoves(jbeamData)

  if opCount == 0 then
    return x, y, z, rx, ry, rz
  end

  local function calculatePosRot(posXSign)
    updateNodeTransformCache(posXSign)

    -- cacheUsingMatrixMath is true if there are any node rotations
    if cacheUsingMatrixMath then
      local opsMat = cacheOpsMat:copy()
      opsMat:inverse()

      tmpVec1:set(rad(-rx), rad(-ry), rad(-rz))
      tmpMat1:setFromEuler(tmpVec1)
      tmpMat1:mul(opsMat)
      tmpVec1:set(x, y, z)
      tmpVec1:set(opsMat:mulP3F(tmpVec1))
      local eulerRot = tmpMat1:toEuler()
      return tmpVec1.x, tmpVec1.y, tmpVec1.z, deg(-eulerRot.x), deg(-eulerRot.y), deg(-eulerRot.z)
    else
      return x - cachePosX, y - cachePosY, z - cachePosZ, rx, ry, rz
    end
  end

  local outX, outY, outZ, outRx, outRy, outRz = calculatePosRot(1)
  if sign(outX) == 1 then
    return outX, outY, outZ, outRx, outRy, outRz
  end

  outX, outY, outZ, outRx, outRy, outRz = calculatePosRot(0)
  if sign(outX) == 0 then
    return outX, outY, outZ, outRx, outRy, outRz
  end

  return calculatePosRot(-1)
end

M.addNodeWithOptions = addNodeWithOptions
M.addNode = addNode
M.increaseMax = increaseMax
M.addBeamWithOptions = addBeamWithOptions
M.addBeam = addBeam
M.addRotator = addRotator
M.getPosAfterNodeRotateOffsetMove = getPosAfterNodeRotateOffsetMove
M.getFlexbodyPosRotAfterNodeRotateOffsetMove = getFlexbodyPosRotAfterNodeRotateOffsetMove
M.getPosRotBeforeNodeRotateOffsetMove = getPosRotBeforeNodeRotateOffsetMove

return M