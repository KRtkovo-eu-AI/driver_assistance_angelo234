-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Well hello there. All the code below is in a *really early* work-in-progress state, since it's just a brief incursion in my spare time. It will likely end up fully replaced, or abandoned, depending on time constraints, on other blocking tasks, etc. So best if you assume this will lead nowhere in the foreseeable future - stenyak

local M = {}

local min, max, abs, floor, sqrt, acos, deg = math.min, math.max, math.abs, math.floor, math.sqrt, math.acos, math.deg
local debug = true
local prevPacenoteSource = nil

-- Add path for saving/loading routes
local function getRouteJsonPath()
  local levelName = core_levels.getLevelName(getMissionFilename())
  return string.format("temp/rally_route_%s.json", level, map)
end

-- from the 3 points, assume they form a circle, and return:
--   - distance from p1 to p2
--   - distance from p2 to p3
--   - circle center
--   - circle angle covered by the points
local function circleDist1Dist2CenterAngleFromPoints(p1, p2, p3)
  local d1 = p1 - p3
  local d2 = p2 - p3
  local asql = d1:squaredLength()
  local bsql = d2:squaredLength()
  local adotb = d1:dot(d2)

  -- calculate lengths
  local d1l = sqrt(asql)
  local d2l = sqrt(bsql)

  -- calculate center
  local condVec = d1:cross(d2)
  local condVecSqLen = condVec:squaredLength()
  local center = p3 + ((bsql * (asql - adotb)) * d1 - (asql * (adotb - bsql)) * d2) / (2 * condVecSqLen + 1e-30), sqrt(condVecSqLen)

  -- calculate angle
  local angleCos = adotb / (d1l*d2l + 1e-30)
  local angleRad = acos(clamp(angleCos, -1, 1))
  local angle = deg(angleRad)*2

  return d1, d2, d1l, d2l, center, angle
end

local ci = ColorI(0,0,0,0)
local function temporaryColorI(color)
  ci.r, ci.g, ci.b, ci.a = floor(color.r*255), floor(color.g*255), floor(color.b*255), floor(color.a*255)
  return ci
end

local green = ColorF(0.1, 0.9, 0.1, 0.6)
local blue = ColorF(0.1, 0.1, 0.9, 0.6)
local grey = ColorF(0.5, 0.5, 0.5, 1.0)
local black = ColorF(0.0, 0.0, 0.0, 1.0)
local red = ColorF(0.9, 0.1, 0.1, 0.6)
local yellow = ColorF(0.9, 0.9, 0.1, 0.6)
local white = ColorF(1.0, 1.0, 1.0, 1.0)
local black = ColorF(0.0, 0.0, 0.0, 1.0)

local darkgreen  = ColorF(0.15, 0.73, 0.15, 1.0)
local darkorange = ColorF(1.00, 0.45, 0.00, 1.0)
local darkred    = ColorF(0.85, 0.00, 0.00, 1.0)
local darkviolet = ColorF(0.58, 0.00, 0.83, 1.0)
local darkcyan   = ColorF(0.00, 0.85, 0.95, 1.0)
local darkblue   = ColorF(0.36, 0.55, 0.80, 1.0)

local inf = 1/0
local severities = { }
table.insert(severities, {name="Hairpin" , velmax=50 , color=darkviolet, name2="square", name3="Slow" })
table.insert(severities, {name="Medium"  , velmax=70 , color=darkred    })
table.insert(severities, {name="Fast"    , velmax=100, color=darkorange })
table.insert(severities, {name="Easy"    , velmax=130, color=darkgreen  })
table.insert(severities, {name="Flat"    , velmax=170, color=darkblue   })
table.insert(severities, {name="Straight", velmax=inf, color=white      })
local severities = { }
table.insert(severities, {name="Hairpin", velmax=50 , color=darkviolet, name2="K", name3="Slow" })
table.insert(severities, {name="2",       velmax=80 , color=darkred    })
table.insert(severities, {name="3",       velmax=110, color=darkorange })
table.insert(severities, {name="4",       velmax=150, color=darkgreen  })
table.insert(severities, {name="5",       velmax=200, color=darkcyan   })
table.insert(severities, {name="6",       velmax=300, color=darkblue   })
table.insert(severities, {name="",        velmax=inf, color=white      })
local tightestId = 1
local slowId = 2
local straightId = #severities


local function getTurnVelocityWithSlickTires(radius)
  local frictionCoef = 1.7 -- slick tires
  local downforce = 9.8 -- earth gravity
  local accel = frictionCoef * downforce
  return sqrt(accel*radius) -- velocity in m/s
end

local function computeNodeData(route)
  for i=2, #route-1 do
    local n = route[i]
    local context = 3
    local ip1 = clamp(i-context, 1, #route)
    local ip2 = clamp(i+context, 1, #route)
    local p1, p2, p3 = route[ip1].pos, route[i].pos, route[ip2].pos
    local p1z, p2z, p3z = p1:z0(), p2:z0(), p3:z0()
    local d1, d2, d1l, d2l, center, angle = circleDist1Dist2CenterAngleFromPoints(p1z, p2z, p3z)
    center.z = p2.z
    n.center = center
    n.nextPos = route[i+1].pos
    n.length = n.pos:distance(n.nextPos)

    n.radius = p1:distance(center)
    --v.vel = 3.6 * (route[i-1].speed or getTurnVelocityWithSlickTires(v.radius)) -- in kmh
    n.vel = 3.6 * getTurnVelocityWithSlickTires(n.radius) -- in kmh
    n.time = n.length / (min(200, n.vel) / 3.6)

    if i > 2 then
      local nPrev = route[i-1]
      n.accel = (nPrev.vel - n.vel) / n.length -- get velocity accel against previous arc
      n.raccel = (nPrev.radius - n.radius) / n.length -- get radius accel against previous arc
    end

    -- get severity (based on velocity)
    for severityId,severity in ipairs(severities) do
      if n.vel < severity.velmax then
        n.severityId = severityId
        n.severity = severity
        break
      end
    end

    local cross = (p3z-p2z):cross(p2z-p1z).z
    n.direction = n.severityId == straightId and "" or (cross > 0 and " right" or " left")
    n.angle = angle * sign(cross) -- positive angle is right, negative is left
  end
end

local function computePacenoteData(pacenote)
  if not pacenote then return end
  if not next(pacenote.nodes) then return end
  local origSlowestNode
  if pacenote.slowestNode then
    if pacenote.slowestNode.severityId == straightId then
      origSlowestNode = pacenote.slowestNode
    end
  end
  pacenote.time = 0
  pacenote.length = 0
  pacenote.angle = 0
  local slowestNode = origSlowestNode
  for _,n in ipairs(pacenote.nodes) do
    pacenote.time = pacenote.time + n.time
    pacenote.length = pacenote.length + n.length
    pacenote.angle = pacenote.angle + n.angle
    slowestNode = slowestNode or n
    if n.vel < slowestNode.vel then slowestNode = origSlowestNode or n end
  end
  pacenote.slowestNode = slowestNode
  pacenote.name = slowestNode.severity.name
  if slowestNode.severityId == tightestId then
    if abs(pacenote.angle) < 60 then
      pacenote.name = slowestNode.severity.name3
    elseif abs(pacenote.angle) < 120 then
      pacenote.name = slowestNode.severity.name2
    end
  elseif slowestNode.severityId == straightId then
    if pacenote.length < 10 then
      pacenote.name = "into"
    elseif pacenote.length < 20 then
      pacenote.name = "then"
    else
      pacenote.name = string.format("%i", floor(0.5+pacenote.length/10)*10)
    end
  end
  --pacenote.name = string.format("[%.1fs] %s", pacenote.time, pacenote.name)
end

local function splitRouteIntoPacenotes(route)
  local pacenotes = {}
  local pacenote
  for i=2, #route-3 do
    local nCurr = route[i]
    local nNext = route[i+1]
    pacenote = pacenote or { nodes={} }
    table.insert(pacenote.nodes, nCurr)
    -- split left turns from right turns from straights
    if nCurr.direction ~= nNext.direction then
      computePacenoteData(pacenote)
      table.insert(pacenotes, pacenote)
      pacenote = nil
    end
  end
  return pacenotes
end

-- merge consecutive pacenotes with the same direction
local function simplifyStraightsConsecutive(pacenotes)
  local nodesMoved = 0
  local i = 1
  while i < #pacenotes do
    local pacenote = pacenotes[i]
    local pacenoteNext = pacenotes[i+1]
    if pacenote.slowestNode.direction == pacenoteNext.slowestNode.direction then
      for _,node in ipairs(pacenoteNext.nodes) do
        nodesMoved = nodesMoved + 1
        table.insert(pacenote.nodes, node)
      end
      computePacenoteData(pacenote)
      table.remove(pacenotes, i+1)
    else
      i = i + 1
    end
  end
  return nodesMoved
end

-- merge straights with next nodes that don't deviate from the line too much
local function simplifyStraightsExtendNext(pacenotes, distanceThreshold, distanceThresholdStraight)
  local movedTotal = 0
  local i = 0
  while i < #pacenotes do
    i = i + 1
    local pacenote = pacenotes[i]
    if pacenote.slowestNode.severityId ~= straightId then goto continue end
    local p1 = pacenote.nodes[1].pos
    local p2 = pacenote.nodes[#pacenote.nodes].pos

    local stillInline = true
    local movedCurr = 0
    local j = i+1
    while j < #pacenotes do
      local pacenoteNext = pacenotes[j]
      local movedNext = 0
      local k = 1
      while k <= #pacenoteNext.nodes do
        local node = pacenoteNext.nodes[k]
        local dist = node.pos:distanceToLine(p1, p2)
        local threshold = pacenoteNext.slowestNode.severityId == straightId and distanceThresholdStraight or distanceThreshold
        if dist > threshold then stillInline = false break end
        -- node is close enough to the line, move it from next pacenote to this one
        table.insert(pacenote.nodes, node)
        table.remove(pacenoteNext.nodes, k)
        movedNext = movedNext + 1
      end
      movedCurr = movedCurr + movedNext
      if movedNext > 0 then computePacenoteData(pacenoteNext) end
      if #pacenoteNext.nodes == 0 then table.remove(pacenotes, j) end
      if not stillInline then break end
    end
    movedTotal = movedTotal + movedCurr
    if movedCurr > 0 then computePacenoteData(pacenote) end
    ::continue::
  end
  return movedTotal
end

-- merge straights with previous nodes that don't deviate from the line too much
local function simplifyStraightsExtendPrev(pacenotes, distanceThreshold, distanceThresholdStraight)
  local movedTotal = 0
  local i = #pacenotes+1
  while i > 1 do
    i = i - 1
    local pacenote = pacenotes[i]
    if pacenote.slowestNode.severityId ~= straightId then goto continue end
    local p1 = pacenote.nodes[1].pos
    local p2 = pacenote.nodes[#pacenote.nodes].pos

    local stillInline = true
    local movedCurr = 0
    local j = i
    while j > 1 do
      j = j - 1
      local pacenotePrev = pacenotes[j]
      local movedPrev = 0
      local k = #pacenotePrev.nodes+1
      while k > 1 do
        k = k - 1
        local node = pacenotePrev.nodes[k]
        local dist = node.pos:distanceToLine(p1, p2)
        local threshold = pacenotePrev.slowestNode.severityId == straightId and distanceThresholdStraight or distanceThreshold
        if dist > threshold then stillInline = false break end
        -- node is close enough to the line, move it from previous pacenote to this one
        table.insert(pacenote.nodes, 1, node)
        table.remove(pacenotePrev.nodes, k)
        movedPrev = movedPrev + 1
      end
      movedCurr = movedCurr + movedPrev
      if movedPrev > 0 then computePacenoteData(pacenotePrev) end
      if #pacenotePrev.nodes == 0 then table.remove(pacenotes, j) end
      if not stillInline then break end
    end
    movedTotal = movedTotal + movedCurr
    if movedCurr > 0 then computePacenoteData(pacenote) end
    ::continue::
  end
  return movedTotal
end

local function simplifyPacenotes(pacenotes)
  local distanceThreshold = 0.5
  local distanceThresholdStraight = 0.5
  log("I", "", string.format("Simplifying %d pacenotes:", #pacenotes))
  for i=1, 10 do
    local n = 0
    n = n + simplifyStraightsExtendNext(pacenotes, distanceThreshold, distanceThresholdStraight)
    n = n + simplifyStraightsExtendPrev(pacenotes, distanceThreshold, distanceThresholdStraight)
    n = n + simplifyStraightsConsecutive(pacenotes)
    log("I", "", string.format(" - Phase %d: %d pacenotes (%d nodes reassigned)", i, #pacenotes, n))
    if n == 0 then break end -- nothing left to do
  end
end

local function getPacenotesFromRoute(route)
  if not route then
    log("E", "", "Unable to compute pacenotes, no route provided")
    return
  end
  if #route < 3 then
    log("E", "", "Unable to compute pacenotes, route too short: "..dumps(#route))
    return
  end
  computeNodeData(route)
  local pacenotes = splitRouteIntoPacenotes(route) -- 2nd pass: merge identical consecutive arcs into a single pacenote
  simplifyPacenotes(pacenotes)
  log("I", "", string.format("Generated a rally route from '%s' with %d pacenotes", prevPacenoteSource, #pacenotes))
  return pacenotes
end

local rlcolor = ColorF(0,0,0,0)
local tagPos = vec3()

local function renderNode(node, renderLine, renderText)
  local color = node.severity.color
  if renderLine then
    debugDrawer:drawSphere(node.pos, 0.7, color)
    debugDrawer:drawCylinder(node.pos, node.nextPos, 0.15, color)
  end

  if renderText then
    local c = node.angle > 0 and darkblue or darkred
    local vel = currentVelocity or node.vel
    local mul = clamp(node.vel/(200), 0, 1)
    rlcolor.r, rlcolor.g, rlcolor.b, rlcolor.a = c.r*mul, c.g*mul, c.b*mul, c.a
    --rlcolor = color
    local txt = string.format("%.0fkmh, %.0fm, %.0fdeg, %.1fm/ss", vel, node.length, node.angle, node.accel or 0)
    --local txt = string.format("%.0fkmh, %0.1fs", vel, node.time)
    tagPos:set(node.pos)
    tagPos.z = tagPos.z + 2
    debugDrawer:drawCylinder(node.pos, tagPos, 0.05, color)
    debugDrawer:drawSphere(node.pos, 0.7, color)
    debugDrawer:drawSphere(node.nextPos, 0.7, color)
    debugDrawer:drawTextAdvanced(tagPos, txt, black, true, false, temporaryColorI(color))
  end
end

local function getPacenoteCall(txt, pacenote, pacenoteNext)
  local linkText = " into"
  if not pacenoteNext then
    linkText = ""
  elseif pacenoteNext.slowestNode.severityId == straightId then
    linkText = " "..pacenoteNext.name
  end
  return  txt..linkText
end
local function getPacenoteText(pacenote)
  local shortThreshold = 0.7 -- in seconds
  local longThreshold = 3.0 -- in seconds
  local veryLongThreshold = 4.0 -- in seconds
  local tightensThreshold = 1.6 -- in normalized percentage (1.6 means 60% tightening)

  local slowestNode = pacenote.slowestNode
  local firstNode = pacenote.nodes[1]
  local lastNode = pacenote.nodes[#pacenote.nodes]
  local veryLong = slowestNode.severityId ~= straightId and (pacenote.time > veryLongThreshold) or false
  local long = not veryLong and slowestNode.severityId ~= straightId and (pacenote.time > longThreshold) or false
  local short = slowestNode.severityId ~= straightId and (pacenote.time < shortThreshold) or false
  local tightens = slowestNode.severityId ~= straightId and (firstNode.vel / lastNode.vel > tightensThreshold) or false
  local opens = slowestNode.severityId ~= straightId and (firstNode.vel / lastNode.vel < 1/tightensThreshold)
  return string.format("%s%s%s%s%s%s" -- %.0fdeg"
    ,pacenote.name
    ,slowestNode.direction
    ,long and " long" or ""
    ,short and " short" or ""
    ,tightens and " tightens" or ""
    ,opens and " opens" or ""
    --,pacenote.angle
  )
end

local pacenoteThickness = 0.35
local pacenoteThickness = 0.15
local function renderPacenote(pacenote, txt)
  local slowestNode = pacenote.slowestNode
  local color = slowestNode.severity.color
  for i,node in ipairs(pacenote.nodes) do
    if i == 1 then
      tagPos:set(node.pos)
      tagPos.z = tagPos.z + 2
      debugDrawer:drawCylinder(node.pos, tagPos, 0.05, color)
      debugDrawer:drawSphere(node.pos, pacenoteThickness*2, black)
      debugDrawer:drawTextAdvanced(tagPos, txt, black, true, false, temporaryColorI(white))
    end
    debugDrawer:drawCylinder(node.pos, node.nextPos, pacenoteThickness, color)
  end
end

local minAmountToRender = 2
local function renderNextPacenotes(pacenotes, timeToRender, currentPosition, currentVelocity)
  local predictedTime = 4 -- upcoming distance to show pacenotes for
  local iClosest, jClosest, distClosest = nil, nil, 1e30
  for i,pacenote in ipairs(pacenotes) do
    for j,node in ipairs(pacenote.nodes) do
      local dist = currentPosition:squaredDistance(node.pos)
      if dist < distClosest then
        iClosest, jClosest, distClosest = i, j, dist
      end
    end
  end

  if iClosest then
    local amountRendered = 0
    local timeToRender = predictedTime -- how many seconds worth of pacenotes to show on screen at once
    local nextNode = jClosest
    for i=iClosest, #pacenotes do
      local pacenote = pacenotes[i]
      for j=nextNode or 1, #pacenote.nodes do
        local node = pacenote.nodes[j]
        local arcVel = min(currentVelocity, node.vel) -- at current or potential reduced speed
        local arcTime = node.length / arcVel
        timeToRender = timeToRender - arcTime
      end
      nextNode = nil
      local txt = getPacenoteText(pacenote)
      renderPacenote(pacenote, txt)
      amountRendered = amountRendered + 1
      if amountRendered == 2 then
        if pacenote.slowestNode.severityId ~= straightId then
          local pacenoteNext = pacenotes[i+1]
          local txtCall = getPacenoteCall(txt, pacenote, pacenoteNext)
          guihooks.trigger('ScenarioRealtimeDisplay', {msg = txtCall})
        end
      end
      if amountRendered >= minAmountToRender and timeToRender <= 0 then break end
    end
  end
end

local function renderAllPacenotes(pacenotes, showNodes)
  for i,pacenote in ipairs(pacenotes) do
    for j,node in ipairs(pacenote.nodes) do
      if showNodes then
        local dist = core_camera.getPosition():distance(node.pos)
        renderNode(node, dist < 1000, dist < 100)
      end
    end

    if not showNodes then
      local txt = getPacenoteText(pacenote)
      renderPacenote(pacenote, txt)
    end
  end
end

local function getWaypointsFromGroundMarkers()
  local startPos = core_groundMarkers.routePlanner.path[1].pos
  local targetPos = core_groundMarkers.routePlanner.path[#core_groundMarkers.routePlanner.path].pos
  return { startPos, targetPos }
end

local function getRouteFromAtoB(startPos, targetPos)
  local startNode1, _ = map.findClosestRoad(startPos)
  local endNode1, _ = map.findClosestRoad(targetPos)
  local path = map.getGraphpath():getPath(startNode1, endNode1)

  local settings = {
    fixPathNodes = false,
    --fixPathNodes = true,
    fixEndpoints = false,
    evolvePathNormals = false,
    evolveMidNormals = false,
    evolveEndNormals = false,
    segDistSplitLim = 4,
    hta = 0.1,
    fRange = 1,
    forceMag = 'angle', -- 'curvature' or 'angle'
    forceScale = nil, -- 'normalize' or nil
    forceMultiplier = 2,
    iterations = 180, -- necessary if running the optimization off-line
    distFromEdge = 0.1,
    dispLimFromCenterAbs = nil,
    dispLimFromCenterRel = 0.2
  }

  local trajectory = map.optimizePath(path, settings) -- will run the optimization off-line
  --map.debugDrawTrajectory(trajectory)
  return trajectory
end

local vehPos = vec3()
local vehVel = vec3()
local pacenotes = nil
local function onUpdate(dt, dtSim)
  if core_groundMarkers.currentlyHasTarget() then
    if prevPacenoteSource ~= 'groundMarkers' then
      prevPacenoteSource = 'groundMarkers'
      log("I", "", string.format("Generating rally pacenotes from '%s'", prevPacenoteSource))
      local waypoints = getWaypointsFromGroundMarkers()
      local route = getRouteFromAtoB(waypoints[1], waypoints[2])
      writeFile(getRouteJsonPath(), lpack.encode(route))
      pacenotes = getPacenotesFromRoute(route)
    end
  else
    if prevPacenoteSource ~= 'fileCache' then
      prevPacenoteSource = 'fileCache'
      log("I", "", string.format("Generating rally pacenotes from '%s'", prevPacenoteSource))
      local route = lpack.decode(readFile(getRouteJsonPath()))
      pacenotes = getPacenotesFromRoute(route)
    end
  end

  if not pacenotes then return end

  local veh = getPlayerVehicle(0)
  local vel = 0
  if veh then
    vehPos:set(veh:getPositionXYZ())
    vehVel:set(veh:getVelocityXYZ())
    vel = vehVel:length()
  end

  if vel < 1 then
    if debug then
      renderAllPacenotes(pacenotes, dtSim < 0.001)
    end
  else
    local timeToRender = 4
    renderNextPacenotes(pacenotes, timeToRender, vehPos, vel)
  end
end

M.onUpdate = onUpdate

return M
