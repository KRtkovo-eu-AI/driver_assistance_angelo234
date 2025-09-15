-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

function C:init()
  self.path = nil
  self.vehIds = {}
  self.states = {}
  self.recoveryStates = {}
  self.time = 0
  self.lapCount = 1
  self.finishCount = 0
  self.started = false
  self.useHotlappingApp = true
  self.useDebugDraw = false
  self.useWaypointAudio = true
  self.sortPlacements = function (a, b)
    -- sorts by descending lap, then descending index, then ascending distance to next waypoint
    if a.lap == b.lap then
      if a.wpIdx == b.wpIdx then
        return a.sqDist < b.sqDist
      else
        return a.wpIdx > b.wpIdx
      end
    else
      return a.lap > b.lap
    end
  end
end

function C:setPathFile(file)
  if not file then return end
  local json = jsonReadFile(file)
  if not json then
    log('E', 'race', 'unable to find race file: ' .. tostring(file))
    return
  end
  local path = require('/lua/ge/extensions/gameplay/race/path')("New Race")
  path:onDeserialized(json)
  self:setPath(path)
end

function C:calcAIPath()
  self.aiPath, self.aiDetailedPath = self.path:getAiPath(true)
end

function C:setPath(path)
  self.path = path
  self.path:autoConfig()
  self:calcAIPath()
end

function C:setVehicleIds(ids)
  self.vehIds = ids
  self.states = {}
  self.recoveryStates = {}
  for _, id in ipairs(ids) do
    self.states[id] = {}
    self.recoveryStates[id] = {}
  end
end

function C:getDistOnNav(pointA, pointB, inNextAiPathIdx, inPrevAiPathIdx, outNextAiPathIdx, outPrevAiPathIdx)
  local inNextPos = self.aiDetailedPath[inNextAiPathIdx].pos
  local inPrevPos = self.aiDetailedPath[inPrevAiPathIdx].pos
  local outNextPos = self.aiDetailedPath[outNextAiPathIdx].pos
  local outPrevPos = self.aiDetailedPath[outPrevAiPathIdx].pos

  local distToAdd = inNextPos:distance(lerp(inPrevPos, inNextPos, clamp(pointA:xnormOnLine(inPrevPos, inNextPos), 0, 1)))
  local distToAddTwo = outPrevPos:distance(lerp(outNextPos, outPrevPos, clamp(pointB:xnormOnLine(outNextPos, outPrevPos), 0, 1)))

  local path = map.getPath(self.aiDetailedPath[inNextAiPathIdx].wp, self.aiDetailedPath[outPrevAiPathIdx].wp)
  local distanceAlongPath = 0
  local distanceCut = pointA:distance(pointB)
  if #path == 2 then
    distanceAlongPath = distanceCut
  else
    for i = 1, #path-1 do
      local a,b = path[i],path[i+1]
      a,b = map.getMap().nodes[a].pos, map.getMap().nodes[b].pos
      distanceAlongPath = distanceAlongPath + (a-b):length()
    end
    distanceAlongPath = distanceAlongPath + distToAdd + distToAddTwo
  end

  return distanceAlongPath, distanceCut
end

local penaltyData = {
  falsePositive = 1, -- if player goes off track just a little, its fine
  totalPenaltyThresh = 5, -- giving the player some slack
  delayToApplyPenalty = 4, -- don't apply penalty right away

  currDelayToApplyPenalty = 0,
  currTotalPenaltyToApply = 0,
  currTotalPenalty = 0,

  offTrackPath = {} -- record the player's pos when he is off track to better determine a penalty
}

function C:calcCutTrackTimePenalty(distCutAlongPath, distCut, offPos, onPos, maxCutTrack)
  -- punish people that have cut the track more than people that have gone offtrack along the track
  local diff = distCutAlongPath - distCut
  return tonumber(string.format("%.1f", (distCutAlongPath / 200) * math.max(diff, 0.5)))
end

function C:applyPenalty(timePenalty)
  dump(string.format("Time penalty : %f seconds", timePenalty))
end

function C:detectOffTrack(data, dt)
  if not data.penaltyData then
    data.penaltyData = deepcopy(penaltyData)
  end

  if data.lastFrameIsOnRoad ~= nil then
    if not data.isOnRoad then
      data.penaltyData.currDelayToApplyPenalty = 0
    end

    -- just drove off the track
    if data.lastFrameIsOnRoad and not data.isOnRoad then
      -- remember where we drove off
      data.offPos = vec3(data.frontPos)
      data.inPrevAiPathIdx = data.aiPathIdx
      data.inNextAiPathIdx = data.aiPathIdx + 1

      data.penaltyData.offTrackPath = {}
    end

    -- while off the track, we record the player's path, to then later determine how deep he has cut the track
    if not data.isOnRoad then
      table.insert(data.penaltyData.offTrackPath, vec3(data.frontPos.x, data.frontPos.y, data.frontPos.z))
    end

    -- drove back on the track
    if not data.lastFrameIsOnRoad and data.isOnRoad then
      local maxCutDepth = 0
      for _, pos in ipairs(data.penaltyData.offTrackPath) do
        local cutDepth = pos:distanceToLineSegment(data.offPos, data.frontPos)
        if cutDepth > maxCutDepth then maxCutDepth = cutDepth end
      end
      local distCutAlongPath, distCut = self:getDistOnNav(data.offPos, data.frontPos, data.inNextAiPathIdx, data.inPrevAiPathIdx, data.aiPathIdx + 1, data.aiPathIdx)
      if distCutAlongPath > data.penaltyData.falsePositive then
        data.penaltyData.currTotalPenalty = data.penaltyData.currTotalPenalty + distCutAlongPath
      end

      if data.penaltyData.currTotalPenalty > data.penaltyData.totalPenaltyThresh then
        local timePenalty = self:calcCutTrackTimePenalty(distCutAlongPath, distCut, data.offPos, data.frontPos, maxCutDepth)
        data.penaltyData.currTotalPenaltyToApply = data.penaltyData.currTotalPenaltyToApply + timePenalty
      end
    end

    -- if we have penalties pending, start delay, so we can apply them all at once
    if data.penaltyData.currTotalPenaltyToApply > 0 then
      data.penaltyData.currDelayToApplyPenalty = data.penaltyData.currDelayToApplyPenalty + dt
      if data.penaltyData.currDelayToApplyPenalty > data.penaltyData.delayToApplyPenalty then
        self:applyPenalty(data.penaltyData.currTotalPenaltyToApply)
        data.penaltyData.currTotalPenaltyToApply = 0
        data.penaltyData.currDelayToApplyPenalty = 0
      end
    end
  end

  data.lastFrameIsOnRoad = data.isOnRoad
end

local sorted = {}
local vehVel = vec3()
local vehDir = vec3()
local vehBBCenter = vec3()
local vehBBHalfExtents = vec3()
function C:getTrackedData(dt)
  local completedCount = 0
  local sortedNextIndex = 1
  local mapNodes = map.getMap().nodes
  local pathLen = #self.aiDetailedPath
  for id, data in pairs(self.states) do
    if data.complete then
      completedCount = completedCount + 1
    else
      local lap = data.currentLap
      local segName = data.currentSegments[1]
      local wpIdx = math.huge
      local sqDist = 0

      local veh = getObjectByID(id)
      if veh then
        vehBBCenter:set(be:getObjectOOBBCenterXYZ(id))
        vehBBHalfExtents:set(be:getObjectOOBBHalfExtentsXYZ(id))
        vehVel:set(veh:getVelocityXYZ())
        vehDir:set(veh:getDirectionVectorXYZ())
        local dir = sign2(vehDir:dot(vehVel)) -- negative if driving in reverse

        vehDir:setScaled(vehBBHalfExtents.y * dir)
        data.frontPos:set(vehBBCenter)
        data.frontPos:setAdd(vehDir)

        if segName then
          wpIdx = self.path.config.graph[segName].linearCPIndex
          sqDist = data.frontPos:squaredDistance(self.path.pathnodes.objects[self.path.config.graph[segName].targetNode].pos)
        end

        local bestDist = math.huge
        local startIdx, endIdx = 1, pathLen - 1
        if data.isOnRoad then
          startIdx, endIdx = math.max(1, data.aiPathIdx - 1), math.min(pathLen - 1, data.aiPathIdx + 1) -- optimized, only previous and next nodes
        end

        for i = startIdx, endIdx do
          local dist = data.frontPos:squaredDistanceToLineSegment(self.aiDetailedPath[i].pos, self.aiDetailedPath[i + 1].pos)
          if dist < bestDist then
            bestDist = dist
            data.aiPathIdx = i
          end
        end

        local n1, n2 = self.aiDetailedPath[data.aiPathIdx], self.aiDetailedPath[data.aiPathIdx + 1]
        data.isOnRoad = true
        if n1 and n2 then
          local xnorm = data.frontPos:xnormOnLine(n1.pos, n2.pos)
          if data.aiPathIdx > 1 and data.aiPathIdx < pathLen - 1 then
            xnorm = clamp(xnorm, 0, 1)
          end
          -- if vehicle is before the start node or after the finish node, leave the xnorm unclamped
          local roadPos = linePointFromXnorm(n1.pos, n2.pos, xnorm)
          local rad = lerp(mapNodes[n1.wp] and mapNodes[n1.wp].radius or 10, mapNodes[n2.wp] and mapNodes[n2.wp].radius or 10, xnorm)

          if data.frontPos:squaredDistance(roadPos) > square(rad) then -- if front position is at the side edge of the road, test all corners
            data.isOnRoad = false
            for _, corner in ipairs(data.currentCorners) do
              if corner:squaredDistance(roadPos) <= square(rad) then
                data.isOnRoad = true
                break
              end
            end
          end
        end
      end

      -- temporarily disabled until errors get resolved
      -- self:detectOffTrack(data, dt)

      if not sorted[sortedNextIndex] then
        sorted[sortedNextIndex] = {}
      end
      sorted[sortedNextIndex].id = id
      sorted[sortedNextIndex].lap = lap
      sorted[sortedNextIndex].wpIdx = wpIdx
      sorted[sortedNextIndex].sqDist = sqDist
      sortedNextIndex = sortedNextIndex + 1
    end
  end
  -- Cut the array down if the size has shrunk
  if sortedNextIndex-1 < #sorted then
    for i=sortedNextIndex, #sorted do
      sorted[i] = nil
    end
  end

  table.sort(sorted, self.sortPlacements)

  for i, v in ipairs(sorted) do
    self.states[v.id].placement = i + completedCount
  end
end

function C:onUpdate(dt)
  if not self.started then return end
  for _, id in ipairs(self.vehIds) do

    if self.time > 0 then
      self:clearEvents(id)
    end

    if self.states[id].requestRecover then
      self:handleRecover(id)
    end
    if self.states[id].active then
      if not self.suspended then
        self:updateVehicle(id, dt)
      end
      if self.useDebugDraw then
        self:drawDebug(id)
      end
      self:digestEvents(id)

    end
  end
  if not self.suspended then
    if not self.path.branching and next(self.states) then
      self:getTrackedData(dt)
    end
    self.time = self.time + dt
  end
end

function C:doSuspend(sus)
  self.suspended = sus
  if self.useHotlappingApp then
    if sus then
      guihooks.trigger('HotlappingTimerPause')
    else
      guihooks.trigger('HotlappingTimerUnpause')
    end
  end
end

function C:skipLap(id, keepLapProgress)
  local state = self.states[id]
  if state.complete or not state.active or not self.path.config.closed then return end

  if not state.invalidLap then
    state.invalidLap = true
    if not keepLapProgress then -- also skips the lap progress and resets the next segment
      state.skippedLap = true
    end
  end
end

function C:changeRaceVehicle(lastId, newId)
  for k, id in ipairs(self.vehIds) do
    if id == lastId then
      self.vehIds[k] = newId
    end
  end

  self.recoveryStates[newId] = self.recoveryStates[lastId]
  self.recoveryStates[lastId] = nil

  self.states[newId] = self.states[lastId]
  self.states[lastId] = nil

  local vehicle = getObjectByID(newId)
  self.states[newId].wheelOffsets = {}
  self.states[newId].currentCorners = {}
  self.states[newId].previousCorners = {}
  local wCount = vehicle:getWheelCount()-1
  if wCount > 0 then
    local vehiclePos = vehicle:getPosition()
    local vRot = quatFromDir(vehicle:getDirectionVector(), vehicle:getDirectionVectorUp())
    local x,y,z = vRot * vec3(1,0,0),vRot * vec3(0,1,0),vRot * vec3(0,0,1)
    --local oobbz = vec3(vehicle:getSpawnWorldOOBB():getHalfExtents()).z/2
    for i=0, wCount do
      local axisNodes = vehicle:getWheelAxisNodes(i)
      local nodePos = vec3(vehicle:getNodePosition(axisNodes[1]))
      local pos = vec3(nodePos:dot(x), nodePos:dot(y), nodePos:dot(z))
      table.insert(self.states[newId].wheelOffsets, pos)
      table.insert(self.states[newId].currentCorners, vRot*pos + vehiclePos)
      table.insert(self.states[newId].previousCorners, vRot*pos + vehiclePos)
    end
  end
end

function C:startAiVehicle(id, aggression)
  if not self.aiPath or not self.aiPath[1] then return end

  local aiPath = shallowcopy(self.aiPath)
  local lapCount = self.lapCount
  if self.path.config.closed then
    table.insert(aiPath, aiPath[1]) -- allows AI to continue to the next lap
    lapCount = lapCount + 1 -- extra lap after finish
  else
    local wpCount = #self.aiPath
    local finalPn = self.path.pathnodes.objects[self.path.endNode or 0]
    local finalWp = self.aiPath[wpCount]
    local finalDir
    if finalPn and not finalPn.missing and finalPn.normal then
      finalDir = finalPn.normal
    else -- try to estimate the direction vector of the finish line
      local n1, n2 = self.aiPath[wpCount], self.aiPath[wpCount - 1]
      if n1 and n2 then
        finalDir = (map.getMap().nodes[n1].pos - map.getMap().nodes[n2].pos):normalized()
      else
        finalDir = getObjectByID(id):getDirectionVector() -- last resort, this shouldn't happen
      end
    end
    local extPath = map.getGraphpath():getRandomPathG(finalWp, finalDir, 250, nil, nil, false) -- extended path beyond the finish line
    if extPath[1] == aiPath[wpCount] then
      table.remove(extPath, 1)
    end
    aiPath = arrayConcat(aiPath, extPath) -- combines the ai path with the extended path
  end

  if self.states[id] then
    self.states[id].isAiVeh = true -- flags this vehicle as AI controlled
    if not aggression and self.states[id].baseAggression then
      aggression = self.states[id].baseAggression
    end
  end
  aggression = aggression or 0.9

  local str = '{wpTargetList = '..serialize(aiPath)
  str = str..', noOfLaps = '..lapCount
  str = str..', aggression = '..aggression..'}'

  getObjectByID(id):queueLuaCommand('controller.setFreeze(false)')
  getObjectByID(id):queueLuaCommand('ai.driveUsingPath('..str..')')
end

function C:stopAiVehicle(id)
  if self.states[id] then
    self.states[id].isAiVeh = false
  end

  getObjectByID(id):queueLuaCommand('ai.setMode("stop")')
end

function C:startRace()
  self.started = true
  self.time = 0
  self.finishCount = 0
  for _, id in ipairs(self.vehIds) do
    local veh = getObjectByID(id)
    if veh then
      self.recoveryStates[id] = {}
      self.states[id] = {
        waitingForRollingStart = self.path.config.rollingStart, -- if we are not started yet, but wait for reachng the first CP.
        active = true, -- if this vehicle is currently racing.
        currentSegments = {}, -- the segments the vehicle is currently in.
        completedPacenotes = {}, --  all the pacenotes that have been triggered
        currentLap = 0, -- current lap
        currentTimes = {}, -- times (with begin/end/duration) for the current lap
        currentHistory = {}, -- segments for the current lap
        historicTimes = {}, -- all the times so far.
        historicSegments = {}, -- all the segments reached so far.
        bestLapTime = {}, -- best lap time out of all the laps
        nextPathnodes = {}, -- currently reachable pathnodes
        overNextPathnodes = {}, -- all pathnodes reachable after the next ones.
        nonBranchingShiftedPathnodes = {}, -- all future pathnodes, if the path is not branching. loops around back to the player
        frontPos = vec3(),
        aiPathIdx = 1,
        isOnRoad = true,
        events = {},
        eventLog = {},
        placement = 0,
        startTime = 0,
        endTime = 0,
        reverse = self.path.config.reverse,
        recoveriesUsed = 0
      }
      self.states[id].events.raceStarted = true
      if not self.path.config.rollingStart then
        self.states[id].currentSegments = self.path.config.startSegments
      else
        self.states[id].currentSegments = {}
      end

      self.states[id].wheelOffsets = {}
      self.states[id].currentCorners = {}
      self.states[id].previousCorners = {}
      local wCount = veh:getWheelCount()-1
      if wCount > 0 then
        local vehiclePos = veh:getPosition()
        local vRot = quatFromDir(veh:getDirectionVector(), veh:getDirectionVectorUp())
        local x,y,z = vRot * vec3(1,0,0),vRot * vec3(0,1,0),vRot * vec3(0,0,1)
        --local oobbz = vec3(vehicle:getSpawnWorldOOBB():getHalfExtents()).z/2
        for i=0, wCount do
          local axisNodes = veh:getWheelAxisNodes(i)
          local nodePos = vec3(veh:getNodePosition(axisNodes[1]))
          local pos = vec3(nodePos:dot(x), nodePos:dot(y), nodePos:dot(z))
          table.insert(self.states[id].wheelOffsets, pos)
          table.insert(self.states[id].currentCorners, vRot*pos + vehiclePos)
          table.insert(self.states[id].previousCorners, vRot*pos + vehiclePos)
        end
      end

      if self.autoAiMode and not veh:isPlayerControlled() then -- automatically assign and start ai vehicle
        self:startAiVehicle(id)
      end

      --self.states[id].insideSegments = self.path.config.startSegments
      self:findNextPathnodes(id)
    end
  end

  if self.useHotlappingApp then
    -- load hotlapping for the hotlapping app.
    if not core_hotlapping then
      extensions.load({'core_hotlapping'})
      guihooks.trigger('setQuickRaceMode')
      guihooks.trigger("HotlappingResetApp")
    end
  end
end

function C:stopRace()
  self.started = false
  if self.useHotlappingApp and core_hotlapping then
    core_hotlapping.newRaceStop()
  end

  for _, id in ipairs(self.vehIds) do
    if self.states[id].isAiVeh then
      self:stopAiVehicle(id)
    end
  end
end

-- this should probably be improved at some point
function C:getVehiclePosition(id)
  local veh = scenetree.findObjectById(id)
  if veh then
    return veh:getPosition()
  else
    if map and map.objects[id] then
      return map.objects[id].pos
    end
  end
  log("E","","No Vehicle position found! " .. id)
  return vec3()
end

function C:abortRace(id)
  local state = self.states[id]
  if state.complete or not state.active then return end
  state.currentSegments = {}
  state.active = false
  state.events.raceAborted = true
  state.endTime = self.time

  if state.isAiVeh then
    self:stopAiVehicle(id)
  end
end

function C:createRecoveryPoint(id, recovery)
  table.insert(self.recoveryStates[id], {
    state = deepcopy(self.states[id]),
    time = self.time,
    recovery = recovery}
    )
end

function C:hasRecoveryPosition(id)
  if not self.recoveryStates[id][#self.recoveryStates[id]] then
    return false
  end
  return true
end

-- returns only true if a recovery state exists
function C:requestRecover(id)
  if not self:hasRecoveryPosition(id) then return false end
  self.states[id].requestRecover = true
  return true
end

function C:handleRecover(id)
  local snap = self.recoveryStates[id][#self.recoveryStates[id]]
  self.states[id] = deepcopy(snap.state)
  if self.recoverAffectsTotalTime then
    self.time = snap.time
  end
  self.states[id].events.recovered = #self.recoveryStates
  self.states[id].events.recoveredTo = snap.recovery

  -- disable repair, third param
  snap.recovery:moveResetVehicleTo(id, nil, false)
  core_camera.resetCamera(0)
  --local veh = scenetree.findObjectById(id)
  --veh:resetBrokenFlexMesh()
  --veh:queueLuaCommand('recovery.recoverInPlace()')
end

function C:completeLap(id, endTime)
  local state = self.states[id]
  table.insert(state.historicSegments, state.currentHistory)
  local timeInfo = {
    lap = state.currentLap,
    beginTime = state.currentTimes[1].beginTime,
    endTime = endTime,
    lapTime = endTime - state.currentTimes[1].beginTime,
    segmentTimes = state.currentTimes
  }
  timeInfo.duration = timeInfo.endTime - timeInfo.beginTime

  table.insert(state.historicTimes, timeInfo)

  state.currentTimes = {}
  state.currentHistory = {}
  state.currentLap = state.currentLap+1
  state.aiPathIdx = 1
  if state.currentLap < self.lapCount then
    state.currentSegments = self.path.config.startSegments
    state.events.lapComplete = true
    state.completedPacenotes = {}
  else
    self.finishCount = self.finishCount + 1 -- tracks how many vehicles have finished the race
    state.currentSegments = {}
    state.complete = true
    state.active = false
    state.placement = self.finishCount -- accurate finish placement
    state.events.raceComplete = true
    state.endTime = endTime

    local veh = getObjectByID(id)
    if veh and self.states[id].isAiVeh then -- lower aggression past the finish line
      veh:queueLuaCommand('ai.setAggression(0.5)')
    end

    if veh and self.logRaceInfo then -- optional logging of race info
      local simpleInfo = {
        totalTime = state.endTime,
        totalTimeFormatted = self:raceTime(state.endTime),
        finalPlacement = state.placement,
        vehConfig = veh.partConfig,
        vehModel = veh.JBeam,
        lapTimes = {},
        lapTimesFormatted = {},
      }
      for _, l in ipairs(state.historicTimes) do
        table.insert(simpleInfo.lapTimes, l.duration)
        table.insert(simpleInfo.lapTimesFormatted, self:raceTime(l.duration))
      end

      do
        local fullpath = "raceRecord/"..os.date("!%Y-%m-%d--%H-%M-%S")..(self.saveFileSuffix and ("-"..self.saveFileSuffix) or "").."/"
        jsonWriteFile(fullpath.."path.path.json",self.path:onSerialize(), true)
        jsonWriteFile(fullpath.."times.json",state.historicTimes, true)
        jsonWriteFile(fullpath.."simpleInfo.json",simpleInfo, true)
        log("I","","Written complete race history to "..fullpath)
      end
    end
  end
end

function C:findNextPathnodes(id)
  local state = self.states[id]
  table.clear(state.nextPathnodes)
  table.clear(state.overNextPathnodes)
  table.clear(state.nonBranchingShiftedPathnodes)
  local lastLap = state.currentLap == self.lapCount-1
  -- find all pathnodes at the end of the current segments and following segments
  local nextIdMap = {}
  local overNextIdMap = {}
  for _, curId in ipairs(state.currentSegments) do
    local elem = self.path.config.graph[curId]
    for _, n in ipairs(elem.nextVisibleSegments) do
      local tn = self.path.config.graph[n].targetNode
      if not nextIdMap[tn] then
        table.insert(state.nextPathnodes, {self.path.pathnodes.objects[tn], elem.overNextCrossesFinish})
        nextIdMap[tn] = true
      end
    end
    if not (elem.overNextCrossesFinish and lastLap) then
      for _, n in ipairs(elem.overNextVisibleSegments) do
        local tn = self.path.config.graph[n].targetNode
        if not overNextIdMap[tn] then
          table.insert(state.overNextPathnodes, {self.path.pathnodes.objects[tn]})
          overNextIdMap[tn] = true
        end
      end
    end
    -- only do this list if the race is completely linear (no branches)
    if not self.path.branching then
      local done = false
      local lastId = state.currentSegments[1]
      table.insert(state.nonBranchingShiftedPathnodes, state.currentSegments[1])
      -- stop already if ths is the last segment in the last lap
      if lastLap and self.path.config.graph[lastId].overNextCrossesFinish then
        done = true
      end

      while not done do
        local last = self.path.config.graph[lastId]
        local nextIdx = last.overNextVisibleSegments[1]
        if not nextIdx then
          -- if there is no successor, stop here
          done = true
        elseif nextIdx == state.currentSegments[1] then
          -- stop if we reached the original segment.
          done = true
        else
          lastId = nextIdx
          table.insert(state.nonBranchingShiftedPathnodes, nextIdx)
          if lastLap and self.path.config.graph[nextIdx].overNextCrossesFinish then
            done = true
          end
        end
      end
    end
  end
  if state.waitingForRollingStart then
    local ssId = self.path.config.startSegments[1]
    local ss = self.path.segments.objects[ssId]
    table.insert(state.nextPathnodes, {ss:getFrom()})
  end

  if #state.nextPathnodes > 1 then
    for _, pn in ipairs(state.nextPathnodes) do
      pn[2] = 'branch'
    end
  else
    local pn = state.nextPathnodes[1]
    if pn then
      if pn[2] and lastLap then
        pn[2] = 'final'
      elseif pn[2] and not lastLap then
        pn[2] = 'lap'
      elseif state.waitingForRollingStart then
        pn[2] = 'start'
      elseif not self.path.startPositions.objects[pn[1].recovery].missing then
        pn[2] = 'recovery' --lap color is used for recovery as well
      else
        pn[2] = 'default'
      end
    end
  end

  -- sort lists for consistency
  table.sort(state.nextPathnodes, function(a,b) return a[1].sortOrder<b[1].sortOrder end)
  table.sort(state.overNextPathnodes, function(a,b) return a[1].sortOrder<b[1].sortOrder end)
end

function C:detectRollingStart(id, dt)
  local state = self.states[id]
  local pos = self:getVehiclePosition(id)
  local inside = {}
  table.clear(state.currentSegments)
  for _, segId in ipairs(self.path.config.startSegments) do
    local currentSegment = self.path.segments.objects[segId]
    local hit, t = currentSegment:contains(pos,state)
    inside[segId] = hit and t or nil
  end
  local t = math.huge
  for k, v in pairs(inside) do
    if v ~= nil then
      table.insert(state.currentSegments, k)
      t = math.min(t,v)
    end
  end
  state.startTime = self.time
  if #state.currentSegments > 0 then
    state.startTime = self.time + dt * t
    state.waitingForRollingStart = false
    state.events.rollingStarted = true
    self:findNextPathnodes(id)
  end
end

local vehPos = vec3()
local vehDir = vec3()
local vehDirUp = vec3()
local vehRot = quat()
function C:updateVehicle(id, dt)
  local state = self.states[id]
  if state.complete then return end
  local vehicle = getObjectByID(id)
  if not vehicle then return end
    -- advance corners
  vehPos:set(vehicle:getPositionXYZ())
  vehDir:set(vehicle:getDirectionVectorXYZ())
  vehDirUp:set(vehicle:getDirectionVectorUpXYZ())
  vehRot:setFromDir(vehDir, vehDirUp)
  for i, corner in ipairs(state.wheelOffsets) do
    state.previousCorners[i]:set(state.currentCorners[i])
    state.currentCorners[i]:setRotate(vehRot, corner)
    state.currentCorners[i]:setAdd(vehPos)
    --debugDrawer:drawLine(vec3(state.previousCorners[i]), vec3(state.currentCorners[i]),  ColorF(1,0,0,1))
    --debugDrawer:drawSphere(vec3(state.currentCorners[i]), 0.025, ColorF(1,0,0,0.25))
  end

  if state.waitingForRollingStart then
    self:detectRollingStart(id, dt)
    return
  end
  self:handlePacenotes(id)
  -- figure out if we entered any next segments
  --debugDrawer:drawSphere(pos, 0.125, ColorF(1,0,0,0.25))
  -- id of the segment we have completed (and then advance to its successors)
  local finishedSegmentId = -1
  local finishedSegmentT = 1
  -- if we have lapped in reaching a pathnode
  local lapped = false

  -- if lap was skipped, cancel the timer and set the lap point as the next pathnode
  if state.invalidLap then
    state.invalidLap = nil
    state.events.lapSkipped = true
    if state.skippedLap then -- sets the segment to the final one
      for segId, currentSegment in pairs(self.path.config.graph) do
        if currentSegment.lastInLap then
          state.currentSegments = {segId}
          self:findNextPathnodes(id)
          break
        end
      end
    end
  end

  -- go through all current segments. check if the segment is finished (reaching its end)
  for _,currentId in ipairs(state.currentSegments) do
    local currentSegment = self.path.config.graph[currentId]

    if currentSegment.lastInLap then
      -- is this element the last in lap?
      -- if we do, we can stop here
      local segment = self.path.segments.objects[currentId]
      if segment:finished(vehPos, state) then
        finishedSegmentId = currentId
        lapped = true
      end
    else
      -- if we have at least one successor, we continue
      for _, segId in ipairs(self.path.config.graph[currentId].successors) do
        local segment = self.path.segments.objects[segId]
        local hit, t = segment:contains(vehPos, state)

        if hit then
          local pn = segment:getFrom()
          finishedSegmentId = currentId
          finishedSegmentT = t
        end
      end
    end
  end
  -- if no segment is finished, we are done here
  if finishedSegmentId == -1 then
    return
  end
  -- otherwise, record the advancement and set up next segments

  -- add the segments we were in into the history
  table.insert(state.currentHistory, state.currentSegments)
  local graphElem = self.path.config.graph[finishedSegmentId]
  local timeInfo = nil
  local pathnode = self.path.pathnodes.objects[graphElem.targetNode]

  if pathnode.visible then
    -- create event that we reached a pathnode that was visible
    state.events.pathnodeReached = true
    state.events.pathnodeReachedId = self.path.segments.objects[finishedSegmentId]:getTo().id

    -- record time
    timeInfo = {
      segment = finishedSegmentId,
      endTime = self.time + finishedSegmentT * dt
    }
    -- get correct startTime for this segment
    if #state.currentTimes > 0 then
      timeInfo.beginTime = state.currentTimes[#state.currentTimes].endTime
    elseif #state.historicTimes > 0 then
      timeInfo.beginTime = state.historicTimes[#state.historicTimes].endTime
    else
      timeInfo.beginTime = state.startTime
    end
    timeInfo.duration = timeInfo.endTime - timeInfo.beginTime
    table.insert(state.currentTimes, timeInfo)
  else
    extensions.hook("onRacePathnodeReached", { race = self, pathnode = pathnode })
  end

  -- if AI racer, check if custom aggression value exists
  if self.states[id].isAiVeh then
    local val, valType, valid = pathnode.customFields:get("aiAggression")
    if valid and valType == "number" then
      if val >= 0 then
        vehicle:queueLuaCommand('ai.setAggression('..val..')')
      else
        local aggr = self.states[id].baseAggression or 0.9
        vehicle:queueLuaCommand('ai.setAggression('..aggr..')')
      end
    end
  end

  if lapped then
    -- complete lap if we have done so.
    self:completeLap(id, timeInfo.endTime)
  else
    -- otherwise set current segments to successors of completed segment.
    state.currentSegments = self.path.config.graph[finishedSegmentId].successors
  end

  -- find the next pathnodes (for displaying them)
  self:findNextPathnodes(id)

  -- check if we have reached a recovery point
  local rec = self.path.isReversed and self.path.pathnodes.objects[graphElem.targetNode]:getReverseRecovery() or self.path.pathnodes.objects[graphElem.targetNode]:getRecovery()
  if not rec.missing then
    state.events.recoveryReached = rec
  end
end

function C:handlePacenotes(id)
  local state = self.states[id]
  if state.complete then return end

  for _, segId in ipairs(state.currentSegments) do
    local pnIds = self.path.config.segmentToPacenotes[segId]
    for _, pni in ipairs(pnIds) do
      if not state.completedPacenotes[pni] then
        local pn = self.path.pacenotes.objects[pni]
        if not pn.missing then
          if pn:intersectCorners(state.previousCorners, state.currentCorners) then
            state.events.pacenoteReached = true
            state.events.pacenoteIdReached = pni
            state.completedPacenotes[pni] = true
            return
          end
        end
      end
    end
  end

end

function C:digestEvents(id)
  local state = self.states[id]
  local events = state.events
  --digest hotlapping event exclusively
  if self.useHotlappingApp and core_hotlapping and id == be:getPlayerVehicleID(0) then -- assumes that the UI should only trigger for the player vehicle
    if events.raceStarted  then
      if state.waitingForRollingStart then
        guihooks.trigger('setQuickRaceMode')
        guihooks.trigger("HotlappingResetApp")
      else
        core_hotlapping.newRaceStart(self)
      end
    elseif events.rollingStarted then
      core_hotlapping.newRaceStart(self)
    elseif events.lapComplete then
      core_hotlapping.newRacePathnodeReached(state, {lapped = true})
    elseif events.lapSkipped then
      core_hotlapping.newRaceLapSkipped(state, {lapped = false})
    elseif events.pathnodeReached then
      core_hotlapping.newRacePathnodeReached(state, {lapped = false})
    elseif events.raceComplete then
      core_hotlapping.newRacePathnodeReached(state, nil)
      core_hotlapping.newRaceStop()
    end
  end

  if events.raceStarted then
    table.insert(state.eventLog, {name = "Race Started.", time = self.time})
    extensions.hook("onRaceStarted", { race = self, time = self.time })
  end
  if events.pathnodeReached then
    local pathnode = self.path.pathnodes.objects[events.pathnodeReachedId]
    table.insert(state.eventLog, { name = "Reached " .. pathnode.name ..".", time = self.time})
    extensions.hook("onRacePathnodeReached", { race = self, time = self.time, pathnode = pathnode })
  end
  if events.lapComplete then
    table.insert(state.eventLog, {name = "Lap " .. state.currentLap.." Complete.", time = self.time})
    state.invalidLap = nil
    state.skippedLap = nil
  end
  if events.lapSkipped then
    table.insert(state.eventLog, {name = "Lap " .. state.currentLap.." Skipped.", time = self.time})
  end
  if events.raceComplete then
    table.insert(state.eventLog, {name = "Race Complete!", time = self.time})
    extensions.hook("onRaceComplete", { race = self, time = self.time })
  end
  if events.raceAborted then
    table.insert(state.eventLog, {name = "Race Aborted.", time = self.time})
    extensions.hook("onRaceAborted", { race = self, time = self.time })
  end
  if events.rollingStarted then
    table.insert(state.eventLog, {name = "Rolling Started.", time = self.time})
  end
  if events.recovered then
    table.insert(state.eventLog, {name = 'Recovered', time = self.time})
    self.recoveryStates[id][events.recovered] = deepcopy(self.states[id])
    state.recoveriesUsed = state.recoveriesUsed + 1
    state.requestRecover = false
  end
  if events.recoveryReached then
    self:createRecoveryPoint(id, events.recoveryReached)
  end

  if not events.recovered and self.useWaypointAudio and (events.rollingStarted or events.pathnodeReached or events.lapComplete or events.raceComplete) then
    Engine.Audio.playOnce('AudioGui', "event:UI_Checkpoint")
  end

  if events.pacenoteReached then
    table.insert(state.eventLog, {name = "Pacenote reached: " .. dumps(self.path.pacenotes.objects[events.pacenoteIdReached].note), time = self.time})
  end
end

function C:clearEvents(id)
  local state = self.states[id]
  local events = state.events
  table.clear(events)
end


------------- DEBUG AND IMGUI STUFF ---------------

function C:drawDebug(id)
  local state = self.states[id]
  for _, node in ipairs(state.nextPathnodes) do
    node[1]:drawDebug('simple', {1,0.16,0.08,1}, dumps(node[2]))
  end
  for _, node in ipairs(state.overNextPathnodes) do
    node[1]:drawDebug('simple', {0.91,0.64,0.1,1}, dumps(node[2]))
  end
  for _, segId in ipairs(state.currentSegments) do
    local pnIds = self.path.config.segmentToPacenotes[segId]
    for _, pni in ipairs(pnIds) do
      if not state.completedPacenotes[pni] then
        self.path.pacenotes.objects[pni]:drawDebug('simple')
      end
    end
  end

end

function C:inDrawEventlog(id, im)
  local eventLog = self.states[id].eventLog or {}
  local colWidth = im.CalcTextSize("99:99:999").x + 15 * im.uiscale[0]
  im.Text("Vehicle " .. id)
  im.Columns(2)
  im.SetColumnWidth(0,colWidth)
  im.Text("Time")
  im.NextColumn()
  im.Text("Event")
  im.NextColumn()
  im.Separator()
  if self.states[id].active then
    im.Text(self:raceTime(self.time))
    im.NextColumn()
    im.NextColumn()
  end
  for i = #eventLog, 1, -1 do
    local e = eventLog[i]
    im.Text(self:raceTime(e.time))
    im.NextColumn()
    im.Text(e.name)
    im.NextColumn()
  end
  im.Columns(1)
end

function C:inDrawTimes(id, im, detail)
  local state = self.states[id]
  local colWidth = im.CalcTextSize("Total 99").x + 15 * im.uiscale[0]
  local ret = false
  im.Columns(2)
  im.SetColumnWidth(0,colWidth)
  if editor and editor.icons then
    editor.uiIconImage(editor.icons.search, im.ImVec2(20, 20))
  else
    im.Text("Det")
  end
  if im.IsItemClicked(0) then
    ret = true
  end
  im.NextColumn()
  if state.complete then
    im.Text("Complete!")
  else
    im.Text("Lap " .. (state.currentLap+1) .."/".. self.lapCount )
  end
  im.NextColumn()
  im.Separator()
  if state.active then
    if self.lapCount > 1 then
      im.Text("Total")
      im.NextColumn()
      im.Text(self:raceTime(self.time - state.startTime))
      im.NextColumn()
    end

    local currentLapStart = state.currentLap == 0 and state.startTime or state.historicTimes[#state.historicTimes].endTime
    im.Text("Lap " .. (state.currentLap+1))
    im.NextColumn()
    im.Text(self:raceTime(self.time - currentLapStart))
    im.NextColumn()
  end
  if state.complete then
    im.Text("Total")
    im.NextColumn()
    im.Text(self:raceTime(state.endTime - state.startTime))
    im.NextColumn()
  end
  if detail then
    self:inDrawSegmentTimes(state.currentTimes, im, 'index', (state.currentLap+1) .. " - ")
    for i = #state.historicTimes, 1, -1 do
      self:inDrawLapTimes(state.historicTimes[i], im)
    end
  else
    for i = #state.historicTimes, 1, -1 do
      im.Text("Lap " .. (state.historicTimes[i].lap+1))
      im.NextColumn()
      im.Text(self:raceTime(state.historicTimes[i].duration))
      im.NextColumn()
    end
  end
  im.Columns(1)
  return ret
end


function C:inDrawSegmentTimes(sTimes, im, mode, prefix)
  mode = mode or "index"
  prefix = prefix or ""
  for i = #sTimes, 1, -1 do
    local seg = sTimes[i]
    local txt = ""
    if mode == 'names' then
      txt = self.path.segments.objects[seg.segment].name
    elseif mode == 'index' then
      txt = "" .. i
    elseif mode == 'id' then
      txt = seg.segment
    end
    im.Text(prefix .. txt)
    im.NextColumn()
    im.Text(self:raceTime(seg.duration))
    im.NextColumn()
  end
end

function C:inDrawLapTimes(lTimes, im)
  im.Text("Lap " ..(1+lTimes.lap) )
  im.NextColumn()
  im.Text(self:raceTime(lTimes.duration))
  im.NextColumn()
  self:inDrawSegmentTimes(lTimes.segmentTimes, im, "index", (1+lTimes.lap) .. " - ")
end

function C:raceTime(time)
  local minutes = math.floor(time/60)
  local seconds = math.floor(time - minutes*60)
  local millis = (time - minutes*60 - seconds)*1000
  return string.format("%02d:%02d.%03d",minutes, seconds, millis)
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end