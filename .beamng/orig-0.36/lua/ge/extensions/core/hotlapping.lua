-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local logTag = 'hotLapping'
local rootDir = 'gameplay/hotlapping/'
local oldRootDir = 'settings/hotlapping/' -- for backwards compatibility

local editMode = false -- whether the hotlapping is editMode or not (app usage in freeroam)
local closed = false -- once the first lap is completed, the circuit is closed
local started = false -- whether the timer is started or not

local useScenarioTimer = false
local useCustomTimerFunc = nil
local startTime
local totalTime

local pausedStart = 0
local currentPauseTime = 0
local totalPauseTime = 0

local currentLap = 0
local currentCP = 0

local bestLapIndex = -1


local pathData, raceData, markers -- path, race, and markers data
local checkPointCount = 0 -- stores amount of checkPoints
local checkPointId -- stores the latest checkpoint id
local nextCheckPointIndex = 0 -- stores the index of the next checkpoint to be passed


local times = {} -- time data for all checkPoints, starting from start time

local lastRealMillis = 0 -- timer when no updates should be posted

local justPassedCPWithinLap = false
local justLapped = false
local justStarted = false

local radius = -1 -- -1 means auto radius for checkpoints
local forceSendToGui = true
local isBranchingScenario = false
local invisible = false

--------------------------------------------------------------------
-- Starting and stopping of hotlapping,
-- placing, removing and passing markers,
-- starting, pausing and stopping of the race
--------------------------------------------------------------------

-- called when the hotlapping starts
local function startHotlapping()
  useCustomTimerFunc = nil
  editMode = true
  started = false
  closed = false

  checkPointCount = 0
  nextCheckPointIndex = 0
  bestLapIndex = -1
  radius = -1

  table.clear(times)

  local vehicle = getPlayerVehicle(0)
  if not vehicle then
    log('E', logTag, 'No vehicle found; hotlapping mode disabled')
    editMode = false
    return
  end

  pausedStart = 0
  currentPauseTime = 0
  totalPauseTime = 0
  useScenarioTimer = false

  if not pathData then
    pathData = require('/lua/ge/extensions/gameplay/race/path')('Hotlapping')
    M.addCheckPoint() -- start position
  end
end

-- called when the hotlapping stops
local function stopHotlapping()
  M.stopTimer()

  editMode = false
  closed = false
  pathData = nil
  checkPointId = nil

  for _, veh in ipairs(getAllVehiclesByType()) do
    if veh.isHotlapping then
      veh:setDynDataFieldbyName("isHotlapping", 0, "")
    end
  end
end

-- sets up the AI vehicles to race along the path
local function startAi()
  if not raceData or not raceData.aiPath[1] then
    if editMode then
      M.start()
      if started then
        startAi() -- recursively called to check if race is valid
      end
      return
    else
      return
    end
  end

  for _, veh in ipairs(getAllVehiclesByType()) do
    if not veh:isPlayerControlled() and not veh.isTraffic and not veh.isParked then
      local route = deepcopy(raceData.aiPath)
      table.insert(route, route[1])

      veh:queueLuaCommand('ai.driveUsingPath({wpTargetList = '..serialize(route)..', noOfLaps = 1000, avoidCars = "on"})')
      veh:queueLuaCommand('ai.setParameters({turnForceCoef = 4, awarenessForceCoef = 0.15})') -- slightly improves racing

      if not veh.isHotlapping then
        veh:queueLuaCommand('recovery.saveHome()')
        veh:queueLuaCommand('ai.setAggression(0.9)') -- sets aggression once (the user could adjust it as desired later)
        veh:setDynDataFieldbyName("isHotlapping", 0, "true")
      end
    end
  end
end

-- stops the racing AI vehicles
local function stopAi(respawn)
  for _, veh in ipairs(getAllVehiclesByType()) do
    if veh.isHotlapping then
      veh:queueLuaCommand('ai.setMode("stop")')

      if respawn then
        veh:queueLuaCommand('recovery.loadHome()')
      end
    end
  end
end

-- returns true if the path data is valid to use for hotlapping
local function validatePathData()
  if not pathData or not pathData.segments.sorted[1] then return false end

  pathData:classify()
  if editMode and not pathData.config.closed then -- if path was edited, ensures that the config is a closed loop
    local seg = pathData.segments:create()
    local firstId = (pathData.startNode and pathData.startNode > 0) and pathData.startNode or pathData.pathnodes.sorted[1].id
    local lastId = (pathData.endNode and pathData.endNode > 0) and pathData.endNode or pathData.pathnodes.sorted[#pathData.pathnodes.sorted].id
    seg:setFrom(lastId)
    seg:setTo(firstId)

    if not pathData.startNode or pathData.startNode == -1 then
      pathData.startNode = pathData.pathnodes.sorted[1].id
    end
  end

  return true
end

-- starts the scenario_race; gets called when the first checkpoint with index 0 is passed
local function start()
  startTime = os.clock() * 1000
  currentCP = 1
  currentLap = 0
  bestLapIndex = -1
  totalTime = 0
  forceSendToGui = true
  started = true
  justStarted = true

  if editMode then
    if not validatePathData() then
      log('W', logTag, 'Could not start hotlapping: Not enough checkpoint data!')
      M.stopTimer()
      return
    end

    -- initialize the race module
    raceData = require('/lua/ge/extensions/gameplay/race/race')()
    raceData:setPath(pathData)
    raceData.lapCount = 1000
    raceData.useHotlappingApp = true
    raceData.useWaypointAudio = true
    raceData:setVehicleIds({be:getPlayerVehicleID(0)})
    raceData:startRace()

    -- initialize the visual race markers
    markers = require('scenario/race_marker')
    markers.init()
    local wps = {}
    for _, pn in ipairs(pathData.pathnodes.sorted) do
      table.insert(wps, {name = pn.id, pos = pn.pos, radius = pn.radius, normal = pn.hasNormal and pn.normal})
    end
    markers.setupMarkers(wps)
    editMode = false

    Engine.Audio.playOnce('AudioGui', "event:UI_Checkpoint")
  end
end

-- skips the current lap
local function skipLap(keepLapProgress)
  if raceData then
    raceData:skipLap(be:getPlayerVehicleID(0), keepLapProgress)
  end
end

-- stops the timer and other race related logic
local function stopTimer()
  started = false
  nextCheckPointIndex = 0
  table.clear(times)

  if raceData then
    raceData:stopRace()
    editMode = true
    raceData = nil
  end

  if markers then
    markers.onClientEndMission()
    markers = nil
  end

  stopAi()
end

-- returns the best pathnode radius for the given position
local function autoRadius(pos)
  local radius = 5
  local n1, n2, dist = map.findClosestRoad(pos)
  if n1 and n2 and dist <= 30 then
    local mapNodes = map.getMap().nodes
    local p1, p2 = mapNodes[n1].pos, mapNodes[n2].pos
    local xnorm = clamp(pos:xnormOnLine(p1, p2), 0, 1)
    radius = math.max(3, lerp(mapNodes[n1].radius, mapNodes[n2].radius, xnorm) + 1)
  end

  return radius
end

-- adds a checkpoint to the track
local function addCheckPoint(cpPos, cpDir, cpRadius)
  if not editMode then return end
  if not cpPos then -- uses the current player position
    local vehId = be:getPlayerVehicleID(0)
    if vehId ~= -1 then
      cpPos = vec3(be:getObjectOOBBCenterXYZ(vehId))
      cpPos.z = be:getSurfaceHeightBelow(cpPos)
    else
      cpPos = core_camera.getPosition()
    end
  end
  if not cpDir then -- uses the current player direction vector
    local veh = getPlayerVehicle(0)
    if veh and veh:getPosition():squaredDistance(cpPos) <= 25 then -- only gets set if vehicle is roughly within checkpoint (otherwise, no normal)
      cpDir = veh:getDirectionVector()
    end
  end
  if not cpRadius then -- uses the default radius
    if radius == -1 then
      cpRadius = autoRadius(cpPos)
    else
      cpRadius = radius
    end
  end

  local pn = pathData.pathnodes:create()
  pn.pos:set(cpPos)
  pn.radius = cpRadius
  pn:setNormal(cpDir) -- if cpDir is nil, then the pathnode will have no normal
  checkPointCount = checkPointCount + 1

  if checkPointId and not pathData.pathnodes.objects[checkPointId].missing then -- creates a segment
    local seg = pathData.segments:create()
    seg:setFrom(checkPointId)
    seg:setTo(pn.id)
  end

  checkPointId = pn.id -- the latest pathnode id
end

-- removes all checkpoints and clears the array saving them
local function clearAllCP()
  checkPointCount = 0
  checkPointId = nil
  stopAi()
end

-- sets all marker objects as visible or invisible (during active hotlapping)
local function setVisible(value)
  invisible = not value and true or false

  if markers then
    if invisible then
      markers.hide()
      if raceData then raceData.useWaypointAudio = false end
    else
      markers.show()
      if raceData then
        raceData.useWaypointAudio = true

        local state = raceData.states[be:getPlayerVehicleID(0)]
        if state then
        local events = state.events
          local wps = {}
          for _, e in ipairs(state.nextPathnodes) do
            wps[e[1].id] = e[2]
          end
          for _, e in ipairs(state.overNextPathnodes) do
            wps[e[1].id] = 'next'
          end

          markers.setModes(wps)
        end
      end
    end
  end
end

-- gets called when a checkpoint gets passed; takes the time and updates the checkpoint indices
local function onCheckPointPassed(index)
  if index == nextCheckPointIndex then
    local finishedRound = false
    if index == 0 then
      -- lapped!
      currentLap = currentLap + 1
      currentCP = 1
      justLapped = true
    else
      currentCP = currentCP + 1
      justPassedCPWithinLap = true
    end

    nextCheckPointIndex = nextCheckPointIndex + 1

    if closed then
      -- wrap index
      nextCheckPointIndex = nextCheckPointIndex % checkPointCount
    end
  end
end

--------------------------------------------------------------------
-- periodically called functions for rendering stuff,
-- updating the times, sending those to the app,
-- also formatting the times into a format usable by the app
--------------------------------------------------------------------

local vecUp = vec3(0, 0, 1)
local vecA = vec3()
local vecB = vec3()
local colorWhite = ColorF(1, 1, 1, 0.2)
local colorOrange = ColorF(1, 0.5, 0, 0.5)
local function onPreRender(dt, dtSim)
  if editMode then
    if pathData then
      for i, pn in ipairs(pathData.pathnodes.sorted) do
        debugDrawer:drawSphere(pn.pos, pn.radius, colorWhite)

        vecA:set(pn.pos)
        vecA.z = vecA.z + pn.radius + 0.25
        vecB:set(vecA)
        vecB.z = vecB.z + 1000
        debugDrawer:drawCylinder(vecA, vecB, 0.3, i == 1 and colorOrange or colorWhite)

        if pn.normal then
          vecA:set(pn.pos)
          vecB:setScaled2(pn.normal, pn.radius)
          vecB:setAdd2(vecA, vecB)
          debugDrawer:drawSquarePrism(vecA, vecB, Point2F(0.5, pn.radius * 0.25), Point2F(0.5, 0), colorOrange)
        end
      end
    end
  else
    if markers and not invisible then
      markers.render(dt, dtSim)
    end
  end
end

local function onUpdate(dt, dtSim)
  if editMode and pathData then
    if not started then
      local node = pathData.pathnodes.objects[pathData.startNode]
      if node and not node.missing and be:getPlayerVehicleID(0) ~= -1 and pathData.segments.sorted[1] then
        local sqDist = node.pos:squaredDistance(getPlayerVehicle(0):getPosition())
        if sqDist <= square(node.radius) then
          M.start()
        end
      end
    end
  end

  if not be:getEnabled() then
    if pausedStart == 0 then
      pausedStart = os.clock() * 1000
      guihooks.trigger("HotlappingTimerPause")
    end
    currentPauseTime = os.clock() * 1000 - pausedStart
  end

  if started and be:getEnabled() then
    if raceData then
      raceData:onUpdate(dtSim)

      local state = raceData.states[be:getPlayerVehicleID(0)]
      if state and not invisible then
        local events = state.events
        if events then
          if events.rollingStarted or events.pathnodeReached or events.raceStarted or events.lapSkipped then
            local wps = {}
            for _, e in ipairs(state.nextPathnodes) do
              wps[e[1].id] = e[2]
            end
            for _, e in ipairs(state.overNextPathnodes) do
              wps[e[1].id] = 'next'
            end

            markers.setModes(wps)
          end
        end
      end
    end

    if currentPauseTime > 0 then
      totalPauseTime = totalPauseTime + currentPauseTime
      currentPauseTime = 0
      pausedStart = 0
      forceSendToGui = true
      justStarted = true
    end
    if useCustomTimerFunc then
      totalTime = useCustomTimerFunc()
    else
      if not useScenarioTimer or not scenario_scenarios.getScenario().timer  then
        totalTime = (os.clock()*1000 - startTime) - totalPauseTime
      else
        totalTime = scenario_scenarios.getScenario().timer*1000
      end
    end
    M.setTime()
    M.passTimeToGUI()
    justStarted = false
    justPassedCPWithinLap = false
    justLapped = false
  end

  lastRealMillis = os.clock()*1000
end

local function passTimeToGUI()
  local dt = os.clock()*1000 - lastRealMillis
  local info = M.getTimeInfo()
  info.stop = justPassedCPWithinLap
  info.justLapped = justLapped
  info.delta = dt
  info.closed = closed
  info.running = started
  info.justStarted = justStarted
  --dump(info.justLapped)
  if info.stop or info.justLapped or forceSendToGui then
    guihooks.trigger("HotlappingTimer", info)
  end
  forceSendToGui = false
end

local function setEndTime()
  -- adjust data for current lap and cp.
  times[currentLap][currentCP]['endTime'] = totalTime
  times[currentLap][currentCP]['duration'] = times[currentLap][currentCP]['endTime'] - times[currentLap][currentCP]['startTime']
  times[currentLap][currentCP]['current'] = false
  times[currentLap]['endTime'] = totalTime
  times[currentLap]['duration'] = times[currentLap]['endTime'] - times[currentLap]['startTime']

  -- after first lap, and if there is a best lap, calc diff for this lap vs best lap.
  -- also calc lap curation until current cp for this lap and best lap, store diff in cp record.
  if currentLap > 1 and bestLapIndex ~= -1 then
    times[currentLap]['diff'] = times[currentLap]['duration'] - times[bestLapIndex]['duration']
    local bestLapDurationUntilThisCP = times[bestLapIndex][currentCP]['endTime'] - times[bestLapIndex]['startTime']
    local currentLapDurationUntilThisCP = times[currentLap][currentCP]['endTime'] - times[currentLap]['startTime']
    times[currentLap][currentCP]['diff'] = currentLapDurationUntilThisCP - bestLapDurationUntilThisCP
  end

  -- after lapping and at least second lap, without having skipped this or the previous lap,
  -- check if just completed lap is better than the best lap and adjust if needed.
  if currentLap>1 and not times[currentLap]['skipped']  then
    if bestLapIndex == -1 then
      bestLapIndex = currentLap
    elseif times[currentLap]['duration'] < times[bestLapIndex]['duration']  then
      bestLapIndex = currentLap
    end
  end
end

-- saves the time, diffs and so on
local function setTime(ignoreNewLap)
  local act = false

  -- create new lap record if not existing.
  if times[currentLap] == nil then
    times[currentLap] = {}
    times[currentLap]['startTime'] = totalTime
    times[currentLap]['lap'] = currentLap
    act = true
    -- adjust diff from best lap
    if currentLap > 2 and bestLapIndex ~= -1 then
      times[currentLap-1]['diff'] = times[currentLap-1]['duration'] - times[bestLapIndex]['duration']
    end
    --set end time for previous lap, if it was not skipped.
    if currentLap > 1 and not times[currentLap-1]['skipped'] then
      times[currentLap-1]['endTime'] = totalTime
      times[currentLap-1]['duration'] = times[currentLap-1]['endTime'] - times[currentLap-1]['startTime']
    end
  end

  -- create new cp record if not existing.
  if times[currentLap][currentCP] == nil then
    times[currentLap][currentCP] = {}
    times[currentLap][currentCP]['startTime'] = totalTime
    times[currentLap][currentCP]['cp'] = currentCP
    act = true

    -- after changing checkpoint, adjust endTime, duration and diff for previous checkpoint.
    -- figure out which cp and lap to change.
    local lapToChange = currentLap
    local cpToChange = currentCP - 1
    if currentCP == 1 then
      lapToChange = currentLap - 1
      if lapToChange >= 1 then
        cpToChange = #times[lapToChange]
      end
    end
    -- if the lap to change is valid, change endTime and Duration. also adjust cp diff to best lap.
    if lapToChange >= 1 and not times[lapToChange]['skipped'] then
      times[lapToChange][cpToChange]['endTime'] = totalTime
      times[lapToChange][cpToChange]['duration'] = times[lapToChange][cpToChange]['endTime'] - times[lapToChange][cpToChange]['startTime']
      if bestLapIndex ~= -1 then
        times[lapToChange]['diff'] = times[lapToChange]['duration'] - times[bestLapIndex]['duration']
        local bestLapDurationUntilCP = times[bestLapIndex][cpToChange]['endTime'] - times[bestLapIndex]['startTime']
        local currentLapDurationUntilCP = times[lapToChange][cpToChange]['endTime'] - times[lapToChange]['startTime']
        times[lapToChange][cpToChange]['diff'] = currentLapDurationUntilCP - bestLapDurationUntilCP
      end
    end
  end

  -- adjust data for current lap and cp.
  times[currentLap][currentCP]['endTime'] = totalTime
  times[currentLap][currentCP]['duration'] = times[currentLap][currentCP]['endTime'] - times[currentLap][currentCP]['startTime']
  times[currentLap][currentCP]['current'] = true
  times[currentLap]['endTime'] = totalTime
  times[currentLap]['duration'] = times[currentLap]['endTime'] - times[currentLap]['startTime']

  -- only compare laps if not branching
  if not isBranchingScenario then
    -- after first lap, and if there is a best lap, calc diff for this lap vs best lap.
    -- also calc lap curation until current cp for this lap and best lap, store diff in cp record.
    if currentLap > 1 and bestLapIndex ~= -1 then
      times[currentLap]['diff'] = times[currentLap]['duration'] - times[bestLapIndex]['duration']
      local bestLapDurationUntilThisCP = times[bestLapIndex][currentCP]['endTime'] - times[bestLapIndex]['startTime']
      local currentLapDurationUntilThisCP = times[currentLap][currentCP]['endTime'] - times[currentLap]['startTime']
      times[currentLap][currentCP]['diff'] = currentLapDurationUntilThisCP - bestLapDurationUntilThisCP
    end

    -- after lapping and at least second lap, without having skipped this or the previous lap,
    -- check if just completed lap is better than the best lap and adjust if needed.
    if justLapped and currentLap>1 and not times[currentLap]['skipped'] and not times[currentLap-1]['skipped']  then
      if bestLapIndex == -1 then
        bestLapIndex = currentLap-1
      elseif times[currentLap-1]['duration'] < times[bestLapIndex]['duration']  then
        bestLapIndex = currentLap-1
      end
    end
  end
end

local function getTimeInfoRaw()
  return times
end

-- gets the full time info for a certain index
local retNormal = {}
local retDetail = {}
local timeInfoRes = {}
local function getTimeInfo()
  local i = 0
  for lapIndex,lapValue in ipairs(times) do

    -- normal times
    if retNormal[lapIndex] then
      table.clear(retNormal[lapIndex])
    else
      retNormal[lapIndex] = {}
    end
    retNormal[lapIndex].lap = lapIndex
    retNormal[lapIndex].total = M.formatMillis(lapValue['endTime'])
    retNormal[lapIndex].duration = M.formatMillis(lapValue['duration'])
    retNormal[lapIndex].durationMillis = lapValue['duration']
    retNormal[lapIndex].durationStyle = ''
    if lapValue['skipped'] then
      retNormal[lapIndex].durationStyle = retNormal[lapIndex].durationStyle ..'text-decoration:line-through; '
    end
    if lapIndex == bestLapIndex then
      retNormal[lapIndex].durationStyle = retNormal[lapIndex].durationStyle ..'font-weight:bold; '
      retNormal[lapIndex].best = true
    end
    if lapValue['diff'] or lapValue['skipped'] then
      if lapValue['skipped'] then
        retNormal[lapIndex].diff = 'Skipped'
      else
        if not isBranchingScenario then
          if lapIndex == currentLap and justPassedCPWithinLap then
            retNormal[lapIndex].diff = M.formatMillis(lapValue[#lapValue-1]['diff'],true)
            retNormal[lapIndex].diffColor = M.getDiffColor(lapValue[#lapValue-1]['diff'])
          end

          if lapIndex ~= currentLap or not started then
            retNormal[lapIndex].diff = M.formatMillis(lapValue['diff'],true)
            retNormal[lapIndex].diffColor = M.getDiffColor(lapValue['diff'])
          end
        end
      end
    end

    -- detail times
    i = i + 1
    -- first, all sections
    for cpIndex,cpValue in ipairs(times[lapIndex]) do
      if retDetail[i] then
        table.clear(retDetail[i])
      else
        retDetail[i] = {}
      end
      retDetail[i].lap = lapIndex ..'-'.. cpIndex
      retDetail[i].duration = M.formatMillis(cpValue['duration'])
      retDetail[i].durationMillis = cpValue['duration']
      retDetail[i].total = M.formatMillis(cpValue['endTime'])
      retDetail[i].durationStyle = 'text-align:center; '
      retDetail[i].isSection = true
      retDetail[i].isLap = false

      if lapValue['skipped'] then
        retDetail[i].durationStyle = retDetail[i].durationStyle ..'text-decoration:line-through; '
      end
      if lapIndex == bestLapIndex then
        retDetail[i].durationStyle = retDetail[i].durationStyle ..'font-weight:bold; '
      end
      if cpValue['diff'] or lapValue['skipped'] then
        if lapValue['skipped'] then
          retDetail[i].diff = 'Skipped'
        else
          retDetail[i].diff = M.formatMillis(cpValue['diff'], true)
          retDetail[i].diffColor = M.getDiffColor(cpValue['diff'])
        end
      end
      i = i + 1
    end

    -- previous laps. include all sections with diffs, then summary of the lap
    if retDetail[i] then
      table.clear(retDetail[i])
    else
      retDetail[i] = {}
    end
    retDetail[i].lap = lapIndex
    retDetail[i].duration = M.formatMillis(lapValue['duration'])
    retDetail[i].durationStyle = 'text-align:left; '
    retDetail[i].isSection = false
    retDetail[i].isLap = true
    if lapValue['skipped'] then
      retDetail[i].durationStyle = retDetail[i].durationStyle ..'text-decoration:line-through; '
    end
    if lapIndex == bestLapIndex then
      retDetail[i].durationStyle = retDetail[i].durationStyle ..'font-weight:bold; '
    end
    if lapValue['diff'] or lapValue['skipped']  then
      if lapValue['skipped'] then
        retDetail[i].diff = 'Skipped'
      else
        retDetail[i].diff = M.formatMillis(lapValue['diff'], true)
        retDetail[i].diffColor = M.getDiffColor(lapValue['diff'])
      end
    end
  end

  -- Trim retNormal to match times length
  if #retNormal > #times then
    for i = #times + 1, #retNormal do
      retNormal[i] = nil
    end
  end

  -- Trim retDetail to match times length
  if #retDetail > i then
    for j = i + 1, #retDetail do
      retDetail[j] = nil
    end
  end

  table.clear(timeInfoRes)
  timeInfoRes.normal = retNormal
  timeInfoRes.detail = retDetail
  return timeInfoRes
end

-- formats the time given nicely
local function formatMillis( timeInMillis, addSign )
  if timeInMillis == nil then
    return nil
  end

  if addSign then
    if timeInMillis >= 0 then
      return '+' .. M.formatMillis(timeInMillis,false)
    else
      return '-' .. M.formatMillis(-timeInMillis,false)
    end
  else
    timeInMillis = math.floor(timeInMillis+ .5)
    return string.format("%.2d:%.2d.%.3d", (timeInMillis / 1000) / 60, (timeInMillis / 1000) % 60, timeInMillis % 1000)
  end
end

-- gets the diff color of a diff
local function getDiffColor(val)
  if val > 0 then
    return 'red'
  elseif val < 0 then
    return 'green'
  else
    return ''
  end
end

--------------------------------------------------------------------
-- loading and saving of tracks,
-- changing size of the checkpoints
--------------------------------------------------------------------

-- restores the track from the given file
local function load(originalFilename)
  local filePath = rootDir..getCurrentLevelIdentifier()..'/'..originalFilename..'.race.json'
  local success = false

  -- new method, load from race path file
  if FS:fileExists(filePath) then
    local json = jsonReadFile(filePath)
    if json then
      pathData = require('/lua/ge/extensions/gameplay/race/path')('Hotlapping')
      pathData:onDeserialized(json)
      success = true
    end
  else
    -- old method, backwards compatibility
    -- we need to search through the entire old root directory due to a previous bug
    local oldFiles = FS:findFiles(oldRootDir, '*.json', -1, true, false)
    for _, file in ipairs(oldFiles) do
      if string.find(file, originalFilename) then
        local json = jsonReadFile(file)
        if json then
          filePath = file
          pathData = require('/lua/ge/extensions/gameplay/race/path')('Hotlapping')
          --pathData:fromLapConfig(lapConfig, true)

          editMode =  true
          for _, info in ipairs(json) do
            local radius = type(info.size) == 'table' and info.size[1] or info.size
            M.addCheckPoint(vec3(info.position), vec3(info.direction), radius)
          end

          success = true
          break
        end
      end
    end
  end

  if success then
    M.stopTimer()
    M.clearAllCP()
    M.startHotlapping()
    log('I', logTag, 'Loaded hotlap config from file: '..filePath)
    guihooks.trigger('HotlappingSuccessfullyLoaded', originalFilename)
  else
    log('W', logTag, 'Could not load file: '..filePath)
  end
end

-- saves the path data to the given file
local function save(filePath)
  if not validatePathData() then
    log('W', logTag, 'Could not serialize course: Not enough checkpoint data!')
    return
  end

  local date = os.date("*t")
  local now = string.format("%.4d-%.2d-%.2d_%.2d-%.2d-%.2d", date.year, date.month, date.day, date.hour, date.min, date.sec)
  filePath = filePath or rootDir..getCurrentLevelIdentifier()..'/'..now..'.race.json'
  jsonWriteFile(filePath, pathData:onSerialize(), true)
  log('I', logTag, 'Saved hotlapping config to file: '..filePath)
  guihooks.trigger('HotlappingSuccessfullySaved', now)
  M.refreshTracklist()
end

-- renames a file
local function rename(oldName, newName)
  local pre = rootDir..getCurrentLevelIdentifier() ..'/'
  if not FS:fileExists(pre..oldName..'.race.json') then
    log('W', logTag, 'Failed renaming '..oldName..' to '..newName..': File not found')
    return
  end
  FS:renameFile(pre..oldName..'.race.json', pre..newName..'.race.json')
  FS:removeFile(pre..oldName..'.race.json')
end

-- reloads the list of all available tracks, and sends them to the app
local function refreshTracklist()
  local tracks = {}
  local files = FS:findFiles(rootDir..getCurrentLevelIdentifier()..'/', '*.race.json', -1, true, false)
  local oldFiles = FS:findFiles(oldRootDir, '*.json', -1, true, false)
  files = arrayConcat(files, oldFiles)
  for _, file in ipairs(files) do
    local dir, fn, e = path.split(file)
    table.insert(tracks, fn:match('[%w%-_]*'))
  end

  --dump(tracks)
  return tracks
end

-- changes the size of all checkpoints, according to first parameter (+1 / -1)
local function changeRadius(sign)
  if not pathData then return end
  if radius == -1 then radius = 5 end

  if sign > 0 then
    radius = radius + 1
  elseif sign < 0 then
    radius = radius - 1
  else
    radius = -1
  end
  if radius ~= -1 then
    radius = clamp(radius, 1, 15)
  end

  for _, pn in ipairs(pathData.pathnodes.sorted) do
    if radius == -1 then
      pn.radius = autoRadius(pn.pos) -- auto sets radius
    else
      pn.radius = radius
    end
  end
end

local function onVehicleResetted(id)
  if raceData and started and id == be:getPlayerVehicleID(0) and getPlayerVehicle(0):getPosition():squaredDistance(pathData.pathnodes.objects[pathData.startNode].pos) <= 6400 then -- player resetted near start node
    stopAi(true) -- this respawns the ai
  end
end

-- resets the app on level load
local function onClientStartMission()
  M.stopHotlapping()
  guihooks.trigger("HotlappingResetApp")
end

local function onClientEndMission()
  M.stopHotlapping()
end

local function  onExtensionUnloaded()
  M.stopHotlapping()
end

--------------------------------------------------------------------
-- New Race System hooks
--------------------------------------------------------------------

local function newRaceStart(race)
  M.start()
  closed = true
  times = {}
  nextCheckPointIndex = 1
  currentLap = 1
  useCustomTimerFunc = function() return race.time * 1000 end
  checkPointCount = #(race.path.pathnodes)-1
  forceSendToGui = true
  started = true
  isBranchingScenario = race.path.config.branching
  if isBranchingScenario then
    log('I', logTag, 'This race has branches. Lap and Checkpoint comparisons will be disabled.')
  end
end

local function newRacePathnodeReached(state, info)
  if state.complete then -- race ended
    started = false
    totalTime = useCustomTimerFunc()
    M.setEndTime()
    justPassedCPWithinLap = true
    justLapped = true
    M.passTimeToGUI()
  else
    if not state.events.recovered then -- ignore recoveries
      if info.lapped then
        currentLap = currentLap + 1
        currentCP = 1
        justLapped = true
        nextCheckPointIndex = 0
      else
        currentCP = currentCP + 1
        justPassedCPWithinLap = true
      end
      nextCheckPointIndex = nextCheckPointIndex + 1
      totalTime = useCustomTimerFunc()
      M.setTime()
    end
  end
end

local function newRaceLapSkipped(state, info)
  if not times[currentLap] then return end
  times[currentLap]['skipped'] = true
  if state.skippedLap then
    currentCP = 1
    nextCheckPointIndex = 1
  end

  forceSendToGui = true
  totalTime = useCustomTimerFunc()
  M.setTime()
end

M.newRaceStart = newRaceStart
M.newRacePathnodeReached = newRacePathnodeReached
M.newRaceLapSkipped = newRaceLapSkipped

M.onGameStateUpdate = function(...)
  forceSendToGui = true
end

M.onMenuToggled = function(...)
  forceSendToGui = true
end

M.onScreenFadeState = function(...)
  forceSendToGui = true
end

M.newRaceStop = function()
  started = false
end

--------------------------------------------------------------------
-- Old Race scenario hooks
--------------------------------------------------------------------

local function onRaceStart()
  useCustomTimerFunc = nil
  M.start()
  closed = true
  times = {}
  nextCheckPointIndex = 1
  currentLap = 1
  useScenarioTimer = true
  checkPointCount = #(scenario_scenarios.getScenario().lapConfig)
  forceSendToGui = true
  started = true
  isBranchingScenario = scenario_scenarios.getScenario().lapConfigBranches ~= nil
  if isBranchingScenario then
    log('I',logTag,'This race has branches. Lap and Checkpoint comparisons will be disabled.')
  end
end

local function onRaceWaypointReached(wpInfo)
  if wpInfo.vehId ~= be:getPlayerVehicleID(0) then return end

  if not wpInfo.next then -- end raced
    started = false
    totalTime = scenario_scenarios.getScenario().timer*1000
    M.setEndTime()
    justPassedCPWithinLap = true
    justLapped = true

    M.passTimeToGUI()
  else
    if wpInfo.lapDiff and wpInfo.lapDiff == 1 then
        -- lapped!

      currentLap = currentLap+1
      currentCP = 1
      justLapped = true
      nextCheckPointIndex = 0
    else
      currentCP = currentCP + 1
      justPassedCPWithinLap = true
    end
    nextCheckPointIndex = nextCheckPointIndex +1

    totalTime = scenario_scenarios.getScenario().timer*1000

    M.setTime()
  end
end

local function onRaceResult(final)
  local scenario = scenario_scenarios.getScenario()
  started = false
  scenario.detailedTimes = M.getTimeInfo()
end

--------------------------------------------------------------------
-- public interface
--------------------------------------------------------------------

M.startHotlapping = startHotlapping
M.stopHotlapping = stopHotlapping

M.start = start
M.skipLap = skipLap

M.startAi = startAi
M.stopAi = stopAi

M.addCheckPoint = addCheckPoint
M.setVisible = setVisible

M.getTimeInfoRaw = getTimeInfoRaw

M.onPreRender = onPreRender
M.onUpdate = onUpdate
M.setTime  = setTime
M.setEndTime = setEndTime

M.passTimeToGUI = passTimeToGUI
M.getTimeInfo = getTimeInfo
M.formatMillis = formatMillis
M.getDiffColor = getDiffColor

M.clearAllCP = clearAllCP
M.stopTimer = stopTimer

M.load = load
M.save = save
M.rename = rename

M.changeRadius = changeRadius
M.refreshTracklist = refreshTracklist

M.onVehicleResetted = onVehicleResetted
M.onClientStartMission = onClientStartMission
M.onClientEndMission = onClientEndMission
M.onExtensionUnloaded = onExtensionUnloaded

M.onRaceStart = onRaceStart
M.onRaceWaypointReached = onRaceWaypointReached
M.onRaceResult = onRaceResult
M.onCheckPointPassed = onCheckPointPassed

return M
