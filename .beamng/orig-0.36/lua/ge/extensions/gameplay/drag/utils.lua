-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local minDistance = 0.8 --m
local minVelToStop = 15 --m/s
local dialsData = {}
local dialsOffset = 0
local logTag = ""
local damageThreshold = 2000

local dragData = nil

M.onExtensionLoaded = function()
  dragData = gameplay_drag_general.getData()
end

local function getFrontWheelDistanceFromStagePos(racer)
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then
    return nil
  end
  return racer._wheelDistances[racer.frontWheelId]:dot(dragData.strip.lanes[racer.lane].stageToEndNormalized)
end

M.getFrontWheelDistanceFromStagePos = getFrontWheelDistanceFromStagePos

local stagePos
local function calculateDistanceOfAllWheelsFromStagePos(racer)
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return end
  stagePos = dragData.strip.lanes[racer.lane].waypoints.stage.transform.pos
  racer._wheelDistances = {}
  for k, wheel in pairs(racer.wheelsCenter) do
    racer._wheelDistances[k] = racer._wheelDistances[k] or vec3()
    racer._wheelDistances[k]:set(wheel.pos)
    racer._wheelDistances[k]:setSub(stagePos)
    --simpleDebugText3d(string.format("Wheel %d: %0.2fm", k, racer._wheelDistances[k]:dot(dragData.strip.lanes[racer.lane].stageToEndNormalized)), wheel.pos, 0.1, ColorF(1,1,1,1))
  end
end

M.calculateDistanceOfAllWheelsFromStagePos = calculateDistanceOfAllWheelsFromStagePos

local function areFrontWheelsParallelToLine(racer, lineTransform)
  if not lineTransform then
    log('E', logTag, 'Invalid line definition.')
    return false
  end

  local dotProduct = racer.vehDirectionVector:dot(lineTransform.y)
  local tolerance = 0.70
  return math.abs(dotProduct) >= tolerance
end

local function isRacerInsideBoundary(racer)
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then
    return false
  end

  local playerLane = racer.lane
  if not playerLane or not dragData.strip.lanes[playerLane] then
    log('E', logTag, 'No valid lane found for racer: ' .. racer.vehId)
    return false
  end

  local boundary = dragData.strip.lanes[playerLane].boundary.transform
  if not boundary or type(boundary) ~= "table" then
    log('E', logTag, 'No valid boundary found for racer: ' .. racer.vehId)
    return false
  end
  local x, y, z = boundary.rot * vec3(boundary.scl.x,0,0), boundary.rot * vec3(0,boundary.scl.y,0), boundary.rot * vec3(0,0,boundary.scl.z)
  return containsOBB_point(boundary.pos, x, y, z, racer.vehPos )
end
M.isRacerInsideBoundary = isRacerInsideBoundary

local function stopAiVehicle(racer)
  local veh = scenetree.findObjectById(racer.vehId)
  if veh then
    veh:queueLuaCommand('ai.setTarget("drag_stop")')
    veh:queueLuaCommand('ai:scriptStop('..tostring(true)..','..tostring(true)..')')
    --log('I', logTag, 'AI stopped on vehicle: ', racer.vehId)
  end
end

local function headsUpWin()
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return {} end

  local winnerList = {}
  for _, racer in pairs(dragData.racers) do
    table.insert(winnerList, {
      vehId = racer.vehId,
      time = racer.timers.time_1_4.value,
      isPlayable = racer.isPlayable,
      disqualified = racer.isDesqualified
    })
  end

  table.sort(winnerList, function(a, b)
    if a.disqualified and b.disqualified then
      return a.isPlayable
    end
    if a.disqualified then return false end
    if b.disqualified then return true end
    return (a.time) < (b.time)
  end)

  return winnerList
end

local function bracketWin()
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return {} end

  local winnerList = {}
  for _, racer in pairs(dragData.racers) do
    --The bracket race is a bit different so we need to add the reaction time to the time_1_4 timer and create a "Package" that will represent the ET + RT.
    local dialDiff = (racer.timers.time_1_4.value + racer.timers.reactionTime.value) - racer.timers.dial.value
    table.insert(winnerList, {
      vehId = racer.vehId,
      dialDiff = dialDiff,
      isPlayable = racer.isPlayable,
      disqualified = racer.isDesqualified
    })
  end

  local qualifiedRacers = {}
  local disqualifiedRacers = {}

  for _, racer in ipairs(winnerList) do
    if racer.disqualified then
      table.insert(disqualifiedRacers, racer)
    else
      table.insert(qualifiedRacers, racer)
    end
  end

  local negativeRacers = {}
  local positiveRacers = {}

  for _, racer in ipairs(qualifiedRacers) do
    if racer.dialDiff < 0 then
      table.insert(negativeRacers, racer)
    else
      table.insert(positiveRacers, racer)
    end
  end
  table.sort(negativeRacers, function(a, b)
    return math.abs(a.dialDiff) < math.abs(b.dialDiff)
  end)
  table.sort(positiveRacers, function(a, b)
    return a.dialDiff < b.dialDiff
  end)
  table.sort(disqualifiedRacers, function(a, b)
    return a.isPlayable and not b.isPlayable
  end)
  winnerList = {}
  for _, racer in ipairs(positiveRacers) do
    table.insert(winnerList, racer)
  end
  for _, racer in ipairs(negativeRacers) do
    table.insert(winnerList, racer)
  end
  for _, racer in ipairs(disqualifiedRacers) do
    table.insert(winnerList, racer)
  end

  return winnerList
end

local winConditions = {
  ["headsUpRace"] = headsUpWin,
  ["bracketRace"] = bracketWin,
  ["dragPracticeRace"] = headsUpWin
}

local function updateLightState(racer, pairId, lightType, newState, onEvent, offEvent)
  if racer.beamState[pairId][lightType] ~= newState then
    racer.beamState[pairId][lightType] = newState
    extensions.hook(newState and onEvent or offEvent, racer.vehId)
  end
end

local preStageThreshold = -0.178
local stageThreshold = 0
local exitThreshold = 0.4
local function processTreeLights(racer, pairId, distance)
  if math.abs(distance) > exitThreshold then
    updateLightState(racer, pairId, "preStage", false, "preStageLightOn", "preStageLightOff")
    updateLightState(racer, pairId, "stage", false, "stageLightOn", "stageLightOff")
    return
  end

  -- using 0.178 as multipliers
  --prestage goes from -2 to 0
  --stage goe fromt -1 to 1

  -- this checks for positiosn >= -0 and <= 0.178*2
  local preStageNew = distance >= preStageThreshold - 0.178 and distance < preStageThreshold + 0.178
  local stageNew = distance >= stageThreshold - 0.178 and distance < stageThreshold + 0.178

  updateLightState(racer, pairId, "preStage", preStageNew, "preStageLightOn", "preStageLightOff")
  updateLightState(racer, pairId, "stage", stageNew, "stageLightOn", "stageLightOff")
end

local function disqualifyRacer(racer, reason)
  if not racer.isDesqualified then
    racer.isDesqualified = true
    racer.desqualifiedReason = reason
    racer.vehObj:queueLuaCommand('electrics.values.throttleOverride = nil')
    extensions.hook("setDisqualifiedLights", racer.vehId)
  end
end

local disqualificationChecks = {
  stageJump = function(racer)
    local distance = M.getFrontWheelDistanceFromStagePos(racer)
    --print(string.format("Distance: %0.2fm for racer: %d", distance, racer.vehId))
    if distance < -0.178 or distance > 0.178 then
      return true, "missions.dragRace.gameplay.disqualified.jumping"
    end
    return false
  end,

  outOfBounds = function(racer, distance)
    return not isRacerInsideBoundary(racer) or distance < -0.33,
           "missions.dragRace.gameplay.disqualified.outOfLane"
  end,

  stationaryTooLong = function(racer, timer)
    return racer.vehSpeed < 1 and timer > 5,
           "missions.dragRace.gameplay.disqualified.stationaryTooLong"
  end
}

-- -----------------
--PUBLIC FUNCTIONS--
-- -----------------

M.generateWinData =  function()
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return {} end
  local winnerList = winConditions[dragData.dragType]()
  if #winnerList > 0 then
    local winner = winnerList[1]
    extensions.hook("onWinnerLightOn", dragData.racers[winner.vehId].lane)
  end
  return winnerList
end

M.changeRacerPhase = function(racer)
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return end
  local index = racer.currentPhase + 1
  if index > #dragData.phases then
    racer.isFinished = true
    return
  end
  racer.currentPhase = index
  --log("I", logTag, "This is the new phase: " .. racer.phases[racer.currentPhase].name .. " for vehicle: " .. tostring(racer.vehId))
end

M.changeAllPhases = function()
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return end
  for vehId, racer in pairs(dragData.racers) do
    local index = racer.currentPhase + 1
    if index > #dragData.phases then
      racer.isFinished = true
      return
    end
    racer.currentPhase = index
  end
end

local randomDelayTimer = 0
--This is called from the dragRace/display.lua once the christmasTree is finished or any other system determine that the race must start
M.startRaceFromTree = function(vehId)
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return end
  if not dragData.racers[vehId].isPlayable then
    randomDelayTimer = math.random() / 2
  end
  dragData.racers[vehId].phases[dragData.racers[vehId].currentPhase].completed = true
end

M.stage = function(phase, racer, dtSim)
  if not dragData or phase.completed then return end

  local distance = getFrontWheelDistanceFromStagePos(racer)
  if not distance then return end

  guihooks.trigger("updateStageApp", distance)

  local laneData = dragData.strip.lanes[racer.lane]
  local stageWaypoint = laneData.waypoints.stage.waypoint
  local vehObj = scenetree.findObjectById(racer.vehId)
  if not vehObj then return end
  racer.vehObj = vehObj

  if not racer.isPlayable then
    if not phase.started then
      phase.timerOffset = phase.timerOffset + dtSim
      if phase.timerOffset >= phase.startedOffset then
        phase.started = true
        local aiMode = stageWaypoint.mode
        local aiSpeed = stageWaypoint.speed - (distance / 4)
        local aiTarget = laneData.waypoints.endLine.name

        racer.vehObj:queueLuaCommand('ai.setState({mode = "manual"})')
        racer.vehObj:queueLuaCommand('electrics.values.throttleOverride = nil')
        racer.vehObj:queueLuaCommand('ai.setSpeedMode("set")')
        racer.vehObj:queueLuaCommand('ai.setSpeed(0)')
        racer.vehObj:queueLuaCommand('ai.setSpeedMode("' .. aiMode .. '")')
        racer.vehObj:queueLuaCommand('controller.setFreeze(0)')
        racer.vehObj:queueLuaCommand([[
          local nc = controller.getController("nitrousOxideInjection")
          if nc then
            local engine = powertrain.getDevice("mainEngine")
            if engine and engine.nitrousOxideInjection and not engine.nitrousOxideInjection.isArmed then
              nc.toggleActive()
            end
          end
        ]])
        racer.vehObj:queueLuaCommand('ai.setSpeed(' .. aiSpeed .. ')')
        racer.vehObj:queueLuaCommand('ai.setTarget("' .. aiTarget .. '")')
        extensions.hook("stageStarted")
      end
    end

    if distance > -5 and distance < -0.178 then
      racer.vehObj:queueLuaCommand('ai.setSpeedMode("' .. stageWaypoint.mode .. '")')
      racer.vehObj:queueLuaCommand('ai.setSpeed(' .. stageWaypoint.speed .. ')')
    elseif racer.beamState[racer.frontWheelId].stage then
      phase.completed = true
      stopAiVehicle(racer)
      return
    end
  else
    if not phase.started then
      phase.timerOffset = phase.timerOffset + dtSim
      if phase.timerOffset >= phase.startedOffset then
        phase.started = true
        extensions.hook("stageStarted")
      end
    end

    local frontWheelState = racer.beamState[racer.frontWheelId]
    if not frontWheelState.preStage and not frontWheelState.stage then
      phase.completeTimer = 0
    end

    if frontWheelState.stage and areFrontWheelsParallelToLine(racer, laneData.waypoints.stage.transform) then
      phase.completeTimer = (phase.completeTimer or 0) + dtSim
      gameplay_drag_general.clearTimeslip()

      if phase.completeTimer > 1 then
        phase.completed = true
        return
      end
    else
      phase.completeTimer = 0
    end
  end
end

M.countdown = function(phase, racer, dtSim)
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return end

  if phase.completed then return end

  local distance = getFrontWheelDistanceFromStagePos(racer)
  if not distance then return end

  local laneData = dragData.strip.lanes[racer.lane]
  local endLineName = laneData.waypoints.endLine.name
  local vehObj = scenetree.findObjectById(racer.vehId)
  if not vehObj then return end
  racer.vehObj = vehObj

  if not racer.isPlayable then
    if not phase.started then
      phase.timerOffset = phase.timerOffset + dtSim

      if phase.timerOffset >= phase.startedOffset then
        racer.vehObj:queueLuaCommand('ai.setState({mode = "manual"})')
        racer.vehObj:queueLuaCommand('ai.setAggression(1)')
        racer.vehObj:queueLuaCommand('ai.setParameters({understeerThrottleControl = "off", oversteerThrottleControl = "off", throttleTcs = "off"})')
        racer.vehObj:queueLuaCommand([[
          local ts = controller.getController("twoStep")
          if ts then ts.toggleTwoStep() end
        ]])
        racer.vehObj:queueLuaCommand([[
          local tb = controller.getController("transbrake")
          if tb then
            tb.toggleTransbrake()
          else
            controller.setFreeze(1)
          end
        ]])
        -- racer.vehObj:queueLuaCommand('ai.setTarget("' .. endLineName .. '")')
        racer.vehObj:queueLuaCommand('electrics.values.throttleOverride = 0.8')
        extensions.hook("startDragCountdown", racer.vehId, dialsData[racer.vehId])
        phase.started = true
      end
    end
  else
    if not phase.started then
      phase.timerOffset = phase.timerOffset + dtSim

      if phase.timerOffset >= phase.startedOffset then
        extensions.hook("startDragCountdown", racer.vehId, dialsData[racer.vehId])
        phase.started = true
      end
    end
  end

  if not racer.isDesqualified then
    local shouldDQ, reason = disqualificationChecks.stageJump(racer)
    if shouldDQ then
      M.startRaceFromTree(racer.vehId)
      disqualifyRacer(racer, reason)
    end
  end
end

local finishBoundTimer = 0
M.race = function(phase, racer, dtSim)
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return end

  if phase.completed then return end

  local laneData = dragData.strip.lanes[racer.lane]
  local endLine = laneData.waypoints.endLine
  local vehObj = scenetree.findObjectById(racer.vehId)
  if not vehObj then return end
  racer.vehObj = vehObj

  if not racer.isPlayable then
    if not phase.started then
      phase.timerOffset = phase.timerOffset + dtSim
      randomDelayTimer = randomDelayTimer - dtSim
      -- TODO: this only needs to delay the AI starting, not the dragRaceStarted hook!
      if phase.timerOffset >= phase.startedOffset and randomDelayTimer <= 0 then
        local aiSpeed = endLine.waypoint.speed
        local aiMode = endLine.waypoint.mode
        local aiTarget = endLine.name
        racer.vehObj:queueLuaCommand('if electrics.values.jatoInput then electrics.values.jatoInput = 1 end')
        racer.vehObj:queueLuaCommand([[
          local tb = controller.getController("transbrake")
          if tb then
            tb.toggleTransbrake()
          else
            controller.setFreeze(0)
          end
        ]])
        racer.vehObj:queueLuaCommand('electrics.values.throttleOverride = nil')
        racer.vehObj:queueLuaCommand('ai.setSpeed(' .. aiSpeed .. ')')
        racer.vehObj:queueLuaCommand('ai.setSpeedMode("' .. aiMode .. '")')
        racer.vehObj:queueLuaCommand('ai.setTarget("' .. aiTarget .. '")')

        phase.started = true
        extensions.hook("dragRaceStarted", racer.vehId)
      end
    end
  else
    if not phase.started then
      phase.timerOffset = phase.timerOffset + dtSim

      if phase.timerOffset >= phase.startedOffset then
        phase.started = true
        print(string.format("Racer: %d started phase: RACE", racer.vehId, phase.name))
        extensions.hook("dragRaceStarted", racer.vehId)
        finishBoundTimer = 0
      end
    end
  end

  if racer.timers.time_1_4.isSet then
    phase.completed = true
    extensions.hook("dragRaceEndLineReached", racer.vehId)

    -- Check if all racers have completed race phase
    local allRacersFinished = true
    for _, r in pairs(dragData.racers) do
      if not r.timers.time_1_4.isSet then
        allRacersFinished = false
        break
      end
    end
    if allRacersFinished then
      local winnerList = M.generateWinData()
    end

    if not gameplay_missions_missionManager.getForegroundMissionId() then
      gameplay_drag_general.sendTimeslipDataToUi()
    end
    if racer.isPlayable then
      gameplay_drag_general.saveDialTimes()
    end
    return
  end

  if racer.vehSpeed < 1 then
    finishBoundTimer = finishBoundTimer + dtSim
    local shouldDQ, reason = disqualificationChecks.stationaryTooLong(racer, finishBoundTimer)
    if shouldDQ then
      finishBoundTimer = 0
      disqualifyRacer(racer, reason)
    end
  end

  local distance = getFrontWheelDistanceFromStagePos(racer)
  if not racer.isDesqualified then
    local shouldDQ, reason = disqualificationChecks.outOfBounds(racer, distance)
    if shouldDQ then
      disqualifyRacer(racer, reason)
    end
  end
end

M.stop = function(phase, racer, dtSim)
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return end
  if phase.completed then return end

  local laneData = dragData.strip.lanes[racer.lane]
  local spawnWaypoint = laneData.waypoints.spawn.waypoint
  local vehObj = scenetree.findObjectById(racer.vehId)
  if not vehObj then return end
  racer.vehObj = vehObj

  if not racer.isPlayable then
    if not phase.started then
      phase.timerOffset = phase.timerOffset + dtSim

      if phase.timerOffset >= phase.startedOffset then
        racer.vehObj:queueLuaCommand('electrics.values.throttleOverride = nil')
        racer.vehObj:queueLuaCommand('ai.setState({mode = "manual"})')
        racer.vehObj:queueLuaCommand('ai.setAggression(0.3)')
        racer.vehObj:queueLuaCommand('if electrics.values.jatoInput then electrics.values.jatoInput = 0 end')

        racer.vehObj:queueLuaCommand([[
          local nc = controller.getController("nitrousOxideInjection")
          if nc then
            local engine = powertrain.getDevice("mainEngine")
            if engine and engine.nitrousOxideInjection and engine.nitrousOxideInjection.isArmed then
              nc.toggleActive()
            end
          end
        ]])
        racer.vehObj:queueLuaCommand('ai.setSpeedMode("' .. spawnWaypoint.mode .. '")')
        racer.vehObj:queueLuaCommand('ai.setSpeed(' .. (minVelToStop / 2) .. ')')
        racer.vehObj:queueLuaCommand('ai.setTarget("drag_stop")')

        phase.started = true
        extensions.hook("stoppingVehicleDrag", racer.vehId)
      end
    end
  else
    if not phase.started then
      phase.timerOffset = phase.timerOffset + dtSim

      if phase.timerOffset >= phase.startedOffset then
        phase.started = true
        extensions.hook("stoppingVehicleDrag", racer.vehId)
      end
    end
  end

  if racer.vehSpeed <= minVelToStop then
    extensions.hook("dragRaceVehicleStopped", racer.vehId)
    phase.completed = true
  end
end

M.emergencyStop = function(phase, racer, dtSim)
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return end
  if phase.completed then return end

  local laneData = dragData.strip.lanes[racer.lane]
  local spawnWaypoint = laneData.waypoints.spawn.waypoint
  local vehObj = scenetree.findObjectById(racer.vehId)
  if not vehObj then return end
  racer.vehObj = vehObj

  if not racer.isPlayable then
    if not phase.started then
      phase.timerOffset = phase.timerOffset + dtSim

      if phase.timerOffset >= phase.startedOffset then
        racer.vehObj:queueLuaCommand('electrics.values.throttleOverride = nil')
        racer.vehObj:queueLuaCommand('if electrics.values.jatoInput then electrics.values.jatoInput = 0 end')

        racer.vehObj:queueLuaCommand([[
          local nc = controller.getController("nitrousOxideInjection")
          if nc then
            local engine = powertrain.getDevice("mainEngine")
            if engine and engine.nitrousOxideInjection and engine.nitrousOxideInjection.isArmed then
              nc.toggleActive()
            end
          end
        ]])

        racer.vehObj:queueLuaCommand('ai.setSpeedMode("set")')
        racer.vehObj:queueLuaCommand('ai.setSpeed(0)')

        phase.started = true
      end
    end
  else
    if not phase.started then
      phase.timerOffset = phase.timerOffset + dtSim

      if phase.timerOffset >= phase.startedOffset then
        phase.started = true
      end
    end
  end

  if racer.vehSpeed <= 0.01 then
    phase.completed = true
  end
end

M.updateRacer = function(racer)
  if not dragData then
    dragData = gameplay_drag_general.getData()
  end
  if not dragData then return end

  if not racer then return end

  -- Get vehicle object with validation boolean check
  local vehObj = scenetree.findObjectById(racer.vehId)
  if not vehObj then
    racer.isValid = false
    return
  end
  racer.vehObj = vehObj

  if not racer.vehPos then racer.vehPos = vec3() end
  if not racer.vehDirectionVector then racer.vehDirectionVector = vec3() end
  if not racer.vehDirectionVectorUp then racer.vehDirectionVectorUp = vec3() end
  if not racer.vehVelocity then racer.vehVelocity = vec3() end

  racer.vehPos:set(racer.vehObj:getPositionXYZ())
  racer.vehDirectionVector:set(racer.vehObj:getDirectionVectorXYZ())
  racer.vehDirectionVectorUp:set(racer.vehObj:getDirectionVectorUpXYZ())
  racer.vehRot = quatFromDir(racer.vehDirectionVector, racer.vehDirectionVectorUp)

  racer.vehVelocity:set(racer.vehObj:getVelocityXYZ())
  racer.prevSpeed = racer.vehSpeed or 0
  racer.vehSpeed = racer.vehVelocity:length()

  -- Validate lane data exists
  if not dragData.strip or not dragData.strip.lanes or not dragData.strip.lanes[racer.lane] then
    return
  end

  local stageToEndNorm = dragData.strip.lanes[racer.lane].stageToEndNormalized

  if not racer.allWheelsOffsets then
    racer.allWheelsOffsets = {}
  end
  if not racer.wheelsCenter then
    racer.wheelsCenter = {}
  end

  for k, offset in pairs(racer.allWheelsOffsets) do
    if not racer.wheelsCenter[k] then
      racer.wheelsCenter[k] = {
        pos = vec3(), -- Create vec3 only once per wheel
        wheelCountInv = 1/#offset
      }
    end
    racer.wheelsCenter[k].pos:set(0, 0, 0)
    for _, wheel in ipairs(offset) do
      racer.wheelsCenter[k].pos:setAdd(racer.vehRot * wheel)
    end
    racer.wheelsCenter[k].pos:setScaled(racer.wheelsCenter[k].wheelCountInv)
    racer.wheelsCenter[k].pos:setAdd(racer.vehPos)
  end

  calculateDistanceOfAllWheelsFromStagePos(racer)

  if not racer.beamState then
    racer.beamState = {}
  end

  for k, distVec in pairs(racer._wheelDistances or {}) do
    if not racer.beamState[k] then
      racer.beamState[k] = {preStage = false, stage = false}
    end
    processTreeLights(racer, k, distVec:dot(stageToEndNorm))
  end

  racer.currentDistanceFromOrigin = getFrontWheelDistanceFromStagePos(racer)
  racer.previousDistanceFromOrigin = racer.currentDistanceFromOrigin or 0
end

M.setDialsData = function(data)
  local count = #data

  if count == 1 then
    dialsData[data[1].vehId] = data[1].dial
    return
  end

  if count == 2 then
    local dial1, dial2 = data[1].dial, data[2].dial
    local diff = math.abs(dial1 - dial2)

    local offset1, offset2 = (dial1 < dial2) and diff or 0, (dial1 > dial2) and diff or 0

    dialsData[data[1].vehId] = offset1
    dialsData[data[2].vehId] = offset2
    return
  end

  for _, value in ipairs(data) do
    dialsData[value.vehId] = value.dial
  end
end


return M