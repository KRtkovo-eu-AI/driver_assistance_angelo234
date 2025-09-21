-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
local im = ui_imgui
M.dependencies = { 'ui_apps_genericMissionData', 'ui_apps_pointsBar' }

-- debug variables
local isDebug = false
local boolTrace = im.BoolPtr(false)
local traceData = {}
local green = ColorF(0, 1, 0, 0.2)
local white = ColorF(1, 1, 1, 1)
local blackI = ColorI(0, 0, 0, 255)
local endScreenDataDebug = {} -- data to be sent to the end screen

-- Steps variables
local sanitizedStepsData = {}
local deactivateVehiclesFromOtherSteps = false

local currentStepIndex = 0
local currentStepParameters = {} -- what are the finish conditions, the scoring conditions, etc.
local currentStepTimeLeft
local currentStepFinished = false
local isInCountdown = true
local currentStepTargetJbeamsCrashed = {}
local currentStepTargetJbeamsCrashesStarted = {}
local currentStepTargetHasMoved = false
local currStepCrashId = 1
local crashUnfinishedThisStep = 0 -- used to make sure we don't go into the next step if a crash has started but not finished
local everyPlayerCrashes = {} -- ordered by steps

local scenarioFinished = false
local scenarioStarted = false

local markers
local fadeToBlackDuration = 0.5

local delayToNextStepButton = 1
local delayToNextStepTimer = 0

local function resetData()
  currentStepTargetHasMoved = false
  scenarioStarted = false
  isInCountdown = true
  everyPlayerCrashes = {}
  endScreenDataDebug = {}
  markers = nil
  currentStepTargetJbeamsCrashed = {}
  currentStepTargetJbeamsCrashesStarted = {}
  scenarioFinished = false
  currStepCrashId = 1
  currentStepIndex = 0
end
-- returns a list of objects with the dynamic field name, sorted by the trailing number
local function findObjectsWithDynField(dynamicFieldName)
  local objects = {}
  local inorderedObjects = {}
  for objId, _ in pairs(map.objects) do
    local object = getObjectByID(objId)
    for _, name in ipairs(object:getDynamicFields()) do
      if string.find(name, dynamicFieldName) then
        table.insert(objects, object:getId())
      end
    end
  end
  return objects
end

local function freezeVehicleById(vehId, freeze)
  local veh = scenetree.findObjectById(vehId)
  if veh then
    core_vehicleBridge.executeAction(veh,'setFreeze', freeze)
  end
end

local function freezePlayerVehicles(freeze)
  for _, stepData in ipairs(sanitizedStepsData) do
    freezeVehicleById(stepData.plVehId, freeze)
  end
end

local wasCrashCamModeLoaded
local crashCamModeTrackingMode
local oldCrashCamSettingValue
local function loadSecondaryExtensions()
  extensions.load("gameplay_crashTest_crashTestTaskList")
  extensions.load("gameplay_crashTest_crashTestScoring")
  extensions.load("gameplay_crashTest_crashTestBoundaries")
  extensions.load("gameplay_crashTest_crashTestCountdown")
  extensions.load("gameplay_util_damageAssessment")
  extensions.load("gameplay_crashTest_crashTestDamageChecker")

  wasCrashCamModeLoaded = freeroam_crashCamMode ~= nil
  oldCrashCamSettingValue = settings.getValue('enableCrashCam')
  if wasCrashCamModeLoaded then
    crashCamModeTrackingMode = freeroam_crashCamMode.getTrackingMode()
  else
    settings.setValue('enableCrashCam', true)
    extensions.load('freeroam_crashCamMode')
  end
  freeroam_crashCamMode.setTrackingMode(2)
  freeroam_crashCamMode.setForcedEnabled(true)
end

local function unloadSecondaryExtensions()
  extensions.unload("gameplay_crashTest_crashTestTaskList")
  extensions.unload("gameplay_crashTest_crashTestScoring")
  extensions.unload("gameplay_crashTest_crashTestBoundaries")
  extensions.unload("gameplay_crashTest_crashTestCountdown")
  extensions.unload("gameplay_util_damageAssessment")
  extensions.unload("gameplay_crashTest_crashTestDamageChecker")

  if not wasCrashCamModeLoaded then
    extensions.unload('freeroam_crashCamMode')
  else
    freeroam_crashCamMode.setTrackingMode(crashCamModeTrackingMode)
    freeroam_crashCamMode.setForcedEnabled(false)
  end
  settings.setValue('enableCrashCam', oldCrashCamSettingValue)
end

local function getMissionRelativePath(path)
  return gameplay_missions_missions.getMissionById(gameplay_missions_missionManager.getForegroundMissionId()).mgr:getRelativeAbsolutePath(path, true)
end

local function processData(crashTestData, deactivateVehiclesFromOtherSteps_)
  deactivateVehiclesFromOtherSteps = deactivateVehiclesFromOtherSteps_
  sanitizedStepsData = {}
  resetData()
  loadSecondaryExtensions()

  local id = 1
  for _, step in ipairs(crashTestData) do
    local sanitizedStep = {}
    sanitizedStep.id = id
    sanitizedStep.needDirectContact = step.needDirectContact
    sanitizedStep.plVehId = findObjectsWithDynField(step.crashTestVehName)[1]
    sanitizedStep.jbeamTargets = findObjectsWithDynField(step.jbeamTargetName)
    sanitizedStep.targetType = step.targetType
    sanitizedStep.hasStepBoundsFile = step.hasStepBoundsFile
    sanitizedStep.impactLocationSubject = step.impactLocationSubject
    sanitizedStep.isThereImpactLocation = step.isThereImpactLocation
    sanitizedStep.impactLocation = step.impactLocation
    sanitizedStep.stepInstructions = step.stepInstructions
    sanitizedStep.timeToImpact = step.timeToImpact
    sanitizedStep.aiType = step.aiType
    sanitizedStep.scriptAiFile = "scriptAi"..id..".track.json"
    sanitizedStep.staticCrashTransformPos =vec3(step.staticCrashTransformPos[1], step.staticCrashTransformPos[2], step.staticCrashTransformPos[3])
    sanitizedStep.staticCrashTransformScl = vec3(step.staticCrashTransformScl[1], step.staticCrashTransformScl[2], step.staticCrashTransformScl[3])
    sanitizedStep.plTargetDestinationPos = vec3(step.plTargetDestinationPos[1], step.plTargetDestinationPos[2], step.plTargetDestinationPos[3])
    sanitizedStep.plTargetDestinationScl = vec3(step.plTargetDestinationScl[1], step.plTargetDestinationScl[2], step.plTargetDestinationScl[3])
    sanitizedStep.targetVehTargetDestinationPos = vec3(step.targetVehTargetDestinationPos[1], step.targetVehTargetDestinationPos[2], step.targetVehTargetDestinationPos[3])
    sanitizedStep.targetVehTargetDestinationScl = vec3(step.targetVehTargetDestinationScl[1], step.targetVehTargetDestinationScl[2], step.targetVehTargetDestinationScl[3])
    sanitizedStep.isThereDamageAssessment = step.isThereDamageAssessment
    sanitizedStep.damageAmount = step.damageAmount
    sanitizedStep.impactSpeed = step.impactSpeed
    sanitizedStep.throttleValue = step.throttleValue
    sanitizedStep.damageTargets = step.damageTargets
    sanitizedStep.damageCondition = step.damageCondition
    sanitizedStep.maxTime = step.maxTime
    sanitizedStep.objective = step.objective
    table.insert(sanitizedStepsData, sanitizedStep)
    id = id + 1
  end

  gameplay_crashTest_crashTestTaskList.setCrashTestDataAndInit(sanitizedStepsData)

  freezePlayerVehicles(true)
end

local function playScriptAi(vehId, scriptAiFile)
  local scriptAiFilePath, succ = getMissionRelativePath(scriptAiFile)
  if not succ then return end

  scriptAiFile = jsonReadFile(scriptAiFilePath)

  scriptAiFile.recording.loopCount = 0
  scriptAiFile.recording.loopType = "firstOnlyTeleport"

  local veh = getObjectByID(vehId)
  veh:queueLuaCommand('ai.startFollowing(' .. serialize(scriptAiFile.recording) .. ')')
end

local function playAi()
  if currentStepParameters.aiType == "ScriptAi" then
    playScriptAi(currentStepParameters.jbeamTargets[1], currentStepParameters.scriptAiFile)
  elseif currentStepParameters.aiType == "Stuck Throttle" then
    local veh = getObjectByID(currentStepParameters.jbeamTargets[1])
    veh:queueLuaCommand('electrics.values.throttleOverride = ' .. currentStepParameters.throttleValue/100)
  end
end

local function countdownFinishedCallback()
  if currentStepParameters.targetType == "A single Jbeam" then
    playAi()
  end
  freezeVehicleById(currentStepParameters.plVehId, false)
  M.trackVehiclesForCrash()
  isInCountdown = false
end

-- deactivates vehicles from other steps if the option is enabled
local function deactivateVehiclesFromOtherStepsFunc(currentStepIndex_, forceActivate)
  if not deactivateVehiclesFromOtherSteps then return end

  if forceActivate == nil then
    forceActivate = false
  end

  for _, stepData in ipairs(sanitizedStepsData) do
    if stepData.id == currentStepIndex_ or forceActivate then
      for _, vehId in ipairs(stepData.jbeamTargets) do
        getObjectByID(vehId):setActive(1)
      end
      getObjectByID(stepData.plVehId):setActive(1)
    else
      for _, vehId in ipairs(stepData.jbeamTargets) do
        getObjectByID(vehId):setActive(0)
      end
      getObjectByID(stepData.plVehId):setActive(0)
    end
  end
end

local function stopAllAi()
  for _, stepData in ipairs(sanitizedStepsData) do
    if stepData.aiType == "ScriptAi" then
      local veh = getObjectByID(stepData.jbeamTargets[1])
      if veh then
        veh:queueLuaCommand('ai.stopFollowing()')
      end
    elseif stepData.aiType == "Stuck Throttle" then
      local veh = getObjectByID(stepData.jbeamTargets[1])
      if veh then
        veh:queueLuaCommand('electrics.values.throttleOverride = nil')
      end
    end
  end
end

local function reset()
  resetData()
  deactivateVehiclesFromOtherStepsFunc(1)
  gameplay_crashTest_crashTestScoring.reset()
  gameplay_crashTest_crashTestTaskList.reset()
  stopAllAi()
  freezePlayerVehicles(true)
end

local crashDetectionSettings = {
  verticallyUnweighted = true,
  minAccel = 0,
  maxAccel = 40,
  minDamage = 150,
  maxDamage = 40000
}
local function tryAddVehicleToCrashDetection(vehId)
  if gameplay_util_crashDetection.isVehTracked(vehId) then
    return
  end
  gameplay_util_crashDetection.addTrackedVehicleById(vehId, crashDetectionSettings, "crashAnalysis");
end

local function trackVehiclesForCrash()
  for _, stepData in ipairs(sanitizedStepsData) do
    for _, vehId in ipairs(stepData.jbeamTargets) do
      tryAddVehicleToCrashDetection(vehId)
    end
    tryAddVehicleToCrashDetection(stepData.plVehId)
  end
end

local function fadeToBlack(job, callbackOneFunc, callbackTwoFunc)
  if deactivateVehiclesFromOtherSteps and currentStepIndex > 0 then
    ui_fadeScreen.start(fadeToBlackDuration)
    job.sleep(fadeToBlackDuration+0.6)
  end
  callbackOneFunc()
  if deactivateVehiclesFromOtherSteps and currentStepIndex > 1 then
    ui_fadeScreen.stop(fadeToBlackDuration)
    job.sleep(fadeToBlackDuration+0.6)
  end
  callbackTwoFunc()
end

local function callbackOne()
  currentStepTargetJbeamsCrashed = {}
  currentStepIndex = currentStepIndex + 1
  currentStepTargetHasMoved = false
  currStepCrashId = 1
  currentStepFinished = false

  currentStepParameters = sanitizedStepsData[currentStepIndex]
  currentStepTimeLeft = currentStepParameters.maxTime


  gameplay_walk.getInVehicle(scenetree.findObjectById(currentStepParameters.plVehId))
  core_camera.setByName(0, "orbit")

  deactivateVehiclesFromOtherStepsFunc(currentStepIndex)
  gameplay_walk.getInVehicle(scenetree.findObjectById(currentStepParameters.plVehId))
  gameplay_crashTest_crashTestTaskList.onNewCrashTestStep(sanitizedStepsData[currentStepIndex])
  gameplay_crashTest_crashTestBoundaries.onNewCrashTestStep(sanitizedStepsData[currentStepIndex])

  freezeVehicleById(currentStepParameters.plVehId, true)

  isInCountdown = true
end

local function startCurrentStep()
  gameplay_crashTest_crashTestCountdown.startNewCountdown(3, countdownFinishedCallback)
end

local function callbackTwo()
  extensions.hook("onCrashTestShowStepDetails", {currStepIndex = currentStepIndex, stepInstructions = sanitizedStepsData[currentStepIndex].stepInstructions})
end

local function goToNextStep()
  core_jobsystem.create(fadeToBlack, 1, callbackOne, callbackTwo)
end

local function onScenarioFinished(endScreenData)
  endScreenData.scoreData = gameplay_crashTest_crashTestScoring.getTotalScoreData()
  endScreenDataDebug = endScreenData

  traceData = debug.tracesimple()
  extensions.hook("onCrashScenarioFinished", endScreenData)
  M.clearMarkers()
  scenarioFinished = true
end

local function delayedEndStepJob(job)
  job.sleep(delayToNextStepButton)
  local nextStepId = currentStepIndex + 1
  if nextStepId > #sanitizedStepsData then
    nextStepId = -1
  end
  extensions.hook('onCrashTestShowEndStepDetails', {nextStep = nextStepId, currentStepScoreData = gameplay_crashTest_crashTestScoring.getStepScoreData(currentStepIndex)})
end

local function onCurrentStepFailed(data, forceEnd)
  if forceEnd == nil then forceEnd = false end
  if not forceEnd and crashUnfinishedThisStep > 0 then return end

  -- mark current step as failed
  gameplay_crashTest_crashTestTaskList.onCrashTestStepFailed(sanitizedStepsData[currentStepIndex])
  stopAllAi()
  data.accepted = false
  onScenarioFinished(data)
end

local function onCurrentStepFinished()
  if crashUnfinishedThisStep > 0 or currentStepFinished then return end

  stopAllAi()

  local veh = scenetree.findObjectById(currentStepParameters.plVehId)
  core_vehicleBridge.executeAction(veh,'setFreeze', true)

  currentStepFinished = true
  -- mark current step as done
  gameplay_crashTest_crashTestTaskList.finishedCurrentTask(sanitizedStepsData[currentStepIndex])
  gameplay_crashTest_crashTestBoundaries.deactivateBounds()
  gameplay_crashTest_crashTestScoring.calculateStepScore(sanitizedStepsData[currentStepIndex], everyPlayerCrashes[currentStepIndex], currentStepTargetJbeamsCrashed)

  ui_apps_pointsBar.setPoints(gameplay_crashTest_crashTestScoring.getTotalScoreData().totalScore)

  if currentStepIndex == #sanitizedStepsData then
    onScenarioFinished({accepted = true, scoreData = gameplay_crashTest_crashTestScoring.getTotalScoreData()})
  else
    core_jobsystem.create(delayedEndStepJob)
  end
end

local function startScenario()
  ui_apps_pointsBar.setPoints(0)

  goToNextStep()

  M.clearMarkers()

  scenarioStarted = true
end

local function isVehACurrentStepTarget(vehId)
  for _, targetId in ipairs(currentStepParameters.jbeamTargets) do
    if targetId == vehId then
      return true
    end
  end
  return false
end

local function isVehACurrentStepDamageTarget(vehId)
  if (currentStepParameters.damageTargets == "Player" or currentStepParameters.damageTargets == "Player and target(s)") and vehId == currentStepParameters.plVehId then
    return true
  elseif (currentStepParameters.damageTargets == "Target(s)" or currentStepParameters.damageTargets == "Player and target(s)") and isVehACurrentStepTarget(vehId) then
    return true
  end
  return false
end


local function checkIfAllJbeamsCrashed()
  local totalTargets = 0
  local totalCrashed = 0

  for _, vehId in ipairs(currentStepParameters.jbeamTargets) do
    if isVehACurrentStepDamageTarget(vehId) then
      totalTargets = totalTargets + 1
      if currentStepTargetJbeamsCrashed[vehId] then
        totalCrashed = totalCrashed + 1
      end
    end
  end

  return {
    allCrashed = (totalCrashed == totalTargets),
    totalTargets = totalTargets,
    totalCrashed = totalCrashed
  }
end

local function isPointInSphere(point, center, radius)
  return point:distance(center) < radius
end

local function isStaticCrashLocationCorrect()
  return isPointInSphere(everyPlayerCrashes[currentStepIndex][1].crashStartData.plCrashPos, currentStepParameters.staticCrashTransformPos, math.max(currentStepParameters.staticCrashTransformScl.x, currentStepParameters.staticCrashTransformScl.y, currentStepParameters.staticCrashTransformScl.z))
end

local function imguiDebug()
  if im.Begin("Crash Scenario Manager") then

    im.Text("Target info : ")
    if currentStepParameters.targetType == "Static object" then
      im.Text("Target is a static object")
    else
      im.Text("Target is Jbeam :")
      im.Text(dumps(currentStepParameters.jbeamTargets))
    end
    im.Separator()


    im.Text("Scenario finished: " .. tostring(scenarioFinished))
    if scenarioFinished then
      im.Text("End screen data:")
      im.Text(dumps(endScreenDataDebug))
      im.Checkbox("Trace", boolTrace)
      if boolTrace[0] then
        im.Text(traceData)
      end
    end
    im.Text("Crashes:")
    im.Text(dumps(everyPlayerCrashes))
  end
end

local function shouldDrawMarker(vehId)
  local draw = true
  if currentStepParameters.objective == "Simple crash" and currentStepTargetJbeamsCrashesStarted[vehId] then
    draw = false
  end
  return draw
end

local vecOffset = vec3(0, 0, 2)
local function displayTargetMarkers(displayOnlyCurrentStep)
  markers = require('scenario/race_marker')
  markers.init()

  local wps = {}
  local modes = {}
  if displayOnlyCurrentStep then --cherry pick which targets to display
    if currentStepParameters.targetType == "Static object" then
      table.insert(wps, {name = "target", pos = currentStepParameters.staticCrashTransformPos + vecOffset, radius = 1})
      modes["target"] = 'default'
    else
      if currentStepParameters.jbeamTargets then
        for _, vehId in ipairs(currentStepParameters.jbeamTargets) do
          if shouldDrawMarker(vehId) then
            local veh = getObjectByID(vehId)
            if veh and not currentStepTargetJbeamsCrashed[vehId] then
              table.insert(wps, {name = vehId, pos = veh:getPosition() + vecOffset, radius = 1})
              modes[vehId] = 'default'
            end
          end
        end
      end
    end
  else --display all targets
    for _, stepData in ipairs(sanitizedStepsData) do
      if stepData.targetType == "Static object" then
        table.insert(wps, {name = "target", pos = stepData.staticCrashTransformPos + vecOffset, radius = 1})
        modes["target"] = 'default'
      else
        for _, vehId in ipairs(stepData.jbeamTargets) do
          local veh = getObjectByID(vehId)
          if veh then
            table.insert(wps, {name = vehId, pos = veh:getPosition() + vecOffset, radius = 1})
            modes[vehId] = 'default'
          end
        end
      end
    end
  end
  markers.setupMarkers(wps, "overhead")
  markers.setModes(modes)
end

local function displayTargetsTexts()
  if currentStepParameters.aiType == "ScriptAi" or currentStepParameters.aiType == "Stuck Throttle" then
    for _, vehId in ipairs(currentStepParameters.jbeamTargets) do
      if shouldDrawMarker(vehId) then
        local veh = getObjectByID(vehId)
        local plVeh = getObjectByID(currentStepParameters.plVehId)
        if veh and not currentStepTargetJbeamsCrashed[vehId] then
          local distance = veh:getPosition():distance(plVeh:getPosition())
          local textToDraw
          if distance > 10 then -- draw distance if far
            local dist, unit = translateDistance(distance, false)
            textToDraw = math.ceil(dist) .. " " .. unit
          else -- draw speed
            local vel = veh:getVelocity():length()
            local vel, unit = translateVelocity(vel, true)
            textToDraw = math.ceil(vel) .. " " .. unit
          end
          debugDrawer:drawTextAdvanced(veh:getPosition() + vecOffset, textToDraw, white, true, false, blackI)
        end
      end
    end
  end
end

local function displayCrashZoneMarkers()
  if currentStepParameters.objective == "Crash target vehicle into target area" then
    debugDrawer:drawSphere(currentStepParameters.targetVehTargetDestinationPos, currentStepParameters.targetVehTargetDestinationScl.x, green)
  end
end

local function onPreRender(dt, dtSim)
  if markers then
    markers.render(dt, dtSim)
  end
end

local function clearMarkers()
  if markers then
    markers.onClientEndMission()
    markers = nil
  end
end

local function checkAiStopped()
  if currentStepParameters.aiType == "ScriptAi" then -- lose if stopped moving (end of script ai path)
    if #currentStepParameters.jbeamTargets > 0 then
      local veh = getObjectByID(currentStepParameters.jbeamTargets[1])
      if veh then
        local vel = veh:getVelocity():length()
        if vel > 2 then
          currentStepTargetHasMoved = true
        end
        if vel < 0.1 and currentStepTargetHasMoved and not currentStepTargetJbeamsCrashesStarted[currentStepParameters.jbeamTargets[1]] then
          onCurrentStepFailed({reason = "The target vehicle arrived without taking damage"})
        end
      end
    end
  end
end

local function updateDisplayTimer()
  if not currentStepTimeLeft then return end

  local data = {
    title = "missions.missions.general.time",
    category = "cornerTimer_virtual",
    style = "time",
    order = 100,
  }
  data.txt = string.format("%d:%02d", math.floor(currentStepTimeLeft / 60), math.floor(currentStepTimeLeft % 60))
  data.minutes = string.format("%02d", math.floor(currentStepTimeLeft / 60))
  data.seconds = string.format("%02d", math.floor(currentStepTimeLeft % 60))
  data.style = "text"
  ui_apps_genericMissionData.setData(data)
end

local function runTimeLeft(dtSim)
  if isInCountdown or currentStepFinished then return end

  currentStepTimeLeft = currentStepTimeLeft - dtSim
  if currentStepTimeLeft <= 0 then
    onCurrentStepFailed({reason = "Time ran out"}, true)
  end
end

local function onUpdate(dtReal, dtSim)
  if isDebug then
    imguiDebug()
  end

  if not scenarioFinished then
    displayTargetMarkers(scenarioStarted)
    displayCrashZoneMarkers()
    if scenarioStarted then
      checkAiStopped()
      displayTargetsTexts()
      runTimeLeft(dtSim)
      updateDisplayTimer()
    end
  end
end

local function checkStepFinished()
  if scenarioFinished then return end

  if currentStepParameters.objective == "Simple crash" and everyPlayerCrashes[currentStepIndex] and everyPlayerCrashes[currentStepIndex][1] then
    if currentStepParameters.targetType == "Static object" then
      -- the player needs to crash within the target area for crashes with static objects
      if everyPlayerCrashes[currentStepIndex][1].isCrashCompleted and isStaticCrashLocationCorrect() then
        onCurrentStepFinished()
      elseif not everyPlayerCrashes[currentStepIndex][1].isCrashCompleted and not isStaticCrashLocationCorrect() then -- the player crashed outside the target area
        onCurrentStepFailed({reason = "Player crashed outside the target area"})
      end
    elseif everyPlayerCrashes[currentStepIndex][1].isCrashCompleted and (currentStepParameters.targetType == "A single Jbeam" or currentStepParameters.targetType == "Multiple Jbeams") then
      local targetsCrashInfo = checkIfAllJbeamsCrashed()
      if targetsCrashInfo.allCrashed then
        onCurrentStepFinished()
      else
        if not next(everyPlayerCrashes[currentStepIndex][1].crashEndData.jbeamsInvolved) then
          onCurrentStepFailed({reason = "The player didn't crash into any target vehicles"})
        else
          onCurrentStepFailed({reason = "Not all targets crashed", targetCrashInfo = targetsCrashInfo})
        end
      end
    end
  elseif currentStepParameters.objective == "Crash target vehicle into target area" then
  end
end

-- basically check if player crashed into the actual target, if not then mission failed
local function onVehicleCrashStarted(crashStartData)
  if scenarioFinished then return end

  crashUnfinishedThisStep = crashUnfinishedThisStep + 1

  local veh = getObjectByID(crashStartData.vehId)

  if crashStartData.vehId == currentStepParameters.plVehId then
    crashStartData.plCrashPos = veh:getPosition()

    if not everyPlayerCrashes[currentStepIndex] then
      everyPlayerCrashes[currentStepIndex] = {}
    end

    everyPlayerCrashes[currentStepIndex][currStepCrashId] =
    {
      isCrashCompleted = false,
      crashStartData = crashStartData,
      crashEndData = nil,
      stepElapsedTime = currentStepParameters.maxTime - currentStepTimeLeft
    }

    if currentStepParameters.objective == "Simple crash" then -- if the objective is to crash once, freeze the player
      core_vehicleBridge.executeAction(veh,'setFreeze', true)
    end
  else -- if the crash is one of the target vehicles
    currentStepTargetJbeamsCrashesStarted[crashStartData.vehId] = true
    clearMarkers()
    if currentStepParameters.aiType == "ScriptAi" then
      veh:queueLuaCommand('ai.stopFollowing()') -- stop ai from AIing anytime it is involved in a crash
    end
  end
end

local function checkIfPlayerWasInvolved(crashEndData)
  local playerWasInvolved = false
  for vehId, _ in pairs(crashEndData.jbeamsInvolved) do
    if vehId == currentStepParameters.plVehId then
      playerWasInvolved = true
    end
  end
  return playerWasInvolved
end

local function onVehicleCrashEnded(crashEndData)
  if scenarioFinished then return end

  crashUnfinishedThisStep = crashUnfinishedThisStep - 1

  if crashEndData.vehId == currentStepParameters.plVehId then
    crashEndData.impactLocationData = gameplay_util_damageAssessment.getTextualDamageLocations({vehId = crashEndData.vehId})
    everyPlayerCrashes[currentStepIndex][currStepCrashId].crashEndData = crashEndData
    everyPlayerCrashes[currentStepIndex][currStepCrashId].isCrashCompleted = true
    currStepCrashId = currStepCrashId + 1
  else -- is not the player vehicle
    if isVehACurrentStepTarget(crashEndData.vehId) then
      currentStepTargetJbeamsCrashed[crashEndData.vehId] = {damageLocationData = gameplay_util_damageAssessment.getTextualDamageLocations({vehId = crashEndData.vehId})}

      if currentStepParameters.objective == "Crash target vehicle into target area" then
        local targetCrashedAtTheRightPlace = isPointInSphere(crashEndData.startCrashVehPos, currentStepParameters.targetVehTargetDestinationPos, math.max(currentStepParameters.targetVehTargetDestinationScl.x, currentStepParameters.targetVehTargetDestinationScl.y, currentStepParameters.targetVehTargetDestinationScl.z))
        if targetCrashedAtTheRightPlace then
          onCurrentStepFinished()
        else
          onCurrentStepFailed({reason = "Target vehicle crashed outside the target area"})
        end
      elseif currentStepParameters.objective == "Simple crash" then
        if currentStepParameters.targetType == "A single Jbeam" then
          if not checkIfPlayerWasInvolved(crashEndData) then
            onCurrentStepFailed({reason = "The target vehicle was damaged but the player was not involved"})
          end
        elseif currentStepParameters.targetType == "Multiple Jbeams" and currentStepParameters.needDirectContact then
          if not checkIfPlayerWasInvolved(crashEndData) then
            onCurrentStepFailed({reason = "A target vehicle was damaged but the player didn't make direct contact"})
          end
        end
      end
    else
      onCurrentStepFailed({reason = "A non-target vehicle was damaged"})
    end
  end

  checkStepFinished()
end


local function onAnyMissionChanged(status, mission)
  if status == "stopped" and mission.missionType == "crashTest" then
    extensions.unload("gameplay_crashTest_scenarioManager")
  end
end

local function onExtensionUnloaded(extension)
  M.clearMarkers()
  unloadSecondaryExtensions()
  core_camera.setByName(0, "orbit")
end

local function onPlayerOutOfBounds()
  onCurrentStepFailed({reason = "Player exited the control area"})
end

local function activatesAllVehForReset()
  deactivateVehiclesFromOtherStepsFunc(0, true)
end

local function goToNextStepFromUI()
  goToNextStep()
end

local function startStepFromUI()
  startCurrentStep()
end

local function onCrashCamEnded()
  core_camera.setByName(0, "external")
end

M.onUpdate = onUpdate
M.onPreRender = onPreRender
M.onAnyMissionChanged = onAnyMissionChanged
M.onExtensionUnloaded = onExtensionUnloaded
M.onCrashCamEnded = onCrashCamEnded

M.onPlayerOutOfBounds = onPlayerOutOfBounds
M.onVehicleCrashEnded = onVehicleCrashEnded
M.onVehicleCrashStarted = onVehicleCrashStarted

M.startScenario = startScenario
M.goToNextStepFromUI = goToNextStepFromUI
M.startStepFromUI = startStepFromUI

M.processData = processData
M.reset = reset
M.activatesAllVehForReset = activatesAllVehForReset

M.setDebug = function(_debug)
  isDebug = _debug
end

-- INTERNAL
M.clearMarkers = clearMarkers
M.trackVehiclesForCrash = trackVehiclesForCrash
return M