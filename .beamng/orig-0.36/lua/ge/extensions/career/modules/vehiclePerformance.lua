-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain vehId at http://beamng.com/bCDDL-1.1.txt
local M = {}
local testInProgress
local cancelTestRequested
local zoomingCamera = false
local targetFOV = 19 -- default FOV
local zoomSpeed = 5 -- default zoom speed
local vehicleBreakCancelDelay = 3
local activatedBrakingCamera = false
local vehId
local blockedActions = core_input_actionFilter.createActionTemplate({"radialMenuActivate", "bigMap", "walkingMode", "resetPhysics", "nodegrabber", "vehicleTriggers", "setShifterMode"}, actionWhitelist)

local uiParams

local function refuelCar(veh)
  core_vehicleBridge.requestValue(veh,
    function(ret)
      for _, tank in ipairs(ret[1]) do
        core_vehicleBridge.executeAction(veh, 'setEnergyStorageEnergy', tank.name, tank.maxEnergy)
      end
    end,
    'energyStorage'
  )
end

local function enableOtherVehicles(inventoryId, enabled)
  gameplay_traffic.scatterTraffic()
  local vehicles = career_modules_inventory.getVehicles()
  for id, vehicle in pairs(vehicles) do
    if id ~= inventoryId then
      local otherVehId = career_modules_inventory.getVehicleIdFromInventoryId(id)
      if otherVehId then
        local otherVeh = getObjectByID(otherVehId)
        otherVeh:setActive(enabled and 1 or 0)
      end
    end
  end
end

local function checkAndAddPerformanceDataToHistory(inventoryId)
  local vehicle = career_modules_inventory.getVehicles()[inventoryId]
  local oldPerformanceData = vehicle.certificationData
  if oldPerformanceData then
    vehicle.performanceHistory = vehicle.performanceHistory or {}
    table.insert(vehicle.performanceHistory, 1, oldPerformanceData)
    if #vehicle.performanceHistory > 10 then
      table.remove(vehicle.performanceHistory)
    end
  end
end

local function invalidateCertification(inventoryId)
  local vehicle = career_modules_inventory.getVehicles()[inventoryId]
  checkAndAddPerformanceDataToHistory(inventoryId)
  vehicle.certificationData = nil
end

local maxVelocity
local function updateCameraZoom(dtReal)
  if not zoomingCamera then return end

  local vehVelocity = getObjectByID(vehId):getVelocity():length()
  local fov = linearScale(vehVelocity, 2, maxVelocity, 20, 65)
  core_camera.setFOV(vehId, fov)

  if fov == targetFOV then
    zoomingCamera = false -- stop camera zooming
  end
end

local function onUpdate(dtReal)
  updateCameraZoom(dtReal)
end

local function setRelativeCamera(offsetVec3, rotationVec3)
  core_camera.setByName(0, 'relative', false)
  core_camera.setOffset(vehId, offsetVec3)
  core_camera.setRotation(vehId, rotationVec3)
end

local function switchToEndCamera()
  zoomingCamera = false
  log('I', 'vehiclePerformance', 'Out of bounds! Switching to external camera')
  core_camera.setByName(0, 'external', false)
end

local function switchToLaunchCamera()
  local bbHalfAxis0 = vec3(be:getObjectOOBBHalfAxisXYZ(vehId, 0))
  local backOffsetDist = bbHalfAxis0:length() * -3.5
  local offsetX = -2.0 -- move camera left (negative)
  local offsetY = backOffsetDist -- behind vehicle
  local offsetZ = 0.6 -- lower camera height, adjust as needed
  setRelativeCamera(vec3(offsetX, offsetY, offsetZ), vec3(15, 180, 10))
  core_camera.setFOV(vehId, 80)
end

local function switchToBrakingCamera()
  local bbHalfAxis1 = vec3(be:getObjectOOBBHalfAxisXYZ(vehId, 1))
  local offsetDist = bbHalfAxis1:length() * 3
  setRelativeCamera(vec3(-offsetDist, offsetDist, 1), vec3(135, 180, 0))
  targetFOV = 18
  zoomSpeed = 5
  zoomingCamera = true
  maxVelocity = getObjectByID(vehId):getVelocity():length()
  core_camera.setFOV(vehId, 65)
end

local function startDragTest(inventoryId)
  extensions.load('util_stepHandler')
  testInProgress = true
  vehId = nil
  local sequence = {}
  local spawnStarted
  local spawnComplete
  local staticVehicleDataRetrieved
  local recordingDataRetrieved
  local recordingDataRequested
  local closestGarage = career_modules_inventory.getClosestGarage().id
  local playerInitialPos = getPlayerVehicle(0):getPosition()
  local vehicleInitialPos = nil
  local certificationData = {}
  activatedBrakingCamera = false

  core_input_actionFilter.setGroup('certificationBlockedActions', blockedActions)
  core_input_actionFilter.addAction(0, 'certificationBlockedActions', true)

  -- fade to black at start
  table.insert(sequence, util_stepHandler.makeStepFadeToBlack())
  table.insert(sequence, util_stepHandler.makeStepWait(0.2))

  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      if not spawnStarted then
        enableOtherVehicles(inventoryId, false)
        if career_modules_inventory.getVehicleIdFromInventoryId(inventoryId) then
          vehId = career_modules_inventory.getVehicleIdFromInventoryId(inventoryId)
          vehicleInitialPos = getObjectByID(vehId):getPosition()
          return true
        end
        local vehObj = career_modules_inventory.spawnVehicle(inventoryId, nil, function()
          spawnComplete = true
        end)
        vehId = vehObj:getID()
        spawnStarted = true
      end
      return spawnComplete
    end
  ))

  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      career_modules_inventory.enterVehicle(inventoryId)
      refuelCar(getObjectByID(vehId))
      return true
    end
  ))

  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      core_vehicleBridge.requestValue(getObjectByID(vehId), function(ret)
        certificationData = ret
        staticVehicleDataRetrieved = true
      end, "getStaticData")
      return staticVehicleDataRetrieved
    end
  ))

  -- =====================================
  -- Drag Test
  -- =====================================

  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      gameplay_drag_general.loadDragDataForMission("/gameplay/missions/west_coast_usa/dragStripRace/data.vehiclePerformanceDragTest.json")
      local racers = { {id = vehId, isPlayable = false, canBeReseted = false, lane = 1}}
      local dragData = gameplay_drag_general.getData()
      if dragData.racers[vehId] then
        dragData.racers[vehId].canBeReseted = false
      end

      spawn.safeTeleport(getObjectByID(vehId), dragData.strip.lanes[1].waypoints.spawn.transform.pos, dragData.strip.lanes[1].waypoints.spawn.transform.rot, nil, nil, nil, true, false)

      getObjectByID(vehId):queueLuaCommand('damageTracker.registerDamageUpdateCallback(function(damageData, damageDataDelta) obj:queueGameEngineLua("career_modules_vehiclePerformance.onVehicleDamaged(" .. serialize(damageData) .. ", " .. serialize(damageDataDelta) .. ")") end)')

      gameplay_drag_general.setVehicles(racers)
      -- put vehicle (or both vehicles) on the drag strip
      gameplay_drag_general.resetDragRace()
      -- start activity
      gameplay_drag_general.startDragRaceActivity()
      core_paths.playPath(core_paths.loadPath("/levels/west_coast_usa/camPaths/drag_pi_1.camPath.json"), 0)
      return true
    end))

  table.insert(sequence, util_stepHandler.makeStepFadeFromBlack())

  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      M.openMenu(uiParams)
      core_vehicleBridge.executeAction(getObjectByID(vehId), 'startRecording', "power")
      return true
    end
  ))

  local initialCheckPos
  local lastCheckTime
  local STUCK_THRESHOLD = 0.1 -- meters of movement required to not be considered stuck
  local CHECK_INTERVAL = 7 -- seconds between position checks
  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function(step, dtTable)
      if cancelTestRequested then
        gameplay_drag_general.unloadRace()
        return true
      end

      -- Initialize first position check
      if not initialCheckPos then
        local vehicle = getObjectByID(vehId)
        initialCheckPos = vehicle:getPosition()
        lastCheckTime = 0
      end

      -- Check position difference after interval
      if lastCheckTime >= CHECK_INTERVAL then
        local vehicle = getObjectByID(vehId)
        local movement = (vehicle:getPosition() - initialCheckPos):length()
        if movement < STUCK_THRESHOLD then
          log('I', 'vehiclePerformance', 'Vehicle stuck - cancelling test')
          M.cancelTest()
        end
        -- Reset for next check interval
        initialCheckPos = vehicle:getPosition()
        lastCheckTime = 0
      else
        lastCheckTime = lastCheckTime + dtTable.dtSim
      end
      local phases = gameplay_drag_general.getData().racers[vehId].phases
      local lastPhase = phases[#phases]
      if lastPhase and lastPhase.started and not activatedBrakingCamera then
        activatedBrakingCamera = true
        switchToBrakingCamera()
      end
      --wait for drag to be finished
      return gameplay_drag_general.getData() and gameplay_drag_general.getData().isCompleted
    end)
  )

  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      if cancelTestRequested then
        return true
      end
      if not recordingDataRequested then
        recordingDataRequested = true
        core_vehicleBridge.executeAction(getObjectByID(vehId),'stopRecording', "power")
        core_vehicleBridge.requestValue(getObjectByID(vehId), function(ret)
          recordingDataRetrieved = true
          tableMerge(certificationData, ret)
        end, "getRecordingData", "power")
      end

      return recordingDataRetrieved
    end
  ))

  local cancelTestTimer = 0
  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function(step, dtTable)
      recordingDataRetrieved = nil
      recordingDataRequested = nil
      if not cancelTestRequested then
        return true
      end
      cancelTestTimer = cancelTestTimer + dtTable.dtReal
      if cancelTestTimer >= cancelTestRequested then
        util_stepHandler.skipToLastStepOrCallback()
        return true
      end
    end
  ))

  local timerKeys = {"time_60", "time_330", "time_1_8", "time_1000", "time_1_4", "velAt_1_4", "velAt_1_8", "time_0_60", "brakingG" }
  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      local performanceData = {}
      for _, key in ipairs(timerKeys) do
        performanceData[key] = gameplay_drag_general.getData().racers[vehId].timers[key].value
      end

      -- Add current date in milliseconds
      performanceData.timeStamp = os.time() * 1000
      performanceData.mileage = career_modules_valueCalculator.getVehicleMileageById(inventoryId)
      tableMerge(certificationData, performanceData)
      return true
    end)
  )

  -- =====================================
  -- Skidpad Test
  -- =====================================

 --[[  table.insert(sequence, util_stepHandler.makeStepFadeToBlack())
  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      local vehicle = getObjectByID(vehId)
      local spawnPoint = scenetree.findObject("skidpadSpawn")
      spawn.safeTeleport(vehicle, spawnPoint:getPosition(), quat(0,0,1,0) * spawnPoint:getRotation(), nil, nil, nil, true, false)
      return true
    end
  ))

  table.insert(sequence, util_stepHandler.makeStepWait(0.2))

  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      local vehicle = getObjectByID(vehId)

      local config = {
        wpTargetList = {"skidpadWP1", "skidpadWP2", "skidpadWP3", "skidpadWP4", "skidpadWP1"},
        noOfLaps = 2,
        aggression = 1
      }

      -- Make the AI drive the route
      vehicle:queueLuaCommand('ai.driveUsingPath(' .. serialize(config) .. ')')
      core_camera.setByName(0, 'orbit')
      core_camera.resetCamera(0)
      core_vehicleBridge.executeAction(vehicle, 'startRecording', "lateralAcceleration")
      return true
    end
  ))

  table.insert(sequence, util_stepHandler.makeStepFadeFromBlack())

  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      M.openMenu(uiParams)
      return true
    end
  ))

  -- Wait for skidpad test to finish
  local timer = 0
  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function(step, dtTable)
      if cancelTestRequested then
        core_vehicleBridge.executeAction(getObjectByID(vehId),'stopRecording', "lateralAcceleration")
        util_stepHandler.skipToLastStepOrCallback()
        return true
      end

      timer = timer + dtTable.dtSim
      if timer >= 30 then
        if not recordingDataRequested then
          recordingDataRequested = true
          core_vehicleBridge.executeAction(getObjectByID(vehId),'stopRecording', "lateralAcceleration")
          core_vehicleBridge.requestValue(getObjectByID(vehId), function(ret)
            recordingDataRetrieved = true
            tableMerge(certificationData, ret)
          end, "getRecordingData", "lateralAcceleration")
        end
        return recordingDataRetrieved
      end
      return false
    end
  )) ]]

  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      testInProgress = nil

      checkAndAddPerformanceDataToHistory(inventoryId)
      career_modules_inventory.getVehicles()[inventoryId].certificationData = certificationData
      career_modules_inventory.setVehicleDirty(inventoryId)
      return true
    end
  ))

  -- =====================================
  -- Reset everything back to how it was before the test
  -- =====================================

  local resetSequence = {}
  table.insert(resetSequence, util_stepHandler.makeStepFadeToBlack())

  -- remove vehicle
  table.insert(resetSequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      testInProgress = nil
      career_modules_inventory.removeVehicleObject(inventoryId, true)
      zoomingCamera = false
      return true
    end)
  )
  table.insert(resetSequence, util_stepHandler.makeStepWait(0.1))

  -- enable other vehicles
  table.insert(resetSequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      spawnStarted = false
      spawnComplete = false
      enableOtherVehicles(inventoryId, true)
      return true
    end)
  )

  -- spawn vehicle again
  table.insert(resetSequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      if vehicleInitialPos then
        if not spawnStarted then
          local vehObj = career_modules_inventory.spawnVehicle(inventoryId, nil, function()
            spawnComplete = true
          end)
          spawnStarted = true
          vehId = vehObj:getID()
        end
        return spawnComplete
      end
      return true
    end)
  )

  -- teleport vehicle to garage if vehicleInitialPos is set
  table.insert(resetSequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      if vehicleInitialPos then
        local vehObj = getObjectByID(vehId)
        if vehObj then
          freeroam_facilities.teleportToGarage(closestGarage, vehObj, false)
        end
      end
      return true
    end)
  )

  -- teleport player back to the initial position
  table.insert(resetSequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      spawn.safeTeleport(getPlayerVehicle(0), playerInitialPos)
      core_camera.setGlobalCameraByName(nil)
      return true
    end
  ))

  -- fade from black
  table.insert(resetSequence, util_stepHandler.makeStepFadeFromBlack())

  table.insert(resetSequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      M.openMenu(uiParams)
      return true
    end
  ))

  util_stepHandler.startStepSequence(sequence, function()
    cancelTestRequested = nil
    core_jobsystem.create(function(job)
      util_stepHandler.startStepSequence(resetSequence, function()
        core_input_actionFilter.setGroup('certificationBlockedActions', blockedActions)
        core_input_actionFilter.addAction(0, 'certificationBlockedActions', false)
        if career_career.isAutosaveEnabled() then
          career_saveSystem.saveCurrent()
        end
      end)
    end, "resetDelay")
  end)
end

local function getAggregateScoresFromVehId(inventoryId)
  local vehicle = career_modules_inventory.getVehicles()[inventoryId]
  local certificationData = vehicle.certificationData
  if not certificationData then return end
  return extensions.gameplay_vehiclePerformance.getAggregateScores(certificationData)
end

local function getVehicleClass(inventoryId)
  local vehicle = career_modules_inventory.getVehicles()[inventoryId]
  local certificationData = vehicle.certificationData
  if not certificationData then return end
  return extensions.gameplay_vehiclePerformance.getClassFromData(certificationData)
end

local function addScoresToPerformanceData(performanceData)
  performanceData.performanceAggregateScores = extensions.gameplay_vehiclePerformance.getAggregateScores(performanceData)
  performanceData.vehicleClass = extensions.gameplay_vehiclePerformance.getClassFromData(performanceData)
end

local function cancelTest(delay)
  cancelTestRequested = delay or 0
  zoomingCamera = false
  extensions.gameplay_skidpadTest.cancelTest()
end

local function onBeamNGTrigger(data)
  if not testInProgress or not data or not data.triggerName or not data.event then return end
  if vehId ~= data.subjectID then return end
  -- Handle enter events
  if data.event == "enter" then
    if data.triggerName == "dragStageTrigger" then
      core_paths.playPath(core_paths.loadPath("/levels/west_coast_usa/camPaths/drag_pi_2.camPath.json"), 0)
    elseif data.triggerName == "drag60ftTrigger" then
      core_paths.playPath(core_paths.loadPath("/levels/west_coast_usa/camPaths/drag_pi_3.camPath.json"), 0)
    elseif data.triggerName == "dragTestTrigger2" then
      core_paths.playPath(core_paths.loadPath("/levels/west_coast_usa/camPaths/drag_pi_4.camPath.json"), 0)
    elseif data.triggerName == "dragTestTrigger3" then
      core_paths.playPath(core_paths.loadPath("/levels/west_coast_usa/camPaths/drag_pi_5.camPath.json"), 0)
    end
  -- Handle exit events
  elseif data.event == "exit" then -- vehicle has gone out of bounds, switch to external camera
    if data.triggerName == "dragEndCamTrigger" then
      core_jobsystem.create(function(job)
        job.sleep(0.1)
        switchToEndCamera()
      end, "dragEndCamDelay")
    end
  end
end

local function onDragCountdownStarted(vehId, dial)
  if not testInProgress then return end
  core_jobsystem.create(function(job)
    job.sleep(1) -- let staging lights appear for one second before switching to launch camera
    switchToLaunchCamera()
  end, "dragCamDelay")
end

local function onComputerAddFunctions(menuData, computerFunctions)
  for _, vehicleData in ipairs(menuData.vehiclesInGarage) do
    local inventoryId = vehicleData.inventoryId
    local computerFunctionData = {
      id = "performanceIndex",
      label = "Performance Index",
      callback = function()
        M.openMenu({inventoryId = inventoryId, computerId = menuData.computerFacility.id, backUIState = "computer"})
      end,
      order = 40
    }

    -- tutorial active
    if menuData.tutorialPartShoppingActive or menuData.tutorialTuningActive then
      computerFunctionData.disabled = true
      computerFunctionData.reason = career_modules_computer.reasons.tutorialActive
    end

    computerFunctions.vehicleSpecific[inventoryId][computerFunctionData.id] = computerFunctionData
  end
end

local function openMenu(options, startTest)
  uiParams = deepcopy(options)
  uiParams.testInProgress = testInProgress
  if startTest then
    startDragTest(options.inventoryId)
  else
    guihooks.trigger('ChangeState', {
      state = 'vehiclePerformance', params = uiParams
    })
  end
end

local function onVehicleDamaged(damageData, damageDataDelta)
  if not testInProgress then return end
  local engineData = damageDataDelta.engine
  if engineData and (engineData.catastrophicOverTorqueDamage or engineData.engineLockedUp) then
    guihooks.trigger('PerformanceTestMessage', {
      message = 'Engine damaged! Cancelling test...'
    })
    cancelTest(vehicleBreakCancelDelay)
  end
end

local function startDragTestFromOutsideMenu(inventoryId, computerId)
  M.openMenu({inventoryId = inventoryId, computerId = computerId, backUIState = "computer"}, true)
end

M.startDragTest = startDragTest
M.startDragTestFromOutsideMenu = startDragTestFromOutsideMenu
M.invalidateCertification = invalidateCertification
M.isTestInProgress = function() return testInProgress end
M.cancelTest = cancelTest
M.getAggregateScoresFromVehId = getAggregateScoresFromVehId
M.getVehicleClass = getVehicleClass
M.addScoresToPerformanceData = addScoresToPerformanceData
M.openMenu = openMenu

M.onDragCountdownStarted = onDragCountdownStarted
M.onBeamNGTrigger = onBeamNGTrigger
M.onUpdate = onUpdate
M.onComputerAddFunctions = onComputerAddFunctions
M.onVehicleDamaged = onVehicleDamaged

return M
