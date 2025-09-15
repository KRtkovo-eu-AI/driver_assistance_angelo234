-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.dependencies = {}

local imgui = ui_imgui

local timeLimit = 0.5 * 60 -- test time limit in s

local vehicleId
local active

local startPosRot
local id

local testDriveInfo
local offZoneTimeLimit = 20
local currOffZoneTimer
local routeNodes = {} -- if player has to follow a route, this list will contains the positions
local warnings = {
  {2, "Don't go off the testing road"},
  {7,  "Last warning"},
  {12,  "You are done"}
}
local currWarnings = 0

local function rtMessageJob(job)
  local message = job.args[1] or ""
  local delay = job.args[2] or 0

  job.sleep(delay)
  guihooks.trigger('ScenarioRealtimeDisplay', {msg = message})
  Engine.Audio.playOnce('AudioGui','event:>UI>Career>Buy_01')
  job.sleep(3.5)
  guihooks.trigger('ScenarioRealtimeDisplay', {msg = ""})
end

local function showMessage(message)
  local helper = {
    ttl = 5,
    msg = message,
    category = "t"
  }
  guihooks.trigger('Message',helper)
end

local function rtMessage(message, delay)
  delay = delay or 0
  core_jobsystem.create(rtMessageJob, 1, message, delay)
end

local function setActive(value)
  active = value
  guihooks.trigger('testDriveActive',value)
  gameplay_rawPois.clear()
end

local function resetData()
  setActive(false)
end

local function checkTimeLeft(dtSim)
  if testDriveInfo.timeLimit and not gameplay_walk.isWalking() then
    local timeBefore = round(testDriveInfo.timeLimit)
    testDriveInfo.timeLimit = testDriveInfo.timeLimit - dtSim
    if testDriveInfo.timeLimit < 0 then
      return false
    else
      local timeAfter = round(testDriveInfo.timeLimit)
      if timeBefore ~= timeAfter then
        guihooks.trigger('updateTestDriveTimer', timeAfter)
      end
      return true
    end
  end
  return true
end

local function checkAreaLimit()
  if testDriveInfo.areaLimit then
    local inZone = true
    -- get actual sites data
    local path = "levels/west_coast_usa/facilities/"..testDriveInfo.areaLimit..".sites.json"
    if not FS:fileExists(path) then return true end
    local sites = gameplay_sites_sitesManager.loadSites(path, true, true)
    sites:finalizeSites()

    -- check for in zone
    local veh = scenetree.findObjectById(vehicleId)
    if veh then
      local oobb = veh:getSpawnWorldOOBB()
      for i = 0, 8 do
        local test = oobb:getPoint(0)
        local zones = sites:getZonesForPosition(test)
        if #zones == 0 then
          inZone = false
          showMessage("Out of area, canceling test drive.")
        end
      end
    end

    -- draw some visuals
    local red = {1,0,0}
    local white = {1,1,1}
    red[2] = math.abs(math.sin(Engine.Platform.getRuntime()*2))
    red[3] = red[2]
    for _, zone in pairs(sites.zones.objects) do
      zone:drawDebug(nil, inZone and red or white, 2, -0.5, not inZone)
    end
    return inZone
  end
  return true
end

-- returns true if ok
local function checkTestDriveInfo(dtSim)
  if not testDriveInfo then return true end --no rules, therefore ok

  return checkTimeLeft(dtSim) and checkAreaLimit()
end

local function setTestDriveInfo(_testDriveInfo)
  testDriveInfo = _testDriveInfo
  -- if the player has to follow a route, extract the positions from a .race
  if testDriveInfo.route then
    local actualPath = "levels/west_coast_usa/facilities/"..testDriveInfo.route..".race.json"
    if not FS:fileExists(actualPath) then return true end

    local path = require('/lua/ge/extensions/gameplay/race/path')("New Path")
    path:onDeserialized(jsonReadFile(actualPath))
    path:autoConfig()

    local routeNodes = {}
    for i, pn in ipairs(path.config.linearSegments) do
      table.insert(routeNodes, path.pathnodes.objects[pn].pos)
    end
    core_groundMarkers.setPath(routeNodes)
  end
end

-- creates the end parking spot when the player is far enough
local function checkCreateEndParkingSpot()
  if not testDriveInfo.endParkingSpot or testDriveInfo.endParkingSpotCreated then return end
  local veh = map.objects[vehicleId]
  if veh then
    if veh.pos:distance(testDriveInfo.endParkingSpot.pos) > 120 then
      gameplay_rawPois.clear()
      testDriveInfo.endParkingSpotCreated = true
    end
  end
end

local function start(_vehicleId, testDriveInfo)
  setTestDriveInfo(testDriveInfo)
  vehicleId = _vehicleId
  local vehObj = getObjectByID(vehicleId)
  gameplay_walk.getInVehicle(vehObj)
  startPosRot = {pos = vehObj:getPosition(), rot = quat(0,0,1,0) * quat(vehObj:getRefNodeRotation())}

  -- create part condition snapshot
  core_vehicleBridge.executeAction(vehObj, 'createPartConditionSnapshot', "beforeTestDrive")
  core_vehicleBridge.executeAction(vehObj, 'setPartConditionResetSnapshotKey', "beforeTestDrive")

  setActive(true)
  core_vehicleBridge.executeAction(vehObj, 'setFreeze', false)
  extensions.hook('onTestDriveStarted')
  gameplay_rawPois.clear()
  career_career.setAutosaveEnabled(false)
end

local function tpTestDriveVehBackToDealership(vehicle)
  vehicle = vehicle or getObjectByID(vehicleId)
  spawn.safeTeleport(vehicle, startPosRot.pos, startPosRot.rot, nil, nil, nil, nil, false)
end

local function resetDataAfterTestDriveDone()
  core_groundMarkers.resetAll()
  testDriveInfo = nil
  currWarnings = 0
  gameplay_markerInteraction.clearCache()
  id = -1
  vehicleId = nil
end

local function endTestDriveJob(job)
  -- if tp is set, the vehicle should be teleported back to the dealership.
  -- if not set, it should be despawned
  local tp = job.args[1]
  if job.args[1] == nil then tp = true end

  simTimeAuthority.set(0.5)
  setActive(false)

  if tp then
    ui_fadeScreen.start(1)
  end
  job.sleep(1.5)

  --execute actions with the test drive vehicle
  local vehicle = getObjectByID(vehicleId)
  if vehicle then
    if tp then
      tpTestDriveVehBackToDealership(vehicle)

      job.sleep(0.1)-- setWalkingMode needs to wait a little bit after the vehicle is tp, or the player is set to walking mode where the veh was before the tp

      if gameplay_walk.isWalking() then
        gameplay_walk.getInVehicle(vehicle) -- hack
      end

      core_vehicleBridge.executeAction(vehicle,'setIgnitionLevel', 0)
      core_vehicleBridge.executeAction(vehicle, 'setFreeze', true)

      local vehicleData = map.objects[vehicleId]
      if not vehicleData then return end

      local veh = scenetree.findObjectById(vehicleId)
      local oobb = veh:getSpawnWorldOOBB()

      local vehPos = vehicleData.pos - vec3(0,0,1.8)
      local dir = (oobb:getPoint(0) - vehPos)
      dir:normalize()

      local finalPos = oobb:getPoint(0) + (dir * 1.3)
      gameplay_walk.setWalkingMode(true, finalPos, quatFromDir(-dir, vec3(0,0,1)))
    else
      --career_modules_inspectVehicle.showVehicle(nil)
    end
  end

  --reset stuff
  resetDataAfterTestDriveDone()

  simTimeAuthority.set(1)
  if tp then
    ui_fadeScreen.stop(1)
    -- fade screen changes the ui state, so we need to change it back here
    extensions.hook('onTestDriveEndedAfterFade')
  end
  job.sleep(1.5)

end

local function onVehicleRepairedByInsurance(amount)
  rtMessage("Repaired test drive vehicle: -" .. amount, 1.8)
end

local function stop()
  if not active then return end

  core_jobsystem.create(endTestDriveJob, 1, tp)
  career_career.setAutosaveEnabled(true)
end

local function abandonTestDrive()
  if not active then return end

  core_jobsystem.create(function(job)
    setActive(false)
    job.sleep(0.2) -- job is needed to display the ui message
    if testDriveInfo.abandonFees > 0 then --private sales don't have abandon fees
      local logBookLabel = "Didn't return the test drive vehicle."
      local label = string.format("Fee for not returning the test drive vehicle : -%i$", testDriveInfo.abandonFees)
      ui_message(label, 5, 'test1')
      career_modules_payment.pay({money = { amount = testDriveInfo.abandonFees, canBeNegative = true}}, {label = logBookLabel})
    end
    ui_message("You have abandoned the sale.", 5, 'test')
    resetDataAfterTestDriveDone()
  end, 1)

end

local function onUpdate(dtReal, dtSim, dtRaw)
  if not vehicleId then return end
  if active then
    if not checkTestDriveInfo(dtSim) then -- will stop the test drive if the player doesn't comply with the test drive rules
      stop()
    end
    checkCreateEndParkingSpot()
  end
end

local function getTimeLeft()
  return testDriveInfo == nil and nil or testDriveInfo.timeLimit
end

local function isActive()
  return active
end

local function onRecalculatedRoute()
  if not testDriveInfo or not testDriveInfo.route then return end

  currWarnings = currWarnings + 1
  for i, node in pairs(warnings) do
    if node[1] == currWarnings then
      showMessage(node[2])
      if i == #warnings then
        stop()
      end
    end
  end
end

local function formatTestDriveToRawPoi(elements)
  if testDriveInfo == nil or not active or not testDriveInfo.endParkingSpot or not testDriveInfo.endParkingSpotCreated then return end
  id = string.format("testDrive-%s-%s-parkingEnd",testDriveInfo.dealershipName, testDriveInfo.route)
  local eps = testDriveInfo.endParkingSpot
  table.insert(elements,  {
    id = id,
    data = { type = "testDriveEnd", id = id},
    markerInfo = {
      parkingMarker = {path = testDriveInfo.endParkingSpot:getPath(), pos = eps.pos, rot = eps.rot, scl = eps.scl },
      bigmapMarker = {pos = eps.pos, name = "ui.career.testDrive.endTestDrive", description = "ui.career.testDrive.endTestDriveDesc", thumbnail = testDriveInfo.dealershipPreview, previews = {testDriveInfo.dealershipPreview}}
    }
  })
end

-- poi list stuff
local function onGetRawPoiListForLevel(levelIdentifier, elements)
  formatTestDriveToRawPoi(elements)
end

local function onPoiDetailPromptOpening(elemData, promptData)
  if testDriveInfo == nil or not active then return end

  local isTestDrivePoi = false
  for _, elem in ipairs(elemData) do
    if elem.type == "testDriveEnd" then
      isTestDrivePoi = true
    end
  end
  if isTestDrivePoi then
    local ret = {}
    ret.label = "ui.career.testDrive.endTestDrive"
    ret.buttonText = "ui.career.testDrive.stopTestDrive"
    ret.buttonFun = function() M.stop(false) end
    table.insert(promptData, ret)
  end
end

local function onCareerModulesActivated(alreadyInLevel)
  resetData()
end


M.onGetRawPoiListForLevel = onGetRawPoiListForLevel
M.onPoiDetailPromptOpening = onPoiDetailPromptOpening
M.onRecalculatedRoute = onRecalculatedRoute
M.getTimeLeft = getTimeLeft
M.stop = stop
M.abandonTestDrive = abandonTestDrive
M.start = start
M.isActive = isActive
M.formatTestDriveToRawPoi = formatTestDriveToRawPoi
M.resetData = resetData

M.onUpdate = onUpdate
M.onCareerModulesActivated = onCareerModulesActivated
M.onVehicleRepairedByInsurance = onVehicleRepairedByInsurance

return M
