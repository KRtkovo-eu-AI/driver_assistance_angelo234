-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local pow = math.pow
local abs = math.abs
local sqrt = math.sqrt
local im = ui_imgui

local plotHelperUtil
local debug = false

local trackedVehIds = {}

local dtSim

-- General crash settings
local minDamageThreshold = 10
local stopCrashDelay = 1.8

local pointAccelSmoother = newTemporalSmoothing(150, 150)

-- Debug
local accelHistories = {} -- Table to store histories per vehicle
local thresholdHistories = {} -- History array for damage threshold per vehicle
local damageHistories = {} -- History array for damage taken per vehicle
local maxAccelHistory = 50
local debugHistoryTimer = 0
local debugHistorySamplesPerSec = 10
local debugSelectedVehicleIndex = im.IntPtr(0)

local function onCrashStarted(crashStartData)
  extensions.hook("onVehicleCrashStarted", crashStartData)
end

local function onCrashEnded(crashEndData)
  extensions.hook("onVehicleCrashEnded", crashEndData)
end

local function resetCrashData(vehId)
  if not trackedVehIds[vehId] then return end

  trackedVehIds[vehId].totalCrashDamage = 0
  trackedVehIds[vehId].jbeamsInvolvedInWholeCrash = {}
  trackedVehIds[vehId].crashTriggered = false
  trackedVehIds[vehId].lastFewFramesSpeeds = {}
  trackedVehIds[vehId].lastFrameDamageSum = 0
end

local veh

-- make sure that this extension onUpdate is called before your extension onUpdate by adding this to your extension's dependencies: {"gameplay_util_crashDetection"} see drift.lua for an example
local function checkCrashBeginningAndEnd(vehData)
  veh = scenetree.findObjectById(vehData.id)
  if not veh then return end

  local crashData = trackedVehIds[vehData.id]
  if not crashData then return end

  local frontPoint = crashData.accelData.front
  local rearPoint = crashData.accelData.rear

  if not vehData.damage or not frontPoint.accel or not rearPoint.accel then return end

  local finalAccel
  local peakAccel = (frontPoint.peakAccel + rearPoint.peakAccel) / 2
  local peakAccelVerticallyUnweighted = (frontPoint.peakAccelVerticallyUnweighted + rearPoint.peakAccelVerticallyUnweighted) / 2

  if crashData.crashSettings.verticallyUnweighted then
    finalAccel = peakAccelVerticallyUnweighted
  else
    finalAccel = peakAccel
  end

  local crashDamageThreshold = linearScale(finalAccel, crashData.crashSettings.minAccel, crashData.crashSettings.maxAccel, crashData.crashSettings.maxDamage, crashData.crashSettings.minDamage)

  table.insert(crashData.lastFewFramesSpeeds, vehData.vel:length() * 3.6)
  if #crashData.lastFewFramesSpeeds > 4 then
    table.remove(crashData.lastFewFramesSpeeds, 1)
  end

  -- Update debug histories when debug is enabled
  if debug then
    debugHistoryTimer = debugHistoryTimer + dtSim * debugHistorySamplesPerSec
    if debugHistoryTimer > 1 then
      -- Initialize histories for this vehicle if they don't exist
      if not accelHistories[vehData.id] then
        accelHistories[vehData.id] = {}
        thresholdHistories[vehData.id] = {}
        damageHistories[vehData.id] = {}
      end

      -- Add new data points
      table.insert(accelHistories[vehData.id], 1, {peakAccel or 0, peakAccelVerticallyUnweighted or 0})
      table.insert(thresholdHistories[vehData.id], 1, crashDamageThreshold)
      table.insert(damageHistories[vehData.id], 1, crashData.totalCrashDamage)

      -- Remove oldest entries if we exceed max history
      accelHistories[vehData.id][maxAccelHistory] = nil
      thresholdHistories[vehData.id][maxAccelHistory] = nil
      damageHistories[vehData.id][maxAccelHistory] = nil

      debugHistoryTimer = debugHistoryTimer - 1
    end
  end

  local newDamage = vehData.damage - crashData.lastFrameDamage -- old API is more consistent
  local newDamageSum = scenetree.findObjectById(vehData.id):getSectionDamageSum()- crashData.lastFrameDamageSum -- new API is more sensitive/triggers a few frames earlier

  if newDamage > minDamageThreshold then
    crashData.totalCrashDamage = crashData.totalCrashDamage + newDamage
    crashData.currentStopCrashDelayTimer = 0
  else
    crashData.currentStopCrashDelayTimer = crashData.currentStopCrashDelayTimer + dtSim
    if crashData.crashTriggered and (crashData.currentStopCrashDelayTimer >= stopCrashDelay or vehData.vel:length() < crashData.crashSettings.minVelocity) then -- crash ended after a certain delay of no damage
      onCrashEnded({vehId = vehData.id, startCrashVehPos = crashData.startCrashVehPos, endCrashVehPos = vec3(vehData.pos.x, vehData.pos.y, vehData.pos.z), jbeamsInvolved = crashData.jbeamsInvolvedInWholeCrash})
      resetCrashData(vehData.id)
    end
  end

  -- potential new crash
  if newDamageSum > 0 and crashData.potentialCrashImpactSpeed == 0 then
    crashData.potentialCrashImpactSpeed = vehData.vel:length() * 3.6
  end

  -- crash triggered
  if crashData.totalCrashDamage >= crashDamageThreshold and not crashData.crashTriggered then
    crashData.crashTriggered = true
    crashData.startCrashVehPos = vec3(vehData.pos.x, vehData.pos.y, vehData.pos.z)
    onCrashStarted({jbeamsInvolved = vehData.objectCollisions, startCrashVehPos = crashData.startCrashVehPos, vehId = vehData.id, crashSpeed = crashData.potentialCrashImpactSpeed})
  end

  crashData.isCrashing = crashData.crashTriggered

  if crashData.isCrashing then
    for vehId, _ in pairs(vehData.objectCollisions) do
      crashData.jbeamsInvolvedInWholeCrash[vehId] = true
    end
  end

  crashData.lastFrameDamage = vehData.damage
  crashData.lastFrameDamageSum = scenetree.findObjectById(vehData.id):getSectionDamageSum() -- new API is more sensitive/triggers a few frames sooner
end

-- verticallyUnweighted : sometimes we want to make it so that the vertical acceleration is weighted less for crash calculations
local function getPeakAccel(accelVec, verticallyUnweighted)
  if verticallyUnweighted == nil then verticallyUnweighted = false end
  return abs(accelVec.x + accelVec.y + accelVec.z * (verticallyUnweighted and 0.3 or 1))
end

local tempVecCurrVel = vec3()
local tempVecCurrAccel = vec3()
local tempVecPos = vec3()
local function updateAccelData(vehData)
  local crashData = trackedVehIds[vehData.id]
  if not crashData then return end

  -- using two arbitrary points to get a more accurate acceleration reading
  for _, point in pairs(crashData.accelData) do
    tempVecPos:set(vehData.pos + point.offsetFromCenter * vehData.dirVec)
    tempVecCurrVel:set((point.lastFramePos and (tempVecPos - point.lastFramePos) or vec3()) / dtSim)
    tempVecCurrAccel:set((point.lastFrameVel and (tempVecCurrVel - point.lastFrameVel) or vec3()) / dtSim)

    if not point.smootherAcc then
      point.smootherAcc = newTemporalSmoothing(700, 700)
    end

    point.lastFrameVel:set(tempVecCurrVel)
    point.lastFramePos:set(tempVecPos)

    point.vel:set(tempVecCurrVel)

    local accel = abs(point.smootherAcc:getUncapped(sqrt(pow(tempVecCurrAccel.x, 2) + pow(tempVecCurrAccel.y, 2) + pow(tempVecCurrAccel.z, 2)), dtSim))

    point.accel = accel
    point.peakAccel = getPeakAccel(tempVecCurrAccel, false)
    point.peakAccelVerticallyUnweighted = getPeakAccel(tempVecCurrAccel, true)
  end
end

local function drawImGuiWindow()
  if im.Begin("Crash Detection Debug") then
    -- Graph setup
    plotHelperUtil = plotHelperUtil or require('/lua/ge/extensions/editor/util/plotHelperUtil')()

    -- Create vehicle selector combo box
    local vehicleList = {}
    local vehicleListStr = ""
    for vehId, _ in pairs(trackedVehIds) do
      if accelHistories[vehId] then
        local veh = getObjectByID(vehId)
        if veh then
          local vehJbeam = veh.jbeam
          table.insert(vehicleList, vehId)
          vehicleListStr = vehicleListStr .. tostring(vehId) .. " (" .. vehJbeam .. ") | " .. trackedVehIds[vehId].owner .. "\0"
        end
      end
    end

    -- Static variable to store selected vehicle index
    if not debugSelectedVehicleIndex then
      debugSelectedVehicleIndex = im.IntPtr(0)
    end

    if #vehicleList > 0 then
      -- Add vehicle selector and exit debug button on same line
      im.PushItemWidth(im.GetContentRegionAvailWidth() - 200) -- Reserve space for button
      im.Combo2("Select Vehicle", debugSelectedVehicleIndex, vehicleListStr)
      im.PopItemWidth()

      im.SameLine()
      if im.Button("Exit Debug") then
        M.setDebug(false)
      end

      im.Dummy(im.ImVec2(1, 10))

      local selectedVehId = vehicleList[debugSelectedVehicleIndex[0] + 1]
      if selectedVehId and accelHistories[selectedVehId] then

        im.BeginChild1("Acceleration Graph##" .. selectedVehId, im.ImVec2(im.GetContentRegionAvailWidth(), 300), true)
        im.PushStyleColor2(im.Col_Text, im.ImVec4(0.2, 1, 0.1, 1))
        im.TextWrapped("Raw acceleration")
        im.PopStyleColor()
        im.PushStyleColor2(im.Col_Text, im.ImVec4(1, 0.1, 0.1, 1))
        im.TextWrapped("Acceleration vertically unweighted")
        im.PopStyleColor()
          local data = {{}, {}}  -- Two datasets: one for accel, one for peakAccel
          for i = 1, #accelHistories[selectedVehId] do
            data[1][i] = {i, accelHistories[selectedVehId][i][1]}  -- Regular acceleration
            data[2][i] = {i, accelHistories[selectedVehId][i][2]}  -- Horizontal weighted acceleration
          end

          plotHelperUtil:setDataMulti(data)
          plotHelperUtil:scaleToFitData()
          plotHelperUtil:setScale(nil, nil, 0, nil)
          plotHelperUtil:draw(im.GetContentRegionAvailWidth(), im.GetContentRegionAvail().y, 400)
        im.EndChild()

        im.BeginChild1("Crash Threshold Graph##" .. selectedVehId, im.ImVec2(im.GetContentRegionAvailWidth(), 300), true)
          -- Prepare threshold chart data
          local thresholdData = {{}, {}}  -- Two datasets: threshold and damage
          for i = 1, #thresholdHistories[selectedVehId] do
            thresholdData[1][i] = {i, thresholdHistories[selectedVehId][i]}  -- Threshold line
            thresholdData[2][i] = {i, damageHistories[selectedVehId][i]}     -- Damage taken line
          end

          plotHelperUtil:setDataMulti(thresholdData)
          plotHelperUtil:scaleToFitData()
          plotHelperUtil:setScale(nil, nil, 0, nil)
          plotHelperUtil:draw(im.GetContentRegionAvailWidth(), im.GetContentRegionAvail().y, 400)
        im.EndChild()

        im.PushStyleColor2(im.Col_Text, im.ImVec4(0.2, 1, 0.1, 1))
          im.TextWrapped("Damage threshold (decreases with " .. (trackedVehIds[selectedVehId].crashSettings.verticallyUnweighted and "vertically unweighted" or "raw") .. " acceleration)")
        im.PopStyleColor()
        im.PushStyleColor2(im.Col_Text, im.ImVec4(1, 0.1, 0.1, 1))
          im.TextWrapped("Damage taken")
        im.PopStyleColor()
      end

      im.Dummy(im.ImVec2(1, 10))
      im.Text("Is crashing: " .. tostring(trackedVehIds[selectedVehId].isCrashing))
      im.Text("Is using vertically unweighted accel: " .. tostring(trackedVehIds[selectedVehId].crashSettings.verticallyUnweighted))
    else
      im.Text("No vehicles being tracked")
    end
  end
  im.End()
end

local currVehData
local vehIdList = {}
-- make sure that this extension onUpdate is called before your extension onUpdate by adding this to your extension's dependencies: {"gameplay_util_crashDetection"} see drift.lua for an example
local function onUpdate(dtReal, _dtSim)

  dtSim = _dtSim
  if not trackedVehIds then return end

  table.clear(vehIdList)
  for vehId, _ in pairs(trackedVehIds) do
    table.insert(vehIdList, vehId)
  end

  for i = #vehIdList, 1, -1 do
    local vehId = vehIdList[i]
    currVehData = map.objects[vehId]
    if not currVehData then
      trackedVehIds[vehId] = nil -- remove if the vehicle doesn't exist anymore
    else
      updateAccelData(currVehData)
      checkCrashBeginningAndEnd(currVehData)
    end
  end

  if debug then
    drawImGuiWindow()
  end
end

local function addTrackedVehicleById(vehId, crashSettings, owner)
  if vehId == nil then
    log('W', 'crashDetection', "Cannot add a vehicle with a nil ID")
    return
  end

  -- can make the crash detection more sensitive by tweaking these values
  local crashSettings = crashSettings or {}
  crashSettings.minAccel = crashSettings.minAccel or 0
  crashSettings.maxAccel = crashSettings.maxAccel or 100
  crashSettings.minDamage = crashSettings.minDamage or 150
  crashSettings.maxDamage = crashSettings.maxDamage or 40000
  crashSettings.verticallyUnweighted = crashSettings.verticallyUnweighted or false
  crashSettings.minVelocity = crashSettings.minVelocity or 1

  trackedVehIds[vehId] = {
    owner = owner or "unknown",
    crashSettings = crashSettings,
    lastFrameDamage = map.objects[vehId] and map.objects[vehId].damage or 0,
    lastFrameDamageSum = scenetree.findObjectById(vehId):getSectionDamageSum() or 0,
    currentStopCrashDelayTimer = 0,
    potentialCrashImpactSpeed = 0,
    totalCrashDamage = 0,
    jbeamsInvolvedInWholeCrash = {},
    lastFewFramesSpeeds = {},
    crashTriggered = false,
    isCrashing = false,
    accelData  = {
      front = {
        offsetFromCenter = 4, -- arbitrary offset from the center of the vehicle
        lastFrameVel = vec3(),
        lastFramePos = vec3(),
        vel = vec3(),
      },
      rear = {
        offsetFromCenter = -4, -- arbitrary offset from the center of the vehicle
        lastFrameVel = vec3(),
        lastFramePos = vec3(),
        vel = vec3(),
      }
    }
  }
end

local function removeTrackedVehicleById(vehId)
  -- Clean up debug histories when removing vehicle
  if accelHistories[vehId] then
    accelHistories[vehId] = nil
    thresholdHistories[vehId] = nil
    damageHistories[vehId] = nil
  end
  if trackedVehIds[vehId] then
    trackedVehIds[vehId] = nil
  end
end

local function setDebug(_debug)
  debug = _debug
end

local function isVehCrashing(vehId)
  if not trackedVehIds[vehId] then return false end
  return trackedVehIds[vehId].isCrashing
end

local function isVehTracked(vehId)
  return trackedVehIds[vehId] ~= nil
end

local function onSerialize()
  return {
    trackedVehIds = trackedVehIds,
    debug = debug,
  }
end

local function onDeserialized(data)
  trackedVehIds = data.trackedVehIds
  debug = data.debug
end

M.onUpdate = onUpdate
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

M.addTrackedVehicleById = addTrackedVehicleById
M.removeTrackedVehicleById = removeTrackedVehicleById

M.setDebug = setDebug
M.isVehCrashing = isVehCrashing
M.isVehTracked = isVehTracked
M.resetCrashData = resetCrashData
return M
