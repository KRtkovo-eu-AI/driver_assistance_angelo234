-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
M.dependencies = {"gameplay_util_crashDetection", "gameplay_util_groundContact"} -- this needs to be there in order for crashDetection onUpdate to be called before drift onUpdate
local im = ui_imgui
local pow = math.pow
local abs = math.abs
local sqrt = math.sqrt

local redF = ColorF(1,0,0,1)
local blueF = ColorF(0,0.5,1,0.5)
local vehId
local resetFlag = false
local isBeingDebugged
local simulatedDriftData
local dtSim

local profiler = LuaProfiler("drift profiler")
local gc
local driftDebugInfo = {
  default = true,
  canBeChanged = true
}

local accelSmoother = newTemporalSmoothing(0.13,0.13)
local pointAccelSmoother = newTemporalSmoothing(150, 150)

local currDriftCompletedTimer = 0
local currFailPointsCooldown = 0 -- after a crash or spinout, we need to wait for a cooldown before we can start drifting again and earn points
local isDrifting
local isOverSteering
local isChainingDrifts
local isCrashing
local lastFrameIsDrifting
local currDegAngle
local currDegAngleSign
local lastDriftTimer = 0
local resetTimer = 0.1
local currResetTimer = 0
local score
local driftChainActiveData = nil -- data about a whole chain of drifts
local driftActiveData = nil -- data about a single drift
local frontPoint = {
  lastFrameVel = vec3(),
  lastFramePos = vec3(),
  vel = vec3(),
  pos = vec3(),
} -- arbitrary point placed in front of the car to detect a drift
local rearPoint = {
  lastFrameVel = vec3(),
  lastFramePos = vec3(),
  vel = vec3(),
  pos = vec3(),
} -- arbitrary point placed behind the car to detect a drift
local accelerationSmoothed -- negative and positive
local lastFrameKphAirSpeed
local distSinceLastFrame -- drift scoring is calculated with distance, and not time
local kphAirSpeed
local isInTheAir
local isReverseCheesing
local currDriftCompleteTime
local crashed = false
local balanceMode = false -- to better balance mission we use cheats

local crashDetectionSettings = {
  verticallyUnweighted = true,
  minAccel = 0,
  maxAccel = 120,
  minDamage = 150,
  maxDamage = 40000
}

local driftOptions = {
  minAngle = 7,
  maxAngle = 165,
  allowDonut = false,
  allowTightDrifts = true,
  totalDriftAngleModulo = true,
  raycastHeight = 0.5,
  raycastDist = 1.8,
  raycastInwardOffset = 0.650,
  wallDetectionLength = 3,
  baseDriftCompleteTime = 1.2,
  failPointsCooldown = 1.65,
  minAirSpeed = 4,
  maxWasDriftingTime = 4, -- used to detect spinouts
  crashDamageThreshold = 150
}

local showImguiCrashWindow = false

local function reset()
  gameplay_util_crashDetection.resetCrashData(vehId)
  driftActiveData = nil
  driftChainActiveData = nil
  resetFlag = true
  isDrifting = false
  balanceMode = false
end

--------- GARBAGE COLLECTION VARIABLES -----------
local vehPos = vec3()
local velDir = vec3(0,0,0)
local wallMulti
local veh
local vehData
local driftAngleDiff
local dir = vec3()
local pos = vec3()
local hitDist
local hitPos = vec3()
local dirVec = vec3()
local corner_FL = vec3()
local corner_FR = vec3()
local corner_BR = vec3()
local corner_BL = vec3()
local center = vec3()
local frontCenter = vec3()
local rearCenter = vec3()
local up = vec3(0,0,1)
local vecZero = vec3(0,0,0)
local tempVec = vec3(0,0,0)
local avrgRefPointsKphSpeed -- instead of picking the car speed, we use two off-centered points
local frontDot -- calculate the front reference point's velocity dot product with vehicle dir
local centerDot -- calculate the center reference point's velocity dot product with vehicle dir
local rearDot -- calculate the rear reference point's velocity dot product with vehicle dir
--------------------------------------------------

local function newDriftActiveData(vehicleData)
  driftActiveData = {
    driftDuration = 0,
    driftDistance = 0,
    closestWallDistanceFront = 0,
    closestWallDistanceRear = 0,
    currDegAngle = 0,
    direction = nil,
    lastFrameVelDir = vec3(velDir),
    totalDriftAngle = 0,
    totalDriftTime = 0,
    angleVelocity = 0,
    totalDonutsInRow = 0,
    lastPos = vec3(vehicleData.pos),
    avgDriftAngle = 0,
    driftUniformity = 0,
    score = 0, -- earned score during that one drift

    -- these are used for average calculation
    totalAngles = 0,
    totalSpeeds = 0,
    totalPerformanceFactors = 0,
    totalSteps = 0,
  }
end

local function newDriftChainActiveData()
  driftChainActiveData = {
    totalDriftDistance = 0,
    totalDriftTime = 0,
    rightDrifts = 0,
    leftDrifts = 0,
    chainedDrifts = 0,
    currentCircleDrift = { -- 360 drifts
      totalAngle = 0
    }
  }
end

-- Debug
local accelHistory = {}
local thresholdHistory = {} -- History array for damage threshold
local damageHistory = {} -- New history array for damage taken
local maxAccelHistory = 50
local debugHistoryTimer = 0
local debugHistorySamplesPerSec = 10

-- Function to calculate yaw from a vector
local function calculateYaw(vector)
  local normalized = vector:normalized()
  return math.atan2(normalized.y, normalized.x)
end

-- Function to calculate the yaw difference
local function yawDifference(vec1, vec2)
  local yaw1 = calculateYaw(vec1)
  local yaw2 = calculateYaw(vec2)
  local difference = yaw2 - yaw1

  -- Normalize the difference to be within -π to π
  while difference > math.pi do
      difference = difference - 2 * math.pi
  end
  while difference < -math.pi do
      difference = difference + 2 * math.pi
  end

  return difference
end

local velocityTip = vec3(0,0,0)
local function calculateDriftAngle(vehData)
  dirVec:set(vehData.dirVec)
  tempVec:set(vehData.vel)
  tempVec:normalize()
  dirVec:normalize()

  currDegAngle = math.abs(yawDifference(tempVec, dirVec) * (180 / math.pi))
  currDegAngleSign = vehData.vel:dot(up:cross(dirVec))

  if simulatedDriftData then currDegAngle = simulatedDriftData.currDegAngle end

  if isBeingDebugged then
    debugDrawer:drawTextAdvanced(center, string.format("Smoothen accel : %0.2f", accelerationSmoothed or 0), ColorF(1,1,1,1), true, false, ColorI(0,0,0,255))

    debugDrawer:drawLine(center, center + dirVec, ColorF(1,0,0,1))
    velocityTip:set(center + vehData.vel:normalized())
    debugDrawer:drawLine(center, velocityTip, ColorF(1,0.3,0,1))
    debugDrawer:drawTextAdvanced(center, string.format("Yaw : %d ° (min req:%d)", currDegAngle, driftOptions.minAngle), ColorF(1,1,1,1), true, false, ColorI(0,0,0,255))
  end
end

local halfAxis0 = vec3()
local halfAxis1 = vec3()
local halfAxis2 = vec3()
local oobbCenter = vec3()
local raycastHeightVec = vec3(0,0,driftOptions.raycastHeight)
local function calculateVehCenterAndWheels()
  halfAxis0:set(be:getObjectOOBBHalfAxisXYZ(vehId, 0))
  halfAxis1:set(be:getObjectOOBBHalfAxisXYZ(vehId, 1))
  halfAxis2:set(be:getObjectOOBBHalfAxisXYZ(vehId, 2))
  oobbCenter:set(be:getObjectOOBBCenterXYZ(vehId))
  corner_FL:set(-push3(halfAxis1)+push3(oobbCenter)+push3(halfAxis0)-push3(halfAxis2)+push3(raycastHeightVec))
  corner_FR:set(-push3(halfAxis1)+push3(oobbCenter)-push3(halfAxis0)-push3(halfAxis2)+push3(raycastHeightVec))
  corner_BR:set(push3(halfAxis1)+push3(oobbCenter)+push3(halfAxis0)-push3(halfAxis2)+push3(raycastHeightVec))
  corner_BL:set(push3(halfAxis1)+push3(oobbCenter)-push3(halfAxis0)-push3(halfAxis2)+push3(raycastHeightVec))
  center:set(
    (corner_FL.x + corner_FR.x + corner_BL.x + corner_BR.x) / 4,
    (corner_FL.y + corner_FR.y + corner_BL.y + corner_BR.y) / 4,
    (corner_FL.z + corner_FR.z + corner_BL.z + corner_BR.z) / 4
  )

  frontCenter:set(
    (corner_FL.x + corner_FR.x + center.x) / 3,
    (corner_FL.y + corner_FR.y + center.y) / 3,
    (corner_FL.z + corner_FR.z + center.z) / 3
  )
  rearCenter:set(
    (corner_BL.x + corner_BR.x + center.x) / 3,
    (corner_BL.y + corner_BR.y + center.y) / 3,
    (corner_BL.z + corner_BR.z + center.z) / 3
  )
end

local scanners = {
  front = {
    limitStart = corner_FR,
    limitEnd = corner_FL,
    startPoint = frontCenter,
    currOffTimer = 0,
    lockLerp = 0,
    lockDist = math.huge,
    value = "closestWallDistanceFront"
  },
  rear = {
    limitStart = corner_BR,
    limitEnd = corner_BL,
    startPoint = rearCenter,
    currOffTimer = 0,
    lockLerp = 0,
    lockDist = math.huge,
    value = "closestWallDistanceRear"
  },
}

local random = 0.5
local tempVec2 = vec3()
local function calculateDistWall()
  random = getBlueNoise1d(random)
  for _, scannerData in pairs(scanners) do

    -- these 3 lines are a no garbage lerp
    tempVec:setSub2(scannerData.limitEnd, scannerData.limitStart)
    tempVec:setScaled(random)
    tempVec:setAdd(scannerData.limitStart)

    tempVec2:setSub2(tempVec, scannerData.startPoint)
    local scanDist = castRayStatic(scannerData.startPoint, tempVec2, driftOptions.wallDetectionLength)


    if isBeingDebugged then
      debugDrawer:drawLine(scannerData.startPoint, scannerData.startPoint + (tempVec2):normalized() * scanDist, blueF)
      debugDrawer:drawSphere(scannerData.startPoint + (tempVec2):normalized() * scanDist, 0.1, blueF)
    end

    if scanDist < scannerData.lockDist then
      scannerData.lockLerp = random
    end

    if scannerData.lockDist < driftOptions.wallDetectionLength or scanDist < driftOptions.wallDetectionLength then
      scannerData.lockDist = castRayStatic(scannerData.startPoint, tempVec2, driftOptions.wallDetectionLength)
      if isBeingDebugged then
        debugDrawer:drawLine(scannerData.startPoint, scannerData.startPoint + (tempVec2):normalized() * scannerData.lockDist, redF)
        debugDrawer:drawSphere(scannerData.startPoint + (tempVec2):normalized() * scannerData.lockDist, 0.15, redF)
      end
    end

    if simulatedDriftData and simulatedDriftData.wallDistance then
      driftActiveData[scannerData.value] = simulatedDriftData.wallDistance
    else
      driftActiveData[scannerData.value] = scannerData.lockDist
    end
  end
end

local function driftFailed()
  currFailPointsCooldown = driftOptions.failPointsCooldown
  driftChainActiveData = nil
  driftActiveData = nil
end

local triggerCrashDelay = 0.3
local triggerCrashDelayTimer = 0
local function checkForCrash()

  if simulatedDriftData then
    if simulatedDriftData.isCrashing then
      crashed = true
    end
  end


  if crashed then
    triggerCrashDelayTimer = triggerCrashDelayTimer + dtSim
    if triggerCrashDelayTimer >= triggerCrashDelay then
      if score.cachedScore > 0 then
        driftFailed()
        extensions.hook("onDriftCrash")
        crashed = false
      end
    end
  end

  isCrashing = gameplay_util_crashDetection.isVehCrashing(vehId)
end

local function detectCircleDrift()
  if isChainingDrifts then
    if isDrifting then
      driftChainActiveData.currentCircleDrift.totalAngle = driftChainActiveData.currentCircleDrift.totalAngle + driftAngleDiff
    end
  end
end

local minSpeedAllowed = 1.5
local minAllowedAngle = 110
local stopSpinoutCheck = false
local function detectSpinout()
  if currFailPointsCooldown > 0 or isCrashing then return end -- to avoid the message "drift failed : spinout" to appear before crashing
  if kphAirSpeed < minSpeedAllowed then
    if currDegAngle > minAllowedAngle and not stopSpinoutCheck then
      driftFailed()
      extensions.hook("onDriftSpinout")
    end
    if lastDriftTimer < driftOptions.maxWasDriftingTime then
      stopSpinoutCheck = true -- at low speed, "currDegAngle" goes crazy, so we check only once otherwise the player will always "spinout"
    end
  end
end

-- all for gc
local tempVecCurrVel = vec3()
local tempVecPos = vec3()
local tempVecPosUp = vec3()
local tempVecPosUp2 = vec3()
local tempNormalizedVel = vec3()
local tempColor = ColorF(1,0.3,1,0.3)
local function updateReferencePointData(point)
  tempVecPos:set(point.pos)

  if isBeingDebugged then
    tempVecPosUp:setAdd2(point.pos, up)
    debugDrawer:drawSphere(tempVecPosUp, 0.2, tempColor)
    if point.vel then
      tempNormalizedVel:set(point.vel)
      tempNormalizedVel:normalize()
      tempVecPosUp2:setAdd2(tempVecPosUp, tempNormalizedVel)
      debugDrawer:drawLineInstance(tempVecPosUp, tempVecPosUp2, 3, tempColor)
    end
  end


  if point.lastFramePos then
    tempVecCurrVel:setSub2(tempVecPos, point.lastFramePos)
    tempVecCurrVel:setScaled(1/dtSim)
  else
    tempVecCurrVel:set(0,0,0)
  end

  point.lastFrameVel:set(tempVecCurrVel)
  point.lastFramePos:set(tempVecPos)

  point.vel:set(tempVecCurrVel)
end

local distFromCenter = 4

local frontPointTempVec = vec3()
local rearPointTempVec = vec3()
local crossUpTempVec = vec3()
local function checkOverSteering()

  isOverSteering = false

  frontPointTempVec:set(dirVec)
  frontPointTempVec:setScaled(distFromCenter)
  frontPointTempVec:setAdd(center)

  rearPointTempVec:set(dirVec)
  rearPointTempVec:setScaled(-distFromCenter)
  rearPointTempVec:setAdd(center)

  frontPoint.pos:set(frontPointTempVec)
  rearPoint.pos:set(rearPointTempVec)
  updateReferencePointData(frontPoint)
  updateReferencePointData(rearPoint)
  if frontPoint.vel and vehData then
    avrgRefPointsKphSpeed = ((frontPoint.vel:length() + rearPoint.vel:length()) / 2) * 3.6

    crossUpTempVec:set(dirVec:cross(up))
    frontDot = frontPoint.vel:normalized():dot(crossUpTempVec)
    centerDot = velDir:dot(crossUpTempVec)
    rearDot = rearPoint.vel:normalized():dot(crossUpTempVec)

    isOverSteering =
    frontDot < 0 and centerDot > 0 or
    centerDot < 0 and frontDot > 0 or
    frontDot > 0 and centerDot > 0 and rearDot > 0 or
    frontDot < 0 and centerDot < 0 and rearDot < 0
  end
end

-- Reverse cheesing is when the player accelerate in reverse while turning the wheel. This gains a lot of point
local function isReverseCheesingFunc()
  isReverseCheesing = (currDegAngle or 0) > 120 and (accelerationSmoothed or 0) > -0.07
  if isBeingDebugged then
    debugDrawer:drawTextAdvanced(center, "Reverse cheesing : " .. tostring(isReverseCheesing), ColorF(1,1,1,1), true, false, ColorI(0,0,0,255))
  end
end

local vec3DistDiff = vec3(0,0,0)
local function detectAndGatherDriftInfo(vehicleData)

  isReverseCheesingFunc()
  isInTheAir = not gameplay_util_groundContact.isOnWheels(vehId)

  checkOverSteering()

  -- override actual data with the simulated data
  if simulatedDriftData then
    isReverseCheesing = false
    isInTheAir = false
    avrgRefPointsKphSpeed = simulatedDriftData.airSpeed
    currFailPointsCooldown = 0
    currResetTimer = 0
    isOverSteering = true
  end

  isDrifting =
  isOverSteering
  and not isReverseCheesing
  and currDegAngle > driftOptions.minAngle and currDegAngle < driftOptions.maxAngle
  and avrgRefPointsKphSpeed > driftOptions.minAirSpeed
  and not gameplay_walk.isWalking()
  and not isInTheAir
  and currFailPointsCooldown <= 0
  and currResetTimer <= 0
  and not isCrashing

  velDir:set(push3(vehicleData.vel):normalized())

  if isDrifting then
    if driftChainActiveData == nil then
      newDriftChainActiveData()
      extensions.hook("onDriftChainStarted")
    end

    if driftActiveData == nil then
      newDriftActiveData(vehicleData)

      if velDir:dot(up:cross(vehicleData.dirVec:normalized())) > 0 then
        driftChainActiveData.rightDrifts = driftChainActiveData.rightDrifts + 1
        driftActiveData.direction = "right"
      else
        driftChainActiveData.leftDrifts = driftChainActiveData.leftDrifts + 1
        driftActiveData.direction = "left"
      end
      driftChainActiveData.chainedDrifts = driftChainActiveData.chainedDrifts + 1
      stopSpinoutCheck = false

      if driftChainActiveData.chainedDrifts > 1 then
        extensions.hook("onDriftTransition")
      end
      extensions.hook("onDriftStatusChanged", true, driftActiveData.direction)
    end

    currDriftCompletedTimer = currDriftCompleteTime

    driftAngleDiff = math.deg(math.acos(velDir:cosAngle(driftActiveData.lastFrameVelDir))) -- angle in deg

    driftActiveData.angleVelocity = driftAngleDiff / dtSim
    driftActiveData.totalDriftAngle = driftActiveData.totalDriftAngle + driftAngleDiff
    driftActiveData.lastFrameVelDir:set(velDir)
    driftActiveData.totalDriftTime = driftActiveData.totalDriftTime + dtSim
    driftActiveData.score = driftActiveData.score + gameplay_drift_scoring.getScoreAddedThisFrame()

    -- total drifting distance
    vec3DistDiff:setSub2(driftActiveData.lastPos, vehicleData.pos)
    driftChainActiveData.totalDriftDistance = driftChainActiveData.totalDriftDistance + vec3DistDiff:length()
    driftActiveData.lastPos:set(vehicleData.pos)
    driftActiveData.driftDistance = driftActiveData.driftDistance + vec3DistDiff:length()
    -- total drift time
    driftChainActiveData.totalDriftTime = driftChainActiveData.totalDriftTime + dtSim

    driftActiveData.totalSpeeds = driftActiveData.totalSpeeds + kphAirSpeed
    driftActiveData.totalAngles = driftActiveData.totalAngles + currDegAngle
    driftActiveData.totalPerformanceFactors = driftActiveData.totalPerformanceFactors + gameplay_drift_scoring.getDriftPerformanceFactor()
    driftActiveData.totalSteps = driftActiveData.totalSteps + 1

    lastDriftTimer = 0

  else
    if driftActiveData then --if just stopped drifting
      extensions.hook("onDriftStatusChanged", false)
      extensions.hook("onDriftActiveDataFinished", driftActiveData)
      driftActiveData = nil
    end

    lastDriftTimer = lastDriftTimer + dtSim
  end

  isChainingDrifts = driftChainActiveData ~= nil

  lastFrameIsDrifting = isDrifting
end

local function driftCoolDown()
  if currDriftCompletedTimer > 0 then
    currDriftCompletedTimer = currDriftCompletedTimer - dtSim
    if balanceMode then currDriftCompletedTimer = 1 end
    if currDriftCompletedTimer < 0 and score.cachedScore > 0 then
      extensions.hook('onDriftCompleted', {
        chainDriftData = driftChainActiveData
      })
      driftChainActiveData = nil
    end
  end
end

local function updateDriftCompleteTime()
  currDriftCompleteTime = linearScale(score.combo, 5, 20, driftOptions.baseDriftCompleteTime, 2.25)
end

-- drift scoring is based on distance, not time
local lastFramePlPos = vec3()
local function calculateDistSinceLastFrame()
  if lastFramePlPos then
    distSinceLastFrame =  M.getVehPos():distance(lastFramePlPos)
    if simulatedDriftData then
      distSinceLastFrame = simulatedDriftData.airSpeed * dtSim / 3.6421
    end
  end

  lastFramePlPos:set(M.getVehPos())
end
local plotHelperUtil
local function imguiDebug()
  if isBeingDebugged then
    if im.Begin("Drift detection") then
      im.Text("Is drifting : " .. ((isDrifting and "Yes") or "No"))
      im.Text("Is crashing : " .. ((isCrashing and "Yes") or "No"))
      im.Text("Is reverse cheesing : " .. ((isReverseCheesing and "Yes") or "No"))
      im.Text("Is in the air : " .. ((isInTheAir and "Yes") or "No"))
      im.Text(string.format("Air speed : %d(%d) kph", kphAirSpeed or 0, avrgRefPointsKphSpeed or 0))
      im.Text(string.format("Min required angle : %0.2f", driftOptions.minAngle or 0))
      im.Text(string.format("Drift complete time : %0.2f", currDriftCompleteTime or 0))
      im.Text(string.format("Time to confirmation : %0.2f", currDriftCompletedTimer))
      im.Text(string.format("Acceleration : %0.2f", accelerationSmoothed or 0))

      im.Dummy(im.ImVec2(1, 10))

      if im.Button("Toggle Imgui Crash Window") then
        showImguiCrashWindow = not showImguiCrashWindow
      end

      if isDrifting then
        local avgDriftAngle = driftActiveData.totalAngles / driftActiveData.totalSteps
        im.Text(string.format("Angle : %d °", currDegAngle))
        im.Text(string.format("Total drift distance : %d", driftChainActiveData.totalDriftDistance))
        im.Text(string.format("Average drift angle : %d", avgDriftAngle))
        im.Text(string.format("Wall distance front : %f", driftActiveData.closestWallDistanceFront))
        im.Text(string.format("Wall distance rear : %f", driftActiveData.closestWallDistanceRear))
      end

      im.Text("Balance mode : " .. (balanceMode and "Yes" or "No"))
    end

    gameplay_util_crashDetection.setDebug(showImguiCrashWindow)
  end
end

local function tryTrackDriftVehForCrash()
  if vehId ~= nil and gameplay_drift_general.getContext() ~= "inAnotherMissionType" and not gameplay_util_crashDetection.isVehTracked(vehId) then
    gameplay_util_crashDetection.addTrackedVehicleById(vehId, crashDetectionSettings, "drift")
  end
end

local function onUpdate(dtReal, _dtSim, dtRaw)
  dtSim = _dtSim

  isBeingDebugged = gameplay_drift_general.getExtensionDebug("gameplay_drift_drift")
  imguiDebug()
  isDrifting = false
  if gameplay_drift_general.getGeneralDebug() then profiler:start() end

  if gameplay_walk.isWalking() or gameplay_drift_general.getPaused() or _dtSim <= 0 then return end

  score = gameplay_drift_scoring.getScore()


  if not resetFlag then reset() end

  if vehId and not veh then
    veh = scenetree.findObjectById(vehId)
  else
    veh = getPlayerVehicle(0)
  end


  if not veh then return end
  vehId = veh:getId()
  vehData = map.objects[vehId]
  if not vehData then return end
  kphAirSpeed = vehData.vel:length() * 3.6

  vehPos:set(vehData.pos)

  tryTrackDriftVehForCrash()

  updateDriftCompleteTime()
  calculateVehCenterAndWheels()

  calculateDriftAngle(vehData)

  calculateDistSinceLastFrame()
  checkForCrash()
  if not isDrifting and (score.cachedScore or 0) > 0 then
    detectSpinout()
  end
  detectAndGatherDriftInfo(vehData)
  detectCircleDrift()
  if isDrifting then
    calculateDistWall()

    driftActiveData.currDegAngle = currDegAngle
  else
    driftCoolDown()
  end
  if currFailPointsCooldown > 0 then
    currFailPointsCooldown = currFailPointsCooldown - dtSim
  end

  if currResetTimer > 0 then
    currResetTimer = currResetTimer - dtSim
  end

  accelerationSmoothed = accelSmoother:get(kphAirSpeed- (lastFrameKphAirSpeed or kphAirSpeed), dtReal)
  lastFrameKphAirSpeed = kphAirSpeed
  if gameplay_drift_general.getGeneralDebug() then
    profiler:add("Drift detection")
    gc = profiler.sections[1].garbage
    profiler:finish(false)
  end
end

local function simulateADrift(data)
  simulatedDriftData = data
end

local function getDriftActiveData()
  return driftActiveData
end

local function getDriftChainActiveData()
  return driftChainActiveData
end

local function setAllowDonut(value)
  driftOptions.allowDonut = value
end

local function setAllowTightDrift(value)
  driftOptions.allowTightDrifts = value
end

local function setVehId(newVehId)
  vehId = newVehId
end

local function getVehId()
  return vehId
end

local function getPlVeh()
  return veh
end

local function getIsDrifting()
  return isDrifting
end

local function getDriftOptions()
  return driftOptions
end

local function getVehCorners()
  return {corner_FR, corner_BR, corner_BL, corner_FL}
end

local function getAirSpeed()
  return kphAirSpeed
end

local function getVehPos()
  if M.doesPlHaveVeh() then
    return vehData.pos
  end
end

local function getCurrDegAngleSigned()
  return (currDegAngle or 0) * sign(currDegAngleSign or 0)
end

local function getCurrDriftCompletedTime()
  return currDriftCompletedTimer / currDriftCompleteTime
end

local function getAngleDiff()
  return driftAngleDiff
end

local function getGC()
  return gc
end

local function getVehPos()
  return vehPos
end

local function getIsCrashing()
  return isCrashing
end

local function getDriftDebugInfo()
  return driftDebugInfo
end

local function onVehicleSwitched(oldId, newVehId)

  vehId = newVehId

  if oldId ~= -1 then
    gameplay_util_crashDetection.removeTrackedVehicleById(oldId)
  end
  if gameplay_drift_general.getContext() ~= "inAnotherMissionType" and newVehId ~= -1 then
    tryTrackDriftVehForCrash()
  end

  currResetTimer = resetTimer
end

-- this is only used for the debug imgui window to test UI
local function onAnyStuntZoneAccomplished()
  currDriftCompletedTimer = currDriftCompleteTime
end

local function onDriftContextChanged(context)
  if context ~= "inAnotherMissionType" then
    tryTrackDriftVehForCrash()
  elseif context == "inAnotherMissionType" then
    gameplay_util_crashDetection.removeTrackedVehicleById(vehId)
  end
end

local function onDriftChallengeFinished()
  if driftChainActiveData ~= nil then
    extensions.hook('onDriftCompleted', {
      chainDriftData = driftChainActiveData
    })
  end
  if isDrifting then
    extensions.hook("onDriftActiveDataFinished", driftActiveData)
  end
end

local function onVehicleCrashStarted(crashStartData)
  if gameplay_walk.isWalking() or gameplay_drift_general.getPaused() or gameplay_drift_general.getFrozen() or dtSim <= 0 then return end

  if crashStartData.vehId == nil or score.cachedScore == nil or vehId == nil then return end

  if crashStartData.vehId == vehId and score.cachedScore > 0 then
    crashed = true
  end
end

local function onDriftPlVehReset()
  reset()
  currResetTimer = resetTimer
end

local function getDistSinceLastFrame()
  return distSinceLastFrame or 0
end

local function getCurrentDriftDuration()
  if isDrifting then
    return driftActiveData.totalDriftTime
  else
    return 0
  end
end

local function getIsOverSteering()
  return isOverSteering
end

local function getIsOverMinSpeedForDrift()
  return avrgRefPointsKphSpeed > driftOptions.minAirSpeed
end

local function doesPlHaveVeh()
  return vehData ~= nil
end

local function getVehData()
  return vehData
end

local function getVehVel()
  return velDir
end

local function getIsInTheAir()
  return isInTheAir
end

local function setBalanceMode(_balanceMode)
  balanceMode = _balanceMode
end

M.onUpdate = onUpdate
M.onVehicleCrashStarted = onVehicleCrashStarted
M.onVehicleSwitched = onVehicleSwitched
M.onDriftPlVehReset = onDriftPlVehReset
M.onDriftContextChanged = onDriftContextChanged
M.onAnyStuntZoneAccomplished = onAnyStuntZoneAccomplished
M.onDriftChallengeFinished = onDriftChallengeFinished
M.reset = reset

M.getDriftActiveData = getDriftActiveData
M.getDriftChainActiveData = getDriftChainActiveData
M.getVehId = getVehId
M.getVehVel = getVehVel
M.getPlVeh = getPlVeh
M.getVehData = getVehData
M.getIsDrifting = getIsDrifting
M.getDriftOptions = getDriftOptions
M.getAirSpeed = getAirSpeed
M.getDistSinceLastFrame = getDistSinceLastFrame
M.getVehPos = getVehPos
M.getAngleDiff = getAngleDiff
M.getVehCorners = getVehCorners
M.getCurrDriftCompletedTime = getCurrDriftCompletedTime
M.getCurrDegAngleSigned = getCurrDegAngleSigned
M.getIsCrashing = getIsCrashing
M.getDriftDebugInfo = getDriftDebugInfo
M.getGC = getGC
M.getCurrentDriftDuration = getCurrentDriftDuration
M.getIsOverSteering = getIsOverSteering
M.getIsOverMinSpeedForDrift = getIsOverMinSpeedForDrift
M.getIsInTheAir = getIsInTheAir

M.doesPlHaveVeh = doesPlHaveVeh

M.setVehId = setVehId
M.setAllowTightDrift = setAllowTightDrift
M.setAllowDonut = setAllowDonut
M.setBalanceMode = setBalanceMode

-- to simulate a drift
M.simulateADrift = simulateADrift
return M