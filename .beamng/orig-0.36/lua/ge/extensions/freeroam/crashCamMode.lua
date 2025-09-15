-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local crashCamEnabled = true
local forcedEnabled = false
local crashCamActive
local crashCamTimer -- counts the duration of an action cam
local crashCamSimTimer
local crashCamData = {}
local timeSinceLastCrashCam = 0
local vehiclesHaveIntersected
local playerVehiclePointsInFrame = 0
local playerVehiclePointsInFrameMaximum = 0

local crashSpeedCutoff = 13 -- how many m/s difference between vehicles to trigger the crash cam
local crashCamCooldown = 1 -- minimum time between two triggers of the crash cam
local crashCamDuration = 4 -- duration of a crash cam slowmo in seconds
local lookAheadTime = 0.2 -- time to look ahead for a crash
local bbSizeFactor = 0.85 -- factor to decrease BB sizes of vehicles for vehicle-vehicle collisions
local deltaTimeForVelocityBasedCheck = 0.1 -- how often to check for velocity based crashes. this is the delta in seconds
local crashSpeedOffsetForVelocityCheck = 3 -- add some additional speed to the cutoff for velocity diff checks, because they tend to be a bit higher than the raycast based checks

local staticCollisionRayCastOffset = 0.2
local outerPointsOffsetFromEdge = 0.1

-- crash cam mode. one of these is chosen at random each crash
-- 1: camera is close above, focuses the crash position and doesnt move
-- 2: camera is further away on the ground and rotates to keep the crash in view
-- 3: time stops while the camera rotates around the crash until time continues
local mode
local modeAttributes = {
  {prob = 45}, -- mode 1
  {prob = 45}, -- mode 2
  {prob = 10, cooldown = 5}, -- mode 3
}

-- 1 : checks collisions between the player vehicle -> traffic vehicles and static objects
-- 2 : checks collisions between the player vehicle -> traffic vehicles, non-traffic vehicles and static objects
local trackingMode = 1

local startedPathCam = false
local startPathTimer -- timing offset for starting the path cam
local seed = vec3(math.random(), math.random(), math.random())
local zVec = vec3(0,0,1)

local rotMtx = MatrixF(true)
local function rotateAroundVec(point, rotationVec, angle)
  local res = vec3(point)
  rotMtx:setFromEuler(vec3(0,0,angle))
  local localToWorldSpace = rotationVec:getRotationTo(zVec)
  res:setRotate(localToWorldSpace)
  res = rotMtx:mulP3F(res)
  localToWorldSpace:inverse()
  res:setRotate(localToWorldSpace)
  return res
end

local function bbsIntersect(bb1, bb2)
  local center1, xAxis1, yAxis1, zAxis1 = bb1:getCenterHalfExtentAxes()
  local center2, xAxis2, yAxis2, zAxis2 = bb2:getCenterHalfExtentAxes()
  return overlapsOBB_OBB(center1, xAxis1, yAxis1, zAxis1, center2, xAxis2, yAxis2, zAxis2)
end

local function intersectRayOBB(rayPos, rayDir, bb)
  return intersectsRay_OBB(rayPos, rayDir, bb:getCenter(), bb:getAxis(0) * bb:getHalfExtents().x, bb:getAxis(1) * bb:getHalfExtents().y, bb:getAxis(2) * bb:getHalfExtents().z)
end

local function predictCrashPoint()
  local playerVeh = getPlayerVehicle(0)
  local bb1 = playerVeh:getSpawnWorldOOBB()

  -- Try to find the point where they crash
  local bbCornerPoints = {}
  table.insert(bbCornerPoints, bb1:getCenter() + bb1:getAxis(0) * bb1:getHalfExtents().x)
  table.insert(bbCornerPoints, bb1:getCenter() - bb1:getAxis(0) * bb1:getHalfExtents().x)
  table.insert(bbCornerPoints, bb1:getCenter() + bb1:getAxis(1) * bb1:getHalfExtents().y)
  table.insert(bbCornerPoints, bb1:getCenter() - bb1:getAxis(1) * bb1:getHalfExtents().y)

  local minDist = math.huge
  local hitPoint
  for _, point in ipairs(bbCornerPoints) do
    local hitDist = intersectRayOBB(point, playerVeh:getVelocity():normalized(), crashCamData.futureBB2)
    if hitDist < minDist then
      hitPoint = point + playerVeh:getVelocity():normalized() * hitDist
      minDist = hitDist
    end
  end

  hitPoint = hitPoint or (crashCamData.futureBB1Center + crashCamData.futureBB2:getCenter()) * 0.5
  local camOffset = vec3(0,0,5) + seed:getBluePointInCircle(2)
  return hitPoint, camOffset
end

local function buildPath()
  local veh1 = scenetree.findObjectById(crashCamData.id1)
  local veh2 = scenetree.findObjectById(crashCamData.id2)
  local bb1 = veh1:getSpawnWorldOOBB()
  local bb2 = veh2:getSpawnWorldOOBB()

  local bbCornerPoints = {}
  table.insert(bbCornerPoints, bb1:getCenter() + bb1:getAxis(0) * bb1:getHalfExtents().x)
  table.insert(bbCornerPoints, bb1:getCenter() - bb1:getAxis(0) * bb1:getHalfExtents().x)
  table.insert(bbCornerPoints, bb1:getCenter() + bb1:getAxis(1) * bb1:getHalfExtents().y)
  table.insert(bbCornerPoints, bb1:getCenter() - bb1:getAxis(1) * bb1:getHalfExtents().y)

  local minDist = math.huge
  local adjustedHitPoint
  for _, point in ipairs(bbCornerPoints) do
    local hitDist = intersectRayOBB(point, crashCamData.velocity:normalized(), bb2)
    if (hitDist >= 0) and (hitDist < minDist) then
      adjustedHitPoint = point + crashCamData.velocity:normalized() * hitDist
      minDist = hitDist
    end
  end
  adjustedHitPoint = adjustedHitPoint or ((bb1:getCenter() + bb2:getCenter()) / 2)

  local path = { looped = false, manualFov = false}
  local startPos = core_camera.getPosition()
  local offset1 = crashCamData.velocity:cross(vec3(0,0,1)):normalized() * 6
  local offset2 = crashCamData.velocity:normalized() * 6
  local offset3 = -offset1
  local markerTimeOffset = 1.2

  local m1 = { fov = core_camera.getFovDeg(), movingEnd = false, movingStart = false, positionSmooth = 0.5, pos = startPos, rot = core_camera.getQuat(), time = 0, trackPosition = false, nearClip = nil  }
  local m2 = { fov = 50, movingEnd = false, movingStart = false, positionSmooth = 0.5, pos = adjustedHitPoint + offset1, rot = quatFromDir(-offset1), time = markerTimeOffset, trackPosition = false, nearClip = nil }
  local m3 = { fov = 50, movingEnd = false, movingStart = false, positionSmooth = 0.5, pos = adjustedHitPoint + offset2, rot = quatFromDir(-offset2), time = markerTimeOffset * 2, trackPosition = false, nearClip = nil }
  local m4 = { fov = 50, movingEnd = false, movingStart = false, positionSmooth = 0.5, pos = adjustedHitPoint + offset3, rot = quatFromDir(-offset3), time = markerTimeOffset * 3, trackPosition = false, nearClip = nil }
  local m5 = { fov = core_camera.getFovDeg(), movingEnd = false, movingStart = false, positionSmooth = 0.5, pos = startPos, rot = core_camera.getQuat(), time = markerTimeOffset * 4, trackPosition = false, nearClip = nil }

  path.markers = {m1, m2, m3, m4, m5}
  return path
end

local function chooseModeBasedOnProb()
  -- Build the mode table
  local modesToChooseFrom = {}
  for _, mode in ipairs(modeAttributes) do
    if not mode.cooldown or timeSinceLastCrashCam > mode.cooldown then
      table.insert(modesToChooseFrom, mode)
    end
  end

  -- Add all pops together
  local totalProb = 0
  for _, mode in ipairs(modesToChooseFrom) do
    totalProb = totalProb + mode.prob
  end

  if totalProb <= 0 then
    return math.random(#modeAttributes)
  end

  -- Choose one pop at random and count up until you reach it
  local chosenPop = math.random(totalProb)
  local probCounter = 0
  for modeId, mode in ipairs(modesToChooseFrom) do
    probCounter = probCounter + mode.prob
    if probCounter >= chosenPop then
      return modeId
    end
  end
end

local previousSimSpeed
local previousCamMode
local previousUiVisibility

-- these are for the velocity based crash check
local playerVelAtLastCheckpoint
local deltaSinceLastCheckpoint = 0

local function toggleActionCam(active)
  if active then
    -- start crash cam
    mode = mode or chooseModeBasedOnProb()
    previousSimSpeed = simTimeAuthority.get()
    previousCamMode = core_camera.getActiveCamName()
    previousUiVisibility = ui_visibility.get()

    crashCamActive = true
    crashCamTimer = 0
    crashCamSimTimer = 0
    vehiclesHaveIntersected = nil
    playerVehiclePointsInFrame = 0
    playerVehiclePointsInFrameMaximum = 0

    if mode == 3 then
    else
      ui_visibility.set(false)
      simTimeAuthority.setInstant(0.15)
      core_camera.setByName(0, 'crash', false, {veh1Id = crashCamData.id1, veh2Id = crashCamData.id2, hitPoint = crashCamData.hitPoint, camOffset = crashCamData.camOffset, camMode = mode})
    end
  else
    -- end crash cam
    local wasCrashCamActive = crashCamActive
    crashCamTimer = 0
    crashCamSimTimer = 0
    startedPathCam = false
    startPathTimer = nil
    timeSinceLastCrashCam = 0
    table.clear(crashCamData)
    mode = nil
    crashCamActive = false
    playerVelAtLastCheckpoint = nil
    deltaSinceLastCheckpoint = 0

    if wasCrashCamActive then
      simTimeAuthority.pauseSmooth(false)
      if previousSimSpeed and simTimeAuthority.get() ~= previousSimSpeed then
        simTimeAuthority.set(previousSimSpeed)
      end
      if previousUiVisibility ~= nil then
        ui_visibility.set(previousUiVisibility)
      end
      if previousCamMode ~= nil then
        core_camera.setByName(0, previousCamMode)
        extensions.hook("onCrashCamEnded")
      end
    end
  end
end

local function setEnabled(enabled)
  if not enabled then
    toggleActionCam(false)
  end
  crashCamEnabled = enabled
end

local function startPathCam(dtReal)
  if startedPathCam then return end
  -- start the path cam
  ui_visibility.set(false)
  simTimeAuthority.pauseSmooth(true, nil, nil, nil, false)
  startPathTimer = startPathTimer or 0
  startPathTimer = startPathTimer + dtReal
  if startPathTimer < 0.8 then
    return
  end

  startedPathCam = true
  local path = buildPath()
  local initData = {}
  initData.useDtReal = true
  initData.finishedPath = function()
    simTimeAuthority.pauseSmooth(false)
    toggleActionCam(false)
  end
  core_paths.playPath(path, 0, initData)
end


local playerBBCenter = vec3()

local playerPos = vec3()
local playerPosLast = vec3()
local playerVel = vec3()
local playerVelLast = vec3()
local futurePlayerBBCenter = vec3()
local playerAxis0, playerAxis1, playerAxis2 = vec3(), vec3(), vec3()
local playerBBHalfAxis0, playerBBHalfAxis1, playerBBHalfAxis2 = vec3(), vec3(), vec3()

local playerVelNormalized = vec3()

local otherVehPos = vec3()
local otherVel = vec3()
local otherBBHalfAxis0, otherBBHalfAxis1, otherBBHalfAxis2 = vec3(), vec3(), vec3()
local futureOtherBBCenter = vec3()
local trafficVehicleIds = {}

local function willCollideWithJbeam(vehId1, otherId)
  if not be:getObjectActive(otherId) then return false end

  otherVehPos:set(be:getObjectPositionXYZ(otherId))
  if otherVehPos:distance(futurePlayerBBCenter) > 30 then return false end

  otherVel:set(be:getObjectVelocityXYZ(otherId))
  if otherVel:length() > 300 then return false end -- dont check any traffic vehicle going over 300 m/s because that is probably a teleport
  if playerVel:distance(otherVel) < crashSpeedCutoff then return false end -- Only slowmo when velocity diff is great enough

  futureOtherBBCenter:set(push3(be:getObjectOOBBCenterXYZ(otherId)) + push3(otherVel) * lookAheadTime)

  otherBBHalfAxis0:set(be:getObjectOOBBHalfAxisXYZ(otherId, 0))
  otherBBHalfAxis0:setScaled(bbSizeFactor)
  otherBBHalfAxis1:set(be:getObjectOOBBHalfAxisXYZ(otherId, 1))
  otherBBHalfAxis1:setScaled(bbSizeFactor)
  otherBBHalfAxis2:set(be:getObjectOOBBHalfAxisXYZ(otherId, 2))
  otherBBHalfAxis2:setScaled(bbSizeFactor)

  if overlapsOBB_OBB(futurePlayerBBCenter, playerBBHalfAxis0, playerBBHalfAxis1, playerBBHalfAxis2, futureOtherBBCenter, otherBBHalfAxis0, otherBBHalfAxis1, otherBBHalfAxis2) then
    -- set the data and toggle the action cam

    -- TODO i think we can refactor this to not need the vehicle BB reference anymore
    local obj = getObjectByID(otherId)
    if not obj then return false end
    local otherBB = obj:getSpawnWorldOOBB()
    local otherTrans = otherBB:getBoxTransform().matrix
    otherTrans:setColumn(3, futureOtherBBCenter)
    local futureOtherBB = OrientedBox3F()
    futureOtherBB:set2(otherTrans, otherBB:getHalfExtents() * 2)

    crashCamData.futureBB1Center = futurePlayerBBCenter
    crashCamData.futureBB2 = futureOtherBB
    crashCamData.velocity = playerVel
    crashCamData.hitPoint, crashCamData.camOffset = predictCrashPoint()
    crashCamData.id1 = be:getPlayerVehicleID(0)
    crashCamData.id2 = otherId
    toggleActionCam(true)
    return
  end
end

local function updatePlayerData()
  local playerVehId = be:getPlayerVehicleID(0)

  playerVelNormalized:set(push3(playerVel):normalized())
  playerBBCenter:set(be:getObjectOOBBCenterXYZ(playerVehId))
  playerAxis0:set(be:getObjectOOBBAxisNormalizedXYZ(playerVehId, 0))
  playerAxis1:set(be:getObjectOOBBAxisNormalizedXYZ(playerVehId, 1))
  playerAxis2:set(be:getObjectOOBBAxisNormalizedXYZ(playerVehId, 2))

  futurePlayerBBCenter:set(push3(playerBBCenter) + push3(playerVel) * lookAheadTime)

  playerBBHalfAxis0:set(be:getObjectOOBBHalfAxisXYZ(playerVehId, 0))
  playerBBHalfAxis0:setScaled(bbSizeFactor)
  playerBBHalfAxis1:set(be:getObjectOOBBHalfAxisXYZ(playerVehId, 1))
  playerBBHalfAxis1:setScaled(bbSizeFactor)
  playerBBHalfAxis2:set(be:getObjectOOBBHalfAxisXYZ(playerVehId, 2))
  playerBBHalfAxis2:setScaled(bbSizeFactor)
end

local function willCollideWithTraffic()
  if not gameplay_traffic then return end

  local trafficList = gameplay_traffic.getTrafficList()
  local parkedList = gameplay_parking.getParkedCarsList()
  if tableIsEmpty(trafficList) and tableIsEmpty(parkedList) then return end

  table.clear(trafficVehicleIds)
  arrayConcat(trafficVehicleIds, trafficList)
  arrayConcat(trafficVehicleIds, parkedList)

  local playerVehId = be:getPlayerVehicleID(0)

  for _, otherId in ipairs(trafficVehicleIds) do
    if willCollideWithJbeam(playerVehId, otherId) then
      return -- no need to check other vehicles
    end
  end
end

local function willCollideWithActiveVehicles()
  local playerVehId = be:getPlayerVehicleID(0)

  for vid, _ in activeVehiclesIterator() do
    if willCollideWithJbeam(playerVehId, vid) then
      return -- no need to check other vehicles
    end
  end
end


local function findCamPos(startPos, recDepth)
  recDepth = recDepth and recDepth + 1 or 1
  if recDepth > 10 then return nil end

  -- Choose a candidate
  local candidate = vec3(startPos)
  candidate = startPos + vec3(0,0,5) + seed:getBluePointInCircle(2)

  -- Check if the candidate is good
  local candidateDistance = candidate:distance(startPos)
  if (castRayStatic(candidate, startPos-candidate, candidateDistance) < candidateDistance
  or castRayStatic(startPos, candidate-startPos, candidateDistance) < candidateDistance) then
    return findCamPos(startPos, recDepth)
  end
  return candidate - startPos
end

local bottomPoints = {}
local function willCollideWithWall()
  if playerVel:length() > crashSpeedCutoff then
    local playerVehId = be:getPlayerVehicleID(0)
    local halfExtentsX, halfExtentsY, halfExtentsZ = be:getObjectOOBBHalfExtentsXYZ(playerVehId)
    local detectedCrashLocation

    -- place bottomPoints "staticCollisionRayCastOffset" m apart
    if not bottomPoints[1] then bottomPoints[1] = vec3() end
    bottomPoints[1]:set(push3(playerBBCenter) - push3(playerAxis1) * (halfExtentsY - 0.2))

    local counter = 2
    for offset = staticCollisionRayCastOffset, halfExtentsX - outerPointsOffsetFromEdge, staticCollisionRayCastOffset do
      if not bottomPoints[counter] then bottomPoints[counter] = vec3() end
      bottomPoints[counter]:set(push3(bottomPoints[1]) - push3(playerAxis0) * offset)
      counter = counter + 1

      if not bottomPoints[counter] then bottomPoints[counter] = vec3() end
      bottomPoints[counter]:set(push3(bottomPoints[1]) + push3(playerAxis0) * offset)
      counter = counter + 1
    end

    -- do the inner raycasts
    local rayCastDist = playerVel:length() * lookAheadTime
    for i = 1, counter-1 do
      local point = bottomPoints[i]
      local hitDist1 = castRayStatic(point, playerVel, rayCastDist, nil)

      if hitDist1 < rayCastDist then -- first raycast hits
        local point2 = point + playerAxis2 * 0.5
        local hitDist2 = castRayStatic(point2, playerVel, rayCastDist * 2, nil)

        if hitDist2 < rayCastDist * 2 then -- upper raycast hits
          local hitPoint1 = point + playerVel:normalized() * hitDist1
          local hitPoint2 = point2 + playerVel:normalized() * hitDist2
          local inclineDir = hitPoint2 - hitPoint1
          local inclineAngle = math.acos(inclineDir:cosAngle(playerVel)) * 180/math.pi

          if inclineAngle > 45 then -- guessed incline angle is greater than 45 degrees
            local rayCastDir
            if i % 2 == 0 then
              -- point is on right side
              rayCastDir = rotateAroundVec(playerVel, playerAxis2, 2 * math.pi/180)
            else
              -- point is on left side
              rayCastDir = rotateAroundVec(playerVel, playerAxis2, -2 * math.pi/180)
            end

            local hitDist3 = castRayStatic(point, rayCastDir, rayCastDist, nil)
            if hitDist3 < rayCastDist then -- horizontal raycast hits
              -- Try to guess the angle of the edge and dont trigger the crash cam if the angle is too small
              local p1 = point + playerVelNormalized * hitDist1
              local p2 = point + rayCastDir:normalized() * hitDist3
              local edgeVector = (p2 - p1):normalized()
              local dotProduct = edgeVector:dot(playerVel)
              local velocityTowardsWall = playerVel - (edgeVector * dotProduct)
              if velocityTowardsWall:length() > crashSpeedCutoff then -- obstacle close enough to right angle
                detectedCrashLocation = point + playerVelNormalized * hitDist1 * 0.9
                break
              else
                detectedCrashLocation = nil
              end
            else
              detectedCrashLocation = nil
            end
          end
        end
      end
    end

    if detectedCrashLocation then
      mode = math.random(2) -- static collision only supports mode 1 and 2
      crashCamData.hitPoint = detectedCrashLocation
      crashCamData.id1 = playerVehId
      if mode == 1 then
        crashCamData.camOffset = findCamPos(crashCamData.hitPoint)
        if not crashCamData.camOffset then mode = 2 end
      end
      toggleActionCam(true)
    end
  end
end

local playerUpVec = vec3()
local function rolloverCheck(dtSim)
  local playerVehId = be:getPlayerVehicleID(0)
  playerUpVec:set(be:getObjectOOBBHalfAxisXYZ(playerVehId, 2))
  if playerUpVec.z < 0 then
    mode = 2
    crashCamData.id1 = playerVehId
    crashCamData.hitPoint = getPlayerVehicle(0):getPosition()
    toggleActionCam(true)
    --core_camera.setByName(0, "external", false)
  end
end

local bbCenter = vec3()
local function crashCheckBasedOnVelocity(dtSim)
  deltaSinceLastCheckpoint = deltaSinceLastCheckpoint + dtSim
  if deltaSinceLastCheckpoint < deltaTimeForVelocityBasedCheck then return end
  if playerVelAtLastCheckpoint and playerVel:distance(playerVelAtLastCheckpoint) > (crashSpeedCutoff + crashSpeedOffsetForVelocityCheck) then

    mode = math.random(2) -- static collision only supports mode 1 and 2
    crashCamData.id1 = be:getPlayerVehicleID(0)

    -- Calculate a crash point by going forward from the center, but do a raycast so we dont end up inside a wall
    local halfExtentsX, halfExtentsY, halfExtentsZ = be:getObjectOOBBHalfExtentsXYZ(crashCamData.id1)
    bbCenter:set(be:getObjectOOBBCenterXYZ(crashCamData.id1))
    local playerVehNormalized = playerVel:normalized()
    local hitDist = castRayStatic(bbCenter, playerVehNormalized, halfExtentsY)
    local crashPos = bbCenter + playerVehNormalized * (hitDist - 0.2)

    crashCamData.hitPoint = vec3(crashPos)

    if mode == 1 then
      crashCamData.camOffset = findCamPos(crashCamData.hitPoint)
      if not crashCamData.camOffset then mode = 2 end
    end
    toggleActionCam(true)
  end
  playerVelAtLastCheckpoint = playerVelAtLastCheckpoint or vec3()
  playerVelAtLastCheckpoint:set(playerVel)
  deltaSinceLastCheckpoint = 0
end

local function isStateFreeroam()
  if core_gamestate.state and (core_gamestate.state.state == "freeroam") then
    return true
  end
  return false
end

local function getNumberOfPointsInCamFrustum(playerVeh)
  local pointsInFrame = 0
  local playerBB = playerVeh:getSpawnWorldOOBB()
  local frustum = Engine.sceneGetCameraFrustum()
  for i = 0, 7 do
    if frustum:isPointContained(playerBB:getPoint(i)) then
      pointsInFrame = pointsInFrame + 1
    end
  end
  local center = playerBB:getCenter()
  if frustum:isPointContained(center) then pointsInFrame = pointsInFrame + 1 end

  local halfExtents = playerBB:getHalfExtents()
  local yAxis = playerBB:getAxis(1)
  local frontPoint = center + yAxis * halfExtents.y * 0.5
  local backPoint = center - yAxis * halfExtents.y * 0.5
  if frustum:isPointContained(frontPoint) then pointsInFrame = pointsInFrame + 1 end
  if frustum:isPointContained(backPoint) then pointsInFrame = pointsInFrame + 1 end
  return pointsInFrame
end

local function onUpdate(dtReal, dtSim)
  if not crashCamEnabled and not forcedEnabled then return end
  if not isStateFreeroam() and not forcedEnabled then
    crashCamEnabled = false
    return
  end
  local playerVehId = be:getPlayerVehicleID(0)
  if playerVehId == -1 then return end

  playerPos:set(be:getObjectPositionXYZ(playerVehId))
  playerVel:set(be:getObjectVelocityXYZ(playerVehId))

  local hasTeleported = core_camera.objectTeleported(playerPos, playerPosLast, playerVelLast, dtReal)
  if not crashCamActive then
    -- crash cam is not active
    if timeSinceLastCrashCam > crashCamCooldown
      and core_replay.getState() ~= "playback"
      and not commands.isFreeCamera()
      and simTimeAuthority.get() == 1
      and not hasTeleported
    then
      updatePlayerData()

      if trackingMode == 1 then
        willCollideWithTraffic()
        willCollideWithWall()
        crashCheckBasedOnVelocity(dtSim)
        --rolloverCheck()
      elseif trackingMode == 2 then
        willCollideWithActiveVehicles()
        willCollideWithWall()
      end
    end
    timeSinceLastCrashCam = timeSinceLastCrashCam + dtSim
  else
    -- crash cam is active
    crashCamTimer = crashCamTimer + dtReal
    crashCamSimTimer = crashCamSimTimer + dtSim
    local playerVeh = getObjectByID(playerVehId)

    if crashCamData.id2 then
      -- crash with other vehicle

      if not vehiclesHaveIntersected then
        vehiclesHaveIntersected = false
        -- check if the two vehicles are intersecting
        local otherVeh = scenetree.findObjectById(crashCamData.id2)
        if bbsIntersect(playerVeh:getSpawnWorldOOBB(), otherVeh:getSpawnWorldOOBB()) then
          vehiclesHaveIntersected = true
        end
      end

      if mode == 3 then
        if vehiclesHaveIntersected then
          startPathCam(dtReal)
        end
      end

      -- stop the crash cam if the crash hasnt happen after a certain time
      if vehiclesHaveIntersected == false and crashCamSimTimer > (lookAheadTime + 0.5) then
        local timerLastCrashTemp
        if mode == 3 then
          -- keep the timer when in mode 3
          timerLastCrashTemp = timeSinceLastCrashCam
        end
        toggleActionCam(false)
        if timerLastCrashTemp then
          timeSinceLastCrashCam = timerLastCrashTemp
        end
      end
    else
      -- crash with wall
    end

    if (mode == 1 or mode == 2) and not (render_openxr and render_openxr.isSessionRunning()) then
      -- switch to normal external cam when the vehicle has been in frame once and then left the frame
      playerVehiclePointsInFrame = getNumberOfPointsInCamFrustum(playerVeh)
      playerVehiclePointsInFrameMaximum = math.max(playerVehiclePointsInFrame, playerVehiclePointsInFrameMaximum)
      if playerVehiclePointsInFrame < playerVehiclePointsInFrameMaximum / 2 then
        core_camera.setByName(0, "external", false)
      end
    end

    -- stop the crash cam after "crashCamDuration" seconds
    if (crashCamTimer > crashCamDuration and not startedPathCam)
      or hasTeleported
    then
      toggleActionCam(false)
    end
  end
  playerPosLast:set(playerPos)
  playerVelLast:set(playerVel)
end

local function onVehicleResetted(vehId)
  if be:getPlayerVehicleID(0) == vehId then
    toggleActionCam(false)
  end
end

local function onReplayStateChanged(state)
  toggleActionCam(false)
end

local function onSerialize()
  toggleActionCam(false)
end

local function onClientEndMission(levelPath)
  setEnabled(false)
end

local function onClientStartMission()
  if isStateFreeroam() then
    setEnabled(true)
  end
end

local function onExtensionLoaded()
  if not settings.getValue('enableCrashCam') then return false end
end

local function onExtensionUnloaded()
  toggleActionCam(false)
end

-- stop the crash cam when the user changes the camera mode
local function trackCamMode()
  if crashCamActive then
    toggleActionCam(false)
  end
end

local function onBeforeBigMapActivated()
  toggleActionCam(false)
end

local function onTogglePause()
  if crashCamActive then
    toggleActionCam(false)
  end
end

local function onVehicleSwitched(oldVehId)
  if crashCamActive then
    core_camera.setVehicleCameraByNameWithId(oldVehId, previousCamMode, false)
  end
  toggleActionCam(false)
end

local function onAnyMissionChanged(state)
  if state == "started" then
    setEnabled(false)
  elseif state == "stopped" then
    setEnabled(true)
  end
end

local function trackVehReset()
  toggleActionCam(false)
end

local function setTrackingMode(mode)
  trackingMode = mode
end

local function getTrackingMode()
  return trackingMode
end

local function setForcedEnabled(enabled)
  forcedEnabled = enabled
end

M.onUpdate = onUpdate
M.onVehicleResetted = onVehicleResetted
M.onReplayStateChanged = onReplayStateChanged
M.onSerialize = onSerialize
M.onClientStartMission = onClientStartMission
M.onClientEndMission = onClientEndMission
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.onSettingsChanged = onSettingsChanged
M.trackCamMode = trackCamMode
M.onVehicleSwitched = onVehicleSwitched
M.onAnyMissionChanged = onAnyMissionChanged
M.onBeforeBigMapActivated = onBeforeBigMapActivated
M.onTogglePause = onTogglePause
M.trackVehReset = trackVehReset
M.setTrackingMode = setTrackingMode
M.setForcedEnabled = setForcedEnabled
M.getTrackingMode = getTrackingMode
return M
