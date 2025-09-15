-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local min, max, abs, sqrt = math.min, math.max, math.abs, math.sqrt

local script = {}
local inScript = nil

local time = 0
local scriptTime = 0

local prevDirPoint = nil
local prevVel = 0
local prevAccel = 0
local aiPos = vec3(0, 0, 0)
local aiVel = vec3(0, 0, 0)
local aiDirVec = vec3(0, 0, 0)
local aiSpeed = 0
local speedDiffSmoother = newTemporalSmoothingNonLinear(math.huge, 0.3)
local targetPos = vec3(0, 0, 0)
local targetLength = 1
local initConditions = {}
local posError = 0

local followInitCounter = 0
local loopCounter = 0
local loopType = "alwaysReset"
local externalForce = 0
local hasCollided = false

local logDataToCSV = nil
local csvLog = nil

local function setLogDataToCSV(val)
  if val == true or (type(val) == 'string' and string.lower(val) ~= 'off') then
    if type(val) == 'string' and string.lower(val) ~= 'on' then
      logDataToCSV = val
    else
      logDataToCSV = true
    end
  else
    logDataToCSV = nil
    csvLog = nil
  end
end

local function setSpeedDiffSmootherOutRate(outRate)
  speedDiffSmoother[true] = min(outRate or speedDiffSmoother[true], 1e+30)
end

local recordSpeed = false
local function setRecordSpeed(val)
  if val == true or val == 'on' or val == 'yes' then
    recordSpeed = true
  else
    recordSpeed = false
  end
end

local function wheelToGroundDist(pos, dir, up)
  --[[
    Calculates amount by which the vehicle wheels (assumed contact point) will be above/below ground
    for the given spawn position and orientation
  --]]

  -- Calculate initial average relative (to ref node) wheel node position in the vehicle reference frame [left, back, up]
  local avgWheelNodePos, numOfWheels, maxWheelRadius = vec3(), 0, -math.huge
  for _, wheel in pairs(wheels.wheels) do
    avgWheelNodePos:setAdd(obj:getOriginalNodePositionRelative(wheel.node1))
    numOfWheels = numOfWheels + 1
    maxWheelRadius = math.max(maxWheelRadius, wheel.radius)
  end
  avgWheelNodePos:setScaled(1 / numOfWheels)

  -- Rotate avgWheelNodePosRelVehCoord to vehicle initial condition orientation
  local leftVec = up:cross(dir)
  avgWheelNodePos:set(
    avgWheelNodePos:dot(vec3(leftVec.x, -dir.x, up.x)),
    avgWheelNodePos:dot(vec3(leftVec.y, -dir.y, up.y)),
    avgWheelNodePos:dot(vec3(leftVec.z, -dir.z, up.z))
  )
  -- Calculate absolute position (i.e. relative to world 0 in world frame)
  avgWheelNodePos:setAdd(pos)
  local vehHeight = obj:getInitialHeight()
  -- Raise the point by vehHeight (it might be below ground) and cast a ray to gauge distance to ground
  local dist = obj:castRayStatic(avgWheelNodePos + vehHeight * up, -up, 10 * vehHeight) - vehHeight -- 10 * vehHeight is just a safe bet
  local dH = maxWheelRadius - dist
  -- Adjust spawn position so that vehicle wheels are above ground when vehicle spawns
  --pos:setAdd(dH * up)

  return dH
end

local function driveCar(steering, throttle, brake, parkingbrake)
  input.event("steering", clamp(-steering, -1, 1), 1)
  input.event("throttle", clamp(throttle, 0, 1), 2)
  input.event("brake", clamp(brake, 0, 1), 2)
  input.event("parkingbrake", clamp(parkingbrake, 0, 1), 2)
end

local function velAccelFrom2dist(l1, l2, t1, t2)
  -- Calculates v0, a0 by curve fitting two data points (s(t1) = l1, s(t2) = l2)
  -- to the function s(t) = v0 * t + 0.5 * a0 * t^2 (i.e. distance vs time under constant acceleration)
  if t1 == 0 and t2 == 0 then return 0, 0 end
  local t1s, t2s = t1 * t1, t2 * t2
  local denom = 1 / (t1s * t2 - t1 * t2s)
  return (l2 * t1s - l1 * t2s) * denom, 2 * (l1 * t2 - l2 * t1) * denom
end

local function getCenterPositionRelative()
  local vehCenterPosRel = obj:getCenterPosition() - obj:getPosition()
  -- vehicle center position relative to the refNode position in the vehicle reference frame [left, back, up]
  local vehCenterPositionRelative = vec3(
    vehCenterPosRel:dot(obj:getDirectionVectorUp():cross(obj:getDirectionVector())),
    -(vehCenterPosRel:dot(obj:getDirectionVector())),
    vehCenterPosRel:dot(obj:getDirectionVectorUp())
  )

  return vehCenterPositionRelative
end

local function getInitialSpawnPositionOrientation(inScript, vehPosType, _timeOffset)
  if inScript == nil then
    return
  end

  local script = deepcopy(inScript.path and inScript.path or inScript)
  if not script[2] then
    return
  end

  local timeOffset = _timeOffset or inScript.timeOffset

  if timeOffset and timeOffset ~= 0 then
    if timeOffset > 0 then
      -- find index k such that script[k].t < timeOffset and script[k+1].t >= timeOffset
      local k = #script
      for i = 2, k do
        if script[i].t >= timeOffset then
          k = i - 1
          break
        end
      end
      -- Remove all elements up to and including k - 1
      if k > 1 then
        for i = 1, #script do
          script[i] = script[(i-1)+k]
        end
      end

      if script[2] then
        local s1t = script[1].t
        local sp = linePointFromXnorm(vec3(script[1]), vec3(script[2]), (timeOffset - s1t) / (script[2].t - s1t))
        script[1] = {x = sp.x, y = sp.y, z = sp.z, t = timeOffset}
      end
    end

    for _, s in ipairs(script) do
      s.t = s.t - timeOffset
    end

    if not script[2] then
      return
    end
  end

  local dir, up, pos

  -- Get initial position and orientation of vehicle at start of path (possibly time offset and/or time delayed)
  if script[1].dir then
    -- vehicle initial orientation vectors exist

    dir = vec3(script[1].dir)
    up = vec3(script[1].up or mapmgr.surfaceNormalBelow(vec3(script[1])))

    local frontPosRelOrig = obj:getOriginalFrontPositionRelative() -- original relative front position in the vehicle coordinate system (left, back, up)
    local vx = dir * -frontPosRelOrig.y
    local vz = up * frontPosRelOrig.z
    local vy = dir:cross(up) * -frontPosRelOrig.x
    pos = vec3(script[1]) - vx - vz - vy
    local dH = wheelToGroundDist(pos, dir, up)
    pos:setAdd(dH * up)
  else
    -- vehicle initial orientation vectors don't exist
    -- estimate vehicle orientation vectors from path and ground normal

    local p1 = vec3(script[1])
    local p1z0 = p1:z0()
    local scriptPosi = vec3()
    local k
    for i = 2, #script do
      scriptPosi:set(script[i].x, script[i].y, 0)
      if p1z0:squaredDistance(scriptPosi) > 0.2 * 0.2 then
        k = i
        break
      end
    end

    if k then
      local p2 = vec3(script[k])
      dir = p2 - p1; dir:normalize()
      up = mapmgr.surfaceNormalBelow(p1)

      local frontPosRelOrig = obj:getOriginalFrontPositionRelative() -- original relative front position in the vehicle coordinate system (left, back, up)
      local vx = dir * -frontPosRelOrig.y
      local vz = up * frontPosRelOrig.z
      local vy = dir:cross(up) * -frontPosRelOrig.x
      pos = p1 - vx - vz - vy
      local dH = wheelToGroundDist(pos, dir, up)
      pos:setAdd(dH * up)
    end
  end

  local rot = quatFromDir(dir:cross(up):cross(up), up)

  return pos, rot
end

local function calculateTarget()
  local scriptLen = #script

  if scriptLen >= 2 then
    local p1, p2 = vec3(script[1]), vec3(script[2])
    local prevPos = linePointFromXnorm(p1, p2, clamp(aiPos:xnormOnLine(p1, p2), 0, 1))
    local curPos = vec3()

    for i = 2, scriptLen do
      curPos:set(script[i].x, script[i].y, script[i].z)
      local diffLen = curPos:distance(prevPos)
      if diffLen >= targetLength then
        local scaledDiff = curPos - prevPos; scaledDiff:setScaled(targetLength / (diffLen + 1e-30))
        targetPos = prevPos + scaledDiff
        return
      end
      targetLength = targetLength - diffLen
      prevPos:set(curPos)
    end
  end

  targetPos = vec3(script[scriptLen])
end

local function calculateTimePoint(t)
  local scriptLen = #script
  if scriptLen == 1 then return vec3(script[1]), vec3(0, 0, 0) end
  if scriptLen >= 2 then
    if t < script[1].t then
      local p1 = vec3(script[1])
      return p1, p1, 0
    end
    if t > script[scriptLen].t then
      local p1 = vec3(script[scriptLen])
      return p1, p1, 1
    end

    for i = 1, scriptLen - 1 do
      local s1, s2 = script[i], script[i+1]
      if t >= s1.t and t <= s2.t then
        local p1, p2 = vec3(s1), vec3(s2)
        if i + 2 > scriptLen then
          local a = (t - s1.t) / max(s2.t - s1.t, 1e-30)
          return p1, p2, a
        else
          local s3 = script[i+2]
          local p3 = vec3(s3)
          local l1 = p2:distance(p1)
          local v1, a1 = velAccelFrom2dist(l1, p3:distance(p2) + l1, s2.t - s1.t, s3.t - s1.t)
          local dt = t - s1.t
          local a = clamp((v1 + 0.5 * a1 * dt) * dt / max(l1, 1e-30), 0, 1)
          return p1, p2, a
        end
      end
    end
  end
end

local function updateGFXrecord(dt)
  local pos = obj:getFrontPosition()
  local scriptLen = #script

  if scriptLen >= 2 then
    local s0, s1 = script[scriptLen], script[scriptLen - 1]
    local p0, p1 = vec3(s0), vec3(s1)

    local posline
    if prevDirPoint then
      posline = linePointFromXnorm(prevDirPoint, p1, pos:xnormOnLine(prevDirPoint, p1))
    else
      posline = linePointFromXnorm(p0, p1, pos:xnormOnLine(p0, p1))
    end

    local pospjlen = (pos - posline):projectToOriginPlane(obj:getDirectionVectorUp()):squaredLength()

    if pospjlen < 0.01 then
      local t2 = time - s1.t
      local l2 = pos:distance(p1)
      if not prevDirPoint then
        local l1 = p0:distance(p1)
        prevVel, prevAccel = velAccelFrom2dist(l1, l2, s0.t - s1.t, t2)
        prevDirPoint = p0
      end

      local pl2 = (prevVel + 0.5 * prevAccel * t2) * t2

      if abs(pl2 - l2) < 0.1 then
        scriptLen = scriptLen - 1
      else
        prevDirPoint = nil
      end
    else
      prevDirPoint = nil
    end
  end

  local v
  if recordSpeed then
    v = vec3(obj:getSmoothRefVelocityXYZ()):length()
  end

  script[scriptLen + 1] = {x = pos.x, y = pos.y, z = pos.z, t = time, v = v}
  time = time + dt
end

local function scriptStop(centerWheel, engageParkingbrake)
  if centerWheel == nil then
    centerWheel = true
    engageParkingbrake = true
  end

  if centerWheel then
    driveCar(0, 0, 0, engageParkingbrake and 1 or 0)
  else
    if engageParkingbrake then
      input.event("parkingbrake", 1, 2)
    end
  end

  if csvLog then
    local fileName
    if type(logDataToCSV) == 'string' then
      fileName = csvLog:write(logDataToCSV)
    else
      fileName = csvLog:write("scriptaiFollowTrajectory")
    end
    -- Write scriptai follow settings to csv
    csvLog = require('csvlib').newCSV("fileName", "externalForce", "speedDiffSmootherOutRate")
    csvLog:add(fileName, externalForce, speedDiffSmoother[true])
    csvLog:write("settings_"..fileName)

    csvLog = nil
    logDataToCSV = nil
  end

  script = {}
  M.updateGFX = nop
end

local function updateGFXfollow(dt)
  if followInitCounter > 0 then
    followInitCounter = followInitCounter - 1
    return
  end

  local scriptLen = #script
  if scriptLen == 0 then
    M.updateGFX = nop
    return
  end

  aiPos:set(obj:getFrontPosition())
  aiVel:set(obj:getVelocity())
  local aiVelLen = aiVel:length()
  local prevDirVec = aiDirVec
  aiDirVec = obj:getDirectionVector()

  repeat
    local s1, s2 = script[1], script[2] or script[1]
    local p1, p2 = vec3(s1), vec3(s2)
    local xnorm = aiPos:xnormOnLine(p1, p2)
    if (p1:squaredDistance(p2) < 0.0025 and s2.t < time) or xnorm > 1 or (xnorm < 0 and s2.t < time and aiVel:dot(p2) < aiVel:dot(p1)) then
      if s1.forceSpeed and not s2.forceSpeed then s2.forceSpeed = s1.forceSpeed end
      table.remove(script, 1)
      scriptLen = scriptLen - 1
    else
      break
    end
  until scriptLen == 0

  local s1, s2, s3 = script[1], script[2] or script[1], script[3]
  local p1, p2 = vec3(s1), vec3(s2)
  if s3 == nil and s2 ~= nil then
    local p3 = 2 * p2 - p1
    s3 = {x = p3.x, y = p3.y, z = p3.z, t = 2 * s2.t - s1.t}
  end

  local p3 = vec3(s3)

  if scriptLen < 3 then
    -- finished
    if loopCounter > 0 then
      loopCounter = loopCounter - 1
    end

    if loopCounter ~= 0 then
      M.startFollowing(inScript, inScript.loopTimeOffset, loopCounter, loopType)
      return
    end

    if scriptLen == 0 or
        (scriptLen == 1 and aiPos:squaredDistance(p1) < 0.25) or
        (scriptLen == 2 and aiPos:xnormOnLine(p1, p2) > 0 and aiPos:squaredDistance(p2) < 0.36 and aiVel:squaredLength() < 0.25) then
      ai.stopFollowing()
      return
    end
  end

  calculateTarget() -- sets upvalues targetPos and targetLength
  local targetPosOnLine = targetPos

  local t1, t2, t3 = s1.t, s2.t, s3.t
  local p2p1 = p2 - p1
  local l1 = p2p1:length()

  -- Estimate required velocity and vehicle to script lag/lead
  local l2 = p3:distance(p2) + l1
  local dt1 = t2 - t1
  local v1, a1 = velAccelFrom2dist(l1, l2, dt1, t3 - t1)
  local v2 = v1 + a1 * dt1
  v1, v2 = max(0, v1), max(0, v2)
  local aiXnormOnSeg = clamp(aiPos:xnormOnLine(p1, p2), 0, 1)
  if a1 == 0 then
    scriptTime = lerp(t1, t2, aiXnormOnSeg)
  else
    local s = aiXnormOnSeg * l1
    local vs = sqrt(max(0, 2 * a1 * s + v1 * v1))
    local ts = (vs - v1) / a1
    if ts >= 0 and ts <= dt1 then
      scriptTime = ts + t1
    else
      scriptTime = lerp(t1, t2, aiXnormOnSeg)
    end
  end
  local reqVel = dt1 == 0 and v2 or lerp(v1, v2, (scriptTime - t1) / dt1) -- required velocity
  local timeDiff = scriptTime - time -- negative value indicates vehicle is lagging script
  if csvLog then csvLog:add(time, aiPos.x, aiPos.y, aiPos.z, timeDiff, speedDiffSmoother[true]) end

  local aiUpVec = obj:getDirectionVectorUp()
  local aiLeftVec = aiDirVec:cross(aiUpVec):normalized()
  local turnleft = p2p1:cross(aiUpVec):normalized()

  local targetaivec = targetPos - aiPos
  local targetai = targetaivec:dot(aiDirVec)
  aiSpeed = aiVel:dot(aiDirVec) * sign(targetai)

  -- oversteer
  local noOversteerCoef = 1
  if aiVelLen > 1 then
    local leftVel = aiLeftVec:dot(aiVel)
    if leftVel * aiLeftVec:dot(targetPosOnLine - aiPos) > 0 then
      local dirDiff = -math.asin(aiLeftVec:dot(targetaivec:normalized()))
      local rotVel = min(1, (prevDirVec:projectToOriginPlane(aiUpVec):normalized() - aiDirVec):length() * dt * 10000)
      noOversteerCoef = max(0, 1 - abs(leftVel * aiVelLen * 0.05) * min(1, dirDiff * dirDiff * aiVelLen * 6) * rotVel)
    end
  end

  -- deviation
  local tp2
  if targetPosOnLine:xnormOnLine(p1, p2) > 1 then
    tp2 = (targetPosOnLine - p2):normalized():dot(turnleft)
  else
    tp2 = (p3 - p2):normalized():dot(turnleft)
  end

  local deviation = (aiPos - p2):dot(turnleft)
  deviation = sign(deviation) * min(5, abs(deviation))
  local reldeviation = sign(tp2) * deviation

  posError = aiPos:distance(linePointFromXnorm(p1, p2, aiXnormOnSeg)) * sign(deviation)

  -- target bending
  local grleft = aiLeftVec:dot(obj:getGravityVector())
  if deviation * grleft > 0 then
    local carturn = turnleft:dot(aiLeftVec)
    targetPos = targetPosOnLine - aiLeftVec * sign(deviation) * min(5, abs(0.01 * deviation * grleft * aiVelLen * min(1, carturn * carturn)))
  end

  local targetDirVec = (targetPos - aiPos):normalized()
  local dirDiff = -math.asin(aiLeftVec:dot(targetDirVec))

  -- understeer
  local steerCoef = reldeviation * min(aiSpeed * aiSpeed, abs(aiSpeed)) * min(1, dirDiff * dirDiff * 4) * 0.2
  local understeerCoef = max(0, -steerCoef) * min(1, abs(aiVel:dot(p2p1:normalized()) * 3))
  local noUndersteerCoef = max(0, 1 - understeerCoef)
  targetLength = max(aiVelLen * 0.65, 3)

  local extForceVel = 0

  -- apply external force
  if externalForce ~= 0 and not hasCollided then
    local wheelTouching = false
    local lwheels = wheels.wheels
    for i = 0, tableSizeC(lwheels) - 1 do
      local wd = lwheels[i]
      if not wd.isBroken then
        if wd.downForceRaw > 0 then
          wheelTouching = true
        end
      end
    end

    if wheelTouching then
      hasCollided = obj:isCollidingWithObject()
      local p1, p2, a = calculateTimePoint(time)
      if p1 and p2 and a then
        local targetPos = lerp(p1, p2, a)
        local up = obj:getDirectionVectorUp()
        local vel = obj:getClusterVelocityWithoutWheels(v.data.refNodes and v.data.refNodes[0].cid or 0)
        local fwd = (p2 - p1):normalized()
        local posAccel = targetPos - aiPos
        posAccel = posAccel + (reqVel * fwd - vel) * dt / clamp(posAccel:length() / max(1e-30, vel:dot(posAccel:normalized())), dt, 1)
        local posAccelL = posAccel:dot(fwd)
        local posAccelT = posAccel - posAccelL * fwd
        posAccel = min(externalForce * 5, 1) * posAccelL * fwd + (externalForce * abs(a - 0.5) * 2) * posAccelT
        posAccel = posAccel:projectToOriginPlane(up) / (dt * dt)
        thrusters.applyAccel(posAccel, dt)
        extForceVel = posAccel:dot(fwd) * dt
        -- if s2.t-time<=dt then print((targetPos - aiPos):projectToOriginPlane(up):length()) end -- measure error
      end
    end
  end

  -- reduce time spring when in understeer
  local dif = (reqVel + extForceVel - aiSpeed) * 3 + clamp(-timeDiff * 6, -1, 1.2) * noUndersteerCoef
  if dif <= 0 then
    speedDiffSmoother:set(dif)
  end
  local curthrottle = clamp(speedDiffSmoother:get(dif, dt), -1, 1)

  -- stay put when starting with negative offset
  local pbrake
  if (scriptTime <= 0 or curthrottle < 0) and max(aiVelLen, aiSpeed) < 0.5 then
    curthrottle = 0
    pbrake = 1
  else
    pbrake = 0
    if s1.forceSpeed then
      local forceSpeedVec = p2 - aiPos;
      forceSpeedVec = forceSpeedVec:projectToOriginPlane(obj:getDirectionVectorUp())
      forceSpeedVec:setScaled(s1.forceSpeed / (forceSpeedVec:length() + 1e-30))
      thrusters.applyVelocity(forceSpeedVec, dt)
      s1.forceSpeed = nil
    end
  end

  -- understeer guard
  if reldeviation < 0 and aiVelLen > 1 then
    curthrottle = curthrottle * noUndersteerCoef
    curthrottle = max(curthrottle, min(0, -1 + understeerCoef * understeerCoef)) -- cut off brake
  else
    if curthrottle > 0 then
      curthrottle = min(1, curthrottle * (1 + abs(deviation))) -- push some more when on the inside of the turn
    end
  end

  local throttle, brake = clamp(curthrottle, 0, 1), clamp(-curthrottle, 0, 1)
  brake = min(1, max(0, brake - 0.1) / (1 - 0.1)) -- reduce brake flutter

  -- print(timeDiff)
  -- print(noUndersteerCoef..','..noOversteerCoef)

  -- wheel speed
  local absAiSpeed = abs(aiSpeed)

  if absAiSpeed > 0.05 then
    if sensors.gz <= 0 then
      local totalSlip = 0
      local propSlip = 0
      local totalDownForce = 0
      local lwheels = wheels.wheels
      for i = 0, tableSizeC(lwheels) - 1 do
        local wd = lwheels[i]
        if not wd.isBroken then
          local lastSlip = wd.lastSlip
          local downForce = wd.downForceRaw
          totalSlip = totalSlip + lastSlip * downForce
          totalDownForce = totalDownForce + downForce
          if wd.isPropulsed then
            propSlip = max(propSlip, lastSlip)
          end
        end
      end

      absAiSpeed = max(absAiSpeed, 3)

      totalSlip = totalSlip * 4 / (totalDownForce + 1e-25)

      -- abs
      brake = brake * square(max(0, absAiSpeed - totalSlip) / absAiSpeed)

      -- tcs, oversteer
      throttle = throttle * min(noOversteerCoef, max(0, absAiSpeed - propSlip * propSlip) / absAiSpeed)
    else
      brake = 0
      throttle = 0
    end
  end

  -- reverse
  if targetai < 0 then
    local targetailen = targetaivec:length()
    if targetai / (targetailen + 1e-30) < -0.5 or targetailen < 8 then
      dirDiff = -dirDiff
      throttle, brake = brake, throttle
    end
  end

  if aiSpeed > 4 and aiSpeed < 30 and abs(dirDiff) > 0.8 and brake == 0 then
    pbrake = 1
  end

  driveCar(dirDiff, throttle, brake, pbrake)

  prevVel, prevAccel = v1, a1
  time = time + dt
end

local function startRecording(recordSpeed)
  table.clear(script)
  time = 0
  setRecordSpeed(recordSpeed)
  M.updateGFX = updateGFXrecord
  prevDirPoint = nil

  local dir, up = obj:getDirectionVector(), obj:getDirectionVectorUp()
  initConditions.dir = {x = dir.x, y = dir.y, z = dir.z}
  initConditions.up = {x = up.x, y = up.y, z = up.z}
end

local function stopRecording()
  --print(">>> AI.stopRecording")
  M.updateGFX = nop

  if script[1] ~= nil then
    script[1].dir = initConditions.dir
    script[1].up = initConditions.up
  end
  return {path = script}
  -- return script
end

local function startFollowing(_inScript, _timeOffset, _loopCounter, _loopType, _externalForce)
  -- print(">>> AI.startFollowing: " .. dumps(inScript))
  -- inScript = testrec
  inScript = _inScript

  if inScript == nil then
    return
  end

  if inScript.path then
    script = inScript.path
  else
    script = inScript
  end

  script = deepcopy(script)

  if not script[2] then
    return
  end

  -- _externalForce = true -- debug
  if _externalForce == nil then _externalForce = inScript.externalForce end
  externalForce = _externalForce and (type(_externalForce) == 'number' and _externalForce or 0.0004) or 0
  hasCollided = false

  loopType = _loopType or "alwaysReset"
  local totalLoopCount = inScript.loopCount or 1
  loopCounter = _loopCounter or totalLoopCount

  local timeOffset = _timeOffset or inScript.timeOffset

  if timeOffset and timeOffset ~= 0 then
    if timeOffset > 0 then
      -- find index k such that script[k].t < timeOffset and script[k+1].t >= timeOffset
      local k = #script
      for i = 2, k do
        if script[i].t >= timeOffset then
          k = i - 1
          break
        end
      end
      -- Remove all elements up to and including k - 1
      if k > 1 then
        for i = 1, #script do
          script[i] = script[(i-1)+k]
        end
      end

      if script[2] then
        local s1t = script[1].t
        local sp = linePointFromXnorm(vec3(script[1]), vec3(script[2]), (timeOffset - s1t) / (script[2].t - s1t))
        script[1] = {x = sp.x, y = sp.y, z = sp.z, t = timeOffset}
      end
    end

    for _, s in ipairs(script) do
      s.t = s.t - timeOffset
    end

    if not script[2] then
      return
    end
  end

  -- Add start delay to script timestamps
  local startDelay = inScript.startDelay or 0
  if startDelay > 0 then
    table.insert(script, 1, deepcopy(script[1]))
    script[2].dir, script[2].up = nil, nil
    for i = 2, #script do
      script[i].t = script[i].t + startDelay
    end
  end

  local dir, up, pos

  -- Get initial position and orientation of vehicle at start of path (possibly time offset and/or time delayed)
  if script[1].dir then
    -- vehicle initial orientation vectors exist

    dir = vec3(script[1].dir)
    up = vec3(script[1].up or mapmgr.surfaceNormalBelow(vec3(script[1])))

    local frontPosRelOrig = obj:getOriginalFrontPositionRelative() -- original relative front position in the vehicle coordinate system (left, back, up)
    local vx = dir * -frontPosRelOrig.y
    local vz = up * frontPosRelOrig.z
    local vy = dir:cross(up) * -frontPosRelOrig.x
    pos = vec3(script[1]) - vx - vz - vy
    local dH = wheelToGroundDist(pos, dir, up)
    pos:setAdd(dH * up)
  else
    -- vehicle initial orientation vectors don't exist
    -- estimate vehicle orientation vectors from path and ground normal

    local p1 = vec3(script[1])
    local p1z0 = p1:z0()
    local scriptPosi = vec3()
    local k
    for i = 2, #script do
      scriptPosi:set(script[i].x, script[i].y, 0)
      if p1z0:squaredDistance(scriptPosi) > 0.2 * 0.2 then
        k = i
        break
      end
    end

    if k then
      local p2 = vec3(script[k])
      dir = p2 - p1; dir:normalize()
      up = mapmgr.surfaceNormalBelow(p1)

      local frontPosRelOrig = obj:getOriginalFrontPositionRelative() -- original relative front position in the vehicle coordinate system (left, back, up)
      local vx = dir * -frontPosRelOrig.y
      local vz = up * frontPosRelOrig.z
      local vy = dir:cross(up) * -frontPosRelOrig.x
      pos = p1 - vx - vz - vy
      local dH = wheelToGroundDist(pos, dir, up)
      pos:setAdd(dH * up)
    end
  end

  followInitCounter = 3
  prevVel = 0
  time = 0
  scriptTime = 0
  posError = 0
  speedDiffSmoother:set(0)

  -- Set vehicle position and orientation at start of path (possibly time offset and/or time delayed)
  if dir then
    if loopType == "alwaysReset" or (loopType == "startReset" and loopCounter == totalLoopCount) then
      obj:requestReset(RESET_PHYSICS)
      obj:queueGameEngineLua("getObjectByID(" .. tostring(obj:getId()) .. "):resetBrokenFlexMesh()")
      local rot = quatFromDir(dir:cross(up):cross(up), up)
      obj:queueGameEngineLua("getObjectByID(" .. obj:getId() .. "):autoplace(false);vehicleSetPositionRotation(" .. obj:getId() .. "," .. pos.x .. "," .. pos.y .. "," .. pos.z .. "," .. rot.x .. "," .. rot.y .. "," .. rot.z .. "," .. rot.w .. ")")
    end

    if controller.mainController then
      controller.mainController.setGearboxMode("arcade")
    end

    wheels.setABSBehavior("arcade")

    if logDataToCSV then
      csvLog = require('csvlib').newCSV("time", "posX", "posY", "posZ", "timeDiff", "speedDiffSmootherOutRate")
    end

    M.updateGFX = updateGFXfollow
  end
end

local function debugDraw()
  local debugDrawer = obj.debugDrawProxy

  if M.debugMode == "all" or M.debugMode == "target" then
    if M.updateGFX == updateGFXfollow then
      debugDrawer:drawSphere(0.2, vec3(targetPos), color(0, 0, 255, 255))
    end
  end

  if M.debugMode == "all" or M.debugMode == "path" then
    for _, s in ipairs(script) do
      debugDrawer:drawSphere(0.2, vec3(s), color(255, 0, 0, 255))
    end
  end
end

local function scriptState()
  if M.updateGFX == updateGFXrecord then
    return {status = "recording", time = time}
  elseif M.updateGFX == updateGFXfollow then
    return {status = "following", scriptTime = scriptTime, time = time, endScriptTime = script[#script].t, posError = posError, targetPos = vec3(targetPos), startDelay = (inScript.startDelay or nil)}
  end
  return nil
end

local function isDriving()
  return M.updateGFX == updateGFXfollow
end

M.updateGFX = nop
M.startRecording = startRecording
M.stopRecording = stopRecording
M.startFollowing = startFollowing
M.stopFollowing = scriptStop
M.scriptStop = scriptStop
M.debugDraw = debugDraw
M.scriptState = scriptState
M.isDriving = isDriving
M.debugMode = "all"
M.getInitialSpawnPositionOrientation = getInitialSpawnPositionOrientation
M.setSpeedDiffSmootherOutRate = setSpeedDiffSmootherOutRate
M.setLogDataToCSV = setLogDataToCSV
M.wheelToGroundDist = wheelToGroundDist

return M
