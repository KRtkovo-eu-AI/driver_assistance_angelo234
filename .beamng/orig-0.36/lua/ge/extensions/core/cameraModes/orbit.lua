-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- ORBIT CAMERA
local collision = require('core/cameraModes/collision')

local C = {}
C.__index = C
local vecY = vec3(0,1,0)
local vecZ = vec3(0,0,1)
local lookBackVec = vec3(0,-0.3,0)

local upperSpeedThreshold = 10
local lowerSpeedThreshold = 1
local pitchTransitionDelay = 1.5

local function getRot(base, vf, vz)
  local nyn = vf:normalized()
  local nxn = nyn:cross(vz):projectToOriginPlane(vecZ):normalized()
  local nzn = nxn:cross(nyn):normalized()
  local nbase = base:normalized()
  return math.atan2(-nbase:dot(nxn), nbase:dot(nyn)), math.asin(nbase:dot(nzn))
end

function C:init()
  self.target = false
  self.camLastTargetPos = vec3()
  self.camLastTargetPos2 = vec3()
  self.camLastPos = vec3()
  self.camLastPos2 = vec3()
  self.camLastPosPerp = vec3()
  self.camVel = vec3()
  self.cameraResetted = 3
  self.lockCamera = false
  self.orbitOffset = vec3()
  self.preResetPos = vec3(1e+300, 0, 0)
  self.smoothedPitchOffset = newTemporalSigmoidSmoothing(2, 2, 2, 2)
  self.dynamicPitchQuat = quat()
  self.targetCenter = vec3(0, 0, 0)
  self.targetLeft = vec3(0, 0, 0)
  self.targetBack = vec3(0, 0, 0)
  self.configChanged = false

  self.collision = collision()
  self.collision:init()

  self:onVehicleCameraConfigChanged()
  self:onSettingsChanged()
  self:reset()
end

function C:calculateDynamicPitchLimit(data, targetPos)
  if not self.dynamicPitchLimit then
    local vehId = data.veh:getID()
    local veh = getObjectByID(vehId)
    local bottomRear = vec3()
    local bbHalfExtents = vec3()
    local bbCenter = vec3()

    bbHalfExtents:set(be:getObjectOOBBHalfExtentsXYZ(vehId))
    bottomRear:setAdd(vec3(0,-bbHalfExtents.y,0))
    bottomRear:setAdd(vec3(0,0,-bbHalfExtents.z))
    bbCenter:set(be:getObjectOOBBCenterXYZ(vehId))

    -- rotate targetPos around bbCenter by the inverse of the vehicle rotation
    local vehRot = quat(0,0,1,0) * quat(veh:getClusterRotationSlow(veh:getRefNodeId()))
    local localTargetPos = vehRot:inversed() * (targetPos - bbCenter)

    -- convert bottomRear to target pos coordinate system
    bottomRear:setSub(localTargetPos)

    -- defaultCamPos is relative to targetPos
    local defaultCamPos = vec3()

    defaultCamPos:set(
    math.sin(math.rad(self.defaultRotation.x)) * math.cos(math.rad(self.defaultRotation.y))
    , -math.cos(math.rad(self.defaultRotation.x)) * math.cos(math.rad(self.defaultRotation.y))
    , -math.sin(math.rad(self.defaultRotation.y)))
    defaultCamPos = defaultCamPos * self.defaultDistance

    local bottomRearDir = bottomRear - defaultCamPos
    local targetDir = -defaultCamPos
    local angleCamTargetCamBottomRear = math.acos(bottomRearDir:cosAngle(targetDir))

    self.dynamicPitchLimit = math.max(math.rad(self.fov/2) - angleCamTargetCamBottomRear, 0)
  end
  return self.dynamicPitchLimit
end

function C:onVehicleCameraConfigChanged()
  self.configChanged = true
  if self.defaultRotation == nil then
    self.defaultRotation = vec3(0, -17, 0)
  end
  self.defaultRotation = vec3(self.defaultRotation)
  self.offset = vec3(self.offset)
  if not self.camRot then self.camRot = vec3(self.defaultRotation) end
  self.camLastRot = vec3(math.rad(self.camRot.x), math.rad(self.camRot.y), 0)
  self.camMinDist = self.distanceMin or 3
  self.camMaxDist = (self.distanceMin or 5) * 10
  self.camDist = self.distance or 5
  self.camLastDist = self.distance or 5
  self.defaultDistance = self.distance or 5
  self.mode = self.mode or 'ref'
  self.skipFovModifier = self.skipFovModifier or false
  self.smoothedPitchOffset:set(0)
  self.dynamicPitchQuat:setFromEuler(0, 0, 0)
  self.aboveCamPitchThreshold = false
  self.dynamicPitchLimit = nil
  self.fov = self.fov or 65
end

function C:onSettingsChanged()
  core_camera.clearInputs() --TODO is this really necessary?
  self.fovModifier = settings.getValue('cameraOrbitFovModifier')
  self.relaxation = settings.getValue('cameraOrbitRelaxation') or 3
  self.maxDynamicFov = settings.getValue('cameraOrbitMaxDynamicFov') or 35
  self.maxDynamicPitch = math.rad(settings.getValue('cameraOrbitMaxDrivingPitch') or 0)
  self.maxDynamicOffset = settings.getValue('cameraOrbitMaxDynamicOffset') or 0
  self.smoothingEnabled = settings.getValue('cameraOrbitSmoothing', true)
end

function C:onVehicleSwitched()
  self.collision:onVehicleSwitched()
end

function C:reset()
  if self.cameraResetted == 0 then
    self.preResetPos = vec3(self.camLastTargetPos2)
    self.cameraResetted = 3
    self.collision:init()
  end
end

function C:setRotation(rot)
  self.camRot = vec3(rot)
end

function C:setFOV(fov)
  self.fov = fov
end

function C:setOffset(v)
  self.orbitOffset = vec3(v)
end

function C:setRefNodes(centerNodeID, leftNodeID, backNodeID, dynamicFovRearNodeID)
  self.refNodes = self.refNodes or {}
  self.refNodes.ref = centerNodeID
  self.refNodes.left = leftNodeID
  self.refNodes.back = backNodeID
  self.rearNodeID = dynamicFovRearNodeID -- specifies which area of the vehicle will have constant screen-size during dolly zoom effect (dynamic FOV effect)
end

-- params in global coords
function C:setRef(center, left, back)
  local prevTarget = self.target
  self.target = center ~= nil and true or false
  self.targetCenter = center
  self.targetLeft = left
  self.targetBack = back
  if self.target ~= prevTarget then self:reset() end
end

function C:setTargetMode(targetMode, camBase)
  self.mode = targetMode
  self.camBase = camBase
end

function C:setDefaultDistance(d)
  self.defaultDistance = d
end

function C:setDistance(d)
  self.camDist = d
end

function C:setMaxDistance(d)
  self.camMaxDist = d
end

function C:setDefaultRotation(rot)
  self.defaultRotation = rot
end

function C:setSkipFovModifier(skip)
  self.skipFovModifier = skip
end

local function isManualRotation()
  return MoveManager.yawRelative ~= 0 or MoveManager.pitchRelative ~= 0 or MoveManager.yawRight > 0.01 or MoveManager.yawLeft > 0.01 or MoveManager.pitchDown > 0.01 or MoveManager.pitchUp > 0.01
end

local ref, left, back, dirxy = vec3(), vec3(), vec3(), vec3()

local nx, ny, nz = vec3(), vec3(), vec3()

local targetPos, camdir, dir = vec3(), vec3(), vec3()

local lastCamPointVec, lastCamLastPerp, moveDir = vec3(), vec3(), vec3()

local rot, calculatedCamPos, camPos, updir, rear = vec3(), vec3(), vec3(), vec3(), vec3()

local resultPos, resultRot, tempQuatB = vec3(), quat(), quat()

local bbCenter, bbHalfAxis1 = vec3(), vec3()

function C:update(data)
  data.res.collisionCompatible = true
  if self.target then
    ref:set(self.targetCenter)
    left:set(self.targetLeft)
    back:set(self.targetBack)
  else
    if self.refNodes then
      ref:set(data.veh:getNodePositionXYZ(self.refNodes.ref))
      left:set(data.veh:getNodePositionXYZ(self.refNodes.left))
      back:set(data.veh:getNodePositionXYZ(self.refNodes.back))
    end
  end

  -- reset cam
  if self.cameraResetted == 3 then
    if self.lockCamera == false or self.configChanged == false then
      -- if a reload hasn't just happened
      self.camRot = vec3(self.defaultRotation)
      self.camDist = self.defaultDistance
      self.lockCamera = false
      core_camera.clearInputs()
    else
      self.cameraResetted = 0
    end
    self.configChanged = false
  end
  if data.teleported then
    self.smoothedPitchOffset:set(0)
    self.dynamicPitchQuat:setFromEuler(0, 0, 0)
  end

  -- calculate the camera offset: rotate with the vehicle
  nx:set(push3(left) - ref)
  ny:set(push3(back) - ref)
  nz:set(push3(nx):cross(ny))
  ny:set(push3(nx):cross(-push3(nz)))
  nx:normalize()
  ny:normalize()
  nz:normalize()

  if not self.camBase or self.cameraResetted > 0 then
    -- this needs to happen here as on init the node data is not existing yet
    if self.offset and self.offset.x and nx:length() ~= 0 and ny:length() ~= 0 then
      self.camBase = vec3(self.offset.x, self.offset.y, self.offset.z)
      self.camOffset2 = nx * self.camBase.x + ny * self.camBase.y + nz * self.camBase.z
      if self.target then
        targetPos = data.pos + ref
      else
        targetPos = data.pos + ref + self.camOffset2
      end
    elseif self.camOffset2 then
      targetPos = data.pos + ref + self.camOffset2
      self.camOffset2 = nil -- we only use previous offset for only one frame when needed
    else
      targetPos = data.veh:getBBCenter() - (data.pos + ref)
    end
  else
    if not self.camOffset2 then self.camOffset2 = vec3() end
    self.camOffset2:set(push3(nx) * self.camBase.x + push3(ny) * self.camBase.y + push3(nz) * self.camBase.z)
    targetPos:set(push3(data.pos) + ref + self.camOffset2)
  end

  local yawDif = 0.1*(MoveManager.yawRight - MoveManager.yawLeft)
  local pitchDif = 0.1*(MoveManager.pitchDown - MoveManager.pitchUp)
  dir:set(vecY)

  if self.cameraResetted == 0 then
    if self.lockCamera == true then
      camdir:set(push3(self.camLastTargetPos) - self.camLastPos2)
      if sign((push3(targetPos) - self.camLastTargetPos):dot(camdir)) < 0 then
        self.camRot.x = self.camRot.x + 180
        self.camLastRot.x = self.camLastRot.x + math.pi
        self.camLastPos2:set(push3(targetPos) + camdir)
        self.camLastPosPerp:set(push3(vecZ):cross(camdir):normalized() * (self.relaxation * -0.8) + targetPos)
      end
    end
  else
    self.lockCamera = false
    if self.cameraResetted >= 1 then
      dir = ref - back
      if targetPos:distance(self.preResetPos) < 4 then
        -- smoothly rotate back to default
        local rx, ry = getRot((self.camLastTargetPos - self.camLastPos), dir, nz)
        self.camLastRot.x = rx
        self.camLastRot.y = ry
        self.camRot.x = self.defaultRotation.x
      else
        self.camRot:set(self.defaultRotation)
        self.camLastRot:set(math.rad(self.camRot.x), math.rad(self.camRot.y) * 1.5, 0)
      end
      self.camLastPos2 = targetPos - dir
      self.camLastPosPerp = vecZ:cross(dir):normalized() * (self.relaxation * -0.8) + targetPos
      dir:normalize()
    end
  end

  local maxRot = 4.5
  if (math.abs(yawDif) + math.abs(pitchDif) + math.abs(MoveManager.yawRelative) + math.abs(MoveManager.pitchRelative) > 0) then
    maxRot = 1000
  end

  -- mouse rotation
  local dtfactor = data.dt * 1000
  local mouseYaw = sign(MoveManager.yawRelative) * math.min(math.abs(MoveManager.yawRelative * 10), maxRot * data.dt) + yawDif * dtfactor
  local mousePitch = sign(-MoveManager.pitchRelative) * math.min(math.abs(MoveManager.pitchRelative * 10), maxRot * data.dt) + pitchDif * dtfactor
  if mouseYaw ~= 0 or mousePitch ~= 0 then
    if self.cameraResetted == 0 then
      self.lockCamera = true
    end
    self.camRot.x = self.camRot.x - mouseYaw
    self.camRot.y = self.camRot.y - mousePitch
    --self.camRot.z = self.camRot.z + 300*data.dt*(MoveManager.rollRight - MoveManager.rollLeft)
  end

  self.camRot.y = math.min(math.max(self.camRot.y, -85), 85)

  -- make sure the rotation is never bigger than 2 PI
  if self.camRot.x > 180 then
    self.camRot.x = self.camRot.x - 360
  elseif self.camRot.x < -180 then
    self.camRot.x = self.camRot.x + 360
  end

  if self.camLastRot.x > math.pi then
    self.camLastRot.x = self.camLastRot.x - math.pi * 2
  elseif self.camLastRot.x < -math.pi then
    self.camLastRot.x = self.camLastRot.x + math.pi * 2
  end

  -- If the camera is colliding with something, dont increase the camDist
  local zoomChange = MoveManager.zoomIn - MoveManager.zoomOut
  local newCamDist
  local adjustedSpeed = data.fastSpeedModifier and data.speed * 3 or data.speed
  if zoomChange ~= 0 and self.collision:collidingCamDistance() then
    if zoomChange < 0 then
      newCamDist = self.collision:collidingCamDistance() + zoomChange * dtfactor * adjustedSpeed * 0.0001 * core_camera.getFovDeg()
    else
      newCamDist = self.camDist
    end
  else
    newCamDist = self.camDist + zoomChange * dtfactor * adjustedSpeed * 0.0001 * core_camera.getFovDeg()
  end
  self.camDist = clamp(newCamDist, self.camMinDist, self.camMaxDist or math.huge)

  if nx:squaredLength() == 0 or ny:squaredLength() == 0 then
    data.res.pos = data.pos
    data.res.rot = quatFromDir(vecY, vecZ)
    return false
  end

  if self.cameraResetted ~= 1 then
    lastCamPointVec:set(push3(targetPos) - self.camLastPos2)
    lastCamLastPerp:set(push3(self.camLastPosPerp) - targetPos)
    if lastCamPointVec:length() < self.relaxation and lastCamLastPerp:length() > self.relaxation * 0.8 then
      moveDir:set(push3(targetPos) - self.camLastTargetPos); moveDir:normalize()
      if math.abs(push3(lastCamPointVec):normalized():dot(moveDir)) > math.abs(push3(lastCamLastPerp):normalized():dot(moveDir)) then
        self.camLastPos2:set(push3(lastCamPointVec):cross(lastCamLastPerp):cross(lastCamLastPerp):normalized() + targetPos)
        lastCamPointVec:set(push3(targetPos) - self.camLastPos2)
      end
    end
    dir:set(lastCamPointVec)
    dir:normalize()

    -- flatten the rotation plane when camera moves perpendicularly
    dirxy:set(dir.x, dir.y, 0)
    local dirxylen = dirxy:length()
    local coef = math.sqrt(math.max(0, 1 - dirxylen))
    dir:set(push3(dir) * math.max(0, 1 - coef) + push3(dirxy) * (coef / (dirxylen + 1e-30)))
    dir:normalize()
  end

  lastCamPointVec:set(push3(self.camLastPos2) - targetPos)
  self.camLastPos2:set(push3(lastCamPointVec) * (self.relaxation / (lastCamPointVec:length() + 1e-30)) + targetPos)

  rot:set(math.rad(self.camRot.x), math.rad(self.camRot.y), math.rad(self.camRot.z))

  -- smoothing
  local dist = self.camDist
  if self.smoothingEnabled then
    local ratio = 1 / (data.dt * 8)
    local srdif = -sign(self.camLastRot.x - rot.x)
    if math.abs(self.camLastRot.x + srdif * 2 * math.pi - rot.x) < math.abs(self.camLastRot.x - rot.x) then
      self.camLastRot.x = self.camLastRot.x + srdif * 2 * math.pi
    end
    local rotxDiff = (1 / (ratio + 1) * rot.x + (ratio / (ratio + 1)) * self.camLastRot.x) - self.camLastRot.x
    rot.x = self.camLastRot.x + sign(rotxDiff) * math.min(math.abs(rotxDiff), maxRot * data.dt)
    rot.y = 1 / (ratio + 1) * rot.y + (ratio / (ratio + 1)) * self.camLastRot.y
    dist = 1 / (ratio + 1) * self.camDist + (ratio / (ratio + 1)) * self.camLastDist
  end

  -- find where the rear of the vehicle is. this is roughly(*) the vehicle area that will occupy the same size in screen space, no matter the speed.
  -- (*) the camera is usually not right behind the car, but behind+higher: this angle is ignored for simplicity. The bounding box is usually a bit too big anyway, so the end result is close enough that we can ignore this asterisk (*) in our calculations. In the same way, if the user rotates the camera left or right, we ignore that horizontal angle too (we always use the same refToRear distance)
  if self.rearNodeID then
    rear:set(data.veh:getNodePositionXYZ(self.rearNodeID))
    rear:setAdd(data.pos)
  else
    local vehId = data.veh:getID()
    bbCenter:set(be:getObjectOOBBCenterXYZ(vehId))
    bbHalfAxis1:set(be:getObjectOOBBHalfAxisXYZ(vehId, 1))
    rear:set(bbCenter)
    rear:setAdd(bbHalfAxis1)
  end

  -- compute how wide the rear of the car is (in screen space) when using the jbeam config (self.camDist). This 'originalWidth' will be preserved in screen space, no matter the FOV we end up applying
  local refToRear = rear:distance(targetPos)
  local hdegToRad = math.pi/180 * 0.5

  local fov = self.fov
  local fovdistDiff = 0
  local fovModifier = self.skipFovModifier and 0 or self.fovModifier

  if (not self.target) and self.fov and fovModifier and self.maxDynamicFov and self.camDist then
    -- compute how much more FOV we're going to add depending on speed (from zero up to self.maxDynamicFov)
    fov = self.fov + fovModifier + self.maxDynamicFov * (math.min(1, data.vel:length()/130))

    -- apply final field of view
    -- compute and apply the camera distance that will preserve the originalWidth
    fovdistDiff = (self.camDist - refToRear) * (math.tan((self.fov+fovModifier) * hdegToRad) / math.tan(fov * hdegToRad) - 1)
  end

  local rotB = rot
  local dirB = dir

  if data.lookBack then
    rotB = lookBackVec
    dirB = back - ref
  end

  calculatedCamPos:set(
    math.sin(rotB.x) * math.cos(rotB.y)
    , -math.cos(rotB.x) * math.cos(rotB.y)
    , -math.sin(rotB.y))
  tempQuatB:setFromDir(dirB)
  calculatedCamPos:setRotate(tempQuatB)
  calculatedCamPos:setScaled(dist + fovdistDiff)

  camPos:set(push3(calculatedCamPos) + targetPos + self.orbitOffset)
  updir:setCross(vecZ, dir)
  self.camLastPosPerp:set(push3(updir) * (self.relaxation * -0.8 / (updir:length() + 1e-30)) + targetPos)

  self.camLastTargetPos2:set(self.camLastTargetPos)
  self.camLastTargetPos:set(targetPos)
  self.camVel:set((push3(camPos) - self.camLastPos) / data.dt)
  self.camLastPos:set(camPos)
  self.camLastRot:set(rot)
  self.camLastDist = dist
  self.cameraResetted = math.max(self.cameraResetted - 1, 0)

  if isManualRotation() then
    self.timeSinceManualRotation = 0
  else
    self.timeSinceManualRotation = self.timeSinceManualRotation and self.timeSinceManualRotation + data.dt or 0
  end

  if self.timeSinceManualRotation > 1 then
    -- Handle camera pitch threshold logic
    if self.aboveCamPitchThreshold then
      if data.vel:length() < lowerSpeedThreshold then
        -- Set initial timer value based on pitch offset
        local startTime = self.smoothedPitchOffset:value() == 1 and pitchTransitionDelay or 0
        self.belowThresholdTimer = (self.belowThresholdTimer or startTime) - data.dt
        -- Reset pitch threshold when timer expires
        if self.belowThresholdTimer <= 0 then
          self.aboveCamPitchThreshold = false
          self.belowThresholdTimer = nil
        end
      else
        -- Reset timer if speed increases
        self.belowThresholdTimer = nil
      end
    else
      self.aboveCamPitchThreshold = data.vel:length() > upperSpeedThreshold
    end

    -- dynamic pitch calculation
    local targetLerpValue = self.aboveCamPitchThreshold and 1 or 0
    self:calculateDynamicPitchLimit(data, targetPos)

    local normalizedPitchOffset
    if targetLerpValue == 1 then
      normalizedPitchOffset = self.smoothedPitchOffset:getWithRateAccel(targetLerpValue, data.dt, 0.3, 0.3, 0.3)
    else
      normalizedPitchOffset = self.smoothedPitchOffset:get(targetLerpValue, data.dt)
    end

    local dynamicPitch = -math.min(self.maxDynamicPitch, self.dynamicPitchLimit) * normalizedPitchOffset
    self.dynamicPitchQuat:setFromEuler(dynamicPitch, 0, 0)
  else
    self.smoothedPitchOffset:get(self.smoothedPitchOffset:value(), data.dt)
  end

  -- height offset calculation
  local velocity = math.min(data.vel:length(), 70)
  local smoothedVelocity = math.max(velocity * 0.05 - 0.2, 0.0)
  local lengthValue = math.min((1.4 * smoothedVelocity) / (smoothedVelocity + 4.1), 1)

  resultPos:set(0, 0, lengthValue)
  resultPos:setScaled(self.maxDynamicOffset)
  resultPos:setAdd(camPos)

  resultRot:setFromDir(push3(targetPos) - camPos)
  resultRot:setMul2(self.dynamicPitchQuat, resultRot)
  data.res.pos:set(resultPos)
  data.res.rot:set(resultRot)
  data.res.fov = fov
  data.res.targetPos:set(targetPos)
  self.collision:update(data)
  return true
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
  local o = ... or {}
  setmetatable(o, C)
  o:init()
  return o
end

