-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
C.__index = C

function C:init()
  self.camMode = 1
  self.hidden = true
  self.cockpitView = false
end

local function bbsIntersect(bb1, bb2)
  return overlapsOBB_OBB(bb1:getCenter(), bb1:getAxis(0) * bb1:getHalfExtents().x, bb1:getAxis(1) * bb1:getHalfExtents().y, bb1:getAxis(2) * bb1:getHalfExtents().z, bb2:getCenter(), bb2:getAxis(0) * bb2:getHalfExtents().x, bb2:getAxis(1) * bb2:getHalfExtents().y, bb2:getAxis(2) * bb2:getHalfExtents().z)
end

local function findCamPos(startPos, recDepth, bestCandidate)
  recDepth = recDepth and recDepth + 1 or 1

  bestCandidate = bestCandidate or vec3(startPos) + vec3(0,3,3)

  if recDepth > 40 then
    return bestCandidate
  end

  -- Choose a candidate on the ground
  local candidate = startPos + vec3(startPos):getRandomPointInCircle(20) + vec3(0,0,1)
  local downCastDist = castRayStatic(candidate, vec3(0,0,-1), 50)
  if downCastDist < 50 then
    candidate = candidate + vec3(0,0,-downCastDist + 2)
    local candidateDistance = candidate:distance(startPos)
    if candidateDistance > 10
    and castRayStatic(candidate, startPos-candidate, candidateDistance) >= candidateDistance
    and castRayStatic(startPos, candidate-startPos, candidateDistance) >= candidateDistance then
      return candidate
    end
  end

  return findCamPos(startPos, recDepth, bestCandidate)
end

function C:setCustomData(data)
  self.veh1Id = data.veh1Id
  self.veh2Id = data.veh2Id
  self.camMode = data.camMode
  local veh1 = scenetree.findObjectById(data.veh1Id)
  local bb1 = veh1:getSpawnWorldOOBB()

  -- Check if we can go into driver cam
  if data.veh2Id and (self.camMode == 1 or self.camMode == 2 and math.random() > 0.5) then
    local veh2 = scenetree.findObjectById(data.veh2Id)
    local driverNode = core_camera.getDriverDataById(veh2:getId())
    if driverNode then
      local driverPos = veh2:getPosition() + veh2:getNodePosition(driverNode)
      if veh2:getDirectionVector():dot(bb1:getCenter() - driverPos) > 0 then
        self.cockpitView = true
      else
        self.cockpitView = false
      end
    end
  end
  if self.camMode == 1 then
    self.hitPoint = data.hitPoint
    self.camOffset = data.camOffset
  elseif self.camMode == 2 then
    self.hitPoint = data.hitPoint
    self.camPos = findCamPos(self.hitPoint)
    self.fov = math.random(20,25)
  end
end

function C:update(data)
  local veh1 = scenetree.findObjectById(self.veh1Id)
  local bb1 = veh1:getSpawnWorldOOBB()

  local veh2
  local bb2
  if self.veh2Id then
    veh2 = scenetree.findObjectById(self.veh2Id)
    bb2 = veh2:getSpawnWorldOOBB()
  end
  if self.cockpitView then
    -- go to driver cam first
    local driverNode = core_camera.getDriverDataById(veh2:getId())
    local driverPos = veh2:getPosition() + veh2:getNodePosition(driverNode or 0)
    data.res.pos = driverPos
    data.res.rot = quatFromDir(bb1:getCenter() - driverPos)
    data.res.fov = 40

    if bbsIntersect(bb1, bb2) then
      self.cockpitView = false
    end
  else
    if self.camMode == 1 then
      -- camera close above crash
      data.res.pos = self.hitPoint + self.camOffset
      data.res.rot = quatFromDir(-self.camOffset)
      data.res.fov = 50

    elseif self.camMode == 2 then
      -- stationary camera
      data.res.pos = vec3(self.camPos)
      data.res.rot = quatFromDir(self.hitPoint - self.camPos)
      data.res.fov = self.fov
    end
  end
  return true
end

return function(...)
  local o = ... or {}
  setmetatable(o, C)
  o:init()
  return o
end
