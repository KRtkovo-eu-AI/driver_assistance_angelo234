-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local dequeue = require('dequeue')

local M = {}
M.recoveryPoints = dequeue.new() -- historic log of positions

local recoveryPointTimedelta = 0.2 -- secs
local recoveredPointSeparation = 2
local recoveredPointSeparationSq = recoveredPointSeparation * recoveredPointSeparation
local logSize = 700
local rollLimit = 45
local pitchLimit = 80
local rewindPositionDelay = 0.7
local debugColor = color(255, 102, 0, 255)

local countDown = 0 -- simDt
local blendTime = 0
local snapshotTimeSmoother = newTemporalSmoothing(0.5)
local useSmartSpawn = true
local upVector = vec3(0,0,1)
local recoverTimer
local rewindPosition
local beamsCount = 0

local relativeNodePositions = {}

local objPos = vec3()

M.updateGFX = nop

local function clear()
  M.recoveryPoints = dequeue.new()
  M.homePoint = nil
end

local function onDeserialized(v)
  tableMerge(M, v)
  M.recoveryPoints = dequeue.new(v.recoveryPoints)
end

local function newRecoveryPoint(objPosition)
  if objPosition then
    objPosition = vec3(objPosition)
  else
    objPosition = vec3(obj:getPositionXYZ())
  end
  return {
    pos = objPosition,
    dirFront = obj:getDirectionVector(),
    dirUp = obj:getDirectionVectorUp()
  }
end

local function blendPoints(a, b, t)
  return {
    pos = vec3(a.pos) + (vec3(b.pos) - vec3(a.pos)) * t,
    dirFront = (vec3(a.dirFront) + (vec3(b.dirFront) - vec3(a.dirFront)) * t):normalized(),
    dirUp = (vec3(a.dirUp) + (vec3(b.dirUp) - vec3(a.dirUp)) * t):normalized()
  }
end

local function getRollPitch(dirFront, dirUp)
  -- find vehicle roll and pitch, in degrees, 0deg being normal upright rotation, +/-180deg being on its roof
  local dirLeft = dirUp:cross(dirFront)
  local roll  = math.deg(math.asin(dirLeft.z))
  local pitch = math.deg(math.asin(dirFront.z))
  if dirUp.z < 0 then -- if we are closer to upside down than to downside up
    -- detect the "on its roof" situation, where angles are zero, and make sure they go all the way to 180deg instead, like this:
    -- original rotation angles:  0deg (ok), 90deg (halfway),      0deg (on its roof), -90deg (halfway), 0deg (ok)
    -- corrected rotation angles: 0deg (ok), 90deg (halfway), +/-180deg (on its roof), -90deg (halfway), 0deg (ok)
    roll  = sign( roll)*(180 - math.abs( roll))
    pitch = sign(pitch)*(180 - math.abs(pitch))
  end
  --log("D", "recovery", "Roll: "..r(roll,2,2)..", Pitch: "..r(pitch,2,2)..", dirUp: "..s(recPoint.dirUp, 2,2))
  return roll, pitch
end

local isFreeCamActive = true
local function setFreeCamActiveFlag(active)
  isFreeCamActive = active
end

local function getFreeCamActiveFlag()
  return isFreeCamActive
end

local function constructAABB()
  local pmin = vec3(math.huge, math.huge, math.huge)
  local pmax = vec3(-math.huge, -math.huge, -math.huge)
  local nodes = v.data.nodes
  for i = 0, tableSizeC(nodes) - 1 do
    local pos = nodes[i].pos
    pmax:setMax(pos)
    pmin:setMin(pos)
  end

  local refPos = vec3(nodes[v.data.refNodes[0].ref].pos)
  pmin = pmin - (refPos)
  pmax = pmax - (refPos)

  -- One corner point plus the neighboring points
  return {
      vec3(pmin.x, pmin.y, pmin.z),
      vec3(pmin.x, pmin.y, pmax.z),
      vec3(pmin.x, pmax.y, pmin.z),
      vec3(pmax.x, pmin.y, pmin.z)
    }
end

local lastRecoveryPoint
local camPos = vec3()
local camRot = quat()
local function setRecoveryPoint(recPoint, resetVehicle, moveTraffic)
  lastRecoveryPoint = recPoint
  -- if the angle limits (in degrees) are surpassed, car is reset to upright position, maintaining the recpoint heading
  local dirFront = vec3(recPoint.dirFront)
  local dirUp = vec3(recPoint.dirUp)

  if resetVehicle then
    if useSmartSpawn then
      local rot = quatFromDir(dirFront, dirUp)
      obj:queueGameEngineLua("spawn.safeTeleport(getObjectByID("..obj:getId().."), vec3("..recPoint.pos.x..","..recPoint.pos.y..","..recPoint.pos.z.."), quat("..rot.x..","..rot.y..","..rot.z..","..rot.w.."), nil, nil, " .. tostring(moveTraffic) ..  ")")
    else
      -- Dont use autoplace when not using smart spawn
      -- if the angle limits (in degrees) are surpassed, car is reset to upright position, maintaining the recpoint heading
      local rot
      local dirFront = vec3(recPoint.dirFront)
      local dirUp = vec3(recPoint.dirUp)
      local roll, pitch = getRollPitch(dirFront, dirUp)
      if pitchLimit ~= nil and (math.abs(pitch) > pitchLimit or math.abs(roll) > rollLimit) then
        rot = quatFromDir(-dirFront, upVector)
      else
        rot = quatFromDir(-dirFront, dirUp)
      end
      obj:queueGameEngineLua("vehicleSetPositionRotation("..obj:getId()..","..recPoint.pos.x..","..recPoint.pos.y..","..recPoint.pos.z..","..rot.x..","..rot.y..","..rot.z..","..rot.w..")")
    end
  end
end

local function saveHome(point)
  M.homePoint = point or newRecoveryPoint()
  if point == nil then
    guihooks.message("vehicle.recovery.saveHome", 5, "recovery")
  end
end

local function loadHome(moveTraffic)
  if M.homePoint == nil then return end
  obj:requestReset(RESET_PHYSICS)     -- fix vehicle + reset velocity
  obj:queueGameEngineLua('getObjectByID('..tostring(obj:getId())..'):resetBrokenFlexMesh()')
  setRecoveryPoint(M.homePoint, true, moveTraffic)
  guihooks.message("vehicle.recovery.loadHome", 5, "recovery")
end

local function updateGFXRecord(dtSim)
  countDown = countDown - dtSim
  if countDown <= 0 then
    countDown = countDown + recoveryPointTimedelta
    if M.recoveryPoints:is_empty() then
      local startPoint = newRecoveryPoint()
      if M.homePoint == nil then
        saveHome(startPoint)
      end
      M.recoveryPoints:push_right(startPoint)
      return
    end

    objPos.x, objPos.y, objPos.z = obj:getPositionXYZ()
    if M.recoveryPoints:peek_right().pos:squaredDistance(objPos) < recoveredPointSeparationSq then
      return -- too close to last recovered point
    end

    while M.recoveryPoints:length() >= logSize do  -- remove old positions
      M.recoveryPoints:pop_left()
    end
    M.recoveryPoints:push_right(newRecoveryPoint(objPos))
  end
end

local function stopRecovering()
  material.forceReset() --here on purpurse, it get called a second type on key up and then it fixed what ever material was stuck
  if M.updateGFX == updateGFXRecord then return end
  M.updateGFX = updateGFXRecord
  --obj:setMeshNameAlpha(1, "", true) -- show everything again
  obj:queueGameEngineLua('getObjectByID('..tostring(obj:getId())..'):resetBrokenFlexMesh()')
  obj:queueGameEngineLua('be.nodeGrabber:clearVehicleFixedNodes('..tostring(obj:getId())..')')
  setRecoveryPoint(lastRecoveryPoint, true)
  lastRecoveryPoint = nil
  if M.recoveryPoints:is_empty() then
    guihooks.message("vehicle.recovery.end", 5, "recovery")
  else
    if snapshotTimeSmoother:value() > 0.9 then
      guihooks.message("vehicle.recovery.quick", 7, "recovery")
    else
      guihooks.message("vehicle.recovery.recovered", 3, "recovery")
    end
  end
  if not isFreeCamActive then
    obj:queueGameEngineLua("commands.setGameCamera()")
  end
  setFreeCamActiveFlag(nil)
  rewindPosition = false
end

local aabb
local camOffsetVec = vec3(0,0,0.1)
local p1, p2, t1, t2, backVec, tempVec = vec3(), vec3(), vec3(), vec3(), vec3(), vec3()
local oobb = {vec3(), vec3(), vec3(), vec3()}
local rot = quat()
local nodeRenderPositions = {}
local nodeRenderPositionsUpToDate = {}
local function updateGFXRecovery(dtSim)
  local dtReal = obj:getRealdt()

  recoverTimer = recoverTimer + dtReal

  if not rewindPosition and recoverTimer > rewindPositionDelay then
    aabb = constructAABB()
    rewindPosition = true
    obj:queueGameEngineLua('be:queueObjectLua('..tostring(obj:getId())..', "if recovery.getFreeCamActiveFlag() ~= nil then recovery.setFreeCamActiveFlag(" .. tostring(commands.isFreeCamera()) .. ") end") commands.setFreeCamera()')
    snapshotTimeSmoother:set(1)
    blendTime = 0
    guihooks.message("vehicle.recovery.recovering", 5, "recovery")
  end
  if not rewindPosition then return end

  blendTime = blendTime + dtReal
  local snapshotTime = snapshotTimeSmoother:getUncapped(0.08, dtReal)
  while blendTime > snapshotTime do
    if M.recoveryPoints:is_empty() then break end
    local lastRecoveredPoint = M.recoveryPoints:pop_right()  -- pop

    if M.recoveryPoints:is_empty() then
      setRecoveryPoint(lastRecoveredPoint)
    end
    blendTime = math.max(blendTime - snapshotTime, 0)
  end
  if M.recoveryPoints:is_empty() then
    stopRecovering()
    return
  end
  local lastRecoveredPoint = M.recoveryPoints:pop_right()
  local nextRecoveryPoint = M.recoveryPoints:peek_right()
  M.recoveryPoints:push_right(lastRecoveredPoint)

  if lastRecoveredPoint and nextRecoveryPoint then
    if lastRecoveredPoint.pos:distance(nextRecoveryPoint.pos) < 20 then
      local p = blendTime / snapshotTime
      local bp = blendPoints(lastRecoveredPoint, nextRecoveryPoint, p)
      setRecoveryPoint(bp)
    else
      setRecoveryPoint(nextRecoveryPoint)
    end
  end
  if lastRecoveryPoint and aabb then
    local dirFront = lastRecoveryPoint.dirFront
    local dirUp = lastRecoveryPoint.dirUp

    backVec:setScaled2(dirFront, -1)
    rot:setFromDir(backVec, dirUp)

    for i, point in ipairs(aabb) do
      oobb[i]:setRotate(rot, point)
      oobb[i]:setAdd(lastRecoveryPoint.pos)
    end

    local beams = v.data.beams
    local nodes = v.data.nodes
    local refPos = nodes[v.data.refNodes[0].ref].pos

    table.clear(nodeRenderPositionsUpToDate)
    for i=0, beamsCount - 1 do
      local beam = beams[i]
      if not nodeRenderPositionsUpToDate[beam.id1] then
        t1:set(relativeNodePositions[beam.id1])
        t1:setRotate(rot)
        p1:setAdd2(lastRecoveryPoint.pos, t1)
        nodeRenderPositionsUpToDate[beam.id1] = true
        nodeRenderPositions[beam.id1]:set(p1)
      end

      if not nodeRenderPositionsUpToDate[beam.id2] then
        t2:set(relativeNodePositions[beam.id2])
        t2:setRotate(rot)
        p2:setAdd2(lastRecoveryPoint.pos, t2)
        nodeRenderPositionsUpToDate[beam.id2] = true
        nodeRenderPositions[beam.id2]:set(p2)
      end

      obj.debugDrawProxy:drawLine(nodeRenderPositions[beam.id1], nodeRenderPositions[beam.id2], debugColor)
    end

    if not isFreeCamActive then
      camPos:set((push3(oobb[1])+oobb[4])/2 + push3(upVector) * (oobb[2].z-oobb[1].z) * 1.3 + (push3(oobb[3])-oobb[1]) * 1.5 + push3(backVec))
      tempVec:setSub2(dirFront, camOffsetVec)
      camRot:setFromDir(tempVec, upVector)
      obj:queueGameEngineLua("core_camera.setPosRot(0, "..camPos.x..","..camPos.y..","..camPos.z..","..camRot.x..","..camRot.y..","..camRot.z..","..camRot.w..")")
    end
  end
  --obj:setMeshNameAlpha(0.6, "", true) -- fade it away... need to be set here becouse sync issues with reset broken props
end

local function startRecovering(useAltMode)
  if useAltMode == nil then useAltMode = false end
  if M.updateGFX == updateGFXRecovery then return end
  M.updateGFX = updateGFXRecovery
  recoverTimer = 0
  useSmartSpawn = useAltMode ~= settings.getValue('enableSmartRecovery', true)

  local newRecoveryPoint = newRecoveryPoint()
  setRecoveryPoint(newRecoveryPoint, false)
  M.recoveryPoints:push_right(newRecoveryPoint)
end

local function recoverInPlace()
  setRecoveryPoint(newRecoveryPoint(), true)
end

local function init(path)
  M.updateGFX = updateGFXRecord
  beamsCount = v.data.beams and tableSizeC(v.data.beams) or 0

  local nodes = v.data.nodes
  table.clear(relativeNodePositions)
  for i = 0, tableSizeC(nodes) - 1 do
    relativeNodePositions[i] = nodes[i].pos - nodes[v.data.refNodes[0].ref].pos
    nodeRenderPositions[i] = vec3()
  end
end

-- public interface
M.init = init
M.startRecovering = startRecovering
M.stopRecovering = stopRecovering
M.saveHome = saveHome
M.loadHome = loadHome
M.onDeserialized = onDeserialized
M.clear = clear
M.recoverInPlace = recoverInPlace

M.setFreeCamActiveFlag = setFreeCamActiveFlag
M.getFreeCamActiveFlag = getFreeCamActiveFlag

return M
