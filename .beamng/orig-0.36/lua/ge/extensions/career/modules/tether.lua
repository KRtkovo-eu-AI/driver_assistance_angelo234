-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local tethers = {}
local drawDebug = false
-- create tethers that will check when the player leaves an area

local vecX = vec3(1,0,0)
local vecY = vec3(0,1,0)
local vecZ = vec3(0,0,1)

local function checkBoxTether(t)
  -- we can assume getPlayerUnicycle() is not nil, because it is checked in isWalking()
  if not getPlayerVehicle(0) then return end
  local playerPos = getPlayerVehicle(0):getPosition()
  if drawDebug then
    M.drawAxisBox(t.p1-t.x1-t.y1-t.z1, t.x1*2, t.y1*2, t.z1*2)
    debugDrawer:drawLine(playerPos, t.p1, ColorF(0,0,1,0.1))
    simpleDebugText3d(string.format("%0.1fm", (t.p1-playerPos):length()), lerp(playerPos, t.p1, 0.5))
  end
  return not containsOBB_point(t.p1, t.x1, t.y1, t.z1, playerPos)

end
-- breaks when the player walks too far away
local function startBoxTether(p1, x1, y1, z1, callback, data)
  local t = {
    checkfun = checkBoxTether,
    p1 = p1, x1 = x1, y1 = y1, z1 = z1,
    callback = callback,
    data = data
  }
  M.addTether(t)
  return t
end

local function startDoorTether(door, bufferLength, callback, data)
  bufferLength = bufferLength or 1
  local area = scenetree.findObject(door[1])
  if area then
    local pos, rot, scl = area:getPosition(), quat(area:getRotation()), area:getScale()
    local zVec, yVec, xVec = rot*vecZ*scl.z, rot*vecY*scl.y, rot*vecX*scl.x
    zVec = zVec:normalized() * (zVec:length() + bufferLength)
    yVec = yVec:normalized() * (yVec:length() + bufferLength)
    xVec = xVec:normalized() * (xVec:length() + bufferLength)
    return startBoxTether(pos, xVec, yVec, zVec, callback, data)
  end
end

local function checkSphereTether(t)
  -- we can assume getPlayerUnicycle() is not nil, because it is checked in isWalking()
  if not getPlayerVehicle(0) then return end
  local playerPos = getPlayerVehicle(0):getPosition()
  if drawDebug then
    debugDrawer:drawSphere(t.p1, t.r1, ColorF(1,0,0,0.2))
    debugDrawer:drawLine(playerPos, t.p1, ColorF(0,0,1,0.1))
    simpleDebugText3d(string.format("%0.1fm", (t.p1-playerPos):length()), lerp(playerPos, t.p1, 0.5))
  end
  return (playerPos-t.p1):length() > t.r1
end
-- breaks when the player walks too far away
local function startSphereTether(p1, r1, callback, data)
  local t = {
    checkfun = checkSphereTether,
    p1 = p1, r1 = r1,
    callback = callback,
    data = data
  }
  M.addTether(t)
  return t
end


local function checkCapsuleTether(t)
  -- we can assume getPlayerUnicycle() is not nil, because it is checked in isWalking()
  if not getPlayerVehicle(0) then return end
  local playerPos = getPlayerVehicle(0):getPosition()
  local xnorm, dist = playerPos:xnormDistanceToLineSegment(t.p1, t.p2)
  if drawDebug then
    debugDrawer:drawSphere(t.p1, t.r1, ColorF(1,0,0,0.2))
    debugDrawer:drawSphere(t.p2, t.r2, ColorF(0,1,0,0.2))
    local p = lerp(t.p1, t.p2, clamp(xnorm, 0, 1))
    debugDrawer:drawLine(playerPos, p, ColorF(0,0,1,0.1))
    debugDrawer:drawLine(t.p1, t.p2, ColorF(0,0,1,0.1))
    simpleDebugText3d(string.format("%0.1fm", (p-playerPos):length()), lerp(playerPos, p, 0.5))
  end

  if xnorm <= 0 then return (playerPos-t.p1):length() > t.r1 end
  if xnorm >= 1 then return (playerPos-t.p2):length() > t.r2 end
  return dist > lerp(t.r1, t.r2, xnorm)
end
-- breaks when the player walks too far away
local function startCapsuleTetherBetweenStatics(p1, r1, p2, r2, callback, data)
  local t = {
    checkfun = checkCapsuleTether,
    p1 = p1, r1 = r1, p2 = p2, r2 = r2,
    callback = callback,
    data = data
  }
  M.addTether(t)
  return t
end

local function checkCapsuleTetherWithVeh(t)
  local veh = scenetree.findObjectById(t.vehId)
  if not veh then return true end
  t.p2 = veh:getPosition()
  return checkCapsuleTether(t)
end
-- breaks when the player walks too far away. one po
local function startCapsuleTetherBetweenStaticAndVehicle(p1, r1, vehId, r2, callback, data)
  local t = {
    checkfun = checkWalkawayCapsuleTether,
    vehId = vehId,
    p1 = p1, r1 = r1, p2 = vec3(), r2 = r2,
    callback = callback,
    data = data
  }
  M.addTether(t)
  return t
end

M.startBoxTether = startBoxTether
M.startSphereTether = startSphereTether
M.startDoorTether = startDoorTether
M.startCapsuleTetherBetweenStaticAndVehicle = startCapsuleTetherBetweenStaticAndVehicle
M.startCapsuleTetherBetweenStatics = startCapsuleTetherBetweenStatics


local function checkVehicleTether(t)
  -- we can assume getPlayerUnicycle() is not nil, because it is checked in isWalking()
  if not getPlayerVehicle(0) then return end
  local playerPos = getPlayerVehicle(0):getPosition()

  local veh = scenetree.findObjectById(t.vehId)
  if not veh then return end
  local vehPos = veh:getPosition()
  if drawDebug then
    debugDrawer:drawSphere(vehPos, t.r1, ColorF(1,0,0,0.2))
    debugDrawer:drawLine(playerPos,vehPos, ColorF(0,0,1,0.1))
    simpleDebugText3d(string.format("%0.1fm", (playerPos - vehPos):length()), lerp(vehPos, playerPos, 0.5))
    --log("I","Tether: " .. string.format("%0.1fm", (playerPos - vehPos):length()))
  end
  -- allow tether to inverse, and call callback when the player gets too close
  if t.inverse then
    return (playerPos - vehPos):length() <= t.r1
  else
    return (playerPos - vehPos):length() > t.r1
  end
end

local function startVehicleTether(vehId, radius, inverse, callback)
  local t = {
    checkfun = checkVehicleTether,
    vehId = vehId,
    r1 = radius,
    callback = callback,
    inverse = inverse or false,
    data = data or {}
  }
  M.addTether(t)
  return t
end
M.startVehicleTether = startVehicleTether



local remove = false
local function onUpdate()
  for _, t in ipairs(tethers) do
    if not t.remove and t.checkfun(t) then
      t.callback(t)
      if drawDebug then
        log("I","","Broke tether!")
        dump(t)
      end
      t.remove = true
    end
    remove = remove or t.remove
  end
  -- cleanup routine
  if remove then
    remove = false
    local idsToRemove = {}
    for id, t in ipairs(tethers) do
      if t.remove then
        table.insert(idsToRemove, id)
      end
    end
    -- remove from the back to avoid ids moving
    for _, id in ipairs(arrayReverse(idsToRemove)) do
      table.remove(tethers, id)
    end
    if not next(tethers) then
      M.onUpdate = nil
      extensions.hookUpdate("onUpdate")
    end
  end
end

local function addTether(t)
  if not next(tethers) then
    M.onUpdate = onUpdate
    extensions.hookUpdate("onUpdate")
  end
  table.insert(tethers, t)
end

local function removeTether(t)
  table.remove(tethers, tableFindKey(tethers, t))
end

M.addTether = addTether
M.removeTether = removeTether
M.onUpdate = nop




local function drawAxisBox(corner, x, y, z, clr)
  clr = clr or color(128,64,64,32)
  -- draw all faces in a loop
  for _, face in ipairs({{x,y,z},{x,z,y},{y,z,x}}) do
    local a,b,c = face[1],face[2],face[3]
    -- spokes
    debugDrawer:drawLine((corner    ), (corner+c    ), ColorF(0,0,0,0.75))
    debugDrawer:drawLine((corner+a  ), (corner+c+a  ), ColorF(0,0,0,0.75))
    debugDrawer:drawLine((corner+b  ), (corner+c+b  ), ColorF(0,0,0,0.75))
    debugDrawer:drawLine((corner+a+b), (corner+c+a+b), ColorF(0,0,0,0.75))
    -- first side
    debugDrawer:drawTriSolid(
      vec3(corner    ),
      vec3(corner+a  ),
      vec3(corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(corner+b  ),
      vec3(corner    ),
      vec3(corner+a+b),
      clr)
    -- back of first side
    debugDrawer:drawTriSolid(
      vec3(corner+a  ),
      vec3(corner    ),
      vec3(corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(corner    ),
      vec3(corner+b  ),
      vec3(corner+a+b),
      clr)
    -- other side
    debugDrawer:drawTriSolid(
      vec3(c+corner    ),
      vec3(c+corner+a  ),
      vec3(c+corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(c+corner+b  ),
      vec3(c+corner    ),
      vec3(c+corner+a+b),
      clr)
    -- back of other side
    debugDrawer:drawTriSolid(
      vec3(c+corner+a  ),
      vec3(c+corner    ),
      vec3(c+corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(c+corner    ),
      vec3(c+corner+b  ),
      vec3(c+corner+a+b),
      clr)
  end
end
M.drawAxisBox = drawAxisBox


return M