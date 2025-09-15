-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local acos = math.acos
local deg = math.deg

local debugUsers = {}
local isDebugEnabled = false

local function onInit()
  debugUsers = {}
  isDebugEnabled = false
end

local function registerDebugUser(user, value)
  debugUsers[user] = value

  isDebugEnabled = false
  for _,v in pairs(debugUsers) do
    isDebugEnabled = isDebugEnabled or v
  end
end

local function updateGFX(dt)
  if not isDebugEnabled then
    return
  end

  local vehForward = obj:getDirectionVector()
  local vehRight = obj:getDirectionVectorRight()

  --local centerPos = vec3()

  local surfaceUp = vec3()
  local count = 0

  for i = 0, wheels.wheelRotatorCount - 1 do
    local wheel = wheels.wheelRotators[i]
    local nodeId = wheel.lastTreadContactNode
    if nodeId then
      local pos = obj:getNodePosition(nodeId) + obj:getPosition()
      local normal = mapmgr.surfaceNormalBelow(pos, 0.1)
      surfaceUp:setAdd(normal)
      --centerPos:setAdd(pos)
      count = count + 1
    end
  end

  surfaceUp:setScaled(1 / count)
  local surfaceRight = vehForward:cross(surfaceUp)
  local surfaceForward = surfaceUp:cross(surfaceRight)

  -- centerPos:setScaled(1 / count)

  -- obj.debugDrawProxy:drawLine(centerPos, centerPos + vectorRight, color(255,0,0,255))
  -- obj.debugDrawProxy:drawLine(centerPos, centerPos + vectorForward, color(0,255,0,255))
  -- obj.debugDrawProxy:drawLine(centerPos, centerPos + vectorUp, color(0,0,255,255))

  local data = {}
  for _,wd in pairs(v.data.wheels) do
    local name = wd.name
    local wheelData = {name = name}
    if wd.steerAxisUp and wd.steerAxisDown then
      local casterSign = -obj:nodeVecCos(wd.steerAxisUp, wd.steerAxisDown, surfaceForward)
      wheelData.caster = deg(acos(obj:nodeVecPlanarCos(wd.steerAxisUp, wd.steerAxisDown, surfaceUp, surfaceForward))) * sign(casterSign)
      wheelData.sai = deg(acos(obj:nodeVecPlanarCos(wd.steerAxisUp, wd.steerAxisDown, surfaceUp, surfaceRight)))
    end
    --local camberSign = obj:nodeVecCos(wd.node2, wd.node2, vectorForward) --unused
    wheelData.camber = (90 - deg(acos(obj:nodeVecPlanarCos(wd.node2, wd.node1, surfaceUp, surfaceRight))))
    local toeSign = obj:nodeVecCos(wd.node1, wd.node2, vehForward)
    wheelData.toe = deg(acos(obj:nodeVecPlanarCos(wd.node1, wd.node2, vehRight, vehForward)))
    if wheelData.toe > 90 then
      wheelData.toe = (180 - wheelData.toe) * sign(toeSign)
    else
      wheelData.toe = wheelData.toe * sign(toeSign)
    end

    -- failsafes for NaN below, broke UI before ...
    if isnan(wheelData.toe) or isinf(wheelData.toe) then
      wheelData.toe = 0
    end
    if isnan(wheelData.camber) or isinf(wheelData.camber) then
      wheelData.camber = 0
    end

    local hasPressure = wd.pressureGroup and v.data.pressureGroups and v.data.pressureGroups[wd.pressureGroup]
    wheelData.pressure = hasPressure and obj:getGroupPressure(v.data.pressureGroups[wd.pressureGroup]) * 0.000145038 or 0

    table.insert(data, wheelData)
  end

  if not playerInfo.firstPlayerSeated then return end
  gui.send('advancedWheelDebugData', data)
end

-- public interface
M.onInit    = onInit
M.updateGFX = updateGFX
M.registerDebugUser = registerDebugUser

return M