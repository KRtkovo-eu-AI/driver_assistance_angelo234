-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local min = math.min
local max = math.max
local abs = math.abs
local acos = math.acos
local ceil = math.ceil
local pi = math.pi

local M = {}
M.dependencies = {'core_groundMarkerArrows'}
M.routePlanner = require('/lua/ge/extensions/gameplay/route/route')()
M.debugPath = false

local decals = {}
local numDecals = 0

local upVec = vec3(0,0,1)
M.colorSet = 'blue'
M.colorSets = {
  blue = {
    decals = {0, 0.4, 1},
    arrows = {0, 0.48, 0.77},
  },
  green = {
    decals = {0, 1, 0},
    arrows = {0, 1, 0},
  },

}

-- global functions for backwards compatibility
local deprecationWarningDone = {}
local function deprecationWarning(oldFunction, newFunction)
  if not deprecationWarningDone[oldFunction] then
    log('W', logTag, string.format('function "%s" is deprecated. Please use function "%s" instead', oldFunction, newFunction))
    deprecationWarningDone[oldFunction] = true
  end
end

local function getPathLength()
  if M.routePlanner.path and M.routePlanner.path[1] then
    return M.routePlanner.path[1].distToTarget
  end
  return 0
end

local function getPathPositionDirection(dist, lastNodeData)
  local walkDist = lastNodeData.dist
  for i = lastNodeData.i, (lastNodeData.pathSize - 1) do
    local a = lastNodeData.path[i].pos
    local b = lastNodeData.path[i + 1].pos
    local nodeDist = (b-a):length()
    if walkDist + nodeDist >= dist then
      local factor = (dist - walkDist) / nodeDist
      local position = (a * (1 - factor)) + (b * factor)
      local normal = (b-a):normalized()
      lastNodeData.i = i
      lastNodeData.dist = walkDist
      return position, normal
    else
      walkDist = walkDist + nodeDist
    end
  end
  lastNodeData.i = lastNodeData.pathSize
  lastNodeData.dist = walkDist
  return nil
end

local decalScale = 3
local function getNewData()
  -- create decals
  local data = {
    texture = 'art/shapes/arrows/arrow_groundmarkers_1.png',
    pos = vec3(),
    position = vec3(0, 0, 0),
    forwardVec = vec3(0, 0, 0),
    color = ColorF(M.color[1], M.color[2], M.color[3], 0 ),
    scale = vec3(0.5*decalScale, 1*decalScale, 2),
    fadeStart = 100,
    fadeEnd = 120
  }
  return data
end

local function calculateAlpha(pos, start, dist)
  local linearAlpha = min(dist, max(0, pos - start)) / dist
  return 0.5-square(square(1-linearAlpha)) -- increase opacity much sooner than a linear ramp
end

M.decalPool = {}
M.decalPoolCount = 0
M.decalDrawingDistance = 120--m
M.decalBlendOffset = 1 --m
M.decalBlendStart = 10--m
M.decalBlendEnd = 100--m
-- M.stepDistance = 5
local function increaseDecalPool(max)
  while M.decalPoolCount < max do
    M.decalPoolCount = M.decalPoolCount +1
    table.insert(M.decalPool, getNewData())
  end
end

local function inverseLerp(min, max, value)
 if abs(max - min) < 1e-30 then return min end
 return (value - min) / (max - min)
end

local function generateDecalsForSegment(from, to, idx, first)
  local last = M.startingStep - ceil(to.distToTarget/M.stepDistance)
  for i = first+1, last, 1 do

    local t = inverseLerp(M.startingStep-from.distToTarget/M.stepDistance, M.startingStep-to.distToTarget/M.stepDistance, i)
    local distFromVehicle = M.startingDist-lerp(from.distToTarget, to.distToTarget, t)
    if distFromVehicle > M.decalDrawingDistance then return idx, false, i end

    M.decalPool[idx].pos = lerp(from.pos, to.pos, t)
    M.decalPool[idx].position:set(M.decalPool[idx].pos.x, M.decalPool[idx].pos.y, M.decalPool[idx].pos.z)
    local normal = (to.pos - from.pos):normalized()
    M.decalPool[idx].forwardVec:set(normal.x, normal.y, normal.z)
    M.decalPool[idx].color.a = 0
    if distFromVehicle > M.decalBlendEnd then
      M.decalPool[idx].color.a = max(M.decalPool[idx].color.a, 1-(distFromVehicle - M.decalBlendEnd) / (M.decalDrawingDistance-M.decalBlendEnd))
    elseif distFromVehicle < M.decalBlendStart then
      M.decalPool[idx].color.a = max(M.decalPool[idx].color.a, ((distFromVehicle-M.decalBlendOffset) / (M.decalBlendStart-M.decalBlendOffset)))
    else
      M.decalPool[idx].color.a = 1
    end
    -- enable this line for "quicker" blending
    --M.decalPool[idx].color.a = 1-square(1-M.decalPool[idx].color.a)
    idx = idx + 1
  end
  return idx, (M.startingDist-to.distToTarget) < M.decalDrawingDistance, last
end

local function generateRouteDecals(startPos)
  profilerPushEvent("Groundmarkers generateRouteDecals")

  if M.decalPoolCount == 0 then
    increaseDecalPool(M.decalDrawingDistance/M.stepDistance + 10)
  end

  local path = M.routePlanner.path
  local totalDist = (startPos - path[1].pos):length() + path[1].distToTarget
  M.startingDist = totalDist
  M.startingStep = ceil(totalDist/M.stepDistance)+1

  local nextIdx, cont, first = 1, true, 1
  local i = 1

  while cont do
    if path[i+1] then
      nextIdx, cont, first = generateDecalsForSegment(path[i], path[i+1], nextIdx, first)
      i = i+1
      cont = cont and path[i+1]
    else
      cont = nil
    end
  end
  M.activeDecalCount = nextIdx+1
  for i = max(nextIdx-1,1), M.decalPoolCount do
    M.decalPool[i].color.a = 0
  end

  -- Update arrows using the new module
  core_groundMarkerArrows.updateArrows(path, totalDist)

  profilerPopEvent('generateRouteDecals')
end

local appTimer = 0
local appInterval = 0.25
local lastGenerationPos
local playerVehPos = vec3()
local function onPreRender(dt)
  if not M.endWP then return end
  profilerPushEvent("Groundmarkers onPreRender")

  local veh = getPlayerVehicle(0)
  if veh then
    M.routePlanner:trackVehicle(veh)
  end

  if veh then
    playerVehPos:set(veh:getPositionXYZ())
  else
    playerVehPos:set(core_camera.getPosition())
  end

  if freeroam_bigMapMode.bigMapActive() or not lastGenerationPos or lastGenerationPos:distance(playerVehPos) > 1 then
    generateRouteDecals(playerVehPos)
    lastGenerationPos = lastGenerationPos or vec3()
    lastGenerationPos:set(playerVehPos)
  end

  if M.debugPath then
    for i, wp in ipairs(M.routePathTmp or {}) do
      debugDrawer:drawSphere(vec3(wp), 0.25, ColorF(1, 0.4, 1,0.2))
      debugDrawer:drawTextAdvanced(wp, String(i), ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
    end
    for i, e in ipairs(M.routePlanner.path) do
      debugDrawer:drawSphere(vec3(e.pos), 1, ColorF(0.23, 0.4, 0.1,0.6))
      if i > 1 then
        debugDrawer:drawSquarePrism(
          vec3(e.pos), vec3(M.routePlanner.path[i-1].pos),
          Point2F(2,0.5),
          Point2F(2,0.5),
          ColorF(0.23, 0.5,0.2, 0.6))
      end
    end
  end

  Engine.Render.DynamicDecalMgr.addDecals(M.decalPool, M.activeDecalCount)
  appTimer = appTimer + dt
  if appTimer > appInterval then
    appTimer = 0
    M.sendToApp()
  end
  profilerPopEvent('Groundmarkers onPreRender')
end

local function sendToApp()
  local data = {
    markers = {},
    color = string.format("#%02X%02X%02XFF", M.color[1]*255, M.color[2]*255, M.color[3]*255)
  }
  local id = 1
  for i, e in ipairs(M.routePlanner.path) do
    data.markers[id] = e.pos.x
    data.markers[id+1] = e.pos.y
    id = id+2
  end

  guihooks.trigger("NavigationGroundMarkersUpdate", data)
end

local function setPath(wp, options)
  profilerPushEvent("Groundmarkers setFocus")
  options = options or {}
  M.endWP = nil
  -- clear pool
  M.decalPoolCount = 0
  M.decalPool = {}
  M.activeDecalCount = 0
  lastGenerationPos = nil
  M.clearPathOnReachingTarget = options.clearPathOnReachingTarget
  M.stepDistance = options.step or 3
  M.color = options.color or M.colorSets[M.colorSet].decals
  M.floatingArrowColor = M.colorSets[M.colorSet].arrows

  M.cutOffDrivability = options.cutOffDrivability
  M.penaltyAboveCutoff = options.penaltyAboveCutoff
  M.penaltyBelowCutoff = options.penaltyBelowCutoff
  M.renderDecals = options.renderDecals ~= false
  M.endWP = (type(wp) == 'table' and wp) or {wp}
  if not wp or tableIsEmpty(M.endWP) then
    M.routePlanner:clear()
    M.endWP = nil
    local data = {
      markers = {}
    }
    guihooks.trigger("NavigationGroundMarkersUpdate", data)
  else
    local veh = getPlayerVehicle(0)
    local vehiclePos = vec3(veh and veh:getPosition() or core_camera.getPosition())
    local multiPath = {}
    table.insert(multiPath, vehiclePos)
    for _, w in ipairs(M.endWP) do
      if type(w) == 'string' then
        if not map.getMap().nodes[w] then
          log("W","","Could not find WP to build route! Ignoring WP: " .. dumps(w))
        else
          table.insert(multiPath, map.getMap().nodes[w].pos)
        end
      elseif type(w) == 'table' and #w == 3 then
        table.insert(multiPath, vec3(w))
      else
        table.insert(multiPath, w)
      end
    end

    profilerPushEvent("Groundmarkers route setupPath")
    M.routePathTmp = multiPath
    M.routePlanner:setRouteParams(M.cutOffDrivability, nil, M.penaltyAboveCutoff, M.penaltyBelowCutoff)
    M.routePlanner:setupPathMulti(multiPath)
    if veh then
      M.routePlanner:trackVehicle(veh)
    end
    profilerPopEvent('Groundmarkers setFocus')
    M.sendToApp()
  end

  core_groundMarkerArrows.createArrowPool(M.floatingArrowColor)

  profilerPopEvent('Groundmarkers setPath')
end

local function setFocus(wp, step, _fadeStart, _fadeEnd, _endPos, _disableVeh, _color, _cutOffDrivability, _penaltyAboveCutoff, _penaltyBelowCutoff, _renderDecals)
  deprecationWarning("core_groundmarkers.setFocus", "core_groundmarkers.setPath")
  local options = {
    step = step,
    color = _color,
    cutOffDrivability = _cutOffDrivability,
    penaltyAboveCutoff = _penaltyAboveCutoff,
    penaltyBelowCutoff = _penaltyBelowCutoff,
    renderDecals = _renderDecals,
  }
  setPath(wp, options)
end

local function resetAll()
  --cleanup on level exit
  setPath(nil)

  decals = {}

  M.cutOffDrivability = nil
  M.penaltyAboveCutoff = nil
  M.penaltyBelowCutoff = nil
  M.renderDecals = nil

  core_groundMarkerArrows.clearArrows()
end

local function onClientEndMission()
  resetAll()
end

local function onExtensionUnloaded()
  resetAll()
end

local function onSerialize()
  core_groundMarkerArrows.clearArrows()
end

local function currentlyHasTarget()
  return M.endWP ~= nil
end
local function getTargetPos()
  return M.endWP and M.endWP[1]
end

M.onAnyMissionChanged = function(state) if state == "started" or state == "stopped" then M.resetAll() end end

-- public interface
M.onPreRender = onPreRender
M.setPath = setPath
M.getPathLength = getPathLength
M.onClientEndMission = onClientEndMission
M.onExtensionUnloaded = onExtensionUnloaded
M.onSerialize = onSerialize
M.resetAll = resetAll
M.generateRouteDecals = generateRouteDecals
M.sendToApp = sendToApp
M.currentlyHasTarget = currentlyHasTarget
M.getTargetPos = getTargetPos

-- deprecated
M.setFocus = setFocus

return M
