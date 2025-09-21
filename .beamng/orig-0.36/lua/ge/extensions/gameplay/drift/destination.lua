-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
local im = ui_imgui

local pathData
local reversedFlag
local extractedWaypoints
local remainingDist
local goingWrongWay
local wrongWayFail
local distToIntendedRoad = 0.1

local maxWrongWayDist
local defaultMaxWrongWayDist = 10
local disableWrongWayAndDist

local profiler = LuaProfiler("Drift destination profiler")
local isBeingDebugged
local gc = 0
local driftDebugInfo = {
  default = false,
  canBeChanged = true
}

local function setRacePath(data)
  if data.reverse == nil then data.reverse = false end
  if data.maxWrongWayDist == nil then data.maxWrongWayDist = defaultMaxWrongWayDist end
  if data.disableWrongWayAndDist == nil then data.disableWrongWayAndDist = false end
  if data.filePath == nil then
    log('W', 'drift', "Tried setting a drift race file path, with no race path")
    return
  end

  reversedFlag = data.reverse
  maxWrongWayDist = data.maxWrongWayDist
  disableWrongWayAndDist = data.disableWrongWayAndDist

  pathData = require('/lua/ge/extensions/gameplay/race/path')("New Path")
  pathData:onDeserialized(jsonReadFile(data.filePath))
  if reversedFlag then
    pathData:reverse()
  end
  pathData:autoConfig()

  extractedWaypoints = {}
  for _, pathNode in pairs(pathData.pathnodes.sorted) do
    table.insert(extractedWaypoints, pathNode.pos)
  end
  -- We use the sorted extractedWaypoints to calculate the "wrong way", so it is important to inverse the array if needed
  if reversedFlag then
    arrayReverse(extractedWaypoints)
  end
end

local nextWaypointPos, previousWaypointPos
local vehData
local vel
local velThreshold = 0.5
local index
local xnorm
local function calcRemainingDist()
  if not extractedWaypoints then return end

  vehData = map.objects[gameplay_drift_drift.getVehId()]
  if not vehData then return end

  distToIntendedRoad = math.huge
  for i = 1, #extractedWaypoints - 1 do
    local dist = vehData.pos:distanceToLineSegment(extractedWaypoints[i], extractedWaypoints[i + 1])
    if dist < distToIntendedRoad then
      distToIntendedRoad = dist
      previousWaypointPos = extractedWaypoints[i]
      nextWaypointPos = extractedWaypoints[i + 1]
      index = i
    end
  end

  remainingDist = 0

  local xnorm = vehData.pos:xnormOnLine(previousWaypointPos, nextWaypointPos)
  remainingDist = remainingDist + lerp(previousWaypointPos, nextWaypointPos, xnorm):distance(nextWaypointPos)
  if index < #extractedWaypoints then
    for i = index, #extractedWaypoints - 1 do
      remainingDist = remainingDist + vec3(extractedWaypoints[i]):distance(extractedWaypoints[i + 1])
    end
  end

  vel = vehData.vel:length()
  local desiredDirection = nextWaypointPos - previousWaypointPos
  goingWrongWay = not disableWrongWayAndDist and vel > velThreshold and (desiredDirection:dot(vehData.vel) <= 0 or vel <= velThreshold)

  if goingWrongWay then
    extensions.hook("onDriftWrongWay")
  end
end

local currWrongWayDist = 0
local lastFramePos = vec3()
local function calcWrongWayFail()
  if goingWrongWay and lastFramePos ~= nil then
    local diff = lastFramePos:distance(gameplay_drift_drift.getVehPos())

    if diff > 0 then
      currWrongWayDist = currWrongWayDist + diff
    end

    if currWrongWayDist > maxWrongWayDist then
      wrongWayFail = true
    end
  else
    wrongWayFail = false
  end

  lastFramePos:set(gameplay_drift_drift.getVehPos())
end

local function imguiDebug()
  if isBeingDebugged then
    if im.Begin("Destination") then
      if not pathData then
        im.Text("No path data so no debug")
      else
        im.Text(string.format("Dist. remaining before fail : %i m", maxWrongWayDist - currWrongWayDist))
        im.Text("Going the wrong way : " .. tostring(goingWrongWay))
        im.Text("Reversed race path : " .. tostring(reversedFlag))
        im.Text(string.format("Dist to intended road : %i m", distToIntendedRoad))
      end
    end
  end
end

local function onUpdate()
  isBeingDebugged = gameplay_drift_general.getExtensionDebug("gameplay_drift_destination")
  imguiDebug()
  if gameplay_drift_general.getGeneralDebug() then profiler:start() end

  if not gameplay_drift_drift.getVehPos() then return end

  calcRemainingDist()
  calcWrongWayFail()

  if gameplay_drift_general.getGeneralDebug() then
    profiler:add("Drift destination")
    gc = profiler.sections[1].garbage
    profiler:finish(false)
  end
end

local function getPathData()
  return pathData
end

local function getWaypoints()
  return extractedWaypoints
end

local function getRemainingDist()
  return remainingDist
end

local function getGoingWrongWay()
  return goingWrongWay
end

local function getWrongWayFail()
  return wrongWayFail
end

local function getDriftDebugInfo()
  return driftDebugInfo
end

local function getGC()
  return gc
end

local function getDistToIntendedRoad()
  return distToIntendedRoad or 0
end

local function getDisableWrongWayAndDist()
  return disableWrongWayAndDist
end

local function reset()
  remainingDist = 0
  currWrongWayDist = 0
  lastFramePos = vec3()
end

M.onUpdate = onUpdate

M.setRacePath = setRacePath

M.getPathData = getPathData
M.getWaypoints = getWaypoints
M.getRemainingDist = getRemainingDist
M.getGoingWrongWay = getGoingWrongWay
M.getWrongWayFail = getWrongWayFail
M.getDriftDebugInfo = getDriftDebugInfo
M.getDistToIntendedRoad = getDistToIntendedRoad
M.getDisableWrongWayAndDist = getDisableWrongWayAndDist
M.getGC = getGC

M.reset = reset
return M