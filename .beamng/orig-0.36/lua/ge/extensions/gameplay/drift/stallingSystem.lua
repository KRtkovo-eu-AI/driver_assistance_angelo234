-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

M.dependencies = {"gameplay_drift_general"}

local driftDebugInfo = {
  default = true,
  canBeChanged = true
}

local im = ui_imgui
local manualDebug = im.BoolPtr(false)

local stuntZonesCount = -1 -- Important to know how many stunt zones we have in order to accurately define variety
local stallingValue = 1-- the lower the worse

local gymkhanaStallingOptions = {
  minStallingValue = 0.5,
  maxStallingValue = 1,
  historyLength = 10,
  stunts = {
    drift = {
      posStallingWeight = 0.35,
      negStallingWeight = 0.15,
      maxRepetitions = 5
    },
    stuntZone = {
      posStallingWeight = 1,
      negStallingWeight = 0.6,
      maxRepetitions = 1
    }
  }
}

-- history of everything the player has done during this drift challenge
local history = {}

local function reset()
  history = {}
  stuntZonesCount = -1
  stallingValue = 1
end

local function getUniqueStuntCount()
  local uniqueStunts = {}

  for _, item in ipairs(history) do
    uniqueStunts[item.stuntId] = true
  end

  local variety = 0
  for _ in pairs(uniqueStunts) do
    variety = variety + 1
  end

  return variety
end

local function canCountScore(i)
  return i > #history - gymkhanaStallingOptions.historyLength
end

local varietyDiff
local appearanceRatio
local function calculateStallingValue()
  local uniqueStunts = getUniqueStuntCount()
  stallingValue = 1

  if #history > 1 and uniqueStunts > 1 then
    local buffer = {}
    local lastStunt
    local stunt
    local id
    local tempScore = 0
    local score = 0
    local maxPossibleScore = 0

    for i = 2, #history, 1 do
      stunt = history[i]
      id = tostring(stunt.stuntId)
      lastStunt = history[i - 1]

      if lastStunt.stuntId ~= stunt.stuntId then
        tempScore = gymkhanaStallingOptions.stunts[stunt.type].posStallingWeight
        for _, stuntj in ipairs(history) do
          buffer[tostring(stuntj.stuntId)] = {repetitions = 0}
        end
      end

      if not buffer[id] then
        buffer[id] = {
          repetitions = 0
        }
      end

      buffer[id] = {
        repetitions = buffer[id].repetitions + 1
      }

      if buffer[id].repetitions > gymkhanaStallingOptions.stunts[stunt.type].maxRepetitions then
        buffer[id].repetitions = 1
        tempScore = -(gymkhanaStallingOptions.stunts[stunt.type].negStallingWeight)
      end
      if tempScore ~= 0 and canCountScore(i) then
        score = score + tempScore
        if tempScore > 0 then
          maxPossibleScore = maxPossibleScore + tempScore
        end
      end
    end

    stallingValue = math.max(math.min(score / maxPossibleScore, gymkhanaStallingOptions.maxStallingValue), gymkhanaStallingOptions.minStallingValue)
  end
end

local function calculateScore(score)
  return score * stallingValue
end

local function processStuntZone(stuntZoneId)
  table.insert(history, {type = "stuntZone", stuntId = stuntZoneId})
  calculateStallingValue()
end

local function processDrift()
  table.insert(history, {type = "drift", stuntId = 0}) -- id 0 is for normal drifts
  calculateStallingValue()
end

local function onDriftStatusChanged(isDrifting)
  if isDrifting then
    processDrift()
  end
end

local function imguiDebug()
  if gameplay_drift_general.getExtensionDebug("gameplay_drift_stallingSystem") then
    if im.Begin("Drift stalling system") then
      im.Text(string.format("Current stalling value : %0.2f", stallingValue))
      if im.Checkbox('Manual debug', manualDebug) then
        reset()
      end
      if manualDebug[0] then
        im.Text("Imaginary stunt zone count : 3")
        if im.Button("Process stunt zone 1") then processStuntZone(1) end
        if im.Button("Process stunt zone 2") then processStuntZone(2) end
        if im.Button("Process stunt zone 3") then processStuntZone(3) end
        if im.Button("Process drift") then processDrift() end
        if im.Button("Reset") then
          reset()
        end
      end
      im.Text("History")
      im.BeginTable("History", 1, nil)
      im.TableNextColumn()
      if next(history) then
        for i = #history, 1, -1 do
          im.Text(string.format("%s %i", history[i].type, history[i].stuntId))
        end
      end
      im.EndTable()
    end
  end
end

local function onUpdate()
  imguiDebug()
end

local function getDriftDebugInfo()
  return driftDebugInfo
end

M.reset = reset

M.calculateScore = calculateScore
M.processStuntZone = processStuntZone

M.getDriftDebugInfo = getDriftDebugInfo

M.onDriftStatusChanged = onDriftStatusChanged
M.onUpdate = onUpdate
return M