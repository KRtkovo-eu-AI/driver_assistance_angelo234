-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
M.dependencies = {"gameplay_drift_general", "gameplay_drift_drift"}

local im = ui_imgui

local isBeingDebugged = true
local profiler = LuaProfiler("Drift cruising profiler")
local gc = 0

local driftDuration = 1.75
local timeToDisableUI = 4
local disableUITimer = 0
local isUIEnabled = false

local function tryEnableUI()
  if not isUIEnabled then
    isUIEnabled = true
    ui_gameplayAppContainers.setContainerContext('gameplayApps', 'drift')
  end
end

local function tryDisableUI()
  if isUIEnabled then
    isUIEnabled = false
    ui_gameplayAppContainers.resetContainerContext('gameplayApps')
  end
end

local function detectStart()
  if gameplay_drift_general.getContext() == "inFreeroam" and gameplay_drift_drift.getCurrentDriftDuration() >= driftDuration then
    tryEnableUI()
  end
end

local function detectEnd(dtSim)
  if gameplay_drift_drift.getIsDrifting() and isUIEnabled then
    disableUITimer = timeToDisableUI
  elseif not gameplay_drift_drift.getIsDrifting() and isUIEnabled then
    if disableUITimer > 0 then
      disableUITimer = disableUITimer - dtSim
      if disableUITimer <= 0 then
        tryDisableUI()
      end
    end
  end
end

local function onUpdate(dtReal, dtSim, dtRaw)
  if gameplay_drift_general.getGeneralDebug() then profiler:start() end

  --detectStart()
  --detectEnd(dtSim)

  if gameplay_drift_general.getGeneralDebug() then
    profiler:add("Drift cruising")
    gc = profiler.sections[1].garbage
    profiler:finish(false)
  end
end


local function getGC()
  return gc
end



M.onUpdate = onUpdate

return M