-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {"gameplay_statistic"}

local function onExtensionLoaded()
  if Steam.isAchievementUnlocked("KILOMETER_KICKOFF") then
    return false
  end
  local odo = gameplay_statistic.metricGet("vehicle/total_odometer.length")
  if odo and odo.value > 1e6 then
    Steam.unlockAchievement("KILOMETER_KICKOFF")
    log("W","LD","unlock direct")
    return false
  else
    gameplay_statistic.callbackRegister("vehicle/total_odometer.length", false, M.statCallback)
    --log("D","LD","reg cb")
  end

end

local function statCallback(key, oldvalue, newvalue)
  if key == "vehicle/total_odometer.length" then
    -- log("E","cb",key..""..dumps(newvalue))
    Steam.setStat("KMPASSED",newvalue.value*0.001)
    if newvalue.value > 1e6 then
      Steam.unlockAchievement("KILOMETER_KICKOFF")
      --log("D","cb","unlock CB")
      gameplay_statistic.callbackRemove("vehicle/total_odometer.length", false, M.statCallback)
      extensions.unload("gameplay_achievement")
    end
  end

end


M.onExtensionLoaded = onExtensionLoaded

M.statCallback = statCallback

return M