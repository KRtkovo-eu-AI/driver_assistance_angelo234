-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local function continueFreeroam()
extensions.freeroam_continueFreeroam.start() 
core_gamestate.loadingScreenActive() --remove after UI is added
end

local activities = {
  ["ContinueFreeroam"] = function() continueFreeroam() end
}

local function startActivity(name)
  log("I", "startActivity", "Requested to start activity: "..name)
  if activities[name] ~= nil then
    activities[name]()
  else
    log("I", "startActivity", "Unknown activity was requested to start")
  end
end

M.startActivity = startActivity

return M
