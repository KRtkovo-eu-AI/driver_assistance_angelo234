-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- a simple example of how to log inputs to a file.
-- this file needs to be in lua\vehicle\tech\dumpPlayerInput.lua

-- look at lua\vehicle\input.lua for more input variables

-- load it in the simulation:
-- 1) Drop down console with `
-- 2) select the target with the combo box the active vehicle (BeamNG - Current vehicle)
-- 3) Copy and paste this and press enter: extensions.load('tech_dumpPlayerInput')

local M = {}

local f = nil
local timer = 0

-- this gets called when the extension is loaded for the first time
local function onExtensionLoaded()
  f = io.open("userInput.csv", "w")
  f:write("time,throttle\n")
end

-- this gets called when the extension is unloaded
local function onExtensionUnoaded()
  if f then
    io.close(f)
  end
end

-- this is called every frame
local function updateGFX(dt)
  if not f then return end
  timer = timer + dt
  f:write(string.format("%f,%f,%f,%f\r\n", timer, input.throttle, input.brake, input.steering))
end

-- public interface
M.updateGFX = updateGFX
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnoaded = onExtensionUnoaded

return M