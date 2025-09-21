-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local myvid = -1
local triggered = false
local vdir = vec3(0,0,0)
local backingdir = vec3(0,0,0)
local velocity = vec3(0,0,0)
-- local vpos = vec3(0,0,0) --debug
local lastBack = 0
local lastFront = 0
local abs = math.abs
local THRESHOLD = 0.96
local THRESHOLDUTURN = -0.90
local REFRESH = 1
local TIMEOUT = 4
local frontTimer = 1
local simTime = 0

local function cancel()
  -- if triggered then print("cancel") end
  triggered = false
  frontTimer = 1
end

local function watchJturn( v , vid , dtSim)
  simTime = simTime + dtSim --probably broken
  if vid ~= myvid then cancel() end
  myvid = vid

  velocity:set(v:getVelocityXYZ())
  if velocity:length() < 8 then cancel(); return end
  velocity:normalize()
  -- vpos:set(v:getPositionXYZ())
  vdir:set(v:getDirectionVectorXYZ())

  if vdir:dot(velocity) < -THRESHOLD then
    triggered = true
    if simTime > lastBack+REFRESH then
      lastBack = simTime
      -- log("I","","back"..dumps(simTime))
      backingdir:set(vdir)
      frontTimer = 1
    end
  end
  if triggered then
    if vdir:dot(velocity) > THRESHOLD and backingdir:dot(velocity) < THRESHOLDUTURN then
      frontTimer = frontTimer - dtSim
      if(frontTimer<0) then
        triggered = false
        -- log("I","","JTURN")
        gameplay_statistic.metricAdd("vehicle/jturn",1)
        cancel()
      end
    end
    if (lastBack + TIMEOUT) < simTime then
      -- log("I","","FRONT 1="..dumps((lastBack + TIMEOUT) < simTime))
      cancel()
    end
  end
  -- print(simTime)
  -- if upWorldVector:dot(vup) > THRESHOLD then
  --   if triggered then
  --     if lastRoof + TIMEOUT > simTime and lastUpright + TIMEOUT*2 > simTime then
  --       log("I","","rolllllllll")
  --       gameplay_statistic.metricAdd("vehicle/rollover",1)
  --     else
  --       log("I","","rol timeout r" ..dumps(lastRoof + TIMEOUT > simTime).."\tup="..dumps(lastUpright + TIMEOUT*2 > simTime))
  --     end
  --     cancel()
  --   end
  --   if simTime > lastUpright+REFRESH then
  --     lastUpright = simTime
  --   end
  -- end
  -- if upWorldVector:dot(vup) < -THRESHOLD and abs(vdir:dot(velocity))<abs(vright:dot(velocity)) and simTime > lastRoof+REFRESH then
  --   log("I","","roof")
  --   triggered = true
  --   lastRoof = simTime
  -- end
  -- print( dumps(upWorldVector:dot(vup)).."\t"..dumps(vdir:dot(velocity)) .."\ttr="..dumps(triggered) )
end

local function onVehicleResetted(vid)
  if myvid==vid then cancel() end
end

local function onExtensionLoaded()
end

local function onVehicleSwitched(oldid, newid, player)
  cancel()
end

local function onPreRender()
  if myvid == -1 then return end
  debugDrawer:drawCylinder(vpos, vpos+vdir, 0.05, ColorF(1,0,0,1), false)
  debugDrawer:drawCylinder(vpos, vpos+velocity, 0.05, ColorF(0,1,0,1), false)
  debugDrawer:drawCylinder(vpos, vpos+vdir*vdir:dot(velocity), 0.05, ColorF(1,1,0,1), false)
end

M.onExtensionLoaded = onExtensionLoaded
M.onVehicleResetted = onVehicleResetted
M.onVehicleSwitched = onVehicleSwitched
-- M.onPreRender = onPreRender

M.workload = watchJturn

return M