-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local myvid = -1
local triggered = false
local vdir = vec3(0,0,0)
local vup = vec3(0,0,0)
local vright = vec3(0,0,0)
local velocity = vec3(0,0,0)
-- local vpos = vec3(0,0,0) --debug
local upWorldVector = vec3(0,0,1)
local lastRoof = 0
local lastUpright = 0
local abs = math.abs
local THRESHOLD = 0.7
local REFRESH = 1
local TIMEOUT = 4
local simTime = 0

local function cancel()
  triggered = false
  lastRoof = 0
  lastUpright = 0
end

local function watchRollover( v , vid , dtSim)
  if vid ~= myvid then cancel() end
  simTime = simTime + dtSim
  myvid = vid

  -- vpos:set(v:getPositionXYZ())
  vup:set(v:getDirectionVectorUpXYZ())
  vdir:set(v:getDirectionVectorXYZ())
  velocity:set(v:getVelocityXYZ())
  vright:setCross(vdir, vup)
  vright:normalize()

  if upWorldVector:dot(vup) > THRESHOLD then
    if triggered then
      if lastRoof + TIMEOUT > simTime and lastUpright + TIMEOUT*2 > simTime then
        -- log("I","","rolllllllll")
        gameplay_statistic.metricAdd("vehicle/rollover",1)
      -- else
      --   log("I","","rol timeout r" ..dumps(lastRoof + TIMEOUT > simTime).."\tup="..dumps(lastUpright + TIMEOUT*2 > simTime))
      end
      cancel()
    end
    if simTime > lastUpright+REFRESH then
      lastUpright = simTime
    end
  end
  if upWorldVector:dot(vup) < -THRESHOLD and abs(vdir:dot(velocity))<abs(vright:dot(velocity)) and simTime > lastRoof+REFRESH then
    -- log("I","","roof")
    triggered = true
    lastRoof = simTime
  end
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
  debugDrawer:drawCylinder(vpos, vpos+vup, 0.05, ColorF(0,0,1,1), false)
  debugDrawer:drawCylinder(vpos, vpos+vdir, 0.05, ColorF(1,0,0,1), false)
  debugDrawer:drawCylinder(vpos, vpos+velocity, 0.05, ColorF(0,1,0,1), false)
  debugDrawer:drawCylinder(vpos, vpos+vright*vright:dot(velocity), 0.05, ColorF(1,1,0,1), false)
  -- debugDrawer:drawCylinder(vpos, vpos+vright*vright:dot(velocity)+vup*vup:dot(velocity), 0.05, ColorF(1,1,0,1), false)
end

M.onExtensionLoaded = onExtensionLoaded
M.onVehicleResetted = onVehicleResetted
M.onVehicleSwitched = onVehicleSwitched
-- M.onPreRender = onPreRender

M.workload = watchRollover

return M