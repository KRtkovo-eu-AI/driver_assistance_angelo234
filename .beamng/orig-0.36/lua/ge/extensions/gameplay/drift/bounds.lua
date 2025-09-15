-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
local im = ui_imgui

local red = {1,0,0}
local white = {1,1,1}

local isBeingDebugged
local gc = 0
local profiler = LuaProfiler("Drift bounds profiler")
local driftDebugInfo = {
  default = false,
  canBeChanged = true
}


local drawBounds

local sites
local isOutOfBounds

local function setBounds(filePath, drawBounds_)
  if drawBounds_ == nil then drawBounds = true end

  sites = gameplay_sites_sitesManager.loadSites(filePath, true, true)
  sites:finalizeSites()
end

local tempData
local zones
local function detectOutOfBounds()
  if not sites then return end

  if gameplay_drift_drift.getVehId() == -1 then return end

  isOutOfBounds = false

  zones = sites:getZonesForPosition(gameplay_drift_drift.getVehPos())
  if #zones == 0 then
    isOutOfBounds = true
    return
  end
end

local function drawZones()
  if not sites or not drawBounds then return end

  red[2] = math.abs(math.sin(Engine.Platform.getRuntime()*2))
  red[3] = red[2]

  for _, zone in pairs(sites.zones.objects) do
    zone:drawDebug(nil, isOutOfBounds and red or white, 2, -0.5, not isOutOfBounds)
  end
end

local function imguiDebug()
  if isBeingDebugged then
    if im.Begin("Drift bounds") then
      im.Text("Is out of bounds : " ..tostring(isOutOfBounds))
    end
  end
end

local function onUpdate()
  isBeingDebugged = gameplay_drift_general.getExtensionDebug("gameplay_drift_bounds")
  imguiDebug()
  if gameplay_drift_general.getGeneralDebug() then profiler:start() end

  detectOutOfBounds()
  drawZones()

  if gameplay_drift_general.getGeneralDebug() then
    profiler:add("Drift bounds")
    gc = profiler.sections[1].garbage
    profiler:finish(false)
  end
end

local function getIsOutOfBounds()
  return isOutOfBounds
end

local function getDriftDebugInfo()
  return driftDebugInfo
end

local function getGC()
  return gc
end

M.onUpdate = onUpdate

M.setBounds = setBounds

M.getIsOutOfBounds = getIsOutOfBounds
M.getDriftDebugInfo = getDriftDebugInfo
M.getGC = getGC

return M