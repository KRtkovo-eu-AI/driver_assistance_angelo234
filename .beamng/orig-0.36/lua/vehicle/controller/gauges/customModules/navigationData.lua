-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local deg = math.deg
local atan2 = math.atan2

local vecX = vec3(1, 0, 0)
local vecY = vec3(0, -1, 0)

local gaugeHTMLTexture
local mapData = {}

local function updateGFX(dt)
end

local function updateGaugeData(moduleData, dt)
  mapData.x, mapData.y = obj:getPositionXYZ()
  local dir = obj:getDirectionVector()
  mapData.rotation = deg(atan2(dir:dot(vecX), dir:dot(vecY)))
  gaugeHTMLTexture:streamJS("updateMap", "updateMap", mapData)
end

local function setupGaugeData(properties, htmlTexture)
  gaugeHTMLTexture = htmlTexture
  obj:queueGameEngineLua(string.format('extensions.ui_uiNavi.requestVehicleDashboardMap(%q, "initMap", %d)', gaugeHTMLTexture.webViewTag, obj:getID()))
end

local function reset()
end

local function init(jbeamData)
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

M.setupGaugeData = setupGaugeData
M.updateGaugeData = updateGaugeData

return M
