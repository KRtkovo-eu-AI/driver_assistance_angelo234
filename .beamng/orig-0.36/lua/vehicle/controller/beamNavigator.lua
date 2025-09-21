-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local htmlTexture = require("htmlTexture")

local screenMaterialName = nil
local htmlFilePath = nil
local textureWidth = 0
local textureHeight = 0
local textureFPS = 0
local updateTimer = 0
local invFPS = 1 / 15 -- 15 FPS
local gpsData = {x = 0, y = 0, rotation = 0, zoom = 1, ignitionLevel = 0}

local function updateGFX(dt)
  updateTimer = updateTimer + dt
  if updateTimer > invFPS and playerInfo.anyPlayerSeated then
    updateTimer = 0
    local pos = obj:getPosition()
    local rotation = math.deg(obj:getDirection()) + 180
    local speed = electrics.values.airspeed * 3.6
    local zoom = math.min(150 + speed * 1.5, 250)

    gpsData.x = pos.x
    gpsData.y = pos.y
    gpsData.rotation = rotation
    gpsData.speed = speed
    gpsData.zoom = zoom -- unused in dash navigation
    gpsData.ignitionLevel = electrics.values.ignitionLevel
    htmlTexture.call(screenMaterialName, "map.updateData", gpsData)
  end
end

local function init(jbeamData)
  screenMaterialName = jbeamData.screenMaterialName or "@screen_gps"
  htmlFilePath = jbeamData.htmlFilePath or "local://local/vehicles/common/navi_screen.html"
  textureWidth = jbeamData.textureWidth or 256
  textureHeight = jbeamData.textureHeight or 128
  textureFPS = jbeamData.textureFPS or 30

  htmlTexture.create(screenMaterialName, htmlFilePath, textureWidth, textureHeight, textureFPS, "automatic")
  obj:queueGameEngineLua(string.format("extensions.ui_uinavi.requestVehicleDashboardMap(%q, nil, %d)", screenMaterialName, obj:getID()))
  if jbeamData.bootscreenImage then
    htmlTexture.call(screenMaterialName, "map.setBootscreenImage", {url = jbeamData.bootscreenImage})
  end
end

M.init = init
M.reset = nop -- this is needed so that we do not call init when reseting
M.updateGFX = updateGFX

return M
