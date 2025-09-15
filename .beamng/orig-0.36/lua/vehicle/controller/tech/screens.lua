-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.relevantDevice = nil
M.defaultOrder = 500

local function updateGFX(dt)
end

local function reset(jbeamData)
  M.init(jbeamData)
end

local function init(jbeamData)
  if not jbeamData.screenDefinitions or type(jbeamData.screenDefinitions) ~= "table" or #jbeamData.screenDefinitions <= 0 then
    log("E", "screens", "No screens defined!")
    return
  end
  local screens = tableFromHeaderTable(jbeamData.screenDefinitions)

  obj:queueGameEngineLua([[
    extensions.load("tech_multiscreen")
    extensions.tech_multiscreen.removeAllViews()
  ]])

  for _, screen in pairs(screens) do
    local queueString = string.format("extensions.tech_multiscreen.addVehicleView(%q, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %d, %.2f, %.2f, %.2f, %.2f, %d, %d, %d, %s)",
      screen.name,
      screen.posX, screen.posY, screen.posZ,
      screen.rotX, screen.rotY, screen.rotZ,
      screen.resX, screen.resY,
      screen.detail,
      screen.fov,
      screen.nearClip, screen.farClip,
      screen.shadows,
      screen.windowX, screen.windowY,
      tostring(screen.borderless))
    obj:queueGameEngineLua(queueString)
  end
end


M.init = init
M.reset = reset
M.updateGFX = updateGFX

return M
