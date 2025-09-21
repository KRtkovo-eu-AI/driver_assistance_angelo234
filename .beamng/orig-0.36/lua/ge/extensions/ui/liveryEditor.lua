-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local api = extensions.editor_api_dynamicDecals
local uiResources = extensions.ui_liveryEditor_resources
local uiFillLayer = extensions.ui_liveryEditor_layers_fill
local uiUserDataApi = extensions.ui_liveryEditor_userData
local layerEdit = extensions.ui_liveryEditor_layerEdit

local function toggleVehicleControls(enable)
  local commonActionMap = scenetree.findObject("VehicleCommonActionMap")
  if commonActionMap then
    commonActionMap:setEnabled(enable)
  end

  local specificActionMap = scenetree.findObject("VehicleSpecificActionMap")
  if specificActionMap then
    specificActionMap:setEnabled(enable)
  end
end

local startNewLivery = function()
  local vehicleObj = getPlayerVehicle(0)

  api.clearLayerStack()
  uiFillLayer.addLayer()
  -- local fillLayer = uiFillLayer.addLayer({
  --   color = vehicleObj.color
  -- })
  -- uiFillLayer.setLayer(fillLayer.uid)
end

local openSavedLivery = function(liveryPath)
  api.loadLayerStackFromFile(liveryPath)
end

local startOrLoadLivery = function(liveryPath)
  if liveryPath then
    openSavedLivery(liveryPath)
  else
    startNewLivery()
  end

  local history = api.getHistory()
  history:clear()
  api.onUpdate_()
end

M.dependencies = {"editor_api_dynamicDecals", "ui_liveryEditor_resources"}

M.saveName = nil
M.savePath = nil
M.isRunning = false
M.initialized = false
M.saveLoaded = false
M.saveLoadWaitTime = 1

local showDecalCursor = function(show)
  local r, g, b = unpack(api.getDecalColor():toTable())
  api.setDecalColor(Point4F.fromTable({r, g, b, show and 1 or 0}))
end

-- STATES
-- isEditingLayer
-- isAddingLayer
M.isAddingLayer = false
M.setDecalTexture = function(texturePath)
  api.setDecalTexturePath("color", texturePath)
end

M.save = function(filename)
  -- local playerVehicle = extensions.core_vehicles.getCurrentVehicleDetails()
  -- api.exportSkin(playerVehicle.current.key, filename)
  local vehicleObj = getPlayerVehicle(0)
  api.exportSkin(vehicleObj.jbeam, filename)
  uiUserDataApi.createSaveFile(filename)
  M.saveName = filename
end

M.setup = function(liveryPath)
  local initialized = false
  if not M.isRunning then
    api.setLayerNameBuildString("@type { - @colormap}")
    api.setup()
    core_vehicle_partmgmt.setSkin("dynamicTextures")
    -- disable api update if editor extensions has been loaded
    extensions.editor_dynamicDecalsTool.doApiUpdate = false

    -- initially disable the api and to be manually toggled for each edit state
    -- api.setEnabled(false)
    initialized = true
  end

    api.setEnabled(true)


  for _, dependency in ipairs(M.dependencies) do
    if string.startswith(dependency, "ui_liveryEditor_") then
      extensions[dependency].setup()
    end
  end

  layerEdit.resetEditState()

  M.savePath = liveryPath
  M.saveLoadWaitTime = 1
  -- startOrLoadLivery(liveryPath)

  -- TODO: check on turning off vehicle entirely
  -- disable vehicle action map
  toggleVehicleControls(false)

  M.useSurfaceNormal(true)

  -- hide decal cursor
  showDecalCursor(false)

  M.isRunning = true
  M.initialized = true
  guihooks.trigger("liveryEditor_SetupSuccess", {initialized})
end

M.deactivate = function()
  layerEdit.resetEditState()
  core_vehicle_partmgmt.setSkin(M.saveName)
  api.clearLayerStack()
  api.setEnabled(false)
  toggleVehicleControls(true)
  M.saveName = nil
  M.savePath = nil
  M.saveLoadWaitTime = 1
  M.saveLoaded = false
  M.isRunning = false
  M.initialized = false
end

M.useSurfaceNormal = function(enable)
  api.setUseSurfaceNormal(enable)
end

M.useMousePosition = function(enable)
  if not M.initialized then
    return
  end
  if api.isUseMousePos() ~= enable then
    api.toggleSetting(api.settingsFlags.UseMousePos.value)
  end
end

M.onUpdate = function(dtReal, dtSim, dtRaw)
  if M.isRunning then
    M.saveLoadWaitTime = M.saveLoadWaitTime - dtReal
    api.onUpdate_()
    if not M.saveLoaded and M.saveLoadWaitTime <= 0 then
      startOrLoadLivery(M.savePath)
      M.saveLoaded = true
    end
  end
end

M.requestSettingsData = function()
  local settingsData = {
    useMousePosition = api.isUseMousePos(),
    useSurfaceNormal = api.getUseSurfaceNormal()
  }
  guihooks.trigger("liveryEditor_settingsData", settingsData)
end

return M
