-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility class for selecting materials functionality, with images and a search bar.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local decalRoadTag = 'RoadAndPath' -- The search tag for decal roads.
local decalPatchTag = 'decal' -- The search tag for decal patches.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module constants.
local im = ui_imgui
local materialSelectWinName, materialSelectWinSize = 'materialSelect', im.ImVec2(300, 500) -- The material selection window.
local simSetNameFilter = im.ImGuiTextFilter()
local materialTagStr = "materialTag"
local emptyStr = ""
local vec32 = im.ImVec2(32, 32)

-- Module state.
local hasMaterialsListBeenComputed = false
local materialSet, texObjs, loadedTextures = {}, {}, 0


-- Ensures the material list has been computed. Computes it if it hasn't been already.
local function ensureMaterialListHasBeenComputed()
  if not hasMaterialsListBeenComputed then
    table.clear(materialSet) -- Fetch the available materials list.
    local materials = Sim.getMaterialSet()
    for i = 0, materials:size() - 1 do
      table.insert(materialSet, materials:at(i))
    end
    hasMaterialsListBeenComputed = true
  end
end

-- Gets the texture object for a given absolute path.
local function getTexObj(absPath)
  if texObjs[absPath] == nil and loadedTextures < 5 then
    local texture = editor.texObj(absPath)
    loadedTextures = loadedTextures + 1
    if texture and not tableIsEmpty(texture) then
      texObjs[absPath] = texture
    else
      texObjs[absPath] = false
    end
  end
  return texObjs[absPath]
end

-- Registers the material selection window.
local function registerWindow() editor.registerWindow(materialSelectWinName, materialSelectWinSize) end

-- Opens the material selection window.
local function openWindow() editor.showWindow(materialSelectWinName) end

-- Closes the material selection window.
local function closeWindow() editor.hideWindow(materialSelectWinName) end

-- Handles the material selection window.
local function handleMaterialSelectionSubWindow(isDecal, onSelectMaterialFunct)
  if editor.beginWindow(materialSelectWinName, "Material Selector###41482", im.WindowFlags_NoCollapse) then
    -- Ensure the material list has been computed, so we have a list of materials to choose from.
    ensureMaterialListHasBeenComputed()

    -- Check if the selected material set are valid.
    if not materialSet or #materialSet < 1 then
      editor.hideWindow(materialSelectWinName)
      return
    end

    loadedTextures = 0

    -- Fixed search bar at top.
    im.PushID1("SimSetNameFilter")
    im.ImGuiTextFilter_Draw(simSetNameFilter, "Search...", 200)
    im.PopID()
    im.tooltip("Type to filter material names by name.")
    im.Separator()

    -- Scrollable list area.
    local listSize = im.ImVec2(0, im.GetContentRegionAvail().y)
    im.BeginChild1("MaterialListScrollArea", listSize, false)
    for i = 1, #materialSet do
      local obj = materialSet[i]
      local objId = obj:getName()
      local tag0 = obj:getField(materialTagStr, 0)
      local tag1 = obj:getField(materialTagStr, 1)
      local tag2 = obj:getField(materialTagStr, 2)

      -- Check if the material is a decal patch or decal road, depending on the layer type.
      local isInFilter = nil
      if isDecal then
        isInFilter = tag0 == decalPatchTag or tag1 == decalPatchTag or tag2 == decalPatchTag
      else
        isInFilter = tag0 == decalRoadTag or tag1 == decalRoadTag or tag2 == decalRoadTag
      end

      if objId ~= emptyStr and isInFilter and im.ImGuiTextFilter_PassFilter(simSetNameFilter, objId) then
        local mat = scenetree.findObject(objId)
        if mat then
          local imgPath = mat:getField("diffuseMap", 0)
          local absPath = imgPath ~= emptyStr and (string.find(imgPath, "/") and imgPath or (mat:getPath() .. imgPath)) or emptyStr
          local texture = getTexObj(absPath)
          local clickedImage = false

          if texture and texture.texId then
            if im.ImageButton("##Preview_" .. objId, texture.texId, vec32) then
              clickedImage = true
            end
            im.SameLine()
          end

          if onSelectMaterialFunct and (im.Selectable1(objId, false) or clickedImage) then
            onSelectMaterialFunct(objId) -- Call the callback function for handling the material selection event, if it exists.
          end
          im.Separator()
        end
      end
    end
    im.EndChild()
    editor.endWindow()
  end
end


-- Public interface.
M.getTexObj =                                           getTexObj

M.registerWindow =                                      registerWindow
M.openWindow =                                          openWindow
M.closeWindow =                                         closeWindow

M.handleMaterialSelectionSubWindow =                    handleMaterialSelectionSubWindow

return M