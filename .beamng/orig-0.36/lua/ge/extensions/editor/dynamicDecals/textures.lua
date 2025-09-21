-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_textures"
local im = ui_imgui

-- reference to the editor tool, set in init()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local browser = nil
local decal = nil
local docs = nil
local selection = nil
local notification = nil
local helper = nil
local textures = nil

local decalTextureTextFilter = im.ImGuiTextFilter()
local textFilterWidth = 200

local contextMenuTexturePath = ""

local selectedTextureCol = im.ImColorByRGB(255,102,0,64)
local selectedTexturesSidecarContent = nil

local bulkChangeTemplate = nil
local newTagName = {}

local function checkFilter_decalTexture(textureFilter, dirName, fileName, extension)
  -- Filter out all files that start with an underscore.
  if fileName:startswith("_") then
    return false
  end
  local basename = fileName:match("(.+)%..+")
  if im.ImGuiTextFilter_PassFilter(textureFilter, basename) then
    return true
  end
  return false
end

local openPopup = false

local function selectTextureFile(filePaths, addToSelection)
  editor.clearObjectSelection()

  if not addToSelection or not editor.selection["dynamicDecalTexture"] then
    -- just clear other selection types
    editor.selection = {}
    editor.selection["dynamicDecalTexture"] = {}
  end

  selection.deselectLayer()
  selectedTexturesSidecarContent = {}

  local sel = editor.selection["dynamicDecalTexture"]
  for _, filePath in ipairs(filePaths) do
    if not tableContains(sel, filePath) then
      table.insert(sel, filePath)
    end
  end

  for _, filePath in ipairs(sel) do
    if textures.sidecarFileExists(filePath) then
      selectedTexturesSidecarContent[filePath] = textures.readSidecarFile(filePath)
      newTagName[filePath] = ""
    end
  end
end

local function handleTextureImageTileClicked(filePath, texturesFiles)
  if im.IsKeyDown(im.GetKeyIndex(im.Key_ReservedForModShift)) then
    local sel = editor.selection["dynamicDecalTexture"]
    if #sel == 1 then
      local a = -1
      local b = -1

      for k,v in ipairs(texturesFiles) do
        if sel[1] == v then a = k end
        if filePath == v then b = k end
      end

      if a ~= -1 and b ~= -1 then
        local files = {}
        local from = a < b and a or b
        local to = a < b and b or a

        for i = from, to do
          table.insert(files, texturesFiles[i])
        end
        selectTextureFile(files)
        return
      end
    end
  end
  selectTextureFile({filePath}, im.IsKeyDown(im.GetKeyIndex(im.Key_ReservedForModCtrl)))
end

local function drawTextureTiles(textureFilePaths, textureFilter, disableVirtualScrolling)
  local thumbnailSize = editor.getPreference("dynamicDecalsTool.textureBrowser.texturePreviewSize")

  local loadImagesCount = editor.getPreference("dynamicDecalsTool.textureBrowser.loadImagesPerFrameCount")
  -- virtual scrolling testing
  local cPos = im.GetCursorPos()
  local scrollY = im.GetScrollY()
  local sel = editor.selection["dynamicDecalTexture"]
  local spaceAvailable = im.GetContentRegionAvail()

  for k, filePath in ipairs(textureFilePaths) do

    local dirName, fileName, extension = path.split(filePath)
    local check = true
    if textureFilter then
      check = checkFilter_decalTexture(textureFilter, dirName, fileName, extension)
    end

    if check then
      -- virtual scrolling
      if editor.getPreference("dynamicDecalsTool.textureBrowser.enableVirtualScrolling") and (disableVirtualScrolling == nil or disableVirtualScrolling == false) then
        local imagePosY = im.GetCursorPosY()
        if (cPos.x + scrollY - thumbnailSize) > imagePosY or imagePosY > (cPos.x + scrollY + spaceAvailable.y) then
          im.Button("##" .. filePath, im.ImVec2(thumbnailSize, thumbnailSize))
        else

          if im.ImTextureHandlerIsCached(filePath) then
            if im.ImageButton(string.format("##textures_imageButton_%s", filePath), editor.getTempTextureObj(filePath).texId, im.ImVec2(thumbnailSize, thumbnailSize), im.ImVec2Zero, im.ImVec2One, (sel and tableContains(sel, filePath)) and selectedTextureCol.Value or nil) then
              handleTextureImageTileClicked(filePath, textureFilePaths)
            end
          else
            if loadImagesCount > 0 then
              im.PushID1("Texture_" .. tostring(k))
              if im.ImageButton(string.format("##textures_imageButton_%s", filePath), editor.getTempTextureObj(filePath).texId, im.ImVec2(thumbnailSize, thumbnailSize), im.ImVec2Zero, im.ImVec2One, (sel and tableContains(sel, filePath)) and selectedTextureCol.Value or nil) then
                handleTextureImageTileClicked(filePath, textureFilePaths)
              end
              im.PopID()
              loadImagesCount = loadImagesCount - 1
            else
              im.Button("##" .. filePath, im.ImVec2(thumbnailSize, thumbnailSize))
            end
          end
        end
      else
        if im.ImTextureHandlerIsCached(filePath) then
          if im.ImageButton(string.format("##textures_imageButton_%s", filePath), editor.getTempTextureObj(filePath).texId, im.ImVec2(thumbnailSize, thumbnailSize), im.ImVec2Zero, im.ImVec2One, (sel and tableContains(sel, filePath)) and selectedTextureCol.Value or nil) then
            handleTextureImageTileClicked(filePath, textureFilePaths)
          end
        else
          if loadImagesCount > 0 then
            im.PushID1("Texture_" .. tostring(k))
            if im.ImageButton(string.format("##textures_imageButton_%s", filePath), editor.getTempTextureObj(filePath).texId, im.ImVec2(thumbnailSize, thumbnailSize), im.ImVec2Zero, im.ImVec2One, (sel and tableContains(sel, filePath)) and selectedTextureCol.Value or nil) then
              handleTextureImageTileClicked(filePath, textureFilePaths)
            end
            im.PopID()
            loadImagesCount = loadImagesCount - 1
          else
            im.Button("##" .. filePath, im.ImVec2(thumbnailSize, thumbnailSize))
          end
        end
      end

      if im.IsItemClicked(1) then
        contextMenuTexturePath = filePath
        openPopup = true
      end

      if im.IsItemHovered() and im.IsMouseDoubleClicked(0) then
        api.setDecalTexturePath("color", filePath)
        decal.checkColorDecalTexturesSdfCompatible()
      end

      if im.BeginDragDropSource(im.DragDropFlags_SourceAllowNullID) then
        local payload = im.ArrayChar(256, filePath)
        im.SetDragDropPayload("DynDecalTextureDrapDrop", payload, im.ArraySize(payload))
        im.TextUnformatted(filePath)
        im.Image(editor.getTempTextureObj(filePath).texId, im.ImVec2(64, 64), im.ImVec2Zero, im.ImVec2One)
        im.EndDragDropSource()
      end
      im.tooltip(string.format("%s\nDouble-click to set texture as color texture\nLMB to select texture\nCtrl+LMB Add texture to selection\nRMB to open context menu", fileName))
      im.SameLine()
      if im.GetContentRegionAvailWidth() < thumbnailSize then
        im.NewLine()
      end
    end

    -- for some reason the cursor position wasn't quite right in the tags tab and always shifted one 'thumbnailSize' up
    -- so we reposition the cursor position here after the last element
    if k == #textureFilePaths then
      local cp = im.GetCursorPos()
      im.SetCursorPos(im.ImVec2(cPos.x, cp.y + thumbnailSize))
    end
  end
end

local function texturesBrowserTabGui()
  local spaceAvailable = im.GetContentRegionAvail()
  im.BeginChild1("DecalTexturesBrowserChild", im.ImVec2(spaceAvailable.x, spaceAvailable.y - (2 * im.GetStyle().ItemSpacing.y +  1 * math.ceil(im.GetFontSize()))), true)
  drawTextureTiles(textures.getTextureFiles(), decalTextureTextFilter)
  im.EndChild()

  if editor.uiIconImageButton(editor.icons.help_outline, tool.getIconSizeVec2(), nil, nil, nil, "DynamicDecals_Browser_Textures_Docs_Button") then
    docs.selectSection({"Browser", "Textures"})
  end
  im.tooltip("Docs")
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.settings, tool.getIconSizeVec2(), nil, nil, nil, "DynamicDecals_Browser_Textures_Prefs_Button") then
    editor.showPreferences("dynamicDecalsTool")
  end
  im.tooltip("Preferences")
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.folder, tool.getIconSizeVec2(), nil, nil, nil, "DynamicDecals_Browser_Textures_OpenDirectory_Button") then
    Engine.Platform.exploreFolder(M.getTexturesDirectoryPath())
  end
  im.tooltip("Open Textures Directory")
  im.SameLine()

  local cPos = im.GetCursorPos()
  local textSpace = im.GetContentRegionAvailWidth() - (im.GetStyle().ItemSpacing.x + textFilterWidth)
  im.PushTextWrapPos(textSpace)
  im.TextUnformatted("Hover over a texture and hit the right mouse button in order to open a context menu.")
  im.SetCursorPos(im.ImVec2(cPos.x + textSpace, cPos.y))
  editor.uiInputSearchTextFilter("Texture Filter", decalTextureTextFilter, textFilterWidth)
  im.PopTextWrapPos()
end

local function tagsBrowserTabGui()
  local tags = textures.getTags()
  local tagsWithReferences = textures.getTagsWithRefs()
  im.BeginChild1("DecalTexturesTagsBrowserChild", nil, true)
  if tags and tagsWithReferences then
    -- im.SeparatorText("Tags")
    -- for _, tag in ipairs(tags) do
    --   im.TextUnformatted(string.format("%s [%d]", tag, #tagsWithReferences[tag]))
    -- end
    -- im.SeparatorText("Tags With References")
    for _, tag in ipairs(tags) do
      if im.TreeNodeEx1(string.format("%s [%d]##BrowserTagsTab", tag, #tagsWithReferences[tag])) then
        drawTextureTiles(tagsWithReferences[tag], nil, true)
        im.TreePop()
      end
    end
  end
  im.EndChild()
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "textureBrowser", nil, {
    {loadImagesPerFrameCount = {"int", 3, "Number of images that are loaded per frame by the decal textures browser."}},
    {enableVirtualScrolling = {"bool", true, "If enabled only visible textures will be loaded in the decal texture browser."}},
    {texturePreviewSize = {"float", 128, "Max width of the decal texture thumbnails in the decal textures browser", nil, 32, 512}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function inspectorGui(inspectorInfo)
  local sel = editor.selection["dynamicDecalTexture"]
  if sel then

    if #sel > 1 then
      im.SeparatorText("Bulk Change")

      im.Columns(2, "dynDecalTexturesInspector_BulkChangeColumns")

      im.TextUnformatted("type")
      im.NextColumn()
      im.SetNextItemWidth(im.GetContentRegionAvailWidth())
      if im.Combo2("##dynDecalTexturesInspector_type", editor.getTempInt_NumberNumber(bulkChangeTemplate.type), "greyscale\0color\0sdf\0\0") then
        bulkChangeTemplate.type = editor.getTempInt_NumberNumber()
      end
      im.NextColumn()

      im.TextUnformatted("SDF compatible")
      im.NextColumn()
      im.SetNextItemWidth(im.GetContentRegionAvailWidth())
      if im.Checkbox("##dynDecalTexturesInspector_isSdfCompatible", editor.getTempBool_BoolBool(bulkChangeTemplate.isSdfCompatible)) then
        bulkChangeTemplate.isSdfCompatible = editor.getTempBool_BoolBool()
      end
      im.NextColumn()

      im.TextUnformatted("tags")
      im.NextColumn()

      for k, tag in ipairs(bulkChangeTemplate.tags) do
        if editor.uiIconImageButton(editor.icons.delete, tool.getIconSizeVec2(), nil, nil, nil, "DynamicDecals_Browser_Textures_BulkChange_RemoveTextureTag") then
          table.remove(bulkChangeTemplate.tags, k)
        end
        im.tooltip("Remove tag")
        im.SameLine()
        im.TextUnformatted(tag)
      end

      if editor.uiInputText(
        "##dynDecalTexturesInspector_BulkChange_newTagName",
        editor.getTempCharPtr(bulkChangeTemplate.newTagName),
        nil,
        im.InputTextFlags_AutoSelectAll,
        nil,
        nil
      ) then
        bulkChangeTemplate.newTagName = editor.getTempCharPtr()
      end
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.add, tool.getIconSizeVec2(), nil, nil, nil, "DynamicDecals_Browser_Textures_BulkChange_AddTextureTag") then
        if bulkChangeTemplate.newTagName ~= "" then
          table.insert(bulkChangeTemplate.tags, bulkChangeTemplate.newTagName)
          bulkChangeTemplate.newTagName = ""
        end
      end
      im.tooltip("Add tag")
      im.NextColumn()

      im.TextUnformatted("vehicle")
      im.NextColumn()
      im.SetNextItemWidth(im.GetContentRegionAvailWidth())
      if editor.uiInputText(
        "##dynDecalTexturesInspector_vehicle",
        editor.getTempCharPtr(bulkChangeTemplate.vehicle),
        nil,
        im.InputTextFlags_AutoSelectAll,
        nil,
        nil
      ) then
        bulkChangeTemplate.vehicle = editor.getTempCharPtr()
      end
      im.NextColumn()

      im.Columns(1, "dynDecalTexturesInspector_BulkChangeColumns")

      if im.Button("Apply##dynDecalTexturesInspector_bulkChange") then
        for _, f in ipairs(sel) do
          selectedTexturesSidecarContent[f].type = bulkChangeTemplate.type
          selectedTexturesSidecarContent[f].isSdfCompatible = bulkChangeTemplate.isSdfCompatible
          selectedTexturesSidecarContent[f].tags = bulkChangeTemplate.tags
          selectedTexturesSidecarContent[f].vehicle = bulkChangeTemplate.vehicle

          textures.updateSidecarFile(f, selectedTexturesSidecarContent[f])
        end
      end
      im.Separator()
      im.Separator()
      im.Separator()
    end

    im.BeginChild1("dynDecalTexturesInspector_Child", nil, true)
    for _, file in ipairs(sel) do
      if selectedTexturesSidecarContent and selectedTexturesSidecarContent[file] and im.TreeNodeEx1(file .. "##dynamicDecalTextureInspector", im.TreeNodeFlags_DefaultOpen) then
        im.Columns(3, "dynDecalTexturesInspectorColumns")
        im.TextUnformatted("version")
        im.NextColumn()
        im.TextUnformatted(tostring(selectedTexturesSidecarContent[file].version))
        im.NextColumn()
        local img = editor.getTempTextureObj(file)
        local maxWidth = im.GetContentRegionAvailWidth()
        local cpos = im.GetCursorPos()
        local ratio = img.size.y / img.size.x
        im.Image(img.texId, im.ImVec2(maxWidth, maxWidth * ratio), im.ImVec2Zero, im.ImVec2One, nil, editor.color.beamng.Value)
        im.SetCursorPos(cpos)
        im.NextColumn()

        im.TextUnformatted("texture resolution")
        im.NextColumn()
        im.TextUnformatted(string.format("x: %d y: %d", img.size.x, img.size.y))
        im.NextColumn()
        im.NextColumn()

        im.TextUnformatted("type")
        im.NextColumn()
        im.SetNextItemWidth(im.GetContentRegionAvailWidth())
        if im.Combo2("##dynDecalTexturesInspector_type", editor.getTempInt_NumberNumber(selectedTexturesSidecarContent[file].type), "greyscale\0color\0\0") then
          selectedTexturesSidecarContent[file].type = editor.getTempInt_NumberNumber()
        end
        im.NextColumn()
        im.NextColumn()

        im.TextUnformatted("SDF compatible")
        im.NextColumn()
        im.SetNextItemWidth(im.GetContentRegionAvailWidth())
        if im.Checkbox("##dynDecalTexturesInspector_isSdfCompatible", editor.getTempBool_BoolBool(selectedTexturesSidecarContent[file].isSdfCompatible)) then
          selectedTexturesSidecarContent[file].isSdfCompatible = editor.getTempBool_BoolBool()
        end
        im.NextColumn()
        im.NextColumn()

        im.TextUnformatted("tags")
        im.NextColumn()
        for k, tag in ipairs(selectedTexturesSidecarContent[file].tags) do
          if editor.uiIconImageButton(editor.icons.delete, tool.getIconSizeVec2(), nil, nil, nil, "DynamicDecals_Browser_Textures_RemoveTextureTag_" .. file) then
            table.remove(selectedTexturesSidecarContent[file].tags, k)
          end
          im.tooltip("Remove tag")
          im.SameLine()
          im.TextUnformatted(tag)
        end

        if editor.uiInputText(
          "##dynDecalTexturesInspector_newTagName_" .. file,
          editor.getTempCharPtr(newTagName[file]),
          nil,
          im.InputTextFlags_AutoSelectAll,
          nil,
          nil
        ) then
          newTagName[file] = editor.getTempCharPtr()
        end
        im.SameLine()
        if editor.uiIconImageButton(editor.icons.add, tool.getIconSizeVec2(), nil, nil, nil, "DynamicDecals_Browser_Textures_AddTextureTag_" .. file) then
          if newTagName[file] ~= "" then
            table.insert(selectedTexturesSidecarContent[file].tags, newTagName[file])
            newTagName[file] = ""
          end
        end
        im.tooltip("Add tag")
        im.NextColumn()
        im.NextColumn()

        im.TextUnformatted("vehicle")
        helper.iconTooltip("Is this decal texture bound to a specific set of vehicles?", true)
        im.NextColumn()
        im.SetNextItemWidth(im.GetContentRegionAvailWidth())
        if editor.uiInputText(
          "##dynDecalTexturesInspector_vehicle_" .. file,
          editor.getTempCharPtr(selectedTexturesSidecarContent[file].vehicle),
          nil,
          im.InputTextFlags_AutoSelectAll,
          nil,
          nil
        ) then
          selectedTexturesSidecarContent[file].vehicle = editor.getTempCharPtr()
        end
        im.NextColumn()
        im.NextColumn()

        im.Columns(1, "dynDecalTexturesInspectorColumns")

        if im.Button("Apply##dynDecalTexturesInspector_" .. file) then
          textures.updateSidecarFile(file, selectedTexturesSidecarContent[file])
        end
        im.SameLine()
        if im.Button("Cancel##dynDecalTexturesInspector_" .. file) then
          selectedTexturesSidecarContent[file] = textures.readSidecarFile(file)
        end

        im.TreePop()
      end
    end
    im.EndChild()
  end
end

local function onEditorGuiFn()
  if im.BeginPopup("TextureContextMenuPopup") then
    if im.Button("Set as decal color texture") then
      api.setDecalTexturePath("color", contextMenuTexturePath)
      decal.checkColorDecalTexturesSdfCompatible()
      im.CloseCurrentPopup()
    end
    if im.Button("Set as decal alpha texture") then
      api.setDecalTexturePath("alpha", contextMenuTexturePath)
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end

  if openPopup then
    im.OpenPopup("TextureContextMenuPopup")
    openPopup = false
  end
end

local function texturesDocsGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
The Textures tab serves as a comprehensive directory of all accessible textures.

In this tab, you'll find an inventory of textures for your design needs.
These textures can be utilized as color or alpha maps, adding depth and complexity to your decals.

The Textures tab not only offers a pre-existing selection but also accommodates your creativity.
You can easily incorporate your own textures, allowing you to personalize your designs.

A right-click on a texture opens up a context menu, simplifying the process of setting a texture to a designated property.
  ]])
  im.PopTextWrapPos()
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  browser = extensions.editor_dynamicDecals_browser
  decal = extensions.editor_dynamicDecals_layerTypes_decal
  docs = extensions.editor_dynamicDecals_docs
  selection = extensions.editor_dynamicDecals_selection
  notification = extensions.editor_dynamicDecals_notification
  helper = extensions.editor_dynamicDecals_helper

  textures = extensions.editor_api_dynamicDecals_textures

  tool.registerOnEditorGuiFn("textures", onEditorGuiFn)
  browser.registerBrowserTab("Textures", texturesBrowserTabGui, 10)
  browser.registerBrowserTab("Tags", tagsBrowserTabGui, 20)
  docs.register({section = {"Browser", "Textures"}, guiFn = texturesDocsGui})

  editor.registerInspectorTypeHandler("dynamicDecalTexture", inspectorGui)

  FS:directoryCreate(M.getTexturesDirectoryPath())

  textures.setup()
  local missingSidecarFiles = textures.getAndFlushMissingSidecarFiles()

  if #missingSidecarFiles > 0 then
    notification.add("Textures", "Missing texture meta files",
      function()
        im.Text("Meta files for new textures have been created. Please check the textures and adjust their meta files accordingly.")
        im.Indent()
        for _, file in ipairs(missingSidecarFiles) do
          if im.SmallButton("Select##DynamicDecalsTextures_" .. file) then
            selectTextureFile({file})
          end
          im.SameLine()
          im.TextUnformatted(file)
        end
        if #missingSidecarFiles > 1 then
          im.Separator()
          if im.SmallButton("Select All##DynamicDecalsTextures") then
            selectTextureFile(missingSidecarFiles)
          end
        end
        im.Unindent()
      end,
      notification.levels.log
    )
  end

  bulkChangeTemplate = shallowcopy(textures.getSidecarTemplate())
  bulkChangeTemplate.newTagName = ""
end

local function onTextureFileAdded(filepath)
  local missingSidecarFiles = textures.getAndFlushMissingSidecarFiles()

  if #missingSidecarFiles > 0 then
    notification.add("Textures", "Missing texture meta files",
      function()
        im.Text("Meta files for new textures have been created. Please check the textures and adjust their meta files accordingly.")
        im.Indent()
        for _, file in ipairs(missingSidecarFiles) do
          if im.SmallButton("Select##DynamicDecalsTextures_" .. file) then
            selectTextureFile({file})
          end
          im.SameLine()
          im.TextUnformatted(file)
        end
        if #missingSidecarFiles > 1 then
          im.Separator()
          if im.SmallButton("Select All##DynamicDecalsTextures") then
            selectTextureFile(missingSidecarFiles)
          end
        end
        im.Unindent()
      end,
      notification.levels.log
    )
  end
end

local function onTextureFileDeleted(filepath)
  if editor.selection["dynamicDecalTexture"] then
    for k, path in ipairs(editor.selection["dynamicDecalTexture"]) do
      if path == filepath then
        table.remove(editor.selection["dynamicDecalTexture"], k)
        if #editor.selection["dynamicDecalTexture"] == 0 then
          editor.selection["dynamicDecalTexture"] = nil
        end
      end
    end
  end
end

M.getTexturesDirectoryPath = function()
  return textures.getTexturesDirectoryPath()
end

M.dynamicDecals_onTextureFileAdded = onTextureFileAdded
M.dynamicDecals_onTextureFileDeleted = onTextureFileDeleted
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M