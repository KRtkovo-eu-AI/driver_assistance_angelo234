-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_fonts"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
local api = nil
local helper = nil
local browser = nil
local notification = nil

-- font atlas generation
local fontPath = ""
local fontDirectory = "/ui/common/"
local destinationDirectory = "/art/dynamicDecals/fonts/"
local fontAtlasJsonExtension = ".font.json"
local glyphPixelHeight = 256
local fontGenNotifications = {}
local fontAtlasPreviewId = 0
local generatedFontAtlases = {}
local generatedFontAtlasesCharPtr = nil
local selectedGlyphId = 1
local glyphCharPtr = nil
local fontAtlasData = nil
local fontAtlasDataMap = {}

local fontPreviewWindowName = logTag .. "_fontPreviewWindow"

-- sdf gen
local generateSdf = false
local sdfPadding = 8
local sdfOnedgeValue = 128
local sdfPixelDistScale = 8.0

-- M.supportedFontFileFormats = {{"Any files", "*"},{"Font files",{".ttf", ".TTF", ".otf", ".OTF"}}, {"TTF files",{".ttf", ".TTF"}}, {"TTF files",{".otf", ".OTF"}}}
M.supportedFontFileFormats = {{"Any files", "*"},{"Font files",{".ttf", ".TTF"}}, {"TTF files",{".ttf", ".TTF"}}}

local function getSelectedFontAtlasName()
  return generatedFontAtlases[fontAtlasPreviewId + 1]
end

local tblx = {}
local function updateGlyphCharPtr()
  if #generatedFontAtlases > 0 then
    local header = fontAtlasData["header"]
    tblx = {}
    for i = header["first_char"], header["first_char"] + header["glyph_count"] - 1, 1 do
      table.insert(tblx, string.format("%d : %s", i, string.char(i)))
    end
    glyphCharPtr = im.ArrayCharPtrByTbl(tblx)
  end
end

local function readFontAtlasData(fontName)
  if fontAtlasDataMap[fontName] then return end
  fontAtlasDataMap[fontName] = jsonReadFile(string.format("%s%s/%s%s", destinationDirectory, fontName, fontName, fontAtlasJsonExtension))
end

local function updateFontAtlasData()
  if #generatedFontAtlases > 0 then
    readFontAtlasData(getSelectedFontAtlasName())
    fontAtlasData = fontAtlasDataMap[getSelectedFontAtlasName()]
  end
end

local function browserTabGui()
  local spaceAvailable = im.GetContentRegionAvail()

  im.BeginChild1("BrowserFontsChild", im.ImVec2(0, im.GetContentRegionAvail().y - math.ceil(im.GetFontSize()) - im.GetStyle().ItemSpacing.y), true)

  if #generatedFontAtlases == 0 then
    im.TextColored(editor.color.warning.Value, "No font atlases have been generated yet.")
    if im.Button("Open fonts section##1st_button") then
      tool.setSectionOpenState("Fonts", true, true)
    end
  end

  for _, fontName in ipairs(generatedFontAtlases) do
    if im.TreeNode1(string.format("%s##browserTabGui", fontName)) then
      readFontAtlasData(fontName)
      local fontData = fontAtlasDataMap[fontName]
      local previewSize = editor.getPreference('dynamicDecalsTool.textureBrowser.texturePreviewSize')
      if fontData and fontData.header and fontData.glyphs then
        local fontAtlasTexObj = editor.getTempTextureObj(string.format("%s%s/%s%s", destinationDirectory, fontName, fontName, "_monospaced.png"))
        local header = fontData.header
        local glyphs = fontData.glyphs
        for key = fontData.header.first_char, (fontData.header.first_char + fontData.header.glyph_count - 1) do
          local glyph = glyphs[tostring(key)]
          if glyph and glyph.exists == true then
            local char = glyphs[tostring(i)]
            im.ImageButton2(
              fontAtlasTexObj.texId,
              im.ImVec2(previewSize, previewSize),
              im.ImVec2(glyph.monospaced_x / header.atlas_monospaced_width, glyph.monospaced_y / header.atlas_monospaced_height),
              im.ImVec2((glyph.monospaced_x + header.glyph_pixel_height) / header.atlas_monospaced_width, (glyph.monospaced_y + header.glyph_pixel_height) / header.atlas_monospaced_height)
            )
            im.tooltip("Double-click to select character as decal texture")
            if im.IsItemHovered() and im.IsMouseDoubleClicked(0) then
              api.setDecalLayerFontPath(fontDirectory .. fontName .. '.ttf')
              api.setDecalLayerFontCharacter(string.char(key))
            end

            im.SameLine()
            if im.GetContentRegionAvailWidth() < previewSize then
              im.NewLine()
            end
          end

          if key == (fontData.header.first_char + fontData.header.glyph_count - 1) then
            im.NewLine()
          end
        end
      end

      im.TreePop()
    end
  end
  im.EndChild()

  im.TextUnformatted("Check the 'Fonts' section in order to generate more font atlases.")
  im.SameLine()
  if im.SmallButton("Open fonts section##2nd_button") then
    tool.setSectionOpenState("Fonts", true, true)
  end
end

local function updateGeneratedFontAtlases()
  local files = FS:findFiles(destinationDirectory, '*' .. fontAtlasJsonExtension, 1, true, false)
  table.sort(files, function(a, b) return string.lower(a) < string.lower(b) end)
  generatedFontAtlases = {}
  for _, file in ipairs(files) do
    local dir, filename, ext = path.split(file)
    local fontName = string.sub(filename, 1, #filename - #fontAtlasJsonExtension)
    local success = false
    local jsonContent = jsonReadFile(file)
    if jsonContent["header"] then
      success = true
    end
    if success then
      table.insert(generatedFontAtlases, fontName)
    end
  end
  generatedFontAtlasesCharPtr = im.ArrayCharPtrByTbl(generatedFontAtlases)

  updateFontAtlasData()
  updateGlyphCharPtr()
end

local function createFontBitmap(path)
  path = path or fontPath
  local res = FontRasterizer.createFontBitmap(path, destinationDirectory, glyphPixelHeight, true, generateSdf, sdfPadding, sdfOnedgeValue, sdfPixelDistScale)
  if res == true then
    table.insert(fontGenNotifications, {msg="Font atlas has been generated in '" .. destinationDirectory .. "'", time = 5})
    updateGeneratedFontAtlases()
  else
    table.insert(fontGenNotifications, {msg=string.format("Not able to create font atlas for '%s'", path), time = 5, color = editor.color.error.Value})
    editor.logWarn(string.format("%s - Not able to create font atlas for '%s'", logTag, path))
    notification.add("Fonts", "Font Atlas creation failed", "Font Atlas creation failed. Check logs for more info.", notification.levels.error)
  end
  return res
end

local function fontPreviewWindowGui()
  if editor.beginWindow(fontPreviewWindowName, "Dynamic Decals Font - Preview") then
    if im.BeginTabBar("FontPreviewTab") then
      local header = fontAtlasData["header"]
      local glyphs = fontAtlasData["glyphs"]

      if im.BeginTabItem("Info##FontPreviewTab") then
        if im.BeginTable('FontPreviewTableInfoTable', 2) then
          im.TableSetupScrollFreeze(0, 1) -- Make top row always visible
          im.TableSetupColumn("Name")
          im.TableSetupColumn("Value")
          im.TableHeadersRow()

          im.TableNextColumn()
          im.TextUnformatted("Font Name")
          im.TableNextColumn()
          im.TextUnformatted(header.font_name)

          im.TableNextColumn()
          im.TextUnformatted("Version")
          im.TableNextColumn()
          im.TextUnformatted(tostring(header.version))

          im.TableNextColumn()
          im.TextUnformatted("Atlas Dimensions")
          im.TableNextColumn()
          im.TextUnformatted(string.format("[%d, %d]", header.atlas_width, header.atlas_height))

          im.TableNextColumn()
          im.TextUnformatted("Atlas Monospaced Dimensions")
          im.TableNextColumn()
          im.TextUnformatted(string.format("[%d, %d]", header.atlas_monospaced_width, header.atlas_monospaced_height))

          im.TableNextColumn()
          im.TextUnformatted("Glyph Pixel Height")
          im.TableNextColumn()
          im.TextUnformatted(string.format("%.1f", header.glyph_pixel_height))

          im.TableNextColumn()
          im.TextUnformatted("First Character")
          im.TableNextColumn()
          im.TextUnformatted(tostring(header.first_char))

          im.TableNextColumn()
          im.TextUnformatted("Glyph Count")
          im.TableNextColumn()
          im.TextUnformatted(tostring(header.glyph_count))

          if header.sdf then
            im.TableNextColumn()
            im.TextUnformatted("SDF Padding")
            im.TableNextColumn()
            im.TextUnformatted(tostring(header.sdf.padding))

            im.TableNextColumn()
            im.TextUnformatted("SDF OnEdge Value")
            im.TableNextColumn()
            im.TextUnformatted(tostring(header.sdf.onedge_value))

            im.TableNextColumn()
            im.TextUnformatted("SDF Pixel Dist Scale")
            im.TableNextColumn()
            im.TextUnformatted(string.format("%.1f", header.sdf.pixel_dist_scale))
          end

          im.EndTable()
        end
        im.EndTabItem()
      end

      if im.BeginTabItem("Atlas##FontPreviewTab") then
        im.BeginChild1("FontPreviewAtlasChild")
        helper.imageWidget(string.format("%s%s/%s%s", destinationDirectory, getSelectedFontAtlasName(), getSelectedFontAtlasName(), "_monospaced.png"), im.GetContentRegionAvailWidth() - 2 * im.GetStyle().FramePadding.x)
        im.EndChild()
        im.EndTabItem()
      end

      if im.BeginTabItem("Glyphs##FontPreviewTab") then
        if im.Checkbox("Mark missing glyphs", editor.getTempBool_BoolBool(editor.getPreference("dynamicDecalsTool.fonts.markMissingGlyphs"))) then
          editor.setPreference("dynamicDecalsTool.fonts.markMissingGlyphs", editor.getTempBool_BoolBool())
        end
        im.BeginChild1("FontPreviewGlyphsChild")
        local glyphPreviewSize = editor.getPreference("dynamicDecalsTool.fonts.glyphPreviewSizeInPreviewWindow")
        local textureObject = editor.getTempTextureObj(string.format("%s%s/%s%s", destinationDirectory, getSelectedFontAtlasName(), getSelectedFontAtlasName(), "_monospaced.png"))
        for i = header.first_char, (header.first_char + header.glyph_count - 1), 1 do
          local char = glyphs[tostring(i)]
          im.ImageButton2(
            textureObject.texId,
            im.ImVec2(glyphPreviewSize, glyphPreviewSize),
            im.ImVec2(char.monospaced_x / header.atlas_monospaced_width, char.monospaced_y / header.atlas_monospaced_height),
            im.ImVec2((char.monospaced_x + header.glyph_pixel_height) / header.atlas_monospaced_width, (char.monospaced_y + header.glyph_pixel_height) / header.atlas_monospaced_height),
            0,
            (editor.getPreference("dynamicDecalsTool.fonts.markMissingGlyphs") and char.exists == false) and im.ImVec4(1, 0, 0, 0.1) or nil
          )
          im.tooltip(string.format("%d : %s\nxadvance: %f", i, i == 32 and "space" or string.char(i), char.xadvance))
          im.SameLine()
          if im.GetContentRegionAvailWidth() <= (glyphPreviewSize + im.GetStyle().ItemSpacing.x) then
            im.NewLine()
          end
        end
        im.EndChild()
        im.EndTabItem()
      end

      if header.sdf and im.BeginTabItem("Atlas SDF##FontPreviewTab") then
        im.BeginChild1("FontPreviewSDFAtlasChild")
        helper.imageWidget(string.format("%s%s/%s%s", destinationDirectory, getSelectedFontAtlasName(), getSelectedFontAtlasName(), "_sdf_monospaced.png"), im.GetContentRegionAvailWidth() - 2 * im.GetStyle().FramePadding.x)
        im.EndChild()
        im.EndTabItem()
      end

      if header.sdf and im.BeginTabItem("Glyphs SDF##FontPreviewTab") then
        if im.Checkbox("Mark missing glyphs", editor.getTempBool_BoolBool(editor.getPreference("dynamicDecalsTool.fonts.markMissingGlyphs"))) then
          editor.setPreference("dynamicDecalsTool.fonts.markMissingGlyphs", editor.getTempBool_BoolBool())
        end
        im.BeginChild1("FontPreviewGlyphsChild")
        local glyphPreviewSize = editor.getPreference("dynamicDecalsTool.fonts.glyphPreviewSizeInPreviewWindow")
        local textureObject = editor.getTempTextureObj(string.format("%s%s/%s%s", destinationDirectory, getSelectedFontAtlasName(), getSelectedFontAtlasName(), "_sdf_monospaced.png"))
        for i = header.first_char, (header.first_char + header.glyph_count - 1), 1 do
          local char = glyphs[tostring(i)]
          im.ImageButton2(
            textureObject.texId,
            im.ImVec2(glyphPreviewSize, glyphPreviewSize),
            im.ImVec2(char.monospaced_x / header.atlas_monospaced_width, char.monospaced_y / header.atlas_monospaced_height),
            im.ImVec2((char.monospaced_x + header.glyph_pixel_height) / header.atlas_monospaced_width, (char.monospaced_y + header.glyph_pixel_height) / header.atlas_monospaced_height),
            0,
            (editor.getPreference("dynamicDecalsTool.fonts.markMissingGlyphs") and char.exists == false) and im.ImVec4(1, 0, 0, 0.1) or nil
          )
          im.tooltip(string.format("%d : %s\nxadvance: %f", i, i == 32 and "space" or string.char(i), char.xadvance))
          im.SameLine()
          if im.GetContentRegionAvailWidth() <= (glyphPreviewSize + im.GetStyle().ItemSpacing.x) then
            im.NewLine()
          end
        end
        im.EndChild()
        im.EndTabItem()
      end

      im.EndTabBar()
    end
  end
  editor.endWindow()
end

local function sectionGui(guiId)
  im.TextUnformatted("Generated Font Atlases")
  im.SameLine()
  im.PushItemWidth(im.GetContentRegionAvailWidth() - (im.GetStyle().ItemSpacing.x + im.CalcTextSize("Preview").x + 2 * im.GetStyle().FramePadding.x))
  if #generatedFontAtlases == 0 then im.BeginDisabled() end
  if im.Combo1("##generatedFontAtlases", editor.getTempInt_NumberNumber(fontAtlasPreviewId), generatedFontAtlasesCharPtr) then
    fontAtlasPreviewId = editor.getTempInt_NumberNumber()
    updateFontAtlasData()
    updateGlyphCharPtr()
  end
  im.PopItemWidth()
  im.SameLine()
  if im.Button("Preview") then
    editor.showWindow(fontPreviewWindowName)
  end
  if #generatedFontAtlases == 0 then im.EndDisabled() end

  if #generatedFontAtlases > 0 and fontAtlasData then
    helper.imageTooltip(string.format("%s%s/%s%s", destinationDirectory, getSelectedFontAtlasName(), getSelectedFontAtlasName(), "_monospaced.png"), 512)

    local header = fontAtlasData["header"]
    local glyphs = fontAtlasData["glyphs"]

    if not header or not glyphs then return end

    if im.TreeNode1("Header##Font") then
      im.TextUnformatted("Version: " .. tostring(header["version"]))
      im.TextUnformatted("Font Name: " .. tostring(header["font_name"]))
      im.TextUnformatted("Atlas Width: " .. tostring(header["atlas_width"]))
      im.TextUnformatted("Atlas Height: " .. tostring(header["atlas_height"]))
      im.TextUnformatted("Glyph Pixel Height: " .. tostring(header["glyph_pixel_height"]))
      im.TextUnformatted("First Char Index: " .. tostring(header["first_char"]))
      im.TextUnformatted("Glyph Count: " .. tostring(header["glyph_count"]))
      im.TextUnformatted("Ascent: " .. tostring(header["ascent"]))
      im.TextUnformatted("Descent: " .. tostring(header["descent"]))
      im.TextUnformatted("Line Gap: " .. tostring(header["line_gap"]))
      im.Separator()
    end

    im.TextUnformatted("Glyph [ASCII]")
    im.SameLine()
    im.PushItemWidth(im.GetContentRegionAvailWidth())
    if im.Combo1("##selectedGlyphForPreview", editor.getTempInt_NumberNumber(selectedGlyphId), glyphCharPtr) then
      selectedGlyphId = editor.getTempInt_NumberNumber()
    end
    im.PopItemWidth()

    local char = glyphs[tostring(header["first_char"] + selectedGlyphId)]
    local cpos = im.GetCursorPos()
    local glyphPreviewSize = editor.getPreference("dynamicDecalsTool.fonts.glyphPreviewSize")
    local textPosX = cpos.x + glyphPreviewSize + im.GetStyle().ItemSpacing.x
    im.SetCursorPosX(textPosX)
    im.TextUnformatted("exists: " .. tostring(char["exists"]))
    im.SetCursorPosX(textPosX)
    im.TextUnformatted("monospaced_x: " .. tostring(char["monospaced_x"]))
    im.SetCursorPosX(textPosX)
    im.TextUnformatted("monospaced_y: " .. tostring(char["monospaced_y"]))
    -- im.SetCursorPosX(textPosX)
    -- im.TextUnformatted("x0: " .. tostring(char["x0"]))
    -- im.SetCursorPosX(textPosX)
    -- im.TextUnformatted("y0: " .. tostring(char["y0"]))
    -- im.SetCursorPosX(textPosX)
    -- im.TextUnformatted("x1: " .. tostring(char["x1"]))
    -- im.SetCursorPosX(textPosX)
    -- im.TextUnformatted("y1: " .. tostring(char["y1"]))
    -- im.SetCursorPosX(textPosX)
    -- im.TextUnformatted("width: " .. tostring(char["width"]))
    -- im.SetCursorPosX(textPosX)
    -- im.TextUnformatted("height: " .. tostring(char["height"]))
    im.SetCursorPosX(textPosX)
    im.TextUnformatted("xoff: " .. tostring(char["xoff"]))
    im.SetCursorPosX(textPosX)
    im.TextUnformatted("yoff: " .. tostring(char["yoff"]))
    im.SetCursorPosX(textPosX)
    im.TextUnformatted("xadvance: " .. tostring(char["xadvance"]))
    local afterTextPos = im.GetCursorPos()

    im.SetCursorPos(cpos)
    im.Image(
      editor.getTempTextureObj(string.format("%s%s/%s%s", destinationDirectory, getSelectedFontAtlasName(), getSelectedFontAtlasName(), "_monospaced.png")).texId,
      im.ImVec2(glyphPreviewSize, glyphPreviewSize),
      im.ImVec2(char.monospaced_x / header.atlas_monospaced_width, char.monospaced_y / header.atlas_monospaced_height),
      im.ImVec2((char.monospaced_x + header.glyph_pixel_height) / header.atlas_monospaced_width, (char.monospaced_y + header.glyph_pixel_height) / header.atlas_monospaced_height),
      nil,
      editor.color.beamng.Value
    )
    if afterTextPos.y > im.GetCursorPos().y then
      im.SetCursorPos(im.ImVec2(im.GetCursorPos().x, afterTextPos.y))
    end
  end
  im.Separator()

  im.TextUnformatted("Generation")
  im.TextUnformatted("Font Path")
  im.PushItemWidth(im.GetContentRegionAvailWidth() - 1 * tool.getIconSize() - 1 * im.GetStyle().ItemSpacing.x)
  im.InputText("##fontPath_InputText", editor.getTempCharPtr(fontPath), nil, im.InputTextFlags_ReadOnly)
  im.PopItemWidth()
  im.SameLine()
  if editor.uiIconImageButton(editor.icons.folder, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, "fontPath_FileDialogButton") then
    editor_fileDialog.openFile(
      function(data)
        fontPath = data.filepath
      end,
      M.supportedFontFileFormats,
      false,
      path.split(fontPath) or fontDirectory,
      true
    )
  end
  im.tooltip("Change font")

  im.TextUnformatted("Glyph Pixel Height")
  im.SameLine()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.InputFloat("##fontAtlasGlyphPixelHeight", editor.getTempFloat_NumberNumber(glyphPixelHeight), 1, 10, "%.0f") then
    glyphPixelHeight = editor.getTempFloat_NumberNumber()
  end
  im.PopItemWidth()

  if im.InputInt("sdf padding", editor.getTempInt_NumberNumber(sdfPadding), 1, 2) then
    sdfPadding = editor.getTempInt_NumberNumber()
  end

  if im.InputInt("sdf onedge value", editor.getTempInt_NumberNumber(sdfOnedgeValue), 1, 2) then
    local value = math.min(math.max(0, editor.getTempInt_NumberNumber()), 255)
    sdfOnedgeValue = value
  end

  if im.InputFloat("sdf Pixel Dist Scale", editor.getTempFloat_NumberNumber(sdfPixelDistScale), 0.1, 1) then
    sdfPixelDistScale = editor.getTempFloat_NumberNumber()
  end

  if fontPath == "" then im.BeginDisabled() end
  if im.Button("Generate Font Atlas") then
    createFontBitmap()
  end
  if fontPath == "" then
    im.EndDisabled()
    helper.iconTooltip("'Font' must not be empty", true)
  end

  for _, notification in ipairs(fontGenNotifications) do
    im.TextColored(notification.color or editor.color.warning.Value, notification.msg)
  end
end

local function onEditorGui()
  fontPreviewWindowGui()
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "fonts", nil, {
    {glyphPreviewSize = {"float", 128, "", nil, 32, 1024}},
    {glyphPreviewSizeInPreviewWindow = {"float", 128, "", nil, 32, 1024}},
    {markMissingGlyphs = {"bool", true, "Whether to mark missing font glyphs in the preview window or not."}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function editModeUpdate(dtReal, dtSim, dtRaw)
  for k, notif in ipairs(fontGenNotifications) do
    notif.time = notif.time - dtReal
    if notif.time < 0 then
      table.remove(fontGenNotifications, k)
    end
  end
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  helper = extensions.editor_dynamicDecals_helper
  browser = extensions.editor_dynamicDecals_browser
  notification = extensions.editor_dynamicDecals_notification

  updateGeneratedFontAtlases()

  tool.registerSection("Fonts", sectionGui, 1030, false, {})
  browser.registerBrowserTab("Fonts", browserTabGui, 40)

  editor.registerWindow(fontPreviewWindowName, im.ImVec2(550, 550))
  tool.registerEditorOnUpdateFn("fonts", editModeUpdate)
  tool.registerOnEditorGuiFn("fonts", onEditorGui)

  api.setFontTextureAtlasPath(destinationDirectory)
end

local function checkOrGenerateFontBitmaps(fontPath)
  if #fontPath > 1 then
    local path, filename, ext = path.split(fontPath)
    local fontName = string.sub(filename, 1, #filename - (#ext + 1))
    if not tableContains(generatedFontAtlases, fontName) then
      -- return false when craetion failed
      return createFontBitmap(fontPath)
    else
      return true
    end
  else
    return true
  end
end

local function getFontDirectory()
  return fontDirectory
end

local function getGeneratedFontAtlases()
  return generatedFontAtlases
end

M.checkOrGenerateFontBitmaps = checkOrGenerateFontBitmaps
M.getFontDirectory = getFontDirectory
M.getGeneratedFontAtlases = getGeneratedFontAtlases

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M