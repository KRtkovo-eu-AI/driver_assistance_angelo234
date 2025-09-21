-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_vehicleColorPalette"
local im = ui_imgui
local docs = nil

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil

local doRandomize = true
local colorPaletteName = "My Color Palette"

local restoreColors = nil

local function metallicPaintDataGui(vehicleObj, id, guiId)
  local metallicPaintData = string.split(vehicleObj:getField('metallicPaintData', id))
  im.TextUnformatted("Metallic")
  im.SameLine()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat(string.format("##VehicleColorPalette%d_Metallic_%s", id, guiId), editor.getTempFloat_NumberNumber(tonumber(metallicPaintData[1])), 0.0, 1.0, "%.2f") then
    local val = string.format("%f %s %s %s", editor.getTempFloat_NumberNumber(), metallicPaintData[2], metallicPaintData[3], metallicPaintData[4])
    vehicleObj:setField('metallicPaintData', id, val)
  end
  im.PopItemWidth()

  im.TextUnformatted("Roughness")
  im.SameLine()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat(string.format("##VehicleColorPalette%d_Roughness_%s", id, guiId), editor.getTempFloat_NumberNumber(tonumber(metallicPaintData[2])), 0.0, 1.0, "%.2f") then
    local val = string.format("%s %f %s %s", metallicPaintData[1], editor.getTempFloat_NumberNumber(), metallicPaintData[3], metallicPaintData[4])
    vehicleObj:setField('metallicPaintData', id, val)
  end
  im.PopItemWidth()

  im.TextUnformatted("Clear Coat")
  im.SameLine()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat(string.format("##VehicleColorPalette%d_ClearCoat_%s", id, guiId), editor.getTempFloat_NumberNumber(tonumber(metallicPaintData[3])), 0.0, 1.0, "%.2f") then
    local val = string.format("%s %s %f %s", metallicPaintData[1], metallicPaintData[2], editor.getTempFloat_NumberNumber(), metallicPaintData[4])
    vehicleObj:setField('metallicPaintData', id, val)
  end
  im.PopItemWidth()

  im.TextUnformatted("Clear Coat Roughness")
  im.SameLine()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat(string.format("##VehicleColorPalette%d_ClearCoatRoughness_%s", id, guiId), editor.getTempFloat_NumberNumber(tonumber(metallicPaintData[4])), 0.0, 1.0, "%.2f") then
    local val = string.format("%s %s %s %f", metallicPaintData[1], metallicPaintData[2], metallicPaintData[3], editor.getTempFloat_NumberNumber())
    vehicleObj:setField('metallicPaintData', id, val)
  end
  im.PopItemWidth()
end

local function randomizeSeed()
  if doRandomize then
    math.randomseed(os.time())
    doRandomize = false
  end
end

local function onGui(guiId)
  local vehicleObj = getPlayerVehicle(0)
  if not vehicleObj then
    im.TextUnformatted("No vehicle")
    im.TreePop()
    return
  end

  if im.Button(string.format("Randomize all colors##vehicleColorPalette%s", guiId)) then
    local colors = deepcopy(editor.getPreference("dynamicDecalsTool.colorPresets.presets"))
    if #colors < 3 then
      editor.logWarn(logTag .. ": There's not enough preset colors to populate all color palettes.")
      return
    end

    randomizeSeed()

    local c1Id = math.ceil(math.random() * #colors)
    local c1 = colors[c1Id]
    table.remove(colors, c1Id)
    local c2Id = math.ceil(math.random() * #colors)
    local c2 = colors[c2Id]
    table.remove(colors, c2Id)
    local c3Id = math.ceil(math.random() * #colors)
    local c3 = colors[c3Id]
    table.remove(colors, c3Id)

    vehicleObj.color = Point4F(c1.value[1], c1.value[2], c1.value[3], vehicleObj.color.w)
    vehicleObj.colorPalette0 = Point4F(c2.value[1], c2.value[2], c2.value[3], vehicleObj.colorPalette0.w)
    vehicleObj.colorPalette1 = Point4F(c3.value[1], c3.value[2], c3.value[3], vehicleObj.colorPalette1.w)

    editor.log(string.format("%s: Randomized color palettes\nColor palette 1 set to '%s'\nColor palette 2 set to '%s'\nColor palette 3 set to '%s'", logTag, c1.name, c2.name, c3.name))
  end
  im.tooltip("Picks a random color for each color palette from the color presets table and applies it.")

  if im.BeginPopup("SaveVehicleColorPalette_" .. guiId) then
    if im.InputText(string.format("##saveVehicleColorPalette_InputWidget_%s", guiId), editor.getTempCharPtr(colorPaletteName), nil, im.InputTextFlags_AutoSelectAll) then
      colorPaletteName = editor.getTempCharPtr()
    end
    if #colorPaletteName == 0 then
      im.TextColored(editor.color.warning.Value, "Name must not be empty")
    end
    if #colorPaletteName == 0 then im.BeginDisabled() end
    if im.Button(string.format("Save##VehicleColorPalette_SaveButton_%s", guiId)) then
      local palettes = editor.getPreference("dynamicDecalsTool.vehicleColorPalette.palettes")
      local c1 = vehicleObj.color
      local c2 = vehicleObj.colorPalette0
      local c3 = vehicleObj.colorPalette1
      table.insert(palettes, {name = colorPaletteName, values = {{c1.x, c1.y, c1.z}, {c2.x, c2.y, c2.z}, {c3.x, c3.y, c3.z}}})
      table.sort(palettes, function(a,b) return string.lower(a.name) < string.lower(b.name) end)
      editor.setPreference("dynamicDecalsTool.vehicleColorPalette.palettes", palettes)
      im.CloseCurrentPopup()
    end
    im.SameLine()
    if #colorPaletteName == 0 then
      im.EndDisabled()
    end
    if im.Button(string.format("Cancel##VehicleColorPalette_Save_CancelButton_%s", guiId)) then
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end

  im.SameLine()
  if im.Button(string.format("Save##vehicleColorPalette", guiId)) then
    im.OpenPopup("SaveVehicleColorPalette_" .. guiId)
  end
  im.tooltip("Save the current color palettes")

  if im.BeginPopup("LoadVehicleColorPalette_" .. guiId) then
    local palettes = editor.getPreference("dynamicDecalsTool.vehicleColorPalette.palettes")
    for k, palette in ipairs(palettes) do
      if im.Button(string.format("Load##VehicleColorPalette_LoadButton_%d%s", k, guiId)) then
        vehicleObj.color = Point4F(palette.values[1][1], palette.values[1][2], palette.values[1][3], vehicleObj.color.w)
        vehicleObj.colorPalette0 = Point4F(palette.values[2][1], palette.values[2][2], palette.values[2][3], vehicleObj.colorPalette0.w)
        vehicleObj.colorPalette1 = Point4F(palette.values[3][1], palette.values[3][2], palette.values[3][3], vehicleObj.colorPalette1.w)
      end
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.delete, tool.getIconSizeVec2(), nil, nil, nil, string.format("##vehicleColorPalette_Load_deleteButton_%d%s", k, guiId)) then
        table.remove(palettes, k)
        editor.setPreference("dynamicDecalsTool.vehicleColorPalette.palettes", palettes)
      end
      im.tooltip("Remove color palette")
      im.SameLine()
      im.ColorButton(string.format("LoadVehicleColorPalette_color1_%d%s", k, guiId), editor.getTempImVec4_TableTable({palette.values[1][1], palette.values[1][2], palette.values[1][3], 1.0}))
      im.SameLine()
      im.ColorButton(string.format("LoadVehicleColorPalette_color2_%d%s", k, guiId), editor.getTempImVec4_TableTable({palette.values[2][1], palette.values[2][2], palette.values[2][3], 1.0}))
      im.SameLine()
      im.ColorButton(string.format("LoadVehicleColorPalette_color3_%d%s", k, guiId), editor.getTempImVec4_TableTable({palette.values[3][1], palette.values[3][2], palette.values[3][3], 1.0}))
      im.SameLine()
      if editor.uiInputText(string.format("##LoadVehicleColorPalette_paletteName_InputWidget_%d%s", k, guiId), editor.getTempCharPtr(palette.name), nil, im.InputTextFlags_AutoSelectAll, nil, nil, editor.getTempBool_BoolBool(false)) then
        palette.name = editor.getTempCharPtr()
      end
      if editor.getTempBool_BoolBool() == true then
        table.sort(palettes, function(a,b) return string.lower(a.name) < string.lower(b.name) end)
        editor.setPreference("dynamicDecalsTool.vehicleColorPalette.palettes", palettes)
      end
    end
    im.Separator()
    if im.Button(string.format("Close##VehicleColorPalette_Load_CloseButton_%s", guiId)) then
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end

  im.SameLine()
  if im.Button(string.format("Load##vehicleColorPalette", guiId)) then
    im.OpenPopup("LoadVehicleColorPalette_" .. guiId)
  end
  im.tooltip("Load color palettes")

  im.SameLine()
  if editor.uiIconImageButton(editor.icons.settings, tool.getIconSizeVec2(), nil, nil, nil, "##openVehicleColorPalettePreferences") then
    editor.showPreferences("dynamicDecalsTool")
  end
  im.tooltip("Open editor preferences window")

  if im.TreeNodeEx1("one", im.TreeNodeFlags_DefaultOpen) then
    im.Columns(2, "ColorPalette1Column")
    im.SetColumnWidth(0, tool.getIconSize() + 2 * im.GetStyle().ItemSpacing.x)
    if editor.uiIconImageButton(editor.icons.arrow_drop_up, tool.getIconSizeVec2(), nil, nil, nil, "##colorPalette1_UpButton") then
      local newC3 = vehicleObj.color
      vehicleObj.color = vehicleObj.colorPalette1
      vehicleObj.colorPalette1 = newC3
    end
    im.tooltip("Switch with color palette 3")
    if editor.uiIconImageButton(editor.icons.arrow_drop_down, tool.getIconSizeVec2(), nil, nil, nil, "##colorPalette1_DownButton") then
      local newC2 = vehicleObj.color
      vehicleObj.color = vehicleObj.colorPalette0
      vehicleObj.colorPalette0 = newC2
    end
    im.tooltip("Switch with color palette 2")
    im.NextColumn()

    local color = vehicleObj.color
    im.TextUnformatted("Color")
    im.SameLine()
    im.PushItemWidth(im.GetContentRegionAvailWidth() - (im.GetStyle().ItemSpacing.x + tool.getIconSize()))
    if im.ColorEdit3("##VehicleColorPaletteColor0_" .. guiId, editor.getTempFloatArray3_TableTable({color.x, color.y, color.z})) then
      local val = editor.getTempFloatArray3_TableTable()
      vehicleObj.color = Point4F(val[1], val[2], val[3], color.w)
    end
    im.PopItemWidth()
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.rotate, tool.getIconSizeVec2(), nil, nil, nil, "##randomizeColor1") then
      randomizeSeed()
      local colors = editor.getPreference("dynamicDecalsTool.colorPresets.presets")
      if #colors == 0 then
        editor.logWarn(logTag .. ": There's not enough preset colors to pick from.")
        return
      end
      local col = colors[math.ceil(math.random() * #colors)]
      vehicleObj.color = Point4F(col.value[1], col.value[2], col.value[3], vehicleObj.color.w)
      editor.log(string.format("%s: Randomized color; Color palette 1 set to '%s'", logTag, col.name))
    end
    im.tooltip("Randomize color. Picks a random color from the color presets and applies it.")

    metallicPaintDataGui(vehicleObj, 0, guiId)

    im.TreePop()
    im.Columns(1)
  end

  if im.TreeNodeEx1("two", im.TreeNodeFlags_DefaultOpen) then
    im.Columns(2, "ColorPalette2Column")
    im.SetColumnWidth(0, tool.getIconSize() + 2 * im.GetStyle().ItemSpacing.x)
    if editor.uiIconImageButton(editor.icons.arrow_drop_up, tool.getIconSizeVec2(), nil, nil, nil, "##colorPalette2_UpButton") then
      local newC1 = vehicleObj.colorPalette0
      vehicleObj.colorPalette0 = vehicleObj.color
      vehicleObj.color = newC1
    end
    im.tooltip("Switch with color palette 1")
    if editor.uiIconImageButton(editor.icons.arrow_drop_down, tool.getIconSizeVec2(), nil, nil, nil, "##colorPalette2_DownButton") then
      local newC3 = vehicleObj.colorPalette0
      vehicleObj.colorPalette0 = vehicleObj.colorPalette1
      vehicleObj.colorPalette1 = newC3
    end
    im.tooltip("Switch with color palette 3")
    im.NextColumn()

    local color = vehicleObj.colorPalette0
    im.TextUnformatted("Color")
    im.SameLine()
    im.PushItemWidth(im.GetContentRegionAvailWidth() - (im.GetStyle().ItemSpacing.x + tool.getIconSize()))
    if im.ColorEdit3("##VehicleColorPaletteColor1_" .. guiId, editor.getTempFloatArray3_TableTable({color.x, color.y, color.z})) then
      local val = editor.getTempFloatArray3_TableTable()
      vehicleObj.colorPalette0 = Point4F(val[1], val[2], val[3], color.w)
    end
    im.PopItemWidth()
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.rotate, tool.getIconSizeVec2(), nil, nil, nil, "##randomizeColor2") then
      randomizeSeed()
      local colors = editor.getPreference("dynamicDecalsTool.colorPresets.presets")
      if #colors == 0 then
        editor.logWarn(logTag .. ": There's not enough preset colors to pick from.")
        return
      end
      local col = colors[math.ceil(math.random() * #colors)]
      vehicleObj.colorPalette0 = Point4F(col.value[1], col.value[2], col.value[3], vehicleObj.color.w)
      editor.log(string.format("%s: Randomized color; Color palette 2 set to '%s'", logTag, col.name))
    end
    im.tooltip("Randomize color. Picks a random color from the color presets and applies it.")

    metallicPaintDataGui(vehicleObj, 1, guiId)

    im.TreePop()
    im.Columns(1)
  end

  if im.TreeNodeEx1("three", im.TreeNodeFlags_DefaultOpen) then
    im.Columns(2, "ColorPalette3Column")
    im.SetColumnWidth(0, tool.getIconSize() + 2 * im.GetStyle().ItemSpacing.x)
    if editor.uiIconImageButton(editor.icons.arrow_drop_up, tool.getIconSizeVec2(), nil, nil, nil, "##colorPalette3_UpButton") then
      local newC2 = vehicleObj.colorPalette1
      vehicleObj.colorPalette1 = vehicleObj.colorPalette0
      vehicleObj.colorPalette0 = newC2
    end
    im.tooltip("Switch with color palette 2")
    if editor.uiIconImageButton(editor.icons.arrow_drop_down, tool.getIconSizeVec2(), nil, nil, nil, "##colorPalette3_DownButton") then
      local newC1 = vehicleObj.colorPalette1
      vehicleObj.colorPalette1 = vehicleObj.color
      vehicleObj.color = newC1
    end
    im.tooltip("Switch with color palette 1")
    im.NextColumn()

    local color = vehicleObj.colorPalette1
    im.TextUnformatted("Color")
    im.SameLine()
    im.PushItemWidth(im.GetContentRegionAvailWidth() - (im.GetStyle().ItemSpacing.x + tool.getIconSize()))
    if im.ColorEdit3("##VehicleColorPaletteColor2_" .. guiId, editor.getTempFloatArray3_TableTable({color.x, color.y, color.z})) then
      local val = editor.getTempFloatArray3_TableTable()
      vehicleObj.colorPalette1 = Point4F(val[1], val[2], val[3], color.w)
    end
    im.PopItemWidth()
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.rotate, tool.getIconSizeVec2(), nil, nil, nil, "##randomizeColor3") then
      randomizeSeed()
      local colors = editor.getPreference("dynamicDecalsTool.colorPresets.presets")
      if #colors == 0 then
        editor.logWarn(logTag .. ": There's not enough preset colors to pick from.")
        return
      end
      local col = colors[math.ceil(math.random() * #colors)]
      vehicleObj.colorPalette1 = Point4F(col.value[1], col.value[2], col.value[3], vehicleObj.color.w)
      editor.log(string.format("%s: Randomized color; Color palette 3 set to '%s'", logTag, col.name))
    end
    im.tooltip("Randomize color. Picks a random color from the color presets and applies it.")

    metallicPaintDataGui(vehicleObj, 2, guiId)

    im.TreePop()
    im.Columns(1)
  end
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "vehicleColorPalette", nil, {
    {palettes = {"table", {
      {values={{1,0.40000003576279,0},{0,0,0},{1,1,1}},name="BeamNG"},
      {values={{0.33725491166115,0.39607846736908,0.30196079611778},{0.61176472902298,0.5686274766922,0.46666669845581},{ 0.39607846736908,0.34901961684227,0.28627452254295}},name="Camouflage"},
      {values={{0.36862745881081,0.18823531270027,0.13725490868092},{0.75294125080109,0.52156865596771,0.32156863808632},{0.95294123888016,0.91372555494308,0.86274516582489}},name="Chocolate"},
      {values={{0.60784316062927,0.43529415130615,0.25098040699959},{0.75686281919479,0.52941179275513,0.27843138575554},{0.89803928136826,0.71544831991196,0.50008457899094}},name="Dunes"},
      {values={{0.85155194997787,0.039455629885197,0.039455629885197},{0.51417005062103,0.088586218655109,0.088586218655109},{0.92802518606186,0.87792932987213,0.87792932987213}},name="Fast Car"},
      {values={{0,0.14901961386204,0.32941177487373},{1,1,1},{0.92941182851791,0.16078431904316,0.22352942824364}},name="France"},
      {values={{0.59423303604126,0.85546106100082,0.94117653369904},{0.41960787773132,0.65490198135376,0.80000007152557},{0.79607850313187,0.94509810209274,0.98039221763611}},name="Icy"},
      {values={{0,0.54901963472366,0.27058824896812},{0.95686280727386,0.9764706492424,1},{0.80392163991928,0.1294117718935,0.16470588743687}},name="Italy"},
      {values={{0.94509810209274,0.89019614458084,0.95294123888016},{0.7607843875885,0.73333334922791,0.94117653369904},{0.56078433990479,0.72156864404678,0.92941182851791}},name="Lilac"},
      {values={{0.04313725605607,0.074509806931019,0.16862745583057},{0.10980392992496,0.14509804546833,0.2549019753933},{0.22745099663734,0.3137255012989,0.41960787773132}},name="Midnight"},
      {values={{0.29803922772408,0.30196079611778,0.04313725605607},{0.4078431725502,0.41176474094391,0.062745101749897},{0.66274511814117,0.6745098233223,0.34901961684227}},name="Moss"},
      {values={{0.011764707043767,0.48235297203064,0.74509805440903},{0.019607843831182,0.37647062540054,0.61176472902298},{0.26666668057442,0.71372550725937,0.90980398654938}},name="Neptune",},
      {values={{0.92941182851791,0.41568630933762,0.35294118523598},{0.95686280727386,0.94509810209274,0.73333334922791},{0.60784316062927,0.75686281919479,0.73725491762161}},name="Pastelle"},
      {values={{0.7294117808342,0.73333334922791,0.74901962280273},{0.49803924560547,0.1803921610117,0.39215689897537},{0.37647062540054,0.43137258291245,0}},name="Retro Handheld Console"},
      {values={{0.99607849121094,0.70588237047195,0.48235297203064},{1,0.49411767721176,0.37254902720451},{0.4627451300621,0.32156863808632,0.52156865596771}},name="Sunset"},
      {values={{0.37022042274475,0.37022042274475,0.37022042274475},{0.69860553741455,0.69860553741455,0.69860553741455},{0.028340101242065,0.028340101242065,0.028340101242065}},name="Tape Deck"},
      {values={{0.93333339691162,0.10980392992496,0.47450983524323},{0.89803928136826,0.76470595598221,0.41176474094391},{0.29803922772408,0.68627452850342,0.78823536634445}},name="The 80's",},
      {values={{0.11764706671238,0.11764706671238,0.14117647707462},{0.57254904508591,0.078431375324726,0.047058828175068},{1,0.97254908084869,0.94117653369904}},name="Vintage"},
      {values={{0.29019609093666,0.36862745881081,0.26666668057442},{0.37254902720451,0.53725492954254,0.19607844948769},{0.87843143939972,0.2039215862751,0.19607844948769}},name="Watermelon"},
    }, "Vehicle Color Palettes", nil, nil, nil, nil, nil,
    function(cat, subCat, item)
      local guiId = "editorPrefs"
      local palettes = editor.getPreference("dynamicDecalsTool.vehicleColorPalette.palettes")
      local vehicleObj = getPlayerVehicle(0)
      if not vehicleObj then
        im.TextUnformatted("No vehicle")
        return
      end
      for k, palette in ipairs(palettes) do
        if editor.getPreference("dynamicDecalsTool.general.debug") then
          if im.Button(string.format("Dump##LoadVehicleColorPalette_dumpButton_%d%s", k, guiId)) then
            print(dumps(palette))
          end
          im.SameLine()
        end
        if im.Button(string.format("Load##VehicleColorPalette_LoadButton_%d%s", k, guiId)) then
          vehicleObj.color = Point4F(palette.values[1][1], palette.values[1][2], palette.values[1][3], vehicleObj.color.w)
          vehicleObj.colorPalette0 = Point4F(palette.values[2][1], palette.values[2][2], palette.values[2][3], vehicleObj.colorPalette0.w)
          vehicleObj.colorPalette1 = Point4F(palette.values[3][1], palette.values[3][2], palette.values[3][3], vehicleObj.colorPalette1.w)
          im.CloseCurrentPopup()
        end
        im.SameLine()
        if editor.uiIconImageButton(editor.icons.delete, tool.getIconSizeVec2(), nil, nil, nil, string.format("##vehicleColorPalette_Load_deleteButton_%d%s", k, guiId)) then
          table.remove(palettes, k)
          editor.setPreference("dynamicDecalsTool.vehicleColorPalette.palettes", palettes)
        end
        im.tooltip("Remove color palette")
        im.SameLine()
        im.ColorButton(string.format("LoadVehicleColorPalette_color1_%d%s", k, guiId), editor.getTempImVec4_TableTable({palette.values[1][1], palette.values[1][2], palette.values[1][3], 1.0}))
        im.SameLine()
        im.ColorButton(string.format("LoadVehicleColorPalette_color2_%d%s", k, guiId), editor.getTempImVec4_TableTable({palette.values[2][1], palette.values[2][2], palette.values[2][3], 1.0}))
        im.SameLine()
        im.ColorButton(string.format("LoadVehicleColorPalette_color3_%d%s", k, guiId), editor.getTempImVec4_TableTable({palette.values[3][1], palette.values[3][2], palette.values[3][3], 1.0}))
        im.SameLine()
        im.PushItemWidth(im.GetContentRegionAvailWidth())
        if editor.uiInputText(string.format("##LoadVehicleColorPalette_paletteName_InputWidget_%d%s", k, guiId), editor.getTempCharPtr(palette.name), nil, im.InputTextFlags_AutoSelectAll, nil, nil, editor.getTempBool_BoolBool(false)) then
          palette.name = editor.getTempCharPtr()
        end
        if editor.getTempBool_BoolBool() == true then
          table.sort(palettes, function(a,b) return string.lower(a.name) < string.lower(b.name) end)
          editor.setPreference("dynamicDecalsTool.vehicleColorPalette.palettes", palettes)
        end
        im.PopItemWidth()
      end
    end
  }},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
The Vehicle Color Palette section is the central place that lets you easily alter vehicle colors so you can quickly check how they affect your design.
]])
  im.PopTextWrapPos()
end

local function editModeUpdate(dtReal, dtSim, dtRaw)
  if restoreColors then
    if restoreColors.timer <= 0 then
      local vehicleObj = getPlayerVehicle(0)
      if vehicleObj then
        vehicleObj.color = Point4F(restoreColors.colors[1][1], restoreColors.colors[1][2], restoreColors.colors[1][3], restoreColors.colors[1][4])
        vehicleObj.colorPalette0 = Point4F(restoreColors.colors[2][1], restoreColors.colors[2][2], restoreColors.colors[2][3], restoreColors.colors[2][4])
        vehicleObj.colorPalette1 = Point4F(restoreColors.colors[3][1], restoreColors.colors[3][2], restoreColors.colors[3][3], restoreColors.colors[3][4])
        restoreColors = nil
        return
      else
        editor.logWarn(string.format("%s: Can't restore vehicle color palette", logTag))
        -- restoreColors.timer = 1
      end
    else
      restoreColors.timer = restoreColors.timer - 1
    end
  end
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  docs = extensions.editor_dynamicDecals_docs

  tool.registerSection("Vehicle Color Palette", onGui, 120, false, {}, {
    {icon = editor.icons.help_outline, tooltip = "Docs", fn = function() docs.selectSection({"Vehicle Color Palette"}) end},
  })
  tool.registerEditorOnUpdateFn("vehicleColorPalette", editModeUpdate)
  docs.register({section = {"Vehicle Color Palette"}, guiFn = documentationGui})
end

M.onSerialize = function()
  local vehicleObj = getPlayerVehicle(0)
  local colors = nil
  if vehicleObj then
    colors = {
      {vehicleObj.color.x, vehicleObj.color.y, vehicleObj.color.z, vehicleObj.color.w},
      {vehicleObj.colorPalette0.x, vehicleObj.colorPalette0.y, vehicleObj.colorPalette0.z, vehicleObj.colorPalette0.w},
      {vehicleObj.colorPalette1.x, vehicleObj.colorPalette1.y, vehicleObj.colorPalette1.z, vehicleObj.colorPalette1.w},
    }
  end

  return {
    colors = colors
  }
end

M.onDeserialized = function(data)
  if data.colors then
    restoreColors = {timer = 5, colors = deepcopy(data.colors)}
  end
end

M.onGui = onGui
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M