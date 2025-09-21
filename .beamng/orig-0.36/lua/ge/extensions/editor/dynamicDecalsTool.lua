-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local ffi = require('ffi')

local M = {}
M.dependencies = {
  "editor_api_dynamicDecals",
  "editor_dynamicDecals_docs",
  "editor_dynamicDecals_helper",
  "editor_dynamicDecals_widgets",
  "editor_dynamicDecals_notification",
  "editor_dynamicDecals_news",
  "editor_dynamicDecals_layerStack",
  "editor_dynamicDecals_selection",
  "editor_dynamicDecals_inspector",
  "editor_dynamicDecals_inspector_utils",
  "editor_dynamicDecals_browser",
  "editor_dynamicDecals_textures",
  "editor_dynamicDecals_export",
  "editor_dynamicDecals_fonts",
  "editor_dynamicDecals_brushes",
  "editor_dynamicDecals_gizmo",
  "editor_dynamicDecals_vehicleColorPalette",
  "editor_dynamicDecals_colorPresets",
  "editor_dynamicDecals_colorHistory",
  "editor_dynamicDecals_camera",
  "editor_dynamicDecals_loadSave",
  "editor_dynamicDecals_settings",
  "editor_dynamicDecals_history",
  "editor_dynamicDecals_meshes",
  "editor_dynamicDecals_layerTypes_decal",
  "editor_dynamicDecals_layerTypes_brushStroke",
  "editor_dynamicDecals_layerTypes_path",
  "editor_dynamicDecals_layerTypes_fill",
  "editor_dynamicDecals_layerTypes_textureFill",
  "editor_dynamicDecals_layerTypes_group",
  "editor_dynamicDecals_layerTypes_linkedSet",
  "editor_dynamicDecals_debugTextures",
  "editor_dynamicDecals_debugSection",
}
local api
local deps = {}

local im = ui_imgui
local logTag = 'editor_dynamicDecalsTool'

M.version = {1,1,0}

-- window names
local toolWindowName = 'dynamicDecalsTool_MainWindow'
local openToolWindowPopupName = 'dynamicDecalsTool_OpenMainWindowPopup'

local iconSize = 20
local iconSizeVec2 = im.ImVec2(0, 0)
local mainScrollY = 0

local brushStrokeEnabled = false
local currentMaskEditingLayerUid = nil

local editorOnUpdateFunctions = {}
local toolbarToolItems = {}
local toolbarActionItems = {}
local onEditorGuiFunctions = {}

local imguiStyle = nil

local debugPref = false

local sections = {}

M.toolModes = {
  none = 0,
  decal = 1,
  brushStroke = 2,
  path = 3,
}
M.toolMode = M.toolModes.decal

M.doApiUpdate = true

local function sectionNode(section, nodeColor)
  if section.setScroll == true then
    im.SetScrollHereY()
    section.setScroll = false
  end
  local colHeaderTitle = debugPref and string.format("!! %s %d", section.name, section.order) or section.name
  local beforeCPos = im.GetCursorPos()
  im.PushStyleColor2(im.Col_Button, nodeColor or im.ImVec4(1,102/255, 0, 0.2))
  local buttonHeight = math.ceil(im.GetFontSize()) + imguiStyle.FramePadding.y
  local headerWidth = im.GetContentRegionAvailWidth() - (section.buttons and #section.buttons or 0) * (buttonHeight + imguiStyle.ItemSpacing.x)
  if im.Button(string.format("##%s", colHeaderTitle), im.ImVec2(headerWidth, buttonHeight)) then
    section.open = not section.open
  end
  local afterCPos = im.GetCursorPos()
  im.PopStyleColor(1)
  if section.buttons then
    for k, button in ipairs(section.buttons) do
      im.SetCursorPos(im.ImVec2(beforeCPos.x + headerWidth + k * (imguiStyle.ItemSpacing.x) + (k - 1) * buttonHeight, beforeCPos.y))
      if editor.uiIconImageButton(button.icon, im.ImVec2(buttonHeight, buttonHeight), nil, nil, nodeColor or im.ImVec4(1,102/255, 0, 0.2), string.format("SectionButton_%s_%d", section.name, k)) then
        button.fn()
      end
      if button.tooltip then im.tooltip(button.tooltip) end
    end
  end

  im.SetCursorPos(im.ImVec2(beforeCPos.x + imguiStyle.FramePadding.x, beforeCPos.y + imguiStyle.FramePadding.y))
  -- button icon
  local iconSize = math.ceil(im.GetFontSize())
  editor.uiIconImage(section.open and editor.icons.keyboard_arrow_down or editor.icons.keyboard_arrow_right, im.ImVec2(iconSize, iconSize), im.GetStyleColorVec4(im.Col_Text))
  -- button text
  im.SetCursorPos(im.ImVec2(beforeCPos.x + 2 * imguiStyle.FramePadding.x + iconSize, beforeCPos.y + imguiStyle.FramePadding.y))
  im.TextUnformatted(colHeaderTitle)
  im.SetCursorPos(afterCPos)
  if section.open then
    if section.window then
      if editor.isWindowVisible(section.windowName) then
        if im.Button(string.format("Close external window##%s", section.name)) then
          editor.hideWindow(section.windowName)
        end
      else
        if im.Button(string.format("Open in external window##%s", section.name)) then
          editor.showWindow(section.windowName)
        end
      end
      im.Separator()
    end
    section.guiFn(string.format("%s_%s", section.name, "section"))
  end
end

local function advancedSectionsGui()
  local nodeColorTbl = editor.getPreference("dynamicDecalsTool.general.advancedSectionsNodeColor")
  local nodeColor = im.ImVec4(nodeColorTbl[1], nodeColorTbl[2], nodeColorTbl[3], nodeColorTbl[4])
  for _, section in ipairs(sections) do
    if section.order >= 1000 then
      sectionNode(section, nodeColor)
    end
  end
end

local advancedSection = {name = "Advanced", order = 1000, open = nil, window = {}, guiFn = advancedSectionsGui}

-- ##### GUI Start #####
M.getSectionWindowName =  function(section)
  return string.format("Dynamic Decals Tool - %s", section.windowName)
end

local function onEditorGui()
  if not DecalShapeRenderApp then return end

  if editor.beginModalWindow(openToolWindowPopupName, "Open now?") then
    im.PushTextWrapPos(im.GetContentRegionAvailWidth())
    im.TextUnformatted("Would you like to open the Vehicle Livery Editor now?")
    local space = im.GetContentRegionAvailWidth()
    local style = im.GetStyle()
    if im.Button("Cancel", im.ImVec2((space - style.ItemSpacing.x) / 2, 0)) then
      editor.hideWindow(openToolWindowPopupName)
    end
    im.SameLine()
    if im.Button("Ok", im.ImVec2((space - style.ItemSpacing.x) / 2, 0)) then
      editor.hideWindow(openToolWindowPopupName)
      editor.showWindow(toolWindowName)
    end
  end
  editor.endModalWindow()

  if editor.beginWindow(toolWindowName, "Dynamic Decals Tool") then
    imguiStyle = im.GetStyle()
    iconSize = math.ceil(im.GetFontSize()) + 2 * imguiStyle.FramePadding.y
    iconSizeVec2.x = iconSize
    iconSizeVec2.y = iconSize

    mainScrollY = im.GetScrollY()
    debugPref = editor.getPreference("dynamicDecalsTool.general.debug")

    -- debug
    --[[
    if im.Button("Show CEF") then
      extensions.ui_visibility.setCef(true)
    end
    im.SameLine()
    if im.Checkbox("Do api.onUpdate_", editor.getTempBool_BoolBool(M.doApiUpdate)) then
      M.doApiUpdate = editor.getTempBool_BoolBool()
    end
    im.SameLine()
    if im.Button("Pop action map") then
      popActionMap("dynamicDecals")
    end
    ]]

    local nodeColorTbl = editor.getPreference("dynamicDecalsTool.general.sectionsNodeColor")
    local nodeColor = im.ImVec4(nodeColorTbl[1], nodeColorTbl[2], nodeColorTbl[3], nodeColorTbl[4])
    for _, section in ipairs(sections) do
      if section.order < 1000 then
        sectionNode(section, nodeColor)
      end
    end

    im.Separator()
    im.Separator()
    sectionNode(advancedSection, nodeColor)
  end
  editor.endWindow()

  if editor.isWindowVisible(toolWindowName) then
    for _, fn in pairs(onEditorGuiFunctions) do
      fn()
    end
  end

  for _, section in ipairs(sections) do
    -- Draw windows
    if section.window then
      if editor.beginWindow(section.windowName, M.getSectionWindowName(section)) then
        section.guiFn(string.format("%s_%s", section.name, "window"))
      end
      editor.endWindow()
    end
  end

  if advancedSection.window then
    if editor.beginWindow(advancedSection.windowName, M.getSectionWindowName(advancedSection)) then
      advancedSection.guiFn(string.format("%s_%s", advancedSection.name, "window"))
    end
    editor.endWindow()
  end
end
-- ##### GUI End #####

local function onWindowMenuItem()
  editor.showWindow(toolWindowName)
end

local function dynamicDecalsToolEditModeToolbar()
  local vertSeparatorHeight = editor.getDefaultIconButtonSize().y

  -- ABOUT
  if editor.uiIconImageButton(editor.icons.new_releases, nil, nil, nil, nil, "AboutToolbarButton") then
    deps.news.showWindow()
  end
  im.tooltip("About")
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.library_books, nil, deps.docs.isWindowVisible() and editor.color.beamng.Value or nil, nil, nil, "DocumentationToolbarButton") then
    if deps.docs.isWindowVisible() then
      deps.docs.hideWindow()
    else
      deps.docs.showWindow()
    end
  end
  im.tooltip("Documentation")
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.settings, nil, editor.isWindowVisible("preferences") and editor.color.beamng.Value or nil, nil, nil, "PreferencesToolbarButton") then
    if editor.isWindowVisible("preferences") then
      editor.hideWindow("preferences")
    else
      editor.showPreferences("dynamicDecalsTool")
    end
  end
  im.tooltip("Preferences")
  im.SameLine()

  editor.uiVertSeparator(vertSeparatorHeight, im.ImVec2(0,0), 2)

  -- TODO: Let layer type modules add these icons
  -- LOAD PROJECT, SAVE PROJECT
  if editor.uiIconImageButton(editor.icons.folder, nil, nil, nil, nil, "LoadProjectToolbarButton") then
    deps.loadSave.loadFileDialog()
  end
  im.tooltip("Load")
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.save, nil, nil, nil, nil, "SaveProjectAsToolbarButton") then
    deps.loadSave.saveAsFileDialog()
  end
  im.tooltip("Save as...")

  local currentProjectFile = deps.loadSave.getCurrentPorjectFilePath()
  local _, __, ext = path.split(currentProjectFile or "")
  if ext == "" then im.BeginDisabled() end
  im.SameLine()
  if editor.uiIconImageButton(editor.icons.save, nil, nil, nil, nil, "SaveProjectToolbarButton") then
    api.saveLayerStackToFile(currentProjectFile)
  end
  im.tooltip(string.format("Save project\nOverwrites %s", currentProjectFile))
  if ext == "" then im.EndDisabled() end

  editor.uiVertSeparator(vertSeparatorHeight, im.ImVec2(0,0), 2)

  -- TODO: Let layer type modules add these icons
  -- BROWSER
  if editor.uiIconImageButton(editor.icons.ab_thumbnails_small, nil, deps.browser.isWindowVisible() and editor.color.beamng.Value or nil, nil, nil, "ToggleBrowserToolbarButton") then
    if deps.browser.isWindowVisible() then
      deps.browser.hideWindow()
    else
      deps.browser.showWindow()
    end
  end
  im.tooltip("Browser")

  editor.uiVertSeparator(vertSeparatorHeight, im.ImVec2(0,0), 2)

  if editor.uiIconImageButton(editor.icons.terrain_reall_smooth_new, nil, api.getUseSurfaceNormal() and editor.color.beamng.Value or nil, nil, nil, "ToggleUseSurfaceNormalToolbarButton") then
    api.setUseSurfaceNormal(not api.getUseSurfaceNormal())
  end
  im.tooltip("Toggle 'Use Surface Normal' setting\nIf enabled the tool calculates a normal based on the vehicle mesh.")
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.adjust, nil, (bit.band(api.getSettings(), api.settingsFlags.UseMousePos.value) == api.settingsFlags.UseMousePos.value) and editor.color.beamng.Value or nil, nil, nil, "ToggleUseMousePosToolbarButton") then
    api.toggleSetting(api.settingsFlags.UseMousePos.value)
  end
  im.tooltip(string.format("Toggle 'Use Mouse Position' setting\n%s", api.settingsFlags.UseMousePos.description))

  editor.uiVertSeparator(vertSeparatorHeight, im.ImVec2(0,0), 2)

  -- TODO: Let layer type modules add these icons
  -- DECAL TOOL, BRUSH STROKE TOOL, PATH TOOL
  if editor.uiIconImageButton(editor.icons.brush, nil, M.toolMode == M.toolModes.decal and editor.color.beamng.Value or nil, nil, nil, "AddDecalToolbarButton") then
    deps.selection.deselectLayer()
    M.toolMode = M.toolModes.decal
    deps.gizmo.setTransformMode(deps.gizmo.transformModes.none)
    -- disable path layer
    M.enablePathLayer(0)
    api.setProjectDynamicDecalsState(true)
  end
  im.tooltip("Decal tool")

  im.SameLine()
  if editor.uiIconImageButton(editor.icons.palette, nil, M.toolMode == M.toolModes.brushStroke and editor.color.beamng.Value or nil, nil, nil, "BrushStrokeToolToolbarButton") then
    deps.selection.deselectLayer()
    M.toolMode = M.toolModes.brushStroke
    deps.gizmo.setTransformMode(deps.gizmo.transformModes.none)
    -- disable path layer
    M.enablePathLayer(0)
  end
  im.tooltip("Brush Stroke tool")

  im.SameLine()
  if editor.uiIconImageButton(editor.icons.tb_left_curve_longer, nil, api.getEnablePathLayer() and editor.color.beamng.Value or nil, nil, nil, "PathToolToolbarButton") then
    deps.selection.deselectLayer()
    M.toolMode = M.toolModes.path
    deps.gizmo.setTransformMode(deps.gizmo.transformModes.none)
    M.enablePathLayer(1)
  end
  im.tooltip("Path tool")

  editor.uiVertSeparator(vertSeparatorHeight, im.ImVec2(0,0), 2)

  -- DECAL TOOL, BRUSH STROKE TOOL, PATH TOOL
  for _, item in ipairs(toolbarToolItems) do
    item.guiFn()
    im.SameLine()
  end
  -- ~~~DECAL TOOL, BRUSH STROKE TOOL, PATH TOOL

  for _, item in ipairs(toolbarActionItems) do
    item.guiFn()
    im.SameLine()
  end

  -- TRANSFORM TOOL
  local selectionData = editor.selection["dynamicDecalLayer"]
  if selectionData then
    editor.uiVertSeparator(vertSeparatorHeight, im.ImVec2(0,0), 2)

    if deps.gizmo.translateFn then
      -- if editor.uiIconImageButton(editor.icons.move, nil, api.projectDynamicDecals == false and editor.color.beamng.Value or nil, nil, nil, "Move tool") then
      if editor.uiIconImageButton(editor.icons.move, nil, deps.gizmo.getTransformMode() == deps.gizmo.transformModes.translate and editor.color.beamng.Value or nil, nil, nil, "Move tool") then
        deps.gizmo.setTransformMode(deps.gizmo.transformModes.translate)
        editor.setAxisGizmoMode(editor.AxisGizmoMode_Translate)
        editor.setAxisGizmoAlignment(editor.AxisGizmoAlignment_Local)
        -- worldEditorCppApi:enableAxisGizmoTranslateAxes(true, true, true)
        -- worldEditorCppApi:setAxisGizmoHideDisabledTranslateAxes(false)
        -- worldEditorCppApi:setAxisGizmoRenderPlane(false)
        -- worldEditorCppApi:setAxisGizmoRenderPlaneHashes(false)
        -- worldEditorCppApi:setAxisGizmoRenderMoveGrid(false)
        -- worldEditorCppApi:setGizmoPlaneGridLineColor(ColorI(255, 0, 0, 255))

      end
      im.tooltip("Move tool")
    end

    if deps.gizmo.rotateFn then
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.rotate, nil, deps.gizmo.getTransformMode() == deps.gizmo.transformModes.rotate and editor.color.beamng.Value or nil, nil, nil, "Rotate tool") then
        deps.gizmo.setTransformMode(deps.gizmo.transformModes.rotate)
        editor.setAxisGizmoMode(editor.AxisGizmoMode_Rotate)
        editor.setAxisGizmoAlignment(editor.AxisGizmoAlignment_Local)
        -- worldEditorCppApi:enableAxisGizmoRotateAxes(false, true, false)
        -- worldEditorCppApi:setAxisGizmoHideDisabledRotateAxes(true)
        -- worldEditorCppApi:setAxisGizmoRenderPlane(false)
        -- worldEditorCppApi:setAxisGizmoRenderPlaneHashes(false)
        -- worldEditorCppApi:setAxisGizmoRenderMoveGrid(false)
        -- worldEditorCppApi:setGizmoPlaneGridLineColor(ColorI(255, 0, 0, 255))

      end
      im.tooltip("Rotate tool")
    end

    if deps.gizmo.scaleFn then
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.scale, nil, deps.gizmo.getTransformMode() == deps.gizmo.transformModes.scale and editor.color.beamng.Value or nil, nil, nil, "Scale tool") then
        deps.gizmo.setTransformMode(deps.gizmo.transformModes.scale)
        editor.setAxisGizmoMode(editor.AxisGizmoMode_Scale)
        editor.setAxisGizmoAlignment(editor.AxisGizmoAlignment_Local)
        -- worldEditorCppApi:enableAxisGizmoScaleAxes(true, true, true)
        -- worldEditorCppApi:setAxisGizmoHideDisabledScaleAxes(false)
        -- worldEditorCppApi:setAxisGizmoRenderPlane(false)
        -- worldEditorCppApi:setAxisGizmoRenderPlaneHashes(false)
        -- worldEditorCppApi:setAxisGizmoRenderMoveGrid(false)
        -- worldEditorCppApi:setGizmoPlaneGridLineColor(ColorI(255, 0, 0, 255))
      end
      im.tooltip("Scale tool")
    end
  end

  editor.uiVertSeparator(vertSeparatorHeight, im.ImVec2(0,0), 2)

  im.SameLine()
  im.Dummy(im.ImVec2(im.GetStyle().ItemSpacing.x, 0))

  if api.getLockDepth() then
    im.SameLine()
    im.TextColored(editor.color.warning.Value, "DEPTH BUFFER LOCKED")
  end

  if api.getLockSurfaceNormal() then
    im.SameLine()
    im.TextColored(editor.color.warning.Value, "SURFACE NORMAL LOCKED")
  end

  -- Layer Masking
  if currentMaskEditingLayerUid then
    im.SameLine()
    im.TextColored(editor.color.warning.Value, "LAYER MASKING ENABLED")
  end
  -- ~ Layer Masking
end

local function dynamicDecalsEditModeUpdate(dtReal, dtSim, dtRaw)
  if api then
    if M.doApiUpdate then
      api.onUpdate_()
    end
    deps.gizmo.editModeUpdate(dtReal, dtSim, dtRaw)
  end

  for _, onUpdateFn in pairs(editorOnUpdateFunctions) do
    onUpdateFn(dtReal, dtSim, dtRaw)
  end
end

local function getDependencyFile(str)
  local file = ""
  for word in str:gmatch("%w+") do
    file = word
  end
  return file
end

local function layerTypesDocumentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
Layer types are distinct categories that define the nature and purpose of individual layers.
Each layer type comes with specific attributes and functionalities tailored to its role, allowing you to build your personalized designs.

Decal Layers:
These layers represent individual decals that you can place on the vehicle. Decal layers offer a range of properties, such as texture, color, scale, rotation, and more.
They're perfect for adding images, patterns, logos, and other graphic elements to your design.

Fill Layers:
This basic layer type enables you to apply a single color to the entire vehicle.
Fill layers provide a foundational base color for your design before adding more intricate details.

Texture Fill Layers:
Similar to fill layers, these layers let you apply patterns or textures to the vehicle.
The texture fills are based on an input texture, allowing for creative customization.

Path Layers:
These layers allow you to create designs that follow a specific path or curve.
Control points guide the tool in interpolating decals along the path. Path layers are excellent for creating flowing and dynamic designs.

Brush Stroke Layers:
These layers simulate the action of applying brush strokes. As you move the tool, it places a decal at each frame, mimicking the process of using a brush.
This is a creative way to add expressive or artistic elements to your designs.

Linked Set Layers:
These specialized layers allow you to bundle various properties from different layer types together.
When you edit properties within a Linked Set Layer, those changes are applied to child layers, streamlining the editing process for multiple layers at once.

Group Layers:
Group layers serve as containers to organize other layers within a hierarchical structure.
They help you manage and structure your design, making it easier to work with complex compositions.
]])
  im.PopTextWrapPos()

  im.Separator()

  for _, child in ipairs(docsSection.children) do
    if im.Button(child.name, im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then
      deps.docs.selectSection(child.name)
    end
  end
end

local function toolbarDocumentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
The toolbar is an essential segment of the dynamic decals tool.
  ]])

  deps.docs.image("/art/dynamicDecals/docs/toolbar.png", nil, "Toolbar")
  im.TextUnformatted([[
It lets you select between a bunch of different tools.

Always make sure to have the right tool for the job.

Once a supported layer is selected, additional tools might come available such as a translate, rotate or scale tool.
  ]])

  deps.docs.image("/art/dynamicDecals/docs/toolbar_with_transform_tools.png", nil, nil, "Toolbar with additional tools to transform the selected layer")
  im.PopTextWrapPos()

  deps.docs.verticalSpacing()
  im.Separator()
  im.Separator()
  deps.docs.verticalSpacing()
  dynamicDecalsToolEditModeToolbar()
end

local tblx = {}
local function onEditorInitialized()
  api = extensions.editor_api_dynamicDecals
  api.setLayerNameBuildString(editor.getPreference("dynamicDecalsTool.general.layerNameBuildString"))
  api.setup()

  editor.addWindowMenuItem("Vehicle Livery Creator", onWindowMenuItem, {groupMenuName = "Vehicles"})
  editor.registerWindow(toolWindowName, im.ImVec2(400, 400))
  -- registerWindow(windowName, defaultSize, defaultPos, defaultVisibleBoolean, modal, centered
  editor.registerWindow(openToolWindowPopupName, im.ImVec2(240, 120), nil, nil, false, true)

  editor.editModes.dynamicDecalsEditMode =
  {
    displayName = "Edit Dynamic Decals",
    onActivate = function()
      -- TODO:  Display window asking the user if the main window should be opened now.
      --        Can't be added now since editor.isWindowVisible is creating a recursive loop.
      if not editor.isWindowVisible(toolWindowName) then
        editor.showWindow(openToolWindowPopupName)
      end

      editor.selectCamera(editor.CameraType_Game)
    end,

    -- onDeactivate = dynamicDecalsEditModeDeactivate,
    onUpdate = dynamicDecalsEditModeUpdate,
    onToolbar = dynamicDecalsToolEditModeToolbar,
    actionMap = "dynamicDecals", -- if available, not required
    icon = editor.icons.format_paint,
    iconTooltip = "Vehicle Livery Creator",
    hideObjectIcons = true,
  }

  for _, dependency in ipairs(M.dependencies) do
    if string.startswith(dependency, "editor_dynamicDecals") then
      local file = getDependencyFile(dependency)
      deps[file] = extensions[dependency]
      if deps[file].setup then
        deps[file].setup(M)
      else
        editor.logWarn(string.format("The dynamicDecal tool module '%s' doesn't provide a setup function which is necessary for the module to work. Module has not been added."))
        deps[file] = nil
      end
    end
  end

  local sectionsOrder = editor.getPreference("dynamicDecalsTool.general.sectionsOrder")
  for _, section in ipairs(sections) do
    if section.window then
      section.windowName = string.format("%s##window", section.name)
      local windowSize = im.ImVec2(400, 400)
      if section.window and section.window.size then
        windowSize = section.window.size
      end
      editor.registerWindow(section.windowName, windowSize)
      if sectionsOrder[section.name] then
        section.order = sectionsOrder[section.name]
      end
    end
  end

  -- sort sections by order
  table.sort(sections, function(a,b) return a.order < b.order end)

  advancedSection.windowName = string.format("%s##window", advancedSection.name)
  local windowSize = im.ImVec2(400, 400)
  if advancedSection.window and advancedSection.window.size then
    windowSize = advancedSection.window.size
  end
  editor.registerWindow(advancedSection.windowName, windowSize)

  deps.docs.register({section = {"Layer Types"}, guiFn = layerTypesDocumentationGui})
  deps.docs.register({section = {"Toolbar"}, guiFn = toolbarDocumentationGui})

  editor.registerInspectorTypeHandler("dynamicDecalBrush", deps.inspector.inspectorGuiBrush)
  editor.registerInspectorTypeHandler("dynamicDecalLayer", deps.inspector.inspectorGuiLayer)

  -- set initial textures
  local texturesDirectoryPath = deps.textures.getTexturesDirectoryPath()
  api.setDecalTexturePath("color", texturesDirectoryPath .. "shape_circle.png")
  api.setDecalTexturePath("normal", texturesDirectoryPath .. "_normal.png")
  api.setDecalTexturePath("alpha", texturesDirectoryPath .. "_one.png")

  api.reprojectLayers()
end

local function onEditorActivated()
end

local function onEditorDeactivated()
end

local function onEditorToolWindowShow(windowName)
  -- activate the edit mode
  if windowName == toolWindowName then
    editor.selectEditMode(editor.editModes.dynamicDecalsEditMode)
    if core_vehicle_partmgmt then
      core_vehicle_partmgmt.setSkin("dynamicTextures")
    else
      editor.logWarn("core_vehicle_partmgmt is nil")
    end
    -- Open layers window by default
    editor.showWindow("Decal Stack / Layers##window")

    if editor.getPreference("dynamicDecalsTool.general.openBrowserWindowOnStartup") then
      deps.browser.showWindow()
    end

    local playerVehicle = getPlayerVehicle(0)
    if not playerVehicle then
      return
    end
    if extensions.core_vehicle_partmgmt.hasAvailablePart(playerVehicle.JBeam .. "_skin_dynamicTextures") == false then
      deps.notification.add("Main", "Current vehicle is not supported", "The current vehicle is not supported yet. The necessary files are missing.", deps.notification.levels.warning)
    end
  end
end

local function onEditorToolWindowHide(windowName)
  -- return to the default edit mode
  if windowName == toolWindowName then
    editor.selectEditMode(editor.editModes.objectSelect)

    for _, section in ipairs(sections) do
      if section.window then
        editor.hideWindow(section.windowName)
      end
    end
    editor.hideWindow(advancedSection.windowName)

    deps.browser.hideWindow()
  end
end

local function onEditorPreferenceValueChanged(path, value)
  if path == "dynamicDecalsTool.general.layerNameBuildString" then
    api.setLayerNameBuildString(value)
  end

  if path == "dynamicDecalsTool.general.sectionsOrder" then
    for _, section in ipairs(sections) do
      if value[section.name] then
        section.order = value[section.name]
      else
        section.order = section.defaultOrder
      end
    end
  end

  for _, dependency in ipairs(M.dependencies) do
    if string.startswith(dependency, "editor_dynamicDecals") and extensions[dependency].editorPreferenceValueChanged then
      extensions[dependency].editorPreferenceValueChanged(path, value)
    end
  end
end

local function onEditorRegisterPreferences(prefsRegistry)
  prefsRegistry:registerCategory("dynamicDecalsTool")
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "general", nil, {
    {debug = {"bool", false, "debug"}},
    {openBrowserWindowOnStartup = {"bool", true, "Automaticaly open the browser window when opening the tool"}},
    {version = {"table", {0,0,1}, "version", nil, nil, nil, true}},
    {layerNameBuildString = {"string", "@type{ - @colormap}", [[
Defines how the layer name is build when adding new layers.
Curly brackets can be used to add conditional parameters (e.g `@uid: @type{ - @colormap}`).
Following strings will be replaced in the process depending on the layer's properties.
* @type : layer type
* @uid : layer uid
* @colormap : color map filename (decal, brushStroke, path, textureFill)
* @alphamap : alpha map filename (decal, brushStroke, path)
* @normalmap : normal map filename (decal, brushStroke, path)
* @metallicmap : metallic map filename (decal, brushStroke, path)
* @roughnessmap : roughness map filename (decal, brushStroke, path)]],
      nil, nil, nil, nil, nil,
      function(cat, subCat, item)
        im.PushItemWidth(im.GetContentRegionAvailWidth())
        if editor.uiInputText(
          "##DynDecalsPreferences_layerNameBuildString",
          editor.getTempCharPtr(editor.getPreference("dynamicDecalsTool.general.layerNameBuildString")),
          nil,
          im.InputTextFlags_AutoSelectAll,
          nil,
          nil
        ) then
          local newValue = editor.getTempCharPtr()
          editor.setPreference("dynamicDecalsTool.general.layerNameBuildString", newValue)
        end
        im.PopItemWidth()
      end}
    },
    {dataPointSphereSize = {"float", 0.05, "dataPointSphereSize", nil, 0.01, 0.2}},
    {dataPointSphereColor = {"table", {1,0,0,0.25}, "dataPointSphereColor", nil, nil, nil, nil, nil, function(cat, subCat, item)
      if im.ColorEdit4("##prefsDataPointSphereColor", editor.getTempFloatArray4_TableTable(editor.getPreference("dynamicDecalsTool.general.dataPointSphereColor")), im.flags(im.ColorEditFlags_NoInputs, im.ColorEditFlags_AlphaPreview)) then
        editor.setPreference("dynamicDecalsTool.general.dataPointSphereColor", editor.getTempFloatArray4_TableTable())
      end
    end}},
    {sectionsNodeColor = {"table", {1, 102/255, 0, 102/255}, "Color of the section's tree nodes", nil, nil, nil, nil, nil, function(cat, subCat, item)
      if im.ColorEdit4("##prefssectionsNodeColor", editor.getTempFloatArray4_TableTable(editor.getPreference("dynamicDecalsTool.general.sectionsNodeColor")), im.flags(im.ColorEditFlags_NoInputs, im.ColorEditFlags_AlphaPreview)) then
        editor.setPreference("dynamicDecalsTool.general.sectionsNodeColor", editor.getTempFloatArray4_TableTable())
      end
    end}},
    {advancedSectionsNodeColor = {"table", {0.15, 0.55, 0.55, 102/255}, "Color of the tree node in the advanced sections", nil, nil, nil, nil, nil, function(cat, subCat, item)
      if im.ColorEdit4("##prefsAdvancedSectionsNodeColor", editor.getTempFloatArray4_TableTable(editor.getPreference("dynamicDecalsTool.general.advancedSectionsNodeColor")), im.flags(im.ColorEditFlags_NoInputs, im.ColorEditFlags_AlphaPreview)) then
        editor.setPreference("dynamicDecalsTool.general.advancedSectionsNodeColor", editor.getTempFloatArray4_TableTable())
      end
    end}},
    {sectionsOrder = {"table", {}, "Order of the sections in the tool window.\nThe lower the number, the higher up the section is displayed.\nIf the number is 1000 or greater, the section will appear in the advanced sections.", nil, nil, nil, nil, nil, function(cat, subCat, item)
      for _, section in ipairs(sections) do
        im.PushItemWidth(im.CalcTextSize("Texture Fill Layer Properties").x + 2 * im.GetStyle().FramePadding.x)
        im.InputText(string.format("##sectionsOrderPrefs_%s_nameInput", section.name), editor.getTempCharPtr(section.name), nil, im.InputTextFlags_ReadOnly)
        im.tooltip("read-only")
        im.PopItemWidth()
        im.SameLine()
        im.PushItemWidth(im.GetContentRegionAvailWidth())
        if editor.uiInputInt(string.format("##sectionsOrderPrefs_%s_input", section.name), editor.getTempInt_NumberNumber(section.order), 1, 10, nil, editor.getTempBool_BoolBool(false)) then

        end
        im.PopItemWidth()
        if editor.getTempBool_BoolBool() then
          section.order = editor.getTempInt_NumberNumber()
          local sectionsOrder = editor.getPreference("dynamicDecalsTool.general.sectionsOrder")
          sectionsOrder[section.name] = section.order
          editor.setPreference("dynamicDecalsTool.general.sectionsOrder", sectionsOrder)
          table.sort(sections, function(a,b) return a.order < b.order end)
        end
      end
    end}},
  })

  for _, dependency in ipairs(M.dependencies) do
    if string.startswith(dependency, "editor_dynamicDecals") and extensions[dependency].registerEditorPreferences then
      extensions[dependency].registerEditorPreferences(prefsRegistry)
    end
  end
end

local function onEditorSaveState(state)
end

local function onEditorLoadState(state)
end

local function onBrushStrokeEnded()
  api.addBrushStrokeLayer()
end

M.registerEditorOnUpdateFn = function(name, onUpdateFn)
  editorOnUpdateFunctions[name] = onUpdateFn
end

M.unregisterEditorOnUpdateFn = function(name)
  if editorOnUpdateFunctions[name] then
    editorOnUpdateFunctions[name] = nil
  end
end

--[[
  summary:
  params
  * name; string; Name of section
  * guiFn; function; Function being called in the section as well as within the window
  * order [optional]; number; At which position the section appears compared to all other sections.
    The lower the number the further up the section appears.
  * defaultOpen [optional]; bool; Whether the section is opened by default or not
  * window [optional]; table {windowSize[ImVec2]}; Whether a dedicated window will be registered or not.
    If a window was registered a button will be displayed in the section to open the window. Window size can be set in table object.
]]
M.registerSection = function(name, guiFn, order, defaultOpen, window, buttons)
  table.insert(sections, {name = name, guiFn = guiFn, order = order or 1000, defaultOrder = order or 1000, open = defaultOpen, window = window, buttons = buttons or {}, setScroll = false})
end

M.setSectionOpenState = function(name, newState, setScroll)
  for _, section in ipairs(sections) do
    if name == section.name then
      if newState == true and section.order >= 1000 then
        advancedSection.open = true
      end
      section.open = newState
      if setScroll then
        section.setScroll = newState
      end
      return
    end
  end
  if name == advancedSection.name then
    advancedSection.open = newState
    if setScroll then
      advancedSection.setScroll = newState
    end
    return
  end
end

M.getSectionOpenState = function(name)
  for _, section in ipairs(sections) do
    if name == section.name then
      return section.open
    end
  end
  if name == advancedSection.name then
    return advancedSection.open
  end
end

M.registerToolbarToolItem = function(name, guiFn, order)
  table.insert(toolbarToolItems, {name = name, guiFn = guiFn, order = order or 1000})
  table.sort(toolbarToolItems, function(a,b) return a.order < b.order end)
end

M.registerToolbarActionItem = function(name, guiFn, order)
  table.insert(toolbarActionItems, {name = name, guiFn = guiFn, order = order or 1000})
  table.sort(toolbarActionItems, function(a,b) return a.order < b.order end)
end

M.registerOnEditorGuiFn = function(name, guiFn)
  onEditorGuiFunctions[name] = guiFn
end

M.unregisterOnEditorGuiFn = function(name)
  if onEditorGuiFunctions[name] then
    onEditorGuiFunctions[name] = nil
  end
end

-- serialization
M.onSerialize = function()
  return {
  }
end

M.onDeserialized = function(data)
end

-- editor interface
M.onEditorGui = onEditorGui
M.onEditorInitialized = onEditorInitialized
M.onEditorActivated = onEditorActivated
M.onEditorDeactivated = onEditorDeactivated
M.onEditorToolWindowShow = onEditorToolWindowShow
M.onEditorToolWindowHide = onEditorToolWindowHide
M.onEditorRegisterPreferences = onEditorRegisterPreferences
M.onEditorPreferenceValueChanged = onEditorPreferenceValueChanged
M.onEditorSaveState = onEditorSaveState
M.onEditorLoadState = onEditorLoadState

-- public interface for dynamic decals tool
M.applyDecal = function(value)
  if not api.projectDynamicDecals then return end

  -- decal masking
  if value == 1 and currentMaskEditingLayerUid then
    api.addMaskDecal(currentMaskEditingLayerUid)
  -- path
  elseif value == 1 and api.getEnablePathLayer() then
    -- Returns false in case the font doesn't exist and can't be generated
    if deps.fonts.checkOrGenerateFontBitmaps(api.getPathLayerFontPath()) then
      api.addPathDataPoint()
    end
  -- brush stroke
  elseif M.toolMode == M.toolModes.brushStroke then
    brushStrokeEnabled = value == 1 and true or false
    api.setEnableBrushStroke(brushStrokeEnabled)
    if brushStrokeEnabled == false then
      onBrushStrokeEnded()
    end
  -- decal
  elseif value == 1 then
    api.addDecal()
    deps.colorHistory.addColorToHistory(api.getDecalColor())
  end
end

M.changeDecalSize = function(increase, mod) api.changeDecalSize(increase, editor.getPreference("dynamicDecalsTool.decalProperties.scaleStep") * (mod or 1)) end
M.changeDecalRotation = function(clockwise, mod) api.changeDecalRotation(clockwise, (editor.getPreference("dynamicDecalsTool.decalProperties.rotationStep") / 180 * math.pi) * (mod or 1)) end

M.undo = function()
  api.undo()
  local selectedLayers = editor.selection["dynamicDecalLayer"]
  if not selectedLayers then return end
  for uid, layer in pairs(selectedLayers) do
    editor.selection["dynamicDecalLayer"][uid] = deepcopy(api.getLayerByUid(uid))
  end
end

M.redo = function()
  api.redo()
  local selectedLayers = editor.selection["dynamicDecalLayer"]
  if not selectedLayers then return end
  for uid, layer in pairs(selectedLayers) do
    editor.selection["dynamicDecalLayer"][uid] = deepcopy(api.getLayerByUid(uid))
  end
end

M.dynamicDecals_onLayerUpdated = function(layerUid)
  -- check if updated layer is being displayed in inspector, update its data
  if not editor.selection then return end
  local selectedLayers = editor.selection["dynamicDecalLayer"]
  if selectedLayers and selectedLayers[layerUid] then
    editor.selection["dynamicDecalLayer"][layerUid] = deepcopy(api.getLayerByUid(layerUid))
  end
end

M.dynamicDecals_moveLayer = function(from, fromParentUid, to, toParentUid)
  if not editor.selection then return end
  local selectedLayers = editor.selection["dynamicDecalLayer"]
  if selectedLayers and toParentUid and selectedLayers[toParentUid] then
    editor.selection["dynamicDecalLayer"][toParentUid] = deepcopy(api.getLayerByUid(toParentUid))
  end
end

M.lockDepth = function(value)
  api.setLockDepth(value == 1 and true or false)
end

M.lockSurfaceNormal = function(value)
  api.setLockSurfaceNormal(value == 1 and true or false)
end

M.enableBrushStroke = function(value)
  brushStrokeEnabled = value == 1 and true or false
  api.setEnableBrushStroke(brushStrokeEnabled)
  if brushStrokeEnabled == false then
    onBrushStrokeEnded()
  end
end

M.enablePathLayer = function(value)
  api.setEnablePathLayer(value == 1 and true or false)
end

M.finishPathLayer = function()
  api.finishPathLayer()
end

M.removeLastPathLayerPoint = function()
  api.removeLastPathLayerPoint()
end

M.onVehicleSwitched  = function()
  if deps.settings then
    deps.settings.updateMaterials()
  end
end

M.getIconSize = function()
  return iconSize
end

M.getIconSizeVec2 = function()
  return iconSizeVec2
end

M.getMainScrollY = function()
  return mainScrollY
end

M.setCurrentMaskEditingLayerUid = function(uid)
  currentMaskEditingLayerUid = uid
end

M.getCurrentMaskEditingLayerUid = function()
  return currentMaskEditingLayerUid
end

M.directoryPath = "/art/dynamicDecals/"

return M