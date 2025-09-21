-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_browser"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local textures = nil
local docs = nil

local toolDecalTexturesBrowserWindowName = 'dynamicDecalTool_DynDecalBrowserWindow'
local tabs = {}
local textureBrowserCurrentTab = 0

local function onGui()
  if editor.beginWindow(toolDecalTexturesBrowserWindowName, "Dynamic Decals Tool - Browser") then
    if im.BeginTabBar("BrowserTabBar") then
      for k, tab in ipairs(tabs) do
        textureBrowserCurrentTab = k
        if im.BeginTabItem(tab.name .. "##Tab") then
          tab.guiFn()
          im.EndTabItem()
        end
      end

      im.EndTabBar()
    end
  end
  editor.endWindow()
end

local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)

end

local function browserDocsGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
The Browser Window serves as a central hub for various assets that seamlessly integrate with the dynamic decal system, enhancing your livery creation experience.

It offers sections with dedicated tabs, such as Textures, Brushes, and Fonts, allowing you to conveniently access and manage different types of design elements.

The browser streamlines your workflow by enabling you to directly drag and drop assets from the tabs to corresponding widgets in the dynamic decal system.
This intuitive interaction sets properties effortlessly, enhancing your creative process.
  ]])
  im.PopTextWrapPos()
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  textures = extensions.editor_dynamicDecals_textures
  docs = extensions.editor_dynamicDecals_docs

  editor.registerWindow(toolDecalTexturesBrowserWindowName, im.ImVec2(800, 400))
  tool.registerOnEditorGuiFn("browser", onGui)
  docs.register({section = {"Browser"}, guiFn = browserDocsGui})
end

local function registerBrowserTab(name, guiFn, order)
  table.insert(tabs, {name = name, guiFn = guiFn, order = order or 1000})
  table.sort(tabs, function(a,b) return a.order < b.order end)
end

local function showWindow()
  editor.showWindow(toolDecalTexturesBrowserWindowName)
end

local function hideWindow()
  editor.hideWindow(toolDecalTexturesBrowserWindowName)
end

local function isWindowVisible()
  return editor.isWindowVisible(toolDecalTexturesBrowserWindowName)
end

M.registerBrowserTab = registerBrowserTab
M.showWindow = showWindow
M.hideWindow = hideWindow
M.isWindowVisible = isWindowVisible

M.onGui = onGui
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M