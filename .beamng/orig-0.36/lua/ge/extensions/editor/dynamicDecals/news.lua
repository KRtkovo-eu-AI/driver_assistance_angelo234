-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_news"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
local helper = nil

local windowName = "Vehicle Livery Creator - News"

local windowBeenClosed = false
local spacing = 0

local news = {
  {title = "New version - 1.1.0", guiFn = function()
    im.BulletText("Added the option to project the decal using the surface's normal;n\nAdded property useSurfaceNormal to layer data")
    im.BulletText("Added 'lockSurfaceNormal' option to sample the normal from one point of the mesh and use it on another")
    im.BulletText("Added mesh masking support; Added meshes property to layer data")
    im.BulletText("Added textures API; Added support for meta data to textures (such as tags)")
    im.BulletText("Added support to enable/disable multiple materials in the settings section")
    im.BulletText("Added support to enable/disable meshes in setting section")
    im.BulletText("Browser window: Displaying generated fonts and their glyphs in font tab;\nGlyphs can be selected by double-clicking them and used as decal textures")
    im.BulletText("Fixed border artifact when using textures with alpha")
    im.BulletText("Fixed paths in exported material file (to not contain \\mods\\unpacked)")
    im.BulletText("Fixed layer mask editing")
    im.BulletText("Fixed layer mask translation using the gizmo")
    im.BulletText("Fixed duplicated layer referencing original layer's properties")
    im.BulletText("Fixed wrong UVs when using path layers with text and linear path type")
    im.BulletText("Fixed incorrect projection while layers are selected")
    im.BulletText("Fixed alpha texture rotation being counter-clockwise while color texture rotation is clockwise")
  end},
  {title = "Fixes - 1.0.1", guiFn = function()
    im.BulletText("Fixed projection for path layers in world editor tool")
    im.BulletText("Fixed projection for brush stroke layers in world editor tool")
    im.BulletText("Fixed texture fill layer projection when using a scale value of less than 1.0")
    im.BulletText("Fixed selected material being wrong in some cases (e.g. for the Gavril Roamer - Sheriff variant)")
    im.BulletText("Fixed error when trying to open world editor in main menu")
    im.BulletText("Fixed error where the tool was referencing a missing shape")
    im.BulletText("Fixed the tool writing an empty json file even if no brushes have been created")
    im.BulletText("Tool: Changed the name of the tool in the 'Windows' dropdown from 'Dynamic Decals' to 'Vehicle Livery Creator'")
    im.BulletText("Tool: Link in news window now refers to forum thread")
    im.BulletText("Tool: Also allow JPG decal textures rather than just PNG files")
    im.BulletText("Tool: Fixed the table of contents sorting in the documentation window")
  end},
  {title = "Initial Version 1.0.0", guiFn = function() im.BulletText("Initial version") end},
}

local function welcomingMessage()
  helper.textUnformattedCentered("Welcome to the world of skin customization in BeamNG!")
  helper.textUnformattedCentered(string.format("v %d.%d.%d", tool.version[1], tool.version[2], tool.version[3]))

  im.TextUnformatted([[

We're delighted to introduce you to our skin creation tool, a work-in-progress tool designed to give you complete freedom in personalizing your vehicles.

Our tool offers an array of features, let's dive into what you can expect:

* Decal Customization: With our tool, you can place decals on your vehicles, making them truly yours. Choose from a diverse collection of preloaded decal textures or import your own designs. Adjust the size, rotation, color, and more to achieve the perfect look.
* Layer System: Get ready to explore endless possibilities with our versatile layer system. Create decal layers, path layers (ideal for text or intricate designs following curves), fill layers, texture fill layers (to fill shapes with captivating patterns), brush stroke layers, and group layers.
  * Layer Masks: Enhance your designs with layer masks. These masks provide you with greater control and flexibility in shaping your decals and compositions.
* SDF support: The tool supports SDF (Signed Distance Field) technology, ensuring your decals and text appear crisp and sharp. Add colored outlines, edge feathering, and other fine details to take your designs to the next level.
* Save, Share, and Export: Once you've crafted your perfect skin, save it for future use or share it with fellow enthusiasts. You can export your designs as skin to seamlessly incorporate them into BeamNG. Alternatively, you can export the raw textures, allowing you to make fine adjustments and further refine your designs using third-party raster image editing tools.
]])

  im.TextColored(editor.color.beamng.Value, "Please keep in mind that this tool is work-in-progress. We're actively working to enhance and refine the experience based on your feedback.")
  im.TextUnformatted("We can't wait to see the incredible liveries you create and share with the community!")

  im.Dummy(spacing)
  if im.Button("Vehicle Livery Creator Thread [Link]", im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then openWebBrowser("https://www.beamng.com/threads/experimental-dynamic-decals.95559/") end
  im.Dummy(spacing)

  im.TextUnformatted([[
Get ready to rev up your creativity and make your vehicles stand out on the track!
  ]])
end

local function editorGui()
  if editor.beginWindow(windowName, windowName) then
    -- im.SetWindowFontScale(1.1)
    spacing = im.ImVec2(1, im.GetStyle().ItemSpacing.y * 2)
    im.PushFont3("cairo_regular_medium")
    local titleSize = im.CalcTextSize("Vehicle Livery Creator")
    im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() / 2 - titleSize.x / 2)
    im.TextColored(editor.color.beamng.Value, "Vehicle Livery Creator")
    im.PopFont()
    local availableSpace = im.GetContentRegionAvail()
    local style = im.GetStyle()
    if im.BeginChild1("DynamicDecals_NewsWindow_Child", im.ImVec2(availableSpace.x, availableSpace.y - (math.ceil(im.GetFontSize()) + 3 * style.ItemSpacing.y)), true) then
    im.Dummy(spacing)
    im.PushTextWrapPos(im.GetContentRegionAvailWidth())
      welcomingMessage()
      im.PopTextWrapPos()
      im.Separator()
      im.Dummy(spacing)

      local i = 1
      for _, version in pairs(news) do
        if im.CollapsingHeader1(string.format("%s", version.title), i == 1 and im.TreeNodeFlags_DefaultOpen or nil) then
          version.guiFn()
        end
        i = i + 1
      end
    end
    im.EndChild()

    if im.Button("OK") then
      windowBeenClosed = true
      editor.hideWindow(windowName)
    end
    im.SameLine()
    im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() - (im.CalcTextSize("Do not show again").x + 2*im.GetStyle().ItemSpacing.x + tool.getIconSize()))
    im.TextUnformatted("Do not show again")
    im.SameLine()
    if im.Checkbox("##dynDecals_news_doNotShowAgainCheckbox", editor.getTempBool_BoolBool(editor.getPreference("dynamicDecalsTool.news.doNotShowAgain"))) then
      editor.setPreference("dynamicDecalsTool.news.doNotShowAgain", editor.getTempBool_BoolBool())
    end
    im.tooltip("Do not pop up modal until a new version of the tool is available")

  end
  editor.endWindow()

  local currentVersion = tool.version
  local prefVersion = editor.getPreference("dynamicDecalsTool.general.version")
  local prefDoNotShowAgain = editor.getPreference("dynamicDecalsTool.news.doNotShowAgain")


  if currentVersion[1] > prefVersion[1]
    or currentVersion[1] == prefVersion[1] and currentVersion[2] > prefVersion[2]
    or currentVersion[1] == prefVersion[1] and currentVersion[2] == prefVersion[2] and currentVersion[3] > prefVersion[3] then

    editor.showWindow(windowName)
    editor.setPreference("dynamicDecalsTool.general.version", currentVersion)
    editor.setPreference("dynamicDecalsTool.news.doNotShowAgain", false)
  end

  if (windowBeenClosed == false and prefDoNotShowAgain == false) then
    editor.showWindow(windowName)
  end
end

local function setup(tool_in)
  tool = tool_in
  helper = extensions.editor_dynamicDecals_helper

  editor.registerWindow(windowName, im.ImVec2(640, 640))

  tool.registerOnEditorGuiFn("news", editorGui)
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "news", nil, {
    {doNotShowAgain = {"bool", false, "Do not pop up modal until a new version of the tool is available"}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

M.showWindow = function() editor.showWindow(windowName) end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M