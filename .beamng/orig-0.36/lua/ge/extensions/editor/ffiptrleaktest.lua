-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
do return M end -- disabled for now


local logTag = 'editor_extension_ptrleaktest'
local im = ui_imgui
local toolWindowName = "ffi ptr leak test"

local demoWindowOpen = ffi.new("FFIBool", false)
local boolTest = im.ffiBool(true)
local floatTest = ffi.new("FFIFloat", 1.0)
local float2Test = ffi.new("FFIFloat2", 0.5, 2.0)
-- local stringTest = ffi.new("FFIString", "NICE")
local stringTest = im.str(128, "NICE")

local image = nil

local function onEditorGui()
  if demoWindowOpen[0] then
    im.ShowDemoWindowTest(demoWindowOpen)
  end

  if editor.beginWindow(toolWindowName, "ffi ptr leak test") then
    if im.Button("demoWindowOpen[0] = true") then
      demoWindowOpen[0] = true
    end

    im.CheckboxTest("Checkbox Test", boolTest)
    im.SliderFloatTest("Slider Float Test", floatTest, 0, 2)
    im.SliderFloat2Test("Slider Float2 Test", float2Test, 0, 3)
    im.InputTextTest("Text Input Test", stringTest)


    im.TextUnformatted("ImVec2& vs ImVec2* test")
    im.Image(image.tex:getID(), im.ImVec2(256,256), im.ImVec2(0,0), im.ImVec2(1,1));
  end
  editor.endWindow()
end

local function onWindowMenuItem()
  editor.showWindow(toolWindowName)
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, im.ImVec2(800, 500))
  editor.addWindowMenuItem("ffi ptr leak test", onWindowMenuItem, nil, true)

  image = editor.texObj("/art/dynamicDecals/textures/00_color_palette_test.png")
end

-- Uncomment all these to enable the test extension
M.onEditorInitialized = onEditorInitialized
M.onEditorGui = onEditorGui
-- M.onEditorActivated = onEditorActivated
-- M.onEditorDeactivated = onEditorDeactivated
-- M.onEditorCanClose = onEditorCanClose
-- M.onEditorDeprecatedPreferencesItem = onEditorDeprecatedPreferencesItem
-- M.onEditorPreferenceValueChanged = onEditorPreferenceValueChanged
-- M.onEditorToolWindowGotFocus = onWindowGotFocus
-- M.onEditorToolWindowLostFocus = onWindowLostFocus
-- M.onExtensionLoaded = onExtensionLoaded
-- --M.onEditorRegisterPreferences = onEditorRegisterPreferences
-- M.onEditorToolWindowHide = function(wndName) print("Tool window was closed: ".. wndName) end
-- M.onEditorToolWindowShow = function(wndName) print("Tool window was opened: ".. wndName) end
-- M.onEditorInspectorFieldChanged = onEditorInspectorFieldChanged
-- M.onEditorInspectorFieldChangedWithOldValues = onEditorInspectorFieldChangedWithOldValues
-- M.onEditorInspectorDynFieldChanged = onEditorInspectorDynFieldChanged
-- M.onEditorPreferencePreLoad = onEditorPreferencePreLoad
-- M.onEditorPreferenceVersionChanged = onEditorPreferenceVersionChanged

return M