-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local im = ui_imgui
local ffi = require('ffi')
local imguiUtils = require('ui/imguiUtils')
local toolWindowName = "extensionsDebug"
local funCheck = im.BoolPtr(false)
local numCheck = im.BoolPtr(false)
local strCheck = im.BoolPtr(false)
local cdataCheck = im.BoolPtr(false)
local boolCheck = im.BoolPtr(false)
local tblCheck = im.BoolPtr(false)
local filter = im.BoolPtr(false)

--[[local function addFilter()
  im.Checkbox("Function", funCheck)
  im.SameLine()
  im.Checkbox("Number",numCheck)
  im.SameLine()
  im.Checkbox("String",strCheck)
  im.SameLine()
  im.Checkbox("Cdata",cdataCheck)
  im.SameLine()
  im.Checkbox("Boolean",boolCheck)
  im.SameLine()
  im.Checkbox("Table",tblCheck)
  im.Separator()
end--]]
local function onEditorGui()
  if editor.beginWindow(toolWindowName, "Extensions Debug Window") then
    im.Checkbox("Filters",filter)
    if filter[0] then
   --  addFilter()
    end
    local sortedKeys = extensions.getLoadedExtensionsNames()
    for _,k in pairs(sortedKeys) do
      if im.TreeNodeEx1(k) then
        if not filter[0] then
          imguiUtils.addRecursiveTreeTable(extensions[k],'',false)
        end
        im.TreePop()
      end
    end
  end
  editor.endWindow()
end

local function onWindowMenuItem()
  editor.showWindow(toolWindowName)
end

local function onExtensionLoaded()
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName)
  editor.addWindowMenuItem("Extensions Debug", onWindowMenuItem, {groupMenuName = 'Debug'})
end


M.onEditorGui = onEditorGui
M.onEditorInitialized = onEditorInitialized
M.onExtensionLoaded = onExtensionLoaded

return M