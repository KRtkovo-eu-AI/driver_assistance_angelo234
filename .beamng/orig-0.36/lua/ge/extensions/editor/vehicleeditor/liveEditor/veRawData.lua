-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local imguiUtils = require('ui/imguiUtils')
local im = ui_imgui

local wndName = "Raw Vehicle Data"
M.menuEntry = "Raw Vehicle Data"

local windowOpen = im.BoolPtr(false)

local function onUpdate()
  if windowOpen[0] ~= true then return end

  if not (vEditor and vEditor.vehicle and vEditor.vehData) then return end

  if im.Begin(wndName, windowOpen) then
    imguiUtils.addRecursiveTreeTable(vEditor.vehData, '', false)
  end
  im.End()
end

local function open()
  windowOpen[0] = true
end

local function onSerialize()
  return {
    windowOpen = windowOpen[0],
  }
end

local function onDeserialized(data)
  windowOpen[0] = data.windowOpen
end

M.open = open

M.onUpdate = onUpdate

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M