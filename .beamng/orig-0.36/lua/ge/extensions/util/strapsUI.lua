-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
-- extensions.load("util_strapsui")

local im = ui_imgui

local rive
local openPtr = im.BoolPtr(true)
local test = false
local prevSize = im.ImVec2(0,0)
local tim = hptimer()
local c = color(0,0,0,0)
local prevBool = false


local function imguiWindow()
  local viewport= im.GetMainViewport()
  local size = im.ImVec2(viewport.Size.x * 0.9, viewport.Size.y * 0.9)
  local pos = im.ImVec2(0, 0)
  if size.x ~= prevSize.x or size.y ~= prevSize.y then
    rive:changeTextureResolution(size.x, size.y)
    prevSize = size
    --print("changeTextureResolution", size.x, size.y)
  end
  im.SetNextWindowSize(size, im.ImGuiCond_Always)
  --im.SetNextWindowPos(pos, im.ImGuiCond_Always)
  local invisibleFlag = im.WindowFlags_NoTitleBar + im.WindowFlags_NoResize + im.WindowFlags_NoMove + im.WindowFlags_NoScrollbar + im.WindowFlags_NoCollapse + im.WindowFlags_NoBackground
  if im.Begin("rive template", openPtr) then --, invisibleFlag) then
    -- rive:ImGui_Image(size.x -10 ,size.y -10) --no input on rive, imgui will still block input
    rive:ImGui_Widget(size.x -10 ,size.y -10) --input will work
    im.End()
  end
end

-- This is a function to refresh values, d[name] for data bindings
local function refreshValues()
  local t = tim:stop()/1000
  local myBool = ((t%10)<5)
  rive:executeJS("dproperty of TypeSelector/property of beamProperties/Label" .. ("TestingLabels"))
  -- rive:executeJS("dBoolean property=" .. (myBool and "1" or "0"))
  -- rive:executeJS("dproperty of typeSelector/selectedItemNumber=" .. (myBool and "1" or "0"))
  -- rive:executeJS("dString property=" .. string.format("t=%.1f", (t%100) ))

  -- rive:executeJS(string.format("dNumber property=%d", (t*12-60)%120 ))
  if myBool ~= prevBool then
    -- rive:executeJS("dTrigger property=1")
    -- c = color(math.random()*255,math.random()*255,math.random()*255,255)
    -- c = color(246,16,16,255)
    -- rive:executeJS(string.format("dColor property=%x", c ))
    -- log("I", "val", string.format("Color property=%x", c ))
  end

  -- rive:executeJS("dEnum property=" ..  (myBool and "test1" or "test2") )

  prevBool = myBool
end

local function onUpdate(dtReal, dtSim, dtRaw)
  refreshValues()
  imguiWindow()

end


local function onSerialize()
  --SetDriftUILayout(false)
  if rive then
    rive:destroy()
    rive = nil
  end
  return {}
end

local function onDeserialized(data)
end

-- This is a function to load the Rive file
local function onExtensionLoaded()
  rive = RiveTexture("Template_Lua_loaded",1,30)
  -- rive:loadURL("/ui/rive/currentdriftapp.riv")
  rive:loadURL("/ui/rive/strapsui.riv")
  rive:setFitLayout(true)
  rive:openConsole(true)
  rive:setEventLuaCallback("util_strapsui.onRiveEvent")

  rive:changeTextureResolution(500,500)

end

-- Events reported by anim are below
local function onRiveEvent(eventname)
  log("I", "onRiveEvent", eventname)
  if eventname == "blinkAnimDoneOut" then
    nop() -- that just a dummy that does nothing
    --combooffset = score.combo
  end
end

local function onExtensionUnloaded()
  if rive then
    rive:destroy()
    rive = nil
  end
end

M.onUpdate = onUpdate
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.onRiveEvent = onRiveEvent

return M