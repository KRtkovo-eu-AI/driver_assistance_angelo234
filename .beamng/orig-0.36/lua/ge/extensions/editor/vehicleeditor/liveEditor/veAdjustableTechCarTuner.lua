-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.menuEntry = "Adjustable Tech Car Tuner"
local im = extensions.ui_imgui
local imguiUtils = require('ui/imguiUtils')
local wndName = "Adjustable Tech Car Tuner"

local windowOpen = im.BoolPtr(false)

local debug = false

local carViews = {
  left = {
    imgPath = 'lua/ge/extensions/editor/vehicleEditor/liveEditor/car_left_cropped.png',
    imgScale = 0.5,
    imgPos = im.ImVec2(200, 100),
  },
  bottom = {
    imgPath = 'lua/ge/extensions/editor/vehicleEditor/liveEditor/car_bottom_cropped.png',
    imgScale = 0.5,
    imgPos = im.ImVec2(200, 450),
  }
}

local lineThickness = 2
local sliderSize = 100
local arrowSize = 10

local whiteColor = im.GetColorU322(im.ImVec4(1, 1, 1, 1), 1)

local initFlag = false

local varsData = {
  ['$wheelbase'] = {fmt = "%.2f m"},
  ['$overhang_F'] = {fmt = "%.2f m"},
  ['$overhang_R'] = {fmt = "%.2f m"},
  ['$trackwidth_F'] = {fmt = "%.2f m"},
  ['$trackwidth_R'] = {fmt = "%.2f m"},
  ['$bodyWidth'] = {fmt = "%.2f m"},
  ['$weightscale'] = {name = 'Weight Scaled', fmt = "%.2f x"},
  ['$cogOffsetY'] = {name = 'COG Offset Y', fmt = "%.2f x"},
  ['$cogOffsetZ'] = {name = 'COG Offset Z', fmt = "%.2f x"},
  ['$yawInertia'] = {name = 'Yaw Inertia', fmt = "%.2f x"},
}

-- local function getCOM()
--   local veh = getPlayerVehicle(0)
--   local vehData = core_vehicle_manager.getPlayerVehicleData()
--   local nodes = vehData.vdata.nodes
--   local com = vec3()
--   local minExtents, maxExtents = vec3(math.huge, math.huge, math.huge), vec3(-math.huge, -math.huge, -math.huge)
--   local totalMass = 0

--   for i = 0, tableSizeC(nodes) do
--     local node = nodes[i]
--     local pos = veh:getNodePosition(i)
--     minExtents:set(math.min(minExtents.x, pos.x), math.min(minExtents.y, pos.y), math.min(minExtents.z, pos.z))
--     maxExtents:set(math.max(maxExtents.x, pos.x), math.max(maxExtents.y, pos.y), math.max(maxExtents.z, pos.z))

--     pos:setScaled(node.nodeWeight)
--     com:setAdd(pos)
--     totalMass = totalMass + node.nodeWeight
--   end
--   com:setDiv(totalMass)
--   return com
-- end

local function localToGlobalPos(x, y)
  local wndPos = im.GetWindowPos()
  return im.ImVec2(wndPos.x + x, wndPos.y + y)
end

local function addVarFloatSlider(x, y, var)
  local vehData = core_vehicle_manager.getPlayerVehicleData()
  local vars = vehData.vdata.variables
  local varData = varsData[var]

  if x and y then
    im.SetCursorPos(im.ImVec2(x, y))
  end
  im.PushItemWidth(sliderSize)
  im.SliderFloat((varData.name and varData.name or '')..'##'..var, varData.valPtr, vars[var].min, vars[var].max, varData.fmt)
  im.PopItemWidth()
end

local function addMeasurementLineAndSlider(view, isHorizontal, x1, y1, x2, y2, spacing, var)
  local dl = im.GetWindowDrawList()

  x1 = x1 * view.imgSize.x + view.imgPos.x
  y1 = y1 * view.imgSize.y + view.imgPos.y
  x2 = x2 * view.imgSize.x + view.imgPos.x
  y2 = y2 * view.imgSize.y + view.imgPos.y

  local varData = varsData[var]
  local valPtr = varData.valPtr
  if not valPtr then return end

  if isHorizontal then
    local my = spacing >= 0 and math.max(y1 + spacing, y2 + spacing) or math.min(y1 + spacing, y2 + spacing)

    -- vertical lines
    im.ImDrawList_AddLine(dl, localToGlobalPos(x1, y1), localToGlobalPos(x1, my), whiteColor, lineThickness)
    im.ImDrawList_AddLine(dl, localToGlobalPos(x2, y2), localToGlobalPos(x2, my), whiteColor, lineThickness)

    -- horizontal line
    im.ImDrawList_AddLine(dl, localToGlobalPos(x1, my), localToGlobalPos(x2, my), whiteColor, lineThickness)
    -- left arrow
    im.ImDrawList_AddLine(dl, localToGlobalPos(x1, my), localToGlobalPos(x1 + arrowSize, my - arrowSize), whiteColor, lineThickness)
    im.ImDrawList_AddLine(dl, localToGlobalPos(x1, my), localToGlobalPos(x1 + arrowSize, my + arrowSize), whiteColor, lineThickness)
    -- right arrow
    im.ImDrawList_AddLine(dl, localToGlobalPos(x2, my), localToGlobalPos(x2 - arrowSize, my - arrowSize), whiteColor, lineThickness)
    im.ImDrawList_AddLine(dl, localToGlobalPos(x2, my), localToGlobalPos(x2 - arrowSize, my + arrowSize), whiteColor, lineThickness)

    addVarFloatSlider((x1 + x2) * 0.5 - sliderSize * 0.5, my - im.GetFontSize() * 0.5, var)
  else
    local mx = spacing >= 0 and math.max(x1 + spacing, x2 + spacing) or math.min(x1 + spacing, x2 + spacing)

    -- horizontal lines
    im.ImDrawList_AddLine(dl, localToGlobalPos(x1, y1), localToGlobalPos(mx, y1), whiteColor, lineThickness)
    im.ImDrawList_AddLine(dl, localToGlobalPos(x2, y2), localToGlobalPos(mx, y2), whiteColor, lineThickness)

    -- vertical line
    im.ImDrawList_AddLine(dl, localToGlobalPos(mx, y1), localToGlobalPos(mx, y2), whiteColor, lineThickness)
    -- left arrow
    im.ImDrawList_AddLine(dl, localToGlobalPos(mx, y1), localToGlobalPos(mx + arrowSize, y1 + arrowSize), whiteColor, lineThickness)
    im.ImDrawList_AddLine(dl, localToGlobalPos(mx, y1), localToGlobalPos(mx - arrowSize, y1 + arrowSize), whiteColor, lineThickness)
    -- right arrow
    im.ImDrawList_AddLine(dl, localToGlobalPos(mx, y2), localToGlobalPos(mx + arrowSize, y2 - arrowSize), whiteColor, lineThickness)
    im.ImDrawList_AddLine(dl, localToGlobalPos(mx, y2), localToGlobalPos(mx - arrowSize, y2 - arrowSize), whiteColor, lineThickness)

    addVarFloatSlider(mx - sliderSize * 0.5, (y1 + y2) * 0.5 - im.GetFontSize() * 0.5, var)
  end
end

local function init()
  local vehData = core_vehicle_manager.getPlayerVehicleData()
  local vdataVars = vehData.vdata.variables

  for var, varData in pairs(varsData) do
    local vdataVar = vdataVars[var]
    if vdataVar then
      local varVal = vdataVar.val
      if not varData.valPtr then
        varData.valPtr = im.FloatPtr(varVal)
      else
        varData.valPtr[0] = varVal
      end
    end
  end
end

local function applyTuning()
  local vehData = core_vehicle_manager.getPlayerVehicleData()

  vehData.config.vars = vehData.config.vars or {}
  for var, varData in pairs(varsData) do
    if varData.valPtr then
      vehData.config.vars[var] = varData.valPtr[0]
    end
  end
  core_vehicle_partmgmt.setConfigVars(vehData.config.vars)
end

local function onUpdate()
  if windowOpen[0] ~= true then return end

  local veh = getPlayerVehicle(0)
  if not veh or veh.Jbeam ~= 'adjustable_tech_car' then return end

  if im.Begin(wndName, windowOpen) then
    if not initFlag then
      init()
      initFlag = true
    end

    local cursorPosX = im.GetCursorPosX()

    local leftView, bottomView = carViews['left'], carViews['bottom']

    im.PushFont3("cairo_semibold_large")
    if im.Button("Apply") then
      applyTuning()
    end
    im.PopFont()

    im.SetCursorPos(leftView.imgPos)
    im.Image(leftView.img.texId, leftView.imgSize, im.ImVec2(0, 0), im.ImVec2(1, 1))
    --im.ImDrawList_AddCircle(im.GetWindowDrawList(), localToGlobalPos(imgPos.x, imgPos.y), 2, whiteColor, 8, 2)

    addMeasurementLineAndSlider(leftView, true, 0.0, 0.65, 0.207, 0.78, 75, '$overhang_F')
    addMeasurementLineAndSlider(leftView, true, 0.769, 0.78, 1.0, 0.65, 75, '$overhang_R')
    addMeasurementLineAndSlider(leftView, true, 0.207, 0.78, 0.769, 0.78, 75, '$wheelbase')

    im.SetCursorPos(bottomView.imgPos)
    im.Image(bottomView.img.texId, bottomView.imgSize, im.ImVec2(0, 0), im.ImVec2(1, 1))

    addMeasurementLineAndSlider(bottomView, false, 0.207, 0.125, 0.207, 0.875, -205, '$trackwidth_F')
    addMeasurementLineAndSlider(bottomView, false, 0.769, 0.125, 0.769, 0.875, 225, '$trackwidth_R')
    addMeasurementLineAndSlider(bottomView, false, 0.5, 0.06, 0.5, 0.935, 525, '$bodyWidth')

    im.SetCursorPos(im.ImVec2(cursorPosX, 800))
    addVarFloatSlider(nil, nil, '$weightscale')
    addVarFloatSlider(nil, nil, '$cogOffsetY')
    addVarFloatSlider(nil, nil, '$cogOffsetZ')
    addVarFloatSlider(nil, nil, '$yawInertia')

    if debug then
      local viewToDebug = bottomView
      local wndPos = im.GetWindowPos()
      im.SetCursorPos(im.ImVec2(5, 50))
      im.Text(string.format("Mouse Pos: %0.2f, %0.2f", im.GetMousePos().x - wndPos.x, im.GetMousePos().y - wndPos.y))
      local x, y = im.GetMousePos().x - wndPos.x  - viewToDebug.imgPos.x, im.GetMousePos().y - wndPos.y - viewToDebug.imgPos.y
      im.Text(string.format("Mouse Pos Rel Img: %0.3f, %0.3f", x / viewToDebug.imgSize.x, y / viewToDebug.imgSize.y))
    end
  end
  im.End()
end

local function onEditorInitialized()
  for k, view in pairs(carViews) do
    view.img = imguiUtils.texObj(view.imgPath)
    view.imgSize = im.ImVec2(view.img.size.x * view.imgScale, view.img.size.y * view.imgScale)
  end
  editor.registerWindow(wndName, im.ImVec2(700,400))
end

local function onVehicleResetted(vid)
  initFlag = false
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

M.onUpdate = onUpdate
M.onEditorInitialized = onEditorInitialized
M.open = open
M.onVehicleResetted = onVehicleResetted

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M