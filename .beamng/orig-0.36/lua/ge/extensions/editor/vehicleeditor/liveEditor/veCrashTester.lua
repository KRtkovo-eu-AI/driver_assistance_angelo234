-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.menuEntry = "Other/Crash Tester"
local im = extensions.ui_imgui
local imguiUtils = require('ui/imguiUtils')
local wndName = "Crash Tester"

local windowOpen = im.BoolPtr(false)

local pickingLocation = false
local testRunning = false
local vehsData = {}
local startPos = nil
local startYaw = 90 -- degrees
local startDir = quatFromAxisAngle(vec3(0,0,1), math.rad(startYaw))
local tempStartYaw = startYaw
local tempStartDir = quatFromAxisAngle(vec3(0,0,1), math.rad(startYaw))

local numVehsPtr = im.IntPtr(1)
local targetSpeedPtr = im.IntPtr(10) -- km/h

local function initTest()
  local vehsList = {}
  for _, vehItem in ipairs(core_vehicles.getVehicleList().vehicles) do
    if vehItem.model.Type == 'Car' or vehItem.model.Type == 'Truck' then
      table.insert(vehsList, vehItem)
    end
  end

  for i = 1, numVehsPtr[0] do
    local randomVehData = vehsList[math.random(tableSize(vehsList))]

    local spawnPos = startPos + startDir * vec3(10 * (i - 1), 0, 0)
    local startVelocity = startDir * vec3(0,1,0) * targetSpeedPtr[0] / 3.6

    local veh = core_vehicles.spawnNewVehicle(randomVehData.model.key, {pos = spawnPos, rot = startDir})
    veh:queueLuaCommand("input.event('parkingbrake', 0, 1)")
    veh:applyClusterVelocityScaleAdd(veh:getRefNodeId(), 1, startVelocity.x, startVelocity.y, startVelocity.z)

    table.insert(vehsData, {veh = veh, initPos = vec3(spawnPos), done = false})
  end
end

local function runTest()
  if testRunning then return end

  initTest()
  testRunning = true
end

local function stopTest()
  testRunning = false

  for _, vehData in ipairs(vehsData) do
    vehData.veh:delete()
  end

  table.clear(vehsData)
end

local function updateFromEditorGui()
  if pickingLocation then
    -- Choosing start rotation
    local io = im.GetIO()

    if io.MouseWheel ~= 0 then
      tempStartYaw = tempStartYaw + io.MouseWheel * 5
      tempStartDir = quatFromAxisAngle(vec3(0,0,1), math.rad(tempStartYaw))
    end

    -- Choosing start position
    hit = cameraMouseRayCast(false, im.flags(SOTTerrain))
    if hit then
      for i = 1, numVehsPtr[0] do
        local pos = hit.pos + tempStartDir * vec3(10 * (i - 1), 0, 0)

        debugDrawer:drawTriSolid(
          pos + tempStartDir * vec3(1,0,0),
          pos + tempStartDir * vec3(-1,0,0),
          pos + tempStartDir * vec3(0,3,0),
          color(255,0,0,128)
        )
      end

      if im.IsMouseClicked(0) and not im.IsAnyItemHovered() and not im.IsWindowHovered(im.HoveredFlags_AnyWindow) then
        startPos = hit.pos
        startYaw = tempStartYaw
        startDir = tempStartDir
        pickingLocation = false
      end
    end
  end

  if startPos then
    for i = 1, numVehsPtr[0] do
      local pos = startPos + startDir * vec3(10 * (i - 1), 0, 0)

      debugDrawer:drawTriSolid(
        pos + startDir * vec3(1,0,0),
        pos + startDir * vec3(-1,0,0),
        pos + startDir * vec3(0,3,0),
        color(255,0,0,255)
      )
    end
  end
end

local function onUpdate()
  if windowOpen[0] ~= true then return end

  if im.Begin(wndName, windowOpen) then
    im.PushItemWidth(100)
    im.SliderInt("Number of Vehicles", numVehsPtr, 1, 15)
    im.SliderInt("Target Speed", targetSpeedPtr, 10, 200, "%d km/h")
    im.PopItemWidth()
    if im.Button(pickingLocation and "Picking Start Location... " or "Pick Start Location") then
      pickingLocation = not pickingLocation
    end

    im.Spacing()
    if startPos then
      if not testRunning then
        if im.Button("Run Test!") then
          runTest()
        end
      else
        if im.Button("Stop Test!") then
          stopTest()
        end
      end
    end

    updateFromEditorGui()
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