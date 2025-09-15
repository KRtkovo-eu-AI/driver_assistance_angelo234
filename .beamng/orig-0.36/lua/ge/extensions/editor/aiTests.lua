-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local im = ui_imgui
local toolWindowName = "AI Path/Plan Tests"
local route = require("/lua/ge/extensions/gameplay/route/route")()
local routeMarkers = {}
local vehicles = {}
local vehicleIds = {}
local currId

local options = {
  ignoreOneWay = im.BoolPtr(false),
  routeDetails = im.BoolPtr(false),
  trackPlayerRoute = im.BoolPtr(false),
  useCurrentRoute = im.BoolPtr(true),
  dynamicCollisions = im.BoolPtr(true),
  aiDebug = im.BoolPtr(false)
}
local params = {
  aggression = {im.FloatPtr(0.5), 0, 2},
  turnForceCoef = {im.FloatPtr(2), 0, 5},
  awarenessForceCoef = {im.FloatPtr(0.25), 0, 5},
  edgeDist = {im.FloatPtr(0), -2, 2},
  lookAheadKv = {im.FloatPtr(0.65), 0.5, 1},
  staticFrictionCoefMult = {im.FloatPtr(0.95), 0.75, 2}
  --driveInLane = {im.BoolPtr(false)}
}
local paramKeys = tableKeysSorted(params)
local routeTracking = false
local aiTracking = false
local _dynamicCollision

local colorWhite = ColorF(1, 1, 1, 1)
local colorIBlack = ColorI(0, 0, 0, 192)

local _uid = 0 -- do not use ever
local function getNextUniqueIdentifier()
  _uid = _uid + 1
  return _uid
end

local function insertVehicle(id) -- adds an AI vehicle to use for the tests
  local obj = getObjectByID(id or 0)
  if not obj then return end

  vehicles[id] = {
    pos = obj:getPosition(),
    rot = quat(obj:getRotation()),
  }

  local details = core_vehicles.getVehicleDetails(id)
  if details.model.Name then
    vehicles[id].model = details.model.Brand and details.model.Brand.." "..details.model.Name or details.model.Name
  else
    vehicles[id].model = obj.model
  end
  if details.current.config_key then
    vehicles[id].config = details.configs and details.configs.Configuration or ''
  else
    vehicles[id].config = ''
  end

  for k, v in pairs(params) do
    vehicles[id][k] = v[1][0]
  end

  vehicleIds = tableKeysSorted(vehicles)
end

local function removeVehicle(id) -- removes an AI vehicle
  if not id or not vehicles[id] then return end
  vehicles[id] = nil
  vehicleIds = tableKeysSorted(vehicles)
end

local function selectVehicle(id) -- selects an AI vehicle, and updates the params
  if not id or not vehicles[id] then return end
  for k, v in pairs(params) do
    v[1][0] = vehicles[id][k]
  end
  currId = id
end

local transformUtilPath = "/lua/ge/extensions/editor/util/transformUtil"
local function newRouteMarker(name) -- returns new marker data
  local point = require(transformUtilPath)(name, name)
  point.allowScale = false
  point.allowRotate = false
  return point
end

local function tabRoute() -- debug navgraph route with a start position, finish position, and optional waypoints
  local change = false

  if not routeMarkers[2] then
    table.clear(routeMarkers)
    table.insert(routeMarkers, newRouteMarker("Start"))
    table.insert(routeMarkers, newRouteMarker("Finish"))
  end

  local count = #routeMarkers

  for i, marker in ipairs(routeMarkers) do
    local isWaypoint = i > 1 and i < count

    im.Text(marker.objectName)
    if isWaypoint then
      im.SameLine()
      if im.Button("Remove##"..marker.objectName) then
        table.remove(routeMarkers, i)
        change = true
        break
      end
    end

    if im.Button("Player Vehicle##"..marker.objectName) then
      if getPlayerVehicle(0) then
        marker:set(getPlayerVehicle(0):getPosition())
        change = true
      end
    end
    im.SameLine()
    if im.Button("Camera##"..marker.objectName) then
      marker:set(core_camera.getPosition() - vec3(0, 0, 1))
      change = true
    end
    change = change or marker:update()
    im.Dummy(im.ImVec2(0, 5))

    if i == count then
      if im.Button("Add Waypoint") then
        table.insert(routeMarkers, count, newRouteMarker("Waypoint "..getNextUniqueIdentifier()))
        change = true
        break
      end
      im.Dummy(im.ImVec2(0, 5))
    end
  end

  im.Separator()
  if im.Button("Set Ground Markers") then
    local path = {}
    for i, marker in ipairs(routeMarkers) do
      table.insert(path, marker.pos)
    end
    core_groundMarkers.setPath(path)
  end
  im.tooltip("Sends route data to the ground markers system.")
  im.SameLine()
  if im.Button("Dump Route") then
    dump(route)
  end
  im.tooltip("Sends route data to the dev console.")

  if im.Checkbox("Override One Way Roads", options.ignoreOneWay) then
    route:setRouteParams(nil, options.ignoreOneWay[0] and 1 or 1e3)
    change = true
  end
  im.Checkbox("Display Details", options.routeDetails)
  im.Checkbox("Track Player Vehicle", options.trackPlayerRoute)

  if change then
    local path = {}
    for i, marker in ipairs(routeMarkers) do
      if string.find(marker.objectName, "Waypoint") then
        marker.objectName = "Waypoint #"..tostring(i - 1)
      end

      table.insert(path, marker.pos)
    end
    route:setupPathMulti(path)
  end
  for i, point in ipairs(route.path) do
    local clr = rainbowColor(#route.path, i, 1)
    local pos = vec3(point.pos)
    debugDrawer:drawSphere(pos, 1, ColorF(clr[1], clr[2], clr[3], 0.6))
    --if point.wp then
      --debugDrawer:drawTextAdvanced(pos, String(point.wp), ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
    --end
    if i > 1 then
      debugDrawer:drawSquarePrism(pos, vec3(route.path[i-1].pos), Point2F(2, 0.5), Point2F(2, 0.5), ColorF(clr[1], clr[2], clr[3], 0.4))
    end
    if options.routeDetails[0] then
      debugDrawer:drawTextAdvanced(pos, String(string.format("%0.1fm", point.distToTarget or -1)), colorWhite, true, false, colorIBlack)
    end
  end

  for i, marker in ipairs(routeMarkers) do
    debugDrawer:drawTextAdvanced(marker.pos, marker.objectName, colorWhite, true, false, colorIBlack)
  end

  if options.trackPlayerRoute[0] then
    local playerVehicle = getPlayerVehicle(0)
    if playerVehicle then
      local idx, dist = route:trackVehicle(playerVehicle)
      im.Text(string.format("Idx: %d, distance: %0.1f", idx or -1, dist or -1))
      routeTracking = true
    else
      im.Text("No player vehicle!")
    end
  else
    if routeTracking then
      route:trackPosition(route.path[1] and route.path[1].pos or vec3())
      routeTracking = false
    end
  end
end

local function tabParams() -- debug AI parameters
  local editEnded = im.BoolPtr(false)

  im.BeginChild1("AI Test Vehicles", im.ImVec2(150 * im.uiscale[0], 0), im.WindowFlags_ChildWindow)

  local _del
  for _, id in ipairs(vehicleIds) do -- vehicle validator loop
    local obj = getObjectByID(id)
    if not obj then
      _del = _del or {}
      table.insert(_del, id)
    else
      if im.Selectable1(obj.jbeam.." ["..id.."]", currId == id) then
        selectVehicle(id)
      end
    end
  end

  if _del then
    for _, v in ipairs(_del) do
      removeVehicle(v)
    end
  end
  im.Separator()
  if im.Selectable1("Setup All Vehicles...", true) then -- sets up all other vehicles
    for _, v in ipairs(getAllVehiclesByType()) do
      if not v.isParkingOnly then
        local id = v:getId()
        insertVehicle(id)
      end
    end
    selectVehicle(vehicleIds[1])
  end

  im.EndChild()
  im.SameLine()

  im.BeginChild1("AI Test Parameters", im.ImVec2(0, 0), im.WindowFlags_ChildWindow)

  local veh = currId and vehicles[currId]
  if veh then

    im.TextUnformatted(veh.model.." "..veh.config)

    im.SameLine()
    if im.Button("Remove From List##aiParams") then
      removeVehicle(currId)
      selectVehicle(vehicleIds[1])
    end

    im.Dummy(im.ImVec2(0, 5))
    im.Separator()

    im.TextUnformatted("Current Vehicle")

    for _, key in ipairs(paramKeys) do
      local p = params[key]
      if type(p[1][0]) == "number" then
        if editor.uiSliderFloat(key, p[1], p[2] or 0, p[3] or 4, "%0.2f", nil, editEnded) then
          veh[key] = p[1][0]
        end
      elseif type(p[0]) == "boolean" then
        if im.Checkbox(key, p[1]) then
          veh[key] = p[1][0]
        end
      end
    end

    im.Dummy(im.ImVec2(0, 5))
    im.Separator()
    im.TextUnformatted("All Vehicles")

    im.Checkbox("Use Current Route##aiParams", options.useCurrentRoute)
    im.tooltip("If true, uses the route from the Route tab; otherwise, generates a random route.")

    im.Checkbox("Enable AI Debug Drawing##aiParams", options.aiDebug)
    im.tooltip("If true, shows the AI trajectory debug mode.")

    if im.Checkbox("Enable Dynamic Collisions##aiParams", options.dynamicCollisions) then
      be:setDynamicCollisionEnabled(options.dynamicCollisions[0])
      _dynamicCollision = true
    end
    im.tooltip("Disable inter-vehicle collisions to compare AI as ghosts.")

    im.SameLine()
    if options.dynamicCollisions[0] or not vehicleIds[2] then im.BeginDisabled() end
    if im.Button("Merge Positions##aiParams") then
      local firstVeh = getObjectByID(vehicleIds[1])
      if firstVeh then
        local pos, rot = firstVeh:getPosition(), firstVeh:getRotation()
        for k, v in pairs(vehicles) do
          getObjectByID(k):setPosRot(pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w)
        end
      end
    end
    im.tooltip("Merge all vehicle positions and rotations to match the first vehicle.")
    if options.dynamicCollisions[0] or not vehicleIds[2] then im.EndDisabled() end

    if not aiTracking then
      if im.Button("Start##aiParams") then
        local path
        if options.useCurrentRoute[0] and route.path[1] then
          path = {}
          for _, v in ipairs(route.path) do
            if v.wp then
              table.insert(path, v.wp)
            end
          end
        else
          local obj = getObjectByID(currId)
          local pos = obj:getPosition()
          local dirVec = obj:getDirectionVector()
          local nodes = map.getMap().nodes
          local n1, n2 = map.findClosestRoad(veh.pos)
          local p1, p2 = nodes[n1].pos, nodes[n2].pos
          if dirVec:dot(p1 - pos) < 0 then
            n1, n2 = n2, n1
          end
          path = map.getGraphpath():getRandomPathG(n1, dirVec, 2000, nil, nil, false)
        end

        for k, v in pairs(vehicles) do
          local obj = getObjectByID(k)
          v.pos:set(obj:getPosition())
          v.rot:set(quatFromDir(obj:getDirectionVector(), obj:getDirectionVectorUp()) * quat(0, 0, 1, 0))

          local aiParams = {}
          for key, _ in pairs(params) do
            aiParams[key] = v[key]
          end

          obj:queueLuaCommand("ai.driveUsingPath({wpTargetList = "..serialize(path)..", avoidCars = 'off'})")
          obj:queueLuaCommand("ai.setAggression("..aiParams.aggression..")")
          obj:queueLuaCommand(string.format("ai.setParameters(%s)", serialize(aiParams)))

          if options.aiDebug[0] then
            obj:queueLuaCommand("ai.setVehicleDebugMode({debugMode = 'trajectory'})")
          else
            obj:queueLuaCommand("ai.setVehicleDebugMode({debugMode = 'off'})")
          end
        end
        aiTracking = true
      end
      im.SameLine()
    else
      if im.Button("Stop##aiParams") then
        for k, v in pairs(vehicles) do
          getObjectByID(k):queueLuaCommand("ai.setMode('stop')")
        end
        aiTracking = false
      end
      im.SameLine()
    end
    if im.Button("Reset##aiParams") then
      for k, v in pairs(vehicles) do
        getObjectByID(k):setPosRot(v.pos.x, v.pos.y, v.pos.z, v.rot.x, v.rot.y, v.rot.z, v.rot.w)
      end
      aiTracking = false
    end
    im.SameLine()
    if im.Button("Reload##aiParams") then
      for k, v in pairs(vehicles) do
        getObjectByID(k):reload()
      end
      aiTracking = false
    end
  end

  im.EndChild()
end

local function onEditorGui()
  if editor.beginWindow(toolWindowName, toolWindowName) then
    if im.BeginTabBar("AI Test Modes") then
      if im.BeginTabItem("Route") then
        tabRoute()
        im.EndTabItem()
      end
      if im.BeginTabItem("AI Vehicles") then
        tabParams()
        im.EndTabItem()
      end
      im.EndTabBar()
    end

    editor.endWindow()
  end
end

local function onWindowMenuItem() editor.showWindow(toolWindowName) end

local function onEditorDeactivated()
  if _dynamicCollision then
    be:setDynamicCollisionEnabled(not settings.getValue('disableDynamicCollision'))
  end
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, im.ImVec2(520, 420))
  editor.addWindowMenuItem(toolWindowName, onWindowMenuItem, {groupMenuName = 'AI'})
end

M.onEditorInitialized = onEditorInitialized
M.onEditorDeactivated = onEditorDeactivated
M.onEditorGui = onEditorGui

return M