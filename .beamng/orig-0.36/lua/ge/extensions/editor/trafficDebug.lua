-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local im = ui_imgui
local toolWindowName = "Traffic Debug"

local traffic

-- ui stuff
local trafficAmountChange = im.IntPtr(0)
local parkingAmountChange = im.IntPtr(0)

local drawTab = nop
local currId = 0

local generalKeys = {"damage", "crashDamage", "speed", "focusDist", "respawnCount", "activeProbability", "camVisible", "isAi"}
local respawnKeys = {"spawnValue", "spawnDirBias", "activeRadius", "finalRadius", "staticVisibility"}
local pursuitKeys = {"mode", "score", "offensesCount", "uniqueOffensesCount"}
local timerKeys = {"main", "arrest", "evade", "arrestValue", "evadeValue"}
local roleKeys = {"actionTimer", "targetId", "targetNear", "targetVisible"}

-- colors
local colors = {
  white = im.ImVec4(1, 1, 1, 1),
  red = im.ImVec4(1, 0, 0, 1),
  yellow = im.ImVec4(1, 1, 0.5, 1),
  silver = im.ImVec4(0.75, 0.75, 0.75, 1),
  grey = im.ImVec4(0.5, 0.5, 0.5, 1)
}

-- debug stuff
local logs = {}
local maxLogsPerVeh = 100

local function appendLog(id, action, data) -- inserts an entry into the log table
  if not logs[id] then logs[id] = {} end
  table.insert(logs[id], {Engine.Platform.getRuntime(), action})
end

local function doBulletTextInfo(key, value) -- validates and displays a bullet point line of text
  local f = type(value) == "number" and "%0.2f" or "%s"
  if type(value) ~= "number" or type(value) ~= "string" then value = tostring(value) end
  im.BulletText(string.format(key..": "..f, value))
end

local function drawGeneralTab()
  local trafficTotalAmount = tableSize(traffic)
  local trafficAmount = gameplay_traffic.getNumOfTraffic()
  local trafficActiveAmount = gameplay_traffic.getNumOfTraffic(true)
  local policeAmount = tableSize(gameplay_police.getPoliceVehicles())
  local parkedAmount = tableSize(gameplay_parking.getParkedCarsList())

  im.Columns(2)
  im.SetColumnWidth(0, 200)

  im.TextUnformatted("All traffic vehicles (also players)")
  im.NextColumn()
  im.TextUnformatted(tostring(trafficTotalAmount))
  im.NextColumn()

  im.Text("AI traffic vehicles")
  im.NextColumn()
  im.TextUnformatted(tostring(trafficAmount))
  im.NextColumn()

  im.Text("Active AI traffic vehicles")
  im.NextColumn()
  im.TextUnformatted(tostring(trafficActiveAmount))
  im.NextColumn()

  im.Text("Police vehicles")
  im.NextColumn()
  im.TextUnformatted(tostring(policeAmount))
  im.NextColumn()

  im.Text("Parked vehicles")
  im.NextColumn()
  im.TextUnformatted(tostring(parkedAmount))
  im.NextColumn()

  im.Columns(1)
  im.Separator()

  im.PushItemWidth(100)
  im.InputInt("Add / Remove Traffic Vehicles", trafficAmountChange, 1)
  im.PopItemWidth()
  im.SameLine()
  local num = math.abs(trafficAmountChange[0])
  local str = trafficAmountChange[0] >= 0 and "Add ("..num..")##trafficAmount" or "Remove ("..num..")##trafficAmount"
  if im.Button(str) then
    if trafficAmountChange[0] > 0 then
      gameplay_traffic.setupTraffic(trafficAmountChange[0], nil, {ignoreDelete = true})
    elseif trafficAmountChange[0] < 0 then
      local trafficAiVehsList = gameplay_traffic.getTrafficList()
      for i = trafficAmount, trafficAmount + trafficAmountChange[0], -1 do
        if trafficAiVehsList[i] then
          getObjectByID(trafficAiVehsList[i]):delete()
        end
      end
    end
  end

  im.PushItemWidth(100)
  im.InputInt("Add / Remove Parked Vehicles", parkingAmountChange, 1)
  im.PopItemWidth()
  im.SameLine()
  num = math.abs(parkingAmountChange[0])
  str = parkingAmountChange[0] >= 0 and "Add ("..num..")##parkingAmount" or "Remove ("..num..")##parkingAmount"
  if im.Button(str) then
    if parkingAmountChange[0] > 0 then
      gameplay_parking.setupVehicles(parkingAmountChange[0], {ignoreDelete = true, ignoreParkingSpots = true})
    elseif parkingAmountChange[0] < 0 then
      local parkedVehsList = gameplay_parking.getParkedCarsList()
      for i = parkedAmount, parkedAmount + parkingAmountChange[0], -1 do
        if parkedVehsList[i] then
          getObjectByID(parkedVehsList[i]):delete()
        end
      end
    end
  end

  if im.Button("Scatter Traffic Vehicles") then
    gameplay_traffic.scatterTraffic()
  end
  if im.Button("Scatter Parked Vehicles") then
    gameplay_parking.scatterParkedCars()
  end

  im.Separator()

  local trafficVars = gameplay_traffic.getTrafficVars()
  local parkingVars = gameplay_parking.getParkingVars()
  local var = im.BoolPtr(gameplay_traffic.debugMode)
  if im.Checkbox("Visual Debug Mode", var) then
    if var[0] then
      for id, veh in pairs(traffic) do
        veh.debugLine = true
        veh.debugText = true
      end
    end

    gameplay_traffic.debugMode = var[0]
  end

  im.Dummy(im.ImVec2(0, 5))
  im.TextUnformatted("Traffic Variables")

  local activeNum = trafficVars.activeAmount
  if activeNum == math.huge then
    activeNum = trafficAmount
  end
  var = im.IntPtr(activeNum)
  im.PushItemWidth(100)
  if im.InputInt("Active Traffic Amount", var, 1) then
    gameplay_traffic.setTrafficVars({activeAmount = math.max(0, var[0])})
  end
  im.PopItemWidth()
  im.tooltip("Sets the maximum amount of active (visible) traffic vehicles.")

  var = im.FloatPtr(trafficVars.spawnValue)
  im.PushItemWidth(100)
  if im.InputFloat("Respawn Rate", var, 0.1, nil, "%.1f") then
    gameplay_traffic.setTrafficVars({spawnValue = clamp(var[0], 0, 3)})
  end
  im.PopItemWidth()
  im.tooltip("Sets how often traffic vehicles will respawn.")

  var = im.FloatPtr(trafficVars.spawnDirBias)
  im.PushItemWidth(100)
  if im.InputFloat("Respawn Direction Bias", var, 0.1, nil, "%.1f") then
    gameplay_traffic.setTrafficVars({spawnDirBias = clamp(var[0], -1, 1)})
  end
  im.PopItemWidth()
  im.tooltip("Sets the average direction of traffic vehicles (-1 = away from you, 1 = towards you).")

  var = im.FloatPtr(trafficVars.baseAggression)
  im.PushItemWidth(100)
  if im.InputFloat("AI Aggression", var, 0.05, nil, "%.2f") then
    gameplay_traffic.setTrafficVars({baseAggression = clamp(var[0], 0.1, 2)})
  end
  im.PopItemWidth()
  im.tooltip("Sets how risky the general driving behavior should be.")

  local speedLimit = trafficVars.speedLimit or -1
  var = im.FloatPtr(speedLimit)
  im.PushItemWidth(100)
  if im.InputFloat("AI Speed Limit", var, 0.5, nil, "%.1f") then
    gameplay_traffic.setTrafficVars({speedLimit = clamp(var[0], -1, 100)})
  end
  im.PopItemWidth()
  if speedLimit >= 0 then
    im.SameLine()
    im.TextColored(colors.grey, string.format('%0.2f', speedLimit * 3.6))
    im.SameLine()
    im.TextColored(colors.grey, "km/h / ")
    im.SameLine()
    im.TextColored(colors.grey, string.format('%0.2f', speedLimit * 2.237))
    im.SameLine()
    im.TextColored(colors.grey, "mph")
  end
  im.tooltip("Sets a strict speed limit for traffic vehicles (-1 = auto).")

  local awareness = trafficVars.aiAware ~= "off" and true or false
  var = im.BoolPtr(awareness)
  if im.Checkbox("AI Awareness", var) then
    gameplay_traffic.setTrafficVars({aiAware = var[0] and "on" or "off"})
  end
  im.tooltip("If true, AI will try to avoid collisions with other vehicles.")

  var = im.BoolPtr(trafficVars.enableRandomEvents)
  if im.Checkbox("Enable Random Events", var) then
    gameplay_traffic.setTrafficVars({enableRandomEvents = var[0]})
  end
  im.tooltip("If true, random events can happen (such as lawless drivers).")

  im.Dummy(im.ImVec2(0, 5))
  im.TextUnformatted("Parking Variables")

  activeNum = parkingVars.activeAmount
  if activeNum == math.huge then
    activeNum = parkedAmount
  end
  var = im.IntPtr(activeNum)
  im.PushItemWidth(100)
  if im.InputInt("Active Parked Amount", var, 1) then
    gameplay_parking.setParkingVars({activeAmount = math.max(0, var[0])})
  end
  im.PopItemWidth()
  im.tooltip("Sets the maximum amount of active (visible) parked vehicles.")

  var = im.FloatPtr(parkingVars.baseProbability)
  im.PushItemWidth(100)
  if im.InputFloat("Parking Spot Probability", var, 0.05, nil, "%.2f") then
    gameplay_parking.setParkingVars({baseProbability = clamp(var[0], 0, 1)})
  end
  im.PopItemWidth()
  im.tooltip("Sets the general probability of any parking spot being used for parked cars to spawn into.")

  var = im.FloatPtr(parkingVars.neatness)
  im.PushItemWidth(100)
  if im.InputFloat("Parking Spot Uniformity", var, 0.05, nil, "%.2f") then
    gameplay_parking.setParkingVars({neatness = clamp(var[0], 0, 1)})
  end
  im.PopItemWidth()
  im.tooltip("Sets how neatly the parked cars will be placed.")

  var = im.FloatPtr(parkingVars.precision)
  im.PushItemWidth(100)
  if im.InputFloat("Parking Spot Precision Judgement", var, 0.05, nil, "%.2f") then
    gameplay_parking.setParkingVars({precision = clamp(var[0], 0, 1)})
  end
  im.PopItemWidth()
  im.tooltip("Sets the precision needed to count as valid parking.")
end

local function drawVehiclesTab()
  im.BeginChild1("Vehicles##trafficDebug", im.ImVec2(180 * im.uiscale[0], 0 ), im.WindowFlags_ChildWindow)
  for id, veh in pairs(traffic) do
    if not veh.isAi then
      im.PushStyleColor2(im.Col_Text, colors.yellow)
      if im.Selectable1("["..id.."] "..veh.model.key, id == currId) then
        currId = id
      end
      im.PopStyleColor()
    end
  end

  for _, id in ipairs(gameplay_traffic.getTrafficList()) do
    local veh = traffic[id]
    local txtColor = colors.white
    if veh.state == "fadeIn" then
      txtColor = colors.red
    elseif not getObjectByID(id):getActive() then
      txtColor = colors.grey
    end

    im.PushStyleColor2(im.Col_Text, txtColor)
    if im.Selectable1("["..id.."] "..veh.model.key, id == currId) then
      currId = id
    end
    im.PopStyleColor()
  end

  im.Separator()

  for _, id in ipairs(gameplay_parking.getParkedCarsList()) do
    im.PushStyleColor2(im.Col_Text, colors.silver)
    if im.Selectable1("["..id.."] "..getObjectByID(id).jbeam, id == currId) then
      currId = id
    end
    im.PopStyleColor()
  end

  im.EndChild()
  im.SameLine()

  im.BeginChild1("Current Vehicle##trafficDebug", im.ImVec2(0, 0), im.WindowFlags_ChildWindow)
  local currVeh = traffic[currId]
  local obj = getObjectByID(currId)

  if obj then
    im.Text("Information")

    local system = currVeh and "traffic" or "parking"
    im.BulletText("System: "..system)
    im.Dummy(im.ImVec2(0, 5))

    local pos = obj:getPosition()
    local dist = pos:distance(core_camera.getPosition())
    local height = clamp(dist / 10, 10, 40)
    local alpha = clamp((dist - 50) / 450, 0.2, 0.8)
    debugDrawer:drawSquarePrism(pos, pos + vec3(0, 0, height), Point2F(0, 0), Point2F(height * 0.25, height * 0.25), ColorF(1, 1, 0.25, alpha))
  end

  if currVeh then
    im.BulletText("Model: "..currVeh.model.name)
    im.BulletText("State: "..currVeh.state)
    im.BulletText("Role: "..currVeh.role.name)
    im.BulletText("Action: "..currVeh.role.actionName)
    im.Dummy(im.ImVec2(0, 5))

    if im.TreeNode1("General Info") then
      for _, key in ipairs(generalKeys) do
        doBulletTextInfo(key, currVeh[key])
      end
      im.TreePop()
    end

    if im.TreeNode1("Respawn Info") then
      for _, key in ipairs(respawnKeys) do
        doBulletTextInfo(key, currVeh.respawn[key])
      end
      im.TreePop()
    end

    if im.TreeNode1("Pursuit Info") then
      local pursuit = currVeh.pursuit
      for _, key in ipairs(pursuitKeys) do
        doBulletTextInfo(key, pursuit[key])
      end

      local timers = pursuit.timers
      for _, key in ipairs(timerKeys) do
        doBulletTextInfo(key, timers[key])
      end
      im.TreePop()
    end

    if im.TreeNode1("Role Info") then
      local role = currVeh.role
      for _, key in ipairs(roleKeys) do
        doBulletTextInfo(key, role[key])
      end
      im.BulletText("flags: "..table.concat(tableKeysSorted(role.flags), ", "))

      im.TreePop()
    end

    if im.TreeNode1("Personality Info") then
      for k, v in pairs(currVeh.role.driver.personality) do
        doBulletTextInfo(k, v)
      end
      im.TreePop()
    end

    im.Separator()

    im.Text("Actions")

    local enableRespawn = im.BoolPtr(currVeh.enableRespawn)
    if im.Checkbox("Enable respawning", enableRespawn) then
      currVeh.enableRespawn = enableRespawn[0]
    end
    im.tooltip("Enables or disables the vehicle respawning by itself if out of sight.")

    local enableEntering = im.BoolPtr(obj.playerUsable == nil or obj.playerUsable == true)
    if im.Checkbox("Enable entering", enableEntering) then
      obj.playerUsable = enableEntering[0]
    end
    im.tooltip("Enables or disables the player switching to or entering the vehicle.")

    local drawLine = im.BoolPtr(currVeh.debugLine)
    if im.Checkbox("Draw debug line", drawLine) then
      currVeh.debugLine = drawLine[0]
    end

    local drawText = im.BoolPtr(currVeh.debugText)
    if im.Checkbox("Draw debug text", drawText) then
      currVeh.debugText = drawText[0]
    end

    if im.Button("Dump Data") then
      dump(currVeh)
    end
    im.tooltip("Displays vehicle data in the developer console (press [~]).")

    if im.Button("Force Respawn") then
      gameplay_traffic.forceTeleport(currVeh.id)
    end

    if im.Button("Refresh Vehicle") then
      currVeh:onRefresh()
    end

    if im.Button("Reset Vehicle") then
      local obj = getObjectByID(currVeh.id)
      obj:queueLuaCommand("recovery.recoverInPlace()")
      currVeh:onRefresh()
    end

    im.Separator()

    im.Text("Logs")

    im.BeginChild1("Action Logs##trafficDebug", im.ImVec2(im.GetWindowContentRegionWidth(), 200), true, im.WindowFlags_None)
    if logs[currVeh.id] then
      for i, v in ipairs(logs[currVeh.id]) do
        if(i > maxLogsPerVeh) then table.remove(logs[currVeh.id], 1) end
        local str = string.format("%0.3f | %s", v[1], v[2])
        if v[3] then
          str = str.." ("..v[3]..")"
        end
        im.Text(str)
      end
    end
    im.EndChild()
  end
  im.EndChild()
end

local function onWindowMenuItem()
  editor.showWindow(toolWindowName)
end

local function onEditorDeactivated()
  gameplay_traffic.debugMode = false
  traffic = nil
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, im.ImVec2(400, 600))
  editor.addWindowMenuItem(toolWindowName, onWindowMenuItem, {groupMenuName = "Debug"})
end

local function onEditorGui()
  if editor.beginWindow(toolWindowName, toolWindowName) then
    if not gameplay_traffic then
      editor.endWindow()
      return
    end

    if not traffic then -- turn on debug mode initially
      gameplay_traffic.debugMode = true
    end
    traffic = gameplay_traffic.getTrafficData()

    if im.BeginTabBar("Traffic Debug Modes") then
      if im.BeginTabItem("General", nil) then
        drawTab = drawGeneralTab
        im.EndTabItem()
      end
      if im.BeginTabItem("Vehicles", nil) then
        drawTab = drawVehiclesTab
        im.EndTabItem()
      end
      im.EndTabBar()
    end

    drawTab()
  end
  editor.endWindow()
end

local function onTrafficAction(id, action, data)
  appendLog(id, action, data)
end

M.onEditorDeactivated = onEditorDeactivated
M.onEditorInitialized = onEditorInitialized
M.onEditorGui = onEditorGui
M.onTrafficAction = onTrafficAction

return M