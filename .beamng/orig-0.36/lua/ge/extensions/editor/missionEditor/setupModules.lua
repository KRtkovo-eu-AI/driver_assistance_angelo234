-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

function C:init(missionEditor)
  self.missionEditor = missionEditor
  self.name = "SetupModules"
end

local imDummy = im.ImVec2(0, 5)
local defaultVehicle = {model = "etk800"}

function C:setMission(mission)
  self.mission = mission
  self.missionInstance = gameplay_missions_missions.getMissionById(mission.id)

  self:setBackwardsCompatibility()
  self.blockedSetupModules = tableValuesAsLookupDict(self.missionInstance.blockedSetupModules or {})

  self.vehicleGroupData = self.mission.setupModules.vehicles.vehicles
  self.vehicleIncludePlayer = im.BoolPtr(self.mission.setupModules.vehicles.includePlayerVehicle and true or false)
  self.vehiclePrioritizePlayer = im.BoolPtr(self.mission.setupModules.vehicles.prioritizePlayerVehicle and true or false)
  self.vehicleSelectors = {}

  self.trafficAmountInput = im.IntPtr(self.mission.setupModules.traffic.amount or 3)
  self.trafficActiveAmountInput = im.IntPtr(self.mission.setupModules.traffic.activeAmount or 3)
  self.trafficParkedAmountInput = im.IntPtr(self.mission.setupModules.traffic.parkedAmount or 0)
  self.trafficRespawnRateInput = im.FloatPtr(self.mission.setupModules.traffic.respawnRate or 1)
  self.trafficUseTrafficInput = im.BoolPtr(self.mission.setupModules.traffic.useTraffic and true or false)
  self.trafficPrevTrafficInput = im.BoolPtr(self.mission.setupModules.traffic.usePrevTraffic and true or false)
  self.trafficUserOptionsInput = im.BoolPtr(self.mission.setupModules.traffic.useGameOptions and true or false)
  self.trafficSimpleVehsInput = im.BoolPtr(self.mission.setupModules.traffic.useSimpleVehs and true or false)
  self.trafficUseCustomGroup = im.BoolPtr(self.mission.setupModules.traffic.useCustomGroup and true or false)

  self.todInput = im.FloatPtr(self.mission.setupModules.environment.time or 0)
  self.todScaleInput = im.FloatPtr(self.mission.setupModules.environment.timeScale or 0)
  self.windSpeedInput = im.FloatPtr(self.mission.setupModules.environment.windSpeed or 0)
  self.windDirectionInput = im.FloatPtr(self.mission.setupModules.environment.windDirAngle or 0)
  self.fogDensityInput = im.FloatPtr(self.mission.setupModules.environment.fogDensity or 0)
  self.todUserSettingInput = im.BoolPtr(self.mission.setupModules.environment.todUserSetting and true or false)
  self.weatherUserSettingInput = im.FloatPtr(self.mission.setupModules.environment.weatherUserSetting and true or false)
end

function C:setBackwardsCompatibility()
  -- check for patched data from mission instance and apply it to the mission data
  for k, v in pairs(self.missionInstance.setupModules) do
    if v._compatibility then
      v._compatibility = nil
      self.mission.setupModules[k] = deepcopy(v)
    end
  end

  -- remove obsolete setup modules
  self.mission.setupModules.playerVehicle = nil
  self.mission.setupModules.timeOfDay = nil
end

function C:getMissionIssues(m)
  self:setMission(m)
  local issues = {}

  for k, v in pairs(m.setupModules) do
    if v.enabled == nil or (v.enabled and tableSize(v) <= 1) then
      table.insert(issues, {label = 'Missing or malformed data for setup module: '..k, severity = 'error'})
    end
  end

  -- more issues could go here

  return issues
end

local function todToTime(val)
  local seconds = ((val + 0.50001) % 1) * 86400
  local hours = math.floor(seconds / 3600)
  local mins = math.floor(seconds / 60 - (hours * 60))
  local secs = math.floor(seconds - hours * 3600 - mins * 60)
  return string.format("%02d:%02d:%02d", hours, mins, secs)
end

function C:draw()
  local inputWidth = 150 * im.uiscale[0]

  im.PushID1(self.name)
  im.Columns(2)
  im.SetColumnWidth(0, 150)

  im.Text("Vehicles")
  im.NextColumn()

  local setupModule = self.mission.setupModules.vehicles
  local isBlocked = self.blockedSetupModules.vehicles
  self.mission.setupModules.vehicles.enabled = not isBlocked
  if isBlocked then
    im.BeginDisabled()
  end
  if im.Checkbox("##setupModuleVehiclesEnabled", im.BoolPtr(setupModule.enabled)) then
    setupModule.enabled = not setupModule.enabled
    self.mission._dirty = true
  end
  im.SameLine()
  if setupModule.enabled then
    if not setupModule.vehicles then
      table.clear(setupModule)
      setupModule.enabled = true
      setupModule.vehicles = self.vehicleGroupData or {}
      setupModule.includePlayerVehicle = self.vehicleIncludePlayer[0]
      setupModule.prioritizePlayerVehicle = self.vehiclePrioritizePlayer[0]
    end

    im.Text("Vehicle Setup")

    local baseCount = #setupModule.vehicles -- provided vehicles only
    local count = baseCount
    if setupModule.includePlayerVehicle then
      count = count + 1
    end
    im.Text("Number of vehicles selectable for this mission: "..count)
    if im.Button("Add New Provided Vehicle") then
      table.insert(setupModule.vehicles, deepcopy(defaultVehicle)) -- deepcopy is important here
      self.mission._dirty = true
    end
    im.tooltip("Adds a vehicle that the user can choose to use for this mission.")

    im.Dummy(imDummy)

    if not self.vehicleSelectors[baseCount] then
      for i = #self.vehicleSelectors + 1, baseCount do
        local currVeh = setupModule.vehicles[i] or {}

        local data = {
          model = currVeh.model or defaultVehicle.model,
          config = currVeh.config,
          customConfigPath = currVeh.customConfigPath,
          paintName = currVeh.paintName,
          paintName2 = currVeh.paintName2,
          paintName3 = currVeh.paintName3
        }

        if currVeh.useCustomConfig and not currVeh.customConfigPath then
          currVeh.customConfigPath = currVeh.configPath
        end

        local vehSelectUtil = require("/lua/ge/extensions/editor/util/vehicleSelectUtil")("Provided Vehicle #"..i, data)
        vehSelectUtil.enablePaints = true
        vehSelectUtil.enableCustomConfig = true
        table.insert(self.vehicleSelectors, vehSelectUtil)
      end
    end

    local delIdx
    for i = 1, baseCount do
      im.HeaderText("Provided Vehicle #"..i)
      im.SameLine()
      if im.Button("Delete##vehicleSelector"..i) then
        delIdx = i
      end

      local util = self.vehicleSelectors[i]
      local veh = setupModule.vehicles[i]
      if util:widget() then -- returns true whenever updated
        veh.model = util.model
        veh.config = util.config
        veh.configPath = util.configPath
        veh.paintName = util.paintName
        veh.paintName2 = util.paintName2
        veh.paintName3 = util.paintName3

        veh.customConfigPath = util.customConfigPath
        veh.useCustomConfig = util.customConfigPath and true or false

        self.mission._dirty = true
      end
      im.Text("Differential Setup for vehicle #"..i)
      im.PushID1("##diff"..i)
      self:displayDiffSetup(veh)
      im.PopID()

      im.Dummy(imDummy)
    end

    if delIdx and setupModule.vehicles[delIdx] then
      table.remove(setupModule.vehicles, delIdx)
      table.remove(self.vehicleSelectors, delIdx)
      self.mission._dirty = true
    end
    im.HeaderText("Player Vehicle")
    if im.Checkbox("Add Player Vehicle to Selections##vehicle", self.vehicleIncludePlayer) then
      setupModule.includePlayerVehicle = self.vehicleIncludePlayer[0]
      self.mission._dirty = true
    end
    im.tooltip("If true, the current player vehicle can be used for the mission.")
    if not setupModule.includePlayerVehicle then
      im.BeginDisabled()
    end
    if im.Checkbox("Set Player Vehicle as Priority##vehicle", self.vehiclePrioritizePlayer) then
      setupModule.prioritizePlayerVehicle = self.vehiclePrioritizePlayer[0]
      self.mission._dirty = true
    end
    im.tooltip("If true, the current player vehicle becomes the default vehicle for the mission.")

    if setupModule.includePlayerVehicle then
      setupModule.playerVehicleDiffs = setupModule.playerVehicleDiffs or {}
      im.Text("Differential Setup for player vehicle")
      im.PushID1("##diffPlayer")
      self:displayDiffSetup(setupModule.playerVehicleDiffs)
      im.PopID()
    end

    if not setupModule.includePlayerVehicle then
      im.EndDisabled()
    end
    --if baseCount < 1 then
      --setupModule.includePlayerVehicle = true -- always true if no other vehicles were provided
    --end
  else
    if isBlocked then
      im.Text("Player vehicle setup is not available for this mission type.")
    else
      table.clear(setupModule)
      setupModule.enabled = false
      im.Text("Select this to enable player vehicle setup.")
    end
  end
  if isBlocked then
    im.EndDisabled()
  end

  im.Separator()
  im.NextColumn()
  im.Text("Traffic")
  im.NextColumn()

  setupModule = self.mission.setupModules.traffic
  isBlocked = self.blockedSetupModules.traffic
  if isBlocked then
    setupModule.enabled = false
    im.BeginDisabled()
  end
  if im.Checkbox("##setupModuleTrafficEnabled", im.BoolPtr(setupModule.enabled)) then
    setupModule.enabled = not setupModule.enabled
    self.mission._dirty = true
  end
  im.SameLine()
  if setupModule.enabled then
    if not setupModule.amount then -- init values
      setupModule.useTraffic = true -- initializes as true
      self.trafficUseTrafficInput[0] = setupModule.useTraffic
      setupModule.amount = self.trafficAmountInput[0]
      setupModule.activeAmount = self.trafficActiveAmountInput[0]
      setupModule.parkedAmount = self.trafficParkedAmountInput[0]
      setupModule.respawnRate = self.trafficRespawnRateInput[0]
      setupModule.usePrevTraffic = self.trafficPrevTrafficInput[0]
      setupModule.useGameOptions = self.trafficUserOptionsInput[0]
      setupModule.useSimpleVehs = self.trafficSimpleVehsInput[0]
      setupModule.useCustomGroup = self.trafficUseCustomGroup[0]
    end

    im.Text("Traffic Setup")
    im.PushItemWidth(inputWidth)
    if im.InputInt("Amount##traffic", self.trafficAmountInput, 1) then
      setupModule.amount = self.trafficAmountInput[0]
      self.mission._dirty = true
    end
    im.tooltip("Amount of traffic vehicles to spawn; -1 = auto amount")
    im.PopItemWidth()

    im.PushItemWidth(inputWidth)
    if im.InputInt("Active Amount##traffic", self.trafficActiveAmountInput, 1) then
      setupModule.activeAmount = self.trafficActiveAmountInput[0]
      self.mission._dirty = true
    end
    im.tooltip("Amount of active traffic vehicles running at the same time; other vehicles stay hidden until they get cycled.")
    im.PopItemWidth()

    if setupModule.amount ~= 0 and setupModule.activeAmount <= 0 then
      im.SameLine()
      im.TextColored(im.ImVec4(1, 1, 0, 1), " Warning: All traffic vehicles will start out as hidden.")
    end

    im.PushItemWidth(inputWidth)
    if im.InputInt("Parked Amount##traffic", self.trafficParkedAmountInput, 1) then
      setupModule.parkedAmount = self.trafficParkedAmountInput[0]
      self.mission._dirty = true
    end
    im.tooltip("Amount of parked vehicles to spawn.")
    im.PopItemWidth()

    im.PushItemWidth(inputWidth)
    if im.InputFloat("Respawn Rate##traffic", self.trafficRespawnRateInput, 0.1, nil, "%.2f") then
      setupModule.respawnRate = self.trafficRespawnRateInput[0]
      self.mission._dirty = true
    end
    im.tooltip("Traffic respawn rate; values can range from 0 to 3.")
    im.PopItemWidth()

    if setupModule.respawnRate and setupModule.respawnRate <= 0 then
      im.SameLine()
      im.TextColored(im.ImVec4(1, 1, 0, 1), " Warning: All traffic vehicles will not respawn during gameplay.")
    end

    local innerDisabled = false
    if im.Checkbox("Enable Traffic as Default Setting##traffic", self.trafficUseTrafficInput) then
      setupModule.useTraffic = self.trafficUseTrafficInput[0]
      self.mission._dirty = true
    end
    im.tooltip("If true, this mission will start with traffic enabled unless the user changes the setting.")

    if im.Checkbox("Keep Previous Traffic##traffic", self.trafficPrevTrafficInput) then
      setupModule.usePrevTraffic = self.trafficPrevTrafficInput[0]
      self.mission._dirty = true
    end
    im.tooltip("If true, this mission will try to use traffic that already existed in freeroam.")

    if im.Checkbox("Use Settings From Traffic Options##traffic", self.trafficUserOptionsInput) then
      setupModule.useGameOptions = self.trafficUserOptionsInput[0]
      self.mission._dirty = true
    end

    if not innerDisabled and setupModule.useGameOptions then innerDisabled = true end
    if innerDisabled then im.BeginDisabled() end
    if im.Checkbox("Use Simple Vehicles##traffic", self.trafficSimpleVehsInput) then
      setupModule.useSimpleVehs = self.trafficSimpleVehsInput[0]
      innerDisabled = true
      self.mission._dirty = true
    end
    if innerDisabled then im.EndDisabled() end

    if not innerDisabled and setupModule.useSimpleVehs then innerDisabled = true end
    if innerDisabled then im.BeginDisabled() end
    if im.Checkbox("Use Custom Vehicle Group##traffic", self.trafficUseCustomGroup) then
      setupModule.useCustomGroup = self.trafficUseCustomGroup[0]
      self.mission._dirty = true
    end
    im.tooltip("If true, enables a custom vehicle group to use for traffic.")
    if setupModule.useCustomGroup then
      if not setupModule.customGroupFile then
        setupModule.customGroupFile = self.mission.missionFolder.."/custom.vehGroup.json"
        self.mission._dirty = true
      end
      if not self.trafficCustomGroupInput then
        self.trafficCustomGroupInput = im.ArrayChar(1024, setupModule.customGroupFile)
      end

      if editor.uiInputFile("Vehicle Group##traffic", self.trafficCustomGroupInput, nil, nil, {{"Vehicle group files", ".vehGroup.json"}}, im.InputTextFlags_EnterReturnsTrue) then
        setupModule.customGroupFile = ffi.string(self.trafficCustomGroupInput)
        self.mission._dirty = true
      end
    else
      setupModule.customGroupFile = nil
      self.trafficCustomGroupInput = nil
    end
    if innerDisabled then im.EndDisabled() end
  else
    if isBlocked then
      im.Text("Traffic setup is not available for this mission type.")
    else
      table.clear(self.mission.setupModules.traffic)
      setupModule.enabled = false
      im.Text("Select this to enable traffic setup.")
    end
  end
  if isBlocked then
    im.EndDisabled()
  end

  im.Separator()
  im.NextColumn()
  im.Text("Environment")
  im.NextColumn()

  setupModule = self.mission.setupModules.environment
  isBlocked = self.blockedSetupModules.environment
  if isBlocked then
    setupModule.enabled = false
    im.BeginDisabled()
  end
  if im.Checkbox("##setupModuleEnvironmentEnabled", im.BoolPtr(setupModule.enabled)) then
    setupModule.enabled = not setupModule.enabled
    self.mission._dirty = true
  end
  im.SameLine()

  if setupModule.enabled then
    if not setupModule.time then -- init values
      setupModule.time = self.todInput[0]
      setupModule.timeScale = self.todScaleInput[0]
      setupModule.windSpeed = self.windSpeedInput[0]
      setupModule.windDirAngle = self.windDirectionInput[0]
      setupModule.fogDensity = self.fogDensityInput[0]
      setupModule.todUserSetting = self.todUserSettingInput[0]
      setupModule.weatherUserSetting = self.weatherUserSettingInput[0]
    end

    im.Text("Environment Setup")
    im.PushItemWidth(inputWidth)
    if im.InputFloat("##tod", self.todInput) then
      self.todInput[0] = clamp(self.todInput[0], 0, 1)
      setupModule.time = self.todInput[0]
      self.mission._dirty = true
    end
    im.PopItemWidth()
    im.SameLine()
    im.Text(todToTime(self.todInput[0]))
    im.SameLine()

    im.PushItemWidth(100 * im.uiscale[0])
    if im.BeginCombo("##todSelector", "...") then
      if core_environment and core_environment.getTimeOfDay() then
        if im.Selectable1("Default##todSelector") then
          self.todInput[0] = -1
          setupModule.time = -1
          self.mission._dirty = true
        end
        if im.Selectable1("Now##todSelector") then
          local now = core_environment.getTimeOfDay().time
          self.todInput[0] = now
          setupModule.time = now
          self.mission._dirty = true
        end
      end
      for i = 0, 48 do
        local val = (i / 48 + 0.5) % 1
        if im.Selectable1(todToTime(val).."##todSelector") then
          setupModule.time = val
          self.todInput[0] = val
          self.mission._dirty = true
        end
      end
      im.EndCombo()
    end
    im.PopItemWidth()
    im.SameLine()
    im.Text("Time")

    im.PushItemWidth(inputWidth)
    if im.InputFloat("Time Scale##environment", self.todScaleInput, 0.1, nil, "%.2f") then
      self.todScaleInput[0] = math.max(0, self.todScaleInput[0])
      setupModule.timeScale = self.todScaleInput[0]
      self.mission._dirty = true
    end
    im.tooltip("Set to 0 to ignore this setting.")
    im.PopItemWidth()

    im.PushItemWidth(inputWidth)
    if im.InputFloat("Wind Speed##environment", self.windSpeedInput, 0.1, nil, "%.2f") then
      self.windSpeedInput[0] = math.max(0, self.windSpeedInput[0])
      setupModule.windSpeed = self.windSpeedInput[0]
      self.mission._dirty = true
    end
    im.PopItemWidth()

    im.PushItemWidth(inputWidth)
    if im.InputFloat("Wind Direction##environment", self.windDirectionInput, 1, nil, "%.2f") then
      self.windDirectionInput[0] = clamp(self.windDirectionInput[0], 0, 360)
      setupModule.windDirAngle = self.windDirectionInput[0]
      self.mission._dirty = true
    end
    im.PopItemWidth()

    im.PushItemWidth(inputWidth)
    if im.InputFloat("Fog Density##environment", self.fogDensityInput, 0.001, nil, "%.3f") then
      self.fogDensityInput[0] = clamp(self.fogDensityInput[0], 0, 1)
      setupModule.fogDensity = self.fogDensityInput[0]
      self.mission._dirty = true
    end
    im.tooltip("Use very low values for typical fog conditions (such as 0.05).")
    im.PopItemWidth()

    im.PushItemWidth(inputWidth)
    if im.Checkbox("Enable User Setting for Time##environment", self.todUserSettingInput) then
      setupModule.todUserSetting = self.todUserSettingInput[0]
      self.mission._dirty = true
    end
    im.tooltip("If true, the user can set the mission time of day.")
    im.PopItemWidth()
  else
    if isBlocked then
      im.Text("Environment setup is not available for this mission type.")
    else
      table.clear(self.mission.setupModules.environment)
      setupModule.enabled = false
      im.Text("Select this to enable environment setup.")
    end
  end
  if isBlocked then
    im.EndDisabled()
  end

  im.Columns(1)
  im.PopID()
end


function C:displayDiffSetup(veh)
  local diffSetupCount = 0
   + (veh.toggleDifferentials and 1 or 0)
   + (veh.lockFrontDiffs and 1 or 0)
   + (veh.lockRearDiffs and 1 or 0)
   + (veh.setTransfercase and 1 or 0)
   + (veh.setRangebox and 1 or 0)
  local diffSetupText = {}
  if veh.toggleDifferentials then table.insert(diffSetupText,"Toggle Diffs") end
  if veh.lockFrontDiffs then table.insert(diffSetupText,"Lock Front Diffs") end
  if veh.lockRearDiffs then table.insert(diffSetupText,"Lock Rear Diffs") end
  if veh.setTransfercase then table.insert(diffSetupText,"Transfercase to 4hi") end
  if veh.setRangebox then table.insert(diffSetupText,"Rangebox to Low") end
  im.SetNextItemWidth(im.GetContentRegionAvailWidth())
  if im.BeginCombo("##setupdiff", diffSetupCount .. " / " .. 5 .. " (" .. table.concat( diffSetupText, ", " )..")") then
    if im.Checkbox("All", im.BoolPtr(diffSetupCount == 5)) then
      veh.toggleDifferentials = diffSetupCount ~= 5
      veh.lockFrontDiffs = diffSetupCount ~= 5
      veh.lockRearDiffs = diffSetupCount ~= 5
      veh.setTransfercase = diffSetupCount ~= 5
      veh.setRangebox = diffSetupCount ~= 5
      self.mission._dirty = true
    end
    if im.Checkbox("Toggle Differentials", im.BoolPtr(veh.toggleDifferentials or false) ) then
      veh.toggleDifferentials = not veh.toggleDifferentials
      self.mission._dirty = true
    end
    if im.Checkbox("Front Differentials", im.BoolPtr(veh.lockFrontDiffs or false) ) then
      veh.lockFrontDiffs = not veh.lockFrontDiffs
      self.mission._dirty = true
    end
    if im.Checkbox("Rear Differentials", im.BoolPtr(veh.lockRearDiffs or false) ) then
      veh.lockRearDiffs = not veh.lockRearDiffs
      self.mission._dirty = true
    end
    if im.Checkbox("Transfercase to 4hi", im.BoolPtr(veh.setTransfercase or false) ) then
      veh.setTransfercase = not veh.setTransfercase
      self.mission._dirty = true
    end
    if im.Checkbox("Rangebox to low", im.BoolPtr(veh.setRangebox or false) ) then
      veh.setRangebox = not veh.setRangebox
      self.mission._dirty = true
    end

    im.EndCombo()
  end
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
