-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local lastDiscover = nil
local freeroamDiscovers = {
  leisurelyDrive = {
    order = 1,
    name = "ui.experiences.leisurelyDrive.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5"}},
    description = "ui.experiences.leisurelyDrive.description",
    image = "/gameplay/discover/images/leisurelyDrive.jpg",
    trigger = function()
      -- enable trafficLoadForFreeroam
      freeroam_freeroam.setForceTrafficLoading({traffic = true, parkedVehicles = true})
      freeroam_freeroam.startFreeroamByName("italy", "spawn_town_east", nil, false)
      M.onPlayerCameraReady = function()
        local vehs = {
          { vec3(189.2717133,-375.0607605,194.2601013), quat(0,0,0.7038784663,0.710320424), "vivace", "vehicles/vivace/vivace_230S_DCT.pc" },
        }
        local setupVehs = {}
        for i, veh in ipairs(vehs) do
          local spawningOptions = sanitizeVehicleSpawnOptions(veh[3], {config = veh[4]})
          spawningOptions.pos = veh[1]
          spawningOptions.rot = veh[2]
          spawningOptions.autoEnterVehicle = i == 1
          local v = core_vehicles.spawnNewVehicle(spawningOptions.model, spawningOptions)
        end
        extensions.load('util_stepHandler')
        local seq = {
          util_stepHandler.makeStepWait(1),
          util_stepHandler.makeStepReturnTrueFunction(function(step)
            step.timeout = math.huge
            gameplay_traffic.setActiveAmount(0)
            freeroam_bigMapMode.setNavFocus(vec3(310.2650452,1816.934692,207.3096924))
            return true
          end),
          util_stepHandler.makeStepWait(1),
          util_stepHandler.makeStepReturnTrueFunction(function(step)
            step.timeout = math.huge
            local playerPos = be:getPlayerVehicle(0):getPosition()
            if playerPos:squaredDistance(vehs[1][1]) > 5*5 then
              gameplay_traffic.setActiveAmount(100)
              return true
            end
            return false
          end),
        }
        util_stepHandler.startStepSequence(seq)
        M.basicControlsIntroPopup()
        M.onPlayerCameraReady = nil
      end
      extensions.hookUpdate('onPlayerCameraReady')
      return true
    end,
    damageTracking = false,
    tasks = function()
      guihooks.trigger('ClearTasklist')
      guihooks.trigger("SetTasklistHeader", {
        label = "ui.experiences.leisurelyDrive.title",
        subtext = {txt = "ui.experiences.general.freeroamLevel", context = {level = "levels.italy.info.title"}}
      })
      guihooks.trigger("SetTasklistTask", {
        label = "ui.experiences.general.explore",
        subtext = "ui.experiences.leisurelyDrive.task.subtext",
        type = "message",
      })
    end,

  },


  johnson_valley = {
    order = 2,
    name = "ui.experiences.johnsonValley.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5-10"}},
    description = "ui.experiences.johnsonValley.description",
    image = "/gameplay/discover/images/offroad.jpg",
    trigger = function()
      -- enable trafficLoadForFreeroam
      freeroam_freeroam.setForceTrafficLoading({traffic = false, parkedVehicles = false})
      freeroam_freeroam.startFreeroamByName("johnson_valley", "spawn_remote_pits", nil, false)
      M.onPlayerCameraReady = function()
        local vehs = {
          { vec3(-568.1539917,-408.5489807,131.8844299), quat(-0.02376894851,-0.02688521755,-0.05357358755,0.9979188809), "pickup", "vehicles/pickup/deserttruck_crawler_A.pc" },
          { vec3(-563.8013306,-403.6004944,131.9705658), quat(0.01239661853,-0.03992601871,0.2992039956,0.9532728916), "racetruck", "vehicles/racetruck/tt2_spec.pc" },
          --{ vec3(-574.324,-404.611,132.277), quat(-0.004031204019,-0.03853915852,-0.4529694097,0.8906835558), "rockbouncer", "vehicles/rockbouncer/rock_crawler.pc" },
          --{ vec3(-574.530,-404.151,132.347), quat(-0.004031204019,-0.03853915852,-0.4529694097,0.8906835558), "roamer", "vehicles/roamer/adventure.pc" },
          { vec3(-573.742,-405.171,132.129), quat(-0.004031204019,-0.03853915852,-0.4529694097,0.8906835558), "utv", "vehicles/utv/plus.pc" },
        }
        local setupVehs = {}
        for i, veh in ipairs(vehs) do
          local spawningOptions = sanitizeVehicleSpawnOptions(veh[3], {config = veh[4]})
          spawningOptions.pos = veh[1]
          spawningOptions.rot = veh[2]
          spawningOptions.autoEnterVehicle = i == 1
          local v = core_vehicles.spawnNewVehicle(spawningOptions.model, spawningOptions)
          if i <= 3 then
            setupVehs[v:getID()] = [[
              for _, v in ipairs(powertrain.getDevicesByType("differential")) do powertrain.toggleDeviceMode(v.name) end
              controller.getControllerSafe("frontLockControl").setDriveMode('locked')
              controller.getControllerSafe("rearLockControl").setDriveMode('locked')
              controller.getControllerSafe("transfercaseControl").setDriveMode('4lo')
              controller.getControllerSafe("transfercaseControl").setDriveMode('high')
              controller.getControllerSafe("rangeboxControl").setDriveMode('low')
            ]]
          end
        end
        extensions.load('util_stepHandler')
        local seq = {
          util_stepHandler.makeStepReturnTrueFunction(function()
            local dones = {}
            for vehId, setup in pairs(setupVehs) do
              local veh = getObjectByID(vehId)
              if veh and veh:isReady() then
                if not dones[vehId] then
                  dones[vehId] = true
                  veh:queueLuaCommand(setup)
                  log("I","discover","Setting up vehicle "..vehId.." with command: "..setup)
                end
              end
            end
            for id, _ in pairs(dones) do
              setupVehs[id] = nil
            end
            if not next(setupVehs) then
              return true
            end
            return false
          end),
        }
        util_stepHandler.startStepSequence(seq)
        M.basicControlsIntroPopup()
        M.onPlayerCameraReady = nil
        extensions.hookUpdate('onPlayerCameraReady')
      end
      extensions.hookUpdate('onPlayerCameraReady')
      return true
    end,
    damageTracking = true,

    tasks = function()
      guihooks.trigger('ClearTasklist')
      guihooks.trigger("SetTasklistHeader", {
        label = "ui.experiences.johnsonValley.title",
        subtext = {txt = "ui.experiences.general.freeroamLevel", context = {level = "levels.johnson_valley.info.title"}}
      })
      guihooks.trigger("SetTasklistTask", {
        label = "ui.experiences.general.explore",
        subtext = "ui.experiences.johnsonValley.task.subtext",
        type = "message",
      })
    end,
  },

  trackday = {
    order = 3,
    name = "ui.experiences.trackday.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5-15"}},
    description = "ui.experiences.trackday.description",
    image = "/gameplay/discover/images/trackday.jpg",
    trigger = function()
      -- enable trafficLoadForFreeroam
      freeroam_freeroam.setForceTrafficLoading({traffic = false, parkedVehicles = false})
      freeroam_freeroam.startFreeroamByName("hirochi_raceway", "spawn_pitlane", nil, false)
      M.onPlayerCameraReady = function()
        local vehChoices = {
          { "sunburst2", "vehicles/sunburst2/sport_RS_DCT.pc" },
          { "etkc", "vehicles/etkc/kc6x_trackday_A.pc" },
          { "scintilla", "vehicles/scintilla/gtx.pc" },
          { "etk800", "vehicles/etk800/856_ttsport_DCT.pc" },
          { "vivace", "vehicles/vivace/vivace_S_410q_M.pc" },
        }
        local vehs = {
          { vec3(-455.5000305,374.7161865,25.23128128), quat(6.270791814e-05,0.0005269290997,0.9929929419,-0.1181724831), vehChoices[1][1], vehChoices[1][2] },
          { vec3(-446.6066589,361.14505,25.16402245), quat(3.034665428e-05,0.0002429556407,0.9922893041,-0.123943039), vehChoices[2][1], vehChoices[2][2] },
          { vec3(-438.1695862,346.2776184,25.1306839), quat(-0.0001477062226,-0.001071917166,0.9906386067,-0.1365063375), vehChoices[3][1], vehChoices[3][2] },
          { vec3(-429.155426,333.7767334,25.10681915), quat(-8.45975059e-05,-0.0007574605165,0.9938206113,-0.1109955479), vehChoices[4][1], vehChoices[4][2] },
          { vec3(-420.4411316,319.9455872,25.14068985), quat(0,-0,0.9944942508,-0.10479115), vehChoices[5][1], vehChoices[5][2] },
        }
        for i, veh in ipairs(vehs) do
          local spawningOptions = sanitizeVehicleSpawnOptions(veh[3], {config = veh[4]})
          spawningOptions.pos = veh[1]
          spawningOptions.rot = veh[2]
          spawningOptions.autoEnterVehicle = i == 1
          core_vehicles.spawnNewVehicle(spawningOptions.model, spawningOptions)
        end
        M.basicControlsIntroPopup()
        M.onPlayerCameraReady = nil
        extensions.hookUpdate('onPlayerCameraReady')
      end
      extensions.hookUpdate('onPlayerCameraReady')

      M.onClientPostStartMission = function()
        core_environment.setTimeOfDay({time=0.79000})
        core_environment.setFogDensity(0.001540986122)
        M.onClientPostStartMission = nil
        extensions.hookUpdate('onClientPostStartMission')
      end
      extensions.hookUpdate('onClientPostStartMission')
      return true
    end,
    damageTracking = true,

    tasks = function()
      guihooks.trigger('ClearTasklist')
      guihooks.trigger("SetTasklistHeader", {
        label = "ui.experiences.trackday.title",
        subtext = {txt = "ui.experiences.general.freeroamLevel", context = {level = "levels.hirochi_raceway.info.title"}}
      })
      guihooks.trigger("SetTasklistTask", {
        label = "ui.experiences.trackday.task1.label",
        subtext = "ui.experiences.trackday.task1.subtext",
        type = "message",
      })
      guihooks.trigger("SetTasklistTask", {
        label = "ui.experiences.general.explore",
        subtext = "ui.experiences.trackday.task2.subtext",
        type = "message",
      })
    end,
  },

  propDestruction = {
    order = 4,
    name = "ui.experiences.propDestruction.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5"}},
    description = "ui.experiences.propDestruction.description",
    image = "/gameplay/discover/images/propDestruction.jpg",
    trigger = function()
      -- enable trafficLoadForFreeroam
      freeroam_freeroam.setForceTrafficLoading({traffic = false, parkedVehicles = false})
      freeroam_freeroam.startFreeroamByName("Industrial", "spawn_factory", nil, false)
      M.onPlayerCameraReady = function()
        local vehs = {
          { vec3(-143.3728027,83.48352814,35.11228561), quat(-0.004508387994,0.0001588892785,0.03522081038,0.999369373), "van", "vehicles/van/h25_worker.pc" },
          { vec3(-171.8866425,127.8188705,34.96841812), quat(0.02066615241,-0.008679200679,0.387113079,0.9217597548), "cannon", "vehicles/cannon/cannon.pc" },
          { vec3(-158.3771362,143.2392883,35.22563553), quat(-0.001331452813,-0.001542004732,-0.7568890667,0.6535401978), "caravan", "vehicles/caravan/default.pc" },
          { vec3(-144.6221008,157.9118805,35.99853516), quat(-1.780725942e-06,-5.771057474e-06,0.9555452647,-0.2948444456), "fridge", "vehicles/fridge/standard.pc" },
          { vec3(-150.1018219,158.1330872,36.21847534), quat(1.766898486e-06,-1.028953559e-05,0.985574711,0.1692409196), "porta_potty", "vehicles/porta_potty/default.pc" },
          { vec3(-146.2707825,159.0570221,36.05832672), quat(0.0001382667643,1.267887785e-06,-0.009169481016,0.9999579499), "piano", "vehicles/piano/standard.pc" },
          { vec3(-138.2297058,102.0587769,34.79919434), quat(-0.0001324840225,0.0001432935834,0.7342591574,0.6788692449), "tv", "vehicles/tv/25inch.pc" },
          { vec3(-136.6850739,104.0551834,34.89838409), quat(-1.370823688e-06,-2.668151342e-06,0.8894732678,-0.4569872053), "couch", "vehicles/couch/couch_free.pc" },
          { vec3(-135.8791656,101.8557205,34.79919052), quat(0.0002344426654,0.0001473668705,-0.5321790558,0.8466317829), "couch", "vehicles/couch/armchair_free.pc" },
          { vec3(-135.1573334,102.8125229,34.79919434), quat(4.743933507e-05,1.696156212e-05,-0.3366698313,0.94162276), "barrels", "vehicles/barrels/empty.pc" },
          { vec3(-131.6051788,157.0005493,36.81754684), quat(0.0001472245881,0.01930025774,0.9997846345,-0.007626472298), "steel_coil", "vehicles/steel_coil/20ton.pc" },
          { vec3(-131.9969635,153.5990143,35.00673676), quat(-2.599318111e-05,-0.003242814254,0.9999626183,-0.008015324778), "pigeon", "vehicles/pigeon/base.pc" }
        }
        for i, veh in ipairs(vehs) do
          local spawningOptions = sanitizeVehicleSpawnOptions(veh[3], {config = veh[4]})
          spawningOptions.pos = veh[1]
          spawningOptions.rot = veh[2]
          spawningOptions.autoEnterVehicle = i == 1
          local veh = core_vehicles.spawnNewVehicle(spawningOptions.model, spawningOptions)
          --veh.playerUsable = i <= 2
        end
        M.basicControlsIntroPopup()
        M.onPlayerCameraReady = nil
        extensions.hookUpdate('onPlayerCameraReady')
      end
      extensions.hookUpdate('onPlayerCameraReady')
      return true
    end,



    tasks = function()
      guihooks.trigger('ClearTasklist')
      guihooks.trigger("SetTasklistHeader", {
        label = "ui.experiences.propDestruction.title",
        subtext = {txt = "ui.experiences.general.freeroamLevel", context = {level = "levels.industrial.info.title"}}
      })
      guihooks.trigger("SetTasklistTask", {
        label = "ui.experiences.propDestruction.task.label",
        subtext = "ui.experiences.propDestruction.task.subtext",
        type = "message",
      })
    end,
  },

  ramplow = {
    order = 5,
    name = "ui.experiences.ramplow.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5"}},
    description = "ui.experiences.ramplow.description",
    image = "/gameplay/discover/images/ramplow.jpg",
    trigger = function()



      freeroam_freeroam.setForceTrafficLoading({traffic = true, parkedVehicles = false})
      freeroam_freeroam.startFreeroamByName("west_coast_usa", "spawn_highway", nil, false)
      M.onPlayerCameraReady = function()
        local vehs = {
          { vec3(-926.2694092,-530.4829712,105.0409012), quat(0.02295934544,0.03978534072,0.5222144473,0.8515762245), "us_semi", "vehicles/us_semi/t82_ramplow.pc" },
        }
        local setupVehs = {}
        for i, veh in ipairs(vehs) do
          local spawningOptions = sanitizeVehicleSpawnOptions(veh[3], {config = veh[4]})
          spawningOptions.pos = veh[1]
          spawningOptions.rot = veh[2]
          spawningOptions.autoEnterVehicle = i == 1
          local v = core_vehicles.spawnNewVehicle(spawningOptions.model, spawningOptions)
        end
        M.basicControlsIntroPopup()
        M.onPlayerCameraReady = nil
        extensions.hookUpdate('onPlayerCameraReady')
      end
      extensions.hookUpdate('onPlayerCameraReady')
      return true
    end,
    tasks = function()
      guihooks.trigger('ClearTasklist')
      guihooks.trigger("SetTasklistHeader", {
        label = "ui.experiences.ramplow.title",
        subtext = {txt = "ui.experiences.general.freeroamLevel", context = {level = "levels.west_coast_usa.info.title"}}
      })
      guihooks.trigger("SetTasklistTask", {
        label = "ui.experiences.ramplow.task.label",
        subtext = "ui.experiences.ramplow.task.subtext",
        type = "message",
      })
    end,
  },


  small_island_pursuit = {
    order = 100,
    missionId = "small_island/chase/001-Small",
    description = "ui.experiences.missions.smallIslandPursuit.description",
    icon = "wigwags",
    image = "/gameplay/discover/images/small_island_pursuit.jpg",
    name = "ui.experiences.missions.smallIslandPursuit.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5"}},
  },
  get_down = {
    order = 101,
    missionId = "cliff/arrive/001-Get",
    description = "ui.experiences.missions.getDown.description",
    icon = "wigwags",
    image = "/gameplay/discover/images/get_down.jpg",
    name = "ui.experiences.missions.getDown.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "2"}},
  },
  garage_to_garage = {
    order = 102,
    missionId = "italy/garageToGarage/002-GlobalGeneric",
    description = "ui.experiences.missions.garageToGarage.description",
    icon = "toGarage",
    image = "/gameplay/discover/images/garage_to_garage.jpg",
    name = "ui.experiences.missions.garageToGarage.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5-30"}},
  },
  tasticola_restock = {
    order = 103,
    missionId = "italy/delivery/003-tastiCola",
    description = "ui.experiences.missions.tasticolaRestock.description",
    icon = "deliveryTruckArrows",
    image = "/gameplay/discover/images/tasticola_restock.jpg",
    name = "ui.experiences.missions.tasticolaRestock.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5"}},
  },
  orchard_hill = {
    order = 104,
    missionId = "italy/rallyStage/003-ssorchardhill",
    description = "ui.experiences.missions.orchardHill.description",
    icon = "raceFlag",
    image = "/gameplay/discover/images/orchard_hill.jpg",
    name = "ui.experiences.missions.orchardHill.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5"}},
  },
  ring_road = {
    order = 105,
    missionId = "small_island/aiRace/001-ring",
    description = "ui.experiences.missions.ringRoad.description",
    icon = "AIRace",
    image = "/gameplay/discover/images/ring_road.jpg",
    name = "ui.experiences.missions.ringRoad.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5"}},
  },
  slithery_drift_short = {
    order = 106,
    missionId = "italy/drift/002-mountainShort",
    description = "ui.experiences.missions.slitheryDriftShort.description",
    icon = "drift01",
    image = "/gameplay/discover/images/slithery_drift_short.jpg",
    name = "ui.experiences.missions.slitheryDriftShort.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5"}},
  },
  ridgeway_rider = {
    order = 107,
    missionId = "johnson_valley/crawl/003-Ridgeway",
    description = "ui.experiences.missions.ridgewayRider.description",
    icon = "rockCrawling01",
    image = "/gameplay/discover/images/ridgeway_rider.jpg",
    name = "ui.experiences.missions.ridgewayRider.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "5"}},
  },
  barrel_knocker = {
    order = 108,
    missionId = "industrial/knockAway/001-barrels",
    description = "ui.experiences.missions.barrelKnocker.description",
    icon = "barrelKnocker01",
    image = "/gameplay/discover/images/barrel_knocker.jpg",
    name = "ui.experiences.missions.barrelKnocker.title",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "2"}},
  },
  drag_strip_race = {
    order = 109,
    missionId = "hirochi_raceway/dragStripRace/001-hirochiDrag_500",
    description = "ui.experiences.missions.dragStripRace.description",
    icon = "stopwatchSectionOutlinedEnd",
    name = "ui.experiences.missions.dragStripRace.title",
    image = "/gameplay/discover/images/drag_strip_race.jpg",
    tag = {txt = "ui.experiences.general.timeTag", context = {time = "1-5"}},
    model = "covet",
    config = "vehicles/covet/15gtz_turbo2_M.pc"
  }
}




M.onLoadingScreenFadeout = onLoadingScreenFadeout
M.onClientStartMission = onClientStartMission

local doneIntroPopups = {}

M.basicControlsIntroPopup = function()
  local deviceOrder = {"wheel","joystick","xinput","gamepad","mouse","keyboard"}
  local devices = {}
  for k, v in pairs(core_input_bindings.bindings) do
    if v.contents.devicetype and v.contents.imagePack then
      table.insert(devices, {device = v.contents.devicetype, imagePack = v.contents.imagePack})
    end
  end
  local function findIndex(arr, val)
    for i, v in ipairs(arr) do
      if v == val then return i end
    end
    return -1
  end
  table.sort(devices, function(a, b) return findIndex(deviceOrder, a.device) < findIndex(deviceOrder, b.device) end)
  table.insert(devices, {device = "fallback", imagePack = "fallback"})
  for i, v in ipairs(devices) do
    local popup = "basicDriving_"..v.imagePack
    if FS:fileExists("/gameplay/discover/popups/"..popup.."/content.html") then
      if not doneIntroPopups[popup] then
        M.introPopup(popup)
      end
      return
    else
      log("I","","Basic Driving Popup not found: "..popup)
    end
  end
end

M.introPopup = function(id)
  if doneIntroPopups[id] then
    return
  end
  doneIntroPopups[id] = true
  local file = "/gameplay/discover/popups/"..id.."/content.html"
  if not FS:fileExists(file) then
    return
  end
  local content = readFile(file):gsub("\r\n","")
  local entry = {
    type = "info",
    content = content,
    flavour = "onlyOk",
    isPopup = true,
  }
  log("I","","Intro Popup: " .. id)
  guihooks.trigger("introPopupTutorial", {entry})
end
--M.onClientEndMission = function() lastDiscover = nil end




local function getDiscoverCards()
  local cards = {
    freeroam = {},
    challenge = {},
  }
  for id, discover in pairs(freeroamDiscovers) do
    if discover.disabled then
      goto continue
    end
    local card = deepcopy(discover)
    card.discoverId = id
    card.icon = card.icon or "road"
    if card.missionId and not card.name then
      local mission = gameplay_missions_missions.getMissionById(card.missionId)
      card.name = card.name or mission.name
      card.description = card.description or mission.description
      card.image = card.image or mission.previewFile
      card.icon = mission.iconFontIcon
    end

    table.insert(cards.freeroam, card)
    ::continue::
  end
  table.sort(cards.freeroam, function(a, b) return a.order < b.order end)
  return cards
end

local function startDiscover(discoverId)
  if lastDiscover then
    log("W", "discover", "Already loading discover: " .. lastDiscover..", ignoring: " .. discoverId)
    return
  end
  local discover = freeroamDiscovers[discoverId]
  lastDiscover = discoverId
  if discover then
    if discover.missionId then
      log('I', 'discover', 'Starting discover mission: ' .. discoverId)
      local mission = gameplay_missions_missions.getMissionById(discover.missionId)
      local scenario = scenario_scenariosLoader.getScenarioDataForMission(mission)
      if discover.model and discover.config then
        log('I', 'discover', 'Setting model and config for discover mission: ' .. discoverId)
        scenario.variables.model = discover.model
        scenario.variables.config = discover.config
      end
      scenario_scenariosLoader.start(scenario)
      M.onLoadingScreenFadeout = function()
        lastDiscover = nil
        M.onLoadingScreenFadeout = nil
        extensions.hookUpdate('onLoadingScreenFadeout')
      end
      extensions.hookUpdate('onLoadingScreenFadeout')
    else
      log('I', 'discover', 'Starting discover: ' .. discoverId)
      discover.trigger()
      M.onLoadingScreenFadeout = function()
        log('I', 'discover', 'Running discover tasks: ' .. discoverId)
        if discover.tasks then
          discover.tasks()
        end
        lastDiscover = nil
        M.onLoadingScreenFadeout = nil
        extensions.hookUpdate('onLoadingScreenFadeout')
      end
      extensions.hookUpdate('onLoadingScreenFadeout')

      M.onClientStartMission = function()
        log('I', 'discover', 'Setting game state to freeroam, discover: ' .. discoverId)
        core_gamestate.setGameState("freeroam","discover", nil)
        M.onClientStartMission = nil
        extensions.hookUpdate('onClientStartMission')
      end
      extensions.hookUpdate('onClientStartMission')
    end
  end
end

-- can be started with -discover <discoverName>
local function onInit()
  setExtensionUnloadMode(M, "manual")
  local cmdArgs = Engine.getStartingArgs()
  for i, v in ipairs(cmdArgs) do
    if v == "-discover" then
      local discover = freeroamDiscovers[cmdArgs[i + 1]]
      if discover then
        discover.trigger()
      end
    end
  end
end

M.getDiscoverCards = getDiscoverCards
M.onInit = onInit
M.startDiscover = startDiscover

M.onSerialize = function()
  local data = {}
  --data.lastDiscover = lastDiscover
  return data
end

M.onDeserialized = function(data)
  --lastDiscover = data.lastDiscover
end
return M
