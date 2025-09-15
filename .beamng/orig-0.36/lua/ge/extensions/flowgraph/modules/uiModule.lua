-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local C = {}
C.moduleOrder = 1000 -- low first, high later
C.idCounter = 0
C.hooks = {'onRequestMissionScreenData', 'onMissionScreenButtonClicked', "onGetIsMissionStartOrEndScreenActive"}
C.dependencies = {'ui_gameplayAppContainers'}
function C:getFreeId()
  self.idCounter = self.idCounter + 1
  return self.idCounter
end

function C:init()
  self:clear()
end

function C:clear()
  self.isBuilding = false
  self.uiLayout = nil
  self.appContainerContext = nil
end

function C:setGameState(...)
  core_gamestate.setGameState(...)
end

function C:setAppContainerContext(context)
  if context and context ~= '' and ui_gameplayAppContainers then
    ui_gameplayAppContainers.setContainerContext('gameplayApps',context)
    self.appContainerContext = context
  end
end

function C:keepGameState(keep)
  self.recoverGameStateWhenExecutionStopped = not keep
end

function C:clearButtonFunctions()
  self.buttonFunctions = {}
end

function C:startUIBuilding(uiMode, node)
  if self.isBuilding then
    log("E","","Tried to start building, but is already building!")
    return
  end
  self.uiLayout = { mode = uiMode, header = nil, layout = {}, buttons = {}, _node = {nodeId = node.id, graphId = node.graph.id}, supportsReplay = false }
  self.isBuilding = true
  log("I","","Starting to build.")
  self:clearButtonFunctions()
end

function C:setupPages()
  -- Static page configuration
  local pageConfig = {
    timeslip = {
      order = 1,
      icon = "timer",
      mandatory = true
    },
    main = {
      order = 2,
      icon = "medal",
      mandatory = true
    },
    rewards = {
      order = 3,
      icon = "beamCurrency",
      mandatory = true,
      hideInCareer = true
    },
    unlocks = {
      order = 4,
      icon = "lockOpened",
      mandatory = true
    },
    leaderboards = {
      order = 5,
      icon = "chartBars"
    },
    drift = {
      order = 6,
      icon = "drift01"
    },
    lapTimes = {
      order = 7,
      icon = "timer"
    },
    rules = {
      order = 8,
      icon = "roadInfo"
    },
    recording = {
      order = 10,
      icon = "playRound"
    },
    crashAnalysisStep = {
      order = 9,
      icon = "test",
      mandatory = true
    }
  }

  -- Build pages lookup from all panels
  local pages = {}
  local nextOrder = 0

  -- Find highest predefined order
  for _, config in pairs(pageConfig) do
    if config.order > nextOrder then
      nextOrder = config.order
    end
  end
  nextOrder = nextOrder + 1

  if not next(self.uiLayout.layout) then
    log("W","","No layout elements found for mission start/end screen!")
    self:addUIElement({
      type = 'textPanel',
      header = "No layout elements found",
      text = "You havent added any elements to this screen.",
      experimental = true,
      pages = {main = true},
    })
  end

  -- Process layout elements
  for _, element in ipairs(self.uiLayout.layout) do
    if element.pages then
      for pageName, enabled in pairs(element.pages) do
        if not career_career.isActive() and pageConfig[pageName].hideInCareer then
          enabled = false
        end
        if enabled and not pages[pageName] then
          local config = pageConfig[pageName] or {}
          pages[pageName] = {
            order = config.order or nextOrder,
            icon = config.icon or "info",
            label = pageName,
            mandatory = self.uiLayout.mode == 'endScreen' and config.mandatory or false
          }
          if not config.order then
            nextOrder = nextOrder + 1
          end
        end
      end
    end
  end

  -- Add pages info to layout
  self.uiLayout.pages = pages
end

function C:finishUIBuilding()
  if not self.isBuilding then
    log("E","","Tried to finish building, but is currently not building!")
    return
  end
  local mission = self.mgr.activity
  log("I","","Finishing Build...")
  log("I", "", dumpsz(self.uiLayout.layout, 2))

  if self.uiLayout.mode == 'startScreen' then
    table.insert(self.uiLayout.buttons, self:addButton(function()
      self.mgr.graphs[self.uiLayout._node.graphId].nodes[self.uiLayout._node.nodeId]:startFromUi()
       end, {
        label = self.uiLayout.startButtonText or "ui.scenarios.start.start",
        focus = true,
        main = true,
      }))

    if mission and mission:hasRules() then
      table.insert(self.uiLayout.buttons, self:addButton(function()
        mission:showRulesAsPopups()
      end, {
        label = "Rules",
        focus = false,
        main = false,
      }))
    end

    self:addReplayPanelAndSupportsReplay()
    self:setupPages()
    guihooks.trigger('ChangeState', {state = 'mission-control', params = { mode = 'startScreen'}})
  elseif self.uiLayout.mode == 'endScreen' then
    self:addReplayPanelAndSupportsReplay()
    self:setupPages()
    guihooks.trigger('ChangeState', {state = 'mission-control', params = { mode = 'endScreen'}})
  end

  log("I","","Done.")
  self.isBuilding = false
end

function C:setStartScreenAsSimple()
  self.uiLayout.simpleStartScreen = true
end

function C:addReplayPanelAndSupportsReplay()
  if self.mgr.activity and self.mgr.activity.supportsReplay and ((self.uiLayout.simpleStartScreen ~= nil and not self.uiLayout.simpleStartScreen) or self.uiLayout.simpleStartScreen == nil)then
    self:addUIElement({
      type = "replayPanel",
      header = "Replay Recording",
      recordingFiles = core_replay.getMissionReplayFiles(self.mgr.activity),
      pages = {
        recording = true
      }
    })
    self.uiLayout.supportsReplay = true
  end
end

function C:onRequestMissionScreenData(mode)
  if self.uiLayout and not string.find(mode, "test") then
    guihooks.trigger("onRequestMissionScreenDataReady", self.uiLayout )
    extensions.hook("onMissionStartScreenReady", self.uiLayout)
  end
end

function C:onGetIsMissionStartOrEndScreenActive(screensActive)
  if self.uiLayout then table.insert(screensActive, self.uiLayout.mode) end
end


-- Elements --
function C:onMissionScreenButtonClicked(button)
  if self.mgr.id ~= button.mgrId then
    return
  end
  self.buttonFunctions[button.funId]()
  self.uiLayout = nil
end



function C:addUIElement(element)
  if not self.isBuilding then
    log("E","","Tried to add ui element, but is currently not building! ")
    return
  end
  table.insert(self.uiLayout.layout, element)
end

function C:addHeader(header)
  if not self.isBuilding then
    log("E","","Tried to add header, but is currently not building! ")
    return
  end
  self.uiLayout.header = header
end

function C:addButton(fun, meta)
  if not self.isBuilding then
    log("E","","Tried to add button, but is currently not building! ")
    return
  end

  local idx = #self.buttonFunctions+1
  self.buttonFunctions[idx] = fun
  meta = meta or {}
  meta.mgrId = self.mgr.id
  meta.funId = idx
  return meta
end


function C:addObjectives(change)
  if not self.isBuilding then
    log("E","","Tried to add objectives, but is currently not building! ")
    return
  end
  local mission = self.mgr.activity
  if not mission then return end
  local unflattenedSettings = {}
  for k, v in pairs(mission.lastUserSettings) do
    table.insert(unflattenedSettings, {key = k, value = v})
  end

  local activeRewards = gameplay_missions_missionScreen.getActiveStarsForUserSettings(mission.id, unflattenedSettings)
  local stars = gameplay_missions_progress.formatStars(mission).stars or {}
  local anyVisible = false
  for _, star in ipairs(stars) do
    star.order = star.globalStarIndex
    if activeRewards and activeRewards.starInfo then
      star.enabled = activeRewards.starInfo[star.key].enabled
      star.message = activeRewards.starInfo[star.key].message
      star.visible = activeRewards.starInfo[star.key].visible
      star.label = activeRewards.starInfo[star.key].label
      anyVisible = star.visible
    end

    if change and change.unlockedStarsAttempt then
      star.unlockAttempt = change.unlockedStarsAttempt[star.key]
    end
    if change and change.unlockedStarsChanged then
      star.unlockChange = change.unlockedStarsChanged[star.key]
    end
    if change and change.starRewards and change.starRewards.originalRewardsPerStar and change.starRewards.originalRewardsPerStar[star.key] then
      star.rewards = change.starRewards.originalRewardsPerStar[star.key]
      for _, reward in ipairs(star.rewards) do
        reward.icon = career_branches.getBranchIcon(reward.attributeKey)
      end
    end
  end

  if anyVisible then
    self:addUIElement({
      type = "objectives",
      formattedProgress = {
        stars = stars,
        message = activeRewards.message,
      },
      pages = {
        main = true,
      }
    })

    -- Add element again for rewards page with only unlockAttempt stars
    local rewardsStars = {}
    for _, star in ipairs(stars) do
      if star.unlockAttempt then
        table.insert(rewardsStars, star)
      end
    end
    --[[
    if change and #rewardsStars > 0 and career_career.isActive() then
      self:addUIElement({
        type = "objectiveRewards",
        formattedProgress = {
          stars = rewardsStars,
          message = activeRewards.message,
        },
        pages = {
          rewards = true,
        }
      })
    end
    ]]--
  end
end

local formatMission = function(m)
  return {
    order = i,
    skill = {m.careerSetup.skill},
    id = m.id,
    icon = m.bigMapIcon.icon,
    label = m.name,
    description = m.description,
    formattedProgress =  gameplay_missions_progress.formatSaveDataForUi(m.id),
    startable = m.unlocks.startable,
    preview = m.previewFile,
    locked = not m.unlocks.visible,
    tier = m.unlocks.maxBranchlevel,
    thumbnailFile = m.thumbnailFile,
    difficulty = m.additionalAttributes.difficulty,
  }
end

function C:setStartButtonText(text)
  self.uiLayout.startButtonText = text
end

function C:addRatings(change)
  if not self.isBuilding then
    log("E","","Tried to add rating, but is currently not building! ")
    return
  end
  local mission = self.mgr.activity
  if not mission then return end
  local prog = {}

  local key = mission.currentProgressKey or "default"
  prog.leaderboardKey = mission.defaultLeaderboardKey or 'recent'
  prog.progressKey = key
  prog.leaderboardChangeKeys = gameplay_missions_progress.getLeaderboardChangeKeys(mission.id)
  local dnq = prog.leaderboardKey == 'highscore' and not (change and change.aggregateChange.newBestKeysByKey[prog.leaderboardChangeKeys['highscore']])
  if not change then dnq = nil end
  local formatted = gameplay_missions_progress.formatSaveDataForUi(mission.id, key, dnq)
  prog.formattedProgress = formatted.formattedProgressByKey[key]
  prog.progressKeyTranslations = formatted.progressKeyTranslations
  if dnq and change then
    -- fixed amount shown hack for dnq
    change.aggregateChange.newBestKeysByKey[prog.leaderboardChangeKeys['highscore']] = 6
  end

  -- pre-format aggregates for the UI. This formatting might be the default later and then be moved to gameplay_missions_progress
  local ownAggregate = {}
  for i, label in ipairs(prog.formattedProgress.ownAggregate.labels) do
    local agg = {
      label = label,
      value = prog.formattedProgress.ownAggregate.rows[1][i],
    }
    if change then
      local key = prog.formattedProgress.ownAggregate.newBestKeys[i]
      agg.newBest = change.aggregateChange.newBestKeysByKey[key]
    end
    table.insert(ownAggregate,agg)
  end
  prog.formattedProgress.ownAggregate = ownAggregate
  prog.formattedProgress.attempts.leaderboardIndex = change and change.aggregateChange.newBestKeysByKey[prog.leaderboardChangeKeys['highscore']] or -1

  if change then
    for _, league in ipairs(change.unlockedLeagues or {}) do
      for i, mId in ipairs(league.missions) do
        local m = gameplay_missions_missions.getMissionById(mId)
        league.missions[i] = formatMission(m)
      end
    end
    for _, elem in ipairs(change.unlockedMissions or {}) do
      local m = gameplay_missions_missions.getMissionById(elem.id)
      elem.formatted = formatMission(m)
      if m.startCondition.type == "league" then
        elem.hidden = true
      end
    end
  end

  self:addUIElement({
    type = "ratings",
    change = change,
    progress = prog,
    pages = {
      leaderboards = true,
    },
    --ownAggregate = ownAggregate
  })
  if change then
    if career_career.isActive() then
      self:addUIElement({
        type = "rewards",
        change = change,
        pages = {
          rewards = true,
        },
      })
    end
    if next(change.unlockedMissions or {}) or next(change.unlockedLeagues or {}) then
      self:addUIElement({
        type = "unlocks",
        change = change,
        pages = {
          unlocks = true,
        },
      })
    end
  end

end

function C:addCrashAnalysisStepDetails(stepDetails)
  local speedScoreDataRows = {}
  if stepDetails.stepScoreData.speedScore then
    table.insert(speedScoreDataRows, {
      { text = stepDetails.stepScoreData.speedScore.targetSpeed },
      { text = stepDetails.stepScoreData.speedScore.actualSpeed },
      { text = stepDetails.stepScoreData.speedScore.diff },
      { text = stepDetails.stepScoreData.speedScore.score },
    })
  end

  self:addUIElement({
    type = "crashTestStepDetails",
    header = translateLanguage("missions.crashTest.general.stagePoints", "missions.crashTest.general.stagePoints", true),
    stepScoreData = stepDetails.stepScoreData,
    fullHeight = true,
    pages = {
      main = true
    },
  })
end

function C:addDriftStats(driftStats)
  -- performance stats table
  local perfStatsRows = {}
  for _, statData in pairs(driftStats.perfStats) do
    table.insert(perfStatsRows, {
      { text = statData.name },
      { text = statData.value },
      order = statData.order
    })
  end
  local perfStatsGrid = {
    labels = { '', '', },
    rows = perfStatsRows,
    leaderboardIndex = 0
  }
  table.sort(perfStatsGrid.rows, function(a, b) return a.order < b.order end)
  for _, statData in ipairs(perfStatsGrid.rows) do
    statData.order = nil
  end

  -- tier stats table
  local tierRows = {}
  local totalTiersScore = 0
  for _, statData in pairs(driftStats.tiersStats) do
    totalTiersScore = totalTiersScore + statData.totalScore
    table.insert(tierRows, {
      { text = statData.name },
      { text = statData.count },
      { text = statData.totalScore },
      order = statData.order
    })
  end
  table.insert(tierRows, {
    { text = "Total :", styling = {totalRow = true} },
    { text = "" },
    { text = tostring(totalTiersScore) .. " points", styling = {totalRow = true} },
    order = 1000
  })
  local tierGrid = {
    labels = {"", "Count", "Total Score"},
    rows = tierRows,
    leaderboardIndex = 0
  }
  table.sort(tierGrid.rows, function(a, b) return a.order < b.order end)
  for _, statData in ipairs(tierGrid.rows) do
    statData.order = nil
  end


  -- drift events table
  local driftEventsRows = {}
  local totalDriftEventsScore = 0
  for _, eventData in pairs(driftStats.driftEvents) do
    totalDriftEventsScore = totalDriftEventsScore + eventData.totalScoreEarned
    table.insert(driftEventsRows, {
      { text = eventData.msg },
      { text = eventData.count },
      { text = eventData.totalScoreEarned },
    })
  end
  table.insert(driftEventsRows, {
    { text = "Total :", styling = {totalRow = true} },
    { text = "" },
    { text = tostring(totalDriftEventsScore) .. " points",  styling = {totalRow = true}  },
  })
  local driftEventsGrid = {
    labels = { "", "Count", "Total score" },
    rows = driftEventsRows,
    leaderboardIndex = 0
  }

  self:addUIElement({
    type = "textPanel",
    header = translateLanguage("missions.drift.general.perfStats", "missions.drift.general.perfStats", true),
    attempt = {
      grids = { perfStatsGrid }
    },
    fullHeight = true,
    pages = {
      drift = true
    }
  })

  self:addUIElement({
    type = "textPanel",
    header = translateLanguage("missions.drift.general.tiers", "missions.drift.general.tiers", true),
    attempt = {
      grids = { tierGrid }
    },
    fullHeight = true,
    pages = {
      drift = true
    }
  })

  self:addUIElement({
    type = "textPanel",
    header = translateLanguage("missions.drift.general.events", "missions.drift.general.events", true),
    attempt = {
      grids = { driftEventsGrid }
    },
    fullHeight = true,
    pages = {
      drift = true
    }
  })
end

function C:addLaptimesForVehicle(state, attempt)
  -- Find the best lap time
  local bestLapTime = math.huge
  local bestLapIndex = -1
  for i, lap in ipairs(state.historicTimes) do
    if lap.lapTime < bestLapTime then
      bestLapTime = lap.lapTime
      bestLapIndex = i
    end
  end

  -- Generate rows with "Best" for the best lap and time difference for others
  local rows = {}
  for i, lap in ipairs(state.historicTimes) do
    local isBest = lap.lapTime == bestLapTime
    local differenceText = isBest and "Best" or string.format("+%d:%02d:%03d",
      math.floor((lap.lapTime - bestLapTime) / 60),
      (lap.lapTime - bestLapTime) % 60,
      1000 * ((lap.lapTime - bestLapTime) % 1)
    )
    table.insert(rows, {
      { text = i },
      { format = "detailledTime", detailledTime = lap.lapTime, text = string.format("%d:%02d:%03d",
        math.floor(lap.lapTime / 60),
        lap.lapTime % 60,
        1000 * (lap.lapTime % 1)
      )},
      { text = differenceText }
    })
  end

  -- Create the grid and UI element
  local grid = {
    labels = { 'Lap', 'Time', '' },
    rows = rows,
    leaderboardIndex = bestLapIndex
  }
  self:addUIElement({
    type = "textPanel",
    header = "Lap Times",
    attempt = {
      grids = { grid }
    },
    pages = {
      lapTimes = true,
    },
  })
end

-- will probably only be used by startPage
function C:nextPage()
  self.pageCounter = self.pageCounter + 1
  table.insert(self.uiLayout.layout,self.pageCounter,{})
end

function C:executionStarted()
  self.serializedRecoveryPromptState = core_recoveryPrompt.serializeState()
  self.gameStateBeginning = deepcopy(core_gamestate.state)
  self.recoverGameStateWhenExecutionStopped = true
  self.genericMissionDataChanged = false
  self.pointsBarChanged = false
  self.appContainerContext = nil
  core_recoveryPrompt.setActive(false)
  extensions.load('ui_apps_genericMissionData')
  extensions.load('ui_apps_pointsBar')
  if self.mgr.activity then
    guihooks.trigger('ClearTasklist')
  end
end

function C:executionStopped()
  if self.genericMissionDataChanged then
    ui_apps_genericMissionData.clearData()
  end
  if self.pointsBarChanged then
    ui_apps_pointsBar.clearData()
  end
  if self.appContainerContext then
    ui_gameplayAppContainers.resetContainerContext('gameplayApps')
  end

  if self.serializedRecoveryPromptState then
    core_recoveryPrompt.deserializeState(self.serializedRecoveryPromptState)
  end
  self.serializedRecoveryPromptState = nil

  if self.recoverGameStateWhenExecutionStopped and self.gameStateBeginning then
    core_gamestate.setGameState(self.gameStateBeginning.state, self.gameStateBeginning.appLayout, self.gameStateBeginning.menuItems, self.gameStateBeginning.options)
  end
  self.recoverGameStateWhenExecutionStopped = nil
  self.gameStateBeginning = nil

  if self.mgr.activity then
    guihooks.trigger('ClearTasklist')
  end
end




return _flowgraph_createModule(C)