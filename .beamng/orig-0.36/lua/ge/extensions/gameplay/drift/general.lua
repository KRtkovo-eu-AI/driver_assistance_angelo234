-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
local im = ui_imgui
local imVec4Red = im.ImVec4(1,0.1,0,1)

M.dependencies = {"gameplay_drift_drift", "gameplay_drift_scoring", "gameplay_drift_statistics", "gameplay_drift_saveLoad", }

local loadedExtensions = {}

local driftDebugInfo = {
  default = true,
  canBeChanged = false
}

-- decides when an extension is loaded/unloaded
local variableExtensions = {
  gameplay_drift_stallingSystem = {
    challengeModes = {"Gymkhana"}
  },
  gameplay_drift_destination = {
    challengeModes = {"A to B", "A to B with stunt zones"},
    contexts = {"inFreeroamChallenge"}
  },
  gameplay_drift_stuntZones = {
    contexts = {"inChallenge"}
  },
  gameplay_drift_display = {
    contexts = {"inChallenge", "inFreeroamChallenge", "inFreeroam"}
  },
  gameplay_drift_quickMessages = {
    contexts = {"inFreeroam", "inChallenge", "inFreeroamChallenge"}
  },
  gameplay_drift_bounds = {
    contexts = {"inChallenge", "inFreeroamChallenge"}
  },
  gameplay_drift_freeroam_driftSpots = {
    contexts = {"inFreeroam", "inFreeroamChallenge"},
  },
  gameplay_drift_scoreboard = {
    contexts = {"inChallenge"},
  },
  gameplay_drift_freeroam_cruising = {
    contexts = {"inFreeroam"},
  },
  gameplay_drift_sounds = {manualLoad = true}
}

local debug = false
local context -- contexts are : "inChallenge" "inFreeroam" "inFreeroamChallenge" "inAnotherMissionType"
local frozen = false -- used to freeze the scoring, drift detection. For exemple when the player goes out of bounds or wrong way
local challengeMode -- challengeModes are : "A to B" "A to B with stunt zones" "Gymkhana" "None"
local paused = false -- used to "pause" the drift systems, used during the end screen of formal mission challenges
local mainDriftUIAppLoaded = false

local contextList = {"inFreeroam", "inChallenge", "inFreeroamChallenge", "inAnotherMissionType"}
local challengeModeList = {"None", "A to B", "A to B with stunt zones", "Gymkhana"}

local firstUpdateFlag = false

local driftExtensions = {}

local function commonReset()
  for extensionName, _ in pairs(loadedExtensions) do
    if extensionName ~= "gameplay_drift_general" and extensions.isExtensionLoaded(extensionName) and _G[extensionName].reset then
      _G[extensionName].reset()
    end
  end
  frozen = false
end

local function clear()
  commonReset()
  if gameplay_drift_stuntZones then gameplay_drift_stuntZones.clear() end
end

local function reset()
  commonReset()
  if gameplay_drift_stuntZones then gameplay_drift_stuntZones.reset() end
end

-- make a list of which extensions are loaded or unloaded
local function checkLoadedExtensions()
  for _, filePath in ipairs(driftExtensions) do
    local extensionName = string.match(extensions.luaPathToExtName(filePath), "extensions_([%w_]+)%.lua")
    local ext = require(string.match(filePath, "(.+)%.lua$"))
    local driftDebugInfo = {
      canBeChanged = false,
      default = false
    }
    if type(ext) == "table" then
      if ext.getDriftDebugInfo then
        driftDebugInfo = ext:getDriftDebugInfo()
      end
      loadedExtensions[extensionName] = {
        loaded = extensions.isExtensionLoaded(extensionName),
        driftDebugInfo = driftDebugInfo
      }
    end
  end
end

local function updateExtensions()
  for extensionName, data in pairs(variableExtensions) do
    if not data.manualLoad then
      local foundMatch = false
      if data.challengeModes then
        for _, challengeMode_ in ipairs(data.challengeModes) do
          if challengeMode_ == challengeMode then
            extensions.load(extensionName)
            foundMatch = true
            break
          end
        end
      end
      if data.contexts then
        for _, context_ in ipairs(data.contexts) do
          if context_ == context then
            extensions.load(extensionName)
            foundMatch = true
            break
          end
        end
      end

      if not foundMatch then
        extensions.unload(extensionName)
      end

      checkLoadedExtensions()
    end
  end
end

local function setChallengeMode(newChallengeMode)
  if newChallengeMode == challengeMode then return end

  challengeMode = newChallengeMode

  updateExtensions()
end

local function setPaused(newPaused)
  paused = newPaused
end

local function setContext(newContext)
  if newContext == context then return end

  context = newContext
  extensions.hook("onDriftContextChanged", context)

  if context == "inFreeroam" then
    setChallengeMode("None")
  end

  updateExtensions()
end

local function setDebug(value)
  debug = value
  if debug then
    checkLoadedExtensions()
  end
  extensions.hook("onDriftDebugChanged", value)
end

local function getGeneralDebug()
  return debug
end

local function getPaused()
  return paused
end

local function getContext()
  return context
end

local function getDriftDebugInfo()
  return driftDebugInfo
end

local function getExtensionDebug(extName)
  if not debug or not loadedExtensions[extName] then return false end
  return loadedExtensions[extName].driftDebugInfo.default
end

local function getFrozen()
  return frozen
end

local function getChallengeMode()
  return challengeMode
end

local function onAnyMissionChanged(status, mission)
  clear()
  if status == "started" then
    if mission.missionType ~= "drift" then
      setContext("inAnotherMissionType")
    end
  elseif status == "stopped" then
    paused = false
    frozen = false
    setContext("inFreeroam")
  end
end

local function onVehicleResetted(vid)
  if vid == be:getPlayerVehicleID(0) then
    extensions.hook("onDriftPlVehReset")
  end
end

local function imguiDebug()
  if gameplay_drift_general.getGeneralDebug() then

    if im.Begin("Drift general") then
      im.PushStyleColor2(im.Col_Text, imVec4Red)
      if im.Button("Exit debug") then setDebug(false) end
      im.PopStyleColor()
      im.SameLine()
      if im.Button("Reset drift") then reset() end

      im.Dummy(im.ImVec2(1, 7))

      im.Text("Drift context : ")

      local s = ""
      for _, n in ipairs(contextList) do
        s = s .. tostring(n) .. "\0"
      end

      local presetPtr = im.IntPtr((tableFindKey(contextList, context)) - 1)
      if im.Combo2("", presetPtr, s) then
        setContext(contextList[presetPtr[0]+1])
      end

      im.Dummy(im.ImVec2(1, 10))

      if context == "inChallenge" then
        im.Text("Challenge mode : ")

        s = ""
        for _, n in ipairs(challengeModeList) do
          s = s .. tostring(n) .. "\0"
        end

        presetPtr = im.IntPtr((tableFindKey(challengeModeList, challengeMode)) - 1)
        if im.Combo2("##"..'t', presetPtr, s) then
          setChallengeMode(challengeModeList[presetPtr[0]+1])
        end
        im.Dummy(im.ImVec2(1, 10))
      end
    end

    im.Separator()

    im.Text("Loaded drift extensions : ")
    if im.BeginTable("Loaded extensions", 3, nil) then
      im.TableSetupColumn("Extension name",nil, 10)
      im.TableSetupColumn("Debug",nil, 4)
      im.TableSetupColumn("Gc",nil, 4)
      im.TableNextColumn()
      im.Text("Extension name")
      im.TableNextColumn()
      im.Text("Debug")
      im.TableNextColumn()
      im.Text("Gc")
      im.TableNextColumn()

      for extensionName, extensionData in pairs(loadedExtensions) do
        if extensionData.loaded then
          im.Text(string.gsub(extensionName, "^gameplay_drift_", ""))
          im.TableNextColumn()
          if not extensionData.driftDebugInfo.canBeChanged then
            im.BeginDisabled()
          end
          local boolPtr = im.BoolPtr(extensionData.driftDebugInfo.default)
          if im.Checkbox("##"..extensionName, boolPtr) then
            extensionData.driftDebugInfo.default = boolPtr[0]
          end
          if not extensionData.driftDebugInfo.canBeChanged then
            im.EndDisabled()
          end

          im.TableNextColumn()

          if _G[extensionName] and _G[extensionName].getGC then
            im.Text(tostring(_G[extensionName].getGC()))
          else
            im.Text("nan")
          end
          im.TableNextColumn()

        end
      end
      im.EndTable()
    end

    im.Separator()

    im.Text("Paused : " .. tostring(paused))
    im.Text("Frozen : " .. tostring(frozen))
    im.Text("Drift UI loaded : " .. tostring(mainDriftUIAppLoaded))
  end
end

local function init()
  setContext("inFreeroam")
  reset()
end

local function checkFrozen()
  local outOfBounds = false
  local goingWrongWay = false
  local isInTheConculdingPhase = false

  if gameplay_drift_destination then
    goingWrongWay = gameplay_drift_destination.getGoingWrongWay()
  end
  if gameplay_drift_bounds then
    outOfBounds = gameplay_drift_bounds.getIsOutOfBounds()
  end

  if gameplay_drift_freeroam_driftSpots then
    isInTheConculdingPhase = gameplay_drift_freeroam_driftSpots.getIsInTheConcludingPhase()
  end
  frozen = outOfBounds or goingWrongWay or isInTheConculdingPhase
end

local function onUpdate()
  imguiDebug()

  if not firstUpdateFlag then
    init()
    firstUpdateFlag = true
  end

  checkFrozen()
end

local function onSerialize()
  return {
    debug = debug,
  }
end

local function onDeserialized(data)
  debug = data.debug
end

local function onExtensionLoaded()
  driftExtensions = FS:findFiles("/lua/ge/extensions/gameplay/drift", '*.lua', -1, false, false)
end

local function onMainUIAppMounted()
  extensions.load("gameplay_drift_sounds")
  mainDriftUIAppLoaded = true
  checkLoadedExtensions()
end

local function onMainUIAppUnmounted()
  extensions.unload("gameplay_drift_sounds")
  mainDriftUIAppLoaded = false
  checkLoadedExtensions()
end

local function getMainDriftUIAppLoaded()
  return mainDriftUIAppLoaded
end

M.reset = reset
M.clear = clear

M.setChallengeMode = setChallengeMode
M.getChallengeMode = getChallengeMode

M.getExtensionDebug = getExtensionDebug
M.getGeneralDebug = getGeneralDebug
M.getContext = getContext
M.getFrozen = getFrozen
M.getPaused = getPaused
M.getDriftDebugInfo = getDriftDebugInfo
M.getMainDriftUIAppLoaded = getMainDriftUIAppLoaded

M.setDebug = setDebug
M.setContext = setContext
M.setPaused = setPaused

M.onVehicleResetted = onVehicleResetted
M.onAnyMissionChanged = onAnyMissionChanged
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized
M.onUpdate = onUpdate
M.onExtensionLoaded = onExtensionLoaded

M.onMainUIAppMounted = onMainUIAppMounted
M.onMainUIAppUnmounted = onMainUIAppUnmounted
return M