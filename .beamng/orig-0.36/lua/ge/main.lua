-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--- you can use this to turn of Just In Time compilation for debugging purposes:
--jit.off()
if jit then	jit.opt.start('minstitch=10000000') end

vmType = 'game'

package.path = 'lua/ge/?.lua;lua/gui/?.lua;lua/common/?.lua;lua/common/libs/?/init.lua;lua/common/libs/luasocket/?.lua;lua/?.lua;core/scripts/?.lua;scripts/?.lua;?.lua'
package.cpath = ''

require('luaCore')
require('common/cdefs')

--log replacement to trace long log lines:
--[[
log = function(a, b, c, ...)
  Lua:log(a, b, c, ...)
  if (type(a) == "string" and string.len(a) > 1000) or
  (type(b) == "string" and string.len(b) > 1000) or
  (type(c) == "string" and string.len(c) > 1000) ) then
    log("W", b, "The long log message above has been triggered by:")
    print(debug.tracesimple())
  end
end
--]]

log = function(...)
  Lua:log(...)
end

print = function(...)
  log("A", "print", tostring(...))
  -- log('A', "print", debug.traceback()) -- find where print is used
end
log("I", "", "============== GELUA VM loading ===============")


require('mathlib')
Point3F = vec3
require("utils")
require("devUtils")
require("ge_utils")
require("luaProfiler")
local STP = require "libs/StackTracePlus/StackTracePlus"
debug.traceback = STP.stacktrace
debug.tracesimple = STP.stacktraceSimple

json = require("json")
guihooks = require("guihooks")
screenshot = require("screenshot")
simTimeAuthority = require("simTimeAuthority")
bullettime = simTimeAuthority -- retrocompatibility
extensions = require("extensions")
extensions.addModulePath("lua/ge/extensions/")
extensions.addModulePath("lua/common/extensions/")
map = require("map")
settings = extensions.core_settings_settings
perf = require("utils/perf")
spawn = require("spawn")
setSpawnpoint= require ("setSpawnpoint")
serverConnection = require("serverConnection")
server = require("server/server")
commands = require("server/commands")
editor = {}

worldReadyState = -1 -- tracks if the level loading is done yet: 0 = no, 1 = yes, load play ui, 2 = all done
parseArgs = require("client/parseArgs")

sailingTheHighSeas = the_high_sea_crap_detector()
photoModeOpen = false
levelLoaded = nil
MoveManager = {}

-- global imgui callbacks, add here any callback exposed from any extension
ConsoleInputCallback = function() end

gameConnection = nil -- backward compatibility, not a sceneobject anymore
function getGame() return gameConnection end -- backward compatibility

-- how to log into a json file:
--globalJsonLog = LogSink()
--globalJsonLog:open('/gamelog.json')

--[[
-- function to trace the memory usage
local maxMemUsage = 0
local function trace_mem(event, line)
  local s = debug.getinfo(2)
  local m, _ = gcinfo()
  if m > maxMemUsage then
    maxMemUsage = m
  end
  Lua:log('D', 'luaperf', tostring(event) .. ' = ' .. tostring(s.what) .. '_' .. tostring(s.source) .. ':' .. tostring(s.linedefined) .. ' / memory usage: ' .. tostring(m) .. ' (max: ' .. tostring(maxMemUsage) .. ')')
end
debug.sethook(trace_mem, "c")
--]]


local ffi = require("ffi")

math.randomseed(os.time())
local cmdArgs = Engine.getStartingArgs()

doStartupProfiling = tableFindKey(cmdArgs, '-startupProfiling')
if not shipping_build and doStartupProfiling and simpleProfilerStart and profilerPushEvent and profilerPopEvent then
  simpleProfilerStart(false) -- debug?
end

-- optimization for shipping builds: disable profiling if -profiling is not set
local profilingEnabled = tableFindKey(cmdArgs, '-profiling')
if shipping_build and not profilingEnabled then
  profilerPushEvent = nop
  profilerPopEvent = nop
end

--Lua:enableStackTraceFile("lua.ge.stack.txt", true)

logAlways=print

local _isSafeMode = tableFindKey(cmdArgs, '-safemode')
function isSafeMode()
  return _isSafeMode
end

function convertPrefabtoJson(filepath)
  log('I', 'convertPrefabtoJson', 'Converting cs prefab: ' .. tostring(filepath) )
  local dir, filename, ext = path.splitWithoutExt(filepath, ".prefab")
  log('I', 'convertPrefabtoJson', '  spawning prefab as = ' .. tostring(filename) )
  local csPrefab = spawnPrefab(filename, filepath, '0 0 0', '0 0 1 0', '1 1 1')
  if csPrefab then
    local newPath = dir..filename..".prefab.json"
    csPrefab:save(newPath, false)
    csPrefab:unload()
    csPrefab:delete()
  else
    log('E','', '      Could not find prefab.')
  end
end

function convertLevelPrefabs(levelPath)
  local filenames = FS:findFiles(levelPath, "*.prefab\t*.prefab.json", -1, true, false)
  log('I', 'convertLevelPrefabs', 'Converting the following files: '..dumps(filenames or {}))
  for _,file in ipairs(filenames) do
    convertPrefabtoJson(file)
  end
end

function validatePrefabJson(filepath)
  log('I', 'validatePrefabJson', 'Validating json prefab: ' .. tostring(filepath) )
  local dir, filename, ext = path.splitWithoutExt(filepath, ".prefab.json")
  log('I', 'validatePrefabJson', '  Validating prefab as = ' .. tostring(filename) )
  local prefab = spawnPrefab(filename, filepath, '0 0 0', '0 0 1', '1 1 1')
  if prefab then
    local newPath = dir..filename..".prefab.json"
    prefab:save(filepath, false)
    prefab:unload()
    prefab:delete()
  else
    log('E','', '      Could not find prefab: '..tostring(filepath))
  end
end

function validateLevelPrefabs(levelPath)
  local filenames = FS:findFiles(levelPath, "*.prefab.json", -1, true, false)
  log('I', 'validateLevelPrefabs', 'Validating the following files: '..dumps(filenames or {}))
  for _,file in ipairs(filenames) do
    validatePrefabJson(file)
  end
end

function convertCStoJson(filenames, newExtSuffix)
  log('I', 'convertCStoJson', 'Conversion started: '..dumps(filenames or {}))

  local persistenceMgr = PersistenceManager()
  persistenceMgr:registerObject('convert_PersistManager')

  for _, fn in pairs(filenames or {}) do
    log('I', 'resaveCSFiles', 'converting ts script: ' .. tostring(fn) )
    -- record known things
    local knownObjects = scenetree.getAllObjects()
    log('I', 'convertCStoJson', 'knownObjects: '..dumps(knownObjects))
    -- convert to map
    local newKnownObjects = {}
    for k, v in pairs(knownObjects) do
      newKnownObjects[v] = 1
    end
    knownObjects = newKnownObjects

    -- load the file
    TorqueScriptLua.exec(fn)

    -- figure out what objects were loaded from that file by diffing with the known objects above
    local knownObjects2 = scenetree.getAllObjects()
    local newObjects = {}
    for _, oName in pairs(knownObjects2) do
      if not knownObjects[oName] then
        local obj = scenetree.findObject(oName)
        log('I', '', ' adding :  ' .. tostring(oName))
        if obj then
          newObjects[oName] = obj
        end
      end
    end
    log('I', 'convertCStoJson', 'newObjects: '..dumps(newObjects))

    for _, obj in pairs(newObjects) do
      log('I', '', ' * ' .. tostring(obj:getClassName()) .. ' - ' .. tostring(obj:getName()) )
      persistenceMgr:setDirty(obj, '')
    end
    persistenceMgr:saveDirtyNewFormat()

    for _, obj in pairs(newObjects) do
      local className = obj:getClassName()
      if not className or (className and className ~= "SimSet" and className ~= "SimGroup") then
        log('I', '', ' *Deleting:  ' .. tostring(className) .. ' - ' .. tostring(obj:getName()) )
        obj:delete()
      end
    end
  end
  persistenceMgr:delete()
  log('I', 'convertCStoJson', 'Conversion all done.')
end

-- immediate command line arguments
-- worked off before anything else
-- called when the world is init'ed
local function handleCommandLineFirstFrame()
  if tableFindKey(cmdArgs, '-flowgraph') then
    if getMissionFilename() == "" then
      freeroam_freeroam.startFreeroam(path.getPathLevelMain("smallgrid"))
      extensions.load('editor_flowgraphEditor')
      editor_flowgraphEditor.requestedEditor = true
      --core_levels.startLevel(path.getPathLevelMain("smallgrid"))
    end
  end

  if tableFindKey(cmdArgs, '-compilemeshes') then
    extensions.load('util_compileMeshes')
  end

  if tableFindKey(cmdArgs, '-LoadSingleLevelForTextureCache') then
    --Using existing extension to run level for 600 frames. Use with param: -testlevel [level name]
    extensions.load('test_singleLevel')
  end
  if tableFindKey(cmdArgs, '-LoadSingleLevelForShaderCache') then
    --Same but waiting for 2000 frames and 35sec(which happen latest). Usage: -testlevel [level name] -testvehicle [vehicle_name] -testsettings [path to json]
    extensions.load('test_singleLevelLong')
  end
  if tableFindKey(cmdArgs, '-LoadTrackBuilder') then
	extensions.load('test_runTrackBuilder')
  end
  if tableFindKey(cmdArgs, '-LoadLightRunner') then
	extensions.load('test_runLightRunner')
  end

  if tableFindKey(cmdArgs, '-convertCSMaterials') then
    local function resaveCSFiles(pattern, fnSuffix)
      local persistenceMgr = PersistenceManager()
      persistenceMgr:registerObject('matConvert_PersistManager')

      local filenames = FS:findFiles('/', pattern, -1, true, false)
      -- dump(filenames)
      for _, fn in pairs(filenames) do
        log('I', 'resaveCSFiles', 'converting ts script: ' .. tostring(fn) )
        -- record known things
        local knownObjects = scenetree.getAllObjects()
        -- convert to map
        local newKnownObjects = {}
        for k, v in pairs(knownObjects) do
          newKnownObjects[v] = 1
        end
        knownObjects = newKnownObjects

        -- load the file
        TorqueScriptLua.exec(fn)

        -- figure out what objects were loaded from that file by diffing with the known objects above
        local knownObjects2 = scenetree.getAllObjects()
        local newObjects = {}
        for _, oName in pairs(knownObjects2) do
          if not knownObjects[oName] then
            local obj = scenetree.findObject(oName)
            log('I', '', ' adding :  ' .. tostring(oName))
            if obj then
              newObjects[oName] = obj
            end
          end
        end

        for _, obj in pairs(newObjects) do
          log('I', '', ' * ' .. tostring(obj:getClassName()) .. ' - ' .. tostring(obj:getName()) )
          persistenceMgr:setDirty(obj, '')
        end
        persistenceMgr:saveDirtyNewFormat()

        for _, obj in pairs(newObjects) do
          local className = obj:getClassName()
          if not className or (className and className ~= "SimSet" and className ~= "SimGroup") then
            log('I', '', ' *Deleting:  ' .. tostring(className) .. ' - ' .. tostring(obj:getName()) )
            obj:delete()
          end
        end
      end
      persistenceMgr:delete()
    end

    resaveCSFiles('materials.cs', '.materials.json')
    resaveCSFiles('*Data.cs', '.datablock.json')
    log('I', 'convertCSMaterials', 'All done, exiting gracefully.')
    shutdown(0)
  end

  if tableFindKey(cmdArgs, '-deps') then
    extensions.util_dependencyTree.test()
    shutdown(0)
  end

  if tableFindKey(cmdArgs, '-disableDynamicCollision') then
    settings.setValue('disableDynamicCollision', true)
  end
end

-- Extensions necessary for game startup procedure:
-- (DO NOT ADD MORE EXTENSIONS TO THIS LIST unless it's required by game startup procedure)
-- (if you have already put your extension on this list, and it's not critical for game startup procedure, please remove it by following the instructions below)
local startupExtensions = {
  'core_audio', 'core_camera', 'core_commandhandler', 'core_flowgraphManager', 'core_gamestate', 'core_hardwareinfo',
  'core_highscores', 'core_input_actionFilter', 'core_input_actions', 'core_input_bindings', 'core_input_categories',
  'core_input_deprecatedActions', 'core_input_vehicleSwitching', 'core_input_virtualInput', 'core_inventory',
  'core_jobsystem', 'core_levels', 'core_modmanager', 'core_multiseat', 'core_multiseatCamera', 'core_online',
  'core_paths', 'core_remoteController', 'core_replay', 'core_settings_audio', 'core_settings_graphic',
  'core_settings_settings', 'core_sounds', 'core_vehicle_colors', 'core_vehicle_manager', 'core_vehicles', 'ui_imgui',
  'ui_apps', 'ui_audio', 'ui_flowgraph_editor', 'ui_visibility', 'ui_gameplayAppContainers', 'campaign_campaignsLoader', 'career_branches',
  'career_career', 'career_saveSystem', 'freeroam_freeroam', 'gameplay_garageMode',
  'gameplay_missions_missions', 'gameplay_missions_progress', 'gameplay_missions_unlocks', 'gameplay_missions_missionScreen',
  'gameplay_statistic', 'render_hdr', 'scenario_quickRaceLoader', 'scenario_scenariosLoader', 'core_windowsConsole',
  'core_devices'
  -- DO NOT ADD MORE EXTENSIONS TO THIS LIST unless it's required by game startup procedure. Instead, try the following:
  --   To load an extension on demand:               extensions.load("my_extension")
  --   To keep an extension loaded across levels:    M.onInit = function() setExtensionUnloadMode(M, "manual") end
  --   To unload an extension when no longer needed: M.myFunction = function() extensions.unload(M) end
  --   If you have already put your extension on this list, and it's not critical for game startup procedure, please remove it by following the instructions above.
}

-- Extensions that get loaded by various game modes (freeroam, etc) and in other circumstances too:
local presetExtensions = {
  'core_checkpoints', 'core_environment', 'core_forest', 'core_gameContext',
  'core_groundMarkers', 'core_multiSpawn', 'core_quickAccess', 'core_funstuff', 'core_recoveryPrompt', 'core_terrain',
  'core_trafficSignals', 'core_trailerRespawn', 'core_vehicleBridge', 'core_vehicle_mirror', 'core_weather',
  'freeroam_bigMapMode', 'freeroam_bigMapPoiProvider', 'freeroam_facilities', 'freeroam_facilities_fuelPrice',
  'freeroam_gasStations', 'freeroam_specialTriggers', 'freeroam_organizations', 'gameplay_city', 'gameplay_markerInteraction',
  'gameplay_missions_missionManager', 'gameplay_missions_startTrigger', 'gameplay_parking', 'gameplay_rawPois',
  'gameplay_traffic', 'gameplay_walk', 'trackbuilder_trackBuilder', 'ui_fadeScreen', 'ui_apps_genericMissionData', 'ui_missionInfo', 'util_richPresence',
  'freeroam_crashCamModeLoader', 'gameplay_speedTraps', 'gameplay_speedTrapLeaderboards', 'gameplay_util_crashDetection',
  'gameplay_util_groundContact', 'gameplay_drift_general',
  'gameplay_drag_freeroamDragStrip', 'gameplay_achievement',  'gameplay_discover'
}

local cmdlineLevelLoadExtensions = {} -- extensions indicated from command line arguments
local manualUnloadExtensions = startupExtensions -- by default, we want all startupExtensions to be manually unloaded (we want them to persist across level loads)

local editorExtensions = {
  'editor_main', 'editor_veMain'
}

if not PlatformSwitches.useEditor then
  editor.isEditorActive = function() return false end
  editor.toggleActive = function(safeMode) end
end

function prepareStartupExtensionsList()
  if PlatformSwitches.useEditor then
    for _,v in ipairs(editorExtensions) do
      if not tableContains(startupExtensions, v) then
        table.insert(startupExtensions, v)
      end
    end
  end
  manualUnloadExtensions = startupExtensions
end

-- load extensions with unloadMode = "manual"
function loadManualUnloadExtensions()
  extensions.load(manualUnloadExtensions)
end

-- unload extensions with unloadMode = "auto"
function unloadAutoExtensions()
  extensions.unloadExcept(manualUnloadExtensions)
end

-- load extensions from the preset list
function loadPresetExtensions()
  extensions.load(presetExtensions, cmdlineLevelLoadExtensions)

  extensions.load(cmdlineLevelLoadExtensions) -- if '-onLevelLoad_ext' extensions were requested via command line arguments, we load those too
  table.clear(cmdlineLevelLoadExtensions) -- note from bruno: i don't know why the cmdline extensions list is one-use-only... but that's how this code has behaved for a long time
end

-- extensions' unloadMode is "auto" by default
--  - "auto": they may get unloaded when switching maps, loading a scenario, and other circumstances
--  - "manual": they will not get unloaded, instead the extension lifetime is controlled explicitly (manually)
function setExtensionUnloadMode(extension, unloadMode)
  local extName = extension
  if type(extension) == 'table' then
    extName = extension.__extensionName__
  end

  if type(extName) ~= 'string' then
    log('E','','Failed to set unload mode "'..dumps(unloadMode)..'" due to unrecognized extension: '..dumps(extension))
    return
  end

  if unloadMode == 'manual' then
    for _, v in ipairs(manualUnloadExtensions) do
      if v == extName then
        return
      end
    end
    table.insert(manualUnloadExtensions, extName)
  elseif unloadMode == 'auto' then
    for i, v in ipairs(manualUnloadExtensions) do
      if v == extName then
        table.remove(manualUnloadExtensions, i)
        return
     end
    end
  else
    log('E','','Failed to set unrecognised unload mode "'..dumps(unloadMode)..'" for extension: '..dumps(extension))
    return
  end
end

function endActiveGameMode(callback)
  local endCallback = function ()
    extensions.unloadExcept(manualUnloadExtensions)

    if type(callback) == 'function' then
      callback()
    end
  end
  -- NOTE: We have to use a callback to serverConnection.disconnect because is it updated in a
  --       State machine
  serverConnection.disconnect(endCallback)
end

function queueCmdlineLevelLoadExtension(extension)
  table.insert(cmdlineLevelLoadExtensions, extension)
end

-- called before the Mission Resources are loaded
function clientPreStartMission(levelPath)
  worldReadyState = 0
  extensions.hook('onClientPreStartMission', levelPath)
  guihooks.trigger('PreStartMission')
  core_levels.maybeLoadDefaultVehicle()
end

-- called when level, car etc. are completely loaded (after clientPreStartMission)
function clientPostStartMission(levelPath)
  --default game state, will get overriden by each mode
  core_gamestate.setGameState('freeroam', 'freeroam', 'freeroam')
  extensions.hook('onClientPostStartMission', levelPath)
end

-- called when the level items are already loaded (after clientPostStartMission)
function clientStartMission(levelPath)
  log("D", "clientStartMission", "starting levelPath: " .. tostring(levelPath))
  extensions.hookNotify('onClientStartMission', levelPath)
  map.assureLoad() --> needs to be after extensions.hook('onClientStartMission', levelPath)
  guihooks.trigger('MenuHide')
 -- SteamLicensePlateVehicleId = nil
end

function clientEndMission(levelPath)
  -- core_gamestate.requestGameState()
  -- log("D", "clientEndMission", "ending levelPath: " .. tostring(levelPath))
  be:physicsStopSimulation()
  simTimeAuthority.pause(false)
  extensions.hookNotify('onClientEndMission', levelPath)
end

function returnToMainMenu()
  endActiveGameMode()
end

function editorEnabled(enabled)
  --print('editorEnabled', enabled)
  extensions.hook('onEditorEnabled', enabled)
  map.setEditorState(enabled)
end

local luaPreRenderMaterialCheckDuration = 0

-- called from c++ side whenever a performance check log is wanted
local geluaProfiler
function requestGeluaProfile()
  geluaProfiler = LuaProfiler("update() and luaPreRender() gelua function calls")
  extensions.setProfiler(geluaProfiler)
end
-- this function is called right before the rendering, and after running the physics
function luaPreRender(dtReal, dtSim, dtRaw)
  if geluaProfiler then geluaProfiler:start() end
  map.updateGFX(dtReal, dtSim)
  if geluaProfiler then geluaProfiler:add("luaPreRender map update") end
  extensions.hook('onPreRender', dtReal, dtSim, dtRaw)
  if geluaProfiler then geluaProfiler:add("luaPreRender extensions") end

  extensions.hook('onDrawDebug', Lua.lastDebugFocusPos, dtReal, dtSim, dtRaw)

  if geluaProfiler then geluaProfiler:add("luaPreRender drawdebug") end

  -- will be used for ge streams later
  -- guihooks.sendStreams(dtReal)

  -- detect if we need to switch the UI around
  if worldReadyState == 1 then
    -- log('I', 'gamestate', 'Checking if vehicle is done rendering material') -- this is far too verbose and seriously slows down the debugging
    luaPreRenderMaterialCheckDuration = luaPreRenderMaterialCheckDuration + dtRaw
    local playerVehicle = getPlayerVehicle(0) or nil

    local allReady = (not playerVehicle) or (playerVehicle and playerVehicle:isRenderMaterialsReady())
    if allReady or luaPreRenderMaterialCheckDuration > 5 then
      log('D', 'gamestate', 'Checking material finished loading')
      core_gamestate.requestExitLoadingScreen('worldReadyState')
      -- switch the UI to play mode
      --guihooks.trigger('ChangeState', 'menu', {'loading', 'backgroundImage.mainmenu'})
      worldReadyState = 2
      luaPreRenderMaterialCheckDuration = 0
      extensions.hook('onWorldReadyState', worldReadyState)


      -- render 100 frames before creating the report
      if simpleProfilerStop and doStartupProfiling and Engine.Render.getFrameId() > 100 and not _levelLoadingReportGenerated then
        extensions.utils_simpleProfiler_report.createReport('level_loading', 'Level loading')
        simpleProfilerStop()
        rawset(_G, '_levelLoadingReportGenerated', true)
      end
    end
  end



  if geluaProfiler then geluaProfiler:add("luaPreRender ending") end
end

local alreadyWarnedFSErrors = false
function checkFSErrors()
  if alreadyWarnedFSErrors then return end
  alreadyWarnedFSErrors = true
  local fsInfo = Engine.Platform.getFSInfo()
  for k,v in pairs(fsInfo) do
    if v then
      guihooks.trigger("toastrMsg", {type="error", title="ui.fsError.title", msg="ui.fsError.msg", config={closeButton=true, timeOut=0, extendedTimeOut=0}})
      log("E", "", "Filesystem errors detected. This typically means a corrupted install and can lead to missing content/levels/vehicles/uiapps/etc and generally broken behaviour.\n - If you are a user please follow the instructions at https://go.beamng.com/verify\n - If you are a dev/support debugging this problem, please check logs during startup, there might be errors with additional information\nDebug data: "..dumps(fsInfo))
      return
    end
  end
end

function updateFirstFrame()
  -- completeIntegrityChunk("base") -- unused for now
  extensions.hook('onFirstUpdate')
  settings.finalizeInit()

  editorEnabled(Engine.getEditorEnabled()) -- make sure the editing tools are in the correct state
  handleCommandLineFirstFrame()

  -- we cannot wait for the UI to be ready if it doesn't exist
  if tableFindKey(cmdArgs, '-noui') or headless_mode then
    -- -headless is working differenly than expected and should not be used
    -- it only prevents opening a main window
    uiReady()
  end
end

-- this function is called after input and before physics
function update(dtReal, dtSim, dtRaw)
  if geluaProfiler then geluaProfiler:start() end
  --local used_memory_bytes, _ = gcinfo()
  --log('D', "update", "Lua memory usage: " .. tostring(used_memory_bytes/1024) .. "kB")
  profilerPushEvent("GE Main Update")

  -- We do not implement the onUpdate hook because we want to control when we tick settings to do its thing.
  settings.settingsTick(dtReal, dtSim, dtRaw)

  extensions.core_input_bindings.updateGFX(dtRaw)
  simTimeAuthority.update(dtReal)
  if geluaProfiler then geluaProfiler:add("update beginning") end

  extensions.hook('onUpdate', dtReal, dtSim, dtRaw)
  if geluaProfiler then geluaProfiler:add("update extensions") end
  if be:getUpdateUIflag() then
    extensions.hook('onGuiUpdate', dtReal, dtSim, dtRaw)
    if geluaProfiler then geluaProfiler:add("update onGuiUpdate") end
  end
  perf.update()

  if geluaProfiler then geluaProfiler:add("update ending") end
  if geluaProfiler then
    geluaProfiler:finish(true)
    geluaProfiler = nil
    extensions.setProfiler(geluaProfiler)
  end
  profilerPopEvent('GE Main Update')
end

-- called when the UI is up and running
function uiReady()
  extensions.hook('onUiReady')
end

-- Also called on reload (Control-L)
function init(reason)
  settings.initSettings(reason)

  --log('D', "init", 'GameEngine Lua (re)loaded')
  flowGraphEditor_ffi_cdef_loaded = false

  -- be sensitive about global writes from now on
  detectGlobalWrites()

  prepareStartupExtensionsList()
  extensions.load(startupExtensions)

  table.clear(cmdlineLevelLoadExtensions)

  core_online.openSession() -- try to connect to online services

  -- import state last
  importPersistentData()

  -- request the UI ready state
  guihooks.trigger('isUIReady')

  map.assureLoad()

  -- world ready to do sth
  if worldReadyState ~= -1 then  -- Do not change to zero when we are sitting in the Main menu on a fresh start of the game
    worldReadyState = 0
  end

  -- put the mods folder in clear view, so users don't put stuff in the wrong place
  if not FS:directoryExists("mods") then FS:directoryCreate("mods") end

  if not FS:directoryExists("trackEditor") or not string.startswith(FS:getFileRealPath("trackEditor"), FS:getUserPath())  then FS:directoryCreate("trackEditor") end
end

function onBeamNGWaypoint(args)
  map.onWaypoint(args)
  extensions.hook('onBeamNGWaypoint', args)
end

-- do not delete - this is the default function name for the BeamNGTrigger from the c++ side
function onBeamNGTrigger(data)
  extensions.hook('onBeamNGTrigger', data)
end

function onFileChanged(files)
  --print("onFileChanged: " .. dumps(files))
  settings.onFilesChanged(files)
  map.onFilesChanged(files)
  extensions.hook('onFilesChanged', files)

  for _,v in pairs(files) do
    --print("onFileChanged: " .. tostring(v.filename) .. ' : ' .. tostring(v.type))
    extensions.hook('onFileChanged', v.filename, v.type)
  end
  extensions.hook('onFileChangedEnd')
end

function physicsEngineEvent(...)
  local args = unpack({...})
  extensions.hook('onPhysicsEngineEvent', args)
end

function vehicleSpawned(vid)
  profilerPushEvent('vehicleSpawned')
  local v = getObjectByID(vid)
  if not v then return end

  -- update the gravity of the vehicle
  if core_environment then
    v:queueLuaCommand("obj:setGravity("..core_environment.getGravity()..")")
  end

  invalidateVehicleCache()
  extensions.hook('onVehicleSpawned', vid, v)
  profilerPopEvent('vehicleSpawned')
end

-- when the player is switching vehicles
function vehicleSwitched(oldVehicle, newVehicle, player)
  profilerPushEvent('vehicleSwitched')
  local oid = oldVehicle and oldVehicle:getId() or -1
  local nid = newVehicle and newVehicle:getId() or -1
  -- local oldinfo = oldVehicle and ("id "..dumps(oid).." ("..oldVehicle:getPath()..")") or dumps(oldVehicle)
  -- local newinfo = newVehicle and ("id "..dumps(nid).." ("..newVehicle:getPath()..")") or dumps(newVehicle)
  --log('I', 'main', "Player #"..dumps(player).." vehicle switched from: "..oldinfo.." to: "..newinfo)
  --Steam.setStat('meters_driven', 1)
  invalidatePlayerVehicles()
  extensions.hook('onVehicleSwitched', oid, nid, player)
  guihooks.trigger('VehicleFocusChanged', {id = nid, mode = true})
  profilerPopEvent('vehicleSwitched')
end

-- Async callback when a vehicle's cluster is teleported
function onSetClusterPosRelRot(vehicleID, cNodeId)
  extensions.hook('onSetClusterPosRelRot', vehicleID, cNodeId)
end

-- Async callback when a vehicle is resetted
function vehicleReset(vehicleID)
  extensions.hook('onVehicleResetted', vehicleID)
end

-- Callback when vehicles are activated or deactivated (e.g. by the traffic pooling system)
-- This is unrelated to tabbing into other cars (that would be the vehicleSwitched() function)
function vehicleActiveChanged(vehicleID, active)
  extensions.hook('onVehicleActiveChanged', vehicleID, active)
end

function onMouseLocked(locked)
  extensions.hook('onMouseLocked', locked)
end

function vehicleDestroyed(vid)
  invalidatePlayerVehicles()
  invalidateVehicleCache()
  extensions.hook('onVehicleDestroyed', vid)
end

function onClusterTeleportNoReset(vehicle, nodeId, px, py, pz, rdx, rdy, rdz, rdw)
  extensions.hook('onClusterTeleportNoReset', vehicle:getId(), nodeId, px, py, pz, rdx, rdy, rdz, rdw)
end

function onCouplerAttached(objId1, objId2, nodeId, obj2nodeId)
  if objId1 ~= objId2 and settings.getValue("couplerCameraModifier", false) then
    local isEnabled = core_couplerCameraModifier ~= nil
    extensions.load('core_couplerCameraModifier')
    if isEnabled == false and core_couplerCameraModifier.checkForTrailer(objId1, objId2) == false then
      extensions.unload('core_couplerCameraModifier')
    end
  end
  extensions.hook('onCouplerAttached', objId1, objId2, nodeId, obj2nodeId)
end

function onCouplerDetached(objId1, objId2, nodeId, obj2nodeId)
  extensions.hook('onCouplerDetached', objId1, objId2, nodeId, obj2nodeId)
  if core_couplerCameraModifier ~= nil then
    extensions.unload('core_couplerCameraModifier')
  end
end

--Trigered when trailer coupler is detached by the user
function onCouplerDetach(objId, nodeId)
  extensions.hook('onCouplerDetach', objId, nodeId)
end

function onAiModeChange(vehicleID, newAiMode)
  extensions.hook('onAiModeChange', vehicleID, newAiMode)
end

function replayStateChanged(...)
  core_replay.stateChanged(...)
end

function openXRStateChanged(...)
  if not render_openxr then return end
  render_openxr.stateChanged(...)
end

function openXRErrorDetected(...)
  if not render_openxr then return end
  render_openxr.errorDetected(...)
end

-- only the vehicle and config are necessary here (the rest of parameters will be set during regular playback anyway, no point duplicating them here too)
function replaySpawnVehicle(jbeamFilename, partConfigData)
  local veh = spawn.spawnVehicle(jbeamFilename, partConfigData, vec3(), quat(), nil, nil, nil)
  if be:getEnterableObjectCount() == 1 then
    commands.setGameCamera()
  end
  return veh
end

function replayStartLevel(levelPath)
  core_replay.startLevel(levelPath)
end

-- called by C++ (not dead code)
function CEFTypingLostFocus()
  guihooks.trigger('CEFTypingLostFocus')
end

function exportPersistentData()
  if not be then return end
  local d = serializePackages()
  d.levelLoaded = levelLoaded
  -- log('D', 'main', 'persistent data exported: ' .. dumps(d))
  -- jsonWriteFile('persistentData.json', d, true)
  be.persistenceLuaData = serialize(d)
end

function importPersistentData()
  if not be then return end
  local s = be.persistenceLuaData
  -- log('E', 'main', '>>>> persistent data imported: ' .. tostring(s))
  -- deserialize extensions first, so the extensions are loaded before they are trying to get deserialized
  local ok, data = pcall(deserialize, s)
  if not ok then
    log('E', 'main', 'Error importing persistent data: ' .. tostring(data))
    writeFile('persistentDataError.txt', s)
  else
    deserializePackages(data)
    if data then
      rawset(_G, 'levelLoaded', data.levelLoaded)
    end
  end
end

function physicsStateChanged(val)
  guihooks.trigger('physicsStateChanged', val)
  if val then
    extensions.hook('onPhysicsUnpaused')
  else
    extensions.hook('onPhysicsPaused')
  end
end

function updateTranslations()
  -- unmount if in use, so we can update the file
  if FS:isMounted('mods/translations.zip') then
    FS:unmount('mods/translations.zip')
  end

  extensions.core_repository.installMod('locales.zip', 'translations.zip', 'mods/', function(data)
    log('D', 'updateTranslations', 'translations download done: mods/translations.zip')
    -- reload the settings to activate the new files
    settings.newTranslationsAvailable = true -- this enforces the UI refresh, fixes some state problems
    settings.load(true)
  end)
end

function enableCommunityTranslations()
  settings.setState( { communityTranslations = 'enable' } )
  updateTranslations()
end

function onInstabilityDetected(vid)
  local v = getObjectByID(vid)
  local jbeamFilename = v:getJBeamFilename()
  simTimeAuthority.pause(true)
  log('E', "", "Instability detected for vehicle ID: "..dumps(vid)..", jbeamFilename: "..dumps(jbeamFilename))
  log("E", "", "Information about all vehicles:")
  for vid,v in vehiclesIterator() do
    log("E", "", " - Vehicle ID: "..dumps(vid)..", jbeamFilename: "..v:getJBeamFilename()..", position: "..dumps(v:getPosition())..", partConfig: "..dumps(v.partConfig))
  end
  ui_message({txt="vehicle.main.instability", context={vehicle=tostring(jbeamFilename)}}, 10, 'instability', "warning")
end

function onSpawnError(status, jbeamFilename)
  log("E", "onSpawnError", "Error "..dumps(status).." spawning vehicle "..dumps(jbeamFilename))
  guihooks.trigger("toastrMsg", {type="error", title="vehicle.main.spawnError.title", msg="vehicle.main.spawnError.msg", context={status=status, vehicle=jbeamFilename}, config={closeButton=true, timeOut=0, extendedTimeOut=0}})
end

function resetGameplay(playerID)
  extensions.hook('onResetGameplay', playerID)
end

function sendUIModules()
  local ok = xpcall(function()
    guihooks.trigger('onUIBootstrap', FS:directoryList('/ui/modModules/', false, true))
  end, debug.traceback)
  if not ok then
    log('E', '', 'UI Bootstrap failed, using fallback')
    guihooks.trigger('onUIBootstrap', {})
  end
end

function loadDirRec(dir)
  local foundfiles = FS:findFiles(dir, "*materials.cs\t*materials.json\t*datablocks.json", -1, true, false)
  local csMaterialFiles = {}
  local datablockFiles = {}
  local jsonMaterialFiles = {}

  for _, filename in ipairs(foundfiles) do
    if string.find(filename, 'datablocks.json') then
      table.insert(datablockFiles, filename)
    elseif string.find(filename, 'materials.json') then
      table.insert(jsonMaterialFiles, filename)
    elseif string.find(filename, 'materials.cs') then
      table.insert(csMaterialFiles, filename)
    end
  end

  -- load old CS materials first:
  for _, filename in ipairs(csMaterialFiles) do
    TorqueScriptLua.exec(filename)
  end

  -- then the new ones
  for _, filename in ipairs(jsonMaterialFiles) do
    loadJsonMaterialsFile(filename)
  end

  -- datablocks
  for _, filename in ipairs(datablockFiles) do
    loadJsonMaterialsFile(filename)
  end
end

local function loadModScriptsRec(dir)
  --print("Loading ModScripts on " .. dir)
  local filefilter = dir .. "/*/modScript.cs"
  local fileC = findFirstFile(filefilter)

  repeat
    if fileC ~= "" then
      --print(" * loading mod script file: " .. fileC)
      require(fileC)
    end
    fileC = findNextFile(filefilter)
  until fileC == ""
end

------------------------------ Entry point code ------------------------------
function onPreStart()
  -- log('I', 'main', 'onPreStart called...')
end

function onPreWindowClose()
  extensions.hook('onPreWindowClose')
end

function onPreExit()
  local p = LuaProfiler("onPreExit()")
  p:start()
  extensions.hook('onPreExit')
  p:add("extensions.onPreExit")
  p:finish(true)
end

function onExit()
  local p = LuaProfiler("onExit()")
  p:start()
  -- onExit is called directly from C++ code
  extensions.hook('onExit')
  p:add("extensions.onExit")

  -- scripts_main.onExit()
  -- Ensure that we are disconnected and/or the server is destroyed.
  -- This prevents crashes due to the SceneGraph being deleted before
  -- the objects it contains.
  serverConnection.noLoadingScreenDisconnect(p)
  p:add("noLoadingScreenDisconnect")

  -- Destroy the physics plugin.
  PhysicsPlugin.destroy()
  p:add("PhysicsPlugin")

  -- TODO(AK) 18/08/2021: check which calls to replace this Parent::onExit with.
  -- Parent::onExit();
  p:add("ParentOnExit")

  local mainEventManager = scenetree.findObject("MainEventManager")
  p:add("MainEventManager.find")
  if mainEventManager then
    mainEventManager:postEvent("onExit", 0)
    p:add("MainEventManager.onExit")
  else
    log("E","", "Couldn't find event manager 'MainEventManager'")
    p:add("MainEventManager.onError")
  end

  postFxModule.savePresetFile('settings/postfxSettings.postfx')
  p:add("savePostFx")
  settings.exit()
  p:add("settings")
  p:finish(true)
end

function onGameEngineStartup()
  -- log('I', "main", "onGameEngineStartup called.....")

  -- make sure some important paths exist
  if not FS:directoryExists("settings/") then FS:directoryCreate("settings/") end
  if not FS:directoryExists("screenshots/") then FS:directoryCreate("screenshots/") end

  -- Set profile directory
  setConsoleVariable("$Pref::Video::ProfilePath", "core/profile")

  local mainEventManager = createObject("EventManager")
  if mainEventManager then
    -- log("I","", "mainEventManager = "..dumps(mainEventManager))
    mainEventManager.queue = "mainEventManagerQueue"
    mainEventManager:registerEvent("onExit")
    mainEventManager:registerEvent("onStart")
    mainEventManager:registerEvent("onPreStart")
    mainEventManager:registerObject("MainEventManager")
  else
    log("E","", "Couldn't create event manager 'MainEventManager'")
  end

  parseArgs.defaultParseArgs()

  onPreStartCallback()

  ---------------------------------------------------------
  -- Either display the help message or startup the app.
  -- This is emulating what mainEventManager:postEvent("onPreStart", 0) call would do for all listeners. However, in the entire codebase we
  -- don't have a single CS file responding to this message. We should delete it.
  -- if scripts_main.onPreStart and type(scripts_main.onPreStart) == 'function' then
  --   scripts_main.onPreStart()
  -- end

  -- TODO(AK) 16/08/2021: This is for Torque Script cs files to hook into. Maybe re-enable after removing CS scripts involved in start up
  -- mainEventManager:postEvent("onPreStart", 0)
  ---------------------------------------------------------
  -- core_main.onStart()
  -- Initialise Core stuff.
  local clientCore = require("client/core")
  clientCore.initializeCore()

  -- log('I', "main", "Initialized Core...")

  -- first check if we have a level file to load
  local levelToLoad = getConsoleVariable("$levelToLoad")
  if levelToLoad ~= "" then
    -- Clear out the $levelToLoad so we don't attempt to load the level again later on.
    setConsoleVariable("$levelToLoad", "")
    local levelFile = "levels/" .. levelToLoad
    freeroam_freeroam.startFreeroam(levelFile)
  end

  -- scripts_main.onStart()
  setConsoleVariable("$pref::Directories::Terrain", "levels/")

  -- log('I', "main", "--------- Initializing Directory: scripts---------");

  -- Load the scripts that start it all...
  local client_init = require("client/init")

  -- Init the physics plugin.
  PhysicsPlugin.init("")

  client_init.initClient()

  if mainEventManager then
    mainEventManager:postEvent("onStart", 0)
  end

  -- Automatically start up the appropriate editor, if any
  if getConsoleBoolVariable("$startWorldEditor") then
    local canvas = scenetree.findObject("Canvas")
    local cursor = scenetree.findObject("DefaultCursor")
    local editorChooseLevelGui = scenetree.findObject("EditorChooseLevelGui")
    if canvas and cursor then
      canvas:setCursor(cursor)
      canvas:setContent(editorChooseLevelGui)
    end
  end
  log("I", "", "============== GELUA VM loaded ================")
end

function onLuaReloaded()
  local clientCore = require("client/core")
  clientCore.reloadCore()
  local client_init = require("client/init")
  client_init.reloadClient()
  log("I", "", "============== GELUA VM reloaded ==============")
end

function updateLoadingProgress(val, txt)
  local msg = string.format("[{val: %u%%, txt: %s}]", math.floor(100 * val), txt)

  guihooks.trigger("UpdateProgress", msg) -- the Json object is inside an array as it is the first argument of the function :)

  local loadingLevel = TorqueScriptLua.getBoolVar("$loadingLevel")

  if loadingLevel then
    local canvas = scenetree.findObject("Canvas")
    if canvas then
      canvas:repaintUIThrottled()
    end
  end
  setConsoleVariable("$lastProgress", val)
  setConsoleVariable("$$lastProgressTxt", txt)
end

function updateTSShapeLoadProgressDynamic(progress, msg)
  if updateTSShapeLoadProgress then
    -- usually the case if the editor is loaded, it usually calls updateLoadingProgress in it then
    updateTSShapeLoadProgress(progress, msg)
  else
    -- usually the case if the editor is NOT loaded
    -- %msg = translate("ui.loading.spawn.collada", "Importing 3D stuff") .. " ...";
    updateLoadingProgress(progress, msg)
  end
end

-- DEPRECATED FUNCTION: if we have released v0.36 or later and these functions still exist, please take a minute to remove them:
function loadGameModeModules(...)
  log('E','','== OUTDATED MOD CODE ==')
  log('E','','loadGameModeModules(xxx) will be deprecated soon. Instead, replace with these calls:\nunloadAutoExtensions()\nloadPresetExtensions()\nextensions.load(xxx) -- you can omit this line if no parameter was passed')
  print(debug.tracesimple())
  log('E','','== OUTDATED MOD CODE ==')

  unloadAutoExtensions()
  loadPresetExtensions()
  extensions.load(...)
end

-- DEPRECATED FUNCTION: if we have released v0.36 or later and these functions still exist, please take a minute to remove them:
function registerCoreModule(extensionName)
  log('E','','== OUTDATED MOD CODE ==')
  log('E','',"registerCoreModule("..dumps(extensionName).." will be deprecated soon. Instead, replace with: setExtensionUnloadMode(M, \"manual\")")
  print(debug.tracesimple())
  log('E','','== OUTDATED MOD CODE ==')

  extensionName = extensions.luaPathToExtName(extensionName)
  setExtensionUnloadMode(extensionName, "manual")
end
