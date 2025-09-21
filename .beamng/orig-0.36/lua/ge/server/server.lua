-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local logTag = 'server.lua'
local loadingProgress, timer2, levelPath

local function endMission(p)
  if scenetree.MissionGroup then
    local missionFilename = getMissionFilename()
    log('I', logTag,"*** Level ended: "..missionFilename)

    TorqueScriptLua.setVar("$instantGroup", 0)
    if p then p:add("endMission.vars") end
    clientEndMission(missionFilename)
    if p then p:add("endMission.clientEndMission") end

    if scenetree.EditorGui then
      TorqueScript.eval("EditorGui.onClientEndMission();")
    end
    if p then p:add("endMission.editorGui") end

    if scenetree.AudioChannelEffects then
      scenetree.AudioChannelEffects:stop(-1.0, -1.0)
    end
    if p then p:add("endMission.audio") end

    decalManagerClear()
    if p then p:add("endMission.decals") end

    scenetree.MissionGroup:deleteAllObjects()
    if p then p:add("endMission.deleteObjects") end
    scenetree.MissionGroup:delete()
    if p then p:add("endMission.delete") end
  end

  if scenetree.MissionCleanup then
    scenetree.MissionCleanup:delete()
    if p then p:add("endMission.cleanup") end
  end

  if scenetree.LevelLoadingGroup then
    scenetree.LevelLoadingGroup:delete()
    if p then p:add("endMission.levelLoading") end
  end

  if clearLevelLogs then
    clearLevelLogs()
    if p then p:add("endMission.clearLogs") end
  end

  setMissionPath("")
    if p then p:add("endMission.finalSet") end
end

-- this function can patch the loading context depending on the program's command line arguments
local function patchLoadingContext(ldgCtx)
  local cmdArgs = Engine.getStartingArgs()
  local i = 1
  while i <= #cmdArgs do
    local v = cmdArgs[i]
    if v == '-levelOffset' then
      if i + 3 <= #cmdArgs then
        local x = tonumber(cmdArgs[i + 1])
        local y = tonumber(cmdArgs[i + 2])
        local z = tonumber(cmdArgs[i + 3])
        if x and y and z then
          local mat = MatrixF(true)
          local pos = Point3F(x, y, z)
          mat:setPosition(pos)
          ldgCtx.tileTransform = mat
          log('I', string.format('### Global level offset set to (%.2f, %.2f, %.2f)', x, y, z))
          i = i + 4 -- Skip the consumed arguments
        else
          log('E', 'Invalid argument for -levelOffset: expected three numeric values.')
        end
      else
        log('E', 'Not enough arguments for -levelOffset: expected three numeric values.')
      end
    else
      i = i + 1 -- Move to the next argument
    end
  end

end

--seems to work for freeroam
local function createGameActual(lvlPath, customLoadingFunction)
  local timerFunc = hptimer()
  profilerPushEvent('createGameActual')
  levelPath = lvlPath

  log('I', 'levelLoading', "Level loading: '"..levelPath.."'...")

  LoadingManager:setLoadingScreenEnabled(true)
  loadingProgress = LoadingManager:push('level')

  rawset(_G, 'gameConnection', {}) -- backward compatibility


  --Engine.Profiler.startCapture()

  profilerPushEvent('init')

  local timer1 = hptimer()
  timer2 = hptimer()

  TorqueScriptLua.setVar("$loadingLevel", true)  -- DO NOT REMOVE, this is used on the c++ side
  levelPath = levelPath:lower()
  if not levelPath:find(".json") and not levelPath:find(".mis") then
    local levelName = path.levelFromPath(levelPath)
    levelPath = path.getPathLevelMain(levelName)
  end

  profilerPushEvent('clientPreStartMission')

  clientPreStartMission(levelPath)
  profilerPopEvent("clientPreStartMission")

  TorqueScriptLua.setVar("$Physics::isSinglePlayer", "true")

  local timeInit = timer1:stopAndReset() / 1000

  -- Load up any core datablocks
  if FS:fileExists("core/art/datablocks/datablockExec.cs") then
    TorqueScriptLua.exec("core/art/datablocks/datablockExec.cs")
  end

  profilerPopEvent("init")
  loadingProgress:update(-1, 'init done')

  profilerPushEvent('datablocks')

  -- Let the game initialize some things now that the
  -- the server has been created

  -- Create the physics world.
  be:physicsInitWorld()
  loadingProgress:update(-1, '')

  -- Load up any objects or datablocks saved to the editor managed scripts
  loadJsonMaterialsFile("art/shapes/particles/managedParticleData.json")
  loadingProgress:update(-1, '')
  loadJsonMaterialsFile("art/shapes/particles/managedParticleEmitterData.json")
  loadingProgress:update(-1, '')
  if FS:fileExists("art/decals/managedDecalData.cs") then
    TorqueScriptLua.exec("art/decals/managedDecalData.cs")
    loadingProgress:update(-1, '')
  end

  -- TorqueScriptLua.exec("art/datablocks/datablockExec.cs")
  loadJsonMaterialsFile("art/datablocks/audioProfiles.datablocks.json")
  loadingProgress:update(-1, '')
  loadJsonMaterialsFile("art/datablocks/sounds.datablocks.json")
  loadingProgress:update(-1, '')
  if FS:fileExists("art/datablocks/player.datablocks.json") then
    loadJsonMaterialsFile("art/datablocks/player.datablocks.json")
    loadingProgress:update(-1, '')
  end
  loadJsonMaterialsFile("art/datablocks/datablockExec.datablocks.json")
  loadingProgress:update(-1, '')

  loadJsonMaterialsFile("art/datablocks/lights.datablocks.json")
  loadingProgress:update(-1, '')
  loadJsonMaterialsFile("art/datablocks/managedDatablocks.datablocks.json")

  local timeDatablocks = timer1:stopAndReset() / 1000

  profilerPopEvent("datablocks")
  loadingProgress:update(-1, 'datablocks done')
  profilerPushEvent('materials')

  endMission()

  local LevelLoadingGroup = createObject("SimGroup")
  if not LevelLoadingGroup then
    log('E', 'levelLoading', "could not create LevelLoadingGroup SimGroup")
    return
  end
  LevelLoadingGroup:registerObject("LevelLoadingGroup")

  --Make the LevelLoadingGroup group the place where all new objects will automatically be added.
  TorqueScriptLua.setVar("$instantGroup", "LevelLoadingGroup")


  TorqueScriptLua.setVar("$missionRunning", "false")
  setMissionFilename(levelPath:gsub("//", "/"))

  local levelDir = path.dirname(levelPath)
  if string.sub(levelDir, -1) ~= '/' then
    levelDir = levelDir.."/"
  end
  setMissionPath(levelDir)

  TorqueScriptLua.setVar("$Server::LoadFailMsg", "")

  -- clear LevelInfo so there is no conflict with the actual LevelInfo loaded in the level
  local levelInfo = scenetree.findObject("theLevelInfo")
  if levelInfo then
    levelInfo:delete()
    levelInfo = nil
  end

  local foundfiles = FS:findFiles(levelDir, "*.cs\t*materials.json\t*data.json\t*datablocks.json", -1, true, false)
  table.sort(foundfiles)

  local tsFilesToExecute = {}
  local jsonFilesToLoad = {}
  for _, filename in ipairs(foundfiles) do
    if string.find(filename, 'datablocks.json') then
      table.insert(jsonFilesToLoad, filename)
    elseif string.find(filename, 'materials.cs') then
      loadingProgress:update(-1, '')
      TorqueScriptLua.exec(filename)
    elseif string.find(filename, 'materials.json') then
      loadingProgress:update(-1, '')
      loadJsonMaterialsFile(filename)
    elseif string.match(filename, "/%a+Data%.json$") then
      table.insert(jsonFilesToLoad, filename)
    elseif string.find(filename, '.cs') then
      table.insert(tsFilesToExecute, filename)
    end
  end

  for  _, filename in pairs(jsonFilesToLoad) do
    loadingProgress:update(-1, '')
    loadJsonMaterialsFile(filename)
  end

  for  _, filename in pairs(tsFilesToExecute) do
    loadingProgress:update(-1, '')
    TorqueScriptLua.exec(filename)
  end

  profilerPopEvent("materials")
  loadingProgress:update(-1, 'materials done')
  profilerPushEvent('objects')

  local timeMat = timer1:stopAndReset()/1000

  local ldgCtx = loadingContext()
  patchLoadingContext(ldgCtx)

  -- if the scenetree folder exists, try to load it
  if FS:directoryExists(levelDir .. 'main/') then
    LoadingManager:loadLevelJsonObjects(levelDir .. 'main/', ldgCtx)
  else
    -- backward compatibility: single file mode
    local levelName = path.levelFromPath(levelPath)
    local json_main = path.getPathLevelMain(levelName)
    if FS:fileExists(json_main) then
      Sim.deserializeObjectsFromFile(json_main, true)
    else
      -- backward compatibility: single .mis file mode
      -- Make sure the level exists
      if not FS:fileExists(levelPath) then
        log('E', 'levelLoading', "Could not find level: "..levelPath)
        return
      end
      TorqueScriptLua.exec(levelPath)
    end
    LoadingManager:_triggerSignalLevelLoaded() -- backward compatibility for older levels
  end
  Engine.Platform.taskbarSetProgressState(1)

  if not scenetree.MissionGroup then
    log('E', 'levelLoading', "MissionGroup not found")
    return
  end

  --[[level cleanup group.  This is where run time components will reside.]]
  local misCleanup = createObject("SimGroup")
  if not misCleanup then
    log('E', 'levelLoading', "could not create MissionCleanup SimGroup")
    return
  end
  misCleanup:registerObject("MissionCleanup")

  --Make the MissionCleanup group the place where all new objects will automatically be added.
  TorqueScriptLua.setVar("$instantGroup", misCleanup:getID())

  log('I', 'levelLoading', "Level loaded: "..getMissionFilename())

  TorqueScriptLua.setVar("$missionRunning", 1)

  -- be:physicsStartSimulation()
  extensions.hook('onClientCustomObjectSpawning', mission)

  if scenetree.AudioChannelEffects then
    scenetree.AudioChannelEffects:play(-1.0, -1.0)
  end

  local timeObjects = timer1:stopAndReset() / 1000

  -- notify the map
  map.onMissionLoaded()

  local timeAIMap = timer1:stopAndReset() / 1000

  -- Load the static level decals.
  if FS:fileExists(levelDir.."main.decals.json") then
    be:decalManagerLoad(levelDir.."main.decals.json", ldgCtx)
  elseif FS:fileExists(levelDir.."../main.decals.json") then
    be:decalManagerLoad(levelDir.."../main.decals.json", ldgCtx)
  end
  local timeDecals = timer1:stopAndReset() / 1000

  profilerPopEvent("objects")
  loadingProgress:update(-1, 'objects done')
  profilerPushEvent('start physics')

  be:physicsStartSimulation()
  local timePhysics = timer1:stopAndReset() / 1000

  profilerPopEvent("start physics")
  loadingProgress:update(-1, 'physics done')
  profilerPushEvent('spawn player')

  -- NOTE(AK): These spawns are only needed by freeroam. Scenario does it's own spawning
  spawn.spawnCamera()
  local timeCam = timer1:stopAndReset() / 1000
  spawn.spawnPlayer()
  extensions.hook('onPlayerCameraReady')
  local timePlayer = timer1:stopAndReset() / 1000
  profilerPopEvent("spawn player")

  ------------------------------------
  if customLoadingFunction then
    log("D",'levelLoading',"*** Delaying fadeout by request.")
    customLoadingFunction()
  else
    M.fadeoutLoadingScreen()
  end
  local timeFade = timer1:stopAndReset() / 1000

  rawset(_G, 'levelLoaded', levelDir)

  local timeTotal = timerFunc:stopAndReset() / 1000
  log('I', 'levelLoading', string.format("Level loaded in %.3fs: init %.3fs + datablocks %.3fs + materials %.3fs + objects %.3fs + ai.map %.3fs + decals %.3fs + physics %.3fs + cam %.3fs + player %.3fs + fade %.3fs", timeTotal, timeInit, timeDatablocks, timeMat, timeObjects, timeAIMap, timeDecals, timePhysics, timeCam, timePlayer, timeFade))
end

local function fadeoutLoadingScreen(skipStart)
  if not levelPath then
    log("W",'fadeoutLoadingScreen',"levelPath is already nil.")
    return
  end
  loadingProgress:update(-1, 'player done')

  core_gamestate.requestExitLoadingScreen(logTag)

  if not skipStart then
    profilerPushEvent('clientPostStartMission')

    clientPostStartMission(levelPath)

    profilerPopEvent("clientPostStartMission")
    profilerPushEvent('clientStartMission')

    clientStartMission(getMissionFilename())

    profilerPopEvent("clientStartMission")
  else
    guihooks.trigger('MenuHide')
  end

  Engine.Platform.taskbarSetProgressState(0)
  TorqueScriptLua.setVar("$loadingLevel", false) -- DO NOT REMOVE, this is used on the c++ side

  LoadingManager:pop(loadingProgress)


  LoadingManager:setLoadingScreenEnabled(false)
  extensions.hook("onLoadingScreenFadeout")
  log('I', 'levelLoading', 'Loading screen disabled after ' .. string.format('%5.3fs', timer2:stopAndReset() / 1000))
  --Engine.Profiler.stopCapture()
  --Engine.Profiler.saveCapture('loading.opt')
  levelPath, timer2, loadingProgress = nil, nil, nil
end

local function destroy(p)
  TorqueScriptLua.setVar("$missionRunning", "false")
  if p then p:add("server.destroy.setvar") end

  --End any running levels
  endMission(p)
  if p then p:add("server.destroy.endMission") end

  be:physicsDestroyWorld()
  if p then p:add("server.destroy.physics") end

  TorqueScriptLua.setVar("$Server::GuidList", "")
  if p then p:add("server.destroy.setvar") end

  -- Delete all the data blocks...
  be:deleteDataBlocks()
  if p then p:add("server.destroy.datablocks") end

  -- Increase the server session number.  This is used to make sure we're
  -- working with the server session we think we are.
  local sessionCnt = (tonumber(TorqueScriptLua.getVar("$Server::Session")) or 0) +1
  TorqueScriptLua.setVar("$Server::Session", sessionCnt)
  if p then p:add("server.destroy.sessioncount") end

  rawset(_G, 'levelLoaded', nil)
  rawset(_G, 'gameConnection', nil) -- backward compatibility
  if p then p:add("server.destroy.rawsets") end
end

local function createGameWrapper(levelPath, customLoadingFunction)
  local function help()
    createGameActual(levelPath, customLoadingFunction)
  end
  --log('I', logTag, 'Loading = '..tostring(core_gamestate.loading()))
  -- yes this is weird, but it fixes the problem with createGame and luaPreRender
  core_gamestate.requestEnterLoadingScreen(logTag, help)
  core_gamestate.requestEnterLoadingScreen('worldReadyState')
  if __cefcontext_ == -1 then
    core_gamestate.loadingScreenActive()
  end
end

M.createGame = createGameWrapper
M.destroy = destroy
M.loadingProgress = loadingProgress
M.fadeoutLoadingScreen = fadeoutLoadingScreen
return M
