-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {"gameplay_drag_general", "gameplay_drag_utils"}
local logTag = ""

local dGeneral, dUtils

local dragData
local hasActivityStarted = false
local debugStart = false

local minDistance = 0.8 --m
local minVelToStop = 15 --m/s

local function clear()
  dragData = nil
  hasActivityStarted = false
  debugStart = false
end

local function onExtensionLoaded()
  --log("I", logTag, "gameplay_drag_dragTypes_headsUpDrag extension loaded")
  dGeneral = gameplay_drag_general
  dUtils = gameplay_drag_utils

  clear()
end

local function resetDragRace()
  if not dragData then
    dragData = dGeneral.getData()
  end
  --log('I', logTag, 'Reseting Drag Race')
  hasActivityStarted = false
  debugStart = false

  dragData.isStarted = false
  dragData.isCompleted = false

  for vehId, racer in pairs(dragData.racers) do
    --log('I', logTag, 'Reseting racer: '.. vehId)
    racer.currentPhase = 1
    racer.isDesqualified = false
    racer.desqualifiedReason = "None"
    racer.isFinished = false
    racer.treeStarted = false
    racer.timersStarted = false
    racer.damage = 0

    --Reset Phases
    --log('I', logTag, 'Reseting phases for: '.. vehId)
    for _, p in ipairs(racer.phases) do
      p.started = false
      p.completed = false
      p.timerOffset = 0
    end

    -- if racer.canBeTeleported then
    --   local veh = scenetree.findObjectById(racer.vehId)
    --   spawn.safeTeleport(veh, dragData.strip.lanes[racer.lane].waypoints.spawn.transform.pos, dragData.strip.lanes[racer.lane].waypoints.spawn.transform.rot, nil, nil, nil, true, racer.canBeReseted)
    --   --log('I', logTag, 'Teleported back to start: ' .. vehId)
    -- end
  end
  extensions.hook("resetDragRaceValues")
end


local function startActivity()
  dragData = dGeneral.getData()

  if dragData.prefabs.christmasTree.isUsed then
    extensions.load('gameplay_drag_times')
  end
  if dragData.prefabs.displaySign.isUsed then
    extensions.load('gameplay_drag_display')
  end

  if not dragData then
    log('E', logTag, 'No drag race data found')
  end
  dragData.isStarted = true
  hasActivityStarted = dragData.isStarted

  local dials = {}
  if dragData.racers then
    for _,racer in pairs(dragData.racers) do
      table.insert(dials, {vehId = racer.vehId, dial = 0})
    end
  end
  dUtils.setDialsData(dials)
end

local function startDebugPhase(pIndex, dData)
  if not pIndex or not dData then return end
  dragData = dData
  --log("I", logTag, "Starting debug phase for index: ".. pIndex)
  for _, racer in pairs(dragData.racers) do
    racer.currentPhase = pIndex
  end
  debugStart = true
end

local function onUpdate(dtReal, dtSim, dtRaw)
  if hasActivityStarted then
    if not dragData then
      log('E', logTag, 'No drag data found!')
      return
    end
    if not dragData.racers then
      log('E', logTag, 'There is no racers in the drag data.')
      return
    end
    local isActivityFinished = true
    for vehId, racer in pairs(dragData.racers) do
      if not racer.isFinished then
        isActivityFinished = false
      end

      dUtils.updateRacer(racer)
      --if racer.currentPhase == 2 or racer.currentPhase == 3 then
        --print("Running Phase: " .. racer.currentPhase)
      --end
      local phase = racer.phases[racer.currentPhase]
      dUtils[phase.name](phase, racer, dtSim)

      if phase.comepleted then
        log('I', logTag, 'Racer: '.. vehId ..' completed phase: '.. phase.name)
      end
      if phase.completed and not phase.dependency and not racer.isFinished then

        dUtils.changeRacerPhase(racer)
      end
    end

    if isActivityFinished then
      dragData.isCompleted = true
      hasActivityStarted = false
      return
    end

    local dependenciesCompleted = true
    for _, r in pairs(dragData.racers) do
      if not r.phases[r.currentPhase].completed then
        dependenciesCompleted = false
      end
    end

    if dependenciesCompleted then
      dUtils.changeAllPhases()
    end
  end



  --DEBUG MODE
    -- if debugStart then
    --   if not dragData then debugStart = false return end
    --   for vehId, racer in pairs(dragData.racers) do
    --     funcPhases[phase.name](vehId, dtSim)
    --   end
    -- end
end

--PUBLIC INTERFACE
M.onExtensionLoaded = onExtensionLoaded
M.onUpdate = onUpdate
M.startDebugPhase = startDebugPhase
M.startActivity = startActivity
M.resetDragRace = resetDragRace

return M