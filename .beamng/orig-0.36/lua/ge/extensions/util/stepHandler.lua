-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--[[
stepper.startStepSequence(
{
  stepper.makeStepFadeToBlack(),
  stepper.makeStepSpawnVehicleSimple("pickup","vehicles/pickup/d15_4wd_A.pc",function() print("Hello")end), stepper.makeStepFadeFromBlack()
  })
]]




local M = {}
local showDebugWindow = false
local taskData = {
  steps = {},
  data = {},
  active = false,
  currentStep = 0
}

-- fading helper
local function taskFadeStep(step)
  if not step.waitForFade then
    if step.direction == "start" then
      ui_fadeScreen.start(step.duration)
    else
      ui_fadeScreen.stop(step.duration)
    end
    step.waitForFade = true
  end
  if (step.direction == "start" and step.fadeState1) or (step.direction == "stop" and step.fadeState3) then
    step.complete = true
    ui_fadeScreen.delayFrames = 1
  end
end
M.onScreenFadeState = function(state)
  if not taskData.active or not taskData.steps[taskData.currentStep] or not taskData.steps[taskData.currentStep].waitForFade then
    return
  end
  taskData.steps[taskData.currentStep]["fadeState"..state] = true
end
local function makeStepFadeToBlack(duration)
  return {
    name = "fadeToBlackStep",
    processTask = taskFadeStep,
    direction = "start",
    timeout = 10,
    duration = duration,
  }
end
local function makeStepFadeFromBlack(duration)
  return {
    name = "fadeFromBlackStep",
    processTask = taskFadeStep,
    direction = "stop",
    timeout = 10,
    duration = duration,
  }
end
M.makeStepFadeToBlack = makeStepFadeToBlack
M.makeStepFadeFromBlack = makeStepFadeFromBlack

-- timing helper
local function taskWaitStep(step)
  if os.time() - step._startingTime > step.duration then
    step.complete = true
  end
end
local function makeStepWait(seconds)
  return {
    name = "waitStep",
    processTask = taskWaitStep,
    duration = seconds,
    timeout = seconds + 1
  }
end
M.makeStepWait = makeStepWait

-- custom function helper
local function taskCustomReturnTrueFunctionStep(step, dtTable)
  if step.fun(step, dtTable) then
    step.complete = true
  end
end

local function makeStepReturnTrueFunction(fun)
  return {
    name = "customReturnTrueStep",
    processTask = taskCustomReturnTrueFunctionStep,
    fun = fun,
  }
end
M.makeStepReturnTrueFunction = makeStepReturnTrueFunction

local function makeStepReturnTrueFunction(fun)
  return {
    name = "customReturnTrueStep",
    processTask = taskCustomReturnTrueFunctionStep,
    fun = fun,
  }
end
M.makeUiMessageStep = function(message)
  return makeStepReturnTrueFunction(function() ui_message(message) return true end)
end

-- vehicle spawning helper
local function taskVehicleSpawnStep(step)
  if not step.vehId then
    local options = step.options
    step.veh = core_vehicles.spawnNewVehicle(options.model, options)
    if not step.veh then
      step.complete = true
      log("E","","Could not spawn vehicle! " .. dumps(step) )
    end
    step.vehId = step.veh:getId()
  else
    local veh = scenetree.findObjectById(step.vehId)
    if veh then
      if veh:isReady() then
        step.complete = true
        (step.callback or nop)(step, veh:getId())
      end
    end
  end
end
local function makeStepSpawnVehicle(spawningOptions, callback)
  return {
    name = "spawnVehicleStep",
    processTask = taskVehicleSpawnStep,
    callback = callback,
    options = spawningOptions
  }
end
local function makeStepSpawnVehicleSimple(model, config, callback)
  return makeStepSpawnVehicle(sanitizeVehicleSpawnOptions(model, {config = config}), callback)
end
M.makeStepSpawnVehicle = makeStepSpawnVehicle
M.makeStepSpawnVehicleSimple = makeStepSpawnVehicleSimple


M.onVehicleGroupSpawned = function(vehIds, groupId)
  --dump(vehIds, groupId)
  --dump(taskData.steps[taskData.currentStep])
  if not taskData.active or not taskData.steps[taskData.currentStep] or not taskData.steps[taskData.currentStep].waitForVehicleGroup then
    return
  end
  if groupId == taskData.steps[taskData.currentStep].groupId then
    taskData.steps[taskData.currentStep].state = 2
    taskData.steps[taskData.currentStep].vehIds = vehIds
  end
end
-- vehicle spawning helper
local function taskSpawnTrafficStep(step)
  if step.state == 0 then
    -- step 0: spawn group
    local group = core_multiSpawn.createGroup(step.options.amount, step.options.generator)
    step.groupId = core_multiSpawn.spawnGroup(group, step.options.amount, {name = group.name, shuffle = true, mode = "traffic"})
    step.waitForVehicleGroup = true
    step.state = 1
  end
  if step.state == 2 then
    gameplay_traffic.setTrafficVars({activeAmount = step.options.active})
    gameplay_traffic.activate(step.vehIds)
    gameplay_traffic.scatterTraffic()
    step.complete = true
  end
end

local function makeStepSpawnTrafficSimple(amount, active, generator)
  return {
    name = "spawnTrafficStep",
    processTask = taskSpawnTrafficStep,
    options = {amount = amount, active = active, generator = generator},
    state = 0
  }
end
M.taskSpawnTrafficStep = taskSpawnTrafficStep
M.makeStepSpawnTrafficSimple = makeStepSpawnTrafficSimple



local function taskLoadLevelStep(step)
  if not step.waitForClientStartMission then
    local foundLevel = false
    for i, v in ipairs(core_levels.getList()) do
      if v.levelName:lower() == step.level:lower() then
        if string.find(v.fullfilename, '.mis') then
          core_levels.startLevel(v.fullfilename)
          foundLevel = true
        else
          core_levels.startLevel(path.getPathLevelMain(v.levelName))
          foundLevel = true
        end
      end
    end
    if not foundLevel then
      log("E","","Could not find level: " .. step.level)
    end
    step.waitForClientStartMission = true
  end
  return step.levelLoaded or false
end
M.onClientStartMission = function(state)
  if not taskData.active or not taskData.steps[taskData.currentStep] then
    return
  end
  taskData.steps[taskData.currentStep].complete = true
end

local function makeLoadLevelStep(level)
  return {
    name = "loadLevel",
    processTask = taskLoadLevelStep,
    level = level,
    timeout = 600
  }
end

M.taskLoadLevelStep = taskLoadLevelStep
M.makeLoadLevelStep = makeLoadLevelStep



-- starting a sequence
local function startStepSequence(steps, callbackWhenFinished)
  if callbackWhenFinished then
    table.insert(steps, makeStepReturnTrueFunction(function() callbackWhenFinished() return true end))
  end
  taskData = {
    steps = steps,
    data = {},
    active = true,
    currentStep = 1
  }
end
M.startStepSequence = startStepSequence

local function skipToLastStepOrCallback()
  taskData.steps[taskData.currentStep].complete = true
  taskData.currentStep = #taskData.steps - 1
end
M.skipToLastStepOrCallback = skipToLastStepOrCallback

local dtTable = {}
local function onUpdate(dtReal, dtSim, dtRaw)
  if showDebugWindow then
    local im = ui_imgui
    im.Begin("Step Handler Debug")
    im.Text("Steps")
    if not taskData.active then im.BeginDisabled() end
    for i, step in ipairs(taskData.steps) do
      im.TextWrapped(string.format("%s%d - %s",taskData.currentStep == i and "ACTIVE " or "", i, step.name or "Unnamed Step"))
      im.Text(dumps(step))
      if debugApprove and i==taskData.currentStep and step.complete then
        if im.Button("Approve##"..i) then
          step.approved = true
        end
      end
      im.Separator()
    end
    im.TextWrapped(dumpsz(taskData.data, 3))
    if not taskData.active  then im.EndDisabled() end
    im.End()
  end

  if taskData.active then
    local stepToHandle = taskData.steps[taskData.currentStep]
    while stepToHandle do
      if not stepToHandle._startingTime then stepToHandle._startingTime = os.time() end
      dtTable.dtReal = dtReal
      dtTable.dtSim = dtSim
      dtTable.dtRaw = dtRaw
      stepToHandle.processTask(stepToHandle, dtTable)
      if os.time() - stepToHandle._startingTime > (stepToHandle.timeout or 120) then
        log("E","","This step timed out ("..(stepToHandle.timeout or 120).."s). Step will be set to complete.")
        dump(stepToHandle)
        stepToHandle.complete = true
      end
      if stepToHandle.complete then
        log("I", logTag, string.format("Completed Step: %s", stepToHandle.name or "Unnamed Task"))
        taskData.currentStep = taskData.currentStep + 1
        stepToHandle = taskData.steps[taskData.currentStep]
        if not stepToHandle then
          taskData.active = false
        end
      else
        stepToHandle = nil
      end
    end
  end
end

M.onUpdate = onUpdate
return M