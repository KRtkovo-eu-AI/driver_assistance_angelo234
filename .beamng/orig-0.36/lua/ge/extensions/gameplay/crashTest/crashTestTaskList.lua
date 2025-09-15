-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {
  'gameplay_crashTest_scenarioManager'
}

local crashTestData = nil

local function getIdForStep(stepData)
  return stepData.plVehId
end

local function setHeader()
  guihooks.trigger("SetTasklistHeader", {
    label = "Crash analysis steps",
    subtext = ""
  })
end

local function buildTaskLabel(stepData)
  local string = "Step " .. stepData.id .. ": "
  if stepData.objective == "Simple crash" then

    string = string .. "Crash into the marked "
    if stepData.targetType == "A single Jbeam" or stepData.targetType == "Static object" then
      string = string .. "target"
    elseif stepData.targetType == "Multiple Jbeams" then
      string = string .. "targets"
    end

    return string
  elseif stepData.objective == "Crash target vehicle into target area" then
    return string .. "Redirect the target into the highlighted area"
  end
end

local function buildTaskSubtext(stepData)
  local string = ""
  if stepData.objective == "Simple crash" then
    local vel, unit = translateVelocity(stepData.impactSpeed/3.6, true)
    string = string .. "At " .. vel .. " " .. unit
  end

  if stepData.isThereDamageAssessment then
    if stepData.damageAmount == "Minor" and stepData.damageCondition == "No more than" then
      string = string .. "Minimize damage to your vehicle"
    elseif stepData.damageAmount ~= "Minor" and stepData.damageCondition == "No more than" then
      string = string .. "Not more than " .. stepData.damageAmount .. " damage"
    elseif stepData.damageAmount == "A lot" and stepData.damageCondition == "At least" then
      string = string .. "Maximize damage"
    elseif stepData.damageAmount ~= "A lot" and stepData.damageCondition == "At least" then
      string = string .. "At least " .. stepData.damageAmount .. " damage"
    end
  end
  return string
end

local function clearTasklist()
  for _, stepData in ipairs(crashTestData) do
    guihooks.trigger("SetTasklistTask", {
      clear = true,
      id = getIdForStep(stepData),
    })
  end
end

local function resetTasklist()
  for _, stepData in ipairs(crashTestData) do
    guihooks.trigger("SetTasklistTask", {
      clear = false,
      label = buildTaskLabel(stepData),
      subtext = buildTaskSubtext(stepData),
      done = false,
      fail = false,
      active = false,
      id = getIdForStep(stepData),
      type = "goal",
      attention = true,
    })
  end
end

local function setFirstTask()
  guihooks.trigger("SetTasklistTask", {
    clear = false,
    label = buildTaskLabel(crashTestData[1]),
    subtext = buildTaskSubtext(crashTestData[1]),
    done = false,
    fail = false,
    active = false,
    id = getIdForStep(crashTestData[1]),
    type = "goal",
    attention = true,
  })
end

local function setCrashTestDataAndInit(crashTestData_)
  crashTestData = crashTestData_

  setHeader()
  setFirstTask()
end

local function onNewCrashTestStep(stepData)
  guihooks.trigger("SetTasklistTask", {
    clear = false,
    label = buildTaskLabel(stepData),
    subtext = buildTaskSubtext(stepData),
    done = false,
    fail = false,
    active = true,
    id = getIdForStep(stepData),
    type = "goal",
    attention = true,
  })
end

local function onCrashTestStepFailed(stepData)
  if not stepData then return end

  guihooks.trigger("SetTasklistTask", {
    clear = false,
    label = buildTaskLabel(stepData),
    subtext = buildTaskSubtext(stepData),
    done = false,
    fail = true,
    active = true,
    id = getIdForStep(stepData),
    type = "goal",
    attention = false,
  })
end

local function finishedCurrentTask(stepData)
  guihooks.trigger("SetTasklistTask", {
    clear = false,
    label = buildTaskLabel(stepData),
    subtext = buildTaskSubtext(stepData),
    done = true,
    fail = false,
    active = true,
    id = getIdForStep(stepData),
    type = "goal",
    attention = false,
  })
end

local function onExtensionUnloaded()
  clearTasklist()
end

local function reset()
  clearTasklist()
end

M.onExtensionUnloaded = onExtensionUnloaded

M.onCrashTestStepFailed = onCrashTestStepFailed
M.finishedCurrentTask = finishedCurrentTask
M.onNewCrashTestStep = onNewCrashTestStep

M.setCrashTestDataAndInit = setCrashTestDataAndInit
M.reset = reset
return M
