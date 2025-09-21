-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local scoringDataPerStep = {}
local M = {}
M.dependencies = {
  "gameplay_util_damageAssessment",
}

local scoreParameters = {
  maxDamageLocationScore = 1000,

  maxTimeScore = 1000,
  minTimeScore = 500,

  maxSpeedScore = 1000,
  maxSpeedOffset = 3, -- that means if offset > 3 kph, then speed score is 0

  sameStepDamageScore = 1000, -- max score if damage is the same
  oneStepDifferentDamageScore = 500,
  twoStepsDifferentDamageScore = 200,

  succesfullyCrashedTargetInZoneScore = 1000,
}

local damageStepToId = {
  ["Minor"] = 1,
  ["Moderate"] = 2,
  ["A lot"] = 3,
}

local function calculateTimeScore(timeGoal, timeElapsed)
  local roundedTime = math.floor(timeElapsed * 10 + 0.5) / 10 -- Round to one decimal place
  local score = scoreParameters.minTimeScore
  if timeElapsed <= timeGoal then
    score = scoreParameters.maxTimeScore
  elseif timeElapsed > timeGoal and timeElapsed <= timeGoal + timeGoal / 100 * 30 then
    score = linearScale(timeElapsed, timeGoal, timeGoal + timeGoal / 100 * 30, scoreParameters.maxTimeScore, scoreParameters.minTimeScore)
  end
  return {score = math.ceil(score), scoreName = "Time to Impact Score", maxScore = scoreParameters.maxTimeScore, timeToImpact = {text = "Time to impact", value = roundedTime, unit = "s"}}
end

local function calculateSpeedScore(speedGoal, speed)
  local trSpeed = translateVelocity(speed / 3.6, true)
  local trSpeedGoal = translateVelocity(speedGoal / 3.6, true)
  local trSpeedOffset, unit = translateVelocity(scoreParameters.maxSpeedOffset / 3.6, true)

  local roundedSpeed = math.floor(trSpeed + 0.5) -- Round to nearest integer
  local speedDiff = math.abs(roundedSpeed - trSpeedGoal)
  local score = linearScale(speedDiff, 0, trSpeedOffset, scoreParameters.maxSpeedScore, 0)
  return {score = math.ceil(score), scoreName = "Impact Speed Score", maxScore = scoreParameters.maxSpeedScore, targetSpeed = {text = "Required impact speed", value = trSpeedGoal, unit = unit} , actualSpeed = {text = "Registered impact speed", value = roundedSpeed, unit = unit}, diff = {text = "Speed difference", value = speedDiff, unit = unit}}
end

local function calculateDamageLocationScore(damageLocationGoal, damageLocationData)
  local totalDamageNonLocationGoal = damageLocationData.totalDamage
  local damageLocation = damageLocationData.damagedLocations[damageLocationGoal] or 0

  local score = damageLocation / totalDamageNonLocationGoal * scoreParameters.maxDamageLocationScore
  local precision = damageLocationData.damagedLocations[damageLocationData.mostDamagedLocation] / totalDamageNonLocationGoal * 100
  return {score = math.ceil(score), scoreName = "Impact Accuracy Score", maxScore = scoreParameters.maxDamageLocationScore, requiredImpactLocation = {text = "Required impact location", value = damageLocationGoal}, actualImpactLocation = {text = "Majority of impact location", value = damageLocationData.mostDamagedLocation, precision = string.format("at %i %%", precision)}}
end

local function calculateIndividualDamageScore(damageGoal, damageCondition, vehId)
  local damageGoalId = damageStepToId[damageGoal]
  local damageAssessmentId = gameplay_util_damageAssessment.getDamageAssessment(vehId).damageSeverity
  local diff = damageGoalId - damageAssessmentId

  if damageCondition == "At least" then
    if diff <= 0 then
      return scoreParameters.sameStepDamageScore
    elseif diff == 1 then
      return scoreParameters.oneStepDifferentDamageScore
    elseif diff == 2 then
      return scoreParameters.twoStepsDifferentDamageScore
    else
      return 0
    end
  elseif damageCondition == "No more than" then
    if diff >= 0 then
      return scoreParameters.sameStepDamageScore
    elseif diff == -1 then
      return scoreParameters.oneStepDifferentDamageScore
    elseif diff == -2 then
      return scoreParameters.twoStepsDifferentDamageScore
    else
      return 0
    end
  end
end

local function calculateDamageScore(stepData)
  if stepData.isThereDamageAssessment then
    local vehIds = {}
    if stepData.damageTargets == "Player" then
      table.insert(vehIds, stepData.plVehId)
    elseif stepData.damageTargets == "Target(s)" then
      for _, targetId in ipairs(stepData.jbeamTargets) do
        table.insert(vehIds, targetId)
      end
    elseif stepData.damageTargets == "Player and target(s)" then
      table.insert(vehIds, stepData.plVehId)
      for _, targetId in ipairs(stepData.jbeamTargets) do
        table.insert(vehIds, targetId)
      end
    end

    local totalDamageScore = 0
    for _, vehId in ipairs(vehIds) do
      totalDamageScore = totalDamageScore + calculateIndividualDamageScore(stepData.damageAmount, stepData.damageCondition, vehId)
    end
    return {score = totalDamageScore / #vehIds, name = "Damage assessment score"}
  end
end

-- onoly triggered if the step is successful
local function calculateStepScore(stepData, stepPlayerCrashes, stepTargetJbeamsCrashed)
  local stepScoreData = {}
  if stepData.objective ~= "Crash target vehicle into target area" then
    if stepData.timeToImpact ~= nil then
      stepScoreData["timeScore"] = calculateTimeScore(stepData.timeToImpact, stepPlayerCrashes[1].stepElapsedTime)
    end
    stepScoreData["speedScore"] = calculateSpeedScore(stepData.impactSpeed, stepPlayerCrashes[1].crashStartData.crashSpeed)
  end
  if stepData.isThereDamageAssessment then
    stepScoreData["damageScore"] = calculateDamageScore(stepData)
  end
  if stepData.isThereImpactLocation then
    local t = next(stepTargetJbeamsCrashed)
    dump(t)
    stepScoreData["damageLocationScore"] = calculateDamageLocationScore(stepData.impactLocation, stepData.impactLocationSubject == "Player" and stepPlayerCrashes[1].crashEndData.impactLocationData or stepTargetJbeamsCrashed[next(stepTargetJbeamsCrashed)].damageLocationData)
  end
  if stepData.objective == "Crash target vehicle into target area" then
    stepScoreData["succesfullyCrashedTargetInZoneScore"] = {score = scoreParameters.succesfullyCrashedTargetInZoneScore, name = "Succesfully crashed target in the target area"}
  end
  table.insert(scoringDataPerStep, stepScoreData)
end

local function getTotalScoreData()
  local totalScore = 0
  for _, stepScoreData in ipairs(scoringDataPerStep) do
    for _, scoreData in pairs(stepScoreData) do
      totalScore = totalScore + scoreData.score
    end
  end
  scoringDataPerStep["totalScore"] = totalScore
  return scoringDataPerStep
end

local function getStepScoreData(stepIndex)
  return scoringDataPerStep[stepIndex]
end

local function reset()
  scoringDataPerStep = {}
end

M.calculateIndividualDamageScore = calculateIndividualDamageScore
M.calculateDamageLocationScore = calculateDamageLocationScore

M.calculateStepScore = calculateStepScore
M.getTotalScoreData = getTotalScoreData
M.getStepScoreData = getStepScoreData

M.reset = reset
return M
