-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local floor = math.floor
local sqrt = math.sqrt
local max = math.max
local min = math.min

local constants = {
  rpmToAV = 0.10471971768,
  avToRPM = 9.5493,
  invPascalToPSI = 0.00014503773773,
  psiToPascal = 6894.757293178
}

M.isExisting = true
M.turboAV = nil

local assignedEngine = nil
local forcedInductionInfoStream = {
  rpm = 0,
  coef = 1,
  boost = 0,
  maxBoost = 0,
  exhaustPower = 0,
  friction = 0,
  backpressure = 0,
  bovEngaged = 0,
  wastegateFactor = 0,
  turboTemp = 0
}

--Turbo related stuff
local curTurboAV = 0
local maxTurboAV = 1
local invMaxTurboAV = 1
local invTurboInertia = 0
local turboInertiaFactor = 1
local turboPressure = 0
local turboPressureRaw = 0
local maxTurboPressure = 1
local maxExhaustPower = 1
local backPressureCoef = 0
local frictionCoef = 0
local turboWhineLoop = nil
local turboHissLoop = nil
local turboWhinePitchPerAV = 0
local turboWhineVolumePerAV = 0
local turboHissVolumePerPascal = 0
local wearFrictionCoef = 1
local damageFrictionCoef = 1
local damageExhaustPowerCoef = 1
local antilagMaxPower = 0
local antilagCoef = 0

local exhaustPower
local friction
local backPressure

-- Wastegate
local wastegateStart
local wastegateLimit
local wastegateFactor = 1
local wastegateRange
local maxWastegateStart = 0
local maxWastegateLimit = 0
local maxWastegateRange = 1
local wastegatePCoef = 0
local wastegateICoef = 0
local wastegateDCoef = 0
local wastegateIntegral = 0
local lastBoostError = 0
local wastegateTargetBoost = 0
local wastegateStartPerGear = 0
local wastegateRangePerGear = 0
local wastegateOffset = 0

-- blow off valve
local bovEnabled = true
local bovEngaged = false
local bovRequested = false
local lastBOVValue = false
local lastEngineLoad = 0
local bovOpenChangeThreshold = 0
local bovOpenThreshold = 0
local bovSoundVolumeCoef = 1
local turboSizeCoef = 1
local bovTimer = 0
local ignitionCutSmoother
local needsBov = false
local bovSound
local antilagCoefSmoother = newTemporalSmoothing(10, 1000)

local flutterSoundVolumeCoef = 1
local flutterSound

--Engine related stuff
local invEngMaxAV = 0

local turbo
local pressureSmoother
local wastegateSmoother
local electricsRPMName
local electricsSpinName
local electricsSpinCoef
local electricsSpinValue

--Damage
local turboDamageThresholdTemperature = 0

local function applyDeformGroupDamage(damageAmount)
  damageFrictionCoef = damageFrictionCoef + linearScale(damageAmount, 0, 0.01, 0, 1)
  damageExhaustPowerCoef = max(damageExhaustPowerCoef - linearScale(damageAmount, 0, 0.01, 0, 0.1), 0)
  damageTracker.setDamage("engine", "inductionSystemDamaged", true, true)
end

local function setPartCondition(odometer, integrity, visual)
  wearFrictionCoef = linearScale(odometer, 30000000, 1000000000, 1, 2)
  local integrityState = integrity
  if type(integrity) == "number" then
    local integrityValue = integrity
    integrityState = {
      damageFrictionCoef = linearScale(integrityValue, 1, 0, 1, 20),
      damageExhaustPowerCoef = linearScale(integrityValue, 1, 0, 1, 0.2)
    }
  end

  damageFrictionCoef = integrityState.damageFrictionCoef or 1
  damageExhaustPowerCoef = integrityState.damageExhaustPowerCoef or 1
end

local function getPartCondition()
  local integrityState = {
    damageFrictionCoef = damageFrictionCoef,
    damageExhaustPowerCoef = damageExhaustPowerCoef
  }

  local frictionIntegrityValue = linearScale(damageFrictionCoef, 1, 20, 1, 0)
  local exhaustPowerIntegrityValue = linearScale(damageExhaustPowerCoef, 1, 0.2, 1, 0)

  local integrityValue = min(frictionIntegrityValue, exhaustPowerIntegrityValue)
  return integrityValue, integrityState
end

local function updateSounds(dt)
  if turboWhineLoop then
    local spindlePitch = curTurboAV * turboWhinePitchPerAV
    local spindleVolume = curTurboAV * turboWhineVolumePerAV
    local hissVolume = max(turboPressure * turboHissVolumePerPascal, 0)
    obj:setVolumePitch(turboHissLoop, hissVolume, 1)
    obj:setVolumePitch(turboWhineLoop, spindleVolume, spindlePitch)

  -- Audio Debug
  -- streams.drawGraph('hiss volum', {value = hissVolume, max = 1})
  -- streams.drawGraph('spin volum', {value = spindleVolume, max = 1})
  -- streams.drawGraph('spin pitch', {value = spindlePitch, max = 1})
  -- if spindlePitch > 0 then print(string.format(" hiss volum = %0.2f / spin volum %.2f / spin pitch %.3f)", hissVolume, spindleVolume, spindlePitch)); end
  end
end

local function updateFixedStep(dt)
  if assignedEngine.engineDisabled then
    M.updateGFX = nop
    M.updateFixedStep = nop
    electrics.values.turboRpmRatio = 0
    electrics.values.turboBoost = 0
    turboPressure = 0
    turboPressureRaw = 0
    curTurboAV = 0
    return
  end
  --calculate wastegate factor
  local boostError = turboPressureRaw - wastegateTargetBoost
  wastegateIntegral = clamp(wastegateIntegral + boostError * dt, -50, 500)
  local wastegateDerivative = (boostError - lastBoostError) / dt
  wastegateFactor = bovEngaged and 0 or wastegateSmoother:getUncapped(clamp((1 - (boostError * wastegatePCoef + wastegateIntegral * wastegateICoef + wastegateDerivative * wastegateDCoef)), 0, 1), dt)

  local engAV = max(1, assignedEngine.outputAV1)
  local engAvRatio = min(engAV * invEngMaxAV, 1)
  local engineRPM = floor(max(assignedEngine.outputRPM or 0, 0))

  --Torque on the turbo's axis
  local throttle = clamp(assignedEngine.requestedThrottle, 0, 1)
  exhaustPower = (0.1 + assignedEngine.exhaustFlowCoef * 0.8) * throttle * throttle * engAvRatio * (turbo.turboExhaustCurve[engineRPM] or 0) * maxExhaustPower * damageExhaustPowerCoef * dt
  friction = frictionCoef * wearFrictionCoef * damageFrictionCoef * dt --simulate some friction and stuff there
  local bovBackPressureCoef = bovRequested and (bovEnabled and 0.4 or 2) or 1 --while bov logic is active, use a small backpressure coef for cars with bov and a large one without a bov
  backPressure = curTurboAV * curTurboAV * backPressureCoef * bovBackPressureCoef * dt --back pressure from compressing the air
  local antilagPower = antilagCoef * antilagMaxPower * dt
  local turboTorque = (((exhaustPower + antilagPower) * wastegateFactor) - backPressure - friction)

  --calculate angular velocity
  curTurboAV = clamp((curTurboAV + dt * turboTorque * invTurboInertia), 0, maxTurboAV)
  M.turboAV = curTurboAV

  local turboRPM = curTurboAV * constants.avToRPM
  turboPressureRaw = assignedEngine.isStalled and 0 or ((turbo.turboPressureCurve[floor(turboRPM)] * constants.psiToPascal) or turboPressure)
  turboPressure = pressureSmoother:getUncapped(turboPressureRaw, dt)

  -- 1 psi = 6% more power
  -- 1 pascal = 0.00087% more power
  assignedEngine.forcedInductionCoef = assignedEngine.forcedInductionCoef * (1 + 0.0000087 * turboPressure * (turbo.turboEfficiencyCurve[engineRPM] or 0))
end

local function updateGFX(dt)
  --Some verification stuff
  if assignedEngine.engineDisabled then
    M.updateGFX = nop
    M.updateFixedStep = nop
    electrics.values.turboRpmRatio = 0
    electrics.values.turboBoost = 0
    turboPressure = 0
    turboPressureRaw = 0
    curTurboAV = 0
    return
  end

  --calculate an arbitary "turbo temp" that reflects the effects of oil and coolant cooling on the actual temps inside the turbo
  local turboTemp = assignedEngine.thermals.exhaustTemperature + (assignedEngine.thermals.coolantTemperature or 0) + assignedEngine.thermals.oilTemperature
  --calculate turbo damage using our turbo temp
  if turboTemp > turboDamageThresholdTemperature then
    damageFrictionCoef = damageFrictionCoef * (1 + (turboTemp - turboDamageThresholdTemperature) * 0.001 * dt)
    damageTracker.setDamage("engine", "turbochargerHot", true)
  else
    damageTracker.setDamage("engine", "turbochargerHot", false)
  end

  --smooth the antilagCoef to not trigger BOV when switching from antilag to full throttle
  local smoothedAntilagCoef = antilagCoefSmoother:get(antilagCoef, dt)
  --engine load needs to take antilag coef into account for correct BOV operation
  local engineLoad = max(assignedEngine.instantEngineLoad, smoothedAntilagCoef)
  --open the BOV if we have very little load or if the engine load drops significantly
  local loadLow = (engineLoad < bovOpenThreshold or (max(assignedEngine.requestedThrottle, smoothedAntilagCoef) <= 0))
  local highLoadDrop = (lastEngineLoad - engineLoad) > bovOpenChangeThreshold
  local notInRevLimiter = assignedEngine.revLimiterWasActiveTimer > 0.1
  local ignitionNotCut = ignitionCutSmoother:getUncapped(assignedEngine.ignitionCutTime > 0 and 1 or 0, dt) <= 0
  bovRequested = needsBov and (loadLow or highLoadDrop) and notInRevLimiter and ignitionNotCut
  bovEngaged = bovEnabled and bovRequested

  bovTimer = max(bovTimer - dt, 0)
  if bovRequested and needsBov and not lastBOVValue and bovTimer <= 0 then
    local relativePressure = min(max(turboPressure / maxTurboPressure, 0), 1)
    if bovEnabled then
      -- Audio Debug
      -- print (string.format(" BOV Volume(pressure) = %0.2f (= relativePressure %.2f * turboSizeCoef %.2f) :::: color(bovSoundVolumeCoef) = %0.2f ", relativePressure * turboSizeCoef, relativePressure, turboSizeCoef, bovSoundVolumeCoef).." :::: "..turbo.bovSoundFileName)
      bovSound = bovSound or obj:createSFXSource2(turbo.bovSoundFileName or "event:>Vehicle>Forced_Induction>Turbo_01>turbo_bov", "AudioDefaultLoop3D", "Bov", assignedEngine.engineNodeID, 0)
      obj:setVolumePitchCT(bovSound, relativePressure * turboSizeCoef, 1, bovSoundVolumeCoef, 0)
      obj:cutSFX(bovSound)
      obj:playSFX(bovSound)
    else
      -- Audio Debug
      -- print (string.format("FLUT Volume(pressure) = %0.2f (= relativePressure %.2f * turboSizeCoef %.2f) :::: color(flutterSoundVolumeCoef) = %0.2f", relativePressure * turboSizeCoef, relativePressure, turboSizeCoef, flutterSoundVolumeCoef).." :::: "..turbo.flutterSoundFileName)
      flutterSound = flutterSound or obj:createSFXSource2(turbo.flutterSoundFileName or "event:>Vehicle>Forced_Induction>Turbo_02>turbo_bov", "AudioDefaultLoop3D", "Flutter", assignedEngine.engineNodeID, 0)
      obj:setVolumePitchCT(flutterSound, relativePressure * turboSizeCoef, 1, flutterSoundVolumeCoef, 0)
      obj:cutSFX(flutterSound)
      obj:playSFX(flutterSound)
    end
    bovTimer = 0.5
  end

  --read gear specific wastegate values at gfx step, no need to have it in fixed step
  local gear = electrics.values.gearIndex or 1
  wastegateStartPerGear = wastegateStart[gear] or maxWastegateStart
  wastegateRangePerGear = wastegateRange[gear] or maxWastegateRange
  wastegateTargetBoost = (wastegateStartPerGear + wastegateRangePerGear * 0.5) + wastegateOffset

  if bovRequested then --if the BOV is supposed to be open and we have positive pressure, we don't actually have any pressure ;)
    if bovEnabled then
      turboPressure = pressureSmoother:getUncapped(0, dt)
    end
  elseif lastBOVValue then
    if bovSound then
      obj:stopSFX(bovSound)
    end
    if flutterSound then
      obj:stopSFX(flutterSound)
    end
  end

  local turboRPM = curTurboAV * constants.avToRPM
  electrics.values[electricsRPMName] = turboRPM
  electricsSpinValue = electricsSpinValue + turboRPM * dt
  electrics.values[electricsSpinName] = (electricsSpinValue * electricsSpinCoef) % 360
  -- Update sounds
  electrics.values.turboRpmRatio = curTurboAV * invMaxTurboAV * 580
  electrics.values.turboBoost = turboPressure * constants.invPascalToPSI

  lastEngineLoad = assignedEngine.instantEngineLoad
  lastBOVValue = bovRequested

  -- Update streams
  if streams.willSend("forcedInductionInfo") then
    forcedInductionInfoStream.rpm = curTurboAV * constants.avToRPM
    forcedInductionInfoStream.coef = assignedEngine.forcedInductionCoef
    forcedInductionInfoStream.boost = electrics.values.boost * constants.psiToPascal * 0.001
    forcedInductionInfoStream.maxBoost = electrics.values.boostMax * constants.psiToPascal * 0.001
    forcedInductionInfoStream.exhaustPower = exhaustPower / dt
    forcedInductionInfoStream.backpressure = backPressure / dt
    forcedInductionInfoStream.friction = friction / dt
    forcedInductionInfoStream.bovEngaged = (bovEngaged and 1 or 0)
    forcedInductionInfoStream.wastegateFactor = wastegateFactor
    forcedInductionInfoStream.turboTemp = turboTemp
    forcedInductionInfoStream.antilagCoef = antilagCoef

    gui.send("forcedInductionInfo", forcedInductionInfoStream)
  end
end

local function reset(jbeamData)
  M.turboAV = 0
  curTurboAV = 0
  turboPressure = 0
  turboPressureRaw = 0
  bovEngaged = false
  lastBOVValue = true
  lastEngineLoad = 0
  wastegateFactor = 1
  bovTimer = 0
  wastegateIntegral = 0
  lastBoostError = 0
  wastegateTargetBoost = 0
  wastegateStartPerGear = 0
  wastegateRangePerGear = 0
  electricsSpinValue = 0
  antilagCoef = 0
  exhaustPower = 0
  backPressure = 0
  friction = 0

  wearFrictionCoef = 1
  damageFrictionCoef = 1
  damageExhaustPowerCoef = 1

  pressureSmoother:reset()
  wastegateSmoother:reset()
  ignitionCutSmoother:reset()
  antilagCoefSmoother:reset()

  damageTracker.setDamage("engine", "turbochargerHot", false)
  damageTracker.setDamage("engine", "turbochargerDamaged", false)
end

local function init(device, jbeamData)
  turbo = deepcopy(jbeamData)
  if turbo == nil then
    M.turboUpdate = nop
    return
  end

  assignedEngine = device

  --log("D", "Turbo", "Initializing turbo subsystem")

  M.turboAV = 0
  curTurboAV = 0
  turboPressure = 0
  turboPressureRaw = 0
  bovEngaged = false
  lastBOVValue = true
  lastEngineLoad = 0
  wastegateFactor = 1
  bovTimer = 0
  wastegateIntegral = 0
  lastBoostError = 0
  wastegateTargetBoost = 0
  wastegateStartPerGear = 0
  wastegateRangePerGear = 0
  antilagCoef = 0

  maxTurboAV = 1
  local maxPossiblePressure = 0

  -- add the turbo pressure curve
  -- define y PSI at x RPM
  local pressurePSIcount = #turbo.pressurePSI
  local tpoints = table.new(pressurePSIcount, 0)
  if turbo.pressurePSI then
    for i = 1, pressurePSIcount do
      local point = turbo.pressurePSI[i]
      tpoints[i] = {point[1], point[2]}
      --Get max turbine rpm
      maxTurboAV = max(point[1] * constants.rpmToAV, maxTurboAV)
      maxPossiblePressure = max(maxPossiblePressure, point[2] * constants.psiToPascal)
    end
  else
    log("E", "Turbo", "No turbocharger.pressurePSI table found!")
    return
  end
  turbo.turboPressureCurve = createCurve(tpoints, true)

  -- add the turbo exhaust curve
  -- simulate pressure factor going between the exhasut and the turbine
  --
  -- add the turbo efficiency curve
  -- simulate power coef per engine RPM
  -- Eg: Small turbos will be more efficient on engine low rpm than high rpm and vice versa
  local engineDefcount = #turbo.engineDef
  local tepoints = table.new(engineDefcount, 0)
  local tipoints = table.new(engineDefcount, 0)
  if turbo.engineDef then
    for i = 1, engineDefcount do
      local point = turbo.engineDef[i]
      tepoints[i] = {point[1], point[2]}
      tipoints[i] = {point[1], min(point[3], 1)}
    end
  else
    log("E", "Turbo", "No turbocharger.engineDef curve found!")
    return
  end
  turbo.turboExhaustCurve = createCurve(tipoints)
  turbo.turboEfficiencyCurve = createCurve(tepoints)

  turboInertiaFactor = (turbo.inertia * 100) or 1

  wastegateStart = {}
  maxWastegateStart = 0
  maxWastegateLimit = 1
  if type(turbo.wastegateStart) == "table" then
    for k, v in pairs(turbo.wastegateStart) do
      wastegateStart[k] = v * constants.psiToPascal
      maxWastegateStart = wastegateStart[k]
    end
  else
    wastegateStart[1] = (turbo.wastegateStart or 0) * constants.psiToPascal
    maxWastegateStart = wastegateStart[1]
  end

  wastegateLimit = {}
  if type(turbo.wastegateLimit) == "table" then
    for k, v in pairs(turbo.wastegateLimit) do
      wastegateLimit[k] = v * constants.psiToPascal
      maxWastegateLimit = wastegateLimit[k]
    end
  elseif type(turbo.wastegateLimit) == "number" then
    wastegateLimit[1] = (turbo.wastegateLimit or 0) * constants.psiToPascal
    maxWastegateLimit = wastegateLimit[1]
  else
    for k, v in pairs(wastegateStart) do
      wastegateLimit[k] = v + 0.01 * constants.psiToPascal
      maxWastegateLimit = wastegateLimit[k]
    end
  end

  wastegateRange = {}
  maxWastegateRange = 1
  for k, v in pairs(wastegateStart) do
    local start = v
    local limit = wastegateLimit[k] or maxWastegateLimit
    wastegateRange[k] = limit - start
    maxWastegateRange = wastegateRange[k]
  end

  maxExhaustPower = turbo.maxExhaustPower or 1

  backPressureCoef = turbo.backPressureCoef or 0.0005
  frictionCoef = turbo.frictionCoef or 0.01

  antilagMaxPower = turbo.maxAntilagPower or 0

  turboDamageThresholdTemperature = turbo.damageThresholdTemperature or 1000

  wastegatePCoef = turbo.wastegatePCoef or 0.0001
  wastegateICoef = turbo.wastegateICoef or 0.0015
  wastegateDCoef = turbo.wastegateDCoef or 0.00

  electricsRPMName = turbo.electricsRPMName or "turboRPM"
  electricsSpinName = turbo.electricsSpinName or "turboSpin"
  electricsSpinCoef = turbo.electricsSpinCoef or 0.1
  electricsSpinValue = 0

  --optimizations:
  invMaxTurboAV = 1 / maxTurboAV
  invEngMaxAV = 1 / ((assignedEngine.maxRPM or 8000) * constants.rpmToAV)
  invTurboInertia = 1 / (0.000003 * turboInertiaFactor * 2.5)
  pressureSmoother = newTemporalSmoothing(200 * constants.psiToPascal, (turbo.pressureRatePSI or 30) * constants.psiToPascal)
  wastegateSmoother = newTemporalSmoothing(50, 50)
  ignitionCutSmoother = newTemporalSmoothing(1, 10)
  bovEnabled = (turbo.bovEnabled == nil or turbo.bovEnabled)
  bovOpenThreshold = turbo.bovOpenThreshold or 0.05
  bovOpenChangeThreshold = turbo.bovOpenChangeThreshold or 0.3
  needsBov = assignedEngine.requiredEnergyType ~= "diesel"
  maxTurboPressure = min(maxWastegateStart * constants.invPascalToPSI * (1 + (maxWastegateRange * constants.invPascalToPSI) * 0.01) * constants.psiToPascal, maxPossiblePressure) --limit this to what the current turbo can actually deliver

  forcedInductionInfoStream.friction = frictionCoef
  --forcedInductionInfoStream.maxBoost = maxWastegateLimit * 0.001
  electrics.values.turboBoostMax = maxWastegateLimit * constants.invPascalToPSI

  wearFrictionCoef = 1
  damageFrictionCoef = 1
  damageExhaustPowerCoef = 1

  damageTracker.setDamage("engine", "turbochargerHot", false)
  damageTracker.setDamage("engine", "turbochargerDamaged", false)

  M.updateGFX = updateGFX
  M.updateFixedStep = updateFixedStep
  M.updateSounds = updateSounds
end

local function initSounds(jbeamData)
  local turboHissLoopFilename = turbo.hissLoopEvent or "event:>Vehicle>Forced_Induction>Turbo_01>turbo_hiss"
  turboHissLoop = obj:createSFXSource(turboHissLoopFilename, "AudioDefaultLoop3D", "TurbochargerWhine", assignedEngine.engineNodeID)
  local turboWhineLoopFilename = turbo.whineLoopEvent or "event:>Vehicle>Forced_Induction>Turbo_01>turbo_spin"
  turboWhineLoop = obj:createSFXSource(turboWhineLoopFilename, "AudioDefaultLoop3D", "TurbochargerWhine", assignedEngine.engineNodeID)

  assignedEngine:setSoundLocation("turbochargerwhine", "Turbo Hiss: " .. turboHissLoopFilename, {assignedEngine.engineNodeID})
  assignedEngine:setSoundLocation("turbochargerhiss", "Turbo Whine: " .. turboWhineLoopFilename, {assignedEngine.engineNodeID})

  turboWhinePitchPerAV = (turbo.whinePitchPer10kRPM or 0.05) * 0.01 * constants.rpmToAV
  turboWhineVolumePerAV = (turbo.whineVolumePer10kRPM or 0.04) * 0.01 * constants.rpmToAV
  turboHissVolumePerPascal = (turbo.hissVolumePerPSI or 0.04) * constants.invPascalToPSI

  turboSizeCoef = turbo.turboSizeCoef or 1
  bovSoundVolumeCoef = turbo.bovSoundVolumeCoef or 0.3
  flutterSoundVolumeCoef = turbo.flutterSoundVolumeCoef or 0.3

  -- Audio Debug
  -- print (string.format("Turbo Hiss and Whine", turbo.hissLoopEvent).." : "..turbo.hissLoopEvent.." : "..turbo.whineLoopEvent)
  -- print (string.format("turbo.hissVolumePerPSI = %.3f : turbo.whineVolumePer10kRPM = %.3f : turbo.whinePitchPer10kRPM = %.3f", turbo.hissVolumePerPSI, turbo.whineVolumePer10kRPM, turbo.whinePitchPer10kRPM))
end

local function resetSounds(jbeamData)
end

local function getTorqueCoefs()
  local coefs = {}
  --we can't know the actual wastegate limit for sure since it's a feedback loop with the pressure, so we just estimate it.
  --lower wastegate ranges lead to more accurate results.
  --local estimatedWastegateLimit = maxWastegateStart * invPascalToPSI * (1 + (maxWastegateRange * invPascalToPSI) * 0.03)
  local estimatedWastegateLimit = (maxWastegateStart + maxWastegateRange * 0.5) * constants.invPascalToPSI

  for k, _ in pairs(assignedEngine.torqueCurve) do
    if type(k) == "number" and k < assignedEngine.maxRPM then
      local rpm = floor(k)
      local turboAV = sqrt(max((0.9 * rpm * constants.rpmToAV * invEngMaxAV * (turbo.turboExhaustCurve[rpm] or 1) * maxExhaustPower * damageExhaustPowerCoef - frictionCoef * wearFrictionCoef * damageFrictionCoef), 0) / backPressureCoef)
      turboAV = min(turboAV, maxTurboAV)
      local turboRPM = floor(turboAV * constants.avToRPM)
      local pressure = turbo.turboPressureCurve[turboRPM] or 0 --pressure without respecting the wastegate
      local actualPressure = min(pressure, estimatedWastegateLimit) --limit the pressure to what the wastegate allows
      coefs[k + 1] = (1 + 0.0000087 * actualPressure * constants.psiToPascal * (turbo.turboEfficiencyCurve[rpm] or 0))
    end
  end

  return coefs
end

local function setWastegateOffset(offset)
  wastegateOffset = offset * constants.psiToPascal
end

local function setAntilagCoef(coef)
  antilagCoef = coef
end

-- public interface
M.init = init
M.initSounds = initSounds
M.resetSounds = resetSounds
M.updateSounds = nop
M.reset = reset
M.updateGFX = nop
M.updateFixedStep = nop
M.getTorqueCoefs = getTorqueCoefs
M.setWastegateOffset = setWastegateOffset
M.setAntilagCoef = setAntilagCoef

M.applyDeformGroupDamage = applyDeformGroupDamage
M.setPartCondition = setPartCondition
M.getPartCondition = getPartCondition

return M
