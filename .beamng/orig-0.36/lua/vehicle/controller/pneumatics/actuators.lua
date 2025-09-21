-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.type = "auxiliary"
M.defaultOrder = 50

local abs = math.abs
local max = math.max
local min = math.min
local sqrt = math.sqrt
local clamp = clamp

local psiToPascal = 6894.757293178
local pascalToPSI = 1 / psiToPascal
local gasConstant = 8.314 -- J/(mol * K)
local tinyPressureDiffThreshold = 25

-- dependent on shape of flow orifice - closer to 1.0 for rounded orifices, decreases with sharpness of edges
-- we use a constant value of 0.97, which is a decent approximation of a round hole (e.g. a tube connection) without being unrealistically close to 1.0
local dischargeCoefficient = 0.97

local airTank = nil
local beamGroups = nil
local crossFlowGroups = {}
local enableCrossFlow = false
local defaultVirtualBufferCapacity = 0
local pressureDumpFlowRate = 0
local averagePressure = 0
local totalBeamCount = 0
local enableDebug = false

-- TODO: Compare internal calculated pressure with actual internal beam pressure (should match)

local function updateBeamAggregates()
  averagePressure = 0
  totalBeamCount = 0

  for _, g in pairs(beamGroups) do
    g.averagePressure = 0
    for _, b in ipairs(g.beams) do
      g.averagePressure = g.averagePressure + b.currentPressure
      averagePressure = averagePressure + b.currentPressure
      totalBeamCount = totalBeamCount + 1
    end
    g.averagePressure = g.averagePressure * g.invBeamCount
  end

  if totalBeamCount > 0 then
    averagePressure = averagePressure / totalBeamCount
  end
end

--- Sets the pressure of all cross-flow group buffers to the average pressure of all participating beam groups
local function resetVirtualBuffers()
  -- first, initialize the pressure and group count to 0
  for _, g in pairs(crossFlowGroups) do
    g.bufferPressure = 0
    g.beamGroupCount = 0
  end

  -- now calculate the sum of all beam groups' pressures and the number of beam groups in each cross-flow group
  for _, g in pairs(beamGroups) do
    local crossFlowGroup = crossFlowGroups[g.crossFlowTag]

    if crossFlowGroup then
      crossFlowGroup.bufferPressure = crossFlowGroup.bufferPressure + g.averagePressure
      crossFlowGroup.beamGroupCount = crossFlowGroup.beamGroupCount + 1
    end
  end

  -- finally, normalize the sum to an average using the beam group count, and calculate the stored energy
  for _, g in pairs(crossFlowGroups) do
    g.bufferPressure = g.beamGroupCount == 0 and 0 or g.bufferPressure / g.beamGroupCount
    g.bufferStoredEnergy = g.bufferPressure * g.bufferCapacity
  end
end

--- Sets the pressure of a single cross-flow group's buffer to the average pressure of all participating beam groups
local function resetVirtualBuffer(crossFlowTag)
  local g = crossFlowGroups[crossFlowTag]

  if not g then
    return
  end

  -- first, initialize the pressure and group count to 0
  g.bufferPressure = 0
  g.beamGroupCount = 0

  -- now calculate the sum of all beam groups' pressures and the number of beam groups in each cross-flow group
  for _, bg in pairs(beamGroups) do
    if bg.crossFlowTag == crossFlowTag then
      g.bufferPressure = g.bufferPressure + bg.averagePressure
      g.beamGroupCount = g.beamGroupCount + 1
    end
  end

  -- finally, normalize the sum to an average using the beam group count, and calculate the stored energy
  g.bufferPressure = g.beamGroupCount == 0 and 0 or g.bufferPressure / g.beamGroupCount
  g.bufferStoredEnergy = g.bufferPressure * g.bufferCapacity
end

local function setBeamPressureCore(cid, pressure, maxPressure, spring, damp)
  obj:setBeamPressureRel(cid, max(0, pressure - powertrain.currentEnvPressure), max(0, maxPressure - powertrain.currentEnvPressure), spring, damp)
end

local function setBeamGroupValveState(groupName, valveState)
  local groupData = beamGroups[groupName]
  if not groupData then
    log("W", "actuators.setBeamGroupValveState", "Can't find pressure beam group: " .. groupName)
  else
    groupData.valveState = clamp(valveState, -1, 1)
  end
end

local function setBeamGroupsValveState(groupNames, valveState)
  for _, g in pairs(groupNames) do
    setBeamGroupValveState(g, valveState)
  end
end

local function toggleBeamGroupValveState(groupName)
  local groupData = beamGroups[groupName]
  if not groupData then
    log("W", "actuators.toggleBeamGroupValveState", "Can't find pressure beam group: " .. groupName)
  else
    local currentValveState = groupData.valveState or 0
    local valveState = currentValveState < 0 and 1 or -1
    groupData.valveState = valveState
  end
end

local function toggleBeamGroupsValveState(groupNames)
  for _, g in pairs(groupNames) do
    toggleBeamGroupValveState(g)
  end
end

local function setBeamGroupMaximumSupplyPressure(groupName, maxSupplyPressure)
  local groupData = beamGroups[groupName]
  if not groupData then
    log("W", "actuators.setBeamGroupMaximumSupplyPressure", "Can't find pressure beam group: " .. (groupName or "nil"))
  else
    groupData.maxSupplyPressure = maxSupplyPressure
  end
end

local function setBeamGroupsMaximumSupplyPressure(groupNames, maxSupplyPressure)
  for _, g in pairs(groupNames) do
    setBeamGroupMaximumSupplyPressure(g, maxSupplyPressure)
  end
end

local function setBeamGroupCrossFlowTag(groupName, crossFlowTag)
  local groupData = beamGroups[groupName]
  if not groupData then
    log("W", "actuators.setBeamGroupCrossFlowTag", "Can't find pressure beam group: " .. (groupName or "nil"))
  else
    local previousTag = groupData.crossFlowTag

    if previousTag == crossFlowTag then
      return
    end

    groupData.crossFlowTag = crossFlowTag

    local prevCrossFlowGroup = crossFlowGroups[previousTag]
    local newCrossFlowGroup = crossFlowGroups[crossFlowTag]

    -- Update beam group count on previous and new cross flow groups
    if prevCrossFlowGroup then
      prevCrossFlowGroup.beamGroupCount = max(0, prevCrossFlowGroup.beamGroupCount - 1)
    end

    if newCrossFlowGroup then
      newCrossFlowGroup.beamGroupCount = newCrossFlowGroup.beamGroupCount + 1

      if newCrossFlowGroup.beamGroupCount == 1 then
        -- If this is the only beam group in the new cross-flow group, reset the average pressure too
        resetVirtualBuffer(crossFlowTag)
      end
    end
  end
end

local function getValveState(groupName)
  local groupData = beamGroups[groupName]
  if not groupData then
    log("W", "actuators.getValveState", "Can't find pressure beam group: " .. (groupName or "nil"))
    return 0
  end
  return groupData.valveState
end

local function getAverageFlowRate(groupName)
  local groupData = beamGroups[groupName]
  if not groupData then
    log("W", "actuators.getAverageFlowRate", "Can't find pressure beam group: " .. (groupName or "nil"))
    return 0
  end
  return groupData.averageFlowRate
end

local function getAveragePressure(groupName)
  local groupData = beamGroups[groupName]
  if not groupData then
    log("W", "actuators.getAveragePressure", "Can't find pressure beam group: " .. (groupName or "nil"))
    return 0
  end
  return groupData.averagePressure
end

local function getBeamGroupsAverageFlowRate(groupNames)
  local sum = 0
  local count = 0
  for _, name in ipairs(groupNames) do
    sum = sum + getAverageFlowRate(name)
    count = count + 1
  end
  return count > 0 and (sum / count) or 0
end

local function updateFixedStep(dt)
  local invTempCoefficient = 1 / (gasConstant * powertrain.currentEnvTemperature) -- for calculating air mass
  local tankPressure = airTank.currentPressure
  local airTankAirDensity = airTank.remainingMass * airTank.invCapacity
  local invAirTankAirDensity = 1 / airTankAirDensity
  local pressureSum = 0

  for _, g in pairs(beamGroups) do
    local crossFlowGroup = crossFlowGroups[g.crossFlowTag]

    if not crossFlowGroup then
      -- Create an implicit cross-flow group, as this group did not reference an explicitly-defined one
      log("W", "actuators", ("Creating implicit cross-flow group %q for beam group %q"):format(g.crossFlowTag, g.name))
      crossFlowGroup = {
        name = g.crossFlowTag,
        bufferStoredEnergy = 0,
        bufferPressure = g.averagePressure,
        bufferCapacity = defaultVirtualBufferCapacity,
        invBufferCapacity = 1 / defaultVirtualBufferCapacity,
        beamGroupCount = 1,
        enableDebug = false,
      }
      crossFlowGroup.bufferStoredEnergy = crossFlowGroup.bufferPressure * crossFlowGroup.bufferCapacity
      crossFlowGroups[g.crossFlowTag] = crossFlowGroup
    end

    local electricsValveState = electrics.values[g.valveStateElectricsName]

    -- if electrics value has changed enough since the last update, we take that instead of the last "requested" state
    if type(electricsValveState) == "number" and abs(electricsValveState - g.lastElectricsValue) > 1e-20 then
      g.valveState = electricsValveState
      g.lastElectricsValue = electricsValveState
    end

    local absValveState = abs(g.valveState)
    local flowRateSum = 0
    local flowRate = 0
    local sourcePressure, flowPressureDiff
    local maxAllowedEnergyTransfer
    local airVolumeMoved, energyTransferred
    local bufferAirMass, bufferAirDensity, invBufferAirDensity

    -- Air flows into or out of a virtual "buffer" (essentially representing the air capacity of the hoses) when the
    -- control valve is not fully closed. If cross-flow is enabled, or if the valve is open at all, air also flows
    -- between that virtual buffer and all of the beams.
    -- Multiple groups sharing the same virtual buffer all "cross flow" between one another, and their valves all
    -- contribute air flow into or out of the virtual buffer. The buffer essentially plays the role of a phsyical
    -- hose connecting the different groups' air circuits.

    if g.valveState > 0 then
      -- airflow occurring between air tank and virtual buffer
      sourcePressure = min(tankPressure, g.maxSupplyPressure)
      flowPressureDiff = sourcePressure - crossFlowGroup.bufferPressure
      flowRate = dischargeCoefficient * absValveState * g.supplyHoseCrossSectionArea * sign(flowPressureDiff) * sqrt(2 * abs(flowPressureDiff) * invAirTankAirDensity)
    else
      -- airflow occurring between virtual buffer and environment
      sourcePressure = powertrain.currentEnvPressure
      flowPressureDiff = sourcePressure - crossFlowGroup.bufferPressure

      -- if the absolute buffer pressure is more than twice the env pressure,
      -- flow rate is limited by the speed of sound, so we clamp the maximum to the constant quickReleaseFlowRate.
      local relativeCurrentPressure = crossFlowGroup.bufferPressure - powertrain.currentEnvPressure
      local flowRateCoef = clamp(relativeCurrentPressure * powertrain.invCurrentEnvPressure, -1, 1)

      flowRate = -pressureDumpFlowRate * absValveState * sign(flowRateCoef) * sqrt(abs(flowRateCoef))
    end

    g.bufferFlowRate = flowRate

    if abs(flowPressureDiff) > tinyPressureDiffThreshold then
      maxAllowedEnergyTransfer = abs(flowPressureDiff) * crossFlowGroup.bufferCapacity -- clamp so that we don't "overshoot" the source pressure
      airVolumeMoved = abs(flowRate) * dt
      energyTransferred = min(maxAllowedEnergyTransfer, crossFlowGroup.bufferPressure * airVolumeMoved) * sign(flowRate)

      -- determine new stored energy quantity
      crossFlowGroup.bufferStoredEnergy = crossFlowGroup.bufferStoredEnergy + energyTransferred
      crossFlowGroup.bufferPressure = crossFlowGroup.bufferStoredEnergy * crossFlowGroup.invBufferCapacity -- PV = e, therefore P = e / V

      -- we can move energy out of the air tank unconditionally, because it would only ever flow that way if the air tank has a higher pressure.
      -- however, we can only move energy INTO the air tank if the buffer pressure is truly higher.
      -- if energy is leaving the buffer due to the "pressure regulator" (maxSupplyPressure) decreasing, it should be dumped to the atmosphere.
      if g.valveState > 0 and (energyTransferred > 0 or crossFlowGroup.bufferPressure > tankPressure) and not g.disableAirConsumption then
        airTank.storedEnergy = max(0, airTank.storedEnergy - energyTransferred)
      end
    elseif g.valveState > 0 then
      -- if the flow rate is very tiny, we will simply "snap" the pressure to the source pressure when the valve is open.
      -- otherwise, the pressure will never "settle" because of floating point inaccuracies
      crossFlowGroup.bufferPressure = sourcePressure
      crossFlowGroup.bufferStoredEnergy = crossFlowGroup.bufferPressure * crossFlowGroup.bufferCapacity
    end

    bufferAirMass = crossFlowGroup.bufferStoredEnergy * airTank.gasMolarMass * invTempCoefficient
    bufferAirDensity = bufferAirMass * crossFlowGroup.invBufferCapacity
    invBufferAirDensity = 1 / bufferAirDensity
    g.averagePressure = 0

    for _, v in pairs(g.beams) do
      local beamLength = max(obj:getBeamLength(v.cid) - v.lengthOffset, 0)
      local beamVolume = v.surface * beamLength + 1e-30
      local invBeamVolume = 1 / beamVolume

      -- Flow simulation for individual actuator (between actuator and virtual buffer)

      if absValveState > 0 or enableCrossFlow then
        -- air flowing between virtual buffer and actuators
        local flowCoef = enableCrossFlow and 1 or absValveState
        sourcePressure = crossFlowGroup.bufferPressure
        flowPressureDiff = sourcePressure - v.currentPressure
        flowRate = dischargeCoefficient * flowCoef * v.supplyHoseCrossSectionArea * sign(flowPressureDiff) * sqrt(2 * abs(flowPressureDiff) * invBufferAirDensity)
      else
        flowPressureDiff = 0
        flowRate = 0
      end

      if abs(flowPressureDiff) > tinyPressureDiffThreshold then
        flowRateSum = flowRateSum + flowRate
        maxAllowedEnergyTransfer = abs(flowPressureDiff) * beamVolume -- clamp so that we don't "overshoot" the source pressure
        airVolumeMoved = abs(flowRate) * dt
        energyTransferred = min(maxAllowedEnergyTransfer, v.currentPressure * airVolumeMoved) * sign(flowRate)

        -- determine new stored energy quantities
        v.storedEnergy = v.storedEnergy + energyTransferred
        v.energyDeltaThisTick = v.energyDeltaThisTick + energyTransferred
        crossFlowGroup.bufferStoredEnergy = crossFlowGroup.bufferStoredEnergy - energyTransferred
        crossFlowGroup.bufferPressure = crossFlowGroup.bufferStoredEnergy * crossFlowGroup.invBufferCapacity -- PV = e, therefore P = e / V

        -- determine new actuator/buffer pressures
        v.currentPressure = v.storedEnergy * invBeamVolume -- PV = e, therefore P = e / V
        pressureSum = pressureSum + v.currentPressure

        setBeamPressureCore(v.cid, v.currentPressure, v.maxBeamPressure, v.spring, v.damp)
      elseif absValveState > 0 or enableCrossFlow then
        -- if the flow rate is very tiny, we will simply "snap" the pressure to the source pressure when the valve is open.
        -- otherwise, the pressure will never "settle" because of floating point inaccuracies
        v.currentPressure = crossFlowGroup.bufferPressure
        v.storedEnergy = v.currentPressure * beamVolume

        setBeamPressureCore(v.cid, v.currentPressure, v.maxBeamPressure, v.spring, v.damp)
      end

      g.averagePressure = g.averagePressure + v.currentPressure
    end

    if g.beamCount > 0 then
      g.averageFlowRate = flowRateSum * g.invBeamCount
      g.averagePressure = g.averagePressure * g.invBeamCount
    else
      g.averageFlowRate = 0
      g.averagePressure = 0
    end
  end

  -- update air tank pressure/mass since we may have modified stored energy
  airTank.currentPressure = airTank.storedEnergy * airTank.invCapacity
  airTank.remainingMass = airTank.storedEnergy * airTank.gasMolarMass * invTempCoefficient

  if totalBeamCount > 0 then
    averagePressure = pressureSum / totalBeamCount
  else
    averagePressure = 0
  end
end

local function updateGFX(dt)
  for _, g in pairs(crossFlowGroups) do
    if g.enableDebug then
      -- streams.drawGraph(g.name .. "_bufferEnergy", { value = g.bufferStoredEnergy, unit = "J" })
      streams.drawGraph(g.name .. "_bufferPressure", { value = (g.bufferPressure - 101325) * pascalToPSI, unit = "PSI" })
    end
  end

  for _, g in pairs(beamGroups) do
    if g.enableDebug then
      -- streams.drawGraph(g.name .. "_avgPressure", { value = (g.averagePressure - 101325) * pascalToPSI, unit = "PSI" })
      streams.drawGraph(g.name .. "_avgFlowRate", { value = g.averageFlowRate * 1000, unit = "L/s" })
      streams.drawGraph(g.name .. "_valveState", { value = g.valveState })
    end

    for _, v in pairs(g.beams) do
      if v.enableDebug then
        streams.drawGraph(v.name .. "_pressure", {value = v.currentPressure * pascalToPSI, unit = "PSI"})
        streams.drawGraph(v.name .. "_energyDelta", {value = v.energyDeltaThisTick, unit = "J"})
      end

      v.energyDeltaThisTick = 0
    end

    if g.hasSoundData then
      local flowRate = g.bufferFlowRate
      local absFlowRate = abs(flowRate)
      local increasing = g.bufferFlowRate > 0
      local startFlowRateThreshold = increasing and g.startFlowRateThresholdPressureIncrease or g.startFlowRateThresholdPressureDecrease
      local stopFlowRateThreshold = increasing and g.stopFlowRateThresholdPressureIncrease or g.stopFlowRateThresholdPressureDecrease
      local volumeFactor = increasing and g.flowRateVolumeFactorIncrease or g.flowRateVolumeFactorDecrease
      local volumeCoef = volumeFactor == 0 and 1 or clamp(absFlowRate * volumeFactor, 0, 1)

      if g.isPlayingIncrease then
        obj:setVolume(g.soundLoopIncrease, g.volumeIncrease * volumeCoef)
      end
      if g.isPlayingDecrease then
        obj:setVolume(g.soundLoopDecrease, g.volumeDecrease * volumeCoef)
      end

      if absFlowRate > startFlowRateThreshold then
        -- Start playing the appropriate sound

        if increasing then
          obj:stopSFX(g.soundLoopDecrease)
          g.isPlayingDecrease = false
          if not g.isPlayingIncrease then
            obj:cutSFX(g.soundLoopIncrease)
            obj:setVolume(g.soundLoopIncrease, g.volumeIncrease * volumeCoef)
            obj:playSFX(g.soundLoopIncrease)
            g.isPlayingIncrease = true
            if g.enableDebug then
              log("W", "actuators", ("[%s] starting increasing sound, fr = %f"):format(g.name, flowRate))
            end
          end
        else
          obj:stopSFX(g.soundLoopIncrease)
          g.isPlayingIncrease = false
          if not g.isPlayingDecrease then
            obj:cutSFX(g.soundLoopDecrease)
            obj:setVolume(g.soundLoopDecrease, g.volumeDecrease * volumeCoef)
            obj:playSFX(g.soundLoopDecrease)
            g.isPlayingDecrease = true
            if g.enableDebug then
              log("W", "actuators", ("[%s] starting decreasing sound, fr = %f"):format(g.name, flowRate))
            end
          end
        end
      elseif absFlowRate < stopFlowRateThreshold then
        -- Stop playing all sounds

        obj:stopSFX(g.soundLoopIncrease)
        obj:stopSFX(g.soundLoopDecrease)
        if g.enableDebug and (g.isPlayingIncrease or g.isPlayingDecrease) then
          log("W", "actuators", ("[%s] stopping all sound, fr = %f"):format(g.name, flowRate))
        end
        g.isPlayingIncrease = false
        g.isPlayingDecrease = false
      end
    end

    electrics.values[g.avgPressureElectricsName] = g.averagePressure - powertrain.currentEnvPressure
  end
end

local function reset()
  for _, g in pairs(beamGroups) do
    for _, v in pairs(g.beams) do
      local beamLength = obj:getBeamLength(v.cid)
      local beamVolume = v.surface * beamLength

      v.targetPressure = v.defaultPressure
      v.currentPressure = v.defaultPressure
      v.storedEnergy = v.defaultPressure * beamVolume
      v.energyDeltaThisTick = 0
    end
    g.hasChangedVelocity = false
    g.averagePressure = 0
    g.isPlayingIncrease = false
    g.isPlayingDecrease = false
    g.valveState = 0
    g.averageFlowRate = 0
    g.averagePressure = 0
    g.bufferFlowRate = 0
    g.maxSupplyPressure = g.maxSupplyPressureInitial
  end

  -- compute average pressure of all beams
  updateBeamAggregates()

  -- initialize pressure of all virtual buffers
  resetVirtualBuffers()
end

local function init(jbeamData)
  local airTankName = jbeamData.airTankName or "mainAirTank"

  airTank = energyStorage.getStorage(airTankName)

  if not airTank then
    log("E", "actuators.init", "Air tank not found: " .. airTankName)
    M.updateFixedStep = nop
  end

  local supplyHoseRadius = jbeamData.supplyHoseRadius or 0.0075 -- meters
  local maxSupplyPressure = jbeamData.maxSupplyPressure or math.huge
  if type(jbeamData.maxSupplyPressurePSI) == "number" then
    maxSupplyPressure = jbeamData.maxSupplyPressurePSI * psiToPascal + 101325
  end

  defaultVirtualBufferCapacity = jbeamData.virtualBufferCapacity or 0.005
  pressureDumpFlowRate = jbeamData.pressureDumpFlowRate or 0.01 -- m^3/s (used when valve is open to atmosphere)
  enableCrossFlow = jbeamData.crossFlowBetweenBeams == true
  enableDebug = jbeamData.debug or false

  local pressureBeamData = v.data[jbeamData.pressuredBeams] or {}
  local crossFlowGroupData = (jbeamData.crossFlowGroups and v.data[jbeamData.crossFlowGroups]) or {}

  local pressuredBeamNames = {}
  for _, v in pairs(pressureBeamData) do
    local name = v.beamName
    pressuredBeamNames[name] = true
  end

  local relevantBeams = {}
  local beams = v.data.beams
  for i = 0, tableSizeC(beams) - 1 do
    local v = beams[i]
    if v.name and pressuredBeamNames[v.name] then
      relevantBeams[v.name] = {
        cid = v.cid,
        defaultPressure = v.pressure,
        maxPressure = v.maxPressure,
        spring = v.beamSpring,
        damp = v.beamDamp,
        surface = v.surface,
        lengthOffset = v.lengthOffset or 0
      }
    end
  end

  crossFlowGroups = {}
  for _, crossFlowData in pairs(crossFlowGroupData) do
    local groupTag = crossFlowData.crossFlowTag
    local virtualBufferCapacity = crossFlowData.virtualBufferCapacity or defaultVirtualBufferCapacity

    crossFlowGroups[groupTag] = {
      name = groupTag,
      bufferStoredEnergy = 0,
      bufferPressure = 0,
      bufferCapacity = virtualBufferCapacity,
      invBufferCapacity = 1 / virtualBufferCapacity,
      beamGroupCount = 0,
      enableDebug = crossFlowData.debug == true,
    }
  end

  beamGroups = {}
  for _, pressureData in pairs(pressureBeamData) do
    local name = pressureData.beamName
    local groupName = pressureData.groupName

    local crossFlowTag = pressureData.crossFlowTag or groupName
    local beamSupplyHoseRadius = pressureData.supplyHoseRadius or supplyHoseRadius
    local supplyHoseCrossSectionArea = math.pi * beamSupplyHoseRadius ^ 2
    local beamMaxSupplyPressure = pressureData.maxSupplyPressure or maxSupplyPressure
    if type(pressureData.maxSupplyPressurePSI) == "number" then
      beamMaxSupplyPressure = pressureData.maxSupplyPressurePSI * psiToPascal + 101325
    end

    local beamEnableDebug = pressureData.debug or false

    if not beamGroups[groupName] then
      beamGroups[groupName] = {
        name = groupName,
        beams = {},
        beamCount = 0,
        invBeamCount = 0,
        isPlayingIncrease = false,
        isPlayingDecrease = false,
        valveState = 0,
        lastElectricsValue = 0,
        averageFlowRate = 0,
        averagePressure = 0,
        valveStateElectricsName = pressureData.valveStateElectricsName or (M.name .. "_" .. groupName .. "_valveState"),
        avgPressureElectricsName = M.name .. "_" .. groupName .. "_pressure_avg",
        supplyHoseCrossSectionArea = supplyHoseCrossSectionArea,
        crossFlowTag = crossFlowTag,
        bufferFlowRate = 0,
        maxSupplyPressure = beamMaxSupplyPressure,
        maxSupplyPressureInitial = beamMaxSupplyPressure,
        disableAirConsumption = pressureData.disableAirConsumption or false,
        enableDebug = false
      }

      local crossFlowGroup = crossFlowGroups[crossFlowTag]

      if crossFlowGroup then
        crossFlowGroup.beamGroupCount = crossFlowGroup.beamGroupCount + 1
      end -- if it is nil, it will just be initialized in updateFixedStep
    end

    local group = beamGroups[groupName]

    if not relevantBeams[name] then
      log("W", "actuators.init", "Can't find beam with name: " .. name)
    else
      local beamLength = max(obj:getBeamLength(relevantBeams[name].cid) - relevantBeams[name].lengthOffset, 0)
      local beamVolume = relevantBeams[name].surface * beamLength

      local beamData = {
        name = name,
        cid = relevantBeams[name].cid,
        maxBeamPressure = relevantBeams[name].maxPressure,
        spring = relevantBeams[name].spring,
        damp = relevantBeams[name].damp,
        surface = relevantBeams[name].surface,
        lengthOffset = relevantBeams[name].lengthOffset,
        defaultPressure = relevantBeams[name].defaultPressure,
        currentPressure = relevantBeams[name].defaultPressure,
        storedEnergy = relevantBeams[name].defaultPressure * beamVolume,
        energyDeltaThisTick = 0,
        supplyHoseCrossSectionArea = supplyHoseCrossSectionArea,
        enableDebug = beamEnableDebug and enableDebug
      }

      --log("D", "actuators.init", ("beam %q volume: %f"):format(name, beamVolume))

      table.insert(group.beams, beamData)
      group.beamCount = group.beamCount + 1
      group.invBeamCount = 1 / group.beamCount
      group.enableDebug = group.enableDebug or beamEnableDebug
    end
  end

  -- compute average pressure of all beams
  updateBeamAggregates()

  -- initialize pressure of all virtual buffers
  resetVirtualBuffers()
end

local function initSounds(jbeamData)
  local pressureBeamSoundData = v.data[jbeamData.groupSounds] or {}

  for _, v in pairs(pressureBeamSoundData) do
    local groupData = beamGroups[v.groupName]
    if groupData then
      groupData.hasSoundData = true
      groupData.volumeIncrease = v.volumeIncrease or v.volume or 1
      groupData.volumeDecrease = v.volumeDecrease or v.volume or 1
      groupData.soundNode = v.node or 0
      groupData.startFlowRateThresholdPressureIncrease = v.startFlowRateThresholdPressureIncrease or v.startFlowRateThreshold or 0.005
      groupData.startFlowRateThresholdPressureDecrease = v.startFlowRateThresholdPressureDecrease or v.startFlowRateThreshold or 0.005
      groupData.stopFlowRateThresholdPressureIncrease = v.stopFlowRateThresholdPressureIncrease or v.stopFlowRateThreshold or 0.0005
      groupData.stopFlowRateThresholdPressureDecrease = v.stopFlowRateThresholdPressureDecrease or v.stopFlowRateThreshold or 0.0005
      groupData.flowRateVolumeFactorIncrease = v.flowRateVolumeFactorIncrease or v.flowRateVolumeFactor or 0
      groupData.flowRateVolumeFactorDecrease = v.flowRateVolumeFactorDecrease or v.flowRateVolumeFactor or 0
      if v.soundIncrease then
        groupData.soundLoopIncrease = obj:createSFXSource2(v.soundIncrease, "AudioDefaultLoop3D", "pneumatics_inc_" .. v.groupName, groupData.soundNode, 0)
        if groupData.soundLoopIncrease then
          bdebug.setNodeDebugText("Pneumatic Actuator", groupData.soundNode, M.name .. " - Inc " .. v.groupName .. ": " .. (v.soundIncrease or "no event"))
        end
      end
      if v.soundDecrease then
        groupData.soundLoopDecrease = obj:createSFXSource2(v.soundDecrease, "AudioDefaultLoop3D", "pneumatics_dec_" .. v.groupName, groupData.soundNode, 0)
        if groupData.soundLoopDecrease then
          bdebug.setNodeDebugText("Pneumatic Actuator", groupData.soundNode, M.name .. " - Dec " .. v.groupName .. ": " .. (v.soundDecrease or "no event"))
        end
      end
    else
      log("W", "actuators.initSounds", "Can't find group with name: " .. v.groupName)
    end
  end
end

local function resetSounds()
  for _, g in pairs(beamGroups) do
    if g.soundLoopIncrease then
      obj:setVolume(g.soundLoopIncrease, 0)
      g.isPlayingIncrease = false
    end
    if g.soundLoopDecrease then
      obj:setVolume(g.soundLoopDecrease, 0)
      g.isPlayingDecrease = false
    end
  end
end

M.init = init
M.reset = reset
M.initSounds = initSounds
M.resetSounds = resetSounds
M.updateFixedStep = updateFixedStep
M.updateGFX = updateGFX

M.setBeamGroupValveState = setBeamGroupValveState
M.setBeamGroupsValveState = setBeamGroupsValveState
M.toggleBeamGroupValveState = toggleBeamGroupValveState
M.toggleBeamGroupsValveState = toggleBeamGroupsValveState
M.setBeamGroupMaximumSupplyPressure = setBeamGroupMaximumSupplyPressure
M.setBeamGroupsMaximumSupplyPressure = setBeamGroupsMaximumSupplyPressure
M.setBeamGroupCrossFlowTag = setBeamGroupCrossFlowTag
M.getValveState = getValveState
M.getAverageFlowRate = getAverageFlowRate
M.getAveragePressure = getAveragePressure
M.getBeamGroupsAverageFlowRate = getBeamGroupsAverageFlowRate

return M
