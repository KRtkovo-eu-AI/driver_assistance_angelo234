-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local min = math.min
local max = math.max

local hasExecutedInitWork = false

local activeParts = {}

local partTypeData = {}
local partOdometerAbsoluteBaseValues = {}
local partOdometerRelativeStartingValues = {}
local paintOdometerAbsoluteBaseValues = {}
local paintOdometerRelativeStartingValues = {}

local partPaints = {}

local lastAppliedPartConditions = {}
local hasSetPartCondition = {}

local savedConditionSnapshots = {}
local resetSnapshotKey

local rootPartId

local paintAgingConstants = {
  wearStartOdometer = 5000000,
  wearEndOdometer = 800000000,
  maxClearcoatRoughnessIncrease = 1,
  minSaturationCoef = 0.95,
  maxColorBrightnessIncreaseR = 0.2,
  maxColorBrightnessIncreaseG = 0.3,
  maxColorBrightnessIncreaseB = 0.3
}

local function createConditionSnapshot(snapshotKey)
  local snapshotData = M.getConditions()
  if not snapshotData or not snapshotKey then
    log("E", "partCondition.createConditionSnapshot", "No snapshot data or no key, cannot create snapshot with key: " .. snapshotKey)
    return
  end

  savedConditionSnapshots[snapshotKey] = snapshotData
end

local function applyConditionSnapshot(snapshotKey)
  if not snapshotKey then
    log("E", "", "No key provided, cannot create snapshot...")
    return
  end
  local snapshotData = savedConditionSnapshots[snapshotKey]
  if not snapshotData then
    log("E", "partCondition.applyConditionSnapshot", "No snapshot data found, cannot apply snapshot with key: " .. snapshotKey)
    return
  end

  hasSetPartCondition = {}
  M.initConditions(snapshotData)
end

local function deleteConditionSnapshots()
  savedConditionSnapshots = {}
  resetSnapshotKey = nil
end

local function setResetSnapshotKey(snapshotKey)
  resetSnapshotKey = snapshotKey
end

local function lookForPowertrainClues(partId, partData)
  local partTypeTags
  for k, v in pairs(partData) do
    if type(v) == "table" then
      if k == "powertrain" then
        local previousPartPath
        for i = 2, #v do
          if v[i].partPath then
            previousPartPath = v[i].partPath
          else
            if type(v) == "table" and #v[i] > 1 then
              local deviceType = v[i][1]
              local deviceName = v[i][2]
              local devicePartId = partId
              if previousPartPath then
                devicePartId = previousPartPath
              end
              partTypeTags = partTypeTags or {}
              partTypeTags[devicePartId] = partTypeTags[devicePartId] or {}
              table.insert(partTypeTags[devicePartId], "powertrainDevice:" .. deviceName)
            --print(string.format("%s -> %s:%s", devicePartId, deviceType, deviceName))
            end
          end
        end
      elseif k == "energyStorage" then
        local previousPartPath
        for i = 2, #v do
          if v[i].partPath then
            previousPartPath = v[i].partPath
          else
            if type(v) == "table" and #v[i] > 1 then
              local storageType = v[i][1]
              local storageName = v[i][2]
              local storagePartId = partId
              if previousPartPath then
                storagePartId = previousPartPath
              end
              partTypeTags = partTypeTags or {}
              partTypeTags[storagePartId] = partTypeTags[storagePartId] or {}
              table.insert(partTypeTags[storagePartId], "energyStorage:" .. storageName)
            --print(string.format("%s -> %s:%s", storagePartId, storageType, storageName))
            end
          end
        end
      else
        if v.radiatorArea and not v.inertia then --look for the radiator part
          --print(string.format("%s -> %s:%s", partId, k, "radiator"))
          partTypeTags = partTypeTags or {}
          partTypeTags[partId] = partTypeTags[partId] or {}
          table.insert(partTypeTags[partId], string.format("powertrainDevice:%s:%s", k, "radiator"))
        elseif v.torqueModExhaust and not v.inertia then --look for the exhaust part
          --print(string.format("%s -> %s:%s", partId, k, "exhaust"))
          partTypeTags = partTypeTags or {}
          partTypeTags[partId] = partTypeTags[partId] or {}
          table.insert(partTypeTags[partId], string.format("powertrainDevice:%s:%s", k, "exhaust"))
        elseif v.turbocharger and not v.inertia then
          partTypeTags = partTypeTags or {}
          partTypeTags[partId] = partTypeTags[partId] or {}
          table.insert(partTypeTags[partId], string.format("powertrainDevice:%s:%s", k, "turbocharger"))
        --print(string.format("%s -> %s:%s", partId, k, "turbocharger"))
        end
      end
    end
  end

  return partTypeTags
end

local function lookForFlexbodyClues()
  local partTypeTags = {}
  for _, flexbody in pairs(v.data.flexbodies) do
    --TODO: check maybe a tag that tells us if the flexbody can change color (and how?), paint vs glas vs plastic etc
    if flexbody.partPath then
      partTypeTags[flexbody.partPath] = partTypeTags[flexbody.partPath] or {}
      table.insert(partTypeTags[flexbody.partPath], string.format("jbeam:flexbody:%s", flexbody.mesh))
    end
  end
  return partTypeTags
end

local function lookForJbeamClues()
  local partTypeTags = {}
  local didFindClues = false
  local beamsPerPart = {}
  for _, beam in pairs(v.data.beams) do
    if beam.partPath then
      local partId = beam.partPath
      if beam.beamDampRebound and beam.beamDampRebound > 0 and beam.beamDampFast and beam.beamDampVelocitySplit and beam.beamDampVelocitySplit < math.huge and beam.beamDampReboundFast then
        didFindClues = true
        partTypeTags[partId] = partTypeTags[partId] or {}
      --table.insert(partTypeTags[partId], string.format("jbeam:damper:%s", beam.name))
      end
      if beam.breakGroup then
        didFindClues = true
        partTypeTags[partId] = partTypeTags[partId] or {}
        local breakGroups = type(beam.breakGroup) == "table" and beam.breakGroup or {beam.breakGroup}
        for _, breakGroup in ipairs(breakGroups) do
          table.insert(partTypeTags[partId], string.format("jbeam:breakGroup:%s", breakGroup))
        end
      end
      beamsPerPart[beam.partPath] = beamsPerPart[beam.partPath] or {beamCids = {}, deformableBeams = 0, breakableBeams = 0}
      --exclude support beams
      if beam.beamType ~= 7 and beam.beamDeform < math.huge and beam.beamDeform < beam.beamStrength then
        beamsPerPart[beam.partPath].deformableBeams = (beamsPerPart[beam.partPath].deformableBeams or 0) + 1
      end
      if beam.beamType ~= 7 and beam.beamStrength < math.huge then
        beamsPerPart[beam.partPath].breakableBeams = (beamsPerPart[beam.partPath].breakableBeams or 0) + 1
      end
      table.insert(beamsPerPart[beam.partPath].beamCids, beam.cid)
    end
  end

  for partId, partData in pairs(beamsPerPart) do
    if partData.deformableBeams > 0 or partData.breakableBeams > 0 then
      didFindClues = true
      partTypeTags[partId] = partTypeTags[partId] or {}
      for _, beamCid in ipairs(partData.beamCids) do
        if v.data.beams[beamCid] and v.data.beams[beamCid].beamType ~= 7 then --exclude support beams (type 7 -> bdebug.lua)
          table.insert(partTypeTags[partId], string.format("jbeam:beamDamage:%d", beamCid))
        end
      end
    end
  end

  return didFindClues and partTypeTags or nil
end

local function preparePartData()
  --combine the part ids with their part data
  --this comes from the loader in two tables, but we need them combined in one table
  --v.data.activeParts is the part ids ("paths"), v.data.activePartsData is the part data
  activeParts = {}
  for partId, partTypeName in pairs(v.data.activeParts) do
    activeParts[partId] = v.data.activePartsData[partTypeName]
  end

  rootPartId = nil

  for partId, partData in pairs(activeParts) do
    --save root part id for later
    if partData.slotType == "main" then
      rootPartId = partId
    end

    local powertrainClues = lookForPowertrainClues(partId, partData)
    if powertrainClues then
      for part, types in pairs(powertrainClues) do
        partTypeData[part] = partTypeData[part] or {}
        for _, partType in pairs(types) do
          table.insert(partTypeData[part], partType)
        end
      end
    end
  end

  local flexbodyClues = lookForFlexbodyClues()
  if flexbodyClues then
    for part, types in pairs(flexbodyClues) do
      partTypeData[part] = partTypeData[part] or {}
      for _, partType in pairs(types) do
        table.insert(partTypeData[part], partType)
      end
    end
  end

  local jbeamClues = lookForJbeamClues()
  if jbeamClues then
    for part, types in pairs(jbeamClues) do
      partTypeData[part] = partTypeData[part] or {}
      for _, partType in pairs(types) do
        table.insert(partTypeData[part], partType)
      end
    end
  end

  for partId, types in pairs(partTypeData) do
    local deduplication = {}
    for _, partType in pairs(types) do
      deduplication[partType] = true
    end
    partTypeData[partId] = {}
    for partType, _ in pairs(deduplication) do
      table.insert(partTypeData[partId], partType)
    end
  end
  hasExecutedInitWork = true
end

local function getRootPartOdometerValue()
  local spawnTimeOdometer = partOdometerAbsoluteBaseValues[rootPartId] or 0
  local odometer = spawnTimeOdometer + max(extensions.odometer.getRelativeRecording() - (partOdometerRelativeStartingValues[rootPartId] or 0), 0)
  return odometer
end

local function getRootPartTripValue()
  local trip = max(extensions.odometer.getRelativeRecording() - (partOdometerRelativeStartingValues[rootPartId] or 0), 0)
  return trip
end

local function setPartMeshPaints(partId, paints)
  local baseColor1 = paints[1].baseColor or {0, 0, 0, 0}
  local baseColor2 = paints[2].baseColor or baseColor1
  local baseColor3 = paints[3].baseColor or baseColor1

  local paintData1Roughness = paints[1].roughness or 0
  local paintData1Metallic = paints[1].metallic or 0
  local paintData1Clearcoat = paints[1].clearcoat or 0
  local paintData1ClearcoatRoughness = paints[1].clearcoatRoughness or 0

  local paintData2Roughness = paints[2].roughness or 0
  local paintData2Metallic = paints[2].metallic or 0
  local paintData2Clearcoat = paints[2].clearcoat or 0
  local paintData2ClearcoatRoughness = paints[2].clearcoatRoughness or 0

  local paintData3Roughness = paints[3].roughness or 0
  local paintData3Metallic = paints[3].metallic or 0
  local paintData3Clearcoat = paints[3].clearcoat or 0
  local paintData3ClearcoatRoughness = paints[3].clearcoatRoughness or 0

  for _, partType in ipairs(partTypeData[partId] or {}) do
    local split = split(partType, ":")
    if split[1] == "jbeam" and split[2] == "flexbody" then
      --TODO improve interface to GE for setting mesh colors
      local colorCmd = string.format("getObjectByID(%d):setMeshColor(%q, ColorI(%d,%d,%d,%d), ColorI(%d,%d,%d,%d), ColorI(%d,%d,%d,%d))", objectId, split[3], baseColor1[1] * 255, baseColor1[2] * 255, baseColor1[3] * 255, 255, baseColor2[1] * 255, baseColor2[2] * 255, baseColor2[3] * 255, 255, baseColor3[1] * 255, baseColor3[2] * 255, baseColor3[3] * 255, 255)
      --ColorI(roughness0, metallic0, clearCoatFactor0, clearCoatRoughness0)
      local paintDataCmd = string.format("getObjectByID(%d):setMeshPaintData(%q, ColorI(%d,%d,%d,%d), ColorI(%d,%d,%d,%d), ColorI(%d,%d,%d,%d))", objectId, split[3], paintData1Roughness * 255, paintData1Metallic * 255, paintData1Clearcoat * 255, paintData1ClearcoatRoughness * 255, paintData2Roughness * 255, paintData2Metallic * 255, paintData2Clearcoat * 255, paintData2ClearcoatRoughness * 255, paintData3Roughness * 255, paintData3Metallic * 255, paintData3Clearcoat * 255, paintData3ClearcoatRoughness * 255)
      obj:queueGameEngineLua(colorCmd)
      obj:queueGameEngineLua(paintDataCmd)
    end
  end
end

local function getAgedPaint(paint, paintOdometer)
  paintOdometer = clamp(paintOdometer, 0, paintAgingConstants.wearEndOdometer)
  local agedPaint = deepcopy(paint)
  local agedColor = deepcopy(paint.baseColor)

  local wearStartOdometer = paintAgingConstants.wearStartOdometer
  local wearEndOdometer = paintAgingConstants.wearEndOdometer
  local roughnessIncrease = linearScale(paintOdometer, wearStartOdometer, wearEndOdometer, 0, paintAgingConstants.maxClearcoatRoughnessIncrease)
  local saturationCoef = linearScale(paintOdometer, wearStartOdometer, wearEndOdometer, 1, paintAgingConstants.minSaturationCoef)
  local colorBrightnessIncrease = {}
  colorBrightnessIncrease[1] = linearScale(paintOdometer, wearStartOdometer, wearEndOdometer, 0, paintAgingConstants.maxColorBrightnessIncreaseR)
  colorBrightnessIncrease[2] = linearScale(paintOdometer, wearStartOdometer, wearEndOdometer, 0, paintAgingConstants.maxColorBrightnessIncreaseG)
  colorBrightnessIncrease[3] = linearScale(paintOdometer, wearStartOdometer, wearEndOdometer, 0, paintAgingConstants.maxColorBrightnessIncreaseG)

  -- a) decrease paint roughness
  if agedPaint.clearcoat > 0 then
    agedPaint.clearcoatRoughness = min(agedPaint.clearcoatRoughness + roughnessIncrease, 1)
  else
    agedPaint.roughness = min(agedPaint.roughness + roughnessIncrease, 1)
  end

  -- b) decrease overall saturation
  local h, s, v = RGBtoHSV(agedColor[1], agedColor[2], agedColor[3])
  s = s * saturationCoef
  agedColor[1], agedColor[2], agedColor[3] = HSVtoRGB(h, s, v)
  -- c) increase lightness, per color
  for i = 1, 3 do
    agedColor[i] = clamp(agedColor[i] + colorBrightnessIncrease[i], 0, 1)
  end

  agedPaint.baseColor = agedColor
  return agedPaint
end

local function getAgedPaints(paints, paintOdometer)
  local agedPaints = {}
  agedPaints[1] = getAgedPaint(paints[1], paintOdometer)
  agedPaints[2] = getAgedPaint(paints[2], paintOdometer)
  agedPaints[3] = getAgedPaint(paints[3], paintOdometer)
  return agedPaints
end

local function setPartPaints(partId, paints, paintOdometer)
  paints[2] = paints[2] or paints[1]
  paints[3] = paints[3] or paints[1]
  partPaints[partId] = paints
  local agedPaints = getAgedPaints(paints, paintOdometer)
  setPartMeshPaints(partId, agedPaints)
end

local function setAllPartPaints(paints, paintOdometer)
  for partId, _ in pairs(activeParts) do
    setPartPaints(partId, paints, paintOdometer)
  end
end

local function setPaintCondition(partId, visual, defaultPaints)
  local visualState = visual
  if type(visual) == "number" then
    local visualValue = visual
    visualState = {
      paint = {
        odometer = linearScale(visualValue, 1, 0, 0, paintAgingConstants.wearEndOdometer),
        originalPaints = deepcopy(v.config.paints)
      }
    }
    --if we want to use a default paint other than what originally was in jbeam, apply this here
    if defaultPaints then
      defaultPaints[2] = defaultPaints[2] or defaultPaints[1]
      defaultPaints[3] = defaultPaints[3] or defaultPaints[1]
      visualState.paint.originalPaints = deepcopy(defaultPaints)
    end
  end
  if not visualState.paint then
    return
  end

  paintOdometerAbsoluteBaseValues[partId] = visualState.paint.odometer --TODO paint
  paintOdometerRelativeStartingValues[partId] = extensions.odometer.getRelativeRecording()

  if visualState.paint.originalPaints then
    setPartPaints(partId, visualState.paint.originalPaints, visualState.paint.odometer)
  end
end

local function getPaintCondition(partId)
  local canProvidePaintCondition = false
  local paintCondition = {odometer = 0, visualValue = 1}
  local hasFlexbody = false
  for _, partType in ipairs(partTypeData[partId] or {}) do
    local split = split(partType, ":")
    if split[1] == "jbeam" and split[2] == "flexbody" then
      hasFlexbody = true
    end
  end
  if hasFlexbody and paintOdometerAbsoluteBaseValues[partId] then
    local paintOdometer = (paintOdometerAbsoluteBaseValues[partId] or 0) + max(extensions.odometer.getRelativeRecording() - (paintOdometerRelativeStartingValues[partId] or 0), 0)
    paintCondition = {odometer = paintOdometer, visualValue = linearScale(paintOdometer, paintAgingConstants.wearStartOdometer, paintAgingConstants.wearEndOdometer, 1, 0), originalPaints = deepcopy(partPaints[partId])}
    canProvidePaintCondition = true
  end
  return paintCondition, canProvidePaintCondition
end

local function initCondition(partId, odometer, integrity, visual, defaultPaints)
  if hasSetPartCondition[partId] then
    log("E", "partCondition.initCondition", string.format("Trying to set part condition on part %q twice. Unexpected results might follow...", partId))
  end
  lastAppliedPartConditions[partId] = {odometer = odometer, integrity = integrity, visual = visual}
  hasSetPartCondition[partId] = true

  local partTypes = partTypeData[partId] or {}
  powertrain.setPartCondition(partTypes, odometer, integrity, visual)
  energyStorage.setPartCondition(partTypes, odometer, integrity, visual)
  beamstate.setPartCondition(partId, partTypes, odometer, integrity, visual)
  setPaintCondition(partId, visual, defaultPaints)

  partOdometerAbsoluteBaseValues[partId] = odometer
  partOdometerRelativeStartingValues[partId] = extensions.odometer.getRelativeRecording()

  extensions.odometer.startRecording()
end

local function getCondition(partId)
  local partOdometerValue = (partOdometerAbsoluteBaseValues[partId] or 0) + max(extensions.odometer.getRelativeRecording() - (partOdometerRelativeStartingValues[partId] or 0), 0)

  local partData = partTypeData[partId]
  local spawnTimeCondition = lastAppliedPartConditions[partId]
  if not spawnTimeCondition then
    log("E", "partCondition.getCondition", "No spawnTimeCondition found for part: " .. dumps(partId))
    return nil
  end

  local powertrainCondition, canProvidePowertrainIntegrityCondition, canProvidePowertrainVisualCondition = powertrain.getPartCondition(partData)
  local energyStorageCondition, canProvideEnergyStorageIntegrityCondition, canProvideEnergyStorageVisualCondition = energyStorage.getPartCondition(partData)
  local jbeamCondition, canProvideBeamstateCondition = beamstate.getPartCondition(partId, partData)
  local paintCondition, canProvidePaintCondition = getPaintCondition(partId, partData)

  local hasIntegrityCondition = canProvidePowertrainIntegrityCondition or canProvideEnergyStorageIntegrityCondition or canProvideBeamstateCondition
  local hasVisualCondition = canProvidePowertrainVisualCondition or canProvideEnergyStorageVisualCondition or canProvidePaintCondition
  if hasIntegrityCondition or hasVisualCondition then
    local integrityState
    local visualState

    if hasIntegrityCondition then
      integrityState = {
        powertrain = canProvidePowertrainIntegrityCondition and powertrainCondition.integrityState or nil,
        energyStorage = canProvideEnergyStorageIntegrityCondition and energyStorageCondition.integrityState or nil,
        jbeam = canProvideBeamstateCondition and jbeamCondition.integrityState or nil
      }
    end
    if hasVisualCondition then
      visualState = {
        powertrain = canProvidePowertrainVisualCondition and powertrainCondition.visualState or nil,
        energyStorage = canProvideEnergyStorageVisualCondition and energyStorageCondition.visualState or nil,
        jbeam = canProvideBeamstateCondition and jbeamCondition.visualState or nil,
        paint = canProvidePaintCondition and paintCondition or nil
      }
    end

    local integrityValue = min(powertrainCondition.integrityValue, energyStorageCondition.integrityValue, jbeamCondition.integrityValue)
    local visualValue = min(powertrainCondition.visualValue, energyStorageCondition.visualValue, jbeamCondition.visualValue, paintCondition.visualValue)

    return {odometer = partOdometerValue, integrityValue = integrityValue, visualValue = visualValue, integrityState = integrityState, visualState = visualState}
  else
    return {odometer = partOdometerValue, integrityValue = spawnTimeCondition.integrity, visualValue = spawnTimeCondition.visual, integrityState = nil, visualState = nil}
  end
end

local function reset()
  lastAppliedPartConditions = {}
  hasSetPartCondition = {}
  partOdometerAbsoluteBaseValues = {}
  partOdometerRelativeStartingValues = {}
  paintOdometerAbsoluteBaseValues = {}
  paintOdometerRelativeStartingValues = {}

  if not hasExecutedInitWork or not resetSnapshotKey then
    return
  end

  applyConditionSnapshot(resetSnapshotKey)
end

local function getConditions()
  if not hasExecutedInitWork then
    preparePartData()
  end

  if tableIsEmpty(hasSetPartCondition) then
    return false
  end

  local result = {}
  for partId, _ in pairs(activeParts) do
    xpcall(
      function()
        result[partId] = getCondition(partId)
        --log("I", "partCondition.getConditions", string.format("Got condition for partId %25s: ", partId) .. string.sub(serialize(result[partId]), 1, 100))
      end,
      function(err)
        log("E", "partCondition.getConditions", "Unable to get condition for partId " .. dumps(partId) .. ":")
        log("E", "partCondition.getConditions", err)
        log("E", "partCondition.getConditions", debug.traceback())
      end
    )
  end
  return result
end

local function initConditions(partsCondition, fallbackOdometer, fallbackIntegrityValue, fallbackVisualValue, defaultPaints)
  if not hasExecutedInitWork then
    preparePartData()
  end

  if not partsCondition then
    log("I", "partCondition.initConditions", "Parts condition not provided for vehicle, assuming fresh vehicle state for vehicle Id: " .. dumps(objectId))
    for partId, _ in pairs(activeParts) do
      initCondition(partId, fallbackOdometer or 0, fallbackIntegrityValue or 1, fallbackVisualValue or 1, defaultPaints)
    end
    createConditionSnapshot("reset")
    setResetSnapshotKey("reset")
    return
  end
  for partId in pairs(activeParts) do
    local odometer, integrity, visual
    local partCondition = partsCondition[partId]
    if partCondition then
      odometer = partCondition.odometer
      local integrityValue = partCondition.integrityValue
      local visualValue = partCondition.visualValue
      local integrityState = partCondition.integrityState
      local visualState = partCondition.visualState
      --odometer, integrityValue, visualValue, integrityState, visualState = unpack(partCondition, 1, table.maxn(partCondition))
      integrity = integrityState or integrityValue
      visual = visualState or visualValue
    end

    odometer = odometer or fallbackOdometer or 0
    integrity = integrity or fallbackIntegrityValue or 1
    visual = visual or fallbackVisualValue or 1
    if odometer and integrity --[[and visual--]] then
      initCondition(partId, odometer, integrity, visual, defaultPaints)
    else
      log("E", "partCondition.initConditions", "Missing odometer, integrityValue or visualValue for part name " .. dumps(partId) .. " in vehicle " .. dumps(objectId) .. ": " .. dumps(partCondition))
    end
  end

  createConditionSnapshot("reset")
  setResetSnapshotKey("reset")
end

--used to make blind calls against partCondition to make sure that everything is inited correctly
local function ensureConditionsInit(fallbackOdometer, fallbackIntegrityValue, fallbackVisualValue)
  if tableIsEmpty(hasSetPartCondition) then
    initConditions(nil, fallbackOdometer, fallbackIntegrityValue, fallbackVisualValue)
  end
end

local function testInit()
  hasSetPartCondition = {} --kill data from last init to avoid dual init warning
  M.initConditions(nil, 812812000, 1, 1)
end

local function testLoad()
  local data = jsonDecode(readFile("partConditionTest.json") or "{}")
  hasSetPartCondition = {} --kill data from last init to avoid dual init warning
  M.initConditions(data, 0, 1, 1)
end

local function testSave()
  if tableIsEmpty(hasSetPartCondition) then
    M.initConditions(nil, 812812000, 1, 0.5)
  end
  local data = M.getConditions()

  writeFile("partConditionTest.json", jsonEncodePretty(data))
end

M.reset = reset

M.getConditions = getConditions
M.initConditions = initConditions
M.ensureConditionsInit = ensureConditionsInit

M.createConditionSnapshot = createConditionSnapshot
M.applyConditionSnapshot = applyConditionSnapshot
M.deleteConditionSnapshots = deleteConditionSnapshots
M.setResetSnapshotKey = setResetSnapshotKey

M.getRootPartOdometerValue = getRootPartOdometerValue
M.getRootPartTripValue = getRootPartTripValue
M.setPartMeshPaints = setPartMeshPaints

M.testSave = testSave
M.testInit = testInit
M.testLoad = testLoad

M.setPartPaints = setPartPaints
M.setAllPartPaints = setAllPartPaints

return M
