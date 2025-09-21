-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local im = extensions.ui_imgui
local jbeamIO = require('jbeam/io')
local jsonAST = require('json-ast')

local wndName = "JBeam Spellchecker"
M.menuEntry = "JBeam Spellchecker"

local windowOpen = im.BoolPtr(false)

local allSections = {
  flexbodies = {
    pos = true,
    rot = true,
    scale = true,
    deformGroup = true,
    deformMaterialBase = true,
    deformMaterialDamaged = true,
    deformSound = true,
    deformVolume = true,
    disableMeshBreaking = true,
    ignoreNodeOffset = true,
  },
  props = {
    baseTranslation = true,
    baseTranslationGlobal = true,
    baseTranslationGlobalElastic = true,
    baseTranslationGlobalRigid = true,
    baseRotation = true,
    baseRotationGlobal = true,
    min = true,
    max = true,
    offset = true,
    multiplier = true,
    deformGroup = true,
    breakGroup = true,
    lightInnerAngle = true,
    lightOuterAngle = true,
    lightBrightness = true,
    lightRange = true,
    lightColor = true,
    lightAttenuation = true,
    lightCastShadows = true,
    flareName = true,
    flareScale = true,
    cookieName = true,
    texSize = true,
    shadowSoftness = true,
    optional = true,
  },
  nodes = {
    frictionCoef = true,
    nodeMaterial = true,
    selfCollision = true,
    collision = true,
    nodeWeight = true,
    group = true,
    chemEnergy = true,
    burnRate = true,
    flashPoint = true,
    specHeat = true,
    smokePoint = true,
    selfIgnitionCoef = true,
    vaporPoint = true,
    fixed = true,
    couplerStrength = true,
    couplerTag = true,
    couplerRadius = true,
    breakGroup = true,
    couplerLock = true,
    importElectrics = true,
    importInputs = true,
    volumeCoef = true,
    noLoadCoef = true,
    fullLoadCoef = true,
    stribeckVelMult = true,
    stribeckExponent = true,
    softnessCoef = true,
    treadCoef = true,
    tag = true,
    loadSensitivitySlope = true,
    pairedNode = true,
    afterFireAudioCoef = true,
    afterFireVisualCoef = true,
    afterFireVolumeCoef = true,
    afterFireMufflingCoef = true,
    exhaustAudioMufflingCoef = true,
    exhaustAudioGainChange = true,
    engineGroup = true,
    baseTemp = true,
    isExhaust = true,
    conductionRadius = true,
    staticCollision = true,
    containerBeam = true,
    selfIgnition = true,
    nodeOffset = true,
    couplerStartRadius = true,
    couplerWeld = true,
    slidingFrictionCoef = true,
    deformGroup = true,
    impactGenericEvent = true,
  },
  beams = {
    beamPrecompression = true,
    beamType = true,
    beamLongBound = true,
    beamShortBound = true,
    deformLimitExpansion = true,
    beamSpring = true,
    beamDamp = true,
    beamDeform = true,
    beamStrength = true,
    deformationTriggerRatio = true,
    deformGroup = true,
    deformLimitStress = true,
    breakGroup = true,
    breakGroupType = true,
    beamLimitDamp = true,
    beamLimitSpring = true,
    springExpansion = true,
    beamLimitDampRebound = true,
    beamDampRebound = true,
    beamDampFast = true,
    beamDampReboundFast = true,
    beamDampVelocitySplit = true,
    optional = true,
    deformLimit = true,
    disableMeshBreaking = true,
    disableTriangleBreaking = true,
    dampExpansion = true,
    transitionZone = true,
    precompressionRange = true,
    beamPrecompressionTime = true,
    boundZone = true,
    dampCutoffHz = true,
    shortBoundRange = true,
    longBoundRange = true,
    highlight = true,
    tag = true,
    name = true,
    pressurePSI = true,
    volumeCoef = true,
    surface = true,
    beamDampVelocitySplitRebound = true,
    maxStress = true,
    colorFactor = true,
    attackFactor = true,
    volumeFactor = true,
    decayFactor = true,
    pitchFactor = true,
    soundFile = true,
    containerBeam = true,
    isExhaust = true,
    noiseFactor = true,
    ['id3:'] = true,
    hydraulicsMinDamp = true,
  },
  triangles = {
    dragCoef = true,
    liftCoef = true,
    stallAngle = true,
    triangleType = true,
    pressureGroup = true,
    pressurePSI = true,
    pressure = true,
    breakGroup = true,
    optional = true,
    groundModel = true,
    group = true,
    externalCollisionBias = true,
    skinDragCoef = true,

  },
  hydros = {
    beamPrecompression = true,
    beamType = true,
    beamLongBound = true,
    beamShortBound = true,
    deformLimitExpansion = true,
    beamSpring = true,
    beamDamp = true,
    beamDeform = true,
    beamStrength = true,
    deformationTriggerRatio = true,
    deformGroup = true,
    deformLimitStress = true,
    breakGroup = true,
    breakGroupType = true,
    beamLimitDamp = true,
    beamLimitSpring = true,
    springExpansion = true,
    beamLimitDampRebound = true,
    beamDampRebound = true,
    beamDampFast = true,
    beamDampReboundFast = true,
    beamDampVelocitySplit = true,
    deformLimit = true,
    disableMeshBreaking = true,
    disableTriangleBreaking = true,
    dampExpansion = true,
    transitionZone = true,
    precompressionRange = true,
    beamPrecompressionTime = true,
    boundZone = true,
    dampCutoffHz = true,
    shortBoundRange = true,
    longBoundRange = true,
    highlight = true,
    tag = true,
    name = true,
    pressurePSI = true,
    volumeCoef = true,
    surface = true,
    beamDampVelocitySplitRebound = true,
    maxStress = true,
    colorFactor = true,
    attackFactor = true,
    volumeFactor = true,
    decayFactor = true,
    pitchFactor = true,
    soundFile = true,
    containerBeam = true,
    isExhaust = true,
    noiseFactor = true,
    hydraulicsMinDamp = true,

    inputSource = true,
    factor = true,
    outLimit = true,
    inLimit = true,
    inputFactor = true,
    inputCenter = true,
    inRate = true,
    outRate = true,
    steeringWheelLock = true,
    autoCenterRate = true,
    optional = true,
  },
  torsionbars = {
    spring = true,
    damp = true,
    deform = true,
    strength = true,
    precompressionAngle = true,
    optional = true,
    name = true,
  },
  torsionHydros = {
    factor = true,
    inLimit = true,
    outLimit = true,
    inputFactor = true,
    inRate = true,
    outRate = true,
    autoCenterRate = true,
    inputSource = true,
    inputCenter = true,
    inputInLimit = true,
    inputOutLimit = true,
    steeringWheelLock = true,
    extentFactor = true,

    spring = true,
    damp = true,
    deform = true,
    strength = true,
    precompressionAngle = true,
    optional = true,
    name = true,
  },
  variables = {
    subCategory = true,
    stepDis = true,
    minDis = true,
    maxDis = true,
  },
  --[[
  information = {},
  slotType = {},
  slots = {},
  pressureWheels = {},
  slidenodes = {},
  mainEngine = {},
  powertrain = {},
  turbocharger = {},
  controller = {},
  vehicleController = {},
  gearbox = {},
  soundConfig = {},
  soundConfigExhaust = {},
  input = {},
  refNodes = {},
  events = {},
  sounds = {},
  cameraChase = {},
  cameraExternal = {},
  camerasInternal = {},
  ropes = {},
  rails = {},
  triggers = {},
  ties = {},
  glowMap = {},
  triggerEventLinks = {},
  gauge = {},
  skinName = {},
  quads = {},
  rotators = {},
  soundscape = {},
  energyStorage = {},
  licenseplateFormat = {},
  electrics = {},
  ]]--
}

-- From common/jbeam/io.lua that parses a JBeam file from its filename
local function parseFile(filename)
  local content = readFile(filename)
  if content then
    local ok, data = pcall(json.decode, content)
    if ok == false then
      log('E', "jbeam.parseFile","unable to decode JSON: "..tostring(filename))
      log('E', "jbeam.parseFile","JSON decoding error: "..tostring(data))
      return nil
    end
    return data
  else
    log('E', "jbeam.parseFile","unable to read file: "..tostring(filename))
  end
end

local function analyzeJBeamFile(filePath, fileName, jbeamFileData)
  for parts, partData in pairs(jbeamFileData) do
    for section, sectionData in pairs(partData) do
      --if section == 'beams' or section == 'nodes' or section == 'triangles' or section == 'flexbodies' or section == 'props' or section == 'torsionbars' then
      --if section == 'torsionHydros' then
      if allSections[section] then
        local header = sectionData[1]
        if type(header) ~= "table" then
          log('W', "", filePath .. " *** Invalid table header: " .. dumpsz(header, 2))
          return -1
        end
        if tableIsDict(header) then
          log('W', "", filePath .. " *** Invalid table header, must be a list, not a dict: "..dumps(header))
          return -1
        end

        local headerSize = #header
        local headerSize1 = headerSize + 1

        -- remove the header from the data, as we dont need it anymore
        table.remove(sectionData, 1)
        --log('D', ""header size: "..headerSize)

        -- walk the list entries
        for rowKey, rowValue in ipairs(sectionData) do
          local mods = nil

          if type(rowValue) ~= "table" then
            log('W', "", filePath .. " *** Invalid table row: "..dumps(rowValue))
            return -1
          end
          if tableIsDict(rowValue) then
            -- case where options is a dict on its own, filling a whole line
            mods = rowValue
          else
            --log('D', "" *** "..tostring(rowKey).." = "..tostring(rowValue).." ["..type(rowValue).."]")

            -- allow last type to be the options always
            --[[
            if #rowValue > headerSize + 1 then -- and type(rowValue[#rowValue]) ~= "table" then
              log('W', "", "*** Invalid table header, must be as long as all table cells (plus one additional options column):")
              log('W', "", "*** Table header: "..dumps(header))
              log('W', "", "*** Mismatched row: "..dumps(rowValue))
              return -1
            end
            ]]--

            -- walk the table row
            -- replace row: reassociate the header colums as keys to the row cells

            -- check if inline options are provided, merge them then
            for rk = headerSize1, #rowValue do
              local rv = rowValue[rk]
              if type(rv) == 'table' and tableIsDict(rv) and #rowValue > headerSize then
                mods = rv
                break
              end
            end
          end

          if mods then
            for modifier, _ in pairs(mods) do
              if not allSections[section][modifier] then
                log('E', '', filePath .. ': Section: ' .. section .. ' Modifier: "' .. modifier .. '" not valid!')
              end
            end
          end
        end

        --[[
        for _, row in ipairs(sectionData) do
          local mods = nil

          if tableIsDict(row) then
            mods = row
          else
            mods = row[#row]
          end

          if mods then
            for modifier, _ in pairs(mods) do
              if not allSections.beams[modifier] then
                log('E', '', filePath .. ': Modifier "' .. modifier .. '" not valid!')
              end
            end
          end
        end
        ]]--
      end

      --[[
      if not allSections[section] then
        log('E', '', filename .. ': Section "' .. section .. '" is not valid!')
      end
      ]]--
    end
  end
end

local function analyze()
  local filePaths = FS:findFiles('vehicles', "*.jbeam", -1, false, false)

  for _, filePath in ipairs(filePaths) do
    local dir, fileName, _ = path.splitWithoutExt(filePath)

    local data = parseFile(filePath)

    analyzeJBeamFile(filePath, fileName, data)

    --break
  end
end

local function onUpdate()
  if windowOpen[0] ~= true then return end

  if im.Begin(wndName, windowOpen) then
    if im.Button("Start Analysis") then
      analyze()
    end
  end

  ::continue::
  im.End()
end

local function open()
  windowOpen[0] = true
end

local function onSerialize()
  return {
    windowOpen = windowOpen[0],
  }
end

local function onDeserialized(data)
  windowOpen[0] = data.windowOpen
end

M.open = open

M.onUpdate = onUpdate

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M