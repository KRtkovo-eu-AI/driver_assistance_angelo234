-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local im = extensions.ui_imgui
local jbeamIO = require('jbeam/io')
local jsonAST = require('json-ast')

local tableInsert, tableClear = table.insert, table.clear

local wndName = "JBeam Modifier Leaking Visualizer"
M.menuEntry = "JBeam Modifier Leaking Visualizer"

local windowOpen = im.BoolPtr(false)

local tableFlags = bit.bor(im.TableFlags_ScrollX, im.TableFlags_ScrollY, im.TableFlags_RowBg, im.TableFlags_BordersOuter, im.TableFlags_BordersV, im.TableFlags_Hideable)

local imRedCol = im.ImVec4(1,0,0,1)
local imYellowCol = im.ImVec4(1,1,0,1)

local useDefaultValuesForLeaking = im.BoolPtr(false)

local modifiersDefaultValues = {
  flexbodies = {
    pos = "",
    rot = "",
    scale = "",
    deformGroup = {[""] = true, [false] = true},
    deformMaterialBase = "",
    deformMaterialDamaged = "",
    deformSound = {[""] = true, [false] = true},
    deformVolume = {[""] = true, [false] = true},
    ignoreNodeOffset = {[""] = true, [false] = true},
    nodeOffset = {[""] = true, [false] = true},
  },
  props = {
    baseTranslation = {[""] = true, [false] = true},
    baseTranslationGlobal = {[""] = true, [false] = true},
    baseTranslationGlobalElastic = {[""] = true, [false] = true},
    baseTranslationGlobalRigid = {[""] = true, [false] = true},
    baseRotation = {[""] = true, [false] = true},
    baseRotationGlobal = {[""] = true, [false] = true},
    min = {[0] = true, [""] = true},
    max = {[100] = true, [""] = true},
    offset = {[0] = true, [""] = true},
    multiplier = {[1] = true, [""] = true},
    deformGroup = "",
    breakGroup = "",
    lightInnerAngle = {[40] = true, [""] = true, [false] = true},
    lightOuterAngle = {[45] = true, [""] = true, [false] = true},
    lightBrightness = {[1] = true, [""] = true, [false] = true},
    lightRange = {[10] = true, [""] = true, [false] = true},
    lightColor = {[""] = true, [false] = true},
    lightAttenuation = {[""] = true, [false] = true},
    lightCastShadows = {[""] = true, [false] = true},
    flareName = {["vehicleDefaultLightflare"] = true, [""] = true, [false] = true},
    flareScale = {[1] = true, [""] = true, [false] = true},
    cookieName = {[""] = true, [false] = true},
    texSize = {[256] = true, [""] = true, [false] = true},
    shadowSoftness = {[1] = true, [""] = true, [false] = true},
    optional = {[""] = true, [false] = true},
  },
  nodes = {
    frictionCoef = 1,
    slidingFrictionCoef = 1,
    nodeMaterial = "",
    selfCollision = false,
    collision = true,
    nodeWeight = 25,
    group = "",
    chemEnergy = false,
    flashPoint = false,
    smokePoint = false,
    specHeat = false,
    vaporPoint = false,
    selfIgnitionCoef = false,
    burnRate = false,
    baseTemp = false,
    conductionRadius = false,
    containerBeam = false,
    selfIgnition = false,
    fixed = false,
    couplerStrength = "FLT_MAX",
    couplerTag = nil,
    couplerRadius = nil,
    breakGroup = nil,
    couplerLock = false,
    importElectrics = nil,
    importInputs = nil,
    volumeCoef = 0.1,
    noLoadCoef = 1,
    fullLoadCoef = 0,
    stribeckVelMult = 1,
    stribeckExponent = 1.75,
    softnessCoef = 0.5,
    treadCoef = 0.5,
    tag = "",
    loadSensitivitySlope = 0,
    pairedNode = "",
    afterFireAudioCoef = nil,
    afterFireVisualCoef = nil,
    afterFireVolumeCoef = nil,
    afterFireMufflingCoef = nil,
    exhaustAudioMufflingCoef = nil,
    exhaustAudioGainChange = nil,
    engineGroup = "",
    isExhaust = nil,
    staticCollision = nil,
    nodeOffset = nil,
    couplerStartRadius = nil,
    couplerWeld = nil,
    deformGroup = "",
    impactGenericEvent = nil,
  },
  beams = {
    beamPrecompression = {[1] = true, [""] = true, [false] = true},
    beamType = {["|NORMAL"] = true, [0] = true, [""] = true, [false] = true},
    beamLongBound = {[1] = true, [""] = true, [false] = true},
    beamShortBound = {[1] = true, [""] = true, [false] = true},
    beamSpring = {[4300000] = true, [""] = true, [false] = true},
    beamDamp = {[580] = true, [""] = true, [false] = true},
    beamDeform = {[220000] = true, [""] = true, [false] = true},
    beamStrength = {["FLT_MAX"] = true, [""] = true, [false] = true},
    deformLimitExpansion = {["FLT_MAX"] = true, [""] = true, [false] = true},
    deformationTriggerRatio = "",
    deformGroup = "",
    deformLimitStress = {["FLT_MAX"] = true, [""] = true, [false] = true},
    breakGroup = "",
    breakGroupType = {[0] = true, [""] = true, [false] = true},
    beamLimitDamp = {[1] = true, [""] = true, [false] = true},
    beamLimitSpring = {[1] = true, [""] = true, [false] = true},
    springExpansion = {[4300000] = true, [""] = true, [false] = true},
    beamLimitDampRebound = {[1] = true, [""] = true, [false] = true},
    beamDampRebound = {[580] = true, [""] = true, [false] = true},
    beamDampFast = {[580] = true, [""] = true, [false] = true},
    beamDampReboundFast = {[1] = true, [""] = true, [false] = true},
    beamDampVelocitySplit = {["FLT_MAX"] = true, [""] = true, [false] = true},
    beamDampVelocitySplitRebound = {["FLT_MAX"] = true, [""] = true, [false] = true},
    optional = {[""] = true, [false] = true},
    deformLimit = {["FLT_MAX"] = true, [""] = true, [false] = true},
    disableMeshBreaking = {[""] = true, [false] = true},
    disableTriangleBreaking = {[""] = true, [false] = true},
    dampExpansion = {[""] = true, [false] = true},
    transitionZone = {[0] = true, [""] = true, [false] = true},
    precompressionRange = {[""] = true, [false] = true},
    beamPrecompressionTime = {[""] = true, [false] = true},
    boundZone = {[1] = true, [""] = true, [false] = true},
    dampCutoffHz = {[0] = true, [""] = true, [false] = true},
    shortBoundRange = {[0] = true, [""] = true, [false] = true},
    longBoundRange = {[0] = true, [""] = true, [false] = true},
    highlight = {[""] = true, [false] = true},
    tag = "",
    name = "",
    pressure = "",
    pressurePSI =  {[""] = true, [30] = true},
    volumeCoef =  {[1] = true, [""] = true, [false] = true},
    surface =  {[1] = true, [""] = true, [false] = true},
    maxStress = nil,
    colorFactor = nil,
    attackFactor = nil,
    volumeFactor = nil,
    decayFactor = nil,
    pitchFactor = nil,
    soundFile = "",
    containerBeam = nil,
    isExhaust = nil,
    noiseFactor = nil,
    ["id3:"] = nil,
    hydraulicsMinDamp = nil,
  },
  triangles = {
    dragCoef = {[100] = true, [""] = true, [false] = true},
    liftCoef = {[100] = true, [""] = true, [false] = true},
    stallAngle = {[0.58] = true, [""] = true, [false] = true},
    triangleType = {["NORMALTYPE"] = true, [0] = true, [""] = true, [false] = true},
    pressureGroup = "",
    pressure = "",
    pressurePSI =  {[""] = true, [30] = true},
    breakGroup = "",
    optional = {[""] = true, [false] = true},
    groundModel = {["asphalt"] = true, [""] = true, [false] = true},
    group = "",
    externalCollisionBias = {[""] = true, [false] = true},
    skinDragCoef = {[0] = true, [""] = true, [false] = true},

  },
  hydros = {
    beamPrecompression = {[1] = true, [""] = true, [false] = true},
    beamType = {["|NORMAL"] = true, [0] = true, [""] = true, [false] = true},
    beamLongBound = {[1] = true, [""] = true, [false] = true},
    beamShortBound = {[1] = true, [""] = true, [false] = true},
    beamSpring = {[4300000] = true, [""] = true, [false] = true},
    beamDamp = {[580] = true, [""] = true, [false] = true},
    beamDeform = {[220000] = true, [""] = true, [false] = true},
    beamStrength = {["FLT_MAX"] = true, [""] = true, [false] = true},
    deformLimitExpansion = {["FLT_MAX"] = true, [""] = true, [false] = true},
    deformationTriggerRatio = "",
    deformGroup = "",
    deformLimitStress = {["FLT_MAX"] = true, [""] = true, [false] = true},
    breakGroup = "",
    breakGroupType = {[0] = true, [""] = true, [false] = true},
    beamLimitDamp = {[1] = true, [""] = true, [false] = true},
    beamLimitSpring = {[1] = true, [""] = true, [false] = true},
    springExpansion = {[4300000] = true, [""] = true, [false] = true},
    beamLimitDampRebound = {[1] = true, [""] = true, [false] = true},
    beamDampRebound = {[580] = true, [""] = true, [false] = true},
    beamDampFast = {[580] = true, [""] = true, [false] = true},
    beamDampReboundFast = {[1] = true, [""] = true, [false] = true},
    beamDampVelocitySplit = {["FLT_MAX"] = true, [""] = true, [false] = true},
    beamDampVelocitySplitRebound = {["FLT_MAX"] = true, [""] = true, [false] = true},
    optional = {[""] = true, [false] = true},
    deformLimit = {["FLT_MAX"] = true, [""] = true, [false] = true},
    disableMeshBreaking = {[""] = true, [false] = true},
    disableTriangleBreaking = {[""] = true, [false] = true},
    dampExpansion = {[""] = true, [false] = true},
    transitionZone = {[0] = true, [""] = true, [false] = true},
    precompressionRange = {[""] = true, [false] = true},
    beamPrecompressionTime = {[""] = true, [false] = true},
    boundZone = {[1] = true, [""] = true, [false] = true},
    dampCutoffHz = {[0] = true, [""] = true, [false] = true},
    shortBoundRange = {[0] = true, [""] = true, [false] = true},
    longBoundRange = {[0] = true, [""] = true, [false] = true},
    highlight = {[""] = true, [false] = true},
    tag = "",
    name = "",
    pressure = "",
    pressurePSI =  {[""] = true, [30] = true},
    volumeCoef =  {[1] = true, [""] = true, [false] = true},
    surface =  {[1] = true, [""] = true, [false] = true},
    maxStress = nil,
    colorFactor = nil,
    attackFactor = nil,
    volumeFactor = nil,
    decayFactor = nil,
    pitchFactor = nil,
    soundFile = "",
    containerBeam = nil,
    isExhaust = nil,
    noiseFactor = nil,
    ["id3:"] = nil,
    hydraulicsMinDamp = nil,

    inputSource = nil,
    factor = nil,
    outLimit = nil,
    inLimit = nil,
    inputFactor = nil,
    inputCenter = nil,
    inRate = nil,
    outRate = nil,
    steeringWheelLock = nil,
    autoCenterRate = nil,
  },
  torsionbars = {
    spring = nil,
    damp = nil,
    deform = nil,
    strength = nil,
    precompressionAngle = nil,
    optional = nil,
    name = nil,
  },
  torsionHydros = {
    factor = nil,
    inLimit = nil,
    outLimit = nil,
    inputFactor = nil,
    inRate = nil,
    outRate = nil,
    autoCenterRate = nil,
    inputSource = nil,
    inputCenter = nil,
    inputInLimit = nil,
    inputOutLimit = nil,
    steeringWheelLock = nil,
    extentFactor = nil,

    spring = nil,
    damp = nil,
    deform = nil,
    strength = nil,
    precompressionAngle = nil,
    optional = nil,
    name = nil,
  },
  variables = {
    subCategory = nil,
    stepDis = nil,
    minDis = nil,
    maxDis = nil,
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

local runTest = false

local sectionNamesSorted = nil
local sectionsPartNamesSorted = nil
local sectionsPartNamesToIdxSorted = nil
local sectionsModNamesSorted = nil
local sectionsWithLeakingMods = nil
local data = nil
local sectionViewing = 1

local particles = require("particles")

local materials, materialsMap = particles.getMaterialsParticlesTable()

-- these are defined in C, do not change the values
local NORMALTYPE = 0
local NODE_FIXED = 1
local NONCOLLIDABLE = 2
local BEAM_ANISOTROPIC = 1
local BEAM_BOUNDED = 2
local BEAM_PRESSURED = 3
local BEAM_LBEAM = 4
local BEAM_HYDRO = 6
local BEAM_SUPPORT = 7


local specialVals = {FLT_MAX = math.huge, MINUS_FLT_MAX = -math.huge}
local typeIds = {
  NORMAL = NORMALTYPE,
  HYDRO = BEAM_HYDRO,
  ANISOTROPIC = BEAM_ANISOTROPIC,
  TIRESIDE = BEAM_ANISOTROPIC,
  BOUNDED = BEAM_BOUNDED,
  PRESSURED = BEAM_PRESSURED,
  SUPPORT = BEAM_SUPPORT,
  LBEAM = BEAM_LBEAM,
  FIXED = NODE_FIXED,
  NONCOLLIDABLE = NONCOLLIDABLE,
  SIGNAL_LEFT = 1,   -- GFX_SIGNAL_LEFT
  SIGNAL_RIGHT = 2,  -- GFX_SIGNAL_RIGHT
  HEADLIGHT = 4,     -- GFX_HEADLIGHT
  BRAKELIGHT = 8,    -- GFX_BRAKELIGHT
  RUNNINGLIGHT = 16, -- GFX_RUNNINGLIGHT
  REVERSELIGHT = 32, -- GFX_REVERSELIGHT
}

local function replaceSpecialValues(val)
  local typeval = type(val)
  if typeval == "table" then
    -- recursive replace
    for k, v in pairs(val) do
      val[k] = replaceSpecialValues(v)
    end
    return val
  end
  if typeval ~= "string" then
    -- only replace strings
    return val
  end

  if specialVals[val] then return specialVals[val] end

  if string.find(val, '|', 1, true) then
    local parts = split(val, "|", 999)
    local ival = 0
    for i = 2, #parts do
      local valuePart = parts[i]
      -- is it a node material?
      if valuePart:sub(1,3) == "NM_" then
        ival = particles.getMaterialIDByName(materials, valuePart:sub(4))
        --log('D', "jbeam.replaceSpecialValues", "replaced "..valuePart.." with "..ival)
      end
      ival = bit.bor(ival, typeIds[valuePart] or 0)
    end
    return ival
  end
  return val
end

-- Get all parts as a list and store output in 'parts' var
local function getPartsRec(ioCtx, partTreeNode, parts)
  if not partTreeNode.children then return end
  local part, jbeamFilename = jbeamIO.getPart(ioCtx, partTreeNode.chosenPartName)
  if part then
    tableInsert(parts, {part = part, jbeamFilename = jbeamFilename})
    local slots = part.slots2 or part.slots
    if slots ~= nil then
      for _, slot in ipairs(slots) do
        local slotId = slot.name or slot.type
        local childPartTreeNode = partTreeNode.children[slotId]
        if childPartTreeNode then
          getPartsRec(ioCtx, childPartTreeNode, parts)
        end
      end
    end
  end
end

local function getParts()
  local parts = {}
  local ioCtx = vEditor.vehData.ioCtx
  getPartsRec(ioCtx, vEditor.vehData.config.partsTree, parts)
  return parts
end

local function getPartsWithASTData(parts)
  local output = {}
  local jbeamCache = {}

  for k, partData in ipairs(parts) do
    local jbeamFilename = partData.jbeamFilename
    local partName = partData.part.partName
    if not jbeamCache[jbeamFilename] then
      local str = readFile(jbeamFilename)
      if str then
        jbeamCache[jbeamFilename] = jsonAST.parse(str, true)
      end
    end
    if jbeamCache[jbeamFilename] then
      tableInsert(output,
        {
          name = partName,
          jbeamFilename = jbeamFilename,
          astHierarchy = jbeamCache[jbeamFilename] and jbeamCache[jbeamFilename].transient.hierarchy or nil,
          data = jbeamCache[jbeamFilename] and jbeamCache[jbeamFilename].transient.luaDataRaw[partName] or nil
        }
      )
    else
      log('E', '', 'Unable to get AST data for following JBeam file, and so its parts are not shown: ' .. jbeamFilename)
    end
  end

  return output
end

local function getLineASTNodes(astHierarchy, nodeIdx, outputNodeIdxs)
  tableInsert(outputNodeIdxs, nodeIdx)
  for _, ni in ipairs(astHierarchy[nodeIdx] or {}) do
    getLineASTNodes(astHierarchy, ni, outputNodeIdxs)
  end
end

local function initAnalyzeModifiersLeaking(partsWithASTData, outSectionsAllModNames, sectionsMods, outMods, outSectionsPartNames, outSectionsPartNameToIdx, modsScalers, partsModsScalers, testSectionsCounter, testSectionsWrongCounter)
  -- First initialize by getting all modifiers per section
  -- and only save sections that have row modifiers defined
  for k, partData in ipairs(partsWithASTData) do
    local part = partData.data
    local partName = partData.name

    for sectionName, section in pairs(part) do
      if type(section) == "table" and #section > 1 then
        -- Go through each line in part's current section
        for lineNum, lineData in ipairs(section) do
          if lineNum > 1 and tableIsDict(lineData) then
            -- Line is dictionary, so it declares modifier(s)
            for mod, modVal in pairs(lineData) do
              if mod ~= "__astNodeIdx" then
                if not outSectionsAllModNames[sectionName] then
                  outSectionsAllModNames[sectionName] = {}
                  sectionsMods[sectionName] = sectionsMods[sectionName] or {}
                  outMods[sectionName] = outMods[sectionName] or {}

                  if runTest and vEditor.vdata[sectionName] and #vEditor.vdata[sectionName] > 0
                  and sectionName ~= "nodes" and sectionName ~= "beams" and sectionName ~= "triangles" then
                    testSectionsCounter[sectionName] = 0
                    testSectionsWrongCounter[sectionName] = 0
                  end
                end
                outSectionsAllModNames[sectionName][mod] = true
              end
            end
          end
        end
      elseif sectionName:match('^scale') and type(section) == "number" then
        modsScalers[sectionName:sub(6, #sectionName)] = {modVal = section, partPath = partName}
      end
    end
  end
  for mod, modData in pairs(modsScalers) do
    partsModsScalers[modData.partPath] = partsModsScalers[modData.partPath] or {}
    partsModsScalers[modData.partPath][mod] = modData.modVal
  end
end

-- Check if current modifiers match vehicle data section item
local function testModifiers(sectionName, lineData, testSectionsCounter, testNodeNameToCID, newSectionModifiers, testSectionsWrongCounter)
  local valid = true

  local currCount = testSectionsCounter[sectionName]
  local item = nil

  if currCount then
    item = vEditor.vdata[sectionName][currCount]

    if sectionName == "nodes" then
      item = vEditor.vdata[sectionName][testNodeNameToCID[lineData[1]]]

      --local cid =
      --[[
      if not (vEditor.vdata.nodes[cid] == currCount) then
        log('E', '', "node ids mismatch: expected: " .. dumps(lineData[1]) .. " got: " .. dumps(cid))
        valid = false
        goto continue
      end
      ]]--

    elseif sectionName == "beams" then
      local n1 = vEditor.vdata.nodes[item.id1]
      local n2 = vEditor.vdata.nodes[item.id2]

      local id1 = tostring(n1.name or n1.cid)
      local id2 = tostring(n2.name or n2.cid)

      if not (id1 == lineData[1] or id1 == lineData[2] and id2 == lineData[1] or id2 == lineData[2]) then
        log('E', '', "beam ids mismatch: expected: " .. dumps(lineData[1]) .. "," .. dumps(lineData[2]) .. " got: " .. dumps(id1) .. "," .. dumps(id2))
        valid = false
        goto continue
      end
    end

    if sectionName == "nodes" then
      newSectionModifiers.group = nil
      newSectionModifiers.engineGroup = nil
      if newSectionModifiers.collision and newSectionModifiers.collision.modVal == true then newSectionModifiers.collision.modVal = nil end -- the default
      if newSectionModifiers.chemEnergy and (type(newSectionModifiers.chemEnergy.modVal) ~= 'number' or newSectionModifiers.chemEnergy.modVal == 0) then newSectionModifiers.chemEnergy.modVal = nil end
      if newSectionModifiers.flashPoint and not newSectionModifiers.flashPoint.modVal then
        -- if not in fire system, clean out the data
        newSectionModifiers.flashPoint = nil
        newSectionModifiers.smokePoint = nil
        newSectionModifiers.specHeat = nil
        newSectionModifiers.vaporPoint = nil
        newSectionModifiers.selfIgnitionCoef = nil
        newSectionModifiers.burnRate = nil
        newSectionModifiers.baseTemp = nil
        newSectionModifiers.conductionRadius = nil
        newSectionModifiers.containerBeam = nil
        newSectionModifiers.selfIgnition = nil
      end
      if newSectionModifiers.selfCollision and not newSectionModifiers.selfCollision.modVal then newSectionModifiers.selfCollision.modVal = nil end -- the default
    elseif sectionName == "beams" then
      --if newSectionModifiers.beamType.modVal == 0 then newSectionModifiers.beamType.modVal = nil end -- the default
      --if newSectionModifiers.beamPrecompression.modVal == 1 then newSectionModifiers.beamPrecompression.modVal = nil end
      --if newSectionModifiers.breakGroupType.modVal == 0 then newSectionModifiers.breakGroupType.modVal = nil end
      --if newSectionModifiers.disableTriangleBreaking.modVal == false then newSectionModifiers.disableTriangleBreaking.modVal = nil end
      --if newSectionModifiers.disableMeshBreaking.modVal == false then newSectionModifiers.disableMeshBreaking.modVal = nil end
    elseif sectionName == "hydros" then
      newSectionModifiers.beamType.modVal = BEAM_HYDRO
    end

    for mod, modData in pairs(newSectionModifiers) do
      --local vehDataModVal = item[mod]
      local modVal = replaceSpecialValues(modData.modVal)
      if modVal == "" then
        modVal = nil
      end
      newSectionModifiers[mod].modVal = modVal
    end

    for mod, modData in pairs(newSectionModifiers) do
      local vehDataModVal = item[mod]
      --[[
      local modVal = replaceSpecialValues(modData.modVal)
      if modVal == "" then
        modVal = nil
      end
      ]]--
      local modVal = modData.modVal
      local vehDataModValType = type(vehDataModVal)
      local modValType = type(modVal)

      if --vehDataModValType == "number" and modValType == "number" and not (vehDataModVal - 0.001 < modVal and vehDataModVal + 0.001 > modVal)
        dumps(vehDataModVal) ~= dumps(modVal) then
        log('E', '', sectionName .. "." .. mod .. ": expected: " .. dumps(vehDataModVal) .. " " .. vehDataModValType .. " got: " .. dumps(modVal) .. " " .. modValType)
        valid = false
        break
      end
    end
  end
  ::continue::

  if not valid then
    --log('E', '', sectionName .. "[" .. currCount .. "]." .. mod .. " ~= " .. tostring(vehDataModVal))
    log('E', '', sectionName .. "[" .. currCount .. "] wrong")
    print("Full JBeam Item Data:")
    print(dumps(item))
    print("Algorithm's Current Modifiers:")
    print(dumps(newSectionModifiers))
    testSectionsWrongCounter[sectionName] = testSectionsWrongCounter[sectionName] + 1
  end

  if currCount then
    testSectionsCounter[sectionName] = currCount + 1
  end
end

local function analyzeModifiersLeaking(partsWithASTData)
  local outMods = {}
  local outSectionsWithLeakingMods = {}
  local outSectionsPartNames = {}
  local outSectionsPartNameToIdx = {}
  local outSectionsAllModNames = {}
  local outPartModifiersScalers = {}

  local sectionsMods = {}

  local testSectionsCounter = nil
  local testSectionsWrongCounter = nil
  local testNodeNameToCID = nil
  if runTest then
    testSectionsCounter = {}
    testSectionsWrongCounter = {}
    testNodeNameToCID = {}

    for i = 0, #vEditor.vdata.nodes - 1 do
      testNodeNameToCID[vEditor.vdata.nodes[i].name or vEditor.vdata.nodes[i].cid] = i
    end
  end

  local modsScalers = {}
  local partsModsScalers = {}

  -- First initialize by getting all modifiers per section
  -- and only save sections that have row modifiers defined
  initAnalyzeModifiersLeaking(partsWithASTData, outSectionsAllModNames, sectionsMods, outMods, outSectionsPartNames, outSectionsPartNameToIdx, modsScalers, partsModsScalers, testSectionsCounter, testSectionsWrongCounter)

  -- For each part go through each section, and for each section go through each row
  -- to get the leaking modifiers
  for _, partData in ipairs(partsWithASTData) do
    local part = partData.data
    local partName = partData.name
    local jbeamFilename = partData.jbeamFilename
    local astHierarchy = partData.astHierarchy

    for sectionName, section in pairs(part) do
      local outModsSection = outMods[sectionName]
      if outModsSection then
        local sectionMods = sectionsMods[sectionName]

        outSectionsPartNames[sectionName] = outSectionsPartNames[sectionName] or {}
        outSectionsPartNameToIdx[sectionName] = outSectionsPartNameToIdx[sectionName] or {}

        tableInsert(outSectionsPartNames[sectionName], partName)
        outSectionsPartNameToIdx[sectionName][partName] = #outSectionsPartNames[sectionName]

        local leakingToPartsEntries = {}

        local header = {}
        local headerSize = 0
        local headerSize1 = 1

        -- Go through each line in part's current section
        for lineNum, lineData in ipairs(section) do
          if lineNum > 1 then
            local astNodeIdxs = {}
            getLineASTNodes(astHierarchy, lineData.__astNodeIdx, astNodeIdxs)
            if tableIsDict(lineData) then
              -- Line is dictionary, so it declares modifier(s)
              for mod, modVal in pairs(lineData) do
                if mod ~= "__astNodeIdx" then
                  local newModVal = deepcopy(modVal)
                  if type(newModVal) == "table" then
                    newModVal["__astNodeIdx"] = nil
                  end
                  -- Apply scalers
                  local isScaled = false
                  if modsScalers[mod] and type(newModVal) == "number" and type(modsScalers[mod].modVal) == "number" then
                    newModVal = newModVal * modsScalers[mod].modVal
                    isScaled = true
                  end
                  sectionMods[mod] = sectionMods[mod] or {modVal = nil, isScaled = false, partPath = nil}
                  sectionMods[mod].modVal = newModVal
                  sectionMods[mod].isScaled = isScaled
                  sectionMods[mod].partPath = partName

                  outModsSection[partName] = outModsSection[partName] or {}
                  outModsSection[partName][mod] = outModsSection[partName][mod] or {modVal = nil, isScaled = false, leakingToParts = {}, leakedFromPart = nil, astNodeData = {}}
                  outModsSection[partName][mod].modVal = newModVal
                  outModsSection[partName][mod].isScaled = isScaled
                  outModsSection[partName][mod].astNodeData.leakSourceASTNodeIdxs = astNodeIdxs
                end
              end

            else
              -- Line is not dictionary, so it declares a JBeam item

              local sectionModsCopy = deepcopy(sectionMods)

              for lineCol = headerSize1, #lineData do
                local colData = lineData[lineCol]
                if type(colData) == 'table' and tableIsDict(colData) and #lineData > headerSize then
                  for mod, modVal in pairs(colData) do
                    if mod ~= "__astNodeIdx" then
                      local newModVal = deepcopy(modVal)
                      if type(newModVal) == "table" then
                        newModVal["__astNodeIdx"] = nil
                      end
                      -- Apply scalers
                      local isScaled = false
                      if modsScalers[mod] and type(newModVal) == "number" and type(modsScalers[mod].modVal) == "number" then
                        newModVal = newModVal * modsScalers[mod].modVal
                        isScaled = true
                      end
                      sectionModsCopy[mod] = sectionModsCopy[mod] or {modVal = nil, partPath = nil}
                      sectionModsCopy[mod].modVal = newModVal
                      sectionModsCopy[mod].isScaled = isScaled
                      sectionModsCopy[mod].partPath = partName
                    end
                  end
                end
              end

              -- If current modifiers part origin is not equal to current part and the modifier is not a default value, modifiers are considered leaking
              for mod, modData in pairs(sectionModsCopy) do
                if modsScalers[mod] then
                  local scaleMod = 'scale'..mod
                  local outModsSectionPart = outModsSection[partName]
                  local partPath = modsScalers[mod].partPath

                  outModsSectionPart[scaleMod] = outModsSectionPart[scaleMod] or {modVal = nil, leakingToParts = {}, leakedFromPart = nil, astNodeData = {}}
                  outModsSectionPart[scaleMod].leakedFromPart = partPath

                  if not leakingToPartsEntries[scaleMod] then
                    outModsSection[partPath] = outModsSection[partPath] or {}
                    outModsSection[partPath][scaleMod] = outModsSection[partPath][scaleMod] or {modVal = nil, leakingToParts = {}, leakedFromPart = nil, astNodeData = {}}
                    tableInsert(outModsSection[partPath][scaleMod].leakingToParts, partName)
                    leakingToPartsEntries[scaleMod] = true
                  end
                end

                if modData.partPath ~= partName and
                (
                  not useDefaultValuesForLeaking[0] or
                  (
                    not modifiersDefaultValues[sectionName] or
                    (
                      (type(modifiersDefaultValues[sectionName][mod]) == 'table' and not modifiersDefaultValues[sectionName][mod][modData.modVal]) or
                      (type(modifiersDefaultValues[sectionName][mod]) ~= 'table' and modData.modVal ~= modifiersDefaultValues[sectionName][mod])
                    )
                  )
                ) then
                  outModsSection[partName] = outModsSection[partName] or {}
                  local outModsSectionPart = outModsSection[partName]

                  if not leakingToPartsEntries[mod] then
                    outModsSection[modData.partPath] = outModsSection[modData.partPath] or {}
                    outModsSection[modData.partPath][mod] = outModsSection[modData.partPath][mod] or {modVal = nil, leakingToParts = {}, leakedFromPart = nil, astNodeData = {}}
                    tableInsert(outModsSection[modData.partPath][mod].leakingToParts, partName)
                    leakingToPartsEntries[mod] = true
                  end

                  outModsSectionPart[mod] = outModsSectionPart[mod] or {modVal = nil, leakingToParts = {}, leakedFromPart = nil, astNodeData = {}}
                  outModsSectionPart[mod].leakedFromPart = modData.partPath

                  outModsSectionPart[mod].astNodeData.affectedRowsASTNodeIdxs = outModsSectionPart[mod].astNodeData.affectedRowsASTNodeIdxs or {}
                  tableInsert(outModsSectionPart[mod].astNodeData.affectedRowsASTNodeIdxs, astNodeIdxs)
                  outSectionsWithLeakingMods[sectionName] = true
                end
              end

              if runTest then
                testModifiers(sectionName, lineData, testSectionsCounter, testNodeNameToCID, sectionModsCopy, testSectionsWrongCounter)
              end
            end
          else
            header = lineData
            headerSize = #header
            headerSize1 = headerSize + 1
          end
        end
      end
    end

    if partsModsScalers[partName] then
      local mods = partsModsScalers[partName]
      for mod, modVal in pairs(mods) do
        for sectionName, outModsSection in pairs(outMods) do
          outModsSection[partName] = outModsSection[partName] or {}
          outModsSection[partName]['scale'..mod] = outModsSection[partName]['scale'..mod] or {modVal = nil, leakingToParts = {}, leakedFromPart = nil, astNodeData = {}}
          outModsSection[partName]['scale'..mod].modVal = modsScalers[mod].modVal
          outSectionsAllModNames[sectionName]['scale'..mod] = true

          outSectionsPartNames[sectionName] = outSectionsPartNames[sectionName] or {}
          outSectionsPartNameToIdx[sectionName] = outSectionsPartNameToIdx[sectionName] or {}

          if outSectionsPartNames[sectionName][#outSectionsPartNames[sectionName]] ~= partName then
            tableInsert(outSectionsPartNames[sectionName], partName)
            outSectionsPartNameToIdx[sectionName][partName] = #outSectionsPartNames[sectionName]
          end
        end
      end
    end
  end

  if runTest then
    for k,v in pairs(testSectionsWrongCounter) do
      print(k .. ": " .. v .. " errors")
    end
    --testSectionsWrongCounter[sectionName] = testSectionsWrongCounter[sectionName] + 1
  end

  return outMods, outSectionsWithLeakingMods, outSectionsPartNames, outSectionsPartNameToIdx, outSectionsAllModNames
end

local function analyze()
  -- Get list of parts in the order that was used for loading the vehicle
  local parts = getParts()

  -- Get parts' data with AST data
  local partsWithASTData = getPartsWithASTData(parts)

  local sectionsPartsMods, outSectionsWithLeakingMods, outSectionsPartNames, outSectionsPartNameToIdx, outSectionsAllModNames = analyzeModifiersLeaking(partsWithASTData)

  -- Generate sorted data for rendering purposes

  sectionNamesSorted = {}
  local sortedTbl = tableKeysSorted(sectionsPartsMods)
  for k,v in pairs(sortedTbl) do
    sectionNamesSorted[k] = v
  end

  sectionsModNamesSorted = {}
  for sectionName, mods in pairs(outSectionsAllModNames) do
    sortedTbl = tableKeysSorted(mods)
    sectionsModNamesSorted[sectionName] = {}

    for k,v in pairs(sortedTbl) do
      sectionsModNamesSorted[sectionName][k] = v
    end
  end

  data = sectionsPartsMods
  sectionsWithLeakingMods = outSectionsWithLeakingMods
  sectionsPartNamesSorted = outSectionsPartNames
  sectionsPartNamesToIdxSorted = outSectionsPartNameToIdx
end

-- Stringifys a table 'tbl' one depth level without newline characters
local function tableToString(tbl)
  local str = ""

  if tableIsDict(tbl) then
    local i = 1
    str = "{"
    for k,v in pairs(tbl) do
      if i > 1 then str = str .. ", " end
      if i > 5 then str = str .. "..." break end

      local vStr = nil
      if type(v) == "string" then vStr = '"' .. v .. '"' else vStr = tostring(v) end
      str = str .. '' .. k .. ' = ' .. vStr
      i = i + 1
    end
    str = str .. "}"
  else
    str = "{"
    for i = 1, #tbl do
      local v = tbl[i]
      if i > 1 then str = str .. ", " end
      if i > 5 then str = str .. "..." break end

      local vStr = nil
      if type(v) == "string" then vStr = '"' .. v .. '"' else vStr = tostring(v) end
      str = str .. vStr
    end
    str = str .. "}"
  end

  return str
end

local tempColVec = vec3(1,1,1)

local function onUpdate()
  if windowOpen[0] ~= true then return end

  if im.Begin(wndName, windowOpen) then
    if not vEditor.vehicle then goto continue end

    if im.Button("Start Analysis") then
      analyze()
    end
    im.SameLine()
    if im.Checkbox("Ignore Default Values (EXPERIMENTAL! Not all default values accounted for)", useDefaultValuesForLeaking) then end

    if not data then goto continue end

    im.Spacing()
    im.Separator()
    im.Spacing()

    local textHeight = im.GetTextLineHeightWithSpacing()

    if im.BeginChild1("##sectionButtons", im.ImVec2(0, textHeight * 2), false, im.WindowFlags_HorizontalScrollbar) then
      local sectionNamesLen = #sectionNamesSorted
      for i = 1, sectionNamesLen do
        local sectionName = sectionNamesSorted[i]
        local isSectionLeaking = sectionsWithLeakingMods[sectionName]

        -- Prevents crash due to not popping color if sectionViewing changed on button click
        local setBtnColFlag = sectionViewing == i

        if isSectionLeaking then im.PushStyleColor2(im.Col_Text, imRedCol) end
        if setBtnColFlag then im.PushStyleColor2(im.Col_Button, im.GetStyleColorVec4(im.Col_ButtonHovered)) end
        if im.Button(sectionName) then
          sectionViewing = i
        end
        if setBtnColFlag then im.PopStyleColor() end
        if isSectionLeaking then im.PopStyleColor() end

        if i ~= sectionNamesLen then
          im.SameLine()
        end
      end
      im.EndChild()
    end

    local sectionName = sectionNamesSorted[sectionViewing]
    if not sectionName then goto continue end

    local sectionSortedModifiers = sectionsModNamesSorted[sectionName]
    local sectionModCount = #sectionSortedModifiers

    if sectionModCount == 0 then goto continue end

    -- imgui only supports up to 64 columns (1st column for parts + 63 columns for modifiers)
    if sectionModCount >= 63 then
      sectionModCount = 63
      im.PushStyleColor2(im.Col_Text, imYellowCol)
      im.Text("Warning! Not all modifiers shown due to ImGui's table maximum column count of 64.")
      im.PopStyleColor()
    end

    local section = data[sectionName]
    local partNamesSorted = sectionsPartNamesSorted[sectionName]

    if im.BeginTable('##visualizationTable', sectionModCount + 1, tableFlags) then
      im.TableSetupColumn('Part', im.TableColumnFlags_NoHide)

      for sectionModIdx = 1, sectionModCount do
        im.TableSetupColumn(sectionSortedModifiers[sectionModIdx])
      end

      im.TableSetupScrollFreeze(1, 1) -- Make header row/column always visible
      im.TableHeadersRow()

      for partIdx, partName in ipairs(partNamesSorted) do
        im.TableNextRow()
        im.TableSetColumnIndex(0)
        if im.Selectable1(partName) then
          vEditor.selectedPart = partName
        end

        local modifiers = section[partName]
        if modifiers then
          for sectionModIdx = 1, sectionModCount do
            local modifierNameFromAll = sectionSortedModifiers[sectionModIdx]
            local modifier = modifiers[modifierNameFromAll]

            if modifier then
              im.TableSetColumnIndex(sectionModIdx)

              if modifier.leakedFromPart then
                local idx = sectionsPartNamesToIdxSorted[sectionName][modifier.leakedFromPart]
                tempColVec.x = math.fmod(idx / 20, 1)
                local r, g, b = HSVtoRGB(tempColVec:xyz())
                im.TableSetBgColor(im.TableBgTarget_CellBg, im.GetColorU322(im.ImVec4(r, g, b, 0.5)), sectionModIdx)
              end

              if modifier.modVal ~= nil then
                local modName = modifierNameFromAll
                local modVal = modifier.modVal
                local modValStr = nil
                local modValType = type(modVal)

                if modValType == "string" then
                  modValStr = '"' .. modVal .. '"'
                elseif modValType == "table" then
                  modValStr = tableToString(modVal)
                else
                  modValStr = tostring(modVal)
                end

                local cellSize = im.ImVec2(im.GetContentRegionAvail().x + im.GetStyle().CellPadding.x * 2, textHeight)

                tempColVec.x = math.fmod(partIdx / 20, 1)
                local r, g, b = HSVtoRGB(tempColVec:xyz())
                local imCol = im.GetColorU322(im.ImVec4(r, g, b, 0.5))

                if im.Selectable1(modValStr .. "##" .. partIdx .. "," .. sectionModIdx) then
                  vEditor.selectedPart = partName
                  tableClear(vEditor.selectedASTNodeMap)
                  if modifier.astNodeData.leakSourceASTNodeIdxs then
                    for k,v in ipairs(modifier.astNodeData.leakSourceASTNodeIdxs) do
                      vEditor.selectedASTNodeMap[v] = true
                    end
                  end
                  vEditor.scrollToNode = true
                end

                local tooltipStr = "Part: " .. partName .. "\nModifier: " .. modName .. " = " .. modValStr
                if #modifier.leakingToParts > 0 then
                  tooltipStr = tooltipStr .. "\nLeaking into parts:"
                  for _,v in ipairs(modifier.leakingToParts) do
                    tooltipStr = tooltipStr .. "\n- " .. v
                  end

                  local cursorPos = im.GetCursorScreenPos()
                  local startPos = im.ImVec2(cursorPos.x - im.GetStyle().CellPadding.x, cursorPos.y - cellSize.y)

                  im.ImDrawList_AddRect(
                    im.GetWindowDrawList(),
                    startPos,
                    im.ImVec2(startPos.x + cellSize.x, startPos.y + cellSize.y),
                    imCol,
                    0,
                    nil,
                    5
                  )
                end
                im.tooltip(tooltipStr)
              else
                if im.Selectable1("##" .. partIdx .. "," .. sectionModIdx) then
                end
              end
              if im.IsItemClicked(1) then
                vEditor.selectedPart = partName
                tableClear(vEditor.selectedASTNodeMap)
                if modifier.astNodeData.affectedRowsASTNodeIdxs then
                  for k,v in ipairs(modifier.astNodeData.affectedRowsASTNodeIdxs) do
                    for k2,v2 in ipairs(v) do
                      vEditor.selectedASTNodeMap[v2] = true
                    end
                  end
                end
                vEditor.scrollToNode = true
              end
            end
          end
        end
      end

      im.EndTable()
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