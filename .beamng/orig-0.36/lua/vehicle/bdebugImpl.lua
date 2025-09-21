-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local max = math.max
local min = math.min
local abs = math.abs

local huge = math.huge

-- these are defined in C, do not change the values
local NORMALTYPE = 0
local BEAM_ANISOTROPIC = 1
local BEAM_BOUNDED = 2
local BEAM_PRESSURED = 3
local BEAM_LBEAM = 4
local BEAM_BROKEN = 5
local BEAM_HYDRO = 6
local BEAM_SUPPORT = 7

local beamTypesNames = {
  [NORMALTYPE] = "NORMALTYPE",
  [BEAM_ANISOTROPIC] = "BEAM_ANISOTROPIC",
  [BEAM_BOUNDED] = "BEAM_BOUNDED",
  [BEAM_PRESSURED] = "BEAM_PRESSURED",
  [BEAM_LBEAM] = "BEAM_LBEAM",
  [BEAM_BROKEN] = "BEAM_BROKEN",
  [BEAM_HYDRO] = "BEAM_HYDRO",
  [BEAM_SUPPORT] = "BEAM_SUPPORT",
}

local beamTypesColors = {
  [NORMALTYPE] = color(0, 223, 0, 255),
  [BEAM_HYDRO] = color(0, 100, 255, 255),
  [BEAM_ANISOTROPIC] = color(255, 135, 63, 255),
  [BEAM_BOUNDED] = color(255, 255, 0, 255),
  [BEAM_LBEAM] = color(92, 92, 92, 255),
  [BEAM_SUPPORT] = color(223, 0, 223, 255),
  [BEAM_PRESSURED] = color(0, 255, 255, 255),
  [BEAM_BROKEN] = color(255, 0, 0, 255),
}

local TRI_NORMAL = 1
local TRI_BACK = 2
local TRI_PRESSURE = 3
local TRI_NONCOLLIDABLE = 4
local TRI_BROKEN = 5

local triTypesNames = {
  'NORMAL',
  'BACK',
  'PRESSURE',
  'NONCOLLIDABLE',
  'BROKEN',
}

local triTypesColors = {
  color(0, 255, 0, 255),
  color(255, 0, 255, 255),
  color(0, 255, 255, 255),
  color(255, 255, 0, 255),
  color(255, 0, 0, 255),
}

local TORBAR_NORMAL = 1
local TORBAR_ANISOTROPIC = 2
local TORBAR_BROKEN = 3

local torbarTypesNames = {
  'NORMAL',
  'ANISOTROPIC',
  'BROKEN',
}

local torbarTypesColors = {
  {color(0,255,0,255), color(0,255,255,255)},
  {color(255,0,255,255), color(0,0,255,255)},
  {color(255,0,0,255), color(255,128,0,255)},
}

local nodeTextMaxDistCap = 10

M.initState = {
  vehicleDebugVisible = false,
  objectId = objectId,
  vehicle = {
    nodeTextMode = 1,
    nodeTextModes = {
      {name = "off"},
      {name = "names"},
      {name = "numbers"},
      {name = "names+numbers"},
      {name = "weights"},
      {name = "materials"},
      {name = "groups"},
      {name = "forces"},
      {name = "relativePositions"},
      {name = "worldPositions"},
    },
    nodeTextMaxDistCap = nodeTextMaxDistCap,
    nodeTextMaxDist = nodeTextMaxDistCap,
    nodeTextShowWheels = false,
    nodeVisMode = 1,
    nodeVisModes = {
      {name = "off"},
      {name = "simple"},
      {name = "highlighted"},
      {name = "weights"},
      {name = "displacement"},
      {name = "velocities"},
      {name = "forces"},
      {name = "density"},
    },
    nodeVisWidthScale = 1,
    nodeVisAlpha = 1,
    nodeDebugTextTypeToID = {},
    nodeDebugTextMode = 1,
    nodeDebugTextModes = {
      {name = "off"},
    },
    beamTextShowWheels = true,
    beamTextMode = 1,
    beamTextModes = {
      {name = "off"},
      {name = "ids"},
      {name = "spawnLength"},
      {name = "liveLength"},
    },
    beamVisMode = 1,
    beamVisModes = {
      {name = "off"},
      {name = "simple"},
      {name = "highlighted"},
      {name = "type"},
      {name = "type+broken"},
      {name = "brokenOnly"},
      {name = "supportOnly"},
      {name = "oldStress"},
      {name = "stress", usesRange = true, rangeMinCap = 0, rangeMaxCap = 100000, rangeMin = 0, rangeMax = 10000, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "displacement", usesRange = true, rangeMinCap = 0.0, rangeMaxCap = 1.0, rangeMin = 0.0, rangeMax = 0.1, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "deformation", usesRange = true, rangeMinCap = 0.0, rangeMaxCap = 1.0, rangeMin = 0.0, rangeMax = 1.0, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "breakgroups"},
      {name = "deformgroups"},
      {name = "boundedBeamBounds"},
      {name = "supportBeamBounds"},
      {name = "frequency", usesSliders = true, sliders = {{name = 'Frequency', val = 100, minVal = 0, maxVal = 1000}, {name = 'Max Amplitude', val = 0.1, minVal = 0, maxVal = 1}}},
      {name = "beamDamp", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamDampFast", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamDampRebound", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamDampReboundFast", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamDampVelocitySplit", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamDeform", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamLimitDamp", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamLimitDampRebound", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamLimitSpring", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamLongBound", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamPrecompression", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamPrecompressionTime", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamShortBound", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamSpring", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "beamStrength", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "boundZone", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "dampCutoffHz", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "dampExpansion", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "deformLimit", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "deformLimitExpansion", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "deformationTriggerRatio", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "longBoundRange", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "precompressionRange", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "shortBoundRange", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "springExpansion", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
    },
    beamVisWidthScale = 1,
    beamVisAlpha = 1,
    torsionBarVisMode = 1,
    torsionBarVisModes = {
      {name = "off"},
      {name = "simple"},
      {name = "type"},
      {name = "withoutBroken"},
      {name = "withBroken"},
      {name = "brokenOnly"},
      {name = "angle", usesRange = true, rangeMinCap = 0, rangeMaxCap = 360, rangeMin = 0, rangeMax = 20, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "stress", usesRange = true, rangeMinCap = 0, rangeMaxCap = 100000, rangeMin = 0, rangeMax = 10000, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "deformation", usesRange = true, rangeMinCap = 0.0, rangeMaxCap = 10.0, rangeMin = 0.0, rangeMax = 1.0, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "damp", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "deform", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "spring", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
      {name = "strength", usesRange = true, autoRange = true, showInfinity = true, rangeMinEnabled = true, rangeMaxEnabled = true, usesInclusiveRange = true},
    },
    torsionBarVisWidthScale = 1,
    torsionBarVisAlpha = 1,
    railsSlideNodesVisMode = 1,
    railsSlideNodesVisModes = {
      {name = "off"},
      {name = "simple"},
      {name = "withoutBroken"},
      {name = "withBroken"},
      {name = "brokenOnly"},
    },
    railsSlideNodesVisWidthScale = 1,
    railsSlideNodesVisAlpha = 1,
    cogMode = 1,
    cogModes = {
      {name = "off"},
      {name = "on"},
      {name = "nowheels"}
    },
    collisionTriangleVisMode = 1,
    collisionTriangleVisModes = {
      {name = "off"},
      {name = "simple"},
      {name = "type"},
      {name = "withoutBroken"},
      {name = "withBroken"},
      {name = "brokenOnly"},
      {name = "collideableOnly"},
    },
    collisionTriangleVisAlpha = 0.5,
    aeroMode = 1,
    aeroModes = {
      {name = "off"},
      {name = "drag+lift"},
      {name = "aoa"},
      {name = "combined"}
    },
    aerodynamicsScale = 0.1,
    tireContactPoint = false,
    steeringGeometry = false,
    steeringGeometryLineLength = 20,
  }
}

M.stateNoReset = {}

M.partsState = {
  partsSelected = {}
}

local nodeDisplayDistance = 0 -- broken atm since it uses the center point of the camera :\
local wheelContacts = {}

local nodesCount = 0
local beamsCount = 0
local trisCount = 0
local torsionBarsCount = 0
local railsCount = 0
local slidenodesCount = 0

local beamsBroken = {}
local beamsDeformed = {}
local deformGroupsTriggerDisplayed = {}

local railsLinksBeams

local requestDrawnNodesCallbacks
local requestDrawnBeamsCallbacks

local viewportSizeX = 0
local viewportSizeY = 0

local cellsToCheck = {
  {0, 0, 0},
  {0, 0, 1},
  {0, 0, -1},
  {0, 1, 0},
  {0, -1, 0},
  {1, 0, 0},
  {-1, 0, 0},
  {1, 1, 0},
  {1, -1, 0},
  {-1, 1, 0},
  {-1, -1, 0},
  {0, 1, 1},
  {0, 1, -1},
  {0, -1, 1},
  {0, -1, -1},
  {1, 0, 1},
  {1, 0, -1},
  {-1, 0, 1},
  {-1, 0, -1}
}
local overlapSize = 0.001
local groupIDCount = 1
local groupIDToEntries, hashToGroupID, jbeamDisplayed, tblPool = {}, {}, {}, {}
local bigOffset = vec3(1e5, 1e5, 1e5)

local tempVec = vec3()
local tempVec2 = vec3()

local function vecRoundNear(v, m)
  v.x = roundNear(v.x, m)
  v.y = roundNear(v.y, m)
  v.z = roundNear(v.z, m)
end

local function getPosHash(v, offX, offY, offZ)
  local x, y, z = roundNear(v.x, overlapSize), roundNear(v.y, overlapSize), roundNear(v.z, overlapSize)
  local hash = bit.bxor(bit.bxor((x + (offX or 0)) * 73856093, (y + (offY or 0)) * 19349663), (z + (offZ or 0)) * 83492791)
  return not isnaninf(hash) and hash or 0
end

local function nodeCollision(p)
  if not M.state.vehicle.tireContactPoint then
    M.nodeCollision = nop
    return
  end
  local wheelId = v.data.nodes[p.id1].wheelID
  if wheelId then
    wheelContacts = wheelContacts or {}
    if not wheelContacts[wheelId] then
      wheelContacts[wheelId] = {totalForce = 0, contactPoint = vec3(0, 0, 0)}
    end
    local wheelC = wheelContacts[wheelId]
    wheelC.totalForce = wheelC.totalForce + p.normalForce
    wheelC.contactPoint = wheelC.contactPoint + vec3(p.pos) * p.normalForce
  end
end

local function beamBroke(id, energy)
  local beam = v.data.beams[id]
  log("I", "bdebug.beamBroken", string.format("beam %d broke: %s [%d]  ->  %s [%d]", id, (v.data.nodes[beam.id1].name or "unnamed"), beam.id1, (v.data.nodes[beam.id2].name or "unnamed"), beam.id2))
  guihooks.message({txt = "vehicle.beamstate.beamBroke", context = {id = id, id1 = beam.id1, id2 = beam.id2, id1name = v.data.nodes[beam.id1].name, id2name = v.data.nodes[beam.id2].name}})

  beamsBroken[id] = true
end

local function printBeamDeformed(id)
  local beam = v.data.beams[id]
  log("I", "bdebug.beamDeformed", string.format("beam %d deformed: %s [%d]  ->  %s [%d]", id, (v.data.nodes[beam.id1].name or "unnamed"), beam.id1, (v.data.nodes[beam.id2].name or "unnamed"), beam.id2))
end

local function printBeamDeformGroupTriggered(deformGroup, beamID)
  local beam = v.data.beams[beamID]
  log("I", "bdebug.beamDeformed", string.format("deformgroup triggered: %s beam %d, %s [%d]  ->  %s [%d]", deformGroup, beamID, (v.data.nodes[beam.id1].name or "unnamed"), beam.id1, (v.data.nodes[beam.id2].name or "unnamed"), beam.id2))
end

local function debugDrawNode(col, node, txt)
  if node.name == nil then
    obj.debugDrawProxy:drawNodeText(node.cid, col, "[" .. tostring(node.cid) .. "] " .. txt, nodeDisplayDistance)
  else
    obj.debugDrawProxy:drawNodeText(node.cid, col, tostring(node.name) .. " " .. txt, nodeDisplayDistance)
  end
end

local function visualizeWheelThermals()
  if M.state.vehicle.wheelThermals then
    local baseTemp = obj:getEnvTemperature() - 10

    for _, wd in pairs(wheels.wheels) do
      local pressureGroupID = v.data.pressureGroups[wd.pressureGroup]

      if pressureGroupID then
        local wheelAvgTemp = obj:getWheelAvgTemperature(wd.wheelID)
        local wheelCoreTemp = obj:getWheelCoreTemperature(wd.wheelID)

        local wheelAirPressure = obj:getGroupPressure(pressureGroupID)
        obj.debugDrawProxy:drawNodeSphere(wd.node1, 0.04, ironbowColor((wheelCoreTemp - baseTemp) * 0.004))
        obj.debugDrawProxy:drawNodeSphere(wd.node2, 0.04, ironbowColor((wheelCoreTemp - baseTemp) * 0.004))
        obj.debugDrawProxy:drawNodeText(wd.node1, ironbowColor((wheelCoreTemp - baseTemp) * 0.004), string.format("%s%.1f %s%.1f %s%.1f", "tT:", wheelAvgTemp - 273.15, "tC:", wheelCoreTemp - 273.15, "psi:", wheelAirPressure*0.000145038-14.5), 0)

        --local wheelAvgTemp = obj:getwheelCoreTemperature(wd.wheelID)

        for _, nid in pairs(wd.treadNodes or {}) do
          obj.debugDrawProxy:drawNodeSphere(nid, 0.02, ironbowColor((obj:getNodeTemperature(nid) - baseTemp) * 0.004))
        end
        for _, nid in pairs(wd.nodes or {}) do
          obj.debugDrawProxy:drawNodeSphere(nid, 0.02, ironbowColor((obj:getNodeTemperature(nid) - baseTemp) * 0.004))
        end
      end
    end
  end
end

local function visualizeTireContactPoint()
  if M.state.vehicle.tireContactPoint and wheelContacts then
    M.nodeCollision = nodeCollision
    for _, c in pairs(wheelContacts) do
      obj.debugDrawProxy:drawSphere(0.02, (c.contactPoint / c.totalForce), color(255, 0, 0, 255))
    end
    table.clear(wheelContacts)
  end
end

local function visualizeSteeringGeometry()
  if M.state.vehicle.steeringGeometry then
    if v.data.wheels then
      local lineLen = M.state.vehicle.steeringGeometryLineLength
      for i = 0, tableSizeC(v.data.wheels) - 1 do
        local w = v.data.wheels[i]
        local node1, node2 = w.node1, w.node2
        if node1 and node2 then
          local node1Pos, node2Pos = obj:getAbsNodePosition(node1), obj:getAbsNodePosition(node2)
          local midPos = (node1Pos + node2Pos) * 0.5
          local dir = (node2Pos - node1Pos):normalized()

          -- drawing same line twice with both depth testing enabled and disabled to show where the line intersects with the ground
          -- TODO: temporary solution to draw from GE Lua side
          obj:queueGameEngineLua(string.format('debugDrawer:drawLine(%s,%s,ColorF(1,0,0,1),false)', -dir * lineLen * 0.5 + midPos, dir * lineLen * 0.5 + midPos))
          obj.debugDrawProxy:drawCylinder(-dir * lineLen * 0.5 + midPos, dir * lineLen * 0.5 + midPos, 0.01, color(255, 0, 0, 255))
        end
      end
    end
  end
end

local function visualizeCollisionTriangles()
  local partsSelected = M.partsState.partsSelected

  local modeID = M.state.vehicle.collisionTriangleVisMode
  local mode = M.state.vehicle.collisionTriangleVisModes[modeID]
  if not mode then return false end

  local modeName = mode.name

  local alpha = M.state.vehicle.collisionTriangleVisAlpha * 255

  -- "off"
  if modeID == 1 then return end

  if playerInfo.firstPlayerSeated then
    obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 60, 0), color(255, 165, 0, 255), "Mode: " .. modeName)
  end

  -- "simple"
  if modeID == 2 then
    local outlineColor = color(0, 0, 0, alpha)

    for i = 0, trisCount - 1 do
      local tri = v.data.triangles[i]

      if partsSelected[tri.partPath or v.config.partsTree.partPath] then
        local frontCol = triTypesColors[TRI_NORMAL]
        local backCol = triTypesColors[TRI_BACK]

        local r,g,b,a = colorGetRGBA(frontCol)
        frontCol = color(r, g, b, alpha)

        local r,g,b,a = colorGetRGBA(backCol)
        backCol = color(r, g, b, alpha)

        -- Front
        obj.debugDrawProxy:drawNodeTriangle(tri.id1, tri.id2, tri.id3, 0, frontCol)
        -- Back
        obj.debugDrawProxy:drawNodeTriangle(tri.id1, tri.id2, tri.id3, -0.001, backCol)

        obj.debugDrawProxy:drawNodeLine(tri.id1, tri.id2, outlineColor)
        obj.debugDrawProxy:drawNodeLine(tri.id2, tri.id3, outlineColor)

        if tri.beamCount == 3 then
          obj.debugDrawProxy:drawNodeLine(tri.id3, tri.id1, outlineColor)
        end
      end
    end

  -- "type"
  elseif modeID == 3 then
    local outlineColor = color(0, 0, 0, alpha)

    for i = 0, trisCount - 1 do
      local tri = v.data.triangles[i]

      if partsSelected[tri.partPath or v.config.partsTree.partPath] then
        local frontCol = triTypesColors[TRI_NORMAL]
        local backCol = triTypesColors[TRI_BACK]
        if tri.pressure then
          frontCol = triTypesColors[TRI_PRESSURE]
        elseif tri.triangleType == 2 then
          frontCol = triTypesColors[TRI_NONCOLLIDABLE]
        end

        local r,g,b,a = colorGetRGBA(frontCol)
        frontCol = color(r, g, b, alpha)

        local r,g,b,a = colorGetRGBA(backCol)
        backCol = color(r, g, b, alpha)

        -- Front
        obj.debugDrawProxy:drawNodeTriangle(tri.id1, tri.id2, tri.id3, 0, frontCol)
        -- Back
        obj.debugDrawProxy:drawNodeTriangle(tri.id1, tri.id2, tri.id3, -0.001, backCol)

        obj.debugDrawProxy:drawNodeLine(tri.id1, tri.id2, outlineColor)
        obj.debugDrawProxy:drawNodeLine(tri.id2, tri.id3, outlineColor)

        if tri.beamCount == 3 then
          obj.debugDrawProxy:drawNodeLine(tri.id3, tri.id1, outlineColor)
        end
      end
    end

    -- Color legend
    if playerInfo.firstPlayerSeated then
      local j = 1
      for i = 1, #triTypesNames do
        if i ~= TRI_BROKEN then
          obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100 + j * 20, 0), triTypesColors[i], triTypesNames[i])
          j = j + 1
        end
      end
    end

  -- "withoutBroken", "withBroken", "brokenOnly"
  elseif modeID == 4 or modeID == 5 or modeID == 6 then
    local outlineColor = color(0, 0, 0, alpha)

    for i = 0, trisCount - 1 do
      local tri = v.data.triangles[i]

      if partsSelected[tri.partPath or v.config.partsTree.partPath] then
        local triBroken = obj:isTriangleBroken(i)

        if (modeID == 4 and not triBroken) or modeID == 5 or (modeID == 6 and triBroken) then
          local frontCol = triTypesColors[TRI_NORMAL]
          local backCol = triTypesColors[TRI_BACK]
          if triBroken then
            frontCol = triTypesColors[TRI_BROKEN]
            backCol = triTypesColors[TRI_BROKEN]
          elseif tri.pressure then
            frontCol = triTypesColors[TRI_PRESSURE]
          elseif tri.triangleType == 2 then
            frontCol = triTypesColors[TRI_NONCOLLIDABLE]
          end

          local r,g,b,a = colorGetRGBA(frontCol)
          frontCol = color(r, g, b, alpha)

          local r,g,b,a = colorGetRGBA(backCol)
          backCol = color(r, g, b, alpha)

          -- Front
          obj.debugDrawProxy:drawNodeTriangle(tri.id1, tri.id2, tri.id3, 0, frontCol)
          -- Back
          obj.debugDrawProxy:drawNodeTriangle(tri.id1, tri.id2, tri.id3, -0.001, backCol)

          obj.debugDrawProxy:drawNodeLine(tri.id1, tri.id2, outlineColor)
          obj.debugDrawProxy:drawNodeLine(tri.id2, tri.id3, outlineColor)

          if tri.beamCount == 3 then
            obj.debugDrawProxy:drawNodeLine(tri.id3, tri.id1, outlineColor)
          end
        end
      end
    end

    -- Color legend
    if playerInfo.firstPlayerSeated then
      for i = 1, #triTypesNames do
        obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100 + i * 20, 0), triTypesColors[i], triTypesNames[i])
      end
    end

  -- "collideableOnly"
  elseif modeID == 7 then
    local outlineColor = color(0, 0, 0, alpha)

    for i = 0, trisCount - 1 do
      local tri = v.data.triangles[i]

      if partsSelected[tri.partPath or v.config.partsTree.partPath] then
        if tri.triangleType ~= 2 and not obj:isTriangleBroken(i) then
          local frontCol = triTypesColors[TRI_NORMAL]
          local backCol = triTypesColors[TRI_BACK]
          if tri.pressure then
            frontCol = triTypesColors[TRI_PRESSURE]
          end

          local r,g,b,a = colorGetRGBA(frontCol)
          frontCol = color(r, g, b, alpha)

          local r,g,b,a = colorGetRGBA(backCol)
          backCol = color(r, g, b, alpha)

          -- Front
          obj.debugDrawProxy:drawNodeTriangle(tri.id1, tri.id2, tri.id3, 0, frontCol)
          -- Back
          obj.debugDrawProxy:drawNodeTriangle(tri.id1, tri.id2, tri.id3, -0.001, backCol)

          obj.debugDrawProxy:drawNodeLine(tri.id1, tri.id2, outlineColor)
          obj.debugDrawProxy:drawNodeLine(tri.id2, tri.id3, outlineColor)

          if tri.beamCount == 3 then
            obj.debugDrawProxy:drawNodeLine(tri.id3, tri.id1, outlineColor)
          end
        end
      end
    end

    -- Color legend
    if playerInfo.firstPlayerSeated then
      local j = 1
      for i = 1, #triTypesNames do
        if i ~= TRI_BROKEN and i ~= TRI_NONCOLLIDABLE then
          obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100 + j * 20, 0), triTypesColors[i], triTypesNames[i])
          j = j + 1
        end
      end
    end
  end
end

local function visualizeAerodynamics()
  local modeID = M.state.vehicle.aeroMode

  -- "off"
  if modeID == 1 then return end

  -- "drag+lift"
  if modeID == 2 then
    obj.debugDrawProxy:drawAerodynamicsCenterOfPressure(color(255, 0, 0, 255), color(55, 55, 255, 255), color(255, 255, 0, 255), color(0, 0, 0, 0), color(0, 0, 0, 0), M.state.vehicle.aerodynamicsScale)

    -- "aoa"
  elseif modeID == 3 then
    obj.debugDrawProxy:drawAerodynamicsCenterOfPressure(color(255, 0, 0, 0), color(55, 55, 255, 0), color(255, 255, 0, 0), color(0, 0, 0, 255), color(0, 0, 0, 0), M.state.vehicle.aerodynamicsScale)

  -- "combined"
  elseif modeID == 4 then
    obj.debugDrawProxy:drawAerodynamicsCenterOfPressure(color(255, 0, 0, 255), color(55, 55, 255, 255), color(255, 255, 0, 255), color(0, 0, 0, 255), color(0, 0, 0, 0), M.state.vehicle.aerodynamicsScale)
  end
end

local function visualizeCOG()
  local modeID = M.state.vehicle.cogMode

  -- "off"
  if not modeID or modeID == 1 then return end

  -- not "off"
  if modeID > 1 then
    local p = obj:calcCenterOfGravity(modeID == 3)
    local vehPos = obj:getPosition()
    local initRefNodePos = v.data.nodes[v.data.refNodes[0].ref].pos
    local rot = quatFromDir(-obj:getDirectionVector(), obj:getDirectionVectorUp())
    local relCOGPos = rot:inversed() * (p - vehPos) + initRefNodePos

    obj.debugDrawProxy:drawAerodynamicsCenterOfPressure(color(0, 0, 0, 0), color(0, 0, 0, 0), color(0, 0, 0, 0), color(0, 0, 0, 0), color(0, 0, 255, 255), 0.1)
    obj.debugDrawProxy:drawSphere(0.1, p, color(255, 0, 0, 255))
    obj.debugDrawProxy:drawText(p + vec3(0, 0, 0.3), color(255, 0, 0, 255), string.format("COG (%0.3f, %0.3f, %0.3f)", relCOGPos.x, relCOGPos.y, relCOGPos.z))

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100, 0), color(0, 0, 0, 255), "COG distance above ground: " .. string.format("%0.3f m", obj:getDistanceFromTerrainPoint(p)))
    end
  end
end

local function visualizeNodesDebugTexts()
  local modeID = M.state.vehicle.nodeDebugTextMode
  if modeID == 1 then return end

  if M.state.vehicle.nodeDebugTextModes[modeID] then
    local nodeColor = color(255,128,0,255)

    for nodeCID, data in pairs(M.state.vehicle.nodeDebugTextModes[modeID].data) do
      local nodePos = obj:getAbsNodePosition(nodeCID)
      for i = #data.textList, 1, -1 do
        local text = data.textList[i]
        obj:queueGameEngineLua('debugDrawer:drawTextAdvanced(' .. tostring(nodePos) .. ',"' .. text .. '",ColorF(1,1,1,1),true,false,ColorI(0,0,0,192))')
      end

      obj.debugDrawProxy:drawNodeSphere(nodeCID, 0.02, nodeColor)
    end
  end
end

local nodePositions = {}
local nodeMinDistFromCam = math.huge
local camPos = vec3()
local textNodeForceAvg = 1
local textNodeWeightAvg = 1

-- Uses tempVec
local function initRenderNodeTexts(partsSelected, showWheels)
  table.clear(jbeamDisplayed)
  table.clear(groupIDToEntries)
  table.clear(hashToGroupID)

  camPos = obj:getCameraPosition()

  groupIDCount = 1
  nodeMinDistFromCam = math.huge

  for i = 0, nodesCount - 1 do
    local node = v.data.nodes[i]
    if partsSelected[node.partPath or v.config.partsTree.partPath] and (showWheels or not node.wheelID) then
      local pos = obj:getNodePosition(i)
      tempVec:set(pos)
      tempVec:setAdd(bigOffset)
      local posHash = getPosHash(tempVec)
      if next(tblPool) == nil then
        table.insert(tblPool, {})
      end
      local exists = false

      nodePositions[i] = obj:getAbsNodePosition(i)

      nodeMinDistFromCam = math.min(nodePositions[i]:distance(camPos), nodeMinDistFromCam)

      -- check adjacent cells for entries
      for k, v in ipairs(cellsToCheck) do
        local hash = getPosHash(tempVec, v[1] * overlapSize, v[2] * overlapSize, v[3] * overlapSize)
        if hashToGroupID[hash] then
          local groupID = hashToGroupID[hash]
          table.insert(groupIDToEntries[groupID], i)
          exists = true
          break
        end
      end
      if not exists then
        groupIDCount = groupIDCount + 1
        hashToGroupID[posHash] = groupIDCount
        groupIDToEntries[groupIDCount] = table.remove(tblPool)
        table.insert(groupIDToEntries[groupIDCount], i)
      end
    end
  end
end

local function getNodeText(node, txt)
  return node.name == nil and "[" .. tostring(node.cid) .. "]" .. (txt and ' ' .. txt or '') or tostring(node.name) .. (txt and ' ' .. txt or '')
end

local function renderNodeText(i, col, txt, nodeTextMaxDist, entries)
  local pos = nodePositions[i]
  local dist = pos:distance(camPos)
  local r,g,b,a = colorGetRGBA(col)
  local distToAlpha = nodeTextMaxDist < nodeTextMaxDistCap and (-1 / nodeTextMaxDist * (dist - nodeMinDistFromCam) + 1) or nodeTextMaxDistCap
  obj.debugDrawProxy:drawText(pos, color(r,g,b, a * distToAlpha), txt)
  --obj.debugDrawProxy:drawNodeText(i, col, txt, nodeDisplayDistance)
  table.clear(entries)
  table.insert(tblPool, entries)
end

local function visualizeNodesTexts()
  local partsSelected = M.partsState.partsSelected

  local modeID = M.state.vehicle.nodeTextMode
  local nodeTextMaxDist = M.state.vehicle.nodeTextMaxDist
  local showWheels = M.state.vehicle.nodeTextShowWheels

  -- "off"
  if modeID == 1 then return end

  -- "names"
  if modeID == 2 then
    local col = color(255, 0, 255, 255)
    initRenderNodeTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local displayAtNode = entries[1]
      for k, i in ipairs(entries) do
        local node = v.data.nodes[i]
        local nodeText = getNodeText(node, nil)
        text = k ~= tblSize and text .. nodeText .. ', ' or text .. nodeText
        jbeamDisplayed[node.cid] = true
      end
      renderNodeText(displayAtNode, col, text, nodeTextMaxDist, entries)
    end

  -- "numbers
  elseif modeID == 3 then
    local col = color(0, 128, 255, 255)
    initRenderNodeTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local displayAtNode = entries[1]
      for k, i in ipairs(entries) do
        local node = v.data.nodes[i]
        local nodeText = tostring(node.cid)
        text = k ~= tblSize and text .. nodeText .. ', ' or text .. nodeText
        jbeamDisplayed[node.cid] = true
      end
      renderNodeText(displayAtNode, col, text, nodeTextMaxDist, entries)
    end

  -- "names+numbers"
  elseif modeID == 4 then
    local col = color(128, 0, 255, 255)
    initRenderNodeTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local displayAtNode = entries[1]
      for k, i in ipairs(entries) do
        local node = v.data.nodes[i]
        local nodeText = getNodeText(node, "" .. node.cid)
        text = k ~= tblSize and text .. nodeText .. ', ' or text .. nodeText
        jbeamDisplayed[node.cid] = true
      end
      renderNodeText(displayAtNode, col, text, nodeTextMaxDist, entries)
    end

  -- "weights"
  elseif modeID == 5 then
    local currNodesCount = 0
    local totalWeight = 0
    local newTotalWeight = 0
    initRenderNodeTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local displayAtNode = entries[1]
      local groupAvgWeight = 0
      for k, i in ipairs(entries) do
        local node = v.data.nodes[i]
        local nodeWeight = obj:getNodeMass(node.cid)
        local nodeText = getNodeText(node, string.format("%.2fkg", nodeWeight))
        text = k ~= tblSize and text .. nodeText .. ', ' or text .. nodeText
        totalWeight = totalWeight + nodeWeight
        groupAvgWeight = groupAvgWeight + nodeWeight
        newTotalWeight = newTotalWeight + nodeWeight
        jbeamDisplayed[node.cid] = true
        currNodesCount = currNodesCount + 1
      end
      renderNodeText(displayAtNode, color(255 * (groupAvgWeight / tblSize / textNodeWeightAvg), 0, 0, 255), text, nodeTextMaxDist, entries)
    end

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 60, 0), color(0, 0, 0, 255), "Weight: " .. string.format("%.2f kg", totalWeight))
    end
    textNodeWeightAvg = newTotalWeight / (currNodesCount + 1e-30)

  -- "materials"
  elseif modeID == 6 then
    -- Averaging colors https://stackoverflow.com/a/29576746
    local materials = particles.getMaterialsParticlesTable()
    initRenderNodeTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local displayAtNode = entries[1]
      local ar, ag, ab, aa = 0,0,0,0
      for k, i in ipairs(entries) do
        local node = v.data.nodes[i]
        local mat = materials[node.nodeMaterial]
        local matname = "unknown"
        local col = color(255, 0, 0, 255) -- unknown material: red
        if mat ~= nil then
          col = color(mat.colorR, mat.colorG, mat.colorB, 255)
          matname = mat.name
        end
        local nodeText = getNodeText(node, matname)
        text = k ~= tblSize and text .. nodeText .. ', ' or text .. nodeText

        local r,g,b,a = colorGetRGBA(col)
        ar = ar + r*r
        ag = ag + g*g
        ab = ab + b*b
        aa = aa + a*a
        jbeamDisplayed[node.cid] = true
      end
      renderNodeText(displayAtNode, color(math.sqrt(ar / tblSize), math.sqrt(ag / tblSize), math.sqrt(ab / tblSize), math.sqrt(aa / tblSize)), text, nodeTextMaxDist, entries)
    end

  -- "groups"
  elseif modeID == 7 then
    local col = color(255, 128, 0, 255)
    initRenderNodeTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local displayAtNode = entries[1]
      for k, i in ipairs(entries) do
        local node = v.data.nodes[i]
        local txt = nil
        if type(node.group) == "table" then
          txt = '{'
          local ngSize = tableSize(node.group)
          local k = 1
          for _,v in pairs(node.group) do
            txt = txt .. v
            if k ~= ngSize then
              txt = txt .. ', '
            end
            k = k + 1
          end
          txt = txt .. '}'
        else
          txt = '{' .. tostring(node.group or '') .. '}'
        end
        local nodeText = getNodeText(node, txt)
        text = k ~= tblSize and text .. nodeText .. ', ' or text .. nodeText
        jbeamDisplayed[node.cid] = true
      end
      renderNodeText(displayAtNode, col, text, nodeTextMaxDist, entries)
    end

  -- "forces"
  elseif modeID == 8 then
    local newAvg = 0
    local invAvgNodeForce = 1 / (textNodeForceAvg * 10 + 300)
    local currNodesCount = 0
    initRenderNodeTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local displayAtNode = entries[1]
      local ar, ag, ab, aa = 0,0,0,0
      for k, i in ipairs(entries) do
        local node = v.data.nodes[i]
        local frc = obj:getNodeForceVector(node.cid)
        local frc_length = frc:length()
        newAvg = newAvg + frc_length

        local nodeText = getNodeText(node, string.format("%0.1f N", frc_length))
        text = k ~= tblSize and text .. nodeText .. ', ' or text .. nodeText

        local c = min(255, (frc_length * invAvgNodeForce) * 255)
        local col = color(c, 0, 0, (c + 100))
        local r,g,b,a = colorGetRGBA(col)
        ar = ar + r*r
        ag = ag + g*g
        ab = ab + b*b
        aa = aa + a*a
        jbeamDisplayed[node.cid] = true
        currNodesCount = currNodesCount + 1
      end
      renderNodeText(displayAtNode, color(math.sqrt(ar / tblSize), math.sqrt(ag / tblSize), math.sqrt(ab / tblSize), math.sqrt(aa / tblSize)), text, nodeTextMaxDist, entries)
    end
    obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 60, 0), color(0, 0, 0, 255), "Average force: " .. string.format("%0.1f N", textNodeForceAvg))
    textNodeForceAvg = (newAvg / (currNodesCount + 1e-30))

  -- "relativePositions"
  elseif modeID == 9 then
    local col = color(0, 255, 0, 255)
    local initRefNodePos = v.data.nodes[v.data.refNodes[0].ref].pos

    initRenderNodeTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local displayAtNode = entries[1]
      for k, i in ipairs(entries) do
        local node = v.data.nodes[i]
        tempVec2:set(obj:getNodePositionRelativeXYZ(node.cid))
        tempVec2:setAdd(initRefNodePos)
        local nodeText = getNodeText(node, string.format("(%0.3f, %0.3f, %0.3f)", tempVec2.x, tempVec2.y, tempVec2.z))
        text = k ~= tblSize and text .. nodeText .. ', ' or text .. nodeText
        jbeamDisplayed[node.cid] = true
      end
      renderNodeText(displayAtNode, col, text, nodeTextMaxDist, entries)
    end

  -- "worldPositions"
  elseif modeID == 10 then
    local col = color(0, 255, 192, 255)
    initRenderNodeTexts(partsSelected, showWheels)
    tempVec:set(obj:getPositionXYZ())
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local displayAtNode = entries[1]
      for k, i in ipairs(entries) do
        local node = v.data.nodes[i]
        tempVec2:setAdd2(tempVec, obj:getNodePosition(node.cid))
        local nodeText = getNodeText(node, string.format("(%0.3f, %0.3f, %0.3f)", tempVec2.x, tempVec2.y, tempVec2.z))
        text = k ~= tblSize and text .. nodeText .. ', ' or text .. nodeText
        jbeamDisplayed[node.cid] = true
      end
      renderNodeText(displayAtNode, col, text, nodeTextMaxDist, entries)
    end
  end
end

local visNodeForceAvg = 1
local nodesDrawn
local function visualizeNodes()
  local dirty = false

  local partsSelected = M.partsState.partsSelected

  local modeID = M.state.vehicle.nodeVisMode
  local mode = M.state.vehicle.nodeVisModes[modeID]
  if not mode then return false end

  local rangeMin = mode.rangeMin or -huge
  local rangeMax = mode.rangeMax or huge

  local minVal = huge
  local maxVal = -huge
  local nodeScale = 0.025 * M.state.vehicle.nodeVisWidthScale
  local alpha = M.state.vehicle.nodeVisAlpha

  nodesDrawn = nodesDrawn or {}
  table.clear(nodesDrawn)
  local ndi = 1

  -- "off"
  if modeID == 1 then return dirty end

  -- highlighted nodes
  for i = 0, nodesCount - 1 do
    local node = v.data.nodes[i]
    if node.highlight then
      local col = parseColor(node.highlight.col or node.highlight.color)
      local r,g,b,a = colorGetRGBA(col)
      col = color(r,g,b,a * alpha)
      local radius = (node.highlight.radius or 0.025) * M.state.vehicle.nodeVisWidthScale
      obj.debugDrawProxy:drawNodeSphere(node.cid, radius, col)
    end
  end

  -- "simple"
  if modeID == 2 then
    for i = 0, nodesCount - 1 do
      local node = v.data.nodes[i]
      if partsSelected[node.partPath or v.config.partsTree.partPath] then
        local c
        if node.fixed then
          c = color(255, 0, 255, 200 * alpha)
        elseif node.selfCollision then
          c = color(255, 255, 0, 200 * alpha)
        elseif node.collision == false then
          c = color(255, 0, 212, 200 * alpha)
        else
          c = color(0, 255, 255, 200 * alpha)
        end
        obj.debugDrawProxy:drawNodeSphere(node.cid, nodeScale, c)

        nodesDrawn[ndi] = node.cid
        ndi = ndi + 1
      end
    end

  -- mode 3 is highlighted nodes

  -- "weights"
  elseif modeID == 4 then
    local totalWeight, _, _ = extensions.vehicleEditor_nodes.calculateNodesWeight()

    local avgNodeScale = 0

    for i = 0, nodesCount - 1 do
      local node = v.data.nodes[i]
      if partsSelected[node.partPath or v.config.partsTree.partPath] then
        local c
        if node.fixed then
          c = color(255, 0, 255, 200 * alpha)
        elseif node.selfCollision then
          c = color(255, 255, 0, 200 * alpha)
        elseif node.collision == false then
          c = color(255, 0, 212, 200 * alpha)
        else
          c = color(0, 255, 255, 200 * alpha)
        end

        local nodeMass = obj:getNodeMass(node.cid)

        local r = (obj:getNodeMass(node.cid) / (totalWeight / nodesCount)) ^ 0.4 * 0.05
        if nodeMass >= rangeMin and nodeMass <= rangeMax then
          local newNodeScale = r * nodeScale * 50
          obj.debugDrawProxy:drawNodeSphere(node.cid, newNodeScale, c)
          avgNodeScale = avgNodeScale + newNodeScale

          nodesDrawn[ndi] = node.cid
          ndi = ndi + 1
        end
      end
    end

    nodeScale = ndi >= 2 and avgNodeScale / (ndi - 1) or nodeScale

  -- "displacement"
  elseif modeID == 5 then
    for i = 0, nodesCount - 1 do
      local node = v.data.nodes[i]
      if partsSelected[node.partPath or v.config.partsTree.partPath] then
        local displacementVec = obj:getNodePositionRelative(node.cid)
        displacementVec:setSub(obj:getOriginalNodePositionRelative(node.cid))
        local displacement = displacementVec:length() * 10

        local a = min(1, displacement) * 255 * alpha
        if a > 5 then
          local r = min(1, displacement) * 255
          obj.debugDrawProxy:drawNodeSphere(node.cid, nodeScale, color(r, 0, 0, a))

          nodesDrawn[ndi] = node.cid
          ndi = ndi + 1
        end
      end
    end

  -- "velocities"
  elseif modeID == 6 then
    local vecVel = obj:getVelocity()
    for i = 0, nodesCount - 1 do
      local node = v.data.nodes[i]
      if partsSelected[node.partPath or v.config.partsTree.partPath] then
        local vel = obj:getNodeVelocityVector(node.cid) - vecVel
        local speed = vel:length()

        if speed >= rangeMin and speed <= rangeMax then
          local c = min(255, speed * 10)
          local col = color(c, 0, 0, (c + 60) * alpha)

          obj.debugDrawProxy:drawNodeSphere(node.cid, nodeScale, col)
          obj.debugDrawProxy:drawNodeVector(node.cid, (vel * 0.3), col)

          nodesDrawn[ndi] = node.cid
          ndi = ndi + 1
        end
      end
    end

  -- "forces"
  elseif modeID == 7 then
    local newAvg = 0
    local invAvgNodeForce = 1 / visNodeForceAvg
    local currNodesCount = 0

    for i = 0, nodesCount - 1 do
      local node = v.data.nodes[i]
      local frc = obj:getNodeForceVector(node.cid)
      local frc_length = frc:length()
      newAvg = newAvg + frc_length
      if partsSelected[node.partPath or v.config.partsTree.partPath] then
        if frc_length >= rangeMin and frc_length <= rangeMax then
          local c = min(255, (frc_length * invAvgNodeForce) * 255)
          local col = color(c, 0, 0, (c + 100) * alpha)
          obj.debugDrawProxy:drawNodeSphere(node.cid, nodeScale, col)
          obj.debugDrawProxy:drawNodeVector3d(nodeScale, node.cid, (frc * invAvgNodeForce), col)

          nodesDrawn[ndi] = node.cid
          ndi = ndi + 1
          currNodesCount = currNodesCount + 1
        end
      end
    end
    visNodeForceAvg = (newAvg / (currNodesCount + 1e-30)) * 10 + 300

  -- "density"
  elseif modeID == 8 then
    local col
    local colorWater = color(255, 0, 0, 200 * alpha)
    local colorAir = color(0, 200, 0, 200 * alpha)
    for i = 0, nodesCount - 1 do
      local node = v.data.nodes[i]
      if partsSelected[node.partPath or v.config.partsTree.partPath] then
        local inWater = obj:inWater(node.cid)
        if inWater then
          col = colorWater
        else
          col = colorAir
        end
        obj.debugDrawProxy:drawNodeSphere(node.cid, nodeScale, col)

        nodesDrawn[ndi] = node.cid
        ndi = ndi + 1
      end
    end
  end

  -- If auto range enabled and at least one beam value exists, use it to calculate range min/max values
  if mode.autoRange and minVal ~= huge and maxVal ~= -huge then
    if not mode.rangeMinCap or (mode.rangeMinCap and minVal < mode.rangeMinCap) then
      mode.rangeMinCap = minVal
      dirty = true
    end

    if not mode.rangeMaxCap or (mode.rangeMaxCap and maxVal > mode.rangeMaxCap) then
      mode.rangeMaxCap = maxVal

      if mode.rangeMinCap == mode.rangeMaxCap then
        local magnitude = math.floor(math.log10(abs(mode.rangeMaxCap)))

        mode.rangeMaxCap = mode.rangeMaxCap + math.pow(10, magnitude - 1)
      end

      dirty = true
    end

    if not mode.rangeMin then
      mode.rangeMin = mode.rangeMinCap
      dirty = true
    end

    if not mode.rangeMax then
      mode.rangeMax = mode.rangeMaxCap
      dirty = true
    end
  end

  if requestDrawnNodesCallbacks and next(requestDrawnNodesCallbacks) ~= nil then
    for _, geFuncName in ipairs(requestDrawnNodesCallbacks) do
      obj:queueGameEngineLua(geFuncName .. "(" .. serialize(nodesDrawn) .. "," .. nodeScale .. ")")
    end
    table.clear(requestDrawnNodesCallbacks)
  end

  return dirty
end

local beamPositions = {}

local function initRenderBeamTexts(partsSelected, showWheels)
  table.clear(jbeamDisplayed)
  table.clear(groupIDToEntries)
  table.clear(hashToGroupID)

  groupIDCount = 1

  local vehPos = obj:getPosition()

  for i = 0, nodesCount - 1 do
    nodePositions[i] = obj:getAbsNodePosition(i)
  end

  for i = 0, beamsCount - 1 do
    local beam = v.data.beams[i]
    if partsSelected[beam.partPath or v.config.partsTree.partPath] and (showWheels or not beam.wheelID) then
      tempVec:setAdd2(nodePositions[beam.id1], nodePositions[beam.id2])
      tempVec:setScaled(0.5)
      if not beamPositions[i] then
        beamPositions[i] = vec3()
      end
      beamPositions[i]:set(tempVec)
      tempVec:setSub(vehPos)
      tempVec:setAdd(bigOffset)
      local posHash = getPosHash(tempVec)
      if next(tblPool) == nil then
        table.insert(tblPool, {})
      end

      local exists = false

      -- check adjacent cells for entries
      for k, v in ipairs(cellsToCheck) do
        local hash = getPosHash(tempVec, v[1] * overlapSize, v[2] * overlapSize, v[3] * overlapSize)
        if hashToGroupID[hash] then
          local groupID = hashToGroupID[hash]
          table.insert(groupIDToEntries[groupID], i)
          exists = true
          break
        end
      end
      if not exists then
        groupIDCount = groupIDCount + 1
        hashToGroupID[posHash] = groupIDCount
        groupIDToEntries[groupIDCount] = table.remove(tblPool)
        table.insert(groupIDToEntries[groupIDCount], i)
      end
    end
  end
end

local function renderBeamText(pos, col, txt, entries)
  obj.debugDrawProxy:drawText(pos, col, txt)
  table.clear(entries)
  table.insert(tblPool, entries)
end

local function visualizeBeamsTexts()
  local partsSelected = M.partsState.partsSelected

  local modeID = M.state.vehicle.beamTextMode
  local showWheels = M.state.vehicle.beamTextShowWheels

  -- "off"
  if modeID == 1 then return end

  -- "ids"
  if modeID == 2 then
    local col = jetColor(0)
    initRenderBeamTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local pos = beamPositions[entries[1]]
      for k, i in ipairs(entries) do
        local beam = v.data.beams[i]
        local beamText = beam.cid
        text = k ~= tblSize and text .. beamText .. ', ' or text .. beamText
        jbeamDisplayed[beam.cid] = true
      end
      renderBeamText(pos, col, text, entries)
    end

  -- "spawnLength"
  elseif modeID == 3 then
    local col = jetColor(0.1)
    initRenderBeamTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local pos = beamPositions[entries[1]]
      for k, i in ipairs(entries) do
        local beam = v.data.beams[i]
        local beamText = string.format("%d: %.3f m", beam.cid, obj:getBeamRefLength(beam.cid))
        text = k ~= tblSize and text .. beamText .. ', ' or text .. beamText
        jbeamDisplayed[beam.cid] = true
      end
      renderBeamText(pos, col, text, entries)
    end

  -- "liveLength"
  elseif modeID == 4 then
    local col = jetColor(0.2)
    initRenderBeamTexts(partsSelected, showWheels)
    for groupID, entries in pairs(groupIDToEntries) do
      local text = ''
      local tblSize = #entries
      local pos = beamPositions[entries[1]]
      for k, i in ipairs(entries) do
        local beam = v.data.beams[i]
        local beamText = string.format("%d: %.3f m", beam.cid, obj:getBeamLength(beam.cid))
        text = k ~= tblSize and text .. beamText .. ', ' or text .. beamText
        jbeamDisplayed[beam.cid] = true
      end
      renderBeamText(pos, col, text, entries)
    end
  end
end

local groupsData = {}
local beamsDrawn
local beamFreqModeAmp = {}

local function visualizeBeams()
  local dirty = false

  local partsSelected = M.partsState.partsSelected

  local modeID = M.state.vehicle.beamVisMode
  local mode = M.state.vehicle.beamVisModes[modeID]
  if not mode then return false end

  local modeName = mode.name

  local rangeMin = mode.rangeMin or -huge
  local rangeMax = mode.rangeMax or huge

  local minVal = huge
  local maxVal = -huge

  local beamScale = 0.002 * M.state.vehicle.beamVisWidthScale
  local alpha = M.state.vehicle.beamVisAlpha

  beamsDrawn = beamsDrawn or {}
  table.clear(beamsDrawn)
  local bdi = 1

  -- "off"
  if modeID == 1 then return dirty end

  -- highlighted beams
  for i = 0, beamsCount - 1 do
    local beam = v.data.beams[i]
    if beam.highlight then
      local col = parseColor(beam.highlight.col or beam.highlight.color)
      local r,g,b,a = colorGetRGBA(col)
      col = color(r,g,b,a * alpha)

      local len = beam.highlight.len or beam.highlight.length
      local radius = (beam.highlight.radius or 0.01) * M.state.vehicle.beamVisWidthScale

      if len then
        local node1, node2 = beam.id1, beam.id2
        local node1Pos, node2Pos = obj:getAbsNodePosition(node1), obj:getAbsNodePosition(node2)
        local midPos = (node1Pos + node2Pos) * 0.5
        local dir = (node2Pos - node1Pos):normalized()
        -- drawing same line twice with both depth testing enabled and disabled to show where the line intersects with the ground
        -- TODO: temporary solution to draw from GE Lua side
        --obj:queueGameEngineLua(string.format('debugDrawer:drawCylinder(%s,%s,%f,ColorF(%f,%f,%f,%f),false)', -dir * len * 0.5 + midPos, dir * len * 0.5 + midPos, radius * 0.5, r/255,g/255,b/255,a/255))
        obj.debugDrawProxy:drawCylinder(-dir * len * 0.5 + midPos, dir * len * 0.5 + midPos, radius, col)
      else
        -- drawing same line twice with both depth testing enabled and disabled to show where the line intersects with the ground
        -- TODO: temporary solution to draw from GE Lua side
        obj.debugDrawProxy:drawBeam3d(beam.cid, radius, col)
        -- obj:queueGameEngineLua(string.format('debugDrawer:drawCylinder(%s,%s,%f,ColorF(%f,%f,%f,%f),false)', node1Pos, node2Pos, radius, r/255,g/255,b/255,a/255))
        -- obj.debugDrawProxy:drawCylinder(node1Pos, node2Pos, radius, col)
      end
    end
  end
  if playerInfo.firstPlayerSeated then
    obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 60, 0), color(255, 165, 0, 255), "Mode: " .. modeName)
  end

  -- "simple"
  if modeID == 2 then
    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(0, 223, 0, 255 * alpha))

        beamsDrawn[bdi] = beam.cid
        bdi = bdi + 1
      end
    end

  -- mode 3 is highlighted beams

  -- "type" | "with broken" | "brokenOnly"
  elseif modeID == 4 or modeID == 5 or modeID == 6 then
    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        local beamType = beam.beamType or 0

        local col = beamTypesColors[beamType]

        local beamBroken = obj:beamIsBroken(beam.cid)

        if (modeID == 5 or modeID == 6) and beamBroken then
          col = beamTypesColors[BEAM_BROKEN]
        end

        if (modeID == 4 and not beamBroken) or modeID == 5 or (modeID == 6 and beamBroken) then
          local r,g,b,a = colorGetRGBA(col)
          obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(r, g, b, a * alpha))

          beamsDrawn[bdi] = beam.cid
          bdi = bdi + 1
        end
      end
    end

    -- Color legend
    if playerInfo.firstPlayerSeated and modeID ~= 6 then
      for i = 0, #beamTypesNames do
        obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100 + i * 20, 0), beamTypesColors[i], beamTypesNames[i])
      end
    end

  -- "supportOnly"
  elseif modeID == 7 then
    local col = beamTypesColors[BEAM_SUPPORT]
    local r,g,b,a = colorGetRGBA(col)
    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        if beam.beamType == BEAM_SUPPORT and not obj:beamIsBroken(beam.cid) then
          obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(r, g, b, a * alpha))

          beamsDrawn[bdi] = beam.cid
          bdi = bdi + 1
        end
      end
    end

  -- "stress (old)"
  elseif modeID == 8 then
    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        local stress = obj:getBeamStress(beam.cid) * 0.0002
        local a = min(1, abs(stress)) * 255 * alpha
        if a > 5 then
          local r = max(-1, min(0, stress)) * 255 * -1
          local b = max(0, min(1, stress)) * 255
          obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(r, 0, b, a))

          beamsDrawn[bdi] = beam.cid
          bdi = bdi + 1
        end
      end
    end

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100, 0), color(255, 0, 0, 255), "Compression")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 120, 0), color(0, 0, 255, 255), "Extension")
    end

  -- "stress (new)"
  elseif modeID == 9 then
    local scaler = 1 / (rangeMax - rangeMin)

    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        local stress = obj:getBeamStressDamp(beam.cid)
        local absStress = abs(stress)

        if mode.rangeMinEnabled and mode.rangeMaxEnabled then
          if mode.usesInclusiveRange and absStress >= rangeMin and absStress <= rangeMax
          or not mode.usesInclusiveRange and absStress > rangeMin and absStress < rangeMax then
            local a = (absStress - rangeMin) * scaler * 255
            if a > 5 then
              local r = max(-1, min(0, (stress + rangeMin) * scaler)) * 255 * -1 -- (red compression)
              local b = max(0, min(1, (stress - rangeMin) * scaler)) * 255 -- (blue extension)
              a = a * alpha
              obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(r, 0, b, a))

              beamsDrawn[bdi] = beam.cid
              bdi = bdi + 1
            end
          end
        elseif not mode.rangeMinEnabled and not mode.rangeMaxEnabled
        or mode.rangeMinEnabled and (mode.usesInclusiveRange and absStress >= rangeMin or not mode.usesInclusiveRange and absStress > rangeMin)
        or mode.rangeMaxEnabled and (mode.usesInclusiveRange and absStress <= rangeMax or not mode.usesInclusiveRange and absStress < rangeMax) then
          local r = stress < 0 and 255 or 0
          local b = stress >= 0 and 255 or 0
          obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(r, 0, b, alpha * 255))

          beamsDrawn[bdi] = beam.cid
          bdi = bdi + 1
        end
      end
    end

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100, 0), color(255, 0, 0, 255), "Compression")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 120, 0), color(0, 0, 255, 255), "Extension")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 140, 0), color(255, 255, 255, 255), string.format("Range Min: %.2f", rangeMin))
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 160, 0), color(255, 255, 255, 255), string.format("Range Max: %.2f", rangeMax))
    end

  -- "displacement"
  elseif modeID == 10 then
    local scaler = 1 / (rangeMax - rangeMin)

    local nodePosCache = {}

    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        tempVec:setSub2(v.data.nodes[beam.id2].pos, v.data.nodes[beam.id1].pos)
        local originalLength = tempVec:length()

        local nodePos1 = nodePosCache[beam.id1] or obj:getNodePosition(beam.id1)
        local nodePos2 = nodePosCache[beam.id2] or obj:getNodePosition(beam.id2)
        nodePosCache[beam.id1] = nodePos1
        nodePosCache[beam.id2] = nodePos2

        tempVec:setSub2(nodePos2, nodePos1)
        local currentLength = tempVec:length()
        local displacement = currentLength - originalLength
        local absDisplacement = abs(displacement)

        if mode.rangeMinEnabled and mode.rangeMaxEnabled then
          if mode.usesInclusiveRange and absDisplacement >= rangeMin and absDisplacement <= rangeMax
          or not mode.usesInclusiveRange and absDisplacement > rangeMin and absDisplacement < rangeMax then
            local a = (absDisplacement - rangeMin) * scaler * 255
            if a > 5 then
              local r = max(-1, min(0, (displacement + rangeMin) * scaler)) * 255 * -1 -- (red compression)
              local b = max(0, min(1, (displacement - rangeMin) * scaler)) * 255 -- (blue extension)
              a = a * alpha
              obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(r, 0, b, a))

              beamsDrawn[bdi] = beam.cid
              bdi = bdi + 1
            end
          end
        elseif not mode.rangeMinEnabled and not mode.rangeMaxEnabled
        or mode.rangeMinEnabled and (mode.usesInclusiveRange and absDisplacement >= rangeMin or not mode.usesInclusiveRange and absDisplacement > rangeMin)
        or mode.rangeMaxEnabled and (mode.usesInclusiveRange and absDisplacement <= rangeMax or not mode.usesInclusiveRange and absDisplacement < rangeMax) then
          local r = displacement < 0 and 255 or 0
          local b = displacement >= 0 and 255 or 0
          obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(r, 0, b, alpha * 255))

          beamsDrawn[bdi] = beam.cid
          bdi = bdi + 1
        end
      end
    end

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100, 0), color(255, 0, 0, 255), "Compression")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 120, 0), color(0, 0, 255, 255), "Extension")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 140, 0), color(255, 255, 255, 255),  string.format("Range Min: %.2f", rangeMin))
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 160, 0), color(255, 255, 255, 255),  string.format("Range Max: %.2f", rangeMax))
    end

  -- "deformation"
  elseif modeID == 11 then
    local deformRange = rangeMax - rangeMin
    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      local deform = obj:getBeamDebugDeformation(beam.cid) - 1
      local deformGroup = beam.deformGroup

      if not beamsDeformed[i] and deform ~= 0 then
        printBeamDeformed(i)
        beamsDeformed[i] = true
      end
      if deformGroup and beamstate.deformGroupsTriggerBeam[deformGroup] and not deformGroupsTriggerDisplayed[deformGroup] then
        printBeamDeformGroupTriggered(deformGroup, beamstate.deformGroupsTriggerBeam[deformGroup])
        deformGroupsTriggerDisplayed[deformGroup] = true
      end

      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        if not obj:beamIsBroken(beam.cid) then
          local absDeform = abs(deform)

          if mode.rangeMinEnabled and mode.rangeMaxEnabled then
            if mode.usesInclusiveRange and absDeform >= rangeMin and absDeform <= rangeMax
            or not mode.usesInclusiveRange and absDeform > rangeMin and absDeform < rangeMax then
              local r = max(min((-deform - rangeMin) / deformRange, 1), 0) * 255
                --red for compression
              local b = max(min((deform - rangeMin) / deformRange, 1), 0) * 255
                --blue for elongation
              local a = min((absDeform - rangeMin) / deformRange, 1) * 255 * alpha
              obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(r, 0, b, a))

              beamsDrawn[bdi] = beam.cid
              bdi = bdi + 1
            end
          elseif not mode.rangeMinEnabled and not mode.rangeMaxEnabled
          or mode.rangeMinEnabled and (mode.usesInclusiveRange and absDeform >= rangeMin or not mode.usesInclusiveRange and absDeform > rangeMin)
          or mode.rangeMaxEnabled and (mode.usesInclusiveRange and absDeform <= rangeMax or not mode.usesInclusiveRange and absDeform < rangeMax) then
            local r = deform < 0 and 255 or 0
            local b = deform >= 0 and 255 or 0
            obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(r, 0, b, alpha * 255))

            beamsDrawn[bdi] = beam.cid
            bdi = bdi + 1
          end
        end
      end
    end

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100, 0), color(255, 0, 0, 255), "Compression")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 120, 0), color(0, 0, 255, 255), "Extension")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 140, 0), color(255, 255, 255, 255),  string.format("Range Min: %.2f", rangeMin))
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 160, 0), color(255, 255, 255, 255),  string.format("Range Max: %.2f", rangeMax))
    end

  -- "breakgroups"
  elseif modeID == 12 then
    tempVec:set(0,0,0)
    local j = 0
    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        if beam.breakGroup and beam.breakGroup ~= "" then
          local breakGroups = type(beam.breakGroup) == "table" and beam.breakGroup or {beam.breakGroup}
          for _, g in pairs(breakGroups) do
            if not groupsData[g] then
              groupsData[g] = {0, vec3(), getContrastColor(j, 255 * alpha)}
              j = j + 1
            end
            local groupData = groupsData[g]
            local pos1, pos2 = obj:getAbsNodePosition(beam.id1), obj:getAbsNodePosition(beam.id2)
            tempVec:setAdd2(pos1, pos2)
            tempVec:setScaled(0.5)
            groupData[1] = groupData[1] + 1
            groupData[2]:setAdd(tempVec)
            obj.debugDrawProxy:drawCylinder(pos1, pos2, beamScale, groupData[3])
            beamsDrawn[bdi] = beam.cid
            bdi = bdi + 1
          end
        end
      end
    end
    for g, groupData in pairs(groupsData) do
      local groupPos = groupData[2]
      groupPos:setScaled(1 / groupData[1])
      obj.debugDrawProxy:drawText(groupPos, groupData[3], g)
    end
    table.clear(groupsData)

  -- "deformgroups"
  elseif modeID == 13 then
    tempVec:set(0,0,0)
    local j = 0
    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      local deform = obj:getBeamDebugDeformation(beam.cid) - 1
      local deformGroup = beam.deformGroup

      if not beamsDeformed[i] and deform ~= 0 then
        printBeamDeformed(i)
        beamsDeformed[i] = true
      end
      if deformGroup and beamstate.deformGroupsTriggerBeam[deformGroup] and not deformGroupsTriggerDisplayed[deformGroup] then
        printBeamDeformGroupTriggered(deformGroup, beamstate.deformGroupsTriggerBeam[deformGroup])
        deformGroupsTriggerDisplayed[deformGroup] = true
      end

      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        if beam.deformGroup and beam.deformGroup ~= "" then
          local deformGroups = type(beam.deformGroup) == "table" and beam.deformGroup or {beam.deformGroup}
          for _, g in pairs(deformGroups) do
            if not groupsData[g] then
              groupsData[g] = {0, vec3(), getContrastColor(j, 255 * alpha)}
              j = j + 1
            end
            local groupData = groupsData[g]
            local pos1, pos2 = obj:getAbsNodePosition(beam.id1), obj:getAbsNodePosition(beam.id2)
            tempVec:setAdd2(pos1, pos2)
            tempVec:setScaled(0.5)
            groupData[1] = groupData[1] + 1
            groupData[2]:setAdd(tempVec)
            obj.debugDrawProxy:drawCylinder(pos1, pos2, beamScale, groupData[3])
            beamsDrawn[bdi] = beam.cid
            bdi = bdi + 1
          end
        end
      end
    end
    for g, groupData in pairs(groupsData) do
      local groupPos = groupData[2]
      groupPos:setScaled(1 / groupData[1])
      obj.debugDrawProxy:drawText(groupPos, groupData[3], g)
    end
    table.clear(groupsData)

  -- "boundedBeamBounds"
  elseif modeID == 14 then
    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if beam.beamType == BEAM_BOUNDED then
        if partsSelected[beam.partPath or v.config.partsTree.partPath] then
          local newBeamScale = beamScale * (0.5 + bdi * 0.01)

          local currLen = obj:getBeamLength(beam.cid)
          local restLen = obj:getBeamRefLength(beam.cid)
          local restLenHalf = restLen * 0.5
          local node1Pos = obj:getAbsNodePosition(beam.id1)
          local node2Pos = obj:getAbsNodePosition(beam.id2)
          local middlePos = (node1Pos + node2Pos) * 0.5
          local node1to2Dir = (node2Pos - node1Pos):normalized()
          local restPos1 = -node1to2Dir * restLenHalf + middlePos
          local restPos2 = node1to2Dir * restLenHalf + middlePos

          local boundZone = type(beam.boundZone) == 'number' and beam.boundZone or 1

          local shortBoundTransEnd = type(beam.shortBoundRange) == 'number' and max(restLen - max(0, beam.shortBoundRange) - boundZone, 0) or max((restLen * max(1 - max(0, beam.beamShortBound or 1)) - boundZone), 0)
          local shortBoundTransStart = type(beam.shortBoundRange) == 'number' and max(restLen - max(0, beam.shortBoundRange), 0) or restLen * max((1 - max(0, beam.beamShortBound or 1)), 0)

          local longBoundTransStart = type(beam.longBoundRange) == 'number' and restLen + max(0, beam.longBoundRange) or restLen * (1 + max(0, beam.beamLongBound or 1))
          local longBoundTransEnd = type(beam.longBoundRange) == 'number' and max(restLen + max(0, beam.longBoundRange) + boundZone, 0) or max((restLen * (1 + max(0, beam.beamLongBound or 1)) + boundZone), 0)

          if currLen < shortBoundTransEnd then
            local shortBoundTransEndPos1 = -node1to2Dir * shortBoundTransEnd * 0.5 + middlePos
            local shortBoundTransStartPos1 = -node1to2Dir * shortBoundTransStart * 0.5 + middlePos

            local shortBoundTransEndPos2 = node1to2Dir * shortBoundTransEnd * 0.5 + middlePos
            local shortBoundTransStartPos2 = node1to2Dir * shortBoundTransStart * 0.5 + middlePos

            obj.debugDrawProxy:drawCylinder(restPos1, shortBoundTransStartPos1, newBeamScale, color(0,0,255,255 * alpha))
            obj.debugDrawProxy:drawCylinder(shortBoundTransStartPos1, shortBoundTransEndPos1, newBeamScale, color(0,128,255,255 * alpha))
            obj.debugDrawProxy:drawCylinder(shortBoundTransEndPos1, node1Pos, newBeamScale, color(0,255,255,255 * alpha))

            obj.debugDrawProxy:drawCylinder(node1Pos, node2Pos, newBeamScale, color(0,255,0,255 * alpha))

            obj.debugDrawProxy:drawCylinder(node2Pos, shortBoundTransEndPos2, newBeamScale, color(0,255,255,255 * alpha))
            obj.debugDrawProxy:drawCylinder(shortBoundTransEndPos2, shortBoundTransStartPos2, newBeamScale, color(0,128,255,255 * alpha))
            obj.debugDrawProxy:drawCylinder(shortBoundTransStartPos2, restPos2, newBeamScale, color(0,0,255,255 * alpha))

          elseif currLen < shortBoundTransStart then
            local shortBoundTransStartPos1 = -node1to2Dir * shortBoundTransStart * 0.5 + middlePos

            local shortBoundTransStartPos2 = node1to2Dir * shortBoundTransStart * 0.5 + middlePos

            obj.debugDrawProxy:drawCylinder(restPos1, shortBoundTransStartPos1, newBeamScale, color(0,0,255,255 * alpha))
            obj.debugDrawProxy:drawCylinder(shortBoundTransStartPos1, node1Pos, newBeamScale, color(0,128,255,255 * alpha))

            obj.debugDrawProxy:drawCylinder(node1Pos, node2Pos, newBeamScale, color(0,255,0,255 * alpha))

            obj.debugDrawProxy:drawCylinder(node2Pos, shortBoundTransStartPos2, newBeamScale, color(0,128,255,255 * alpha))
            obj.debugDrawProxy:drawCylinder(shortBoundTransStartPos2, restPos2, newBeamScale, color(0,0,255,255 * alpha))

          elseif currLen < restLen then
            obj.debugDrawProxy:drawCylinder(restPos1, node1Pos, newBeamScale, color(0,0,255,255 * alpha))

            obj.debugDrawProxy:drawCylinder(node1Pos, node2Pos, newBeamScale, color(0,255,0,255 * alpha))

            obj.debugDrawProxy:drawCylinder(node2Pos, restPos2, newBeamScale, color(0,0,255,255 * alpha))

          elseif currLen < longBoundTransStart then
            obj.debugDrawProxy:drawCylinder(node1Pos, restPos1, newBeamScale, color(255,0,0,255 * alpha))

            obj.debugDrawProxy:drawCylinder(restPos1, restPos2, newBeamScale, color(0,255,0,255 * alpha))

            obj.debugDrawProxy:drawCylinder(restPos2, node2Pos, newBeamScale, color(255,0,0,255 * alpha))

          elseif currLen < longBoundTransEnd then
            local longBoundTransStartPos1 = -node1to2Dir * longBoundTransStart * 0.5 + middlePos

            local longBoundTransStartPos2 = node1to2Dir * longBoundTransStart * 0.5 + middlePos

            obj.debugDrawProxy:drawCylinder(node1Pos, longBoundTransStartPos1, newBeamScale, color(255,128,0,255 * alpha))
            obj.debugDrawProxy:drawCylinder(longBoundTransStartPos1, restPos1, newBeamScale, color(255,0,0,255 * alpha))

            obj.debugDrawProxy:drawCylinder(restPos1, restPos2, newBeamScale, color(0,255,0,255 * alpha))

            obj.debugDrawProxy:drawCylinder(restPos2, longBoundTransStartPos2, newBeamScale, color(255,0,0,255 * alpha))
            obj.debugDrawProxy:drawCylinder(longBoundTransStartPos2, node2Pos, newBeamScale, color(255,128,0,255 * alpha))
          else
            local longBoundTransEndPos1 = -node1to2Dir * longBoundTransEnd * 0.5 + middlePos
            local longBoundTransStartPos1 = -node1to2Dir * longBoundTransStart * 0.5 + middlePos

            local longBoundTransEndPos2 = node1to2Dir * longBoundTransEnd * 0.5 + middlePos
            local longBoundTransStartPos2 = node1to2Dir * longBoundTransStart * 0.5 + middlePos

            obj.debugDrawProxy:drawCylinder(node1Pos, longBoundTransEndPos1, newBeamScale, color(255,255,0,255 * alpha))
            obj.debugDrawProxy:drawCylinder(longBoundTransEndPos1, longBoundTransStartPos1, newBeamScale, color(255,128,0,255 * alpha))
            obj.debugDrawProxy:drawCylinder(longBoundTransStartPos1, restPos1, newBeamScale, color(255,0,0,255 * alpha))

            obj.debugDrawProxy:drawCylinder(restPos1, restPos2, newBeamScale, color(0,255,0,255 * alpha))

            obj.debugDrawProxy:drawCylinder(restPos2, longBoundTransStartPos2, newBeamScale, color(255,0,0,255 * alpha))
            obj.debugDrawProxy:drawCylinder(longBoundTransStartPos2, longBoundTransEndPos2, newBeamScale, color(255,128,0,255 * alpha))
            obj.debugDrawProxy:drawCylinder(longBoundTransEndPos2, node2Pos, newBeamScale, color(255,255,0,255 * alpha))
          end

          beamsDrawn[bdi] = beam.cid
          bdi = bdi + 1
        end
      end
    end

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100, 0), color(0,255,255,255), "Short Bound")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 120, 0), color(0,128,255,255), "Short Bound Transition")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 140, 0), color(0,0,255,255), "Contraction")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 160, 0), color(0,255,0,255), "Beam")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 180, 0), color(255,0,0,255), "Expansion")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 200, 0), color(255,128,0,255), "Long Bound Transition")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 220, 0), color(255,255,0,255), "Long Bound")
    end

  -- "supportBeamBounds"
  elseif modeID == 15 then
    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if beam.beamType == BEAM_SUPPORT then
        if partsSelected[beam.partPath or v.config.partsTree.partPath] then
          local currLen = obj:getBeamLength(beam.cid)
          local restLen = obj:getBeamRefLength(beam.cid)
          local restLenHalf = restLen * 0.5
          local node1Pos = obj:getAbsNodePosition(beam.id1)
          local node2Pos = obj:getAbsNodePosition(beam.id2)
          local middlePos = (node1Pos + node2Pos) * 0.5
          local node1to2Dir = (node2Pos - node1Pos):normalized()
          local dirPerp = node1to2Dir:perpendicularN()
          local restLenCol = currLen >= restLen and color(255, 0, 0, alpha * 255 * 0.33) or color(0, 0, 255, alpha * 255 * 0.33)

          local longBoundHalfLen = type(beam.beamLongExtent) == 'number' and (restLen + max(0, beam.beamLongExtent)) * 0.5 or restLen * (1 + max(0, beam.beamLongBound or 1)) * 0.5

          --cylinder representing long bound
          obj.debugDrawProxy:drawCylinder(-node1to2Dir * longBoundHalfLen + middlePos + dirPerp * 0.005, node1to2Dir * longBoundHalfLen + middlePos + dirPerp * 0.005, beamScale * 1, color(255, 255, 0, alpha * 255 * 0.5))

          -- beam representing rest length
          obj.debugDrawProxy:drawCylinder(-node1to2Dir * restLenHalf + middlePos, node1to2Dir * restLenHalf + middlePos, beamScale, restLenCol)

          -- beam representing full length
          obj.debugDrawProxy:drawCylinder(node1Pos, node2Pos, beamScale * 0.5, color(0, 255, 0, alpha * 255))

          beamsDrawn[bdi] = beam.cid
          bdi = bdi + 1
        end
      end
    end

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100, 0), color(0, 0, 255, 255), "Contraction")
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 120, 0), color(255, 0, 0, 255), "Expansion")
    end

  -- frequency
  elseif modeID == 15 then
    local freq, ampMax = mode.sliders[1].val, mode.sliders[2].val
    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        local amplitude = obj:getBeamFrequencyAmplitude(i, freq, 10) --obj:detectBeamFrequency(i)
        beamFreqModeAmp[i] = amplitude --0.5 * beam.beamSpring * amplitude^2
      end
    end
    local ampScaler = 1 / ampMax
    for beamID, energy in pairs(beamFreqModeAmp) do
      local a = min(255, energy * ampScaler * 255 * alpha)
      obj.debugDrawProxy:drawBeam3d(beamID, beamScale, color(255, 0, 0, a))
      beamsDrawn[bdi] = beamID
      bdi = bdi + 1
    end

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100, 0), color(0, 0, 0, 255), string.format("%.2f Hz", freq))
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 120, 0), color(0, 0, 0, 255), string.format("Max Amplitude: %.2f m", ampMax))
    end

    table.clear(beamFreqModeAmp)

  -- the rest
  elseif modeID >= 16 then
    -- Do rendering and get min/max values for next frame rendering
    local scaler = 1 / (rangeMax - rangeMin)

    for i = 0, beamsCount - 1 do
      local beam = v.data.beams[i]
      if partsSelected[beam.partPath or v.config.partsTree.partPath] then
        local val = tonumber(beam[modeName])
        if val then
          minVal = val ~= -huge and min(val, minVal) or minVal
          maxVal = val ~= huge and max(val, maxVal) or maxVal

          if abs(val) == huge then
            if mode.showInfinity then
              local a = alpha * 255
              obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(255, 0, 255, a))

              beamsDrawn[bdi] = beam.cid
              bdi = bdi + 1
            end
          else
            if mode.rangeMinEnabled and mode.rangeMaxEnabled then
              if mode.usesInclusiveRange and val >= rangeMin and val <= rangeMax
              or not mode.usesInclusiveRange and val > rangeMin and val < rangeMax then
                local relValue = scaler * (val - rangeMin)

                local r = (relValue + (1 - relValue)) * 255
                local g = (1 - relValue) * 255
                local b = (1 - relValue) * 255
                local a = alpha * 255

                obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(r, g, b, a))

                beamsDrawn[bdi] = beam.cid
                bdi = bdi + 1
              end
            elseif not mode.rangeMinEnabled and not mode.rangeMaxEnabled
            or mode.rangeMinEnabled and (mode.usesInclusiveRange and val >= rangeMin or not mode.usesInclusiveRange and val > rangeMin)
            or mode.rangeMaxEnabled and (mode.usesInclusiveRange and val <= rangeMax or not mode.usesInclusiveRange and val < rangeMax) then
              obj.debugDrawProxy:drawBeam3d(beam.cid, beamScale, color(255, 0, 0, alpha * 255))

              beamsDrawn[bdi] = beam.cid
              bdi = bdi + 1
            end
          end
        end
      end
    end

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100, 0), color(255, 255, 255, 255), string.format("Range Min: %.2f", rangeMin))
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 120, 0), color(255, 0, 0, 255),     string.format("Range Max: %.2f", rangeMax))
      if mode.showInfinity then
        obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 140, 0), color(255, 0, 255, 255),  "Includes FLT_MAX")
      end
    end
  end

  -- If auto range enabled and at least one beam value exists, use it to calculate range min/max values
  if mode.autoRange and minVal ~= huge and maxVal ~= -huge then
    if not mode.rangeMinCap or (mode.rangeMinCap and minVal < mode.rangeMinCap) then
      mode.rangeMinCap = minVal
      dirty = true
    end

    if not mode.rangeMaxCap or (mode.rangeMaxCap and maxVal > mode.rangeMaxCap) then
      mode.rangeMaxCap = maxVal

      if mode.rangeMinCap == mode.rangeMaxCap then
        local magnitude = math.floor(math.log10(abs(mode.rangeMaxCap)))

        mode.rangeMaxCap = mode.rangeMaxCap + math.pow(10, magnitude - 1)
      end

      dirty = true
    end

    if not mode.rangeMin then
      mode.rangeMin = mode.rangeMinCap
      dirty = true
    end

    if not mode.rangeMax then
      mode.rangeMax = mode.rangeMaxCap
      dirty = true
    end
  end

  if requestDrawnBeamsCallbacks and next(requestDrawnBeamsCallbacks) ~= nil then
    for _, geFuncName in ipairs(requestDrawnBeamsCallbacks) do
      obj:queueGameEngineLua(geFuncName .. "(" .. serialize(beamsDrawn) .. "," .. beamScale .. ")")
    end
    table.clear(requestDrawnBeamsCallbacks)
  end

  return dirty
end

local function drawTorsionBar(torbar, nodeScale, beamScale, alpha, color1, color2)
  color2 = color2 or color1

  local r,g,b,a = colorGetRGBA(color1)
  color1 = color(r, g, b, alpha)

  local r,g,b,a = colorGetRGBA(color2)
  color2 = color(r, g, b, alpha)

  local id1, id2, id3, id4 = torbar.id1, torbar.id2, torbar.id3, torbar.id4

  if id1 and id2 and id3 and id4 then
    local node1Pos = obj:getAbsNodePosition(id1)
    local node2Pos = obj:getAbsNodePosition(id2)
    local node3Pos = obj:getAbsNodePosition(id3)
    local node4Pos = obj:getAbsNodePosition(id4)

    obj.debugDrawProxy:drawNodeSphere(id1, nodeScale, color(255, 0, 0, alpha))
    obj.debugDrawProxy:drawNodeSphere(id2, nodeScale, color(255, 125, 0, alpha))
    obj.debugDrawProxy:drawNodeSphere(id3, nodeScale, color(255, 255, 0, alpha))
    obj.debugDrawProxy:drawNodeSphere(id4, nodeScale, color(0, 255, 0, alpha))

    -- obj.debugDrawProxy:drawCylinder(node1Pos, node2Pos, beamScale, col)
    -- obj.debugDrawProxy:drawCylinder(node2Pos, node3Pos, beamScale, col)
    -- obj.debugDrawProxy:drawCylinder(node3Pos, node4Pos, beamScale, col)

    -- triangle 1
    obj.debugDrawProxy:drawCylinder(node1Pos, node2Pos, beamScale, color1)
    obj.debugDrawProxy:drawCylinder(node1Pos, node3Pos, beamScale, color1)

    --obj.debugDrawProxy:drawNodeTriangle(id1, id2, id3, 0, color(255, 0, 255, alpha * 0.5))

    -- triangle 2
    obj.debugDrawProxy:drawCylinder(node4Pos, node2Pos, beamScale, color2)
    obj.debugDrawProxy:drawCylinder(node4Pos, node3Pos, beamScale, color2)

    --obj.debugDrawProxy:drawNodeTriangle(id4, id2, id3, 0, color(0, 255, 255, alpha * 0.5))

    -- axis
    obj.debugDrawProxy:drawCylinder(node2Pos, node3Pos, beamScale * 4, color(255, 128, 0, alpha))
  end
end

local function visualizeTorsionBars()
  local dirty = false

  local partsSelected = M.partsState.partsSelected

  local modeID = M.state.vehicle.torsionBarVisMode
  local mode = M.state.vehicle.torsionBarVisModes[modeID]
  if not mode then return false end

  local modeName = mode.name

  local rangeMin = mode.rangeMin or -huge
  local rangeMax = mode.rangeMax or huge

  local minVal = huge
  local maxVal = -huge

  local nodeScale = 0.02 * M.state.vehicle.torsionBarVisWidthScale
  local beamScale = math.max(0.01 * M.state.vehicle.torsionBarVisWidthScale - 0.008, 0.00025)
  local alpha = M.state.vehicle.torsionBarVisAlpha * 255

  -- "off"
  if modeID == 1 then return dirty end

  if playerInfo.firstPlayerSeated then
    obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 60, 0), color(255, 165, 0, 255), "Mode: " .. modeName)
  end

  -- "simple"
  if modeID == 2 then
    for i = 0, torsionBarsCount - 1 do
      local torbar = v.data.torsionbars[i]
      if partsSelected[torbar.partPath or v.config.partsTree.partPath] then
        drawTorsionBar(torbar, nodeScale, beamScale, alpha, unpack(torbarTypesColors[TORBAR_NORMAL]))
      end
    end

  -- "type"
  elseif modeID == 3 then
    for i = 0, torsionBarsCount - 1 do
      local torbar = v.data.torsionbars[i]
      if partsSelected[torbar.partPath or v.config.partsTree.partPath] then
        local col1, col2
        if torbar.spring2 or torbar.damp2 then
          -- anisotropic torsionbar
          col1, col2 = unpack(torbarTypesColors[TORBAR_ANISOTROPIC])
        else
          -- regular torsionbar
          col1, col2 = unpack(torbarTypesColors[TORBAR_NORMAL])
        end

        drawTorsionBar(torbar, nodeScale, beamScale, alpha, col1, col2)
      end
    end

    -- Color legend
    if playerInfo.firstPlayerSeated then
      local j = 1
      for i = 1, #torbarTypesNames do
        if i ~= TORBAR_BROKEN then
          obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100 + j * 20, 0), torbarTypesColors[i][1], torbarTypesNames[i])
          j = j + 1
        end
      end
    end

  -- "withoutBroken", "withBroken", "brokenOnly"
  elseif modeID == 4 or modeID == 5 or modeID == 6 then
    for i = 0, torsionBarsCount - 1 do
      local torbar = v.data.torsionbars[i]
      if partsSelected[torbar.partPath or v.config.partsTree.partPath] then
        local torbarBroken = obj:torsionbarIsBroken(torbar.cid)
        if (modeID == 4 and not torbarBroken) or modeID == 5 or (modeID == 6 and torbarBroken) then
          --local startAngle, endAngle = 0, 0
          local col1, col2
          local sizeMult = 1

          if not torbarBroken then
            if torbar.spring2 or torbar.damp2 then
              -- anisotropic torsionbar, purple blue shade
              col1, col2 = unpack(torbarTypesColors[TORBAR_ANISOTROPIC])
            else
              -- regular torsionbar, green-cyan shade
              col1, col2 = unpack(torbarTypesColors[TORBAR_NORMAL])
            end
          else
            -- broken ones will have red-orange shade
            col1, col2 = unpack(torbarTypesColors[TORBAR_BROKEN])
            sizeMult = 2
          end

          --local col = jetColor((startAngle + (endAngle - startAngle) * torbar.cid / (torsionBarsCount+1))/ 360, alpha)
          drawTorsionBar(torbar, nodeScale * sizeMult, beamScale * sizeMult, alpha, col1, col2)
        end
      end
    end

    -- Color legend
    if playerInfo.firstPlayerSeated then
      for i = 1, #torbarTypesNames do
        obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100 + i * 20, 0), torbarTypesColors[i][1], torbarTypesNames[i])
      end
    end

  -- "angle"
  elseif modeID == 7 then
    local scaler = 1 / (rangeMax - rangeMin)

    for i = 0, torsionBarsCount - 1 do
      local torbar = v.data.torsionbars[i]
      if partsSelected[torbar.partPath or v.config.partsTree.partPath] then
        local id1, id2, id3, id4 = torbar.id1, torbar.id2, torbar.id3, torbar.id4

        if id1 and id2 and id3 and id4 then
          local angle = math.abs(obj:getTorsionbarAngle(i)) * 180.0 / math.pi

          if mode.rangeMinEnabled and mode.rangeMaxEnabled then
            if mode.usesInclusiveRange and angle >= rangeMin and angle <= rangeMax
            or not mode.usesInclusiveRange and angle > rangeMin and angle < rangeMax then
              local a = min((angle - rangeMin) * scaler, 1) * alpha
              drawTorsionBar(torbar, nodeScale, beamScale, a, color(0,255,0,255), color(0,255,255,255))
            end
          elseif not mode.rangeMinEnabled and not mode.rangeMaxEnabled
          or mode.rangeMinEnabled and (mode.usesInclusiveRange and angle >= rangeMin or not mode.usesInclusiveRange and angle > rangeMin)
          or mode.rangeMaxEnabled and (mode.usesInclusiveRange and angle <= rangeMax or not mode.usesInclusiveRange and angle < rangeMax) then
            drawTorsionBar(torbar, nodeScale, beamScale, alpha, color(255,0,0,255), color(255,128,0,255))
          end
        end
      end
    end

  -- "stress"
  elseif modeID == 8 then
    local scaler = 1 / (rangeMax - rangeMin)

    for i = 0, torsionBarsCount - 1 do
      local torbar = v.data.torsionbars[i]
      if partsSelected[torbar.partPath or v.config.partsTree.partPath] then
        local id1, id2, id3, id4 = torbar.id1, torbar.id2, torbar.id3, torbar.id4

        if id1 and id2 and id3 and id4 then
          local stress = math.abs(obj:getTorsionbarAngle(i)) * torbar.spring

          if mode.rangeMinEnabled and mode.rangeMaxEnabled then
            if mode.usesInclusiveRange and stress >= rangeMin and stress <= rangeMax
            or not mode.usesInclusiveRange and stress > rangeMin and stress < rangeMax then
              local a = min((stress - rangeMin) * scaler, 1) * alpha
              drawTorsionBar(torbar, nodeScale, beamScale, a, color(0,255,0,255), color(0,255,255,255))
            end
          elseif not mode.rangeMinEnabled and not mode.rangeMaxEnabled
          or mode.rangeMinEnabled and (mode.usesInclusiveRange and stress >= rangeMin or not mode.usesInclusiveRange and stress > rangeMin)
          or mode.rangeMaxEnabled and (mode.usesInclusiveRange and stress <= rangeMax or not mode.usesInclusiveRange and stress < rangeMax) then
            drawTorsionBar(torbar, nodeScale, beamScale, alpha, color(255,0,0,255), color(255,128,0,255))
          end
        end
      end
    end

  -- "deformation"
  elseif modeID == 9 then
    local deformRange = rangeMax - rangeMin
    for i = 0, torsionBarsCount - 1 do
      local torbar = v.data.torsionbars[i]
      if partsSelected[torbar.partPath or v.config.partsTree.partPath] then
        local id1, id2, id3, id4 = torbar.id1, torbar.id2, torbar.id3, torbar.id4
        if id1 and id2 and id3 and id4 then
          local deform = obj:getTorsionbarDeformation(i)
          local absDeform = abs(deform)

          if mode.rangeMinEnabled and mode.rangeMaxEnabled then
            if mode.usesInclusiveRange and absDeform >= rangeMin and absDeform <= rangeMax
            or not mode.usesInclusiveRange and absDeform > rangeMin and absDeform < rangeMax then
              local r = max(min((-deform - rangeMin) / deformRange, 1), 0) * 255
                --red for compression
              local b = max(min((deform - rangeMin) / deformRange, 1), 0) * 255
                --blue for elongation
              local a = min((absDeform - rangeMin) / deformRange, 1) * alpha
              drawTorsionBar(torbar, nodeScale, beamScale, a, color(r, 0, b, a))
            end
          elseif not mode.rangeMinEnabled and not mode.rangeMaxEnabled
          or mode.rangeMinEnabled and (mode.usesInclusiveRange and absDeform >= rangeMin or not mode.usesInclusiveRange and absDeform > rangeMin)
          or mode.rangeMaxEnabled and (mode.usesInclusiveRange and absDeform <= rangeMax or not mode.usesInclusiveRange and absDeform < rangeMax) then
            local r = deform < 0 and 255 or 0
            local b = deform >= 0 and 255 or 0
            drawTorsionBar(torbar, nodeScale, beamScale, alpha, color(r, 0, b, alpha))
          end
        end
      end
    end

  -- the rest
  elseif modeID >= 10 then
    -- Do rendering and get min/max values for next frame rendering
    local scaler = 1 / (rangeMax - rangeMin)

    for i = 0, torsionBarsCount - 1 do
      local torbar = v.data.torsionbars[i]
      if partsSelected[torbar.partPath or v.config.partsTree.partPath] then
        local val = tonumber(torbar[modeName])

        if val then
          minVal = val ~= -huge and min(val, minVal) or minVal
          maxVal = val ~= huge and max(val, maxVal) or maxVal

          if abs(val) == huge then
            if mode.showInfinity then
              local sizeMult = 1
              -- For 'spring' visualization mode, also set scale of nodes/beams
              if modeName == 'spring' then
                sizeMult = 3
              end
              drawTorsionBar(torbar, nodeScale * sizeMult, beamScale * sizeMult, alpha, color(255, 0, 255, alpha))
            end
          else
            if mode.rangeMinEnabled and mode.rangeMaxEnabled then
              if mode.usesInclusiveRange and val >= rangeMin and val <= rangeMax
              or not mode.usesInclusiveRange and val > rangeMin and val < rangeMax then
                local relValue = scaler * (val - rangeMin)
                --if not isnan(relValue) then
                local r = (relValue + (1 - relValue)) * 255
                local g = (1 - relValue) * 255
                local b = (1 - relValue) * 255

                local sizeMult = 1
                -- For 'spring' visualization mode, also set scale of nodes/beams
                if modeName == 'spring' then
                  sizeMult = relValue * 1.5 + 1
                end

                drawTorsionBar(torbar, nodeScale * sizeMult, beamScale * sizeMult, alpha, color(r, g, b, alpha))
                --end
              end
            elseif not mode.rangeMinEnabled and not mode.rangeMaxEnabled
            or mode.rangeMinEnabled and (mode.usesInclusiveRange and val >= rangeMin or not mode.usesInclusiveRange and val > rangeMin)
            or mode.rangeMaxEnabled and (mode.usesInclusiveRange and val <= rangeMax or not mode.usesInclusiveRange and val < rangeMax) then
              drawTorsionBar(torbar, nodeScale, beamScale, alpha, color(255, 0, 0, alpha))
            end
          end
        end
      end
    end

    if playerInfo.firstPlayerSeated then
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 100, 0), color(255, 255, 255, 255), string.format("Range Min: %.2f", rangeMin))
      obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 120, 0), color(255, 0, 0, 255),     string.format("Range Max: %.2f", rangeMax))
      if mode.showInfinity then
        obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 140, 0), color(255, 0, 255, 255),  "Includes FLT_MAX")
      end
    end
  end

  -- If auto range enabled and at least one torsionbar value exists, use it to calculate range min/max values
  if mode.autoRange and minVal ~= huge and maxVal ~= -huge then
    if not mode.rangeMinCap or (mode.rangeMinCap and minVal < mode.rangeMinCap) then
      mode.rangeMinCap = minVal
      dirty = true
    end

    if not mode.rangeMaxCap or (mode.rangeMaxCap and maxVal > mode.rangeMaxCap) then
      mode.rangeMaxCap = maxVal

      if mode.rangeMinCap == mode.rangeMaxCap then
        local magnitude = math.floor(math.log10(abs(mode.rangeMaxCap)))

        mode.rangeMaxCap = mode.rangeMaxCap + math.pow(10, magnitude - 1)
      end

      dirty = true
    end

    if not mode.rangeMin then
      mode.rangeMin = mode.rangeMinCap
      dirty = true
    end

    if not mode.rangeMax then
      mode.rangeMax = mode.rangeMaxCap
      dirty = true
    end
  end

  return dirty
end

local function drawRailSlidenodes(rail, slidenodes, nodeScale, beamScale, slideNodeScale, defaultCol, linkColorsSizes)
  -- Draw Rail
  local links = rail['links:']
  local linksNodeCount = #links

  for i = 2, linksNodeCount do
    local prevNodeCID = links[i - 1]
    local nodeCID = links[i]
    local nodePos = obj:getAbsNodePosition(nodeCID)
    local prevNodePos = obj:getAbsNodePosition(prevNodeCID)

    local col, sizeMult = defaultCol, 1

    if linkColorsSizes then
      local linkColSize = linkColorsSizes[i - 1]
      col = linkColSize.color
      sizeMult = linkColSize.sizeMult
    end

    obj.debugDrawProxy:drawNodeSphere(prevNodeCID, nodeScale * sizeMult, col)
    obj.debugDrawProxy:drawNodeSphere(nodeCID, nodeScale * sizeMult, col)
    obj.debugDrawProxy:drawCylinder(prevNodePos, nodePos, beamScale * sizeMult, col)
    prevNodePos = nodePos
  end

  -- Draw Slidenodes
  for _, slidenode in ipairs(slidenodes) do
    obj.debugDrawProxy:drawNodeSphere(slidenode.id, slideNodeScale, defaultCol)
  end
end

-- find slidenodes attached to this rail by rail's name
local function getSlideNodes(theRailName)
  local slidenodes = {}
  for j = 0, slidenodesCount - 1 do
    local slidenode = v.data.slidenodes[j]
    if slidenode.railName == theRailName then
      table.insert(slidenodes, slidenode)
    end
  end
  return slidenodes
end

-- Returns list of broken links
local function getBrokenRailLinks(rail)
  local links = rail['links:']
  local brokenLinks = {}
  for i = 1, #links - 1 do
    local beams = railsLinksBeams[rail][i]

    for k, beam in ipairs(beams) do
      if obj:beamIsBroken(beam.cid) then
        brokenLinks[i] = true
      end
    end
  end

  return brokenLinks
end

local function visualizeRailsSlideNodes()
  local partsSelected = M.partsState.partsSelected

  local modeID = M.state.vehicle.railsSlideNodesVisMode
  local mode = M.state.vehicle.railsSlideNodesVisModes[modeID]
  if not mode then return false end

  local modeName = mode.name

  local linkNodeScale = 0.01 * M.state.vehicle.railsSlideNodesVisWidthScale
  local beamScale = math.max(0.01 * M.state.vehicle.railsSlideNodesVisWidthScale - 0.008, 0.00025)
  local slideNodeScale = 0.02 * M.state.vehicle.railsSlideNodesVisWidthScale
  local alpha = M.state.vehicle.railsSlideNodesVisAlpha * 255

  -- "off"
  if modeID == 1 then return end

  -- initialization
  if not railsLinksBeams then
    railsLinksBeams = {}

    -- Find beams between t nodes
    for name, rail in pairs(v.data.rails or {}) do
      if name ~= 'cids' then
        local links = rail['links:']
        railsLinksBeams[rail] = {}

        for i = 2, #links do
          local prevNodeCID = links[i - 1]
          local nodeCID = links[i]

          railsLinksBeams[rail][i - 1] = {}

          -- Find beams between these two nodes
          for j = 0, beamsCount - 1 do
            local beam = v.data.beams[j]

            if (beam.id1 == prevNodeCID and beam.id2 == nodeCID) or (beam.id2 == prevNodeCID and beam.id1 == nodeCID) then
              table.insert(railsLinksBeams[rail][i - 1], beam)
            end
          end

          prevNodeCID = nodeCID
        end
      end
    end
  end

  if playerInfo.firstPlayerSeated then
    obj.debugDrawProxy:drawText2D(vec3(viewportSizeX - 450 - 40, 60, 0), color(255, 165, 0, 255), "Mode: " .. modeName)
  end

  -- "simple"
  if modeID == 2 then
    for name, rail in pairs(v.data.rails or {}) do
      if name ~= 'cids' then
        -- find slidenodes attached to this rail
        local slidenodes = getSlideNodes(name)
        local col = jetColor(rail.cid/(railsCount + 1), alpha)

        drawRailSlidenodes(rail, slidenodes, linkNodeScale, beamScale, slideNodeScale, col)
        --if partSelectedIdx == 1 or partSelected == rail.partPath then end
      end
    end

  -- "withoutBroken", "withBroken", "brokenOnly"
  elseif modeID == 3 or modeID == 4 or modeID == 5 then
    for name, rail in pairs(v.data.rails or {}) do
      if name ~= 'cids' then
        local links = rail['links:']
        local linksNodeCount = #links
        if links and linksNodeCount >= 2 then
          -- find slidenodes attached to this rail
          local slidenodes = getSlideNodes(name)
          local brokenLinks = getBrokenRailLinks(rail)

          if (modeID == 3 and not next(brokenLinks)) or modeID == 4 or (modeID == 5 and next(brokenLinks)) then
            -- non broken ones will have green-blue shade
            local startAngle, endAngle = 90, 240
            local col = jetColor((startAngle + (endAngle - startAngle) * rail.cid / (railsCount + 1)) / 360, alpha)

            -- broken ones will have red-orange shade
            startAngle, endAngle = 0, 45
            local brokenCol = jetColor((startAngle + (endAngle - startAngle) * rail.cid / (railsCount + 1)) / 360, alpha)

            local linkColorsSizes = {}
            for i = 1, linksNodeCount - 1 do
              linkColorsSizes[i] = {color = brokenLinks[i] and brokenCol or col, sizeMult = brokenLinks[i] and 2 or 1}
            end

            drawRailSlidenodes(rail, slidenodes, linkNodeScale, beamScale, slideNodeScale, col, linkColorsSizes)
          end
        end
      end
    end
  end
end

local function updateUIs()
  -- INTENTIONALLY CALLING FROM GAME ENGINE LUA TO WORKAROUND A BUG
  obj:queueGameEngineLua(string.format("guihooks.trigger('BdebugUpdate',%s,%s)", serialize(M.state), serialize(M.stateNoReset)))

  -- This is fine though
  obj:queueGameEngineLua(string.format("extensions.hook('onBDebugUpdate',%s,%s)", serialize(M.state), serialize(M.stateNoReset)))
end

local function recieveViewportSize(sizeX, sizeY)
  viewportSizeX, viewportSizeY = sizeX, sizeY
end

--local lastTime = 0
local function debugDraw(focusPos)
  -- local currTime = os.clock()
  -- local dt = currTime - lastTime
  -- lastTime = currTime

  local dirty = false

  obj:queueGameEngineLua("getObjectByID(" .. obj:getID() .. "):queueLuaCommand('bdebug.recieveViewportSize('.. ui_imgui.GetMainViewport().Size.x .. ',' .. ui_imgui.GetMainViewport().Size.y .. ')' )")

  visualizeWheelThermals()
  visualizeTireContactPoint()
  visualizeSteeringGeometry()
  visualizeCollisionTriangles()
  visualizeAerodynamics()
  visualizeCOG()

  visualizeNodesDebugTexts()
  visualizeNodesTexts()
  dirty = visualizeNodes() or dirty
  visualizeBeamsTexts()
  dirty = visualizeBeams() or dirty
  dirty = visualizeTorsionBars() or dirty
  visualizeRailsSlideNodes()

  if dirty then
    updateUIs()
  end
end

local function updateDebugDraw()
  -- Only enable debugDraw if one of the modes are enabled and M.state.vehicleDebugVisible is true
  M.debugDraw = nop
  for k, v in pairs(M.state.vehicle) do
    if type(v) ~= "table" and v ~= M.initState.vehicle[k] and M.state.vehicleDebugVisible then
      --lastTime = os.clock()
      M.debugDraw = debugDraw
      break
    end
  end

  -- "type + broken" | "brokenOnly"
  if ((M.state.vehicle.beamVisMode == 4 or M.state.vehicle.beamVisMode == 5) and M.state.vehicleDebugVisible) then
    -- Report beams broken before tool was open
    for id = 0, beamsCount - 1 do
      local beam = v.data.beams[id]
      if not beamsBroken[id] and obj:beamIsBroken(id) then
        log("I", "bdebug.beamBroken", string.format("beam %d broke: %s [%d]  ->  %s [%d]", id, (v.data.nodes[beam.id1].name or "unnamed"), beam.id1, (v.data.nodes[beam.id2].name or "unnamed"), beam.id2))
        guihooks.message({txt = "vehicle.beamstate.beamBroke", context = {id = id, id1 = beam.id1, id2 = beam.id2, id1name = v.data.nodes[beam.id1].name, id2name = v.data.nodes[beam.id2].name}})
      end
      beamsBroken[id] = true
    end
    M.beamBroke = beamBroke
  else
    M.beamBroke = nop
  end
end

local function updateDebugDrawAndSendState()
  updateDebugDraw()
  updateUIs()
end

-- Request/send drawn nodes to GE Lua function
local function requestDrawnNodesGE(geFuncName)
  requestDrawnNodesCallbacks = requestDrawnNodesCallbacks or {}
  table.insert(requestDrawnNodesCallbacks, geFuncName)
end

-- Request/send drawn beams to GE Lua function
local function requestDrawnBeamsGE(geFuncName)
  requestDrawnBeamsCallbacks = requestDrawnBeamsCallbacks or {}
  table.insert(requestDrawnBeamsCallbacks, geFuncName)
end

local function onPlayersChanged(m)
  if m then
    updateDebugDrawAndSendState()
  end
end

local function setState(state, stateNoReset, notSendBack)
  if state.objectId ~= objectId then
    return
  end

  M.state.vehicleDebugVisible = false
  M.state = state
  M.state.vehicle = M.state.vehicle or deepcopy(M.initState.vehicle)
  for k, v in pairs(M.state.vehicle) do
    if type(v) ~= "table" and v ~= M.initState.vehicle[k] then
      M.state.vehicleDebugVisible = true
    end
  end
  M.stateNoReset = stateNoReset

  updateDebugDraw()
  if not notSendBack then
    updateUIs()
  end
end

local function setMode(modeVar, modesVar, mode)
  if M.state.vehicle[modeVar] and M.state.vehicle[modesVar] then
    if mode > #M.state.vehicle[modesVar] then
      mode = 1
    elseif mode < 1 then
      mode = #M.state.vehicle[modesVar]
    end

    M.state.vehicle[modeVar] = mode

    if mode ~= 1 then
      M.state.vehicleDebugVisible = true
    end
  end

  updateDebugDrawAndSendState()
end

local function setPartsSelected(parts)
  M.partsState.partsSelected = parts
end

-- Sets the text to display at a node using the node debug text visualization
-- "type" is the group the text belongs to
-- "nodeCID" is the id of the node at runtime
-- "text" is the text you want to display at the node
local function setNodeDebugText(type, nodeCID, text)
  -- If type doesn't exist, create it!
  if not M.state then return
    log('E', 'bdebugImpl.setNodeDebugText', string.format('bdebugImpl.setNodeDebugText(%s, %d, %s) not successful because bdebugImpl.lua is not fully initialized!', type, nodeCID, text))
  end
  local id = M.state.vehicle.nodeDebugTextTypeToID[type]
  if not id then
    table.insert(M.state.vehicle.nodeDebugTextModes, {name = type, data = {}})
    M.state.vehicle.nodeDebugTextTypeToID[type] = #M.state.vehicle.nodeDebugTextModes
    id = M.state.vehicle.nodeDebugTextTypeToID[type]
  end
  local mode = M.state.vehicle.nodeDebugTextModes[id]

  -- If node data doesn't exist, create it!
  if not mode.data[nodeCID] then
    mode.data[nodeCID] =
    {
      textList = {},
    }
  end

  -- Add text to list
  table.insert(
    mode.data[nodeCID].textList,
    text
  )

  updateDebugDrawAndSendState()
end

-- Removes the text displaying at a node
-- "type" is the text group
-- "nodeCID" is the id of the node at runtime
local function clearNodeDebugText(type, nodeCID)
  local id = M.state.vehicle.nodeDebugTextTypeToID[type]
  if id then
    M.state.vehicle.nodeDebugTextModes[id].data[nodeCID] = nil
  end
  updateDebugDrawAndSendState()
end

-- Removes a specific text group
-- "type" is the text group
local function clearTypeNodeDebugText(type)
  local id = M.state.vehicle.nodeDebugTextTypeToID[type]
  if id then
    table.remove(M.state.vehicle.nodeDebugTextModes, id)

    -- Subtract one from mode to keep same mode selected
    if M.state.vehicle.nodeDebugTextMode >= id then
      M.state.vehicle.nodeDebugTextMode = M.state.vehicle.nodeDebugTextMode - 1
    end
    M.state.vehicle.nodeDebugTextTypeToID[type] = nil

    -- Update type to ID lookups as the ids have been shifted down
    for i = id, #M.state.vehicle.nodeDebugTextModes do
      local currType = M.state.vehicle.nodeDebugTextModes[i].name
      M.state.vehicle.nodeDebugTextTypeToID[currType] = M.state.vehicle.nodeDebugTextTypeToID[currType] - 1
    end
  end
  updateDebugDrawAndSendState()
end

-- Removes all text groups
local function clearAllNodeDebugText()
  M.state.vehicle.nodeDebugTextModes = {{name = "off"}}
  M.state.vehicle.nodeDebugTextMode = 1
  table.clear(M.state.vehicle.nodeDebugTextTypeToID)
  updateDebugDrawAndSendState()
end

local function isEnabled()
  return M.state.vehicleDebugVisible
end

local function setEnabled(enabled)
  M.state.vehicleDebugVisible = enabled
  updateDebugDrawAndSendState()
end

-- User input events

-- function used by the input subsystem - AND NOTHING ELSE
-- DO NOT use these from the UI
local function toggleEnabled()
  M.state.vehicleDebugVisible = not M.state.vehicleDebugVisible
  updateDebugDrawAndSendState()
end

local function nodetextModeChange(change)
  setMode("nodeTextMode", "nodeTextModes", M.state.vehicle.nodeTextMode + change)

  local modeName = M.state.vehicle.nodeTextModes[M.state.vehicle.nodeTextMode].name
  guihooks.message({txt = "vehicle.bdebug.nodeTextMode", context = {nodeTextMode = "vehicle.bdebug.nodeTextMode." .. modeName}}, 3, "debug")
end

local function nodevisModeChange(change)
  setMode("nodeVisMode", "nodeVisModes", M.state.vehicle.nodeVisMode + change)

  local modeName = M.state.vehicle.nodeVisModes[M.state.vehicle.nodeVisMode].name
  guihooks.message({txt = "vehicle.bdebug.nodeVisMode", context = {nodeVisMode = "vehicle.bdebug.nodeVisMode." .. modeName}}, 3, "debug")
end

local function nodedebugtextModeChange(change)
  setMode("nodeDebugTextMode", "nodeDebugTextModes", M.state.vehicle.nodeDebugTextMode + change)

  local modeName = M.state.vehicle.nodeDebugTextModes[M.state.vehicle.nodeDebugTextMode].name
  guihooks.message({txt = "vehicle.bdebug.nodeDebugTextMode", context = {nodeDebugTextMode = modeName}}, 3, "debug")
end

local function skeletonModeChange(change)
  setMode("beamVisMode", "beamVisModes", M.state.vehicle.beamVisMode + change)

  local modeName = M.state.vehicle.beamVisModes[M.state.vehicle.beamVisMode].name
  guihooks.message({txt = "vehicle.bdebug.beamVisMode", context = {beamVisMode = "vehicle.bdebug.beamVisMode." .. modeName}}, 3, "debug")
end

local function colTrisModeChange(change)
  setMode("collisionTriangleVisMode", "collisionTriangleVisModes", M.state.vehicle.collisionTriangleVisMode + change)

  local modeName = M.state.vehicle.collisionTriangleVisModes[M.state.vehicle.collisionTriangleVisMode].name
  guihooks.message({txt = "vehicle.bdebug.collisionTriangleVisMode", context = {collisionTriangleVisMode = "vehicle.bdebug.collisionTriangleVisMode." .. modeName}}, 3, "debug")
end

local function cogChange(change)
  setMode("cogMode", "cogModes", M.state.vehicle.cogMode + change)

  local modeName = M.state.vehicle.cogModes[M.state.vehicle.cogMode].name
  guihooks.message({txt = "vehicle.bdebug.cogMode", context = {cogMode = "vehicle.bdebug.cogMode." .. modeName}}, 3, "debug")
end

local function resetModes()
  M.state = deepcopy(M.initState)
  guihooks.message("vehicle.bdebug.clear", 3, "debug")
  updateDebugDrawAndSendState()
end

local function init(savedState, newPartialState)
  log('D', 'bdebugImpl.init', 'init')
  nodesCount = v.data.nodes and tableSizeC(v.data.nodes) or 0
  beamsCount = v.data.beams and tableSizeC(v.data.beams) or 0
  trisCount = v.data.triangles and tableSizeC(v.data.triangles) or 0
  torsionBarsCount = v.data.torsionbars and tableSizeC(v.data.torsionbars) or 0
  railsCount = v.data.rails and tableSize(v.data.rails) or 0
  slidenodesCount = v.data.slidenodes and tableSizeC(v.data.slidenodes) or 0

  railsLinksBeams = nil

  M.state = deepcopy(savedState or M.initState)
  M.state.vehicle.nodeDebugTextTypeToID = newPartialState.vehicle.nodeDebugTextTypeToID
  M.state.vehicle.nodeDebugTextMode = newPartialState.vehicle.nodeDebugTextMode
  M.state.vehicle.nodeDebugTextModes = newPartialState.vehicle.nodeDebugTextModes

  if newPartialState.partsSelected then
    setPartsSelected(newPartialState.partsSelected)
  end

  updateDebugDrawAndSendState()
end

local function reset()
  table.clear(beamsBroken)
  table.clear(beamsDeformed)
  table.clear(deformGroupsTriggerDisplayed)
end

M.nodeCollision = nop
M.beamBroke = nop
M.debugDraw = nop

M.recieveViewportSize = recieveViewportSize
M.requestState = updateUIs
M.requestDrawnNodesGE = requestDrawnNodesGE
M.requestDrawnBeamsGE = requestDrawnBeamsGE
M.onPlayersChanged = onPlayersChanged
M.setState = setState
M.setPartsSelected = setPartsSelected
M.setNodeDebugText = setNodeDebugText
M.clearNodeDebugText = clearNodeDebugText
M.clearTypeNodeDebugText = clearTypeNodeDebugText
M.clearAllNodeDebugText = clearAllNodeDebugText

M.isEnabled = isEnabled
M.setEnabled = setEnabled
M.toggleEnabled = toggleEnabled
M.nodetextModeChange = nodetextModeChange
M.nodevisModeChange = nodevisModeChange
M.nodedebugtextModeChange = nodedebugtextModeChange
M.skeletonModeChange = skeletonModeChange
M.colTrisModeChange = colTrisModeChange
M.cogChange = cogChange
M.resetModes = resetModes

M.init = init
M.reset = reset

return M