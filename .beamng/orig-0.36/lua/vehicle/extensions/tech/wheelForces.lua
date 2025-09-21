-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a test extension for research about what wheel information we can get from the simulation.

local M = {}

local wheelData = {}
local tmpVectorSum = vec3()

local drawWheelCoordinateSystem = false
local drawRimBeamForces = false
local drawSidewallBeamForces = false
local drawSuspensionBeamForces = true
local drawTreadNodeForces = false

local function onPhysicsStep(dt)
  --print("wheellogger physics step")
  --obj:getNodeForceVector()
  for _, data in ipairs(wheelData) do
    local smoothX
    local smoothY
    local smoothZ

    if drawRimBeamForces then
      tmpVectorSum:set(0, 0, 0)
      for _, rBeam in ipairs(data.rimBeamsNode1) do
        local beamStress = obj:getBeamStress(rBeam)
        local beamVector = obj:getBeamVectorFromNode(rBeam, data.axleNode1):normalized()
        local forceVector = beamVector * beamStress
        tmpVectorSum = tmpVectorSum + forceVector
      end
      tmpVectorSum = tmpVectorSum * 0.001
      smoothX = data.rimBeamsNode1SmootherX:get(tmpVectorSum.x, dt)
      smoothY = data.rimBeamsNode1SmootherY:get(tmpVectorSum.y, dt)
      smoothZ = data.rimBeamsNode1SmootherZ:get(tmpVectorSum.z, dt)
      data.rimBeamsNode1VectorSum:set(smoothX, smoothY, smoothZ)

      tmpVectorSum:set(0, 0, 0)
      for _, rBeam in ipairs(data.rimBeamsNode2) do
        local beamStress = obj:getBeamStress(rBeam)
        local beamVector = obj:getBeamVectorFromNode(rBeam, data.axleNode2):normalized()
        local forceVector = beamVector * beamStress
        tmpVectorSum = tmpVectorSum + forceVector
      end
      tmpVectorSum = tmpVectorSum * 0.001
      smoothX = data.rimBeamsNode2SmootherX:get(tmpVectorSum.x, dt)
      smoothY = data.rimBeamsNode2SmootherY:get(tmpVectorSum.y, dt)
      smoothZ = data.rimBeamsNode2SmootherZ:get(tmpVectorSum.z, dt)
      data.rimBeamsNode2VectorSum:set(smoothX, smoothY, smoothZ)
    end

    if drawSidewallBeamForces then
      tmpVectorSum:set(0, 0, 0)
      for _, sBeamData in ipairs(data.sidewallBeamsNode1) do
        local beamStress = obj:getBeamStress(sBeamData.beamId)
        local beamVector = obj:getBeamVectorFromNode(sBeamData.beamId, sBeamData.rimNodeId):normalized()
        local forceVector = beamVector * beamStress
        tmpVectorSum = tmpVectorSum + forceVector
      end
      tmpVectorSum = tmpVectorSum * 0.001
      smoothX = data.sidewallBeamsNode1SmootherX:get(tmpVectorSum.x, dt)
      smoothY = data.sidewallBeamsNode1SmootherY:get(tmpVectorSum.y, dt)
      smoothZ = data.sidewallBeamsNode1SmootherZ:get(tmpVectorSum.z, dt)
      data.sidewallBeamsNode1VectorSum:set(smoothX, smoothY, smoothZ)

      tmpVectorSum:set(0, 0, 0)
      for _, sBeamData in ipairs(data.sidewallBeamsNode2) do
        local beamStress = obj:getBeamStress(sBeamData.beamId)
        local beamVector = obj:getBeamVectorFromNode(sBeamData.beamId, sBeamData.rimNodeId):normalized()
        local forceVector = beamVector * beamStress
        tmpVectorSum = tmpVectorSum + forceVector
      end
      tmpVectorSum = tmpVectorSum * 0.001
      smoothX = data.sidewallBeamsNode2SmootherX:get(tmpVectorSum.x, dt)
      smoothY = data.sidewallBeamsNode2SmootherY:get(tmpVectorSum.y, dt)
      smoothZ = data.sidewallBeamsNode2SmootherZ:get(tmpVectorSum.z, dt)
      data.sidewallBeamsNode2VectorSum:set(smoothX, smoothY, smoothZ)
    end

    if drawSuspensionBeamForces then
      tmpVectorSum:set(0, 0, 0)
      for _, sBeam in ipairs(data.suspensionBeamsNode1) do
        local beamStress = obj:getBeamStress(sBeam)
        local beamVector = obj:getBeamVectorFromNode(sBeam, data.axleNode1):normalized()
        local forceVector = beamVector * beamStress
        tmpVectorSum = tmpVectorSum + forceVector
      end
      tmpVectorSum = tmpVectorSum * 0.001
      smoothX = data.suspensionBeamsNode1SmootherX:get(tmpVectorSum.x, dt)
      smoothY = data.suspensionBeamsNode1SmootherY:get(tmpVectorSum.y, dt)
      smoothZ = data.suspensionBeamsNode1SmootherZ:get(tmpVectorSum.z, dt)
      data.suspensionBeamsNode1VectorSum:set(smoothX, smoothY, smoothZ)


      tmpVectorSum:set(0, 0, 0)
      for _, sBeam in ipairs(data.suspensionBeamsNode2) do
        local beamStress = obj:getBeamStress(sBeam)
        local beamVector = obj:getBeamVectorFromNode(sBeam, data.axleNode2):normalized()
        local forceVector = beamVector * beamStress
        tmpVectorSum = tmpVectorSum + forceVector
      end
      tmpVectorSum = tmpVectorSum * 0.001
      smoothX = data.suspensionBeamsNode2SmootherX:get(tmpVectorSum.x, dt)
      smoothY = data.suspensionBeamsNode2SmootherY:get(tmpVectorSum.y, dt)
      smoothZ = data.suspensionBeamsNode2SmootherZ:get(tmpVectorSum.z, dt)
      data.suspensionBeamsNode2VectorSum:set(smoothX, smoothY, smoothZ)

    end

    if drawTreadNodeForces then
      tmpVectorSum:set(0, 0, 0)
      for _, tNode in ipairs(data.treadNodes) do
        local forceVector = obj:getNodeForceVector(tNode):normalized()
        tmpVectorSum = tmpVectorSum + forceVector
      end
      tmpVectorSum = tmpVectorSum * 0.001
      smoothX = data.treadNodesSmootherX:get(tmpVectorSum.x, dt)
      smoothY = data.treadNodesSmootherY:get(tmpVectorSum.y, dt)
      smoothZ = data.treadNodesSmootherZ:get(tmpVectorSum.z, dt)
      data.treadNodesVectorSum:set(smoothX, smoothY, smoothZ)
    end
  end
end

local function updateGFX()
end

local function onDebugDraw(focusPos)
  local vehiclePos = obj:getPosition()
  local upVector = obj:getDirectionVectorUp()
  --local leftVector = -obj:getDirectionVectorRight()
  --local forwardVector = obj:getDirectionVector()
  for _, data in ipairs(wheelData) do
    local axleNodePos1 = obj:getNodePosition(data.axleNode1)
    local axleNodePos2 = obj:getNodePosition(data.axleNode2)
    local wheelLeftVector = (axleNodePos1 - axleNodePos2):normalized() * data.wheelDirection
    local wheelForwardVector = upVector:cross(wheelLeftVector)

    local p1 = vehiclePos + axleNodePos1 + wheelLeftVector * data.wheelDirection * 0.2
    local p2

    if drawWheelCoordinateSystem then
      local p2RefX = p1 + wheelLeftVector
      local p2RefY = p1 + wheelForwardVector
      local p2RefZ = p1 + upVector
      obj.debugDrawProxy:drawCylinder(p1, p2RefX, 0.01, color(255, 0, 0, 255))
      obj.debugDrawProxy:drawCylinder(p1, p2RefY, 0.01, color(0, 0, 255, 255))
      obj.debugDrawProxy:drawCylinder(p1, p2RefZ, 0.01, color(0, 255, 0, 255))
    end

    if drawRimBeamForces then
      local vectorSum = data.rimBeamsNode1VectorSum + data.rimBeamsNode2VectorSum

      local p2x = p1 + vectorSum:dot(wheelLeftVector) * wheelLeftVector
      local p2y = p1 + vectorSum:dot(wheelForwardVector) * wheelForwardVector
      local p2z = p1 + vectorSum:dot(upVector) * upVector
      obj.debugDrawProxy:drawCylinder(p1, p2x, 0.05, color(0, 255, 0, 255))
      obj.debugDrawProxy:drawCylinder(p1, p2y, 0.05, color(255, 0, 0, 255))
      obj.debugDrawProxy:drawCylinder(p1, p2z, 0.05, color(0, 0, 255, 255))

      for _, beam in ipairs(data.rimBeamsNode1) do
        p1 = vehiclePos + obj:getNodePosition(v.data.beams[beam].id1)
        p2 = vehiclePos + obj:getNodePosition(v.data.beams[beam].id2)
        obj.debugDrawProxy:drawCylinder(p1, p2, 0.01, color(255, 255, 0, 128))
      end
      for _, beam in ipairs(data.rimBeamsNode2) do
        p1 = vehiclePos + obj:getNodePosition(v.data.beams[beam].id1)
        p2 = vehiclePos + obj:getNodePosition(v.data.beams[beam].id2)
        obj.debugDrawProxy:drawCylinder(p1, p2, 0.01, color(255, 255, 0, 128))
      end
    end

    if drawSidewallBeamForces then
      local vectorSum = data.sidewallBeamsNode1VectorSum + data.sidewallBeamsNode2VectorSum

      local p2x = p1 + vectorSum:dot(wheelLeftVector) * wheelLeftVector
      local p2y = p1 + vectorSum:dot(wheelForwardVector) * wheelForwardVector
      local p2z = p1 + vectorSum:dot(upVector) * upVector

      obj.debugDrawProxy:drawCylinder(p1, p2x, 0.05, color(0, 255, 0, 255))
      obj.debugDrawProxy:drawCylinder(p1, p2y, 0.05, color(255, 0, 0, 255))
      obj.debugDrawProxy:drawCylinder(p1, p2z, 0.05, color(0, 0, 255, 255))

      for _, beamData in ipairs(data.sidewallBeamsNode1) do
        p1 = vehiclePos + obj:getNodePosition(v.data.beams[beamData.beamId].id1)
        p2 = vehiclePos + obj:getNodePosition(v.data.beams[beamData.beamId].id2)
        obj.debugDrawProxy:drawCylinder(p1, p2, 0.01, color(128, 255, 128, 128))
      end
      for _, beamData in ipairs(data.sidewallBeamsNode2) do
        p1 = vehiclePos + obj:getNodePosition(v.data.beams[beamData.beamId].id1)
        p2 = vehiclePos + obj:getNodePosition(v.data.beams[beamData.beamId].id2)
        obj.debugDrawProxy:drawCylinder(p1, p2, 0.01, color(128, 255, 128, 128))
      end
    end

    if drawSuspensionBeamForces then
      local vectorSum = - data.suspensionBeamsNode1VectorSum - data.suspensionBeamsNode2VectorSum

      local p2x = p1 + vectorSum:dot(wheelLeftVector) * wheelLeftVector
      local p2y = p1 + vectorSum:dot(wheelForwardVector) * wheelForwardVector
      local p2z = p1 + vectorSum:dot(upVector) * upVector

      obj.debugDrawProxy:drawCylinder(p1, p2x, 0.05, color(0, 255, 0, 255))
      obj.debugDrawProxy:drawCylinder(p1, p2y, 0.05, color(255, 0, 0, 255))
      obj.debugDrawProxy:drawCylinder(p1, p2z, 0.05, color(0, 0, 255, 255))

      for _, beamId in ipairs(data.suspensionBeamsNode1) do
        p1 = vehiclePos + obj:getNodePosition(v.data.beams[beamId].id1)
        p2 = vehiclePos + obj:getNodePosition(v.data.beams[beamId].id2)
        obj.debugDrawProxy:drawCylinder(p1, p2, 0.01, color(128, 255, 128, 128))
      end
      for _, beamId in ipairs(data.suspensionBeamsNode2) do
        p1 = vehiclePos + obj:getNodePosition(v.data.beams[beamId].id1)
        p2 = vehiclePos + obj:getNodePosition(v.data.beams[beamId].id2)
        obj.debugDrawProxy:drawCylinder(p1, p2, 0.01, color(255, 255, 0, 128))
      end
    end

    if drawTreadNodeForces then
      local vectorSum = data.treadNodesVectorSum

      local p2x = p1 + vectorSum:dot(wheelLeftVector) * wheelLeftVector
      local p2y = p1 + vectorSum:dot(wheelForwardVector) * wheelForwardVector
      local p2z = p1 + vectorSum:dot(upVector) * upVector

      obj.debugDrawProxy:drawCylinder(p1, p2x, 0.05, color(0, 0, 255, 255))
      obj.debugDrawProxy:drawCylinder(p1, p2y, 0.05, color(0, 255, 0, 255))
      obj.debugDrawProxy:drawCylinder(p1, p2z, 0.05, color(255, 0, 0, 255))

      for _, treadNodeId in ipairs(data.treadNodes) do
        p1 = vehiclePos + obj:getNodePosition(treadNodeId)
        obj.debugDrawProxy:drawSphere(0.02, p1, color(255, 0, 0, 255))
      end
    end
  end
end

local function onExtensionLoaded()
  enablePhysicsStepHook()
  print("wheellogger loaded")
  wheelData = {}

  for id, wd in pairs(wheels.wheels) do
    local data = {}
    data.wheelId = id
    data.wheelName = wd.name
    data.axleNode1 = wd.node1
    data.axleNode2 = wd.node2
    data.wheelDirection = wd.wheelDir
    --rim beams
    data.rimBeamsNode1 = {}
    data.rimBeamsNode2 = {}
    data.rimBeamsNode1VectorSum = vec3()
    data.rimBeamsNode2VectorSum = vec3()
    data.rimBeamsNode1SmootherX = newTemporalSmoothing(15, 15)
    data.rimBeamsNode1SmootherY = newTemporalSmoothing(15, 15)
    data.rimBeamsNode1SmootherZ = newTemporalSmoothing(15, 15)
    data.rimBeamsNode2SmootherX = newTemporalSmoothing(15, 15)
    data.rimBeamsNode2SmootherY = newTemporalSmoothing(15, 15)
    data.rimBeamsNode2SmootherZ = newTemporalSmoothing(15, 15)
    --sidewall beams
    data.sidewallBeamsNode1 = {}
    data.sidewallBeamsNode2 = {}
    data.sidewallBeamsNode1VectorSum = vec3()
    data.sidewallBeamsNode2VectorSum = vec3()
    data.sidewallBeamsNode1SmootherX = newTemporalSmoothing(15, 15)
    data.sidewallBeamsNode1SmootherY = newTemporalSmoothing(15, 15)
    data.sidewallBeamsNode1SmootherZ = newTemporalSmoothing(15, 15)
    data.sidewallBeamsNode2SmootherX = newTemporalSmoothing(15, 15)
    data.sidewallBeamsNode2SmootherY = newTemporalSmoothing(15, 15)
    data.sidewallBeamsNode2SmootherZ = newTemporalSmoothing(15, 15)
    --suspension beams
    data.suspensionBeamsNode1 = {}
    data.suspensionBeamsNode2 = {}
    data.suspensionBeamsNode2VectorSum = vec3()
    data.suspensionBeamsNode1VectorSum = vec3()
    data.suspensionBeamsNode1SmootherX = newTemporalSmoothing(15, 15)
    data.suspensionBeamsNode1SmootherY = newTemporalSmoothing(15, 15)
    data.suspensionBeamsNode1SmootherZ = newTemporalSmoothing(15, 15)
    data.suspensionBeamsNode2SmootherX = newTemporalSmoothing(15, 15)
    data.suspensionBeamsNode2SmootherY = newTemporalSmoothing(15, 15)
    data.suspensionBeamsNode2SmootherZ = newTemporalSmoothing(15, 15)
    --tread nodes
    data.treadNodes = {}
    data.treadNodesVectorSum = vec3()
    data.treadNodesSmootherX = newTemporalSmoothing(15, 15)
    data.treadNodesSmootherY = newTemporalSmoothing(15, 15)
    data.treadNodesSmootherZ = newTemporalSmoothing(15, 15)

    local node1 = wd.node1
    local node2 = wd.node2
    local node1BeamEndNodeLookup = {}
    local node2BeamEndNodeLookup = {}
    local rimBeamLookup = {}

    local rimBeams = v.data.wheels[id].rimBeams

    for _, beamId in pairs(rimBeams) do
      local rBeam = v.data.beams[beamId]
      rimBeamLookup[beamId] = true
      if rBeam.id1 == node1 then
        node1BeamEndNodeLookup[rBeam.id2] = rBeam.id2
        table.insert(data.rimBeamsNode1, beamId)
      elseif rBeam.id2 == node1 then
        node1BeamEndNodeLookup[rBeam.id1] = rBeam.id1
        table.insert(data.rimBeamsNode1, beamId)
      elseif rBeam.id1 == node2 then
        node2BeamEndNodeLookup[rBeam.id2] = rBeam.id2
        table.insert(data.rimBeamsNode2, beamId)
      elseif rBeam.id2 == node2 then
        node2BeamEndNodeLookup[rBeam.id1] = rBeam.id1
        table.insert(data.rimBeamsNode2, beamId)
      else
        print("rim beam not attached to node1 or node2")
      end
    end

    local sidewallBeams = v.data.wheels[id].sideBeams
    local sidewallReinfBeams = v.data.wheels[id].reinfBeams

    for _, beamId in pairs(sidewallBeams) do
      local sBeam = v.data.beams[beamId]
      local node1BeamEndNode = node1BeamEndNodeLookup[sBeam.id1] or node1BeamEndNodeLookup[sBeam.id2]
      local node2BeamEndNode = node2BeamEndNodeLookup[sBeam.id1] or node2BeamEndNodeLookup[sBeam.id2]
      if node1BeamEndNode then
        table.insert(data.sidewallBeamsNode1, {beamId = beamId, rimNodeId = node1BeamEndNode})
      elseif node2BeamEndNode then
        table.insert(data.sidewallBeamsNode2, {beamId = beamId, rimNodeId = node2BeamEndNode})
      else
        print("sidewall beam not attached to any rim beam")
      end
    end

    for _, beamId in pairs(sidewallReinfBeams) do
      local rBeam = v.data.beams[beamId]
      local node1BeamEndNode = node1BeamEndNodeLookup[rBeam.id1] or node1BeamEndNodeLookup[rBeam.id2]
      local node2BeamEndNode = node2BeamEndNodeLookup[rBeam.id1] or node2BeamEndNodeLookup[rBeam.id2]
      if node1BeamEndNodeLookup[rBeam.id1] or node1BeamEndNodeLookup[rBeam.id2] then
        table.insert(data.sidewallBeamsNode1, {beamId = beamId, rimNodeId = node1BeamEndNode})
      elseif node2BeamEndNodeLookup[rBeam.id1] or node2BeamEndNodeLookup[rBeam.id2] then
        table.insert(data.sidewallBeamsNode2, {beamId = beamId, rimNodeId = node2BeamEndNode})
      else
        print("sidewall reinf beam not attached to any rim beam")
      end
    end
--dump(v.data.beams)
    for beamId, beam in pairs(v.data.beams) do
      --we only care for non-rim beams
      if not rimBeamLookup[beamId] then
        if (beam.id1 == node1 and beam.id2 == node2) or (beam.id2 == node1 and beam.id1 == node2) then break end
        if beam.id1 == node1 or beam.id2 == node1 then --if this beam connects to axle node 1 of the current wheel
          table.insert(data.suspensionBeamsNode1, beamId)
        elseif beam.id1 == node2 or beam.id2 == node2 then --if this beam connects to axle node 2 of the current wheel
          table.insert(data.suspensionBeamsNode2, beamId)
        end
      end
    end
    print("wheelforces.lua: " .. wd.name)
    dump(data.suspensionBeamsNode1)
    dump(data.suspensionBeamsNode2)

    for _, treadNodeId in pairs(v.data.wheels[id].treadNodes) do
      table.insert(data.treadNodes, treadNodeId)
    end

    table.insert(wheelData, data)
  end
end

local function onReset()
end

M.onExtensionLoaded = onExtensionLoaded
M.onReset = onReset
M.updateGFX = updateGFX
M.onPhysicsStep = onPhysicsStep
M.onDebugDraw = onDebugDraw

return M
