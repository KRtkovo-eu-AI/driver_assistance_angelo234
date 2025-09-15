-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local max = math.max
local min = math.min
local abs = math.abs

local function update(cylinder, inputPressure, dt)
  --**VALVE LOGIC**--
  local valveDirection = electrics.values[cylinder.directionElectricsName] or 0
  local valveSign = sign(valveDirection)
  local valveDirectionSmooth = cylinder.valveSmoother:getCapped(valveDirection or 0, dt)
  valveDirection = valveSign * min(abs(valveDirectionSmooth), abs(valveDirectionSmooth))
  --**/VALVE LOGIC**--

  local cylinderArea = (valveSign >= 0 and cylinder.cylinderExtendArea or cylinder.cylinderContractArea)

  cylinder.valveDirection = valveDirection --for logger

  --types of dcv: (directional control valve)
  --float center: A B connect to T, P blocked, so, slipForce = 0, pumpFlow is blocked
  --[WON'T DO] open center: A B and P connect to T, so, slipForce = 0, pumpFlow is free
  --[WON'T DO] tandem center: A B blocked, P connected to T, so slipForce is high, pumpFlow is free
  --closed center: A B P T blocked, so slipForce is high, pumpFlow is blocked
  --drop: like closed but with high enough input, A/B connect to T rather than being blocked
  --[WON'T DO] X regen center: A B connect to P, T is blocked (we prob dont need this, its used to extend cylinders faster than pump flow alone)

  --**VALVE LOGIC**--

  local dragCoef = (cylinder.minimumDragCoef + (1 - valveDirection * valveDirection) * cylinder.dragCoefRange)
  local speedLimit = valveDirection * valveDirection * cylinder.maxSpeed

  local slipForce = cylinder.cylinderReliefPressure * cylinder.invBeamCount * cylinderArea
  local cylinderMaxForce = cylinderArea * cylinder.invBeamCount * inputPressure
  local cylinderForce = cylinderMaxForce * valveSign

  --special hybrid valve type where A & B are conencted to T upon activation (wheeloader bucket drops to ground, also used for one sided cylinder ie dump truck)

  if cylinder.valveType == "drop" then
    --experiment if scaling might be a better option rather than outright using a threshold
    if valveDirection < cylinder.dropInputThreshold then
      slipForce = 0
      cylinderForce = 0
    end
  elseif cylinder.valveType == "float" then
    dragCoef = cylinder.minimumDragCoef
    slipForce = 0
    cylinderForce = cylinderMaxForce * valveDirection
  end
  --**/VALVE LOGIC**--

  --cylinder.cylinderForce = cylinderForce --for logger

  local cylinderForceSmooth = cylinder.cylinderForceSmoother:get(cylinderForce)
  cylinderForce = sign(cylinderForce) * min(abs(cylinderForceSmooth), abs(cylinderForce))

  --cylinder.cylinderForceSmooth = cylinderForce --for logger

  cylinder.cylinderFlow = 0

  --this inner loop is for sets of beams representing 1 cylinder in jbeam
  local currentExtend = 0
  local currentBeamVelocity = 0
  for i = 1, cylinder.beamCount do
    local cid = cylinder.beamCids[i]
    --actuateBeam(int outId, float force, float speedLimit, float slipForce, float frictionForce, float slipSpeedLimit, float minExtend, float maxExtend)
    local previousCylinderVelocity = cylinder.previousBeamVelocities[cid]
    local dragForce = previousCylinderVelocity * previousCylinderVelocity * cylinderArea * cylinder.invBeamCount * dragCoef + cylinder.frictionCoef
    local beamVelocity = obj:actuateBeam(cid, cylinderForce, speedLimit, slipForce, dragForce, cylinder.cylinderReliefSlipSpeedLimit, cylinder.minExtend, cylinder.maxExtend)
    currentExtend = currentExtend + obj:getBeamLength(cid)
    currentBeamVelocity = currentBeamVelocity + beamVelocity
    --print(string.format("%.2f - %.2f - %.2f", cylinder.minExtend, currentExtend, cylinder.maxExtend))
    cylinder.cylinderFlow = cylinder.cylinderFlow + beamVelocity * cylinderArea
    cylinder.previousBeamVelocities[cid] = cylinder.beamVelocitySmoothers[cid]:get(beamVelocity)
  end
  cylinder.currentExtend = currentExtend * cylinder.invBeamCount
  cylinder.currentExtendPercent = linearScale(cylinder.currentExtend, cylinder.minExtend, cylinder.maxExtend, 0, 1)
  cylinder.velocity = currentBeamVelocity * cylinder.invBeamCount

  local tankFlow = 0
  --**VALVE LOGIC**--
  if cylinder.valveType == "drop" then
    --special hybrid valve type where A & B are conencted to T upon activation (wheeloader bucket drops to ground)
    --experiment if scaling might be a better option rather than outright using a threshold
    if valveDirection < cylinder.dropInputThreshold then
      --set beam velocity to 0 to not affect the acc
      cylinder.cylinderFlow = 0
      tankFlow = 0.1
    end
  end
  --**/VALVE LOGIC**--

  --allow pump to flow when valve is closed but force flow to cylinder as valve opens
  return max(0, cylinder.cylinderFlow * valveSign), tankFlow
end

local function updateSounds(cylinder, dt)
  if cylinder.movementSound then
    local absVelocitySmooth = cylinder.movementVelocitySmoothing:get(abs(cylinder.velocity), dt)
    local volume = linearScale(absVelocitySmooth, cylinder.movementLoopVolumeMinVelocity, cylinder.movementLoopVolumeMaxVelocity, cylinder.movementLoopVolumeMin, cylinder.movementLoopVolumeMax)
    obj:setVolumePitchCT(cylinder.movementSound, volume, 1, 0, 0)

    if cylinder.showDebugGraphSound then
      guihooks.graph({"Velocity", cylinder.velocity, 0.2, "", true}, {"Velocity (smooth)", absVelocitySmooth, 0.2, "", true}, {"Volume", volume, 1, ""})
    end
  end
end

local function initSounds(cylinder, cylinderData)
  local cylinderMovementEvent = cylinderData.movementLoopEvent
  local movementLoopNode = cylinderData.movementLoopNode and beamstate.nodeNameMap[cylinderData.movementLoopNode]
  local movementLoopNodeId = movementLoopNode or 0
  if cylinderMovementEvent then
    cylinder.movementSound = obj:createSFXSource2(cylinderMovementEvent, "AudioDefaultLoop3D", "movementSound", movementLoopNodeId, 1)
    obj:setVolumePitchCT(cylinder.movementSound, 0, 0, 0, 0)
    obj:playSFX(cylinder.movementSound)
  end

  bdebug.setNodeDebugText("Hydraulics", movementLoopNodeId, cylinder.name .. " - Cylinder Movement Loop: " .. (cylinderMovementEvent or "no event"))

  local velocitySmoothingInRate = cylinderData.movementLoopVelocitySmoothingInRate or 5
  local velocitySmoothingStartAccel = cylinderData.movementLoopVelocitySmoothingStartAccel or 2
  local velocitySmoothingStopAccel = cylinderData.movementLoopVelocitySmoothingStopAccel or 2
  local velocitySmoothingOutRate = cylinderData.movementLoopVelocitySmoothingOutRate or 5
  cylinder.movementVelocitySmoothing = newTemporalSigmoidSmoothing(velocitySmoothingInRate, velocitySmoothingStartAccel, velocitySmoothingStopAccel, velocitySmoothingOutRate)

  cylinder.movementLoopVolumeMin = cylinderData.movementLoopVolumeMin or 0
  cylinder.movementLoopVolumeMax = cylinderData.movementLoopVolumeMax or 1
  cylinder.movementLoopVolumeMinVelocity = cylinderData.movementLoopVolumeMinVelocity or 0.001
  cylinder.movementLoopVolumeMaxVelocity = cylinderData.movementLoopVolumeMaxVelocity or 0.05

  cylinder.showDebugGraphSound = cylinderData.showDebugGraphSound or false
end

local function resetSounds(cylinder, cylinderData)
  cylinder.movementVelocitySmoothing:reset()
end

local function reset(cylinder, jbeamData)
  cylinder.cylinderForceSmoother:reset()
  cylinder.valveSmoother:reset()
  cylinder.velocity = 0
  cylinder.currentExtend = 0
  cylinder.currentExtendPercent = 0

  for _, bvs in pairs(cylinder.beamVelocitySmoothers) do
    bvs:reset()
  end
  for k, _ in pairs(cylinder.previousBeamVelocities) do
    cylinder.previousBeamVelocities[k] = 0
  end
end

local function new(cylinderData, pumpDevice)
  local cylinder = {
    connectedPump = pumpDevice,
    name = cylinderData.name,
    valveType = cylinderData.valveType or "closed", --drop, float
    dropInputThreshold = cylinderData.dropInputThreshold or -0.8,
    cylinderReliefPressure = cylinderData.cylinderReliefPressure or 50000000,
    cylinderReliefSlipSpeedLimit = cylinderData.cylinderReliefSlipSpeedLimit or 0.1,
    minimumDragCoef = cylinderData.minimumDragCoef or 10000000,
    maximumDragCoef = cylinderData.maximumDragCoef or 100000000,
    frictionCoef = cylinderData.frictionCoef or 1,
    maxSpeed = cylinderData.maxSpeed,
    minExtend = cylinderData.minExtend,
    maxExtend = cylinderData.maxExtend,
    currentExtend = 0,
    currentExtendPercent = 0,
    velocity = 0,
    directionElectricsName = cylinderData.directionElectricsName,
    beamCids = {},
    beamCount = 0,
    invBeamCount = 0,
    beamVelocitySmoothers = {},
    previousBeamVelocities = {},
    cylinderForceSmoother = newExponentialSmoothing(50),
    valveSmoother = newTemporalSmoothing(10, 10),
    reset = reset,
    initSounds = initSounds,
    resetSounds = resetSounds,
    update = update,
    updateSounds = updateSounds
  }

  cylinder.dragCoefRange = max(cylinder.maximumDragCoef - cylinder.minimumDragCoef, 0)

  cylinder.cylinderExtendArea = cylinderData.pistonDiameter * cylinderData.pistonDiameter * 3.1416 / 4
  cylinder.shaftArea = cylinderData.shaftDiameter * cylinderData.shaftDiameter * 3.1416 / 4
  cylinder.cylinderContractArea = cylinder.cylinderExtendArea - cylinder.shaftArea

  local currentExtend = 0
  for _, bt in pairs(cylinderData.beamTags) do
    if beamstate.tagBeamMap[bt] then
      for _, cid in pairs(beamstate.tagBeamMap[bt]) do
        table.insert(cylinder.beamCids, cid)
        cylinder.beamVelocitySmoothers[cid] = newExponentialSmoothing(500)
        cylinder.previousBeamVelocities[cid] = 0
        cylinder.beamCount = cylinder.beamCount + 1
        currentExtend = currentExtend + obj:getBeamRestLength(cid)
      end
    end
  end
  cylinder.invBeamCount = 1 / cylinder.beamCount
  cylinder.currentExtend = cylinder.currentExtend * cylinder.invBeamCount
  cylinder.currentExtendPercent = linearScale(cylinder.currentExtend, cylinder.minExtend, cylinder.maxExtend, 0, 1)

  return cylinder
end

M.new = new

return M
