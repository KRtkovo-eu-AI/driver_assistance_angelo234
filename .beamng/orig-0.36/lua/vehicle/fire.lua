-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local flammableNodes = {}
local hotNodes = {}
local wheelNodes = {}
local fireNodeCounter = 0
local currentNodeKey = nil
local centreNode = 0

local tEnv = 0
local tSteam = 100
local maxNodeTemp = 1950
local tirePopTemp = 650
local waterCoolingCoef = 6000
local collisionEnergyMultiplier = 1
local vaporFlashPointModifier = 0.5
local vaporCondenseTime = 10 --seconds
local vaporCondenseCoef = 100 / vaporCondenseTime

local fireballSoundDelay = 2
local fireballSoundTimer = 0
local fireBurnSoundObj = nil
local fireBurnSoundPlaying = true

local fireParticleSmall = 25
local fireParticleMedium = 27
local fireParticleLarge = 29

local smokeParticleSmall = 51
local smokeParticleMedium = 52

--shorter functions for increased performance
local random = math.random
local sqrt = math.sqrt
local min, max = math.min, math.max

M.debugData = {}

local function getClosestHotNodeTempDistance(cid)
  if next(hotNodes) == nil then
    return 0, math.huge
  end

  local minSquareDistance = math.huge
  local minDistanceTemperature = 0
  for k, v in pairs(hotNodes) do
    local squareDistance = obj:nodeSquaredLength(k, cid)
    if squareDistance <= minSquareDistance then
      minSquareDistance = squareDistance
      minDistanceTemperature = v.temperature
    end
  end

  return minDistanceTemperature, sqrt(minSquareDistance)
end

local function getCircular(tab, key)
  local k, v = next(tab, key)
  if k == nil then
    return next(tab, nil)
  end
  return k, v
end

local function updateGFX(dt)
  local rand = random(1) --use the same random value for all effects in this frame

  --we combine these two values only once per frame here, we need the result of this quite often below
  local countTime = dt * fireNodeCounter
  local airSpeed = electrics.values.airflowspeed
  tEnv = obj:getEnvTemperature() - 273.15

  -- Node Iteration --
  local currentNode

  currentNodeKey, currentNode = getCircular(flammableNodes, currentNodeKey)
  local mycid = currentNodeKey
  local invSpecHeat = 1 / currentNode.weightSpecHeatCoef
  local countTimeInvSpecHeat = countTime * invSpecHeat
  local curTemperature = currentNode.temperature

  -- Steam Handling --
  local underWater = obj:inWater(mycid) and 1 or 0
  currentNode.lastUnderWaterValue = underWater

  --dynamic baseTemp handling, either use the exhaust manifold temp from the drivetrain or the static base temp from jbeam
  currentNode.baseTemp = currentNode.useThermalsBaseTemp and controller.mainController.fireEngineTemperature or currentNode.staticBaseTemp

  --emit steam if indicated
  if curTemperature > tSteam and underWater == 1 then
    obj:addParticleByNodesRelative(currentNodeKey, centreNode, rand * -2, 24, 0, 1)
  end

  -- Vapor Handling --
  if currentNode.containerBeam then
    local containerBeamBroken = obj:beamIsBroken(currentNode.containerBeam)
    if not currentNode.containerBeamBroken and containerBeamBroken then
      currentNode.vaporState = 20 + random(80)
      obj:addParticleByNodesRelative(mycid, centreNode, 1, 55, 0.1, 3) -- add fuel splash particles
      obj:addParticleByNodesRelative(mycid, centreNode, 1, 50, 0.1, 3)
      currentNode.containerBeamBroken = true
      currentNode.canIgnite = true
      if not currentNode.ignoreContainerBeamBreakMessage then
        guihooks.message("vehicle.fire.fuelTankRuptured", 10, "vehicle.damage.fueltank")
      end
    end
    currentNode.vaporState = max(currentNode.vaporState - (vaporCondenseCoef * dt), 0) -- condense fuel again
    currentNode.isVapor = currentNode.vaporState >= currentNode.vaporPoint and 1 or 0
  end

  fireballSoundTimer = fireballSoundTimer >= fireballSoundDelay and 0 or min(fireballSoundTimer + dt, fireballSoundDelay)

  --reduce smokePoint by X% if the node is vaporized
  local vaporCorrectedSmokePoint = currentNode.smokePoint - (currentNode.smokePoint * currentNode.isVapor * vaporFlashPointModifier)

  if fireBurnSoundObj then
    if next(hotNodes) == nil then
      if fireBurnSoundPlaying then
        obj:stopSFX(fireBurnSoundObj)
        fireBurnSoundPlaying = false
      end
    else
      if not fireBurnSoundPlaying then
        obj:playSFX(fireBurnSoundObj)
        fireBurnSoundPlaying = true
      end
    end
  end

  -- Particles --
  for hotcid, node in pairs(hotNodes) do
    if hotcid ~= currentNodeKey then
      -- Heat Transfer --
      --radiate, conduct heat from hotNodes node to current nodeframe
      local dist = obj:nodeLength(currentNodeKey, hotcid) --distance to nearby nodes, for heat radiation
      -- conduction
      if dist < node.conductionRadius then
        curTemperature = curTemperature + (node.temperature - curTemperature) * min(0.5, 0.2 * countTimeInvSpecHeat / (dist + 1e-30))
      end
      local burningCoef = node.temperature > vaporCorrectedSmokePoint and 1 or 0 --1 when actually burning, 0 otherwise
      local radiation = (24 * node.intensity * burningCoef) / (1 + dist * dist * dist) --radiation of heat depends on flame intensity, base heat, and distance to surrounding nodes, factor is arbitary, can be adjusted to change radiation speed
      curTemperature = curTemperature + radiation * countTimeInvSpecHeat --radiate heat to current node; multiply it by the delta T (hotNum)
    end

    node.flameTick = node.flameTick >= 1 and 0 or node.flameTick + 10 * (1 + airSpeed * 0.05) * dt
    node.smokeTick = node.smokeTick >= 1 and 0 or node.smokeTick + 1.2 * (1 + airSpeed * 0.05) * dt

    local vaporCorrectedNodeSmokePoint = node.smokePoint - (node.smokePoint * node.isVapor * vaporFlashPointModifier)
    local vaporCorrectedNodeFlashPoint = node.flashPoint - (node.flashPoint * node.isVapor * vaporFlashPointModifier)

    if node.intensity > 0 and node.temperature >= vaporCorrectedNodeFlashPoint and currentNode.lastUnderWaterValue == 0 then
      if fireBurnSoundObj == nil  then
        fireBurnSoundObj = obj:createSFXSource("event:>Vehicle>Fire>Fire_Burn_Loop", "AudioDefaultLoop3D", "fireburn", -1) or false
        fireBurnSoundPlaying = true
      end
      local x = min(node.intensity, 1)
      obj:setNodeVolumePitchCT(fireBurnSoundObj, hotcid, x * (0.5*x + 0.5), 1, 0, 0)

      if node.flameTick >= 1 then
        local rootedIntensity = sqrt(node.intensity)

        --small flames for low intensity fire
        fireParticleSmall = airSpeed < 10 and 25 or 26
        obj:addParticleByNodesRelative(hotcid, centreNode, rand * -2 * rootedIntensity, fireParticleSmall, 0, 1)

        if node.intensity > 0.15 then
          --medium flames for medium intensity fire
          fireParticleMedium = airSpeed < 10 and 27 or 28
          obj:addParticleByNodesRelative(hotcid, centreNode, rand * -2 * rootedIntensity, fireParticleMedium, 0, 1)

          if node.intensity > 0.3 then
            --large flames for high-intensity fire
            fireParticleLarge = airSpeed < 10 and 29 or 30
            obj:addParticleByNodesRelative(hotcid, centreNode, rand * -2 * rootedIntensity, fireParticleLarge, 0, 1)

            if node.intensity > 10 then
              node.vaporState = 0
              --huge fireball for explosions
              if fireballSoundTimer >= fireballSoundDelay then
                sounds.playSoundOnceFollowNode("event:>Vehicle>Fire>Fire_Ignition", mycid, 3)
              end

              obj:addParticleByNodesRelative(hotcid, centreNode, 0, 31, 0.5, 10)
              --huge smoke puff for explosions
              obj:addParticleByNodesRelative(hotcid, centreNode, 0, 32, 0, 1)
              --spray of sparks
              obj:addParticleByNodesRelative(hotcid, centreNode, 0, 9, 0.5, 100)
            end
          end
        end
      end
    end

    if node.smokeTick >= 1 then
      if node.smokePoint and node.temperature > vaporCorrectedNodeSmokePoint then
        local rootedIntensity = sqrt(node.intensity)
        --node emits smoke if close to flash point
        smokeParticleSmall = airSpeed < 10 and 51 or 43
        obj:addParticleByNodesRelative(hotcid, centreNode, rand * -2 * rootedIntensity * (1 + airSpeed * 0.1), smokeParticleSmall, 0, 1)

        if node.temperature > node.flashPoint * 4 then
          smokeParticleMedium = airSpeed < 10 and 52 or 53
          obj:addParticleByNodesRelative(hotcid, centreNode, rand * -2 * rootedIntensity * (1 + airSpeed * 0.1), smokeParticleMedium, 0, 1)
        end
      end
    end
  end

  -- Cooling Down ---
  --coefficient of heat transfer, based on airspeed
  local hc = 0.0006 * (waterCoolingCoef * underWater + 1) * (25 + 0.2 * airSpeed)
  --if the engine is dead, our nodes can cool below their baseTemp (baseTemp represents the engine's constant heat)
  local minTemp = (drivetrain.engineDisabled or underWater == 1) and tEnv or max(tEnv, currentNode.baseTemp)
  --heat is lost to the surroundings at a rate of temperature * hc. Lower limit = 0
  --temperature is the node's heat divided by its mass * specific heat
  curTemperature = min(curTemperature + (minTemp - curTemperature) * min(1, hc * countTimeInvSpecHeat), maxNodeTemp)

  -- Fire / Smoke --
  if currentNode.canIgnite and curTemperature >= vaporCorrectedSmokePoint then
    if currentNode.chemEnergy > 0 and underWater == 0 then
      local burnRate = currentNode.burnRate + currentNode.burnRate * currentNode.isVapor * 10
      local chemEnergyRatio = currentNode.chemEnergy / currentNode.originalChemEnergy

      currentNode.intensity = min(2 * chemEnergyRatio, 1) * (curTemperature / maxNodeTemp) * burnRate
      curTemperature = curTemperature + currentNode.intensity * 100 * countTimeInvSpecHeat
      currentNode.chemEnergy = max(currentNode.chemEnergy - currentNode.intensity * 3 * countTime, 0)

      if currentNode.chemEnergy / currentNode.originalChemEnergy < 0.01 then
        currentNode.chemEnergy = 0
      end
    else
      currentNode.intensity = 0
    end

    hotNodes[mycid] = currentNode --add it to the hot node list

    if wheelNodes[mycid] and curTemperature > tirePopTemp then --tire popping
      local wheelData = wheelNodes[mycid]
      beamstate.deflateTire(wheelData.wheelID, 1)
      sounds.playSoundOnceFollowNode("event:>Vehicle>Fire>Fire_Ignition", mycid, 2)
      --puff of flame on tire burst
      obj:addParticleByNodesRelative(mycid, centreNode, 0, 31, 0.5, 20)

      obj:addParticleByNodesRelative(mycid, centreNode, 0, 29, 0.5, 20)

      --obj:addParticleByNodesRelative(mycid, centreNode, 0, 9, 0.5, 100)

      -- we only want to deflate the tires once, so we just pretend this wheel node is not actually a wheel node anymore
      wheelNodes[wheelData.node1] = nil
      wheelNodes[wheelData.node2] = nil
    end
  else
    --we are below flashpoint, which means that this node is not hot (anymore)
    hotNodes[mycid] = nil --remove hotnode from list
    currentNode.intensity = 0 --kill any flames that might still exist
  end

  currentNode.temperature = curTemperature
end

local function nodeCollision(p)
  --add energy to node
  local collisionNodeId = p.id1
  local node = flammableNodes[collisionNodeId]

  if not node or not node.temperature or p.depth > 0 then
    return
  end

  local collisionEnergy = p.slipForce * p.slipVel * lastDt
  node.temperature = min(maxNodeTemp, node.temperature + collisionEnergy * collisionEnergyMultiplier * node.selfIgnitionCoef / node.weightSpecHeatCoef)

  local vaporCorrectedSmokePoint = node.smokePoint - (node.smokePoint * node.isVapor * vaporFlashPointModifier)
  if node.canIgnite and node.temperature >= vaporCorrectedSmokePoint and not hotNodes[collisionNodeId] then
    hotNodes[collisionNodeId] = node --add it to the hot node list

    node.burnRate = node.burnRate or 0
    --same as main intensity calculation, just simplified for the ignition case
    node.intensity = node.burnRate * node.temperature / maxNodeTemp
    node.flameTick = 1
    node.smokeTick = 1
  end
end

local function init()
  if fireBurnSoundObj then
    obj:stopSFX(fireBurnSoundObj)
    fireBurnSoundPlaying = false
  end
  M.updateGFX = nop

  table.clear(flammableNodes)
  M.debugData.flammableNodes = flammableNodes
  table.clear(hotNodes)
  wheelNodes = {}
  currentNodeKey = nil
  centreNode = 0
  fireNodeCounter = 0
  fireballSoundTimer = 0
  tEnv = obj:getEnvTemperature() - 273.15

  local containerBeamCache = {}

  --create a cache of all available container beams for easy access
  if v.data.beams then
    for k, b in pairs(v.data.beams) do
      if b.containerBeam then
        containerBeamCache[b.containerBeam] = k
      end
    end
  end

  if v.data.nodes then
    local centreNodeDist = 100
    for _, node in pairs(v.data.nodes) do
      local nodeDist = sqrt((node.pos.x * node.pos.x) + (node.pos.y * node.pos.y) + (node.pos.z * node.pos.z))
      if nodeDist < centreNodeDist then --find the centre-most node and store it for particle reference
        centreNodeDist = nodeDist
        centreNode = node.cid
      end

      if node.flashPoint then
        --we can assume this node is part of the fire system
        local staticBaseTemp = (type(node.baseTemp) == "number") and node.baseTemp or tEnv
        flammableNodes[node.cid] = {
          name = node.name,
          flashPoint = node.flashPoint,
          smokePoint = node.smokePoint or node.flashPoint,
          burnRate = node.burnRate,
          staticBaseTemp = staticBaseTemp,
          useThermalsBaseTemp = node.baseTemp == "thermals",
          conductionRadius = node.conductionRadius or 0,
          selfIgnitionCoef = node.selfIgnitionCoef or 0,
          temperature = tEnv,
          intensity = 0,
          chemEnergy = node.chemEnergy or 0,
          originalChemEnergy = node.chemEnergy or 0,
          weightSpecHeatCoef = node.nodeWeight * (node.specHeat or 1),
          lastUnderWaterValue = 0,
          flameTick = 0,
          smokeTick = 0,
          vaporState = 0,
          vaporPoint = node.vaporPoint,
          isVapor = 0,
          containerBeam = containerBeamCache[node.containerBeam],
          ignoreContainerBeamBreakMessage = node.ignoreContainerBeamBreakMessage or false,
          containerBeamBroken = false,
          canIgnite = containerBeamCache[node.containerBeam] == nil
        }

        fireNodeCounter = fireNodeCounter + 1
      end
    end
  end

  --cache wheelnodes for easy access from update
  if wheels.wheels then
    for id, wd in pairs(wheels.wheels) do
      wheelNodes[wd.node1] = {wheelID = id, node1 = wd.node1, node2 = wd.node2}
      wheelNodes[wd.node2] = {wheelID = id, node1 = wd.node1, node2 = wd.node2}
    end
  end

  --activate fire sim if configured nodes are found
  if fireNodeCounter > 0 then
    M.updateGFX = updateGFX
    M.nodeCollision = nodeCollision
  end
end

local function igniteNode(cid, temp)
  local node = flammableNodes[cid]
  if not node then
    return
  end
  node.temperature = temp or maxNodeTemp
end

local function igniteRandomNode()
  local possibleNodes = {}
  for k, n in pairs(flammableNodes) do
    if n and n.canIgnite and not wheelNodes[k] and n.intensity <= 0 then
      table.insert(possibleNodes, k)
    end
  end

  if #possibleNodes <= 0 then
    return
  end

  igniteNode(possibleNodes[random(#possibleNodes)])
end

local function igniteRandomNodeMinimal()
  local possibleNodes = {}
  for k, n in pairs(flammableNodes) do
    if n and n.canIgnite and not wheelNodes[k] and n.intensity <= 0 then
      table.insert(possibleNodes, k)
    end
  end

  if #possibleNodes <= 0 then
    return
  end

  local node = flammableNodes[possibleNodes[random(#possibleNodes)]]

  if not node then
    return
  end

  node.temperature = node.flashPoint + 10
end

local function igniteVehicle()
  for cid, _ in pairs(flammableNodes) do
    if not wheelNodes[cid] then --don't ignite wheelnodes right away to delay the tire popping a bit
      igniteNode(cid)
    end
  end
end

local function explodeVehicle()
  for cid, node in pairs(flammableNodes) do
    if node.containerBeam then
      node.vaporState = 100
      node.containerBeamBroken = true
      node.canIgnite = true
      node.isVapor = 1
    end

    igniteNode(cid)
  end
end

local function explodeNode(cid)
  local node = flammableNodes[cid]
  if not node then
    return
  end
  node.temperature = maxNodeTemp
  node.vaporState = 100
  node.containerBeamBroken = true
  node.canIgnite = true
  node.isVapor = 1
  hotNodes[cid] = node
end

local function extinguishVehicle()
  for cid, node in pairs(flammableNodes) do
    node.temperature = tEnv
    node.intensity = 0
    obj:addParticleByNodes(cid, centreNode, -1, 48, 0, 15)
  end
end

local function extinguishVehicleSlowly()
  for _, node in pairs(flammableNodes) do
    node.chemEnergy = 0
  end
end

-- public interface
M.igniteNode = igniteNode
M.igniteRandomNodeMinimal = igniteRandomNodeMinimal
M.igniteRandomNode = igniteRandomNode
M.igniteVehicle = igniteVehicle
M.explodeVehicle = explodeVehicle
M.explodeNode = explodeNode
M.extinguishVehicle = extinguishVehicle
M.extinguishVehicleSlowly = extinguishVehicleSlowly
M.getClosestHotNodeTempDistance = getClosestHotNodeTempDistance
M.hotNodes = hotNodes
M.flammableNodes = flammableNodes
M.reset = init
M.init = init

--by default, fire sim is not active on an object
M.updateGFX = nop
M.nodeCollision = nop

return M
