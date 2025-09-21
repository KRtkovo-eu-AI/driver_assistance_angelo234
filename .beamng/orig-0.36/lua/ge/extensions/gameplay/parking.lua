-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'core_vehicleActivePooling'}

local logTag = "parking"

local checkRadius = 60 -- radius comparison to last parking spot check
local innerRadius = 40 -- radius comparison to player vehicle
local checkDist = 100 -- distance ahead of focus point to check if new parking spots are needed
local viewDist = 120 -- minimum distance needed for a queued parked car to appear
local freeCamViewDist = 160 -- minimum distance needed for a queued parked car to appear, in free camera mode
local lookDist = 200 -- distance ahead of focus point to query for parking spots
local areaRadius = 200 -- radius to search within for parking spots
local parkedVehIds, parkedVehData = {}, {}
local trackedVehData = {} -- parking tracking, can be used with the player vehicle
local currParkingSpots = {} -- cached parking spots, updated periodically
local queuedIndex = 1

-- common functions --
local min = math.min
local max = math.max
local random = math.random

--------
local sites, vehPool, vars
local aheadPos, lastPos, debugPos, tempVec = vec3(), vec3(), vec3(), vec3()
local focus
local active = false
local worldLoaded = false
local parkingSpotsAmount = 0
local respawnDelay = 0

M.debugLevel = 0

local function loadSites() -- loads sites data containing parking spots
  -- by default, the file "city.sites.json" in the root folder of the current level will be used
  if not gameplay_city then return end
  gameplay_city.loadSites()
  sites = gameplay_city.getSites()
  parkingSpotsAmount = sites and #sites.parkingSpots.sorted or 0
end

local function setSites(data) -- sets sites data, can override the default sites data
  if type(data) == "string" then
    if FS:fileExists(data) then
      sites = gameplay_sites_sitesManager.loadSites(data)
    end
  elseif type(data) == "table" and data.parkingSpots then -- assuming that given data is valid sites data
    sites = data
  else
    sites = nil
  end
  parkingSpotsAmount = sites and #sites.parkingSpots.sorted or 0
end

local function setState(val) -- activates or deactivates the parking system
  active = val and true or false
  if active then
    if not sites then
      loadSites()
    end
    focus = gameplay_traffic.getFocus()
    aheadPos:set(focus.pos)
    lastPos:set(aheadPos)
  end
end

local function getState()
  return active
end

local function getParkingSpots() -- returns a table of all current parking spots
  if not sites then
    loadSites()
  end
  return sites and sites.parkingSpots
end

local function moveToParkingSpot(vehId, parkingSpot, lowPrecision) -- assigns a parked vehicle to a parking spot
  local obj = getObjectByID(vehId)
  local width, length = obj.initialNodePosBB:getExtents().x - 0.1, obj.initialNodePosBB:getExtents().y
  local backwards, offsetPos, offsetRot

  if parkingSpot.customFields.tags.forwards then
    backwards = false
  elseif parkingSpot.customFields.tags.backwards then
    backwards = true
  else
    backwards = random() > 0.75 + vars.neatness * 0.25 -- backwards direction is less common by default
  end

  if not parkingSpot.customFields.tags.perfect then -- randomize position and rotation slightly
    local offsetVal = 1 - square(vars.neatness)
    local xGap, yGap = max(0, parkingSpot.scl.x - width), max(0, parkingSpot.scl.y - length)
    local xRandom, yRandom = randomGauss3() / 3 - 0.5, clamp(randomGauss3() / 3 - (backwards and 0.75 or 0.25), -0.5, 0.5)
    offsetPos = vec3(xRandom * offsetVal * xGap, yRandom * offsetVal * yGap, 0)
    offsetRot = quatFromEuler(0, 0, (randomGauss3() / 3 - 0.5) * offsetVal * 0.25)
  end

  local options = {
    skipVehicleIntersectionCheck = true
  }
  parkingSpot:moveResetVehicleTo(vehId, lowPrecision, backwards, offsetPos, offsetRot, true, false, nil, options)
  if M.debugLevel > 0 then
    log("I", logTag, "Teleported vehId "..vehId.." to parking spot "..parkingSpot.id)
  end

  getObjectByID(vehId):queueLuaCommand("electrics.setIgnitionLevel(0)")

  if parkedVehData[vehId] then
    if parkedVehData[vehId].parkingSpotId then
      sites.parkingSpots.objects[parkedVehData[vehId].parkingSpotId].vehicle = nil
    end

    if parkedVehData[vehId].randomPaint then
      core_vehicle_manager.setVehiclePaintsNames(vehId, {getRandomPaint(vehId, 0.75)})
    end

    -- the following code is somewhat hacky
    if parkingSpot.customFields.tags.street then -- enables tracking, so that AI can try to avoid this vehicle
      if not map.objects[vehId] then getObjectByID(vehId):queueLuaCommand("mapmgr.enableTracking()") end
    else -- disables tracking, to optimize performance
      getObjectByID(vehId):queueLuaCommand("mapmgr.disableTracking()")
    end

    parkingSpot.vehicle = vehId -- parking spot contains this vehicle
    parkedVehData[vehId].parkingSpotId = parkingSpot.id -- vehicle is assigned to this parking spot
    parkedVehData[vehId].activeRadius = 50
    parkedVehData[vehId]._teleport = nil
  end
end

local defaultParkingSpotSize = vec3(2.5, 6, 3)
local function checkDimensions(vehId) -- checks if the vehicle would fit in a standard sized parking spot
  local obj = getObjectByID(vehId)
  if not obj then return false end

  local extents = obj.initialNodePosBB:getExtents()
  return  extents.x <= defaultParkingSpotSize.x and
          extents.y <= defaultParkingSpotSize.y and
          extents.z <= defaultParkingSpotSize.z
end

local function checkParkingSpot(vehId, parkingSpot) -- checks if a parking spot is ready to use for a parked vehicle
  local obj = getObjectByID(vehId or 0)

  if parkingSpot.vehicle or parkingSpot.ignoreOthers or parkingSpot.customFields.tags.ignoreOthers or parkingSpot:hasAnyVehicles() or not obj then
    return false
  end

  if parkingSpot:vehicleFits(vehId) then
    -- ensure that the parking spot is not too oversized for the vehicle
    local size = obj.initialNodePosBB:getExtents()
    local psSize = parkingSpot.scl
    if size.x / psSize.x < 0.5 or size.y / psSize.y < 0.5 then
      return false
    end
  else
    return false
  end

  return true
end

local function findParkingSpots(pos, minRadius, maxRadius) -- finds and returns a sorted array of parking spot objects and distances
  if not sites then return {} end
  pos = pos or core_camera.getPosition()
  minRadius = minRadius or 0
  maxRadius = maxRadius or areaRadius

  local psList = sites:getRadialParkingSpots(pos, minRadius, maxRadius)
  -- each entry in psList contains: v.ps (parking spot object), v.squaredDistance (squared distance to parking spot)

  if M.debugLevel > 0 then
    log("I", logTag, "Found and validated "..#psList.." parking spots in area")
  end
  table.sort(psList, function(a, b) return a.squaredDistance < b.squaredDistance end) -- sorts from closest to farthest

  return psList
end

local function updateParkingSpots(psList, pos) -- updates the distances of the parking spots in the cached list
  if not psList or type(psList[1]) ~= "table" then return psList end
  for i, v in ipairs(psList) do
    psList[i].squaredDistance = pos:squaredDistance(v.ps.pos)
  end

  table.sort(psList, function(a, b) return a.squaredDistance < b.squaredDistance end) -- sorts from closest to farthest
  return psList
end

local emptyFilters = {}
local defaultFilters = {useProbability = true}
local function filterParkingSpots(psList, filters) -- filter the sorted list of parking spots (as returned by findParkingSpots)
  if not psList or type(psList[1]) ~= "table" then return psList end
  filters = filters or defaultFilters

  local psCount = #psList
  local timeDay = 0

  if filters.useProbability then
    local timeObj = core_environment.getTimeOfDay()
    if timeObj and timeObj.time then
      timeDay = timeObj.time
    end
  end

  for i = psCount, 1, -1 do
    local ps = psList[i].ps
    local remove = false

    if filters.checkVehicles then -- strict but slow check for other vehicles occupying this spot
      if ps:hasAnyVehicles() then
        remove = true
      end
    end

    if filters.useProbability then
      local prob = ps.customFields:has("probability") and ps.customFields:get("probability") or vars.baseProbability
      if type(prob) ~= "number" then prob = 1 end
      prob = prob * vars.baseProbability

      local dayValue = 0.25 + math.abs(timeDay - 0.5) * 1.5 -- max 1 for midday, min 0.25 for midnight
      local timeDayCoef = dayValue

      if ps.customFields.tags.nightTime then
        local nightValue = 1 - math.abs(timeDay - 0.5) * 1.5 -- opposite of dayValue
        if ps.customFields.tags.dayTime then
          timeDayCoef = max(timeDayCoef, nightValue)
        else
          timeDayCoef = nightValue
        end
      end
      prob = prob * timeDayCoef

      if prob <= random() then
        remove = true
      end
    end

    if remove then
      table.remove(psList, i)
    end
  end

  if M.debugLevel > 0 then
    log("I", logTag, "Filtered and accepted "..#psList.." / "..psCount.." parking spots")
  end

  return psList
end

local function forceTeleport(vehId, psList, minDist, maxDist) -- forces a parked car to teleport to a new parking spot
  if not parkedVehData[vehId] or parkedVehData[vehId].ignoreForceTeleport then return end  -- ignoreForceTeleport is a special flag for vehicles that should not be force teleported

  minDist = minDist or 0
  maxDist = maxDist or 10000
  psList = psList or findParkingSpots(focus and focus.pos, minDist, maxDist)

  for _, psData in ipairs(psList) do
    local ps = psData.ps
    if psData.squaredDistance >= square(minDist) and psData.squaredDistance <= square(maxDist) and checkParkingSpot(vehId, ps) then
      if parkedVehData[vehId].parkingSpotId then
        sites.parkingSpots.objects[parkedVehData[vehId].parkingSpotId].vehicle = nil
        parkedVehData[vehId].parkingSpotId = nil
      end

      moveToParkingSpot(vehId, ps, not getObjectByID(vehId):isReady())
      break
    end
  end
end

local function getRandomParkingSpots(originPos, minDist, maxDist, minCount, filters) -- returns a list of random parking spots, with a bias for origin position
  if not sites then return {} end
  minDist = minDist or 0
  maxDist = maxDist or 10000
  local radius = max(minDist, 100)
  local psList, psCount
  if not minCount or minCount <= 0 then
    minCount = math.huge
    radius = maxDist
  end

  repeat -- find enough parking spots in the area
    psList = findParkingSpots(originPos or core_camera.getPosition(), minDist, radius)
    if psList[1] then
      lastPos:set(psList[1].ps.pos)
    end

    psList = filterParkingSpots(psList, filters)
    psCount = #psList
    radius = radius * 2
  until psCount >= minCount or radius >= maxDist

  if minCount == math.huge then
    minCount = max(1, math.ceil(psCount / 4)) -- auto minimum count
  end
  if psCount < minCount then return {} end

  local finalPsList = {}
  local selected = {}
  local ratio = min(0.95, 1 - (minCount / psCount)) -- ratio of selectable parking spots
  local fallbackValue = 1

  repeat -- randomize which parking spots are selected, with a bias towards nearer parking spots
    for i, ps in ipairs(psList) do
      local minValue = min(fallbackValue, lerp(ratio, 1, square(i / psCount))) -- minValue is lower for nearer parking spots
      if not selected[ps.ps.name] and random() >= minValue then
        selected[ps.ps.name] = 1
        table.insert(finalPsList, ps)
      end
      if finalPsList[minCount] then break end
    end

    fallbackValue = fallbackValue / 2
  until finalPsList[minCount] -- repeat until the minimum number of parking spots is reached

  return finalPsList, psList
end

local function scatterParkedCars(vehIds, minDist, maxDist) -- randomly teleports all parked vehicles to parking spots
  vehIds = vehIds or parkedVehIds
  local randomPsList, psList = getRandomParkingSpots(focus and focus.pos, minDist, maxDist, #vehIds)
  if not psList then return end

  randomPsList = arrayConcat(psList, randomPsList)

  for _, id in ipairs(vehIds) do
    forceTeleport(id, randomPsList)
  end
end

local function enableTracking(vehId, autoDisable) -- enables parking spot tracking for a driving vehicle
  vehId = vehId or be:getPlayerVehicleID(0)
  if not getObjectByID(vehId) then return end

  setState(true)

  trackedVehData[vehId] = {
    isOversized = checkDimensions(vehId),
    autoDisableTracking = autoDisable and true or false,
    inside = false,
    preParked = false,
    parked = false,
    event = "none",
    aheadPos = vec3(),
    focusPos = vec3(),
    maxDist = 80,
    parkingTimer = 0
  }
end

local function disableTracking(vehId) -- disables parking spot tracking for a driving vehicle
  vehId = vehId or be:getPlayerVehicleID(0)
  trackedVehData[vehId] = nil
end

local function getTrackingData()
  return trackedVehData
end

local function getCurrentParkingSpot(vehId) -- returns the parking spot id of a properly parked vehicle (no tracked data needed)
  vehId = vehId or be:getPlayerVehicleID(0)
  if not getObjectByID(vehId) then return end

  if trackedVehData[vehId] then
    if trackedVehData[vehId].preParked then
      return trackedVehData[vehId].parkingSpotId -- existing tracked data
    end
  else
    local obj = getObjectByID(vehId)
    if not obj then return end

    local psList = findParkingSpots(obj:getPosition(), 0, 15)
    if psList[1] and psList[1].ps:vehicleFits(vehId) and psList[1].ps:checkParking(vehId, vars.precision) then
      return psList[1].ps.id
    end
  end
end

local function resetParkingVars() -- resets parking variables to default
  vars = {
    precision = 0.8, -- parking precision required for valid parking
    neatness = 0, -- generated parked vehicle precision
    parkingDelay = 0.5, -- time delay until a vehicle is considered parked
    baseProbability = 0.75, -- probability coefficient for spawning in parking spots (usually from 0 to 1; default 0.75 for realistic looking scattering in parking lots)
    activeAmount = math.huge
  }
end
resetParkingVars()

local function setParkingVars(data, reset) -- sets parking related variables
  if reset then resetParkingVars() end
  if type(data) ~= "table" then return end

  vars = tableMerge(vars, data)

  if vars.activeAmount and vehPool then
    vehPool:setMaxActiveAmount(vars.activeAmount)
    vehPool:setAllVehs(true)
  end
end

local function setActiveAmount(amount) -- sets the maximum amount of active (visible) vehicles
  amount = amount or math.huge
  setParkingVars({activeAmount = amount})
end

local function getParkingVars() -- gets parking related variables
  return vars
end

local bbCenter, vehDirection, bbHalfExtents = vec3(), vec3(), vec3()
local result = {}
local function trackParking(vehId) -- tracks parking status of a driving vehicle
  local valid = false
  table.clear(result)
  result.cornerCount = 0
  local obj = getObjectByID(vehId or 0)
  if not obj then return valid, result end

  local vehData = trackedVehData[vehId]
  bbCenter:set(be:getObjectOOBBCenterXYZ(vehId))
  vehDirection:set(obj:getDirectionVectorXYZ())
  bbHalfExtents:set(be:getObjectOOBBHalfExtentsXYZ(vehId))

  vehDirection:setScaled(bbHalfExtents.y)
  vehData.aheadPos:setAdd2(bbCenter, vehDirection) -- tracks the position ahead of the vehicle

  local maxDist = M.debugLevel >= 3 and 400 or vehData.maxDist
  if vehData.focusPos:squaredDistance(vehData.aheadPos) >= square(maxDist * 0.5) then -- focus pos and nearby parking spots low frequency update
    vehData.psList = findParkingSpots(vehData.aheadPos, 0, maxDist)
    vehData.psList = filterParkingSpots(vehData.psList, emptyFilters)
    vehData.focusPos:set(vehData.aheadPos)
  end

  vehData.psList = updateParkingSpots(vehData.psList, vehData.aheadPos) or {}

  if M.debugLevel > 0 then
    for _, v in ipairs(vehData.psList) do
      local ps = v.ps
      local psDirVec = vec3(0, 1, 0):rotated(ps.rot)
      local dColor = ps.vehicle and ColorF(1, 0.5, 0.5, 0.2) or ColorF(1, 1, 1, 0.2)
      if ps.vehicle == vehId then dColor = ColorF(0.5, 1, 0.5, 0.2) end
      debugDrawer:drawSquarePrism(ps.pos - psDirVec * ps.scl.y * 0.5, ps.pos + psDirVec * ps.scl.y * 0.5, Point2F(0.6, ps.scl.x), Point2F(0.6, ps.scl.x), dColor)
    end
  end

  local bestPs
  for _, v in ipairs(vehData.psList) do -- nearest parking spot
    if v.ps:vehicleFits(vehId) and (not v.ps.vehicle or v.ps.vehicle == vehId) then
      bestPs = v.ps
      break
    end
  end

  if bestPs then
    result.parkingSpotId = bestPs.id
    result.parkingSpot = bestPs

    if not bestPs.vertices[1] then bestPs:calcVerts() end
    valid, result.corners = bestPs:checkParking(vehId, vars.precision) -- checks if all vehicle corners are inside the parking spot with respect to the precision
    for _, v in ipairs(result.corners) do
      if v then
        result.cornerCount = result.cornerCount + 1
      end
    end

    if M.debugLevel >= 2 then
      for i, v in ipairs(result.corners) do
        local dColor = v and ColorF(0.3, 1, 0.3, 0.5) or ColorF(1, 0.3, 0.3, 0.5)
        debugDrawer:drawCylinder(bestPs.vertices[i], bestPs.vertices[i] + vec3(0, 0, 10), 0.05, dColor)
      end
    end
  end

  return valid, result
end

local function processNextSpawn(vehId, ignorePool) -- processes the next vehicle to respawn
  local oldId, newId = vehId, vehId

  if not ignorePool then
    local inactiveId = vehPool.inactiveVehs[1]
    if inactiveId then
      if #vehPool.activeVehs < vehPool.maxActiveAmount then -- amount of active vehicles is less than the expected limit
        newId = inactiveId
      else
        oldId, newId = vehPool:cycle(oldId, inactiveId) -- cycles the pool
      end
    end
  end

  for _, psData in ipairs(currParkingSpots) do
    local ps = psData.ps
    -- TODO: adjust distance by height if in free camera mode
    -- also, consider adjusting distance in relation to focus direction
    if ps.pos:squaredDistance(focus.pos) > square(commands.isFreeCamera() and freeCamViewDist or viewDist) and checkParkingSpot(newId, ps) then -- prevents respawn if player is too near to parking spot
      vehPool:setVeh(newId, true)
      moveToParkingSpot(newId, ps)
      break
    end
  end
end

local function processVehicles(vehIds, ignoreScatter) -- activates a group of vehicles, to allow them to teleport to new parking spots
  table.clear(parkedVehIds)
  table.clear(parkedVehData)
  if vehPool then
    core_vehicleActivePooling.deletePool(vehPool.id)
    vehPool = nil
  end

  setState(true)
  if not sites or not vehIds then
    setState(false)
    return
  end

  for _, id in ipairs(vehIds) do
    local obj = getObjectByID(id)
    if obj then
      if not vehPool then
        vehPool = core_vehicleActivePooling.createPool()
        vehPool.name = "parkedCars"
      end

      obj.uiState = 0
      obj.playerUsable = false
      obj:setDynDataFieldbyName("ignoreTraffic", 0, "true")
      obj:setDynDataFieldbyName("isParked", 0, "true")
      gameplay_walk.addVehicleToBlacklist(id)

      table.insert(parkedVehIds, id)
      vehPool:insertVeh(id)

      local psId = getCurrentParkingSpot(id)
      if psId then
        sites.parkingSpots.objects[psId].vehicle = id -- saves the vehicle id to this spot
      end

      parkedVehData[id] = {
        parkingSpotId = psId, -- current parking spot id
        activeRadius = 50, -- radius that keeps the vehicle active if player is near
        randomPaint = true -- randomizes paint after respawning
      }
    end
  end

  if not parkedVehIds[1] then
    setState(false)
    return
  end

  if worldLoaded and not ignoreScatter then
    scatterParkedCars(vehIds)
  end

  extensions.hook("onParkingVehiclesActivated", parkedVehIds)
  log("I", logTag, "Processed and teleported "..#parkedVehIds.." parked vehicles")
end

local function deleteVehicles(amount)
  amount = amount or #parkedVehIds
  for i = amount, 1, -1 do
    local id = parkedVehIds[i] or 0
    local obj = getObjectByID(id)
    if obj then
      obj:delete()
      table.remove(parkedVehIds, i)
      parkedVehData[id] = nil
    end
  end
end

local function setupVehicles(amount, options) -- spawns and prepares simple parked vehicles
  options = options or {}
  if not options.ignoreDelete then
    deleteVehicles()
  end

  if not sites then
    loadSites()
  end

  amount = amount or -1
  if amount == -1 then
    amount = settings.getValue("trafficParkedAmount")
    if amount == 0 then -- auto amount
      amount = clamp(gameplay_traffic.getIdealSpawnAmount(nil, true), 4, 16)
    end
  end

  local group
  if type(options.vehGroup) == "table" then
    group = options.vehGroup
  else
    local params = {filters = {}}

    params.allConfigs = true
    params.filters.Type = {propparked = 1}
    params.minPop = 0

    group = core_multiSpawn.createGroup(amount, params)
  end

  if amount <= 0 or not group or not group[1] then
    if amount <= 0 then
      log("I", logTag, "Parked vehicle amount to spawn is zero!")
    else
      log("I", logTag, "Parked vehicle group is empty!")
    end
    return false
  end

  local transforms
  local psList = getRandomParkingSpots(options.pos, nil, nil, amount, {checkVehicles = true})
  if psList[1] then
    if psList[amount] then
      transforms = {}
      for _, ps in ipairs(psList) do
        table.insert(transforms, {pos = ps.ps.pos, rot = ps.ps.rot})
      end
    end
  else
    if not options.ignoreParkingSpots then
      log("I", logTag, "No parking spots found, skipping parked cars...")
      return false
    end
  end

  core_multiSpawn.spawnGroup(group, amount, {name = "autoParking", mode = "roadBehind", gap = 50, customTransforms = transforms, instant = not worldLoaded, ignoreAdjust = not worldLoaded})

  return true
end

local function getParkedCarsList(override)
  if override then
    local list = {}
    for _, v in ipairs(getAllVehicles()) do
      if v.isParked == "true" then
        table.insert(list, v:getId())
      end
    end
    return list
  else
    return parkedVehIds
  end
end

local function getParkedCarsData()
  return parkedVehData
end

local function resetAll() -- resets everything
  active = false
  sites = nil
  parkingSpotsAmount = 0
  table.clear(parkedVehIds)
  table.clear(parkedVehData)
  table.clear(trackedVehData)
  resetParkingVars()
end

local function onVehicleGroupSpawned(vehList, groupId, groupName)
  if groupName == "autoParking" then
    processVehicles(vehList, true)
  end
end

local function onVehicleDestroyed(id)
  if parkedVehData[id] then
    table.remove(parkedVehIds, arrayFindValueIndex(parkedVehIds, id))
    if sites and parkedVehData[id].parkingSpotId then
      sites.parkingSpots.objects[parkedVehData[id].parkingSpotId].vehicle = nil
    end
    parkedVehData[id] = nil
  end
  if trackedVehData[id] then
    disableTracking(id)
  end
end

local function onVehicleActiveChanged(vehId, active)
  if vehPool and parkedVehData[vehId] then
    if not active then
      parkedVehData[vehId]._teleport = true
    else
      if parkedVehData[vehId]._teleport then -- force teleport if flag exists
        for _, otherVeh in ipairs(getAllVehicles()) do
          local otherId = otherVeh:getId()
          if otherVeh:getActive() and not parkedVehData[otherId] then
            local radius = otherVeh:isPlayerControlled() and 100 or 20
            if otherVeh:getPosition():squaredDistance(getObjectByID(vehId):getPosition()) < square(radius) then
              forceTeleport(vehId, nil, 100)
              break
            end
          end
        end
      end
    end
  end
end

local vehPos = vec3()
local function onUpdate(dt, dtSim)
  if not active or not sites or not be:getEnabled() or freeroam_bigMapMode.bigMapActive() then return end

  tempVec:set(focus.dirVec)
  tempVec.z = 0
  tempVec:normalize()
  tempVec:setScaled2(tempVec, checkDist)
  aheadPos:setAdd2(focus.pos, tempVec)
  aheadPos.z = 0

  if not worldLoaded and parkedVehIds[1] and focus.pos.z ~= 0 then -- is this needed?
    --scatterParkedCars()
    worldLoaded = true
  end

  for id, data in pairs(trackedVehData) do
    local valid, pData = trackParking(id)
    data.parkingSpotId = pData.parkingSpotId
    data.parkingSpot = pData.parkingSpot

    if not valid then
      data.parkingTimer = 0
    end

    if pData.cornerCount >= 2 then -- at least two vehicle corners
      data.lastParkingSpotId = data.parkingSpotId
    end

    if not data.inside and pData.cornerCount > 0 then -- entered parking spot bounds
      data.inside = true
      data.event = "enter"
      extensions.hook("onVehicleParkingStatus", id, data)
    elseif data.inside and pData.cornerCount == 0 then -- exited parking spot bounds
      data.inside = false
      data.event = "exit"
      extensions.hook("onVehicleParkingStatus", id, data)
    end

    if data.lastParkingSpotId then
      if not data.parked and valid then
        data.preParked = true
        data.parkingTimer = data.parkingTimer + dtSim
        if data.parkingTimer >= vars.parkingDelay then -- valid parking (after a small delay)
          data.parked = true
          data.event = "valid"
          sites.parkingSpots.objects[data.lastParkingSpotId].vehicle = id
          extensions.hook("onVehicleParkingStatus", id, data)

          if data.autoDisableTracking then
            disableTracking(id)
          end
        end
      elseif data.preParked and not valid then -- invalid parking
        data.preParked = false
        data.parked = false
        data.event = data.inside and "invalid" or "exit"
        sites.parkingSpots.objects[data.lastParkingSpotId].vehicle = nil
        extensions.hook("onVehicleParkingStatus", id, data)
      end
    end
  end

  local parkedVehCount = #parkedVehIds
  if not parkedVehIds[1] or parkedVehCount >= parkingSpotsAmount then return end -- unable to teleport vehicles to new parking spots

  -- only search for parking spots whenever needed (whenever look ahead point is far enough from the last position)
  if vars.baseProbability > 0 and respawnDelay == 0 and aheadPos:squaredDistance(lastPos) >= square(checkRadius) then -- updates parking spots if away from focus position
    local actualLookDist = lookDist

    if commands.isFreeCamera() then
      local height = max(-1e6, be:getSurfaceHeightBelow(focus.pos))
      height = focus.pos.z - height
      height = clamp(square(height) / 15, 0, 200)
      actualLookDist = actualLookDist + height
    end

    tempVec:set(focus.dirVec)
    tempVec.z = 0
    tempVec:normalize()
    tempVec:setScaled2(tempVec, actualLookDist)
    aheadPos:setAdd2(focus.pos, tempVec) -- set the center point of the parking spot search
    aheadPos.z = 0

    currParkingSpots = findParkingSpots(aheadPos, 0, areaRadius)
    currParkingSpots = filterParkingSpots(currParkingSpots)

    if M.debugLevel >= 3 then
      debugPos:set(aheadPos)
    end

    tempVec:resize(checkDist)

    aheadPos:setAdd2(focus.pos, tempVec)
    aheadPos.z = 0
    lastPos:set(aheadPos)

    for _, id in ipairs(parkedVehIds) do
      parkedVehData[id].searchFlag = false -- reset search flag for all parked cars
    end

    respawnDelay = respawnDelay + 0.25
  end

  if M.debugLevel >= 3 then
    local vecUpHigh = vec3(0, 0, 1000)
    debugDrawer:drawCylinder(aheadPos, aheadPos + vecUpHigh, 0.25, ColorF(1, 1, 0, 0.5))
    debugDrawer:drawCylinder(lastPos, lastPos + vecUpHigh, 0.25, ColorF(0, 1, 0, 0.5))
    if core_terrain.getTerrain() then
      lastPos.z = core_terrain.getTerrainHeight(lastPos)
      debugDrawer:drawCylinder(lastPos, lastPos + vec3(0, 0, 1), checkRadius, ColorF(0, 1, 0, 0.1))
      lastPos.z = 0

      debugPos.z = core_terrain.getTerrainHeight(debugPos)
      debugDrawer:drawCylinder(debugPos, debugPos + vec3(0, 0, 1), areaRadius, ColorF(0, 1, 1, 0.1))
      debugPos.z = 0
    end
  end

  -- cycle through array of parked vehicles one at a time, to save on performance
  local currId = parkedVehIds[queuedIndex] or 0
  local currVeh = parkedVehData[currId]
  if be:getObjectActive(currId) and not currVeh.ignoreTeleport then
    vehPos:set(be:getObjectPositionXYZ(currId))

    tempVec:setSub2(vehPos, focus.pos)
    local tempDist = tempVec:squaredLength()
    if tempDist < 2500 then -- ensures that vehicle stays active if the player was close to it (helps to prevent respawning if the player stays in one parking lot)
      currVeh.activeRadius = max(currVeh.activeRadius, 100 - math.sqrt(tempDist))
    end

    if not currVeh.searchFlag and currParkingSpots[1] then
      local mainRadius = currVeh.activeRadius
      local extraRadius = 0

      tempVec:normalize() -- normalized direction vector of focus point to vehicle
      local dotDirVec = focus.dirVec:dot(tempVec) -- in relation to focus direction

      if commands.isFreeCamera() then
        extraRadius = focus.pos.z - vehPos.z
        extraRadius = clamp(square(extraRadius) / 30, 0, 300) * ((dotDirVec + 1) * 0.5)
      end

      if dotDirVec > 0 then
        extraRadius = extraRadius + square(dotDirVec) * (150 + focus.dist)
      end

      mainRadius = mainRadius + extraRadius

      if square(mainRadius) < focus.pos:squaredDistance(vehPos) then
        local valid = true

        for _, veh in ipairs(getAllVehicles()) do
          if not veh.isTraffic and not veh.isParked and map.objects[veh:getId()] then
            local mapData = map.objects[veh:getId()]
            if vehPos:squaredDistance(mapData.pos) < square(innerRadius) then -- prevents respawning if too close
              valid = false
              break
            end
          end
        end

        if valid then
          processNextSpawn(currId)
          currVeh.searchFlag = true -- stop searching until next parking spot query
        end
      end
    end

    if currVeh._teleport then
      forceTeleport(currId, nil, 100)
    end
  end

  queuedIndex = queuedIndex + 1
  if queuedIndex > parkedVehCount then
    queuedIndex = 1
  end

  if respawnDelay > 0 then
    respawnDelay = max(0, respawnDelay - dtSim) -- prevents rapid parking spot searching or respawning
  end
end

local function onClientStartMission()
  if not sites then
    worldLoaded = true
  end
end

local function onClientEndMission()
  resetAll()
  worldLoaded = false
end

local function onSerialize()
  local data = {active = active, debugLevel = M.debugLevel, parkedVehIds = deepcopy(parkedVehIds), trackedVehIds = tableKeys(trackedVehData), vars = deepcopy(vars)}
  resetAll()
  return data
end

local function onDeserialized(data)
  worldLoaded = true
  processVehicles(data.parkedVehIds, true)
  for _, v in ipairs(data.trackedVehIds) do
    enableTracking(v)
  end
  setParkingVars(data.vars)
  active = data.active
  M.debugLevel = data.debugLevel
end

-- public interface
M.setSites = setSites
M.setState = setState
M.getState = getState
M.setupVehicles = setupVehicles
M.processVehicles = processVehicles
M.deleteVehicles = deleteVehicles
M.getParkedCarsList = getParkedCarsList
M.getParkedCarsData = getParkedCarsData
M.enableTracking = enableTracking
M.disableTracking = disableTracking
M.resetAll = resetAll

M.getTrackingData = getTrackingData
M.getParkingSpots = getParkingSpots
M.findParkingSpots = findParkingSpots
M.filterParkingSpots = filterParkingSpots
M.getRandomParkingSpots = getRandomParkingSpots
M.checkParkingSpot = checkParkingSpot
M.moveToParkingSpot = moveToParkingSpot
M.getCurrentParkingSpot = getCurrentParkingSpot
M.forceTeleport = forceTeleport
M.scatterParkedCars = scatterParkedCars
M.setActiveAmount = setActiveAmount
M.setParkingVars = setParkingVars
M.getParkingVars = getParkingVars

M.onUpdate = onUpdate
M.onVehicleActiveChanged = onVehicleActiveChanged
M.onVehicleDestroyed = onVehicleDestroyed
M.onVehicleGroupSpawned = onVehicleGroupSpawned
M.onClientStartMission = onClientStartMission
M.onClientEndMission = onClientEndMission
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M