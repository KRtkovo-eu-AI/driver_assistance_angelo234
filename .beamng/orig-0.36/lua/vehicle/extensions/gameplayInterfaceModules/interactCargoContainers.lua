-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
M.moduleActions = {}
M.moduleLookups = {}

local max = math.max
local abs = math.abs
local moduleName = "interactCargoContainers"

local expressionParser

local cargoContainerCache = nil
local cargoContainerById = nil

local functionFieldNames = {
  beamSpringFunction = true,
  beamLimitSpringFunction = true,
  beamDampFunction = true,
  nodeWeightFunction = true,
  beamStrengthFunction = true,
  beamDeformFunction = true,
  beamShortBoundFunction = true,
  beamLongBoundFunction = true
}

local function buildContainerCache()
  cargoContainerCache = {}
  cargoContainerById = {}
  local cargoContainerGroupIdToCacheIndex = {}
  local cargoContainerGroupIdToCid = {}
  local idx = 1
  for _, container in pairs(v.data.cargoStorage or {}) do
    -- generate an entry for the list that will be sent back to geLua.
    local entry = {
      id = container.cid,
      cargoTypes = container.cargoTypes,
      capacity = container.capacity,
      name = container.name or "Unnamed Container",
      groupId = container.groupId,
      partPath = container.partPath
    }
    table.insert(cargoContainerCache, entry)

    cargoContainerById[container.cid] = {
      nodes = {},
      beams = {},
      smoothers = {
        volume = newTemporalSmoothing(container.maxVolumeRate or 1000, container.maxVolumeRate or 1000),
        density = newTemporalSmoothing(2, 2, nil, 1) -- need to smooth density? probably...
      },
      target = {
        volume = 0,
        density = 1
      },
      reachedTarget = true
    }
    cargoContainerGroupIdToCacheIndex[container.groupId .. container.partPath] = idx
    cargoContainerGroupIdToCid[container.groupId .. container.partPath] = container.cid
    idx = idx + 1
  end

  -- store all nodes for groups
  for _, node in pairs(v.data.nodes) do
    if node.cargoGroup and cargoContainerGroupIdToCacheIndex[node.cargoGroup .. node.partPath] then
      local hasValidFunction = true
      for functionName, _ in pairs(functionFieldNames) do
        if node[functionName] then
          hasValidFunction = true
        end
      end
      if hasValidFunction then
        table.insert(cargoContainerById[cargoContainerGroupIdToCid[node.cargoGroup .. node.partPath]].nodes, node.cid)
      end

      -- give one of the node id's to the entry, so that geLua can use it for detachment test
      cargoContainerCache[cargoContainerGroupIdToCacheIndex[node.cargoGroup .. node.partPath]].nodeId = node.cid
    end
  end

  -- store all beams for groups
  for _, beam in pairs(v.data.beams) do
    if beam.cargoGroup and cargoContainerGroupIdToCacheIndex[beam.cargoGroup .. beam.partPath] then
      local hasValidFunction = true
      for functionName, _ in pairs(functionFieldNames) do
        if beam[functionName] then
          hasValidFunction = true
        end
      end
      if hasValidFunction then
        table.insert(cargoContainerById[cargoContainerGroupIdToCid[beam.cargoGroup .. beam.partPath]].beams, beam.cid)
      end
    end
  end

  -- wrap cargoContainerCache another time to conform to return value format for gameplay interface functions.
  cargoContainerCache = {cargoContainerCache}
end

local functionCache = {}
local function clearFunctionResults()
  for _, data in pairs(functionCache) do
    data.result = nil
  end
end

local function getFunctionResult(expr, container)
  if not functionCache[expr] then
    expressionParser = expressionParser or require("jbeam/expressionParser")
    local fun, vars = expressionParser.compileSafe("$" .. expr)
    functionCache[expr] = {fun = fun, vars = vars, result = nil}
  end
  if not functionCache[expr].result then
    functionCache[expr].vars["$volume"] = container.smoothers.volume.state
    functionCache[expr].vars["$density"] = container.smoothers.density.state
    functionCache[expr].result = functionCache[expr].fun()
  --dump(expr, functionCache[expr].result)
  end
  return functionCache[expr].result
end

local function applyNodeAndBeamValues(container, dtSim)
  clearFunctionResults()

  --update smoothers
  container.reachedTarget = false
  container.smoothers.volume:get(container.target.volume, dtSim)
  container.smoothers.density:get(container.target.density, dtSim)

  container.reachedTarget = abs(container.smoothers.volume.state - container.target.volume) < 1e-30 and abs(container.smoothers.density.state - container.target.density) < 1e-30

  for _, nodeId in ipairs(container.nodes) do
    local node = v.data.nodes[nodeId]
    if node.nodeWeightFunction then
      local nodeWeight = getFunctionResult(node.nodeWeightFunction, container)
      obj:setNodeMass(node.cid, nodeWeight)
    --dump("setNodeMass To  ", node.cid, nodeWeight)
    end
  end

  for _, beamId in ipairs(container.beams) do
    local beam = v.data.beams[beamId]

    if beam.beamSpringFunction or beam.beamDampFunction then
      local beamSpring = beam.beamSpring
      local beamDamp = beam.beamDamp
      --dump("setBeamSpringDamp From", beam.cid, beamSpring, beamDamp)

      if beam.beamSpringFunction then
        beamSpring = getFunctionResult(beam.beamSpringFunction, container)
      end
      if beam.beamDampFunction then
        beamDamp = getFunctionResult(beam.beamDampFunction, container)
      end
      obj:setBeamSpringDamp(beam.cid, beamSpring, beamDamp, -1, -1)
    --dump("setBeamSpringDamp To  ", beam.cid, beamSpring, beamDamp)
    end

    if beam.beamLimitSpringFunction or beam.beamLimitDampFunction then
      local beamLimitSpring = -1
      local beamLimitDamp = -1
      --dump("setBeamSpringDamp From", beam.cid, beamSpring, beamDamp)
      if beam.beamLimitSpringFunction then
        beamLimitSpring = getFunctionResult(beam.beamLimitSpringFunction, container)
      end
      if beam.beamLimitDampFunction then
        beamLimitDamp = getFunctionResult(beam.beamLimitDampFunction, container)
      end
      obj:setBoundedBeamSpringDampLimits(beamId, beamLimitSpring, beamLimitDamp, -1)
    --dump("setBeamSpringDamp To  ", beam.cid, beamSpring, beamDamp)
    end

    if beam.beamStrengthFunction then
      local beamStrength = getFunctionResult(beam.beamStrengthFunction, container)
      obj:setBeamStrength(beam.cid, beamStrength)
    --dump("setBeamStrength To  ", beam.cid, beamStrength)
    end
    if beam.beamDeformFunction then
      local beamDeform = getFunctionResult(beam.beamDeformFunction, container)
      obj:setBeamDeform(beam.cid, beamDeform)
    --dump("beamDeform To  ", beam.cid, beamDeform)
    end

    if beam.beamShortBoundFunction then
      local beamShortBound = getFunctionResult(beam.beamShortBoundFunction, container)
      obj:setBoundedBeamShortBound(beam.cid, beamShortBound)
    --dump("setBoundedBeamShortBound To  ", beam.cid, beamShortBound)
    end

    if beam.beamLongBoundFunction then
      local beamLongBound = getFunctionResult(beam.beamLongBoundFunction, container)
      obj:setBoundedBeamLongBound(beam.cid, beamLongBound)
    --dump("setBoundedBeamLongBound To  ", beam.cid, beamLongBound)
    end
  end

  return container.reachedTarget
end

local anyContainerNeedsUpdate = false
local function updateGFX(dtSim)
  anyContainerNeedsUpdate = false
  for _, container in pairs(cargoContainerById) do
    if not container.reachedTarget then
      applyNodeAndBeamValues(container, dtSim)
      anyContainerNeedsUpdate = anyContainerNeedsUpdate or not container.reachedTarget
    end
  end
  if not anyContainerNeedsUpdate then
    M.setUpdateEnabled(false)
  end
end

local isUpdating = false
local function setUpdateEnabled(enabled)
  if enabled and not isUpdating then
    --log("I","","Start updating cargo containers...")
    isUpdating = true
    M.updateGFX = updateGFX
    extensions.hookUpdate("updateGFX")
  elseif not enabled and isUpdating then
    isUpdating = false
    M.updateGFX = nil
    extensions.hookUpdate("updateGFX")
  --log("I","","Cargo containers updated.")
  end
end

local function setCargoContainers(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"table", "string"})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end

  if not cargoContainerCache then
    buildContainerCache()
  end

  local mode = params[2] or "updateExplicit"

  -- set all container weights according to the params data.
  for _, setContainerData in pairs(params[1] or {}) do
    -- first check if all containers have proper data
    if not setContainerData.containerId or not setContainerData.volume then
      return {failReason = "Container Data missing either containerId or volume values."}
    end
  end
  -- only then actually update
  anyContainerNeedsUpdate = false
  for id, _ in pairs(cargoContainerById) do
    local setContainerData = (params[1] or {})[id]
    local target = cargoContainerById[id].target
    local container = cargoContainerById[id]
    -- default non-set containers to 0 weight by default
    target.volume = (setContainerData and setContainerData.volume) or (mode == "updateExplicit" and target.volume or 0) or 0
    target.density = (setContainerData and setContainerData.density) or (mode == "updateExplicit" and target.density or 1) or 1

    container.reachedTarget = abs(container.smoothers.volume.state - container.target.volume) < 1e-30 and abs(container.smoothers.density.state - container.target.density) < 1e-30

    if not container.reachedTarget then
      container.reachTargetDuration = max(abs(container.smoothers.volume.state - container.target.volume) / container.smoothers.volume[false], abs(container.smoothers.density.state - container.target.density) / container.smoothers.density[false])
    end

    anyContainerNeedsUpdate = anyContainerNeedsUpdate or not container.reachedTarget
  end
  if anyContainerNeedsUpdate then
    M.setUpdateEnabled(true)
  end

  --dump(cargoContainerById)
end

local function getCargoContainers(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {})
  if not dataTypeCheck then
    return {failReason = dataTypeError}
  end
  -- create the cache if it doesnt exist yet.
  if not cargoContainerCache then
    buildContainerCache()
  end

  for _, entry in ipairs(cargoContainerCache[1]) do
    local container = cargoContainerById[entry.id]
    entry.reachTargetDuration = container.reachTargetDuration
    if container.reachedTarget then
      entry.reachTargetTimeRemaining = 0
    else
      entry.reachTargetTimeRemaining = math.max(math.abs(container.smoothers.volume.state - container.target.volume) / container.smoothers.volume[false], math.abs(container.smoothers.density.state - container.target.density) / container.smoothers.density[false])
    end
    entry.targetVolume = container.target.volume
    entry.currentVolume = container.smoothers.volume.state
    entry.rateVolume = container.smoothers.volume[false]
    entry.targetDensity = container.target.density
    entry.currentDensity = container.smoothers.density.state
  end

  return cargoContainerCache
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleActions.setCargoContainers = setCargoContainers
  M.moduleLookups.getCargoContainers = getCargoContainers
  cargoContainerCache = nil
  cargoContainerById = nil
end

local function onReset()
  cargoContainerCache = nil
  cargoContainerById = nil
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration
M.setUpdateEnabled = setUpdateEnabled
M.updateGFX = nop
M.onReset = onReset
return M
