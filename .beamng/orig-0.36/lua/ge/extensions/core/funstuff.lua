local M = {}
M.dependencies = {
  "core_jobsystem",
  'gameplay_forceField',
}

local function toggleForceField()
  if gameplay_forceField.isActive() then
    if gameplay_forceField.getForceMultiplier() > 0 then
      gameplay_forceField.deactivate(true)
      gameplay_forceField.setForceMultiplier(-0.5)
      gameplay_forceField.activate()
    else
      gameplay_forceField.deactivate()
    end
  else
    gameplay_forceField.setForceMultiplier(1)
    gameplay_forceField.activate()
  end
  return {"reload"}
end

local function flingUpward()
  local veh = getPlayerVehicle(0)
  if veh then
    veh:applyClusterVelocityScaleAdd(veh:getRefNodeId(), 1, 0, 0, 15)
  end
  return {"hide"}
end

local function flingDownward()
  local veh = getPlayerVehicle(0)
  if veh then
    veh:applyClusterVelocityScaleAdd(veh:getRefNodeId(), 1, 0, 0, -15)
  end
  return {"hide"}
end

local function boost()
  local veh = getPlayerVehicle(0)
  if veh then
    local dir = veh:getDirectionVector()
    dir:normalize()
    dir = dir * 25
    veh:applyClusterVelocityScaleAdd(veh:getRefNodeId(), 1, dir.x, dir.y, dir.z)
  end
  return {"hide"}
end

local function boostBackwards()
  local veh = getPlayerVehicle(0)
  if veh then
    local dir = veh:getDirectionVector()
    dir:normalize()
    dir = dir * - 25
    veh:applyClusterVelocityScaleAdd(veh:getRefNodeId(), 1, dir.x, dir.y, dir.z)
  end
  return {"hide"}
end



local function breakAllBreakgroups()
  getPlayerVehicle(0):queueLuaCommand("beamstate.breakAllBreakgroups()")
  return {"hide"}
end

local function breakHinges()
  getPlayerVehicle(0):queueLuaCommand("beamstate.breakHinges()")
  return {"hide"}
end

local function deflateTires()
  getPlayerVehicle(0):queueLuaCommand("beamstate.deflateTires()")
  return {"hide"}
end

local function deflateRandomTire()
  getPlayerVehicle(0):queueLuaCommand("beamstate.deflateRandomTire()")
  return {"hide"}
end

local function igniteVehicle()
  getPlayerVehicle(0):queueLuaCommand("fire.igniteVehicle()")
  return {"hide"}
end

local function extinguishVehicle()
  getPlayerVehicle(0):queueLuaCommand("fire.extinguishVehicle()")
  return {"hide"}
end

local function randomizeColors()
  local veh = getPlayerVehicle(0)
  if not veh then return end
  local modelInfo = core_vehicles.getModel(veh.JBeam)
  if not modelInfo then return end
  local paints = tableKeys(tableValuesAsLookupDict(modelInfo.model.paints or {}))
  local paint1, paint2, paint3 = paints[math.random(1, #paints)], paints[math.random(1, #paints)], paints[math.random(1, #paints)]
  local paintData = {
    createVehiclePaint({x=paint1.baseColor[1], y=paint1.baseColor[2], z=paint1.baseColor[3], w=paint1.baseColor[4]}, {paint1.metallic, paint1.roughness, paint1.clearcoat, paint1.clearcoatRoughness}),
    createVehiclePaint({x=paint2.baseColor[1], y=paint2.baseColor[2], z=paint2.baseColor[3], w=paint2.baseColor[4]}, {paint2.metallic, paint2.roughness, paint2.clearcoat, paint2.clearcoatRoughness}),
    createVehiclePaint({x=paint3.baseColor[1], y=paint3.baseColor[2], z=paint3.baseColor[3], w=paint3.baseColor[4]}, {paint3.metallic, paint3.roughness, paint3.clearcoat, paint3.clearcoatRoughness})}
  veh.color = ColorF(paintData[1].baseColor[1], paintData[1].baseColor[2], paintData[1].baseColor[3], paintData[1].baseColor[4]):asLinear4F()
  veh.colorPalette0 = ColorF(paintData[2].baseColor[1], paintData[2].baseColor[2], paintData[2].baseColor[3], paintData[2].baseColor[4]):asLinear4F()
  veh.colorPalette1 = ColorF(paintData[3].baseColor[1], paintData[3].baseColor[2], paintData[3].baseColor[3], paintData[3].baseColor[4]):asLinear4F()
  veh:setMetallicPaintData(paintData)
  return {"reload"}
end

local function explodeVehicle()
  local vehicle = getPlayerVehicle(0)
  if not vehicle then return end
  local boundingBox = vehicle:getSpawnWorldOOBB()
  local halfExtents = boundingBox:getHalfExtents()
  -- Get vehicle's transform to convert local to world coordinates
  local pos = vehicle:getPosition()
  local rot = quatFromDir(vehicle:getDirectionVector(), vehicle:getDirectionVectorUp())
  -- Create a point at the bottom center in local space (Z is up/down)
  local localPoint = vec3(math.random()*0.25-0.125, math.random()*0.25-0.125, -halfExtents.z-0.8)
  -- Convert to world space
  local worldPoint = rot * localPoint + pos
  local longestHalfExtent = math.max(math.max(halfExtents.x, halfExtents.y), halfExtents.z)
  local vehicleSizeFactor = longestHalfExtent/3
  local mass = -30000000000000
  local planetRadius = 5
  local command = string.format('obj:setPlanets({%f, %f, %f, %d, %f})', worldPoint.x, worldPoint.y, worldPoint.z, planetRadius, mass * vehicleSizeFactor * 1)

  -- Create a job to remove the planet after 1 second
  core_jobsystem.create(function(job, dtTable)
    be:queueAllObjectLua(command)
    if vehicle then
      vehicle:queueLuaCommand("fire.explodeVehicle()")
      vehicle:queueLuaCommand("beamstate.breakAllBreakgroups()")
    end
    local remainingTime = 0.2
    while remainingTime >= 0 do
      remainingTime = remainingTime - dtTable.dtSim
      job.sleep(0)
    end
    be:queueAllObjectLua("obj:setPlanets({})")
  end)

  return {"hide"}
end


local vehicleOptions = nil
local function randomVehicle()
  local vehs =  core_vehicles.getVehicleList().vehicles
  if not vehicleOptions then
    vehicleOptions = {}
    for _, v in ipairs(vehs) do
      local passType = true
      passType = passType and (v.model.Type == 'Car' or v.model.Type == 'Truck') and v.model['Body Style'] ~= 'Bus' and v.model['isAuxiliary'] ~= true -- always only use cars or trucks
      if passType then
        local model = {
          model = v.model.key,
          configs = {},
          paints = tableKeys(tableValuesAsLookupDict(v.model.paints or {}))
        }
        for _, c in pairs(v.configs) do
          local passConfig = true
          passConfig = passConfig and c["Top Speed"] and c["Top Speed"] > 19 and c['isAuxiliary'] ~= true -- always have some minimum speed
          if passConfig then
            table.insert(model.configs, {
              config = c.key,
              name = c.Name,
            })
          end
        end
        if #model.configs > 0 then
          table.insert(vehicleOptions, model)
        end
      end
    end
  end
  local model = vehicleOptions[math.random(1, #vehicleOptions)]
  local config = model.configs[math.random(1, #model.configs)]
  local options = {config = config}
  local spawningOptions = sanitizeVehicleSpawnOptions(model.model, options)
  if be:getPlayerVehicle(0) then
    core_vehicles.replaceVehicle(spawningOptions.model, spawningOptions.config)
  else
    core_vehicles.spawnNewVehicle(spawningOptions.model, spawningOptions.config)
  end
  randomizeColors()
  return {"hide"}
end

local function randomRoute()
  local veh = getPlayerVehicle(0)
  if not veh then return end

  local name_a, name_b, distance = map.findClosestRoad(veh:getPosition())
  if not name_a or not name_b then return end
  local a = map.getMap().nodes[name_a]
  local b = map.getMap().nodes[name_b]

  local xnorm = clamp(veh:getPosition():xnormOnLine(a.pos, b.pos), 0, 1)
  -- if we are closer to point b, swap it around
  if xnorm > 0.5 then
    name_a, name_b = name_b, name_a
    a = map.getMap().nodes[name_a]
    b = map.getMap().nodes[name_b]
    xnorm = 1-xnorm
  end
  local bestTarget = nil
  local bestDistance = 0
  local targets = tableKeys(map.getMap().nodes)
  for i = 1, 10 do
    local target = map.getMap().nodes[targets[math.random(1, #targets)]]
    freeroam_bigMapMode.setNavFocus(target.pos)
    local length = core_groundMarkers.routePlanner.path[1].distToTarget
    if length > bestDistance then
      bestDistance = length
      bestTarget = target
    end
  end
  --local wps = map.getGraphpath():getRandomPathG(name_a, fwd, 1000 + math.random(0, 2000), nil, nil, false)
  --local wps = map.getGraphpath():getRandomPath(name_b, name_a, nil)
  freeroam_bigMapMode.setNavFocus(bestTarget.pos)
  return {"hide"}
end

local function openLatches()
  core_vehicleBridge.executeAction(getPlayerVehicle(0), 'latchesOpen')
  return {"hide"}
end

local function closeLatches()
  core_vehicleBridge.executeAction(getPlayerVehicle(0), 'latchesClose')
  return {"hide"}
end

local function setFuelLevel(level)
  local veh = getPlayerVehicle(0)
  if veh then
    core_vehicleBridge.requestValue(veh,function(ret)
      if not ret or not ret[1] then return end
      for _, tank in ipairs(ret[1]) do
        core_vehicleBridge.executeAction(veh,'setEnergyStorageEnergy', tank.name, tank.maxEnergy * level)
      end
    end, 'energyStorage')
  end
  return {"hide"}
end

local function registerFunstuffActions(entries)
  core_quickAccess.addEntry({ level = '/root/sandbox/funStuff/', generator = function(entries)

    table.insert(entries, {startSlot = 1, endSlot = 1, dynamicSlot = {id="funstuff_left"}})
    table.insert(entries, {startSlot = 2, endSlot = 2, dynamicSlot = {id="funstuff_upLeft"}})
    table.insert(entries, {startSlot = 3, endSlot = 3, dynamicSlot = {id="funstuff_up"}})
    table.insert(entries, {startSlot = 4, endSlot = 4, dynamicSlot = {id="funstuff_upRight"}})
    table.insert(entries, {startSlot = 5, endSlot = 5, dynamicSlot = {id="funstuff_right"}})
    -- Add main funstuff menu entries

    table.insert(entries, {
      title = "ui.radialmenu2.funstuff.Other",
      icon = "vehicleFeatures03",
      uniqueID = "funstuff_other",
      startSlot = 6, endSlot = 6,
      ["goto"] = "/root/sandbox/funStuff/other/"
    })

    table.insert(entries, {
      title = "ui.radialmenu2.funstuff.Forces",
      icon = "wavesSignalSentRight",
      uniqueID = "funstuff_forces",
      startSlot = 7, endSlot = 7,
      ["goto"] = "/root/sandbox/funStuff/forces/"
    })

    table.insert(entries, {
      title = "ui.radialmenu2.funstuff.Destruction",
      icon = "hammerParticles",
      uniqueID = "funstuff_destruction",
      startSlot = 8, endSlot = 8,
      ["goto"] = "/root/sandbox/funStuff/destruction/"
    })
  end})

  -- Destruction submenu
  core_quickAccess.addEntry({ level = '/root/sandbox/funStuff/destruction/', generator = function(entries)
    if not core_input_actionFilter.isActionBlocked("funBreak") then
      table.insert(entries, {
        uniqueID = "funBreak",
        title = "ui.radialmenu2.funstuff.Break",
        desc = "ui.radialmenu2.funstuff.Break.desc",
        icon = 'cogsDamaged',
        onSelect = breakAllBreakgroups
      })
    end
    if not core_input_actionFilter.isActionBlocked("funHinges") then
      table.insert(entries, {
        uniqueID = "funHinges",
        title = "ui.radialmenu2.funstuff.Hinges",
        desc = "ui.radialmenu2.funstuff.Hinges.desc",
        icon = 'hingeBroken',
        onSelect = breakHinges
      })
    end
    if not core_input_actionFilter.isActionBlocked("funTires") then
      table.insert(entries, {
        uniqueID = "funTires",
        title = "ui.radialmenu2.funstuff.Tires",
        desc = "ui.radialmenu2.funstuff.Tires.desc",
        icon = 'tireDeflated',
        onSelect = deflateTires
      })
      table.insert(entries, {
        uniqueID = "funRandomTire",
        title = "ui.radialmenu2.funstuff.RandomTire",
        desc = "ui.radialmenu2.funstuff.RandomTire.desc",
        icon = 'tireAirPuff',
        onSelect = deflateRandomTire
      })
    end
    if not core_input_actionFilter.isActionBlocked("funFire") then
      table.insert(entries, {
        uniqueID = "funFire",
        title = "ui.radialmenu2.funstuff.Fire",
        desc = "ui.radialmenu2.funstuff.Fire.desc",
        icon = 'fire',
        onSelect = igniteVehicle
      })
    end
    if not core_input_actionFilter.isActionBlocked("funExtinguish") then
      table.insert(entries, {
        uniqueID = "funExtinguish",
        title = "ui.radialmenu2.funstuff.Extinguish",
        desc = "ui.radialmenu2.funstuff.Extinguish.desc",
        icon = 'fireExtinguisher',
        onSelect = extinguishVehicle
      })
    end
    if not core_input_actionFilter.isActionBlocked("funBoom") then
      table.insert(entries, {
        uniqueID = "funBoom",
        title = "ui.radialmenu2.funstuff.Boom",
        desc = "ui.radialmenu2.funstuff.Boom.desc",
        icon = 'explosion',
        onSelect = explodeVehicle
      })
    end
  end})

  -- Forces submenu
  core_quickAccess.addEntry({ level = '/root/sandbox/funStuff/forces/', generator = function(entries)
    if not core_input_actionFilter.isActionBlocked("forceField") then
      local e = {
        title = 'ui.radialmenu2.funstuff.ForceField',
        desc = {txt = 'ui.radialmenu2.funstuff.ForceField.desc', context = {status = 'ui.radialmenu2.funstuff.ForceField.inactive'}},
        icon = 'forceFieldPush1',
        uniqueID = 'forceField',
        startSlot = 2,
        onSelect = toggleForceField
      }
      if gameplay_forceField.isActive() then
        e.color = 'var(--bng-orange-400)'
        if gameplay_forceField.getForceMultiplier() > 0 then
          e.desc = {txt = 'ui.radialmenu2.funstuff.ForceField.desc', context = {status = 'ui.radialmenu2.funstuff.ForceField.repulsion'}}
        end
        if gameplay_forceField.getForceMultiplier() < 0 then
          e.desc = {txt = 'ui.radialmenu2.funstuff.ForceField.desc', context = {status = 'ui.radialmenu2.funstuff.ForceField.attraction'}}
          e.icon = 'forceFieldPull1'
        end
      end
      table.insert(entries, e)
    end
    if not core_input_actionFilter.isActionBlocked("forceField") then
      table.insert(entries, {
        title = "ui.radialmenu2.funstuff.Fling",
        desc = "ui.radialmenu2.funstuff.Fling.desc",
        icon = 'pushUp',
        uniqueID = 'flingUpward',
        startSlot = 3,
        onSelect = flingUpward
      })
      table.insert(entries, {
        title = "ui.radialmenu2.funstuff.FlingDownward",
        desc = "ui.radialmenu2.funstuff.FlingDownward.desc",
        icon = 'pushDown',
        startSlot = 7,
        uniqueID = 'flingDownward',
        onSelect = flingDownward
      })


      table.insert(entries, {
        title = "ui.radialmenu2.funstuff.Boost",
        desc = "ui.radialmenu2.funstuff.Boost.desc",
        icon = 'carFast',
        startSlot = 5,
        uniqueID = 'boost',
        onSelect = boost
      })
      table.insert(entries, {
        title = "ui.radialmenu2.funstuff.BoostBackwards",
        desc = "ui.radialmenu2.funstuff.BoostBackwards.desc",
        icon = 'carFastReverse',
        uniqueID = 'boostBackwards',
        startSlot = 1,
        onSelect = boostBackwards
      })
    end
  end})

  -- Other submenu
  core_quickAccess.addEntry({ level = '/root/sandbox/funStuff/other/', generator = function(entries)
    if not core_input_actionFilter.isActionBlocked("latchesOpen") then
      table.insert(entries, {
        uniqueID = "funLatchesOpen",
        title = "ui.radialmenu2.funstuff.LatchesOpen",
        desc = "ui.radialmenu2.funstuff.LatchesOpen.desc",
        icon = "vehicleDoorsOpen",
        onSelect = openLatches
      })
    end
    if not core_input_actionFilter.isActionBlocked("latchesClose") then
      table.insert(entries, {
        uniqueID = "funLatchesClose",
        title = "ui.radialmenu2.funstuff.LatchesClose",
        desc = "ui.radialmenu2.funstuff.LatchesClose.desc",
        icon = "vehicleDoorsClose",
        onSelect = closeLatches
      })
    end

    if not core_input_actionFilter.isActionBlocked("parts_selector") then
      table.insert(entries, {
        uniqueID = "funSetFuelLevelFull",
        title = "ui.radialmenu2.funstuff.SetFuelLevelFull",
        icon = "gaugeFull",
        onSelect = function() return setFuelLevel(1) end
      })
      table.insert(entries, {
        uniqueID = "funSetFuelLevelEmpty",
        title = "ui.radialmenu2.funstuff.SetFuelLevelEmpty",
        icon = "gaugeEmpty",
        onSelect = function() return setFuelLevel(0) end
      })
      table.insert(entries, {
        uniqueID = "funRandomizeColors",
        title = "ui.radialmenu2.funstuff.RandomizeColors",
        icon = "sprayCan",
        onSelect = randomizeColors
      })
    end
    if not core_input_actionFilter.isActionBlocked("vehicle_selector") then
      table.insert(entries, {
        uniqueID = "funRandomVehicle",
        title = "ui.radialmenu2.funstuff.RandomVehicle",
        icon = "carStarred",
        onSelect = randomVehicle
      })
    end
    if not core_input_actionFilter.isActionBlocked("toggleBigMap") then
      table.insert(entries, {
        uniqueID = "funRandomRoute",
        title = "ui.radialmenu2.funstuff.RandomRoute",
        icon = "pathDice",
        onSelect = randomRoute
      })
    end
  end})
end

M.breakAllBreakgroups = breakAllBreakgroups
M.breakHinges = breakHinges
M.deflateTires = deflateTires
M.deflateRandomTire = deflateRandomTire
M.igniteVehicle = igniteVehicle
M.extinguishVehicle = extinguishVehicle
M.explodeVehicle = explodeVehicle
M.toggleForceField = toggleForceField
M.openLatches = openLatches
M.closeLatches = closeLatches
M.flingUpward = flingUpward
M.flingDownward = flingDownward
M.boost = boost
M.boostBackwards = boostBackwards

M.registerFunstuffActions = registerFunstuffActions

return M
