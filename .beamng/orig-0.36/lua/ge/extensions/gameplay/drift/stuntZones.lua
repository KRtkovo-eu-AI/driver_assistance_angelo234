-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

M.dependencies = {"gameplay_drift_general"}

local driftDebugInfo = {
  default = true,
  canBeChanged = true
}

local im = ui_imgui

local drawLines = im.BoolPtr(true)
local benchmarkCount = im.IntPtr(30)

local driftActiveData = {}
local isDrifting

local white = ColorF(1, 1, 1, 1) -- White
local green = ColorF(110/255, 219/255, 121/255, 1) -- Green
local red = ColorF(1, 0.07, 0.14, 1) -- Red
local blue = ColorF(110/255, 197/255, 219/255, 1) -- Blue

local nearStuntZone = {} -- this list keeps track of which stunt zone types the player has been near of so we can display help messages such as "Donut inside the circle!", only once
local stuntZones = {}
-- {
--   id = x,
--   type = "donut",
--   zoneData = {
--     cooldown = x,
--     scl = x,
--     txt = "x",
--     pos = vec(x,x,x)
--   },
--   activeData = {
--     currCooldown = x,
--   },
--   drawData = {}
-- }

local count = 0

local defaultDecalPath = "art/shapes/arrows/t_arrow_opaque_d.color.png"
local t, cosX, sinY, x, y, data, invAmount, filledPerc, cooldownPerc

local function increaseDecalPool(max)
  while #decals < max do
    table.insert(decals,
    {
      texture = defaultDecalPath,
      position = vec3(0, 0, 0),
      forwardVec = vec3(0, 0, 0),
      color = ColorF(1, 0, 0, 1 ),
      scale = vec3(1, 1, 4),
      fadeStart = 100,
      fadeEnd = 150
    })
  end
end

local function getDecalColor(cooldownPerc, filledPerc, t)
  if (cooldownPerc or 100) >= 100 then
    if (filledPerc or 0) > 0 then
      if ((t * 100) or (100 - t * 100)) < (filledPerc or 0) then
        return blue
      else
        return green
      end
    else
      return green
    end
  else
    if ((t * 100) or (100 - t * 100)) <= (100 - cooldownPerc or 0) then
      return red
    else
      return white
    end
  end
end


local function manageCooldown(stuntZone, dtSim)
  if not stuntZone.activeData.currCooldown then return end -- if this stunt zone doesn't have a cooldown

  if stuntZone.activeData.currCooldown > 0 then
    stuntZone.activeData.currCooldown = stuntZone.activeData.currCooldown - dtSim
  end
end

local result
local pos = vec3()

local function imguiDebug()
  if gameplay_drift_general.getExtensionDebug("gameplay_drift_stuntZones") then
    if im.Begin("Drift stunt zones") then
      im.Separator()
      im.Text("Gymkhana options")
      if im.Button("Spawn stunt zones around me") then
        M.setStuntZones({
          {type = "donut", cooldown = 8, pos = pos + vec3(10, 0, 0), scl = 10},
          {type = "donut", cooldown = 8, pos = pos + vec3(30, 0, 0), scl = 10},
          {type = "driftThrough", cooldown = 8, rot = quat(0, 0, 0, 1), pos = pos + vec3(10, 20, 0), scl = vec3(8, 1, 1)},
          {type = "driftThrough", cooldown = 8, rot = quat(0, 0, 0, 1), pos = pos + vec3(10, 29, 0), scl = vec3(8, 1, 1)},
          {type = "hitPole", pos = pos + vec3(-10, 0, 0)},
          {type = "hitPole", pos = pos + vec3(-15, 0, 0)},
          {type = "nearPole", pos = pos + vec3(-10, 20, 0)},
          {type = "nearPole", pos = pos + vec3(-15, 20, 0)}
        })
      end
      if im.Button("Remove stunt zones") then
        M.clear()
      end
      if im.Button("Reset stunt zones") then
        M.reset()
      end

      im.Separator()
      im.Text("Benchmark")
      local tempStuntZones = {}
      im.InputInt("Spawn n stunt zones", benchmarkCount)
      if im.Button("Spawn donut zones") then
        for i = 1, benchmarkCount[0], 1 do
          table.insert(tempStuntZones, {type = "donut", cooldown = 8, pos = pos + vec3(0, i * 20.5, 0), scl = 10})
        end
        M.setStuntZones(tempStuntZones)
      end
      if im.Button("Spawn drift throughs") then
        for i = 1, benchmarkCount[0], 1 do
          table.insert(tempStuntZones, {type = "driftThrough", cooldown = 8, rot = quat(0, 0, 0, 1), pos = pos + vec3(0, i * 10, 0), scl = vec3(8, 1, 1)})
        end
        M.setStuntZones(tempStuntZones)
      end
      if im.Button("Spawn hit poles") then
        for i = 1, benchmarkCount[0], 1 do
          table.insert(tempStuntZones,{type = "hitPole", pos = pos + vec3(0, i * 10, 0)})
        end
        M.setStuntZones(tempStuntZones)
      end
      if im.Button("Spawn near poles") then
        for i = 1, benchmarkCount[0], 1 do
          table.insert(tempStuntZones, {type = "nearPole", pos = pos + vec3(20, i * 10, 0)})
        end
        M.setStuntZones(tempStuntZones)
      end

      im.Separator()
      im.Text("Stunt zone count : " .. #stuntZones)
      im.Checkbox('Draw lines', drawLines)

      if im.BeginChild1("Stunt zones", im.ImVec2(im.GetWindowContentRegionWidth(), 150)) then
        for _, stuntZone in ipairs(stuntZones) do
          im.Text(stuntZone.data.zoneData.type .. " id : " .. stuntZone.data.id)
          im.SameLine()
          if im.Button("Score ##"..stuntZone.data.id) then
            stuntZone:accomplish()
          end
        end
      end
      im.End()
    end
  end
end

local function onUpdate(dtReal, dtSim, dtRaw)
  pos:set(gameplay_drift_drift.getVehPos() or vec3(0,0,0))

  imguiDebug()

  if gameplay_drift_general.getPaused() or gameplay_drift_general.getFrozen() then return end

  isDrifting = gameplay_drift_drift.getIsDrifting()

  local isInside
  if gameplay_drift_drift.doesPlHaveVeh() then
    for _, stuntZone in ipairs(stuntZones) do
      if stuntZone:isAvailable() then

        -- detect if the player is near for the first time
        if not nearStuntZone[stuntZone.data.zoneData.type] and pos:distance(stuntZone.data.zoneData.pos) <= stuntZone.data.nearDist then
          extensions.hook("onNearStuntZoneFirst", stuntZone)
          nearStuntZone[stuntZone.data.zoneData.type] = true
        end

        -- detect the actual stunt
        if isDrifting then
          if not stuntZone.isPlayerInside then
            isInside = true
          else
            isInside = stuntZone:isPlayerInside()
          end

          if isInside then
            stuntZone:detectStunt()
          end
        end
      end

      if stuntZone.onUpdate then stuntZone:onUpdate(dtReal, dtSim) end

      manageCooldown(stuntZone, dtSim)

    end
  end
end

local function reset()
  for _, stuntZone in ipairs(stuntZones) do
    stuntZone:reset()
  end
end

local function clear()
  for i = #stuntZones, 1, -1 do
    M.clearStuntZone(stuntZones[i].data.id)
  end

  stuntZones = {}
  nearStuntZone = {}
end

local function setStuntZones(zones)
  clear()

  local stuntZoneId = 1
  -- specific setup if needed / sanitizing
  for _, stuntZone in ipairs(zones) do
    local createdStuntZone = {
      id = stuntZoneId,
      zoneData = stuntZone
    }

    -- create a "near" distance to display so help texts when approachin a stunt zone
    local nearDist = 6
    if stuntZone.scl then
      if type(stuntZone.scl) == "number" then
        nearDist = stuntZone.scl + 6
      else
        nearDist = math.max(stuntZone.scl.x, stuntZone.scl.y, stuntZone.scl.z) + 4
      end
    end
    createdStuntZone.nearDist = nearDist

    -- setting up default points/score if none are specified
    if not createdStuntZone.zoneData.score then
      createdStuntZone.zoneData.score = gameplay_drift_scoring.getScoreOptions().defaultPoints[stuntZone.type]
    end

    -- setting up some draw data, later used to draw the zones
    if stuntZone.type == "donut" then
      createdStuntZone.drawData = {
        decalCount = stuntZone.scl * 10,
      }
    elseif stuntZone.type == "driftThrough" then
      createdStuntZone.x = stuntZone.rot* vec3(stuntZone.scl.x,0,0)
      createdStuntZone.y = stuntZone.rot * vec3(0,stuntZone.scl.y,0)
      createdStuntZone.z = stuntZone.rot * vec3(0,0,stuntZone.scl.z)

      local pointOne = stuntZone.pos + createdStuntZone.x + createdStuntZone.y
      local pointTwo = stuntZone.pos + createdStuntZone.x - createdStuntZone.y
      local pointThree = stuntZone.pos - createdStuntZone.x + createdStuntZone.y
      local pointFour = stuntZone.pos - createdStuntZone.x - createdStuntZone.y

      if pointOne:distance(pointThree) < pointOne:distance(pointTwo) then
        createdStuntZone.drawData = {
          pointA = pointOne,
          pointB = pointTwo,
          pointC = pointThree,
          pointD = pointFour,
        }
      else
        createdStuntZone.drawData = {
          pointA = pointTwo,
          pointB = pointFour,
          pointC = pointOne,
          pointD = pointThree,
        }
      end
    end

    table.insert(stuntZones, require("gameplay/drift/stuntZones/".. stuntZone.type)(createdStuntZone))

    stuntZoneId = stuntZoneId + 1
  end
end

local function onDriftStatusChanged(isDrifting)
  if not isDrifting then
    for _, stuntZone in ipairs(stuntZones) do
      if stuntZone.data.zoneData.type == "donut" then
        stuntZone.activeData.totalDriftAngle = 0
      end
    end
  end
end

local function clearStuntZone(id)
  local i = 1
  for _, stuntZone in ipairs(stuntZones) do
    if stuntZone.data.id == id then
      if stuntZone.clear then
        stuntZone:clear()
      end
      table.remove(stuntZones, i)
    end
    i = i + 1
  end
end

local function getStuntZones()
  return stuntZones
end

local function onExtensionUnloaded()
  clear()
end

local function onSerialize()
  clear()
end


local function onVehicleDestroyed(vehId)
  for _, stuntZone in ipairs(stuntZones) do
    if stuntZone.onVehicleDestroyed then
      stuntZone:onVehicleDestroyed(vehId)
    end
  end
end

local function getDrawLines()
  return drawLines[0]
end

local function getGreenColor()
  return green
end

local function getWhiteColor()
  return white
end

local function getRedColor()
  return red
end

local function getDriftDebugInfo()
  return driftDebugInfo
end

local function getLineThickness(linePos)
  return 250 / linePos:distance(core_camera.getPosition())
end

local function onDriftDebugChanged()
  drawLines = im.BoolPtr(true)
end

local function onAnyStuntZoneAccomplished(data)
  if gameplay_drift_stallingSystem then
    gameplay_drift_stallingSystem.processStuntZone(data.stuntZoneId)
  end

  extensions.hook(data.subHookName, data.subHookData)
end

M.onUpdate = onUpdate
M.onExtensionUnloaded = onExtensionUnloaded
M.onDriftStatusChanged = onDriftStatusChanged
M.onDriftDebugChanged = onDriftDebugChanged
M.onVehicleDestroyed = onVehicleDestroyed
M.onSerialize = onSerialize
M.onAnyStuntZoneAccomplished = onAnyStuntZoneAccomplished
M.clear = clear
M.reset = reset
M.setStuntZones = setStuntZones
M.getStuntZones = getStuntZones

M.increaseDecalPool = increaseDecalPool
M.getDecalColor = getDecalColor
M.getLineThickness = getLineThickness
M.getGreenColor = getGreenColor
M.getWhiteColor = getWhiteColor
M.getRedColor = getRedColor
M.getDriftDebugInfo = getDriftDebugInfo

-- TEST
M.getDrawLines = getDrawLines

-- INTERNAL
M.clearStuntZone = clearStuntZone
return M