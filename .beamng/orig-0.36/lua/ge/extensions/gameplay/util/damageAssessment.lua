-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local damageThresholds = {
  {10000, "Minor", 1},
  {30000, "Moderate", 2},
  {50000, "Severe", 3},
}

local noDamageThreshold = 50
local debugMode = false

local impactCooldown = 0.5  -- Minimum time between impact detections in seconds
local lastImpactTime = 0
local lastImpactLocations = {}

local function getDamageAssessment(vehId)
  local damage = map.objects[vehId].damage
  if damage <= noDamageThreshold then
    return {damageName = "No damage", damageSeverity = 0}
  end
  for i = #damageThresholds, 1, -1 do
    local threshold = damageThresholds[i]
    if damage >= threshold[1] then
      return {damageName = damageThresholds[math.min(i+1, #damageThresholds)][2], damageSeverity = damageThresholds[math.min(i+1, #damageThresholds)][3]}
    end
  end
  return {damageName = damageThresholds[1][2], damageSeverity = damageThresholds[1][3]}
end

local function getSectionsDamageRaw(vehId)
  if vehId == nil then
    vehId = be:getPlayerVehicleID(0)
  end
  local oobb = scenetree.findObjectById(vehId):getSpawnWorldOOBB()
  local centerVec = oobb:getCenter()
  local halfExtents = oobb:getHalfExtents()
  local xAxis = oobb:getAxis(0)
  local yAxis = oobb:getAxis(1)
  local zAxis = oobb:getAxis(2)

  local sectionsDamageRaw = {}
  -- Grid cell size
  local cellSize = vec3(
    halfExtents.x * 2 / 3,
    halfExtents.y * 2 / 3,
    halfExtents.z * 2 / 3
  )

  for i = 0, 26 do
    -- Convert linear index to 3D grid coordinates
    local y = math.floor(i / 9)
    local x = math.floor((i % 9) / 3)
    local z = i % 3

    local offset = vec3(
      (x - 1) * cellSize.x,
      (y - 1) * cellSize.y,
      (z - 1) * cellSize.z
    )

    -- Transform offset to world space using OOBB axes
    local worldPos = centerVec +
      xAxis * offset.x +
      yAxis * offset.y +
      zAxis * offset.z

    local sectionDamage = scenetree.findObjectById(vehId):getSectionDamage(i)
    sectionsDamageRaw[i] = sectionDamage

    if debugMode then
      -- lerp color
      local maxDamage = 500000
      local t = math.min(sectionDamage / maxDamage, 1.0)
      local color = ColorF(t, 1-t, 0, 1)

      local paddingFactor = 0.99
      local halfCell = cellSize * 0.5 * paddingFactor
      local corners = {
        worldPos + xAxis * halfCell.x + yAxis * halfCell.y + zAxis * halfCell.z,
        worldPos + xAxis * halfCell.x + yAxis * halfCell.y - zAxis * halfCell.z,
        worldPos + xAxis * halfCell.x - yAxis * halfCell.y + zAxis * halfCell.z,
        worldPos + xAxis * halfCell.x - yAxis * halfCell.y - zAxis * halfCell.z,
        worldPos - xAxis * halfCell.x + yAxis * halfCell.y + zAxis * halfCell.z,
        worldPos - xAxis * halfCell.x + yAxis * halfCell.y - zAxis * halfCell.z,
        worldPos - xAxis * halfCell.x - yAxis * halfCell.y + zAxis * halfCell.z,
        worldPos - xAxis * halfCell.x - yAxis * halfCell.y - zAxis * halfCell.z
      }


      local edges = {
        {1,2}, {1,3}, {1,5},
        {2,4}, {2,6},
        {3,4}, {3,7},
        {4,8},
        {5,6}, {5,7},
        {6,8},
        {7,8}
      }

      for _, edge in ipairs(edges) do
        debugDrawer:drawLine(corners[edge[1]], corners[edge[2]], color)
      end

      debugDrawer:drawText(worldPos, string.format("%i (%i)", sectionDamage, i), color)
    end
  end
  return sectionsDamageRaw
end

local damageLocations = {
  front = {
    name = "Front",
    id = 0,
    damageRequirements = {
      cells = {
        3, 4, 5,
      },
    }
  },
  fontLeft = {
    name = "Front Left",
    id = 1,
    damageRequirements = {
      cells = {
        6, 7, 8,
      },
    }
  },
  frontRight = {
    name = "Front Right",
    id = 2,
    damageRequirements = {
      cells = {
        0, 1, 2,
      },
    }
  },
  left = {
    name = "Left",
    id = 3,
    damageRequirements = {
      cells = {
        15, 16, 17,
      },
    }
  },
  right = {
    name = "Right",
    id = 4,
    damageRequirements = {
      cells = {
        9, 10, 11,
      },
    }
  },
  rearLeft = {
    name = "Rear Left",
    id = 5,
    damageRequirements = {
      cells = {
        26, 25, 24,
      },
    }
  },
  rearRight = {
    name = "Rear Right",
    id = 6,
    damageRequirements = {
      cells = {
        20, 19, 18,
      },
    }
  },
  rear = {
    name = "Rear",
    id = 7,
    damageRequirements = {
      cells = {
        21, 22, 23,
      },
    }
  },
}

local minDamageThreshold = 1000

local function getCellLocation(cellId)
  for _, location in pairs(damageLocations) do
    for _, cellId_ in ipairs(location.damageRequirements.cells) do
      if cellId == cellId_ then
        return location.name
      end
    end
  end
  return "Unknown"
end

-- data.vehId : the vehicle id to get the damage from, can be nil (in which case the player's vehicle will be used)
-- if oldSectionsDamage is specified, this function will return the difference between the new and old sections damage (useful to know the damages for a specific crash), if not specified it will return the locations of the damages

  local function getTextualDamageLocations(data)
  local textualDamageLocations = {
    damagedLocations = {},
    mostDamagedLocation = nil,
    totalDamage = 0
  }
  if data == nil then
    data = {}
  end

  if data.vehId == nil then
    data.vehId = be:getPlayerVehicleID(0)
  end

  if data.oldSectionsDamageRaw == nil then
    data.oldSectionsDamageRaw = {}
    for i = 0, 26 do
      data.oldSectionsDamageRaw[i] = 0
    end
  end

  if data.newSectionsDamageRaw == nil then
    data.newSectionsDamageRaw = getSectionsDamageRaw(data.vehId)
  end

  local mostDamaged = 0
  for cellId, damage in pairs(data.newSectionsDamageRaw) do
    local damageDiff = damage - data.oldSectionsDamageRaw[cellId]
    if damageDiff > minDamageThreshold then
      local location = getCellLocation(cellId)
      textualDamageLocations.damagedLocations[location] = (textualDamageLocations.damagedLocations[location] or 0) + damageDiff
      textualDamageLocations.totalDamage = textualDamageLocations.totalDamage + damageDiff
      if damageDiff > mostDamaged then
        textualDamageLocations.mostDamagedLocation = location
        mostDamaged = damageDiff
      end
    end
  end

  return textualDamageLocations
end

local function onUpdate()
  if debugMode then
    getSectionsDamageRaw()
  end
end

local function setDebugMode(debugMode_)
  debugMode = debugMode_
end

M.onUpdate = onUpdate
M.setDebugMode = setDebugMode
M.getDamageAssessment = getDamageAssessment
M.getSectionsDamageRaw = getSectionsDamageRaw
M.getTextualDamageLocations = getTextualDamageLocations

return M
