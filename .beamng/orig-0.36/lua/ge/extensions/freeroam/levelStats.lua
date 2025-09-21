-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local statsFilePath = '/settings/cloud/uiStats.json'
local statsCache
local function getStats()
  statsCache = statsCache or jsonReadFile(statsFilePath) or { levels = {}, vehicles = {} }
  statsCache.levels = statsCache.levels or {}
  statsCache.vehicles = statsCache.vehicles or {}
  return statsCache
end

function M.recordLevelSpawn(levelName, spawnPointName)
  spawnPointName = spawnPointName or "__default__" -- the default spawnpoint has no name, so we store this internal name as equivalent in the stats file
  local stats = getStats()
  local now = os.date("!%Y-%m-%dT%H:%M:%SZ") -- ISO8601 format
  stats.levels[levelName] = stats.levels[levelName] or {}
  local l = stats.levels[levelName]
  l.count = (l.count or 0) + 1
  l.lastUsed = now
  l.spawnPoints = l.spawnPoints or {}
  l.spawnPoints[spawnPointName] = l.spawnPoints[spawnPointName] or {}
  local s = l.spawnPoints[spawnPointName]
  s.spawnPointName = spawnPointName
  s.count = (s.count or 0) + 1
  s.lastUsed = now
  log('D', '', string.format("Saving stats for level %q, spawnpoint %q, to file: %q", levelName, spawnPointName, statsFilePath))
  jsonWriteFile(statsFilePath, stats, true)
end

function M.getTopLevels()
  local result = {}
  for levelName, data in pairs(getStats().levels) do
    table.insert(result, { levelName = levelName, count = data.count, lastUsed = data.lastUsed })
  end
  table.sort(result, function(a, b) return a.lastUsed > b.lastUsed end)
  local lastUsed = result[1] and result[1].levelName
  table.sort(result, function(a, b) if a.count == b.count then return a.lastUsed > b.lastUsed end return a.count > b.count end)
  for i, data in ipairs(result) do
    if lastUsed == data.levelName then
      table.remove(result, i)
      table.insert(result, 1, data)
      break
    end
  end
  return result
end

function M.getTopSpawnPoints(levelName)
  local result = {}
  for spawnPointName, data in pairs(getStats().levels[levelName] and getStats().levels[levelName].spawnPoints or {}) do
    table.insert(result, { spawnPointName = spawnPointName ~= "__default__" and spawnPointName or nil, count = data.count, lastUsed = data.lastUsed })
  end
  table.sort(result, function(a, b) return a.lastUsed > b.lastUsed end)
  local lastUsed = result[1] and result[1].spawnPointName
  table.sort(result, function(a, b) if a.count == b.count then return a.lastUsed > b.lastUsed end return a.count > b.count end)
  for i, data in ipairs(result) do
    if lastUsed == data.spawnPointName then
      table.remove(result, i)
      table.insert(result, 1, data)
      break
    end
  end
  return result
end

function M.getUiStats()
  local topLevels = M.getTopLevels()
  local topSpawnPoints = {}
  for _, level in ipairs(topLevels) do
    local spawnPoints = M.getTopSpawnPoints(level.levelName)
    for _, spawnPoint in ipairs(spawnPoints) do
      table.insert(topSpawnPoints, spawnPoint)
    end
  end
  return { topLevels = topLevels, topSpawnPoints = topSpawnPoints }
end

return M