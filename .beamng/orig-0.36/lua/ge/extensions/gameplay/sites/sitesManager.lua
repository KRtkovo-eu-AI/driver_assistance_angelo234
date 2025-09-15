-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local logTag = "sitesManager"

local sitesCache = {}
local sitesByLevel = nil
local currentLevel = nil

M.getAllLevelSites = function()
  -- caches and returns all *.sites.json files per level
  sitesByLevel = {}
  local fileCount = 0
  for _, info in ipairs(core_levels.getList()) do
    local level = string.lower(info.levelName)
    sitesByLevel[level] = {}
    local sites = FS:findFiles(info.misFilePath, '*.sites.json', -1, false, true)

    for _, sitePath in ipairs(sites) do
      table.insert(sitesByLevel[level], sitePath)
      fileCount = fileCount + 1
    end
  end
  log("D", logTag, "Found " .. fileCount .." sites files.")
end

M.loadSites = function (filepath, force, ignoreCache)
  if sitesCache[filepath] and not force then
    return sitesCache[filepath]
  else
    local data = jsonReadFile(filepath)
    if data then
      local dir, filename, ext = path.split(filepath)
      local site = require('/lua/ge/extensions/gameplay/sites/sites')()
      site:onDeserialized(data)
      site.dir = dir
      site.filename = filename
      if not ignoreCache then
        sitesCache[filepath] = site
      end
      site:finalizeSites()

      return site
    else
      log("E", logTag, "Could not load file: " .. filepath)
    end
    return nil
  end
end

M.onModManagerReady = function()
  M.getAllLevelSites()
end

M.onClientStartMission = function()
  currentLevel = getCurrentLevelIdentifier()
end

M.onClientEndMission = function()
  table.clear(sitesCache) -- cleanup and remove sites data from cache
  --if currentLevel and sitesByLevel[currentLevel] then
    --for _, file in ipairs(sitesByLevel[currentLevel]) do
      --if sitesCache[file] then
        --sitesCache[file] = nil
      --end
    --end
  --end
end

M.onSerialize = function()
  if sitesByLevel == nil or not next(sitesCache) then
    return nil
  end
  local ret = {
    sitesByLevel = deepcopy(sitesByLevel),
    currentLevel = currentLevel,
    sitesCache = {}
  }
  for fp, site in pairs(sitesCache) do
    ret.sitesCache[fp] = site:onSerialize()
  end

  return ret
end

M.onDeserialized = function(data)
  if data == nil then
    return
  end
  sitesByLevel = data.sitesByLevel
  currentLevel = data.currentLevel or getCurrentLevelIdentifier()
  for fp, s in pairs(data.sitesCache) do
    local site = require('/lua/ge/extensions/gameplay/sites/sites')()
    site:onDeserialized(s)
    site:finalizeSites()
    sitesCache[fp] = site
  end
end

-- UTIL functions
M.getSitesFilesByLevel = function()
  if not sitesByLevel then M.getAllLevelSites() end
  return sitesByLevel
end
M.getCurrentLevelSitesFiles = function(name)
  if not sitesByLevel then M.getAllLevelSites() end
  return sitesByLevel[getCurrentLevelIdentifier()] or {}
end
M.getCurrentLevelSitesFileByName = function(name)
  for _, site in ipairs(M.getCurrentLevelSitesFiles()) do
    local _, siteName = path.split(site)
    if name == string.sub(siteName, 0, -12) then
      return site
    end
  end
end

M.getBestParkingSpotForVehicleFromList = function(vehId, parkingSpots)
  vehId = vehId or be:getPlayerVehicleID(0)

  for _, spot in ipairs(parkingSpots) do
    if spot:vehicleFits(vehId) and not spot:hasAnyVehicles(vehId) then
      return spot
    end
  end

  return parkingSpots[#parkingSpots] -- use the last parking spot as a fallback
end

return M
