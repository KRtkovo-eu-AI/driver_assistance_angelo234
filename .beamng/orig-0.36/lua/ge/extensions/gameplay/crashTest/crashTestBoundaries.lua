-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
M.dependencies = { "gameplay_crashTest_scenarioManager" }

local red = {1,0,0}
local white = {1,1,1}

local isBeingDebugged
local isPlayerOutOfBounds

local drawBoundsWhenNear -- only the current vehicle draws the bounds

local sites

local function setParameters(params)
  if not params or not params.filePath then
    sites = nil
    isPlayerOutOfBounds = false
    return
  end
  if params.drawBoundsWhenNear_ == nil then drawBoundsWhenNear = true end

  sites = gameplay_sites_sitesManager.loadSites(params.filePath, true, true)
  if not sites then return end

  sites:finalizeSites()
end

local veh
local oobb
local function checkOutOfBounds(vehId)
  if not sites then return end

  veh = scenetree.findObjectById(vehId)

  if not veh or not veh.getSpawnWorldOOBB then return end

  oobb = veh:getSpawnWorldOOBB()

  for i = 0, 8 do
    local test = oobb:getPoint(0)
    local zones = sites:getZonesForPosition(test)
    if #zones == 0 then
      isPlayerOutOfBounds = true
      return
    end
  end

  isPlayerOutOfBounds = false
end

local function drawBounds()
  if not sites then return end

  red[2] = math.abs(math.sin(Engine.Platform.getRuntime()*2))
  red[3] = red[2]

  for _, zone in pairs(sites.zones.objects) do
    zone:drawDebug(nil, isPlayerOutOfBounds and red or white, 2, -0.5, not isPlayerOutOfBounds)
  end
end

local function onUpdate()
  checkOutOfBounds(be:getPlayerVehicleID(0))
  if isPlayerOutOfBounds then
    gameplay_crashTest_scenarioManager.onPlayerOutOfBounds()
  end

  if drawBoundsWhenNear then
    drawBounds()
  end
end

local function setDebug(debug)
  isBeingDebugged = debug
end

local function getMissionRelativePath(path)
  return gameplay_missions_missions.getMissionById(gameplay_missions_missionManager.getForegroundMissionId()).mgr:getRelativeAbsolutePath(path, true)
end

local function checkForStepBoundsFile(stepData)
  if stepData.hasStepBoundsFile then
    setParameters({filePath = getMissionRelativePath("boundsStep" .. stepData.id .. ".sites.json")})
    dump("new step")
  else
    setParameters({filePath = getMissionRelativePath("bounds.sites.json")})
  end
end

local function onNewCrashTestStep(stepData)
  checkForStepBoundsFile(stepData)
end

local function deactivateBounds()
  setParameters({filePath = nil})
end

M.onUpdate = onUpdate

M.setParameters = setParameters
M.setDebug = setDebug
M.onNewCrashTestStep = onNewCrashTestStep
M.deactivateBounds = deactivateBounds
return M