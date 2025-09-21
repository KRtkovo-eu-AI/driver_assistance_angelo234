-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- settings
require("jit")
local physicsFPS = 2000
local defaultVehicleCount = 40
local testVehicles = {'pickup'}

-- do not change below
local physicsSteps = 100 -- max physics steps do *NOT* increase beyond 100
local gfxSteps = 200
local currentObjects = 0
local version = '0.5'
local BeamEngine = initBeamEngine(physicsFPS)


local function loadVehicle(vehicleDir)
  local vehicleBundle = require("jbeam/loader").loadVehicleStage1(1, vehicleDir, nil)
  return lpack.encode({vdata  = vehicleBundle.vdata, config = vehicleBundle.config})
end

function myBenchStep(testVehicle, vehLPackData, n, steps)
  steps = steps or gfxSteps
  local hp = HighPerfTimer()
  local hpSR = hp.stopAndReset
  -- spawn n vehicles
  for i=currentObjects + 1, n do
    BeamEngine:spawnObject2(i, testVehicle, vehLPackData, Vector3(300 * i, 300 * i, 1)) -- spawn 1m up
    currentObjects = currentObjects + 1
  end
  local spawntime = hp:stop()

  -- run the update loop and measure the total time

  local dt = (physicsSteps + 0.1) / physicsFPS
  local mint = math.huge
  local min = math.min
  local bupdate = BeamEngine.update
  collectgarbage("stop")
  jit.off()
  for i=1, steps do
    hpSR(hp)
    bupdate(BeamEngine, dt, dt)
    mint = min(mint, hpSR(hp))
  end
  jit.on()
  collectgarbage("restart")

  if BeamEngine:instabilityDetected() then
    log('E', "bananabench", ' *** INSTABILITY ***')
  end
  return spawntime, mint * steps
end

local logcache = {}
function benchLog(level, origin, msg, newline)
  if level ~= 'A' and level ~= 'D' and level ~= 'S' then
    table.insert(logcache, {level = level, origin = origin, msg = msg, newline = newline})

    -- record max severity
    if level == 'W' and logcache.max == nil then
      logcache.max = 'warn'
    elseif level == 'E' and (logcache.max == nil or logcache.max == 'warn') then
      logcache.max = 'error'
    end
  end
end

local function getAllVehicles()
  local vehicles = {}
  for _, v in ipairs(FS:findFiles('/vehicles', '*', 0, false, true)) do
    if v ~= '/vehicles/common' then
      table.insert(vehicles, string.match(v, '/vehicles/(.*)'))
    end
  end
  return vehicles
end

function benchPhysics(vehicles, vehicleMin, vehicleMax)
  setPowerPlanMaxPerformance()
  if BeamEngine == nil then
    log('E', "bananabench", 'error loading libbeamng')
    return
  end
  vehicles = vehicles or testVehicles
  vehicleMin = vehicleMin or 1
  vehicleMax = vehicleMax or VehiclesToTest or defaultVehicleCount
  local res = {
    hw = hw,
    --vehicles = vehicles,
    physicsFPS = physicsFPS,
    physicsSteps = physicsSteps,
    gfxSteps = gfxSteps,
    version=version,
    tests = {},
    vehicleMax=vehicleMax,
    vehicleMin=vehicleMin,
    time = 0,
  }

  -- to disable dynamic collision:
  -- BeamEngine:setDynamicCollisionEnabled(false)

  logcache = {}
  local timeTotalSum = 0
  for k,vehicle in pairs(vehicles) do
    local testVehicle = 'vehicles/' .. vehicle .. '/'

    local vehLPackData = loadVehicle(testVehicle)
    local test = {}

    -- init
    BeamEngine:deleteAllObjects()
    local obj = BeamEngine:spawnObject2(k, testVehicle, vehLPackData, Vector3(300 * k, 300 * k, 1)) -- spawn 1m up
    local nodecount = obj:getNodeCount()
    local beamcount = obj:getBeamCount()
    currentObjects = 1

    local skipVehicle = false
    if beamcount == nil or nodecount == nil then
      log('E', "bananabench", 'unable to get beam or node count: vehicle failed to spawn?')
      skipVehicle = true
    end

    if not skipVehicle then
      --print(' *** ' .. vehicle .. ' ***')
      test.maxMbeams = 0
      test.version = 2
      test.maxRealtimeVehicles = 0
      test.tests = {}


      -- test start
      local allTestsTime = 0
      local allTestsSpawnTime = 0
      print('   +----------+----------+')
      print('   | MBeams/s | % Realt  |')
      print('---+----------+----------+')
      for n = vehicleMin, vehicleMax, 1 do
        local t = {
          vehicles = n,
          res = {}
        }
        BeamEngine:setDynamicCollisionEnabled(1)

        -- do a warmup round
        collectgarbage("collect")
        myBenchStep(testVehicle, vehLPackData, n, 5)

        local totalSpawnTime = 0

        --
        local st, totalTime = myBenchStep(testVehicle, vehLPackData, n)
        totalSpawnTime = totalSpawnTime + st

        allTestsTime = allTestsTime + totalTime
        allTestsSpawnTime = allTestsSpawnTime + totalSpawnTime

        local totalSteps = physicsSteps * gfxSteps
        local totalBeams = totalSteps * beamcount * n
        local totalNodes = totalSteps * nodecount * n

        -- then calc the stats and output them
        local nodespersec = totalNodes / totalTime
        local Mbeamspersec = (totalBeams / totalTime) / 1000000
        if Mbeamspersec > test.maxMbeams then
          test.maxMbeams = Mbeamspersec
          test.maxMbeamsNum = n
        end
        if n == 0 then
          nodespersec = 0
          Mbeamspersec = 0
        end
        t.res.Mbeamspersec = Mbeamspersec
        local percentRealtime = (100 / physicsFPS * totalSteps) / totalTime
        if percentRealtime > 100 then
          test.maxRealtimeVehicles = math.max(test.maxRealtimeVehicles, n)
        end
        t.res.percentRealtime = percentRealtime
        t.res.time = totalTime
        t.res.spawntime = totalSpawnTime

        local function formatRes(res)
          return lpad(string.format("%0.3f", t.res.Mbeamspersec), 8, ' ')  .. ' | ' .. lpad(string.format("%0.2f", t.res.percentRealtime), 8, ' ')
        end
        --dump(t)
        --local diff = t.res[2].Mbeamspersec / t.res[1].Mbeamspersec
        t.msg = lpad(t.vehicles, 2, ' ') .. " | "  .. formatRes(t.res) .. ' | '
        table.insert(test.tests, t)
        print(t.msg)
      end

      timeTotalSum = timeTotalSum + allTestsTime
      test.time = allTestsTime

      print("Max Mbeams/s:   " .. string.format("%0.3f", test.maxMbeams) .. " Mbeams/s")
      res.tests[vehicle] = test
    end
  end

  res.time = timeTotalSum
  print("")
  print(" BANANAA!!!")
  print(" .______,# ")
  print(" \\ -----'/ ")
  print("  `-----' ")

  restorePowerPlan()
  return res
end

local M = {}

M.physics = benchPhysics
M.getAllVehicles = getAllVehicles

return M
