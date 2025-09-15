-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local bench = require('lua/console/bananabench')

local function exportCSV(res)
  path = args[2]
  local header = 'Vehicle,Count,MBeams,RealTime,SpawnTime,ExecTime,Collision'
  local row = '%s,%d,%f,%f,%f,%f,%s'

  print('*** CSV START')

  print(header)

  -- handle:write(header .. '\n')

  for vecname, v in pairs(res.tests) do
    for i, test in ipairs(v.tests) do
      local wDynCol = test.res[1]
      local nDynCol = test.res[2]
      local line = string.format(row,
                                 vecname,
                                 test.vehicles,
                                 wDynCol.Mbeamspersec,
                                 wDynCol.percentRealtime,
                                 wDynCol.time,
                                 wDynCol.spawntime,
                                 'DynamicCollision')
      print(line)
      local line = string.format(row,
                                 vecname,
                                 test.vehicles,
                                 nDynCol.Mbeamspersec,
                                 nDynCol.percentRealtime,
                                 nDynCol.time,
                                 nDynCol.spawntime,
                                 'NoDynamicCollision')
      print(line)
    end
  end

  print('*** CSV END')
end

local res = bench.physics(bench.getAllVehicles())
exportCSV(res)
