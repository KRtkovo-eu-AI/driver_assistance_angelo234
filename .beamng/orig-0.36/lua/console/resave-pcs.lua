-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local function getAllVehicles()
    local vehicles = {}
    local dir = FS:openDirectory('vehicles')
    if dir then
        local entry = nil
        repeat
            entry = dir:getNextFilename()
            if not entry then break end
            table.insert(vehicles, entry)
        until not entry

        FS:closeDirectory(dir)
    end
    return vehicles
end

local function getConfigList(vdir)
    local dir = FS:openDirectory(vdir)
    if dir then
        -- find the lua module files now
        local file = nil
        local files = {}
        repeat
            file = dir:getNextFilename()
            if not file then break end
            if string.endswith(file, ".pc") then
                if FS:fileExists(vdir.."/"..file) then
                    table.insert(files, file)
                end
            end
        until not file
        FS:closeDirectory(dir)

        return files
    end
    return {}
end

function ResaveConfigs()
    io.write("Retreiving vehicles list\n")
    local vehicles = getAllVehicles()
    local configCount = 0
    for _,vdir in pairs(vehicles) do
      --do we have a vehicle that is known to fail on spawning?
      if vdir == "common" or vdir == "box" or vdir == "bathtub" then goto skipme end
      io.write("Resaving configurations for " .. vdir .. "\n")
      BeamEngine:deleteAllObjects()
      BeamEngine:spawnObject(1, "vehicles/" .. vdir .. "/", nil, vec3(0,3,0))
      local be = BeamEngine:getSlot(0)
      local configs = getConfigList("vehicles/" .. vdir .. "/")
      --check if we have any configs
      if #configs == 0 then goto skipme end
      for _,config in pairs(configs) do
          io.write("\tresaving config " .. config .. "\n")
          be:queueLuaCommand("partmgmt.loadLocal('" .. config .. "')")
          be:queueLuaCommand("partmgmt.saveLocal('" .. config .. "')")
          BeamEngine:update(2, 2)
          configCount = configCount + 1
      end
      ::skipme::
    end
    io.write("Resaved " .. configCount .. " configurations.")
end

ResaveConfigs()
