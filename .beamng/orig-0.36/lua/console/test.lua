-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- this is the main test file

rerequire("lua/console/benchphysics")


--rerequire("lua/tests/simpleload")


function loadOneVehicle()
    BeamEngine:deleteAllObjects()
    BeamEngine:spawnObject(1, "vehicles/pickup", nil, vec3(0, 0, 0))
end

function benchJSON()
    directory = "vehicles/pickup"
    dir = FS:openDirectory(directory)
    if dir then
        local file = nil
        local jbeamFiles = {}
        repeat
            file = dir:getNextFilename()
            if not file then break end
            if string.find(file, ".jbeam") and not string.find(file, ".jbeamc") then
                if FS:fileExists(directory.."/"..file) then
                    table.insert(jbeamFiles, directory.."/"..file)
                end
            end
        until not file

        local allParts = {}

        log('I', "lua.test", "* loading jbeam files:")
        for k,v in pairs(jbeamFiles) do
            local content = readFile(v)
            if content ~= nil then
                local state, parts = pcall(json.decode, content)
                if state == false then
                    log('W', "lua.test", "unable to decode JSON: "..v)
                    log('W', "lua.test", "JSON decoding error: "..parts)
                    return nil
                end
                log('I', "lua.test", "  * " .. v .. " with "..tableSize(parts).." parts")
                allParts = tableMerge(allParts, parts)
            else
                log('W', "lua.test", "unable to read file: "..v)
            end
        end
        FS:closeDirectory(dir)
    end
end

--loadOneVehicle()
