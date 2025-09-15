-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

BeamEngine:deleteAllObjects()
v = newVehicle()
v:spawn("pickup", -1, vec3(0, 0, 0.1))
--v = newVehicle()
--v:spawn("hatch", -1, vec3(0, 0, 10.1))
--v = newVehicle()
--v:spawn("pickup", -1, vec3(0, 0, 20.1))

--print (BeamEngine:getSlotCount() .. " vehicles spawned")

function test(n)
    for i=0,n,1
    do
        --hp = HighPerfTimer()
        BeamEngine:update(1/2000, 1/2000)
        --print(" - "..hp:stop().-" ms")
    end
end

test(5000)

