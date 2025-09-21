-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

function testStep(i)
    spawnObjects(i)
    frames = 1000
    hp = HighPerfTimer()
    updateLoop(frames)
    t = hp:stop()
    --print("this took "..t..' ms, which is '..(t / frames).." ms per update")
    return t/frames
end

function benchPhysics()
    io.write("Benchmark running: ")
    file = io.open("phys-scale.csv", "w")
    file:write("vehicle count;delay\n")
    local count = 8
    for i=0,count,1
    do
        io.write(i .. " ")
        d = testStep(i)
        file:write(i..";"..d.."\n")
        file:flush()
    end
    file:close()
    print(" done!")
end