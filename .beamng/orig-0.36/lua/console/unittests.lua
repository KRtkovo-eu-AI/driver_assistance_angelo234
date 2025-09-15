-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--print " *** starting unittests  ***"

-- http://wiki.garrysmod.com/page/Lua/Tutorials/Using_Metatables
--[[
jit_state = jit.status()
if (jit_state) then
    log('D', "lua.unittests", "* JIT enabled")
else
    log('D', "lua.unittests", "* JIT disabled")
end
]]--

require("mathlib")

function test_vec3_function()
    a = vec3(0,0,0)
    assert(tostring(a) == "(0, 0, 0)", "vec3 constructor test failed")
    a = vec3(1.234,2.134,3.124)
    assert(tostring(a) == "(1.234, 2.134, 3.124)", "vec3 float point test failed")
    a = vec3(0,1337.13371337,-1)
    assert(tostring(a) == "(0, 1337.134, -1)", "vec3 float point test 2 failed")
    a = vec3(0,133007.133,-1)
    assert(tostring(a) == "(0, 133007.141, -1)", "vec3 float point precision test failed")

    a = vec3(1,0,3)
    b = vec3(2,2,-2)
    c = a + b
    assert(tostring(c) == "(3, 2, 1)", "vec3 operator +")
    c = a - b
    --log('D', "lua.unittests", tostring(c))
    assert(tostring(c) == "(-1, -2, 5)", "vec3 operator -")
    c = a * b
    assert(tostring(c) == "(2, 0, -6)", "vec3 operator *")
    c = a / b
    assert(tostring(c) == "(0.5, 0, -1.5)", "vec3 operator /")
    c = -a
    assert(tostring(c) == "(-1, -0, -3)", "vec3 unary -")

    assert(a ~= b, "vec3 not equals operator")
    assert(a == a, "vec3 equals operator")
end

function test_vec3_performance()
    -- speed test
    hp = HighPerfTimer()
    for i=0,100000,1
    do
        c = a + b
        c = a - b
        c = a * b
        c = a / b
    end
    td = hp:stop()
    --log('D', "lua.unittests", "*** " .. td .. "ms")
    if (td > 850)
    then
        log('D', "lua.unittests", "warning: vec3 too slow, was " .. td .. "ms, should be under 850 ms")
    end
end

test_vec3_function()
--jit.on()
test_vec3_performance()
--jit.off()
--test_vec3_performance()


--print " *** unittests completed ***"
