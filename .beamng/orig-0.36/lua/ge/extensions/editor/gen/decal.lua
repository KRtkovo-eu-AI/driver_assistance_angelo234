local im = ui_imgui

local D = {}
D.ui = {
    laneL = 1,
    laneR = 1,
    middleYellow = false,
    middleDashed = false,
    tpick = 2,
}

local mat = createObject("Material")
mat:setField("diffuseColor", 0, '1 1 1 0.9')
mat:registerObject('mat_white')

local default = {
    react_time = 5,
    nbranch = 4,
    laneWidth = 3,
    laneNum = 2,
    mat = 'road1_concrete',
--    matline = 'mat_white', -- 'm_line_white'
    matline = 'ChaulkLine', -- 'm_line_white'
    matlinedash = 'crossing_white',
    rjunc = 50*2/3,
    rround = 20,
    radcoeff = 4/2,
    rexit = 20,
    wexit = 4,
    sidemargin = 0.3,
    bank = 2,
    wline = 0.2,
    v2tmin = 5,
}
D.default = default
local out = {avedit = {}, apick = {}, apoint = {}, apath = {},
    default=default}
local incontrol
local incommand = {
--    'exit_right'
--    'turn_left'
--    'turn_right'
} -- {'exit_right'}
local ccommand
local icroad = 0
local amatch = {} -- ind,skip list of exit branches that need matching


--local ffi = require("ffi")
--local ffifound, ffi = pcall(require, 'ffi')
--if not ffifound then
--  ffi = {offsetof = function(v, atr) return v[atr] end}
--end
--local mathLib = require("/lua/ge/extensions/editor/gen/audll")

local nwlib
if false then
    local ffi = require("ffi")
    --local ffi = require "ffi"
    --local mathLib = ffi.load("audll")
    --local mathLib = ffi.load("test.dll")
    local nwlib = ffi.load("nwlib.dll")

    ffi.cdef[[
        float[3] forWeight();
    ]]

    ffi.cdef[[
        int mul(int a, int b);
    ]]
    --print('?? if_ADD:'..tostring(mathLib))
    local result = mathLib.mul(10, 20)
    print('??_____________ DLL_res:'..tostring(result))
end

--[[
]]
--__INTERNAL_Debug.disableSandbox()

--ffi.C.add(1,2)
--local mathLib = ffi.load("audll")
--print('?? if_LIB:'..tostring(result))


local U = require('/lua/ge/extensions/editor/gen/utils')
local R = require('/lua/ge/extensions/editor/gen/render')

local indrag, inmerge, _dbdrag
--U.camSet({7.95, 4.16, 4.54, -0.0890567, -0.144725, 0.839285, -0.516454})
local car = {
    target = nil,
    vel = nil, --vec3(0,0,0),
    pos = nil,
    gaz = 0,
}
local veh = getPlayerVehicle(0)
if veh then
    car.pos = veh:getPosition()
end


local lo = U.lo


lo('= DECAL:')

--local function lo(ms, yes)
--    if indrag and not yes then return end
--    if U._PRD ~= 0 then return end
--    print(ms)
--end

--    lo('= DECAL:'..tostring(veh)..':'..tostring(scenetree.findObject("Vegetation")))

--[[
local mm = {}

mm[{1,2}] = 5
lo('?? if_MM:'..tostring(mm[{1,2}]))
for key,v in pairs(mm) do
    local akey = {key}
    U.dump(key, '?? for_val:'..v..':'..tostring(mm[akey[1] ]))
end
]]


local Render = require('/lua/ge/extensions/editor/gen/render')
--input = require("/lua/vehicle/input")
--guihooks = require("/lua/common/guihooks")
--local AI = require("/lua/vehicle/ai")
--local U = require('/lua/ge/extensions/editor/gen/utils')

local groupDecal,groupLines
if false then
    local groupDecal = scenetree.findObject('e_road')
    if true and groupDecal == nil then
        groupDecal = createObject("SimGroup")
        groupDecal:registerObject('e_road')
        if U._PRD == 0 or U._MODE == 'conf' and scenetree.MissionGroup then
            scenetree.MissionGroup:addObject(groupDecal)
        end
        groupDecal = scenetree.findObject('e_road')
    end

    local groupLines = scenetree.findObject('lines')
    if true and groupLines == nil then
        groupLines = createObject("SimGroup")
        groupLines:registerObject('lines')
        if U._PRD == 0 then
            scenetree.MissionGroup:addObject(groupLines)
        end
        groupLines = scenetree.findObject('lines')
    end
end


local cpick -- selected road ID
local apick = {}
local askip = {} -- IDs of roads to skip
local cdesc, cdescmo -- selected description index
local croad = nil
local adec,aref = {},{}
-- number of static roads
local nstat
local cdec
local aedit = {}
local anodesel = {} -- selected nodes
local tb --= extensions.editor_terrainEditor.getTerrainBlock()
local L = tb and tb:getObjectBox():getExtents().x or nil
--local grid
local mask = {}
local ajunc = {}
local cjunc -- = 3 --44
    out.injunction = cjunc
local tersize, tfr, grid -- terrain params
local dupd = {}



local function foldersUp()
        lo('>> D.foldersUp:')
    groupDecal = scenetree.findObject('e_road')
    if true and groupDecal == nil then
        groupDecal = createObject("SimGroup")
        groupDecal:registerObject('e_road')
        if scenetree.MissionGroup then
--        if U._MODE == 'conf' and scenetree.MissionGroup then
            scenetree.MissionGroup:addObject(groupDecal)
        end
        groupDecal = scenetree.findObject('e_road')
        groupDecal.canSave = false
    end

    groupLines = scenetree.findObject('lines')
    if true and groupLines == nil then
        groupLines = createObject("SimGroup")
        groupLines:registerObject('lines')
        if U._PRD == 0 then
            scenetree.MissionGroup:addObject(groupLines)
        end
        groupLines = scenetree.findObject('lines')
        groupLines.canSave = false
    end
end


D.inject = function(inreload)
        lo('>>******************* D.inject:'..tostring(inreload)) --..tostring(iM))
    groupDecal = scenetree.findObject('e_road')
    foldersUp()
--[[
    if true and groupDecal == nil then
        groupDecal = createObject("SimGroup")
        groupDecal:registerObject('e_road')
        if U._MODE == 'conf' and scenetree.MissionGroup then
--        if U._PRD == 0 and scenetree.MissionGroup then
            scenetree.MissionGroup:addObject(groupDecal)
        end
        groupDecal = scenetree.findObject('e_road')
--                print('??^^^^^^ decal_E_ROAD_ADDED:')
        groupDecal.canSave = false
    end

    groupLines = scenetree.findObject('lines')
    if true and groupLines == nil then
        groupLines = createObject("SimGroup")
        groupLines:registerObject('lines')
        if U._PRD == 0 then
            scenetree.MissionGroup:addObject(groupLines)
        end
        groupLines = scenetree.findObject('lines')
        groupLines.canSave = false
    end
]]
    if U._PRD == 0 and not inreload then
    --    U.camSet({-127.07, 676.75, 188.31, 0.298212, -0.0179038, 0.0571938, 0.952616})
    --    U.camSet({414.02, 968.96, 125.14, 0.265359, 0.032627, -0.117591, 0.956396})
    --    U.camSet({70.69, 853.55, 117.71, -0.305298, -0.0838128, 0.251114, -0.914719})
    --    U.camSet({151.35, 1164.72, 138.39, 0.294969, 0.0378498, -0.121514, 0.946993})
    --    U.camSet({149.43, 1214.16, 136.47, 0.45275, 0.102006, -0.194688, 0.864123})
    --    U.camSet({-8.30, 1127.94, 137.68, 0.181908, 0.215401, -0.733011, 0.619037})
    --    U.camSet({-216.14, 1344.74, 135.28, 0.382675, 0.167956, -0.365115, 0.831891})
    --    U.camSet({-145.81, 1099.96, 131.14, -0.111989, -0.24528, 0.875976, -0.399954})
    --    U.camSet({-44.76, 795.81, 233.86, 0.445464, 0.281614, -0.454126, 0.718349})
    --    U.camSet({-18.87, 765.20, 211.53, -0.491272, -0.119161, 0.203383, -0.838503})
    --    U.camSet({-208.09, 720.14, 172.55, 0.358161, 0.320381, -0.584679, 0.653627})
    --    U.camSet({-203.82, 664.70, 174.44, -0.435658, -0.0441097, 0.0905614, -0.894458})
    --    U.camSet({-132.35, 560.43, 162.75, -0.0372739, 0.315831, -0.941549, -0.111118})
    --    U.camSet({-384.76, 622.43, 204.24, 0.241714, 0.142216, -0.48675, 0.827299})
    --    U.camSet({-698.42, 526.78, 632.25, 0.446083, 0.431727, -0.545217, 0.563347})
    --    U.camSet({-771.46, 712.64, 175.45, -0.177056, 0.0983056, -0.475364, -0.856164})
    --    U.camSet({1696.46, -942.70, 524.09, 0.013474, -0.308599, 0.950192, 0.0414863})
    -- mountain point
    --    U.camSet({-30.48, -693.50, 265.28, 0.30394, -0.0205816, 0.0643509, 0.950293})
    --    U.camSet({46.60, -601.12, 245.63, 0.190945, 0.0271396, -0.138075, 0.971462})
    -- city planning
--        U.camSet({109.45, -811.52, 327.03, -0.0737026, -0.208325, 0.919434, -0.325284})
--        U.camSet({38.11, -1643.56, 154.33, 0.170512, 0.260691, -0.795241, 0.520151})

    --!!    U.camSet({-93.74, -417.18, 349.95, 0.0691057, -0.0918969, 0.793939, 0.597026})
    --    U.camSet({-960.78, -33.85, 186.33, -0.257073, -0.0503236, 0.185398, -0.947105})
    --    U.camSet({339.13, 778.51, 255.28, 0.127376, 0.286722, -0.867733, 0.385493})
    --    U.camSet({-381.09, -13.67, 294.88, 0.371467, -0.125723, 0.294907, 0.871341})

--        U.camSet({-405.81, 253.16, 335.91, -0.181747, -0.332468, 0.812025, -0.443902})
        U.camSet({-405.81, 253.16, 335.91, 0.235263, -0.430966, 0.764645, 0.417417})

    --    U.camSet({52.50, 1063.48, 193.93, -0.0234219, -0.171844, 0.975823, -0.133005})
    --    U.camSet({7.95, 4.16, 4.54, -0.0890567, -0.144725, 0.839285, -0.516454})
    --    U.camSet({41.45, 19.20, 17.16, 0.0878348, 0.109668, -0.772779, 0.618927})
    end
end

--[[
local rdlist = editor.getAllRoads()
for id,_ in pairs(rdlist) do
    local obj = scenetree.findObjectById(id)
--        lo('?? if_ROAD:'..tostring(id)..':'..tostring(obj))
    local anode = editor.getNodes(obj)
    if obj:getNodeWidth(0) > 1.5 then
        local list = {}
        for i,n in pairs(anode) do
            list[#list+1] = n.pos
        end
        adec[#adec+1] = {list=list, body=obj, id=id}
    end
end
    lo('??+++++++++++++++++++++++++++++++++ ADEC:'..#adec)
]]

local function ind4id(id)
    for i,rd in pairs(adec) do
        if rd.id == id then
            return rd.ind
        end
    end
end


D.clear = function()
        lo('>> D.clear:'..tostring(U._MODE)) --..(nstat or 0))
    askip = {}
    if ({conf=0,ter=0})[U._MODE] then
        D.undo()
        for _,dec in pairs(adec) do
            local ind = dec.ind
            local rd = adec[ind]
            if rd and scenetree.findObjectById(rd.id) then
                if rd and U._PRD == 0 then
                    rd.body:setField('material', 0, 'WarningMaterial')
                end
                if rd then
                    rd.body:setPosition(rd.body:getPosition())
                end
            end
        end
        if groupDecal then
            groupDecal:deleteAllObjects()
        end
        if groupLines then
            local aobj = groupLines:getObjects()
--                lo('?? pre:'..tableSize(editor.getAllRoads())..':'..#scenetree.getAllObjects())
            for _,o in pairs(aobj) do
--                lo('??^^^^^^^^^^^^^^^^^^^^^ for_gl_obj:'..tostring(o))
                editor.deleteRoad(tonumber(o))
                editor.deleteObject(tonumber(o))
--                scenetree.findObjectById(tonumber(o)):delete()
--                o:delete()
            end
            groupLines:deleteAllObjects()
--                lo('?? post:'..tableSize(editor.getAllRoads())..':'..#scenetree.getAllObjects())
        end
        for ind,_ in pairs(dupd) do
            local rd = adec[ind]
            if rd and scenetree.findObjectById(rd.id) then
                if rd.listrad then
                    rd.list = deepcopy(rd.listrad)
                    D.nodesUpdate(rd)
                end
            end
        end
            lo('<< D.clear:'..#editor.getAllRoads())
        return
    end
    local adec = editor.getAllRoads()
--        lo('?? rem_from_ed:'..tostring(fedit)..':'..tostring(adec))
    for id,_ in pairs(adec) do
        local obj = scenetree.findObjectById(id)
--            lo('?? for_DEC:'..id..':'..tostring(obj))
        if obj then
            obj:delete()
        end
    end
end


local across = {} -- from_[rdi,ndi] -> {to_[rdi,ndi]}, rdi - index in adec
--    lo('??_______________________________________ DECAL:'..tostring(D.junctionUp))

local function forCross(dbgid)
    across = {}
    local gmax = math.ceil(2*L/grid)
        lo('>> forCross: L='..L..':'..gmax..':'..#aref)
            local nc = 0
    for i,row in pairs(aref) do
        for j,col in pairs(row) do
            local agrid = {}
            for _,a in pairs({-1,0,1}) do
                for _,b in pairs({-1,0,1}) do
                    if i+a < 1 or i+a > gmax or j+b < 1 or j+b > gmax then
                    else
                        agrid[#agrid+1] = {i+a,j+b}
                    end
                end
            end
            for rsrc,asrc in pairs(aref[i][j]) do
--                        if i == 526 and j == 355 then
--                            lo('?? for_SRC:'..rsrc)
--                        end
                for n = asrc[1],asrc[2] do
                    for _,g in pairs(agrid) do
                        if g == nil or g[1] == nil or g[2] == nil or aref[g[1]] == nil then
--                            U.dump(g, '!! ERR_nil:')
--                            return
                        end
                        if aref[g[1]] ~= nil and aref[g[1]][g[2]] ~= nil then
--                            U.dump(aref[g[1]][g[2]], '?? for_reff:'..tostring(g[1])..':'..tostring(g[2]))
--                            if true then return end
                            for rtgt,atgt in pairs(aref[g[1]][g[2]]) do
--                                        if i == 526 and j == 355 and g[1] == 526 and g[2] == 355 then
--                                            lo('?? for_TGT:'..rtgt..':'..atgt[1]..':'..atgt[2])
--                                        end
                                if rtgt ~= rsrc then
--                                        if rtgt ==  192 then
--                                            lo('??________ check_double:'..atgt[1]..'>'..atgt[2])
--                                        end
                                    for k = atgt[1],atgt[2] do
--                                            if rtgt == 362 and i == 526 and j == 355 and g[1] == 526 and g[2] == 355 then
--                                                lo('?? for_dist:'..n..'>'..k..':'..rsrc..'>'..rtgt..':'..tostring(adec[rsrc].list[n])..':'..tostring(adec[rtgt].list[k])..':'..(adec[rsrc].list[n] - adec[rtgt].list[k]):length())
--                                            end
                                        local d = (adec[rsrc].list[n] - adec[rtgt].list[k]):length()
                                        -- n: source node index
                                        if d < 1 then
--[[
                                                    if rsrc == 68 and rtgt == 192 then
                                                        lo('??_______ chd:'..rsrc..' n:'..n)
                                                    end
]]
                                                    nc = nc + 1
                                            if across[rsrc] == nil then
                                                across[rsrc] = {}
                                            end
                                            if across[rsrc][n] == nil then
                                                across[rsrc][n] = {}
                                            end
                                            local replaced = false
--[[
                                                    if rsrc == 68 and rtgt == 192 then
                                                        U.dump(across[rsrc][n], '??____ chd2:'..rsrc..':'..n)
                                                    end
]]
                                            for _,l in pairs(across[rsrc][n]) do
--                                                if rsrc == 68 and rtgt == 192 then
--                                                    lo('?? for_src:'..n)
--                                                end
                                                if l[1] == rtgt then
--??
                                                    replaced = true
                                                    if d < l[3] then
                                                        -- replace
                                                        replaced = true
                                                        l[2] = k
                                                        l[3] = d
                                                    end
                                                end
                                            end
                                            if not replaced then
                                                -- just append
                                                local toskip = false
--                                                    if rtgt == 192 and rsrc == 68 then
--                                                        lo('??____ appending:'..rsrc..':'..n..'>'..rtgt..':'..k..':'..d)
--                                                    end
                                                if true then --and rsrc == 68 then
                                                    for ni,al in pairs(across[rsrc]) do
--                                                                U.dump(al, '?? checking:'..ni)
                                                        for il,l in pairs(al) do
    --                                                                U.dump(l, '?? link_check:')
                                                            if l[1] == rtgt then
    --                                                            lo('?? check dist:'..rtgt..':'..l[2])
                                                                if d < l[3] then
--                                                                        U.dump(al, '?? to_remove:'..rtgt..':'..il)
                                                                    table.remove(al, il)
--                                                                        U.dump(across[rsrc], '?? removed:'..#al..':'..ni)
                                                                    if true then --and rsrc == 68 then
--                                                                        U.dump('?? removing:'..ni)
                                                                        if #al == 0 then
                                                                            across[rsrc][ni] = nil
--                                                                            across[rsrc]
--                                                                            table.remove(across[rsrc], ni)
                                                                        end
                                                                    end
    --                                                                        table.remove(across[rsrc], ni)
                                                                    break
                                                                else
                                                                    toskip = true
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                                if not toskip then
                                                    across[rsrc][n][#across[rsrc][n] + 1] = {rtgt, k, d}
--                                                        if rtgt == 232 and rsrc == 68 then
    --                                                        U.dump(across[rtgt], '??____ appended:'..rsrc..':'..n)
--                                                            U.dump(across[rsrc][n], '??____ appended:'..rsrc..':'..n)
--                                                        end
                                                end
                                            end
--                                            adec[rsrc].list[n].z = core_terrain.getTerrainHeight(adec[rsrc].list[n])

--                                            if i == 526 and j == 355 then
--                                                U.dump(across[rsrc][n], '?? S-T:'..rsrc..':'..rtgt)
--                                            end
--                                                out.avedit[#out.avedit + 1] = adec[rsrc].list[n]
                                        end
--                                        lo('?? for_ND:'..rsrc..':'..rtgt..':'..n..':'..k..':'..tostring(adec[rsrc][n])..':'..tostring(adec[rtgt][k]))
--                                        return
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    for i,r in pairs(adec) do
        for k,n in pairs(r.list) do
            adec[i].list[k].z = core_terrain.getTerrainHeight(adec[i].list[k])
        end
    end
--[[
    for _,p in pairs(out.avedit) do
        p.z = core_terrain.getTerrainHeight(p)
    end
]]
    out.across = across
        lo('<< forCross:'..tableSize(across)..':'..nc) --..':'..tostring(out.avedit[1])..':'..tostring(out.avedit[2]))
--        U.dump(across)
    return across
end


local function decalsLoad(list) --l, g, dbg)
--        lo('>> decalsLoad:')
--    if not g then g = 10 end
--    if l then L = l end
--    grid = g
    tb = extensions.editor_terrainEditor.getTerrainBlock()
--        print('?? decalsLoad:'..tostring(tb))
    if tb then
        L = tb and tb:getObjectBox():getExtents().x or nil
        tersize = tb:getObjectBox():getExtents()
        tfr = tb:getWorldBox().minExtents
        grid = tb:getSquareSize()
    end
    foldersUp()
    mask = {}
    local obj = scenetree.findObject("Vegetation")
    if U._PRD==0 and obj then
        obj:delete()
    end

--        if not list then return end

    local nrd
    if list then
        adec = list
        nrd = #adec
    else
        local rdlist = editor.getAllRoads()
            lo('>> decalsLoad:'..tostring(L)..' m/p=:'..tostring(g)..' N='..tableSize(rdlist)..':'..tostring(obj))
            local icheck
        if tableSize(rdlist) > 400 then return end
        nrd = 1
        adec = {}
    --    local n = 1
        for roadID,_ in pairs(rdlist) do
            local rd = scenetree.findObjectById(roadID)
    --                lo('?? PI:'..tostring(`rd and rd:getOrCreatePersistentID())) -- getField('material')) or) --rd.persistentId))
    --            local mt = rd.material -- rd:getField('material')
    --            lo('?? :'..rd.material)
            if rd then
                local mt = rd.material -- rd:getField('material')
    --                lo('?? rd:'..rd.material..':'..rd.__parent)
                if rd.material == 'road_invisible' then
    --                lo('?? mat_set:')
                    if U._PRD == 0 then
                        rd.material = 'WarningMaterial'
                    end
                end
                local nlist = editor.getNodes(rd)
                local anode = {}
                local aw = {}
                for _,n in pairs(nlist) do
                    anode[#anode+1] = vec3(n.pos.x,n.pos.y)
                    aw[#aw+1] = n.width
                end
    --                if roadID == dbg then
    --                    U.dump(anode, '?? for_NODES:'..dbg)
    --                end
                adec[#adec+1] = {id = roadID, body=rd, list = anode, aw = aw, ind=#adec+1}
    --                if rd:getOrCreatePersistentID() == 'c8a0aa18-8cec-4e3e-a549-4271e2d5545a' then
    --                if rd:getOrCreatePersistentID() == 'f3821e74-24c6-4630-b32b-9d28e0a5cd4b' then
    --                    icheck = #adec
    --                    lo('?? check_ID:'..icheck)
    --                end
                    if U._MODE == 'conf' and U._PRD == 0 then
                        rd:setField('material', 0, 'WarningMaterial')
                    end

                    if U._PRD == 0 and rd:getOrCreatePersistentID() == 'cec5c8fc-862f-4e55-8787-b1a36c8a8311' then
                        lo('??__________ for_CHECK2:'..adec[#adec].ind)
                        --icheck = rd:getID() -- #adec --
    --                    icheck = adec[#adec].ind
                        rd:setField('hidden',0,'false')
                    end
    --            rd:setField('hidden',0,'false')
                nrd = nrd + 1
            end
        end
        nstat = #adec
    end

    aref = {}
    for ir,r in pairs(adec) do
        if r.skip or #U.index(askip, r.id) > 0 then goto cont end
        if U._PRD == 0 then
            r.body:setField('hidden',0,'false')
        end
--            if list then
--                lo('?? for_LR:'..ir..':'..#r.list..':'..tostring(r.isline))
--            end
--            if r.id == dbg then
--                lo('??^^^^^^^^^^^^^^^^^ dL_CHECK:'..r.id..':'..r.ind..':'..tostring(r.skip)..':'..#r.list..':'..ir)
--            end
        for k,n in pairs(r.list) do
--                nn = nn + 1
            local i,j = math.floor((n.y + L)/grid) + 1,math.floor((n.x + L)/grid) + 1
            if aref[i] == nil then
                aref[i] = {}
            end
            if aref[i][j] == nil then
                aref[i][j] = {}
            end
            if aref[i][j][ir] == nil then
                aref[i][j][ir] = {math.huge, 0} -- nodes index interval
--                    lo('?? for_ir:'..nn..' i:'..i..' j:'..j..' ir:'..ir..':'..#aref[i][j][ir]..':'..#aref[i][j])
            end
--                if #aref[i][j] > 0 then
--                    lo('?? for_ref:'..i..':'..j..':'..#aref[i][j]..' n:'..tostring(n)..':'..ir)
--                end
            if k < aref[i][j][ir][1] then aref[i][j][ir][1] = k end
            if k > aref[i][j][ir][2] then aref[i][j][ir][2] = k end
        end
        ::cont::
    end
--        local n = adec[231].list[1]
--        local i,j = math.floor((n.y + L)/grid) + 1,math.floor((n.x + L)/grid) + 1
--        U.dump(aref[526][355], '???*************** aref:'..tostring(aref[526])..':'..tostring(n)..':'..i..':'..j)
--        lo('?? pos:'..tostring(adec[302].list[1])..':'..tostring(adec[49].list[1])..':'..tostring(adec[362].list[3])..':'..tostring(adec[49].list[1]))
    forCross() --dbg)
-- junctions
        lo('?? for_across:'..tableSize(out.across)..':'..#adec)
--        U.dump(across)
--        D.toMark(adec[2].list,'cyan',nil,0.1,1)
    local djunc = {}
    ajunc = {}
            out.acyan = {}
    for ind,d in pairs(out.across) do
        for k,c in pairs(d) do
            local abranch = {ind}
            local abr = {adec[ind]}
            for i,b in pairs(c) do
                abranch[#abranch+1] = b[1]
                abr[#abr+1] = adec[b[1]]
--                    lo('?? to_CSTAR:'..ind..':'..i..':'..b[1]..':'..tostring(adec[b[1]].ind)..':'..tostring(adec[b[1]].isline))
            end
            local stamp = U.stamp(abranch)
--                lo('?? for_stamp:'..ind..':'..k..':'..stamp)
--                U.dump(c, '?? for_a:'..ind)
            if not djunc[stamp] then
                djunc[stamp] = 1
--                U.dump(c, '?? for_end:'..ind..':'..k) --..':'..c[1][1])
                ajunc[#ajunc+1] = {p=adec[ind].list[k], list=abr}
                -- sort junction star
                table.sort(ajunc[#ajunc].list, function(a, b)
                    local dira,dirb
                    local p = ajunc[#ajunc].p --adec[ind].list[k]
                    if not a.list or not b.list then return false end
                    dira = (a.list[1]:distance(p)<1 and -- distance is not actually small_dist
                        a.list[2]-a.list[1] or
                        a.list[#a.list-1] - a.list[#a.list]):normalized()
                    dirb = (b.list[1]:distance(p)<1 and
                        b.list[2]-b.list[1] or
                        b.list[#b.list-1] - b.list[#b.list]):normalized()
                    return U.vang(U.proj2D(dira),vec3(1,0),true) > U.vang(U.proj2D(dirb),vec3(1,0),true)
                end)
--[[
                if #ajunc == 11 then
                        D.toMark({ajunc[#ajunc].p},'mag',nil,0.2,1)
                    U.dump(c, '?? for_CROSS8:'..ind..':'..k)
                        for t,b in pairs(ajunc[#ajunc].list) do
                            local p = ajunc[#ajunc].p
                            local dirb = U.proj2D(U.proj2D(b.list[1]):distance(U.proj2D(p))<U.small_dist and
                                b.list[2]-b.list[1] or
                                b.list[#b.list-1] - b.list[#b.list]):normalized()
                                lo('?? for_BB:'..b.ind..':'..tostring(b.list[1])..':'..U.proj2D(b.list[1]):distance(U.proj2D(p))..':'..tostring(dirb)..':'..tostring(p)..':'..U.vang(dirb,vec3(1,0),true))
                            if t == 1 then

                            end
                        end
--                    lo('?? for_J8:'..#abr)
                end
]]
                for i,b in pairs(abr) do
                    if ajunc[#ajunc].p:distance(b.list[1]) < 1 then
--                                lo('?? is_FR:')
                        b.jfr = #ajunc
                    else
--                                lo('?? isTO:')
                        b.jto = #ajunc
                    end
--                            if b.ind == 19 then
--                                lo('??^^^^^^^^^^^^ for_BR_19:'..#ajunc..':'..b.ind..':'..ajunc[#ajunc].p:distance(b.list[1])..':'..tostring(b.jfr)..':'..tostring(b.jto))
--                            end
                end
--                if #ajunc == 24 then
--                    U.dump(ajunc[#ajunc].list, '?? for_JUNC:')
--                end
            end
        end
--        lo('?? forc:'..tostring(ind)..':'..tostring(c[1]))
--        U.dump(c, '?? forc1:'..ind)
    end
        lo('?? ajunc:'..#ajunc) --..':'..tostring(ajunc[1].p))
--        U.dump(ajunc, '?? for_ajunc:')
--            icheck = 32 --157
--            D.road2ter(0) --{icheck})
--            D.decalSplit(adec[50], vec3(-139,-427,113.6))
--    adec,aref = decalsLoad('/levels/ce_test/main/MissionGroup/decalroads/AI_decalroads/items.level.json', 4096, 10)
    out.adec = adec
        if U._PRD == 0 then
--            if cjunc then D.onVal('b_junction') end
--            D.decalPlot()
        end
--        U.dump(adec[10].list, '?? list:')
        lo('<< decalsLoad:'..(nrd-1)..':'..#across..' adec:'..#adec..'/'..tableSize(editor.getAllRoads())..' aref:'..#aref..':'..tableSize(aref)) --..':'..tostring(adec[49].list[1]))
--        U.camSet({-497.75, 315.61, 227.73, -0.10119, 0.277522, -0.897572, -0.327271})
--        U.camSet({-129.04, 608.38, 193.10, 0.208937, 0.0198323, -0.0923885, 0.973353})

--        U.camSet({-127.07, 676.75, 188.31, 0.298212, -0.0179038, 0.0571938, 0.952616})

        --[-127.07, 676.75, 188.31, 0.298212, -0.0179038, 0.0571938, 0.952616]
        --[-129.04, 608.38, 193.10, 0.208937, 0.0198323, -0.0923885, 0.973353]
    return adec,ajunc,across,aref
end
--[[
                    lo('?? to_list:'..(#adec+1)..':'..tostring(list and list[#adec+1].isline or nil))
    --             = list and list[nrd] or {id = roadID, list = anode, aw = aw, ind=#adec+1}
    --            desc.list = anode
    --            desc.body = rd
    --            desc.id = roadID
    --            desc.aw = aw
                local desc
                if list then
                    adec[list[nrd].ind] = desc
                else
    --                desc.ind = #adec+1
                end
    if list then
        for _,desc in pairs(list) do
            rdlist[desc.id] = true
                if desc.isline then
                    lo('?? dL_line:'.._)
                end
        end
    else
        rdlist = editor.getAllRoads()
    end
    for id,_ in pairs(rdlist) do
        local obj = scenetree.findObjectById(id)
        if not obj then
        end
    end
]]
--[[
        if U._PRD==0 and icheck then
            D.decalSplit(adec[icheck], vec3(-139,-427,113.6))
--            adec[icheck].body:setField('hidden',0,'true')
--            adec[icheck].body:setField('hidden',0,'true')
--            adec[icheck].skip = true
                lo('?? hidden_set:'..icheck)
        end
]]
--        D.decalSplit(adec[icheck], vec3(-139,-427,113.6))
--[[
    if false and U._PRD == 0 then
        local ifr,ito,iex=42,31,47
--        local ifr,ito,iex=40,36,29
        adec[ifr].aexo = {adec[iex]}
        adec[ito].aexi = {adec[iex]}
        adec[iex].frto = {ifr,ito}
    end
]]
--[[
                table.sort(ajunc[#ajunc].list, function(a, b)
                    local dira,dirb
                    local p = adec[ind].list[k]
                    dira = (a.list[1]:distance(p)<U.small_dist and
                        a.list[2]-a.list[1] or
                        a.list[#a.list-1] - a.list[#a.list]):normalized()
                    dirb = (b.list[1]:distance(p)<U.small_dist and
                        b.list[2]-b.list[1] or
                        b.list[#b.list-1] - b.list[#b.list]):normalized()
                    return U.vang(dira,vec3(1,0),true) > U.vang(dirb,vec3(1,0),true)
                end)
]]

--                    out.acyan[#out.acyan+1] = adec[ind].list[k]
--                djunc[stamp] = 0
--                ajunc[#ajunc+1] = {p=c[1]}
--[[
                if false then
                    local L,ne = D.roadLength({body=rd})
                    local nn = math.floor(L/5+0.5)
                    local step = L/nn
        --                U.dump(list, '??+++++++++ rd_check:'..roadID..':'..ne..' L:'..L..':'..nn..':'..step)
                    local newlist = {list[1].pos}
                    out.avedit = {}
                    for i=1,nn-1 do
        --                    lo('?? forNode:'..i)
                        local p = D.onSide({body=rd}, 'middle', i*step)
                        newlist[#newlist+1] = p
        --                            out.avedit[#out.avedit+1] = p
                    end
                    newlist[#newlist+1] = list[#list].pos
                    D.nodesUpdate({body=rd, w=list[1].width}, newlist)
                end
--                    out.avedit = newlist
--                    U.dump(newlist, '?? nodes:')
                if false and rd:getOrCreatePersistentID() == 'f3821e74-24c6-4630-b32b-9d28e0a5cd4b' then
--                if false and rd:getOrCreatePersistentID() == 'cc833153-1735-4c3f-8a4a-2153923f4a94' then
--                    local ne = rd:getEdgeCount()
                    local L,ne = D.roadLength({body=rd})
                    local nn = math.floor(L/5+0.5)
                    local step = L/nn
                        U.dump(list, '??+++++++++ rd_check:'..roadID..':'..ne..' L:'..L..':'..nn..':'..step)
                    local newlist = {list[1].pos}
                    out.avedit = {}
                    for i=1,nn-1 do
                            lo('?? forNode:'..i)
                        local p = D.onSide({body=rd}, 'middle', i*step)
                        newlist[#newlist+1] = p
--                            out.avedit[#out.avedit+1] = p
                    end
                    newlist[#newlist+1] = list[#list].pos
                            out.avedit = newlist
                            U.dump(newlist, '?? nodes:')
                    D.nodesUpdate({body=rd, w=list[1].width}, newlist)
                end
]]


-- distance of node to rode start
local function node2dist(rd)
    local d = 0
    local ps = rd.list[1]
    local adist = {0}
    for i=2,#rd.list do
        adist[#adist+1] = adist[#adist] + rd.list[i]:distance(rd.list[i-1])
    end
    return adist
end


local function ecurv(rd, i)
    local a,b,c = D.epos(rd.body, i-1),D.epos(rd.body, i),D.epos(rd.body, i+1)

    return U.vang(c-b, b-a, true)/a:distance(c)
end


local function edge2dist(rd, dbg)
    local d = 0
    local ne = rd.body:getEdgeCount()
    local e2d = {}
    e2d[0] = 0
    local acurve = {{d=0,curv=0}}
    for i = 1,ne-1 do
        d = d + D.epos(rd.body,i-1):distance(D.epos(rd.body, i))
--        d = d + rd.body:getMiddleEdgePosition(i-1):distance(rd.body:getMiddleEdgePosition(i))
        e2d[i] = d
        if i<ne-1 then
            acurve[#acurve+1] = {d=d, curv=ecurv(rd,i)}
            if (D.epos(rd.body,i+1)-D.epos(rd.body,i)):dot(D.epos(rd.body,i)-D.epos(rd.body,i-1)) < 0 then
                -- decal corrupted
                lo('!! edge2dist_CORRUPT:')
                return
            end
        else
            acurve[#acurve+1] = {d=d, curv=0}
        end
    end
    rd.L = e2d[#e2d]

    return e2d,ne,acurve
end


local function forZ(p)
    return core_terrain.getTerrainHeight(p)
end


local legend = {
    red = {ColorF(1,0,0,0.3), 0.04, 0.3},
    green = {ColorF(0,1,0,0.3), 0.04, 0.3},
    mag = {ColorF(1,0,1,0.3), 0.04, 0.3},
    cyan = {ColorF(0,1,1,0.3), 0.04, 0.3},
    yel = {ColorF(1,1,0,0.3), 0.04, 0.3},
    white = {ColorF(1,1,1,0.3), 0.1, 1},
    blue = {ColorF(0,0,1,0.3), 0.04, 0.3},
}

local function toMark(list, cname, f, r, op, keepz)
    if not list then return end
    legend[cname][2] = r or 0.04
    legend[cname][3] = op or 0.3
    out[cname] = {}
    for i,p in pairs(list) do
        local v = f and f(p) or p
--            lo('?? for_v:'..tostring(v))
--            if cname == 'red' then
--                lo('??*************** toMark:'..tostring(v)..':'..tostring(keepz)..':'..tostring(forZ(v)))
--            end
        if v then
            v = vec3(v.x,v.y,v.z)
            if not keepz then
                v.z = forZ(v)
            end
            out[cname][#out[cname]+1] = v
        end
    end
--        lo('<< toMark:'..cname..':'..#out[cname])
end
D.toMark = toMark


local margin = 8 -- merge to background width
local jmargin = 10
local maxslope = math.pi/9
local shmap = {}
if tb then
    tersize = tb:getObjectBox():getExtents()
    tfr = tb:getWorldBox().minExtents
    grid = tb:getSquareSize()
        lo('?? if_TB: size:'..tostring(tersize)..':'..forZ(vec3(-tersize.x/2+25,0.2))..' wb:'..tostring(tb:getWorldBox())..':'..tostring(tfr))
--            U.dump(extents, '?? for_TB:'..tostring(tb)..' sz:'..tostring(tb:getSquareSize()))
else
    lo('!!____________________________________________ ERR_NO_TB:')
end

local function p2grid(p)
--        local i = math.floor((p.y - tfr.y)/grid+0.5) + 1
    return math.floor((p.x - tfr.x)/grid+0.5) + 1,math.floor((p.y - tfr.y)/grid+0.5) + 1
end
local function grid2p(ij)
    return vec3(tfr.x + (ij[2]-1)*grid, tfr.y + (ij[1]-1)*grid)
end


local function forSquare(c, r, f)
    local jc,ic = p2grid(c)
    local ri = math.floor(r/grid+0.5)
    local arr = {}
    for i=ic-ri,ic+ri do
        if not f then
            arr[i] = {}
        end
        for j = jc-ri,jc+ri do
            if f then
                f(i,j,ic,jc)
            else
                arr[i][#arr[i]+1] = j
            end
        end
    end
    return arr
end


local aconf
local crossset = {}

D.undo = function(silent)
        lo('?? D.undo:'..tableSize(adec)..'/'..#adec..':'..tableSize(mask)..':'..tostring(silent))
    for ij,d in pairs(mask) do
--            U.dump(ij, '?? undo_ij:'..tostring(grid2p(ij))..':'..tostring(d[1]))
        tb:setHeightWs(grid2p(d[4]), d[1]) -- 107) -- d[1])
    end
    mask = {}
    crossset = {}
    if not silent and tb then
        tb:updateGrid()
        for i,rd in pairs(adec) do
            local obj = scenetree.findObjectById(rd.id)
            if obj then
                obj:setPosition(obj:getPosition())
            end
        end
        out.inconform = false
    end
end


D.del = function(ind)
    local rd = adec[ind]
    adec[ind] = nil
    if not rd then return end
    local obj = scenetree.findObjectById(rd.id)
    if obj then
        rd.body:delete()
    end
--        lo('<< del:'..tableSize(adec)..':'..#adec)
end


local function terSet(ij, h, fix, dbg)
    local ijs = U.stamp(ij, true)
    if mask[ijs] and mask[ijs][3] == 0 then return end

    local p = grid2p(ij)
    if not mask[ijs] or fix == 0 then
        local shight = mask[ijs] and mask[ijs][1] or core_terrain.getTerrainHeight(p)
        mask[ijs] = {shight, h, fix or 1, ij}
    else
        local cn = mask[ijs][3]<0 and 0 or mask[ijs][3]
        mask[ijs][2] = (mask[ijs][2]*cn + h)/(cn + 1)
        mask[ijs][3] = cn + 1
    end
    if not dbg then
        tb:setHeightWs(p, mask[ijs][2])
    end

    return ijs
end


local function terSet_(ij, h, fix, dbg)
    local ijs = U.stamp(ij, true)
    if mask[ijs] and mask[ijs][3] == 0 then return end
    local p = grid2p(ij)
    if not mask[ijs] then
        mask[ijs] = {core_terrain.getTerrainHeight(p), h, 1, ij}
    elseif fix then
        mask[ijs][2] = h
        mask[ijs][3] = fix
        mask[ijs][4] = ij
    else
        -- superpose with existing
--            lo('?? terSet_mask:')
        local cm = mask[ijs][3]<0 and 0 or mask[ijs][3]
        mask[ijs][2] = (mask[ijs][2]*cm + h)/(cm + 1)
        mask[ijs][3] = cm + 1
    end
--    if fix ~= -1
--    if fix then mask[ijs][3] = 0 end
    if not dbg then
        tb:setHeightWs(p, mask[ijs][2])
    end

    return ijs
end


-- flatten area around cross center
local function terFlat(c, r, m, dbg)
    if not m then m = jmargin end
--        r = r*2/4
    local aset = {}
    local hc,h = forZ(c)
        local aout = {}
        lo('?? terFlat:'..r..':'..m)
    forSquare(c, r + m, function(i, j, ic, jc)
        local d = vec3(i, j):distance(vec3(ic,jc))*grid
        if d <= r+m then
            local fix
            if d > r then
                -- fade out
                fix = -1
                h = U.spline2(hc, forZ(grid2p({i,j})), d-r, m)
            else
                -- set height to h
--                fix = 0
                h = hc
            end
            aset[#aset+1] = terSet({i,j}, h, fix, dbg)
--                aout[#aout+1] = grid2p({i,j})
--            mask
--            tb:setHeightWs(p, h)
        end
    end)

    return aset
--        toMark(aout, 'white')
end



local dhist = {}

local function road2ter(ilist, precomp, dbg)
--        dbg = true
--        dbg = false
--        dbg = cdesc == 170
        if ilist == 0 then
--            ilist = {170}
--            return
        end
        if ilist == 0 then return end
        lo('>> road2ter:'..tostring(ilist and #ilist or nil)..':'..tostring(out.inall)..':'..#apick..':'..tostring(cdesc))
    if not ilist then
        if dbg then lo('?? road2ter_ifind:'..tostring(cpick)..':'..tostring(croad)..':'..tostring(cdesc)..':'..#apick) end
        if cdesc then
            ilist = {cdesc}
        elseif #apick > 0 then
            ilist = {}
            for _,ind in pairs(apick) do
                ilist[#ilist+1] = ind
            end
        else
            -- conform all
            ilist = {}
--            return
        end
    end
        lo('?? r2t_list:'..#ilist)
    out.acirc = {}
    local list = {}
    aconf = deepcopy(ilist)
    for _,ind in pairs(ilist) do
        list[#list+1] = adec[ind]
    end
    if #list == 0 and out.inall then
        list = adec
            lo('?? for_ALL:'..#list)
    end
        if #list == 1 then
--            dbg = true
--                lo('?? road2ter_one:'..list[1].ind..':'..tostring(dbg))
        end
--        list = adec
--        dbg = false
--        lo('?? for_LIST:'..#list)
    local r2w = 2 -- ratio of juct flat radius to max branch width

    local askip = {}
    for _,dec in pairs(list) do
--            dbg = dec.ind == 142 and true or nil
        local pinstep = 30 -- distance between pin points
--    for _,dec in pairs(adec) do
        local ind = dec.ind
--    for _,ind in pairs(ilist) do

        -- get intervals pinned to jusntions
        local rd = adec[ind]
        local bank = default.bank
        rd.L,rd.ne,rd.acurve = D.roadLength(rd)
        local jrad -- radius of  junction flat area
        rd.djrad = {} -- node -> flattening radius
        local andist = node2dist(rd)
--        local n2e = D.node2edge(rd.body)
        rd.apin = {}
        rd.across = {} -- node > ind,pos,dir
        local itofill = 3
    --    local h
        local function forDir(rd, ie, dir)
    --            lo('?? for_dir:'..rd.ind..':'..ie..':'..dir)
    --            if dir < 0 then
    --                out.acyan = {D.epos(rd.body, ie)}
    --            end
            return (D.epos(rd.body, ie+dir) - D.epos(rd.body, ie)):normalized()
        end

        local bcross = across[ind]
            lo('?? conf_road:'.._..':'..ind..':'..tostring(adec[ind].list[1])..':'..tableSize(bcross))
        if bcross then
                out.avedit = {}
                if dbg then U.dump(bcross, '?? road2ter:'..ind..':'..adec[ind].id..':'..tableSize(bcross)..' w:'..tostring(rd.aw and rd.aw[1] or rd.w)) end
            --- go over cross points on the branch
            local ccross
            for i,ref in pairs(bcross) do
                ccross = i
    --                lo('?? for_I:'..i)
    --                U.dump(ref, '?? for_end:'..i)
        --        ref.dir = i==1 and 1 or -1
                local abranch = {{ird=ind, inode=i}} --{{ind,i}}
                ---- go over branches of junction
                ---- build junction stamp
                local ai = {ind}
                for _,b in pairs(ref) do
                    ai[#ai+1] = b[1]
                end
                local stamp = U.stamp(ai)..(#abranch == 1 and '_'..i or '')
                local c = adec[ind].list[i]
                if c then
        --                lo('??++++++++++++++++++ cross_STAMP:'..i..':'..stamp)
                    jrad = 0
                    for j,b in pairs(ref) do
            --                U.dump(b, '?? for_B:'..tostring(j))
            --                lo('?? for_B2:'..tostring(isnan(b))..':'..tostring(j)..':'..tostring(b))
            --                U.dump(ep, '?? for_ep:')
                        abranch[#abranch+1] = {ird=b[1], inode=b[2]}
                        c = c + adec[b[1]].list[b[2]]
                        if adec[b[1]].aw[b[2]] > jrad then
                            jrad = adec[b[1]].aw[b[2]]/2 * r2w
                        end
            --            for k,b in pairs(ep) do
            --                abranch[#abranch+1] = {b[1],b[2]}
            --            end
            --            ::continue::
                    end
            --            U.dump(ref, '?? for_jrad:'..i..':'..jrad)
                    c = c/#abranch
                else
                    break
                end
                local hc = forZ(c)
                if #abranch == 1 then
                    bank = 0
                end
                if not jrad or jrad == 0 then jrad = default.rexit end
                -- flatten junction area
--                        jrad = 15
                rd.jrad = jrad
    --            if not crossset[stamp] then crossset[stamp] = {} end
    --            if not precomp and #crossset[stamp] < 2 then
                if (not precomp and not crossset[stamp]) or #abranch == 1 then
                    if cjunc then
                        jrad = ajunc[cjunc].r or jrad
                    end
                        if true or dbg then lo('??^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ pre_flat:'..i..':'..tostring(jrad)..':'..tostring(c)) end
                    local aset = terFlat(c, jrad, nil) --, dbg ~= nil)
                    crossset[stamp] = aset
                        if U._PRD == 0 then
                            out.acirc[#out.acirc+1] = {p=c+vec3(0,0,0), r=jrad}
                            out.acirc[#out.acirc+1] = {p=c+vec3(0,0,0), r=jrad+jmargin}
                        end
                end
    --[[
                if true then
    --                local stamp = U.stamp({ind,i},true)

                    if #U.index(crossset, stamp) == 0 then
                        crossset[#crossset+1] = stamp
                        local aset = terFlat(c, jrad, nil, dbg ~= nil)
                        if dbg then
                            lo('?? to_flat:'..i..':'..tostring(jrad)..':'..tableSize(mask))
                        end
                    else
    --                    lo('??+++++++++++++++ has_cross:'..stamp)
                    end
                    for j,b in pairs(ref) do
                        stamp = U.stamp({b[1],b[2]},true)
                        if #U.index(crossset, stamp) == 0 then
    --                        crossset[#crossset+1] = stamp
                        end
                    end
    --                    U.dump(crossset, '?? crossset:')
                end
    ]]
        --            out.acyan = {c+vec3(0,0,forZ(c))}
                rd.apin[#rd.apin+1] = {d = i == 1 and 0 or rd.L, h=hc, fix=true}
                rd.across[#rd.across+1] = {inode=i, p=c, dir=i==1 and 1 or -1}
                    if dbg then
                        out.avedit[#out.avedit+1] = c
    --                    out.acyan = {c}
                        lo('?? for_c:'..i..':'..tostring(c))
                    end
        --            U.dump(abranch, '?? for_end_br:'..i)
        --            U.dump(n2e)
                --- get branches sides crossing
                local pinext,r = 0
                if true then
        --            for j,b in pairs(abranch) do
                    local dir0
                    local aang = {}
                    for j=1,#abranch do
                        local dir
                        local b = abranch[j]
                        --- branch direction
                        local dir = forDir(adec[b.ird], b.inode==1 and 0 or adec[b.ird].body:getEdgeCount()-1, b.inode == 1 and 1 or -1)
                        if j==1 then
                            dir0 = dir
                        end
                        aang[#aang+1] = {ind=b.ird, ang=U.vang(dir0, dir, true), dir=b.inode==1 and 1 or -1}
        --                        lo('?? for_c_b:'..i..':'..j..':'..tostring(dir))
        --                    out.avedit[#out.avedit+1] = c+dir*5
    --- find ADJACENT branches
                    end
                    table.sort(aang, function(a,b)
                        return a.ang < b.ang
                    end)
        --                U.dump(aang, '?? for_AANG:'..i)
                    pinext = 0
                    local pc
                    for k,d in pairs(aang) do
                            if dbg then U.dump(d, '?? for_ang:'..k) end
                        if d.ang == 0 then
                            -- look for ajacent
                            if k > 1 and math.abs(aang[k-1].ang) < math.pi/2 then
                                --- right neighbour
                                pc = D.borderCross(rd.body, adec[aang[k-1].ind].body, d.dir==1 and 'right' or 'left', aang[k-1].dir==1 and 'left' or 'right', d.dir==1 and 0 or 1, aang[k-1].dir==1 and 0 or 1)
                                    if dbg then lo('??++++++++++++ cross_right:'..k..':'..tostring(pc)..':'..d.dir..':'..tostring(aang[k-1].dir)..':'..tostring(aang[k-1].ind)) end
        --                            out.acyan = {pc+vec3(0,0,forZ(pc))}
        --                            out.acyan = {D.epos(rd.body, d.dir==-1 and rd.body:getEdgeCount()-1 or 0, d.dir==1 and 'right' or 'left')}

        --                            out.avedit = {c}
                                if pc then
                                    r = pc:distance(D.epos(rd.body, d.dir==-1 and rd.body:getEdgeCount()-1 or 0, d.dir==1 and 'right' or 'left'))
            --                                lo('?? for_r:'..r)
            --                        pc = pc + vec3(0,0,core_terrain.getTerrainHeight(pc))
                                    if r > pinext then
                                        pinext = r
                                    end
                                end
                            end
                            if k < #aang and math.abs(aang[k+1].ang)<math.pi/2 then
                                    if dbg then U.dump(aang[k+1], '?? b_left:') end
                                pc = D.borderCross(rd.body, adec[aang[k+1].ind].body, d.dir==-1 and 'right' or 'left', aang[k+1].dir==-1 and 'left' or 'right', d.dir==1 and 0 or 1, aang[k+1].dir==1 and 0 or 1)
    --                            pc = D.borderCross(rd.body, adec[aang[k+1].ind].body, d.dir==-1 and 'right' or 'left', aang[k+1].dir==-1 and 'left' or 'right', d.dir==1 and 0 or 1, aang[k-1].dir==1 and 0 or 1)
                                if pc then
                                    r = pc:distance(D.epos(rd.body, d.dir==-1 and rd.body:getEdgeCount()-1 or 0, d.dir==-1 and 'right' or 'left'))
            --                        pc = pc + vec3(0,0,core_terrain.getTerrainHeight(pc))
            --                            out.avedit = {c + forZ(c)} -- vec3(0,0,core_terrain.getTerrainHeight(c))}
                                    if r > pinext then
                                        pinext = r
                                    end
                                end
        --                            out.avedit = {pc, D.epos(rd.body, d.dir==-1 and rd.body:getEdgeCount()-1 or 0, d.dir==-1 and 'right' or 'left')}
                                    if dbg then lo('??+++++++++++++++++++ cross_left:'..tostring(c)..':'..r..':'..d.dir) end
        --                            out.acyan = {c, D.epos(rd.body, d.dir==-1 and rd.body:getEdgeCount()-1 or 0, d.dir==-1 and 'right' or 'left')}
                            end
                            break
        --                        out.avedit = rd.apin
                        end
                    end
        --                U.dump(aang, '?? aang:')
                end
        --            lo('??________________________________ pe_r:'..i..':'..pinext..'/'..jrad)
    -- PIN ENDS
    --                lo('??^^^^^^^^^^ for_RAD:'..i..':'..rd.ind..':'..jrad..'/'..pinext)
                rd.djrad[i] = math.max((rd.w or 0)/2,pinext)
    --            rd.djrad[i] = math.max(jrad,pinext)

                local rma = math.max(pinext,jrad)
                if rma == 0 then rma = default.rexit end
                rd.apin[#rd.apin+1] = {d = i==1 and rma/2 or (rd.L - rma/2), h=hc, fix=true}
                rd.apin[#rd.apin+1] = {d = i==1 and rma or (rd.L - rma), h=hc, fix=true}
                        if dbg then lo('?? for_ap:'..rd.ind..':'..#rd.apin) end
--[[
                if pinext > jrad then
                    rd.apin[#rd.apin+1] = {d = i==1 and pinext or (rd.L - pinext), h=hc, fix=true}
                else
                    rd.apin[#rd.apin+1] = {d = i==1 and jrad or (rd.L - jrad), h=hc, fix=true}
                end
]]
        --        inpre = i
            end
--                lo('??___________ if_PREC:'..#list..':'..tostring(rd.body)..)
            if precomp and #list == 1 then
                -- compute jrad only
                rd.body:setField('material', 0, rd.mat or default.mat)
                editor.updateRoadVertices(rd.body)
                return
            end
                if dbg then U.dump(rd.djrad, '?? for_len:'..jrad..'/'..rd.L) end
            if 2*(jrad+jmargin)>rd.L then
                -- road too short
                    lo('?? too_SHORT:'..rd.ind)
                askip[#askip+1] = rd.ind
                goto continue
            end

            if tableSize(bcross) == 1 then
    --                lo('?? for_bc:'..ccross)
                if ccross > 1 then itofill = 1 end
                -- open end, last node
                rd.apin[#rd.apin+1] = {d=ccross > 1 and 0 or rd.L, h=forZ(ccross>1 and rd.list[1] or rd.list[#rd.list]), fix=true}
            end
            table.sort(rd.apin, function(a,b)
                return a.d < b.d
            end)
        --        U.dump(rd.apin, '?? apin:'..rd.L..':'..jrad)

        --        U.dump(bcross, '?? road2ter:'..ind..':'..adec[ind].id)
        --            U.dump(rd.across, '?? for_rdcross:'..#rd.across..':'..core_terrain.getTerrainHeight(rd.across[1])..':'..core_terrain.getTerrainHeight(rd.across[3]))
            for i=2,#rd.across do
                -- evaluate slope
        --            U.dump(rd.across[i], '?? forAC:')
        --            lo('?? for_DH:'..(core_terrain.getTerrainHeight(rd.across[i][2])-core_terrain.getTerrainHeight(rd.across[i-1][2])))
                if math.abs((core_terrain.getTerrainHeight(rd.across[i].p)-core_terrain.getTerrainHeight(rd.across[i-1].p))/(andist[rd.across[i].inode]-andist[rd.across[i-1].inode])) > math.tan(maxslope) then
                    lo('!! high_SLOPE:'..ind..':'..rd.id)
                    askip[#askip+1] = ind
                    goto continue
                end
            end
    --            U.dump(out.yel, '?? pre_height:'..#rd.apin..':'..#out.yel)
        else
--                dbg = true
                U.dump(rd.frtoside, '??+++++++++++++++++++++++++++++++ for_FRTO:'..rd.L..':'..forZ(rd.frtoside[1].p))
--                U.dump(rd.apin, '?? ex_APIN:')
--                local cs = D.borderCross(rd.body, adec[rd.frto[1]].body, 'left', 'left', 0, 0)
--                local dn,p = D.toSide(cs, rd)
--                toMark({cs, p}, 'red')
            rd.apin[#rd.apin+1] = {d=0, h=forZ(rd.list[1])}
            -- middle
            local p = D.onSide(rd, 'middle', rd.frtoside[1].d/2)
            rd.apin[#rd.apin+1] = {d=rd.frtoside[1].d/2, h=forZ(p)}
--                U.dump(rd.apin, '?? for_PIN:')
--                toMark(rd.apin, 'blue', nil, 0.1, 0.4)
--                toMark({rd.apin}, 'blue', nil, 0.1, 0.4)
            rd.apin[#rd.apin+1] = {d=rd.frtoside[1].d, h=forZ(rd.frtoside[1].p)} -- rd.frtoside[1].p

--            rd.apin[#rd.apin+1] = {d=rd.frtoside[2].d, h=forZ(rd.frtoside[2].p)} --rd.frtoside[2].p
            rd.apin[#rd.apin+1] = {d=rd.frtoside[2].d, h=forZ(rd.frtoside[2].p)} --rd.frtoside[2].p
--            p =
            p = D.onSide(rd, 'middle', (rd.L+rd.frtoside[2].d)/2)
            rd.apin[#rd.apin+1] = {d=(rd.L+rd.frtoside[2].d)/2, h=forZ(p)} --p
            rd.apin[#rd.apin+1] = {d=rd.L, h=forZ(rd.list[#rd.list])} --rd.list[#rd.list]
--                toMark({rd.frtoside[1].p,rd.frtoside[2].p}, 'blue', nil, 0.1, 0.4)
--                if true then return end
--                toMark(rd.apin, 'blue', nil, 0.1, 0.4)
--            for _,p in pairs(rd.apin)
--                if true then return end
--                toMark(rd.apin, 'blue', function(d)
--                    return
--                end, 0.1, 0.4)
            if false then
                rd.apin[#rd.apin+1] = rd.list[1]
                -- middle
                local p = D.onSide(rd, 'middle', rd.frtoside[1].d/2)
                rd.apin[#rd.apin+1] = p
    --                U.dump(rd.apin, '?? for_PIN:')
    --                toMark(rd.apin, 'blue', nil, 0.1, 0.4)
    --                toMark({rd.apin}, 'blue', nil, 0.1, 0.4)
                rd.apin[#rd.apin+1] = rd.frtoside[1].p

                rd.apin[#rd.apin+1] = rd.frtoside[2].p
                p = D.onSide(rd, 'middle', (rd.L+rd.frtoside[2].d)/2)
                rd.apin[#rd.apin+1] = p
                rd.apin[#rd.apin+1] = rd.list[#rd.list]
                    toMark(rd.apin, 'blue', nil, 0.1, 0.4)

                -- exit conform
                rd.apin[#rd.apin+1] = {d=0, h=forZ(rd.list[1])}
                local cs = D.borderCross(rd.body, adec[rd.frto[1]].body, 'left', rd.frtoside[1], 0, 0)
                local dn,p = D.toSide(cs, rd)
                rd.apin[#rd.apin+1] = {d=cs:distance(rd.list[1])+4, h=forZ(cs)}
    --            rd.apin[#rd.apin+1] = {d=rd.list[2]:distance(rd.list[1]), h=forZ(rd.list[2])}
                --TODO:
                cs = D.borderCross(rd.body, adec[rd.frto[2]].body, 'left', rd.frtoside[2], 0, 0)
                dn,p = D.toSide(cs, rd)
                        lo('?? cs2:'..tostring(cs)..':'..rd.frtoside[2]..' L:'..rd.L..'/'..cs:distance(rd.list[#rd.list])..':'..(rd.L-(cs:distance(rd.list[#rd.list])+3)))
                rd.apin[#rd.apin+1] = {d=rd.L-(cs:distance(rd.list[#rd.list])+3), h=forZ(cs)}
    --            rd.apin[#rd.apin+1] = {d=rd.L-(rd.list[#rd.list]:distance(rd.list[#rd.list-1])), h=forZ(rd.list[#rd.list-1])}
                rd.apin[#rd.apin+1] = {d=rd.L, h=forZ(rd.list[#rd.list])}
                        toMark({cs, p},'red')
                        if true then return end
            end
        end
--            if dbg then U.dump(rd.apin, '?? APIN:'..rd.ind) end
            if dbg then U.dump(rd.apin, '??^^^^^^^^^^^^^^^^^^ for_APIN::'..itofill) end

            if false then
                tb:updateGrid()
                return
            end
-- PINS to terrain height
--- fill pin distance intervals
        local i = itofill
        ---- get number of steps
        local l = math.abs(rd.apin[i+1].d - rd.apin[i].d)
--        local nstep = math.floor(l/pinstep+0.5)
        pinstep = math.min(l-0.1,pinstep)
        local nstep = math.ceil(l/pinstep)
--                lo('??_____________________ NSTEP:'..pinstep)
        local step = l/nstep
--            lo('?? step_p:'..i..':'..step..':'..nstep)
        for k=1,nstep-1 do
            table.insert(rd.apin, i+1, {d=rd.apin[i+1].d - step})
        end
            if dbg then
                U.dump(rd.apin, '?? post_pin_FILL:'..#rd.apin..':'..l..':'..nstep..':'..i)
                toMark(rd.apin, 'green', function(d) return D.onSide(rd, 'middle', d.d) end, 0.15, 0.2)
                    lo('?? pre_height:'..#rd.apin)
            end
--- SET HEIGHTS
        local ppos,d,p = D.epos(rd.body, 0),0
        local cpin = 1
        local dist = rd.apin[cpin].d
    --        out.acyan = {}

        local cie = 1
        for j=1,#rd.apin do
            local pin = rd.apin[j]
            dist = pin.d
            if pin.fix then
                goto continue
            end
--                if dbg then lo('?? for_pin:'..j) end
            d = 0
            for i = cie,rd.ne-1 do
--                    lo('?? for_e:'..i)
                local pos = D.epos(rd.body, i)
                local ppos = D.epos(rd.body, i-1)
                local dd = (pos - ppos):length()
                local ishift = bcross and 0 or 1
                d = d + dd
                if d > dist then
                    local c = (d - dist)/dd
                    p = c*ppos + (1-c)*pos
                    if bcross then
                        pin.h = forZ(p)
                    else
                        -- middle pin for exit
--                        pin.h = (rd.apin[3].h+rd.apin[5].h)/2
                        pin.h = U.spline2(rd.apin[3].h, rd.apin[#rd.apin-2].h, dist - rd.apin[3].d, rd.apin[#rd.apin-2].d - rd.apin[3].d)
                    end

--                    elseif j > 2+ishift and j < #rd.apin-1-ishift then
--                        pin.h = U.spline2(rd.apin[2].h, rd.apin[#rd.apin-1].h, dist - rd.apin[2].d, rd.apin[#rd.apin-1].d - rd.apin[2].d)
--                    end
--                            lo('??^^^^^^^^ pH:'..j..':'..i..':'..rd.apin[2].h..'>'..rd.apin[#rd.apin-1].h..':'..dist..':'..rd.apin[#rd.apin-1].d..':'..rd.apin[2].d)
--                        lo('?? to_height: i:'..i..' dd:'..dd..' d:'..d..'/ dist:'..dist..' c:'..c..':'..tostring(p)..':'..pin.h)
--                    cie = i
--                    ppos = pos
                    break
    --                    lo('?? set_HP:'..cpin..' ie:'..i..':'..d..'/'..dist..' dd:'..dd..' c:'..c..':'..tostring(forZ(p)))
                end
--                ppos = pos
            end
            ::continue::
        end

            if dbg then
                U.dump(rd.apin, '?? pre_slope:')
                toMark(rd.apin, 'green', function(d)
                    local pp = D.onSide(rd, 'middle', d.d)
                    pp.z = forZ(pp) + 1
                    return pp
                end, 0.15, 0.2, true)
--                    U.dump(out['green'], '?? for_G:')
            end
    --            U.dump(out.blue, '?? for_blue:')

        if bcross then
--- ADJUST SLOPES
            local function curv(list, i)
                local a,b,c = vec3(list[i-1].d,list[i-1].h), vec3(list[i].d,list[i].h), vec3(list[i+1].d,list[i+1].h)

                return U.vang(c-b, b-a, true)/a:distance(c)
            end

            local dir = 1
            for i=2,#rd.apin-1, dir do
                local slope = (rd.apin[i].h-rd.apin[i-dir].h)/(rd.apin[i].d-rd.apin[i-dir].d)
                if math.abs(slope) > math.tan(maxslope) then
                    local dh = (math.abs(slope) - math.tan(maxslope))*(rd.apin[i].d-rd.apin[i-dir].d)
                        if dbg then lo('?? to_ADJUST1:'..i..':'..dh..':'..rd.apin[i].d..':'..rd.apin[i-dir].d) end
                    ----- eval curvatures
                    local a,b
                    if i > 2 and not rd.apin[i-dir].fix then
                        a = curv(rd.apin, i-dir)
        --                    lo('?? curv_a:'..a)
                    else
                        a = 0
                    end
                    if i < #rd.apin and not rd.apin[i].fix then
                        b = curv(rd.apin, i)
        --                    lo('?? curv_b:'..b)
                    else
                        b = 0
                    end
                        if dbg then lo('?? a_b:'..i..':'..a..':'..b..':'..-dh*a/(a+b)..':'..-dh*b/(a+b)..'/'..dh) end
                    if a == 0 and b == 0 then
                        lo('!! ERR_slope_ADJUST:')
                        goto continue
                    end
                    rd.apin[i-dir].h = rd.apin[i-dir].h - dh*a/(math.abs(a)+math.abs(b))
                    rd.apin[i].h = rd.apin[i].h - dh*b/(math.abs(a)+math.abs(b))
                end
        --            lo('?? for_slope:'..i..':'..slope..'/'..math.tan(maxslope))
            end
            dir = -1
            for i=#rd.apin-1,2,dir do
                local slope = (rd.apin[i].h-rd.apin[i-dir].h)/(rd.apin[i].d-rd.apin[i-dir].d)
                if math.abs(slope) > math.tan(maxslope) then
                    local dh = (math.abs(slope) - math.tan(maxslope))*(rd.apin[i].d-rd.apin[i-dir].d)
                        if dbg then lo('?? to_ADJUST2:'..i..'>'..(i-dir)..':'..dh..':'..rd.apin[i].d..':'..rd.apin[i-dir].d..':'..curv(rd.apin, i-dir)..':'..curv(rd.apin, i)) end
                    ----- eval curvatures
                    local a,b
                        if dbg then lo('?? for_i:'..i..':'..tostring(rd.apin[i-dir].fix)) end
                    if i < #rd.apin-1 and not rd.apin[i-dir].fix then
                        a = curv(rd.apin, i-dir)
                            if dbg then lo('?? curv_a:'..a) end
                    else
                        a = 0
                    end
                    if i > 1 and not rd.apin[i].fix then
                        b = curv(rd.apin, i)
                            if dbg then lo('?? curv_b:'..b) end
                    else
                        b = 0
                    end
                        if dbg then lo('?? a_b:'..i..':'..a..':'..b..':'..-dh*a/(a+b)..':'..-dh*b/(a+b)..'/'..dh) end
                    if a == 0 and b == 0 then
                        lo('!! ERR_slope_ADJUST2:')
                        goto continue
                    end
                    rd.apin[i-dir].h = rd.apin[i-dir].h + dh*a/(math.abs(a)+math.abs(b))
                    rd.apin[i].h = rd.apin[i].h + dh*b/(math.abs(a)+math.abs(b))
                end
        --            lo('?? for_slope:'..i..':'..slope..'/'..math.tan(maxslope))
            end
        end

    --[[
                toMark(rd.apin, 'blue', function(d)
                    local ps = D.onSide(rd, 'middle', d.d)
                    ps.z = d.h
                    return ps
                end, 0.1, 0.8, true)
    ]]
            if dbg then U.dump(rd.apin, '?? aPIN_fix:') end

--- SPLINE
---- pins
        local ap,apar = {}
        for i = 1,#rd.apin do
            ap[#ap+1] = {rd.apin[i].d,rd.apin[i].h}
        end
--            U.dump(ap, '?? pre_pS:')
        apar = U.paraSpline(ap)
--                U.dump(apar, '??********* apar0:'..#ap)

        table.insert(apar, 1, apar[1])
        apar[#apar+1] = apar[#apar]

--                U.dump(apar, '??********* apar:'..#ap)
    --        D.forHSpline()
            if dbg then lo('?? a_pp:'..#apar..':'..#rd.apin..' L:'..rd.L) end
---- curvature
        local sstep = 10
        local nsstep = math.floor(rd.L/sstep+0.5)
        sstep = rd.L/nsstep
        local ap = {rd.list[1]}
        local acp = {{0}}
--            lo('?? sstart:'..tostring(acp[]))
        for i=1,nsstep do
            local pm = D.onSide(rd, 'middle', i*sstep)
--                lo('?? for_c_dist:'..i..':'..(i*sstep)..'/'..rd.L..tostring((i*sstep)-rd.L==0)..':'..((i*sstep)-rd.L)..':'..tostring(pm))
            if math.abs(i*sstep - rd.L) < U.small_val then
                pm = D.epos(rd.body,rd.ne-1)
            end
            if pm then
                -- position
                ap[#ap+1] = pm
                -- distance
                acp[#acp+1] = {i*sstep}
            end
        end
--            U.dump(ap, '?? for_AP:')
--            U.dump(acp, '?? for_ACP:'..rd.L)
        for i=2,nsstep-1 do
            if (ap[i+1]-ap[i]):dot(ap[i]-ap[i-1]) < 0 then
                askip[#askip+1] = rd.ind
                lo('!! ERR_sharp_TURN:'..rd.ind)
                goto continue
            end
        end
--            if dbg then toMark(ap, 'mag', nil, 0.1) end
--[[
]]
        -- edges
            local ae = {}
        for i=1,rd.ne do
            ae[#ae+1] = D.epos(rd.body, i-1)+vec3(0,0,2)
        end
            if dbg then
                lo('?? edges:'..#ae..':'..tostring(ae[#ae])..':'..tostring(rd.list[#rd.list]))
--                toMark(ae, 'mag', nil, 0.05, 0.2)
                out.acyan = {ae[#ae]}
            end
--                U.dump(ap, '??^^^^^^^^^^^^^^^^^^ for_AP:'..nsstep..':'..#acp)
        for i=2,#acp-1 do
            local a,b,c = U.proj2D(ap[i-1]),U.proj2D(ap[i]),U.proj2D(ap[i+1])
            acp[i][2] = U.vang(c-b, b-a, true)/a:distance(c)
        end
        acp[1][2] = acp[2][2]
        acp[#acp][2] = acp[#acp-1][2]
        local cpar = U.paraSpline(acp)
        table.insert(cpar, 1, cpar[1])
        cpar[#cpar+1] = cpar[#cpar]
--            U.dump(acp, '??**************** ACP:'..#acp)
----------------------------
-- CONFORM
----------------------------
        local e2d,ne,e2curve = edge2dist(rd, dbg)
        if not e2d then
            askip[#askip+1] = ind
            goto continue
        end
        rd.e2d = e2d
--- curvature spline
--[[
            local mark_curv = {}
        for i=1,#e2curve do
            acp[#acp+1] = {e2curve[i].d,e2curve[i].curv}
                lo('?? for_CURV:'..i..':'.. e2curve[i].curv)
                mark_curv[#mark_curv+1] = {p = D.epos(rd.body, i), c = e2curve[i].curv}
        end
        local cpar = U.paraSpline(acp)
        table.insert(cpar, 1, cpar[1])
        cpar[#cpar+1] = cpar[#cpar]
                toMark(mark_curv, 'blue', function(d)
                    return d.p + vec3(0,0,forZ(d.p)+math.abs(d.c)*100)
                end, nil, nil, true)

            if dbg then lo('?? for_curv_spline:'..#cpar..':'..#acp..'/'..rd.ne..':'..#e2curve) end
]]
        local r = (rd.aw[1]/2 + margin + 0.1)*2/math.sqrt(2) -- scan stamp size
        local step = 2*math.sqrt(math.pow(r,2) - math.pow(rd.aw[1]/2 + margin, 2)) -- distance between stamps
        local cext = 1 -- how far edges to check for toPath
        local d = 0
        local ri = math.floor(r/grid+0.5) -- integer stamp size

        if #apar > 0 then
                local dbg_step = 100
                out.aseg = {}
                local amark,am1,am2,am3,am4,am5 = {},{},{},{},{},{}
    --    for d=step*0,step*0,step do
            local aset = {}
            for d=0,rd.L,step do
                    if dbg then lo('??+++++++++++++++ for_STEP:'..d/step..':'..d..'/'..rd.L) end
                -- TODO: eval for nearest node
                local w = rd.aw[1] -- current road width
                -- stamp center
                local pc = D.onSide(rd, 'middle', d, 0)
                if d == rd.L then pc = rd.list[#rd.list] end
                    am4[#am4+1] = pc
--                        lo('?? AM4:'..rd.ind..':'..#am4)
--                    out.acyan = {pc+vec3(0,0,forZ(pc))}
                local jc,ic = p2grid(pc)
                local pci = grid2p({ic,jc})
        --            lo('?? for_CENTER:'..ic..':'..jc..':'..tostring(pc))
        --            lo('?? ij_c:'..tostring(jc)..':'..tostring(ic)..':'..ri..':'..r)
-- get relevant edges
                local ae = {}
                local iem = 1
                    am3 = {}
                local rma = math.max(r*cext, ri*grid*2/math.sqrt(2))+2
--                local rma = math.max(r*cext, r*2/math.sqrt(2))
                for i=iem,ne-1 do
                    local epre = i
--                    if e2d[i]
                    if #ae == 0 then
--                        if D.epos(rd.body,i):distance(pc) < rma then
                        if d - e2d[i] < rma then
                                if dbg and math.floor(d/step+0.5) == dbg_step then
                                    lo('??+++++ dd:'..(d - e2d[i]-rma)..':'..(e2d[i-1]-e2d[i]))
                                end
                            -- open list
                            if i > 0 then
                                ae[#ae+1] = i-1
                                am3[#am3+1] = D.epos(rd.body,i-1)
                            end
                            ae[#ae+1] = i
                                am3[#am3+1] = D.epos(rd.body,i)
                        end
                    else
        --                    lo('?? if_e2d:'..i)
--                        if D.epos(rd.body,i):distance(pc) <= rma then
                        if e2d[i] - d <= rma then
                            -- append to list
                            ae[#ae+1] = i
                                am3[#am3+1] = D.epos(rd.body,i)
                        else
                            -- close list
                            ae[#ae+1] = i
                                am3[#am3+1] = D.epos(rd.body,i)
                            break
                        end
                    end
                end
                    if dbg and math.floor(d/step+0.5) == dbg_step then
                        lo('??________________ for_AE:'..#ae)
                        toMark(ae, 'blue', function(i) return D.epos(rd.body, i)+vec3(0,0,1) end, 0.08, 0.3)
                        local ppi = D.onSide(rd,'middle',d)
                        out.acyan = {pc+vec3(0,0,forZ(pc))}
                    end
        --            U.dump(ae, '?? AE:'..#ae)

        --                U.dump(legend['blue'], '?? for_blue:'..tostring(legend['blue'][1].alpha))
        --[[
                    out.ared = {}

                    for i=1,#ae do
                        out.ared[#out.ared+1] = D.epos(rd.body,ae[i])
                        out.ared[#out.ared] = out.ared[#out.ared] + vec3(0,0,forZ(out.ared[#out.ared]))
                    end
                        U.dump(out.ared, '?? for_ae:'..#ae)
        ]]
        --            U.dump(rd.e2d, '?? e2d:')
--                    local amark = {}
--                    local n = 0
                    if dbg then lo('?? step:'..d/step..':'..d..'/'..rd.L) end
--                    local cp = vec3(576.5, 1056.5)
--                    local cp = vec3(417, 1092)
--                    local cp = vec3(82, 692)
--                    local cp = vec3(48, 897)
                    local cp = vec3(50,896)
                    amark = {}
                    am2 = {}
                for i=ic-ri,ic+ri do
        --                lo('?? for_i:'..i)
                    for j = jc-ri,jc+ri do
        --                    lo('?? for_j:'..i..':'..j)
        --                    if mask[{i,j}] then
        --                        lo('?? mask_is:'..d..':'..i..':'..j)
        --                    end
                        local ij = U.stamp({i,j},true)
--                                    amark[#amark+1] = p + vec3(0,0,forZ(p))
--[[
]]
--                                    lo('?? mark:'..tostring(amark[#amark])..':'..math.floor(d/step)..':'..tostring(math.floor(d/step)==15))
                        if true then
--                        if not mask[ij] or mask[ij][3] ~= 0 then
--                        if not mask[ij] or mask[ij][3] ~= 0 then
                            -- distance to middle
                            local p = grid2p({i,j})
                                    if math.floor(d/step+0.5) then --== dbg_step and (not mask[ij] or mask[ij][3] ~= 0) then
--                                    if math.floor(d/step+0.5) == dbg_step and (not mask[ij]) then-- or mask[ij][3] ~= 0) then
--                                    if math.floor(d/step+0.5) == dbg_step and (not mask[ij] or mask[ij][3] ~= 0) then
--                                    if math.floor(d/step+0.5) == dbg_step and (not mask[U.stamp(ij,true)] or mask[U.stamp(ij,true)][3] ~= 0) then

                                        amark[#amark+1] = p --grid2p({i,j})-- p + vec3(0,0,forZ(p))
                                    end
                                if d == dbg_step*step then
--                                    amark[#amark+1] = p + vec3(0,0,forZ(p))
                                end
                                local pdbg = p:distance(cp) < 1.5 and d/step == dbg_step and true or false
--                                if i == 3091 and j == 2614 then
--                                    pdbg = true
--                                end
                                if not dbg then pdbg = false end
                            local crv = acp[math.floor(d/sstep+0.5)+1] and acp[math.floor(d/sstep+0.5)+1][2] or 0
--                                if p:distance(cp) < 3.5 then
--                                    lo('?? for_crv:'..d..':'..math.floor(d/sstep+0.5)..':'..tostring(crv))
--                                end
                            local pp,dp,ds,ie = D.toPath(p, rd, ae, nil, crv, pdbg) -- point on path, distance to path, distance from start
                            if pp then
                                if dp < w/2+margin then
        --                                amark[#amark+1] = p
                                    -- set height from spline
                                    --- get pin interval
                                    for k=2,#rd.apin do
                                        if rd.apin[k].d > ds then
                                            local pin1,pin2 = rd.apin[k-1],rd.apin[k]
--                                                if dbg then lo('?? for_k:'..k) end
                                            local s1 = apar[k-1]
                                            local s2 = apar[k]
                                            local a = s1[1]*ds^2 + s1[2]*ds + s1[3]
                                            local b = s2[1]*ds^2 + s2[2]*ds + s2[3]
                                            local h = U.spline2(a, b, ds-pin1.d, pin2.d-pin1.d)
                                            local fix = 0
                                            local dh = 0
                                            ---- eval curvature
--                                            local crv --= ecurv(ie)
--                                            local epnext = D.epos(rd.body, ie+1)
--                                            local nmove = 0
--[[
                                            local ep = D.epos(rd.body, ie)
                                            while pp:distance(ep) > rd.e2d[ie+1] - rd.e2d[ie] and ie<rd.ne-2 do
                                                ie = ie + 1
                                            end
]]
--                                            local s = pp:distance(D.epos(rd.body, ie))/(rd.e2d[ie+1] - rd.e2d[ie])
--                                                if bank == 2 then
--                                                    bank = 3
--                                                    lo('??********** BB2:'..ds..':'..tostring(s))
--                                                end
                                            ---- spline
                                            local ips = math.floor(ds/sstep)+1
                                            local crv = ips < #cpar and D.spline(cpar, ips, ds, ds%sstep/sstep) or nil
--                                            local crv --= (ie < rd.ne-1 and 0 <= s and s <= 1) and D.spline(cpar, ie+1, ds, s) or nil
--                                                if pp:distance(rd.list[2]) < 3 then
--                                            local rdir = -U.vang(p - pp, D.epos(rd.body, ie) - D.epos(rd.body, ie == 0 and ie+1 or ie-1), true)
                                            local rdir = (p - pp):dot(D.epos(rd.body, ie, 'left') - D.epos(rd.body, ie, 'right'))
                                            if bcross then
                                                if crv then
                                                    dh = 2*(crv*bank*math.min(dp, w/2+1)*rdir/math.abs(rdir) + math.abs(crv*bank*(w/2+1)))
    --                                                if dh < 0 then dh = 0 end
                                                    h = h + dh
    --                                                    h = h + crv*dp*bank*2*rdir/math.abs(rdir)
                                                else
                                                    lo('?? no_CURV:'..i..':'..j)
    --                                                        am2[#am2+1] = p
                                                end
                                            end
--[[
                                            if false and dp <= w/2+0 then
                                                -- set banking
                                                local crv = ecurv(ie)
                                                if crv then
                                                    local rdir = -U.vang(p - pp, D.epos(rd.body, ie) - D.epos(rd.body, ie == 0 and ie+1 or ie-1), true)
                                                    dh = crv*dp*bank*2*rdir/math.abs(rdir)

--                                                    h = h + crv*dp*bank*2*rdir/math.abs(rdir)
                                                end
                                            end
]]
                                            if not bcross then
                                                if ds < rd.apin[3].d - 0 then
                                                    -- aling exit with main branch
                                                    local ds,ps = D.toSide(p, adec[rd.frto[1]], rd.frtoside[1])
--                                                        if dbg and math.floor(d/step+0.5)==dbg_step then
--                                                            lo('?? on_SIDE:'..tostring(ps)..':'..tostring(p))
--                                                        end
                                                    h = forZ(ps)
--                                                    am1[#am1+1] = p
--                                            if dbg and math.floor(d/step+0.5)==dbg_step and (not mask[ij] or mask[ij][3] ~=0) then
--                                                    am4[#am4+1] = p
                                                elseif ds > rd.apin[#rd.apin-2].d + 0 then
                                                    local ds,ps = D.toSide(p, adec[rd.frto[2]], rd.frtoside[2])
                                                    h = forZ(ps)
--                                                    am1[#am1+1] = p
                                                else
--                                                    am4[#am4+1] = p
                                                end
                                            end

                                            if dp > w/2+(not bcross and 1 or 1) then
--                                                    am4[#am4+1] = p
                                                -- spline to margin
                                                h = U.spline2(h, forZ(p), dp-(w/2+1), margin)
                                                fix = nil
--                                                        amark[#amark+1] = p + vec3(0,0,forZ(p)+dh)
--[[
                                                    if rdir > 0 then
                                                        if p:distance(vec3(398,1083)) < 2 then
                                                            lo('?? for_curve:'..tostring(crv))
                                                            out.aseg[#out.aseg+1] = {p+vec3(0,0,forZ(p)+1), pp+vec3(0,0,forZ(pp)+1)}
                                                        end
                                                        if p:distance(vec3(382,1071)) < 2 then
                                                            lo('?? for_curve2:'..tostring(crv))
                                                            out.aseg[#out.aseg+1] = {p+vec3(0,0,forZ(p)+1), pp+vec3(0,0,forZ(pp)+1)}
                                                        end
                                                        amark[#amark+1] = p + vec3(0,0,forZ(p)+dh)
                                                    end
                                                    if p:distance(cp) < 0.5 then
--                                                    if p:distance(vec3(587, 1066)) < 0.5 then
--                                                        am2[#am2+1] = p
--                                                            lo('??______ for_DH: dh'..dh..' crv:'..tostring(crv)..':'..rdir..':'..tostring(p))
                                                    end
                                                    if p:distance(cp) < 1.1 then
                                                            lo('?? for_DH_near2:'..i..':'..j..' dh:'..dh..' crv:'..tostring(crv)..':'..ds..':'..ie)
                                                    end
                                                    if rdir > 0 and p:distance(cp) < 10 then
--                                                        out.aseg[#out.aseg+1] = {p+vec3(0,0,forZ(p)+1), pp+vec3(0,0,forZ(pp)+1)}
--                                                        amark[#amark+1] = p + vec3(0,0,forZ(p))
                                                    end
                                            elseif false then
--                                                am2[#am2+1] = p
                                                if p:distance(cp) < 1.5 then -- and d/step == 4 then
--                                                    if p:distance(vec3(587, 1066)) < 0.5 then
                                                    am2[#am2+1] = p + vec3(0,0,forZ(p)+dp-4)
                                                    out.aseg[#out.aseg+1] = {p+vec3(0,0,forZ(p)+1), pp+vec3(0,0,forZ(pp)+1)}
                                                        lo('?? for_step:'..d/step..':'..i..':'..j)
                                                end
                                                if p:distance(cp) < 0.5 then
--                                                    if p:distance(vec3(587, 1066)) < 0.5 then
--                                                        am2[#am2+1] = p
                                                        lo('??______ for_DH: dh:'..dh..' crv:'..tostring(crv)..':'..ds..':'..ie) --..tostring(p))
                                                end
                                                if p:distance(cp) < 1.1 then
                                                        lo('?? for_DH_near:'..i..':'..j..' dh:'..dh..' crv:'..tostring(crv)..':'..ds..':'..ie)
                                                end
                                            else
                                                if dbg and math.floor(d/step+0.5)==dbg_step then -- and (mask[ij] and mask[ij][3]==0) then
                                                    am4[#am4+1] = p
                                                end
]]
--                                                    amark[#amark+1] = p + vec3(0,0,forZ(p)+dh)
                                            else
                                                if dbg and math.floor(d/step+0.5)==dbg_step then
--                                                    am4[#am4+1] = p
                                                end
                                            end

--                                            if dbg and math.floor(d/step+0.5)==dbg_step and (mask[ij] and mask[ij][3]==0) then
                                            if dbg and math.floor(d/step+0.5)==dbg_step and (not mask[ij] or mask[ij][3] ~=0) then
--                                                am4[#am4+1] = p
                                                if ds < rd.apin[2].d then
--                                                    am4[#am4+1] = p
                                                end
--                                                if
--                                                amark[#amark+1] = p + vec3(0,0,forZ(p))
                                            end
--                                                fix = false
                                            local set
--                                            if true then
                                            if dbg then
                                                set = terSet({i,j}, h, fix, dbg ~= nil and bcross)
                                                if cjunc and (rd.L-ds)>12 then
                                                    if math.floor(d/step+0.5)==dbg_step then
--                                                        am4[#am4+1] = p
                                                    end
                                                end
--                                            if cjunc and U.proj2D(p):distance(U.proj2D(ajunc[cjunc].p))>=20 then
                                            else
                                                set = terSet({i,j}, h, fix) --, dbg ~= nil and bcross)
                                            end
                                            if set then
                                                aset[#aset+1] = set
                                            end
--                                                n = n + 1
                                            break
                                        else
                                            -- far from start
                                            if dbg and math.floor(d/step+0.5)==dbg_step then
                                                if p:distance(cp) < 5 then
--                                                    am2[#am2+1] = p
                                                end
--                                                lo('?? for_DS:'..k..':'..tostring(ds)..'/'..rd.apin[k].d..':'..tostring(p))
                                            end
        --                                    D.forHSpline()
                                        end
                                    end
--                                    amark[#amark+1] = p
                                else
                                -- far from road
                                    if dbg and p:distance(cp) < 2 then
                                        lo('?? farf:'..crv)
                                        out.aseg[#out.aseg+1] = {p+vec3(0,0,forZ(p)+1), pp+vec3(0,0,forZ(pp)+1)}
                                        am2[#am2+1] = p
                                    end
--                                    am2[#am2+1] = p
                                end
                                    if dbg and math.floor(d/step+0.5)==dbg_step then
--                                        am4[#am4+1] = p
--                                        amark[#amark+1] = p + vec3(0,0,forZ(p))
                                    end
                            else --if false then
                                if dbg and math.floor(d/step+0.5) == dbg_step then
                                        lo('?? no_proj:'..tostring(p)..':'..(i-ic)..':'..(j-jc))
--                                    am2[#am2+1] = p
--                                    lo('?? for_curv:'..tostring(crv))
                                end
--                                if p:distance(vec3(587,1066)) < 3 then
--                                    am2[#am2+1] = p --+ vec3(0,0,forZ(p))
                                if i == 3091 and j == 2614 then
--                                        U.dump(ae,'?? for_IJ:'..i..':'..j..':'..#ae..':'..#am3..':'..tostring(D.epos(rd.body,1)))
--                                        toMark(am3, 'mag', nil, 0.05, 0.3)
--                                        out.acyan = {D.epos(rd.body,ae[1])+vec3(0,0,forZ(D.epos(rd.body,ae[1])))}-- {rd.list[1]}
                                end
--                                    lo('?? miss:')
                            end
--                                        amark[#amark+1] = p + vec3(0,0,forZ(p))
        --[[
                                if i == ic+5 and j == jc+5 and pp then
                                    out.white = {p + vec3(0,0,forZ(p)), pp + vec3(0,0,forZ(pp))}
                                        U.dump(out.ared, '?? pp:'..tostring(p)..':'..tostring(pp))
                                end
                                out.acyan[#out.acyan+1] = p + vec3(0,0,forZ(p))
        ]]
                        else
        --                    lo('?? mask_fixed:')
        --                    amark[#amark+1] = grid2p({i,j})
                        end
                    end
                end
--                        lo('?? for_MISS:'..#amark..':'..#am2..':'..#am3..'/'..#ae)
--                    toMark(amark, 'white', nil, 0.05, 0.3, true)
--                    toMark(am2, 'red', nil, 0.07, 0.3, true)
--                    toMark(am3, 'mag', nil, 0.05, 0.3)

        --            lo('?? set:'..n)
        --            for key,d in pairs(mask) do
        --                U.dump(d, '?? for_mask:'..tostring(key))
        --            end
        --            U.dump(mask, '?? for_MASK:')
--                        toMark(am1, 'white', nil, 0.05, 0.3) --, true)
--                        toMark(am4,'yel',nil,0.06,0.4)
        --                toMark(amark, 'yel', nil, 0.05, 0.6)
                    if dbg and math.floor(d/step+0.5)==dbg_step then
                            lo('?? to_MARK:'..#amark..':'..#am2..':'..#am4)
                        toMark(amark, 'white', nil, 0.05, 0.3) --, true)
--                        toMark(am2,'red',nil,0.1,0.7)
                        toMark(am4,'yel',nil,0.06,0.4)
                    end
                    if d == 4*step then
--                        toMark(amark, 'white', nil, nil, 0.1)
                    else
        --                toMark(amark, 'yel', nil, 0.05, 0.6)
                    end
            end
--                    if dbg then toMark(am4,'yel',nil,0.06,0.8) end
            rd.aset = aset
                lo('??____________ conf_DONE:')
        else
            lo('!! NO_APIN:'..ind)
        end
        rd.e2d = nil
        rd.apin = nil
        ::continue::
    end

    --TODO: throughs "unable to convert"
    tb:updateGrid()
    out.inconform = true
            U.dump(askip, '?? to_skip:')
    for _,rd in pairs(list) do
        if #U.index(askip, rd.ind) == 0 then
--            local ind = dec.ind
    --    for _,ind in pairs(ilist) do
--            local rd = adec[ind]
--                abr[i].mat = (not abr[i].mat or abr[i].mat=='WarningMaterial') and default.mat or abr[i].mat
            rd.body:setField('material', 0, (not rd.mat or rd.mat=='WarningMaterial') and default.mat or rd.mat)
        end
        rd.body:setPosition(rd.body:getPosition())
    end
    if #list == 1 then
        -- tohistory
        local rd = list[1]
--            lo('?? for_ASET:'..rd.ind..':'..tostring(rd.aset and #rd.aset or nil))
        if not dhist[rd.ind] then
            dhist[rd.ind] = {}
        end
        dhist[rd.ind][#dhist[rd.ind]+1] = deepcopy(mask)
            lo('?? to_hist:'..rd.ind..':'..tableSize(dhist[rd.ind][#dhist[rd.ind]])..':'..tostring(rd.aset and tableSize(rd.aset) or nil)..' lines:'..tostring(rd.aline and #rd.aline or nil))
        -- update ajacent
        local bcross = across[rd.ind]
        if bcross then
            -- update adjacent roads
            for _,ref in pairs(bcross) do
                for j,b in pairs(ref) do
                    adec[b[1]].body:setPosition(adec[b[1]].body:getPosition())
                end
            end
        end
        if rd.aline then
            for i,l in pairs(rd.aline) do
                l.body:setPosition(l.body:getPosition())
            end
        end
    end
        lo('<< road2ter:'..#askip)
end
D.road2ter = road2ter
--[[
        for i=2,#rd.apin-1,2 do
            ---- get number of steps
            local l = rd.apin[i+1].d - rd.apin[i].d
            local nstep = math.floor(l/pinstep+0.5)
            local step = l/nstep
    --            lo('?? step_p:'..i..':'..step..':'..nstep)
            for k=1,nstep-1 do
                table.insert(rd.apin, i+1, {d=rd.apin[i+1].d - step})
            end
        end
]]
--[[
        for i=1,rd.ne-1 do
            while cpin < #rd.apin and rd.apin[cpin].fix do
                cpin = cpin + 1
    --                lo('?? for_CPIN2:'..cpin..'/'..#rd.apin)
                dist = rd.apin[cpin].d
            end
            if cpin > #rd.apin then
                break
            end
            local pos = D.epos(rd.body, i)
                out.acyan = {pos + vec3(0,0,forZ(pos))}
            local dd = (pos - ppos):length()
            d = d + dd
                lo('?? for_ipin:'..i..':'..cpin..':'..tostring(pos)..' dd:'..dd..' d:'..d..'/ dist:'..dist)
            if d > dist then
    --                out.acyan[#out.acyan+1] = pos + vec3(0,0,forZ(pos))
                local c = (d - dist)/dd
                p = c*ppos + (1-c)*pos
                    lo('?? set_HP:'..cpin..' ie:'..i..':'..d..'/'..dist..' dd:'..dd..' c:'..c..':'..tostring(forZ(p)))
                if not rd.apin[cpin].fix then
                    rd.apin[cpin].h = forZ(p)
--                        lo('?? for_H:'..cpin..':'..tostring(rd.apin[cpin].h))
                end
                ---- next pin
                cpin = cpin + 1
                dist = rd.apin[cpin].d
    --                lo('?? for_CPIN_next:'..cpin..'/'..#rd.apin)
    --            return c*ppos + (1-c)*pos + margin*U.vturn(pos - ppos, (side == 'left' and -1 or 1)*math.pi/2):normalized(),i
            end
            ppos = pos
        end
]]
    --[[
            U.dump(rd.apin, '?? pin_pre_slope:'..#rd.apin)
            toMark(rd.apin, 'white', function(d)
                local ps = D.onSide(rd, 'middle', d.d)
                ps.z = d.h
                return ps
            end, 0.2, 0.3, true)
    ]]

    --        ap[#ap+1] = {rd.apin[i].d,rd.apin[i].h}
    --        local apar = U.paraSpline(ap)
    --        table.insert(apar, 1, apar[1])
    --        apar[#apar+1] = apar[#apar]
--[[
            if false then
                if not start then
                    if not rd.apin[i].fix then
                        start = true
                        if i > 2 then
                            ap[#ap+1] = {rd.apin[i-2].d,rd.apin[i-2].h}
                        end
                        if i > 1 then
                            ap[#ap+1] = {rd.apin[i-1].d,rd.apin[i-1].h}
                        end
                        ap[#ap+1] = {rd.apin[i].d,rd.apin[i].h}
                    end
                else
                    ap[#ap+1] = {rd.apin[i].d,rd.apin[i].h}
                    if rd.apin[i].fix then
                        -- close list
                        start = nil
                        if i < #rd.apin then
                            ap[#ap+1] = {rd.apin[i+1].d,rd.apin[i+1].h}
                        end
                        apar = U.paraSpline(ap)
                            lo('?? to_pspline:'..#ap..':'..#apar)
                        ap = {}
        --                table.insert(apar, 1, apar[1])
        --                apar[#apar+1] = apar[#apar]
        --                    U.dump(apar, '?? apar:'..i)
                    end
                end
            end
]]
    --        rd.aspline[#rd.aspline+1] = apar

--[[
            if roadID == 28353 then
                editor_roadUtils.reloadDecals(rd)
                rd = scenetree.findObjectById(roadID)
                list = editor.getNodes(rd)
                lo('?? decalsLoad_width:'..list[1].width)
            end
            if #adec < 5 then
                lo('?? lanes:'..tostring(rd.lanesRight)..':'..rd.lanesLeft)
            end
]]

--[[
local function decalsParse(path, l, g)
    L = l
    grid = g
    local str = io.open(path):read('*all')
        lo('>> decalsLoad:'..path..':'..#str)
    local tdec = U.chop(str, '"nodes":[[', true)
    table.remove(tdec, 1)
    for _,t in pairs(tdec) do
--            lo('?? for_t1:'..t)
        t = U.chop(t, '] ],"renderPriority"')[1]
--            lo('?? for_t2:'..t)
local apair = U.chop(t,'],[')
local anode = {}
for i,c in pairs(apair) do
    local ac = U.split(c,',')
--            lo('?? for_c:'..tostring(c)..':'..ac[1]..':'..ac[2])
--            break
    anode[#anode+1] = vec3(ac[1],ac[2])
end
adec[#adec+1] = anode
--        break
end
--        U.dump(adec[1], '?? ndec:'..#adec) --..':'..tdec[1])
-- to squares
--    local step = 10
local nn = 0
for ir,r in pairs(adec) do
for k,n in pairs(r) do
        nn = nn + 1
    local i,j = math.floor((n.y + L)/grid) + 1,math.floor((n.x + L)/grid) + 1
    if aref[i] == nil then
        aref[i] = {}
    end
    if aref[i][j] == nil then
        aref[i][j] = {}
    end
    if aref[i][j][ir] == nil then
        aref[i][j][ir] = {math.huge, 0}
--                    lo('?? for_ir:'..nn..' i:'..i..' j:'..j..' ir:'..ir..':'..#aref[i][j][ir]..':'..#aref[i][j])
    end
--                if #aref[i][j] > 0 then
--                    lo('?? for_ref:'..i..':'..j..':'..#aref[i][j]..' n:'..tostring(n)..':'..ir)
--                end
    if k < aref[i][j][ir][1] then aref[i][j][ir][1] = k end
    if k > aref[i][j][ir][2] then aref[i][j][ir][2] = k end
end
end
--        lo('<< decalsLoad:'..tostring(#aref)..':'..tostring(#aref[100])..':'..nn)
--        U.dump(aref[365][455][1], '?? forr:')
--    for i = 1,math.ceil(4096*2/step) do
--    end
return adec,aref
end
]]


--local adec = {}
--local adecstat =

local function decalUp(desc, ext)
--[[
                list = list,
                w = 0.3,
                mat = 'line_yellow'
]]
--        U.dump(desc, '>> decalUp:')
    if not desc.list then return end
    desc.listrad = deepcopy(desc.list)
    local an = {}
    local aw = {}
    for _,p in pairs(desc.list) do
        an[#an + 1] = {pos = p, width = desc.w}
        aw[#aw + 1] = desc.w
    end
    local id = editor.createRoad(
        an,
        {overObjects = false, drivability = 1.0}
    )
    --    {{pos = apos[1], width = w, drivability = 1}},
    local road = scenetree.findObjectById(id)
    road:setField('material', 0, desc.mat or default.mat)
    road:setField('drivability', 0, 2)
    if desc.lane then
        road.lanesRight = desc.lane[1] or 1
        road.lanesLeft = desc.lane[2] or 1
    end
    road:setField('distanceFade', 0, string.format("%f %f", 0, 200))
--    groupDecal:add(road.obj)
    groupLines:add(road.obj)

    desc.body = road
    desc.id = id
    desc.ne = road:getEdgeCount()

    desc.aw = aw

    if not ext then
        adec[#adec+1] = desc
        desc.ind = #adec
    end

    return road
end
D.decalUp = decalUp


-- 0-based
local function epos(rd, i, side)
--        lo('>> epos:')
    if not side then side = 'middle' end
    local p
    if side == 'left' then
        p = rd:getLeftEdgePosition(i)
    elseif side == 'middle' then
        p = rd:getMiddleEdgePosition(i)
    else
        p = rd:getRightEdgePosition(i)
    end
    p.z = 0

    return p
end
D.epos = epos


-- index of edge closest to p
local function p2e(p, rd, istart)
    if not istart then istart = 0 end
    local dmi, imi = math.huge
    local ne = rd.body:getEdgeCount()
    for i=istart,ne,1 do
        local d = epos(rd.body, i):distance(p)
        if d < dmi then
            dmi = d
            imi = i
        end
    end
    return imi
end


-- edge index (0-based) for the node
local function edge4node(rd, inode)
    local obj = rd.body
    local nsec = obj:getEdgeCount()
    for i=1,nsec do
        if rd.list[inode]:distance(epos(obj, i-1, 'middle')) < U.small_dist then
            return i-1
        end
    end
end


local function roadLength(rd, side)
    if not side then side = 'middle' end
    local d,nsec = 0, rd.body:getEdgeCount()
    local eprev = epos(rd.body, 0, side)
--        lo('?? rdL:'..tostring(eprev))
    for i=1,nsec-1 do
        local ce = epos(rd.body, i, side)
--            lo('?? rdL2:'..tostring(ce))
        d = d + eprev:distance(ce)
        eprev = ce
    end
    return d,nsec
end
D.roadLength = roadLength


--local function alongSide(p, rd, side)
--end


local function toPath(p, rd, aind, side, crv, dbg)
    if not side then side = 'middle' end
        if dbg then
            U.dump(aind, '?? toPath:'..rd.ne)
        end
    local dmi,imi,smi = math.huge
    for _,i in pairs(aind) do
        local d = epos(rd.body, i, side):distance(p)
        if d < dmi then
            dmi = d
            imi = i
        end
    end
        if dbg then lo('?? toPath_imi:'..tostring(imi)..':'..side) end

    if imi then
        local tmi = imi > 0 and imi-1 or imi
        local sd = U.vang(epos(rd.body, tmi+1)-epos(rd.body,tmi), p-epos(rd.body,tmi),true)
--        if false then
        if (sd > 0 and crv > 0) or (sd < 0 and crv < 0) then
            -- outer side
            local e1,e2,e1p,e2p,s,c = imi,imi
            if imi > 0 then
                e1 = e1 - 1
            end
            e1p = epos(rd.body, e1, side)
            e2p = epos(rd.body, e2, side)
            s = closestLinePoints(e1p, e2p, p, p+U.vturn(e2p-e1p,math.pi/2))
            if s>1 then
                e1 = e2
                if e2 < rd.ne-1 then
                    e2 = e2 + 1
                    e1p = epos(rd.body, e1, side)
                    e2p = epos(rd.body, e2, side)
                    s = closestLinePoints(e1p, e2p, p, p+U.vturn(e2p-e1p,math.pi/2))
                end
            elseif s < 0 then
                e2 = e1
                if e1 > 0 then
                    e1 = e1-1
                    e1p = epos(rd.body, e1, side)
                    e2p = epos(rd.body, e2, side)
                    s = closestLinePoints(e1p, e2p, p, p+U.vturn(e2p-e1p,math.pi/2))
                end
            end
            if 0<=s and s<=1 then
                c = e1p*(1-s) + e2p*s
            else
                c = e1p
                e2 = e1
            end
            if not c then return end

            return c, c:distance(p), rd.e2d[e1]*(1-s) + rd.e2d[e2]*s, e1
        else
            -- inner side
            local s = math.huge
            local e1,e2 = imi,imi
            local e1p,e2p
            while s > 1 or s < 0 do
                if e1 > 0 then
                    e1 = e1 - 1
                end
                if e2 < rd.ne-1 then
                    e2 = e2 + 1
                end
                e1p = epos(rd.body, e1, side)
                e2p = epos(rd.body, e2, side)
                s = closestLinePoints(e1p, e2p, p, p+U.vturn(e2p-e1p,math.pi/2))
                if e1 == 0 and e2 == rd.ne-1 then break end
            end
                if dbg then lo('?? e1_e2:'..e1..'>'..e2..':'..s) end
            if 0<=s and s<=1 then
                local c = e1p*(1-s) + e2p*s
    --                    lo('?? if_e2d:'..tostring(rd.e2d)..':'..e1..':'..e2)
                return c, c:distance(p), rd.e2d[e1]*(1-s) + rd.e2d[e2]*s, e1
            end
        end
    end
    if true then return end

--[[
    if false and imi then
        local s = math.huge
        local e1,e2 = imi,imi
        local e1p,e2p
        while s > 1 or s < 0 do
            if e1 > 0 then
                e1 = e1 - 1
            end
            if e2 < rd.ne-1 then
                e2 = e2 + 1
            end
            e1p = epos(rd.body, e1, side)
            e2p = epos(rd.body, e2, side)
            s = closestLinePoints(e1p, e2p, p, p+U.vturn(e2p-e1p,math.pi/2))
            if e1 == 0 and e2 == rd.ne-1 then break end
        end
            if dbg then lo('?? e1_e2:'..e1..'>'..e2..':'..s) end
        if 0<=s and s<=1 then
            local c = e1p*(1-s) + e2p*s
--                    lo('?? if_e2d:'..tostring(rd.e2d)..':'..e1..':'..e2)
            return c, c:distance(p), rd.e2d[e1]*(1-s) + rd.e2d[e2]*s, e1
        end
    end
]]
--    end
end
D.toPath = toPath


-- distance from point p to the side of rd
--[[
local function toSide_(p, rd, side)
    local ns = rd.body:getEdgeCount()
    for i=1,ns do
        local e1,e2 = epos(rd, side, i-1), epos(rd, side, i)
        local vn = U.vturn(e2-e1, math.pi)
        local s = closestLinePoints(e1, e2, p, p+vn)

        if 0 <= s and s <= 1 then
            local vp = e1*(1-s)+e2*s
            return vp:distance(p), vp
        end
    end
end
]]

-- distance from p to side
local function toSide(p, rd, side)
    local dmi,imi = math.huge
    local ns = rd.body:getEdgeCount()
    for i=1,ns do
        local d = p:distance(epos(rd.body, i-1, side))
        if d < dmi then
            dmi = d
            imi = i-1
        end
    end
    if imi then
        local ifr = imi>0 and imi-1 or 0
        local ito = imi<ns-1 and imi+1 or ns-1
        local d,p = U.toLine(p, {epos(rd.body, ifr, side),epos(rd.body, ito, side)})

        return d,p,ifr
    end
--    return epos(rd.body, imi-1, side)
end
D.toSide = toSide


local function line2side(a, b, rd, side)
    local ns = rd.body:getEdgeCount()
    local dmi,pmi,imi = math.huge
    for i=1,ns-1 do
        local p,s = U.line2seg(a, b, epos(rd.body, i-1, side), epos(rd.body, i, side))
        if p then
            if p:distance(a) < dmi then
                dmi = p:distance(a)
                pmi = p
--                imi =
            end
--                out.avedit = {epos(rd.body, i-1, side), epos(rd.body, i, side)}
--            return p
        end
    end
    if pmi then return pmi end
end

-- point an the side at margin at a dist from rd start
local function onSide(rd, side, dist, margin, dbg)
        if dbg then lo('?? onSide0:'..dist..'/'..rd.L..':'..tostring(dbg)..':'..tostring(side)..':'..roadLength(rd)) end
    if not margin then margin = 0 end
--        lo('>> onSide:'..side..':'..dist..':'..margin)
    local obj = rd.body
--    local anode = editor.getNodes(rd)
    local nsec = obj:getEdgeCount()
--        lo('>> onSide:'..nsec..':'..tostring(rd:getMiddleEdgePosition(0)))
--[[
    local fpos = function(i)
--        epos(rd, i, side)
        if side == 'left' then
            return rd:getLeftEdgePosition(i)
        else
            return rd:getRightEdgePosition(i)
        end
    end
]]
    if false and side == 'middle' and rd.e2d then
        local e2d = rd.e2d
        for ie,d in pairs(e2d) do
            if d - dist > -U.small_dist then
                local dd = e2d[ie-1] - d
                local c = (d - dist)/dd
                local ppos,pos = epos(obj,ie-1),epos(obj,ie)
                return c*ppos + (1-c)*pos +
                    margin*U.vturn(pos - ppos, (side == 'left' and -1 or 1)*math.pi/2):normalized(),ie
            end
        end
    else
        local ifr,ito,step = 0,nsec-1,1
        local d = 0
        local ppos = epos(obj, ifr, side)
        for i=ifr+1,ito,step do
            local pos = epos(obj, i, side)
            local dd = (pos - ppos):length()
            d = d + dd
    --            if dbg then lo('?? onSide:'..i..'/'..nsec..':'..d..'/'..dist..':'..tostring(epos(rd, nsec))..':'..tostring(epos(rd, i+1))) end
                    if dbg then lo('?? for_d:'..i..':'..d..'/'..dist..':'..tostring(d==dist)) end
            if d - dist > -U.small_val then
                    if dbg then lo('?? to_ret:'..i) end
                local c = (d - dist)/dd

                return c*ppos + (1-c)*pos + margin*U.vturn(pos - ppos, (side == 'left' and -1 or 1)*math.pi/2):normalized(),i
            end
            ppos = pos
        end
    end
    if math.abs(dist - L) < U.small_dist then
        return rd.list[#rd.list],nsec
    end
--    local fpos = side == 'left' and rd:getLeftEdgePosition or rd:
--    rd:getMiddleEdgePosition(i)
end
D.onSide = onSide


local edges

-- crossing of lines on margin distence from road borders
local function borderCross(rda, rdb, sidea, sideb, starta, startb, margin)
    if not margin then margin = 0 end
    local nsa,nsb = rda:getEdgeCount(),rdb:getEdgeCount()
--        lo('>> borderCross: sa:'..sidea..' sb:'..sideb..' st_a:'..starta..' st_b:'..startb..':'..nsa..':'..nsb..':'..margin)
    local dira, dirb = 1, 1
    if starta == 1 then
        starta, dira = nsa-1, -1
    end
    if startb == 1 then
        startb, dirb = nsb-1, -1
    end
    local ishit
--            out.acyan = {}
    for i = 1,nsa-1 do
        for j = 1,nsb-1 do
            local sa = {epos(rda, starta + dira*(i-1), sidea), epos(rda, starta + dira*i, sidea)}
--            local na = margin*U.vturn(sa[2] - sa[1], (sidea == 'left' and -1 or 1)*math.pi/2):normalized()
            local na = margin*(epos(rda, starta + dira*(i-1)) - epos(rda, starta + dira*(i-1), sidea)):normalized()
            local sb = {epos(rdb, startb + dirb*(j-1), sideb), epos(rdb, startb + dirb*j, sideb)}
--            local nb = margin*epos(rdb)
--            U.vturn(sb[2] - sb[1], (sideb == 'left' and -1 or 1)*math.pi/2):normalized()
--            local nb = margin*U.vturn(sb[2] - sb[1], (sideb == 'left' and -1 or 1)*math.pi/2):normalized()
            local nb = margin*(epos(rdb, startb + dirb*(i-1)) - epos(rdb, startb + dirb*(i-1), sideb)):normalized()
            local sas = {sa[1] + na, sa[2] + na}
            local sbs = {sb[1] + nb, sb[2] + nb}
--            local c = U.line2seg(
--                sas[1], sas[2],
--                sbs[1], sbs[2]
--            )
--                lo('?? BC:'..i..':'..j..':'..tostring(U.proj2D(sas[1]))..':'..tostring(U.proj2D(sas[2]))..':'..tostring(U.proj2D(sbs[1]))..':'..tostring(U.proj2D(sbs[2])))
            local c,s = U.segCross(U.proj2D(sas[1]),U.proj2D(sas[2]),U.proj2D(sbs[1]),U.proj2D(sbs[2]))
            if c then
--                    toMark({U.proj2D(sas[1]),U.proj2D(sas[2])},'cyan',nil,0.1,1)
                return c,i,j,s
            end
--[[
                if i == 7 then
--                    out.acyan = {sas[1],sas[2]}
                end
                if j==10 and i == 7 then
                    local c = U.segCross(U.proj2D(sas[1]),U.proj2D(sas[2]),U.proj2D(sbs[1]),U.proj2D(sbs[2]))
                        lo('?? cross_7_10:'..tostring(c))
                    out.acyan = {sas[1],sas[2], sbs[1],sbs[2]}
                end
--                out.acyan[#out.acyan+1] = sas[1]
--                if i==1 and j ==1 then
--                    out.acyan = sb
--                end
--                lo('?? if_cross:'..tostring(starta + dira*i)..':'..tostring(startb + dirb*j)..':'..tostring(c))
            if c and U.line2seg(
                sbs[1], sbs[2],
                sas[1], sas[2]
            ) then
--                    lo('?? l2sa:'..tostring(sa[1])..':'..tostring(sa[2])..':'..tostring(c))
--                    lo('?? l2sb:'..tostring(sb[1])..':'..tostring(sb[2])..':'..tostring(c))
--                    c = epos(rda, 0, sidea)
--                    lo('?? borderCross:'..i..'/'..nsa..':'..j..'/'..nsb..':'..tostring(c)) --..':'..(startb + dirb*j)..':'..(startb + dirb*(j+1)))
--                    out.acyan[#out.acyan+1] = c
--                    out.avedit[#out.avedit+1] = sa[1]
--                    out.avedit[#out.avedit+1] = sa[2]
--                    out.acyan[#out.acyan+1] = c
--                    out.avedit[#out.avedit+1] = sa[1]
--                    out.avedit[#out.avedit+1] = sa[2]
--                    out.avedit[#out.avedit+1] = sb[1]
--                    out.avedit[#out.avedit+1] = sb[2]
--                    out.avedit[#out.avedit+1] = c
--                    out.avedit[#out.avedit+1] = epos(rda, starta, sidea)
--                    out.avedit[#out.avedit+1] = epos(rdb, startb+2, sideb)
--                    out.avedit = {vec3(50,0,0)}
--                    lo('<< borderCross:')
                return c,i,j
--                ishit = true
--                out.avedit = {c}
--                break
            end
]]
--[[
                    lo('?? borderCross:'..i..'/'..nsa..':'..j..'/'..nsb..':'..tostring(c)) --..':'..(startb + dirb*j)..':'..(startb + dirb*(j+1)))
                    out.avedit = {}
                    out.avedit[#out.avedit+1] = epos(rda, starta, sidea)
                    out.avedit[#out.avedit+1] = epos(rdb, startb, sideb)
                    out.acyan[#out.acyan+1] = c
]]
        end
        if ishit then break end
    end
        lo('<< borderCross_NONE:')
end
D.borderCross = borderCross



-- point at the rd side at margin with step
local function pin2sideD(rd, side, db, de, sidemargin, pinstep, dbg)
    if not pinstep then pinstep = 6 end
    local list = {}
    if de > rd.L then de = rd.L end
    local nstep = math.floor((de-db)/pinstep+0.5)
    pinstep = (de-db)/nstep
    for i = 0,nstep do
        local pmid,ei = onSide(rd, 'middle', db+(i)*pinstep)
            if dbg then lo('?? pin2sideD:'..i..':'..tostring(ei)) end
--        if not ei and math.abs(db+(i)*pinstep - rd.L) < U.small_dist then
--            pmid = rd.list[#rd.list]
--            ei = rd.ne
--        end
        if ei then
            local un = (epos(rd.body,ei,side=='left' and 'right' or 'left') - epos(rd.body,ei,side)):normalized()
            list[#list+1] = pmid + un*(sidemargin-rd.w/2)
        else
            lo('!! ERR_pin2sideD_NO_ONSIDE:'..(db+(i)*pinstep)..':'..de..'/'..rd.L)
        end
--[[
        local pmid,d,ei = onSide(rd, side, db+(i)*pinstep)
            lo('?? pin2sideD:'..i..':'..tostring(ei))
        local un = (epos(rd.body,ei,side=='left' and 'right' or 'left') - epos(rd.body,ei,side)):normalized()
        list[#list+1] = pmid + un*(sidemargin-rd.w/2)
]]
--        list[#list+1] = onSide(rd, side, db+(i)*pinstep, sidemargin)
    end
--    list[#list+1] = onSide(rd, side, de, sidemargin)

    return list
end

--local function pin2side(rd, side, ifr, ito, margin, step)
local function pin2side(rd, side, cs, ce, ibs, ibe, sidemargin, pinstep) --  side, ifr, ito, margin, step)
    local list = {cs}
    local oside = side == 'left' and 'right' or 'left'
    for i = ibs+pinstep,ibe-pinstep+1,pinstep do
        list[#list+1] = epos(rd, i, side)
        list[#list] = list[#list] + sidemargin*(epos(rd, i, oside)-epos(rd, i, side)):normalized()
    end
    list[#list+1] = ce

    return list
end


local function sideLine(rd, side, rdfr, sidefr, rdto, sideto, desc) -- sidemargin, pinstep)
    if not desc or not rd then return end
    rd = rd.body
    rdfr = rdfr.body
    rdto = rdto.body
    local sidemargin,pinstep = default.sidemargin,2
--    if not sidemargin then sidemargin = default.sidemargin end
--    if not pinstep then pinstep = 2 end
--    local wline = 0.1
    local cs,ias,ibs,sb = borderCross(rd, rdfr, side, sidefr, 0, 0, sidemargin)

    if not ias then
        lo('!! ERR_sideLine_NOCROSS_START:'..side..':'..sidefr..':'..sideto)
        return
    end
    local ce,iae,ibe,se = borderCross(rd, rdto, side, sideto, 0, 0, sidemargin)
    if not iae then
        lo('!! ERR_sideLine_NOCROSS_END:'..side..':'..sidefr..':'..sideto)
        return
    end
    if (ce-cs):length() < 1.8 then
--            lo('!!--------- line_SHORT:')
        desc.on = false
--        rd:setField('hidden', 0, 'true')
        return desc
    end
--        lo('?? sideLine:'..tostring(rd)..':'..tostring(rdfr)..':'..tostring(rdto)..':'..tostring(cs)..':'..tostring(ias)..':'..tostring(ce))
--TODO: use pin2sideD
    desc.list = pin2side(rd, side, cs, ce, ias, iae, sidemargin, pinstep) -- or {}
--    desc.list = pin2sideD(rd, side, ds, de, sidemargin) -- or {}

--        lo('<< sideLine:'..#desc.list)
    return desc

--    return {list=pin2side(rd, side, cs, ce, ias, iae, sidemargin, pinstep),
--        w=wline*1.5, mat='m_line_white', lane={1, 0}}
--    return decalUp({list=pin2side(rd, side, cs, ce, ias, iae, sidemargin, pinstep),
--        w=wline*1.5, mat='m_line_white', lane={1, 0}})
end

--[[
local function exitUp(da, db, laneWidth, dexit)
    local rda,rdb = da.body,db.body
    if not dexit then dexit = default.rexit end
        lo('?? exitUp:'..tostring(dexit))
    -- distance between merge points
    local mergestep = dexit/2
    -- margin scale for 2nd merge point
    local cmargin = -0.1
--    local rda = a.rd
    local list = {}
    list[#list+1] = onSide(rda, 'left', dexit + mergestep, laneWidth/2)
    list[#list+1] = onSide(rda, 'left', dexit, laneWidth/2*cmargin)
--            out.avedit[#out.avedit+1] = pleft
    list[#list+1] = onSide(rdb, ' right', dexit, laneWidth/2*cmargin)
    list[#list+1] = onSide(rdb, ' right', dexit + mergestep, laneWidth/2)

--            out.avedit[#out.avedit+1] = pright
--            out.avedit = list
    local desc = {list=list, w=laneWidth*1, mat='checkered_line_alt', lane={1, 0}, frto={da.ind, db.ind}}
--    da.aex[#da.aex+1] = desc
--    db.aex[#db.aex+1] = desc

    return desc
end
]]
--[[
    local rd = decalUp(desc)
--    local junc = ajunc[cjunc]
--    junc.aexit[#junc.aexit+1] = {body=rd}

    -- set guide lines
--    local pinstep = 2
--    local sidemargin = 0.2
--    local wline = 0.1
    if false then
        junc.aline[#junc.aline+1] = {body=sideLine(rd, 'left', rda, 'left', rdb, 'right')}
        junc.aline[#junc.aline+1] = {body=sideLine(rd, 'right', rda, 'left', rdb, 'right')}

        junc.aline[#junc.aline+1] = {body=sideLine(rda, 'left', rd, 'left', rdb, 'right')}
        junc.aline[#junc.aline+1] = {body=sideLine(rdb, 'right', rda, 'left', rd, 'left')}
    end

    return rd
]]


local function nodesUpdate(rd, apos, frto, awidth, dbg)
    if dbg then return end
--        if true then return end
    if not rd or not rd.body then return end
    if not awidth then awidth = {} end
--        U.dump(awidth, '?? nodesUpdate:')
    if rd.on == false then
--            lo('?? nodesUpdate_HIDE:')
--        rd.body:setField('hidden', 0, 'true')
    else
--        rd.body:setField('hidden', 0, 'false')
    end
    if not apos then apos = rd.list end
    if not apos then return end
    local anode = editor.getNodes(rd.body)
    if #anode ~= #apos then
--        lo('!! ERR_nodesUpdate:'..#anode..'<'..#apos..':'..tostring(frto))
--        return
    end
--            U.dump(apos, '?? nodesUpdate:'..#anode..':'..#apos)
--                lo('?? e_UPD:'..junc.wexit) -- tostring(junc.aexit[i].body)..':'..#anode)
    for k,n in pairs(apos) do
        local w = awidth[k] or rd.w
--            w = 4*default.laneWidth
        editor.setNodePosition(rd.body, k-1, apos[k])
        editor.setNodeWidth(rd.body, k-1, w)
--                lo('?? for_width:'..k..':'..w)
        if k>#anode then
            editor.addRoadNode(rd.body:getID(), {
                pos = apos[k],
                width = awidth[k] or rd.body:getNodeWidth(#anode-1), drivability = 1, index = k-1})
            rd.aw[#rd.aw+1] = w
        end
    end
    dupd[rd.ind] = true
    if not frto then
--            lo('?? to_DEL:'..(#anode-#apos))
        for k=#anode-#apos,1,-1 do
            editor.deleteRoadNode(rd.body:getID(),#apos + k-1)
        end
--            lo('?? nodes_deleted:'..#editor.getNodes(rd.body))
    end
    editor.updateRoadVertices(rd.body)
--        lo('<< nodesUpdate:')
end
D.nodesUpdate = nodesUpdate

--[[
local function forExits(junc, yes)
--    junc.aexit = {}
--    local i = 1
--    if junc.aexit[]
    local function exitRender(edesc, i, j, inupdate)
--    local function exitOn(edesc, inupdate)

        if inupdate then
            -- update
            local d = junc.aexit[i]
            if d.dirty then
                nodesUpdate(d.body, edesc.list)
            end
        else
            -- create
            junc.aexit[#junc.aexit+1] = {ind=#junc.aexit+1, body=decalUp(edesc), desc=edesc}
            junc.list[i].aex[#junc.list[i].aex+1] = junc.aexit[#junc.aexit].ind
            junc.list[i+1].aex[#junc.list[i+1].aex+1] = junc.aexit[#junc.aexit].ind
        end
    end

    local inupdate = #junc.aexit>0 -- and true or false
    local ne,nl = 1,1
    for i = 1,1 do
        -- build exit
        local da,db = junc.list[i], junc.list[i+1]
        local edesc = exitUp(da, db, junc.wexit, junc.rexit)
    --    local edesc = exitUp(junc.list[1].body, junc.list[2].body, junc.wexit, junc.rexit)
    --        U.dump(junc.aexit, '?? for_el:'..junc.rexit)
        -- render exit
        exitRender(edesc, inupdate)
        local rda,rdb = junc.list[i].body,junc.list[i+1].body
        local ldesc = sideLine(junc.aexit[i].body, 'left', rda, 'left', rdb, 'right')
        if inupdate then
--            nodesUpdate(d.body, edesc.list)
        else
            junc.aline[#junc.aline+1] = {body=decalUp(ldesc)}
        end
    end
end
]]
--[[
        if inupdate then
            -- update
            local d = junc.aexit[i]
            if d.dirty then
                nodesUpdate(d.body, edesc.list)
            end
        else
            -- create
            junc.aexit[#junc.aexit+1] = {ind=#junc.aexit+1, body=decalUp(edesc), desc=edesc}
            junc.list[i].aex[#junc.list[i].aex+1] = junc.aexit[#junc.aexit].ind
            junc.list[i+1].aex[#junc.list[i+1].aex+1] = junc.aexit[#junc.aexit].ind
        end
]]


local function forNodes(bdesc, cb)
    local anode = editor.getNodes(bdesc.body)
    local ifr,dir = 1,1
    if bdesc.io < 0 then
        ifr,dir = #anode,-1
    end
    for i = ifr,#anode-ifr+1,dir do
        if cb(i, anode[i]) == false then break end
    end
end


local function forEdges(bdesc, side, cb)
end


local function toJunc(junc, ib)
    local bdesc = junc.list[ib]
--    local rd = bdesc.body
--        U.dump(bdesc, '>> toJunc:'..tostring(rd)..':'..bdesc.io)
    local anode = editor.getNodes(bdesc.body)
    forNodes(bdesc, function(i, n)
        local r = (n.pos-junc.p):length()
        if r <= default.rjunc+U.small_val then
            local p = junc.p + bdesc.dir*r
            p.z = 0
--                lo('?? for_node:'..i..':'..tostring(n.pos)..'>'..tostring(p)..' dir:'..tostring(bdesc.dir))
            editor.setNodePosition(bdesc.body, i-1, p)
            bdesc.desc.list[i] = p
        end
    end)
    editor.updateRoadVertices(bdesc.body)
end


--[[
    for i,d in pairs(desc.list) do
        d.body = decalUp(d.desc)
--        rd.lanesRight = default.laneNum
--        rd.lanesLeft = 2
    end
    for i,d in pairs(desc.aexit) do
        if d.dirty then
            lo('?? for_ex:'..i)
        end
    end
        exitUp(desc.list[1].body, desc.list[2].body, desc.wlane, desc.rexit)
]]
--[[
local function junctionUpdate_(jdesc, yes)
        lo('>> junctionUpdate:'..tostring(yes))
    for i,b in pairs(jdesc.list) do
        if b.dirty then
--                U.dump(b.aex, '?? jup:'..i)
            toJunc(jdesc, i)
            for _,ie in pairs(b.aex) do
--                lo('?? ex_hide:'..tostring(jdesc.aexit[ie].body.hidden))
                jdesc.aexit[ie].body:setField('hidden', 0, yes and 'false' or 'true')
                if yes then jdesc.aexit[ie].dirty = true end
                jdesc.aexit[ie].dirty = true
            end
        end
    end
--    if out.pdrag then
--        lo('?? jup:'..out.pdrag.ind)
--    end
--    desc.list[out.pdrag.ind]
--    if yes then
--    end
    forExits(jdesc, yes)
    for _,e in pairs(jdesc.aexit) do
        e.dirty = false
    end
end
]]

local function line4exitFree(edesc, a, b, sidea, sideb)
    local aline = {}
    aline[#aline+1] = sideLine(edesc, 'left',
        a, sidea, b, sideb,
        {w=default.wline*1.5, mat=default.matline, lane={1,0}})
    aline[#aline+1] = sideLine(edesc, 'right',
        a, sidea, b, sideb,
        {w=default.wline*1.5, mat=default.matline, lane={1,0}})

    return aline
end


local function line4exit(desc, a, b, forself)
    local aline = {}
    local hide -- = true
    aline[#aline+1] = sideLine(desc, 'left',
        a, a.io==1 and 'left' or 'right',
        b, b.io==1 and 'right' or 'left', {w=default.wline*1.5, mat=default.matline, lane={1,0}})
--            lo('?? line4exit:'..tostring(aline[#aline].list[1])..':'..tostring(toSide(aline[#aline].list[1],b)..'/'..(b.w/2)))
--            lo('?? line4exit:'..tostring(aline[#aline].list[1])..':'..tostring(toSide(aline[#aline].list[1],b,b.io==1 and 'right' or 'left')..'/'..(b.w/2)))
--            out.acyan = {aline[#aline].list[1]+vec3(0,0,forZ(aline[#aline].list[1]))}
--        lo('?? line1:'..tostring(aline[#aline].list))
    if #aline == 0 then
            lo('!! line4exit_NOLINE:')
        return {}
    end
    if aline[#aline].list and b.w then
--            lo('??^^^^^^^^^^^^ if_HIDE:'..toSide(aline[#aline].list[1],b)..'/'..(b.w/2 + 0.4))
        hide = toSide(aline[#aline].list[1],b) < (b.w/2 + 0.4)
--        hide = toSide(aline[#aline].list[1],b,b.io==1 and 'right' or 'left') < (b.w/2 + 0.4)
    end
--        hide = false
--            lo('??^^^^^^^^^^^^ if_HIDE:'..tostring(hide))
    aline[#aline].on = hide ~= true
    aline[#aline+1] = sideLine(desc, 'right',
        a, a.io==1 and 'left' or 'right',
        b, b.io==1 and 'right' or 'left', {w=default.wline*1.5, mat=default.matline, lane={1,0}})
--        lo('?? line2:'..tostring(aline[#aline].list))
--        lo('?? line4exit:'..tostring(aline[#aline]))
--    aline[#aline].hide = hide
    if not forself then
        aline[#aline+1] = sideLine(
            a, a.io==1 and 'left' or 'right', desc, 'left',
            b, b.io==1 and 'right' or 'left', {w=default.wline*1.5, mat=default.matline, lane={1,0}})
    --        lo('?? line3:'..tostring(aline[#aline].list))
    --        lo('?? line4exit:'..tostring(aline[#aline]))
        aline[#aline].on = hide ~= true
        aline[#aline+1] = sideLine(
            b, b.io==1 and 'right' or 'left',
            a, a.io==1 and 'left' or 'right',
            desc, 'left', {w=default.wline*1.5, mat=default.matline, lane={1,0}})
    --        lo('?? line4exit:'..tostring(aline[#aline]))
    --        lo('?? line4:'..tostring(aline[#aline].list))
        aline[#aline].on = hide ~= true
    end
--        lo('<< line4exit:'..tostring(aline[#aline].on)..':'..tostring(hide))

    return aline
end


-- exit connecting points pa,pb
local function forExitFree(a, b, pa, pb, sidea, sideb)
        lo('>> forExitFree:'..a.ind..':'..sidea..':'..b.ind..':'..sideb)
    pa = U.proj2D(pa)
    pb = U.proj2D(pb)
    local mergestep = 18
    local edesc = {
        w=default.wexit*1,
        mat=default.mat,
        lane={1, 0},
--        j=#ajunc+1, da=rexit, db=rexit,
        frto={a.ind, b.ind}}
    local e2da,e2db = edge2dist(a),edge2dist(b)
    -- projection to branches side
    local da,psa,iea = toSide(pa, a, sidea)
    local db,psb,ieb = toSide(pb, b, sideb)
--            toMark({psa,psb},'red',nil,0.1,0.6)
    -- distance from branch start
    da = e2da[iea] + pa:distance(epos(a.body,iea,sidea))
    db = e2db[ieb] + pb:distance(epos(b.body,ieb,sideb))
    local cmargin = -0.1

    cmargin = cmargin*pa:distance(pb)/10
    mergestep = math.max(mergestep,mergestep*pa:distance(pb)/50)

    pa = onSide(a,sidea,da,cmargin)
    pb = onSide(b,sideb,db,cmargin)
    -- merge-plus points on branches
    local pma = onSide(a,sidea,da + (sidea == 'right' and -1 or 1)*mergestep,edesc.w/2)
    local pmb = onSide(b,sideb,db + (sideb == 'right' and 1 or -1)*mergestep,edesc.w/2)
            toMark({pa,pb},'cyan',nil,0.1,0.6)
--        pma = onSide(a,sidea,e2da[iea])
--        lo('?? at_A:'..iea..':'..pa:distance(epos(a.body,iea,sidea))..':'..tostring(pma)..':'..tostring(pa))
--        toMark({psa,psb}, 'blue', nil, 0.1, 0.5)
--        toMark({epos(b.body,ieb,sideb)}, 'red', nil, 0.1, 0.5)
--        toMark({pma,pmb}, 'red', nil, 0.1, 0.5)
--        toMark({epos(a.body,iea,sidea)}, 'red', nil, 0.1, 0.5)

    local list = {pma,pa,pb,pmb}
--            if true then return end
    -- middle node
    local x,y = U.lineCross(list[1],list[2],list[#list],list[#list-1])
    local cross = vec3(x,y)
    local d,p = U.toLine(cross, {list[2],list[3]})
    table.insert(list, 3, (list[2]+list[3])/2 + (cross-p)*8/15)
--        toMark(list, 'red', nil, 0.1, 0.5)

--    list[#list+1] = onSide(a,sidea)
    edesc.list = list

    decalUp(edesc)
    if true then
        if not a.aexo then a.aexo = {} end
        if not b.aexi then b.aexi = {} end
        a.aexo[#a.aexo+1] = edesc.ind or 0
        b.aexi[#b.aexi+1] = edesc.ind or 0
    end
    local ca = borderCross(edesc.body, a.body, 'left', sidea, 0, 0)
    local cb = borderCross(edesc.body, b.body, 'left', sideb, 0, 0)
--            toMark({ca,cb},'yel',nil,0.1,0.4)
    local inset = 3 -- extra distance from border crosses to inside of exit
    if true then
        local t
        local e2de = edge2dist(edesc)
        local dfr,pfr,ifr = toSide(ca, edesc, 'middle')
        local dto,pto,ito = toSide(cb, edesc, 'middle')
                lo('?? if_SIDE:'..tostring(dfr)..':'..tostring(dto))
        dfr = e2de[ifr] + pfr:distance(epos(edesc.body,ifr)) + inset
        dto = e2de[ito] + pto:distance(epos(edesc.body,ito)) - inset
        pfr = onSide(edesc,'middle',dfr)
        pto = onSide(edesc,'middle',dto)
--            toMark({pfr,pto},'yel',nil,0.1,0.4)
        t,pfr = toSide(pfr, a, sidea)
        t,pto = toSide(pto, b, sideb)
        edesc.frtoside = {
            {ind=a.ind, d=dfr, p=pfr, side=sidea},
            {ind=b.ind, d=dto, p=pto, side=sideb},
--            {ind=a.ind, d=dfr, p=pfr, side=sidea},
--            {ind=b.ind, d=dto, p=pto, side=sideb},
        }
    end

-- lines
--    edesc.aline = {}
--    line4exit(edesc, rda, rdb)
    if true then
        -- lines
        edesc.aline = line4exitFree(edesc, a, b, sidea, sideb)
            lo('?? if_LINES:'..#edesc.aline)
        for i,ldesc in pairs(edesc.aline) do
            decalUp(ldesc)
        end
    end
--    a.aexo[#a.aexo] = nil
--    b.aexi[#b.aexi] = nil

    return edesc
end
D.forExitFree = forExitFree


--TODO: unify with forExit
-- brach2branch exit
local function forExitB(a, b, desc, isround, dbg) -- w, da, db)
--        lo('>> forExit:')
--    if not ind then ind = 1 end
--    if not w then w = default.laneWidth end
    local w = desc.w
    local rda,rdb = a.body,b.body
--    if not da then da = default.rexit end
--    if not db then db = da end
    local da,db = desc.da,desc.db
--        lo('?? exitUp:'..tostring(da))
    -- distance between merge points
--    local mergestepa,mergestepb = math.sqrt(da/2)+da/2,math.sqrt(db/2)+da/2
    local mergestepa,mergestepb = da/2,db/2
    if desc.stepa then mergestepa = desc.stepa end
    if desc.stepb then mergestepb = desc.stepb end

    local cmargin = desc.cmargin or -0.1
    local sidea = a.io == 1 and 'left' or 'right'
    local sideb = b.io == 1 and 'right' or 'left'
    if isround then
        sidea = 'left'
        sideb = 'right'
    end
    -- correct the inner merge point margin
    if not isround then
        cmargin = cmargin + (1-da/20)*w/3
        da = a.io==1 and da or roadLength(a,sidea)-da
        db = b.io==1 and db or roadLength(b,sideb)-db
    end
    -- margin scale for 2nd merge point
--    local margin
--    local sidea = desc.sidea or 'left'
--    local rda = a.rd
    local list,iin = {}
--[[
            if list[#list] then
                lo('?? for_side:'..sidea..':'..da..':'..w)
                out.ared = {list[#list]}
            end
]]
--    local inmargin = w -- (1-da/20)*w/3
    local dshift = 0
    if isround and da+mergestepa < 0 then
        dshift = 2*math.pi*(a.r+a.w/2)
--        da = 2*math.pi*a.r + da
    end
            if dbg then lo('?? forExit_w:'..w..':'..da..':'..mergestepa..':'..tostring(desc.stepa)..':'..a.r..'/'..(a.r+a.w/2-w/2)) end
--            lo('??***************** forExit_for_a:'..sidea..':'..da..':'..mergestepa..':'..(w/2)..' L:'..a.L..':'..(da)..':'..cmargin)
    list[#list+1] = onSide(a, sidea, da + (isround and 1 or a.io)*mergestepa + dshift, w/2)
    list[#list+1] = onSide(a, sidea, da, w/2*cmargin) --, (20 ))
--            out.avedit[#out.avedit+1] = pleft
    list[#list+1] = onSide(b, sideb, db, w/2*cmargin)
    list[#list+1],iin = onSide(b, sideb, db + (isround and 1 or b.io)*mergestepb, w/2)
--            toMark({list[3],list[4]}, 'blue', nil, 0.1, 0.5)

    -- middle
    local x,y = U.lineCross(list[1],list[2],list[#list],list[#list-1])
    local cross = vec3(x,y)
    local d,p = U.toLine(cross, {list[2],list[3]})
--    local u = ((list[2]-list[1]):normalized() + (list[3]-list[4]):normalized()):normalized()
--    (list[2]+list[3])/2 +
--            out.avedit = {list}
--        out.avedit = {cross, p}
    table.insert(list, 3, (list[2]+list[3])/2 + (cross-p)*6/15)
--            out.avedit = {list[1], list[2], list[3], list[4]}
--            out.avedit[#out.avedit+1] = vec3(x,y)

    desc.list = list
    desc.iin = iin -- edge index for exit in
    desc.frto = {a.ind,b.ind}

--[[
    if not a.aexo then a.aexo = {} end
    if not b.aexi then b.aexi = {} end
    if #U.index(a.aexo, desc.ind) == 0 then
        a.aexo[#a.aexo+1] = desc.ind
    end
    if #U.index(b.aexi, desc.ind) == 0 then
        b.aexi[#b.aexi+1] = desc.ind
    end
        lo('?? inex:'..#a.aexo)
]]

--            out.avedit[#out.avedit+1] = pright
--            out.avedit = list
--    local desc = {list=list, w=w*1, mat='checkered_line_alt', lane={1, 0}, frto={a.ind, b.ind}}
--    da.aex[#da.aex+1] = desc
--    db.aex[#db.aex+1] = desc
--    if not a.aex then a.aex = {} end
--    a.aex[ind] = desc
--        lo('<< forExit:')
    return desc
end


-- branch to/from roundabout exit
local function forExit(a, b, desc, isround, dbg) -- w, da, db)
--    if not ind then ind = 1 end
--    if not w then w = default.laneWidth end
    local w = desc.w
    local rda,rdb = a.body,b.body
--    if not da then da = default.rexit end
--    if not db then db = da end
    local da,db = desc.da,desc.db
        lo('>> forExit:'..tostring(isround)..':'..tostring(a.ij)..':'..tostring(b.r)..' da:'..da..' db:'..db..' dbg:'..tostring(dbg))
--        lo('?? exitUp:'..tostring(da))
    -- distance between merge points
--    local mergestepa,mergestepb = math.sqrt(da/2)+da/2,math.sqrt(db/2)+da/2
    local mergestepa,mergestepb = da/2,db/2
    if desc.stepa then mergestepa = desc.stepa end
    if desc.stepb then mergestepb = desc.stepb end
    local sidea = desc.sidea or 'left'
    local sideb = 'right'
    if not a.r then
        sidea = a.io == 1 and 'left' or 'right'
    end
    if not b.r then
        sideb = b.io == 1 and 'right' or 'left'
    end
--    local L
    if not a.r then
        --- set distance
        da = a.io == 1 and da or roadLength(a,sidea)-da
    end
    if not b.r then
        --- set distance
        db = b.io == 1 and db or roadLength(b,sideb)-db
--        db = b.io == 1 and db or b.L-db
    end
    -- margin scale for 2nd merge point
--    local margin
    local cmargin = desc.cmargin or -0.1
--    local rda = a.rd
    local list,iin = {}
    -- correct the inner merge point margin
--    if not isround then
--        cmargin = cmargin + (1-da/20)*w/3
--    end
    local dshift = 0
    if isround and da+mergestepa < 0 then
        dshift = 2*math.pi*(a.r+a.w/2)
--        da = 2*math.pi*a.r + da
    end
            if dbg then lo('?? forExit_w:'..tostring(b.io)..':'..w..' da:'..da..' db:'..db..'/'..b.L..':'..mergestepa..':'..tostring(desc.stepa)..':'..tostring(a.r)) end --..'/'..(a.r+a.w/2-w/2)) end
    list[#list+1] = onSide(a, sidea, da + (not a.r and a.io or 1)*mergestepa + dshift, w/2)
    list[#list+1] = onSide(a, sidea, da, w/2*cmargin) --, (20 ))
--            out.avedit[#out.avedit+1] = pleft
    list[#list+1] = onSide(b, sideb, db, w/2*cmargin)
--            lo('?? for_LAST3:'..tostring(list[#list]))
--            if dbg then toMark({list[#list], b.list[#b.list]}, 'red', nil, 0.1, 0.5) end
    list[#list+1],iin = onSide(b, sideb, db + (not b.r and b.io or 1)*mergestepb, w/2)
    -- middle
    local x,y = U.lineCross(list[1],list[2],list[#list],list[#list-1])
    local cross = vec3(x,y)
    local d,p = U.toLine(cross, {list[2],list[3]})
    table.insert(list, 3, (list[2]+list[3])/2 + (cross-p)*6/15)
--            out.avedit = {list[1], list[2], list[3], list[4]}
--            out.avedit[#out.avedit+1] = vec3(x,y)

    desc.list = list
    desc.iin = iin -- edge index for exit in
    desc.frto = {a.ind,b.ind}

        lo('<< forExit:')
    return desc
end
--[[
            if list[#list] then
                lo('?? for_side:'..sidea..':'..da..':'..w)
                out.ared = {list[#list]}
            end
]]
--    local inmargin = w -- (1-da/20)*w/3
--    local u = ((list[2]-list[1]):normalized() + (list[3]-list[4]):normalized()):normalized()
--    (list[2]+list[3])/2 +
--            out.avedit = {list}
--        out.avedit = {cross, p}
--[[
    if not a.aexo then a.aexo = {} end
    if not b.aexi then b.aexi = {} end
    if #U.index(a.aexo, desc.ind) == 0 then
        a.aexo[#a.aexo+1] = desc.ind
    end
    if #U.index(b.aexi, desc.ind) == 0 then
        b.aexi[#b.aexi+1] = desc.ind
    end
        lo('?? inex:'..#a.aexo)
]]

--            out.avedit[#out.avedit+1] = pright
--            out.avedit = list
--    local desc = {list=list, w=w*1, mat='checkered_line_alt', lane={1, 0}, frto={a.ind, b.ind}}
--    da.aex[#da.aex+1] = desc
--    db.aex[#db.aex+1] = desc
--    if not a.aex then a.aex = {} end
--    a.aex[ind] = desc


-- position on ilane at a dist d from start
--local function dist2lane(d, rd, ilane)
--end
local function line4lane(rd, k, dw, mat)
    if not mat then mat = default.matline end
    local margin = rd.w/(rd.lane[1] + rd.lane[2])*k - rd.w/2
--        lo('?? line4lane:'..k..':'..margin..':'..rd.w)
    local ldesc
    if rd.ij then
        -- for branch
        local u = (epos(rd.body,0,'left') - epos(rd.body,0,'right')):normalized()
        ldesc = {
            list = {rd.list[1] + rd.dir*dw + margin*u, rd.list[#rd.list] + margin*u},
            w = default.wline*1.5, mat = mat, lane = {1, 0}}
        return ldesc
    end
--    decalUp(ldesc)
end

-- ilane counts from right side
local function line4laneD(rd, ilane, dr, mat, dbg)
    if not mat then mat = default.matline end
    local margin = rd.w/(rd.lane[1] + rd.lane[2])*ilane -- - rd.w/2
--        U.dump(rd.lane, '?? line4laneD:'..rd.w..':'..ilane..':'..margin)
--        lo('?? line4laneD:'..rd.L..':'..dr)
    local u = (epos(rd.body,0,'left') - epos(rd.body,0,'right')):normalized()
    local drb,dre = rd.djrad[1],rd.djrad[#rd.list]
    local ldesc = {
        list = pin2sideD(rd, 'right', drb or 0, rd.L-(dre or 0), margin, nil, dbg),
--        list = pin2sideD(rd, 'right', dr, rd.L-dr, margin, nil, dbg),
        w = default.wline*1.5, mat = mat, lane = {1, 0}}

    return ldesc
end


local function junctionRound_(r, w)
    if not cjunc then return end
    local jdesc = ajunc[cjunc]
    local toupdate = jdesc.round and true or false
    if not w then
        w = toupdate and jdesc.round.w or default.laneWidth*2
    end
    if not r then
        r = toupdate and jdesc.round.r or default.rround
    end
        lo('>> junctionRound:'..r..':'..tostring(toupdate))
--    for i=1,1 do
--        local b = jdesc.list[i]
    -- circle
    local list = {}
    local nstep = 16
    for i = 0,nstep+2 do
        list[#list+1] = jdesc.p + r*vec3(math.cos(i*2*math.pi/nstep), math.sin(i*2*math.pi/nstep))
    end
    if not jdesc.round then
        jdesc.round = {list = list, r=r, w = w, lane={math.floor(w/default.laneWidth+0.5),0}, aexo={}, aexi={}}
        decalUp(jdesc.round)
    else
        toupdate = true
        local awidth = {}
        jdesc.round.w = w
        jdesc.round.r = r
--        for i=1,#list do
--            awidth[#awidth+1] = w
--        end
        nodesUpdate(jdesc.round, list, nil, awidth)
    end

    for i,b in pairs(jdesc.list) do
        local nodelast = b.io == 1 and b.list[#b.list] or b.list[1]
        local toend = (jdesc.p-nodelast):length()
        if true then
            local apos = {}
--[[
]]
            if toupdate then
--                U.dump(b.listrad, '?? LISTRAD:'..i)
            else
                -- save radial nodes
                b.listrad = deepcopy(b.list)
            end
            forNodes(b, function(i, n)
                local x = (b.listrad[i]-jdesc.p):length()
--                local x = (n.pos-jdesc.p):length()
                x = r + (toend-r)/toend*x
--                    dist = r + dist*(toend-(dist))/(toend-r)
                local p = jdesc.p + b.dir*x
                p.z = 0
    --                lo('?? for_node:'..i..':'..tostring(n.pos)..'>'..tostring(p)..' dir:'..tostring(bdesc.dir))
--                    editor.setNodePosition(bdesc.body, i-1, p)
                apos[#apos+1] = p

                if x <= r+default.rjunc+U.small_val then
--                        lo('?? frto:'..i..':'..x..'>'..(r + (toend-r)/toend*x))
    --                    bdesc.desc.list[i] = p
                end
            end)
--                out.avedit = apos
--                lo('?? for_apos:'..#apos)
            nodesUpdate(b, apos, {1,#apos})
--            b.listrad = deepcopy(b.list)
            b.list = apos
--[[
                out.avedit = {}
                for k,p in pairs(b.list) do
--                    lo('?? for_list:'..k)
                    out.avedit[#out.avedit+1] = p
                end
]]
--                out.avedit = b.list -- {b.list[2]}
--                U.dump(b.list, '?? b_LIST:'..i..':'..#b.list)
--                out.avedit = b.listrad
        end

        local ang = U.vang(jdesc.round.list[1]-jdesc.p,b.dir, true)
        if ang < 0 then
            ang = 2*math.pi + ang
        end

        -- LANE LINES update
        local il = 1
        -- middle line
--            U.dump(b.aline[il].list, '?? pre_ml:')
        for i = #b.list,#b.listrad do
            b.list[#b.list+1] = b.listrad[i]
        end
--[[
                out.avedit = {}
                for k,p in pairs(b.listrad) do
--                    lo('?? for_list:'..k)
                    out.avedit[#out.avedit+1] = p
                end
]]

        local ldesc = line4lane(b, b.lane[1], jdesc.round.w/2)
            U.dump(ldesc.list, '?? for_middle:'..i..':'..il)
--            out.avedit = b.listrad
--            out.ared = {ajunc[cjunc].p}-- ldesc.list
--            U.dump(b.list)
--            U.dump(ldesc.list, '??++++++++++++ for_ml:'..#b.list)
--                out.avedit = {b.list[#b.list]}
        nodesUpdate(b.aline[il], ldesc.list)
        il = il + 1
        -- lanes right
        for k=1,b.lane[1]-1 do
            ldesc = line4lane(b, k, jdesc.round.w/2, default.matlinedash)
            nodesUpdate(b.aline[il], ldesc.list)
            il = il + 1
        end
        -- lanes left
        for k=1,b.lane[2]-1 do
            ldesc = line4lane(b, k+b.lane[1], jdesc.round.w/2, default.matlinedash)
            nodesUpdate(b.aline[il], ldesc.list)
            il = il + 1
        end
        for il,d in pairs(adec[b.aexo[1]].aline) do
            d.body:setField('hidden', 0, 'true')
        end

--            lo('?? if_EXITS:'..tostring(b.aexo and #b.aexo or 0))
--[[
                if false then
                    for il,d in pairs(adec[b.aexo[1] ].aline) do
                        d.body:setField('hidden', 0, 'true')
                    end
                    for il,d in pairs(adec[b.aexi[1] ].aline) do
                        d.body:setField('hidden', 0, 'true')
                    end
                    for il,d in pairs(b.aline) do
                        d.body:setField('hidden', 0, 'true')
                    end
                end
]]
        -- build exits
        for _,idec in pairs(b.aexo) do
            local e = adec[idec]
--                    U.dump(e, '?? ex_upd:'.._)
--                U.dump(e, '?? for_exito:'..i..':'..e.frto[1]..'>'..e.frto[2]..':'..jdesc.round.in d..':'..e.db..':'..U.vang())
--                        U.dump(b.aline, '?? for_line:')
            -- get distance on circle
--            local p -- = line2side(jdesc.p, b.list[1], jdesc.round, 'right')
            local edist = math.min(default.laneWidth*2, jdesc.round.w) -- 10*0.6
            local estep = edist --6*0.6
            e.cmargin = 0.9
            e.w = edist
--            e.lane = {math.floor(e.w/default.laneWidth+0.5),0}
            e.lane = {2,0}
--                lo('??_____________________ is ang:'..i..':'..ang..' w:'..e.w)
            e.da = (jdesc.round.w/2 + edist)*1
            e.db = (r+jdesc.round.w/2)*ang+b.w/2 + edist
            e.stepa = estep*1.5
            e.stepb = estep*1
            e.mat = 'road1_concrete'
--                U.dump(e, '?? for_e:')
            forExit(adec[e.frto[1]], jdesc.round, e)
            if not toupdate then
                U.pop(adec[e.frto[2]].aexo, e.ind)
                jdesc.round.aexi[#jdesc.round.aexi+1] = e.ind
            end
            nodesUpdate(e)
            e.body:setField('material', 0, e.mat)

--            local p = onSide(jdesc.round, 'right', (r+jdesc.round.w/2)*ang + b.w/2 + edist)
--            if p then
--                     out.avedit = {p}
--            end
            if true then
                local eleft = toupdate and adec[b.aexi[1]] or {}
                eleft.stepa = -e.stepb
                eleft.stepb = e.stepa
                eleft.da = (r+jdesc.round.w/2)*(ang == 0 and 2*math.pi or ang) - b.w/2 - edist
--                        lo('??^^^ if_upd:'..i..':'..tostring(toupdate)..' ang:'..ang..':'..edist..' da:'..eleft.da)
                if eleft.da < 0 then
                    eleft.da = 2*math.pi*(r+jdesc.round.w/2) + eleft.da
--                    eleft.da = math.pi*3/2*(r+jdesc.round.w/2)
--                        lo()
--                    local pons = onSide(jdesc.round, 'right', 2*math.pi + eleft.da - 8, 0) -- math.pi*4/2*(r+jdesc.round.w/2), 0)
--                    lo('?? pons:'..tostring(pons))
--                    if pons then
--                        out.avedit = {pons}
--                    end
                end
                eleft.db = e.da
                eleft.sidea = 'right'
                eleft.cmargin = e.cmargin
                eleft.mat = e.mat
                eleft.w = e.w
                eleft.lane = e.lane

                    lo('??________________ pre_EXIT:'..i)
                forExit(jdesc.round, adec[e.frto[1]], eleft)
                if toupdate then
                    nodesUpdate(eleft)
                else
                    decalUp(eleft)
--                        out.avedit = eleft.list
                    jdesc.round.aexo[#jdesc.round.aexo+1] = eleft.ind
                    --TODO: append
                    b.aexi = {eleft.ind}
                end
            end
--            eleft.frto = {jdesc.round.ind,b.ind}
--[[
                    local p = onSide(jdesc.round, 'right', (r+jdesc.round.w/2)*ang - b.w/2 - edist, 0)
                        lo('?? for_p:'..tostring(p))
                    out.avedit = {p}
]]
--                    out.avedit = {adec[jdesc.round.ind].list[1]}

            if false then
                forExit(adec[e.frto[1]], adec[e.frto[2]], e)
    --                    forExit(jdesc.list[e.frto[1]], jdesc.list[e.frto[2]], e)
                nodesUpdate(e)
                -- lines right
                for k,l in pairs(b.aline) do
                    l.on = true
                end
--                    lo('?? for_lines_o:'.._)
                local bout,bin = b,U.mod(i+1,jdesc.list)
                nodesUpdate(sideLine(e, 'left', bout, 'left', bin, 'right', b.aline[1]))
                nodesUpdate(sideLine(e, 'right', bout, 'left', bin, 'right', b.aline[2]))
                nodesUpdate(sideLine(bout, 'left', e, 'left', bin, 'right', b.aline[3]))
                nodesUpdate(sideLine(bin, 'right', bout, 'left', e, 'left', b.aline[4]))
            end

--                    sideLine(e, 'right', bout, 'left', bin, 'right', {w=default.wline*1.5, mat='m_line_white', lane={1, 0}})
--                    sideLine(bout, 'left', e, 'left', bin, 'right', {w=default.wline*1.5, mat='m_line_white', lane={1, 0}})
--                    sideLine(bin, 'right', bout, 'left', e, 'left', {w=default.wline*1.5, mat='m_line_white', lane={1, 0}})
--                        lo('??^^^^^^^^^^ bu1:')
        end

    end
    -- exit lines
    local c
    local aline,line,ex,rdfr,rdto,newline
    for i,br in pairs(jdesc.list) do
        if true then
            --- LEFT
            ex = adec[br.aexi[1]]
            rdfr = jdesc.round
            rdto = br
            if c then
                rdfr = adec[U.mod(i-1,jdesc.list).aexo[1]]
--                rdto = adec[U.mod(i+1,jdesc.list).aexi[1]]
--                out.avedit = {c}
            end
            if i>1 then
                newline = sideLine(ex, 'right', rdfr, 'right', rdto, 'right', {w=default.wline*1.5, mat=default.matline, lane={1,0}})
--                    lo('??_______________ if_line:'..i..':'..tostring(newline)..':'..tostring(ex)..':'..#br.aexi..':'..tostring(ex.aline and #ex.aline or 0))
            end
--                out.avedit = ex.list
--            line = ex.aline[1]
--            line.list = newline.list
            if newline then
                if toupdate then
                    ex.aline[1].list = newline.list
                    nodesUpdate(ex.aline[1])
                else
                    decalUp(newline)
                    if not ex.aline then ex.aline = {} end
                    ex.aline[#ex.aline+1] = adec[#adec]
--                    br.aexi[1] = adec[#adec]
                end
            end
--            nodesUpdate(line)
--            line.body:setField('hidden', 0, 'false')

            --- RIGHT
            ex = adec[br.aexo[1]]
            c = borderCross(ex.body, adec[U.mod(i+1,jdesc.list).aexi[1]].body, 'right', 'right', 0, 0, default.sidemargin)
--                out.avedit = {c}
    --            lo('?? diff:'..c:distance(aline[1].list[#aline[1].list]))
    --        aline[1].list[#aline[1].list] = c
            rdto = jdesc.round
            if c then
                rdto = adec[U.mod(i+1,jdesc.list).aexi[1]]
            end
            newline = sideLine(ex, 'right', br, 'left', rdto, 'right', {w=default.wline*1.5, mat=default.matline, lane={1,0}})

            line = ex.aline[2]
            if newline then
                line.list = newline.list
                nodesUpdate(line)
                line.body:setField('hidden', 0, 'false')
            else
                lo('!! ERR_NO_LINE:'..i)
            end

--                    lo('??_____ if_LINE:'..tostring(line)..':'..tostring(aline[1]))

--            adec[br.aexo[1]].aline[2].list = aline[1].list
    --        aline = line4exit(adec[b.aexo[1]], b, jdesc.round)
    --        line = adec[b.aexo[1]].aline[2]
    --        line.list = aline[1].list

    --        line.list[#line.list] = c -- line.list[#line.list] + vec3(5,5)
    --            out.avedit = {line.list[#line.list]}

    --        nodesUpdate(line)
            --- left
--                lo('?? exi:'..#b.aexi)
--            aline = line4exit(adec[b.aexi[1]], jdesc.round, b)
--                lo('?? al:'..tostring(aline))
    --            U.dump(aline, '?? line2:'..#adec[b.aexo[1]].aline)
    --            out.avedit = aline[1].list
        end
    end
    -- last left line
    local i,br =1,jdesc.list[1]
    ex = adec[br.aexi[1]]
    rdfr = jdesc.round
    rdto = br
    if c then
        rdfr = adec[U.mod(i-1,jdesc.list).aexo[1]]
--        out.avedit = {c}
    end
    newline = sideLine(ex, 'right', rdfr, 'right', rdto, 'right', {w=default.wline*1.5, mat=default.matline, lane={1,0}})
    if newline then
        if toupdate then
            ex.aline[1].list = newline.list
            nodesUpdate(ex.aline[1])
        else
            decalUp(newline)
            if not ex.aline then ex.aline = {} end
            ex.aline[#ex.aline+1] = adec[#adec]
        end
    end

    jdesc.rround = r

        lo('<< junctionRound:')
end


local function junctionRound(r, w, dbg)
    if not cjunc then return end
    local jdesc = ajunc[cjunc]
            lo('??========================= jR:'..tostring(r)..':'..#jdesc.list[1].list..':'..tostring(jdesc.r))
--    if not r then
--        r = jdesc.r or default.rround
--    end
    local toupdate = jdesc.round and true or false
    if not w then
            lo('?? jRw:'..tostring(jdesc.round and jdesc.round.w or nil))
        w = toupdate and jdesc.round.w or default.laneWidth*2
    end
    if not r then
        if toupdate then
            r = jdesc.round.r or default.rround
        else
            r = jdesc.r or default.rround
        end
    end
        lo('>> junctionRound:'..r..':'..tostring(jdesc.round and jdesc.round.r or nil)..':'..tostring(toupdate)..':'..tostring(jdesc.list[1].listrad))
--    for i=1,1 do
--        local b = jdesc.list[i]
    -- circle
    local ang -- = U.vang(vec3(1,0,0), jdesc.list[1].dir, true)
--        lo('?? for_ang0:'..tostring(ang)..':'..tostring(jdesc.list[1].dir))
    local list = {}
    local nstep = 16
    for i = 0,nstep+2 do
        list[#list+1] = jdesc.p + r*vec3(math.cos(i*2*math.pi/nstep), math.sin(i*2*math.pi/nstep))
--        list[#list+1] = jdesc.p + r*vec3(math.cos(i*2*math.pi/nstep+ang-math.pi/2), math.sin(i*2*math.pi/nstep+ang-math.pi/2))
    end
    if not jdesc.round then
        jdesc.round = {list = list, r=r, w = w, lane={math.floor(w/default.laneWidth+0.5),0}, aexo={}, aexi={}}
        decalUp(jdesc.round)
    else
        toupdate = true
            lo('?? for_WIDTH:'..jdesc.round.w..':'..w)
        jdesc.round.w = w
        jdesc.round.r = r
        local awidth = {}
        for i=1,#list do
            awidth[#awidth+1] = w
        end
        nodesUpdate(jdesc.round, list, nil, awidth, dbg)
        jdesc.round.body:setPosition(jdesc.round.body:getPosition())
--        editor.updateRoadVertices(b.body)
    end
--        if toupdate then return end
--    for i = 3,3 do
--        local b = jdesc.list[i]
    for i,b in pairs(jdesc.list) do
--                lo('??+++++++++++++ branch_upd0:'..i..':'..#b.list)
        local nodelast = b.io == 1 and b.list[#b.list] or b.list[1]
        local toend = (jdesc.p-nodelast):length()
        if true then
                lo('?? for_B:'..i..':'..b.L)
            local apos = {}

            if toupdate then
--                U.dump(b.listrad, '?? LISTRAD:'..i)
            else
                -- save radial nodes
                b.listrad = deepcopy(b.list)
            end
            local list = b.listrad or b.list
--                    toMark(list, 'blue')
-- move branch nodes
            if U._MODE == 'conf' then
                local ndi = b.io==1 and 1 or #b.list
                b.list[ndi] = jdesc.p + b.dir*r
                if b.list[ndi]:distance(b.list[ndi+b.io]) < b.w or (b.list[ndi+b.io]-b.list[ndi]):dot(b.dir)<0 then
                    lo('??*************** to_CUT:'..i)
                    table.remove(b.list,ndi+b.io)
                    b.listrad = deepcopy(b.list)
                end
--                    apos[#apos+1] = b.list[ndi]
                nodesUpdate(b)
--                        lo('??+++++++++++++ branch_upd:'..i..':'..#b.list)
--                nodesUpdate(b, apos, {1,#apos}, nil)
            else
                forNodes(b, function(i, n)
                    local x = (list[i]-jdesc.p):length()
    --                local x = (n.pos-jdesc.p):length()
                    x = r + (toend-r)/toend*x
    --                    dist = r + dist*(toend-(dist))/(toend-r)
                    local p = jdesc.p + b.dir*x
                    p.z = 0
        --                lo('?? for_node:'..i..':'..tostring(n.pos)..'>'..tostring(p)..' dir:'..tostring(bdesc.dir))
    --                    editor.setNodePosition(bdesc.body, i-1, p)
                    apos[#apos+1] = p

                    if x <= r+default.rjunc+U.small_val then
    --                        lo('?? frto:'..i..':'..x..'>'..(r + (toend-r)/toend*x))
        --                    bdesc.desc.list[i] = p
                    end
                end)
                nodesUpdate(b, apos, {1,#apos}, nil)
    --            b.listrad = deepcopy(b.list)
                b.list = apos
            end
            b.L = roadLength(b)
                lo('?? for_B_:'..i..':'..b.L)
--                toMark(apos, 'mag')
--                out.avedit = apos
--                U.dump(apos, '?? apos:')
--                lo('?? for_apos:'..#apos)
--[[
                out.avedit = {}
                for k,p in pairs(b.list) do
--                    lo('?? for_list:'..k)
                    out.avedit[#out.avedit+1] = p
                end
]]
--                out.avedit = b.list -- {b.list[2]}
--                U.dump(b.list, '?? b_LIST:'..i..':'..#b.list)
--                out.avedit = b.listrad
        end
--            lo('??____FOR_BR1:'..#jdesc.list[1].list)
        ang = U.vang(jdesc.round.list[1]-jdesc.p,b.dir, true)
        if ang < 0 then
            ang = 2*math.pi + ang
        end
--                lo('?? for_BBB2:'..i..':'..#b.list..'/'..#b.listrad..' aline:'..#b.aline)
        --- compete nodes
--            U.dump(b.aline[il].list, '?? pre_ml:')
        for i = 1,#b.listrad-#b.list do
--        for i = #b.list,#b.listrad do
            b.list[#b.list+1] = b.listrad[#b.list+i]
        end
--            lo('??____FOR_BR2:'..i..':'..#b.list..':'..#jdesc.list[1].list)
--[[
                out.avedit = {}
                for k,p in pairs(b.listrad) do
--                    lo('?? for_list:'..k)
                    out.avedit[#out.avedit+1] = p
                end
]]
-- LANE LINES update
        local il = 1
        -- middle
        local ldesc = U._MODE == 'conf' and line4laneD(b, b.lane[1], jdesc.round.w/2) or line4lane(b, b.lane[1], jdesc.round.w/2)
--            U.dump(b.lane, '?? for_middle:'..i..':'..il..':'..tostring(#b.aline))
--            toMark(ldesc.list, 'yel')
--            toMark(b.aline[il].list, 'yel')
--            out.avedit = b.listrad
--            out.ared = {ajunc[cjunc].p}-- ldesc.list
--            U.dump(b.list)
--            U.dump(ldesc.list, '??++++++++++++ for_ml:'..#b.list)
--                out.avedit = {b.list[#b.list]}
        nodesUpdate(b.aline[il], ldesc.list)
--            if toupdate then return end

        if il < #b.aline then
            -- lanes right
            il = il + 1
            for k=1,b.lane[1]-1 do
                ldesc = line4laneD(b, k, jdesc.round.w/2, default.matlinedash)
                nodesUpdate(b.aline[il], ldesc.list)
                il = il + 1
            end
            -- lanes left
            for k=1,b.lane[2]-1 do
                ldesc = line4laneD(b, k+b.lane[1], jdesc.round.w/2, default.matlinedash)
                nodesUpdate(b.aline[il], ldesc.list)
                il = il + 1
            end
        end
        if #b.aexo > 0 and adec[b.aexo[1]] and adec[b.aexo[1]].aline then
            for il,d in pairs(adec[b.aexo[1]].aline) do
                d.body:setField('hidden', 0, 'true')
            end
        end
--            if U._MODE == 'conf' then break end

--            lo('?? if_EXITS:'..tostring(b.aexo and #b.aexo or 0))
--[[
                if false then
                    for il,d in pairs(adec[b.aexo[1] ].aline) do
                        d.body:setField('hidden', 0, 'true')
                    end
                    for il,d in pairs(adec[b.aexi[1] ].aline) do
                        d.body:setField('hidden', 0, 'true')
                    end
                    for il,d in pairs(b.aline) do
                        d.body:setField('hidden', 0, 'true')
                    end
                end
]]
--                lo('?? for_B4:'..#b.aexo)
        -- build exits
        for _,idec in pairs(b.aexo) do
                lo('??******* for_EXo:'..i..':'..idec)
-- update RIGHT
            local e = adec[idec] or {}
--                    U.dump(e, '?? ex_upd:'.._)
--                U.dump(e, '?? for_exito:'..i..':'..e.frto[1]..'>'..e.frto[2]..':'..jdesc.round.in d..':'..e.db..':'..U.vang())
--                        U.dump(b.aline, '?? for_line:')
            -- get distance on circle
--            local p -- = line2side(jdesc.p, b.list[1], jdesc.round, 'right')
            local edist = math.min(default.laneWidth*2, jdesc.round.w) -- 10*0.6
            local estep = edist --6*0.6
            e.cmargin = 0.9
            e.w = edist
--            e.lane = {math.floor(e.w/default.laneWidth+0.5),0}
            e.lane = {2,0}
--                lo('??_____________________ is ang:'..i..':'..ang..' w:'..e.w)
            e.da = (jdesc.round.w/2 + edist)*1
            e.db = (r+jdesc.round.w/2)*ang+b.w/2 + edist
            e.stepa = estep*1.5
            e.stepb = estep*1
            e.mat = 'road1_concrete'
--                U.dump(e, '?? for_e:')
            forExit(b, jdesc.round, e, true)
            if not toupdate then
                U.pop(adec[e.frto[2]].aexo, e.ind)
                jdesc.round.aexi[#jdesc.round.aexi+1] = e.ind
            end
--                lo('?? for_EX_right:'..i..':'..tostring(e.id))
            if e.id then
                nodesUpdate(e)
                e.body:setField('material', 0, e.mat)
            else
                decalUp(e)
                b.aexo[_] = e.ind
--                    lo('?? eNEW:'..i..':'..tostring(e.ind))
            end

--            local p = onSide(jdesc.round, 'right', (r+jdesc.round.w/2)*ang + b.w/2 + edist)
--            if p then
--                     out.avedit = {p}
--            end
-- create LEFT
            if true then
                local eleft = toupdate and adec[b.aexi[1]] or {}
                eleft.stepa = -e.stepb
                eleft.stepb = e.stepa
                eleft.da = (r+jdesc.round.w/2)*(ang == 0 and 2*math.pi or ang) - b.w/2 - edist
--                        lo('??^^^ if_upd:'..i..':'..tostring(toupdate)..' ang:'..ang..':'..edist..' da:'..eleft.da)
                if eleft.da < 0 then
                    eleft.da = 2*math.pi*(r+jdesc.round.w/2) + eleft.da
--                    eleft.da = math.pi*3/2*(r+jdesc.round.w/2)
--                        lo()
--                    local pons = onSide(jdesc.round, 'right', 2*math.pi + eleft.da - 8, 0) -- math.pi*4/2*(r+jdesc.round.w/2), 0)
--                    lo('?? pons:'..tostring(pons))
--                    if pons then
--                        out.avedit = {pons}
--                    end
                end
                eleft.db = e.da
                eleft.sidea = 'right'
                eleft.cmargin = e.cmargin
                eleft.mat = e.mat
                eleft.w = e.w
                eleft.lane = e.lane
--                    if i == 2 then eleft.w = 1 end
                    lo('??________________ pre_EXIT_left:'..i..':'..tostring(toupdate)..':'..tostring(i==2)..' w:'..eleft.w..':'..eleft.stepa)
                forExit(jdesc.round, b, eleft, true, i == 3)
--                forExit(jdesc.round, adec[e.frto[1]], eleft, true, i == 3)
--                        U.dump(eleft.list, '?? for_LEFT:'..i)
--                        toMark(b.list, 'mag')
--                        toMark(eleft.list, 'blue')
                if toupdate then
                    nodesUpdate(eleft)
                else
--                        lo('?? new_LEFT:'..i..':'..tostring(eleft.list and #eleft.list or nil))
                    decalUp(eleft)
--                        toMark(eleft.list)
--                        out.avedit = eleft.list
                    jdesc.round.aexo[#jdesc.round.aexo+1] = eleft.ind
                    --TODO: append
                    b.aexi = {eleft.ind}
                end
            end
--            eleft.frto = {jdesc.round.ind,b.ind}
--[[
                    local p = onSide(jdesc.round, 'right', (r+jdesc.round.w/2)*ang - b.w/2 - edist, 0)
                        lo('?? for_p:'..tostring(p))
                    out.avedit = {p}
            if false then
                forExit(adec[e.frto[1] ], adec[e.frto[2] ], e)
    --                    forExit(jdesc.list[e.frto[1] ], jdesc.list[e.frto[2] ], e)
                nodesUpdate(e)
                -- lines right
                for k,l in pairs(b.aline) do
                    l.on = true
                end
--                    lo('?? for_lines_o:'.._)
                local bout,bin = b,U.mod(i+1,jdesc.list)
                nodesUpdate(sideLine(e, 'left', bout, 'left', bin, 'right', b.aline[1]))
                nodesUpdate(sideLine(e, 'right', bout, 'left', bin, 'right', b.aline[2]))
                nodesUpdate(sideLine(bout, 'left', e, 'left', bin, 'right', b.aline[3]))
                nodesUpdate(sideLine(bin, 'right', bout, 'left', e, 'left', b.aline[4]))
            end
]]
--                    out.avedit = {adec[jdesc.round.ind].list[1]}


--                    sideLine(e, 'right', bout, 'left', bin, 'right', {w=default.wline*1.5, mat='m_line_white', lane={1, 0}})
--                    sideLine(bout, 'left', e, 'left', bin, 'right', {w=default.wline*1.5, mat='m_line_white', lane={1, 0}})
--                    sideLine(bin, 'right', bout, 'left', e, 'left', {w=default.wline*1.5, mat='m_line_white', lane={1, 0}})
--                        lo('??^^^^^^^^^^ bu1:')
        end
--            if toupdate and i==2 then break end
    end
    -- exit lines
    if true or U._MODE ~= 'conf' then

        local c
        local aline,line,ex,rdfr,rdto,newline
--        for i=3,3 do
--            local br = jdesc.list[i]
        for i,br in pairs(jdesc.list) do
            if true then
                --- LEFT
                ex = adec[br.aexi[1]]
                rdfr = jdesc.round
                rdto = br
                if c then
                    rdfr = adec[U.mod(i-1,jdesc.list).aexo[1]]
    --                rdto = adec[U.mod(i+1,jdesc.list).aexi[1]]
    --                out.avedit = {c}
                end
                if i>1 then
                    newline = sideLine(ex, 'right', rdfr, 'right', rdto,
                        rdto.io == 1 and 'right' or 'left',
                        {w=default.wline*1.5, mat=default.matline, lane={1,0}})
--                        lo('??***************** if_LINE:'..tostring(newline.list))
--                        toMark(newline.list, 'blue')
--                    newline = sideLine(ex, 'right', rdfr, 'right', rdto, 'right', {w=default.wline*1.5, mat=default.matline, lane={1,0}})
    --                    lo('??_______________ if_line:'..i..':'..tostring(newline)..':'..tostring(ex)..':'..#br.aexi..':'..tostring(ex.aline and #ex.aline or 0))
                end
    --                out.avedit = ex.list
    --            line = ex.aline[1]
    --            line.list = newline.list
                if newline then
                    if toupdate and ex.aline then
                        ex.aline[1].list = newline.list
                        nodesUpdate(ex.aline[1])
                    else
                        decalUp(newline)
                        if not ex.aline then ex.aline = {} end
                        ex.aline[#ex.aline+1] = adec[#adec]
    --                    br.aexi[1] = adec[#adec]
                    end
                end
    --            nodesUpdate(line)
    --            line.body:setField('hidden', 0, 'false')

                --- RIGHT
                ex = adec[br.aexo[1]]
                if ex then
                    c = borderCross(ex.body, adec[U.mod(i+1,jdesc.list).aexi[1]].body, 'right', 'right', 0, 0, default.sidemargin)
        --                out.avedit = {c}
            --            lo('?? diff:'..c:distance(aline[1].list[#aline[1].list]))
            --        aline[1].list[#aline[1].list] = c
                    rdto = jdesc.round
                    if c then
                        rdto = adec[U.mod(i+1,jdesc.list).aexi[1]]
                    end
                    newline = sideLine(ex, 'right',
                        br, br.io==1 and 'left' or 'right',
                        rdto, 'right', {w=default.wline*1.5, mat=default.matline, lane={1,0}})
                    if newline then
                        if ex.aline and #ex.aline>1 then
                            line = ex.aline[2]
    --                    if line and newline then
                            line.list = newline.list
                            nodesUpdate(line)
                            line.body:setField('hidden', 0, 'false')
                        else
                            ex.aline = {}
                            ex.aline[2] = newline
                            decalUp(newline)
    --                        lo('!! ERR_NO_LINE:'..i)
                        end
                    end
                else
                    lo('!! ERR_no_EX_RIGHT:'..i..':'..tostring(br.aexo[1]))
                        toMark(br.list,'blue')
                end

    --                    lo('??_____ if_LINE:'..tostring(line)..':'..tostring(aline[1]))

    --            adec[br.aexo[1]].aline[2].list = aline[1].list
        --        aline = line4exit(adec[b.aexo[1]], b, jdesc.round)
        --        line = adec[b.aexo[1]].aline[2]
        --        line.list = aline[1].list

        --        line.list[#line.list] = c -- line.list[#line.list] + vec3(5,5)
        --            out.avedit = {line.list[#line.list]}

        --        nodesUpdate(line)
                --- left
    --                lo('?? exi:'..#b.aexi)
    --            aline = line4exit(adec[b.aexi[1]], jdesc.round, b)
    --                lo('?? al:'..tostring(aline))
        --            U.dump(aline, '?? line2:'..#adec[b.aexo[1]].aline)
        --            out.avedit = aline[1].list
            end
        end
        -- last left line
        local i,br =1,jdesc.list[1]
        ex = adec[br.aexi[1]]
        rdfr = jdesc.round
        rdto = br
        if c then
            rdfr = adec[U.mod(i-1,jdesc.list).aexo[1]]
    --        out.avedit = {c}
        end
        newline = sideLine(ex, 'right', rdfr, 'right',
            rdto, rdto.io==1 and 'right' or 'left', {w=default.wline*1.5, mat=default.matline, lane={1,0}})
        if newline then
            if toupdate then
                ex.aline[1].list = newline.list
                nodesUpdate(ex.aline[1])
            else
                decalUp(newline)
                if not ex.aline then ex.aline = {} end
                ex.aline[#ex.aline+1] = adec[#adec]
            end
        end

        jdesc.rround = r
    end
    for _,rd in pairs(jdesc.list) do
        rd.body:setPosition(rd.body:getPosition())
    end
        lo('<< junctionRound:'..#jdesc.list[1].list)
end
D.junctionRound = junctionRound


local function vert2junc(abr)
--        U.dump(abr, '>> vert2junc:')
        out.acyan = {abr[1].list[1]}
--        default.wexit = 2.4
    -- exits
--        ajunc[cjunc].r = 30 -- ajunc[cjunc].r + 10
    local junc = ajunc[cjunc]
        lo('>> vert2junc:'..tostring(cjunc)..':'..tostring(junc and junc.r or nil)) --..ajunc[cjunc].r) --..':'..tostring(abr[1].list[1])) --..':'..tostring(abr[1].ij))
--        junc.r = 10
    local rexit = (junc and junc.r-2) or default.rexit
--        rexit = 20
    local aex = {}
--    for i=1,1 do
    for i=1,#abr do
        local bout = abr[i]
            lo('?? if_EXITS:'..tostring(bout.aexo and #bout.aexo or nil))
        if bout.aexo and #bout.aexo > 0 then
--            goto skip
        end
--            U.dump(bout.djrad, '?? for_rad:'..i)
        local bin = U.mod(i+1,abr)
        local ex = {w=default.wexit*1,
--            mat='mat_white',
            mat=default.mat,
--            mat='checkered_line_alt',
            lane={1, 0}, j=#ajunc+1, da=rexit, db=rexit, frto={bout.ind, bin.ind}}
--        local ex = {w=default.laneWidth*1, mat='checkered_line_alt', lane={1, 0}, da=default.rexit, db=default.rexit, frto={i, U.mod(i+1,n)}}
--        aex[#aex+1] = ex
        ex = forExitB(abr[i], bin, ex)
        local ang = U.vang(abr[i].dir,bin.dir,true)
        if ang < math.pi*3/4 and ang > math.pi/3 then
--                lo('?? if_aexo:'..#abr[i].aexo)
            decalUp(ex)
--            if #abr[i].aexo == 1 then
--                lo('??___________________ exit_NEXT:')
--                toMark(ex.list, 'blue')
--            end
--            ex.body:setField('improvedSpline', 0, 'false')

--            editor.updateRoadVertices(ex.body)
--            ex.body:setPosition(ex.body:getPosition())
            -- lines
            ex.aline = line4exit(ex, bout, bin)
            for k,ldesc in pairs(ex.aline) do
                decalUp(ldesc)
--                    lo('??*************** if_LINE_BODY:'..i..':'..k..':'..tostring(ldesc.body)..':'..tostring(ldesc.on))
                if ldesc.body and ldesc.on==false then
                    ldesc.body:setField('hidden', 0, 'true')
                end
            end
        else
            lo('?? exit_SKIP:'..abr[i].ind)
        end
--            lo('??_____________ for_ang:'..i..':'..U.vang(abr[i].dir,bin.dir,true)..':'..tostring(ex.ind))
        abr[i].aexo[#abr[i].aexo+1] = ex.ind or 0
        bin.aexi[#bin.aexi+1] = ex.ind or 0

--                lo('?? for_BR:'..i..':'..#bout.list)
--                if i == 4 then
--                    toMark(bout.list, 'blue')
--                end
        ::skip::
    end
--        if true then return end
-- branch LINES
        local m1 = {}
        out.acyan = {}
    if junc then -- and not abr[1].ij then
--        for i=3,3 do
--            local b = abr[i]
        for i,b in pairs(abr) do
--                U.dump(b.djrad, '??++++++++++++++ for_rad:'..i)
--                    lo('?? for_mid:'..tostring(b.ne)..':'..b.io..':'..b.w)
--- middle
            b.aline[#b.aline+1] = line4laneD(b, b.lane[1], U.mod(i+1, abr).w/2, default.matline)
            decalUp(b.aline[#b.aline])

--- right
            for k=1,b.lane[1]-1 do
--                    lo('??********** for_RIGHT:'..k)
                b.aline[#b.aline+1] = line4laneD(b, k, U.mod(i+1, abr).w/2, default.matlinedash)
                decalUp(b.aline[#b.aline])
            end
--- lanes left
            for k=1,b.lane[2]-1 do
                b.aline[#b.aline+1] = line4laneD(b, k+b.lane[1], U.mod(i+1, abr).w/2, default.matlinedash)
                decalUp(b.aline[#b.aline])
            end
--            pin2side(b, 'middle', dw, ce, ias, iae, sidemargin, pinstep)
        end
--            toMark(m1, 'blue', nil, 0.1)
    else
        for i,b in pairs(abr) do
            local dw = U.mod(i+1, abr).w/2
            local u = (epos(b.body,0,'left') - epos(b.body,0,'right')):normalized()
            -- middle line
            b.aline[#b.aline+1] = line4lane(b, b.lane[1], U.mod(i+1, abr).w/2)
            decalUp(b.aline[#b.aline])
--                        out.acyan = b.aline[#b.aline].list
    --            lo('?? if_body:'..tostring(b.aline[#b.aline].body))
            -- lanes right
            for k=1,b.lane[1]-1 do
                b.aline[#b.aline+1] = line4lane(b, k, U.mod(i+1, abr).w/2, default.matlinedash)
                decalUp(b.aline[#b.aline])
            end
            -- lanes left
            for k=1,b.lane[2]-1 do
                b.aline[#b.aline+1] = line4lane(b, k+b.lane[1], U.mod(i+1, abr).w/2, default.matlinedash)
                decalUp(b.aline[#b.aline])
            end
        end
    end
end
--[[
            -- distance from center
            local dw = U.mod(i+1, abr).w/2
            local margin = b.w/(b.lane[1]+b.lane[2])*(b.io==1 and b.lane[1] or b.lane[2])
            local pb,ib = onSide(b, b.io==1 and 'right' or 'left', dw, margin)
--                    out.acyan[#out.acyan+1] = pb+vec3(0,0,forZ(pb))
            local pe,ie = onSide(b, b.io==1 and 'right' or 'left', b.L - dw, margin)
--                    out.acyan[#out.acyan+1] = pe+vec3(0,0,forZ(pe))
--                    lo('?? ps_pe:'..tostring(ps)..':'..tostring(pe)..':'..ib..':'..ie)
--                    m1[#m1+1] = epos(b.body,ib)
--                    m1[#m1+1] = epos(b.body,ie-1)
            local line = {w=default.wline*1.5, mat=default.matline, lane={1,0}}
--            line.list = pin2side(b.body, b.io==1 and 'right' or 'left', pb, pe, ib, ie-1, margin, 1)
            line.list = pin2sideD(b, b.io==1 and 'right' or 'left', dw, b.L-dw, margin, 6)
--                U.dump(line.list, '?? ll:'..i)

            for j=1,#line.list do
                line.list[j] = line.list[j] + vec3(0,0,forZ(line.list[j]))
            end
--                out.acyan = list
--                toMark(list, 'red', nil, 0.1)
--                U.dump(line,'?? for_line:'..i)
            b.aline[#b.aline+1] = line
            local obj = decalUp(b.aline[#b.aline])
            obj:setPosition(obj:getPosition())
]]


local function junctionUp(pos, n, inrand, aang)
--        n = 8
--        inrand = true
    if inrand == nil then
        inrand = true
    end
    if not n then n = 4 end
        lo('>> junctionUp:'..tostring(pos)..':'..n)
    local step = default.rjunc --*2/3
    local abr = {}
    for i=1,n do
        local list = {pos}
        local ang = aang and aang[i] or (2*math.pi/n + (inrand and 1 or 0)*0.4*U.rand(-math.pi/12, math.pi/12))
--                if i==4 then
--                    ang = ang + math.pi/80
--                end
--            lo('?? for_ang:'..i..':'..ang)
        for k = 1,2 do
            list[#list+1] = list[#list] + step*vec3(math.cos(i*ang), math.sin(i*ang))
        end
        local lane = {2,2} -- inrand and {math.random(1,3),math.random(1,3)} or {2,2}
        if true or inrand then
            local dl = math.random(0,1)
--                dl = 1
--            dl = 0
            lane[1] = lane[1] + dl
            lane[2] = lane[2] - dl
        end
--            if i==1 then lane = {2,2} end
--            if i==1 then lane = {1,1} end
        abr[#abr+1] = {
--            ind = i,
            ij = {#ajunc+1, i}, -- indexes in junction
            list = list,
            w = default.laneWidth*(lane[1]+lane[2]),
            mat = default.mat,
--            lane = {1,default.laneNum}, aexi={}, aexo={}, aline={}, io=1,
--            lane = {default.laneNum,default.laneNum}, aexi={}, aexo={}, aline={}, io=1,
            lane = lane, aexi={}, aexo={}, aline={}, io=1,
            dir = (list[2]-list[1]):normalized()}
--        abr[#abr].listrad = deepcopy(abr[#abr].list)
        decalUp(abr[#abr])
        amatch[#amatch+1] = {ind=abr[#abr].ind}
        abr[#abr].L = roadLength(abr[#abr])
    end
    -- exits
    vert2junc(abr)

    ajunc[#ajunc+1] = {list=abr, p=pos}
    cjunc = #ajunc

    return ajunc[#ajunc]
end
--[[
    if false then

        local aex = {}
        for i=1,n do
            local bout = abr[i]
            local bin = U.mod(i+1,abr)
            local ex = {w=default.wexit*1,
    --            mat='mat_white',
                mat=default.mat,
    --            mat='checkered_line_alt',
                lane={1, 0}, da=default.rexit, j=#ajunc+1, db=default.rexit, frto={bout.ind, bin.ind}}
    --        local ex = {w=default.laneWidth*1, mat='checkered_line_alt', lane={1, 0}, da=default.rexit, db=default.rexit, frto={i, U.mod(i+1,n)}}
    --        aex[#aex+1] = ex
            if true then
                ex = forExit(abr[i], bin, ex)
    --                lo('?? if_aexo:'..#abr[i].aexo)
    --            abr[i].aexo[#abr[i].aexo+1] = ex
    --            bin.aexi[#bin.aexi+1] = ex
                decalUp(ex)
                abr[i].aexo[#abr[i].aexo+1] = ex.ind
                bin.aexi[#bin.aexi+1] = ex.ind
                -- lines
    --            aline[#aline+1] = {
    --                list = {bout.list[1]+bout.dir*5, bout.list[#bout.list]},
    --                w=default.wline*1.5, mat=default.matline, lane={1, 0}}
    --                out.avedit = pin2side(bout.body, 'right', bout.list[1], bout.list[#bout.list], 1, bout.body:getEdgeCount()-1, 1, 2)
                ex.aline = line4exit(ex, bout, bin)
                for k,ldesc in pairs(ex.aline) do
                    decalUp(ldesc)
    --                bout.aline[k] = ldesc
    --                bin.aline[4+k] = ldesc
                end
            end
        end
        for i,b in pairs(abr) do
            local dw = U.mod(i+1, abr).w/2
            local u = (epos(b.body,0,'left') - epos(b.body,0,'right')):normalized()
    --        = {
    --            list = {bout.list[1]+bout.dir*5, bout.list[#bout.list]},
    --            w=default.wline*1.5, mat=default.matline, lane={1, 0}}

            -- middle line
            b.aline[#b.aline+1] = line4lane(b, b.lane[1], U.mod(i+1, abr).w/2)
            decalUp(b.aline[#b.aline])
    --            lo('?? if_body:'..tostring(b.aline[#b.aline].body))
            -- lanes right
            for k=1,b.lane[1]-1 do
                b.aline[#b.aline+1] = line4lane(b, k, U.mod(i+1, abr).w/2, default.matlinedash)
                decalUp(b.aline[#b.aline])
            end
            -- lanes left
            for k=1,b.lane[2]-1 do
                b.aline[#b.aline+1] = line4lane(b, k+b.lane[1], U.mod(i+1, abr).w/2, default.matlinedash)
                decalUp(b.aline[#b.aline])
            end
        end

    end
            local aline = {}
            aline[#aline+1] = sideLine(ex, 'left', bout, 'left', bin, 'right', {w=default.wline*1.5, mat=default.matline, lane={1, 0}})
            aline[#aline+1] = sideLine(ex, 'right', bout, 'left', bin, 'right', {w=default.wline*1.5, mat=default.matline, lane={1, 0}})
            aline[#aline+1] = sideLine(bout, 'left', ex, 'left', bin, 'right', {w=default.wline*1.5, mat=default.matline, lane={1, 0}})
            aline[#aline+1] = sideLine(bin, 'right', bout, 'left', ex, 'left', {w=default.wline*1.5, mat=default.matline, lane={1, 0}})
]]
--        bout.aline[#bout.aline+1] = {list=bout.list,w=default.wline*1.5, mat=default.matline, lane={1, 0}}
--        decalUp(bout.aline[#bout.aline])
--[[
        bout.aline[#bout.aline+1] = sideLine(ex, 'left', bout, 'left', bin, 'right')
        bin.aline[#bin.aline+1] = bout.aline[#bout.aline]
        bout.aline[#bout.aline+1] = sideLine(ex, 'right', bout, 'left', bin, 'right')
        bin.aline[#bin.aline+1] = bout.aline[#bout.aline]
        bout.aline[#bout.aline+1] = sideLine(bout, 'left', ex, 'left', bin, 'right')
        bin.aline[#bin.aline+1] = bout.aline[#bout.aline]
        bout.aline[#bout.aline+1] = sideLine(bin, 'right', bout, 'left', ex, 'left')
        bin.aline[#bin.aline+1] = bout.aline[#bout.aline]
]]
--        local ldesc = sideLine(ex, 'left', bout, 'left', bin, 'right')
--        decalUp(ldesc)
--        decalUp()
--[[
        junc.aline[#junc.aline+1] = {body=sideLine(rd, 'left', rda, 'left', rdb, 'right')}
        junc.aline[#junc.aline+1] = {body=sideLine(rd, 'right', rda, 'left', rdb, 'right')}

        junc.aline[#junc.aline+1] = {body=sideLine(rda, 'left', rd, 'left', rdb, 'right')}
        junc.aline[#junc.aline+1] = {body=sideLine(rdb, 'right', rda, 'left', rd, 'left')}
]]
--[[
            margin = b.w/2 - b.w/(b.lane[1]+b.lane[2])*k
                lo('?? for_dash:'..k)
            ldesc = {
                list = {b.list[1] + b.dir*dw - margin*u, b.list[#b.list] - margin*u},
                w=default.wline*1.5, mat=default.matlinedash, lane={1, 0}}
            decalUp(ldesc)
            b.aline[#b.aline+1] = ldesc
]]
--            b.aline[#b.aline+1] = {
--                list = {b.list[1] + b.dir*dw + margin*u, b.list[#b.list] + margin*u},
--                w=default.wline*1.5, mat=default.matlinedash, lane={1, 0}}
--            b.aline[#b.aline+1] = {
--                list = {b.list[1]+b.dir*dw, b.list[#b.list]},
--                w=default.wline*1.5, mat=default.matlinedash, lane={1, 0}}
--[[
        local margin = b.w/2 - b.w/(b.lane[1]+b.lane[2])*b.lane[1]
        ldesc = {
            list = {b.list[1] + b.dir*dw - margin*u, b.list[#b.list] - margin*u},
            w=default.wline*1.5, mat=default.matline, lane={1, 0}}
        b.aline[#b.aline+1] = ldesc
]]


local function forPlot(render)
        lo('>> forPlot:'..tableSize(amatch)..':'..tostring(render))
--        out.aplot = {}
--            amatch[2] = nil
--            U.dump(amatch)
    local function ifCross(k,m)
        local dbg
--        if k == 6 and m == 1 then
--            lo('?? ifCross:'..k..':'..m)
--            dbg = true
--        end
        local pfr = adec[amatch[k].ind]
        pfr = pfr.list[#pfr.list]
        local pto = adec[amatch[m].ind]
        pto = pto.list[#pto.list]
        for j,d in pairs(amatch) do
            if j~=k and j~=m and d.fr~=k and d.fr~=m then
                if d.fr then
                    local pfr2 = adec[amatch[d.fr].ind]
                    pfr2 = pfr2.list[#pfr2.list]
                    local pto2 = adec[amatch[j].ind]
                    pto2 = pto2.list[#pto2.list]
                    if U.segCross(pfr,pto,pfr2,pto2) then
                        lo('?? crossed_L:'..amatch[d.fr].ind..':'..amatch[j].ind)
                        return true
                    end
--                    if U.line2seg(pfr,pto,pfr2,pto2) and U.line2seg(pfr2,pto2,pfr,pto) then
--                            U.dump(amatch)
--                        return true
--                    end
    --                out.avedit = {pfr, pto}
                end
            end
        end
--            U.dump(pfr)
        local jifr,jito = adec[amatch[k].ind].ij,adec[amatch[m].ind].ij
        for _,j in pairs(ajunc) do
            for ib,b in pairs(j.list) do
                if (_ ~= jifr[1] or ib ~= jifr[2]) and (_ ~= jito[1] or ib ~= jito[2]) then
                    if U.segCross(pfr,pto,j.p,b.list[#b.list]) then
                            lo('?? crossed_B:'.._..':'..k)
                        return true
                    end
                end
            end
        end
        return false
    end
    local aplot = {}
    local match = true
        local n = 0
    local atry = {}
    while match and n < 20 do
--        local atry = {}
        match = false
        for k,_ in pairs(amatch) do
            if not indrag then
                lo('??_ for_k:'..k..':'..tostring(amatch[k].to)..':'..tostring(amatch[k].fr))
            end
            if amatch[k].to or amatch[k].fr or amatch[k].skip then
                goto continue
            end
            local src = adec[amatch[k].ind].list[#adec[amatch[k].ind].list]
            -- find minimal target available
            local last
            local dmi,imi = math.huge
            for m,_ in pairs(amatch) do
                local a,b = adec[amatch[k].ind],adec[amatch[m].ind]
                if m ~= k and a.ij[1] ~= b.ij[1] and not amatch[m].to and not amatch[m].match and not amatch[m].skip then
                    local tgt = b.list[#b.list]
                    local dist = src:distance(tgt)
                    if not indrag then
                        lo('??__ for_m:'..m..':'..dist..'/'..dmi..' t_m:'..tostring(atry[m])..' t_k:'..tostring(atry[k]))
                    end
                    if dist<dmi then -- and not ifCross(k,m) then
--                        local stamp = U.stamp({k,m})
                        -- check if target available
                        local tomatch = true
                        if atry[m] then
                            if dist > atry[m].d then
                                tomatch = false
                            else --+
                                if atry[k] and dist > atry[k].d then
                                    tomatch = false
                                end
                            end
                        elseif atry[k] then
                            if dist > atry[k].d then
                                tomatch = false
                            else --+
                                if atry[m] and dist > atry[m].d then
                                    tomatch = false
                                end
                            end
                        end
                            lo('?? to_match:'..k..':'..m..':'..tostring(tomatch)..':'..tostring(atry[k])..':'..tostring(atry[m]))
                        if tomatch then
                            if atry[m] then
                                -- drop
                                local ifr = atry[m].ifr
                                local ito = m
                                amatch[ifr].to = nil
                                amatch[ito].fr = nil
                            end
                            if atry[k] then
                                -- drop
                                local ifr = atry[k].ifr
                                local ito = k
                                amatch[ifr].to = nil
                                amatch[ito].fr = nil
--                                amatch[atry[k].ifr].to = nil
--                                atry[k] = nil
                            end
                                lo('?? pre_cross_imi:'..tostring(imi))
                            if imi then
                                amatch[imi].fr = nil
                            end
                            if not ifCross(k,m) then
                                if imi then atry[imi] = nil end
                                dmi = dist
                                imi = m
                                atry[m] = {ifr=k, d=dmi}
                                amatch[k].to = m
                                amatch[m].fr = k
                                    lo('??+++ match:'..m..'<'..k)
                                match = true
                            else
                                lo('?? CROSSED:'..k..':'..m)
                            end
--                                U.dump(atry)
                        end
                        for s,l in pairs(amatch) do
                            if l.to then
                                --- check if no intersections
                            end
                        end
                    end
                end
            end
            if not imi and not amatch[k].fr then
                -- no match remove from
                amatch[k].skip = true
                    U.dump(atry,'?? no_MATCH:'..k) --..':'..tostring(out.aplot))
--                    U.dump(amatch)
--                    return
            end
            ::continue::
        end
            n = n + 1
--            U.dump(atry, '?? atry:'..n..':'..tostring(out.aplot)..':'..tostring(match))
--            U.dump(amatch)
    end
    for k,d in pairs(amatch) do
        if not d.skip and d.to and not d.match then
            local rda,rdb = adec[d.ind],adec[amatch[d.to].ind]
--                U.dump(rdb.list, '?? rda:')
            aplot[#aplot+1] = {
                fr = k, --rda.ind,
                to = d.to, --rdb.ind,
                e={rda.list[#rda.list],rdb.list[#rdb.list]}}
        end
    end
    if false then
        for k,t in pairs(atry) do
                lo('?? to_plot:'..t.ifr..'>'..k)
            aplot[#aplot+1] = {
                fr = t.ifr,
                to = k, -- ajunc[imi[1] ].list[imi[2] ],
                e = {
                    adec[amatch[t.ifr].ind].list[#adec[amatch[t.ifr].ind].list],
                    adec[amatch[k].ind].list[#adec[amatch[k].ind].list]}}
        end
    end
    if not indrag then
        U.dump(atry, '?? atry:'..n..':'..tostring(out.aplot)..':'..tostring(match))
        U.dump(amatch)
    end
        lo('<< forPlot:'..#aplot)
    for i,d in pairs(amatch) do
--        d.skip = nil
    end
    if render then
        for i,d in pairs(aplot) do
            D.branchMerge(adec[amatch[d.fr].ind], adec[amatch[d.to].ind])
            amatch[d.fr].skip = true
            amatch[d.to].skip = true
            amatch[d.fr].match = true
            amatch[d.to].match = true
        end
        return
    end
    return aplot
end
D.forPlot = forPlot
--[[
                        if not atry[stamp] or dist < atry[stamp].d then
                            local ifcan = true
                            if atry[k] then
                                if dist<atry[k].d then
                                    if not ifCross() then
                                            lo('??*** cancel:'..k..'>'..atry[k].ifr)
                                        amatch[atry[k].ifr].skip = nil
                                        atry[k] = nil
                                    else
                                        ifcan = false
                                    end
                                else
                                    ifcan = false
                                end
                            end
                            if ifcan then
                                dmi = dist
                                imi = m
                                if atry[stamp] then
                                    amatch[atry[stamp].ifr].skip = nil
                                        lo('??*** replace:'..stamp..':'..k..'/'..atry[stamp].ifr)
                                end
                                atry[stamp] = {ifr=k, d=dmi}
                                amatch[k].skip = true
                                if last then atry[last] = nil end
                                last = stamp
                                    lo('?? match:'..k..'>'..m)
                            end
                        end
                        if atry[m] then
                            if dist < atry[m].d then
                                dmi = dist
                                atry[m] = {ifr=k, d=dmi}
                                if last then atry[last] = nil end
                                last = m
                            end
                        else
                            dmi = dist
                            atry[m] = {ifr=k, d=dist}
                            if last then atry[last] = nil end
                            last = m
                        end
]]


local function forPlot_(ind)
    if not ind then return end
    local d = ajunc[ind]
    if not d then return end
    local aplot = {}
    local atry = {}
        lo('>> forPlot: junc:'..ind)
    local tomatch = {}
    for i=1,#d.list do
        tomatch[i] = true
    end
--        tomatch = {[1] = true, [4]=true}
    local function ifCross(c,d)
        for k,v in pairs(atry) do

        end
    end
        local n = 0
    while tableSize(tomatch) > 0 and n<15 do
        for i,_ in pairs(tomatch) do
            local bfr = d.list[i]
            local dmi,imi = math.huge
            local lasttry
            -- fit branch
            for k,dto in pairs(ajunc) do
                if k ~= ind then
                    for j,bto in pairs(dto.list) do
                        local dist = (bto.list[#bto.list]-bfr.list[#bfr.list]):length()
                            lo('?? for_tgt:'..i..'>'..j..':'..dist..'/'..dmi)
                            out.ared = {bfr.list[#bfr.list]}
                            out.agreen = {bto.list[#bto.list]}
                        if dist < dmi then
                            local sto = U.stamp({k,j},true)
                                lo('?? less:'..i..'>'..j..':'..sto..':'..tostring(atry[sto]))
                            if atry[sto] then
                                -- check if closer
                                if dist < atry[sto].d then
                                    dmi = dist
                                    imi = {k,j}
                                    tomatch[atry[sto].ifr] = true
                                    atry[sto] = {d=dist, ifr=i, imi=imi}
                                    if lasttry then atry[lasttry] = nil end
                                    lasttry = sto
                                end
                            else
                                    lo('?? to_new:'..j..':'..dmi..'>'..sto)
                                dmi = dist
                                imi = {k,j}
                                atry[sto] = {d=dmi, ifr=i, imi=imi}
                                if lasttry then atry[lasttry] = nil end
                                lasttry = sto
--                                tomatch[i] = nil
                            end
                        end
--                                break
                    end
                end
            end
            for k,v in pairs(atry) do
                tomatch[v.ifr] = nil
            end
--                U.dump(atry, '?? to_try:'..n)
--                U.dump(tomatch, '?? tM:')
            break
--            if i==2 then break end
        end
            n = n+1
--        break
    end
        U.dump(atry, '?? to_try:'..n)

    for k,t in pairs(atry) do
        local bfr = d.list[t.ifr]
        local imi = t.imi
        if imi then
            aplot[#aplot+1] = {
                fr = bfr,
                to = ajunc[imi[1]].list[imi[2]],
                e = {bfr.list[#bfr.list], ajunc[imi[1]].list[imi[2]].list[#ajunc[imi[1]].list[imi[2]].list]}}
        end
    end

    if false then

        for i,bfr in pairs(d.list) do
    --        atry[i] = {}
            local dmi,imi = math.huge
            for k,dto in pairs(ajunc) do
                if k ~= ind then
                    for j,bto in pairs(dto.list) do
                        local dist = (bto.list[#bto.list]-bfr.list[#bfr.list]):length()
                        if dist < dmi then
                            local stamp = U.stamp({k,j},true)
                            if not atry[stamp] then -- or dmi<atry[stamp].d then
    --                            atry[stamp] = {d=dmi, ifr=i, imi=imi}
    --                            dmi = dist
    --                            imi = {k,j}
                                dmi = dist
                                imi = {k,j}
                            end
    --                        dmi = dist
    --                        imi = {k,j}
    --                            lo('?? for_imi:'..i..':'..k..':'..j)
                        end
                    end
                end
            end
            if imi then
                local stamp = U.stamp(imi,true)
                if not atry[stamp] or dmi<atry[stamp].d then
                    atry[stamp] = {d=dmi, ifr=i, imi=imi}
                end
    --                U.dump(aplot[#aplot], '?? for_br:'..i..':'..imi[1]..':'..imi[2])
            else
                lo('?? none_for_branch:'..i)
            end
        end
            U.dump(atry, '?? forPlot_ATRY:')
        for k,t in pairs(atry) do
            local bfr = d.list[t.ifr]
            local imi = t.imi
            if imi then
                aplot[#aplot+1] = {
                    fr = bfr,
                    to = ajunc[imi[1]].list[imi[2]],
                    e = {bfr.list[#bfr.list], ajunc[imi[1]].list[imi[2]].list[#ajunc[imi[1]].list[imi[2]].list]}}
            end
        end

    end
        lo('<< forPlot:'..#aplot)
    return aplot
end


local function branchMerge(a, b, desc)
        lo('?? branchMerge:'..a.id..':'..b.id)
    local list = {}
    local nea = a.body:getEdgeCount()
    local neb = b.body:getEdgeCount()
    local lasta = a.body:getMiddleEdgePosition(nea-1)
    local lastb = b.body:getMiddleEdgePosition(neb-1)
    local nstep = 8
    local c2nd = 1+lasta:distance(lastb)/80 --2
--        lo('?? coeff:'..lasta:distance(lastb)/100)
    list[#list+1] = a.body:getMiddleEdgePosition(nea-nstep)
    list[#list+1] = lasta + c2nd*(lasta - list[#list])
--    list[#list+1] = (a.body:getMiddleEdgePosition(nea-1)+b.body:getMiddleEdgePosition(neb-1))/2
    list[#list+1] = lastb + c2nd*(lastb - b.body:getMiddleEdgePosition(neb-nstep))
    list[#list+1] = b.body:getMiddleEdgePosition(neb-nstep)

--        out.avedit = {list[3],list[4]}
--[[
    for j = 1,nstep do
        list[#list+1] = a.body:getMiddleEdgePosition(nea-(nstep-j)*step-1)
    end
    for j = 1,nstep do
        list[#list+1] = b.body:getMiddleEdgePosition(neb-(j-1)*step-1)
    end
]]
    if not desc then
        desc = {
            list=list,
            w = default.laneWidth*default.laneNum*2,
            mat = default.mat,
            lane = {default.laneNum,default.laneNum}}
        decalUp(desc)
    else
        nodesUpdate(desc, list)
    end
    local rdlen = roadLength(desc)
--        lo('?? for_len:'..tostring(rdlen)..':'..tostring(epos(a.body, 0, 'left'))..':'..tostring(epos(a.body, nea-1, 'left'))..':'..tostring(epos(a.body, nea, 'left')))
    local pmid = onSide(desc, 'middle', rdlen/2)
    if a.dir:dot(b.dir) > 0 then
        local d,p = U.toLine(pmid, {list[2],list[3]})
        pmid = pmid + (pmid-p):normalized()*list[2]:distance(list[3])/4
    end
--    pmid = pmid + U.vturn((list[3]-list[2]))
    table.insert(list, 3, pmid)
    nodesUpdate(desc, list, nil)


    -- 1st iteration
    local aend = epos(a.body, nea-1, 'middle')
    local va = U.vturn(aend-list[1], math.pi/2)
    local pa = epos(a.body, nea-1, 'left')
    local pca = line2side(pa, pa+va, desc, 'left')
--        out.avedit = {p,p+v} -- {pc} -- {epos(a.body, nea-1, 'middle'), list[#list]}
--        lo('?? to_side:'..tostring(pc))
    local epsa = list[2]:distance(list[1])/aend:distance(list[1])
    local spa = list[2]
    list[2] = spa + epsa*(pa-pca)

    local bend = epos(b.body, neb-1, 'middle')
    local vb = U.vturn(bend-list[#list], math.pi/2)
    local pb = epos(b.body, neb-1, 'left')
    local pcb = line2side(pb, pb+vb, desc, 'right')
--        out.avedit = {p,pc}
    local epsb = list[#list-1]:distance(list[#list])/bend:distance(list[#list])
    local spb = list[#list-1]
    list[#list-1] = spb + (pb-pcb)*epsb
--        lo('?? ends2:'..tostring(list[2])..':'..tostring(list[#list-1]))
--        out.avedit = {list[2], pb, pb+vb} -- list[#list-1]}
--        if true then return end

    nodesUpdate(desc, list)

    -- 2nd iteration
    --- start
    local pc2 = line2side(pa, pa+va, desc, 'left')
    local dlt = pca:distance(pc2)
--        lo('?? ee:'..eps..':'..pc:distance(p)..':'..dlt)
    epsa = epsa*pca:distance(pa)/dlt
    list[2] = spa + epsa*(pa-pca)
    --- end
    local pc2 = line2side(pb, pb+vb, desc, 'right')
    local dlt = pcb:distance(pc2)
--        lo('?? ee:'..eps..':'..pc:distance(p)..':'..dlt)
    epsb = epsb*pcb:distance(pb)/dlt
    list[#list-1] = spb + epsb*(pb-pcb)

    -- adjust width
    local aw = nil
--[[
    aw = {}
    for i=1,#desc.list do
        if i==1 or i==#list then
            aw[#aw+1] = 3 --desc.w*0.8
        else
            aw[#aw+1] = desc.w
        end
    end
        U.dump(aw, '?? lastITER:'..#list..':'..#desc.list)
]]
    nodesUpdate(desc, list, nil, aw)

    a.link = desc
    b.link = desc
    desc.fr = a
    desc.to = b
--    nodesUpdate(desc, list)
end
D.branchMerge = branchMerge

local function junctionDown(ind)
    if not ind then return end
    local jdesc = ajunc[ind]
--        U.dump(ajunc, '>> junctionDown:'..tostring(ind)..':'..#ajunc)
    for i,b in pairs(jdesc.list) do
--        if U._MODE == 'conf' then
--            b.listrad = deepcopy(b.list)
--        end
        -- exits
        for k,e in pairs(b.aexo) do
            editor.deleteRoad(adec[e].id)
--??            b.aexo.id = nil
            for j,l in pairs(adec[e].aline) do
                editor.deleteRoad(l.id)
            end
        end
        -- lines
        for k,e in pairs(b.aexo) do
--            editor.deleteRoad(adec[e].id)
        end
        editor.deleteRoad(b.id)
        local m,ind = D.inMatch(b.ind)
        amatch[ind] = nil
        adec[b.ind] = nil
--            lo('?? down:'..i..':'..b.id)
        for k,l in pairs(b.aline) do
            editor.deleteRoad(l.id)
        end
        if b.link and b.link.body then
            editor.deleteRoad(b.link.id)
            b.link.body = nil
            b.link.id = nil
        end
        for k,d in pairs(amatch) do
            if d.ind == b.ind then
                amatch[k] = nil
            end
        end
    end
    if jdesc.round then
        editor.deleteRoad(jdesc.round.id)
    end
    for i,b in pairs(jdesc.list) do
        for k,e in pairs(b.aexi) do
            if adec[e].id then
                editor.deleteRoad(adec[e].id)
            end
        end
    end

    table.remove(ajunc, ind)
    D.matchClear()
--[[
    be:reloadCollision()
    for i,d in pairs(ajunc) do
        for j,b in pairs(d.list) do
            local rd = b.body
            rd:setPosition(rd:getPosition())
            editor.updateRoadVertices(rd)
            editor_roadUtils.reloadDecals(rd)
                lo('?? upd:')
--            editor_roadUtils.reloadDecals(b.body)
--            editor.updateRoadVertices(b.body)
        end
    end
]]
        lo('<< junctionDown:'..#ajunc)
end


local function junctionUpdate(jdesc, show)
        lo('>> junctionUpdate:'..tostring(show))
--    for i=1,1 do
--        local b = jdesc.list[i]
    for i,b in pairs(jdesc.list) do
        if b.dirty then
--            b.dirty = false
            local tohide
            if jdesc.dirty then
                tohide = 'true'
            elseif show then
                tohide = 'false'
            end
            if tohide then
                for ie,e in pairs(b.aexi) do
                    if adec[e] then
                        adec[e].body:setField('hidden', 0, tohide)
                    end
                end
                for ie,e in pairs(b.aexo) do
                    if adec[e] then
                        adec[e].body:setField('hidden', 0, tohide)
                    end
                end
--                    lo('??+++++++++++++++++++++++++++ line_hide:'..#b.aline..':'..tostring(tohide)..':'..tostring(b.aexo[1]))
--                for il,d in pairs(b.aline) do
                if adec[b.aexo[1]] then
                    for il,d in pairs(adec[b.aexo[1]].aline) do
                        if d.body then
                            d.body:setField('hidden', 0, tohide)
                        end
                    end
                end
                if adec[b.aexi[1]] and adec[b.aexi[1]].aline then
--                        U.dump(adec[b.aexi[1]].aline, '?? for_ae_line:'..i)
                    for il,d in pairs(adec[b.aexi[1]].aline) do
                        if d.body then
                            d.body:setField('hidden', 0, tohide)
                        end
                    end
                end
                for il,d in pairs(b.aline) do
                    d.body:setField('hidden', 0, tohide)
                end
                if b.link and b.link.body then
                    b.link.body:setField('hidden', 0, tohide)
                end
--                    U.dump(amatch, '?? amatch:')
            end
            local apos = {}
                lo('?? pre_b_upd:'..i..':'..#editor.getNodes(b.body))
-- branch nodes position
            if U._MODE ~= 'conf' then
                forNodes(b, function(i, n)
                    local list = b.listrad or b.list
                    local r = (list[i]-jdesc.p):length()
    --                local r = (n.pos-jdesc.p):length()
                    if true or r <= default.rjunc+(jdesc.rround or 0)+U.small_val then
                        local p = jdesc.p + b.dir*r
                        p.z = 0
            --                lo('?? for_node:'..i..':'..tostring(n.pos)..'>'..tostring(p)..' dir:'..tostring(bdesc.dir))
    --                    editor.setNodePosition(bdesc.body, i-1, p)
                        apos[#apos+1] = p
    --                    bdesc.desc.list[i] = p
                    end
                end)
                b.listrad = deepcopy(apos)
    --                lo('?? branch_update:'..i..':'..#apos)
                b.list = apos
                if not jdesc.round then
                    nodesUpdate(b, apos, {})
                end
            end
-- exits update
--            local bout,bin = b,U.mod(i+1,jdesc.list)
            if show then
                    lo('??++++++++++++++++++++++++ junctionUpdate_render_branch:'..i..':'..tostring(show))
                for _,idec in pairs(b.aexo) do
    --                    U.dump(e, '?? ex_upd:'.._)
                    local e = adec[idec]
                    local isrelevant
                    if e then
                        for k,b in pairs(ajunc[cjunc].list) do
                            if b.ind == e.frto[2] then
                                isrelevant = true
                                break
                            end
                        end
                    end
                    if e and isrelevant then
                            lo('?? for_exito:'.._..':'..tostring(e.id))
--- exit nodes
                        forExitB(adec[e.frto[1]], adec[e.frto[2]], e)
    --                    forExit(jdesc.list[e.frto[1]], jdesc.list[e.frto[2]], e)
                        nodesUpdate(e)
    --                        U.dump(b.aline, '?? for_line:')
                        -- lines right
                        for k,l in pairs(b.aline) do
                            l.on = true
                        end
--- exit lines
                        local bout,bin = b,U.mod(i+1,jdesc.list)
                        local aline = line4exit(e, bout, bin)
                        if aline then
                            for k,l in pairs(aline) do
    --                                lo('?? for_LINE:'..i..':'..k..':'..tostring(l.list and #l.list or nil))
                                if e.aline[k] then
                                    if e.aline[k].body then
                                        nodesUpdate(e.aline[k], l.list)
        --                                e.aline[k].body:setField('hidden', 0, e.aline[k].hide and 'true' or 'false')
                                    else
                                        e.aline[k].list = l.list
        --                                    toMark(l.list, 'mag')
                                        decalUp(e.aline[k])
                                        editor.updateRoadVertices(e.aline[k].body)
        --                                e.aline[k].body:setField('hidden', 0, e.aline[k].hide and 'true' or 'false')
        --                                e.aline[k].body:setPosition(e.aline[k].body:getPosition())
                                            lo('??+++++++ new_BODY:'..i..':'..k..':'..tostring(e.aline[k].body))
                                    end

                                end
                            end
                        else
                            lo('!! ERR_NO_ELINES:')
                        end
                    end
--                    nodesUpdate(sideLine(e, 'left', bout, 'left', bin, 'right', b.aline[1]))
--                    nodesUpdate(sideLine(e, 'right', bout, 'left', bin, 'right', b.aline[2]))
--                    nodesUpdate(sideLine(bout, 'left', e, 'left', bin, 'right', b.aline[3]))
--                        lo('??______________ pre_line_up:'.._)
--                    nodesUpdate(sideLine(bin, 'right', bout, 'left', e, 'left', b.aline[4]))

--                    sideLine(e, 'right', bout, 'left', bin, 'right', {w=default.wline*1.5, mat='m_line_white', lane={1, 0}})
--                    sideLine(bout, 'left', e, 'left', bin, 'right', {w=default.wline*1.5, mat='m_line_white', lane={1, 0}})
--                    sideLine(bin, 'right', bout, 'left', e, 'left', {w=default.wline*1.5, mat='m_line_white', lane={1, 0}})
--                        lo('??^^^^^^^^^^ bu1:')
                end
                for _,idec in pairs(b.aexi) do
                    local e = adec[idec]
                    if e then
--                            lo('?? for_exiti:'.._)
                        forExitB(adec[e.frto[1]], adec[e.frto[2]], e)
    --                    forExit(jdesc.list[e.frto[1]], jdesc.list[e.frto[2]], e)
                        nodesUpdate(e)
    --                        lo('?? for_lines_i:'.._..':'..#b.aline)
--- exit lines
---- lines left
                        local bout,bin = U.mod(i-1,jdesc.list),b
                        local aline = line4exit(e, bout, bin)
                        for i,l in pairs(aline) do
                            nodesUpdate(e.aline[i], l.list)
                        end
                    end
    --                    U.dump(e, '?? ex_upd:'.._)
--                    nodesUpdate(sideLine(e, 'left', bout, 'left', bin, 'right', b.aline[4+1]))
--                    nodesUpdate(sideLine(e, 'right', bout, 'left', bin, 'right', b.aline[4+2]))
--                    nodesUpdate(sideLine(bout, 'left', e, 'left', bin, 'right', b.aline[4+3]))
--                    nodesUpdate(sideLine(bin, 'right', bout, 'left', e, 'left', b.aline[4+4]))
--                        lo('?? bu2:')
                end
-- BRANCH LINES
                local il = 1
                -- middle line
                local ldesc
                if U._MODE == 'conf' then
                        lo('?? if_WIDTH:'..i..'/'..#jdesc.list..':'..tostring(jdesc.list[i].w)..':'..tostring(U.mod(i+1, jdesc.list).w))
                    ldesc = line4laneD(b, b.lane[1], U.mod(i+1, jdesc.list).w/2)
                else
                    ldesc = line4lane(b, b.lane[1], U.mod(i+1, jdesc.list).w/2)
                end
                nodesUpdate(b.aline[il], ldesc.list)
                il = il + 1
                -- lanes right
                for k=1,b.lane[1]-1 do
                    if U._MODE == 'conf' then
                        ldesc = line4laneD(b, k, U.mod(i+1, jdesc.list).w/2, default.matlinedash)
                    else
                        ldesc = line4lane(b, k, U.mod(i+1, jdesc.list).w/2, default.matlinedash)
                    end
                    nodesUpdate(b.aline[il], ldesc.list)
                    il = il + 1
                end
                -- lanes left
                for k=1,b.lane[2]-1 do
                    if U._MODE == 'conf' then
                        ldesc = line4laneD(b, k+b.lane[1], U.mod(i+1, jdesc.list).w/2, default.matlinedash)
                    else
                        ldesc = line4lane(b, k+b.lane[1], U.mod(i+1, jdesc.list).w/2, default.matlinedash)
                    end
                    nodesUpdate(b.aline[il], ldesc.list)
                    il = il + 1
                end
--[[
]]
                if b.link and b.link.body then
                    branchMerge(b, b.link.to, b.link)
                end
            end
            -- lines update
--                        lo('?? hide:'..ie)
--                    if yes then jdesc.aexit[ie].dirty = true end
--                    jdesc.aexit[ie].dirty = true
        end
    end
    jdesc.dirty = false
end


local function junctionUp_(pos, n)
        lo('>> junctionUp:'..tostring(pos)..':'..n)
    local step = default.rjunc
    out.avedit = {}
    local abr = {}
--    local adesc = {}
    local laneWidth = default.laneWidth
    for i=1,n do
        -- right side
--        local shift = 1
        local list = {pos}
        list[#list+1] = list[#list] + vec3(step*math.cos(i*2*math.pi/n), step*math.sin(i*2*math.pi/n))
        local desc = {list=list, w=laneWidth*default.laneNum*2, mat=default.mat, lane = {default.laneNum,default.laneNum}}
        local rd = decalUp(desc)
        rd.lanesRight = default.laneNum
        rd.lanesLeft = 2
        abr[#abr + 1] = {ind=i, body=rd, dir=(list[2]-list[1]):normalized(), io=1, desc=desc, aex={}}
--        adesc[#adesc+1] = desc
--[[
            local pleft = onSide(rd, 'left', 10, laneWidth/2)
            out.avedit[#out.avedit+1] = pleft
            lo('?? for_lanes:'..tostring(rd.lanesLeft)..':'..tostring(pleft))
            local pright = onSide(rd, ' right', 10, laneWidth/2)
            out.avedit[#out.avedit+1] = pright
]]
    end
    ajunc[#ajunc+1] = {list=abr, p=pos, aexit={}, aline={}, wlane=laneWidth, rexit=default.rexit, wexit=default.wexit}
    cjunc = #ajunc
    forExits(ajunc[#ajunc])
end


--[[
        if (p - vec3(0, 0, 0)):length() < 300 then
            lo('?? CC0:'..tostring(p))
        end
]]
--[[
                                            if across[rsrc] == nil then
                                                across[rsrc] = {}
                                            end
                                            if across[rsrc][rtgt] == nil then
                                                across[rsrc][rtgt] = {}
                                            end
                                            across[rsrc][rtgt] = n
--                                            across[rsrc][rtgt][#across[rsrc][rtgt] + 1] = n

                                            if across[rtgt] == nil then
                                                across[rtgt] = {}
                                            end
                                            if across[rtgt][rsrc] == nil then
                                                across[rtgt][rsrc] = {}
                                            end
                                            across[rtgt][rsrc] = k
]]
--                                            across[rtgt][rsrc][#across[rtgt][rsrc] + 1] = k


local function onSelect()
    lo('>> onSelect:'..tableSize(editor.selection.object))
    for i = 1, tableSize(editor.selection.object) do
        local selectedObject = scenetree.findObjectById(editor.selection.object[i])
            lo('?? SO:'..tostring(selectedObject:getClassName()))
        if selectedObject and selectedObject:getClassName() == "DecalRoad" then
--              local id = editor.selection.object[i]
              croad = scenetree.findObjectById(editor.selection.object[i])
                    lo('?? R_seld:'..tostring(croad))
              break
--              local nsec = rd:getEdgeCount()
--                lo('?? onSelect_selected:'..editor.selection.object[i]..':'..rd:getNodeCount()..':'..nsec)
--          table.insert(selectedRoadsIds, editor.selection.object[i])
        end
      end
end


local isfirst = true

local function hmapChange(p, h, yes)
--    if hsave == nil then hsave = core_terrain.getTerrainHeight(p) end
    local hsave = core_terrain.getTerrainHeight(p)
--    shmap[p.x..'_'..p.y] = core_terrain.getTerrainHeight(p)
--    if not out.inconform then
--    end
    if not shmap[p.y] then
        shmap[p.y] = {}
    end

    if not shmap[p.y][p.x] then
        shmap[p.y][p.x] = hsave
    end

    if isfirst or yes then
--        shmap[x..y] = hsave
--        shmap[#shmap+1] = {p, hsave}
    end
    if not tb then
--        lo('!! NO_TB:')
        return
--        tb =
    end
--        lo('?? hmapChange:'..tostring(p)..':'..h)
    tb:setHeightWs(p, h)
end
D.hmapChange = hmapChange


local function restore()
        lo('>> D.restore:'..#shmap)
--        if true then return end
--[[
    for _,v in pairs(shmap) do
        tb:setHeightWs(v[1], v[2])
    end
]]
    for y,row in pairs(shmap) do
        for x,h in pairs(row) do
            tb:setHeightWs(vec3(x,y), h)
        end
    end
    shmap = {}

    if not tb then
        tb = extensions.editor_terrainEditor.getTerrainBlock()
    end
    if not tb then return end
    tb:updateGrid() -- TODO: no arguments makes warning
--    out.avedit = {}
    out.apick = {}
    out.apoint = {}
    out.inconform = false
    if croad then
        lo('?? decal.restore:'..tostring(croad:getPosition()))
--        croad:setPosition(croad:getPosition())
    end
end


local dline = {}

local function widthRestore()
    lo('>>+++++!!!!!!!!!!!!!! widthRestore:'..tableSize(aedit))
        if true then return end
    for id,data in pairs(aedit) do
        local anode = editor.getNodes(data.body)
        for i,n in pairs(anode) do
            editor.setNodeWidth(data.body, i-1, data.w)
        end
        editor.updateRoadVertices(data.body)
        data.body:setField("material", 0, 'WarningMaterial')
        -- clean marking lines
        if groupLines then
            groupLines:deleteAllObjects()
        end
--[[
        local rd = dline[id..'_middle']
        if rd then rd:delete() end
        rd = dline[id..'_left']
        if rd then rd:delete() end
        rd = dline[id..'_right']
        if rd then rd:delete() end
]]
    end

    restore()
end


local mantle = 4
local aeinfo, aepin
local aspline -- {parabola (a,b,c)}

local function forHSpline(ipin, between)
--    lo('>> forHSpline:'..ipin..':'..between..':'..aepin[ipin+1][1]..':'..aepin[ipin+1][2])
--    return aeinfo[aepin[ip1]]
    local s1 = aspline[ipin]
    local s2 = aspline[ipin+1]
    local d1 = aeinfo[aepin[ipin][1]].d
    local d2 = aeinfo[aepin[ipin+1][1]].d
    local d = d1 + (d2 - d1)*between
--        lo('?? forHSpline:'..d..'/'..d2..':'..between)
    local a = s1[1]*d^2 + s1[2]*d + s1[3]
    local b = s2[1]*d^2 + s2[2]*d + s2[3]
    local h = U.spline2(a, b, between, 1)

--    local h -- = forSpline(ipin, between)
--            h = aepin[ipin][2] + 1*(aepin[ipin+1][2] - aepin[ipin][2])*between
    return h,h
end
D.forHSpline = forHSpline


local function spline(apar, k, t, s)
    local s1 = apar[k]
    local s2 = apar[k+1]
    local a = s1[1]*t^2 + s1[2]*t + s1[3]
    local b = s2[1]*t^2 + s2[2]*t + s2[3]

    return U.spline2(a, b, s)
end
D.spline = spline


local function inAdec(rid)
    for i,r in pairs(adec) do
        if r.id == rid then
--            cdec = i
            return i
        end
    end
    return nil
end


local crossSpot = {}
local conformed = {}

local rdlist = (editor and editor.getAllRoads) and editor.getAllRoads() or {}

lo('?? deacl:'..tostring(tableSize(rdlist)))


local function ter2road(pth, cid)
    if #adec == 0 then
        decalsLoad(L)
    end
        lo('?? ter2road:'..tostring(cpick)..':'..tostring(cid))
    if cid then cpick = cid end
    if not cpick then return end
--    U.dump(adec, '>> ter2road:'..tostring(#adec or #adec)..':'..tostring(cpick)..':'..#editor.getAllRoads())

--        if true then return end
--[[
            local x, y = -295, 34
            for i = x+1,x+20 do
                for j = y+1,y+20 do
                    hmapChange(vec3(i,j), 300)
--                    tb:setHeightWs(vec3(i,j), 300)
                end
            end
            tb:updateGrid()
            if true then return end
]]
--    rdlist = editor.getAllRoads()
--[[
    if false and tableSize(adec) == 0 then
            lo('!! TO_LOAD:')
        tb = extensions.editor_terrainEditor.getTerrainBlock()
        adec,aref = decalsLoad('/levels/ce_test/main/MissionGroup/decalroads/AI_decalroads/items.level.json', 4096, 10)
        forCross()
    end
]]
    local anode
--    local rcross

    if pth then
        local list = {}
        for i,p in pairs(pth) do
            list[#list+1] = p.pos
        end
        croad = decalUp({
            list = list,
            w = pth[1].width,
            mat = 'WarningMaterial',
        })
        anode = pth
    else
        --    croad = scenetree.findObjectById(editor.selection.object[1])
        croad = scenetree.findObjectById(cpick)
        --            lo('>> ter2road:'..tostring(editor.selection.object[1]))
        anode = editor.getNodes(croad)
                lo('?? ter2road:'..tableSize(adec)) --..':'..tostring(adec[1].list[1])..':'..croad:getEdgeCount()..':'..tostring(anode[2].width))
        if #U.index(conformed, cpick) == 0 then
            conformed[#conformed + 1] = cpick
        end
        -- identify in adec
        cdec = inAdec(cpick)
--        rcross = 5*croad:getNodeWidth(0)/3.5 -- radius of pinned around crossroad
    end
--[[
    for i,r in pairs(adec) do
        if r.id == cpick then
--        if (r.list[1] - anode[1].pos):length() < 0.1 and #r.list == #anode then
--            lo('?? match1:'..i..':'..tostring(r[1])..':'..tostring(anode[1].pos)..' n1:'..#r..' n2:'..#anode)
            cdec = i
        end
    end
]]
--[[
        if (r[#r] - anode[#anode].pos):length() < 0.1 then
            lo('?? match2:'..i)
        end
]]
    out.avedit = {}

    local acp = {} -- crossing nodes indexes
    if cdec and across[cdec] then
        for n,lst in pairs(across[cdec]) do
            lo('?? for_cross:'..n)
            acp[#acp + 1] = n
--            out.avedit[#out.avedit+1] = adec[cdec].list[n]
        end
    end
    table.sort(acp)
    --
--            U.dump(across[cdec], '?? ter2road: cdec='..cdec..' nodes:'..tostring(#adec[cdec]))
--            U.dump(across[cdec], '?? ter2road: cdec='..cdec..' nodes:'..tostring(#adec[cdec]))
            U.dump(out.avedit)
    local rcross = 5*croad:getNodeWidth(0)/3.5 -- radius of pinned around crossroad
            U.dump(acp, '?? acp:'..anode[1].width..':'..tostring(anode[1].pos)..' rcross:'..rcross)
    local dpin = 20 -- distance between pins
    local cint = 1 -- index of upcoming crossing node
    local nsec = croad:getEdgeCount()
    -- edge[0] == node[1]

--            U.dump(andist, '??____ andist:')
--            U.dump(aedist, '?? aedist:')
--            U.dump(acp, '?? acp:')
    -- distances of edges and nodes from start
    local andist,prepos = {}
    aeinfo = {}
    local cdist,nxtnode = 0,1
    for i = 0,nsec-1 do
        local epos = croad:getMiddleEdgePosition(i)
--            if i < 5 then lo('?? for_aei:'..i..':'..) end
        if (epos - anode[nxtnode].pos):length() < 0.02 then
            andist[nxtnode] = cdist
            nxtnode = nxtnode + 1
        end
--                if i < 5 then lo('?? for_d:'..i..':'..cdist..':'..tostring(epos)) end
        if prepos then
--               if i < 5 then lo('?? for_d2:'..i..':'..cdist) end
            cdist = cdist + (epos - prepos):length()
        end
            aeinfo[#aeinfo+1] = {d = cdist, pos = epos, ipin = nil}
        prepos = epos
    end
        lo('?? aeidst:'..#aeinfo..':'..#anode)

    -- set pins
    aepin = {} -- {edge_index(1-baed),pinned_height}
    cint = 1
    local cheight = core_terrain.getTerrainHeight(vec3(anode[1].pos.x,anode[1].pos.y))
--            aepin = {{1, cheight}}
--    local cheight = core_terrain.getTerrainHeight(vec3(anode[acp[cint]].pos.x,anode[acp[cint]].pos.y))
    local inswitch = false
    local cdist = 0
    out.apick = {}
    for i = 0,nsec-1 do
--            lo('?? for_i:'..i..':'..tostring(aedist[i+1])..' cint:'..cint..' acp:'..tostring(acp[cint])..':'..tostring(andist[acp[cint]]))
--            lo('?? for_e:'..i..':'..cint..':'..tostring(acp[cint]))
        if acp[cint] and math.abs(aeinfo[i+1].d - andist[acp[cint]]) < rcross then
            -- TODO: process tail
            -- pin near cross
            aepin[#aepin+1] = {i+1, cheight}
--            aepin[i+1] = cheight
            inswitch = false
--                lo('?? for_CROSS:'..i..':'..cint..' ei:'..acp[cint])
                local p = croad:getMiddleEdgePosition(i)
--                    lo('?? for_p:'..i..':'..tostring(p)..':'..aepin[i])
                    p.z = aepin[#aepin][2]
--                    out.apick[#out.apick + 1] = p -- vec3(p.x,p.y,aepin[#aepin][2])
--                    lo('?? for_pin1:'..#aepin..':'..tostring(p)..':'..aeinfo[i+1].d)
        else
            if #aepin > 0 and not inswitch and acp[cint+1] then
                cint = cint + 1
                    lo('??++++ NEXT_cross:'..cint..':'..i..'/'..nsec)
                inswitch = true
                cheight = core_terrain.getTerrainHeight(vec3(anode[acp[cint]].pos.x,anode[acp[cint]].pos.y))
                cdist = aeinfo[i+1].d
            end
            if aeinfo[i+1].d - cdist > dpin then
                if (aeinfo[#aeinfo].d - aeinfo[i+1].d) >= rcross + 2/3*dpin then
                    -- not too close to cross area
                    local p = croad:getMiddleEdgePosition(i)
                    aepin[#aepin+1] = {i+1, core_terrain.getTerrainHeight(vec3(p.x,p.y))}
    --                aepin[i+1] = core_terrain.getTerrainHeight(vec3(p.x,p.y))
                    cdist = aeinfo[i+1].d
                        out.apick[#out.apick + 1] = p -- vec3(p.x,p.y,aepin[#aepin][2])
                end
--                    lo('?? for_pin2:'..#aepin..':'..tostring(p)..':'..aeinfo[i+1].d)
            end
        end
--        cdist -
    end
    if #aepin > 0 then
        if aepin[1][1] > 1 then
            table.insert(aepin, 1, {1, cheight})
        end
        if aepin[#aepin][1] < nsec then
            local p = croad:getMiddleEdgePosition(nsec-1)
            aepin[#aepin + 1] = {nsec, core_terrain.getTerrainHeight(vec3(p.x,p.y))}
        end
    end
--            U.dump(aepin, '?? aepin:'..nsec)

    -- pin info {dist_from_start,pin_interval,position}
--    local aeinfo = {}
    local nxtpin = 1
    for i = 1,#aeinfo do
--            lo('?? for_e:'..i..':'..nxtpin..':'..aepin[nxtpin][1])
                if not aepin[nxtpin] then
                    lo('!! NO_pin:'..nxtpin)
                end
        if aepin[nxtpin] and i == aepin[nxtpin][1] then
--        if (aedist[i][3] - aedist[aepin[nxtpin][1]][3]):length() < 0.02 then
--            lo('?? NXT_PIN:'..i..':'..nxtpin) --..aepin[nxtpin][1])
            aeinfo[i].ipin = nxtpin
--            prepin = nxtpin
            nxtpin = nxtpin + 1
        else
            aeinfo[i].ipin = nxtpin - 1
        end
            if i < 20 then
--                U.dump(aeinfo[i], '?? aedist:'..i)
            end
    end

    -- spline
    local ain = {}
    for i,p in pairs(aepin) do
        ain[#ain + 1] = {aeinfo[p[1]].d, p[2]}
    end
--        U.dump(aepin, '?? AEPIN:'..croad:getEdgeCount())
        U.dump(ain, '?? AIN:')
    aspline = U.paraSpline(ain)
    table.insert(aspline, 1, aspline[1])
    aspline[#aspline + 1] = aspline[#aspline]
        U.dump(aspline, '?? splined:'..#aspline)
--[[
    for i = 2,2 do --#aepin-1 do
        local acoef = U.paraSpline({
            {aeinfo[aepin[i-1][1] ].d, aepin[i-1][2]},
            {aeinfo[aepin[i][1] ].d, aepin[i][2]},
            {aeinfo[aepin[i+1][1] ].d, aepin[i+1][2]},
        })
            U.dump(acoef, '?? splined:'..i)
    end
]]

    -- build stamps
    out.apath = {}
    out.avedit = {croad:getMiddleEdgePosition(0)}
    out.apoint = {}
    local a,b = 1,1
    local w = a*croad:getNodeWidth(0) + mantle -- a*anode[1].width + mantle
    local r = w*b
    local asq = {croad:getMiddleEdgePosition(0)} -- square stamps position
    local ap = {} -- points within squares {dist2chord,height}
    local cd = 0
    local ex,ey = vec3(1,0,0),vec3(0,1,0)
    local nstamp = 0
    for i = 1,nsec-1 do
        local s = croad:getMiddleEdgePosition(i)-croad:getMiddleEdgePosition(i-1)
--            lo('?? for_s: i='..i..' cd:'..cd..':'..s:length()..' w:'..(a*w))
        if cd + s:length() > w then
            -- limiting edges for distance check
            local ifr,ito -- 1-based
            local fromCenter = w - cd
            for k = i-1,0,-1 do
                if fromCenter > (r+1)*2/math.sqrt(2) or k == 0 then
                    ifr = k + 1
                    break
                end
                fromCenter = fromCenter + (aeinfo[k+1].d - aeinfo[k-1+1].d)
            end
            fromCenter = cd + s:length() - w
            for k = i,nsec-1,1 do
                if fromCenter > (r+1)*2/math.sqrt(2) or k == nsec-1 then
                    ito = k + 1
                    break
                end
                fromCenter = fromCenter + (aeinfo[k+1+1].d - aeinfo[k+1].d)
            end
--                if #out.apath == 1 then
--                    lo('?? fr-to:'..(#out.apath+1)..':'..ifr..':'..ito)
--                    out.apoint = {aeinfo[ifr].pos, aeinfo[ito].pos}
--                end
            -- current width
            w = a*(croad:getLeftEdgePosition(i) - croad:getRightEdgePosition(i)):length() + mantle -- a*croad:getNodeWidth(0) + mantle
--                lo('??_______ for_width:'..i..':'..w)
            -- set new square
            nstamp = nstamp + 1
            local p = croad:getMiddleEdgePosition(i-1) + s:normalized()*(w - cd)
--                lo('?? stamp:'..i..':'..(p - out.avedit[#out.avedit]):length()..' cd:'..cd..' s:'..s:length())
--            out.avedit[#out.avedit + 1] = p
            cd = cd + s:length() - w

            local pth = {p - ex*r - ey*r}
            pth[#pth+1] = pth[#pth] + 2*r*ex
            pth[#pth+1] = pth[#pth] + 2*r*ey
            pth[#pth+1] = pth[#pth] - 2*r*ex
            pth[#pth+1] = pth[#pth] - 2*r*ey

--            out.apath[#out.apath + 1] = pth
--                lo('?? nSQ:'..#out.apath)

            if true or #out.apath == 2 then -- 106 then
--            if #out.apath > 1 and #out.apath < 9 then
--            if #out.apath == 7 or #out.apath == 8 then

                local xfr,xto = math.floor(pth[1].x),math.ceil(pth[2].x)
                local yfr,yto = math.floor(pth[1].y),math.ceil(pth[4].y)
--                        local np = 0
                for y = yfr,yto do
--[[
                    if ap[y] == nil then
                        ap[y] = {}
                    end
                        if ap[y][x] == nil then
                            ap[y][x] = {math.huge,nil} -- {dist2chord,height}
                        end
]]
                    for x = xfr,xto do
                        local t = vec3(x,y)
                        t.z = core_terrain.getTerrainHeight(t)

                        local dmi, imi = math.huge -- 1-based

                        for k = ifr,ito-1 do
                            local dt = t:distanceToLineSegment(aeinfo[k].pos, aeinfo[k+1].pos)
                            if dt < dmi then -- ap[y][x][1] then
--                                ap[y][x][1] = dt
                                dmi = dt
                                imi = k
                            end
--                                    out.apoint[#out.apoint+1] = aedist[k][3]
                        end

                                                    -- get height
                        --                                lo('?? xy:'..np..':'....ifr..':'..ito..' imi:'..imi..' xy:'..x..':'..y)
                        local rwidth = (croad:getLeftEdgePosition(imi) - croad:getRightEdgePosition(imi)):length()
                            --anode[1].width
                        if dmi < rwidth + mantle then
                            local p1, p2 = closestLinePoints(U.proj2D(aeinfo[imi].pos), U.proj2D(aeinfo[imi+1].pos), t, U.proj2D(t))
                            -- TODO: p1 may be > 1 (non-ontersecting segments)
                        --                                        lo(aedist[imi][1]..':'..andist[aedist[imi][2]])
                        --                                    local c = (aedist[imi][1] - andist[aedist[imi][2]])/(andist[aedist[imi][2]+1] - andist[aedist[imi][2]])
                            if not aeinfo[imi] or not aepin[aeinfo[imi].ipin] or not aepin[aeinfo[imi].ipin + 1] then
                                lo('!! ERR_noinf:'..imi..'/'..nsec..':'..aeinfo[imi].ipin)
                                return
                            end
                            local pin1,pin2 = aepin[aeinfo[imi].ipin][1],aepin[aeinfo[imi].ipin+1][1]
                        --                                        U.dump(aeinfo[imi], '?? dt: ie:'..imi..':'..ap[y][x][1]..':'..tostring(t)..':'..tostring(aeinfo[imi].pos)..':'..tostring(p1)..':'..tostring(p2)..' pin1:'..pin1..' pin2:'..pin2..' d1:'..aeinfo[pin1].d..' d2:'..aeinfo[pin2].d)
                            local fromOrigin = aeinfo[imi].d + (aeinfo[imi+1].d - aeinfo[imi].d)*p1
                            local c = (fromOrigin - aeinfo[pin1].d)/(aeinfo[pin2].d - aeinfo[pin1].d)
                            local hbase, _h = forHSpline(aeinfo[imi].ipin, c)
--                                if nstamp == 4 then
--                                    lo('?? dh:'..(hbase-_h))
--                                end
                        --                                        lo('?? for_dist:'..aeinfo[pin1].d..':'..aeinfo[imi].d..':'..aeinfo[pin2].d..' c:'..c..' h:'..hbase..' d:'..ap[y][x][1]..':'..dmi..' w:'..rwidth)
                        --                                    local hter = t.z -- core_terrain.getTerrainHeight(p)

                            if imi == 1 and c*aeinfo[2].d < -1 then
                                -- skip 1st end
--                                out.apoint[#out.apoint+1] = t
                            elseif imi == nsec-1 and c*(aeinfo[nsec].d - aeinfo[nsec-1].d) - 1 > 1 and c > 1 then
                                -- skip last end
                        --                                                lo('?? END:'..imi..'/'..nsec)
--                                out.apoint[#out.apoint+1] = t
                            elseif crossSpot[y] and crossSpot[y][x] and crossSpot[y][x] ~= cpick then
                                -- skip another road points
--                                out.apoint[#out.apoint+1] = t
                            else
                                local h
                                if dmi < rwidth/2 + 1.4 then
                        --                                                lo('?? to_height: n='..np..' imi:'..imi..'/'..nsec..' pin:'..aeinfo[imi].ipin..':'..x..':'..y..' c='..c..' h='..hbase..':'..aeinfo[imi].ipin..' pin1:'..pin1..' pin2:'..pin2)
                                    h = hbase
                        --                                            h = 102
--                                        h = 300
                                    if true then
                                        hmapChange(t, h) -- t.z)
                                    else
                                        h = t.z
                                    end
                                    if fromOrigin < rcross then -- and dmi < rwidth/2 then
--                                        out.apoint[#out.apoint+1] = t
                                        -- to spot
                                        if crossSpot[y] == nil then
                                            crossSpot[y] = {}
                                        end
                                        crossSpot[y][x] = cpick
                                    end
                        --                                                out.avedit[#out.avedit+1] = vec3(t.x,t.y,h)
                                elseif dmi < rwidth/2 + mantle then
                                    h = U.spline2(hbase, t.z, dmi - (rwidth/2), mantle)
--                                            h = 300
                                    hmapChange(t, h)
--                                    out.avedit[#out.avedit+1] = vec3(t.x,t.y,h)
                                end
                            end
                        end
                    end
                end
            end
        else
            cd = cd + s:length()
        end
    end
    tb:updateGrid()
--    be:reloadCollision()
--    editor.updateRoadVertices(croad)
--    editor_roadUtils.reloadDecals(croad)
    out.inconform = true
    isfirst = false
    croad:setPosition(croad:getPosition())
    -- refresh nodes marking
    anode = editor.getNodes(croad)
    out.apicknode = {}
    for _,n in pairs(anode) do
        out.apicknode[#out.apicknode + 1] = n.pos
    end

    if pth then
        scenetree.MissionGroup:removeObject(croad)
    else
        D.middleUp(croad)
        D.sideUp(croad)

        if not aedit[cpick] then
            aedit[cpick] = {body = croad, w = croad:getNodeWidth(0)}
        end
    end
    if cid then cpick = nil end
        lo('<< ter2road:'..#out.avedit..':'..#shmap..':'..aeinfo[#aeinfo].d) --....croad:getField("Material", "")..':'..tostring(croad:getPosition()))
--        U.dump(ap, '?? asq:'..tostring(asq[1])..':'..tableSize(ap))
end

--[[
                        if true or dmi < anode[1].width/2 + 1 + 0*mantle then
--                            out.avedit[#out.avedit+1] = t + vec3(0,0,1)
                        end
                                np = np + 1

                            if true or np == 50 then
                            end
    --                    if 1 < #out.apath and #out.apath < 8 then
                        if true or #out.apath == 8 then
    --                        if #out.apath == 8 then
    --                        out.apoint = {}
    --                            U.dump(aedist, '?? aedist:'..ifr..':'..ito)
                        end
]]
--                                    local h = U.spline2(hbase, hter, a, a + b)
--                                        hmapChange(t, h)
--                                    local topin1 = aeinfo[pin1]
--                                    aeinfo[ipin].d
--                                    out.apoint = {}
--                                    out.apick = {t}
--[[
                    if ap[y][x] == nil then
                        ap[y][x] = {math.huge,nil} -- {dist2chord,height}
                    end
                    -- get distance to chord
                    local ae = {}
                    local kshift = 0
                    if i > 1 then
                        ae[#ae + 1] = croad:getMiddleEdgePosition(i-2)
                    else
                        kshift = 1
                    end
                    ae[#ae + 1] = croad:getMiddleEdgePosition(i-1)
                    ae[#ae + 1] = croad:getMiddleEdgePosition(i)
                    if i < nsec-1 then
                        ae[#ae + 1] = croad:getMiddleEdgePosition(i+1)
                    end
                    local imi   -- edge index
                    for k = 2,#ae do
                        local dt = t:distanceToLineSegment(ae[k-1], ae[k])
                        if ap[y][x][1] > dt then
--                            ap[y][x] = {dt, i + k + kshift - 4}
                            ap[y][x][1] = dt
                            imi = i + k + kshift - 4
                        end
                    end
                    -- get height
                    --- node_interval
                    if imi == nil then
--                        U.dump(ae,'?? for_IMI:'..i..':'..tostring(imi)..':'..#ae..':'..tostring(t))
--                        out.apoint[#out.apoint+1] = t
                        out.apick = {t}
                        for _,pe in pairs(ae) do
--                            out.apoint[#out.apoint+1] = pe
                        end
--                        return
                    else
                        local inode = aedist[imi][2]
                        local p1,p2 = vec3(anode[inode].pos.x,anode[inode].pos.y),vec3(anode[inode+1].pos.x,anode[inode+1].pos.y)
                        local vp = vec3(-(p2-p1).y,(p2-p1).x)
                        local cx,cy = U.lineCross(p1, p2, t, t+vp)
                                if i == 10 then
                                    lo('?? for_cross:'..cx..':'..cy)
                                end

                        if ap[y][x][1] < w/2 + mantle + 1 then
                            if #out.apath == 2 then
--                                out.avedit[#out.avedit+1] = t
                            end
                        end
                    end
]]
--                        t:distanceToLineSegment(a, b)
--[[
    local aedist,cdist = {{0,1,croad:getMiddleEdgePosition(0)}},0 -- {dist_from_start,node_interval}
    local nxtnode = 1
    local andist = {}
    local prepos
    for i = 0,nsec-1 do
        local epos = croad:getMiddleEdgePosition(i)
        if (epos - anode[nxtnode].pos):length() < 0.02 then
            nxtnode = nxtnode + 1
--            lo('?? NODE_hit:'..i..':'..cnode)
        end

        if prepos then
            cdist = cdist + (epos - prepos):length()
--            aedist[#aedist+1] = {d = cdist, n = nxtnode-1, p = epos}
            aedist[#aedist+1] = {cdist, nxtnode-1, epos}
        end
--            lo(i..':'..tostring(epos)..'/'..tostring(anode[acp[cint] ].pos))
        if (epos-anode[acp[cint] ].pos):length() < 0.02 then
    --            lo('?? got_cross:'..acp[cint])
                andist[acp[cint] ] = cdist
                cint = cint + 1
            end
            prepos = epos
        end
]]
--[[
        if (epos - anode[nxtnode].pos):length() < 0.02 then
            nxtnode = nxtnode + 1
--            lo('?? NODE_hit:'..i..':'..cnode)
        end
]]
--[[
--                lo('?? for)cint:'..cint)
            if cint < #acp then
--                cint = cint - 1
            end
]]
--        U.dump(andist, '?? ndist:')
--        lo('?? EN:'..tostring(croad:getMiddleEdgePosition(nsec-1))..':'..tostring(anode[#anode].pos))
--        lo('?? EN:'..tostring(croad:getMiddleEdgePosition(0))..':'..tostring(anode[1].pos))

--        U.dump(anode[1], '?? for_adec:'..tostring(adec[1][1]))
--[[
local function forLanes()
    return im.IntPtr(1), im.IntPtr(1)
end
]]

local function atEdge(p, rd, start)
    if not start then start = 0 end
    local nsec = rd:getEdgeCount()
    for i = start,nsec-1 do
        local t = U.proj2D(p)
        local p1, p2 = closestLinePoints(
            U.proj2D(rd:getMiddleEdgePosition(i)), U.proj2D(rd:getMiddleEdgePosition(i+1)),
            t, vec3(t.x, t.y, 1))
        if p1 >= 0 and p1 <= 1  then
            return i
        end
    end
end


local function node2edge(rd, dbg)
    local anode = editor.getNodes(rd)
    local nsec = rd:getEdgeCount()
    local nxtnode = 1
    local list = {}
    local cdist, prepos = 0
    for i = 0,nsec-1 do
        local epos = rd:getMiddleEdgePosition(i)
--            if dbg then lo('?? nn:'..i..':'..tostring(nxtnode)..'/'..#anode) end
        if (epos - anode[nxtnode].pos):length() < 0.02 then
            list[#list + 1] = {e = i, d = cdist}
            nxtnode = nxtnode + 1
            if nxtnode > #anode then
                lo('!! node2edge_NODE_EX:'..nxtnode..'/'..#anode)
                break
            end
        end
        if prepos then
            cdist = cdist + (epos - prepos):length()
        end
        prepos = epos
    end
--        U.dump(list, '<< node2edge:'..#list)
    return list
end


local function middleUp(rdobj)
        if true then return end
    if D.ui.laneL == 0 or D.ui.laneR == 0 then return end
    if not rdobj then rdobj = croad end
    -- build nodes
    local list = {}
    local anode = editor.getNodes(rdobj)
    local c = ((D.ui.laneL + D.ui.laneR)/2 - D.ui.laneL)/(D.ui.laneL + D.ui.laneR)*2
        lo('?? middleUp_c:'..c..':'..tostring(D.ui.middleYellow))
    local v
    for i = 1,#anode-1 do
        v = U.proj2D(anode[i+1].pos - anode[i].pos):normalized()
        v = vec3(-v.y,v.x)
        list[#list+1] = anode[i].pos + v*c*anode[i].width/2
    end
    -- last node
    local nsec = rdobj:getEdgeCount()
    v = (rdobj:getLeftEdgePosition(nsec-1) - rdobj:getRightEdgePosition(nsec-1)):normalized()
    list[#list+1] = anode[#anode].pos + v*c*anode[#anode].width/2

--        list[#list+1] = anode[#anode].pos + v*c*anode[i].width/2

    local rd = dline[cpick..'_middle']
    if rd then
        rd:delete()
        dline[cpick..'_middle'] = nil
    end

    local mat = D.ui.middleYellow and 'line_yellow' or
                    (D.ui.middleDashed and 'line_white_dashed' or 'line_white')
    rd = decalUp({
        list = list,
        w = 0.3,
        mat = mat,
--                lanes = {1, 0},
    })
    dline[cpick..'_middle'] = rd
    rd.name = 'line_'..cpick..'_'..rd:getID()   --:registerObject('road'..'_'..'13213111')
        lo('<< middleUp____:'..tostring(dline[cpick..'_middle'].name)..':'..tostring(rd.obj))
    groupLines:add(rd.obj)
end


local function sideUp(rdobj)
    if U._MODE == 'conf' then return end
        lo('>>_________________ sideUp:')
    if not rdobj then rdobj = croad end
    -- build nodes
    local list,listr = {},{}
    local margin = 0.3
    local anode = editor.getNodes(rdobj)
    local nsec = rdobj:getEdgeCount()

    -- trim ends
    local pp = {}
    for i,rd in pairs(adec) do
        if rd.id == cpick then
            cdec = i
--                U.dump(us, '?? us:')
--                U.dump(pp, '?? neigh:')
--[[
            local function comp(a, b)
                return math.abs(a.ang - us.ang) < math.abs(b.ang - us.ang)
            end
            table.sort(star, comp)
                U.dump(star, '?? star_post:')
            for _,b in pairs(star) do

            end
]]
            break
        end
    end
    --- start
    local function forEnd(ndi)
        lo('?? forEnd:'..tostring(across and #across or 'NONE')..'/'..tostring(cdec))
        if not cdec then return end
        local astem,us = U.clone(across[cdec][ndi])
        astem[#astem + 1] = {cdec, ndi}
    --        U.dump(across[cdec][ndi], '?? sideUp.stem:') --across[cdec], '?? acr:'..i)
        local star = U.forStar(astem, adec)
    --        U.dump(star, '?? star: rdi:'..cdec..' node:'..ndi)
        -- get neighbours
        for k,f in pairs(star) do
            if f.rdi == cdec then
                us = f
    --                    table.remove(star, k)
                pp = {star[(k-2) % #star + 1], star[k % #star + 1]}
                break
            end
        end
        if not pp[1] then return nil end

--            U.dump(pp, '?? NB: us='..cpick..' end='..ndi) --..#adec[pp[2].rdi].list)
--            U.dump(bright, '?? for_RIGTH_branch:')
        local usfr = ndi == 1 and 0 or (nsec - 1) --
--        local usto = ndi == 1 and (nsec - 1) or 0
        local usto = usfr + (us.ndi - us.fr)*20
        local usstep = us.ndi - us.fr

        local function forSide(br, flip)
--                U.dump(br, '>> forSide:'..tostring(flip)..':'..us.rdi..'>'..br.rdi)
            local rdthem = adec[br.rdi].body
            --        local n2e = node2edge(rdthem)

            local nsecthem = rdthem:getEdgeCount()
            local themfr = br.fr == 1 and 0 or (nsecthem - 1)
            local themto = themfr + (br.ndi-br.fr)*20
            local themstep = br.ndi-br.fr

--                lo('?? ends: us='..usfr..'>'..usto..' them='..themfr..'>'..themto) --..':'..tostring(rdthem)..':'..(bright.ndi-bright.fr)..':'..((bright.ndi-bright.fr)*20)..':'..bright.fr..':'..(bright.fr + (bright.ndi-bright.fr)*20))
                out.avedit = {}
                out.apick = {}
            -- get closest edges
            local dmi,imi,jmi = math.huge
            for i = usfr,usto,usstep do
                local cdmi,cjmi = math.huge
                for j = themfr,themto,themstep do
    --                    lo('?? for_j:'..j)
                    local eus = flip and rdobj:getLeftEdgePosition(i) or rdobj:getRightEdgePosition(i)
                    local ethem = flip and rdthem:getRightEdgePosition(j) or rdthem:getLeftEdgePosition(j)
                    local d = (eus - ethem):length()
                    if d < cdmi then
                        cdmi = d
                        cjmi = j
                    end
                end
--                    out.apick[#out.apick + 1] = rdthem:getRightEdgePosition(cjmi)
    --                lo('?? cdmi:'..i..':'..cdmi..':'..tostring(rdthem:getLeftEdgePosition(jmi)))
    --                out.avedit = {croad:getRightEdgePosition(i), rdthem:getLeftEdgePosition(jmi)}
--                    if true then break end
                if cdmi < dmi then
                    dmi = cdmi
                    imi = i
                    jmi = cjmi
                end
            end
--                lo('?? mi: us='..tostring(imi)..':'..tostring(dmi)..' them:'..tostring(jmi))
    --            if true then return end

--                out.apick = {croad:getRightEdgePosition(imi), rdthem:getRightEdgePosition(jmi)}
            local ifr = imi > 0 and imi - 1 or 0 -- 0-based
            local ito = (imi < nsec - 1) and imi + 1 or nsec - 1
            local jfr = jmi > 0 and jmi - 1 or 0 -- 0-based
            local jto = (jmi < nsec - 1) and jmi + 1 or nsecthem - 1
            local us1 = (usstep > 0 and not flip) and rdobj:getRightEdgePosition(ifr) or rdobj:getLeftEdgePosition(ifr)
            local us2 = (usstep > 0 and not flip) and rdobj:getRightEdgePosition(ito) or rdobj:getLeftEdgePosition(ito)
            local them1 = (themstep > 0 and not flip) and rdthem:getLeftEdgePosition(jfr) or rdthem:getRightEdgePosition(jfr)
            local them2 = (themstep > 0 and not flip) and rdthem:getLeftEdgePosition(jto) or rdthem:getRightEdgePosition(jto)
            local x,y = U.lineCross(
                us1, us2,
                them1, them2
            )
--                lo('?? uth:'..tostring(us1)..':'..tostring(us2)..':'..tostring(them1)..':'..tostring(them2)..':')
                out.avedit = {us1, us2, them1, them2}
                out.apick = {vec3(x,y,core_terrain.getTerrainHeight(vec3(x,y)))}
--                lo('?? fr_to:'..ifr..'>'..ito..'/'..nsec..':'..jfr..'>'..jto..'/'..nsecthem)

            local u = us1 - us2
            local v = them1 - them2
--            local u = usstep*U.proj2D(croad:getRightEdgePosition(ifr) - croad:getRightEdgePosition(ito)):normalized()
--            local v = themstep*U.proj2D(rdthem:getLeftEdgePosition(jfr) - rdthem:getLeftEdgePosition(jto)):normalized()
--                out.apoint = {out.apick[#out.apick] + u, out.apick[#out.apick] + v}
            local ang = U.vang(u, v)/2
            local p = vec3(x,y) + (u + v):normalized()*margin/math.sin(ang)
            p.z = core_terrain.getTerrainHeight(p)
            local ie = atEdge(p, rdobj)
--                lo('?? mm: p='..tostring(p)..':'..tostring(imi)..':'..jmi..':'..dmi..':'..x..':'..y..' ie:'..tostring(ie))
    --            lo('?? p:'..tostring(p)..':'..tostring(ie))
--                out.apick[#out.apick + 1] = p
--                out.apoint = {p}
            return p,ie
        end
        local bright = ndi == 1 and pp[1] or pp[2]
        local bleft = ndi == 1 and pp[2] or pp[1]

        local pr,ier
        if bright and us.ang - bright.ang < math.pi then
            pr,ier = forSide(bright)
        end
        local pl,iel
        if bleft and bleft.ang-us.ang < math.pi then
            pl,iel = forSide(bleft, true)
        end
--            lo('?? for_angs:'..us.ang..'>'..bleft.ang..':'..(bleft.ang-us.ang)..':'..math.pi)

        return pr,ier,pl,iel
    end
    local pR,ie,pL,iel = forEnd(1)
--    local pRe,iee,pLe,iele = forEnd(#anode)
--            lo('?? sideUp.ending:'..tostring(pR)..':'..tostring(pL)) --..':'..tostring(pLe)..':'..tostring(iele))
    local n2e
    if pR or pL then n2e = node2edge(rdobj) end

    local nxtnode = 1
    for i = 0,nsec-1 do
        local epos = rdobj:getMiddleEdgePosition(i)
            if i < 10 then
--                lo('?? for_e:'..i..':'..nxtnode..':'..tostring(epos)..':'..tostring(anode[nxtnode].pos))
            end
        if (epos - anode[nxtnode].pos):length() < 0.02 then
--                lo('?? node_HIT:'..nxtnode..':'..i)
            local v
            if nxtnode < #anode then
                v = U.proj2D(anode[nxtnode+1].pos - anode[nxtnode].pos):normalized()
                v = vec3(-v.y,v.x)
                list[#list+1] = rdobj:getLeftEdgePosition(i) - v*margin
                listr[#listr+1] = rdobj:getRightEdgePosition(i) + v*margin
                nxtnode = nxtnode + 1
            else
                -- last node
--                    lo('?? LAST:'..i..'/'..nsec)
                v = (rdobj:getLeftEdgePosition(i) - rdobj:getRightEdgePosition(i)):normalized()
                list[#list+1] = rdobj:getLeftEdgePosition(i) - v*margin
                listr[#listr+1] = rdobj:getRightEdgePosition(i) + v*margin
                break
            end
        end
    end
    if pR then
--        local n2e = node2edge(croad)
--            U.dump(n2e, '?? n2e:'..ie)
        local ipre
        for i,n in pairs(n2e) do
            if ie and n.e > ie then
                ipre = i - 1
                break
            end
        end
        if ipre and n2e and n2e[ipre + 2] then
            for i =1,ipre-1 do
                table.remove(listr, i)
            end
            local d12 = (anode[ipre+1].pos - rdobj:getMiddleEdgePosition(ie)):length() --   n2e[ipre + 1].d - n2e[ipre].d
            local d23 = n2e[ipre + 2].d - n2e[ipre + 1].d
            local en2 = math.ceil((n2e[ipre + 2].e + ie)/2)
--                lo('?? distR:'..tostring(ipre)..':'..n2e[2].e..':'..ie..' 12:'..d12..' 23:'..d23..':'..en2)
            listr[1] = pR
            local vp = (rdobj:getLeftEdgePosition(en2) - rdobj:getRightEdgePosition(en2)):normalized()
            listr[2] = rdobj:getRightEdgePosition(en2) + vp*margin
        end
--            out.apick = {listr[1],listr[2],listr[3],listr[4]}
--        listr[1] = pR
    end
    if pL then
        local ipre
        for i,n in pairs(n2e) do
            if iel and n.e > iel then
                ipre = i - 1
                break
            end
        end
        if ipre then
            for i =1,ipre-1 do
                table.remove(list, i)
            end
            local d12 = (anode[ipre+1].pos - rdobj:getMiddleEdgePosition(iel)):length() --   n2e[ipre + 1].d - n2e[ipre].d
            local d23 = n2e[ipre + 2].d - n2e[ipre + 1].d
            local en2 = math.floor((n2e[ipre + 2].e + iel)/2) - 1
--                lo('?? distL:'..tostring(ipre)..':'..n2e[2].e..':'..iel..' 12:'..d12..' 23:'..d23..':'..en2)
            list[1] = pL
            local vp = (rdobj:getLeftEdgePosition(en2) - rdobj:getRightEdgePosition(en2)):normalized()
            list[2] = rdobj:getLeftEdgePosition(en2) - vp*margin
        end
    end
    local rd = dline[cpick..'_left']
    if rd then
        rd:delete()
        dline[cpick..'_left'] = nil
    end
    rd = dline[cpick..'_right']
    if rd then
        rd:delete()
        dline[cpick..'_right'] = nil
    end

    rd = decalUp({
        list = list,
        w = 0.3,
        mat = 'line_white',
    })
    dline[cpick..'_left'] = rd
    rd.name = 'line_'..cpick..'_'..rd:getID()
    groupLines:add(rd.obj)

    rd = decalUp({
        list = listr,
        w = 0.3,
        mat = 'line_white',
    })
    dline[cpick..'_right'] = rd
    rd.name = 'line_'..cpick..'_'..rd:getID()
    groupLines:add(rd.obj)
end

--        local c = (n2e[ipre + 1].d - d12) + (d12 + d23)/2
--[[
        local d12 = n2e[2].d - n2e[1].d
        local d23 = n2e[3].d - n2e[2].d
        local d2 = (d12 + d23)/2 - d12
]]
--            out.avedit = {p}
--            if true then return end
--        local p =
--        local rde = adec[pp[1].rdi].body
--        local fr1,to1 = 1,20
--        local fr2,to2
--[[
            out.apick = {
                croad:getRightEdgePosition(ifr),
                croad:getRightEdgePosition(ito),
                rdthem:getLeftEdgePosition(jfr),
                rdthem:getLeftEdgePosition(jto),
            }
]]
--[[
    local rightNeigh = ndi == 1 and pp[1] or pp[2]
    local rdr = cdec[rightNeigh.rdi].body
    for i = 0,10 do --nsec-1 do
        local p = croad:getRightEdgePosition(i)
        local w = (rdr:getRightEdgePosition(i) - rdr:getleftEdgePosition(i)):length()
        lo('?? for_p:'..tostring(p))
    end
]]
--            if true then return end
--    forStar()



local function laneSet()
    lo('>> laneSet:'..D.ui.laneL..':'..D.ui.laneR)
    if D.ui.laneL == 0 or D.ui.laneR == 0 then
        local rd = dline[cpick..'_middle']
        if rd then
            rd:delete()
            dline[cpick..'_middle'] = nil
        end
    else
        middleUp()
    end
end


local function matApply(mat, o)
    local rdobj = croad and croad or o
    rdobj:setField("material", 0, mat)
        lo('??***************** dec.matApply:'..mat..':'..tostring(o)..':'..tostring(cpick)..':'..tostring(rdobj))
    if o then
        cpick = o:getID()
    end

    if not aedit[cpick] then
        aedit[cpick] = {body = rdobj, w = rdobj:getNodeWidth(0)}
    end
    -- side marking
--[[
    local rdl = dline[cpick..'_left']
    if rdl then
        rdl:delete()
        dline[cpick..'_left'] = nil
    end
    local rdr = dline[cpick..'_right']
    if rdr then
        rdr:delete()
        dline[cpick..'_right'] = nil
    end
]]
    middleUp(rdobj)
    sideUp(rdobj)
--[[
    -- build nodes
    local list,listr = {},{}
    local anode = editor.getNodes(croad)
    local nsec = croad:getEdgeCount()
    --- left
    local nxtnode = 1
    for i = 0,nsec-1 do
        local epos = croad:getMiddleEdgePosition(i)
            if i < 10 then
--                lo('?? for_e:'..i..':'..nxtnode..':'..tostring(epos)..':'..tostring(anode[nxtnode].pos))
            end
        if (epos - anode[nxtnode].pos):length() < 0.02 then
--                lo('?? node_HIT:'..nxtnode..':'..i)
            if nxtnode < #anode then
                local v = U.proj2D(anode[nxtnode+1].pos - anode[nxtnode].pos):normalized()
                v = vec3(-v.y,v.x)
                list[#list+1] = croad:getLeftEdgePosition(i) - v*0.3
                listr[#listr+1] = croad:getRightEdgePosition(i) + v*0.3
                nxtnode = nxtnode + 1
            else
                break
            end
        end
    end
    dline[cpick..'_left'] = decalUp({
        list = list,
        w = 0.3,
        mat = 'line_white',
    })
    dline[cpick..'_right'] = decalUp({
        list = listr,
        w = 0.3,
        mat = 'line_white',
    })
]]
end


local function update(rd)
    if not rd then rd = croad end
--    rd:setPosition(rd:getPosition())
--    editor.updateRoadVertices(rd)
--    editor_roadUtils.reloadDecals(rd)
--[[
    out.inconform = true
    isfirst = false
    local dpos = croad:getPosition()
    croad:setPosition(dpos)
]]
end


local im = ui_imgui

local cmover


local function forRoad()
    if U._PRD ~= 0 and not U._MODE == 'conf' then return end
--        if cpick then
--            lo('?? forRoad:'..tostring(cpick))
--            cpick = nil
--        end
    if not cpick and #apick == 0 then return nil end

    return croad or #apick --and croad:getField("material", "") ~= 'WarningMaterial'
--    return tableSize(editor.selection.object) == 1 and scenetree.findObjectById(editor.selection.object[1]):getClassName() == "DecalRoad"
--    return croad
end


local function unselect()
        lo('>> unselect:')
    cpick = nil
    croad = nil
    cdesc = nil
    out.apoint = nil
    out.avedit = nil
    out.anode = nil
    out.apicknode = nil
end


--[[
local function forStar(astem, ard)
    U.dump(astem, '>> forStar:')
    if not ard then ard = adec end
        out.avedit = {}
    local aflag = {}
    for _,c in pairs(astem) do
        local lst = ard[c[1] ].list
        local nxt
        if c[2] > 1 then
            local nxt = lst[c[2] - 1]
            local v = nxt - lst[c[2] ]
            aflag[#aflag+1] = {rdi = c[1], fr = c[2], ndi = c[2] - 1, ang = math.atan2(v.y, v.x) % (2*math.pi), v = (nxt - lst[c[2] ]):normalized(), next = nxt}
--                    out.avedit[#out.avedit + 1] = nxt
--                    out.avedit[#out.avedit].z = core_terrain.getTerrainHeight(nxt)
        end
        if c[2] < #lst then
            local nxt = lst[c[2] + 1]
            local v = nxt - lst[c[2] ]
            aflag[#aflag+1] = {rdi = c[1], fr = c[2], ndi = c[2] + 1, ang = math.atan2(v.y, v.x) % (2*math.pi), v = (nxt - lst[c[2] ]):normalized(), next = nxt}
--                    out.avedit[#out.avedit + 1] = nxt
--                    out.avedit[#out.avedit].z = core_terrain.getTerrainHeight(nxt)
        end
    end
    local function comp(a, b)
        return a.ang < b.ang
    end
    table.sort(aflag, comp)
--    aang[#aang + 1] = {i, math.atan2(v.y, v.x) % (2*math.pi)}
        U.dump(aflag, '<< forStar:')
    return aflag
end
]]


local function juncClear(jdesc)
    if not jdesc then jdesc = ajunc[cjunc] end
    for _,d in pairs(jdesc.list) do
        editor.deleteRoad(d.body:getID())
    end
    for _,d in pairs(jdesc.aexit) do
        editor.deleteRoad(d.body:getID())
    end
    for _,d in pairs(jdesc.aline) do
        editor.deleteRoad(d.body:getID())
    end
end


D.ifRound = function(r, abr)
    local wma = 0
    for i,rd in pairs(abr) do
        if rd.w > wma then
            wma = rd.w
        end
    end
--        lo('?? ifR:'..tostring(wma)..'/'..r..':'..(r >= wma*2/3))
--        lo('<< ifRound:'..r..'/'..wma..':'..(wma*3/2))
    return r >= wma*3/2
end


local dval = {}

local function onVal(key, val)
    local jdesc = ajunc[cjunc]
--        lo('?? D.onVal:'..key..':'..tostring(val))
        if not incontrol then lo('?? D.onVal:'..key..':'..tostring(val)..':'..tostring(cjunc)..':'..tostring(incontrol)) end
    if incontrol then
--        indrag = true
    end
    if false then
    elseif key == 'b_road' then
--            U.dump(out.inplot, '?? b_road:'..#out.inplot)
        decalUp({
            w = default.laneWidth*3,
            mat='WarningMaterial',
            list = out.inplot})
        local rd = adec[#adec]
        across[rd.ind] = {}
        across[rd.ind][1] = {}
        across[rd.ind][#rd.list] = {}
        out.inplot = nil
        out.aseg = nil
        out['red'] = nil
        out['blue'] = nil
        out['cyan'] = nil
        out['green'] = nil
        out.acyan = nil
    elseif key == 'b_junction' then
--            lo('?? for_junc:'..cjunc..':'..#ajunc[cjunc].list)
        local abr = ajunc[cjunc].list
        local lane = {} -- {2,2}
        -- prepair branches info
        for i,b in pairs(abr) do
            if b.list[1]:distance(ajunc[cjunc].p) < 0.5 then -- U.small_dist then
                b.io=1
            else
                b.io=-1
            end
                lo('?? for_B:'..i..':'..b.ind..':'..b.io..':'..#b.list..':'..tostring(b.list[1])..':'..tostring(b.list[2])..':'..tostring(ajunc[cjunc].p))
            b.dir = (b.io==1 and (b.list[2]-b.list[1]) or (b.list[#b.list-1]-b.list[#b.list])):normalized()
        end
        local toconf
        --- order by angle
        abr[1].ang = 0
        for i=2,#abr do
            abr[i].ang = U.vang(abr[1].dir, abr[i].dir, true)
        end
        table.sort(abr, function(a,b)
            return a.ang < b.ang
        end)
        local ai = {}
        for _,b in pairs(abr) do
            ai[#ai+1] = abr.ind
        end
        local stamp = U.stamp(ai)

        local rma = 0
        for i,b in pairs(abr) do
--                    road2ter({abr[1].ind},true)
            local w = editor.getNodes(b.body)[1].width
            local nln = math.floor(w/default.laneWidth+0.5)
            lane[1] = math.random(1,nln-1)
                lane[1] = 2
            lane[2] = nln-lane[1]
            b.ij = {cjunc, i} -- indexes in junction
            b.w = w -- default.laneWidth*(lane[1]+lane[2])
            mat = default.mat
            b.lane = {lane[1],lane[2]}
--                U.dump(b.lane,'??__________________________ for_rd_w:'..tostring(w)..':'..nln..':'..math.random(1,nln-1)..':'..tostring(ajunc[cjunc].p)..':'..tostring(b.list[1]))
            b.aexi={}
            b.aexo=b.aexo or {}
            b.aline={}
            --TODO:
            if i == 1 then
--                local stamp = U.stamp({b.ind,b.io==1 and 1 or #b.list},true)
                if not crossset[stamp] then
                    toconf = true
                        lo('?? to_CONF:')
                end
            end
                lo('?? if_CONF:'..i..':'..tostring(toconf))
            if toconf then
                abr[i].mat = (not abr[i].mat or abr[i].mat=='WarningMaterial') and default.mat or abr[i].mat
                road2ter({abr[i].ind}, true) --,false)
            end
            if not abr[i].djrad or tableSize(abr[i].djrad)==0 then
                abr[i].djrad = {}
                abr[i].djrad[1] = abr[i].w/2
                abr[i].djrad[#abr[i].list] = abr[i].w/2
            end
                U.dump(abr[i].djrad, '?? djrad:'..#abr[i].list)
            local jr = abr[i].io == 1 and abr[i].djrad[1] or abr[i].djrad[#abr[i].list] or abr[i].w/2
            if jr > rma then
                rma = jr
            end
--                    if true then break end
--                    b.ne = b.body:getEdgeCount()
--                    b.L = roadLength(b)
        end
--        rma = math.max(rma,default.rexit)
            lo('?? for_JRAD:'..rma)
        ajunc[cjunc].r = rma*default.radcoeff
--            U.dump(dang, '?? dang:')
--                road2ter({abr[1].ind},true)
--                ajunc[cjunc].r = abr[1].io == 1 and abr[1].djrad[1] or abr[1].djrad[#abr[1].list]
        ajunc[cjunc].list = abr
        vert2junc(ajunc[cjunc].list)

        return
    elseif key == 'b_conform' then
        out['white'] = nil
        if cjunc then
            if not ajunc[cjunc] then return end
                lo('?? junc_CONF:'..cjunc..':'..#ajunc[cjunc].list..' r:'..tostring(ajunc[cjunc].r))
            local rma = ajunc[cjunc].r or 0
            if rma == 0 then
                local junc = ajunc[cjunc]
                for i,b in pairs(junc.list) do
                    road2ter({b.ind}, true)
                        U.dump(b.djrad, '??______ for_r:'..b.ind..':'..b.jrad)
                    local ni = b.list[1]:distance(junc.p) < U.small_dist and 1 or #b.list
                    if b.djrad[ni] > rma then
                        rma = b.djrad[ni]
                    end
    --                if b.jrad > rma then
    --                    rma = b.jrad
    --                end
    --                ajunc[cjunc].r = rma
                end
                    lo('?? rma:'..rma)
                ajunc[cjunc].r = rma*default.radcoeff
            end

            if true then
                local list = {}
                for i,b in pairs(ajunc[cjunc].list) do
                    list[#list+1] = b.ind
                end
                road2ter(list)
                be:reloadCollision()
                for i,b in pairs(ajunc[cjunc].list) do
--                    for j,n in pairs(b.list) do
--                        editor.setNodePosition(b.body, j-1, n+vec3(0,0,10))
--                    end
--                    b.body:setPosition(b.body:getPosition())
--                    editor.updateRoadVertices(b.body)
-- update lines
                    local ii = 1
        --- middle
                    local list = line4laneD(b, b.lane[1], U.mod(i+1, ajunc[cjunc].list).w/2, default.matline).list
                    nodesUpdate(b.aline[ii], list)
                    ii = ii + 1
        --- right
                    for k=1,b.lane[1]-1 do
                        list = line4laneD(b, k, U.mod(i+1, ajunc[cjunc].list).w/2, default.matlinedash).list
                        nodesUpdate(b.aline[ii], list)
                        ii = ii + 1
                    end
        --- lanes left
                    for k=1,b.lane[2]-1 do
                        list = line4laneD(b, k+b.lane[1], U.mod(i+1, ajunc[cjunc].list).w/2, default.matlinedash).list
                        nodesUpdate(b.aline[ii], list)
                        ii = ii + 1
                    end
                    b.body:setPosition(b.body:getPosition())
                    editor.updateRoadVertices(b.body)
                end
            end
        else
--[[
                for i,b in pairs(ajunc[cjunc].list) do
    --                editor.updateRoadVertices(b.body)
                    if b.aline then
                        -- update lines
                        for j,l in pairs(b.aline) do
--                            l.body:setPosition(l.body:getPosition())
                            editor.updateRoadVertices(l.body)
                        end
                    end
                end
]]
--                lo('?? if_EX:'..tostring())
            road2ter(nil) --,nil,true)
        end
        return
    elseif key == 'conf_all' then
        out.inall = not out.inall
        unselect()
        return
    elseif ({conf_jwidth=0,conf_margin=0,conf_mslope=0,conf_bank=0})[key] then
        dval[key] = val
    elseif key == 'round_w' then
--        if incontrol then return end
        jdesc.dirty = true
        if not jdesc.round then jdesc.round = {} end
        jdesc.round.w = val
--            lo('?? jdesc_w:'..tostring(jdesc.round.w))
--        junctionRound(ajunc[cjunc].round.r, val)
    elseif key == 'round_r' and jdesc.round then
--        if incontrol then return end
--            lo('?? onVal.round_r:'..val)
        if not D.ifRound(val, jdesc.list) then return end
        jdesc.dirty = true
        jdesc.round.r = val
        jdesc.r = val
--        junctionRound(val, ajunc[cjunc].round.w)
    elseif key == 'exit_r' then
--        jdesc.rexit = val
        jdesc.r = val
        for _,d in pairs(jdesc.list) do
            d.dirty = true
            for k,e in pairs(d.aexo) do
                if adec[e] then
                    adec[e].da = val
                    adec[e].db = val
                end
            end
        end
        --TODO: reconform
--        for _,d in pairs(jdesc.aexit) do
--            d.dirty = true
--        end
    elseif key == 'branch_n' then
            lo('?? branch_n:'..val..':'..tostring(cjunc)..':'..tostring(indrag))
        local p = ajunc[cjunc].p
        local isround = ajunc[cjunc].round
        junctionDown(cjunc)
        junctionUp(p, val)
        if isround then
            junctionRound()
        end
    elseif key == 'exit_w' then
        for _,d in pairs(jdesc.list) do
            d.dirty = true
            for k,e in pairs(d.aexo) do
                if adec[e] then
                    adec[e].w = val
                end
            end
        end
--[[
        jdesc.wexit = val
        for _,d in pairs(jdesc.aexit) do
            d.dirty = true
        end
]]
    elseif key == 'junction_round' then
            lo('?? onVal_junction_round:'..tostring(jdesc.r))
--        if not D.ifRound(jdesc.r, jdesc.list) then return end
        jdesc.r = jdesc.r - default.laneWidth
        junctionRound()
        key = nil
    end
--    junctionUpdate(jdesc)
    incontrol = key
--    indrag = true
end
D.onVal = onVal


local function inView()
    return not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not im.IsAnyItemHovered()
end


-- input = require("/lua/vehicle/input")

local function toTarget(ang, throttle) --cpos, cvel)
--        lo('?? toTarget:'..tostring(ang)..':'..tostring(throttle))
    if not veh then return end
    local br = 0
    if not ang then ang = 0 end
    if not throttle then throttle = car.gaz end
    if not throttle then throttle = 0.1 end
    car.gaz = throttle
    if throttle < 0 then
        br = -throttle
        throttle = 0
    end
--            if throttle <= 0 then
--                lo('?? toTarg:'..throttle..':'..br)
--            end
--    if true then return end
--        lo('>> toTarget:'..tostring(car.target)..':'..tostring(ang))
--    veh:queueLuaCommand('ai.driveToTarget("'..car.target.x..'_'..car.target.y..'_'..car.pos.x..'_'..car.pos.y..'_'..car.vel.y..'_'..car.vel.y..'",0.1,0,0.2,"1")')

--[[
    input.event("steering", steering, "FILTER_AI", nil, nil, nil, "ai")
    input.event("throttle", throttle, "FILTER_AI", nil, nil, nil, "ai")
    input.event("brake", brake, "FILTER_AI", nil, nil, nil, "ai")
    input.event("parkingbrake", parkingbrake, "FILTER_AI", nil, nil, nil, "ai")
]]

--    veh:queueLuaCommand('ai.driveCar('..ang..','..throttle..','..br..',0,1)')
end
D.toTarget = toTarget

toTarget(0, 0)

-- position of point at edge ind on side lane
local function onLane(ie, rd, side, ilane)
    local er,el = epos(rd.body, ie, 'right'), epos(rd.body, ie, 'left')
    local margin = er:distance(el)/(rd.body.lanesLeft + rd.body.lanesRight)*(ilane - 1/2)
--        lo('?? onLane_margin:'..margin..':'..side..':'..ilane)
--        local tgt = side == 'right' and (er+(el-er):normalized()*margin) or (el+(er-el):normalized()*margin)
--        out.avedit[#out.avedit+1] = el
--        out.avedit[#out.avedit+1] = tgt
--        margin = 0

    return side == 'right' and (er+(el-er):normalized()*margin) or (el+(er-el):normalized()*margin)
end


local function forBranch(rd, side)
    if not rd then return end
        lo('?? forBranch:'..tostring(rd.ij and rd.ij[2] or nil)..' idec:'..rd.ind..':'..tostring(side)) --..':'..ajunc[rd.ij[1]].list[rd.ij[2]].ind..':'..tostring(rd.dir))
    if not side then side = 'middle' end
    -- in junction, next branch
    local ama,imi = math.huge
    if side == 'right' then
        ama = -ama
    end
    if not rd.ij then return end
    for i,b in pairs(ajunc[rd.ij[1]].list) do
        local cang = U.vang(b.dir, -rd.dir, true) --b.dir:dot(-rd.dir)
        if i ~= rd.ij[2] then
                lo('?? if_nbr:'..i..':'..cang..':'..ama)
            local hit
            if side == 'middle' and math.abs(cang) < ama then
                hit = true
            elseif side == 'right' and cang > ama then
                hit = true
            elseif side == 'left' and cang < ama then
                hit = true
            end
            if hit then
                ama = cang
                imi = i
            end
        end
    end
    if imi then
        local rdnext = ajunc[rd.ij[1]].list[imi]
            lo('??++++++++++++++++++ to_next:'..rdnext.ind..':'..ama)
        car.target.exit = {
            p=onLane(3, rdnext, 'right', 1), --rdnext.list[1],
            idec=rdnext.ind}
                            U.dump(car.target.exit, '?? targ_update_ex:')
--                            car.target.rd = ajunc[rd.ij[1]].list[imi]
--                            car.target.side = 'right'
--                            car.target.p = car.target.rd.list[1]
    end
        lo('<< forBranch:')
end


local function toCross(side)

end


local function toExit(side)
        lo('>> toExit:'..tostring(car.target)..':'..tostring(car.target and car.target.rd or nil))
    if not car.target then return end
--        U.dump(car.target, '>> toExit:')
    local rd = car.target.rd
        if rd then
            lo('?? toEx:'..rd.ind)
        end
    if not rd or not rd.aexo then return end
        lo('?? toExit:'..tostring(car.target)..':'..tostring(#rd.aexo))
--        U.dump(rd, '>> toExit:')
    local dmi,pmi,imi = math.huge
    for i,idec in pairs(rd.aexo) do
        local e = adec[idec]
        if car.pos:distance(e.list[1]) < dmi then
            dmi = car.pos:distance(e.list[1])
--            pmi = e.list[1]
            imi = e.ind
        end
--[[
        if (car.target.p-car.pos):dot(e.list[1]-car.pos)>0 then
            if car.pos:distance(e.list[1]) < dmi then
                dmi = car.pos:distance(e.list[1])
                pmi = e.list[1]
                imi = e.ind
            end
        end
]]
    end
    if dmi then
        local ie = p2e(car.pos, adec[imi])
        if ie then
            car.target.exit = {p = epos(adec[imi].body, ie+1, 'right'), idec = imi, side='right'}
            return true
        end
--        car.target.exit = {p = pmi, idec = imi}
--                out.avedit = {pmi}
    end
        U.dump(car.target.exit,'<< toExit:'..tostring(dmi)..':'..#rd.aexo)
    return false
--        U.dump(rd.aexo[1],'?? aex:'..#rd.aexo)
end


-- edge at dist from ie
local function fromEdge(rd, ie, side, dist)
    local ito,step = rd.body:getEdgeCount()-1, 1
    if side == 'left' then
        ito,step = 0, -1
    end
    local d,pp = 0,epos(rd.body, ie, side)
--        lo('?? fromEdge:'..dist..'/'..epos(rd.body, ito, side):distance(car.pos)..':'..ie..'>'..ito..':'..step..':'..side)
    for i = ie+step,ito,step do
        local p = epos(rd.body, i, side)
        d = d + p:distance(pp)
--            lo('?? for_e_d:'..i..':'..d)
        if d>dist then
            return i
        end
        pp = p
    end
end


local function toLane(dir)
    -- get distance to end
    local ito = car.target.side == 'left' and 0 or car.target.rd.body:getEdgeCount()-1
    local pend = epos(car.target.rd.body, ito, car.target.side)
    local toend = pend:distance(car.pos)/car.vel:length()
    local newlane = car.target.lane+dir
        lo('?? toLane:'..tostring(toend)..':'..tostring(newlane)..':'..car.target.rd.ind)
    if toend > default.react_time then
        local ie = p2e(car.pos, car.target.rd)
        ie = fromEdge(car.target.rd, ie, car.target.side, default.react_time*car.vel:length())
            lo('?? ie:'..tostring(ie))
        if ie then
            local p = onLane(ie, car.target.rd, car.target.side, newlane)
            car.target.exit = {p=p, idec=car.target.rd.ind, ie=ie, lane=newlane, side=car.target.side}
--            car.target.p = p
--            car.target.lane = newlane
                if p then out.avedit = {p} end
        end
    end
        lo('<< toLane:')
end


D.matchClear = function()
    for i,m in pairs(amatch) do
        m.skip = nil
    end
end


D.inMatch = function(ind)
    for i,m in pairs(amatch) do
        if m.ind == ind then
            return m,i
        end
    end
end


local function forTarget()
    local dircar = veh:getDirectionVector()
        lo('>>^^^^^^^^^^ forTarget:'..tostring(dircar)..':'..tostring(car.target and car.target.p)..':'..tostring(car.target and car.target.rd and car.target.rd.ind..'_'..car.target.side or nil))
    local dmi,ijmi = math.huge
    out.avedit = {}

    if not car.target then
            lo('?? pick_TARGET:')
        for i,rd in pairs(adec) do
--            if onroad and onroad ~= rd.id then
--                goto continue
--            end
--                lo('?? for_rd:'..tostring(rd.body)..':'..tostring(rd.ind)..':'..#rd.list)
            if rd and rd.ij and rd.body and rd.body:getNodeWidth(0) > 1.5 then -- and rd.body:containsPoint(car.pos) then
    --                lo('?? onROAD:'..rd.id)
                for k,n in pairs(rd.list) do
                    if dircar:dot(n-car.pos) > 0 and n:distance(car.pos) >= default.v2tmin then
                        if car.pos:distance(n) < dmi then
                            dmi = car.pos:distance(n)
    --                        lo('?? forTarget_DMI:'..i..':'..k..' dmi:'..dmi)
                            ijmi = {i,k}
                        end
                    end
                end
            end
    --                lo('?? for_rd:'..i..':'..tostring(dmi))
            ::continue::
        end
        if ijmi then
                U.dump(ijmi, '?? cand_found:'..tostring(dmi)..':'..adec[ijmi[1]].id..':'..tostring(car.pos))
                local icheck = 43
                if false and ijmi[1] ~= icheck then
                        lo('?? wrongWay:'..tostring(car.pos))
                    for i,p in pairs(adec[ijmi[1] ].list) do
                        lo('?? cdist:'..ijmi[1]..':'..car.pos:distance(p))
                    end
                    for i,p in pairs(adec[icheck].list) do
                        lo('?? dist_check:'..icheck..':'..car.pos:distance(p)..':'..tostring(car.pos)..':'..tostring(p))
                    end
                        out.avedit = {adec[ijmi[1] ].list[ijmi[2] ]}
                    return
                end
            local rd = adec[ijmi[1]]
--                out.ared = {rd.list[1]}
--                out.agreen = {rd.list[#rd.list]}
            -- get side
            local ie = p2e(car.pos, rd)
            local side = car.pos:distance(epos(rd.body,ie,'left')) < car.pos:distance(epos(rd.body,ie,'left')) and 'left' or 'right'                 lo('?? at_EDGE:'..tostring(ie))
                lo('?? at_EDGE:'..tostring(ie)..':'..side)
--                if true then return end
            local obj = adec[ijmi[1]].body
            local margin = obj:getNodeWidth(ijmi[2]-1)/(obj.lanesLeft+obj.lanesRight)/2
            -- set target
            local ndi = edge4node(adec[ijmi[1]], ijmi[2])
            local dirroad = epos(obj,ndi == 0 and 1 or ndi) - epos(obj, ndi == 0 and 0 or (ndi-1))
            local ito,step = obj:getEdgeCount()-1,1
            local dir = dirroad:dot(dircar) > 0 and 1 or -1
                lo('?? for_dir:'..tostring(dirroad)..':'..tostring(dircar)..':'..dir)
            -- get lane
            local s = toSide(car.pos,rd,dir>0 and 'right' or 'left')
            car.lane = math.floor(s/2/margin+1)
                lo('??+++++++ for_LANE:'..car.lane..' s:'..s..' marg:'..margin)
            if dir == -1 then
                ito,step = 0,-1
            end
            for k=ndi,ito,step do
                local er,el,target = epos(obj, k, 'right'),epos(obj, k, 'left')
                target = onLane(k, rd, dir>0 and 'right' or 'left', car.lane)
--                out.avedit = {target}
--                    if true then return end
--[[
                if dir > 0 then
                    target = er + (el-er):normalized()*margin
                else
                    target = el + (er-el):normalized()*margin
                end
]]
                local ang = (target-car.pos):normalized():dot((target-(car.pos+dircar*1)):normalized())
    --                lo('?? for_ang:'..k..':'..ang)
                if ang > 0.995 then -- math.sqrt(2)/2 then
    --            if dircar:dot((target-(car.pos+dircar*1)):normalized()) > math.sqrt(2)/2 then
                        out.agreen = {target}
    --                    out.avedit = {adec[ijmi[1]].list[ijmi[2]], target}
                        U.dump(ijmi, '?? found:'..tostring(target)..':'..adec[ijmi[1]].id..':'..dir)
                    car.target = {
                        p=target,
                        rd=rd,
                        ie=k, -- target edge index
                        side=dir>0 and 'right' or 'left',
                        lane=car.lane or 1}
                    break
                end
    --                lo('?? for_next:'..k..':'..ang) --dircar:dot((target-car.pos):normalized()))
            end
        end
    end

    local target = car.target
    local onroad
--        lo('?? forTarget_if_command:'..#incommand..':'..tostring(car.target)) --.exit))
--            lo('?? if_T0:'..tostring(car.target)..':'..tostring(car.target.rd))
--    if car.target and not car.target.exit then
    if car.target and car.target.rd then
--            lo('?? if_T1:')
        local rd = car.target.rd
--        onroad = rd.id
        local ito,step = rd.body:getEdgeCount()-1,1
        if car.target.side == 'left' then
            ito,step = 0,-1
        end
        if car.target.ie == ito then
                lo('??___________ END_OF:'..rd.ind..' at:'..ito..':'..car.target.side..' b2b:'..tostring(rd.ij)..' b2l:'..tostring(rd.link)..' e2b:'..tostring(rd.j))
--                lo('?? to_search:')
            target = nil
            car.target.isend = true
--                    car.target.rd = nil
            if rd.ij and car.target.ie==0 and not ajunc[cjunc].round then
                -- find junction branch
                forBranch(rd)
--                    U.dump(car.target, '')
                target = car.target
            elseif rd.frto then
--            elseif rd.j then
                -- from exit to branch, e2b
                car.target.rd = adec[rd.frto[2]]
                car.target.ie = p2e(car.target.p,car.target.rd)
--                car.target.rd = ajunc[rd.j].list[rd.frto[2]]
--                    out.ared = {car.target.rd.list[1]}
--                    out.agreen = {car.target.rd.list[#car.target.rd.list]}
--                    U.dump(rd.frto,'?? E2B:'..tostring(rd.j)..':'..tostring(car.target.rd)..' e:'..tostring(car.target.ie))
--                    if true then return end
                car.target.p = onLane(car.target.ie+1, car.target.rd, 'right', 1)
--                car.target.p = onLane(rd.iin, car.target.rd, 'right', 1)
--                car.target.ie = rd.iin
--                    out.avedit = {car.target.p}
--                    if true then return end
                target = car.target
            elseif rd.link then
                    lo('?? to_link:')
                -- from branch to link
                car.target.rd = rd.link
                --??
                car.target.ie = p2e(car.target.p,rd.link)
--                    lo('?? ')

                local lb,le = rd.link.list[1],rd.link.list[#rd.link.list]
--                    out.avedit = {lb,le}
--                    lo('?? lble:'..tostring(lb)..':'..tostring(le))
                local nodei = 2
                local rev = false
                if rd.list[#rd.list]:distance(lb) > rd.list[#rd.list]:distance(le) then
                    rev = true
                    nodei = #rd.link.list - 1
                end
--                    if true then return end
                local ie = edge4node(rd.link, nodei) + (rev and 3 or - 3)
--                    lo('?? nodei:'..tostring(nodei)..':'..ie)

                car.target.side = rev and 'left' or 'right'
                car.target.ie = ie
                car.target.p = onLane(ie, rd.link, car.target.side, 1)
--                    lo('?? for_link:'..tostring(rd.link.ind)..':'..nodei..':'..ie)
--                    out.acyan = {car.target.p}
--                    out.avedit = {car.target.p}
--                    if true then return end
                target = car.target
--                car.target.ie
            elseif rd.to then
                -- from link to branch
                if car.target.ie == rd.body:getEdgeCount() - 1 then
                    car.target.rd = rd.to
                else
                    car.target.rd = rd.fr
                end
                car.target.side = 'left'
                    lo('?? for_BR:'..car.target.ie..'/'..rd.body:getEdgeCount())
                car.target.ie = car.target.rd.body:getEdgeCount() - 8
                car.target.p = onLane(car.target.ie, car.target.rd, 'left', 1)
--                    out.green = {car.target.p}
                    lo('?? ro_BRANCH:'..car.target.rd.ind)
                target = car.target
            end
            if not target then
                -- generate
                if false then
                        lo('?? to_GEN:'..tostring(car.vel))
    --                local ang = U.rand(-math.pi/6, math.pi/6)
                    local r = U.rand(100, 400)
                    junctionUp(car.pos + r*U.vturn(car.vel:normalized(), U.rand(-math.pi/6, math.pi/6)), math.random(3,5)) --*vec3(math.cos(ang),math.sin(ang)), math.random(3,5))
                end
--                car.target = nil
            end
--                U.dump('?? no_TARGET:')
--            return
        elseif car.target.lane then
                lo('?? target_LANE:'..car.target.lane)
            local dist = 0
            local pt,pi=car.target.p,car.target.ie
            local borderL, borderR = {},{}
            local curvma = 0
--                        lo('?? if_T2:'..car.target.ie+step..':'..ito..':'..step)
            for i=car.target.ie+step,ito,step do
                local cp = onLane(i, rd, car.target.side, car.target.lane)
--                    out.avedit[#out.avedit+1] = cp
--                    out.avedit[#out.avedit+1] = epos(rd.body, i, 'left')
--                    if true then return end
                -- check being in borders
                local isin = true
                for k=1,#borderL  do
                    if U.toLine(borderL[k], {car.pos, cp}) < 1. or U.toLine(borderR[k], {car.pos, cp}) < 1. then
                        isin = false
                        out.ared = {cp}
                        break
                    end
                end
                if not isin then
                    break
                end
                borderL[#borderL+1] = onLane(i, rd, car.target.side, car.target.lane+1/2)
                borderR[#borderR+1] = onLane(i, rd, car.target.side, car.target.lane-1/2)
--                    out.avedit[#out.avedit+1] = borderL[#borderL]
--                    out.avedit[#out.avedit+1] = borderR[#borderR]
                -- eval curvarture
                if math.abs(i-ito) > 1 then
                    local p1,p2,p3 = epos(rd.body, i),epos(rd.body, i+1),epos(rd.body, i+2)
                    local cc = U.vang(p3-p1,p2-p1)/p1:distance(p3)
                    if cc > curvma then
                        curvma = cc
                    end
                end
                pt = cp
                pi = i
            end
--                lo('?? newtag:'..tostring(pt)..':'..pi..':'..car.target.side)
--                    out.avedit = borderL
                    out.avedit[#out.avedit+1] = pt
            if pt then
                car.target.p = pt
                car.target.ie = pi
                car.target.curv = curvma
--                    lo('?? CC:'..pi..':'..tostring(curvma)) --..'/'..car.vel:length()..':'..pi..'/'..ito)
                if pi == ito then
                        lo('?? last_EDGE:'..rd.ind..':'..pi)
                    if car.vel and rd.ij and pi ~= 0 and not rd.link then
--                            if true then return end
        --                local ang = U.rand(-math.pi/6, math.pi/6)
                        D.inMatch(rd.ind).skip = nil
                        local r = U.rand(250, 350)
                            lo('?? to_GEN:'..tostring(car.vel)..':'..r..':'..#amatch)
                        local jdesc = junctionUp(car.target.p + r*U.vturn(car.vel:normalized(), U.rand(-math.pi/6, math.pi/6)), math.random(3,5)) --*vec3(math.cos(ang),math.sin(ang)), math.random(3,5))
--                            lo('?? post_GEN:'..#amatch)
--                            U.dump(amatch, '?? AMATCH:')
--                            if true then return end

                        local ami,imi = math.huge
                        for i,b in pairs(jdesc.list) do
                            if U.vang(car.vel, b.dir) < ami then
                                ami = U.vang(car.vel, b.dir)
                                imi = i
                            end
                        end
                        if imi then
                            amatch[#amatch-#jdesc.list+imi].skip = true
                        end
--                        amatch[#amatch-1].skip = true
--                        amatch[#amatch].skip = true
--                        amatch[#amatch-2].skip = true
                            out.acyan = {adec[amatch[#amatch-3].ind].list[#adec[amatch[#amatch-3].ind].list]}
                        forPlot(true)
                    end
                end

--[[
                if pi == ito then
                        lo('??********************** END_OF:'..rd.ind..':'..pi)
                    car.target.isend = true
--                    car.target.rd = nil
                    if rd.ij and pi==0 then
                        forBranch(rd)
                    end
                end
]]
--                    lo('<< forTraget:'..tostring(car.target.p)..':'..car.target.rd.ind)
                target = car.target
            end
--                lo('<< forTraget3:')
--            return
        end
--            lo('<< forTraget3:')
--        return
    end
        lo('<< forTarget:'..tostring(target and target.rd and target.rd.ind..':'..target.side or nil)..':'..tostring(target and target.p or nil))
    if true then return target end

--[[
]]
    if false then
        for i,rd in pairs(adec) do
            if onroad and onroad ~= rd.id then
                goto continue
            end
            if rd.body:getNodeWidth(0) > 1.5 then -- and rd.body:containsPoint(car.pos) then
    --                lo('?? onROAD:'..rd.id)
                for k,n in pairs(rd.list) do
                    if dircar:dot(n-car.pos) > 0 and n:distance(car.pos) >= default.v2tmin then
                        if car.pos:distance(n) < dmi then
                            dmi = car.pos:distance(n)
    --                        lo('?? forTarget_DMI:'..i..':'..k..' dmi:'..dmi)
                            ijmi = {i,k}
                        end
                    end
                end
            end
    --                lo('?? for_rd:'..i..':'..tostring(dmi))
            ::continue::
        end
        if ijmi then
                U.dump(ijmi, '?? cand_found:'..tostring(dmi)..':'..adec[ijmi[1]].id..':'..tostring(car.pos))
                local icheck = 43
                if false and ijmi[1] ~= icheck then
                        lo('?? wrongWay:'..tostring(car.pos))
                    for i,p in pairs(adec[ijmi[1] ].list) do
                        lo('?? cdist:'..ijmi[1]..':'..car.pos:distance(p))
                    end
                    for i,p in pairs(adec[icheck].list) do
                        lo('?? dist_check:'..icheck..':'..car.pos:distance(p)..':'..tostring(car.pos)..':'..tostring(p))
                    end
                    out.avedit = {adec[ijmi[1] ].list[ijmi[2] ]}
                    return
                end
            -- get lane
            local obj = adec[ijmi[1]].body
            local margin = obj:getNodeWidth(ijmi[2]-1)/(obj.lanesLeft+obj.lanesRight)/2
            local ndi = edge4node(adec[ijmi[1]], ijmi[2])
            local dirroad = epos(obj,ndi == 0 and 1 or ndi) - epos(obj, ndi == 0 and 0 or (ndi-1))
            local ito,step = obj:getEdgeCount()-1,1
            local dir = dirroad:dot(dircar) > 0 and 1 or -1
            if dir == -1 then
                ito,step = 0,-1
            end
            for k=ndi,ito,step do
                local er,el,target = epos(obj, k, 'right'),epos(obj, k, 'left')
                if dir > 0 then
                    target = er + (el-er):normalized()*margin
                else
                    target = el + (er-el):normalized()*margin
                end
                local ang = (target-car.pos):normalized():dot((target-(car.pos+dircar*1)):normalized())
    --                lo('?? for_ang:'..k..':'..ang)
                if ang > 0.995 then -- math.sqrt(2)/2 then
    --            if dircar:dot((target-(car.pos+dircar*1)):normalized()) > math.sqrt(2)/2 then
                        out.avedit = {target}
    --                    out.avedit = {adec[ijmi[1]].list[ijmi[2]], target}
                        U.dump(ijmi, '?? found:'..tostring(target)..':'..adec[ijmi[1]].id..':'..dir)
                    return {
                        p=target,
                        rd=adec[ijmi[1]],
                        ie=k,
                        side=dir>0 and 'right' or 'left',
                        lane=1}
                end
    --                lo('?? for_next:'..k..':'..ang) --dircar:dot((target-car.pos):normalized()))
            end
        end
    end
end
--[[
                        if false then
                            lo('?? if_injunc:'..rd.ind..':'..ajunc[rd.ij[1] ].list[rd.ij[2] ].ind..':'..tostring(rd.dir))
                            -- in junction, next branch
                            local ama,imi = -math.huge
                            for i,b in pairs(ajunc[rd.ij[1] ].list) do
    --                                lo('?? if_nbr:'..i..':'..)
                                local cang = b.dir:dot(-rd.dir)
                                if i ~= rd.ij[2] and cang > ama then
                                    ama = cang
                                    imi = i
                                end
                            end
                            if imi then
                                local rdnext = ajunc[rd.ij[1] ].list[imi]
                                    lo('??++++++++++++++++++ to_next:'..rdnext.ind..':'..ama)
                                car.target.exit = {
                                    p=onLane(1, rdnext, 'right', 1), --rdnext.list[1],
                                    idec=rdnext.ind}
    --                            U.dump(car.target, '?? targ_update:')
    --                            car.target.rd = ajunc[rd.ij[1] ].list[imi]
    --                            car.target.side = 'right'
    --                            car.target.p = car.target.rd.list[1]
                            end
                        end
]]
--        local er,el = epos(obj, ndi, 'right'),epos(obj, ndi, 'left')
--[[
        local target
        if true then
            if dirroad:dot(adec[ijmi[1] ].list[ijmi[2] ]-car.pos) > 0 then
                -- goto right lane
                else
                -- goto left lane
                return el + (er-el):normalized()*margin
            end
--            car.gaz = 0.1
        end

--            local target =
            lo('?? for_LANES:'..tostring(obj.lanesLeft)..':'..obj:getNodeWidth(ijmi[2]-1)..':'..tostring(ndi))
            out.avedit = {adec[ijmi[1] ].list[ijmi[2] ], car.target}
]]


local function decalSplit(rd, p, ext)
    if not rd then return end
    p = U.proj2D(p)
    local e2d = edge2dist(rd)
    if not e2d then
        lo('!! ERR_decalSplit_NOE2D:'..tostring(rd))
        return
    end
    local d,ps,ifr = toSide(p, rd)
--        toMark({ps, epos(rd.body,ifr)},'red')
    -- splitting point and distance
    ps = U.proj2D(ps)
--        U.dump(e2d, '?? for_e2:'..ifr..':'..tostring(ps)..':'..tostring(epos(rd.body,ifr)))
    local dsplit = e2d[ifr] + ps:distance(epos(rd.body,ifr))
    local pinstep = 30

    -- build 1st
    local function cut(rd, dfr,dto)
        local dspl = dto-dfr
        local nstep = math.floor(dspl/pinstep+0.5)
        local step = dspl/nstep
--            lo('?? decalSplit:'..rd.ind..':'..dfr..':'..dto..':'..tostring(ifr)..' dspl:'..dspl..':'..nstep..'/'..step..' L:'..rd.L..':'..e2d[#e2d])
        local list = {}
        for i=0,nstep do
            local d = dfr + i*step
--        for d=dfr,dto,step do
--        for d=dfr,dfr+nstep*step,step do
            local p = onSide(rd, 'middle', d)
            if p then
                list[#list+1] = p
            end
        end
        return {list=list,w=default.laneWidth*3,mat='WarningMaterial'}
    end

    local pair = {}
    local desc
    desc = cut(rd, 0, dsplit)
    decalUp(desc, ext)
    desc.list[#desc.list] = ps
    pair[1] = desc
--        if desc.body then
--            toMark(desc.list,'green')
--        end
--        local ps = onSide(rd, 'middle', rd.L, 0, true)
--        toMark({ps,rd.list[#rd.list]}, 'red', nil, 0.2, 0.6)
    desc = cut(rd, dsplit, rd.L)
    decalUp(desc, ext)
    desc.list[1] = ps
    pair[2] = desc
--        toMark(desc.list,'green',nil,0.1,0.6)

    local obj = scenetree.findObjectById(rd.id)
    obj:setField('hidden', 0, 'true')
    askip[#askip+1] = rd.id
--    rd.skip = true

--    adec[rd.ind] = nil
--    if obj then obj:delete() end
    return pair
end
D.decalSplit = decalSplit

local fout

local ap = {}
local function forCirc(c, r, step, f, dbg)
--        dbg = true
    local istep = math.max(math.floor(step/grid),1)
    local ir = math.ceil(r/grid)
    local jc,ic = p2grid(c)

--        lo('?? forCirc:'..tostring(c)..' r:'..r..':'..ir..':'..istep) --..':'..tostring(grid2p({ic,jc})))
--        toMark({grid2p({ic,jc})}, 'cyan', nil, 0.1, 0.6)
    for i = ic-ir,ic+ir,istep do
--            if dbg then lo('?? for_I:'..i..':'..ic..':'..ir..':'..istep) end
        for j = jc-ir,jc+ir,istep do
--                fout:write(tostring('fC:'..i..':'..j..':'..ir..'\n'))
            local p = grid2p({i,j})
--                if dbg then lo('?? forCirc:'..i..':'..j..':'..tostring(p)..'/'..tostring(c)) end
            if p:distance(c) < r then
                if f then f(p) end
                ap[#ap+1] = p
            end
        end
    end
--        U.dump(ap, '?? for_AP:'..#ap..':'..tostring(ap[3]))
        if dbg then toMark(ap, 'white', nil, 0.08, 0.8) end
end


local function curvH(a,b,c)
    return U.curv(vec3(0,forZ(a)), vec3(b:distance(a),forZ(b)), vec3(c:distance(b),forZ(c)))
end


local function geomEval(ac, list, i)
    local d1,d2 = list[i]:distance(list[i-1]), list[i]:distance(list[i+1])
    local s =
        ac[1]*(d1 + d2) +
        ac[2]*curvH(list[i-1],list[i],list[i+1]) +
        ac[3]*U.curv(list[i-1],list[i],list[i+1])

    return s
end



local function forAC()
    if not nwlib then return end

    return nwlib.forWeight()
end

--[[
local ta = {1,2}
ta[#ta+1] = 11
--ta[1] = nil
table.remove(ta, 1)
print('??*********** tta:'..#ta..':'..ta[2])
]]

local DMI_SEG = 30 -- minimal distance between segments

local function decalPlot(pth, w, ard, ext, dbg)
        lo('>> decalPlot:'..#pth..' w:'..tostring(w)..':'..tostring(ext))
        ap = {}
        if not pth then pth = {
--vec3(-61.38508606,-169.9675598,0),
--vec3(-9.894002914,-419.5056152,0)
vec3(-187.3,-1763.5),
vec3(-137,-1692.7),
            }
                toMark(pth, 'mag', nil, 0.2)
        end
--        pth = {vec3(1886.662842,-1487.655273),vec3(1916.427979,-1421.535034)}
--        pth = {vec3(), vec3()}
--        pth = {vec3(1702.53,-1436.44), vec3(1660.6,-1344.61)}
--        pth = {vec3(1549.39,-1408.29), vec3(1543.12,-1313.23)}
--        pth[1] = pth[1] + vec3(0,0,forZ(pth[1]))
--        pth[2] = pth[2] + vec3(0,0,forZ(pth[2]))
--        out.ared = {list[1]}
--        out.agreen = {list[2]}
--        toMark({pth[1]},'blue', nil, 0.1)
--        toMark({pth[2]},'green', nil, 0.1)
--        out.inplot = pth
    local ac = forAC()
        ac = {0,1,0}
    -- project to pin-points
    local pinstep = 32
--    local rlook = 10
--    local pfr,pto = U.proj2D(pth[1]),U.proj2D(pth[2])
--    local nstep = math.floor(pfr:distance(pto)/pinstep+0.5)
--        U.dump(pth, '>> decalPlot:'..#pth..':'..pfr:distance(pto))

        local ap = {pth[1]}

--        local fout = io.open('./out.txt', "w")
--        fout = io.open(editor.getLevelPath()..'./out.txt', "w")
    local list = {} -- {{e={pfr,pto},done=false}}
    for i=2,#pth do
        list[#list+1] = {e={U.proj2D(pth[i-1]),U.proj2D(pth[i])},done=false}
    end

    local function ifValid(pt, i)
        local ama = math.pi*0.76
        local valid = true
        if pt:distance(list[i].e[1]) < pinstep/2 then
            valid = false
        end
        if pt:distance(list[i].e[2]) < pinstep/2 then
            valid = false
        end
        -- check ends angles
        if i > 1 then
            local ang = U.vang(pt-list[i].e[1], list[i-1].e[1]-list[i-1].e[2])
            if ang < ama then
                valid = false
            end
        end
        if i < #list then
            local ang = U.vang(pt-list[i].e[2], list[i+1].e[2]-list[i+1].e[1])
            if ang < ama then
                valid = false
            end
        end
        if ard then
            for _,d in pairs(ard) do
                local pp = U.toPoly(pt, d.list)
                if pp and pp:distance(pt) < DMI_SEG then
                    return false
                end
            end
        end

        return valid
    end
--        if dbg then return end
        local n = 0
    local gradstep = 5 --5
    local cseek = 0.82 -- rate of semi-distance between ends
    local done = false
    while not done and #list < 30 and n<100 do
        done = true
        for i,d in pairs(list) do
--                fout:write(tostring(n..':'..i..'\n'))
--                lo('??_________ inE_LIST:'..i..'/'..#list..':'..tostring(d.done))
            if not d.done then
                done = false
                local a,b = d.e[1],d.e[2]

                    local ps
                local cmi,smi,pmi = math.huge,math.huge
                local dist = a:distance(b)
--                    fout:write(tostring('o2:'..n..':'..i..':'..(dist/6)..':'..grid..':'..tostring(a)..':'..tostring(b)..'\n'))
                forCirc((a+b)/2, dist*1/2*cseek, dist/6, function(p)
            --            lo('?? if_CURVE:'..tostring(p)..':'..tostring(forZ(p))..':'..p:distance(pfr)..':'..pto:distance(p))
--                        fout:write(tostring('oc:'..n..':'..i..':'..tostring(p)..'\n'))
                    local crv = geomEval(ac, {a, p, b}, 2)
--                    local crv = math.abs(curvH(a, p, b))
                    if crv<cmi and ifValid(p, i) then
                        cmi = crv
                        pmi = p
                    end
--                    if crv<cmi and ifValid(p, i) then
--                    end
                end) --, i == 2 and #list == 2) --, true)--, n==4)
--                    fout:write(tostring('o3:'..n..':'..i..'\n'))

--                    lo('?? if_pmi:'..tostring(pmi))
--                        if i == 2 and #list == 2 then
--                            lo('?? if_PMI:'..tostring(pmi))
--                        end
--                    if dbg then return end

                if pmi then
                    ps = pmi
--                        local ap = {}
                    -- gradient minimize
                    --- shift direction
    --                local u = ((a-pmi):normalized()+(b-pmi):normalized()):normalized()
                    local dc = -U.small_val
                    local dcma = 0
                    local ns = 0
                    while true and ns < 20 do
    --                while dc < dcma and n < 10 do
                        local dcx = math.abs(curvH(a,pmi+vec3(gradstep,0,0),b)) - cmi
                        local dcy = math.abs(curvH(a,pmi+vec3(0,gradstep,0),b)) - cmi
                        local u = vec3(-dcx, -dcy):normalized()
                        local pt = pmi + u*gradstep
                        if not ifValid(pt, i) then
                            break
                        end
                        pmi = pt
                        dc = math.abs(curvH(a,pmi,b)) - cmi
                        if dc < dcma then --and U.vang(pmi-a,b-pmi)<math.pi/2 then
                            dcma = dc
                        else
                            break
                        end
                        ns = ns + 1
                    end
                    local pps = list[i].ps
                    table.remove(list,i)
                    table.insert(list, i, {e={pmi,b},ps=ps})
                    table.insert(list, i, {e={a,pmi},ps=pps})

                    if pmi:distance(a) < pinstep*cseek then
                        list[i].done = true
--                        list[#list+1] = {a,pmi}
                    end
                    if pmi:distance(b) < pinstep*cseek then
                        list[i+1].done = true
--                        list[#list+1] = {pmi,b}
                    end
                            ap[#ap+1] = pmi
--                            toMark(ap, 'red', nil, 0.1, 0.6)
--                            toMark({pmi}, 'red', nil, 0.1, 0.6)
                    break
                else
                    list[i].done = true
                    break
                end
--                        if dbg then return end
            end
        end
--            U.dump(list, '?? LIST:'..tostring(done)..':'..n)
            if dbg then lo('?? next_L: L:'..#list..' n:'..n..':'..tostring(done)) end
--            if true then break end
        n = n + 1
--            if dbg then return end
    end
        list[#list+1] = {e={list[#list].e[2],pth[2]}}
        -- check curvature
        local acrv = {}
        for i=2,#list-1 do
--                lo('?? for_i:'..i..':'..tostring(list[i-1]))
--                U.dump(list[i-1], '?? l-1:'..i)
            acrv[#acrv+1] = U.curv(list[i-1].e[1],list[i].e[1],list[i+1].e[1])
        end
            if dbg then U.dump(acrv, '?? for_curv:') end
        out.aseg = {}
        local ap2 = {}
    ap = {}
    for i=1,#list do
        local d = list[i]
        ap[#ap+1] = d.e[1] + vec3(0,0,forZ(d.e[1]))
            ap2[#ap2+1] = {
                list[i].e[1]+vec3(0,0,forZ(list[i].e[1])),
                U.proj2D(list[i].e[2]) + vec3(0,0,forZ(U.proj2D(list[i].e[2])))}
--        for i,d in pairs(list) do
        if i > 1 then
--                lo('?? for_seg:'..i..':'..tostring(d.ps)..':'..tostring(d.e[1]))
                if i and d.ps then
                    out.aseg[#out.aseg+1] = {d.ps+vec3(0,0,forZ(d.ps)),d.e[1]+vec3(0,0,forZ(d.e[1]))}
                end
--                ap2[#ap2+1] = d.ps+vec3(0,0,forZ(d.ps)+3)
        end
    end
        local ap3 = {}
--        lo('?? ap0:'..#ap)
    for i=#ap-1,2,-1 do
        local crv = math.abs(U.curv(U.proj2D(ap[i+1]),U.proj2D(ap[i]),U.proj2D(ap[i-1])))
            if dbg then lo('?? for_CURVE:'..i..':'..crv) end
        if crv > 0.02 then
--            lo('?? to_CHOP:'..i)
            ap3[#ap3+1] = ap[i]
            table.remove(ap, i)
        end
    end
--        lo('?? ap1:'..#ap)
        toMark(ap3,'yel',nil,0.2)
    out.inplot = ap
        out.aseg = ap2
--        ap[#ap+1] = pth[2]
        toMark(ap, 'red', nil, 0.1, 0.6)
        lo('<< decalPlot: n:'..n..':'..#list..':'..#ap2..':'..tostring(out.inplot))
--        out.aplot = {{e=ap2}}
--        Render.path(out['red'], color(255,200,200,155), 4)
        -- check curvature

--        out.inplot = out['red']
--        out.a
--        out.aplot = list

--        fout.close()
    if true then
--            lo('?? pre_add:'..#adec)
--            toMark(out.inplot, 'mag', nil, 0.2)
        local obj = decalUp({
            w = w or default.laneWidth*3,
--            w = default.laneWidth*5,
            mat='WarningMaterial',
            list = out.inplot}, ext)
        local rd = adec[#adec]
--              toMark(rd.list, 'mag', nil, 0.1, 1)
-- set equidistant nodes
--            lo('?? pre_e2d:')
--        rd.e2d = edge2dist(rd)
--            lo('?? post_e2d:')
        rd.L = roadLength(rd)
        local nstep = math.floor(rd.L/pinstep+0.5)
        local step = rd.L/nstep
        list = {}
        for i = 0,nstep do --step*nstep,step do
            local d = i*step
--        for d = 0,step*nstep,step do
--                lo('?? to_LIST:'..d..'/'..rd.L..':'..(step*nstep)) --..':'..tostring(onSide(rd,'middle',870))..':'..tostring(onSide(rd,'middle',902)))
            list[#list+1] = onSide(rd,'middle',d)
        end
--              toMark({onSide(rd,'middle',870)}, 'mag', nil, 0.1, 1)
--              toMark({list[#list-1]}, 'mag', nil, 0.1, 1)
--              toMark(out.inplot, 'mag', nil, 0.1, 1)
--              toMark(list, 'mag', nil, 0.1, 1)
        nodesUpdate(rd, list)
        rd.list = list

        across[rd.ind] = {}
        across[rd.ind][1] = {}
        across[rd.ind][#rd.list] = {}
        out.inplot = nil
        out.aseg = nil
        out['red'] = nil
        out['yel'] = nil
        out['blue'] = nil
        out['cyan'] = nil
        out['green'] = nil
        out.acyan = nil
--            lo('?? decalPlot.added:'..#adec..':'..rd.ind..':'..obj:getID()..':'..rd.id)
--        decalsLoad()
        return adec[#adec]
    end


    if false then
        local list = {}
        for d=0,nstep*pinstep,pinstep do
--            list[#list+1] = pfr + u*d
--            list[#list] = list[#list] + vec3(0,0,forZ(list[#list]))
        end
    --        U.dump(list, '?? for_LIST:')
    --        if true then return end
        out.inplot = list
        -- gracdient ascent
        for i=2,#list-1 do
            --- eval slope
            ---- shift direction
    --        local dir = ((list[i-1]-list[i]):normalized()+(list[i+1]-list[i]):normalized()):normalized()
        end
    end
end
D.decalPlot = decalPlot
--                            ap[#ap+1] = pmi
                            -- insert to list
--                            if U.vang(pmi-a,b-pmi)>(math.pi*1/2)  then
--                                lo('??_______ SHARP:'..#list..':'..tostring(pmi)..':'..(U.vang(pmi-a,b-pmi)-0)..'>'..tostring(math.pi/2)..':'..tostring(U.vang(pmi-a,b-pmi)<(math.pi*5/6)))
--                            end
--                                lo('?? for_curv:'..U.vang(pmi-a,b-pmi))
--                            lo('?? for_DC:'..dc..':'..n..':'..tostring(u))
--                        dc = cmi - U.

--[[
                        if false then
                            if pt:distance(list[i].e[1]) < pinstep/2 then
                                break
                            end
                            if pt:distance(list[i].e[2]) < pinstep/2 then
                                break
                            end
                            local ama = math.pi/4
                            if i > 1 then
                                -- check ends angles
                                local ang = U.vang(pt-list[i].e[1], list[i-1].e[1]-list[i-1].e[2])
                                if ang < ama then
                                    lo('?? SHARP:'..i)
                                    break
                                end
                            end
                            if i < #list then
                                local ang = U.vang(pt-list[i].e[2], list[i+1].e[2]-list[i+1].e[1])
                                if ang < ama then
                                    break
                                end
                            end
                        else
]]


D.setCar = function(pos, ang, lane)
    if not ang then ang = 0 end
    if not veh then
        lo('?? ERR_setCar_NO_VEH:')
        return
    end
    if lane then car.lane = lane end
    car.pos = pos
--    veh:setPositionNoPhysicsReset(pos)
    veh:setPosRot(pos.x, pos.y, pos.z, 0, 0, ang, 0) -- rot.w)
end


local function keyUD(dir)
    if dir == -1 and car.gaz == -1 then return end
        lo('>> D.keyUD:'..dir..':'..#adec..':'..tostring(car.pos))
    if dir == 1 then
        local target = car.target or forTarget()
--            U.dump(out.avedit,'?? if_target:'..tostring(target))
        if target then
            car.target = target
            car.gaz = 0.2
        end
    elseif dir == -1 then
        car.gaz = -1
        toTarget()
    end
        lo('<< D.keyUD:'..tostring(car.target))
end


local function apply(key, val)

    if false then
    elseif key == 'conf_bank' then
        bank = val
    elseif key == 'conf_jwidth' then
    elseif key == 'conf_margin' then
        margin = val
    elseif key == 'conf_mslope' then
        maxslope = 2*math.pi/360*val
    end
        lo('?? apply:'..tostring(aconf and #aconf or nil)..':'..tostring(out.inconform)..':'..tostring(out.inall)..':'..tostring(cdesc))

    local toundo
    if out.inall then
        toundo = true
    elseif aconf then
        if cdesc then --and #aconf == 1 and aconf[1] == cdesc then
            if adec[cdesc].aset then
                local rd = adec[cdesc]
                    lo('?? undo_local:'..#adec[cdesc].aset..':'..tableSize(mask)..':'..tostring(rd.jrad))
                    local amark = {}
                local eb,ee = U.proj2D(rd.list[1]),U.proj2D(rd.list[#rd.list])
                for _,ijs in pairs(rd.aset) do
--                        lo('?? for_ijs:'..tostring(ijs))
                    local d = mask[ijs]
                    if d then
                        local p = grid2p(d[4])

                        if p:distance(eb) > rd.jrad and p:distance(ee) > rd.jrad then
--                                amark[#amark+1] = p
                            tb:setHeightWs(p, d[1])
                            mask[ijs] = nil
                        end
                    end
                end
--                        toMark()
--                        tb:updateGrid()
--                        rd.body:setPosition(rd.body:getPosition())
--                        return
            elseif #aconf == 1 and aconf[1] == cdesc and dhist[cdesc] and #dhist[cdesc] then
                mask = dhist[cdesc][#dhist[cdesc]]
                table.remove(dhist[cdesc], #dhist[cdesc])
                    lo('?? from_HIST:'..cdesc)
                toundo = true
            end
        end
    end
    if toundo then
        lo('?? UNDO:')
        D.undo(true)
    end
    if false and cdesc then
            lo('?? set_one:'..tostring(cdesc))
        road2ter({cdesc})
    elseif aconf and #aconf then
        road2ter(aconf)
    end
        lo('<< apply:'..key..':'..val..':'..tostring(cdesc)..':'..tostring(cpick))
end


D.test = function()
    if true then return end
    decalPlot()
end


local function onUpdate(rayCast)
--        lo('?? D.unUpd:')
        if _dbdrag then return end
--    if true then return end
--    if im.IsWindowHovered(im.HoveredFlags_AnyWindow) or im.IsAnyItemHovered() then return end
--    if not out.dbg then
--        lo('??********************************************* D.IN_UPDATE:')
--        out.dbg = true
--    end

--    local rayCastHit
    local rayCast,rayCastHit = cameraMouseRayCast(false)
--            lo('?? D.upd:'..tostring(rayCast))
--    if not rayCast then return end
    if rayCast then
--        lo('?? D.onUpdate:'..tostring(rayCast.object.name))
        if rayCast.object.name ~= 'theTerrain' then return end
    end

    if editor.keyModifiers.ctrl then
        if im.IsKeyPressed(im.GetKeyIndex(im.Key_Z)) then
--            lo('?? CTRL_Z:')
            if U._MODE == 'conf' then
                D.undo()
                for _,dec in pairs(adec) do
                    local ind = dec.ind
                    local rd = adec[ind]
                    rd.body:setField('material', 0, 'WarningMaterial')
                    rd.body:setPosition(rd.body:getPosition())
                end
            elseif U._PRD == 0 then
                restore()
            end
            return
        end
    end
    if editor.keyModifiers.alt then
--            lo('?? to_plot:'..tostring(cjunc)..':'..tostring(out.aplot))
        if not U._MODE=='conf' and not out.aplot then
            out.aplot = forPlot()
        end
--[[
        if out.aplot then
            -- render links
            for i,d in pairs(out.aplot) do
                Render.path(d.e, color(255,255,0,155), 4)
            end
        else
            out.aplot = forPlot()
        end
]]
--        if not inplot then
--            inplot = true
--        end
    end
    if im.IsKeyReleased(im.GetKeyIndex(im.Key_ModAlt)) then
        if U._MODE ~= 'conf' then out.aplot = nil end
        inmerge = nil
    end

    local rdhit
    if true and rayCast then
        rayCastHit = rayCast.pos

        -- EVENTS
        --- MOUSE
    --    local aiRoadsSelectable = editor.getPreference("roadEditor.general.aiRoadsSelectable")
    --            local hasroads = 0
        local ishit = false
--[[

]]
--        if true then
--        out.pdrag = nil
        if not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not im.IsAnyItemHovered() then
            for ind,rd in pairs(adec) do
                local roadID = rd.id
--            for roadID, _ in pairs(rdlist) do
                local road = scenetree.findObjectById(roadID)
                if road and not rd.skip and #U.index(askip,rd.id) == 0 and (not road.name or string.find(road.name, 'line_') ~= 1) then
--                if road and not rd.skip and (not road.name or string.find(road.name, 'line_') ~= 1) then
                    if road:containsPoint(rayCastHit) ~= -1 then
                        rdhit = ind
                        if cmover ~= roadID then
                            if roadID ~= cpick and not indrag then -- and not cjunc then
            --                    lo('?? SWITCH_select:'..tostring(cmover)..'>'..roadID)
                                cmover = roadID
                                cdescmo = ind
                                -- hightlight
                                out.anode = {}
                                local anode = editor.getNodes(road)
                                out.ared = {anode[1].pos}
                                for i = 2,#anode-1 do
                                    local n = anode[i]
--                                for _,n in pairs(anode) do
                                    out.anode[#out.anode + 1] = n.pos
                                end
                                if icroad ~= ind then
--                                    lo('?? over_IND:'..ind)
                                    icroad = ind
                                end
                                out.agreen = {anode[#anode].pos}
                                -- check junctions
                                local dmi,jmi,imi = math.huge
                                for k,d in pairs(ajunc) do
                                    if d.list then
                                        for i,r in pairs(d.list) do
                                            if roadID == r.id then
    --                                                lo('?? for_rd:'..roadID..':'..i)
                                                local tocenter = (d.p - rayCast.pos):length()
                                                if tocenter < dmi then
                                                    dmi = tocenter
                                                    jmi = k
                                                    imi = i
                                                end
        --                                        out.avedit = {}
                                            end
                                        end
                                    end
--[[
                                            cjunc = k
                                            for i,b in pairs(d.list) do
                                                if b.io == 1 and (b.list[2]-anode[2].pos):length() < U.small_val then
    --                                            if b.io == 1 and (b.desc.list[2]-anode[2].pos):length() < U.small_val then
                                                    local p = d.p + b.dir*(20 + (d.rround or 0))
                                                    out.pdrag = {ind=i, p=p, dir={p + 2*U.vturn(b.dir,math.pi/2), p - 2*U.vturn(b.dir,math.pi/2)}}
                                                    break
    --                                                out.apicknode = {d.p + b.dir*20}
    --                                                out.avedit = {d.p + b.dir*20}
    --                                                out.atri = {{d.p + b.dir*20, d.p + b.dir*20+vec3(2,0,0), d.p + b.dir*20+vec3(0,2,0)}}
                                                end
                                            end
]]
--[[
                                    if (anode[1].pos-d.p):length() < U.small_val then
                                        cjunc = k
--                                        out.avedit = {}
                                        for i,b in pairs(d.list) do
                                            if b.io == 1 and (b.list[2]-anode[2].pos):length() < U.small_val then
--                                            if b.io == 1 and (b.desc.list[2]-anode[2].pos):length() < U.small_val then
                                                local p = d.p + b.dir*(20 + (d.rround or 0))
                                                out.pdrag = {ind=i, p=p, dir={p + 2*U.vturn(b.dir,math.pi/2), p - 2*U.vturn(b.dir,math.pi/2)}}
                                                break
--                                                out.apicknode = {d.p + b.dir*20}
--                                                out.avedit = {d.p + b.dir*20}
--                                                out.atri = {{d.p + b.dir*20, d.p + b.dir*20+vec3(2,0,0), d.p + b.dir*20+vec3(0,2,0)}}
                                            end
                                        end
                                    end
]]
                                end
                                if jmi and ajunc[jmi].list[imi].dir then
--                                        lo('?? JMI:'..jmi)
                                    local p = ajunc[jmi].p + ajunc[jmi].list[imi].dir*(20 + (ajunc[jmi].rround or 0))
                                    out.pdrag = {jind=jmi, ind=imi, p=p} --, dir={p + 2*U.vturn(b.dir,math.pi/2), p - 2*U.vturn(b.dir,math.pi/2)}}
--                                    cjunc = jmi
                                end
                            else
                                cmover = roadID
                            end
                        end
                        ishit = true
                        break
                    end
                end
            --        hasroads = hasroads + 1
            end
            if not rdhit and not cpick and #apick == 0 then
                out.ared = nil
                out.agreen = nil
                icroad = nil
            else
                if editor.keyModifiers.shift then
--                    lo('?? over_RD:'..tostring(rdhit)..':'..tostring(adec[rdhit].id))
                end
            end
        end

    --            lo('?? if_is:'..tostring(ishit))
        if not ishit then -- and not cpick then
            cmover = nil
            out.anode = nil
        end
    end
--        if true then return end

    if im.IsMouseDown(0) and rayCast and not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not im.IsAnyItemHovered() then
--            if _dbdrag then return end
--            lo('?? ifDRAG:'..tostring(out.pdrag))

        if out.pdrag and (rayCast.pos - out.pdrag.p):length()<2 then
            if not indrag then
                -- START DRAG
                indrag = rayCast.pos
                    lo('?? to_DRAG:'..tostring(indrag)..':'..out.pdrag.ind)
                out.avedit = {}
                ajunc[cjunc].dirty = true


                for _,m in pairs(amatch) do
                    if m.ind == out.pdrag.ind then
                        out.agreen = nil
                        out.ared = nil
                        out.anode = nil
--                        out.avedit = nil
--                        cjunc = out.pdrag.jind
--                        cjunc = adec[m.ind].ij[1]
                        m.match = nil
                        m.skip = nil
                        if m.fr then
                            amatch[m.fr].match = nil
                            amatch[m.fr].skip = nil
                            amatch[m.fr].fr = nil
                            amatch[m.fr].to = nil
                        end
                        if m.to then
                            amatch[m.to].match = nil
                            amatch[m.to].skip = nil
                            amatch[m.to].fr = nil
                            amatch[m.to].to = nil
                        end
                        m.fr = nil
                        m.to = nil
                            U.dump(amatch, '?? for_am:')
                        break
                    end
                end
                return
            end
        end
        if indrag then
            if (rayCast.pos-indrag):length() > 1. then
--                    lo('?? dragging:'..out.pdrag.ind..':'..tostring(rayCast.pos-out.indrag))
                -- update road angle
                local bdesc = ajunc[cjunc].list[out.pdrag.ind]
                bdesc.dir = (rayCast.pos-ajunc[cjunc].p):normalized()
                bdesc.dirty = true
                out.pdrag.p = ajunc[cjunc].p + bdesc.dir*(20 + (ajunc[cjunc].rround or 0))
                junctionUpdate(ajunc[cjunc])
                out.aplot = forPlot()
--                out.avedit[#out.avedit+1] = rayCast.pos
                indrag = rayCast.pos
--                        _dbdrag = true
            end
        end
    end
    if im.IsMouseReleased(0) and (inView() or incontrol) then
        if #adec == 0 then
            cjunc = nil
            cdescmo = nil
            cmover = nil
        end
            lo('?? D.mup:'..tostring(cjunc)..':'..tostring(incontrol)..':'..tostring(cmover)..' ind:'..tostring(cdescmo)..':'..tostring(indrag)) --..':'..tostring(rayCast and rayCast.pos or nil)..':'..tostring(cdescmo and adec[cdescmo].body:getOrCreatePersistentID() or nil))
        for key,val in pairs(dval) do
                lo('?? for_val:'..key..':'..val)
            apply(key, val)
            dval[key] = nil
            return
        end
        if incontrol and cjunc then
            out.avedit = nil
            incontrol = nil
            if U._MODE ~= 'conf' or not ajunc[cjunc].round then
                junctionUpdate(ajunc[cjunc], true)
            end
            if ajunc[cjunc].round then
                junctionRound()
--                out.ared = {ajunc[cjunc].p}
            end
--[[
            for _,b in pairs(ajunc[cjunc].list) do
                b.dirty = false
            end
            for _,e in pairs(ajunc[cjunc].aexit) do
                e.body:setField('hidden', 0, 'false')
            end
            junctionUpdate(ajunc[cjunc], true)
]]
--            out.avedit = nil
            out.pdrag = nil
            out.aplot = nil
        elseif false and editor.keyModifiers.ctrl then
            local ps,rd,side
            if rayCast and cdescmo then
                rd = adec[cdescmo]
                -- compare distance to sides
                local p = U.proj2D(rayCast.pos)
                local dl,pl = D.toSide(p, rd, 'left')
                local dr,pr = D.toSide(p, rd, 'right')
                ps = dl < dr and pl or pr
                side = dl < dr and 'left' or 'right'
            end
            if out.sidepick and rd.ind ~= out.sidepick.ind then
                local rda,rdb = adec[out.sidepick.ind],rd
                D.road2ter({rda.ind})
                D.road2ter({rdb.ind})
                local edesc = D.forExitFree(rda, rdb, out.sidepick.p, ps, out.sidepick.side, side)
                if edesc then
                        U.dump(edesc.frtoside, '?? ex_FRTO:')
                    D.road2ter({edesc.ind})
--                    edesc.frtoside = {out.sidepick.side, side}
                end
                out.sidepick = nil
            elseif ps then
                out.sidepick = {
                    p=ps+vec3(0,0,forZ(ps)),
                    ind=cdescmo,
                    side=side}
                    U.dump(out.sidepick, '?? for_sp:')
            end
            return 1
--[[
                if rayCast and cdescmo then
                    local rd = adec[cdescmo]
                    local p = U.proj2D(rayCast.pos)
                    local dl,pl = toSide(p, rd, 'left')
                    local dr,pr = toSide(p, rd, 'right')
                    local ps = dl < dr and pl or pr
                    local side = dl<dr and 'left' or 'right'
                    if out.sidepick and rd.ind ~= out.sidepick.ind then
                        local rda,rdb = adec[out.sidepick.ind],rd
                        local edesc = forExitFree(rda, rdb, out.sidepick.p, ps, out.sidepick.side, side)
                        edesc.frtoside = {out.sidepick.side, side}
                            lo('?? tgt:'..out.sidepick.ind..'>'..rd.ind..':'..side)
                        out.sidepick = nil
                        return rd.ind
                    else
                            lo('?? src:'..tostring(ps))
                        out.sidepick = {
                            p=ps+vec3(0,0,forZ(ps)),
                            ind=cdescmo,
                            side=side}
                    end
    --                        lo('?? if_SIDE:'..tostring(cmover)..':'..tostring(cdescmo)..':'..dl..':'..dr)
    --                        toMark({ps},'blue')
                elseif out.sidepick then
                        lo('!! no_ROAD_HIT:')
                    out.sidepick = nil
                    return math.huge
                end
]]
        elseif editor.keyModifiers.alt then
                lo('?? D.for_ALT:')
            if rayCast then
                    lo('?? check_HIT:'..tostring(rayCast.pos)..':'..#adec)
                local p = U.proj2D(rayCast.pos)
                local dmi,jmi,nmi,ie1,ie2 = math.huge
                for j,dec in pairs(adec) do
                    if not out.sidepick or out.sidepick.src ~= dec.id then
--                    if not out.sidepick or out.sidepick.src ~= dec.ind then
                        for k,n in pairs(dec.list) do
                                if j == #adec then
    --                                lo('?? for_DIST:'..j..':'..k..':'..p:distance(n)..':'..tostring(n))
                                end
                            local d = p:distance(U.proj2D(n))
--[[
                            local d,pp,iep = D.toSide(p, dec)
                            ie1 = iep
                            ie2 = ie1 < (adec[j].ne or adec[j].body:getEdgeCount())-1 and ie1+1 or ie1
                            if j == 129 or j == 157 then
                                lo('?? for_129:'..j..':'..k..':'..d..':'..ie1..':'..ie2)
                            end
]]
--                            if not out.sidepick or out.sidepick.list[1]:distance(U.proj2D(n)) > U.small_dist then
                            local k2 = k < #dec.list and k+1 or k
                            local isin = (U.proj2D(dec.list[k])-p):dot(U.proj2D(dec.list[k2])-p)
                            if d < dmi and isin<0 then --and (D.epos(adec[j].body, ie1)-p):dot(D.epos(adec[j].body,ie2)-p)<0 then
                                dmi = d
                                jmi = j
                                nmi = k
                            end
--                            end
                        end
                    end
                end
--                    lo('?? for_DIR:'..jmi..':'..ie1..':'..ie2..':'..tostring(D.epos(adec[jmi].body, ie1)-p)..':'..tostring((D.epos(adec[jmi].body,ie2)-p))..':'..tostring(p))-- :dot(D.epos(adec[jmi].body,ie2)-p))
                    lo('?? if_HIT:'..tostring(dmi)..':'..tostring(jmi)) --..':'..nmi..':'..adec[jmi].ind) --..' ie:'..ie1..':'..tostring(adec[jmi].ne))
                local ps,ds = rayCast.pos
                if dmi and jmi then
                    if p:distance(adec[jmi].list[nmi]) < 4 and (nmi == 1 or nmi == #adec[jmi].list) then
                        -- end node pick
                        ps = adec[jmi].list[nmi]
                            lo('??++++++++++++++++ end_node:'..dmi..':'..nmi..':'..tostring(ps))
                        out.acyan = {ps}
--                            out.inj = true
                    else --if (nmi ~= 1 and nmi ~= #adec[jmi].list) then
                        -- side pick
                        ds,ps = toSide(p,adec[jmi],'middle')
                            lo('?? inside:'..tostring(ps)..':'..ds..'/'..(adec[jmi].aw[nmi])..':'..jmi)
--                            D.toMark({ps},'cyan')
                        if ds < adec[jmi].aw[nmi]/3 then
--                        if ds < adec[jmi].aw[nmi]/2+2 then
                            ps = ps + vec3(0,0,forZ(ps))
                        elseif ds < adec[jmi].aw[nmi]*2/3 then
                                lo('??+++++++++++++++++++++++++++++++++++++ if_SIDE:')
                            local rd,side = adec[jmi]
                            if true then
                                local p = U.proj2D(rayCast.pos)
                                local dl,pl = D.toSide(p, rd, 'left')
                                local dr,pr = D.toSide(p, rd, 'right')
                                ps = dl < dr and pl or pr
                                side = dl < dr and 'left' or 'right'
                            end
                            if out.sidepick and rd.ind ~= out.sidepick.ind then
                                local rda,rdb = adec[out.sidepick.ind],rd
                                D.road2ter({rda.ind})
                                D.road2ter({rdb.ind})
                                local edesc = D.forExitFree(rda, rdb, out.sidepick.list[1], ps, out.sidepick.side, side)
                                if edesc then
                                        U.dump(edesc.frtoside, '?? ex_FRTO:')
                                    D.road2ter({edesc.ind})
                --                    edesc.frtoside = {out.sidepick.side, side}
                                end
                                out.sidepick = nil
                                return 1
                            elseif ps then
                                out.sidepick = {
                                    list = {ps+vec3(0,0,forZ(ps))},
--                                    p=ps+vec3(0,0,forZ(ps)),
                                    ind=cdescmo,
                                    side=side}
                                        U.dump(out.sidepick, '?? for_sp:')
                            end
                            return 1
                        else
                            ps = rayCast.pos
                            ds = nil
                        end
    --                        toMark({ps}, 'blue')
--[[
                        if false then
                            if ds < adec[jmi].aw[nmi]/3 then
--                            if ds < adec[jmi].aw[nmi]/2+2 then
                                if out.sidepick then
                                    local a = adec[out.sidepick.src]
                                    local b = adec[jmi]
                                    decalSplit(a, out.sidepick.p)
                                    decalSplit(b, ps+vec3(0,0,forZ(ps)))

                                    local pinstep = 20
                                    local pfr,pto = U.proj2D(out.sidepick.p),U.proj2D(ps)
                                    local dlen = pfr:distance(pto)
                                    local nstep = math.floor(dlen/pinstep+0.5)
                                    local step = dlen/nstep
        --                                lo('?? decalSplit:'..rd.ind..':'..dfr..':'..dto..':'..tostring(ifr)..' dspl:'..dspl..':'..nstep..'/'..step)
                                    local u = (pto-pfr):normalized()
                                    local list = {}
                                    for d=0,nstep*step,step do
                                        list[#list+1] = pfr+u*d
                                    end
                                    local desc = {list=list,w=default.laneWidth*3,mat='WarningMaterial'}
                                    D.decalUp(desc)

                                    decalsLoad()
        --                            forCross()
                                    out.sidepick = nil
                                else
                                    out.sidepick = {p=ps+vec3(0,0,forZ(ps)), src=jmi}
                                end
                            else
                                out.sidepick = nil
                            end
                        end
]]
                    end
--[[
                    if ps then
                        if out.sidepick then
                            decalPlot({out.sidepick.p,ps})
                        else
                            out.sidepick = {p=ps, src=jmi}
                        end
                    end
                elseif false then
                    if out.sidepick then
--                            lo('?? if_SP:'..tostring(out.sidepick)..':'..tostring(out.sidepick.list and #out.sidepick.list or 0))
                        out.sidepick.list[#out.sidepick.list+1] = rayCast.pos
                        if #out.sidepick.list == 2 then
--                                lo('?? SP_DOWN:')
--                            out.inplot = out.sidepick.list
                            D.decalPlot(out.sidepick.list)
                            out.sidepick = nil
                            return 1
                        end
                    end
-- NEW start
                    out.sidepick = {list={rayCast.pos}}
                        lo('?? sPICK:'..tostring(rayCast.pos)..':'..#out.sidepick.list)
                    return 1
                            if out.inj then
                                if #out.sidepick.list == 2 then
                                    local id = D.decalPlot(out.sidepick.list, true)
                                    out.sidepick = nil
                                end
                                return 1
                            end
]]
                end
                if out.sidepick then
                    if out.sidepick.side then
                            lo('!!_______________ quit_PICK:')
                        out.sidepick = nil
                        return 1
                    end
--                            lo('?? if_SP:'..tostring(out.sidepick)..':'..tostring(out.sidepick.list and #out.sidepick.list or 0))
                    out.sidepick.list[#out.sidepick.list+1] = ps
                    out.sidepick.split[#out.sidepick.split+1] = ds and jmi or false

                    if #out.sidepick.list == 2 then
                            U.dump(out.sidepick, '?? for_SP:') --..tableSize(out.sidepick.split)..':'..jmi..':'..adec[jmi].id)
--                                lo('?? SP_PRE:'..#across)
--                            out.inplot = out.sidepick.list
                        if out.sidepick.split[1] then
                            decalSplit(adec[out.sidepick.split[1]], out.sidepick.list[1])
                        end
                        if out.sidepick.split[2] then
                            decalSplit(adec[out.sidepick.split[2]], out.sidepick.list[2])
                        end
                        local id = D.decalPlot(out.sidepick.list).id
                        out.sidepick = nil
                            lo('?? pre_LOAD:') --..tostring(out.inj))
                        decalsLoad() --nil,nil,id)
                            lo('?? post_LOAD:')
--                                lo('?? SP_POST:'..#across)
--                            U.dump(across[#across], '?? ADDED:'..adec[#adec].ind..':'..adec[#adec].id)
                            for _,d in pairs(adec) do
                                if d.id == id then
--                                    U.dump(across[d.ind], '?? cross:'..d.id..':'..d.ind)
                                    local cross = {}
                                    cross[1] = {}
                                    cross[#d.list] = {}
                                    across[d.ind] = cross
                                    break
                                end
                            end
--                                U.dump(across, '?? ACR:')
                        return 1
                    end
                end
-- NEW start
                out.sidepick = {list={ps},split={ds and jmi or false},src= jmi and adec[jmi].id or nil}
                    lo('?? sPICK:'..tostring(rayCast.pos)..':'..#out.sidepick.list)
                return 1
            else
                lo('!! NO_rayCast:')
            end
--[[
            if false then
                if rayCast and cdescmo then
                    local rd = adec[cdescmo]
                    local p = U.proj2D(rayCast.pos)
                    local dl,pl = toSide(p, rd, 'left')
                    local dr,pr = toSide(p, rd, 'right')
                    local ps = dl < dr and pl or pr
                    local side = dl<dr and 'left' or 'right'
                    if out.sidepick and rd.ind ~= out.sidepick.ind then
                        local rda,rdb = adec[out.sidepick.ind],rd
                        local edesc = forExitFree(rda, rdb, out.sidepick.p, ps, out.sidepick.side, side)
                        edesc.frtoside = {out.sidepick.side, side}
                            lo('?? tgt:'..out.sidepick.ind..'>'..rd.ind..':'..side)
                        out.sidepick = nil
                        return rd.ind
                    else
                            lo('?? src:'..tostring(ps))
                        out.sidepick = {
                            p=ps+vec3(0,0,forZ(ps)),
                            ind=cdescmo,
                            side=side}
                    end
    --                        lo('?? if_SIDE:'..tostring(cmover)..':'..tostring(cdescmo)..':'..dl..':'..dr)
    --                        toMark({ps},'blue')
                elseif out.sidepick then
                        lo('!! no_ROAD_HIT:')
                    out.sidepick = nil
                    return math.huge
                end
            end
]]
--[[
                    decalUp(edesc)
                    if not rda.aexo then rda.aexo = {} end
                    if not rdb.aexi then rdb.aexi = {} end
                    rda.aexo[#rda.aexo+1] = edesc.ind or 0
                    rdb.aexi[#rdb.aexi+1] = edesc.ind or 0
-- lines
                    edesc.aline = {}
                    line4exit(edesc, rda, rdb)
--                    edesc.aline = line4exit(edesc, rda, rdb, true)
]]
        else-- if no keymodifyers then
            out.inplot = nil
            out.sidepick = nil
--                lo('?? nnc:'..tostring(incontrol))
            -- detect junction hit
            local dmi,imi = math.huge
            for i,d in pairs(ajunc) do
                if rayCast and rayCast.pos:distance(d.p) < 3 then
                        lo('??^^^^^^^^ junc_hit:'..tostring(i)..':'..tostring(cjunc)..'/'..#ajunc)
                    if cjunc == i then
                        cjunc = nil
                        out.acyan = nil
                        out.injunction = nil
                        out.jdesc = nil
                    else
                        cjunc = i
                        cpick = nil
                        apick = {}
                        out.jdesc = ajunc[cjunc]
                        out.injunction = ajunc[cjunc]
                        out.acyan = {d.p}
                        out.ared = nil
                        out.agreen = nil

                        cpick = nil
                        out.apicknode = nil
                        cmover = cpick
                            lo('?? for_jlist:'..#ajunc[cjunc].list)

--                                onVal('b_junction')
--                                onVal('b_conform')
--                                junctionRound()
                    end
--                        lo('?? junc_hit2:'..tostring(cjunc)..':'..tostring(ajunc[cjunc].r))
                    return 1
                end
            end

            if cmover == cpick then
                -- deselect
                cpick = nil
                out.apicknode = nil
                cmover = cpick
                    lo('?? desel:', true)
            else
                -- new pick
                if editor.keyModifiers.ctrl then
                    local ind = cdescmo or rdhit
                    local inpick = U.index(apick,ind)
                    if #inpick == 0 then
                        apick[#apick+1] = ind
                    else
                        table.remove(apick, inpick[1])
                    end
                    out.ared = {}
                    out.agreen = {}
                    for _,ind in pairs(apick) do
--                                lo('?? ind:'..tostring(ind))
                        local rd = adec[ind]
                        out.ared[#out.ared+1] = rd.list[1]
                        out.ared[#out.ared+1] = rd.list[#rd.list]
                    end
                        U.dump(out.ared, '?? ared:')
                        lo('??**************** D.onUp_DONE:'..tostring(cdescmo)..':'..#apick)
                    if cdescmo then return cdescmo or 0 end
                else
                    apick = {}
                    cpick = cmover
                    out.inall = false
                    out.apicknode = out.anode
                    cdesc = cdescmo
                    out.anode = nil
                    if #U.index(conformed, cpick) == 0 then
                        out.inconform = false
                    else
                        out.inconform = true
                    end
                    cjunc = nil
                    out.acyan = nil
                    out.injunction = nil
                    out.jdesc = nil
                    return cdesc
                end
                out.apoint = nil
                croad = scenetree.findObjectById(cpick)
                        local ind = inAdec(cpick)
                        U.dump(across[ind], '?? picked:'..tostring(ind)..':'..tostring(cpick)..':'..tostring(croad)..':'..tostring(croad and croad:getOrCreatePersistentID() or nil))
--                croad:setField("material", 0, "road1_concrete")
            end

        end
            lo('?? D.mup_out:')
        indrag = nil
        incontrol = nil
    end

    local cvel,cpos
    if veh then
        local ctime = os.clock()
        if not car.time or ctime - car.time > 0.1 then --0.02 then
--                lo('?? for_time:'..ctime..':'..tostring(car.time))
--                lo('?? for_DV:'..tostring(veh:getDirectionVector())..':'..tostring(veh:getVelocity():normalized()))
            if veh:getVelocity():length() > 8 then
--                lo('?? gaz_DOWN:')
                car.gaz = 0.07
            end
            if car.gaz < 0 then
--                    lo('?? for_back_vel:'..tostring(veh:getVelocity()))
                if veh:getVelocity():length() < 0.1 then
                        lo('?? for_back_STOP:'..tostring(veh:getVelocity()))
                    car.gaz = 0
                    toTarget()
                    car.target = nil
                end
            end
            if car.target then
                car.pos = veh:getPosition()
                car.vel = veh:getVelocity()
                local inexit
                if car.target.exit then -- and car.pos:distance(car.target.exit.p) < car.vel*3.  then
--                        lo('?? for_EXIT:'..tostring(car.target.exit.p))
                    if car.target.exit.p then
                            out.ared = {car.target.exit.p}
                        local toexit = car.pos:distance(car.target.exit.p)
                        local time2exit = toexit/car.vel:length()
--                            lo('?? time_to_exit:'..car.vel:length()..':'..toexit..':'..(toexit/car.vel:length()))
                        local dir = car.vel:normalized()
                        local togo
                        local rdto = adec[car.target.exit.idec]
                        if rdto.ij and rdto.ind ~= car.target.rd.ind and car.target.rd.ij then
                            local bind = car.target.rd.ij[2]
                            local brnext = U.mod(bind+1, ajunc[car.target.rd.ij[1]].list)
                            if car.pos:distance(car.target.rd.list[1]) < brnext.w/2+1 then
                                togo = true
                            end
                        elseif time2exit < default.react_time then
                            togo = true
                        end
                        if togo then
                            -- TAKE EXIT
--                        if (car.target.exit.p-car.target.p):dot(dir) < 3 or time2exit < 5 then
--                        if toexit < car.pos:distance(car.target.p) + 2 or time2exit < 5 then
                            -- goto exit
                            car.target.side = car.target.exit.side or 'right'
                            car.target.rd = adec[car.target.exit.idec]
                            car.target.lane = car.target.exit.lane or 1
                            car.target.ie = car.target.exit.ie or 0
                                out.green = {car.target.p}
                            if car.target.rd.ij then
                                car.target.p = car.target.exit.p
                            else
                                car.target.p = onLane(car.target.ie, car.target.rd, car.target.side, car.target.lane)
                            end
                            car.target.exit = nil
                                lo('??_____________________ to_EXIT:'..car.target.rd.ind..':'..tostring(car.target.rd.ij)..' lane:'..car.target.lane)
                                out.avedit = {car.target.p}
    --                            out.avedit[#out.avedit+1] = car.target.p
    --                            U.dump(car.target, '?? if_EXIT:')
    --                            toTarget(0,-1)
                            toTarget()
                            inexit = true
    --                            lo('?? toEX:')
                        end
                    end
                end
                local forcommand = true
                if #incommand > 0 then
                    if ccommand ~= incommand[1] then
                        lo('?? for_COMMAND:'..incommand[1]..':'..tostring(inexit)..':'..tostring(car.target.lane))
                    end
--                        U.dump(car.target, '?? for_COMMAND:'..incommand[1]..':'..tostring(inexit))
                    ccommand = incommand[1]
                    forcommand = true
                    local done = true
                    if not inexit then
                        if false then
                        elseif incommand[1] == 'left' and car.target.rd then
--                            U.dump(adec[car.target.rd.ind].lane, '?? if_LANE:'..tostring(car.target.lane)..':'..tostring(car.target.side))
                            local nlane = car.target.side=='right' and adec[car.target.rd.ind].lane[2] or adec[car.target.rd.ind].lane[1]
                            if car.target.lane < nlane then
                                toLane(1)
                            else
                                incommand[1] = 'turn_left'
                                done = false
                            end
                        elseif incommand[1] == 'right' then
                            if car.target.lane > 1 then
                                toLane(-1)
                            else
                                incommand[1] = 'exit_right'
                                done = false
                                forcommand = false
                            end
                        elseif incommand[1] == 'turn_left' then
                            forBranch(car.target.rd, 'left')
    --                        toCross('left')
                        elseif incommand[1] == 'turn_right' then
                            forBranch(car.target.rd, 'right')
    --                        toCross('right')
                        elseif incommand[1] == 'exit_right' then
                            done = toExit('right')
                        elseif incommand[1] == 'lane_left' then
                        end
                    end
                    if done then
                        incommand = {}
                        forcommand = false
                    end
--                        _dbdrag = true
                end
--                if (#incommand == 0 and not inexit and not car.target.exit and car.target.p:distance(car.pos) < default.v2tmin and car.gaz>0) then
                if (#incommand > 0 and forcommand) or (#incommand == 0 and not inexit and not car.target.exit and car.target.p:distance(car.pos) < default.v2tmin and car.gaz>0) then
--                elseif not inexit and not car.target.exit and car.target.p:distance(car.pos) < default.v2tmin and car.gaz>0 then
                        lo('?? pre_target:'..tostring(car.target.rd)) --.ind)
                    local target = forTarget()
--                        lo('?? next_target:'..tostring(target))
                    if not target then
                        -- not found, stop
                        toTarget(0, -1)
                        car.target = nil
                    else
--                            lo('?? change_target:'..tostring(car.target.p)..'>'..tostring(target.p))
                        car.target = target
                        if car.target then
                            out.ared = {car.target.rd.list[1]}
                            out.agreen = {car.target.rd.list[#car.target.rd.list]}
                                lo('??************************ new_target:'..car.target.rd.ind)
                        end
                    end
--                    toTarget(0, -1)
                end
                if car.target then
                    local ang = -U.vang(car.vel, car.target.p-car.pos, true)
                    if car.vel:length() < 0.2 then
                        ang = 0
                    end
    --                    c = 2/3
    --                local c = car.target.curv and 5/3 or 3/4
                    local c = (car.target.curv and car.target.curv>0.001 and 5/3) or 3/4
                        c = 5/3
                    toTarget(c*ang)
                else
                    toTarget(0, -1)
                end
--[[
                if false then
                    car.vel = veh:getVelocity()
                    car.pos = veh:getPosition()
    --                toTarget()
                    if U.vang(car.vel, car.target.p-car.pos) < 0.3 then
    --                    lo('??_____________________ for_ang:'..tostring(U.vang(car.vel, car.target-car.pos)))
    --                    toTarget()
                    end
                end
]]
            end
            car.time = ctime
        end
    end

    if im.IsMouseClicked(0) and inView() and rayCast then
            lo('?? D.click:'..tostring(rayCast.pos)..':'..tostring(out.pdrag)..':'..tostring(cmover)..':'..tostring(cmover and scenetree.findObjectById(cmover):getOrCreatePersistentID() or nil))
        if editor.keyModifiers.shift then
            D.setCar(rayCast.pos)
        end
        if editor.keyModifiers.alt then
            if out.aplot and not inmerge then
                    lo('?? to_MERGE:'..#out.aplot)
                for _,d in pairs(out.aplot) do
    --                U.dump(d, '?? merging:'.._)
                    branchMerge(adec[amatch[d.fr].ind], adec[amatch[d.to].ind])
                    amatch[d.fr].skip = true
                    amatch[d.to].skip = true
                    amatch[d.fr].match = true
                    amatch[d.to].match = true
--                    branchMerge(adec[d.fr], adec[d.to])
                end
                inmerge = true
                out.aplot = nil
--[[
            else
                if false and cmover and cdescmo then
                    local rd = adec[cdescmo]
                    local p = U.proj2D(rayCast.pos)
                    local dl,pl = toSide(p, rd, 'left')
                    local dr,pr = toSide(p, rd, 'right')
                    local ps = dl < dr and pl or pr
                    local side = dl<dr and 'left' or 'right'
                    if out.sidepick then
                        forExitFree(adec[out.sidepick.ind], rd, out.sidepick.p, ps, out.sidepick.side, side)
                        out.sidepick = nil
                            lo('?? tgt:'..rd.ind..':'..side)
                        return rd.ind
                    else
                        out.sidepick = {
                            p=ps+vec3(0,0,forZ(ps)),
                            ind=cdescmo,
                            side=side}
                    end
--                        lo('?? if_SIDE:'..tostring(cmover)..':'..tostring(cdescmo)..':'..dl..':'..dr)
--                        toMark({ps},'blue')
                end
]]
            end
--!!            D.setCar(rayCast.pos)
--            veh:setPositionNoPhysicsReset(rayCast.pos)
--            scenetree.findObject("thePlayer"):setPosition(vec3(x, y, core_terrain.getTerrainHeight(vec3(x,y))))
--        elseif editor.keyModifiers.ctrl then -- and out.injunction then --not indrag and U._MODE ~= 'conf' then
        elseif editor.keyModifiers.ctrl and U._PRD == 0 and U._MODE ~= 'conf' then
--        elseif editor.keyModifiers.ctrl and U._PRD == 0 then -- and not indrag and U._MODE ~= 'conf' then
                lo('?? for_junc:')
            cjunc = D.junctionUp(rayCast.pos, default.nbranch)
                    return 1
        elseif out.pdrag and not U._MODE == 'conf' then
                lo('?? for_cj:'..tostring(out.pdrag.jind)..':'..tostring(indrag))
            cjunc = out.pdrag.jind
            out.jdesc = ajunc[cjunc]
            out.pdrag = nil
                out.acyan = {ajunc[cjunc].p}
        elseif false and rayCast and veh then
            car.target = {p = rayCast.pos}
            car.pos = veh:getPosition()
            local ang = -U.vang(car.vel or vec3(-1,0,0), car.target.p-car.pos, true)
--            local ang = -U.vang(car.pos, car.target, true)
            toTarget(ang)
--            local cvel = veh:getVelocity()
--            veh:queueLuaCommand('ai.driveToTarget("'..car.target.x..'_'..car.target.y..'_'..cpos.x..'_'..cpos.y..'_'..cvel.y..'_'..cvel.y..'",0.2,0,5,"1")')
    --        input.event("throttle", 2, "FILTER_AI", nil, nil, nil, "ai")
    --        guihooks.trigger("AIStateChange") --, getState())
                lo('?? to_input:'..tostring(cpos)..':'..tostring(veh:getVelocity()))
        elseif false then
            -- detect junction hit
            local dmi,imi = math.huge
            for i,d in pairs(ajunc) do
                if rayCast.pos:distance(d.p) < 3 then
                        lo('?? junc_hit:'..tostring(i)..':'..tostring(cjunc))
                    if cjunc == i then
                        cjunc = nil
                        out.acyan = nil
                        out.injunction = nil
                        out.jdesc = nil
                    else
                        cjunc = i
                        cpick = nil
                        apick = {}
                        out.injunction = cjunc
                        out.jdesc = ajunc[cjunc]
                        out.acyan = {d.p}
                        out.ared = nil
                        out.agreen = nil
--                                onVal('b_junction')
--                                onVal('b_conform')
--                                junctionRound()
                    end
                    break
                end
            end
        end
--            lo('?? clicked_cj:'..cjunc)
    end

    if U._MODE == 'conf' and im.IsMouseClicked(0) and inView() then -- not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not im.IsAnyItemHovered() then
        if not rayCast or rayCast.object.name ~= 'theTerrain' then return end
            lo('?? decal_CLICK:'..':'..tostring(rayCast.object.name)..':'..tostring(cmover)..':'..tostring(cpick)..'/'..tostring(croad)..':'..tostring(rayCastHit)..':'..tostring(im.IsWindowHovered(im.HoveredFlags_AnyWindow))..':'..tostring(im.IsAnyItemHovered()))
        if cmover and not cjunc then
            -- check hitting node
            local nodehit
            if cpick and croad then
                local anode = editor.getNodes(croad)
                for i,n in pairs(anode) do
                    if rayCastHit and (rayCastHit - n.pos):length() < 1 then
                            lo('?? for_e:'..tostring(n.edges))
                        nodehit = i
                        if anodesel[i] then
                            anodesel[i] = nil
                        else
                            anodesel[i] = true
                        end
                        break
                    end
--                    lo('?? if_hit:'..i..':'..tostring((rayCastHit - n.pos):length()))
                end
            end
                lo('?? if_rc:'..tostring(nodehit)..':'..tostring(cmover)..':'..tostring(rayCastHit))
            if false and nodehit == nil and rayCastHit then
                if cmover == cpick then
                    -- deselect
                    cpick = nil
                    out.apicknode = nil
                    cmover = cpick
                        lo('?? desel:', true)
                else
                    -- new pick
                    if editor.keyModifiers.ctrl then
                        local ind = cdescmo or rdhit
                        local inpick = U.index(apick,ind)
                        if #inpick == 0 then
                            apick[#apick+1] = ind
                        else
                            table.remove(apick, inpick[1])
                        end
                        out.ared = {}
                        out.agreen = {}
                        for _,ind in pairs(apick) do
--                                lo('?? ind:'..tostring(ind))
                            local rd = adec[ind]
                            out.ared[#out.ared+1] = rd.list[1]
                            out.ared[#out.ared+1] = rd.list[#rd.list]
                        end
                            U.dump(out.ared, '?? ared:')
                            lo('??**************** D.onUp_DONE:'..tostring(cdescmo)..':'..#apick)
                        if cdescmo then return cdescmo or 0 end
                    else
                        apick = {}
                        cpick = cmover
                        out.inall = false
                        out.apicknode = out.anode
                        cdesc = cdescmo
                        out.anode = nil
                        if #U.index(conformed, cpick) == 0 then
                            out.inconform = false
                        else
                            out.inconform = true
                        end
                        return cdesc
                    end
--                    out.avedit = {}
--                    out.apick = nil
                    out.apoint = nil
--                    D.ui.laneR = 23
--[[
                    D.ui.laneL = 1
                    laneR = 1,
                    middleYellow = false,
                    middleDashed = false,
]]

                    croad = scenetree.findObjectById(cpick)
                            local ind = inAdec(cpick)
                            U.dump(across[ind], '?? picked:'..tostring(ind)..':'..tostring(cpick)..':'..tostring(croad)..':'..tostring(croad and croad:getOrCreatePersistentID() or nil))
    --                croad:setField("material", 0, "road1_concrete")
                end
                out.avedit = {}
                out.apick = nil
                anodesel = {}
            end
--                forRoad()
--            local rd = scenetree.findObjectById(roadID)
--                lo('?? PI:'..tostring(rd and rd:getOrCreatePersistentID())) -- getField('material')) or) --rd.persistentId))

                lo('??****** seled:'..tableSize(anodesel)..':'..#anodesel..' cdesc:'..tostring(cdesc)..':'..tostring(nodehit)..':'..tostring(cpick)..':'..#apick)
        elseif editor.keyModifiers.ctrl then
            return 0
        end
        return cmover
    end

    --- KEYBOARD
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_RightArrow)) then
		incommand = {'right'}
--		incommand = {'exit_right'}
	end
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_LeftArrow)) then
        incommand = {'left'}
--        incommand = {'turn_left'}
--		W.keyUD(-1)
	end
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_UpArrow)) then
		keyUD(1)
	end
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_DownArrow)) then
		keyUD(-1)
	end
    if im.IsKeyPressed(im.GetKeyIndex(im.Key_Delete)) then
        if cjunc then
            junctionDown(cjunc)
        end
    end
    if im.IsKeyPressed(im.GetKeyIndex(im.Key_Enter)) then
        rdlist = editor.getAllRoads()
            lo('?? new_list:'..tostring(tableSize(rdlist)))
    end
    if cpick and U._PRD == 0 and U._MODE ~= 'conf' then
        if im.IsKeyPressed(im.GetKeyIndex(im.Key_UpArrow)) then
            if tableSize(anodesel) == 1 and anodesel[1] then
                local ndi = 1
                lo('?? to_EXTEND:'..cpick)

                -- merge roads via extend
                for i,rd in pairs(adec) do
                    if rd.id == cpick then
                        cdec = i
                        local astem = U.clone(across[cdec][ndi])
                        astem[#astem + 1] = {i, ndi}
                            U.dump(across[cdec][ndi], '?? stem:') --across[cdec], '?? acr:'..i)
                        local star,us = U.forStar(astem, adec)
--                            U.dump(star, '?? star_got:'..i)
                        for _,f in pairs(star) do
                            if f.rdi == i then
                                us = f
                                break
                            end
                        end
                        local ami,fmi = math.huge
                        for _,f in pairs(star) do
                            if f.rdi ~= us.rdi then
                                local da = math.abs(math.pi - math.abs(us.ang-f.ang))
                                if da < ami then
                                    ami = da
                                    fmi = f
                                end
--                                lo('?? ang:'..f.rdi..':'..math.abs(math.pi - math.abs(us.ang-f.ang)))
                            end
                        end
                        -- add extra node
                        local w = adec[fmi.rdi].body:getNodeWidth(fmi.ndi - 1)
                            lo('?? mina:'..fmi.rdi..':'..ami..':'..w..':'..cdec)
                        local nodeInfo = {
                            pos = fmi.next,
                            width = w, drivability = 1, index = 0 }
--                            U.dump(across[fmi.rdi], '?? POST_cross:')

--                        for k,lnk in pairs(across[fmi.rdi][])

                        if true then
                            editor.addRoadNode(cpick, nodeInfo)

                            -- update crossings
                            --- from us
                            U.dump(across[cdec][ndi], '?? to_checkll:')
                            local abranch,irem = {}
                            for k,lnk in pairs(across[cdec][ndi]) do
                                    lo('?? to_check:'..k..':'..lnk[1])
                                if lnk[1] == fmi.rdi then
                                    lo('?? to_drop:'..lnk[1])
                                    irem = k
--                                    arem[#arem + 1] = k
--                                    table.remove(across[cdec][ndi], k)
    --                                across[cdec][ndi][k] = nil
                                else
                                    abranch[#abranch + 1] = lnk[1]
                                end
                            end
                            if irem then
                                table.remove(across[cdec][ndi], irem)
                            end
--                                U.dump(across[cdec][ndi], '?? POST_rem:')
--                                U.dump(abranch, '?? to_clean:')
                            --- from theirs
                            for _,rdi in pairs(abranch) do
--                                    U.dump(across[rdi], '?? to_up:'..rdi)
                                for n,alnk in pairs(across[rdi]) do
                                    for k,lnk in pairs(alnk) do
                                        if lnk[1] == cdec then
--                                            U.dump(alnk, '?? TO_update:'..cdec)
                                            lnk[2] = lnk[2] + 1
--                                            table.remove(alnk, k) --] = nil
                                            break
                                        end
                                    end
                                end
--                                    U.dump(across[rdi], '?? post_up:'..rdi)
                            end
                            for n,alnk in pairs(across[fmi.rdi]) do
                                for k,lnk in pairs(alnk) do
                                    if lnk[1] == cdec then
                                        table.remove(alnk, k) --] = nil
                                        break
                                    end
                                end
                            end

                                    table.insert(out.apicknode, 1, fmi.next)
                                    out.avedit = {}
                                    local anode = editor.getNodes(croad)
                                    out.avedit = {anode[1].pos}
                        end
--[[
]]
--[[
                        if across[cdec] then
                            for n,lst in pairs(across[cdec]) do
                                lo('?? for_cross:'..n)
                                acp[#acp + 1] = n
                                out.avedit[#out.avedit+1] = adec[cdec].list[n]
                            end
                        end
]]
                        break
                    end
                end
            else
                -- change road width
                if not aedit[cpick] then
                    aedit[cpick] = {body = croad, w = croad:getNodeWidth(0)}
                end
                local anode = editor.getNodes(croad)
    --            lo('?? key_R1:'..anode[1].width..':'..croad:getNodeWidth(1))
                for i,_ in pairs((tableSize(anodesel) > 0) and anodesel or anode) do
    --                    lo('?? to_w:'..i)
                    editor.setNodeWidth(croad, i-1, croad:getNodeWidth(i-1) + 1)
                end
                editor.updateRoadVertices(croad)
    --            lo('?? key_R2:'..#anode..':'..anode[1].width..':'..croad:getNodeWidth(0))
    --            update(croad)
    --            anode = editor.getNodes(croad)
                    lo('?? key_R3:'..#anodesel..'/'..#anode..':'..anode[1].width..':'..croad:getNodeWidth(0))
    --            restore()
                ter2road()
                if croad:getField("material", "") ~= 'WarningMaterial' then
                    middleUp()
                    sideUp()
                end
            end
        elseif im.IsKeyPressed(im.GetKeyIndex(im.Key_DownArrow)) then
            local anode = editor.getNodes(croad)
            for i,_ in pairs((tableSize(anodesel) > 0) and anodesel or anode) do
--            for i,n in pairs(anode) do
                editor.setNodeWidth(croad, i-1, croad:getNodeWidth(i-1) - 1)
            end
            editor.updateRoadVertices(croad)
--            restore()
            ter2road()
            if croad:getField("material", "") ~= 'WarningMaterial' then
                middleUp()
                sideUp()
            end
        elseif im.IsKeyPressed(im.GetKeyIndex(im.Key_RightArrow)) then
            -- mantle width up
            mantle = mantle + 0.4
            ter2road()
        elseif im.IsKeyPressed(im.GetKeyIndex(im.Key_LeftArrow)) then
            -- mantle width down
            mantle = mantle - 0.4
            ter2road()
        end
    end

--    local ncolour = (cpick and cmover == cpick) and ColorF(0,1,1,0.7) or ColorF(1,1,0,0.5)

    local function sphere(pos, r, c)
        r = r*math.sqrt((pos-core_camera.getPosition()):length())
        debugDrawer:drawSphere(pos, r, c) -- ColorF(1,1,0,0.4))
    end

    local function line(pfr, pto, c, w)
        debugDrawer:drawLine(pfr, pto, c, w)
    end

    -- MARKING
    for key,l in pairs(legend) do
        if out[key] then
            local c = legend[key][1]
            c.alpha = legend[key][3]
            for i,p in pairs(out[key]) do
                sphere(p, legend[key][2], c)
            end
        end
    end

    local rbase = 0.1
    if out.aplot then
        -- render links
        for i,d in pairs(out.aplot) do
            Render.path(d.e, color(255,255,0,155), 4)
        end
    end
    if out.inplot then
--        Render.path(out.inplot, color(255,255,0,155), 4)
    end
    if out.sidepick then
--            lo('?? out.sidepick'..'')
        if rayCast then
            if out.sidepick.list then
                for i=1,#out.sidepick.list-1 do
                    debugDrawer:drawLine(out.sidepick.list[i], out.sidepick.list[i+1], ColorF(1,1,0,1), 4)
                end
                sphere(out.sidepick.list[#out.sidepick.list], 0.05, ColorF(0,1,1,0.8))
                debugDrawer:drawLine(out.sidepick.list[#out.sidepick.list], rayCast.pos, ColorF(1,1,0,1), 4)
            else
                debugDrawer:drawLine(out.sidepick.p, rayCast.pos, ColorF(1,1,0,1), 4)
                sphere(out.sidepick.p, 0.1, ColorF(0,1,1,1))
            end
        elseif inView() then
            lo('?? sp_NO_rc:')
        end
    end
    if out.aseg then
        for i,e in pairs(out.aseg) do
--            U.dump(e, '?? for_eg:'..#out.aseg)
--            Render.path(e, color(255,255,0,155), 1)
--            debugDrawer:drawLine(e[1], e[2], ColorF(1,1,0,1), 4)
            debugDrawer:drawLine(e[1], e[2], ColorF(0,0,1,1), 4)
        end
    end
--    if U._PRD == 0 and cjunc and ajunc[cjunc] and ajunc[cjunc].r then
    if cjunc and ajunc[cjunc] and ajunc[cjunc].r then
        R.circle(ajunc[cjunc].p+vec3(0,0,1), ajunc[cjunc].r, color(255,255,0,255), 4)
    end
    if U._PRD==0 and out.pdrag then
        sphere(out.pdrag.p, 0.2, ColorF(1,1,1,0.4))
--        Render.path(out.pdrag.dir, color(255,255,250,155), 8)
--        debugDrawer:drawLine(vec3(0,0,0), vec3(10,0,0), c, w)
--        line(out.pdrag.dir[1], out.pdrag.dir[2], ColorF(1,0,0,0.4), 14)
    end
    if out.acirc then
        for _,d in pairs(out.acirc) do
            R.circle(d.p, d.r, color(255,255,0,255), 4)
        end
    end
    if out.atri then
        for _,t in pairs(out.atri) do
           debugDrawer:drawTriSolid(t[1],t[2],t[3],color(255,255,220,200))
        end
    end
    if out.ared ~= nil then
        for _,p in pairs(out.ared) do
            sphere(p, 2*rbase, ColorF(1,0,0,0.4))
        end
    end
    if out.agreen ~= nil then
        for _,p in pairs(out.agreen) do
            sphere(p, 2*rbase, ColorF(0,1,0,0.4))
        end
    end
    if cjunc and ajunc[cjunc] then
        out.acyan = {ajunc[cjunc].p}
    end
    if out.acyan ~= nil then
        for _,p in pairs(out.acyan) do
            sphere(p, rbase, ColorF(0,1,1,0.4))
        end
    end
    if out.anode ~= nil then
        for _,p in pairs(out.anode) do
            local r = 0.1*math.sqrt((p-core_camera.getPosition()):length())
            debugDrawer:drawSphere(p, r, ColorF(1,1,0,0.3))
        end
--    end
    end
    if out.apicknode ~= nil then
        for i,p in pairs(out.apicknode) do
            local r = 0.1*math.sqrt((p-core_camera.getPosition()):length())
            local clr = anodesel[i] and ColorF(1,0,1,0.6) or ColorF(0,1,1,0.6)
            debugDrawer:drawSphere(p, r, clr)
        end
    end
    if out.avedit ~= nil and #out.avedit>0 then
--            lo('?? onUp_avedit:'..tostring(out.avedit[1]))
--            _dbdrag = true
        for _,s in pairs(out.avedit) do
            if s and s.x then
                local r = 0.05*math.sqrt((s-core_camera.getPosition()):length())
    --                r = 2
    --            sphere(s, r, ColorF(1,1,0,1))
                debugDrawer:drawSphere(s, r, ColorF(1,1,0,1))
            end
--            debugDrawer:drawSphere(s, r, ColorF(1,1,1,0.5))
        end
    end
--    if cpick then return true end

--        lo('?? rdlist:'..tostring(aiRoadsSelectable)..':'..hasroads)
end

--[[
            if #anodesel > 0 then
                for i,_ in pairs(anodesel) do
                    editor.setNodeWidth(croad, i-1, croad:getNodeWidth(i-1) + 1)
                end
            else
                for i,n in pairs(anode) do
                    if true or i < 3 then
                        editor.setNodeWidth(croad, i-1, croad:getNodeWidth(i-1) + 1)
                    end
    --                n.width = n.width + 0.5
                end
            end
]]


    lo('?? adec2:'..#adec)

D.decalsLoad = decalsLoad
D.forRoad = forRoad
D.junctionUp = junctionUp
D.ter2road = ter2road
D.restore = restore
D.matApply = matApply
D.laneSet = laneSet
D.widthRestore = widthRestore
--D.forStar = forStar
D.middleUp = middleUp
D.sideUp = sideUp
D.node2edge = node2edge
--D.forLanes = forLanes
D.unselect = unselect

D.onUpdate = onUpdate
D.onSelect = onSelect
--D.onEditorObjectSelectionChanged = onEditorObjectSelectionChanged

D.out = out

return D