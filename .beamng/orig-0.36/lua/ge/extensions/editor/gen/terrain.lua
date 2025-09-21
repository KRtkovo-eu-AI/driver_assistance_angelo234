local T = {
    out = {},
}

local im = ui_imgui
local ffi = require("ffi")

local U = require('/lua/ge/extensions/editor/gen/utils')
local R = require('/lua/ge/extensions/editor/gen/render')
local D
local lo = U.lo

lo('= TERRAIN:')

local dbc = {} -- {1,1}

local groupEdit = scenetree.findObject('edit')
local tb = extensions.editor_terrainEditor.getTerrainBlock()
local tersize, tfr, grid
if tb then
    grid = tb:getSquareSize()
    tersize = tb:getObjectBox():getExtents()/grid
    tfr = tb:getWorldBox().minExtents
        lo('?? tersize:'..tostring(tersize))
end
local mask = {}
local aregion = {}
local adec = {}
local aloop = {}

local default = {
--    DMI_INS = 20, -- minimal distance to cross to insert radial
    M2S_MARG = 16,  -- margin of lattice starts inside of region
    LAT_SPACE = 15, -- lattice roads spacing
    RAND_RATE = 0.2, -- grid random share

    WMI_TOMERGE = 1.24, -- ration of width to extend slice to highway
    SLICE_MIN = 20, -- minimal lattice slice length
    END_MIN = 8, -- minimal length on lattice slice hanging end
}
T.out.default = default
--[[
local DMI_INS = 20 -- minimal distance to cross to insert radial
local M2S_MARG = 8  -- margin of lattice starts inside of region
local LAT_SPACE = 10 -- lattice roads spacing
local SLICE_MIN = 20 -- minimal lattice slice length
local END_MIN = 8 -- minimal length on lattice slice hanging end
]]


T.inject = function(oD)
    D = oD
end


local function forZ(p)
    return core_terrain.getTerrainHeight(p)
end

local deflrad, deflalpha = 0.06,0.6
local acolor = {'red','green','mag','cyan','yel','white','blue'}
local legend = {
    red = {{1,0,0,0.3}, deflrad, deflalpha},
    green = {{0,1,0,0.3}, deflrad, deflalpha},
    mag = {{1,0,1,0.3}, deflrad, deflalpha},
    cyan = {{0,1,1,0.3}, deflrad, deflalpha},
    yel = {{1,1,0,0.3}, deflrad, deflalpha},
    white = {{1,1,1,0.3}, deflrad, deflalpha},
    blue = {{0,0,1,0.3}, deflrad, deflalpha},
}

local function toMark(list, cname, f, r, op, keepz)
    if not list or #list == 0 then return end
    legend[cname][2] = r or 0.04
    if not op then op = legend[cname][3] end
--    legend[cname][3] = op or 0.3
    T.out[cname] = {}
    for i,p in pairs(list) do
        local v = f and f(p) or p
        if v then
            v = vec3(v.x,v.y,v.z)
            if not keepz then
                v.z = forZ(v)
            end
            T.out[cname][#T.out[cname]+1] = v
        end
    end
--        lo('<< toMark:'..cname..':'..#out[cname])
end


local function clear()
        lo('?? Ter.clear:'..#adec)
    for i=#adec,1,-1 do
        T.decalDel(i)
    end
    D.decalsLoad({})
    adec = {}
    aloop = {}
    for key,_ in pairs(legend) do
        T.out[key] = nil
    end
end


local function grid2p(ij)
    return vec3(tfr.x + (ij[2]-1)*grid, tfr.y + (ij[1]-1)*grid)
end

local function terFlat()
        lo('>> terFlat:'..tostring(tb))
--    for i=1
    if not tb then
        tb = extensions.editor_terrainEditor.getTerrainBlock()
    end
        lo('??************************ terFlat:'..tostring(tersize)..':'..tostring(tfr))
    for i=1,tersize.y do
        for j=1,tersize.x do
            tb:setHeightWs(grid2p({i,j}), 0)
        end
    end
    tb:updateGrid()
        lo('<< terFlat:')
end


T.clear = function()
--    terFlat()
    local rdlist = editor.getAllRoads()
    for roadID,_ in pairs(rdlist) do
        local rd = scenetree.findObjectById(roadID)
        if rd then
            rd:setField('hidden',0,'true')
        end
    end
    for _,p in pairs(mask) do
        tb:setHeightWs(p, 0)
    end

    local amat = core_terrain.getTerrain():getMaterials()
    for _, m in ipairs(amat) do -- get all existing materials from terrain
            lo('??^^^^^^^^^^^^^^^^^^^^^^^^^ for_MAT:'..tostring(m:getInternalName())..':'..tostring(m.baseColorBaseTex))
        m.baseColorBaseTex = '/levels/test_art_v2/art/terrains/t_terrain_base_dirt_b.png'
    end
    if not tb then
        tb = extensions.editor_terrainEditor.getTerrainBlock()
    end
    local ifr,ito = -2000,2000
    if false then
        for i=ifr,ito do
            for j=ifr,ito do
                tb:setMaterialIdxWs(vec3(i,j), 7)
            end
        end
        tb:updateGridMaterials(vec3(ifr,ito,0), vec3(ifr,ito,0))
        tb:updateGrid()
    end
        lo('<< Ter.clear:'..tostring(tb:getMaterialIdxWs(vec3(1,1))))
end


local function terSet(p, h)
    local ijs = U.stamp({p.y,p.x}, true)
    mask[ijs] = p
    tb:setHeightWs(p, h)
end

-- freq amplitude - number of wavelets
-- freq duration - wavelet height
--local T = 0
local dwave = {} -- frequency, ampitude
local ama -- max amplitude
local f2p = {} -- frequency -> array of wavelets {position,height}
local ampVanish = 4 -- ration of amps to abrupt the frequency
local amp2n = 4 -- coef of prop nwaves = c*amp/ama
local dwc

local function corr(dw1, dw2)
    for f,d in pairs(dw2) do
        if not f2p[f] then f2p[f] = {list={}} end
        local ami = dw1 and dw1[f]/ampVanish or ama/8
        if dw2[f][1] < ami then
            -- frequency ends
            dw2[f][2] = 0
            f2p[f].skip = true
        elseif dw2[f][2] == 0 and dw2[f][1]>ama/5 then --amp2n then
            -- frequency seed
            local nw = math.floor(dw2[f][1]/ama*amp2n+0.5)
            local list = {}
            for i=1,nw do
                list[#list+1] = {vec3(
                    math.random(1,tersize.x),
                    math.random(1,tersize.y)), 1}
            end
            f2p[f] = {list=list}
        else
            -- frequency goes on
            for _,p in pairs(f2p[f].list) do
                p[2] = p[2] + 1
            end
        end
    end
        U.dump(f2p, '<< corr:')
end


local fmi,fma = 100,2000 -- min/max frequencies
local f2r = 0.02  -- r = c*(f-fmi)/(fma-fmi)*tersize

-- f - frequency, p - {center,intensity}
local function waveUp(f, p)
    local r = math.floor(f2r*(fma-f)/(fma-fmi)*tersize.x + 0.5)
        U.dump(p, '?? waveUp:'..f..':'..r)
--    if true then return end
    for i = p[1].y-r,p[1].y+r do
        for j = p[1].x-r,p[1].x+r do
            if i > 0 and j > 0 and i < tersize.y and j < tersize.x then
                local pos = grid2p({i,j})
                local d = math.sqrt((i-p[1].y)^2+(j-p[1].x)^2)
                if d < r then
                    terSet(pos, 20*p[2]*(1-d/r))
--                    tb:setHeightWs(pos, 20*p[2]*(1-d/r))
                end
            end
        end
    end
end


local function wav2ter(dw)
    if not tersize then return end
    corr(dwc, dw)
    dwc = dw
--    for
    for f,d in pairs(f2p) do
        for _,p in pairs(d.list) do
            waveUp(f, p)
        end
    end
    tb:updateGrid()
end


local mask,ravg,iR,grid,c = {}


local function toWorld(j,i)
    return vec3(c.x+(j-iR)*grid,c.y+(i-iR)*grid)
end

local function regionPlot()
        if #aregion == 0 then
            aregion = {
{p=vec3(-80.86,-1760.73), w=0.968},
{p=vec3(-266.25,-1648.27), w=0.968},
{p=vec3(-106.34,-1530.07), w=1.065},
            }
            aregion = {
{p=vec3(-3.17,-1084.04), w=1},
{p=vec3(-75.42,-962.44), w=1},
{p=vec3(182.18,-1095.08), w=1},
            }
--[[
            aregion = {
                {p=2*vec3(-91,-909),w=1},
                {p=2*vec3(-99,-855),w=1},
                {p=2*vec3(-39,-874),w=1},
                {p=2*vec3(-67,-878),w=1},
            }
]]
        end
        lo('>> regionPlot:'..#aregion)
        for i,r in pairs(aregion) do
            aregion[i] = {p=r.p, w=r.w}
        end
            U.dump(aregion, '?? for_areg:')
--        U.dump(aregion, '>> regionPlot:'..#aregion)
    mask = {}
    -- get diameter
    local dma = 0
    local afill = {}
    c = vec3(0,0,0)
        local ap = {}
    for i = 1,#aregion-1 do
        afill[i] = {}
        c = c + aregion[i].p
        for j = i+1,#aregion do
            local d = aregion[i].p:distance(aregion[j].p)
            if d > dma then
                dma = d
            end
        end
            ap[#ap+1] = aregion[i].p
    end
            ap[#ap+1] = aregion[#aregion].p
    afill[#aregion] = {}
    c = c + aregion[#aregion].p
    c = c/#aregion
    -- middle distance
    grid = dma/60
    if grid < 1 then grid = 1 end
    ravg = math.sqrt(dma*dma/#aregion/math.pi)
    local R = 1.5*ravg + dma/2
    iR = math.floor(R/grid+0.5)
    for i = 1,2*iR do
        for j = 1,2*iR do
--    for i = -iR,iR do
--        for j = -iR,iR do
--    for i = c.y - R,c.y + R do
--        for j = c.x - R,c.x + R do
            local p = toWorld(j,i) --c.x+(j-iR)*grid,c.y+(i-iR)*grid
--            ap[#ap+1] = vec3(x,y)
            local wma,ima,pma = 0
--            local dmi,imi = math.huge
            for k,d in pairs(aregion) do
                -- pick strongest
                local dist = p:distance(vec3(d.p.x,d.p.y))
                local w = d.w/dist
                if w > wma then
                    wma = w
                    ima = k
                    pma = toWorld(j,i)
                end
--                if dist < dmi then
--                    dmi = dist
--                    imi = k
--                end
            end
            if ima then
                if pma:distance(U.proj2D(aregion[ima].p)) < 1.8*aregion[ima].w*ravg then
                    if not mask[i] then mask[i] = {} end
                    mask[i][j] = ima
--                    ap[#ap+1] = p
--                if pma:distance(vec3(c.x,c.y)) < R then
--                        lo('?? to_mm:'..i..':'..j..':'..tostring(ima)..':'..tostring(pma))
                    afill[ima][#afill[ima]+1] = pma
                end
            end
        end
    end
--        toMark({c}, 'cyan', nil, 0.04)
--[[
        ap = {}
    for i = 2,2*iR-1 do
        for j = 2,2*iR-1 do
            if mask[i] and mask[i][j] then
                ap[#ap+1] = toWorld(j, i)
            end
        end
    end
]]
--        toMark(ap, 'white', nil, 0.2, 0.5)
    for i,list in pairs(afill) do
        aregion[i].amark = {}
        for _,p in pairs(list) do
            aregion[i].amark[#aregion[i].amark+1] = p+vec3(0,0,forZ(p))
        end
--        if i <= #acolor then
--            toMark(aregion[i].amark, acolor[i], nil, 0.03*dma/100)
--        end
    end
        lo('<< regionPlot:'..dma..':'..tostring(T.out['white'] and #T.out['white'] or nil))
end


local function decalUp(dec)
    D.decalUp(dec, true)
    adec[#adec+1] = dec
    dec.ind = #adec
--        lo('?? T.decalUp:'..dec.ind..'/'..#adec)
end

local function decalDel(ind)
    local rd = adec[ind]
    adec[ind] = nil
    if not rd then return end
    local obj = scenetree.findObjectById(rd.id)
    if obj then
        obj:delete()
    end
--        lo('<< decalDel:'..ind..':'..#adec)
end
T.decalDel = decalDel


local aslice = {}

local function pathHit(path, a, b, inends, margin, dbg)
--        lo('>> pathHit:'..#path..':'..#aslice)
    if not margin then margin = 0 end
    local aseg = {}
    local dhit = {}
    local ahit = {}
--        toMark({path[1].p},'green',nil,0.1,1)
    local function angValid(ang)
        return math.abs(math.pi-math.abs(ang)) > 0.2 and math.abs(ang) > 0.2
    end
    local hasclose
    for i=1,#path do
        local c,d = path[i].p,U.mod(i+1,path).p
        local p,s = U.line2seg(a,b,c,d)
        local ang
        if not p and not dhit[i] then --and not hasclose then
            -- check being close
--            U.dump(path[i], '?? if_close:')
            local dist,pp = U.toLine(path[i].p, {a,b})
                if #ahit == 0 then
--                    toMark({pp},'yel',nil,0.1,1)
                end
            if dist < path[i].w/2+margin/2 and U.vang(U.mod(i+1,path).p-path[i].p, pp-path[i].p, true)>0 then
--                    U.dump(dhit,'??^^^^^^^^^^ is_CLOSE:'..i..':'..tostring(pp)..':'..U.vang(U.mod(i+1,path).p-path[i].p,pp-path[i].p,true)) --..':'..tostring(pp-a)..':'..tostring(d-c)..':'..tostring(path[i].w))
--                    toMark({path[i].p}, 'blue', nil, 0.1, 1)
                    if dbg then
                        lo('??^^^^^^^^^^ is_CLOSE:'..i..':'..tostring(pp)..':'..U.vang(U.mod(i+1,path).p-path[i].p,pp-path[i].p,true)) --..':'..tostring(pp-a)..':'..tostring(d-c)..':'..tostring(path[i].w))
                        toMark({a,b}, 'cyan', nil, 0.1, 1)
                    end
--                    return
                p = pp
                ang = U.vang(b-a, d-c, true)
                local sgn = U.vang(d-c,path[i].p-pp)>0 and -1 or 1
--                local sgn = -1 -- (b-a):dot(d-c) > 0 and -1 or 1
                if angValid(ang) and U.vang(d-U.mod(i-1,path).p,p-U.mod(i-1,path).p,true)>0 then
--                        toMark({c,p},'yel',nil,0.1,1)
                    ahit[#ahit+1] = {p=p, ang=sgn*ang, w=path[i].w, ind=i, isclose = true}
--                        lo('?? hit1:'..tostring(p))
                    hasclose = true
                elseif dbg then
                    lo('?? close_SKIP:')
                end
                ang = U.vang(b-a, c-U.mod(i-1,path).p, true)
                if angValid(ang) then
                    ahit[#ahit+1] = {p=p, ang=sgn*ang, w=path[i].w, ind=i, isclose=true}
--                        lo('?? hit2:'..tostring(p))
                    hasclose = true
                elseif dbg then
                    lo('?? close_SKIP:')
                end
--                ahit[#ahit+1] = {p=p, ang=U.vang(p-a, c-U.mod(i-1,path).p, true), w=path[i].w}
--                ahit[#ahit+1] = {p=p, ang=U.vang(p-a, d-c, true), w=path[i].w}
--                    U.dump(ahit, '?? CLOSE_dhit:'..tostring(b))
                    if #ahit == 3 then
--                        toMark({p}, 'yel', nil, 0.12, 1)
                    end
--                return
            end
        elseif p then
            -- check orientation
            local inout = U.vang(p-a, d-c, true)
--                lo('?? hit:'..i) --..tostring(p)..':'..inout)
            local ang = U.vang(p-a, d-c, true)
            if angValid(ang) then
                ahit[#ahit+1] = {p=p, ang=ang, w=path[i].w, ind=i}
                dhit[i] = true
                dhit[U.mod(i+1,#path)] = true
--                        if #ahit == 1 then
--                            toMark({p}, 'green', nil, 0.12, 1)
--                        end
            end
        end
    end
    table.sort(ahit, function(u,v)
        return (b-a):dot(u.p-a) < (b-a):dot(v.p-a)
    end)
        if dbg then
            toMark(ahit, 'green', function(d) return d.p end, 0.08, 0.6)
            U.dump(ahit, '?? for_HIT:'..#ahit)
            for i,d in pairs(ahit) do
                lo('?? for_HIT:'..i..':'..path[d.ind].seg.ind)
            end
        end
    if #ahit % 2 == 1 then
        return
    end
    for i,h in pairs(ahit) do
        if i>1 and ahit[i-1].isclose then
            -- merge ajacent hits
            ahit[i-1].p = (ahit[i].p+ahit[i-1].p)/2
            ahit[i].p = (ahit[i].p+ahit[i-1].p)/2
        end
    end
    if inends then
--        U.dump(ahit, '?? end_H:')
        return ahit
    end
--       a = ahit[1].p
    for i=1,#ahit,2 do
        local h1,h2 = ahit[i],ahit[i+1]
--            U.dump(h1, '?? hit1:'..i..':'..tostring(ahit[i].p))
--            U.dump(h2, '?? hit2:'..i..':'..tostring(ahit[i].p))
        local list = {
            h1.p + (h1.ang > 0 and -1 or 1)*(1*h1.w/2/math.abs(math.sin(h1.ang))+margin)*(h1.p-a):normalized(),
            h2.p + (h2.ang > 0 and -1 or 1)*(1*h2.w/2/math.abs(math.sin(h2.ang))+margin)*(h2.p-a):normalized()}
        if list[1]:distance(list[2]) > default.SLICE_MIN
            and (list[1]-list[2]):dot(h1.p-h2.p)>0 then
--            aseg[#aseg+1] = list
                if dbg then U.dump(list, '?? hit2slice:'..i) end
            aslice[#aslice+1] = list
--[[
                if #aslice == 1 then
                    lo('?? to_SLICE:'..list[1]:distance(list[2])..':'..#ahit..':'..tostring(a)..':'..tostring(b))
                    toMark({list[1]},'red',nil,0.1,1)
                    toMark({ahit[1].p,ahit[2].p,ahit[3].p,ahit[4].p}, 'yel',nil,0.1,1)
                    toMark({a}, 'green',nil,0.1,1)
                end
]]
        else
--            lo('??^^^^^ skip:')
        end
--        aseg[#aseg+1] = list
--            toMark({a,b}, 'blue', nil, 0.1, 0.8)
--            toMark(list, 'blue', nil, 0.1, 0.8)
--        local dec = {list=list, w=1*D.default.laneWidth, mat='WarningMaterial'}
--        decalUp(dec)
--    for i,p in pairs(ahit) do
    end
--        lo('<< pathHit:'..#ahit..':'..#aslice)

    return aseg
end


local function close(a, b)
    return U.proj2D(a):distance(U.proj2D(b)) < 1
end


local function regionSlice(loop, dir, margin, space, rand_rate)
    if not margin then margin = default.M2S_MARG end
    if not space then space = default.LAT_SPACE end
    if not rand_rate then rand_rate = default.RAND_RATE end
--        lo('>> regionSlice:'..tostring(margin)..':'..tostring(space))
    for i=1,#loop do
        loop[i].p = U.proj2D(loop[i].p)
    end
--        toMark({loop[1].p,loop[1].p+dir},'yel',nil,0.1,1)
    dir = U.proj2D(dir)
    local dnorm = U.vturn(dir, math.pi/2):normalized()
    local dmi,dma,imi,ima = math.huge,0
    -- project loop along dir
    for i,n in pairs(loop) do
--        local p,s =
        local p,s = U.lineHit({n.p, n.p+dir}, {vec3(0,0,0), dnorm})
--        local p,s = U.line2seg(n, n+dir, vec3(0,0,0), dnorm)
--            lo('?? regionSlice:'..tostring(s)..':'..tostring(n)..':'..tostring(n+dir)..':'..tostring(dnorm))
--            if true then return end
        if s then
            if s > dma then
                dma = s
                ima = i
            end
            if s < dmi then
                dmi = s
                imi = i
            end
        end
    end
--        lo('?? regionSlice:'..tostring(dmi)..':'..tostring(imi)..':'..tostring(loop[1]))
    local function forCorner(ind,marg)
        local p = loop[ind].p
--            toMark({p}, 'cyan', nil, 0.3, 1)
        local bfr,bto = U.mod(ind-1,loop).seg,loop[ind].seg
        if bfr.ind == bto.ind then
            local dir = U.mod(ind+1,loop).p - U.mod(ind-1,loop).p
            dir = U.vturn(dir, math.pi/2):normalized()
--                toMark({p, p+5*dir}, 'cyan', nil, 0.2, 1)
            return p+marg*dir
        else
            local dira, dirb = close(p, bfr.list[1]) and 1 or -1, close(p, bto.list[1]) and 1 or -1
--                lo('?? forCorner:'..ind..':'..dira..':'..dirb)
--                lo('?? dir_AB:'..tostring(p)..':'..tostring(bfr.list[1])..':'..dira..':'..dirb)
            return D.borderCross(bfr.body, bto.body
    --            , 'right', 'left', 0, 0, -margin)
                , dira==1 and 'right' or 'left', dirb==1 and 'left' or 'right', dira==1 and 0 or 1, dirb==1 and 0 or 1
                , -marg)

        end
    end
--        lo('?? RR:'..rand_rate)
    aslice = {}
    if dmi then
--            toMark({loop[imi].p,U.mod(imi-1,loop).p}, 'green', nil, 0.1, 1)
        -- get starting point
--            U.dump(loop[imi].seg, '?? if_corner:'..loop[imi].seg.ind..':'..U.mod(imi-1,loop).seg.ind)
--            U.dump(loop[imi].seg, )
        local cb = forCorner(imi, margin)
        if not cb then
            cb = loop[imi].p + (margin + loop[imi].w/2)*U.vturn(U.mod(imi+1,loop).p-U.mod(imi-1,loop).p,math.pi/2):normalized()
        end
        local ce = forCorner(ima, margin)
        if not ce then
            ce = loop[ima].p + (margin + loop[imi].w/2)*U.vturn(U.mod(ima+1,loop).p-U.mod(ima-1,loop).p,math.pi/2):normalized()
        end
--            lo('?? if_CORNER:'..tostring(cb)..':'..tostring(ce))
--            toMark({cb, ce},'blue',nil,0.1,1)
        local nslice = #aslice
        if cb and ce then
            dnorm = (dnorm*(ce - cb):dot(dnorm)):normalized()
            local W = math.abs((cb-ce):dot(dnorm))
            local nstep = math.floor(W/space)
            local step = W/nstep
--                lo('??^^^^^^^^^^^^^ rS.dist:'..W..':'..nstep)
--                toMark({cb,ce},'blue',nil,0.1,1)
    --            toMark({cb,cb+dnorm*10},'blue',nil,0.1,1)
    --            nstep = 1
    --            step = 17
--                nstep = 1
--            for i=nstep,nstep do
--            for i=5,5 do
            for i=0,nstep do
                --!!
                local c = cb+dnorm*step*(i + (dbc and 0 or 1)*U.rand(-rand_rate, rand_rate))
--                    toMark({c, c+dir},'cyan',nil,0.1,1)
                local aseg = pathHit(loop, c, c+dir, false, margin) --, true)
--                if not pathHit(loop, c, c+dir) then
--                    break
--                end
--                cb = cb+dnorm*step
            end
            -- check side margins
--[[
            local cp
            local a,b = aslice[#aslice][1],aslice[#aslice][2]
                lo('?? dSLICE:'..nslice..':'..#aslice..':'..tostring(a))
                local am1 = {}
            for i,d in pairs(loop) do
                local pp,s = U.toLine(d.p, {a,b})
                if pp < margin/2+loop[i].w then
                    lo('?? to_CUT:'..i)
                    am1[#am1+1] = loop[i].p
                end
            end
                    toMark(am1,'yel',nil,0.1,1)
]]
--                lo('?? aslice:'..#aslice)
            return aslice
        else
            lo('!! ERR_NO_CORNERS:'..tostring(cb)..':'..tostring(ce)..':'..tostring(imi)..':'..tostring(ima))
--            if not cb then
--                toMark({loop[imi].p},'red',nil,0.1,1)
--            end
        end
--            toMark({cb,ce},'blue',nil,0.1,1)
--            LAT_SPACE = 7
    end
end
--[[
        local bfr,bto = U.mod(imi-1,loop).seg,loop[imi].seg
        local p = loop[imi].p
        local dira, dirb = close(p, bfr.list[1]) and 1 or -1, close(p, bto.list[1]) and 1 or -1
--            lo('?? dir_AB:'..tostring(p)..':'..tostring(bfr.list[1])..':'..dira..':'..dirb)
        local c = D.borderCross(bfr.body, bto.body
--            , 'right', 'left', 0, 0, -margin)
            , dira==1 and 'right' or 'left', dirb==1 and 'left' or 'right', dira==1 and 0 or 1, dirb==1 and 0 or 1
            , -margin)
        if c then
--            lo('?? fr_to:'..bfr.ind..':'..bto.ind..':'..tostring(c))
                toMark({c, c+dir},'blue',nil,0.1,1)
--            pathHit(loop, c, c+dir)
        end
]]

local ajunc,across

local function sectorPlot(j, i, desc)
    local junc = ajunc[j]
    local lst = junc.list
    local pc = junc.p
--local function sectorPlot(pc, lst, i)
--local function sectorPlot(reg, t, lst, i)
    local b = lst[i]
--        lo('?? sectorPlot:'..tostring(b))
--        U.dump(b, '?? sectorPlot:'..tostring(b))
--        toMark(b.list, 'yel')
    local loop

    local function forNext(br, p)
        if not p then p = loop[#loop].p end
        local list = br.list
        local ifr,ito,dir = 1,#list,1
        if p:distance(list[1]) > 1 then
            ifr,ito,dir = #list,1,-1
        end
--            lo('?? forNext:'..ifr..':'..ito..'/'..#list)
        for i=ifr,ito-dir,dir do
            loop[#loop+1] = {p = list[i], w = br.aw[i], seg = br}
--                lo('?? to_LOOP:'..i..':'..tostring(loop[#loop]))
        end
        return ito
    end
--        toMark(loop,'yel',function(d) return d.p end,0.1,1)

    local function crossNext(dsc, ni)
        local junc = ajunc[ni == 1 and dsc.jfr or dsc.jto]

--            toMark(dsc.list,'yel')
--            lo('?? crossNext:'..dsc.ind..':'..tostring(dsc.jfr)..':'..dsc.jto)
--            local a = junc.list[3].list[1]
--            local b = junc.list[3].list[#junc.list[3].list]
--            lo('?? for_a:'..tostring(a))
--            toMark({a, b}, 'yel', nil, 0.5, 1)
--            toMark({junc.list[1].list[1],junc.list[1].list[#junc.list[1].list]}, 'mag', nil, 0.1, 1)
        if not junc then
            lo('!! ERR_crossNext:'..tostring(ni)..':'..tostring(ni == 1 and dsc.jfr or dsc.jto)..':'..#ajunc)
--            U.dump(dsc, '!! ERR_crossNext:'..tostring(ni)..':'..tostring(ni == 1 and dsc.jfr or dsc.jto)..':'..#ajunc)
            return
        end
        for i,b in pairs(junc.list) do
--                lo('?? for_BR:'..i..':'..b.ind)
            if b.ind == dsc.ind then
--                lo('?? for_NEXT:'..i..':'..U.mod(i-1,#junc.list))
                local br = U.mod(i-1, junc.list)

                return br,forNext(br,dsc.list[ni])
            end
        end
    end

    local function trim(asli2,asli1)
        if not asli2 or not asli1 then return end
        for k,s in pairs(asli2) do
            local L = s[1]:distance(s[2])
            local dmi,dma,lmi,lma,pmi,pma = math.huge,0
            for l,t in pairs(asli1) do
                -- s - distance from s[1]
                local c,s = U.segCross(t[1],t[2],s[1],s[2])
                if c then
                    if s < dmi then
                        dmi = s
                        lmi = l
                        pmi = c
                    end
                    if s > dma then
                        dma = s
                        lma = l
                        pma = c
                    end
    --                                        lo('?? crossed:'..k..':'..l..':'..s)
                end
            end
            if lmi and lma then
    --                lo('?? lmm:'..k..':'..dmi..':'..dma..':'..(dmi*L)..':'..((1-dma)*L))
                if dmi*L < default.END_MIN then
                    -- cut off
                    s[1] = pmi
--                        lo('??************* TRIMMED1:')
                end
                if (1-dma)*L < default.END_MIN then
                    s[2] = pma
--                        lo('??************* TRIMMED2:')
                end
            end
        end
    end

    local dsplit = {}
    local function toExt(s, hit, nsli, dbg)
        if not hit or not hit.ind then
            lo('!! ERR_NO_HIT_IND:')
            return
        end
        hit.p.z = 0
        local seg = loop[hit.ind].seg
        local pe1,pe2 = U.proj2D(seg.list[1]),U.proj2D(seg.list[#seg.list])
        local d1,d2 = hit.p:distance(pe1), hit.p:distance(pe2)
--            lo('??+++++++++++++++++ to_EXT:'..d1..':'..d2)
        if d1>40 and d2>40 then
                if dbg then
                    toMark({seg.list[1],hit.p},'green',nil,0.1,1)
                    lo('?? dd:'..tostring(hit.p)..':'..tostring(seg.list[1]))
                end
--                        toMark({s.list[1]},'green',nil,0.1,1)
            if hit.p:distance(U.proj2D(s.list[1])) < hit.p:distance(U.proj2D(s.list[#s.list])) then
                s.list[1] = hit.p --s.list[1] -- + (a.p-s.list[1])*0.1
            else
                s.list[#s.list] = hit.p
            end
--                lo('?? to_MERGE:') --..'/'..#asli)
            if not dsplit[seg.ind] then dsplit[seg.ind] = {} end
            dsplit[seg.ind][#dsplit[seg.ind]+1] = hit.p
            if nsli > 2 then
                s.w = 2*D.default.laneWidth
            end
--                lo('?? if_SKIp:'..tostring(s.skip))
            s.skip = nil
        end
    end

    local function ext(asli)
        for i,s in pairs(asli) do
            if s.w > default.WMI_TOMERGE*D.default.laneWidth then
                local ahit = pathHit(loop, s.list[1], s.list[#s.list], true)
--                    lo('?? for_HITS:'..#ahit)
--                    toMark({ahit[1].p},'green')
                local ih1,ih2
                if ahit then
                    local dmi,kmi = math.huge
                    for k,h in pairs(ahit) do
                        local dist = h.p:distance(s.list[1])
                        if dist < dmi then
                            dmi = dist
                            kmi = k
                        end
                    end
                    ih1 = kmi
                    dmi,kmi = math.huge
                    for k,h in pairs(ahit) do
                        local dist = h.p:distance(s.list[#s.list])
                        if dist < dmi then
                            dmi = dist
                            kmi = k
                        end
                    end
                    ih2 = kmi
                end
                if ih1 and ih2 then
                    toExt(s, ahit[ih1], #asli)
                    toExt(s, ahit[ih2], #asli)
                end
            end
        end
    end

    local cb,dir,sdir = b
    if not cb then
        lo('!! NO_CB:'..tostring(b))
        return
    end
    if adec[b.ind].isline then
--                            U.dump(b, '?? for_bst:'..k..':'..b.jfr)
        dir = cb.list[#cb.list]-cb.list[1]
    else
        local bbase = U.mod(i+1,lst)
        dir = bbase.list[#bbase.list]-bbase.list[1]
    end
    sdir = dir
    if desc and desc.dir1 then
        dir = U.vturn(dir, 2*math.pi*desc.dir1/360)
    end
-- GET LOOP
--        lo('?? if_line:'..i..':'..tostring(adec[b.ind].isline)..':'..tostring(dir))
--                        loop = {{p=reg.p, w=b.aw[k==b.jfr and 1 or #b.aw], seg=b}}
    loop = {}
    local ito = forNext(cb,pc)
            local n = 0
    while cb and  cb.list[ito]:distance(pc) > 1 and n < 1000 do
        cb,ito = crossNext(cb, ito)
--                                lo('?? for_nxtl:'..cb.ind..':'..ito)
            n = n+1
    end
    local margin,grid,rrate = default.M2S_MARG, default.LAT_SPACE, default.RAND_RATE
    if desc then
        -- update
--        desc.list = loop
        margin = desc.margin
        grid = desc.grid
        rrate = desc.randrate
    else
        -- create
        aloop[#aloop+1] = {list=loop, junc=j, ind=i, margin=margin, grid=grid, randrate = rrate}
    end
-- SLICE DOMAIN
    local aslice = {}
    local asli1 = regionSlice(loop, dir, margin, grid, rrate) or {}
--[[
                lo('?? for_SLI1:'..tostring(asli1 and #asli1 or nil)..':'..#loop)
            for _,s in pairs(asli1) do
                local desc = {list=s, w=(1+U.rand(-0.2,0.4))*D.default.laneWidth, mat='WarningMaterial', skip=true}
                decalUp(desc)
            end
            if true then return end
]]
    local dir2 = U.vturn(sdir, math.pi/2)
    if desc and desc.dir2 then
        dir2 = U.vturn(dir2, 2*math.pi*desc.dir2/360)
    end
--        lo('?? dir_B:')
    local asli2 = regionSlice(loop, dir2, margin, grid, rrate)
--[[
        for _,s in pairs(asli2) do
            local desc = {list=s, w=(_==1 and 1.4 or 1)*D.default.laneWidth, mat='WarningMaterial', skip=true}
--            local desc = {list=s, w=(1+U.rand(-0.2,0.4))*D.default.laneWidth, mat='WarningMaterial', skip=true}
    --        decalUp(desc)
            aslice[#aslice+1] = desc
        end
        ext(aslice)
        for _,s in pairs(asli2) do
            local desc = {list=s, w=(1+U.rand(-0.2,0.4))*D.default.laneWidth, mat='WarningMaterial', skip=true}
            decalUp(desc)
        end
        if true then return end
]]
--                            toMark(asli2[1],'yel',nil,0.1,1)
--        lo('?? for_ASLI:'..#asli1..':'..#asli2)
    -- cut ends
    trim(asli2,asli1)
    trim(asli1,asli2)
    local aslice,cw = {}
    local hasexit
    if asli1 and #asli1 then
        hasexit = false
        for i,s in pairs(asli1) do
            if i>1 and aslice[#aslice].w > default.WMI_TOMERGE*D.default.laneWidth then
                cw = U.rand(1-0.2,default.WMI_TOMERGE)
            else
                cw = 1+U.rand(-0.2,0.4)
            end
--                cw = _==1 and 1.4 or 1
            if cw > default.WMI_TOMERGE then
                hasexit = true
            end
            local desc = {list=s, w=cw*D.default.laneWidth, mat='WarningMaterial', skip=true}
    --        decalUp(desc)
            aslice[#aslice+1] = desc
        end
        if not hasexit and #aslice>0 then
--                lo('??^^^^^^^ TO_EXIT1:'..desc.w)
            local desc = aslice[math.floor(#aslice/2+0.5)]
            desc.w = (default.WMI_TOMERGE + 0.05)*D.default.laneWidth
        end
    else
        lo('!! NO_SLICE1:')
    end
    --!!
    if asli2 and #asli2 then
        hasexit = false
        for i,s in pairs(asli2) do
            if i>1 and aslice[#aslice].w > default.WMI_TOMERGE*D.default.laneWidth then
                cw = U.rand(1-0.2,default.WMI_TOMERGE)
            else
                cw = 1+U.rand(-0.2,0.4)
            end
--            cw = 1+U.rand(-0.2,0.4)
            if cw > default.WMI_TOMERGE then
                hasexit = true
            end
            local desc = {list=s, w=cw*D.default.laneWidth, mat='WarningMaterial', skip=true}
    --        decalUp(desc)
            aslice[#aslice+1] = desc
        end
        if not hasexit and #aslice>0 then
--                lo('??^^^^^^^ TO_EXIT2:')
            local desc = aslice[math.floor(#aslice/2+0.5)]
            desc.w = (default.WMI_TOMERGE + 0.05)*D.default.laneWidth
        end
    else
        lo('!! NO_SLICE2:')
    end
    ext(aslice)
    --TODO: multiple split
--        U.dump(dsplit, '?? for_SPLIT:'..#dsplit)

    for i,desc in pairs(aslice) do
        decalUp(desc)
    end
    -- render
    if desc then
--            lo('?? if_ASL:'..tostring(desc.aslice))
        desc.aslice = aslice
    else
        aloop[#aloop].aslice = aslice
    end
end
--[[
                local a = ahit[1]
                local seg = loop[a.ind].seg
                local d1,d2 = a.p:distance(seg.list[1]), a.p:distance(seg.list[#seg.list])
                    lo('??+++++++++++++++++ to_EXT:'..d1..':'..d2)
                if d1>40 and d2>40 then
--                        toMark({s.list[1]},'green',nil,0.1,1)
                    if a.p:distance(s.list[1]) < a.p:distance(s.list[#s.list]) then
                        s.list[1] = a.p --s.list[1] -- + (a.p-s.list[1])*0.1
                    else
                        s.list[#s.list] = a.p
                    end
                        lo('?? to_MERGE:'..i..'/'..#asli)
                    if not dsplit[seg.ind] then dsplit[seg.ind] = {} end
                    dsplit[seg.ind][#dsplit[seg.ind]+1] = a.p
--                    asplit[#asplit+1] = {p=a.p, seg=seg}
                    --TODO: rebuild loop
--                        toMark({a.p},'blue',nil,0.1,1)
                end
]]
--[[
                    local pair = D.decalSplit(seg, a.p)
                    adec[#adec+1] = pair[1]
                    pair[1].ind = #adec
                    adec[#adec+1] = pair[2]
                    pair[2].ind = #adec
]]
--                    U.dump(s, '??+++++++++++++++++ to_EXT:')
--                    U.dump(ahit, '??+++++++++++++++++ to_EXT:'..d1..':'..d2)
--[[
                    toMark(ahit, 'blue', function(d)
--                        return loop[d.ind].p
                        return d.p
                    end, 0.1, 1)
                    toMark(ahit, 'yel', function(d)
                        return loop[d.ind].p
--                        return d.p
                    end, 0.1, 1)
]]
--[[
        for t,v in pairs(loop) do
            lo('?? in_LOOP:'..t..':'..tostring(v.p)..':'..tostring(v.w))
            if not v.w then
                U.dump(v, '?? no_W:'..t)
                toMark({loop[1].p}, 'cyan', nil, 0.1, 1)
                return
            end
        end
]]
--        lo('?? if_LOOP:'..#loop..':'..n)
--                            U.dump(loop,'?? if_LOOP:'..#loop)
--                        table.remove(loop,#loop)


local function roadPlace()
        lo('>> roadPlace:'..tableSize(mask)..':'..iR)
--        U.dump(mask)
        local am1 = {}
    T.out.inplace = true
    for i,r in pairs(aregion) do
        r.amark = nil
    end
        U.dump(aregion, '?? roadPlace_areg:')
        local outputFile = io.open('./tmp/reg_save_'..tostring(os.clock())..'.json', "w")
        lo('??^^^^^^^^^^^^^^^^ if_FILE:'..tostring(outputFile))
        if outputFile then
            local jdata = jsonEncode(aregion)
            outputFile:write(jdata)
            outputFile:close()
        end
    -- find nodes
    local dnode = {}
    for i = 2,2*iR-1 do
        for j = 2,2*iR-1 do
            if not mask[i] or not mask[i][j] then goto continue end
--                        am1[#am1+1] = toWorld(j,i)
            local cr = mask[i][j]
--                    lo('?? for_cr:'..i..':'..j..':'..tostring(cr))
            -- check the neighbourhood
            local ari = {cr}
            for a=-1,1 do
                for b = -1,1 do
                    if a ~= 0 and b ~= 0 then
                        local val = mask[i+a] and mask[i+a][j+b] or 0
                        if #U.index(ari,val) == 0 then
                            ari[#ari+1] = val
--                            if #ari == 3 then
--                                U.dump(ari, '?? to_ARI:'..i..':'..j..':'..tostring(cr)..':'..val)
--                                am1[#am1+1] = toWorld(j,i)
--                            end
                        end
                    end
                end
            end
            if #ari == 3 then
                local stamp = U.stamp(ari)
                if not dnode[stamp] then dnode[stamp] = {{},{}} end
                local v = vec3(j,i)
                local ord = 1
                if #dnode[stamp][1] > 0 and v:distance(dnode[stamp][1][1]) > 3 then
                    ord = 2
                end
                dnode[stamp][ord][#dnode[stamp][ord]+1] = vec3(j,i)
--                    U.dump(ari, '?? for_cross:')
            end
            ::continue::
        end
    end
--        U.dump(dnode, '?? for_dnode:')
        local ap = {}
    local anode = {}
    for key,seg in pairs(dnode) do
        for i=1,2 do
            if #seg[i] > 0 then
                local p = vec3(0,0,0)
                for _,e in pairs(seg[i]) do
                    p = p + e
                end
                p = p/#seg[i]
                p = toWorld(p.x,p.y)
                    ap[#ap+1] = p -- toWorld(p.x,p.y)
                seg[i] = p
                anode[#anode+1] = {p=p, key=U.split(key,'_')}
            else
                seg[i] = nil
            end
        end
    end
--        U.dump(anode, '?? pre_merge_ap:')
--        toMark(ap, 'cyan', nil, 0.2)
--        toMark(am1, 'yel', nil, 0.05, 0.1)
    -- merge close nodes
    local dmin = 15
    local goon = true
        local n = 0
    while goon and n<10 do
        goon = false
        for i=1,#anode-1 do
            for j =i+1,#anode do
--                    lo('?? if_merge:'..i..':'..j..':'..tostring(anode[i].p)..':'..tostring(anode[j].p)..':'..anode[i].p:distance(anode[j].p))
                if anode[i].p:distance(anode[j].p)<dmin then
                    -- merge
                    U.union(anode[i].key, anode[j].key)
                    table.remove(anode, j)
                    goon = true
                        U.dump(anode[i], '??++++++++++++++++++++++++ merged:')
                    break
                end
            end
            if goon then break end
        end
        n = n + 1
    end
        U.dump(anode, '?? post_merge_ap:')
--        toMark(anode,'red',function(d) return d.p end)
    -- build stars
    for i,r in pairs(aregion) do
        r.star = {} -- indexes of relevant nodes
        for k,n in pairs(anode) do
            if #U.index(n.key,i) > 0 then
                r.star[#r.star+1] = k
            end
        end
        -- order by angle
        table.sort(r.star, function(a, b)
            return U.vang(anode[a].p-r.p,vec3(1,0), true) > U.vang(anode[b].p-r.p,vec3(1,0), true)
        end)
--[[
            if i == 4 then
                for _,k in pairs(r.star) do
                    lo('?? for_ang:'..k..':'..U.vang(anode[k].p-r.p, vec3(1,0),true))
                end
            end
]]
--            U.dump(r.star, '?? for_star:'..i)
    end
-- BUILD SEGMENTS
    local aseg = {}
    for i,r in pairs(aregion) do
        for k,ni in pairs(r.star) do
            local nito = U.mod(k+1, r.star)
            local stamp = U.stamp({ni, nito}, true)
            local sinv = U.stamp({nito, ni}, true)
--                lo('?? if_STAMP:'..i..':'..stamp..':'..tostring(aseg[stamp]))
            if aseg[sinv] and #U.index(aseg[sinv].areg,i)==0 then
                aseg[sinv].areg[#aseg[sinv].areg+1] = i
--                aseg[sinv] = aseg[sinv] + 1
            else
                if not aseg[stamp] then aseg[stamp] = {areg = {}} end
                aseg[stamp].areg[#aseg[stamp].areg+1] = i
            end
        end
--            break
    end
        U.dump(aseg, '?? for_ASEG:')
    -- place decals
    for key,s in pairs(aseg) do
        --- add segment
        local ai = U.split(key,'_')
        local ni,nito = ai[1],ai[2]
--            lo('?? for_fr_to:'..ni..':'..nito)
        local pth = {anode[ni].p}
        if #s.areg == 1 then
            ---- check angles
            local r = aregion[s.areg[1]]
            local ang = U.vang(anode[ni].p-r.p, anode[nito].p-r.p, true)
            if ang < 0 then ang = 2*math.pi + ang end
            if ang > math.pi/3 then
--                    lo('?? to_split:'..key)
                local nstep = math.ceil(ang/math.pi*3)
                local astep = ang/nstep
                for ia=1,nstep-1 do
                    pth[#pth+1] = r.p + 1.5*ravg*r.w*U.vturn(pth[1]-r.p, ia*astep):normalized()
                end
--                pth[#pth+1] = anode[nito].p
            end
        end
        pth[#pth+1] = anode[nito].p
--[[
                    if #s==1 and s[1] == 3 then
--                        toMark(pth, 'mag', nil, 0.2)
--                        break
                    end
        if key == '6_2' then
            U.dump(pth, '?? 6_2:'..ni..':'..nito)
            D.decalPlot(pth)
                        toMark(pth, 'mag', nil, 0.2)
        end
]]
        if true then
--                U.dump(pth, '??____ prePATH:')
            local desc = D.decalPlot(pth, D.default.laneWidth*(#s.areg==1 and 4 or 3), adec)
            s.desc = desc
            s.list = desc.list
            adec[#adec+1] = desc
            desc.ind = #adec
            for _,ir in pairs(s.areg) do
                local reg = aregion[ir]
                if not reg.aseg then reg.aseg = {} end
                reg.aseg[#reg.aseg+1] = desc
            end
        end
--        toMark(pth, 'mag', nil, 0.1, 1)
    end
--        if true then return end
-- BUILD RADIALS
        am1 = {}
    local desc,reg
    for k,s in pairs(aseg) do
        reg = aregion[s.areg[1]]
        if not reg.arad then reg.arad = {} end
        if #s.areg == 2 then
            local dmi,imi = math.huge
            for i,p in pairs(s.list) do
                local L = aregion[s.areg[1]].p:distance(p)+aregion[s.areg[2]].p:distance(p)
                if L < dmi then
                    dmi = L
                    imi = i
                end
            end
                if s.areg[1] == 1 and s.areg[2] == 3 then
                    lo('??_____________________ if_IMI12:'..tostring(imi))
                end
            if imi and imi ~= 1 and imi ~= #s.list then
                    am1[#am1+1] = s.list[imi]
--                reg = aregion[s.areg[1]]
                desc = {list={reg.p,s.list[imi]}, w=3*3, mat='WarningMaterial'}
                decalUp(desc)
                desc.isline = true
                reg.arad[#reg.arad+1] = desc
                --adec[#adec+1] = desc
                --desc.ind = #adec
--                reg.arad.isline = true

                reg = aregion[s.areg[2]]
                desc = {list={reg.p,s.list[imi]}, w=3*3, mat='WarningMaterial'}
                decalUp(desc)
                if not reg.arad then reg.arad = {} end
                desc.isline = true
                reg.arad[#reg.arad+1] = desc
                --adec[#adec+1] = desc
                --desc.ind = #adec

                local pair = D.decalSplit(s.desc,s.list[imi])
                adec[#adec+1] = pair[1]
                pair[1].ind = #adec
                adec[#adec+1] = pair[2]
                pair[2].ind = #adec

--                reg.arad.isline = true
            end
        else --if s.areg[1]==1 then
-- radial OUT
                lo('?? for_OUT:'..s.areg[1])
--                U.dump(reg.arad, '??++++++++++++++++++++++++++++ arad:')
            local rmi = 0
            if reg.arad and #reg.arad > 0 then
                for k,d in pairs(reg.arad) do
                    rmi = rmi + d.list[1]:distance(d.list[#d.list])
                end
                rmi = rmi/#reg.arad
            else
                rmi = 150
            end
            local r = aregion[s.areg[1]]
            local p = U.toPoly(r.p, s.list)
            local pmi
            if p and r.p:distance(p) < r.p:distance(s.list[1]) and r.p:distance(p) < r.p:distance(s.list[#s.list]) then
                pmi = p
            else
                pmi = r.p:distance(s.list[1]) < r.p:distance(s.list[#s.list]) and s.list[1] or s.list[#s.list]
            end
            if pmi then
                local v,pair = (pmi - aregion[s.areg[1]].p):normalized()

                if false then
                    desc = D.decalPlot({aregion[s.areg[1]].p, pmi, pmi+v*rmi}, 3*D.default.laneWidth, adec)
                    adec[#adec+1] = desc
                    desc.ind = #adec
                    pair = D.decalSplit(desc,pmi,true)
                    adec[#adec+1] = pair[1]
                    pair[1].ind = #adec
                    adec[#adec+1] = pair[2]
                    pair[2].ind = #adec
                else
                    desc = D.decalPlot({aregion[s.areg[1]].p, pmi}, 4*D.default.laneWidth, adec)
                    adec[#adec+1] = desc
                    desc.ind = #adec
                    reg.arad[#reg.arad+1] = desc
                    desc = D.decalPlot({pmi, pmi+v*rmi}, 5.2*D.default.laneWidth, adec)
                    adec[#adec+1] = desc
                    desc.ind = #adec
                end
--                    toMark(s.desc.list,'cyan')
--                    local ne = s.desc.body:getEdgeCount()
--                    U.dump(s.desc, '?? pre_SPLIT:'..tostring(s.desc.body)..':'..ne)
--                pair = D.decalSplit(s.desc, pmi,true)
--                    lo('?? R1_OUT:'..tostring(pair))
                pair = D.decalSplit(s.desc,pmi,true)
                if pair then
                    adec[#adec+1] = pair[1]
                    pair[1].ind = #adec
                    adec[#adec+1] = pair[2]
                    pair[2].ind = #adec
                end
--                s.desc.body:delete()
--                    lo('?? split:'..#pair)

--                desc = {list={aregion[s.areg[1]].p,pmi}, w=3*2}
--                D.decalUp(desc)
--                    am1[#am1+1] = pmi
            end
        end
    end
--        toMark(am1,'blue',nil,0.1)
        lo('?? pre_LOAD:'..#adec)
--        for i,d in pairs(adec) do
--            lo('?? for_DEC:'..i..':'..d.ind)
--        end
    local lst
    lst,ajunc,across = D.decalsLoad(adec)
--        U.dump(ajunc,'?? pre_LOAD:'..#am1..':'..#adec)
-- APPEND valency-2 junctions
    for i,reg in pairs(aregion) do
        local jset
        for j,junc in pairs(ajunc) do
            if junc.p:distance(reg.p) < 1 then
--                    U.dump(junc, '?? toJ set:'..i)
                junc.list = reg.arad
                for i,b in pairs(junc.list) do
                    if junc.p:distance(b.list[1]) < 1 then
                        b.jfr = #ajunc
                    else
                        b.jto = #ajunc
                    end
                end
                jset = true
                break
            end
        end
        if not jset then
--                lo('?? to_APP:'..i..':'..#reg.arad)
            ajunc[#ajunc+1] = {p=reg.p, list=reg.arad}
            for i,b in pairs(ajunc[#ajunc].list) do
                if ajunc[#ajunc].p:distance(b.list[1]) < 1 then
                    b.jfr = #ajunc
                else
                    b.jto = #ajunc
                end
            end
        end
    end
--        if true then return end
--        for i,c in pairs(ajunc) do
--            lo('?? for_junc:'..i..':'..tostring(c.p))
--        end
--        toMark(adec[1].list,'cyan',nil,0.1,1)
--        toMark(ajunc, 'cyan',function(d)
--            return d.p
--        end, 0.1, 1)
--    adec = list
--[[
        toMark(ajunc, 'mag', function(d)
            return d.p
        end, 0.1, 0.8)
]]
        lo('?? post_LOAD:'..#lst..':'..#ajunc..':'..#adec)
--        U.dump(ajunc[1], '?? for_ADEC:'..#list..':'..#ajunc)
--        U.dump(ajunc, '?? for_ADEC:'..#list..':'..#ajunc)
--        toMark(am1, 'mag', nil, 0.1, 1)
--        U.dump(adec[1], '?? dec:')
    -- build lattice
--        toMark(adec[15].list,'green',nil,0.1,1)
--        toMark(ajunc,'red',function(d) return d.p end, 0.1, 1)
--        U.dump(across, '?? across:')
    for t,reg in pairs(aregion) do
        reg.amark = nil
            if (not dbc or #dbc==0) or (dbc and t==dbc[1]) then
--            U.dump(reg.arad, '?? for_REG:'..i)
--            lo('?? for_JUNC:'..#ajunc)
        for k,c in pairs(ajunc) do
--                lo('?? if_dist:'.._..':'..tostring(c.p)..':'..tostring(reg.p))
            if c.p:distance(reg.p) < U.small_dist then
                for i,b in pairs(c.list) do
--                        lo('?? for_SEC:'..t..':'..i)
--                        lo('?? for_SEC:'..t..':'..i)
--                    U.dump(adec[b.ind], '?? if_line:'..i..':'..tostring(adec[b.ind].isline))
                        if (not dbc or #dbc==0) or (dbc and i==dbc[2]) then
                    sectorPlot(k, i)
--                    sectorPlot(reg.p, c.list, i)
                        end
                end
--                    break
            end
        end
            end
    end
        lo('<< roadPlace:'..#ap)
end
--[[
            if not aseg[stamp] then aseg[stamp] = {} end
            if
            if not aseg[stamp] or aseg[stamp] == i then
                aseg[stamp] = i
                -- add segment
                local pth = {anode[ni].p}
                local ang = U.vang(anode[ni].p-r.p, anode[nito].p-r.p, true)
                if ang < 0 then ang = 2*math.pi + ang end
                    lo('?? for_ang:'..ni..'>'..nito..':'..ang)
                if ang > math.pi/3 then
                    local nstep = math.ceil(ang/math.pi*3)
                    local astep = ang/nstep
                    for s=1,nstep-1 do
                        pth[#pth+1] = r.p + 1.5*ravg*r.w*U.vturn(pth[1]-r.p, s*astep):normalized()
                    end
                    pth[#pth+1] = anode[nito].p
                        toMark(pth, 'mag', nil, 0.1)
                        break
                end
            end
]]
--            aseg[stamp][#aseg[stamp]+1] = ni
--            aseg[stamp][#aseg[stamp]+1] = nito
--[[
    local function crossNext_(deci, ni)
            U.dump(across[deci][ni], '?? crossNext:'..deci..':'..ni)
            toMark({adec[deci].list[ni]}, 'mag', nil, 0.2, 1)
        -- find left turn
        local b = adec[deci]
        local nip = ni == 1 and 2 or #b.list-1

    end
]]
--[[
    local loop
    local function forNext(br, p)
        if not p then p = loop[#loop].p end
        local list = br.list
        local ifr,ito,dir = 1,#list,1
        if p:distance(list[1]) > 1 then
            ifr,ito,dir = #list,1,-1
        end
--            lo('?? forNext:'..ifr..':'..ito..'/'..#list)
        for i=ifr,ito-dir,dir do
            loop[#loop+1] = {p = list[i], w = br.aw[i], seg = br}
--                lo('?? to_LOOP:'..i..':'..tostring(loop[#loop]))
        end
        return ito
    end
    local function crossNext(dsc, ni)
        local junc = ajunc[ni == 1 and dsc.jfr or dsc.jto]
--            lo('?? crossNext:'..dsc.ind..':'..tostring(dsc.jfr)..':'..dsc.jto)
--            local a = junc.list[3].list[1]
--            local b = junc.list[3].list[#junc.list[3].list]
--            lo('?? for_a:'..tostring(a))
--            toMark({a, b}, 'yel', nil, 0.5, 1)
--            toMark({junc.list[1].list[1],junc.list[1].list[#junc.list[1].list]}, 'mag', nil, 0.1, 1)
        for i,b in pairs(junc.list) do
--                lo('?? for_BR:'..i..':'..b.ind)
            if b.ind == dsc.ind then
--                lo('?? for_NEXT:'..i..':'..U.mod(i-1,#junc.list))
                local br = U.mod(i-1, junc.list)

                return br,forNext(br,dsc.list[ni])
            end
        end
    end
    local function trim(asli2,asli1)
        for k,s in pairs(asli2) do
            local L = s[1]:distance(s[2])
            local dmi,dma,lmi,lma,pmi,pma = math.huge,0
            for l,t in pairs(asli1) do
                -- s - distance from s[1]
                local c,s = U.segCross(t[1],t[2],s[1],s[2])
                if c then
                    if s < dmi then
                        dmi = s
                        lmi = l
                        pmi = c
                    end
                    if s > dma then
                        dma = s
                        lma = l
                        pma = c
                    end
--                                        lo('?? crossed:'..k..':'..l..':'..s)
                end
            end
            if lmi and lma then
--                lo('?? lmm:'..k..':'..dmi..':'..dma..':'..(dmi*L)..':'..((1-dma)*L))
                if dmi*L < default.END_MIN then
                    -- cut off
                    s[1] = pmi
                end
                if (1-dma)*L < default.END_MIN then
                    s[2] = pma
                end
            end
        end
    end
]]
--[[
        for j,d in ipairs(reg.arad) do
            if d.isline then
                lo('?? for_line:'..j)

            end
        end
]]
--[[
    for i,r in pairs(aregion) do
        for k,s in pairs(aseg) do
            if #U.index(s.areg, i) > 0 then
                local p = U.toPoly(r.p, s.list)
                if p and p:distance(s.list[1]) > DMI_INS and p:distance(s.list[#s.list]) > DMI_INS then
                    -- do insert
                end
            end
        end
    end
]]
--[[
                    local cb,dir = b
                    if adec[b.ind].isline then
--                            U.dump(b, '?? for_bst:'..k..':'..b.jfr)
                        dir = cb.list[#cb.list]-cb.list[1]
                    else
                        local bbase = U.mod(i+1,c.list)
                        dir = bbase.list[#bbase.list]-bbase.list[1]
                    end
-- GET LOOP
                        lo('?? if_line:'..i..':'..tostring(adec[b.ind].isline)..':'..tostring(dir))
--                        loop = {{p=reg.p, w=b.aw[k==b.jfr and 1 or #b.aw], seg=b}}
                    loop = {}
                    local ito = forNext(cb, reg.p)
                            local n = 0
                    while cb.list[ito]:distance(reg.p) > 1 and n < 100 do
                        cb,ito = crossNext(cb, ito)
--                                lo('?? for_nxtl:'..cb.ind..':'..ito)
                            n = n+1
                    end
                    aloop[#aloop+1] = {list=loop, reg=t, ind=i}
                        lo('?? if_LOOP:'..#loop..':'..n)
--                            U.dump(loop,'?? if_LOOP:'..#loop)
--                        table.remove(loop,#loop)
-- slice domain
                    local asli1 = regionSlice(loop, dir,7)
                    local asli2 = regionSlice(loop, U.vturn(dir, math.pi/2), 9)
--                            toMark(asli2[1],'yel',nil,0.1,1)
                    -- cut ends
                    trim(asli2,asli1)
                    trim(asli1,asli2)
                    -- render
                    for i,s in pairs(asli1) do
                        local desc = {list=s, w=(1+U.rand(-0.2,0.4))*D.default.laneWidth, mat='WarningMaterial', skip=true}
                        decalUp(desc)
                    end
                    for i,s in pairs(asli2) do
                        local desc = {list=s, w=(1+U.rand(-0.2,0.4))*D.default.laneWidth, mat='WarningMaterial', skip=true}
                        decalUp(desc)
                    end
]]
--[[
                        for i,s in pairs(asli1) do
                            local desc = {list=s, w=1*D.default.laneWidth, mat='WarningMaterial'}
                            decalUp(desc)
                        end
                        if true then return end
                    for i,s in pairs(asli2) do
                        local desc = {list=s, w=1*D.default.laneWidth, mat='WarningMaterial'}
                        decalUp(desc)
                    end
                        if true then return end
]]
--                        loop[#loop+1] = adec[b.ind].list[2]
                        -- get circle
--                            lo('?? for_JUNC:'.._..':'..#loop..':'..n)
--[[
                        cb,ito = crossNext(cb, ito)
                            lo('?? for_seg1:'..cb.ind..':'..ito)
                        cb,ito = crossNext(cb, ito)
                            lo('?? for_seg2:'..cb.ind)
]]
--                            U.dump(adec[b.ind], '??^^^^^^^^^^^^^^ for_P:'.._..':'..tostring(reg.p))
--                        local bnxt = crossNext(b, 2)
--                        loop[#loop+1] = bnxt.list[1]
--                        loop[#loop+1] = bnxt.list[#bnxt.list]

--                            U.dump(across[b.ind], '?? ')
--                        forNext(loop, adec[b.ind].list)
--                        local seg = adec[b.ind]
--                        local dir = reg.p:distance(seg.list[1]) < U.small_dist and 1 or -1
--                        loop[#loop+1] = dir == 1 and seg.list[1] or seg.list[#seg.list]
--                        loop[#loop+1] = dir == 1 and seg.list[#seg.list] or
--                            U.dump(across[b.ind], '?? for_branch:'..i)
--                            toMark(loop, 'yel', function(d)
--                                return d.p
--                            end, 0.1, 0.8)
--                            toMark(b.list, 'mag', nil, 0.2, 1)
--                                break

--                    U.dump(c.list,'?? SLICE:'..#c.list..':'.._..':'..tostring(c.p))
--                    U.dump(c.list,'?? SLICE2:'..#c.list)
--[[
                table.sort(c.list, function(a, b)
                    local dira,dirb
                    dira = (a.list[1]:distance(c.p)<U.small_dist and
                        a.list[2]-a.list[1] or
                        a.list[#a.list-1] - a.list[#a.list]):normalized()
                    dirb = (b.list[1]:distance(c.p)<U.small_dist and
                        b.list[2]-b.list[1] or
                        b.list[#b.list-1] - b.list[#b.list]):normalized()
                    return U.vang(dira,vec3(1,0),true) > U.vang(dirb,vec3(1,0),true)
                end)
]]



--[[
T.onVal = function(key, val)
        lo('?? Ter.onVal:'..key)
    if false then
    elseif key == 'b_road' then
        roadPlace()
    end
end
    if false then
        for i=0,#aregion-1 do
            for j=i+1,#aregion do
                for _,nd in pairs(anode) do
                    if #U.index(nd.key,i)>0 and #U.index(nd.key,j)>0 then
                        local stamp = U.stamp({i,j})
                        if not aseg[stamp] then aseg[stamp] = {} end
                        aseg[stamp][#aseg[stamp]+1] = nd.p
                    end
                end
            end
        end
            U.dump(aseg, '?? aSEG:')
        -- place decals
        for key,s in pairs(aseg) do
            if #s == 2 then
                local aind = U.split(key,'_')
                if aind[1] == 0 then
                    local pm = (s[1]+s[2])/2
                    local reg = aregion[aind[2] ]
                        lo('?? for_bound:'..key..':'..tostring(pm)..':'..tostring(reg.p)..':'..tostring(c))

                    if (pm-reg.p):dot(pm-c) < 0 then
                        -- set middle boundary node
                        local pb = reg.p - 1.5*r*reg.w*(pm-reg.p):normalized()
                            lo('?? for_dec:'..tostring(pb)..':'..tostring(s[1])..':'..r..':'..(r*reg.w))
                            toMark({s[1],pb}, 'cyan', nil, 0.05, 0.8)
                            D.decalPlot({s[1],pb})
                            D.decalPlot({s[2],pb})
    --                        break
                    end
                end
            end
    --            break
        end
    end
]]
--[[
if false then
    local terrBlockName = 'TFlat'
    local terrBlock = TerrainBlock()
    local to = scenetree.findObject(terrBlockName)
    if to then
        print('?? to_DEL:'..tostring(to)..':'..tostring(tonumber(to))..':'..to:getID())
        to:delete()
    else
    end
    terrBlock:setName(terrBlockName)
    terrBlock:registerObject(terrBlockName)

    --    table.insert(terrainImpExp.textureMaps, {path=path, selected=false, material=matName or "", materialId = im.IntPtr(matId or 0), channel="R", channelId=im.IntPtr(0)})

    local materials = {'m_plaster_worn_01_bat'}
    --	for _,map in ipairs(terrainImpExp.textureMaps) do
    --		table.insert(materials, map.material)
    --	end

    local done = terrBlock:importMaps(
        '/lua/ge/extensions/editor/gen/assets/tflat2.png',
        1, 50
        ,ffi.string('/levels/gridmap_v2/art/terrains/t_macro_holes_r.png')
    --    ,ffi.string('/lua/ge/extensions/editor/gen/assets/tflat.png')
    --        ,ffi.string(im.ArrayChar(128))
    --        ffi.string(terrainImpExp.holeMapTexture),
        ,materials
        ,{path='/levels/gridmap_v2/art/terrains/dirt_overlay.color.png',selected=true}
        ,im.BoolPtr(false)
    --        , terrainImpExp.textureMaps, terrainImpExp.flipYAxis[0]
    )
        print('??^^^^^^^^^^^^^^^^^^^ if_DONE:'..tostring(done)..':'..tostring(scenetree.findObject('m_plaster_worn_01_bat')))


    to = scenetree.findObject(terrBlockName)
        print('?? if_NEW:'..tostring(to)..':'..tostring(groupEdit))
    groupEdit:addObject(terrBlock)
end
]]


local function test()
        lo('>> test:')
    if true then return end

    if true then
        local sample = '1863.864'
            --'16745.35'
            --'7876.179'
            -- '202.449'
            -- '63345.239'
        local fl = io.open('./tmp/reg_save_'..sample..'.json')
        local str = fl:read('*all')
        fl.close()
        aregion = jsonDecode(str)
        for i,d in pairs(aregion) do
            d.p = vec3(d.p.x,d.p.y)
            d.p.z = forZ(d.p)
--                U.dump(d, '?? for_reg:')
        end
            U.dump(aregion, '?? for_reg:')
        T.out.aregion = aregion
        regionPlot()
        roadPlace()
        return
    end

    local fl = io.open('/lua/ge/extensions/editor/gen/assets/chunk.json')
    local str = fl:read('*all')
    fl.close()
    local dw = jsonDecode(str)
    ama = 0
    for i,v in pairs(dw) do
        if v > ama then
            ama = v
        end
        dw[i] = {v, 0}
    end
        U.dump(dw, '<< test:')
    wav2ter(dw)
end
--test()
T.test = test


local dval = {}

T.onVal = function(key, val)
    dval[key] = val
    if false then
    elseif key == 'b_road' then
            lo('?? to_roads:'..#aregion)
        ajunc = {}
        across = nil
        aloop = {}
        adec = {}
            local fl = io.open('./tmp/reg_save'..'_63345.239'..'.json')
            local str = fl:read('*all')
            fl.close()
--            aregion = jsonDecode(str)
            for i,d in pairs(aregion) do
                d.p = vec3(d.p.x,d.p.y)
                aregion[i] = {p=d.p, w=d.w}
            end
        regionPlot()
        roadPlace()
--        roadPlace()
    elseif key == 'sec_margin' then
    elseif key == 'sec_grid' then
    elseif key == 'sec_rand' then
    end
end


local function apply(key, val)
    if false then
    elseif key == 'sec_griddir1' then
        local loop = aloop[T.out.insector]
        for i,rd in pairs(loop.aslice) do
            decalDel(rd.ind)
        end
        -- rebuild
        loop.dir1 = val
        sectorPlot(loop.junc, loop.ind, loop)
    elseif key == 'sec_griddir2' then
        local loop = aloop[T.out.insector]
        for i,rd in pairs(loop.aslice) do
            decalDel(rd.ind)
        end
        -- rebuild
        loop.dir2 = val
        sectorPlot(loop.junc, loop.ind, loop)
    elseif key == 'sec_rand' then
        local loop = aloop[T.out.insector]
        for i,rd in pairs(loop.aslice) do
            decalDel(rd.ind)
        end
        -- rebuild
        loop.randrate = val
        sectorPlot(loop.junc, loop.ind, loop)
    elseif key == 'sec_margin' then
        local loop = aloop[T.out.insector]
        for i,rd in pairs(loop.aslice) do
            decalDel(rd.ind)
        end
        -- rebuild
        loop.margin = val
        sectorPlot(loop.junc, loop.ind, loop)
    elseif key == 'sec_grid' then
--            lo('?? apply_for_LOOP:'..tostring(T.out.insector)..':'..#adec)
        local loop = aloop[T.out.insector]
--            lo('?? sec_grid:'..T.out.insector..':'..loop.junc..':'..loop.ind)
        -- clean
        for i,rd in pairs(loop.aslice) do
--                lo('?? to_del:'..rd.ind..':'..#adec) --..':'..adec[rd.ind].ind)
            decalDel(rd.ind)
--            D.del(rd.ind)
--            adec[rd.ind] = nil
--            rd.body:delete()
        end
        -- rebuild
        loop.grid = val
        sectorPlot(loop.junc, loop.ind, loop)
    end
end


local ctime

local function onUpdate()
--        if true then return end
    local now = os.clock()
    if not ctime then
        ctime = now
    elseif now-ctime > 2 then
--            lo('?? next:')
        -- get chunk
        local dw = {}
        -- render chunk
--        wav2ter()
        ctime = now
    end
--------------------
-- MOUSE
--------------------
    local rayCast = cameraMouseRayCast(false)

    if im.IsMouseClicked(0) and U.inView() and rayCast then
            lo('?? Ter.click:'..#aregion)
--            if true then return end
        if editor.keyModifiers.shift then

-- REGION PICK
            local torem
            for i,d in pairs(aregion) do
                if U.angDist(d.p) < 0.06 then
--                if d.p:distance(rayCast.pos) < 6 then
                    if T.out.inplace then
                        T.out.inplace = false
                            lo('??++++++ TO_PLOT:')
                        clear()
                        regionPlot()
                        return true
                    else
                            lo('??++++++ TO_REM:')
                        table.remove(aregion, i)
                        torem = true
                    end
                end
            end
            if not T.out.inplace and not torem then
                aregion[#aregion+1] = {p=rayCast.pos, w=1}
                if #aregion > 1 then
                    regionPlot()
                end
            end
            T.out.aregion = aregion
-- SECTOR PICK
            if T.out.inplace then
                    lo('?? sector_PICK:'..#aregion..':'..#aloop)
                local p = U.proj2D(rayCast.pos)
                local dmi,lmi,imi,pmi = math.huge
                for i,loop in pairs(aloop) do
                    local poly = {}
                    for j,v in pairs(loop.list) do
                        poly[#poly+1] = v.p
                    end
                    local c,ni = U.toPoly(p, poly)
                    if c:distance(p)<dmi then
                        if U.vang(U.mod(ni+1,loop.list).p-c,p-c,true) > 0 then
                            dmi = c:distance(p)
                            imi = i
                            pmi = c
                        end
                    end
                end
                if imi then
                    T.out.insector = imi
                    toMark(aloop[imi].list,'yel',function(d)
                        return d.p
                    end,0.1,0.7)
    --                toMark({pmi},'blue',nil,0.1,1)
                    return 1
                end
            else
            end

            return true
        elseif editor.keyModifiers.ctrl then
        elseif false and #aregion then
--[[
            for i,reg in pairs(aregion) do
                if reg.p:distance(rayCast.pos) < 10 then
                    lo('?? to_reg_PLOT:'..i)
                    return 1
                end
            end
]]
                lo('?? sector_PICK:'..#aregion..':'..#aloop)
            local p = U.proj2D(rayCast.pos)
            local dmi,lmi,imi,pmi = math.huge
            for i,loop in pairs(aloop) do
                local poly = {}
                for j,v in pairs(loop.list) do
                    poly[#poly+1] = v.p
                end
                local c,ni = U.toPoly(p, poly)
                if c:distance(p)<dmi then
                    if U.vang(U.mod(ni+1,loop.list).p-c,p-c,true) > 0 then
                        dmi = c:distance(p)
                        imi = i
                        pmi = c
                    end
                end
--[[
                for j,v in pairs(loop.list) do
                    if v.p:distance(p)<dmi then
                        if U.vang(U.mod(j+1,loop.list).p-v.p,p-v.p,true) > 0 then
                            dmi=v.p:distance(p)
                            pmi = v.p
                        end
                    end
                end
]]
            end
            if imi then
                T.out.insector = imi
                toMark(aloop[imi].list,'yel',function(d)
                    return d.p
                end,0.1,0.7)
--                toMark({pmi},'blue',nil,0.1,1)
                return 1
            end
        end
    end

    if im.IsMouseReleased(0) then
        for key,val in pairs(dval) do
                lo('?? for_val:'..key..':'..val)
            apply(key, val)
            dval[key] = nil
            return
        end
    end

    local w = im.GetIO().MouseWheel
	if w ~= 0 and editor.keyModifiers.ctrl then
        for i,d in pairs(aregion) do
            if rayCast and d.p:distance(rayCast.pos)/core_camera.getPosition():distance(rayCast.pos) < 0.02 then
--            if d.p:distance(rayCast.pos) < 5 then
                -- region resize
--                    lo('>> Ter.wheel:'..w)
                d.w = d.w*math.pow(1.1, w>0 and 1 or -1)
                -- normalize
                local s = 0
                for j,o in pairs(aregion) do
                    s = s + o.w
                end
                s = s/#aregion
                for j,o in pairs(aregion) do
                    o.w = o.w/s
                end
                regionPlot()
                break
--                    lo('<< Ter.wheel:')
--                    U.dump(aregion, '?? Ter.wheel:')
--                return true
            end
        end
	end
--------------------
-- MARKING
--------------------
    for i,d in pairs(aregion) do
        if true or U._PRD == 1 then
            R.sphere(d.p, 0.4*d.w, {1,1,1,0.3})
        end
        local key = U.mod(i,acolor)
        local c = legend[key][1]
        c[4] = legend[key][3]
--                lo('?? for_LKEY:'..tostring(key)..':'..tostring(d.amark and #d.amark or nil))
        if d.amark then
--                lo('?? for_LKEY2:'..tostring(key)..':'..tostring(d.amark and #d.amark or nil)..':'..tostring(legend[key][2])..':'..tostring(d.amark[1]))
            for _,p in pairs(d.amark) do
                R.sphere(p, legend[key][2], c)
            end
        end
    end

    for key,d in pairs(legend) do
        if T.out[key] and #T.out[key] > 0  then
            local c = legend[key][1]
            c[4] = legend[key][3]
--                lo('?? for_mark:'..tostring(T.out[key][1]))
            for i,p in pairs(T.out[key]) do
                R.sphere(p, legend[key][2], c)
            end
        end
    end
end

T.onUpdate = onUpdate

return T