local Net = {out = {}}

require 'socket'

local U = require('/lua/ge/extensions/editor/gen/utils')
local Render = require('/lua/ge/extensions/editor/gen/render')
local Mesh = require('/lua/ge/extensions/editor/gen/mesh')

local im = ui_imgui

local indrag = false

local adbg = {}
local inanim


local function lo(ms, yes)
    if true then return end
    if indrag and not yes then return end
    print(ms)
end


U.segmentCross = function(a1, a2, b1, b2, inner)
    local x, y = U.lineCross(a1, a2, b1, b2)
    local p = vec3(x, y, 0)
    local eps = 0.01
    -- TODO: handle inner
    if (p - a1):length() >= (a1 - a2):length() - eps or (p - a2):length() >= (a1 - a2):length() - eps then
        return nil
    end
    if (p - b1):length() >= (b1 - b2):length() - eps or (p - b2):length() >= (b1 - b2):length() - eps then
        return nil
    end
--    print('<< segmentCross:'..x..':'..y)
    return x, y
end

U.fanOrder = function(apos)
    local aang = {}
    for i,v in pairs(apos) do
        aang[#aang + 1] = {i, math.atan2(v.y, v.x) % (2*math.pi)}
    end
    local function comp(a, b)
        return a[2] < b[2]
    end
    table.sort(aang, comp)

    return aang
end

U.vnorm = function(v)
    return vec3(-v.y, v.x, 0):normalized()
end

U.hightOnCurve = function(p, rdinfo, start, dbg) --, av, step, dbg)
    local decal = scenetree.findObjectById(rdinfo.id)
    local av, step = rdinfo.av, rdinfo.avstep
--U.hightOnCurve = function(p, decal, start, av, step, dbg)
--U.hightOnCurve = function(p, aheight, decal, start)
--  print('>> hightOnCurve:'..tostring(p)..':'..#aheight..':'..decal:getEdgeCount()..':'..start)
            if dbg == true then
                adbg[#adbg+1] = {p+vec3(0,0,0), ColorF(1,0,1,1), 0.2}
                adbg[#adbg+1] = {decal:getMiddleEdgePosition(start)+vec3(0,0,0), ColorF(1,1,0,1), 0.5}
            end
    local w = 4
    local mi, imi = 1/0
    local ifr = math.max(1, start-25)
    for i = ifr,start+25 do --*dir,dir do
                if dbg then print('?? hightOnCurve_i:'..i..'/'..start) end
        if (i + 1)*step + 1 > #av then
            break
        end
        local ni, nip = U.proj2D(decal:getMiddleEdgePosition(i)), U.proj2D(decal:getMiddleEdgePosition(i+1))
        -- projections
        local pi, pip = (p - ni):dot(nip - ni), (p - nip):dot(nip - ni)
        local shadow = math.abs(pi) + math.abs(pip)
        if shadow < mi then
            mi = shadow
            imi = i
        end
        if dbg then
            print('?? hc:'..i..'/'..start..':'..decal:getEdgeCount()..':'..step..' pi:'..pi..' pip:'..pip)
        end
        if dbg and pi * pip < 0 then
            print('??___<:'..i..'/'..start..':'..tostring(p)..':'..tostring(ni)..':'..tostring(nip)..':'..p:distanceToLineSegment(ni, nip))
--            adbg[#adbg+1] = {decal:getMiddleEdgePosition(i),ColorF(0,1,1,1), 0.2}
--            adbg[#adbg+1] = {decal:getMiddleEdgePosition(i+1),ColorF(0,1,1,1), 0.2}
            local api = math.abs(pi) --/(p - ni):length()/(nip - ni):length()
            local apip = math.abs(pip) --/(p - nip):length()/(nip - ni):length()
            adbg[#adbg+1] = {
                (1 - api/(api + apip))*(decal:getMiddleEdgePosition(i)+vec3(0,0,1))+
                api/(api + apip)*(decal:getMiddleEdgePosition(i+1)+vec3(0,0,1)),
                ColorF(0,1,1,1), 0.2}
        end
        if pi * pip <= 0 and p:distanceToLineSegment(ni, nip) < w then
    --      print('?? hoc:'..i..':'..pi..':'..pip..' ni:'..tostring(ni)..' nip:'..tostring(nip))
            pi = math.abs(pi) --/(p - ni):length()/(nip - ni):length()
            pip = math.abs(pip) --/(p - nip):length()/(nip - ni):length()

            local h = av[i*step + 1].z * (1 - pi/(pi + pip)) +
                av[(i + 1)*step + 1].z * pi/(pi + pip)
--            print('<< hightOnCurve:'..i..'/'..start..' p:'..tostring(p))
    --        local h = aheight[i]*(1 - pi/(pi + pip)) + aheight[i+1]*pi/(pi + pip)
    --      print('<< hightOnCurve:'..i..':'..(i + 1)..' h:'..aheight[i]..':'..aheight[i+1]..':'..pi..':'..pip..' h:'..h)
            return h
        end
--    print('?? for_node:'..i..':'..tostring(cp)..' d:'..(p-cp):length())
    end
    if imi ~= nil then
        -- assign height of closest vertex
        print('?? hightOnCurve:'..imi..':'..#av)
        return av[imi*step + 1].z
    end
    print('!!!!!!!!!!!!!! hightOnCurve.NONE:')
--  print('<< hightOnCurve:'..imi..'/'..decal:getEdgeCount()..':'..aheight[imi])
--  return aheight[imi]
end



Mesh.strip = function(L, apos, av, af)
    local i = #av/L
    for i = 1,L do
        av[#av + 1] = apos[i]
    end
    for k = 1,L-1 do
        af[#af + 1] = {v = L*(i - 1) + (k - 1 + 0), n = 0, u = 0}
        af[#af + 1] = {v = L*(i - 1) + (k - 1 + L), n = 0, u = 1}
        af[#af + 1] = {v = L*(i - 1) + (k - 1 + 1), n = 0, u = 3}

        af[#af + 1] = {v = L*(i - 1) + (k - 1 + 1), n = 0, u = 3}
        af[#af + 1] = {v = L*(i - 1) + (k - 1 + L), n = 0, u = 1}
        af[#af + 1] = {v = L*(i - 1) + (k - 1 + L + 1), n = 0, u = 2}
    end
    return av, af
end


local groupEdit
if U._PRD == 0 then
    groupEdit = scenetree.findObject('e_road')
    lo('??_______________________________________ nw_e_road:'..tostring(groupEdit))
    if groupEdit then
        scenetree.MissionGroup:removeObject(groupEdit)
        groupEdit:delete()
        lo('?? removed2:')
    end
    groupEdit = scenetree.findObject('e_road')
    lo('= NETWORK:'..tostring(groupEdit))
    if groupEdit == nil then
            lo('?? if_SG:'..tostring(scenetree.findObject('SimGroup')))
        groupEdit = scenetree.findObject('SimGroup')
        if not groupEdit then groupEdit = createObject('SimGroup') end
        groupEdit:registerObject('e_road')
        groupEdit = scenetree.findObject('e_road')
            lo('?? e_road:'..tostring(groupEdit))
        scenetree.MissionGroup:removeObject(groupEdit)
        scenetree.MissionGroup:addObject(groupEdit)
    end
end

local  tb = extensions.editor_terrainEditor.getTerrainBlock()
--lo('??************** for_TB:'..tostring(tb:getObjectBox():getExtents()))
--U.dump(tb:getObjectBox():getExtents(), '??********** for_TB:')
--local groupEdit = scenetree.findObject('edit')

local anode, apath, apairskip = {}, {}, {}
local edges = {astamp = {}, infaces = {}}
local jointpath = {}
local sregion = {}
local adecalID = {}
local ameshID = {}

local roads = {} -- path, astamp, levels
local exits = {} -- exits road info
local astamp = {}

local circleMax = 80
local wa, wb = 0.5, 0.5
local massCenter = vec3(0, 0, 0)

local wHeighway = 4
local wExit = 2
local hLevel = 3    -- space between levels
local mat = 'WarningMaterial' --'dirt_road_tread_ruts'

local nIter = 0

local function circleFit(stamp, r)
--    if d == nil then
--        nIter = 0
--    else
--        nIter = nIter + 1
--    end
    local pair = U.split(stamp, '_')
    local i, j = pair[1], pair[2]
    local c1, c2 = anode[i], anode[j]
    local vn = c2.pos - c1.pos
    local d = vn:length()
    local a, b = c1.r, c2.r
--    print('>> circleFit:'..stamp..':'..r..' anode:'..#anode..' iter:'..nIter..' d:'..tostring(math.abs(d - (a + b)))..':'..tostring(massCenter))
    vn = vn:normalized()
    local ang = math.atan2(vn.y, vn.x)
    local dmiddle, flip = 0, 1
--    print('?? circleFit_d:'..stamp..':'..tostring(d)..':'..tostring(vn))
    -- get center position
    local x, y
--    print('?? circleFit.pre_int:'..#anode..':'..d..':'..tostring(c1.pos)..':'..tostring(c2.pos)..':'..(d -(a+b)))
    if math.abs(d - (a + b)) > 0.0001 then
        dmiddle = (d - (a+b))/2
        x = -(a+b-d+2*r)*(a-b)/(2*d)
        y = math.sqrt((d+b-a)*(d+a-b)*(a+b+d+2*r)*(a+b-d+2*r))/(2*d)
    --    print('!! FOR_INTER:'..i..":"..j..":"..d..':'..x)
        -- CHECK IN-OUT
        local v = U.vturn({x=x, y=y}, ang)
        v = c1.pos + vn*(c1.r + dmiddle) + vec3(v.x, v.y, 0)
        local vflip = U.vturn({x=x, y=-y}, ang)
        vflip = c1.pos + vn*(c1.r + dmiddle) + vec3(vflip.x, vflip.y, 0)
        --TODO: massCenter role?
        if (vflip - massCenter/#anode):length() > (v - massCenter/#anode):length() then
--            print('?? TO_flip:'..stamp..':'..tostring(massCenter)..':'..#anode..':'..dmiddle)
            y = -y
            flip = -1
        end
    else
        x = -r*(a - b)/(a + b)
        y = 2/(a + b) * math.sqrt(r*a*b*(r + a + b))
        if edges.infaces[stamp] == 1 then
--        if inCounter(i, j) == 1 then
--            print('?? to_flip:'..i..':'..j)
            y = -y
            flip = -1
        end
    end

    local v = U.vturn({x = x, y = y}, ang)
--            print('?? cF2:'..x..':'..y..'::'..tostring(v)..':'..ang)
    local nd = {
        pos = c1.pos + vn*(c1.r + dmiddle) + vec3(v.x, v.y, 0),
        r = r,
        stamp = stamp,
--        d = d,
    }
    if math.abs((nd.pos - c1.pos):length()/(c1.r + nd.r)) > 0.001 then
        local spos = nd.pos
        v = U.vturn({x=-x, y=y}, ang)
        nd.pos = c1.pos + vn*(c1.r + dmiddle) + vec3(v.x, v.y, 0)
--        print('!! sign: x:'..x..' y:'..y..' vn:'..tostring(vn)..' ang:'..ang..':'..tostring(spos)..'>'..tostring(nd.pos)..':'..math.abs((spos - c1.pos):length()/(c1.r + nd.r)))
    end

    --check for intersections
    local mi, inear = 1/0, nil
    for k = 1,#anode do
        if k ~= i and k ~= j then
            local dist = (nd.pos - anode[k].pos):length()
            if dist - (nd.r + anode[k].r) < -0.01 then
    --          print('!! INTER:'..i..":"..j..":"..k)
                if dist < mi then
                    mi = dist
                    inear = k
                end
            end
        end
    end
    if nIter >= #anode then
        print('!! circleFit.NO_FIT:'..stamp..':'..nIter)
        nIter = 0
        return
    elseif inear ~= nil then
        local k = inear
--        print('??******* int_resolve:'..i..':'..j..':'..inear..':'..tostring(nd.pos))
        -- select longer side
        local ind, mid = i, j
        local di = (anode[i].pos - anode[k].pos):length() - (anode[i].r + anode[k].r)
        local dj = (anode[j].pos - anode[k].pos):length() - (anode[j].r + anode[k].r)
        if dj > di then
          ind = j
          mid = i
        end
        edges.infaces[U.stamp({mid, ind})] = 2
        edges.infaces[U.stamp({mid, k})] = 2
        nIter = nIter + 1

        return circleFit(U.stamp({ind, k}), r) --, (anode[ind].pos - anode[k].pos):length())
    end

    anode[#anode+1] = nd
--    massCenter = massCenter + nd.pos

    local ccount = edges.infaces[stamp]
    if ccount == nil then
      ccount = 0
    end
    if d ~= nil then
      ccount = 1
    end
    edges.infaces[stamp] = ccount + 1
    local snew
    snew = U.stamp({i, #anode})
    if #U.index(edges.astamp, snew) == 0 then edges.astamp[#edges.astamp + 1] = snew end
    edges.infaces[snew] = -flip
--    print('?? sf2:'..snew..':'..edges.infaces[snew])
    snew = U.stamp({j, #anode})
    if #U.index(edges.astamp, snew) == 0 then edges.astamp[#edges.astamp + 1] = snew end
    edges.infaces[snew] = flip
    nIter = 0
--    print('?? sf3:'..snew..':'..edges.infaces[snew])
--            U.dump(edges.infaces, '?? Infaces:'..stamp)

--    print('<<------ circleFit:'..stamp..':'..tostring(nd.pos)..':'..tostring(nd.d)) --..tostring(i)..':'..tostring(j)..':'..tostring(#anode)..':'..tostring(counter['1_2']))
    return nd
end


local ccenter

local function circleSeed(nnode, pos, r, mode)
    if not pos then pos = vec3(0,0,0) end
    if r then circleMax = r end
--    massCenter = vec3(pos.x, pos.y)
    local h = core_terrain.getTerrainHeight(pos)
    if not h then return end
    pos.z = h
    ccenter = pos
        print('>>+++++++++++++++++++++++++ circleSeed:'..nnode..':'..tostring(mode)..':'..#anode..':'..tostring(pos)..' h:'..tostring(h))

    edges = {astamp = {}, infaces = {}}
    local ar = {}
    local resize
    local state
    local _dbg -- = true

    if mode ~= nil and mode.resize ~= nil then
--        print('?? circleSeed.resize:'..#anode)
        resize = mode.resize
        state = anode
    elseif _dbg then
        local fname = 'city.out'
        local lines = {}
        for line in io.lines(fname) do
          lines[#lines + 1] = line
--          print('?? line:'..line)
        end
        state = {}
        for i,l in pairs(lines) do
            local a = U.split(l,',')
            state[#state+1] = {
                pos = vec3(a[1],a[2],a[3]),
                r = a[4],
                stamp = U.stamp({a[5], a[6]}), ang = a[7] }
--            print('?? snode:'..i..' r:'..snode[#snode][4])
        end
    end
    if state ~= nul then
        nnode = #state
    end

    if state == nil then
        fout = io.open('city.out', "w")
    end
--    print('?? sC2:')

    for i = 1,nnode do
        if state == nil then
            ar[#ar + 1] = circleMax*(1/3 + math.random())
        else
--            print('?? from_state:'..i..':'..tostring(state[i]))
            ar[#ar + 1] = state[i].r
            if resize ~= nil and i == sregion[1] then
                if resize > 0 then
                    ar[#ar] = ar[#ar] * 1.1
                else
                    ar[#ar] = ar[#ar] / 1.1
                end
            end
        end
    end
    anode = {}
    anode[#anode + 1] = { pos = vec3(0, 0, 0), r = ar[1], stamp = '0_0' }
    local ang = 2*math.random()*math.pi
    if state ~= nil then
        ang = state[2].ang
    end
    anode[#anode + 1] = { pos = U.vturn(vec3(ar[1] + ar[2], 0, 0), ang), r = ar[2], ang = ang, stamp = '0_1' }
--            print('?? FOR_2:'..tostring(ang)..':'..tostring(anode[#anode].pos)..':'..tostring(anode[#anode].ang))
--    U.dump(anode[#anode], '?? FOR_2:'..tostring(ang))
    edges.astamp[#edges.astamp + 1] = U.stamp({1, 2})
--    massCenter = anode[1].pos + anode[2].pos

--    U.dump(anode[1], '?? node1:')
--    U.dump(anode[2], '?? node2:')

    for i = 3,nnode do
-- pick pair
        local apair = {}
        for o,s in pairs(edges.astamp) do
            if (edges.infaces[s] ~= nil and math.abs(edges.infaces[s]) == 1) or #edges.astamp == 1 then
                apair[#apair + 1] = s
            end
        end
        local pair, d
--        if state ~= nil then
        if state ~= nil and (#edges.astamp == 1 or edges.infaces[state[i].stamp] == nil or math.abs(edges.infaces[state[i].stamp]) < 2) then
            pair = state[i].stamp
            d = state[i].d
--            print('?? res_pair:'..i..":"..tostring(pair))
        else
            if state ~= nil then
--                print('?? RE_pair:'..i..':'..state[i].stamp..':'..tostring(edges.infaces[state[i].stamp]))
            end
            pair = apair[math.random(#apair)]
        end
        if pair == nil then
            print('!! NO_PAIRS:'..i..'/'..nnode..':'..#anode)
            U.dump(edges.astamp)
            U.dump(edges.infaces)
            return
        end
--        print('?? pre_fit:'..#anode..':'..tostring(anode[#anode].pos)..':'..anode[#anode].r)
        circleFit(pair, ar[i], d)

--        U.dump(anode[#anode], '?? node:'..i)
    end
    for _,n in pairs(anode) do
        n.pos = n.pos + (pos or vec3(0))
--        massCenter = massCenter + n.pos
    end
--    massCenter = massCenter/#anode -- + pos
--    massCenter.z = 0
    if false and nnode == 8 then
        anode = {
            {
                r = 56.23165287237,
                pos = vec3(-260.81,45.77,75.6288681),
                stamp = '0_0', },
            {
                r = 101.7567088192,
                pos = vec3(-211.3173176,-104.2659851,75.6288681),
                ang = 5.0310211785822,
                stamp = '0_1', },
            {
                r = 73.582018542625,
                pos = vec3(-131.1301325,51.66247503,75.6288681),
                stamp = '1_2', },
            {
                r = 72.271762133914,
                pos = vec3(-42.05185778,-63.82945674,75.6288681),
                stamp = '2_3', },
            {
                r = 93.670418980753,
                pos = vec3(32.92513028,84.20857464,75.6288681),
                stamp = '3_4', },
            {
                r = 103.31101611965,
                pos = vec3(-225.411412,201.3360734,75.6288681),
                stamp = '1_3', },
            {
                r = 94.167013538972,
                pos = vec3(-36.466385,258.7586209,75.6288681),
                stamp = '5_6', },
            {
                r = 106.04398032296,
                pos = vec3(-55.40616376,-241.6444351,75.6288681),
                stamp = '2_4', },
        }
    end

    if false then

        anode = {

            {
                r = 99.269598795442,
                pos = vec3(-295.65,35.83,72.0375061),
                stamp = '0_0', },
            {
                r = 69.109113803938,
                pos = vec3(-393.7719336,172.6637568,72.0375061),
                ang = 2.1928993495521,
                stamp = '0_1', },
            {
                r = 76.120076604937,
                pos = vec3(-470.5163963,49.368156,72.0375061),
                stamp = '1_2', },
            {
                r = 40.765230543178,
                pos = vec3(-503.0901429,161.6228934,72.0375061),
                stamp = '2_3', },
            {
                r = 103.61538074011,
                pos = vec3(-421.2681243,-123.4885544,72.0375061),
                stamp = '1_3', },
            {
                r = 70.966541858009,
                pos = vec3(-246.7261819,-127.2246032,72.0375061),
                stamp = '1_5', }
        }

    end
    if false then
        anode = {
            {
                r = 95.86762335677,
                pos = vec3(-295.65,35.83,72.0375061),
                stamp = '0_0' },
            {
                r = 77.168477102244,
                pos = vec3(-261.7174646,205.5063835,72.0375061),
                ang = 1.3734163015527,
                stamp = '0_1' },
            {
                r = 86.985128777978,
                pos = vec3(-421.615975,168.3732084,72.0375061),
                stamp = '1_2' },
            {
                r = 85.265262792207,
                pos = vec3(-474.0151165,4.28625779,72.0375061),
                stamp = '1_3' },
            {
                r = 60.281420603618,
                pos = vec3(-561.0384233,120.9515101,72.0375061),
                stamp = '3_4' },
            {
                r = 55.549801652843,
                pos = vec3(-358.595332,296.2192289,72.0375061),
                stamp = '2_3' },
        }
    end

    massCenter = vec3(0,0,0)
    for _,n in pairs(anode) do
        massCenter = massCenter + n.pos
    end
--    massCenter = massCenter/#anode -- + pos
    massCenter.z = 0
    local dpos = pos - massCenter/#anode
    for _,n in pairs(anode) do
        n.pos = n.pos + dpos
    end
    massCenter = massCenter + dpos*#anode

    if true and nnode == 8 then

        anode = {
            {
                r = 56.23165287237,
                pos = vec3(-316.1867336,-209.4999149,117.4858208),
                stamp = '0_0', },
            {
                r = 101.7567088192,
                pos = vec3(-266.6940512,-359.5359,117.4858208),
                ang = 5.0310211785822,
                stamp = '0_1', },
            {
                r = 73.582018542625,
                pos = vec3(-186.5068661,-203.6074399,117.4858208),
                stamp = '1_2', },
            {
                r = 72.271762133914,
                pos = vec3(-97.42859134,-319.0993716,117.4858208),
                stamp = '2_3', },
            {
                r = 93.670418980753,
                pos = vec3(-22.45160328,-171.0613403,117.4858208),
                stamp = '3_4', },
            {
                r = 103.31101611965,
                pos = vec3(-280.7881456,-53.9338415,117.4858208),
                stamp = '1_3', },
            {
                r = 94.167013538972,
                pos = vec3(-91.84311856,3.488706003,117.4858208),
                stamp = '5_6', },
            {
                r = 106.04398032296,
                pos = vec3(-110.7828973,-496.91435,117.4858208),
                stamp = '2_4', }
        }
        massCenter = vec3(-1372.682007,-1810.163452,334.8556213)
    end
        U.dump(anode, '?? anode:'..tostring(massCenter))
    U.dump(edges.astamp)
    Net.out.inseed = true
    print('<< circleSeed:'..#anode)
    return anode
end


local function forStar(sc, path)
    local ause = U.index(path, sc)
--        print('?? for use: sc:'..sc..' sp:'..sp..' used:'..#ause)
    local star, anear = {}, {}
    for _,iuse in pairs(ause) do
        anear[#anear + 1] = iuse - 1
        anear[#anear + 1] = iuse + 1
    end
    for _,j in pairs(anear) do
        if j > 0 and j <= #path then
            if path[j] ~= 0 and #U.index(star, path[j]) == 0 then
                star[#star + 1] = path[j]
            end
        end
    end
    return star
end


local function pathsUp()
    print('>> pathsUp:'..#anode)
    apath = {}
    local path, isloop = {}, false
    local ib = 1
    path[#path + 1] = ib
    local astep = {}
    local n = 0
    local cp = path[#path]

    local function forOdd()
        local afree = {}
        for i = 1,#anode do
            if #forStar(i, jointpath) % 2 == 1 then
                afree[#afree + 1] = i
            end
        end
        return afree
    end

    local nmax = #anode*2
    while not isloop and n < nmax do
        n = n + 1
                if #path == 1 then
--                    print('?? path_start:'..path[1])
                end
        -- get directions
--        cp = path[#path]
        local ap = {}
        local dma = 0
        for i,n in pairs(anode) do
            local step = U.stamp({cp, i})
            if i ~= cp and #U.index(astep, step) == 0 then
                local d, ang = nil, 0
                if #path > 1 and i ~= path[#path - 1] then
                    ang = U.vang(n.pos - anode[cp].pos, anode[cp].pos - anode[path[#path - 1]].pos)
                    d = (anode[cp].pos - n.pos):length()
                elseif #path == 1 then
                    d = (anode[cp].pos - n.pos):length()
                end
                if d ~= nil then
                    if d > dma then
                        dma = d
                    end
                    local used = #U.index(path, i)
                    ap[#ap + 1] = { ind = i, d = d, ang = ang, used = used }
                end
            end
        end
                if #path == 1 then
--                    print('?? path_start2:'..#ap)
                end
        -- pick step
        local mi, imi = 1/0, nil
        local crossed, xc, yc
        for o,p in pairs(ap) do
            local w = wa * p.d/dma + wb * p.ang/math.pi + (1 - (wa + wb)) * p.used
            if w < mi then
                mi = w
                imi = p.ind
                -- TODO: check for "free" intersections
                crossed = nil
                local function forCross(pth)
                    for j =2,#pth do
                        xc, yc = U.segmentCross(
                            anode[pth[j - 1]].pos, anode[pth[j]].pos,
                            anode[cp].pos, anode[imi].pos)
                        if yc ~= nil then
        --                    print('??++++ HAS_CROSS:'..cp..':'..p.ind)
--                            crossed = U.stamp({pth[j - 1], pth[j]})
--                            break
                            return U.stamp({pth[j - 1], pth[j]})
                        end
                    end
                end
                for _,pth in pairs(apath) do
                    crossed = forCross(pth)
                    if crossed ~= nil then
                        break
                    end
                end
                if crossed == nil then
                    crossed = forCross(path)
                end
            end
--            if not crossed then
--            end
        end
        if crossed ~= nil then
--            local ic = U.split(crossed, '_')
--            local x, y = U.segmentCross(anode[cp].pos, anode[imi].pos, anode[ic[1]].pos, anode[ic[2]].pos)
--            print('?? CROSS:'..tostring(x)..':'..tostring(y))
            print('?? CROSS:'..cp..':'..imi..' X '..crossed..':'..tostring(xc)..':'..tostring(yc))
        end
        if imi ~= nil then
            if crossed ~= nil then
                -- new path
                if #path > 1 then
                    apath[#apath + 1] = path
                    for _,s in pairs(path) do
                        jointpath[#jointpath + 1] = s
                    end
                    jointpath[#jointpath + 1] = 0
--                            U.dump(path, '?? path:'..#apath)
                end
                --- find free nodes
--                        U.dump(jointpath, '?? JPATH:')
--                        U.dump(U.index(jointpath, 7), '?? i_7:')
                local afree = forOdd()
--[[
                for i = 1,#anode do
                    if #forStar(i, jointpath) % 2 == 1 then
                        afree[#afree + 1] = i
                    end
                end
]]
                if #afree == 0 then
                    print('?? NO_NODES:'..n)
                    break
                elseif #path == 1 and #afree == 1 then
                    -- TODO:
                    print('!! SINGLE_FREE:')
                    break
                end
                path = { afree[1] }
                        U.dump(afree, '?? afree:')
                        U.dump(apath[#apath], '?? path:'..path[1])
--                path = { afree[math.random(#afree)] }
                cp = path[#path]
                print('?? HAS_CROSS:'..n..'/'..(#anode*2)..':'..tostring(cp)..'/'..#afree)
--                        break
            else
                -- check for shortcut
                local d = (anode[cp].pos - anode[imi].pos):length()
                for j = 1,#anode do
                    if j ~= cp and
                    (anode[cp].pos - anode[j].pos):length() < d and
                    U.vang(anode[cp].pos - anode[j].pos, anode[cp].pos - anode[imi].pos) < 0.2 then
                        print('??+++++++ FOR_SC:'..cp..':'..imi..'>'..j)
                        imi = j
                        astep[#astep + 1] = U.stamp({cp, imi})
                        break
                    end
                end
                astep[#astep + 1] = U.stamp({cp, imi})
                cp = imi
                path[#path + 1] = imi
            end
        else
            print('?? NO_IMI:')
            break
        end
        if n >= nmax then
            local afree = forOdd()
            if #afree > 0 then
                U.dump(afree, '?? MORE:')
                --TODO: increase nmax
            end
        end
--        cp = path[#path]
    end
--            local x,y = U.segmentCross(anode[8].pos, anode[15].pos, anode[1].pos, anode[13].pos)
--            print('?? FOR_CROSS:'..x..':'..y)
--    apath[#apath + 1] = path
    U.dump(anode, '<< pathsUp:'..#jointpath..'/'..#anode..':'..#apath)
    U.dump(apath, '?? JP:')
end


local function path2decal(rd, w)
    print('>> path2decal:'..#rd.path)
    local path = rd.path
            U.dump(path)
    local newRoadID = editor.createRoad(
        {},
    --    {{pos = apos[1], width = w, drivability = 1}},
        {overObjects = true, drivability = 1.0}
    )
    local n = 0
    local dec2path = {} -- decal node index to region node number
    local path2dec = {}
    rd.id = newRoadID
    for i = 1,#path do
--        print('?? for_node:'..i..':'..path[i]..'<'..tostring(path[i - 1]))
        local h = 0
        -- check fan
        local function isObtuse(pth, p, dbg)
            if p == nil then
                return nil
            end
            local star = forStar(p, jointpath)
            local aang = linkOrder(p, star)
            if #aang > 2 then
                for o = 2,#aang do
                    if aang[o][2] - aang[o - 1][2] > 7/6*math.pi then
                        return aang[o - 1][1], p,  aang[o][1]
                    end
                end
                if aang[1][2] - aang[#aang][2] + 2*math.pi > 7/6*math.pi then
                    return aang[#aang][1], p, aang[1][1]
                end
            end
        end
        local nb, nc, ne
        local tounfold = false
        nb, nc, ne = isObtuse(path, path[i - 1])
        if nc == path[i - 1] and (nb == path[i] or ne == path[i]) then
--            print('??==== obt1: >'..path[i]..':'..tostring(nb)..'>'..tostring(nc)..'>'..tostring(ne))
            tounfold = true
        end
        nb, nc, ne = isObtuse(path, path[i])
        if nc == path[i] and (nb == path[i - 1] or ne == path[i - 1]) then
--            print('??==== obt2: >'..path[i]..':'..tostring(nb)..'>'..tostring(nc)..'>'..tostring(ne))
            tounfold = true
        end
        if tounfold and i > 1 then
            local pmiddle = (anode[path[i]].pos + anode[path[i - 1]].pos)/2
            local v = anode[path[i]].pos - anode[path[i - 1]].pos
            local vn = vec3(-v.y, v.x, 0):normalized()
            --?? massCenter role
            if (anode[path[i]].pos - massCenter/#anode):dot(vn) < 0 then
                vn = -vn
            end
            pmiddle = pmiddle + vn * v:length()/4
            local nodeInfo = {
                pos = pmiddle + vec3(0, 0, h),
                width = w, drivability = 1, index = n }
            editor.addRoadNode(newRoadID, nodeInfo)
            n = n + 1
        end
        local nodeInfo = {
            pos = anode[path[i]].pos + vec3(0, 0, h),
            width = w, drivability = 1, index = n }
        editor.addRoadNode(newRoadID, nodeInfo)
        n = n + 1
        dec2path[n] = i
        path2dec[i] = n
    end
            U.dump(dec2path, '?? DEC2PATH:')
    rd.dec2path = dec2path
    rd.path2dec = path2dec
    rd.w = w
    local road = scenetree.findObjectById(newRoadID)
    road:setField("material", 0, mat)

    print('<< path2decal:'..n)
    return road
end


local function linkOrder(s, star, p)
--    print('>> linkOrder')
    local aang, angp = {}, 0
    for _,i in pairs(star) do
        local v = anode[i].pos - anode[s].pos
        aang[#aang + 1] = {i, math.atan2(v.y, v.x) % (2*math.pi)}
        if p ~= nil and i == p then
            angp = aang[#aang][2]
        end
--        print('?? for_ang:'..tostring(v:normalized())..':'..aang[#aang][2])
    end
    -- relative to p
    for o,_ in pairs(aang) do
        aang[o][2] = (aang[o][2] - angp) % (2*math.pi)
    end
    local function comp(a, b)
        return a[2] < b[2]
    end
    table.sort(aang, comp)

--    U.dump(aang, '<< linkOrder:'..angp)
    return aang
end


local function roadAni(dlist, n)
    lo('>> roadAni:')
    for key,list in pairs(dlist) do
--        U.dump(list, '?? ani_LIST:'..key..':'..#list)
        if n <= #list then
            editor.addRoadNode(key, {
                pos = list[n].pos,
                width = list[n].width, drivability = 1, index = list[n].index})
        end
    end
end


local function path2decal(rd, w)
    print('>> path2decal:'..#rd.path)
    local path = rd.path
            U.dump(path)
    local newRoadID = editor.createRoad(
        {},
    --    {{pos = apos[1], width = w, drivability = 1}},
        {overObjects = false, drivability = 1.0, parent = 'edit'},
        groupEdit
    )
        print('?? path2decal.ID:'..tostring(newRoadID))
--        if true then return end

    local nodelist = {}
    local n = 0
    local dec2path = {} -- decal node index to region node number
    local path2dec = {}
    rd.id = newRoadID
    for i = 1,#path do
--        print('?? for_node:'..i..':'..path[i]..'<'..tostring(path[i - 1]))
        local h = 0
        -- check fan
        local function isObtuse(pth, p, dbg)
            if p == nil then
                return nil
            end
            local star = forStar(p, jointpath)
            local aang = linkOrder(p, star)
            if #aang > 2 then
                for o = 2,#aang do
                    if aang[o][2] - aang[o - 1][2] > 7/6*math.pi then
                        return aang[o - 1][1], p,  aang[o][1]
                    end
                end
                if aang[1][2] - aang[#aang][2] + 2*math.pi > 7/6*math.pi then
                    return aang[#aang][1], p, aang[1][1]
                end
            end
        end
        local nb, nc, ne
        local tounfold = false
        nb, nc, ne = isObtuse(path, path[i - 1])
        if nc == path[i - 1] and (nb == path[i] or ne == path[i]) then
--            print('??==== obt1: >'..path[i]..':'..tostring(nb)..'>'..tostring(nc)..'>'..tostring(ne))
            tounfold = true
        end
        nb, nc, ne = isObtuse(path, path[i])
        if nc == path[i] and (nb == path[i - 1] or ne == path[i - 1]) then

--            print('??==== obt2: >'..path[i]..':'..tostring(nb)..'>'..tostring(nc)..'>'..tostring(ne))
            tounfold = true
        end

        if tounfold and i > 1 then
            local pmiddle = (anode[path[i]].pos + anode[path[i - 1]].pos)/2
--                lo('?? unfolding:'..path[i-1]..'>'..path[i]..':'..tostring(massCenter)..':'..tostring(anode[path[i]].pos))
            local v = anode[path[i]].pos - anode[path[i - 1]].pos
            local vn = -vec3(-v.y, v.x, 0):normalized()
            --?? massCenter role
            if (U.proj2D(anode[path[i]].pos) - massCenter/#anode):dot(vn) < 0 then
                vn = -vn
            end
            pmiddle = pmiddle + vn * v:length()/4
            local nodeInfo = {
                pos = pmiddle + vec3(0, 0, h),
                width = w, drivability = 1, index = n }
            editor.addRoadNode(newRoadID, nodeInfo)
            nodelist[#nodelist+1] = {pos = nodeInfo.pos, ind = 0, tp = 1, w = w}
            n = n + 1
        end
        local nodeInfo = {
            pos = anode[path[i]].pos + vec3(0, 0, h),
            width = w, drivability = 1, index = n }
        editor.addRoadNode(newRoadID, nodeInfo)
        nodelist[#nodelist+1] = {pos = nodeInfo.pos, ind = path[i], tp = 0, w = w}
        n = n + 1
        dec2path[n] = i
        path2dec[i] = n
    end
    rd.dec2path = dec2path
    rd.path2dec = path2dec
    rd.w = w
    local road
    local road = scenetree.findObjectById(newRoadID)
    groupEdit:add(road)
--    road:delete()
--[[
        road:setField("material", 0, mat)
        road:setField('distanceFade', 0, string.format("%f %f", 0, 200))
        groupEdit:add(road)
]]
    if true then
        -- split to segments
        local nedge = road:getEdgeCount()
    --    local acnode = editor.getNodes(road)
    --        U.dump(dec2path, '?? DEC2PATH:'..tostring(nedge)..':'..#acnode)
        local aseg = {}
        local cseg = {}
        local cedge = 0
--            U.dump(nodelist, '?? NLIST:'..#nodelist)
        for k,n in pairs(nodelist) do
    --    for k,n in pairs(acnode) do
            for i = cedge,nedge do
                local epos = road:getMiddleEdgePosition(i)
                if U.proj2D(n.pos - epos):length() < 0.1 then
    --                lo('?? HIT:'..k..'<'..i..':'..tostring(n.pos)..':'..tostring(nodelist[k].pos))
                    cedge = i + 1
                    if #cseg == 0 then
                        cseg[#cseg + 1] = {pos = n.pos, ind = n.ind}
                        cseg[#cseg + 1] = {pos = road:getMiddleEdgePosition(i+1), ind = n.ind}
                        cseg[#cseg + 1] = {pos = road:getMiddleEdgePosition(i+4), ind = n.ind}
    --                    lo('?? tos1:')
                    elseif n.tp == 1 then
                        cseg[#cseg + 1] = {pos = n.pos, ind = 0, tp = 1}
                    else
                        cseg[#cseg + 1] = {pos = road:getMiddleEdgePosition(i-4), ind = n.ind}
                        cseg[#cseg + 1] = {pos = road:getMiddleEdgePosition(i-1), ind = n.ind}
                        cseg[#cseg + 1] = {pos = n.pos, ind = n.ind}
                        aseg[#aseg+1] = cseg
                        cseg = {
                            {pos = n.pos, ind = n.ind},
                            {pos = road:getMiddleEdgePosition(i+1), ind = n.ind},
                            {pos = road:getMiddleEdgePosition(i+4), ind = n.ind},
                        }
    --                    lo('?? tos2:')
                    end
                    break
                end
            end
        end
        road:delete()
--            U.dump(aseg, '?? ASEG:'..#aseg)
        local ani = {}
        local inani --= true
        -- segments to roads
        local step = 15
        local astep = 8
--        for i =1,1 do
--            local s = aseg[i]
        for i,s in pairs(aseg) do
--                U.dump(s, '?? for_seg:'..i)
            newRoadID = editor.createRoad(
                {},
            --    {{pos = apos[1], width = w, drivability = 1}},
                {overObjects = false, drivability = 1.0, parent = 'edit'}
    --            ,groupEdit
            )
            road = scenetree.findObjectById(newRoadID)
            road:setField("material", 0, mat)
            road:setField('distanceFade', 0, string.format("%f %f", 0, 200))
            groupEdit:add(road)
            ani[newRoadID] = {}
            local cani = ani[newRoadID]

            if true then
                local an = {s[3]}
                for _,n in pairs(s) do
                    if n.tp and n.tp == 1 then
                        an[#an+1] = n
                    end
                end
                an[#an+1] = s[#s-2]
--                U.dump(an, '?? AN:')

                local ci = 0

                -- 2 at the first cross
                for k = 1,2 do
                    editor.addRoadNode(newRoadID, {
                        pos = s[k].pos, --material = 'road1_concrete', -- mat,
                        width = w, drivability = 1, index = ci})
                    ci = ci + 1
                end

                if false then

                    for k=1,#an-1 do
                        local sc,sp,sn = an[k].pos
                        local hc = core_terrain.getTerrainHeight(sc)

                        -- first
                        cani[#cani+1] = {pos = sc, width = w, index = ci}
                        if not inani then
                            editor.addRoadNode(newRoadID, {
                                pos = sc,
                                width = w, drivability = 1, index = ci})
                        end
                        ci = ci + 1

    --                    local v = an[k+1].pos-an[k].pos
    --                    v = v:normalized()
                        local htgt = core_terrain.getTerrainHeight(an[k+1].pos)
                        local a,b = (an[k].pos-sc):length(),(an[k+1].pos-sc):length()
    --                        lo('?? for_VVV:'..tostring(an[k+1]))
                        local v = an[k+1].pos - sc
                        local h = core_terrain.getTerrainHeight(sc)
                        local dh = (htgt - h)/v:length()
                        v = v:normalized()
                        local svmi
                            local N = 0
                        while b > 1.5*step and N < 40 do
                            N = N+1
                            -- look around
                            local hmi,vmi,ami = math.huge,v --astep/2
                            for ia = 1, astep-1 do
                                local a = (ia/astep - 1/2)*math.pi
            --                        lo('?? for_s:'..tostring(sc)..':'..tostring(v)..':'..tostring(a))
                                local cp = sc + step*U.vturn(v, a)
                                local ch = core_terrain.getTerrainHeight(cp)
                                local cdh = (ch - h)/step
                                if math.abs(cdh - dh) < hmi then
    --                            if ch < hmi then
                                    hmi = math.abs(cdh - dh)
                                    vmi = U.vturn(v, a)
                                    ami = a
            --                        imi = ia
                                end
                            end
                            sc = sc + step*(a*v + b*(vmi+(svmi or vec3(0,0,0))))/(a+b)

                            cani[#cani+1] = {pos = sc, width = w, index = ci}
                            if not inani then
                                editor.addRoadNode(newRoadID, {
                                    pos = sc,
                                    width = w, drivability = 1, index = ci})
                            end
                            ci = ci + 1

                            a,b = (an[k].pos-sc):length(),(an[k+1].pos-sc):length()
    --                            lo('?? for_AB:'..N..':'..a..':'..b..':'..ami)
                            v = an[k+1].pos - sc
                            h = core_terrain.getTerrainHeight(sc)
                            dh = (htgt - h)/v:length()
                            v = v:normalized()
                            svmi = vmi
                        end
                    end

                end

                -- last
                cani[#cani+1] = {pos = an[#an].pos, width = w, index = ci}
                if not inani then
                    editor.addRoadNode(newRoadID, {
                        pos = an[#an].pos,
                        width = w, drivability = 1, index = ci})
                end
                ci = ci + 1

                -- 2 at the first cross
                for k = #s-1,#s do
                    cani[#cani+1] = {pos = s[k].pos, width = w, index = ci}
                    if not inani then
                        editor.addRoadNode(newRoadID, {
                            pos = s[k].pos,
                            width = w, drivability = 1, index = ci})
                    end
                    ci = ci + 1
                end

            elseif true then
                for k = 1,#s do
--                    U.dump(s[k], '?? for_node:')
                    editor.addRoadNode(newRoadID, {
                        pos = s[k].pos, --material = 'road1_concrete', -- mat,
                        width = w, drivability = 1, index = k-1})
                end
            elseif false then

                -- 2 at the first cross
                local ci = 0
                for k = 1,3 do
                    editor.addRoadNode(newRoadID, {
                        pos = s[k].pos, --material = 'road1_concrete', -- mat,
                        width = w, drivability = 1, index = ci})
                    ci = ci + 1
                end

                if false then

                    local sc,sp,sn = s[2].pos
                    local hc = core_terrain.getTerrainHeight(sc)

                    local v = s[j+1].pos-s[j].pos
                    v = v:normalized()
                    local a,b = (s[j-1].pos-sc):length(),(s[j+1].pos-sc):length()

                    while b > step do
                        -- look around
                        local hmi,vmi = math.huge,v --astep/2
                        for ia = 1, astep-1 do
                            local a = (ia/astep - 1/2)*math.pi
        --                        lo('?? for_s:'..tostring(sc)..':'..tostring(v)..':'..tostring(a))
                            local cp = sc + step*U.vturn(v, a)
                            local h = core_terrain.getTerrainHeight(cp)
                            if h < hmi then
                                hmi = h
                                vmi = step*U.vturn(v, a)
        --                        imi = ia
                            end
                        end
                            lo('?? for_dir:'..i..':'..tostring(vmi))
        --                    a = 0
        --                    b = 1
                        sc = sc + (a*v + b*vmi)/(a+b)
                        editor.addRoadNode(newRoadID, {
                            pos = sc, --material = 'road1_concrete', -- mat,
                            width = w, drivability = 1, index = ci})
                        ci = ci + 1
                        break
                    end

                end
                -- 2 at the first cross
                for k = #s-2,#s do
                    editor.addRoadNode(newRoadID, {
                        pos = s[k].pos, --material = 'road1_concrete', -- mat,
                        width = w, drivability = 1, index = ci})
                        ci = ci + 1
                end

            end
                if i == 10 then
                    lo('??+++++++++++++++++++ for_SEG:'..newRoadID)
                    Net.cid = newRoadID
                end
        end
--            U.dump(ani, '?? for_ANI:')
        local nma = 0
        for _,s in pairs(ani) do
            if #s > nma then
                nma = #s
            end
        end
--        inanim = {ain = ani, cb = roadAni, n = 0, N = nma}
--            U.dump(inanim, '?? for_ANI:')
        return
    else
    end
--    baseroad:delete()
--[[
    for i=0,nedge-1 do
        local epos = road:getMiddleEdgePosition(i)
        for k,n in pairs(acnode) do
            if (n.pos - epos):length() < 0.1 then
                lo('?? HIT:'..i..'>'..k)
            end
        end
    end
]]
--        scenetree.findObjectById(newRoadID):delete()

    print('<< path2decal:'..n..':'..newRoadID)
--    return
    return road
end


local function levelsUp(pth, iground)
    local alev, levels = {}, {}
        lo('>> levelsUp:'..#pth)
    if #pth == 0 then return {} end
    local function toLevels(pth, start, dir, clev)
        local clvl = 1
        if clev ~= nil then
            clvl = clev
        end
        local ib, ie = start, #pth
        if dir == -1 then
            ie = 1
        end
--            U.dump(pth, '?? pre_lev:'..tostring(ib)..':'..tostring(ie))
        for o = ib,ie,dir do
            local p = pth[o]
    --        for o,p in pairs(dec) do
            local set = 0
            if p then

                if alev[p] == nil then
                    alev[p] = { false, false, false, false, false }
                end
                if not alev[p][clvl] then
                    -- look down
                    local isdown = false
                    for i = clvl-1,1,-1 do
                        if alev[p][i] then
                            isdown = true
                            if clvl - i > 1 then
                                clvl = clvl - 1
                                break
                            end
                        end
                    end
                    if clvl > 1 and not isdown then
                        clvl = clvl - 1
                    end
                    alev[p][clvl] = true
                    set = clvl
                end
                if set == 0 then
                    local ll, lu
                    local lc
                    -- check lower levels
                    for i = clvl-1,1,-1 do
                        if not alev[p][i] then
                            alev[p][i] = true
                            ll = i
                            lc = i
                            break
                        end
                    end
                    for i = clvl+1, #alev[p] do
                        if not alev[p][i] then
                            alev[p][i] = true
                            lu = i
                            lc = i
                            break
                        end
                    end
                    if ll ~= nil and lu ~= nil then
                        if clvl - ll < lu - clvl then
                            alev[p][ll] = true
                            set = ll
                        else
                            alev[p][lu] = true
                            set = lu
                        end
                    elseif lc ~= nil then
                        alev[p][lc] = true
                        set = lc
                    end
                    clvl = set
                end

            end

--            print('?? level_set:'..o..' node:'..pth[o]..' lvl:'..set)
            levels[o] = set
        end

        return levels
    end
    print('?? GROUND:'..iground..'/'..#pth..':'..tostring(pth[iground]))
    toLevels(pth, iground, 1)
    toLevels(pth, iground - 1, -1, levels[iground])

    return levels
end


local function forShortcut(sc, aang, imi)
    local icheck
    if imi > 2 then
        icheck = imi - 1
    end
    if imi < #aang then
        icheck = imi + 1
    end
    if math.abs(aang[icheck][2] - aang[imi][2]) < 0.2 then
        if (anode[aang[icheck][1]].pos - anode[sc].pos):length()
        < (anode[aang[imi][1]].pos - anode[sc].pos):length() then
            -- make shortcut
            return icheck
        end
    end
end


local function stem(dec)
    if not dec or not dec[1] or not dec[2] then return {} end
        print('>> stem:'..dec[1]..':'..dec[2])
    local path = jointpath --apath[ord]
    local isloop = false
    local n = 0
    local sc, sp = dec[2], dec[1]
    astamp[#astamp + 1] = U.stamp({sp, sc})
    while not isloop and n < 130 do
        n = n + 1
--        print('?? for_sc:'..sc..':'..n)
        -- get the star
        local star = forStar(sc, path)
--        print('?? for_star:'..sc..':'..#star)
--        U.dump(star)
        -- get co-direction
        if true then
            --- ordered angles
--                    print('?? pre_order:'..sc.."<"..sp..':'..#star)
            local aang = linkOrder(sc, star, sp)
            --- pick opposite branch
--                U.dump(aang, '?? aang:')
            local imi
            if #aang % 2 == 0 then
                imi = #aang/2 + 1
            else
                local a1, a2 = aang[(#aang-1)/2 + 1], aang[(#aang-1)/2 + 2]
                        if a2 == nil then
                            print('!! ERR_a2:'..#aang..':'..sc.."<"..sp..':'..aang[1][1]..' star:'..#star)
                        end
                if math.abs(a1[2] - math.pi) < math.abs(a2[2] - math.pi) then
                    imi = (#aang-1)/2 + 1
                else
                    imi = (#aang-1)/2 + 2
                end
            end
            if #aang > 2 and true then
                -- check for shortcuts
                local ialt = forShortcut(sc, aang, imi)
                if ialt ~= nil then
                    U.dump(aang, '??********** FOR_SC:'..sc..':'..aang[imi][1]..'>'..aang[ialt][1])
                end
                if false then
                    dec[#dec + 1] = aang[ialt][1]
                    astamp[#astamp + 1] = U.stamp({sc, aang[ialt][1]})
                    dec[#dec + 1] = aang[imi][1]
                    astamp[#astamp + 1] = U.stamp({sc, aang[imi][1]})
                    sp = aang[ialt][1]
                    sc = dec[#dec]
                    imi = nil
                end
            end
            if imi ~= nil then
--                print('?? co_DIR:'..n..':'..sp..'>'..sc..'>'..aang[imi][1])
                local stamp = U.stamp({ sc, aang[imi][1] })
                if #U.index(astamp, stamp) > 0 then
                    print('?? LOOPED:'..n..':'..sc..'>'..aang[imi][1])
                    isloop = true
                else
                    dec[#dec + 1] = aang[imi][1]
                    astamp[#astamp + 1] = stamp
                    sp = sc
                    sc = dec[#dec]
                end
            end
        end
    end

--    U.dump(dec, '?? dec:')
--    U.dump(astamp, '<< stem:')
    return dec
end


local marginSide, marginEnd = 0.1, 0.2

local function decal2mesh(road, nm, dbg)
--        if true then return end
    if nm == nil then
        nm = 'road'
    end
    local hmap = road.hmap
    local rd = scenetree.findObjectById(road.id)
    local adec = editor.getNodes(rd)
    local nsec = rd:getEdgeCount()
    print('>> decal2mesh:'..nsec..':'..#road.apin)

    local av, auv, an, af = {}, {}, {}, {}
    local av2, auv2, an2, af2 = {}, {}, {}, {}
    an[#an+1] = {x = 0, y = 0, z = 1}
    an2[#an2+1] = {x = 0, y = 0, z = -1}

    auv[#auv + 1] = {u = 0, v = 1}
    auv[#auv + 1] = {u = 0, v = 0}
    auv[#auv + 1] = {u = 1, v = 0}
    auv[#auv + 1] = {u = 1, v = 1}

--[[
    if false then
        av[#av + 1] = vec3(2, -5, 1)
        av[#av + 1] = vec3(0, -5, 1)
        av, af = Mesh.strip(2, {vec3(2, -7, 1), vec3(0, -7, 1)}, av, af)

        local rdm = createObject("ProceduralMesh")
--        rdm:setPosition(vec3(0, 0, 0))
        rdm:setPosition(vec3(0, 0, 0))
        rdm.isMesh = true
        rdm.canSave = false
        rdm:registerObjectWithIdSuffix(nm)
--        scenetree.MissionGroup:add(rdm.obj)
        groupEdit:add(rdm.obj)
        rdm:createMesh({{{
            verts = av,
            uvs = auv,
            normals = an,
            faces = af,
            material = "WarningMaterial",
        }}})
        return
    end
]]

    auv2[#auv2 + 1] = {u = 0, v = 1}
    auv2[#auv2 + 1] = {u = 0, v = 0}
    auv2[#auv2 + 1] = {u = 1, v = 0}
    auv2[#auv2 + 1] = {u = 1, v = 1}

    local h1, h2 = hmap[0], hmap[road.apin[2][1]]
    local dirb = (rd:getMiddleEdgePosition(0) - rd:getMiddleEdgePosition(1)):normalized()
    local dire = (rd:getMiddleEdgePosition(nsec - 1) - rd:getMiddleEdgePosition(nsec - 2)):normalized()
    local perp = (rd:getRightEdgePosition(0) - rd:getLeftEdgePosition(0)):normalized()

    local h = h1
--    av[#av+1] = rd:getLeftEdgePosition(0) - marginSide*perp + marginEnd*dirb
--    av[#av].z = 1
    av[#av+1] = rd:getLeftEdgePosition(0) - marginSide*perp + marginEnd*dirb
    av[#av].z = h1
    av[#av+1] = rd:getRightEdgePosition(0) + marginSide*perp + marginEnd*dirb
    av[#av].z = h1
--    av[#av+1] = rd:getRightEdgePosition(0) + marginSide*perp + marginEnd*dirb
--    av[#av].z = 1

    av2[#av2 + 1] = rd:getRightEdgePosition(0) + marginSide*perp + marginEnd*dirb
    av2[#av2].z = h1 - 0.2
    av2[#av2 + 1] = rd:getLeftEdgePosition(0) - marginSide*perp + marginEnd*dirb
    av2[#av2].z = h1 - 0.2

            if dbg == true then
                U.dump(road.apin, '?? mesh_apin:'..nsec)
            end
    local L = 2
    local cpin = 2
    for i = 1,nsec - 1 do
        -- set height
        if i > road.apin[cpin][1] then
            cpin = cpin + 1
            h1 = h2
            h2 = hmap[road.apin[cpin][1]]
            if dbg == true then print('?? mesh_cpin:'..i..':'..cpin) end
--            print('??-------- next:'..i..':'..cpin..':'..road.apin[cpin - 1])
        end
--        print('?? for_h: i='..i..' h1:'..h1..' h2:'..h2..' cpin:'..cpin..':'..tostring(road.apin[cpin - 1])..':'..tostring(road.apin[cpin]))
        if road.apin[cpin-1][2] == 1 and road.apin[cpin][2] == 1 then
            if dbg == true then print('?? mesh_spline1:'..i..':'..(cpin-1)..'>'..cpin) end
            h = U.spline1(h1, h2, i - road.apin[cpin - 1][1], road.apin[cpin][1] - road.apin[cpin - 1][1])
        else
--            h = U.spline1(h1, h2, i - road.apin[cpin - 1][1], road.apin[cpin][1] - road.apin[cpin - 1][1])
            h = U.spline2(h1, h2, i - road.apin[cpin - 1][1], road.apin[cpin][1] - road.apin[cpin - 1][1])
            if dbg then
                print('?? for_i:'..i..' cpin:'..cpin..' ap:'..road.apin[cpin - 1][1]..' h:'..h..'/'..h2)
            end
        end
--                h = 0.2
        -- perp
        local perp = U.proj2D(rd:getRightEdgePosition(i) - rd:getLeftEdgePosition(i)):normalized()
        perp.z = 0

        local apos = {
--            rd:getLeftEdgePosition(i) - marginSide*perp + vec3(0, 0, h + 1),
            U.proj2D(rd:getLeftEdgePosition(i) - marginSide*perp) + vec3(0, 0, h),
            U.proj2D(rd:getRightEdgePosition(i) + marginSide*perp) + vec3(0, 0, h),
--            rd:getRightEdgePosition(i) + marginSide*perp + vec3(0, 0, h + 1),
        }
        av, af = Mesh.strip(2, apos, av, af)
--        av, af = Mesh.strip(L, apos, av, af)

        -- back
        apos = {
--            rd:getLeftEdgePosition(i) - marginSide*perp + vec3(0, 0, h + 1),
            U.proj2D(rd:getRightEdgePosition(i) + marginSide*perp) + vec3(0, 0, h - 0.2),
            U.proj2D(rd:getLeftEdgePosition(i) - marginSide*perp) + vec3(0, 0, h - 0.2),
--            rd:getRightEdgePosition(i) + marginSide*perp + vec3(0, 0, h + 1),
        }
        av2, af2 = Mesh.strip(2, apos, av2, af2)
    end

    local rdm = createObject("ProceduralMesh")
    rdm:setPosition(vec3(0, 0, 0))
    rdm.isMesh = true
    rdm.canSave = false
    rdm:registerObjectWithIdSuffix(nm)
    --    aidMesh[#aidMesh+1] = id
    groupEdit:add(rdm.obj)
--    scenetree.MissionGroup:add(rdm.obj)
    rdm:createMesh({{{
        verts = av,
        uvs = auv,
        normals = an,
        faces = af,
        material = "WarningMaterial",
    }}})
 --   rdm.avb = av
 --   rdm.avstep = L
    road.meshid = id
    road.av = av
    road.avstep = L
--    be:reloadCollision()
    ameshID[#ameshID + 1] = id

    return rdm
end


local function decalUpdate(dec, hmap, apin, dbg)
    be:reloadCollision()
        lo('>> decalUpdate:'..tostring(#hmap)..':'..tostring(#apin))
--    local dec = scenetree.findObjectById(id)
    local adec = editor.getNodes(dec)
--        print('?? to_del:'..r.id..':'..#adec..':'..tostring(r.meshid)..':'..#r.av..':'..nsec)

    -- reset heights
    for o,n in pairs(adec) do
        if dbg then
--            print('?? for_UPD:'..o..'/'..#adec..':'..tostring(apin[o]))
        end
--                print('?? np:'..o..':'..tostring(n.pos))
        n.pos.z = hmap[apin[o][1]] + 0.2
--                editor.setNodePosition(rd, o, n.pos + 1*vec3(0, 0, hmap[r.apin[o]] + 0.2))
    end
    dec:delete()
-- update decal
    local idnew = editor.createRoad(adec,
        {overObjects = true, drivability = 1.0}
    )
--    r.id = id
--    rdmesh.decal = id
    local road = scenetree.findObjectById(idnew)
    road:setField("material", 0, mat)

    adecalID[#adecalID + 1] = idnew

    return idnew
end


local function toDecals()
        print('>>-------------------------------- toDecals:'..#apath..':'..tostring(#jointpath))
    exits = {}
    astamp = {}
    local list1 = stem({jointpath[1], jointpath[2]})
    local list2 = stem({jointpath[2], jointpath[1]})
    local dec = {}
    for i = #list2,3,-1 do
        dec[#dec + 1] = list2[i]
    end
    for i = 1,#list1 do
        dec[#dec + 1] = list1[i]
    end
    -- levels
    --            print('?? lvl_start:'..dec[#list2 - 1]..':'..dec[#list2 - 2]..':'..dec[#list2 - 3])
    local levels = levelsUp(dec, #list2 - 1)
            U.dump(levels, '?? LEVELS:'..#dec)
    roads[#roads + 1] = {
        id = nil, path = dec, levels = levels,
        dec2path = {}, meshid = nil,
        av = {}, avstep = nil,
        hmap = {}, apin = {},
        decal = nil, -- base decal reference
    }
    -- build decal
    for _,r in pairs(roads) do
        local rd = path2decal(r, wHeighway)
--            if true then return end
    --        r.decal = rd
        if true and rd then

            -- mesh
            --- get height map
            local adec = editor.getNodes(rd)
            local nsec = rd:getEdgeCount()
            local hmap, apin = {}, {}
    --                U.dump(levels, '?? LEV:')
    --                U.dump(r.dec2path, '?? dec2path:'..nsec)
            local cdec = 1
            local chight
            for i = 0,nsec - 1 do
                local pedge = rd:getMiddleEdgePosition(i)
                for j = cdec,#adec do
                    local n = adec[j]
                    if (pedge - n.pos):length() < 0.01 then
                        if levels[r.dec2path[j]] == nil then
                            hmap[i] = chight
                                lo('?? to_hmap:'..i..':'..tostring(chight))
                        else
                            hmap[i] = levels[r.dec2path[j]]
                            chight = hmap[i]
                        end
                        hmap[i] = (hmap[i] - 1) * hLevel + pedge.z
                        apin[#apin + 1] = {i}
                            lo('??__ APH:'..i..':'..tostring(hmap[i])..':'..tostring(pedge)) --#apin)
        --                    print('?? e2n:'..i..':'..j..':'..tostring(levels[r.dec2path[j]])..':'..tostring(hLevel))
                        cdec = j + 1
                        break
                    end
                end
            end
--                if true then return end
            --- last
            hmap[nsec - 1] = (levels[r.dec2path[#adec]] - 1) * hLevel
            apin[#apin + 1] = {nsec - 1}
        --                print('?? FOR_LAST:'..tostring(rd:getMiddleEdgePosition(nsec - 1))..':'..tostring(rd:getMiddleEdgePosition(nsec))..':'..tostring(adec[#adec].pos)..':'..#adec)
            r.hmap = hmap
            r.apin = apin
--                U.dump(r, '?? for_road:')

            --- mesh
            if true then
                local rdmesh = decal2mesh(r, 'road')
--                    if true then return end
                if true then
                    lo('??***************************** if_mesh:')
                    --- update decal
--                    print('?? PRE_UP: nsec='..nsec..' adec='..#adec..':'..tostring(rd)..'::'..#hmap..'::'..tostring(#r.apin))
--                        U.dump(hmap, '?? hmap:')
--                        U.dump(r.apin, '?? apin:')
                    local id = decalUpdate(rd, hmap, r.apin) --, true)
                    r.id = id
                    rdmesh.decal = id
                end
            end
        end
--[[
]]
    end
    U.dump(dec, '<< toDecals:'..#roads)
end


local dex = 30 -- distance from center to exit strt
local dexpre = 10 -- distance of pre exit decal point to exit
local dexpree = 5 -- distance of pre exit on exit

local function av2hmap(dec, list, dbg)
--            print('>> av2hmap:'..tostring(dbg))
    local hmap, apin = {}, {}
    for i = 1,#list do
        local ind, rdinfo, ei = list[i][1], list[i][2], list[i][3]
--        if ind == 11 then
--            dbg = true
--        else
--            dbg = false
--        end
        if ei == nil then
            hmap[ind] = list[i][2]
            apin[#apin + 1] = {ind}
        else
--            local branch = scenetree.findObjectById(rdinfo.id)
                if dbg then
--                    adbg[#adbg + 1] = {dec:getMiddleEdgePosition(ind), ColorF(1,1,0,1)}
                end
--                print('?? av2hmap:'..ind..':'..tostring(dec:getMiddleEdgePosition(ind))..':'..ei)
            local h = U.hightOnCurve(dec:getMiddleEdgePosition(ind), rdinfo, ei, dbg)
--            local h = U.hightOnCurve(
--                U.proj2D(dec:getMiddleEdgePosition(ind)),
--                branch, ei, rdinfo.av, rdinfo.avstep) --, true)
            if h ~= nil then
                hmap[ind] = h
                apin[#apin + 1] = {ind}
            else
                print('!! av2hmap.NO_H:'..ind..':'..tostring(dbg))
            end
        end
    end

    return hmap, apin
end

local function pos(i, side, rdinfo)
    if side == 1 then
        -- ritgh edge
        return U.proj2D(rdinfo.av[(i + 1)*rdinfo.avstep])
    else
        -- left edge
        return U.proj2D(rdinfo.av[i*rdinfo.avstep + 1])
    end
end

local function posAtDistance(ifrom, d, side, dir, rdinfo)
    local s, ipre = 0, nil
    for i = ifrom, ifrom + dir*100, dir do
        local v = pos(i + dir, side, rdinfo) - pos(i, side, rdinfo)
        s = s + v:length()
        if s > d then
--            print('<< posAtDistance:'..ifrom..':'..i..':'..(s - d)..':'..(v:length()))
            return pos(i, side, rdinfo) + v:normalized()*(d - (s - v:length()))
        end
    end
end

local function exitPos(rdinfo, ind, side, dir)
    local vn = (pos(ind, side, rdinfo) - pos(ind, -side, rdinfo)):normalized()
    local p = pos(ind, side, rdinfo) - vn*marginSide
    local pp = posAtDistance(ind, dexpre, side, dir, rdinfo) - vn*(wExit/2 + marginSide)

    return p, pp
end

local function edgeAtDist(ifrom, d, side, dir, rdinfo) --av, step)
    local s, ipre = 0, nil
    for i = ifrom, ifrom + dir*100, dir do
        s = s + U.proj2D(pos(i + dir, side, rdinfo) - pos(i, side, rdinfo)):length()
        if s > d then
           return i + dir
        end
    end
end

local function forOut(rdinfo, ei, dec, dir, dbg)
    if dir == nil then
        dir = 1
    end
    local ne = dec:getEdgeCount()
    local ib, ie = 1, ne - 1
    if dir == -1 then
        ib = ne - 1
        ie = 1
    end
    local droot = scenetree.findObjectById(rdinfo.id)
    local dp
    for i = ib,ie,dir do -- 0-based
        local d = U.proj2D(dec:getMiddleEdgePosition(i)):distanceToLine(
            U.proj2D(droot:getMiddleEdgePosition(ei + dir)),
            U.proj2D(droot:getMiddleEdgePosition(ei))
        )
                if dbg then
                    print('?? forOut:'..i..':'..d..':'..tostring(dp))
                end
--            adbg[#adbg + 1] = {droot:getMiddleEdgePosition(ei + 1), ColorF(1,1,0,1)}
--            adbg[#adbg + 1] = {droot:getMiddleEdgePosition(ei - 1), ColorF(1,1,0,1)}
--            print('?? forOut:'..i..'/'..ne..':'..d)
        if dp ~= nil and d > dp and d > (rdinfo.w + wExit)/2 + 2*marginSide then
            return i
        end
        dp = d
    end
end

local function forIn(rdinfo, ei, dec)
    return forOut(rdinfo, ei, dec, -1)
end


local function decalUp(apos, w, m)
    local an = {}
    for _,p in pairs(apos) do
        an[#an + 1] = {pos = p, width = w}
    end
    local id = editor.createRoad(an,
        {overObjects = true, drivability = 1.0}
    )
    --    {{pos = apos[1], width = w, drivability = 1}},
    local road = scenetree.findObjectById(id)
    if m == nil then
        m = mat
    end
    road:setField("material", 0, m)

    return road
end


local function hmap4decal(rd, av, step)
    local adec = editor.getNodes(rd)
    local nsec = rd:getEdgeCount()
    local hmap, apin = {}, {}
    local cdec = 1
    local chight
    for i = 0,nsec - 1 do
        local pedge = rd:getMiddleEdgePosition(i)
        for j = cdec,#adec do
            local n = adec[j]
            if U.proj2D(pedge - n.pos):length() < 0.01 then
                hmap[i] = av[i*step + 1].z
                apin[#apin + 1] = {i}
                break
            end
        end
    end
    --- last
    hmap[nsec - 1] = av[(nsec - 1)*step + 1].z
    apin[#apin + 1] = {nsec - 1}

    return hmap, apin
end


-- looped exit
local function eLoop(cpos, brout, dirout, brin, dirin)
    print('>> eLoop:'..dirout..':'..dirin)

    local ieo = edgeAtDist(brout[2], 1, dirout, dirout, roads[brout[1]])
    local pout, ppout = exitPos(roads[brout[1]], ieo, dirout, -dirout)
    local iei = edgeAtDist(brin[2], 1, dirin, -dirin, roads[brin[1]])
    local pin, ppin = exitPos(roads[brin[1]], iei, dirin, dirin)

            adbg[#adbg + 1] = {pout}
            adbg[#adbg + 1] = {ppout, ColorF(0,0,1,1)}
            adbg[#adbg + 1] = {pin}
            adbg[#adbg + 1] = {ppin, ColorF(0,0,1,1)}
    local vtop = dex*3/9*((pout - cpos) + (pin - cpos)):normalized()
    local an = {ppout, pout}
    an[#an + 1] = pout + 0.65*U.vturn(vtop, math.pi/8)
    an[#an + 1] = (pout + pin)/2 + vtop
    an[#an + 1] = pin + 0.65*U.vturn(vtop, -math.pi/8)
    an[#an + 1] = pin
    an[#an + 1] = ppin
    ---- base decal
    local edec = decalUp(an, wExit)

    -- geometry
    local jo = forOut(roads[brout[1]], ieo, edec) --, nil, true)
        adbg[#adbg + 1] = {edec:getLeftEdgePosition(jo)+vec3(0,0,hp), ColorF(0,1,0,1)}
    local ji = forIn(roads[brin[1]], iei, edec)
        adbg[#adbg + 1] = {edec:getLeftEdgePosition(ji)+vec3(0,0,hp), ColorF(0,1,0,1)}
        print('?? eLoop.BE:'..brout[2]..'>'..jo..':'..brin[2]..':'..ji)
    local ro, ri = roads[brout[1]], roads[brin[1]]
    local hpin = {
        {0, ro, ieo},
        {jo, ro, ieo},
        {ji, ri, iei},
        {edec:getEdgeCount()-1, ri, iei},
    }
    local hmap, apin = av2hmap(edec, hpin)
    -- mesh
    local rdinfo = {id = edec:getID(), hmap = hmap, apin = apin}
    decal2mesh(rdinfo, 'exit') --, true)
    --- hmap for decal
    hmap, apin = hmap4decal(edec, rdinfo.av, rdinfo.avstep)
    rdinfo.id = decalUpdate(edec, hmap, apin)
end


local function junctionUp(icirc)
    local inedge = {}
    -- get branches
    for i,r in pairs(roads) do
        local inpath = U.index(r.path, icirc)
--                U.dump(inpath, '?? inpath:'..i)
--                U.dump(r.apin, '?? r:'..i)
        for _,j in pairs(inpath) do
            inedge[#inedge + 1] = {i, r.apin[r.path2dec[j]][1]}
        end
    end
    U.dump(inedge, '?? inedge')
    -- get star of vectors
    local adir = {}
    for _,s in pairs(inedge) do
        print('?? for_s:'..s[1]..':'..s[2])
        local decal = scenetree.findObjectById(roads[s[1]].id)
        for _,dir in pairs {1, -1} do
                    print('?? for_dir:'..dir..':'..s[2]..':'..tostring(decal:getEdgeCount())..':'..tostring(decal:getMiddleEdgePosition(s[2]))) --..tostring(roads[s[1]].decal:getMiddleEdgePosition(s[2] + dir))..':'..tostring(roads[s[1]].decal:getMiddleEdgePosition(s[2])))
            adir[#adir + 1] =
                decal:getMiddleEdgePosition(s[2] + dir) -
                decal:getMiddleEdgePosition(s[2])
        end
    end
            U.dump(adir, '?? adir:')
    local aang = U.fanOrder(adir)
            U.dump(aang, '?? aang:')
--    for o = 3,3 do
    for o = 1,#aang do
        local brout = inedge[round(aang[o][1]/2)] -- {iroad, edge}
        local dirout = 2*(aang[o][1] % 2 - 0.5) -- 1 - index increase from center, -1 - decrease
        local brin = inedge[round(aang[o % #aang + 1][1]/2)]
        local dirin = 2*(aang[o % #aang + 1][1] % 2 - 0.5)

        local ieo = edgeAtDist(brout[2], dex, dirout, dirout, roads[brout[1]])
        local pout, ppout = exitPos(roads[brout[1]], ieo, -dirout, dirout)
        local iei = edgeAtDist(brin[2], dex, dirin, dirin, roads[brin[1]])
        local pin, ppin = exitPos(roads[brin[1]], iei, dirin, dirin)
                local hp = 0
--                adbg[#adbg + 1] = {pout+vec3(0,0,hp)}
--                adbg[#adbg + 1] = {ppout+vec3(0,0,hp), ColorF(0,0,1,1)}
--                adbg[#adbg + 1] = {pin+vec3(0,0,hp)}
--                adbg[#adbg + 1] = {ppin+vec3(0,0,hp), ColorF(0,0,1,1)}
                print('?? ie:'..ieo..'>'..iei..':'..dirout..'>'..dirin..':'..(ppout-pout):length())
        --- 2D geometry
        local cpos = U.proj2D(anode[icirc].pos)
        local d = cpos:distanceToLine(pout, pin)
        local dang = aang[o % #aang + 1][2] - aang[o][2]
        if o == #aang then
            dang = aang[1][2] + 2*math.pi - aang[#aang][2]
        end
        local dpush = (dang - math.pi/2)/math.pi*2.5
                print('?? FOR_ANG:'..o..':'..dang..':'..dpush)
        if dpush < 0 then dpush = 0 end
        local pmid = cpos + d*(3/5 + dpush) * U.vnorm(pout - pin)
--        local pmid = cpos + d*(3/5 + (aang[o % #aang + 1][2] - aang[o][2] - math.pi/2)/math.pi*2) * U.vnorm(pout - pin)
--                adbg[#adbg + 1] = {pmid}
        ---- base decal
        local edec = decalUp({
            ppout, pout,
            pmid,
--            squeeze(pin, pout, 1/4),
            pin, ppin}, wExit)
        --- height geometry
        local jo = forOut(roads[brout[1]], ieo, edec) --, nil, true)
--                adbg[#adbg + 1] = {edec:getLeftEdgePosition(jo)+vec3(0,0,hp), ColorF(0,1,0,1)}
        local ji = forIn(roads[brin[1]], iei, edec)
--                print('?? BE:'..o..':'..brout[2]..'> jo='..jo..':'..brin[2]..'> ji='..ji)
--                adbg[#adbg + 1] = {edec:getLeftEdgePosition(ji)+vec3(0,0,hp), ColorF(0,1,0,1)}
        local ro, ri = roads[brout[1]], roads[brin[1]]
        local ne = edec:getEdgeCount()
        local hpin = {}
        if false then
            hpin = {
                {0, ro, ieo},
                {jo, ro, ieo},
                {ji, ri, iei},
                {ne-1, ri, iei}
            }
        end
        for i = 0,jo,1 do
            hpin[#hpin + 1] = {i, ro, ieo}
        end
        if jo%2 == 1 then
--            hpin[#hpin + 1] = {jo, ro, ieo}
        end
--        print('?? for_j:'..ji..':'..(ne - 1))
        for i = ji,ne-1,1 do
--            print('?? ffff:'..i)
            hpin[#hpin + 1] = {i, ri, iei}
        end
        if (ne-1)%2 == 1 then
--            hpin[#hpin + 1] = {ne-1, ri, iei}
        end

--                U.dump(hpin,'?? hpin:')
        local hmap, apin = av2hmap(edec, hpin)
                U.dump(hmap,'?? hmap_PRE:')
                U.dump(apin,'?? apin_PRE:')
        --- mesh
        local rdinfo = {id = edec:getID(), hmap = hmap, apin = apin,
            meshid = nil, fr = brout,  to = brin, av = {}, w = wExit
--            meshid = nil, fr = {exout.rd, exout.ci},  to = {exin.rd, exin.ci}, av = {}, w = wExit
        }
        decal2mesh(rdinfo, 'exit') --, true)
        --- hmap for decal
        hmap, apin = hmap4decal(edec, rdinfo.av, rdinfo.avstep)
        rdinfo.id = decalUpdate(edec, hmap, apin)

        exits[#exits + 1] = rdinfo

        if #aang == 4 and math.pi - dang > math.pi*6/15 then
            eLoop(cpos, brout, -dirout, brin, -dirin)
        end
--                break
    end
end


--========================================================================================

local stime

local function onUpdate()
--    pathsUp()


    if inanim and inanim.N and inanim.n < inanim.N+1 then
        local ctime = math.floor(socket.gettime()*1000)
        if not stime then
            stime = ctime
        end
        if ctime - stime > 300 then
            lo('?? ani_STEP:'..inanim.n..':'..ctime) --..tostring(math.floor(socket.gettime()*1000)))
    --        inanim = {ain = ani, cb = roadAni}
    --        lo()
            inanim.n = inanim.n + 1
            inanim.cb(inanim.ain, inanim.n)
            stime = ctime
        end
--                inanim.n = inanim.N
    end

    if im.IsMouseClicked(0) then
        local rayCast = cameraMouseRayCast(true)
        if not rayCast then return end

        local nm = rayCast.object.name
    --            print('?? MC:'..cameraMouseRayCast(true).object.name)
        if not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not im.IsAnyItemHovered() then
--            lo('?? network.MC:'..nm)
            if true and nm then
                local icirc
                for i = 1,#anode do
                    if anode[i].r and (anode[i].pos - rayCast.pos):length() < anode[i].r then
                        icirc = i
                        break
                    end
                end
                    lo('?? network.MC:'..nm..':'..tostring(icirc))
                if string.find(nm, 'road_') == 1 then
                    local rd = rayCast.object
                            print('?? if_dec:'..tostring(rd.decal))
                    if rd.decal ~= nil then
                        print('?? onClick.for_junc:'..rd.decal)
                        if icirc ~= nil then
                            print('?? region:'..icirc..':'..rd:getID())
                            junctionUp(icirc)
                        end
                    end
                elseif string.find(nm, 'exit_') == 1 then
                    local rd = rayCast.object
                    Net.exitBranch(rd)
                else
                    lo('?? N.to_SEED:'..#anode)
--                        print('?? for_scope:'..tostring(W.forScope())..':'..nm)
--                    return W.mdown(rayCast, {R=R, D=D}) or nm ~= 'theTerrain'
                end

            end
        end
    end

    if #anode then -- and #roads == 0 then
--        lo('?? for_c:'..#anode)
        -- circles
        for o,n in pairs(anode) do
            local clr = color(255,255,255,255)
            if #U.index(Net.sregion, o) > 0 then
                -- selected
                clr = color(255,255,0,255)
            end
            if n.r then
                Render.circle(n.pos, n.r, clr, 4, true)
                debugDrawer:drawText(n.pos, String(o), ColorF(0,0,1,1))
                -- debugDrawer:drawLine(v[1], v[2], ColorF(1,1,1,1), 4)
            end
        end
    end

--    debugDrawer:drawText(massCenter, 'MC', ColorF(1,0,0,1))

    if #apath > 0 then -- and #roads == 0 then
        -- paths
        for i,l in pairs(apath) do
            for j,s in pairs(l) do
                if j > 1 then
                    local c, w
                    if #U.index(apairskip, U.stamp({l[j-1], l[j]})) == 0 then
                        c = color(math.floor(255*j/#l),math.floor(255*(#l-j)/#l),0,255)
                        w = 2
                    else
                        c = color(0,255,255,255)
                        w = 1
                    end
                    if anode[l[j-1]] == nil or anode[l[j]] == nil then
                        print('!! NO_NODE:'..j..':'..l[j]..':'..l[j - 1]..'/'..#anode)
                        apath = {}
                    else
                        debugDrawer:drawLine(anode[l[j-1]].pos, anode[l[j]].pos, ColorF(j/#l,(#l-j)/#l,0,1), w)
--                        Render.path({ anode[l[j-1]].pos+vec3(0,0,20), anode[l[j]].pos+vec3(0,0,20) }, c, w)
                    end
                end
            end
        end
    end

end


Net.circleSeed = circleSeed
Net.pathsUp = pathsUp
Net.toDecals = toDecals

Net.onUpdate = onUpdate

return Net