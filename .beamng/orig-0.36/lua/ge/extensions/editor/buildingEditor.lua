--lo('== CE:')
--local meshEditor = dofile("/lua/ge/extensions/editor/meshEditor.lua")
--print('= buildingEditor')
local _MODE = 0

if _MODE == 1 then
--    local function
	local ffi = require "ffi"
end

local adbg = {}
local inedit = nil

require "socket"

local apack = {
	'/lua/ge/extensions/editor/gen/world',
	'/lua/ge/extensions/editor/gen/ui',
	'/lua/ge/extensions/editor/gen/decal',
	'/lua/ge/extensions/editor/gen/region',
}
local W, UI, D, R, UU

local function reload(mode, soft)
--		print('??^^^^^^^^^^^^^^^ reload:'..UU._MODE)
--	lo('>> ce.reload:'..#apack)
--            if true then return end
	if W ~= nil and not soft then
		-- remove world objects
		W.clear()

--        inedit = nil

--        lo('?? adesc:'..#W.adesc)
--            lo('?? for_m:'..tostring(id))
--        for id,_ in pairs(W.adesc) do
--            scenetree.findObjectById(id):delete()
--        end
	end

	unrequire(apack[1])
	W = rerequire(apack[1])
	unrequire(apack[2])
	UI = rerequire(apack[2])
	if UU._MODE == 'conf' then
		W.reload('conf')
		editor.showWindow('LAT')
		UU._MODE = 'conf'
		UU.out._MODE = 'conf'
		UI.inject({U = UU, W = W})
		UI.hint(editor.editModes.cityEditMode)
		W.up(D, true, true, 'conf')

		return
	end
--	if not mode then end
--	if not mode and UU._MODE ~= 'conf' then end
	unrequire(apack[1])
	W = rerequire(apack[1])
	unrequire(apack[2])
	UI = rerequire(apack[2])
--	unrequire(apack[3])
--	D = rerequire(apack[3])
--	unrequire(apack[4])
--	R = rerequire(apack[4])

	if not soft then
--			print('?? for_up')
		W.up(D)
	end

	editor.showWindow('BAT')
	if UU._PRD == 0 and ({conf=0,ter=0})[UU._MODE] then
		editor.showWindow('LAT')
	end
end

unrequire(apack[1])
W = rerequire(apack[1])
unrequire(apack[2])
UI = rerequire(apack[2])
unrequire(apack[3])
D = rerequire(apack[3]) --require('/lua/ge/extensions/editor/gen/decal')
unrequire(apack[4])
R = rerequire(apack[4])

UU = require('/lua/ge/extensions/editor/gen/utils')

local lo = UU.lo

--reload()
-----------------------------------------------------------------------
-- UTIL
-----------------------------------------------------------------------
--lo('??**** UU:'..UU.t)
local U = {}

--local function lo(ms)
--  	if U._PRD ~= 0 then return end
--	print(ms)
--end

U.index = function(list, p)
local apos = {}
	for i = 1,#list do
		if list[i] == p then
		apos[#apos+1] = i
		end
	end
	return apos
end

U.toggle = function(list, val)
	if val == nil then return end
	local ind = U.index(list, val)
	if #ind > 0 then
		table.remove(list, ind[1])
	else
		list[#list + 1] = val
	end
end

U.stamp = function(list)
	table.sort(list)
	if #list == 0 then
		return ''
	end
	local s = tostring(list[1])
	for o = 2,#list do
		s = s..'_'..list[o]
	end
	return s
end


U.split = function(s, d)
	local t = {}
	for str in string.gmatch(s, "([^"..d.."]+)") do
		t[#t + 1] = tonumber(str)
	end
	return t
end


U.hightOnCurve = function(p, rdinfo, start, dbg) --, av, step, dbg)
	local decal = scenetree.findObjectById(rdinfo.id)
	local av, step = rdinfo.av, rdinfo.avstep
--U.hightOnCurve = function(p, decal, start, av, step, dbg)
--U.hightOnCurve = function(p, aheight, decal, start)
--  lo('>> hightOnCurve:'..tostring(p)..':'..#aheight..':'..decal:getEdgeCount()..':'..start)
			if dbg == true then
				adbg[#adbg+1] = {p+vec3(0,0,0), ColorF(1,0,1,1), 0.2}
				adbg[#adbg+1] = {decal:getMiddleEdgePosition(start)+vec3(0,0,0), ColorF(1,1,0,1), 0.5}
			end
	local w = 4
	local mi, imi = 1/0
	local ifr = math.max(1, start-25)
	for i = ifr,start+25 do --*dir,dir do
				if dbg then lo('?? hightOnCurve_i:'..i..'/'..start) end
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
			lo('?? hc:'..i..'/'..start..':'..decal:getEdgeCount()..':'..step..' pi:'..pi..' pip:'..pip)
		end
		if dbg and pi * pip < 0 then
			lo('??___<:'..i..'/'..start..':'..tostring(p)..':'..tostring(ni)..':'..tostring(nip)..':'..p:distanceToLineSegment(ni, nip))
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
	--      lo('?? hoc:'..i..':'..pi..':'..pip..' ni:'..tostring(ni)..' nip:'..tostring(nip))
			pi = math.abs(pi) --/(p - ni):length()/(nip - ni):length()
			pip = math.abs(pip) --/(p - nip):length()/(nip - ni):length()

			local h = av[i*step + 1].z * (1 - pi/(pi + pip)) +
				av[(i + 1)*step + 1].z * pi/(pi + pip)
--            lo('<< hightOnCurve:'..i..'/'..start..' p:'..tostring(p))
	--        local h = aheight[i]*(1 - pi/(pi + pip)) + aheight[i+1]*pi/(pi + pip)
	--      lo('<< hightOnCurve:'..i..':'..(i + 1)..' h:'..aheight[i]..':'..aheight[i+1]..':'..pi..':'..pip..' h:'..h)
			return h
		end
--    lo('?? for_node:'..i..':'..tostring(cp)..' d:'..(p-cp):length())
	end
	if imi ~= nil then
		-- assign height of closest vertex
--		lo('?? hightOnCurve:'..imi..':'..#av)
		return av[imi*step + 1].z
	end
--	lo('!!!!!!!!!!!!!! hightOnCurve.NONE:')
--  lo('<< hightOnCurve:'..imi..'/'..decal:getEdgeCount()..':'..aheight[imi])
--  return aheight[imi]
end


U.spline1 = function(a, b, i, n)
	return (a*(n - i) + b*i)/n
end

U.spline2 = function(a, b, i, n)
	local c = 2*(b - a)/(n * n)
	if i < n/2 then
	  return a + c*i*i
	else
	  return b - c*(i - n)*(i - n)
	end
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


U.vang = function(u, v)
	return math.acos(u:dot(v)/u:length()/v:length())
end


U.proj2D = function(v)
	return vec3(v.x, v.y, 0) --{ x = v.x, y = v.y, z = 0 }
end


U.vnorm = function(v)
	return vec3(-v.y, v.x, 0):normalized()
end


U.vturn = function(v, ang)
	return vec3(math.cos(ang)*v.x - math.sin(ang)*v.y, math.sin(ang)*v.x + math.cos(ang)*v.y, v.z)
end


U.lineCross = function(a1, a2, b1, b2)
	local x = ((a1.x*a2.y - a1.y*a2.x)*(b1.x - b2.x) - (a1.x - a2.x)*(b1.x*b2.y - b1.y*b2.x))/
	  ((a1.x - a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x - b2.x))
	local y = ((a1.x*a2.y - a1.y*a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x*b2.y - b1.y*b2.x))/
	  ((a1.x - a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x - b2.x))
	return x, y
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
--    lo('<< segmentCross:'..x..':'..y)
	return x, y
end


U.dump = function(t, msg, lvl)
  if true then return end
--    lo('?? dump:'..tostring(lvl)..':'..type(t))
	if type(t) == 'table' then
		local s = '{ '
		for o,e in pairs(t) do
--            lo(type(o)..':'..type(e)..':'..o..':'..tostring(e))
			local lt = U.dump(e, nil, 1)
--            lo('?? dret:'..lt)
			s = s..o..' = '..lt..', '
		end
		if msg ~= nil then lo(msg) end
		if lvl == nil then lo(s ..' }') end
		return s..' }'
	else
--        lo('?? dump_nn:'..type(t)..':'..tostring(t))
		return tostring(t)
	end
	return ''
end
-----------------------------------------------------------------------
-- RENDER
-----------------------------------------------------------------------
local Render = {}

--local ffifound, ffi = pcall(require, 'ffi')

local buf = ffi.new("float[?]", math.max(6*100))

local function toLineBuf(u, v, pos)
	buf[pos + 0] = u.x
	buf[pos + 1] = u.y
	buf[pos + 2] = u.z
	buf[pos + 3] = v.x
	buf[pos + 4] = v.y
	buf[pos + 5] = v.z
end

local pathUp = function(buf, nStep, c, w)
	if w == nil then
		w = 2
	end
	if c == nil then
		c = color(0, 0, 0, 255)
	end
	ffi.C.BNG_DBG_DRAW_LineInstance_MinArgBatch(buf, nStep, w, c)
end

local once = false

Render.path = function(apos, c, w)
	for i = 1,#apos - 1 do
		toLineBuf(apos[i], apos[i + 1], (i-1)*6)
	end
	pathUp(buf, #apos - 1, c, w)
end


local circleGrid = 16

Render.circle = function(center, r, c)
	local v = vec3(center.x+r,center.y,0)

	for i = 1, circleGrid do
		--TODO: use heightmap
		local l = 2*r*math.sin(math.pi/circleGrid)
		local arg = math.pi/2 + (i - 1 + 1/2)*2*math.pi/circleGrid
		local dv = l*vec3(
			math.cos(arg),
			math.sin(arg),
			0)
		toLineBuf(v, v + dv, (i-1)*6)
--        lo('?? for_i:'..tostring(i)..tostring(center)..'::'..tostring(r)..' dv='..tostring(dv)..' v='..tostring(v)..tostring(v+dv))
		v = v + dv
	end
	pathUp(buf, circleGrid, c, 1)
end


-----------------------------------------------------------------------
-- MESH
-----------------------------------------------------------------------
local Mesh = {}

Mesh.rect = function(u, v, av, af)
--    lo('?? rect:'..tostring(u)..':'..tostring(v))
	av[#av + 1] = vec3(0, 0, 0)
	av[#av + 1] = av[#av] + u
	av[#av + 1] = av[#av] + v
	av[#av + 1] = av[#av] - u

	af[#af + 1] = {v = 0, n = 0, u = 0}
	af[#af + 1] = {v = 3, n = 0, u = 1}
	af[#af + 1] = {v = 1, n = 0, u = 3}

	af[#af + 1] = {v = 1, n = 0, u = 3}
	af[#af + 1] = {v = 3, n = 0, u = 1}
	af[#af + 1] = {v = 2, n = 0, u = 2}

	if true then
		af[#af + 1] = {v = 0, n = 0, u = 0}
		af[#af + 1] = {v = 1, n = 0, u = 1}
		af[#af + 1] = {v = 3, n = 0, u = 3}

		af[#af + 1] = {v = 3, n = 0, u = 3}
		af[#af + 1] = {v = 1, n = 0, u = 1}
		af[#af + 1] = {v = 2, n = 0, u = 2}
	end

	return av, af
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

Mesh.up = function(constructor)
	local av, auv, an, af = {}, {}, {}, {}
	an[#an+1] = {x = 0, y = 0, z = 1}

	auv[#auv + 1] = {u = 0, v = 1}
	auv[#auv + 1] = {u = 0, v = 0}
	auv[#auv + 1] = {u = 1, v = 0}
	auv[#auv + 1] = {u = 1, v = 1}

	local rdm = createObject("ProceduralMesh")
	rdm:setPosition(vec3(0, 0, 0))
	rdm.isMesh = true
	rdm.canSave = false
--	rdm:registerObjectWithIdSuffix(nm)
--[[
	rdm:registerObject(nm)
	local id = rdm:getID()
	rdm:unregisterObject()
	rdm:registerObject(nm..'_'..id)
]]
	scenetree.MissionGroup:add(rdm.obj)
	rdm:createMesh({{{
		verts = av,
		uvs = auv,
		normals = an,
		faces = af,
		material = "WarningMaterial",
	}}})
	return
end

-----------------------------------------------------------------------
-- NETWORK
-----------------------------------------------------------------------
local Net = {}


local circleMax = 40
--local astate = {}

local massCenter = vec3(0, 0, 0)
local anode, apath, apairskip = {}, {}, {}
local jointpath = {}
--local apair = {}
local edges = {astamp = {}, infaces = {}}
Net.anode = anode
--local aestamp = {}
--local infaces = {}
-- step weight params
local wa, wb = 0.5, 0.5

local roads = {} -- path, astamp, levels
local exits = {} -- exits road info

local adecalID = {}
local ameshID = {}

-- road geometry
local wHeighway = 4
local wExit = 2
local hLevel = 3    -- space between levels
local marginSide, marginEnd = 0.1, 0.2
local mat = 'dirt_road_tread_ruts'
local dex = 30 -- distance from center to exit strt
local dexpre = 10 -- distance of pre exit decal point to exit
local dexpree = 5 -- distance of pre exit on exit

local function forStar(sc, path)
	local ause = U.index(path, sc)
--        lo('?? for use: sc:'..sc..' sp:'..sp..' used:'..#ause)
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


Net.pathsUp = function()
--	lo('>> pathsUp:'..#anode)
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
--                    lo('?? path_start:'..path[1])
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
--                    lo('?? path_start2:'..#ap)
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
		--                    lo('??++++ HAS_CROSS:'..cp..':'..p.ind)
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
--            lo('?? CROSS:'..tostring(x)..':'..tostring(y))
--			lo('?? CROSS:'..cp..':'..imi..' X '..crossed..':'..tostring(xc)..':'..tostring(yc))
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
--					lo('?? NO_NODES:'..n)
					break
				elseif #path == 1 and #afree == 1 then
					-- TODO:
--					lo('!! SINGLE_FREE:')
					break
				end
				path = { afree[1] }
						U.dump(afree, '?? afree:')
						U.dump(apath[#apath], '?? path:'..path[1])
--                path = { afree[math.random(#afree)] }
				cp = path[#path]
--				lo('?? HAS_CROSS:'..n..'/'..(#anode*2)..':'..tostring(cp)..'/'..#afree)
--                        break
			else
				-- check for shortcut
				local d = (anode[cp].pos - anode[imi].pos):length()
				for j = 1,#anode do
					if j ~= cp and
					(anode[cp].pos - anode[j].pos):length() < d and
					U.vang(anode[cp].pos - anode[j].pos, anode[cp].pos - anode[imi].pos) < 0.2 then
--						lo('??+++++++ FOR_SC:'..cp..':'..imi..'>'..j)
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
--			lo('?? NO_IMI:')
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
--            lo('?? FOR_CROSS:'..x..':'..y)
--    apath[#apath + 1] = path
	U.dump(path, '<< pathsUp:'..#jointpath..'/'..#anode..':'..#apath)
end


local function linkOrder(s, star, p)
--    lo('>> linkOrder')
	local aang, angp = {}, 0
	for _,i in pairs(star) do
		local v = anode[i].pos - anode[s].pos
		aang[#aang + 1] = {i, math.atan2(v.y, v.x) % (2*math.pi)}
		if p ~= nil and i == p then
			angp = aang[#aang][2]
		end
--        lo('?? for_ang:'..tostring(v:normalized())..':'..aang[#aang][2])
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

local astamp = {}

local function stem(dec)
--	lo('>> stem:'..dec[1]..':'..dec[2])
	local path = jointpath --apath[ord]
	local isloop = false
	local n = 0
	local sc, sp = dec[2], dec[1]
	astamp[#astamp + 1] = U.stamp({sp, sc})
	while not isloop and n < 130 do
		n = n + 1
--        lo('?? for_sc:'..sc..':'..n)
--        for i = 2,#path do
--            local sc, sp = path[i], path[i - 1]
		-- get the star
		local star = forStar(sc, path)
--[[
		local ause = U.index(path, sc)
--        lo('?? for use: sc:'..sc..' sp:'..sp..' used:'..#ause)
		local star, anear = {}, {}
		for _,iuse in pairs(ause) do
			anear[#anear + 1] = iuse - 1
			anear[#anear + 1] = iuse + 1
		end
		for _,j in pairs(anear) do
			if j > 0 and j <= #path then
				if #U.index(star, path[j]) == 0 and path[j] ~= 0 then
					star[#star + 1] = path[j]
				end
			end
		end
]]
--        lo('?? for_star:'..sc..':'..#star)
--        U.dump(star)
		-- get co-direction
		if true then
--            if #star > 4 then
			--- ordered angles
--                    lo('?? pre_order:'..sc.."<"..sp..':'..#star)
			local aang = linkOrder(sc, star, sp)
			--- pick opposite branch
--                U.dump(aang, '?? aang:')
			local imi
			if #aang % 2 == 0 then
				imi = #aang/2 + 1
			else
				local a1, a2 = aang[(#aang-1)/2 + 1], aang[(#aang-1)/2 + 2]
						if a2 == nil then
--							lo('!! ERR_a2:'..#aang..':'..sc.."<"..sp..':'..aang[1][1]..' star:'..#star)
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
--[[
				if imi > 2 then
					-- !!
					if false then
--                    if math.abs(aang[imi - 1][2] - aang[imi][2]) < 0.2 then
						if (anode[aang[imi - 1][1] ].pos - anode[sc].pos):length()
						< (anode[aang[imi][1] ].pos - anode[sc].pos):length() then
							-- make shortcut
							lo('?? go_for:'..sc..'>'..aang[imi - 1][1]..':'..aang[imi][1])
							dec[#dec + 1] = aang[imi - 1][1]
							astamp[#astamp + 1] = U.stamp({sc, aang[imi - 1][1]})
							dec[#dec + 1] = aang[imi][1]
							astamp[#astamp + 1] = U.stamp({sc, aang[imi][1]})
							sp = aang[imi - 1][1]
							sc = dec[#dec]
							imi = nil
--                                lo('??************** CHECK_SHC:'..sc..'>'..aang[imi][1]..':'..aang[imi-1][1])
							-- imi = nil
						end
					end
				end
]]
			end
			if imi ~= nil then
--                lo('?? co_DIR:'..n..':'..sp..'>'..sc..'>'..aang[imi][1])
				local stamp = U.stamp({ sc, aang[imi][1] })
				if #U.index(astamp, stamp) > 0 then
--					lo('?? LOOPED:'..n..':'..sc..'>'..aang[imi][1])
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
	U.dump(astamp, '<< stem:')
	return dec
end


Net.decalUp = function(apos, w, m)
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


Net.path2decal = function(rd, w)
--	lo('>> path2decal:'..#rd.path)
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
--        lo('?? for_node:'..i..':'..path[i]..'<'..tostring(path[i - 1]))
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
--            lo('??==== obt1: >'..path[i]..':'..tostring(nb)..'>'..tostring(nc)..'>'..tostring(ne))
			tounfold = true
		end
		nb, nc, ne = isObtuse(path, path[i])
		if nc == path[i] and (nb == path[i - 1] or ne == path[i - 1]) then
--            lo('??==== obt2: >'..path[i]..':'..tostring(nb)..'>'..tostring(nc)..'>'..tostring(ne))
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

	lo('<< path2decal:'..n)
	return road
end

--{je, ri, exin.ei},
local function av2hmap(dec, list, dbg)
--            lo('>> av2hmap:'..tostring(dbg))
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
--                lo('?? av2hmap:'..ind..':'..tostring(dec:getMiddleEdgePosition(ind))..':'..ei)
			local h = U.hightOnCurve(dec:getMiddleEdgePosition(ind), rdinfo, ei, dbg)
--            local h = U.hightOnCurve(
--                U.proj2D(dec:getMiddleEdgePosition(ind)),
--                branch, ei, rdinfo.av, rdinfo.avstep) --, true)
			if h ~= nil then
				hmap[ind] = h
				apin[#apin + 1] = {ind}
			else
				lo('!! av2hmap.NO_H:'..ind..':'..tostring(dbg))
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
--            lo('<< posAtDistance:'..ifrom..':'..i..':'..(s - d)..':'..(v:length()))
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
					lo('?? forOut:'..i..':'..d..':'..tostring(dp))
				end
--            adbg[#adbg + 1] = {droot:getMiddleEdgePosition(ei + 1), ColorF(1,1,0,1)}
--            adbg[#adbg + 1] = {droot:getMiddleEdgePosition(ei - 1), ColorF(1,1,0,1)}
--            lo('?? forOut:'..i..'/'..ne..':'..d)
		if dp ~= nil and d > dp and d > (rdinfo.w + wExit)/2 + 2*marginSide then
			return i
		end
		dp = d
	end
end

local function forIn(rdinfo, ei, dec)
	return forOut(rdinfo, ei, dec, -1)
end

local function squeeze(a, b, s)
	local pmid = (b - a):normalized()
	pmid = (a + b)/2 + s*(b - a):length()*vec3(pmid.y, -pmid.x, 0)

	return pmid
end

-- looped exit
local function eLoop(cpos, brout, dirout, brin, dirin)
	lo('>> eLoop:'..dirout..':'..dirin)

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
	local edec = Net.decalUp(an, wExit)

	-- geometry
	local jo = forOut(roads[brout[1]], ieo, edec) --, nil, true)
		adbg[#adbg + 1] = {edec:getLeftEdgePosition(jo)+vec3(0,0,hp), ColorF(0,1,0,1)}
	local ji = forIn(roads[brin[1]], iei, edec)
		adbg[#adbg + 1] = {edec:getLeftEdgePosition(ji)+vec3(0,0,hp), ColorF(0,1,0,1)}
		lo('?? eLoop.BE:'..brout[2]..'>'..jo..':'..brin[2]..':'..ji)
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
	Net.decal2mesh(rdinfo, 'exit') --, true)
	--- hmap for decal
	hmap, apin = Net.hmap4decal(edec, rdinfo.av, rdinfo.avstep)
	rdinfo.id = Net.decalUpdate(edec, hmap, apin)
end


Net.junctionUp = function(icirc)
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
		lo('?? for_s:'..s[1]..':'..s[2])
		local decal = scenetree.findObjectById(roads[s[1]].id)
		for _,dir in pairs {1, -1} do
					lo('?? for_dir:'..dir..':'..s[2]..':'..tostring(decal:getEdgeCount())..':'..tostring(decal:getMiddleEdgePosition(s[2]))) --..tostring(roads[s[1]].decal:getMiddleEdgePosition(s[2] + dir))..':'..tostring(roads[s[1]].decal:getMiddleEdgePosition(s[2])))
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
				lo('?? ie:'..ieo..'>'..iei..':'..dirout..'>'..dirin..':'..(ppout-pout):length())
		--- 2D geometry
		local cpos = U.proj2D(anode[icirc].pos)
		local d = cpos:distanceToLine(pout, pin)
		local dang = aang[o % #aang + 1][2] - aang[o][2]
		if o == #aang then
			dang = aang[1][2] + 2*math.pi - aang[#aang][2]
		end
		local dpush = (dang - math.pi/2)/math.pi*2.5
				lo('?? FOR_ANG:'..o..':'..dang..':'..dpush)
		if dpush < 0 then dpush = 0 end
		local pmid = cpos + d*(3/5 + dpush) * U.vnorm(pout - pin)
--        local pmid = cpos + d*(3/5 + (aang[o % #aang + 1][2] - aang[o][2] - math.pi/2)/math.pi*2) * U.vnorm(pout - pin)
--                adbg[#adbg + 1] = {pmid}
		---- base decal
		local edec = Net.decalUp({
			ppout, pout,
			pmid,
--            squeeze(pin, pout, 1/4),
			pin, ppin}, wExit)
		--- height geometry
		local jo = forOut(roads[brout[1]], ieo, edec) --, nil, true)
				adbg[#adbg + 1] = {edec:getLeftEdgePosition(jo)+vec3(0,0,hp), ColorF(0,1,0,1)}
		local ji = forIn(roads[brin[1]], iei, edec)
				lo('?? BE:'..o..':'..brout[2]..'> jo='..jo..':'..brin[2]..'> ji='..ji)
				adbg[#adbg + 1] = {edec:getLeftEdgePosition(ji)+vec3(0,0,hp), ColorF(0,1,0,1)}
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
--        lo('?? for_j:'..ji..':'..(ne - 1))
		for i = ji,ne-1,1 do
--            lo('?? ffff:'..i)
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
		Net.decal2mesh(rdinfo, 'exit') --, true)
		--- hmap for decal
		hmap, apin = Net.hmap4decal(edec, rdinfo.av, rdinfo.avstep)
		rdinfo.id = Net.decalUpdate(edec, hmap, apin)

		exits[#exits + 1] = rdinfo

		if #aang == 4 and math.pi - dang > math.pi*6/15 then
			eLoop(cpos, brout, -dirout, brin, -dirin)
		end
--                break
	end
end


Net.junctionUp_ = function(icirc)
			Net.jUp(icirc)
			if true then return end
	lo('>>------------- junctionUp:')
--    local road
	local inedge = {}
	local aexit = {} -- {rind =, eind =}
	local cpos = U.proj2D(anode[icirc].pos)
	local adir = {}
	local dir2road = {}
	--------------------------
	-- CHOOSE BRANCHES PAIRS
	--------------------------
	for i,r in pairs(roads) do
		inedge[i] = {}
		aexit[i] = {}
		local inpath = U.index(r.path, icirc)
				U.dump(inpath,'?? inpath:'..icirc..':'..tostring(cpos)..' rid:'..tostring(r.id))
		local indec = {}
		for k,v in pairs(r.dec2path) do
--            lo('?? d2p:'..k..':'..i)
			if #U.index(inpath, v) > 0 then
				indec[#indec + 1] = k
				inedge[i][#(inedge[i])+1] = r.apin[k][1]
			end
		end
		local decal = scenetree.findObjectById(r.id)
				U.dump(inedge[i], '?? junctionUp_inedge:'..i)
				lo('?? edge_0:'..tostring(r.av[1])..':'..tostring(r.av[r.avstep])..':'..tostring(decal:getMiddleEdgePosition(0)))
		--- exits positions
		local aee = {}
		for o,e in pairs(inedge[i]) do
			local av, step = r.av, r.avstep
--            local lv, rv = e*r.avstep, (e + 1)*r.avstep
			lo('?? for_edge:'..e..':'
			..tostring(r.av[e*step+1])..':'..tostring(r.av[e*step + r.avstep])..':'
			..tostring(decal:getMiddleEdgePosition(e)))
--            lo('?? for_edge:'..tostring(r.av[0])..':'..tostring(r.av[r.avstep-1])..':'..tostring(r.av[lv]))
			for j = e + 1,#av/step do
				if (decal:getMiddleEdgePosition(j) - cpos):length() > dex then
					local chir = 1
					if j > e then chir = -1 end
					aee[#aee + 1] = {
						rd = i,
						ci = e, -- central edge
						ei = j,
						chir = chir,
						dir = U.proj2D(decal:getMiddleEdgePosition(j) - cpos):normalized(),
						tangent = U.proj2D(decal:getMiddleEdgePosition(j + 1) - decal:getMiddleEdgePosition(j)):normalized(),
						tanto = U.proj2D(decal:getMiddleEdgePosition(j - 1) - decal:getMiddleEdgePosition(j)):normalized(),
					}
					adir[#adir + 1] = aee[#aee].dir
					dir2road[#adir] = {i, #aee}
					break
				end
			end
			for j = e - 1,1,-1 do
				if (decal:getMiddleEdgePosition(j) - cpos):length() > dex then
					local chir = 1
					if j > e then chir = -1 end
					aee[#aee + 1] = {
						rd = i,
						ci = e, -- center edge (identifies branch)
						ei = j, -- exit edge index (0-based)
						chir = chir,
						dir = U.proj2D(decal:getMiddleEdgePosition(j) - cpos):normalized(),
						tangent = U.proj2D(decal:getMiddleEdgePosition(j - 1) - decal:getMiddleEdgePosition(j)):normalized(),
						tanto = U.proj2D(decal:getMiddleEdgePosition(j + 1) - decal:getMiddleEdgePosition(j)):normalized(),
					}
					adir[#adir + 1] = aee[#aee].dir
					dir2road[#adir] = {i, #aee}
					break
				end
			end
		end
--        U.dump(aee, '?? exs:')
		aexit[i] = aee
	end
			U.dump(aexit, '?? exs:')
	-- get branches
	local aang = U.fanOrder(adir)
			U.dump(aang, '??_______________ aang:')

	-------------------
	-- EXITS for PAIRS
	-------------------
--    for o,a in pairs(aang) do
--[[
	local function fedge(rd, dir)
		if dir == -1 then
			return function(e) return rd.av[e*rd.avstep + 1] end
--          return function(i) return rd:getLeftEdgePosition(i) end
		else
			return function(e) return rd.av[(e + 1)*rd.avstep] end
--            return function(i) return rd:getRightEdgePosition(i) end
		end
	end
]]
	local function fedge(rd, dir)
		if dir == -1 then
			return function(e) return rd.av[e*rd.avstep + 1] end
	--          return function(i) return rd:getLeftEdgePosition(i) end
		else
			return function(e) return rd.av[(e + 1)*rd.avstep] end
	--            return function(i) return rd:getRightEdgePosition(i) end
		end
	end

--    for o = 3,3 do
	for o = 1,#aang do
		local a, b = aang[o], aang[o % #aang + 1]
		lo('?? for_branch:'..o..':'..a[1]..':'..dir2road[a[1]][1]..':'..dir2road[a[1]][2])
--        local iroad = dir2road[a[1]][1]
		local d2ro = dir2road[a[1]]
		local d2ri = dir2road[b[1]]
		local exout = aexit[d2ro[1]][d2ro[2]]
		local exin = aexit[d2ri[1]][d2ri[2]]
--        U.dump(exout, '?? for_exo:'..o)
--        U.dump(exin, '?? for_exi:'..o)
		--- exit position
		---- in/out roads
		local ro, ri = roads[d2ro[1]], roads[d2ri[1]]
		---- in/out decals
		local deco, deci = scenetree.findObjectById(ro.id), scenetree.findObjectById(ri.id)
				lo('?? d_io:'..tostring(deco)..':'..tostring(deci))
		--- build exit decal
		local pout = fedge(ro, exout.chir)(exout.ei)
		local vno = U.proj2D(pout - deco:getMiddleEdgePosition(exout.ei)):normalized()
		pout = pout - vno * (0*wExit/2 + marginSide)
		local ppout = pout - vno*wExit/2 + exout.tangent * dexpre
				adbg[#adbg + 1] = {pout}
				adbg[#adbg + 1] = {ppout, ColorF(0, 0, 1, 1)}
		local pin = fedge(ri, -exin.chir)(exin.ei)
		local vni = U.proj2D(pin - deco:getMiddleEdgePosition(exin.ei)):normalized()
		pin = pin - vni * (0*wExit/2 + marginSide)
		local ppin = pin - vni*wExit/2 + exin.tangent * dexpre
				adbg[#adbg + 1] = {pin}
				adbg[#adbg + 1] = {ppin, ColorF(0, 0, 1, 1)}
		---- middle exit node
		local d = cpos:distanceToLine(pout, pin)
		local pmid = cpos + d*4/5 * (exout.dir + exin.dir):normalized()
--        local pmid = cpos + dex*9/9 * (exout.dir + exin.dir):normalized()
				adbg[#adbg + 1] = {pmid}
		---- base decal
		local edec = Net.decalUp({U.proj2D(ppout), U.proj2D(pout), U.proj2D(pmid), U.proj2D(pin), U.proj2D(ppin)}, wExit)
		--- heightmap for mesh
		---- ged edges intersection
		local ne = edec:getEdgeCount()
--                adbg[#adbg + 1] = {deco:getMiddleEdgePosition(exout.ei + exout.chir)}
		local jb, je, d
--        local hmap, apin = {}, {}
		for i = 1,ne do
			if jb == nil then
				d = edec:getLeftEdgePosition(i):distanceToLine(
					U.proj2D(deco:getMiddleEdgePosition(exout.ei)),
					U.proj2D(deco:getMiddleEdgePosition(exout.ei + exout.chir))
	--                U.proj2D(fedge(ro, exout.chir)(exout.ei)),
	--                U.proj2D(fedge(ro, exout.chir)(exout.ei + exout.chir))
				)
				if d > wHeighway/2 + 2*marginSide then
	--                lo('?? E_OUT:'..i)
					adbg[#adbg + 1] = {edec:getLeftEdgePosition(i), ColorF(0,1,0,1)}
					jb = i
				end
			else
				d = edec:getLeftEdgePosition(i):distanceToLine(
					U.proj2D(deci:getMiddleEdgePosition(exin.ei)),
					U.proj2D(deci:getMiddleEdgePosition(exin.ei + exin.chir))
				)
				if d < wHeighway/2 + 2*marginSide then
					adbg[#adbg + 1] = {edec:getLeftEdgePosition(i - 1), ColorF(0,1,0,1)}
					je = i - 1
					break
				end
			end
		end
		lo('??---------------- BE:'..tostring(jb)..':'..tostring(je))
		local hmap, apin = av2hmap(edec, {
			{0, ro, exout.ei},
			{round(jb/2), ro, exout.ei},
			{jb, ro, exout.ei},
			{je, ri, exin.ei},
			{round((ne - 1 + je)/2), ri, exin.ei},
--            {je + 4, ri, exin.ei},
--            {je + 8, ri, exin.ei},
			{ne - 1, ri, exin.ei},
		})
--        apin[3][2] = 1
--        apin[4][2] = 1
				U.dump(hmap,'?? E_hmap:')
				U.dump(apin,'?? E_apin:')
--        fromAV(edec, 0, ro, exout.ei)
--        fromAV(edec, jb, ro, exout.ei)
--        fromAV(edec, je, ri, exin.ei)
--        fromAV(edec, ne - 1, ri, exin.ei)

		local rdinfo = {id = edec:getID(), meshid = nil,
			hmap = hmap, apin = apin, fr = {exout.rd, exout.ci},  to = {exin.rd, exin.ci}, av = {}, w = wExit}
		Net.decal2mesh(rdinfo, 'exit') --, true)
--                lo('?? for_mesh:'..#rdinfo.av)
		--- hmap for decal
		hmap, apin = Net.hmap4decal(edec, rdinfo.av, rdinfo.avstep)
--                U.dump(hmap, '?? hmap_up:')
--                U.dump(apin, '?? apin_up:')
		--- update decal
--                U.dump(hmap, '?? E_hmap:')
--                U.dump(apin, '?? E_apin:')
		rdinfo.id = Net.decalUpdate(edec, hmap, apin, true)
		exits[#exits + 1] = rdinfo
--                break
	end
end


--[[
	local el, er = pos(ind, -dir), pos(ind, dir) --av[ind*step + 1], av[(ind + 1)*step]
	local e
	if dir == 1 then
		e = er
	else
		e = el
	end
]]


local function exitIO(eo, ei, cho, chi, inforo, infori)
	local pout, ppout, pin, ppin

	local function fedge(rd, dir)
		if dir == -1 then
			-- left edge
			return function(e) return rd.av[e*rd.avstep + 1] end
		else
			--right edge
			return function(e) return rd.av[(e + 1)*rd.avstep] end
		end
	end
	local deco, deci = scenetree.findObjectById(inforo.id), scenetree.findObjectById(infori.id)
	pout = fedge(inforo, -1)(eo)
--            lo('?? for_dec:'..tostring(inforo.id)..':'..tostring(deco))
	local vno = U.proj2D(pout - deco:getMiddleEdgePosition(eo)):normalized()
	pout = pout - vno * (0*wExit/2 + marginSide)
	local eop
	for i = eo,eo+cho*10,cho do
		if (pout - deco:getMiddleEdgePosition(i)):length() > dexpree then
			eop = i
			break
		end
	end
	ppout = fedge(inforo, -1)(eop) - vno*wExit/2
	pin = fedge(infori, -1)(ei)
	local vni = U.proj2D(pin - deci:getMiddleEdgePosition(ei)):normalized()
	pin = pin - vni * (0*wExit/2 + marginSide)
	local eip
	for i = ei,ei+chi*10,chi do
		if (pin - deci:getMiddleEdgePosition(i)):length() > dexpree then
			eip = i
			break
		end
	end
	ppin = fedge(infori, -1)(eip) - vni*wExit/2
--    lo('<< exitIO:'..tostring(pout)..':'..tostring(ppout)..':'..tostring(pin)..':'..tostring(ppin))

	return U.proj2D(pout), U.proj2D(ppout), U.proj2D(pin), U.proj2D(ppin)
end


local function forLimits(ro, ri, dec, eo, ei, cho, chi)
	local deco, deci = scenetree.findObjectById(ro.id), scenetree.findObjectById(ri.id)
	local jb, je
	local d, pd
	local ne = dec:getEdgeCount()
	for i = 1,ne do
		if jb == nil then
			d = U.proj2D(dec:getRightEdgePosition(i)):distanceToLine(
				U.proj2D(deco:getMiddleEdgePosition(eo)),
				U.proj2D(deco:getMiddleEdgePosition(eo + cho))
--                U.proj2D(fedge(ro, exout.chir)(exout.ei)),
--                U.proj2D(fedge(ro, exout.chir)(exout.ei + exout.chir))
			)
--                    lo('?? for_jb:'..i..':'..d..':'..tostring(pd))
			if pd ~= nil and d > pd and d > ro.w/2 + 2*marginSide then
--            if pd ~= nil and d > pd and d > ro.w/2 + 2*marginSide then
--                lo('?? E_OUT:'..i)
--                adbg[#adbg + 1] = {edec:getLeftEdgePosition(i), ColorF(0,1,0,1)}
				jb = i
				pd = nil
			end
		else
			d = U.proj2D(dec:getRightEdgePosition(i)):distanceToLine(
				U.proj2D(deci:getMiddleEdgePosition(ei)),
				U.proj2D(deci:getMiddleEdgePosition(ei + chi))
			)
			if d < ri.w/2 + 2*marginSide then
--            if pd ~= nil and d > pd and d < ri.w/2 + 2*marginSide then
				--                adbg[#adbg + 1] = {edec:getLeftEdgePosition(i - 1), ColorF(0,1,0,1)}
				je = i - 1
				break
			end
		end
		pd = d
	end
	return jb, je
end


local function height4edge(ei, hmap)
	local mi, hmi = 1/0, nil
	for i,h in pairs(hmap) do
--        lo('?? height4edge:'..i..':'..h)
		if math.abs(i - ei) < mi then
			mi = math.abs(i - ei)
			hmi = h
		end
	end
	return hmi
end


local function edge4pos(p, dec)
	local ne = dec:getEdgeCount()
--        lo('>> edge4pos:'..ne)
	local mi, imi = 1/0, nil
	for i = 1,ne do
		local d = U.proj2D(p - dec:getMiddleEdgePosition(i - 1)):length()
		if d < mi then
			mi = d
			imi = i
		end
	end
	return imi
end


local function edge4dist(d, dec)
	local ne = dec:getEdgeCount()
	local s, cs = 0, 0
	for i = 1,ne-1 do
		s = s + (dec:getMiddleEdgePosition(i) - dec:getMiddleEdgePosition(i - 1)):length()
	end
	for i = 1,ne-1 do
		cs = cs + (dec:getMiddleEdgePosition(i) - dec:getMiddleEdgePosition(i - 1)):length()
		if cs/s >= d then
			return i
		end
	end
end


Net.exitBranch = function(robj)
	local idobj = robj:getID()
	lo('>> exitBranch:'..idobj)
	-- get exits branches
	local ro, ri
	for _,e in pairs(exits) do
		if e.meshid == idobj then
--            U.dump(e, '?? for_out:')
			ro = e
			break
		end
	end
	for _,e in pairs(exits) do
		if e.to[1] == ro.to[1] and e.to[2] == ro.to[2] and e.meshid ~= idobj then
--            U.dump(e, '?? for_in:')
			ri = e
		end
	end
	local deco, deci = scenetree.findObjectById(ro.id), scenetree.findObjectById(ri.id)
	-- set edges indices
	local eio, eii = round(#ro.av/ro.avstep*1/2), round(#ri.av/ri.avstep*3/7)
	-- get decal nodes positions
--    local pout, ppout, pin, ppin = exitIO(eio, eii, -1, 1, ro, ri)
--            adbg[#adbg + 1] = {pout}
--            adbg[#adbg + 1] = {ppout, ColorF(0,0,1,1)}
--            adbg[#adbg + 1] = {pin}
--            adbg[#adbg + 1] = {ppin, ColorF(0,0,1,1)}

	local pout, ppout = exitPos(ro, eio, -1, -1)
	local pin, ppin = exitPos(ri, eii, -1, 1)
			lo('?? TEST_POS:'..tostring(pout)..':'..tostring(tp))
			adbg[#adbg + 1] = {pout, ColorF(1,1,0,1)}
			adbg[#adbg + 1] = {ppout, ColorF(0,1,1,1)}
			adbg[#adbg + 1] = {pin, ColorF(1,1,0,1)}
			adbg[#adbg + 1] = {ppin, ColorF(0,1,1,1)}

	--- configure 2D geometry (middle points)
	local pmid = squeeze(pout, pin, 1/3)
	local pm1 = squeeze(pout, pmid, 1/9)
	local pm2 = squeeze(pmid, pin, 1/9)
--    local pmid = (pin - pout):normalized()
--    pmid = (pin + pout)/2 + (pin - pout):length()/3*vec3(pmid.y, -pmid.x, 0)
			adbg[#adbg + 1] = {pmid, ColorF(1,0,1,1)}
	--- background decal
	local dec = Net.decalUp({ppout, pout, pm1, pmid, pm2, pin, ppin}, wExit)
	local ne = dec:getEdgeCount()
	--- configure z-geometry
	-- get overlap limits
	local jb, je = forLimits(ro, ri, dec, eio, eii, 1, -1)
--    local jb, je = forLimits(deco, deci, dec, eio, eii, 1, -1)
	lo('?? j_BE:'..jb..':'..je)
			adbg[#adbg + 1] = {dec:getRightEdgePosition(jb), ColorF(0,1,0,1)}
			adbg[#adbg + 1] = {dec:getRightEdgePosition(je), ColorF(0,1,0,1)}
--    lo('??***** e4p1:'..tostring(pm1)..':'..tostring(edge4pos(pm1, dec)))
--    lo('??***** e4p2:'..tostring(pm2)..':'..tostring(edge4pos(pm2, dec)))
	-- get branches heights

--    local hb = height4edge(ri.to[2], roads[ri.to[1]].hmap)
--            lo('?? hb:'..tosatring(hb))
	local ho = height4edge(ro.fr[2], roads[ro.fr[1]].hmap)
	local hi = height4edge(ro.to[2], roads[ro.to[1]].hmap)
			lo('?? for_bht:'..ro.to[2]..':'..ri.to[2]..':'..tostring(ho)..':'..tostring(hi))
	local hmap, apin = av2hmap(dec, {
		{0, ro, eio},
		{jb, ro, eio},
		{edge4pos(pm1, dec), hi - 2.5},
		{edge4pos(pm2, dec), ho + 3},
		{je, ri, eii},
		{ne - 1, ri, eii},
	})
			U.dump(hmap, '?? PREMESH_hm:')
			U.dump(apin, '?? PREMESH_apin:')
	local rdinfo = {id = dec:getID(), meshid = nil,
		hmap = hmap, apin = apin}
	Net.decal2mesh(rdinfo, 'exit')

	--- hmap for decal
	hmap, apin = Net.hmap4decal(dec, rdinfo.av, rdinfo.avstep)
	rdinfo.id = Net.decalUpdate(dec, hmap, apin)
end


Net.decal2mesh = function(road, nm, dbg)
	if nm == nil then
		nm = 'road'
	end
	local hmap = road.hmap
	local rd = scenetree.findObjectById(road.id)
	local adec = editor.getNodes(rd)
	local nsec = rd:getEdgeCount()
	lo('>> decal2mesh:'..nsec..':'..#road.apin)

	local av, auv, an, af = {}, {}, {}, {}
	local av2, auv2, an2, af2 = {}, {}, {}, {}
	an[#an+1] = {x = 0, y = 0, z = 1}
	an2[#an2+1] = {x = 0, y = 0, z = -1}

	auv[#auv + 1] = {u = 0, v = 1}
	auv[#auv + 1] = {u = 0, v = 0}
	auv[#auv + 1] = {u = 1, v = 0}
	auv[#auv + 1] = {u = 1, v = 1}


	if false then
		av[#av + 1] = vec3(2, -5, 1)
		av[#av + 1] = vec3(0, -5, 1)
		av, af = Mesh.strip(2, {vec3(2, -7, 1), vec3(0, -7, 1)}, av, af)

		local rdm = createObject("ProceduralMesh")
--        rdm:setPosition(vec3(0, 0, 0))
		rdm:setPosition(vec3(0, 0, 0))
		rdm.isMesh = true
		rdm.canSave = false
		rdm:registerObject(nm)
		local id = rdm:getID()
		rdm:unregisterObject()
		rdm:registerObject(nm..'_'..id)
		scenetree.MissionGroup:add(rdm.obj)
		rdm:createMesh({{{
			verts = av,
			uvs = auv,
			normals = an,
			faces = af,
			material = "WarningMaterial",
		}}})
		return
	end


--[[
	auv2[#auv2 + 1] = {u = 1, v = 0}
	auv2[#auv2 + 1] = {u = 0, v = 0}
	auv2[#auv2 + 1] = {u = 1, v = 1}
	auv2[#auv2 + 1] = {u = 0, v = 1}
]]
	auv2[#auv2 + 1] = {u = 0, v = 1}
	auv2[#auv2 + 1] = {u = 0, v = 0}
	auv2[#auv2 + 1] = {u = 1, v = 0}
	auv2[#auv2 + 1] = {u = 1, v = 1}


	if false then
		local i = 1

		local perp = rd:getRightEdgePosition(60) - rd:getLeftEdgePosition(60)
--        av[#av + 1] = rd:getLeftEdgePosition(60)
--        av[#av].z = 1
		av[#av + 1] = rd:getLeftEdgePosition(60)
--        av[#av + 1] = rd:getMiddleEdgePosition(60)
--        av[#av + 1] = rd:getRightEdgePosition(60)
		av[#av + 1] = rd:getLeftEdgePosition(60) + perp
		av[#av + 1] = rd:getLeftEdgePosition(60) + 2*perp
--        av[#av + 1] = rd:getRightEdgePosition(60)
--        av[#av].z = 1
		for i = 1,1 do
			perp = rd:getRightEdgePosition(60 + i) - rd:getLeftEdgePosition(60 + i)
			local apos = {
--                rd:getLeftEdgePosition(60 + i) + vec3(0, 0, 1),
				rd:getLeftEdgePosition(60 + i) + vec3(0, 0, 1),
				rd:getLeftEdgePosition(60 + i) + perp + vec3(0, 0, 1),
				rd:getLeftEdgePosition(60 + i) + 2*perp + vec3(0, 0, 1),
--                rd:getRightEdgePosition(60 + i) + vec3(0, 0, 1),
--                rd:getRightEdgePosition(60 + i) + vec3(0, 0, 1),
			}
			av, af = Mesh.strip(3, apos, av, af)
--            av, af = Mesh.strip(4, apos, av, af)
		end

		local rdm = createObject("ProceduralMesh")
		rdm:setPosition(vec3(0, 0, 0))
		rdm.isMesh = true
		rdm.canSave = false
		rdm:registerObject(nm)
		local id = rdm:getID()
		rdm:unregisterObject()
		rdm:registerObject(nm..'_'..id)
		scenetree.MissionGroup:add(rdm.obj)
		rdm:createMesh({{{
			verts = av,
			uvs = auv,
			normals = an,
			faces = af,
			material = "WarningMaterial",
		}}})
		return
	end


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
			if dbg == true then lo('?? mesh_cpin:'..i..':'..cpin) end
--            lo('??-------- next:'..i..':'..cpin..':'..road.apin[cpin - 1])
		end
--        lo('?? for_h: i='..i..' h1:'..h1..' h2:'..h2..' cpin:'..cpin..':'..tostring(road.apin[cpin - 1])..':'..tostring(road.apin[cpin]))
		if road.apin[cpin-1][2] == 1 and road.apin[cpin][2] == 1 then
			if dbg == true then lo('?? mesh_spline1:'..i..':'..(cpin-1)..'>'..cpin) end
			h = U.spline1(h1, h2, i - road.apin[cpin - 1][1], road.apin[cpin][1] - road.apin[cpin - 1][1])
		else
--            h = U.spline1(h1, h2, i - road.apin[cpin - 1][1], road.apin[cpin][1] - road.apin[cpin - 1][1])
			h = U.spline2(h1, h2, i - road.apin[cpin - 1][1], road.apin[cpin][1] - road.apin[cpin - 1][1])
			if dbg then
				lo('?? for_i:'..i..' cpin:'..cpin..' ap:'..road.apin[cpin - 1][1]..' h:'..h..'/'..h2)
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

--[[
]]
	local rdm = createObject("ProceduralMesh")
	rdm:setPosition(vec3(0, 0, 0))
	rdm.isMesh = true
	rdm.canSave = false
	rdm:registerObject(nm)
	local id = rdm:getID()
	rdm:unregisterObject()
	rdm:registerObject(nm..'_'..id)
	--    aidMesh[#aidMesh+1] = id
	scenetree.MissionGroup:add(rdm.obj)
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


	if false then
		local rdm2 = createObject("ProceduralMesh")
		rdm2:setPosition(vec3(0, 0, 0))
		rdm2.isMesh = true
		rdm2.canSave = false
		rdm2:registerObject(nm)
		id = rdm2:getID()
		rdm2:unregisterObject()
		rdm2:registerObject(nm..'_'..id)
		--    aidMesh[#aidMesh+1] = id
		scenetree.MissionGroup:add(rdm2.obj)
		rdm2:createMesh({{{
			verts = av2,
			uvs = auv2,
			normals = an2,
			faces = af2,
			material = "WarningMaterial",
		}}})
		ameshID[#ameshID + 1] = id
	end

 --[[
	rdm = createObject("ProceduralMesh")
	rdm:setPosition(vec3(0, 0, 0))
	rdm.isMesh = true
	rdm.canSave = false
	rdm:registerObject(nm)
	local id = rdm:getID()
	rdm:unregisterObject()
	rdm:registerObject(nm..'_'..id)
--    aidMesh[#aidMesh+1] = id
	scenetree.MissionGroup:add(rdm.obj)
	rdm:createMesh({{{
	  verts = av2,
	  uvs = auv2,
	  normals = an2,
	  faces = af2,
	  material = "WarningMaterial",
	}}})
 ]]
	return rdm
end

local function levelsUp(pth, iground)
	local alev, levels = {}, {}
	local function toLevels(pth, start, dir, clev)
		local clvl = 1
		if clev ~= nil then
			clvl = clev
		end
		local ib, ie = start, #pth
		if dir == -1 then
			ie = 1
		end
		for o = ib,ie,dir do
			local p = pth[o]
	--        for o,p in pairs(dec) do
			local set = 0
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
			lo('?? level_set:'..o..' node:'..pth[o]..' lvl:'..set)
			levels[o] = set
		end

		return levels
	end
	lo('?? GROUND:'..iground..'/'..#pth..':'..pth[iground])
	toLevels(pth, iground, 1)
	toLevels(pth, iground - 1, -1, levels[iground])

	return levels
end


Net.decalUpdate = function(dec, hmap, apin, dbg)
	be:reloadCollision()
--    local dec = scenetree.findObjectById(id)
	local adec = editor.getNodes(dec)
--        lo('?? to_del:'..r.id..':'..#adec..':'..tostring(r.meshid)..':'..#r.av..':'..nsec)

	-- reset heights
	for o,n in pairs(adec) do
		if dbg then
--            lo('?? for_UPD:'..o..'/'..#adec..':'..tostring(apin[o]))
		end
--                lo('?? np:'..o..':'..tostring(n.pos))
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


Net.hmap4decal = function(rd, av, step)
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


Net.toDecals = function()
		lo('>>-------------------------------- toDecals:'..#apath)
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
--            lo('?? lvl_start:'..dec[#list2 - 1]..':'..dec[#list2 - 2]..':'..dec[#list2 - 3])
	local levels = levelsUp(dec, #list2 - 1)
			U.dump(levels, '?? LEVELS:')
	roads[#roads + 1] = {
		id = nil, path = dec, levels = levels,
		dec2path = {}, meshid = nil,
		av = {}, avstep = nil,
		hmap = {}, apin = {},
		decal = nil, -- base decal reference
	}
	-- build decal
	for _,r in pairs(roads) do
		local rd = Net.path2decal(r, wHeighway)
--        r.decal = rd
		-- mesh
		--- get height map
		local adec = editor.getNodes(rd)
		local nsec = rd:getEdgeCount()
		local hmap, apin = {}, {}
				U.dump(levels, '?? LEV:')
				U.dump(r.dec2path, '?? dec2path:'..nsec)
		local cdec = 1
		local chight
		for i = 0,nsec - 1 do
			local pedge = rd:getMiddleEdgePosition(i)
			for j = cdec,#adec do
				local n = adec[j]
				if (pedge - n.pos):length() < 0.01 then
					if levels[r.dec2path[j]] == nil then
						hmap[i] = chight
					else
						hmap[i] = levels[r.dec2path[j]]
						chight = hmap[i]
					end
					hmap[i] = (hmap[i] - 1) * hLevel
					apin[#apin + 1] = {i}
--                    lo('?? e2n:'..i..':'..j..':'..tostring(levels[r.dec2path[j]])..':'..tostring(hLevel))
					cdec = j + 1
					break
				end
			end
		end
		--- last
		hmap[nsec - 1] = (levels[r.dec2path[#adec]] - 1) * hLevel
		apin[#apin + 1] = {nsec - 1}
--                lo('?? FOR_LAST:'..tostring(rd:getMiddleEdgePosition(nsec - 1))..':'..tostring(rd:getMiddleEdgePosition(nsec))..':'..tostring(adec[#adec].pos)..':'..#adec)
		r.hmap = hmap
		r.apin = apin
		--- mesh
		local rdmesh = Net.decal2mesh(r, 'road')
		if true then
		--- update decal
			lo('?? PRE_UP: nsec='..nsec..' adec='..#adec..':'..tostring(rd)..'::'..#hmap..'::'..tostring(#r.apin))
				U.dump(hmap, '?? hmap:')
				U.dump(r.apin, '?? apin:')
			local id = Net.decalUpdate(rd, hmap, r.apin) --, true)
			r.id = id
			rdmesh.decal = id
		end
	end
	U.dump(dec, '<< toDecals:'..#roads)
end


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
--    lo('>> circleFit:'..stamp..':'..r..' anode:'..#anode..' iter:'..nIter..' d:'..tostring(math.abs(d - (a + b)))..':'..tostring(massCenter))
	vn = vn:normalized()
	local ang = math.atan2(vn.y, vn.x)
	local dmiddle, flip = 0, 1
--    lo('?? circleFit_d:'..stamp..':'..tostring(d)..':'..tostring(vn))
	-- get center position
	local x, y
--    lo('?? circleFit.pre_int:'..#anode..':'..d..':'..tostring(c1.pos)..':'..tostring(c2.pos)..':'..(d -(a+b)))
	if math.abs(d - (a + b)) > 0.0001 then
		dmiddle = (d - (a+b))/2
		x = -(a+b-d+2*r)*(a-b)/(2*d)
		y = math.sqrt((d+b-a)*(d+a-b)*(a+b+d+2*r)*(a+b-d+2*r))/(2*d)
	--    lo('!! FOR_INTER:'..i..":"..j..":"..d..':'..x)
		-- CHECK IN-OUT
		local v = U.vturn({x=x, y=y}, ang)
		v = c1.pos + vn*(c1.r + dmiddle) + vec3(v.x, v.y, 0)
		local vflip = U.vturn({x=x, y=-y}, ang)
		vflip = c1.pos + vn*(c1.r + dmiddle) + vec3(vflip.x, vflip.y, 0)
		--TODO: massCenter role?
		if (vflip - massCenter/#anode):length() > (v - massCenter/#anode):length() then
--            lo('?? TO_flip:'..stamp..':'..tostring(massCenter)..':'..#anode..':'..dmiddle)
			y = -y
			flip = -1
		end
	else
		x = -r*(a - b)/(a + b)
		y = 2/(a + b) * math.sqrt(r*a*b*(r + a + b))
		if edges.infaces[stamp] == 1 then
--        if inCounter(i, j) == 1 then
--            lo('?? to_flip:'..i..':'..j)
			y = -y
			flip = -1
		end
	end

	local v = U.vturn({x = x, y = y}, ang)
--            lo('?? cF2:'..x..':'..y..'::'..tostring(v)..':'..ang)
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
--        lo('!! sign: x:'..x..' y:'..y..' vn:'..tostring(vn)..' ang:'..ang..':'..tostring(spos)..'>'..tostring(nd.pos)..':'..math.abs((spos - c1.pos):length()/(c1.r + nd.r)))
	end

	--check for intersections
	local mi, inear = 1/0, nil
	for k = 1,#anode do
		if k ~= i and k ~= j then
			local dist = (nd.pos - anode[k].pos):length()
			if dist - (nd.r + anode[k].r) < -0.01 then
	--          lo('!! INTER:'..i..":"..j..":"..k)
				if dist < mi then
					mi = dist
					inear = k
				end
			end
		end
	end
	if nIter >= #anode then
		lo('!! circleFit.NO_FIT:'..stamp..':'..nIter)
		nIter = 0
		return
	elseif inear ~= nil then
		local k = inear
--        lo('??******* int_resolve:'..i..':'..j..':'..inear..':'..tostring(nd.pos))
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
	massCenter = massCenter + nd.pos

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
--    lo('?? sf2:'..snew..':'..edges.infaces[snew])
	snew = U.stamp({j, #anode})
	if #U.index(edges.astamp, snew) == 0 then edges.astamp[#edges.astamp + 1] = snew end
	edges.infaces[snew] = flip
	nIter = 0
--    lo('?? sf3:'..snew..':'..edges.infaces[snew])
--            U.dump(edges.infaces, '?? Infaces:'..stamp)

--    lo('<<------ circleFit:'..stamp..':'..tostring(nd.pos)..':'..tostring(nd.d)) --..tostring(i)..':'..tostring(j)..':'..tostring(#anode)..':'..tostring(counter['1_2']))
	return nd
end


local _dbg = true
local fout
local function dump(node, yes)
--    lo('>> dump:'..tostring(yes)..':'..node.stamp)
	if (_dbg or state ~= nil) and yes == nil then
		return
	end
	local v, pair = node.pos, U.split(node.stamp, '_')
	fout:write(v.x..','..v.y..','..v.z..','..node.r..','..pair[1]..','..pair[2]..','..tostring(node.ang)..'\n')
end
Net.sregion = {} --selected regions inds

Net.circleSeed = function(nnode, mode)
	if U._PRD ~= 0 then return end
	lo('>>+++++++++++++++++++++++++ circleSeed:'..nnode..':'..tostring(mode)..':'..#anode..':'..tostring(_dbg))
	edges = {astamp = {}, infaces = {}}
	local ar = {}
	local resize
	local state

	if mode ~= nil and mode.resize ~= nil then
--        lo('?? circleSeed.resize:'..#anode)
		resize = mode.resize
		state = anode
	elseif _dbg then
		local fname = 'city.out'
		local lines = {}
		for line in io.lines(fname) do
		  lines[#lines + 1] = line
--          lo('?? line:'..line)
		end
		state = {}
		for i,l in pairs(lines) do
			local a = U.split(l,',')
			state[#state+1] = {
				pos = vec3(a[1],a[2],a[3]),
				r = a[4],
				stamp = U.stamp({a[5], a[6]}), ang = a[7] }
--            lo('?? snode:'..i..' r:'..snode[#snode][4])
		end
	end
	if state ~= nul then
		nnode = #state
	end

	if state == nil then
		fout = io.open('city.out', "w")
	end
--    lo('?? sC2:')

	for i = 1,nnode do
		if state == nil then
			ar[#ar + 1] = circleMax*(1/3 + math.random())
		else
--            lo('?? from_state:'..i..':'..tostring(state[i]))
			ar[#ar + 1] = state[i].r
			if resize ~= nil and i == Net.sregion[1] then
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
--            lo('?? FOR_2:'..tostring(ang)..':'..tostring(anode[#anode].pos)..':'..tostring(anode[#anode].ang))
--    U.dump(anode[#anode], '?? FOR_2:'..tostring(ang))
	edges.astamp[#edges.astamp + 1] = U.stamp({1, 2})
	massCenter = anode[1].pos + anode[2].pos

	dump(anode[1])
	dump(anode[2])

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
--            lo('?? res_pair:'..i..":"..tostring(pair))
		else
			if state ~= nil then
--                lo('?? RE_pair:'..i..':'..state[i].stamp..':'..tostring(edges.infaces[state[i].stamp]))
			end
			pair = apair[math.random(#apair)]
		end
		if pair == nil then
			lo('!! NO_PAIRS:'..i..'/'..nnode..':'..#anode)
			U.dump(edges.astamp)
			U.dump(edges.infaces)
			return
		end
--        lo('?? pre_fit:'..#anode..':'..tostring(anode[#anode].pos)..':'..anode[#anode].r)
		circleFit(pair, ar[i], d)

		dump(anode[#anode])
	end
	U.dump(edges.astamp)
	lo('<< circleSeed:'..#anode)
end

--*********************************************************************
local M = {}



--local anode = Net.anode

local camInitial = vec3(0, 0, 200)

local im = ui_imgui

local nNodes = 16
local nnodePtr = im.IntPtr(1)

-->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
local amesh = {}
local cmesh = nil

local function wallUp(pos, info, nm, obj)
	lo('>> wallUp:'..tostring(pos)..':'..tostring(info.u)..':'..tostring(info.v))
--    local u, v = u, vec3(0, 0, info.h)
	local av, auv, an, af = {}, {}, {}, {}

--    an[#an+1] = {x = 0, y = 0, z = 1}
	an[#an+1] = info.u:cross(info.v):normalized()

	auv[#auv + 1] = {u = 0, v = 1}
	auv[#auv + 1] = {u = 0, v = 0}
	auv[#auv + 1] = {u = 1, v = 0}
	auv[#auv + 1] = {u = 1, v = 1}

	av, af = Mesh.rect(info.u, info.v, av, af)

	for o,v in pairs(av) do
		av[o] = av[o] + pos
--        v = v + pos
	end
	local rdm = obj
	if rdm == nil then
		rdm = createObject("ProceduralMesh")
		rdm:setPosition(vec3(0,0,0))
	--    rdm:setPosition(pos)
		rdm.isMesh = true
		rdm.canSave = false
		rdm:registerObject(nm)
		local id = rdm:getID()
		rdm:unregisterObject()
		rdm:registerObject(nm..'_'..id)
		scenetree.MissionGroup:add(rdm.obj)
		info.pos = pos
		info.id = id
		amesh[#amesh + 1] = info
	end
	rdm:createMesh({{{
		verts = av,
		uvs = auv,
		normals = an,
		faces = af,
		material = "WarningMaterial",
	}}})
	-- windows

	be:reloadCollision()

	lo('<< wallUp:')
end


local ahouse = {}

local function houseUp(pos, prop)
	local house = {afloor = {}}
	local cheight = 0
	for i,f in pairs(prop.afloor) do
		local p = pos
		for j,w in pairs(f.awall) do
			wallUp(p + vec3(0, 0, cheight), w, 'o_wall_'..i..'_'..j)
--            wallUp(f.base[j] + vec3(0, 0, cheight), w, 'w'..i..j)
			p = p + (f.base[j % #f.base + 1] - f.base[j])
		end
		cheight = cheight + f.h
	end
	ahouse[#ahouse + 1] = house
end

local function regionUp_(p)
	-- build params
	local base = {
		vec3(0, 0, 0),
		vec3(-15, 0, 0),
		vec3(-15, -8, 0),
		vec3(0, -8, 0),
	}
	local afloor = {}
	--- ground floor
	afloor[#afloor + 1] = nil

	local nfloor = 2
--    for i = 1,1 do
	for i = 1,nfloor do
		local h = 3
		local awall = {}
		local mat = ''
--        for j = 1,1 do
		for j = 1,#base do
			local winspace, winbot = 2, 1
			awall[#awall + 1] = {
				u = base[j % #base + 1] - base[j], v = vec3(0, 0, h),
				mat = mat, win = '',
				winspace = winspace, winbot = winbot
			}
		end
		afloor[#afloor + 1] = {base = base, h = h, awall = awall}
	end
	-- render
	W.houseUp(p, {afloor = afloor})



	if false then
		local obj = createObject('TSStatic')
		obj:setField('shapeName', 0, '/assets/windows/s_R_HS_B_WN_03_0.6x1.2.dae')
	--    obj:setField('shapeName', 0, '/levels/smallgrid/art/shapes/misc/gm_cube_1m.dae')
	--    obj:setPosition(vec3(2, -2, 0))
	--    obj.scale = vec3(2, 2, 2)
		obj:setPosition(p)
		obj.scale = vec3(1, 1, 1)
		obj:registerObject("gm_window_test")
		scenetree.MissionGroup:add(obj)
	end
	if false then
		local base = {
			vec3(0, 0, 0),
			vec3(-15, 0, 0),
			vec3(-15, -8, 0),
			vec3(0, -8, 0),
		}
		for i = 1,#base do
--        for i = 1,2 do
			local u = base[i % #base + 1] - base[i]
			wallUp({base = u, h = 6, pos = p}, 'wall')
			p = p + u
		end
	end

	lo('<< regionUp:')
--    local root_group = SimGroup()
--    root_group:addObject(obj)
end


-- EVENTS
local function circleResize(ind, dir)
--    lo('?? circleResize:'..ind..':'..dir)
--    anode = {}
--    counter = {}
	Net.circleSeed(nNodes, {resize = dir})
--    aloop = {}
	Net.pathsUp()
end

local myGroup
local cmouse = nil
local cobj = nil

local function onClick(alt)

--    myGroup = display.newGroup()
--            local pt2i = editor.screenToClient(Point2I(im.GetMousePos().x, im.GetMousePos().y))
--            local canvas = scenetree.findObject("Canvas")
--            local srayCast = staticRayCast()
--        lo('?? onClick:'..cameraMouseRayCast(true).object.name..':'..cameraMouseRayCast().object.name)
--        if true then return end
	local rayCast = cameraMouseRayCast(true)
	if rayCast == nil then
		lo('!! onClick_NO_RAYCAST:')
		return
	end
--        editor.editModes.cityEditMode.auxShortcuts['Alt-Click'] = tostring(rayCast.pos)
--        extensions.hook("onEditorEditModeChanged", nil, nil)
--		lo('?? CE.onClick:'..#anode) --..editor.editModes.cityEditMode.auxShortcuts['Alt-Click'])
	cmouse = rayCast.pos
	cobj = rayCast.object:getID()
--            lo('?? SO:'..tostring(rayCast.object.name)..':'..cobj..':'..tostring(rayCast.pos)) --..tostring(editor.objectIconHitId)..':'..tostring(editor.selection.object))
	-- identify circle
	local icirc
	for i = 1,#anode do
		if anode[i].r and (anode[i].pos - rayCast.pos):length() < anode[i].r then
			icirc = i
			break
		end
	end
	if rayCast ~= nil then
	-- check meshes selection
		local nm = rayCast.object.name
--		    lo('?? CE.RC:'..tostring(nm)..':'..tostring(rayCast.pos)) --..' true:'..tostring(cameraMouseRayCast(true).object.name)..' false:'..tostring(cameraMouseRayCast(false).object.name)) --..':'..tostring(editor.selection.forestItem)
		if nm ~= nil then
			if U._PRD == 0 then
				if string.find(nm, 'road_') == 1 then
					local rd = rayCast.object
						lo('?? if_dec:'..tostring(rd.decal))
					if rd.decal ~= nil then
						lo('?? onClick.for_junc:'..rd.decal)
						if icirc ~= nil then
								lo('?? region:'..icirc..':'..rd:getID())
							Net.junctionUp(icirc)
						end
					end
				elseif string.find(nm, 'exit_') == 1 then
					local rd = rayCast.object
					Net.exitBranch(rd)
				end
			else
--					lo('?? for_scope:'..tostring(W.forScope())..':'..nm)
				return W.mdown(rayCast, {R=R, D=D}) or nm ~= 'theTerrain'
			end
		else
--            inedit =
--            W.mdown(rayCast)
--                lo('?? ce_edit:'..tostring(inedit))
		end
	end
	lo('<< onClick:')
end
--[[
		elseif string.find(nm, 'o_') == 1 then
			local id = rayCast.object:getID()
			editor.selectObjectById(id)
			lo('?? to_select:'..id)
			if string.find(nm, 'o_building_') == 1 then
				W.houseUp(nil, id)
				inedit = id
				rayCast = cameraMouseRayCast(true)
				id = rayCast.object:getID()
--                    lo('?? for_part:'..tostring(rayCast.object))
			end
			W.partOn(id)

			if false then
				for _,inf in pairs(amesh) do
					if inf.id == editor.selection.object[1] then
						lo('?? found:')
						cmesh = {obj = rayCast.object, info = inf}
	--                    rayCast.object:createMesh({{{}}})
					end
				end
			end
--            local anode = editor.getNodes(rayCast.object)
--            lo('?? nodes:'..#anode)
--            editor.selectObjects({rayCast.object:getID()})
		elseif nm == 'theForest' then
			local its = W.out.fdata:getItemsCircle(rayCast.pos, 0.2)
			lo('?? fits:'..tostring(#its))
			if #its > 0 then
				W.forestEdit(its[1]:getKey())
			end
		elseif icirc ~= nil then
			lo('?? for_circle:'..tostring(icirc))
			U.toggle(Net.sregion, icirc)
		else
			if alt then
				W.regionUp(rayCast.pos)
			end
		end
]]
--[[
			-- get region index
			local mi, imi = 1/0, nil
			for o,n in pairs(anode) do
				local d = (n.pos - rayCast.pos):length()
				if d < mi then
					mi = d
					imi = o
				end
			end
			if imi ~= nil then
				lo('?? region:'..imi..':'..rd:getID())
				Net.junctionUp(imi)
			end

					if #Net.sregion == 1 then
						_dbg = false
--                                circleResize(Net.sregion[1], 1)
						_dbg = true
					else
--                                Net.circleSeed(nNodes)
					end
]]
--<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
--local screenshot = require("screenshot")
--lo('?? SSH:'..tostring(screenshot))
local inpoint = false

local function onUpdate()

--    if M.onEGui() then return end
--    local rayCast = cameraMouseRayCast(false)
--    lo('?? onUpd:')

--    if D.onUpdate() then return end
--    D.onUpdate()
--    R.onUpdate({D = D, W = W})
	W.onUpdate()

	if false and myGroup ~= nil then
		local square = display.newRect( myGroup, 0, 0, 100, 100 )  --red square is at the bottom
		square:setFillColor( 1, 0, 0 )
	end

	-- debug points
	if W.out.border ~= nil then
		for _,base in pairs(W.out.border) do
			for i = 1,#base do
				debugDrawer:drawLine(base[i], base[i%#base + 1], ColorF(1,0,0,1), 2)
			end
		end
--        local base = W.out.border
	end
--[[
	if W.out.wsplit ~= nil and inpoint then
--        lo('?? LINE:')
		debugDrawer:drawLine(W.out.wsplit[1], W.out.wsplit[2], ColorF(1,1,0,1))
	end
]]
	if W.out.frame ~= nil then
		local p = W.out.frame[1]
		local u,v,w = W.out.frame[2],W.out.frame[3],W.out.frame[4]
		local lw = 4
--        debugDrawer:drawLineInstance(p, p + u, lw, ColorF(0,1,1,0.4))
		debugDrawer:drawLine(p, p + u, ColorF(0,1,1,0.4),lw)
		debugDrawer:drawLine(p, p + v, ColorF(0,1,1,0.4),lw)
		debugDrawer:drawLine(p + v, p + v + u, ColorF(0,1,1,0.4),lw)
		debugDrawer:drawLine(p + u, p + v + u, ColorF(0,1,1,0.4),lw)

		debugDrawer:drawLine(p, p + w, ColorF(0,1,1,0.4),lw)
		debugDrawer:drawLine(p + u + v, p + u + v + w, ColorF(0,1,1,0.4),lw)
		debugDrawer:drawLine(p + v, p + v + w, ColorF(0,1,1,0.4),lw)
		debugDrawer:drawLine(p + u, p + u + w, ColorF(0,1,1,0.4),lw)

		p = p + w
		debugDrawer:drawLine(p, p + u, ColorF(0,1,1,0.4),lw)
		debugDrawer:drawLine(p, p + v, ColorF(0,1,1,0.4),lw)
		debugDrawer:drawLine(p + v, p + v + u, ColorF(0,1,1,0.4),lw)
		debugDrawer:drawLine(p + u, p + v + u, ColorF(0,1,1,0.4),lw)
	end
	if W.out.apath ~= nil then
		for _,p in pairs(W.out.apath) do
			for i = 2,#p do
				debugDrawer:drawLine(p[i-1], p[i], ColorF(1,1,0,1),6)
			end
		end
	end
	if D.out.apath ~= nil then
		for _,p in pairs(D.out.apath) do
			for i = 2,#p do
				debugDrawer:drawLine(p[i-1], p[i], ColorF(1,1,0,1),2)
			end
		end
	end

	if W.out.apick ~= nil then
		for _,s in pairs(W.out.apick) do
			local r = 0.01*math.sqrt((s-core_camera.getPosition()):length())
			debugDrawer:drawSphere(s, r, ColorF(0,1,1,.6), true)
		end
	end
	if D.out.apick ~= nil then
		for _,s in pairs(D.out.apick) do
			local r = 0.03*math.sqrt((s-core_camera.getPosition()):length())
			debugDrawer:drawSphere(s, r, ColorF(0,1,1,1))
		end
	end

	if W.out.avedit ~= nil then
		for _,s in pairs(W.out.avedit) do
			local r = 0.02*math.sqrt((s-core_camera.getPosition()):length())
	--        lo('?? for_dbg:'..tostring(s))
			if W.out.apop and W.out.apop[_] then
				debugDrawer:drawSphere(s, r, ColorF(1,1,0,1),false)
			else
				debugDrawer:drawSphere(s, r, ColorF(1,1,0,1),true)
			end
	--        debugDrawer:drawSphere(s, 0.2, ColorF(1,1,0,1))
		end
	end
--[[
	if D.out.avedit ~= nil then
--        lo('?? FOR_D_out:'..#D.out.avedit)
		for _,s in pairs(D.out.avedit) do
			local r = 0.02*math.sqrt((s-core_camera.getPosition()):length())
	--        lo('?? for_dbg:'..tostring(s))
			debugDrawer:drawSphere(s, r, ColorF(1,1,1,1))
	--        debugDrawer:drawSphere(s, 0.2, ColorF(1,1,0,1))
		end
	end
]]

	if W.out.apoint ~= nil then
		for _,p in pairs(W.out.apoint) do
			local r = 0.03*math.sqrt((p-core_camera.getPosition()):length())
			debugDrawer:drawSphere(p, r, ColorF(1,0,1,1), false)
		end
	end
	if D.out.apoint ~= nil then
		for _,p in pairs(D.out.apoint) do
			local r = 0.03*math.sqrt((p-core_camera.getPosition()):length())
			debugDrawer:drawSphere(p, r, ColorF(1,0,1,1))
		end
	end

	for _,s in pairs(adbg) do
--        lo('?? DS:'..tostring(s[1]))
		local c = ColorF(1,0,0,1)
		if s[2] ~= nil then
			c = s[2]
		end
		local w = 0.6
		if s[3] ~= nil then
			w = s[3]
		end
		debugDrawer:drawSphere(s[1], 0.6, c)
	end

	if #anode then
		-- circles
		for o,n in pairs(anode) do
			local clr = color(255,255,255,255)
			if #U.index(Net.sregion, o) > 0 then
				-- selected
				clr = color(255,255,0,255)
			end
			if n.r then
				Render.circle(n.pos, n.r, clr)
			end
		end
	end
	if #apath > 0 then
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
						lo('!! NO_NODE:'..j..':'..l[j]..':'..l[j - 1]..'/'..#anode)
						apath = {}
					else
						Render.path({ anode[l[j-1]].pos, anode[l[j]].pos }, c, w)
					end
				end
			end
		end
	end
	-- decals sceletons
	for i,rd in pairs(roads) do
		local path = {}
		for j = 1,#rd.path  do
			path[#path + 1] = anode[rd.path[j]].pos
 --           Mesh.toLineBuf(anode[rd.path[j-1]].pos, anode[rd.path[j]].pos, (j-2)*6)
		end
		Render.path(path, color(255,255,0,50), 3)
--        M.pathUp(buf, #l, color(255,255,0,50), 5)
	end
	for o = 1,#anode do
		debugDrawer:drawText(anode[o].pos, String(tostring(o)), ColorF(0,0,1,1))
	end

	-- EVENTS

	--- KEYS
--    if im.IsKeyPressed(im.GetKeyIndex(im.Key_Backspace)) then
--        W.onKey(im.Key_Backspace)
--    end
--    if im.IsKeyPressed(im.GetKeyIndex(im.Key_D)) and editor.keyModifiers.ctrl then
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_X)) and editor.keyModifiers.ctrl then
		reload()
	end

	if im.IsKeyPressed(im.GetKeyIndex(im.Key_Enter)) then
		W.onKey(im.Key_Enter)
	end
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_Backspace)) then
		W.onKey(im.Key_Backspace)
	end
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_Space)) then
		W.onKey(im.Key_Space)
	end
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_Escape)) then
		W.onKey(im.Key_Escape)
	end

--[[
		lo('?? if_edit:'..tostring(inedit))
		if inedit ~= nil then
			scenetree.findObject('edit'):deleteAllObjects()
			W.houseUp(W.adesc[inedit], inedit)
			inedit = nil
		end
]]

	inpoint = false
--    if im.IsKeyPressed(im.GetKeyIndex(im.Key_Z)) then
--[[
	if editor.keyModifiers.alt then
--    if editor.keyModifiers.shift then
--        W.mpoint()
		inpoint = true

--        if not editor.keyModifiers.alt then
--        if not editor.keyModifiers.alt and not editor.keyModifiers.ctrl then
--        end
--    elseif inpoint then
--        lo('?? pointOff:')
--        inpoint = false
	end
]]
	if im.IsKeyReleased(im.GetKeyIndex(im.Key_ModAlt)) == true then
--        lo('?? ALT_OFF:')
		W.keyAlt(false)
	end
	if im.IsKeyReleased(im.GetKeyIndex(im.Mod_Shift)) == true then
		W.keyShift(false)
	end

	--- MOUSE
	local ishit -- TODO (NOT) don't ruin the fun with camelcase, the var name is fine as-is :-D
	if im.IsMouseClicked(0) then
--            lo('?? MC:'..cameraMouseRayCast(true).object.name)
		if not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not im.IsAnyItemHovered() then
			ishit = onClick(editor.keyModifiers.alt)
--				lo('?? CE_postclick:'..tostring(ishit))
			if im.IsWindowHovered(im.HoveredFlags_AnyWindow) or im.IsAnyItemHovered() then
				ishit = true
			end
			if not ishit then
--                D.onUpdate()
			end
		end
	end
	if U._PRD==0 and not ishit then
		D.onUpdate()
		R.onUpdate({D = D, W = W})
	end

--[[
		if inpoint then
--            W.wallSplit()
		else
		end
]]
--[[
	if im.IsMouseDragging(0) then
		local rayCast = cameraMouseRayCast(true)
		lo('?? ce_drag:'..tostring(rayCast))

		if rayCast ~= nil and not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not im.IsAnyItemHovered() then
			lo('?? ce_drag:'..tostring(rayCast))
			W.mdrag(rayCast)
		else
--            lo('!! NO_raycast:x')
		end

		if editor.selection.object then -- and inedit then
		end
	end
]]
	if im.IsMouseReleased(0) then
--            lo('?? CE_mup:')
		if false and W.mup(cameraMouseRayCast(true), {R = R, D = D}) then
--                lo('!!_____________ ce.STOP_PROPAGATE:')
--            inedit = nil
			return
		end
		if im.IsKeyPressed(im.GetKeyIndex(im.Key_X)) then
				lo('?? bE_RELOAD:')
			reload()
		end
	end

	local w = im.GetIO().MouseWheel
	if w ~= 0 then
		-- scroll
--        lo('?? ce_scroll:')
		if false then
			if W.mwheel(w, cameraMouseRayCast(true)) then
			elseif U._PRD == 0 and #Net.sregion == 1 then
				circleResize(Net.sregion[1], w)
			end
		end
	end
--[[
			if string.find(rayCast.object.name, 'o_') == 1 then
				W.matMove(rayCast.pos - cmouse, rayCast.object:getID())
			elseif rayCast.object.name == 'theForest' then
				local its = W.out.fdata:getItemsCircle(rayCast.pos, 0.2)
--                lo('?? fits:'..tostring(#its))
				if #its > 0 then
					W.mdrag(its[1]:getKey())
				end
			end
]]
--            lo('?? for_drag:'..tostring(rayCast.object.name))
--            lo('?? drag:'..tostring(cmouse)..'>'..tostring(rayCast.pos)..':'..tostring(cobj))
--            W.matMove(rayCast.pos - cmouse, cobj)
--[[
		if false and editor.selection.object and cmesh then
			lo('?? dragging:'..editor.selection.object[1])
			cmesh.obj:createMesh({{{}}})
			wallUp(cmesh.info.pos, cmesh.info, nil, cmesh.obj)
--            cmesh.obj:createMesh({{{}}})
		end
]]
--    if inCircleSel ~= nil then
--  if editor.keyModifiers.alt then
--[[
	local w = im.GetIO().MouseWheel
	if w ~= 0 then
		if editor.keyModifiers.alt then
			if W.mwheel(w) then
			elseif #Net.sregion == 1 then
				circleResize(Net.sregion[1], w)
			end
		end
	end
]]
--    if imgui.IsItemHovered() then
--[[
	if editor.keyModifiers.alt then
		if im.IsKeyPressed(im.GetKeyIndex(im.Key_RightArrow)) then
--            W.goAround()
			if false then
				-- move camera
				local direction = vec3(core_camera.getForward())
				local startPoint = vec3(core_camera.getPosition())

				lo('?? RA:'..tostring(direction)..':'..tostring(startPoint))
				local newPos = U.vturn(U.proj2D(startPoint - direction), 0.05) + vec3(0, 0, startPoint.z)
			end

--           core_camera.setPosition(0, U.proj2D(direction) + newPos)
--            local newDir = U.proj2D(direction) - newPos
--            lo('?? RA2:'..tostring(newDir)..':'..tostring(newPos))
--            core_camera.setRotation(0, quatFromDir(newDir, vec3(0,0,1)))

--++            screenshot.doScreenshot(nil, nil, 'for_ster', 'png')
----            screenshot.takeScreenshot()
		end
		if im.IsKeyPressed(im.GetKeyIndex(im.Key_LeftArrow)) then
			local direction = vec3(core_camera.getForward())
			local fwd = core_camera.getForward()
			local startPoint = vec3(core_camera.getPosition())

			lo('?? LA:'..tostring(direction)..tostring(fwd)..':'..tostring(startPoint))
			local newPos = U.vturn(startPoint - direction, 0.2)
--            core_camera.setPosition(0, newPos)
--            core_camera.setRotation(0, quatFromDir(direction,vec3(0,0,1)))
--            screenshot.doScreenshot(nil, nil, 'for_ster', 'png')
		end

--        if im.IsKeyPressed(im.GetKeyIndex(im.Key_A)) then
		-- take shot
--        end
	end
]]

	if im.IsKeyPressed(im.GetKeyIndex(im.Key_UpArrow)) then
		W.keyUD(1)
	end
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_DownArrow)) then
		W.keyUD(-1)
	end

	if im.IsKeyPressed(im.GetKeyIndex(im.Key_RightArrow)) then
		W.keyRL(1)
	end
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_LeftArrow)) then
		W.keyRL(-1)
	end

--    lo('?? wheel:'..tostring(w))
end


local function onUp()
		print('?? BE_onUp:')
	editor.clearObjectSelection()
	editor.selectEditMode(editor.editModes.cityEditMode)

	reload('conf')
	W.reload('conf')
	editor.showWindow('LAT')
	UU._MODE = 'conf'
	UU.out._MODE = 'conf'
	UI.inject({U = UU, W = W})
	UI.hint(editor.editModes.cityEditMode)
	W.up(D, nil, true, 'conf')

--	editor.hideWindow('BAT')
end
M.onUp = onUp


local isfirst = true

if U._PRD == 1 then
	if scenetree.findObject('edit') then
--      lo('??____ FOUND_edit:')
		scenetree.MissionGroup:removeObject(scenetree.findObject('edit'))
	end
	if scenetree.findObject('lines') then
		scenetree.MissionGroup:removeObject(scenetree.findObject('lines'))
	end
	if scenetree.findObject('e_road') then
		scenetree.MissionGroup:removeObject(scenetree.findObject('e_road'))
		local obj = scenetree.findObject('e_road')
		if obj then
			obj:delete()
--			print('??^^^^^^^^^^ BE_removed:')
		end
	end
--		print('??^^^^^^^^^^ BE:'..tostring(scenetree.findObject('e_road')))
end

local function onActivate()
--		print('>>______________________________________________________ onActivate:'..tostring(U._PRD))
--	lo('>> onActivate:'..tostring(isfirst)..':'..tostring(U._PRD)..' edit:'..tostring(scenetree.findObject('edit')))

	inedit = true
	if U._PRD == 1 then
		if scenetree.findObject('edit') then
--      lo('??____ FOUND_edit:')
			scenetree.MissionGroup:removeObject(scenetree.findObject('edit'))
		end
		if scenetree.findObject('lines') then
			scenetree.MissionGroup:removeObject(scenetree.findObject('lines'))
		end
		if scenetree.findObject('e_road') then
			scenetree.MissionGroup:removeObject(scenetree.findObject('e_road'))
		end
	end
--[[
    else
        lo('??_____________ edit_TOMAKE:')
--    	local go = createObject('edit')
      local go = createObject("SimGroup")
      go:registerObject('edit')
  		scenetree.MissionGroup:addObject(go)
      lo('??__________ MAID_edit:')
]]
	editor.showWindow('BAT')
	if UU._PRD == 0 and ({conf=0,ter=0})[UU._MODE] then
		editor.showWindow('LAT')
	end

--    W.buildingGen()
--    if _dbg then scenetree.findObject("thePlayer"):setMeshAlpha(0, "", false) end

--    UI.hint(editor.editModes.cityEditMode)

--[[
	if scenetree.findObject("edit") == nil then
		local go = createObject("SimGroup")
		go:registerObject('edit')
		scenetree.MissionGroup:addObject(go)
	end
]]
--            core_camera.setRotation(0, quatFromDir(vec3(0, -1, -1),vec3(0,0,1)))
--            core_camera.setPosition(0, vec3(0, 40, 60))
	if U._PRD == 0 and isfirst then
		isfirst = false
		local pos = vec3(0, 0, 5) - 2*vec3(0, -10, -1)
		local q = quatFromDir(vec3(0, -10, -1),vec3(0,0,1))

--[[
		-- for base conform, "close to hill"
		pos = vec3(15, -72, 10)
		local hter = core_terrain.getTerrainHeight(pos)
		pos.z = pos.z + hter
		q = quat(0.0490453, 0.179371, -0.947768, 0.259147)
]]

		core_camera.setRotation(0, q)
		core_camera.setPosition(0, pos)

		local cpos = core_camera.getPosition()
		local sz = cpos.z
		cpos.z = 0
		local ch = core_terrain.getTerrainHeight(cpos)
--            lo('??___ for_z:'..ch..':'..sz)
		if true or sz < ch + 1 then
			cpos.z = ch + 10
			core_camera.setPosition(0, cpos)
--            lo('?? set:')
		end
	end
--		lo('?? pre_up:'..#anode)

--            core_camera.setRotation(0, quatFromDir(vec3(0, -0.0001, -1),vec3(0,0,1)))
--            core_camera.setPosition(0, camInitial)

--            core_camera.setPosition(0, vec3(0, 6, 2.5))
--            core_camera.setRotation(0, quatFromDir(vec3(0, -0.9, -0.3),vec3(0,0,1)))
--!!
	if U._PRD == 0 and #anode == 0 then
		Net.circleSeed(nNodes)
		Net.pathsUp()
		if true or _dbg then
			Net.toDecals()
			Net.junctionUp(6)
		end
	end
--			print('?? to_inject:'..tostring(U._PRD))
	W.up(D, nil, true)

--    editor.registerWindow("PANE", im.ImVec2(200, 500)) --, im.ImVec2(400, 400))
--    editor.showWindow("PANE")
--	lo('<< onActivate:'..#anode)
end


local function onDeactivate()
--		print('>> onDeactivate:'..tostring(U._PRD)..':'..tostring(scenetree.findObject('e_road')))
		if scenetree.findObject('e_road') then
			scenetree.MissionGroup:removeObject(scenetree.findObject('e_road'))
			local obj = scenetree.findObject('e_road')
			if obj then
				obj:delete()
			end
		end
		if scenetree.findObject('e_road') then
			print('!! BAT.CLEANUP_FOLDERS')
		end
	W.onQuit(false)
	inedit = false
	editor.hideWindow("BAT")
	if U._PRD == 1 then
--			print('?? BAT_deact:'..tostring(scenetree.findObject('e_road')))
		if scenetree.findObject('edit') then
			scenetree.MissionGroup:removeObject(scenetree.findObject('edit'))
        		lo('?? onDeactivate_edit_REMOVED:'..tostring(scenetree.findObject('edit')))
	--        scenetree.findObject('edit'):delete()
		end
		if scenetree.findObject('lines') then
			scenetree.MissionGroup:removeObject(scenetree.findObject('lines'))
	--        scenetree.findObject('edit'):delete()
		end
--		U._MODE = 'BAT'
	end

--[[
	if scenetree.findObject('edit') then
		scenetree.findObject('edit'):delete()
	end
	if scenetree.findObject('lines') then
		scenetree.findObject('lines'):delete()
	end
]]
end


local function onEditorGui()
--!!        if true then return end
	if inedit then
		UI.control(W.dmat.wall, W, D, R)
--        lo('?? EGUI:')
		-- render materilas list
--        return UI.control(W.dmat.wall, W, D, R)
	end

--[[
	nnodePtr[0] = nNodes
	if im.SliderInt("# Nodes_", nnodePtr, 4, 32, "%d") then
		nNodes = nnodePtr[0]
		_dbg = nil

		massCenter = vec3(0, 0, 0)
		anode, apath, apairskip = {}, {}, {}
		jointpath = {}
		edges = {astamp = {}, infaces = {}}

		Net.circleSeed(nNodes)
		Net.pathsUp()
	end
	if im.Button('Decals_') then
		if #roads > 0 then
			-- cleanup
			for _,id in pairs(adecalID) do
				scenetree.findObjectById(id):delete()
			end
			adecalID = {}
			for _,id in pairs(ameshID) do
				scenetree.findObjectById(id):delete()
			end
			ameshID = {}
			roads = {}
			adbg = {}
--            apath, apairskip = {}, {}
--            jointpath = {}
			--local apair = {}
--            edges = {astamp = {}, infaces = {}}
		else
			Net.toDecals()
		end
	end
	if im.Button('Save') then
		fout = io.open('city_'..tostring(socket.gettime())..'.out', "w")
		for _,nd in pairs(anode) do
			dump(nd, true)
		end
		fout:close()
	end
]]
--[[
	if editor.beginWindow("PANE", "PANE") then
--        lo('?? BW:')
		local spacing = im.ImVec2(500, 20)
		im.Dummy(spacing)
	end
	editor.endWindow()
	editor.showWindow("PANE")
]]
end

--??
local function customDecalRoadMaterialsFilter(materialSet)
	--TODO: attention, if this function is registered as a field filter it needs to return a (filtered) table of objects out of the materialSet
	-- otherwise it will break the inspector materials popup menu
end


local function onDeserialize(data)
--		print('?? onDeserialize:'..tostring(scenetree.findObject('e_road')))
--	if scenetree.findObject('e_road') then
--		scenetree.MissionGroup:removeObject(scenetree.findObject('e_road'))
--	end
--	unrequire(apack[2])
--	UI = rerequire(apack[2])
--	reload()
	local jdesc = jsonDecode(data['jdata'])
--	local mode = data['mode']
--	U._MODE = mode
		lo('>> bE.onDeserialize:'..tostring(data)..':'..tostring(data['jdata'] and true or false))
	W.recover(jdesc, function()
--		lo('?? soft_RELOAD:')
		unrequire(apack[2])
		UI = rerequire(apack[2])
--[[
		unrequire(apack[3])
		D = rerequire(apack[3])
		unrequire(apack[4])
		R = rerequire(apack[4])
]]
		editor.showWindow('BAT')
--		reload(true)
	end)
--		U.dump(data, '?? onDeserialize:')
--		lo('?? onDeserialize:'..#adesc)
--	W.onQuit()
end


local function onSerialize()
--		print('??^^^^^^^^^^ onSerialize:'..tostring(scenetree.findObject('e_road')))
	if scenetree.findObject('e_road') then
		scenetree.MissionGroup:removeObject(scenetree.findObject('e_road'))
		local obj = scenetree.findObject('e_road')
		if obj then
			obj:delete()
		end
	end
	local jdata = W.onQuit()
		print('?? onSer:'..tostring(U._MODE))
		lo('>> bE.onSerialize:') --..#jdata)
	return {jdata = jdata}
end


local function onEditorInitialized()
		lo('?? onEditorInitialized:')
	if false then
		return
	end
	local fname = '/lua/ge/extensions/editor/gen/inprod'
	local fprod = io.open(fname, "r")
	if fprod then
		editor.addWindowMenuItem("LandscapeGenerator (WIP)", onUp, {groupMenuName = 'Experimental'})
		fprod.close()
	end

--	editor.addWindowMenuItem("CityEditor (WIP)", onUp, {groupMenuName = 'Experimental'})

	editor.editModes.cityEditMode =
	{
		displayName = "Edit " .. 'meshEditor.type',
		onActivate = onActivate,
		onDeactivate = onDeactivate,
--		onDeserialize = onDeserialize,
		onUpdate = onUpdate, --meshEditor.onUpdate_,
--      onToolbar = onToolbar,
		actionMap = "CE",
--      onCopy = meshEditor.copySettingsAM,
--      onPaste = meshEditor.pasteFieldsAM,
--      onDeleteSelection = meshEditor.onDeleteSelection,
--      onSelectAll = onSelectAll, --meshEditor.onSelectAll,

--		icon = editor.icons.build, -- U._PRD == 0 and editor.icons.build or editor.icons.houseWrenchRoof,
		icon = editor.icons.houseWrench, -- U._PRD == 0 and editor.icons.build or (editor.icons.houseWrench or editor.icons.house_wrench or editor.icons.build), --Roof,
--        iconTooltip = "CityEditor(!WIP!)",
		iconTooltip = "Building Architect",
--[[
]]

--      iconTooltip = "CE2",
		auxShortcuts = {},
		hideObjectIcons = true
	}
	-- inject
	UI.hint(editor.editModes.cityEditMode)


--??
			--editor.registerCustomFieldInspectorFilter("DecalRoad", "Material")
--            editor.registerCustomFieldInspectorFilter("DecalRoad", "Material", customDecalRoadMaterialsFilter)
			editor_roadUtils.reloadTemplates()

--    editor.editModes.cityEditMode.auxShortcuts['Alt-Click'] = 'new building'
--    editor.editModes.cityEditMode.auxShortcuts[editor.AuxControl_Copy] = "Copy objects"
end

local function onInput(type, value)
	lo('?? INP:')
end

local function onEditorObjectSelectionChanged()
	--    lo('>> onEditorObjectSelectionChanged:')
	if tableSize(editor.selection.object) == 1 and editor.selection.object[1] then
		D.out.inconform = false
			--TODO: this print was in production, please comment when releasing
			--lo('>> onEditorObjectSelectionChanged:'..tostring(editor.selection.object[1])) --..tostring(scenetree.findObjectById(editor.selection.object[1]).name))
--        D.ter2road()
	end
end

--print('<<^^^^^^^^^^^^^^^^^ nE:'..tostring(scenetree.findObject('e_road')))

--[[
if scenetree.findObject('e_road') then
	scenetree.MissionGroup:removeObject(scenetree.findObject('e_road'))
	local obj = scenetree.findObject('e_road')
	if obj then
		obj:delete()
		print('??^^^^^^^^^^^ e_road_REMOVED:')
	end
end
if scenetree.findObject('lines') then
	scenetree.MissionGroup:removeObject(scenetree.findObject('lines'))
	local obj = scenetree.findObject('lines')
	if obj then
		obj:delete()
	end
end

local function onEditorObjectSelectionChanged()
--    lo('>> onEditorObjectSelectionChanged:')
	D.onSelect()
end
]]

M.onDeserialize = onDeserialize
M.onSerialize = onSerialize

M.onEditorInitialized = onEditorInitialized
--M.onEGui = onEditorGui
M.onEditorGui = onEditorGui -- onEditorGui
M.onInput = onInput
M.onEditorObjectSelectionChanged = onEditorObjectSelectionChanged

--M.onUpdate = onUpdate
--M.onExtensionLoaded = onExtensionLoaded
--[[
M.onEditorInspectorHeaderGui = meshEditor.onEditorInspectorHeaderGui_
M.onEditorRegisterPreferences = meshEditor.onEditorRegisterPreferences_
M.onEditorPreferenceValueChanged = meshEditor.onEditorPreferenceValueChanged_
M.onEditorInspectorFieldChanged = meshEditor.onEditorInspectorFieldChanged_
M.onEditorAxisGizmoAligmentChanged = meshEditor.onEditorAxisGizmoAligmentChanged_
M.onEditorObjectSelectionChanged = meshEditor.onEditorObjectSelectionChanged_
]]

return M

--[[
					if #U.index(jointpath, i) == 0 then
						afree[#afree + 1] = i
					else
						local inpath = U.index(jointpath, i)
						if #inpath == 1 then
							if jointpath[inpath[1] + 1] == 0 or jointpath[inpath[1] - 1] == 0 then
								-- is open end
								afree[#afree + 1] = i
							end
						end
					end
]]
--[[
		if path[i] == 11 and path[i - 1] == 20 then
--            if path[i] == 20 and path[i - 1] == 12 then
			nb, nc, ne = isObtuse(path, path[i - 1], true)
			lo('?? for_20>11:'..tostring(nb)..':'..tostring(nc)..':'..tostring(ne))
		else
		end
		nb, nc, ne = isObtuse(path, path[i + 1])
		if nc == path[i - 1] and (nb == path[i] or ne == path[i]) then
			lo('??==== obt2:'..path[i - 1]..':'..tostring(nb)..'>'..tostring(nc)..'>'..tostring(ne))
		end
		local star = forStar(path[i], path)
		local aang = linkOrder(path[i], forStar(path[i], path))
		if #aang > 2 then
			for o = 2,#aang do
				if aang[o][2] - aang[o - 1][2] > 7/6*math.pi then

				end
			end
		end
		if #aang > 3 then
			U.dump(star, '?? for_star:'..i..':'..path[i])
			U.dump(aang, '?? for_link:')
		end
		local aang = linkOrder(path[i], forStar(path[i], path))
		for o,l in pairs(aang) do
		end
		if #aang > 4 then
			U.dump(aang, '?? AANG:'..i..':'..#aang)
		end
]]

--[[!
			av[#av + 1] = rd:getLeftEdgePosition(60 + i)
			av[#av].z = 1
			av[#av + 1] = rd:getLeftEdgePosition(60 + i)
			av[#av + 1] = rd:getRightEdgePosition(60 + i)
			av[#av + 1] = rd:getRightEdgePosition(60 + i)
			av[#av].z = 1

			af[#af + 1] = {v = 4*(i - 1) + 0, n = 0, u = 0}
			af[#af + 1] = {v = 4*(i - 1) + 3, n = 0, u = 3}
			af[#af + 1] = {v = 4*(i - 1) + 1, n = 0, u = 1}

			af[#af + 1] = {v = 4*(i - 1) + 1, n = 0, u = 1}
			af[#af + 1] = {v = 4*(i - 1) + 3, n = 0, u = 3}
			af[#af + 1] = {v = 4*(i - 1) + 4, n = 0, u = 4}

			af[#af + 1] = {v = 4*(i - 1) + 1, n = 0, u = 1}
			af[#af + 1] = {v = 4*(i - 1) + 4, n = 0, u = 4}
			af[#af + 1] = {v = 4*(i - 1) + 2, n = 0, u = 2}

			af[#af + 1] = {v = 4*(i - 1) + 2, n = 0, u = 2}
			af[#af + 1] = {v = 4*(i - 1) + 4, n = 0, u = 4}
			af[#af + 1] = {v = 4*(i - 1) + 5, n = 0, u = 5}

]]

--[[
+        af[#af + 1] = {v = 3*(i - 1) + 1, n = 0, u = 1}
		af[#af + 1] = {v = 3*(i - 1) + 3, n = 0, u = 3}
		af[#af + 1] = {v = 3*(i - 1) + 2, n = 0, u = 2}


		af[#af + 1] = {v = 3*(i - 1) + 1, n = 0, u = 1}
		af[#af + 1] = {v = 3*(i - 1) + 2, n = 0, u = 2}
		af[#af + 1] = {v = 3*(i - 1) + 3, n = 0, u = 3}

		af[#af + 1] = {v = 2*(i - 1) + 1, n = 0, u = 1}
		af[#af + 1] = {v = 2*(i - 1) + 2, n = 0, u = 2}
		af[#af + 1] = {v = 2*(i - 1) + 3, n = 0, u = 3}


		av[#av + 1] = rd:getLeftEdgePosition(60)
		av[#av + 1] = rd:getRightEdgePosition(60)

		av[#av + 1] = rd:getLeftEdgePosition(61)
		av[#av + 1] = rd:getRightEdgePosition(61)

		af[#af + 1] = {v = 2*(i - 1) + 0, n = 0, u = 0}
		af[#af + 1] = {v = 2*(i - 1) + 2, n = 0, u = 2}
		af[#af + 1] = {v = 2*(i - 1) + 1, n = 0, u = 1}

		af[#af + 1] = {v = 2*(i - 1) + 1, n = 0, u = 1}
		af[#af + 1] = {v = 2*(i - 1) + 2, n = 0, u = 2}
		af[#af + 1] = {v = 2*(i - 1) + 3, n = 0, u = 3}
]]

--[[
		av[#av+1] = rd:getLeftEdgePosition(i) - marginSide*perp -- + vec3(0, 0, h - rd:getLeftEdgePosition(i).z)
		av[#av].z = h
		av[#av+1] = rd:getRightEdgePosition(i) + marginSide*perp -- + vec3(0, 0, h - rd:getRightEdgePosition(i).z)
		av[#av].z = h

		af[#af + 1] = {v = 2*(i - 1), n = 0, u = 0}
		af[#af + 1] = {v = 2*(i - 1) + 2, n = 0, u = 2}
		af[#af + 1] = {v = 2*(i - 1) + 1, n = 0, u = 1}

		af[#af + 1] = {v = 2*(i - 1) + 1, n = 0, u = 1}
		af[#af + 1] = {v = 2*(i - 1) + 2, n = 0, u = 2}
		af[#af + 1] = {v = 2*(i - 1) + 3, n = 0, u = 3}


		av2[#av2 + 1] = rd:getRightEdgePosition(i) + marginSide*perp
		av2[#av2].z = h - 0.2
		av2[#av2 + 1] = rd:getLeftEdgePosition(i) - marginSide*perp
		av2[#av2].z = h - 0.2

		af2[#af2 + 1] = {v = 2*(i - 1), n = 0, u = 0}
		af2[#af2 + 1] = {v = 2*(i - 1) + 2, n = 0, u = 2}
		af2[#af2 + 1] = {v = 2*(i - 1) + 1, n = 0, u = 1}

		af2[#af2 + 1] = {v = 2*(i - 1) + 1, n = 0, u = 1}
		af2[#af2 + 1] = {v = 2*(i - 1) + 2, n = 0, u = 2}
		af2[#af2 + 1] = {v = 2*(i - 1) + 3, n = 0, u = 3}


		av2[#av2 + 1] = rd:getLeftEdgePosition(i) - marginSide*perp
		av2[#av2].z = h + 1
		av2[#av2 + 1] = rd:getLeftEdgePosition(i) - marginSide*perp
		av2[#av2].z = h - 0.0
		av2[#av2 + 1] = rd:getRightEdgePosition(i) + marginSide*perp
		av2[#av2].z = h - 0.0
		av2[#av2 + 1] = rd:getRightEdgePosition(i) + marginSide*perp
		av2[#av2].z = h + 1

		af2[#af2 + 1] = {v = 3*(i - 1) + 0, n = 0, u = 0}
		af2[#af2 + 1] = {v = 3*(i - 1) + 3, n = 0, u = 3}
		af2[#af2 + 1] = {v = 3*(i - 1) + 1, n = 0, u = 1}

		af2[#af2 + 1] = {v = 3*(i - 1) + 3, n = 0, u = 3}
		af2[#af2 + 1] = {v = 3*(i - 1) + 4, n = 0, u = 4}
		af2[#af2 + 1] = {v = 3*(i - 1) + 1, n = 0, u = 1}

		af2[#af2 + 1] = {v = 3*(i - 1) + 2, n = 0, u = 2}
		af2[#af2 + 1] = {v = 3*(i - 1) + 3, n = 0, u = 3}
		af2[#af2 + 1] = {v = 3*(i - 1) + 0, n = 0, u = 0}

		af2[#af2 + 1] = {v = 3*(i - 1) + 2, n = 0, u = 2}
		af2[#af2 + 1] = {v = 3*(i - 1) + 5, n = 0, u = 5}
		af2[#af2 + 1] = {v = 3*(i - 1) + 3, n = 0, u = 3}
]]

--[[
		af2[#af2 + 1] = {v = 3*(i - 1) + 1, n = 0, u = 1}
		af2[#af2 + 1] = {v = 3*(i - 1) + 2, n = 0, u = 2}
		af2[#af2 + 1] = {v = 3*(i - 1) + 4, n = 0, u = 4}

		af2[#af2 + 1] = {v = 3*(i - 1) + 4, n = 0, u = 4}
		af2[#af2 + 1] = {v = 3*(i - 1) + 2, n = 0, u = 2}
		af2[#af2 + 1] = {v = 3*(i - 1) + 5, n = 0, u = 5}



		af2[#af2 + 1] = {v = 3*(i - 1) + 1, n = 0, u = 1}
		af2[#af2 + 1] = {v = 3*(i - 1) + 4, n = 0, u = 4}
		af2[#af2 + 1] = {v = 3*(i - 1) + 2, n = 0, u = 2}

		af2[#af2 + 1] = {v = 3*(i - 1) + 2, n = 0, u = 2}
		af2[#af2 + 1] = {v = 3*(i - 1) + 4, n = 0, u = 4}
		af2[#af2 + 1] = {v = 3*(i - 1) + 5, n = 0, u = 5}

]]
--[[
		af[#af + 1] = {v = L*(i - 1) + (k - 1 + 0), n = 0, u = k - 1}
		af[#af + 1] = {v = L*(i - 1) + (k - 1 + L), n = 0, u = k - 1 + L}
		af[#af + 1] = {v = L*(i - 1) + (k - 1 + 1), n = 0, u = k}

		af[#af + 1] = {v = L*(i - 1) + (k - 1 + 1), n = 0, u = k}
		af[#af + 1] = {v = L*(i - 1) + (k - 1 + L), n = 0, u = k - 1 + L}
		af[#af + 1] = {v = L*(i - 1) + (k - 1 + L + 1), n = 0, u = k + L}
]]
--[[
	av[#av + 1] = rd:getLeftEdgePosition(60)
	av[#av].z = 1
	av[#av + 1] = rd:getLeftEdgePosition(60)
--        av[#av + 1] = rd:getMiddleEdgePosition(60)
	av[#av + 1] = rd:getRightEdgePosition(60)
	av[#av + 1] = rd:getRightEdgePosition(60)
	av[#av].z = 1
]]
--[[
			if false then
				av[#av + 1] = rd:getLeftEdgePosition(60 + i)
				av[#av].z = 1
				av[#av + 1] = rd:getLeftEdgePosition(60 + i)
				av[#av + 1] = rd:getRightEdgePosition(60 + i)
				av[#av + 1] = rd:getRightEdgePosition(60 + i)
				av[#av].z = 1

				af[#af + 1] = {v = 4*(i - 1) + 0, n = 0, u = 0}
				af[#af + 1] = {v = 4*(i - 1) + 4, n = 0, u = 4}
				af[#af + 1] = {v = 4*(i - 1) + 1, n = 0, u = 1}

				af[#af + 1] = {v = 4*(i - 1) + 1, n = 0, u = 1}
				af[#af + 1] = {v = 4*(i - 1) + 4, n = 0, u = 4}
				af[#af + 1] = {v = 4*(i - 1) + 5, n = 0, u = 5}

				af[#af + 1] = {v = 4*(i - 1) + 1, n = 0, u = 1}
				af[#af + 1] = {v = 4*(i - 1) + 5, n = 0, u = 5}
				af[#af + 1] = {v = 4*(i - 1) + 2, n = 0, u = 2}

				af[#af + 1] = {v = 4*(i - 1) + 2, n = 0, u = 2}
				af[#af + 1] = {v = 4*(i - 1) + 5, n = 0, u = 5}
				af[#af + 1] = {v = 4*(i - 1) + 6, n = 0, u = 6}

				af[#af + 1] = {v = 4*(i - 1) + 2, n = 0, u = 2}
				af[#af + 1] = {v = 4*(i - 1) + 6, n = 0, u = 6}
				af[#af + 1] = {v = 4*(i - 1) + 3, n = 0, u = 3}

				af[#af + 1] = {v = 4*(i - 1) + 3, n = 0, u = 3}
				af[#af + 1] = {v = 4*(i - 1) + 6, n = 0, u = 6}
				af[#af + 1] = {v = 4*(i - 1) + 7, n = 0, u = 7}
			end
]]

--        aexit[i]
--        U.dump(indec, '?? junctionUp:'..#r.av)
--[[
				if (decal:getMiddleEdgePosition(j) - cpos):length() > dex + dexpre then
					aee[#aee].pre = j
					break
				end
		lo('?? for_rd:'..r.meshid)
		if r.meshid == meshid then
			road = r
			break
		end
]]
--[[
		hmap[0] = U.hightOnCurve(
			U.proj2D(edec:getMiddleEdgePosition(0)),
			deco, exout.ei, ro.av, ro.avstep)
		apin[#apin + 1] = 0
				lo('?? for_hh:'..tostring(th)..':'..hmap[0])
		local h = U.hightOnCurve(
			U.proj2D(edec:getMiddleEdgePosition(0)),
			deco, exout.ei, ro.av, ro.avstep)
			lo('??+++++++++++++++ h0:'..h)
		hmap[0] = h --ro.av[exout.ei*ro.avstep + 1].z
		apin[#apin + 1] = 0
]]

--[[
		local jo = math.floor(jb/2)
		local h = U.hightOnCurve(
			U.proj2D(edec:getMiddleEdgePosition(jo)),
			deco, exout.ei, ro.av, ro.avstep)
		hmap[jo] = h
		h = U.hightOnCurve(
			U.proj2D(edec:getMiddleEdgePosition(jb)),
			deco, exout.ei, ro.av, ro.avstep)
		hmap[jb] = h -- ro.av[exout.ei*ro.avstep + 1].z
		apin[#apin + 1] = jo
		apin[#apin + 1] = jb

]]


--[[

		lo('??++++ hi_pre:'..ne..':'..tostring(edec:getMiddleEdgePosition(je))..':'..exin.ei..':'..#ri.av..':'..ri.avstep)
		h = U.hightOnCurve(
			U.proj2D(edec:getMiddleEdgePosition(ne)),
			deci, exin.ei, ri.av, ri.avstep, true)
		lo('??+++++++ hi:'..tostring(h)..':'..tostring(fromAV(edec, ne, ri, exin.ei)))

--        hmap[je] = ri.av[exin.ei*ri.avstep + 1].z
		hmap[ne] = ri.av[exin.ei*ri.avstep + 1].z
--        apin[#apin + 1] = je
		apin[#apin + 1] = ne

		hmap[je] = fromAV(edec, je, ri, exin.ei)
		apin[#apin + 1] = je
		hmap[ne] = fromAV(edec, ne, ri, exin.ei)
		apin[#apin + 1] = ne
		hmap[je] = fromAV(edec, je, ri, exin.ei)
		apin[#apin + 1] = je
		hmap[ne] = fromAV(edec, ne, ri, exin.ei)
		apin[#apin + 1] = ne


		h = U.hightOnCurve(
			U.proj2D(edec:getMiddleEdgePosition(je)),
			deci, exout.ei, ro.av, ro.avstep)
		hmap[je] = ri.av[exin.ei*ri.avstep + 1].z
		hmap[ne] = ri.av[exin.ei*ri.avstep + 1].z
		apin[#apin + 1] = je
		apin[#apin + 1] = ne

]]
--[[
		if false then
			be:reloadCollision()
			scenetree.findObjectById(r.id):delete()
				lo('?? to_del:'..r.id..':'..#adec..':'..tostring(r.meshid)..':'..#r.av..':'..nsec)
			-- reset heights
			if true then
				for o,n in pairs(adec) do
	--                lo('?? np:'..o..':'..tostring(n.pos))
					n.pos.z = hmap[r.apin[o] ] + 0.2
	--                editor.setNodePosition(rd, o, n.pos + 1*vec3(0, 0, hmap[r.apin[o] ] + 0.2))
end
end
-- update decal
local id = editor.createRoad(
	adec,
	{overObjects = true, drivability = 1.0}
)
r.id = id
rdmesh.decal = id
local road = scenetree.findObjectById(id)
road:setField("material", 0, mat)
end
--        scenetree.findObjectById(r.id):delete()

]]

--[[
	local alev = {}
	local levels = {}
	levels = toLevels(levels, dec, #list2 - 1, 1)
	levels = toLevels(levels, dec, #list2 - 1, -1)
]]
--[[
		hmap[0] = fromAV(edec, 0, ro, exout.ei)
		apin[#apin + 1] = 0
		hmap[jb] = fromAV(edec, jb, ro, exout.ei)
		apin[#apin + 1] = jb

		hmap[je] = fromAV(edec, je, ri, exin.ei)
		apin[#apin + 1] = je
		hmap[ne - 1] = fromAV(edec, ne - 1, ri, exin.ei)
		apin[#apin + 1] = ne - 1
]]
		--        ro.av[exout.ei*ro.avstep + 1]

		--[[
--                U.dump(hmap, '?? hmap_up:')
--                U.dump(apin, '?? apin_up:')
		--- update decal
--                U.dump(hmap, '?? E_hmap:')
--                U.dump(apin, '?? E_apin:')
		if false then
			local brin = inedge[round(aang[o % #aang + 1][1]/2)]
			local dirin = 2*(aang[o % #aang + 1][1] % 2 - 0.5)
					lo('?? dir_OI:'..dirout..':'..dirin)
			local pin, ppin = exitPos(roads[brin[1] ], dex,
	--            edgeAtDist(brin[2], dex, dirin, dirin, roads[brin[1] ]),
	dirin, -dirin)
	adbg[#adbg + 1] = {pin}
	adbg[#adbg + 1] = {ppin, ColorF(0,0,1,1)}
end

]]

--                U.dump(hpin, '?? hpin:')
--[[
		hpin = {
			{0, ro, ieo},
--            {round(jb/2), ro, exout.ei},
			{jb, ro, ieo},
			{je, ri, exin.ei},
--            {round((ne - 1 + je)/2), ri, exin.ei},
--            {je + 4, ri, exin.ei},
--            {je + 8, ri, exin.ei},
			{ne - 1, ri, exin.ei},
		}
]]

--[[
local function exitPos_(rdinfo, ind, side, dir)
	local dpre = dexpree
	local av, step = rdinfo.av, rdinfo.avstep

	local function pos(i, sd)
		if sd == 1 then
			-- right edge
			return av[(i + 1)*step]
		else
			-- left edge
			return av[i*step + 1]
		end
	end
	local s, ipre = 0, nil
	for i = ind, ind - dir*100, -dir do
--                lo('?? for_e:'..i..':'..(i - dir)..'/'..ind..':'..tostring(pos(i, side))..':'..tostring(U.proj2D(pos(i, side) - pos(i - dir, side)):length()))
		s = s + U.proj2D(pos(i, side) - pos(i - dir, side)):length()
		if s > dpre then
			ipre = i
			break
		end
	end
		lo('?? ipre:'..ind..':'..ipre..':'..U.proj2D(pos(ind, side) - pos(ind - dir, side)):length())
	-- normal vector
	local vn = (pos(ipre, side) - pos(ipre, -side)):normalized()
	local p = pos(ind, side) - vn*marginSide
	local pp = pos(ipre, side) - vn*(wExit/2 + marginSide)

	return p, pp
end
]]

--[[
local Wall = {}
function Wall:up(v, h)
	local f = {}
	setmetatable(f, Floor)
	f.v = v
	f.h = h
	return f
end

function Wall:on()
end

local Floor = {}
function Floor:up(base)
	local o = {}
	setmetatable(o, Floor)
	o.base = base
	return o
end

function Floor:add(wall)
end

]]
