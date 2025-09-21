--lo('== UTILS:')
--[[
local a = {2,4,6,8}
print('??^^^^^^^^^^^ TST1:'..#a)
a[3] = nil
print('??^^^^^^^^^^^ TST2:'..#a..':'..tableSize(a))
a[#a+1] = 9
print('??^^^^^^^^^^^ TST3:'..#a..':'..tableSize(a)..':'..tostring(a[3])..':'..tostring(a[#a]))
]]

local U = {out = {}}

U._PRD = 1
U._MODE = --'ter'
	'BAT'
	--'conf'
U.out = {
	_MODE = U._MODE
}

local small_val = 0.00001
local small_ang = 0.001
local small_dist = 0.01
local indrag

local function lo(ms, yes)
  	if U._PRD ~= 0 then return end
	if indrag and not yes then return end
	print(ms)
end


U.inView = function()
    return not ui_imgui.IsWindowHovered(ui_imgui.HoveredFlags_AnyWindow) and not ui_imgui.IsAnyItemHovered()
end


local function camSet(acoord)
	core_camera.setPosition(0, vec3(acoord[1], acoord[2], acoord[3]))
	core_camera.setRotation(0, quat(acoord[4], acoord[5], acoord[6], acoord[7]))
end


local function angDist(a, b)
	if not b then
		local rayCast = cameraMouseRayCast(false)
		if rayCast then b = rayCast.pos end
	end
	if not b then return end

	return U.vang(a-core_camera.getPosition(), b-core_camera.getPosition())
end
U.angDist = angDist


local function fromJSON(t)
	if type(t) == 'table' then
		if t.x and t.y then
			return vec3(t.x,t.y,t.z)
		else
			for o,e in pairs(t) do
	--            lo(type(o)..':'..type(e)..':'..o..':'..tostring(e))
				t[o] = U.fromJSON(e)
	--            lo('?? dret:'..lt)
			end
			return t
		end
--        if lvl == 0 then lo(s ..' }') end
--        return s..' }'
	else
		return t
--        lo('?? dump_nn:'..type(t)..':'..tostring(t))
--        return tostring(t)
	end
end


local function dump(t, msg, lvl)
  	if false and U._PRD ~= 0 then return end
	if not lvl or lvl == true then lvl = 0 end
--    lo('?? dump:'..tostring(lvl)..':'..type(t))
	if not t then
		lo((msg or '')..tostring(t))
		return
	end
	if type(t) == 'table' then
		local s = '{ '
		local indent = ''
		for i = 1,lvl do
			indent = indent..'\t'
		end
		for o,e in pairs(t) do
--            lo(type(o)..':'..type(e)..':'..o..':'..tostring(e))
			local lt = U.dump(e, nil, lvl+1)
--            lo('?? dret:'..lt)
			s = s..'\r\n'..indent..o..' = '..tostring(lt)..','
		end
		if msg ~= nil then lo(msg) end
		if lvl == 0 then lo(s ..' }') end
		return s..' }'
	else
--        lo('?? dump_nn:'..type(t)..':'..tostring(t))
		return tostring(t)
	end
	return ''
end


local function markUp(list, rma, clr, plus)
	if list and list.list then
		plus = list.plus
		list = list.list
	end
	if not list then return end
	for _,p in pairs(list) do
		local r = rma * math.sqrt((p - core_camera.getPosition()):length())
--        p.z = core_terrain.getTerrainHeight(U.proj2D(p))
		debugDrawer:drawSphere(p + plus, r, clr, false)
	end
end


local function debugPoints(list, plus)
	local out = {}
	for _,v in pairs(list) do
		out[#out + 1] = v + plus
	end
	return out
end


local function fileCopy(old_path, new_path)
		lo('?? fileCopy:'..old_path..':'..new_path)
	local old_file = io.open(old_path, "rb")
	local new_file = io.open(new_path, "wb")
	local old_file_sz, new_file_sz = 0, 0
	if not old_file or not new_file then
	  return false
	end
	while true do
	  local block = old_file:read(2^13)
	  if not block then
		old_file_sz = old_file:seek( "end" )
		break
	  end
	  new_file:write(block)
	end
	old_file:close()
	new_file_sz = new_file:seek( "end" )
	new_file:close()
	return new_file_sz == old_file_sz
end


local function forBackup(name)
--        lo('??+++++++++++++++++++++++++ forBackup:'..tostring(name), true)
	if not name then return end
	local astr = U.split(name, '.', true)
	astr[#astr - 1] = astr[#astr - 1]..'.s'

	return table.concat(astr, ".")
end


local function forOrig(name)
	local astr = U.split(name, '.', true)
	if astr[#astr - 1] == 's' then
		table.remove(astr, #astr - 1)
	end
	return table.concat(astr, ".")
end

----------------------------------
-- ARRAY
----------------------------------

local function rand(a, b)
	return a + math.random()*(b - a)
end


local function perm(a)
	local L = #a
	local aperm ={}
	local n = 0
	while L > 0 and n<10 do
		aperm[#aperm+1] = table.remove(a, math.random(L))
		L = #a
		n = n+1
	end
	return aperm
end


local function index(list, p, prop)
	local apos = {}
	if not list then return apos end
	for i,_ in pairs(list) do
--    for i = 1,#list do
		if prop then
			if list[i][prop] == p then
				apos[#apos+1] = i
			end
		else
			if list[i] == p then
				apos[#apos+1] = i
			end
		end
	end
	return apos
end


U.pop = function(list, e)
	local ind = U.index(list, e)[1]
	if ind then
		table.remove(list, ind)
	end
end


local function clone(arr)
	if not arr then return end
	local tnew = {}
	for k,v in pairs(arr) do
		tnew[k] = v
	end
	return tnew
end


local function push(arr, val, single)
	if single and #index(arr, val) > 0 then return end
	arr[#arr + 1] = val
end


local function union(a,b)
	for j,q in pairs(b) do
		if #index(a, q) == 0 then
			a[#a+1] = q
		end
	end
end
U.union = union


local function boxMark(obj, out)
	-- mark object
	local ob = obj:getWorldBox()
	out.avedit = {}
	local c = vec3(0,0,0)
	for _,x in pairs({ob.minExtents.x, ob.maxExtents.x}) do
		for _,y in pairs({ob.minExtents.y, ob.maxExtents.y}) do
			for _,z in pairs({ob.minExtents.z, ob.maxExtents.z}) do
				out.avedit[#out.avedit+1] = vec3(x, y, z)
				c = c + vec3(x, y, z)
			end
		end
	end
	return c/8
end


local function mod(ai, n)
	if type(n) == 'table' then
		if type(ai) == 'table' then
		elseif ai then
			return n[(ai - 1) % #n + 1]
		end
	else
		if type(ai) == 'table' then
			for o,i in pairs(ai) do
				ai[o] = (i - 1) % n + 1
			end
		else
			return (ai - 1) % n + 1
		end
	end
	return ai
end


--local function toLinear()

----------------------------------
-- STRINGS
----------------------------------

local function split(s, d, plain)
	local t = {}
	if not s or not d then return t end
	for str in string.gmatch(s, "([^"..d.."]+)") do
		if plain then
			t[#t + 1] = str
		else
			t[#t + 1] = tonumber(str)
		end
	end
	return t
end


local chop = function(str, delimiter)
	local result = {};
	local startPoint, endPoint = 1, 1;
	for i = 1, str:len() do
		if (i + 1) > str:len() then
			endPoint = str:len();
		end
		if (str:sub(i, i + (delimiter:len() - 1)) == delimiter) then
			endPoint = i - 1;
			table.insert(result, str:sub(startPoint, endPoint));
			startPoint = i + (delimiter:len());
		elseif (i == str:len()) then
			endPoint = str:len();
			table.insert(result, str:sub(startPoint, endPoint));
		end
	end
	return result;
end


local function stamp(list, plain)
	if not plain then
		table.sort(list)
	end
	if #list == 0 then
		return ''
	end
	local s = tostring(list[1])
	for o = 2,#list do
		s = s..'_'..list[o]
	end
	return s
end

----------------------------------
-- GEOMETRY
----------------------------------
local function rcWH(rc)
	return math.max(math.abs(rc[2].x-rc[1].x), math.abs(rc[2].x-rc[3].x)),
		math.max(math.abs(rc[2].y-rc[1].y), math.abs(rc[2].y-rc[3].y))
end

--[[
local function cycle(list, n, dir)
	if not dir then dir = 1 end
	for i=n,1,-1 do
		local itm = table.remove(list, i)
		table.insert(list, #list - n-1, itm)
	end
end
]]

U.curv = function(a,b,c)
--	local a,b,c = vec3(list[i-1].d,list[i-1].h), vec3(list[i].d,list[i].h), vec3(list[i+1].d,list[i+1].h)
	return U.vang(c-b, b-a, true)/a:distance(c)
end


U.line2line = function(a1, a2, b1, b2)
	local x = ((a1.x*a2.y - a1.y*a2.x)*(b1.x - b2.x) - (a1.x - a2.x)*(b1.x*b2.y - b1.y*b2.x))/
	  ((a1.x - a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x - b2.x))
	local y = ((a1.x*a2.y - a1.y*a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x*b2.y - b1.y*b2.x))/
	  ((a1.x - a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x - b2.x))

	return vec3(x,y)
end


local function lineCross(a1, a2, b1, b2)
	local x = ((a1.x*a2.y - a1.y*a2.x)*(b1.x - b2.x) - (a1.x - a2.x)*(b1.x*b2.y - b1.y*b2.x))/
	  ((a1.x - a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x - b2.x))
	local y = ((a1.x*a2.y - a1.y*a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x*b2.y - b1.y*b2.x))/
	  ((a1.x - a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x - b2.x))
	return x, y
end


U.linePlaneHit = function(line, aplane)
	local mi = math.huge
	for i,p in pairs(aplane) do
		local d = intersectsRay_Plane(line[1], line[2]-line[1], p[1], (p[2]-p[1]):cross(p[3]-p[1]))
--            U.dump(p, '?? linePlaneHit:'..d..':'..tostring(line[1])..':'..tostring(line[2]), true)
		if d < mi then mi = d end
	end
	return mi
end


U.onLine = function(a, b, list, sence)
  sence = sence or 0
  for _,p in pairs(list) do
    if (a-p):cross(b-p):length() > sence then
      return false
    end
  end
  return true
end


U.between = function(a,b,list,sence)
  sence = sence or 0
  local L = (a-b):length()
  for _,p in pairs(list) do
    if math.abs((p-a):length()+(p-b):length()-L) > sence then
      return false
    end
  end
  return true
end


local function lineHit(src, tgt)
	local x,y = lineCross(src[1],src[2],tgt[1],tgt[2])
	local p = vec3(x,y)
	local s = (p-tgt[1]):length()/(tgt[2]-tgt[1]):length()

	return p,s
end
U.lineHit = lineHit


local function line2seg(a, b, c, d, sense)
	if not sense then sense = 0 end
--        lo('?? for_cross:'..tostring(a)..':'..tostring(b)..':'..tostring(c)..':'..tostring(d)) --..':'..tostring(p))
--[[
  local ang = U.vang(a-b,c-d)
  if math.abs(ang) <= sense or math.abs(ang-math.pi) <= sense then
    local rc,rd = ((a+b)/2-c):length(),((a+b)/2-d):length()
      lo('?? line2seg:'..rc..':'..rd..' ang:'..ang)
--    return rc < rd and c,0 or d,1
  end
]]
	if U.onLine(a,b,{c,d},sense) then
		local rc,rd = ((a+b)/2-c):length(),((a+b)/2-d):length()
		return rc < rd and c,0 or d,1
	end

	local x,y = lineCross(a, b, c, d)
--		lo('?? l2s:'..tostring(x)..':'..tostring(y))
	local p = vec3(x,y)
	if (c-p):dot(d-p) <= sense then
--    if (vec3(c[1],c[2],0)-p):dot(vec3(d[1],d[2],0)-p) <= 0 then
		local s = (p-c):length()/(d-c):length()
		if math.abs(s) <= sense then
			return c,0
		elseif math.abs(1-s) <= sense then
			return d,1
		else
			return p,s
		end
	end
end


U.toPoly = function(p, base)
	-- closest node
	local dmi,imi = math.huge
	for i,v in pairs(base) do
		local d = p:distance(v)
		if d<dmi then
			dmi = d
			imi = i
		end
	end
	if imi then
		local c,s
		if imi > 1 then
			c,s = U.line2seg(p,p+U.vturn(base[imi-1]-base[imi],math.pi/2), base[imi-1],base[imi])
		end
		if imi < #base then
			c,s = U.segCross(p,p+U.vturn(base[imi+1]-base[imi],math.pi/2), base[imi],base[imi+1])
		end

		return c or base[imi],imi
	end
end


U.polyCross = function(base, a, dir)
--        if U._PRD == 1 then return end
	a = U.proj2D(a)
--        U.dump(base, '>> polyCross:'..tostring(a)..':'..tostring(dir:normalized()))
	local ahit = {}
	for i=1,#base do
--            lo('?? if_HIT:'..i..':'..tostring(U.proj2D(base[i]))..':'..tostring(U.proj2D(U.mod(i+1,base))))
		local p = U.line2seg(a, a + dir, U.proj2D(base[i]), U.proj2D(U.mod(i+1,base)))
		if p then
			if (p - U.mod(i+1,base)):length() == 0 then
				ahit[#ahit+1] = {U.mod(i+1,#base), p}
			else
				ahit[#ahit+1] = {i, p}
			end
		end
	end
--        U.dump(ahit, '??++++++++++++++++++++ polyCross:')
	local mileft,miright = math.huge,math.huge
	-- nearist to cursor border crossing points
	local pleft,pright
	for i,p in pairs(ahit) do
		local dp = p[2]-a
		local l = dp:length()
--            lo('?? if_Z:'..i..':'..tostring((U.mod(p[1]+1,base)-U.mod(p[1],base)):cross(dp)))
		if dir:dot(dp) < 0 and l < mileft then
			if (U.mod(p[1]+1,base)-U.mod(p[1],base)):cross(dp).z < 0 then
				mileft = l
				pleft = p
			end
--            lo('?? for_LEFT:'..tostring((U.mod(p[1]+1,base)-U.mod(p[1],base)):cross(dp)))
--            mileft = l
		end
		if dir:dot(dp) > 0 and l < miright then
			if (U.mod(p[1]+1,base)-U.mod(p[1],base)):cross(dp).z < 0 then
				miright = l
				pright = p
			end
--            lo('?? for_RIGHT:'..tostring((U.mod(p[1]+1,base)-U.mod(p[1],base)):cross(dp)))
--            mileft = l
		end
	end
	return {pleft,pright}
end


U.polyCut = function(base, pfr, pto)
--        U.dump(base, '?? polyCut_base:'..#base)
--        U.dump(pfr, '?? polyCut_fr:')
--        U.dump(pto, '?? polyCut_to:')
  if not pfr then return end
	local imap = {pfr[1]}
	local b12 = {pfr[2]}
	for i=pfr[1]+1,pfr[1]+#base do
		local ci = U.mod(i, #base)
		b12[#b12+1] = base[ci]
		imap[#imap+1] = U.mod(i,#base)
		if ci == pto[1] then
			-- finalize
			b12[#b12+1] = pto[2]
			imap[#imap+1] = pto[1]
			break
		end
	end
	local map
--        U.dump(imap, '?? CUT12_m:')
--        U.dump(b12, '?? CUT12_b:')
	b12,map = U.polyStraighten(b12)
--        U.dump(b12, '?? CUT12_b_post:')
--        U.dump(map, '?? CUT12_m_post:')
	local imap12 = {}
	for k,v in pairs(map) do
		imap12[#imap12+1] = imap[v]
	end
--        U.dump(imap12, '?? map12:')

	imap = {pto[1]}
	local b21 = {pto[2]}
	for i=pto[1]+1,pto[1]+#base do
		local ci = U.mod(i, #base)
		b21[#b21+1] = base[ci]
		imap[#imap+1] = U.mod(i,#base)
		if ci == pfr[1] then
			-- finalize
			b21[#b21+1] = pfr[2]
			imap[#imap+1] = pfr[1]
			break
		end
	end
--        U.dump(imap, '?? CUT21_m:')
--        U.dump(b21, '?? CUT21_b:')
	b21,map = U.polyStraighten(b21)
--        U.dump(b21, '?? CUT21_b_post:')
--        U.dump(map, '?? CUT21_m_post:')
	local imap21 = {}
	for k,v in pairs(map) do
		imap21[#imap21+1] = imap[v]
	end
--        U.dump(imap21, '?? map21:')
	return {b12, b21}, {imap12,imap21}
--    return {U.polyStraighten(b12), U.polyStraighten(b21)}
end
--[[
	local list,map = U.polyStraighten(b12)
		U.dump(b12, '?? CUT12_b_post:')
		U.dump(map, '?? CUT12_m_post:')
--        U.dump(U.polyStraighten(b12))
		U.dump(b21, '?? CUT21:')
--        U.dump(U.polyStraighten(b21, true))
	U.polyStraighten(b12)

	U.polyStraighten(b12)
	U.polyStraighten(b21)
	return {b12, b21}
]]


-- mrc = {uvorigin, uvscale}
local function rcPave(rc, aloop, mrc, apatch, istest)
	if not mrc then
		mrc = {{0,0}, {1,1}}
	end
--        lo('>> rcPave:', true)
--        if true then return end
--            dump(apatch, '>> rcPave:'..#aloop)
--            dump(rc, '>> rcPave_hull:')
			indrag = true
	local out = {}
	local aeref = deepcopy(aloop)
	table.insert(aeref, 1, rc)
--    aeref[#aeref+1] = rc

	local e4v,v4e = {},{}
	local ae,av = {},{}
	local iv = 1
--            local nvert = #rc
	-- initial objects linking
	for i,loop in pairs(aeref) do
--            nvert = nvert + #loop
		v4e[iv] = {p=loop[1], ij={i,1}} -- vert_ind->containing_edge_ind_list,  p=node pos, ij={loop_ind,node_in_loop_ind}
		for j=2,#loop do
			iv = iv + 1
			v4e[iv] = {p=loop[j], ij={i,j}}
			ae[#ae+1] = {loop[j-1],loop[j],iloop = i}

			e4v[#e4v+1] = {} -- edge_index -> vert_list {ind=vert_ind, d=relative_dist_to_edge_start}
			-- strat
			e4v[#e4v][#e4v[#e4v]+1] = {ind = iv-1, d = 0, done = 0} -- ind=node, d=relative_dist_to_edge_start
			-- end
			e4v[#e4v][#e4v[#e4v]+1] = {ind = iv, d = 1, done = 0}
--                lo('?? to_v4e: i='..i..' j='..j..':'..#e4v..'>'..(iv-1)..':'..iv)
			v4e[iv-1][#v4e[iv-1]+1] = #e4v
			v4e[iv][#v4e[iv]+1] = #e4v
		end
--        iv = iv + 1
		ae[#ae+1] = {loop[#loop],loop[1],iloop = i}
		e4v[#e4v+1] = {}
		e4v[#e4v][#e4v[#e4v]+1] = {ind = iv, d = 0, done = 0}
		e4v[#e4v][#e4v[#e4v]+1] = {ind = iv-#loop+1, d = 1, done = 0}

--            lo('?? to_v4e2: i='..i..':'..#e4v..'>'..(iv-1)..':'..iv)
		v4e[iv][#v4e[iv]+1] = #e4v
		v4e[iv-#loop+1][#v4e[iv-#loop+1]+1] = #e4v

		iv = iv + 1
--        v4e[iv] = {p=}
	end
	-- original vertices number
	local nv = #v4e
--        lo('??________________________ rcPave:'..nvert..':'..#rc..':'..#aloop)
		if istest then lo('??_____________________ NV:'..nv..':'..#nvert, true) end
	local estamp = {}
	local sense = 0.000001
	for k=#rc+1,nv do
--    for k=1,nv-#rc do
		local dmi,cmi,imi,isend,isdupe = math.huge

		local function forCross(vinfo, v)
				local dbg = false
--                    vinfo.ij[1] == 2 and vinfo.ij[2] ==1 -- false -- #v4e == 14
--                    vinfo.ij[1] == 1 and vinfo.ij[2] ==2 -- false -- #v4e == 14
				if dbg then
					lo('>>************ forCross:'..tostring(v), true)
--                    U.dump(vinfo, '>>************ forCross:'..tostring(v)..':'..#v4e)
				end
			local p = vinfo.p
			local c,d
			for i,e in pairs(ae) do
--                isend = false
				if e.iloop ~= vinfo.ij[1] then
					c = line2seg(p, p+v, e[1], e[2], sense)
						if i == 2 then
--                            lo('?? if_CROSS:'..tostring(p)..':'..tostring(p+v)..':'..tostring(e[1])..':'..tostring(e[2]), true)
						end
--                        if dbg then U.dump(e4v[i], '?? for_E:'..i..':'..tostring(c)) end
--                        if dbg then lo('?? for_E:'..i..':'..e4v[i][1].ind..':'..e4v[i][2].ind..':'..tostring(c), true) end
					if dbg and c then
--                        U.dump(e4v[i], '??+++ for_e:'..i..':'..tostring(c))
--                        U.dump(e, '??+++ for_e:'..i..':'..tostring(c))
					end
					if c then
						d = (c - p):length()
						if d == 0 then
								if istest then U.dump(e4v[i],'?? on_EDGE:'..i) end
							isdupe = true
							dmi = d
							cmi = c
							imi = i
							break
--                            dmi = d
--                            cmi = c
--                            imi = i
						elseif v:dot(p-c) < 0 then    -- intersection is in v-direction
--                            d = (c - p):length()
							if d < dmi then
								dmi = d
								cmi = c
								imi = i
								if (e[1] - cmi):length() < sense then
									isend = e4v[i][1].ind
									if dbg then U.dump(e4v[i], '??_____________ END1:'..isend) end
								elseif (e[2] - cmi):length() < sense then
	--                                isend = true
									isend = e4v[i][#e4v[i]].ind
									if dbg then U.dump(e4v[i], '??_____________ END2:'..isend) end
								else
									if dbg then lo('??___________ not_END:', true) end
									isend = false
								end
							end
						end
					end
				end
			end
			if dbg then lo('<<************ forCross:'..tostring(cmi)..' d:'..tostring(dmi)..' isend:'..tostring(isend), true) end
		end

		local p = v4e[k].p
		local loop = aeref[v4e[k].ij[1]]
		local j = v4e[k].ij[2]
			if istest then lo('?? pre_CROSS:'..k, true) end
		isdupe = false
		imi = nil
		forCross(v4e[k], (p - U.mod(j-1,loop)):normalized())
		forCross(v4e[k], (p - U.mod(j+1,loop)):normalized())
		if isdupe then
				if istest then U.dump(e4v[imi], '??__________________ DUPE:'..tostring(cmi)..':'..tostring(imi)) end
			v4e[k].isdupe = true
		end
--            lo('?? post_CROSS:'..k..':'..tostring(cmi)..':'..tostring(isend))
			if v4e[k].ij[1] == 2 and v4e[k].ij[2] ==1 then
				lo('?? for_2:'..k..' cmi:'..tostring(cmi)..':'..tostring(isend))
			end
		if isdupe then
			-- old vert to old edge
			local d = (cmi-ae[imi][1]):length()/(ae[imi][2]-ae[imi][1]):length()
--                U.dump(e4v[imi], '?? v:'..k..' to:'..tostring(d))
			e4v[imi][#e4v[imi]+1] = {ind = k, d = d, done = 0}
--                lo('?? v4e:'..k..':'..#v4e[k]..':'..imi, true)
			v4e[k][#v4e[k]+1] = imi
--                U.dump(v4e[k], '?? v___:'..k..' to:'..tostring(d)..':'..imi)
		elseif imi then
--            U.dump(ae[imi], '?? for_cross: k='..k..' ie:'..imi..':'..tostring(p))
			-- edges connecting loops
--            ae[#ae+1] = {p,cmi} -- edge_ends_coords
			-- links
			--- new vert to prev edge
--            if true then
--                    lo('?? next_vert:'..)
			local vn
			if not isend then
				local d = (cmi-ae[imi][1]):length()/(ae[imi][2]-ae[imi][1]):length()
				e4v[imi][#e4v[imi]+1] = {ind = #v4e+1, d = d, done = 0}
				vn = #v4e+1
			else
				vn = isend
			end
			local est = stamp({k,vn})
--[[
				if est == '3_5' then
					lo('??______________________________________ is_3_5:')
					for i,_ in pairs(estamp) do
						lo('?? if_STAMP:'..i..':'.._)
					end
				end
]]
			local ie = index(estamp, est)
			if #ie == 0 then
				ae[#ae+1] = {p,cmi} -- edge_ends_coords
				--- new edge
				e4v[#e4v+1] = {}
				e4v[#e4v][#e4v[#e4v]+1] = {ind = k, d = 0, done = 0}
				e4v[#e4v][#e4v[#e4v]+1] = {ind = vn, d = 1, done = 0}
				estamp[#e4v] = est
--                estamp[#estamp+1] = est
			--- p-vertex to new edge
				v4e[k][#v4e[k]+1] = #ae
			else
				-- TODO: add to v4e
				v4e[k][#v4e[k]+1] = ie[1]
				lo('!!!!!!!!!!!!!!!!!!!____________u.EXISTS:'..est..':'..k..':'..ie[1])
			end
--            e4v[#e4v][#e4v[#e4v]+1] = {ind = #v4e+1, d = 1}
					if isend then
--                        U.dump(e4v[#e4v], '??********************* new_e4v:'..#v4e..':'..tostring(isend))
					end
			--- p-vertex to new edge
--            v4e[k][#v4e[k]+1] = #ae
			if not isend then
				--- new vertex
				v4e[#v4e+1] = {p=cmi} --, ij={#ae}}
					if istest then lo('?? new_VERT:'..#v4e, true) end
				---- to new edge
				v4e[#v4e][#v4e[#v4e]+1] = #ae
				---- to prev edge(s)
				v4e[#v4e][#v4e[#v4e]+1] = imi
			else
				-- new edge to existing vert
				v4e[isend][#v4e[isend]+1] = #e4v
			end
				if istest then lo('??+++++++++++++++++++++++++ post_CROSS:'..#v4e, true) end
		else
			lo('?? NO_CROSS:', true)
		end
	end
	for i=1,#e4v do
		table.sort(e4v[i], function(a,b)
			return a.d < b.d
		end)
	end
--        if istest then U.dump(e4v, '?? e4v:') end
		if istest then U.dump(v4e, '?? v4e:'..nv) end
		if istest then U.dump(ae, '?? ae:') end
	local albl = {}
	for i = 1,#v4e do
--    for i = (#aloop+1)*4+1,#v4e do
		albl[#albl+1] = {v4e[i].p, i}
	end
--        U.dump(v4e, '?? av:'..nv)
--            U.dump(estamp, '?? estamp:'..nv)
--            if true and istest then return {},{},ae,{},albl end

-------------------
-- GO FOR RECTS
-------------------
			local ns, nmax = 1,istest and 115 or 1800
			if istest then lo('??=============================== FOR_RECTS:'..nmax, true) end
	local function stepFrom(n1, n2)
			if istest then lo('>> stepFrom:'..n1..'>'..n2, true) end
		local dbg = false --(n1 == 15 and n2 == 4)
			if dbg then U.dump(v4e[n2], '>>__________________________ stepFrom:'..n1..'>'..n2..'/'..ns) end
		local astar = {}
		for _,ie in ipairs(v4e[n2]) do
				if dbg then U.dump(e4v[ie], '??***** for_edge:'..ie) end
			for i,n in ipairs(e4v[ie]) do
				if n.ind == n2 then
--                    lo('?? look_around:'..n2)
					if n.d == 0 then
						if e4v[ie][i+1] and e4v[ie][i+1].ind ~= n1 then
							if dbg then lo('?? if_next1:'..e4v[ie][i+1].ind) end
							astar[#astar+1] = {e4v[ie][i+1],ie,i} --.ind
						end
					elseif n.d == 1 then
						if e4v[ie][i-1].ind ~= n1 then
							if dbg then lo('?? if_next2:'..e4v[ie][i-1].ind) end
							astar[#astar+1] = {e4v[ie][i-1],ie,i} --.ind
						end
					else
						if dbg then lo('?? if_next3:') end
						if e4v[ie][i+1] and e4v[ie][i+1].ind ~= n1 then
							astar[#astar+1] = {e4v[ie][i+1],ie,i,n.d} --.ind
						end
						if e4v[ie][i-1].ind ~= n1 then
							astar[#astar+1] = {e4v[ie][i-1],ie,i,n.d} --.ind
						end
					end
				end
			end
		end
			if istest and dbg then U.dump(astar, '?? star:') end
		local v = v4e[n2].p - v4e[n1].p
		local ama,ima = -math.huge
		for _,iv in pairs(astar) do
			local u = (v4e[iv[1].ind].p - v4e[n2].p):normalized()
			local d = v:cross(u).z
--                lo('?? for_ang:'..iv[1].ind..':'..d)
			if d > ama then
				ama = d
				ima = iv --.ind
			end
		end
--            U.dump(ima, '<< stepFrom:'..tostring(ima[1].ind))
		if ima then
--            e4v[ima[2]][ima[3]] = nil
--            table.remove(e4v[ima[2]], ima[3])
--            if not e4v[ima[2]][ima[3]].done then
--                e4v[ima[2]][ima[3]].done = 0
--            end
			e4v[ima[2]][ima[3]].done = e4v[ima[2]][ima[3]].done + 1
--            e4v[ima[2]][ima[3]].done = true
--            ima.done = true
			if istest then lo('<< stepFrom:'..tostring(ima[1].ind), true) end
		else
			if istest then lo('<< stepFrom:'..'NONE', true) end
		end
		return ima[1].ind,ima[2]
--[[
		for _,i in  ipairs(e4v[ie]) do
			if i.ind == iv then
				if i.d == 1 then
				end
			end
		end
]]
	end

	local arc = {}
	local togo = true
	while togo and ns < nmax do --151 do
				lo('??================================= for_ns:'..ns)
				ns = ns + 1
		togo = false
		local rc = {}
		for i,list in pairs(e4v) do
				lo('?? for_edge:'..i)
			for k,n in ipairs(list) do
				if n.done == 0 then
--                if not n.done then
--                if n.d ~= 1 and not n.done then
					n.done = n.done + 1
--                    n.done = true
					-- starting node
--                    rc[#rc+1] = n.ind
					togo = true
--                            if istest then U.dump(list, '?? step:'..n.ind..':'..n.d) end
					local iprev, inext = n.ind, n.d < 1 and list[k+1].ind or list[k-1].ind
					rc[#rc+1] = n.ind
--                    rc[#rc+1] = inext
					local pext,ext,inxt = i
					while inext ~= rc[1] and ns < nmax do
								ns = ns + 1
						inxt,ext = stepFrom(iprev, inext)
--[[
						if ext == pext then
							rc[#rc] = inext
						else
							rc[#rc+1] = inext
						end
]]
						if ext ~= pext then
--                        if true or ext ~= pext then
							rc[#rc+1] = inext
							lo('?? for_node:'..#rc..':'..iprev..'>'..inext)
						end
							lo('??+++++ next:'..tostring(inxt)..' ext:'..pext..'>'..tostring(ext)..':'..inext)
--                        rc[#rc+1] = inext
						if not inxt then break end
--                        rc[#rc+1] = inxt
						iprev = inext
						inext = inxt
						pext = ext
					end
						if istest then U.dump(rc, '??<<<<<<< for_rc:') end
					if #rc > 2 then
						arc[#arc+1] = rc
					end
					break
				end
			end
			if togo then break end
		end
--        if togo then break end
	end
		if istest then U.dump(arc, '?? arc:'..#arc..':'..ns) end
--!!    table.remove(arc, #arc)
-----------------------------
-- CLEAN PATHS
-----------------------------
	for k=1,#arc do
		local rc = arc[k]
--            U.dump(rc, '?? for_rc:')
		local ifirst
		for i=2,#rc-1 do
			if (v4e[rc[i+1]].p - v4e[rc[i]].p):dot(v4e[rc[i]].p - v4e[rc[i-1]].p) < 0.00001 then
--                lo('?? first_in_rc:'..k..':'..i)
				ifirst = i
				break
			end
		end
		local nins = 0
		if ifirst then
			for i = ifirst-1,1,-1 do
				local ce = table.remove(rc, i)
				table.insert(rc, #rc+1-nins, ce)
				nins = nins + 1
	--            rc[#rc+1] = ce
			end
		end
--            U.dump(rc, '?? shifted:'..k)
	end
	for k=1,#arc do
		local rc = arc[k]
		for i=#rc-1,2,-1 do
			if (v4e[rc[i+1]].p - v4e[rc[i]].p):dot(v4e[rc[i]].p - v4e[rc[i-1]].p) > 0.00001 then
				-- remove colinear
--                lo('?? to_REM:'..k..':'..i)
				table.remove(rc, i)
			end
		end
		if (v4e[rc[1]].p - v4e[rc[#rc]].p):dot(v4e[rc[#rc]].p - v4e[rc[#rc-1]].p) > 0.00001 then
			-- remove colinear
--            lo('?? to_REM:'..k..':'..#rc)
			table.remove(rc, #rc)
		end
--            U.dump(rc, '?? cleaned:'..k)
	end
	local stbase = stamp({1,2,3,4})
--    local stbase = stamp({#aloop*4+1,#aloop*4+2,#aloop*4+3,#aloop*4+4})
	local ast = {}
	for k=2,#aloop+1 do
		ast[#ast+1] = stamp({(k-1)*4+1,(k-1)*4+2,(k-1)*4+3,(k-1)*4+4})
	end
		--U.dump(ast, '??___________________ ast:')

	local astamp = {}
	local apath = {}
	if #aloop == 0 then
		ast = {}
		arc = {{1,2,3,4}}
	else
		for k=#arc,1,-1 do
			local st = stamp({arc[k][1],arc[k][2],arc[k][3],arc[k][4]})
			local ist = index(ast,st)[1]
--                lo('?? for_stamp:'..stamp({arc[k][1],arc[k][2],arc[k][3],arc[k][4]})..':'..tostring(ist), true)
			if not ist then ist = 0 end
			if ist == 1 then
--            if ist == #aloop + 1 then
				-- border rc
				table.remove(arc, k)
			elseif #index(astamp, st) > 0 then
				table.remove(arc, k)
			else
				local rc = arc[k]
				local pth = {}
				for _,p in pairs(rc) do
					pth[#pth+1] = v4e[p].p
				end
				pth[#pth+1] = pth[1]
				if pth and #pth < 20 then
					apath[#apath+1] = pth
					astamp[#astamp+1] = st
				else
					lo('!! WRONG_PATH:')
				end
			end
		end
--[[
		for _,rc in pairs(arc) do
			local pth = {}
			for _,p in pairs(rc) do
				pth[#pth+1] = v4e[p].p
			end
			pth[#pth+1] = pth[1]
			apath[#apath+1] = pth
		end
]]
	end
--        U.dump(v4e, '?? for_RC:'..#arc)
		if istest then U.dump(arc, '?? cleaned:'..#arc..':'..stbase) end
		apath = {}
--        U.dump(arc, '?? cleaned:'..#arc..':'..stbase)

--        if true then return {},{},ae,apath,albl end
--        U.dump(ast, '?? ast:')
--    local u,v = rc[2]-rc[1],rc[4]-rc[1]
--    local an = {u:cross(v):normalized()}
---------------------------------
-- MESH
---------------------------------
	local h = (rc[4]-rc[1]):length()
	local an = {(rc[2]-rc[1]):cross(rc[4]-rc[1]):normalized()}
	local av,auv = {},{}
	local morig,mscale = mrc[1],mrc[2] --{0,0},{1,1}
	for _,n in pairs(v4e) do
		av[#av+1] = n.p
		auv[#auv+1] = {u = (n.p.x-rc[1].x)*mscale[1] + morig[1], v = (n.p.y-rc[1].y)*mscale[2] + morig[2]}
--        auv[#auv+1] = {u = (n.p.x-rc[1].x)*mscale[1] + morig[1], v = (h - (n.p.y-rc[1].y))*mscale[2] + morig[2]}
	end
	-- triangulate
	local af, afhole = {},{}
--            arc = {}
	if not istest then
		for k=#arc,1,-1 do
			local ist = index(ast,stamp({arc[k][1],arc[k][2],arc[k][3],arc[k][4]}))[1]
	--        if not ist then ist = 0 end
	--            lo('?? for_rc:'..k..':'..tostring(ist)..':'..stamp({arc[k][1],arc[k][2],arc[k][3],arc[k][4]}), true)
	--[[
	--            lo('?? for_ist:'..k..':'..ist,true)
			if ist == #aloop + 1 then
				-- border rc
				table.remove(arc, k)
			else
	]]
			if ist then
	--                lo('?? to_hole:'..k,true)
				-- hole mesh
				if #index(apatch,ist) > 0 then
						lo('?? to_patch:'..ist, true)
					afhole[#afhole+1] = {v=arc[k][1]-1, n=0, u=arc[k][1]-1}
					afhole[#afhole+1] = {v=arc[k][4]-1, n=0, u=arc[k][4]-1}
					afhole[#afhole+1] = {v=arc[k][2]-1, n=0, u=arc[k][2]-1}

					if true then
						--- back side
						afhole[#afhole+1] = {v=arc[k][2]-1, n=0, u=arc[k][2]-1}
						afhole[#afhole+1] = {v=arc[k][4]-1, n=0, u=arc[k][4]-1}
						afhole[#afhole+1] = {v=arc[k][3]-1, n=0, u=arc[k][3]-1}
					end
				end
			else
				-- body mesh
				af[#af+1] = {v=arc[k][1]-1, n=0, u=arc[k][1]-1}
				af[#af+1] = {v=arc[k][4]-1, n=0, u=arc[k][4]-1}
				af[#af+1] = {v=arc[k][2]-1, n=0, u=arc[k][2]-1}

				if true then
					--- back side
					af[#af+1] = {v=arc[k][2]-1, n=0, u=arc[k][2]-1}
					af[#af+1] = {v=arc[k][4]-1, n=0, u=arc[k][4]-1}
					af[#af+1] = {v=arc[k][3]-1, n=0, u=arc[k][3]-1}
				end
			end
	--        if stamp({arc[k][1],arc[k][2],arc[k][3],arc[k][4]}) == stbase then
	--            table.remove(arc, k)
	--            break
	--        end
		end
	end
--        U.dump(af, '?? AF:'..#af)

	local mbody = {
		verts = av,
		faces = af,
		normals = an,
		uvs = auv,
		material = 'WarningMaterial',
	}
	local mhole = {
		verts = av,
		faces = afhole,
		normals = an,
		uvs = auv,
--        material = 'm_metal_frame_trim_01',
		material = 'm_transparent_glass_window',
	}

--        U.dump(e4v, '?? e4v:')
--        U.dump(apath, '?? apath:')
--    lo('<< rcPave: av:'..#av..' af:'..#af..' afhole:'..#afhole..'/'..ns, true)

	return mbody,mhole,ae,apath,albl--ae,apath
end
--[[
		if true then return ae,arc end
	local arc = {}
	for k=1,nv-4 do
		--
				U.dump(e4v[v4e[k][3] ], '?? for_v:'..k..' e:'..v4e[k][3])
				local rc = {v4e[k].p}
				-- pick closest crossing
				local ie = v4e[k][3] -- new edge stemming from corner node
				local av = e4v[ie] --
		--            U.dump(av, '?? sorted:')
				local iv = av[U.index(av, k, 'ind')[1]+1] -- index of next node
				-- next edge
				local adir = {} -- set of directions from vertex
				for j,e in ipairs(v4e[iv]) do
					local vlist = e4v[e]
					U.dump(vlist, '?? for_VLIST:'..j..':'..iv..':'..tostring(rc[#rc]))
					local ind = index(vlist, iv, 'ind')[1]
					if ind then
						if ind > 1 then
							adir[#adir+1] = {iv=vlist[ind-1].ind, v = (v4e[vlist[ind-1].ind].p - rc[#rc]):normalized()}
								lo('?? for_p:'..tostring(v4e[vlist[ind-1].ind].p), true)
						end
						if ind < #vlist then
							adir[#adir+1] = {iv=vlist[ind+1].ind, v = (v4e[vlist[ind+1].ind].p - rc[#rc]):normalized()}
						end
					end
					U.dump(adir, '?? for_DIR:'..k..':'..tostring(iv))
					break
		--            for j,av in ipairs(vlist) do
		--                U.dump(,'?? for_vlist:')
		--            end
		--            if e4v[e].
		--            adir[#adir]
				end
					U.dump(rc, '?? for_rc:')
				arc[#arc+1] = rc
						break
			end
		--        U.dump(ae, '<< rcPave:'..#out)
			return ae
]]
--[[
		for i,l in ipairs(av) do
			if l.ind == k then
					U.dump(av[i+1], '?? nxt:'..k)
				rc[#rc+1] = v4e[av[i+1].ind].p
				iv = av[i+1].ind
			end
		end
]]
--[[
					-- step direction
					local v = (v4e[inext].p - v4e[n.ind].p):normalized()
					-- next step
					U.dump(v4e[inext], '?? for_next:'..inext)
					--- build star
					for _,ie in ipairs(v4e[inext]) do
						U.dump(e4v[ie], '?? for_edge:'..ie)
						local edge = e4v[ie]
						step(n.ind, inext)

						if ie == i  then
						else
						end
						if ie ~= i then
							-- look along the edge
--                            for _,
--                            U.dump(ae[ie], '?? for_edge:'..ie)
--                            local v =
						end
					end
]]


local function rcPave0(rc, aloop)
	local out = {}
	local aeref = deepcopy(aloop)
	aeref[#aeref+1] = rc
	-- build array of edges
	local aedge = {}
	local e4v,v4e = {},{}
	for i,pth in pairs(aeref) do
		for j=2,#pth do
			e4v[#e4v+1] = {}
			aedge[#aedge+1] = {pth[j-1],pth[j]}
		end
		-- looping edge
		aedge[#aedge+1] = {pth[#pth],pth[1]}
	end
	local iep = 0
	for i,loop in pairs(aloop) do
		for j,p in pairs(loop) do
			iep = iep + 1
			local dmi,cmi,imi = math.huge
			local ie = 0
			for ir,pth in pairs(aeref) do
--                cmi = nil
				if ir ~= i then
					ie = ie + 1
					local c,d
					local function forCross(v)
						for jr = 2,#pth do
--                                lo('?? for_step: i='..i..' ir='..ir..' j='..j..' jr='..jr..':'..tostring(p)..':'..tostring(U.mod(j-1,loop))..':'..tostring(pth[jr])..':'..tostring(pth[jr-1]))
							c = line2seg(p, p+v, pth[jr], pth[jr-1])
							if c and v:dot(p-c) < 0 then
								d = (c - p):length()
								if d < dmi then
									dmi = d
									cmi = c
									imi = ie
								end
							end
						end
						if ir <= #aloop+1 then
							-- looping segment
							c = line2seg(p, p+v, pth[1], pth[#pth])
--                                lo('?? for_TRY: c='..tostring(c)..' p='..tostring(p)..' v:'..tostring(v))
							if c and v:dot(p-c) < 0 then
								d = (c - p):length()
								if d < dmi then
									dmi = d
									cmi = c
									imi = ie
								end
							end
						end
					end
					forCross((p - U.mod(j-1,loop)):normalized())
					forCross((p - U.mod(j+1,loop)):normalized())
				end
			end
--                lo('??=== for_cmi:'..j..' p:'..tostring(p)..' c:'..tostring(cmi))
			if cmi then
				aeref[#aeref + 1] = {p,cmi}
				aedge[#aedge+1] = aeref[#aeref]

				--TODO: get vertex index is exists
				local iv = #v4e+1

				if not e4v[imi] then e4v[imi] = {} end
				-- link vertex to edge
				e4v[imi][#e4v[imi]+1] = {ind = iv, d = 1}
--                e4v[iep]
				-- link edge to vertex
				v4e[#v4e+1] = {imi,#e4v[imi]}
			end
		end
--        break
	end
	for i,pth in pairs(aeref) do
--            dump(pth, '?? for_pth:'..i..'/'..#aloop)
		for j=2,#pth do
			out[#out+1] = {pth[j-1],pth[j]}
		end
		if i <= #aloop+1 then
			out[#out+1] = {pth[#pth],pth[1]}
		end
	end
		lo('<< rcPave:'..#aedge)
	return out
end


local function segNear(p, v, ae, push)
	lo('>> segNear:'..tostring(p)..':'..tostring(v))
	local dmi,imi,cmi = math.huge
	for i,e in pairs(ae) do
	--            lo('?? for_edge:'..i..':'..tostring(e[1])..':'..tostring(e[2]))
		if (p-e[1]):length() > 0 and (p-e[2]):length() > 0 then
			local c = line2seg(p, p+v, e[1], e[2])
			if c then
				local d = (c - p):length()
				if d < dmi then
					dmi = d
					imi = i
					cmi = c
				end
			end
		end
	end
	if push and imi then
		ae[#ae + 1] = {p,cmi}
	end
		dump(ae[imi], '?? segNear:'..tostring(push)..':'..tostring(p)..'>'..tostring(cmi)..':'..#ae)
end


-- s - distance of p from c
local function segCross(a,b,c,d)
	local p,s = line2seg(a,b,c,d)
	if p and line2seg(c,d,a,b) then
		return p,s
	end
end
U.segCross = segCross


local function rcPave__(rc, aloop)
	local ae = {}
	for i,loop in pairs(aloop) do
		for j = 1,#loop-1 do
			ae[#ae+1] = {loop[j], loop[j+1]}
		end
		-- last
		ae[#ae+1] = {loop[#loop],loop[1]}
	end
		lo('?? for_ae:'..#ae)

	local aeref = U.clone(ae)
	table.insert(aeref, {rc[1],rc[2]})
	table.insert(aeref, {rc[2],rc[3]})
	table.insert(aeref, {rc[3],rc[4]})
	table.insert(aeref, {rc[4],rc[1]})

	for i,loop in pairs(aloop) do
			dump(loop, '?? for_loop:'..i..'/'..#loop)
		for j,p in pairs(loop) do
--                lo('?? for_p:'..j..':'..tostring(p))
--                lo('?? for_p:'..j..':'..tostring(p)..':'..tostring(U.mod(j-1,loop)))

			segNear(p, (p-U.mod(j-1,loop)):normalized(), aeref, (i == 1) and true or false)
--            segNear(p, (p-U.mod(j-1,loop)):normalized(), aeref, (i == 1 and j == 1) and true or false)
--                return aeref
		end
	end

	return aeref
end

local function line2seg_(a, b, c, d)
	local x,y = lineCross({x=a[1],y=a[2]}, {x=b[1],y=b[2]}, {x=c[1],y=c[2]}, {x=d[1],y=d[2]})
	local p = vec3(x,y,0)
	if (vec3(c[1],c[2],0)-p):dot(vec3(d[1],d[2],0)-p) <= 0 then
		return p
	end
end


local function segNear_(p, ae, axis, dir)
--        U.dump(ae, '?? AE:')
	local fr,to = 1,#ae
	if dir > 0 then
		fr,to = #ae,1
	end
	local dmi,imi = math.huge
	for i = fr,to,-dir do
--    for i,e in pairs(ae) do
		if (ae[i][1][axis] - p[axis])*dir <= 0 then
			break
		end
		-- distance to segment
		local c = line2seg(
			p, {p[1]+(axis==1 and 1 or 0), p[2]+(axis==2 and 1 or 0)},
			ae[i][1], ae[i][2])
		if c then
			local r = c - vec3(p[1],p[2],0)
			local d = r:length()
			if d < dmi then
				dmi = d
				imi = i
			end
		end
	end
		U.dump(ae[imi], '?? segNear:'..p[1]..':'..p[2])
end


local function rcPave_(rc, ahole)
	local aex,aey = {},{} --clone(ahole)
	for i,h in pairs(ahole) do
--        aex[#aex+1] = {h[1],{h[1].x,h[2].y},i,1}
--        aex[#aex+1] = {{h[2].x,h[1].y},h[2],i,2}

--        aey[#aey+1] = {h[1],{h[2][1],h[1][2]},i,1}
--        aey[#aey+1] = {{h[1][1],h[2][2]},h[2],i,2}

		aex[#aex+1] = {h[1],{h[1][1],h[2][2]},i,1}
		aex[#aex+1] = {{h[2][1],h[1][2]},h[2],i,2}

		aey[#aey+1] = {h[1],{h[2][1],h[1][2]},i,1}
		aey[#aey+1] = {{h[1][1],h[2][2]},h[2],i,2}
	end
	table.sort(aex, function(a, b)
		return a[1][1] < b[1][1]
	end)
	table.sort(aey, function(a, b)
		return a[1][2] < b[1][2]
	end)
		dump(aex, '?? AEX:')
		dump(aey, '?? AEY:')
	local ax = U.clone(aex)
	table.insert(ax, 1, {rc[1], {rc[1][1],rc[2][2]}})
	ax[#ax+1] = {{rc[2][1],rc[1][2]}, rc[2]}

	local ay = U.clone(aey)
	table.insert(ay, 1, {rc[1], {rc[2][1],rc[1][2]}})
	ay[#ay+1] = {{rc[1][1],rc[2][2]}, rc[2]}

	for i,e in pairs(aex) do
			U.dump(e, '??======================== for_E_VERT:'..i)
		if e[4] == 1 then
			-- look left
			segNear(e[1], ax, 1, -1)
			-- look down
			segNear(e[1], ay, 2, -1)
			-- look left
			segNear(e[2], ax, 1, -1)
			-- look up
			segNear(e[2], ay, 2, 1)
		end
			if i == 2 then return end
		if e[4] == 2 then
			-- look right
			segNear(e[1], ax, 1, 1)
			-- look down
			segNear(e[1], ay, 2, -1)
			-- look right
			segNear(e[2], ax, 1, 1)
			-- look up
			segNear(e[2], ay, 2, 1)
		end
	end
end

--[[
	if false then
		table.sort(ae, function(a, b)
			return a[1].x < b[1].x
		end)
		local aex = U.clone(ae)
		table.insert(aex, {rc[1],rc[3]})
		table.insert(aex, {rc[1],rc[2]})
		table.insert(aex, {rc[3],rc[2]})
		table.insert(#aex+1, {rc[2],rc[3]})

		table.sort(ae, function(a, b)
			return a[1].y < b[1].y
		end)
		local aey = U.clone(ae)
		table.insert(aey, {rc[1],rc[3]})
	end
]]


local function polyCenter(base)
	if not base then return end
	local c = vec3(0,0,0)
	local s = 0
	for j = 1,#base do
		local ds = (U.mod(j+1,base) - U.mod(j,base)):length() + (U.mod(j,base) - U.mod(j-1,base)):length()
		c = c + base[j]*ds
		s = s + ds
	end
	c = c/s
	return c
end


U.polyStraighten = function(base, dbg)
--    base = U.clone(base)
	base = deepcopy(base)
	local map = {}
	local mbase = {}
	for j = #base,1,-1 do
		local todrop = false
		local ang = U.vang(U.mod(j+1,base) - base[j],base[j] - U.mod(j-1,base))
		if (U.mod(j-1,base) - base[j]):length() < 0.0001 then
--            table.remove(base,j)
			todrop = true
		elseif ang < small_ang or math.abs(ang - math.pi) < small_ang then
--            table.remove(base,j)
			todrop = true
		end
--            lo('?? if_DROP:'..j..':'..tostring(todrop))
		if not todrop then
			table.insert(mbase, 1, base[j])
			table.insert(map, 1, j)
--            mbase[#mbase+1] = base[j]
--            map[#map+1] = j
		else
			table.remove(base, j)
		end
	end
--[[
	for j = #base,1,-1 do
			if dbg then
				lo('?? PS:'..j..':'..tostring(base[j])..':'..tostring(U.mod(j+1,base)))
			end
		local todrop = false
		if (U.mod(j+1,base) - base[j]):length() < 0.0001 then
			table.remove(base,j)
			todrop = true
		end
		local ang = U.vang(U.mod(j+1,base) - base[j],base[j] - U.mod(j-1,base))
		if ang < small_ang or math.abs(ang - math.pi) < small_ang then
			table.remove(base,j)
			todrop = true
		end
		if not todrop then
			table.insert(map, 1, j)
		end
	end
]]
	return mbase,map
end
--[[
		if U.vang(U.mod(j+1,base) - base[j],base[j] - U.mod(j-1,base)) < small_ang then
			table.remove(base,j)
		else
			table.insert(map, 1, j)
		end
]]


--local function forCurv(ap, i)
--	if i < 2 or i > #list-1 then return end
--	return U.vang(ap[i]-ap[i-1])
--end


local function forStar(astem, ard)
--    U.dump(astem, '>> forStar:')
--    if not ard then ard = adec end
--        out.avedit = {}
	local aflag = {}
	for _,c in pairs(astem) do
		local lst = ard[c[1]].list
		local nxt
		if c[2] > 1 then
			local nxt = lst[c[2] - 1]
			local v = nxt - lst[c[2]]
			aflag[#aflag+1] = {rdi = c[1], fr = c[2], ndi = c[2] - 1, ang = math.atan2(v.y, v.x) % (2*math.pi), v = (nxt - lst[c[2]]):normalized(), next = nxt}
--                    out.avedit[#out.avedit + 1] = nxt
--                    out.avedit[#out.avedit].z = core_terrain.getTerrainHeight(nxt)
		end
		if c[2] < #lst then
			local nxt = lst[c[2] + 1]
			local v = nxt - lst[c[2]]
			aflag[#aflag+1] = {rdi = c[1], fr = c[2], ndi = c[2] + 1, ang = math.atan2(v.y, v.x) % (2*math.pi), v = (nxt - lst[c[2]]):normalized(), next = nxt}
--                    out.avedit[#out.avedit + 1] = nxt
--                    out.avedit[#out.avedit].z = core_terrain.getTerrainHeight(nxt)
		end
	end
	local function comp(a, b)
		return a.ang < b.ang
	end
	table.sort(aflag, comp)
--    aang[#aang + 1] = {i, math.atan2(v.y, v.x) % (2*math.pi)}
--        U.dump(aflag, '<< forStar:')
	return aflag
end

local function det2(m)
	return m[1][1]*m[2][2] - m[1][2]*m[2][1]
end


local function det3(m)
	return
	m[1][1]*det2({{m[2][2],m[2][3]},
				  {m[3][2],m[3][3]}}) -
	m[1][2]*det2({{m[2][1],m[2][3]},
				  {m[3][1],m[3][3]}}) +
	m[1][3]*det2({{m[2][1],m[2][2]},
				  {m[3][1],m[3][2]}})
end


-- returns: list of parabola coeffs (a,b,c) passing through each triple of points from ap
local function paraSpline(ap)
--        U.dump(ap, '>> paraSpline:'..#ap)
	local apar = {}
	for i = 2,#ap-1 do
		local p1 = {x = ap[i-1][1], y = ap[i-1][2]}
		local p2 = {x = ap[i][1], y = ap[i][2]}
		local p3 = {x = ap[i+1][1], y = ap[i+1][2]}
		local d = {{p1.x^2, p1.x, 1},
				   {p2.x^2, p2.x, 1},
				   {p3.x^2, p3.x, 1},
				}
		local m1 = {{p1.y, p1.x, 1},
					{p2.y, p2.x, 1},
					{p3.y, p3.x, 1},
				}
		local m2 = {{p1.x^2, p1.y, 1},
					{p2.x^2, p2.y, 1},
					{p3.x^2, p3.y, 1},
				}
		local m3 = {{p1.x^2, p1.x, p1.y},
					{p2.x^2, p2.x, p2.y},
					{p3.x^2, p3.x, p3.y},
				}
		apar[#apar+1] = {det3(m1)/det3(d), det3(m2)/det3(d), det3(m3)/det3(d)}
	end
	return apar
end


local function spline2(a, b, i, n)
	if not n then n = 1 end
	local c = 2*(b - a)/(n * n)
	if i < n/2 then
	  return a + c*i*i
	else
	  return b - c*(i - n)*(i - n)
	end
end


local function vang(u, v, signed, dbg)
--    if signed then
--        return -math.atan2(-v[2],v[1]) + math.atan2(-u[2],u[1])
--    end
--        lo('?? vang:'..(u:normalized()-v:normalized()):length())
	if not u or u:length()==0 or not v or v:length()==0 then return 0 end
	if signed then
		local crs = u:normalized():cross(v:normalized()).z
--            lo('>> vang:'..tostring(signed)..':'..(u:normalized()-v:normalized()):length())
		if (u:normalized()-v:normalized()):length() < 0.00000001 then return 0 end
	--    math.acos(u:normalized():dot(v:normalized()))
		if math.abs(crs) < 0.0000001 then
      return u:dot(v) < 0 and math.pi or 0
    end
		return (crs > 0 and 1 or -1)*math.acos(u:dot(v)/u:length()/v:length())
	end
--        lo('?? vang:'..tostring(u)..':'..tostring(v)..':'..tostring(math.acos(u:dot(v)/u:length()/v:length())))
	local cs = u:dot(v)/u:length()/v:length()
--    if dbg then lo('?? vang:'..tostring(cs)..':'..tostring(cs+1)..':'..tostring(cs+1==0)) end
--`        lo('?? vang:'..tostring(cs)..':'..tostring(cs==-1))
--        if dbg then lo('?? vang:'..tostring(u)..':'..tostring(v)..':'..tostring(cs)..':'..tostring(math.acos(cs))) end
	if math.abs(cs+1)<0.0000001 then
--        lo('?? vang:'..tostring(cs)..':'..tostring(cs==-1))
		return math.pi
	elseif math.abs(cs-1)<0.0000001 then
		return 0
	end
	return math.acos(cs)
end


local function perp(v)
	return vec3(-v.y, v.x)
end


local function vturn(v, ang)
	return vec3(math.cos(ang)*v.x - math.sin(ang)*v.y, math.sin(ang)*v.x + math.cos(ang)*v.y, v.z)
end


local function inRC(p, list, prop, strict, dbg)
	local d1, d2
	local miss = 0.1
	if not list then return end
	for i=1,#list do
--    for i,rc in pairs(list) do
		local rc = prop == nil and list[i] or list[i][prop]
		if not rc or #rc<2 then return nil,d1,d2 end
		local vn = -(rc[2]-rc[1]):cross(rc[#rc]-rc[1])
--        lo('?? inRC_vn:'..tostring(vn))
--		local vn = (rc[3]-rc[2]):cross(rc[2]-rc[1])
		local isin = true
		for j=1,#rc do
          if dbg then
            lo('?? inRC_vv:'..j..':'..tostring(rc[j])..' p:'..tostring(p)..' a:'..tostring(p-rc[j])..' b:'..tostring(mod(j+1,rc)-rc[j]))
          end
		  	if strict then
				if (p-rc[j]):cross(mod(j+1,rc)-rc[j]):dot(vn) <= 0 then
	--            if (p-rc[j]):cross(mod(j+1,rc)-rc[j]).z > 0 then
					isin = false
					break
				end
			else
				if (p-rc[j]):cross(mod(j+1,rc)-rc[j]):dot(vn) < 0 then
	--            if (p-rc[j]):cross(mod(j+1,rc)-rc[j]).z > 0 then
					isin = false
					break
				end
			end
--            lo('?? ifcross:'..j..':'..tostring())
		end
		if isin then return i end
--[[
		if rc and #rc > 3 then
			d1 = p:distanceToLine(rc[1],rc[2]) + p:distanceToLine(rc[3],rc[4]) - rc[1]:distanceToLine(rc[3],rc[4])
	--                dump(rc, '?? for_rc1:'..i..' d:'..d1..':'..tostring(p:distanceToLine(rc[1],rc[2]))..':'..tostring(p:distanceToLine(rc[3],rc[4]))..':'..tostring(rc[1]:distanceToLine(rc[3],rc[4])))
			if p:distanceToLine(rc[1],rc[2]) + p:distanceToLine(rc[3],rc[4]) < rc[1]:distanceToLine(rc[3],rc[4]) + miss then
				d2 = p:distanceToLine(rc[2],rc[3]) + p:distanceToLine(rc[4],rc[1]) - rc[1]:distanceToLine(rc[2],rc[3])
	--                dump(rc, '?? for_rc2:'..i..' d:'..d2..':'..tostring(p:distanceToLine(rc[2],rc[3]))..':'..tostring(p:distanceToLine(rc[4],rc[1]))..':'..tostring(rc[1]:distanceToLine(rc[2],rc[3])))
				if p:distanceToLine(rc[2],rc[3]) + p:distanceToLine(rc[4],rc[1]) < rc[1]:distanceToLine(rc[2],rc[3]) + miss then
					return i
				end
			end
		end
]]
	end
	return nil, d1, d2
end


local function proj2D(v)
	return vec3(v.x, v.y, 0)
end


-- distance from p to line
local function toLine(p, line)
	if not p or not line then return end
	local v = p - line[1]
	local u = (line[2] - line[1]):normalized()

	return math.sqrt(v:length()^2 - v:dot(u)^2), p+u*v:dot(u)-v
end


local function less(a, b, v)
--    if (a - b):dot(v) == 0 then return 0 end
	return (a - b):dot(v)
end


U.ray2plane = function(ray, p, norm)
	local d = intersectsRay_Plane(ray.pos, ray.dir, p, norm)
	if d and not isnan(d) then
		return ray.pos + ray.dir:normalized()*d
	end
end


local function onPlane(rayCast, p, norm)
	norm = norm or vec3(0,0,1)
	local dirhit = (rayCast.pos - core_camera.getPosition()):normalized()
	local d = intersectsRay_Plane(core_camera.getPosition(), dirhit, p, norm)

	return core_camera.getPosition() + dirhit*d
end


local function polyMargin(base, margin, askip)
	askip = askip == nil and {} or askip
	if margin == nil or not base or #base < 2 then return base end

	local up = -(base[2]-base[1]):cross(base[1]-base[(-1) % #base + 1]):normalized()
	local mbase = {}
	for i = 1,#base do
		local b = base[i]
		local bpre = base[(i - 2) % #base + 1]
		local bpost = base[i % #base + 1]
		local v1 = (b - bpre):normalized()
		local v2 = (b - bpost):normalized()
		if v2:length() == 0 then
			v2 = -v1:cross(vec3(0, 0, 1))
--            lo('?? NEW_V2:'..tostring(v2))
		end
		if v1:length() == 0 then
			v1 = -v2:cross(vec3(0, 0, 1))
--            lo('?? NEW_V1:'..tostring(v1))
		end
--                if (b - bpost):length() == 0 then
--                    lo('??_____ polyMargin:'..tostring((b - bpost):normalized()))
--                end
		local dp
		if v2:cross(v1):length() < small_val then
--                lo('??*********************************** polyMargin:'..i..':'..tostring(v1))
			dp = v1:cross(vec3(0,0,1))*margin
		else
			dp = (v1 + v2):normalized() * margin/math.sin(U.vang(v1,v2)/2) --math.pi/4)
			if v2:cross(v1):dot(up) < 0 then
				dp = -dp
			end
		end

		if #U.index(askip, i) > 0 then
			if #U.index(askip, (i - 2) % #base + 1) == 0 then
				dp = v2:normalized() * margin
			else
				dp = vec3(0, 0, 0)
			end
--                lo('?? PM_skip1:'..i)
		elseif #U.index(askip, (i - 2) % #base + 1) > 0 then
			dp = v1:normalized() * margin
--                lo('?? PM_skip2:'..((i - 2) % #base + 1))
		end

		mbase[#mbase + 1] = base[i] + dp
		if #mbase > 1 and (mbase[#mbase] - mbase[#mbase - 1]):length() < 0.0001 then
			mbase[#mbase - 1] = vec3(mbase[#mbase].x, mbase[#mbase].y, mbase[#mbase].z)
--            lo('??********** polyMargin_close:'..i..':'..(mbase[#mbase] - mbase[#mbase - 1]):length())
		end
	end
--            U.dump(mbase, '<< polyMargin:')
	return mbase
end

U.small_ang = small_ang
U.small_dist = small_dist
U.small_val = small_val
U.lo = lo

U.clone = clone
U.camSet = camSet
U.dump = dump
U.fileCopy = fileCopy
U.forBackup = forBackup
U.forOrig = forOrig
U.fromJSON = fromJSON
U.debugPoints = debugPoints
U.boxMark = boxMark
U.markUp = markUp

U.mod = mod
U.index = index
U.proj2D = proj2D
U.chop = chop
U.split = split
U.stamp = stamp
U.push = push
U.rand = rand
U.perm = perm

U.forStar = forStar
U.inRC = inRC
U.vang = vang
U.less = less
U.lineCross = lineCross
U.line2seg = line2seg
U.perp = perp
U.vturn = vturn
U.paraSpline = paraSpline
U.polyCenter = polyCenter
U.polyMargin = polyMargin
U.rcPave = rcPave
U.rcWH = rcWH
U.spline2 = spline2
U.toLine = toLine
U.onPlane = onPlane

U._mode = 'GEN'
--U.out = {}

return U