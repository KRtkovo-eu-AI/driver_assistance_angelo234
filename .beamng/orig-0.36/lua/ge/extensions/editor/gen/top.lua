local Top = {}

local U = require('/lua/ge/extensions/editor/gen/utils')
local M = require('/lua/ge/extensions/editor/gen/mesh')

local W

local small_ang = U.small_ang
local lo = U.lo


local function inject(w)
	W = w
end


local function chop(base)
--        U.dump(base, '>> cut:'..tostring(#base), true)
	for i = 1,#base do
		local u = base[i] - U.mod(i-1,base)
		local v = U.mod(i+1,base) - base[i]
--                lo('?? for_ang:'..i..':'..tostring(u)..':'..tostring(v), true)
		if u:cross(v).z < 0 then
--                lo('?? conq:'..i)
			for k = 2,#base-2 do
				local u1,u2 = base[i],U.mod(i+1,base)
				local v1,v2 = U.mod(i+k,base),U.mod(i+k+1,base)
				local s = closestLinePoints(v1, v2, u1, u2)
--                    lo('?? if_cross:'..k..'>'..s..'<'..U.mod(i+k,#base)..':'..U.mod(i+k+1,#base)..':'..i..':'..U.mod(i+1,#base))
				if 0 <= s and s < 1 then
					local dir = (v1 + s*(v2-v1) - u1):dot(u2-u1)
--                        lo('?? for_cross:'..i..'>'..k..':'..dir)
					if dir < 0 then
--                            lo('?? is_cross:'..i..'>'..U.mod(i+k,#base)..':'..k..':'..(i+k))
						local chunk = {U.mod(i+k,#base)}
						local rest = {i}
						for j = 1,#base do
							local ci = U.mod(i+k+j,#base)
--                                lo('?? for_chunk:'..j..':'..ci..'/'..i)
							chunk[#chunk+1] = ci
							if ci == i then
--                                U.dump(chunk, '?? chunk:')
								break
							end
						end
						for j = 1,#base do
							local ci = U.mod(i+j,#base)
--                                lo('?? for_rest:'..j..':'..ci)
							rest[#rest+1] = ci
							if ci == U.mod(i+k,#base) then
--                                U.dump(rest, '?? rest:')
								return chunk,rest
							end
						end
						break
					end
				end
			end
--            lo('!! ERR_cut_NOINTERSECT:')
		end
	end
	return base
end


local function forChunks(base)
--        U.dump(base,'>> forChunks:'..#base)
	local achunk = {}
	local rest,chunk = {}
	local cbase = base
	local dbase, dchunk = {}
	for i=1,#base do
		dbase[#dbase+1] = i
	end
	local dbasepre = dbase

	local n = 0
	while rest and n < 10 do
		n = n + 1
--            U.dump(cbase, '?? for_N:'..n)
		chunk,rest = chop(cbase)
--            U.dump(chunk, '?? for_N_chunk:'..n)
		dbase = {}
		cbase = {}
		dchunk = {}
		if rest then
			for _,i in pairs(rest) do
				dbase[#dbase+1] = dbasepre[i]
				cbase[#cbase+1] = base[dbasepre[i]]
			end
			for _,i in pairs(chunk) do
				dchunk[#dchunk+1] = dbasepre[i]
			end
			dbasepre = dbase
--                U.dump(cbase, '?? cBASE:')
--                U.dump(dbasepre, '?? dBASE:')
		else
--            U.dump(dbasepre, '?? last:')
			for i=1,#chunk do
				dchunk[#dchunk+1] = dbasepre[i]
			end
		end
		achunk[#achunk+1] = dchunk

--        achunk[#achunk+1] = chunk
--        break
	end
--        table.remove(achunk, 2)
--        chunk = table.remove(achunk, 3)
--        table.insert(achunk, 1, chunk)
--        U.dump(achunk,'<< forChunks:'..#achunk)
	return achunk
end


local function pave(base, uvscale, uvini)
	-- filter splits
	base = U.clone(base)
	for j = #base,1,-1 do
		if U.vang(U.mod(j+1,base) - base[j],base[j] - U.mod(j-1,base)) < small_ang then
			table.remove(base,j)
		end
	end
	local achunk = forChunks(base)
--        U.dump(achunk, '??___________________ pave:')
	local w = (base[2]-base[1]):cross(base[#base]-base[1]):normalized()
	local av,auv={},{}
	local u = (base[2]-base[1]):normalized()
--    local v = vec3(-u.y, u.x, 0)
	local v = -u:cross(w)
	local ref = base[1]
	for _,b in ipairs(base) do
		av[#av+1] = b
--        auv[#auv+1] = {u=(b-ref):dot(u), v=(b-ref):dot(v)}
	end
--        U.dump(av, '?? pave_BASE:')
	auv = M.forUV(av, 1, uvini, (av[2]-av[1]):cross(av[3]-av[2]), uvscale or {1,1}) --, uvini, w, scale, auv)
	local af = {}
	for _,c in pairs(achunk) do
		af = M.zip(c, af)
	end
--        U.dump(achunk, '<< pave:'..#av..':'..#af, true)
--        U.dump(af, '<< pave:'..#av..':'..#af)
	return av,auv,af,{w}
end


local function pairsUp(base,ishift)
--        U.dump(base, '?? pairsUp:')
	ishift = ishift or 0
	local ainpair = {}
	local apair = {}
	local amult = {}
--    local ishift = 0

	for i =1,#base do
		local a,b = U.mod(i+1,base)-U.mod(i,base),U.mod(i+2,base)-U.mod(i+1,base)
		local ang = (U.mod(i+1,base)-U.mod(i,base)):cross(U.mod(i+2,base)-U.mod(i+1,base)).z
		if ang < 0 then
			ishift = i-1
			break
		end
--            lo('?? for_ang:'..i..':'..tostring(ang)..':'..tostring(a)..':'..tostring(b))
	end
--    if not ishift then return end
		W.out.avedit = {base[1]}
	for i = 1,#base-2 do
		local cflat = {}
		local dmi,imi = math.huge
		local c = (U.mod(i+ishift,base)+U.mod(i+ishift+1,base))/2
--            lo('?? pU_i:'..U.mod(i+ishift,#base)..' ishift:'..ishift)
		-- find nearest center
		for j = i+ishift+2,i+ishift+(#base-2) do
			local d = (c - (U.mod(j,base)+U.mod(j+1,base))/2):length()
			local ang = math.abs(U.vang(U.mod(i+ishift+1,base)-U.mod(i+ishift,base),U.mod(j+1,base)-U.mod(j,base))-math.pi)
--                local u,v = U.mod(i+1,base)-U.mod(i,base), U.mod(j+1,base)-U.mod(j,base)
--                if j == 8 then
--                    lo('?? pU_j:'..U.mod(j,#base)..' d:'..d..'/'..tostring(dmi)..' ang:'..tostring(ang))
--                end
			if d < dmi and ang < 10*small_ang then
--                    lo('?? pU_j:'..U.mod(j,#base)..' d:'..d..'/'..tostring(dmi)..' ang:'..tostring(ang)) --..':'..tostring(u)..':'..tostring(v)..':'..tostring(U.vang(u,v))..':'..tostring(u:dot(v)/u:length()/v:length())..':'..tostring(math.acos(u:dot(v)/u:length()/v:length())))
				local cmi = {U.mod(i+ishift,#base),U.mod(j,#base)}
				local fits = true
				local v1,v2 = U.mod(cmi[1]+1,base) - U.mod(cmi[1],base), (U.mod(cmi[2],base)+U.mod(cmi[2]+1,base))/2 - U.mod(cmi[1],base)
				local z = v1:cross(v2).z
--                    U.dump(cmi,'?? to_check:'..z..':'..tostring(v1)..':'..tostring(v2))
				if z < 0 then
					fits = false
				end
				-- check winding/cross-bounds
				for _,p in pairs(apair) do
					local ps = deepcopy(p)
					table.sort(ps)
					local imis = deepcopy(cmi)
					table.sort(imis)
					if not (ps[1] > imis[2] or imis[1] > ps[2] or (ps[1]<=imis[1] and imis[2]<=ps[2]) or (imis[1]<=ps[1] and ps[2]<=imis[2])) then
						fits = false
--                            lo('?? no_FITS:'..U.mod(i+ishift,#base)..':'..U.mod(j,#base))
						break
					end
				end
				if fits then
					dmi = d
					imi = cmi
				end
			end
		end
--            lo('?? pairsUp:'..i..' imi:'..tostring(imi and imi[1] or nil)..':'..tostring(imi and imi[2] or nil))
		if imi then

			if (#U.index(ainpair,imi[1])==0 or #U.index(ainpair,imi[2])==0) then
				if not amult[imi[1]] then amult[imi[1]] = {imi[2]} end
				if not amult[imi[2]] then amult[imi[2]] = {imi[1]} end
				if #U.index(ainpair,imi[1]) > 0 then
					amult[imi[1]][#amult[imi[1]]+1] = imi[2]
				end
				if #U.index(ainpair,imi[2]) > 0 then
					amult[imi[2]][#amult[imi[2]]+1] = imi[1]
				end

				apair[#apair+1] = imi
--                            lo('?? for_pair:'..imi[1]..':'..imi[2])
				ainpair[#ainpair+1] = imi[1]
				ainpair[#ainpair+1] = imi[2]
			end
--                U.dump(imi, '??+++++++++++ for_IMI:'..#U.index(ainpair,imi[1])..':'..#U.index(ainpair,imi[2]))
--        if imi then -- and #U.index(ainpair,imi[1])==0 then -- and #U.index(ainpair,imi[2])==0 then
--            if math.abs(U.vang(U.mod(i+1,base)-U.mod(i,base),U.mod(imi[2]+1,base)-U.mod(imi[2],base))-math.pi) < 10*small_ang then
--            end
		end
--            lo('?? to_check:'..imi[1]..':'..imi[2])
	end
--        U.dump(apair, '?? pairsUp_pairs:')
--        U.dump(amult, '?? pairsUp_muilt:')
	return apair,amult
end


-- gable: stick thin ab 4gone to thick bc
local function stick(a, b, c, rw) -- rw - branch width ratio
--    lo('>> stick:'..rw)
	local av,aV = {},{} -- {a[1],(a[1]+a[2])/2,a[2]},{}
	-- middle
	local abMid = {(a[1]+a[2])/2,(b[1]+b[2])/2}
	local bcMid = {(b[1]+b[2])/2,(c[1]+c[2])/2}
	-- cross of extended thick right edge with thin lift
	local x,y = U.lineCross(a[2],b[2],b[1],c[1])
	local vcross = vec3(x,y)
--        W.out.avedit = {vcross}
--        lo('?? for_cross:'..tostring(x)..':'..tostring(y))
	-- cross middle thin with thick plate
	x,y = U.lineCross(abMid[1],abMid[2],vcross,c[1])
	local xm,ym = U.lineCross(abMid[1],abMid[2],bcMid[1],bcMid[2])
--            W.out.avedit = {vec3(xm,ym)} --{vec3(x,y)} -- {vcross,c[1]} -- {abMid[1],abMid[2]} --vec3(xm,ym)}
	local dir = vec3(xm,ym)-vec3(x,y)
--            W.out.avedit = {vec3(x,y),vec3(xm,ym),vec3(x,y)+dir*rw}
	local av = {b[1],vec3(x,y)+dir*rw,vcross}
--            W.out.avedit = av
--    local av = {b[1],vec3(x,y)+dir:normalized()*rw,vcross}
	local aV = {vcross,(b[2]+vcross)/2,b[2]}
--            W.out.avedit = aV

	return av,aV
end


local function forGable(base, apair, dbg)
		if dbg then
			U.dump(base, '?? forGable:'..tostring(apair))
		end
	if not apair then
--            U.dump(base,'?? forGable:')
		base = U.polyStraighten(base)
		local pair,mult = pairsUp(base)
		for _,m in pairs(mult) do
			if #m > 1 then
--                local ang = U.vang(U.mod(4+1,base) - base[4],base[4] - U.mod(4-1,base),false,true)
				lo('!! ERR_forGable_MULT:'..tostring(apair)..':'..#base..':'..#U.polyStraighten(base)) --..':'..ang, true)
--                U.dump(base)
--                U.dump(mult,'!! ERR_forGable_MULT:')
--                U.dump(pair,'!! ERR_forGable_PAIR:')
--                U.dump(base,'!! ERR_forGable_BASE:')
				return {}
			end
		end
		apair = pair
	end
	if not apair then return end
	local opair = {apair[1]}
	for i=2,#apair do
		local cp = apair[i]
		if opair[1][1]-1 == U.mod(apair[i][1],#base) then
			table.insert(opair, 1, apair[i])
		elseif opair[1][1]-1 == U.mod(apair[i][2],#base) then
			table.insert(opair, 1, {apair[i][2],apair[i][1]})
		end
		if opair[#opair][1]+1 == U.mod(apair[i][1],#base) then
			opair[#opair+1] = apair[i]
		elseif opair[#opair][1]+1 == U.mod(apair[i][2],#base) then
			opair[#opair+1] ={apair[i][2],apair[i][1]}
--            table.insert(opair, 1, apair[i])
		end
	end
--        U.dump(opair, '?? for_OPAIR:')
	apair = opair
--[[
	for i=1,#apair do
		table.sort(apair[i])
	end
	table.sort(apair, function(a,b)
		return a[1] < b[1]
	end)
]]
--        U.dump(apair, '>> forGable:'..#base..':'..tostring(apair))

--    local an,auv,af = {},{},{}

	local function meshClose(pair, ap, aroof)
--            U.dump(pair, '?? meshClose:'..tostring(ap and #ap or nil))
		if not ap then return end
--            lo('?? for_p:'..tostring(pair[1]))
		ap[#ap+1] = U.mod(pair[1]+1,base)
		ap[#ap+1] = (U.mod(pair[1]+1,base)+U.mod(pair[2],base))/2
		ap[#ap+1] = U.mod(pair[2],base)

		aroof = aroof or {}
--        local aroof ={}
		for i=1,#ap,6 do
			local an,auv,af = {},{},{}
			local base = {ap[i],ap[i+1],ap[i+2],ap[i+3],ap[i+4],ap[i+5]}

--                U.dump(base, '?? to_CLOSE:'..#base)
			an,auv,af = M.zip2(base,{1,2,5,4})
			aroof[#aroof+1] = {av=base,an=an,auv=auv,af=af} --{base,an,auv,af}

			an,auv,af = M.zip2(base,{6,5,2,3})
			aroof[#aroof+1] = {av=base,an=an,auv=auv,af=af}
		end
		return aroof
	end

	local adata = {}
	local av

	if #apair == 1 then
		av = {
			base[apair[1][1]],
			(base[apair[1][1]] + U.mod(apair[1][2]+1,base))/2,
			U.mod(apair[1][2]+1,base)
		}
	else

		for i=1,#apair-1 do
			-- pick ajecent
			if apair[i][1]+1 == apair[i+1][1] then
				local a = {base[apair[i][1]], U.mod(apair[i][2]+1,base)}
				local b = {base[apair[i+1][1]], U.mod(apair[i][2],base)}
				local c = {U.mod(apair[i+1][1]+1,base), U.mod(apair[i+1][2],base)}
				if not av  then
					av = {a[1],(a[1]+a[2])/2,a[2]}
				end
				--- pairs width ratio
				local d = U.toLine(a[2], {a[1],b[1]})
				local D = U.toLine(b[2], {b[1],c[1]})
				local winv,ainv
	--                lo('?? a1:'..tostring(c[1])..' b:'..tostring(b[1])..' c:'..tostring(a[1]))
	--                lo('?? for_ang:'..ang..':'..U.vang(a[1]-b[1], b[1]-c[1], true))
				if d/D - 1 > U.small_val then
--                if D<d then
					winv = true
					local sa = a
					a = c
					c = sa
					a = {a[2],a[1]}
					b = {b[2],b[1]}
					c = {c[2],c[1]}
	--                    lo('?? a2:'..tostring(a[1])..' b:'..tostring(b[1])..' c:'..tostring(c[1]))
	--                    lo('?? new_ANG:'..U.vang(b[1]-a[1], c[1]-b[1], true))
				end
				local ang = U.vang(b[1]-a[1], c[1]-b[1], true)
				if ang > small_ang then
					ainv = true
					a = {a[2],a[1]}
					b = {b[2],b[1]}
					c = {c[2],c[1]}
				end
--                    lo('?? inv: w:'..i..':'..tostring(winv)..' a:'..tostring(ainv)..':'..(d/D)..':'..d..':'..D..':'..tostring(a[2])..':'..tostring(a[1])..':'..tostring(b[1]))
	--                lo('?? a:'..tostring(a[1])..' b:'..tostring(b[1])..' c:'..tostring(c[1]))
	--                lo('?? for_turn:'..U.vang(b[1]-a[1], c[1]-b[1], true))
	--                W.out.avedit = c
	--                U.dump(a, '?? for_a:')
				local ap,aP = stick(a,b,c, winv and D/d or d/D)
	--                lo('?? if_INV:'..tostring(winv)..':'..tostring(ainv))
	--                W.out.avedit = ap
				if winv then
					local sap = ap
					ap = aP
					aP = sap

					ap = {ap[3],ap[2],ap[1]}
					aP = {aP[3],aP[2],aP[1]}
				end
				if ainv then
					ap = {ap[3],ap[2],ap[1]}
					aP = {aP[3],aP[2],aP[1]}
				end
	--                W.out.avedit = ap

				for _,v in pairs(ap) do
					av[#av+1] = v
				end
				for _,v in pairs(aP) do
					av[#av+1] = v
				end
	--                lo('?? for_av:'..#av)
			else
				-- dump to mesh
				meshClose(apair[i], av, adata)
	--                U.dump(av, '?? for_av_in:')

				av = nil
			end
		end

	end

--        U.dump(av, '?? forGable.for_av2:'..tostring(av and #av or nil))
	if not av then return {} end
	meshClose(apair[#apair],av,adata)
--        U.dump(adata, '?? for_data:')
			if dbg then
				for i,d in pairs(adata) do
--                    U.dump(d[1], '?? for_DATA:')
					for k=2,#d.av,3 do
						d.av[k] = d.av[k]+vec3(0,0,1)
					end
				end
			end
--    lo('<<^^^^^^^^^^^^^^^ forGable:'..#adata)
	return adata,apair
end
--[[
	-- thick plates
	local apThick = {
		vec3(x,y,0),c[1],
		(vec3(x,y,0)+b[2])/2, bcMid[2],
		b[2],c[2],
	}
--    local p2 = {c[2],b[1],bcMid[1],bcMid[2]}
	-- thin plates
	local apThin
	local q1 = {}
	local q2 = {}
	-- thin ridge cross plate
	--- pairs width ratio
--    local d = U.toLine(a[2], {a[1],b[1]})
--    local D = U.toLine(b[2], {b[1],c[1]})
--        lo('?? dist:'..d..':'..D)
	-- right thick plate

	return apThick,apThin
]]
--            an,auv,af = M.zip2(base,{1,2,5,4},an,auv,af)
--            an,auv,af = M.zip2(base,{6,5,2,3},an,auv,af)
--[[
					for k=2,#base,3 do
						base[k] = base[k]+vec3(0,0,1)
					end

					for k=2,#base,3 do
						base[k] = base[k]+vec3(0,0,1)
					end
					for k=1,#base do
--                        base[k] = base[k]+vec3(0,0,0.1)
					end
					for k=2,#base,3 do
						base[k] = base[k]+vec3(0,0,1)
					end
					U.dump(base, '?? for_BASE:'..i)
					W.out.avedit = {base[2],base[5]}
]]
--            an,auv,af = M.zip2(base,{2,5,4,1},an,auv,af)
--            an,auv,af = M.zip2(base,{3,6,5,2},an,auv,af)

--            an,auv,af = M.zip2(base,{1,4,5,2},an,auv,af)
--            an,auv,af = M.zip2(base,{6,3,2,5},an,auv,af)
	--[[
	--            an,auv,af = M.zip2(P,{1,2,4,3},an,auv,af)
	--            an,auv,af = M.zip2(P,{6,5,3,4},an,auv,af)
	--            av[#av+1] = U.mod(apair[i][1]+1,base)
	--            av[#av+1] = (U.mod(apair[i][1]+1,base)+U.mod(apair[i][2],base))/2
	--            av[#av+1] = U.mod(apair[i][2],base)

			local er1 = U.mod(i+1,base)-U.mod(i,base)
			local er2 = U.mod(i+2,base)-U.mod(i-1,base)
			local el1 = U.mod(#base-i,base)-U.mod(#base-i+1,base)
			local el2 = U.mod(#base-i-1,base)-U.mod(#base-i,base)
			local d1 = U.toLine(U.mod(i,base), {U.mod(#base-i,base),U.mod(#base-i+1,base)})
			local d2 = U.toLine(U.mod(i+1,base), {U.mod(#base-i-1,base),U.mod(#base-i,base)})
				lo('?? for_D:'..i..':'..tostring(d1)..':'..tostring(d2)..':'..tostring(U.mod(i,base))..':'..tostring(U.mod(i+1,base)))
			if d1<d2 then

			end
	]]


local function forRidge(floor, base, ichild, ishift, ischeck) --, achunk)
	local flat = floor.top.ridge and floor.top.ridge.flat
	-- find parallel pairs
	local aflat = {}
	local desctop = ichild and floor.top.achild[ichild] or floor.top
	local sbase
	if not base then
		base = floor and floor.base or base
		sbase = deepcopy(base)
--            floor.top.margin = 0.
		base = floor and U.polyMargin(floor.base, floor.top.margin) or base
	else
		sbase = deepcopy(base)
		if desctop.margin then
			base = U.polyMargin(base, desctop.margin)
		end
	end
	base = U.polyStraighten(base)
		  lo('>>++++++++ forRidge:'..tostring(ichild)..':'..tostring(ishift)..':'..tostring(floor.top.margin)..':'..tostring(base[1]),true)

--[[
		base = {
			vec3(-0.716707144,0.9078321689,0),
			vec3(-12.56234233,-1.010742044,0),
			vec3(-11.28329286,-8.907832169,0),
			vec3(0.5623423313,-6.989257956,0), }
		base = {
			vec3(-1.937387983,-9.957783447,0),
			vec3(-2.063050671,2.041558572,0),
			vec3(-10.06261202,1.957783447,0),
			vec3(-9.936949329,-10.04155857,0), }
]]
--        U.dump(base, '>> forRidge:'..tostring(floor)..':'..(floor and floor.top.margin or 'NONE')..':'..tostring(flat)..':'..tostring(W))
	if #base ~= 4 then flat = false end
	if #base%2 == 1 then return false end
--        if floor then base = floor.base end

	local apair,amult = pairsUp(base,ishift)
--        U.dump(apair, '??+++++ APAIR:')
--        U.dump(amult, '??+++++ AMULT:')
	for _,l in pairs(amult) do
		if #l > 1 then return false end
	end
--        lo('??+++++ dist:'..(base[apair[1][1]]-base[apair[1][1]+1]):length())
	if #apair == 0 then
		return false
	end
--    if (#base - #apair*2) ~= 6

	for key,list in pairs(amult) do
--            lo('?? for_M:'..key..':'..#list)
		if #list > 1 then
			U.dump(list, '?? has_MULT:'..key)
			-- find pairs in between
		end
	end

--        local ch4 = table.remove(apair,4)
--        table.remove(apair,3)
--        table.insert(apair,1,ch4)
--        U.dump(amult, '?? fR.for_MULT:')
--        U.dump(apair, '?? fR.forRidge.apair:'..tostring(floor))
	--TODO: order pairs
	local function fit()
		for i = 1,#apair-1 do
			local pair = apair[i]
			for j=i+1,#apair do
				local p = apair[j]
--                    U.dump(p, '?? comp:'..i..':'..pair[1])
				if pair[1] == U.mod(p[2]+1,#base) then
					local p1 = table.remove(p, 1)
					p[#p+1] = p1
--                        U.dump(p, '>> flipped:'..j)
				end
				if pair[1] == U.mod(p[1]+1,#base) then
					pair = table.remove(apair,i)
					table.insert(apair,j,pair)
					return true
				end
			end
		end
		return false
	end
	local n = 0
	while fit() and n<100 do
		n  = n + 1
	end

--        U.dump(apair, '??+++++ FIT:')
--        lo('??+++++ dist:'..(base[apair[1][1]]-base[apair[1][1]+1]):length())
--    if fit() then
	local cpair = 1
	local athread = {}
	local cthread = {}
	local inpairs = false
	local cpair = 1
	local indown = false
	local istart = #apair>0 and apair[cpair][2]+1 or 1
	for k = istart,istart+#base-1 do
		local i = U.mod(k,#base)
--    for i = 1,#base do
--            U.dump(apair[cpair],'?? for_I:'..i..':'..cpair..':'..tostring(indown), true)
		if indown then
--            U.dump(cthread,'?? for_DOWN:'..i,true)
			if #apair>0 and i == apair[cpair][2] then
				if not inpairs then
					-- close thread
					inpairs = true
					cthread[#cthread+1] = i
					if #cthread > 2 then
						athread[#athread+1] = cthread
					end
					cthread = {}
				end
			elseif #apair>0 and i == apair[cpair][2]+1 then
				if cpair < #apair then
					cpair = cpair - 1
				end
			else
				if inpairs then
					-- out of pairs
					inpairs = false
					cthread[#cthread+1] = i-1
					if cpair == #apair then
						indown = true
					end
				end
				cthread[#cthread+1] = i
			end
		else
			if #apair>0 and i == apair[cpair][1] then
				if not inpairs then
					-- close thread
					inpairs = true
					cthread[#cthread+1] = i
					if #cthread > 2 then
						athread[#athread+1] = cthread
					end
					cthread = {}
				end
			elseif #apair>0 and i == apair[cpair][1]+1 then
				if cpair < #apair then
					cpair = cpair + 1
--                        lo('?? next:'..cpair..':'..i,true)
				end
			else
				if inpairs then
					-- out of pairs
					inpairs = false
					cthread[#cthread+1] = i-1
					if cpair == #apair then
						indown = true
					end
				end
				cthread[#cthread+1] = i
			end
		end
	end
	if #apair == 0 then athread[#athread+1] = cthread end
	for i,t in pairs(athread) do
		if #t % 2 ~= 0 then
			return false
		end
	end
	if ischeck then return true end
--        U.dump(athread, '?? _aTHREAD:'..istart)
--[[
		U.dump(apair, '?? forRidge.for_PAIRS:')


	for i = 1,#base-2 do
		local u = base[i+1]-base[i]
		for j = i+2,#base do
			local v = U.mod(j+1,base) - base[j]
--                lo('?? for_uv:'..i..':'..j..':'..tostring(u)..':'..tostring(v), true)
			if math.abs(U.vang(u,v)-math.pi) < 10*small_ang then
				apair[#apair+1] = {i,j}
			end
		end
	end
]]

	local achunk = {}
	for i,t in pairs(athread) do
		local cbase = {}
		for _,k in pairs(t) do
			cbase[#cbase+1] = base[k]
		end
		--??
		local cchunk = forChunks(cbase)
--        U.dump(cchunk, '?? for_CCHUNK:'..i)
		for _,c in pairs(cchunk) do
			local chunk = {}
			for _,j in pairs(c) do
				chunk[#chunk+1] = t[j]
			end
			achunk[#achunk+1] = chunk
		end
	end
	local archunk = {}
	for i,p in pairs(apair) do
		archunk[#archunk+1] = {
			p[1],U.mod(p[1]+1,#base),p[2],U.mod(p[2]+1,#base),
		}
	end
--        U.dump(achunk, '??_________ ACHUNK:')
--        U.dump(archunk, '??_________ ARCHUNK:')

--[[
		achunk = {}
	-- check being in chunks
	local achunk = forChunks(base)
		U.dump(achunk, '??_________ ACHUNK2:')
--        U.dump(achunk, '?? forRidge.achunk:')
	local noridge = {}
	for i=1,#achunk do
		noridge[#noridge+1] = i
	end
]]

	local av = {}
	local auv = {}
	local af = {} --,aft = {},{}

	for i,p in pairs(apair) do
		--- get centers
		local c1 = (base[p[1]] + U.mod(p[2]+1,base))/2
		local c2 = (U.mod(p[1]+1,base) + base[p[2]])/2
		local f1,f2,f3,f4
		if i == 1 then
			--- shift middle inward
			c1 = c1 + desctop.tip*(c2-c1):normalized()
		end
		if i == #apair then
			--- shift middle inward
			c2 = c2 + desctop.tip*(c1-c2):normalized()
		end
--[[
		if floor and floor.top.tip then
			if i == 1 then
				--- shift middle inward
				c1 = c1 + floor.top.tip*(c2-c1):normalized()
			end
			if i == #apair then
				--- shift middle inward
				c2 = c2 + floor.top.tip*(c1-c2):normalized()
			end
		end
				if not flat then
					af[#af+1] = {v = #av+0, n = 0, u = #av+0}
					af[#af+1] = {v = #av+5, n = 0, u = #av+5}
					af[#af+1] = {v = #av+2, n = 0, u = #av+2}
				end
				if not flat then
						lo('?? fR_end:'..#av)
					af[#af+1] = {v = #av+1, n = 0, u = #av+1}
					af[#af+1] = {v = #av+3, n = 0, u = #av+3}
					af[#af+1] = {v = #av+4, n = 0, u = #av+4}
				end
			if i == #apair then
				c2 = c2 + floor.top.tip*(c1-c2):normalized()
				if not flat then
					af[#af+1] = {v = #av+1, n = 0, u = #av+1}
					af[#af+1] = {v = #av+3, n = 0, u = #av+3}
					af[#af+1] = {v = #av+4, n = 0, u = #av+4}
				end
			end
]]
--            lo('?? cc:'..tostring(c1)..':'..tostring(c2)..':'..p[1]..':'..p[2], true)
--            lo('?? ffff:'..tostring(f1)..':'..tostring(f2)..':'..tostring(f3)..':'..tostring(f4), true)

		-- split chunk
--                aft = M.zip({#av+3, #av+4, #av+6, #av+5}, aft)

--            W.out.aforest = {}
		if flat then
			f1 = c1*(100-flat)/100 + base[p[1]]*flat/100
			f2 = c2*(100-flat)/100 + U.mod(p[1]+1,base)*flat/100
			f3 = c2*(100-flat)/100 + base[p[2]]*flat/100
			f4 = c1*(100-flat)/100 + U.mod(p[2]+1,base)*flat/100
--[[
			af = M.zip({#av+3, #av+4, #av+7, #av+8}, af)
			av[#av+1] = base[p[1] ]
			av[#av+1] = U.mod(p[1]+1,base)
			av[#av+1] = f1
			av[#av+1] = f2
]]
			local withback = nil

			af = M.zip({#av+1, #av+2, #av+4, #av+3}, af, withback)
			av[#av+1] = base[p[1]]
			av[#av+1] = U.mod(p[1]+1,base)
			av[#av+1] = f1
			av[#av+1] = f2

			af = M.zip({#av+1, #av+2, #av+4, #av+3}, af, withback)
			av[#av+1] = base[p[2]]
			av[#av+1] = U.mod(p[2]+1,base)
			av[#av+1] = f3
			av[#av+1] = f4

			af = M.zip({#av+1, #av+2, #av+4, #av+3}, af, withback)
			av[#av+1] = U.mod(p[2]+1,base)
			av[#av+1] = U.mod(p[1],base)
			av[#av+1] = f4
			av[#av+1] = f1

			af = M.zip({#av+1, #av+2, #av+4, #av+3}, af, withback)
			av[#av+1] = U.mod(p[1]+1,base)
			av[#av+1] = U.mod(p[2],base)
			av[#av+1] = f2
			av[#av+1] = f3

			-- flat cover
--                lo('?? fR_preflat:'..#af)
			af = M.zip({#av-5, #av-4, #av-1, #av-0}, af, withback)
--                lo('?? fR_postflat:'..#af)
		else
			if i==1 then
				af[#af+1] = {v = #av+0, n = 0, u = #av+0}
				af[#af+1] = {v = #av+5, n = 0, u = #av+5}
				af[#af+1] = {v = #av+2, n = 0, u = #av+2}
			end

			af = M.zip({#av+1, #av+2, #av+4, #av+3}, af)
			av[#av+1] = base[p[1]]
			av[#av+1] = U.mod(p[1]+1,base)
			av[#av+1] = c1
			av[#av+1] = c2

--            W.out.aforest = {}
--            W.out.aforest[#W.out.aforest+1] = c1 + vec3(0,0,8) --W.base2world(W.forDesc(), {3, 1}, c1)
--            W.out.aforest[#W.out.aforest+1] = W.forDesc() and (W.base2world(W.forDesc(), {3, 1}, c2) + vec3(0,0,8)) or nil
--                lo('?? sF:'..tostring(c1)..':'..tostring(c2), true)

			af = M.zip({#av+1, #av+2, #av+4, #av+3}, af)
			av[#av+1] = base[p[2]]
			av[#av+1] = U.mod(p[2]+1,base)
			av[#av+1] = c2
			av[#av+1] = c1
			-- ending triangle
			if i == #apair then
				af[#af+1] = {v = #av-7, n = 0, u = #av-7}
				af[#af+1] = {v = #av-5, n = 0, u = #av-5}
				af[#af+1] = {v = #av-4, n = 0, u = #av-6}
			end
		end
	end
--        U.dump(base, '?? base:')
--        U.dump(af, '?? fR.for_AF:'..#af)
--        floor = {top = {margin = 0, mat = 'WM'}}
	local achild = {}
	-- add ridges to achild
	achild[#achild+1] = {
		shape = 'ridge',
		ridge = {on = true, data = {av=av,af=af}},
	--        ridge = {on = true, data = {av=av,auv=auv,af=af,apair=apair}},
		margin = desctop.margin or 0, tip = desctop.tip or 0,
		mat = desctop.mat or floor.top.mat or 'WarningMaterial',
--        margin = floor and floor.top.margin or 0, tip = floor and floor.top.tip or 0,
--        mat = floor and floor.top.mat or 'WarningMaterial'
	}
--            lo('?? forRidge.for_AV:'..tostring(floor)..':'..#av..':'..#af..':'..#floor.top.achild..':'..#achild..':'..tostring(flat),true)
	-- add noridge children
--    for t = 1,0 do

--        U.dump(athread, '?? for_THREAD:')
	for _,t in pairs(athread) do
		local cbase = {}
		for _,i in pairs(t) do
			cbase[#cbase+1] = sbase[i]
		end
--            U.dump(cbase, '?? cbase:'.._)
		achild[#achild+1] =
			{base = cbase, shape = 'flat',
			tip = desctop.tip or 0, margin = desctop.margin or 0,
			mat = desctop.mat or floor.top.mat or 'WarningMaterial',
--            tip = floor and floor.top.tip or 0, margin = floor and floor.top.margin or 0,
--            mat = floor and floor.top.mat or 'WarningMaterial',
			istart = t[1], aside = t}
	end

	if floor and not ichild then
		if not floor.top.achild then floor.top.achild = {} end
--            if iswrong then
--                U.dump(af, '?? for_AF:')
--            end
		-- TODO: base on child identification
--        if floor.top.achild[#achild] then
--            achild[#achild].id = floor.top.achild[#achild].id
--        end
		if #floor.top.achild ~= #achild then
--            lo('??*************** child_REM:'..#floor.top.achild, true)
			for _,c in pairs(floor.top.achild) do
				if c.id then
					scenetree.findObjectById(c.id):delete()
				end
			end
		else
--                lo('?? fR_param:'..#achild)
			for i,c in pairs(achild) do
				c.id = floor.top.achild[i].id
				c.shape = floor.top.achild[i].shape
--                    lo('?? fR_param_:'..i..':'..tostring(c.id),true)
			end
		end
--            U.dump(achild, '?? fR.achild:'..tostring(flat))
		floor.top.achild = achild
		floor.top.ridge.apair = apair
--            lo('<<____________________ forRidge:'..#achild, true)
	end
--    if ichild then
--        floor.top.achild[ichild] = achild[1]
--            U.dump(floor.top.achild[ichild], 'rU_ch:'..ichild)
--    end
	desctop.shape = 'ridge'
--        if floor then U.dump(floor.top.achild, '?? aCHILD:') end
	return true,{av=av,af=af,apair=apair}
end
--[[
		if false then
			for j = 1,#base do
				if j ~= i and j ~= U.mod(i+1,#base) and j ~= U.mod(i-1,#base) then
					local d = (c - (U.mod(j,base)+U.mod(j+1,base))/2):length()
					if d < dmi then
						dmi = d
						imi = {i,j}
						for _,p in pairs(apair) do
							local ps = deepcopy(p)
							table.sort(ps)
							local imis = deepcopy(imi)
							table.sort(imis)
							if i == 6 and not (ps[1] > imis[2] or imis[1] > ps[2] or (ps[1]<imis[1] and imis[2]<ps[2]) or (imis[1]<ps[1] and ps[2]<imis[2])) then
--                            if i == 6 and not (ps[1] > imis[2] or imis[1] > ps[2] or (ps[1]<imis[1] and imis[2]<ps[2]) or (imis[1]<ps[1] and ps[2]<imis[2])) then
	--                        if i == 6 then
								U.dump(ps, '?? for61:'..tostring(ps[1]<imis[1] and imis[2]<ps[2])..':'..tostring(imis[1]<ps[1] and ps[2]<imis[2]))
								U.dump(imis, '?? for62:')
								imi = nil
								break
							else
								dmi = d
							end
						end
					end
				end
			end
		else
--            for j = i+2,#base do
		end

		for j = i+2,#base do
			local d = (c - (U.mod(j,base)+U.mod(j+1,base))/2):length()
			if i == 3 then
				lo('??_____ for_3:'..j..':'..d)
			end
			if i == 5 then
				lo('??_____ for_5:'..j..':'..d)
			end

			if d < dmi then
				dmi = d
				imi = {i,j}
			end

		end
]]

--        U.dump(av, '?? for_AV:'..#av)
--        U.dump(auv, '?? for_AUV:'..#auv)
--        U.dump(aft, '?? for_AF:'..#aft)
--        af = aft
--        table.pack(1,2,3,4)
--        U.dump(noridge, '?? in_CHUNKS:'..#noridge)
--        U.dump(af, '?? for_AF:'..#base..':'..#af)
--[[
		for key,_ in pairs(noridge) do
				U.dump(achunk[key], '??***************** for_chunk:'..key)
			local cbase = {}
			for _,i in pairs(achunk[key]) do
				cbase[#cbase+1] = sbase[i]
	--            av[#av+1] = base[i]
	--            auv[#auv+1] = {u=(av[#av]-ref):dot(u), v=(av[#av]-ref):dot(v)}
			end
	--        af = M.zip(achunk[key], af)
			achild[#achild+1] =
				{base = cbase, shape = 'flat', tip = floor.top.tip, margin = floor.top.margin, mat = floor.top.mat,
				istart = achunk[key][1], aside = achunk[key]}
--            if floor.top.achild[#achild] then
--                achild[#achild].id = floor.top.achild[#achild].id
--            end
		end
]]
--[[
	if false then
		local p2c = {}
		local cpair = {}
		local baseext = deepcopy(base)
		local aridge = {}
			local iswrong
		local nstep = 0

		for i = #apair,1,-1 do
			local p = apair[i]
			local isin
			for k,c in pairs(achunk) do
	--                U.dump(c, '?? if_inchunk:'..p[1]..':'..p[2]..':'..#U.index(c,p[1]))
				if #c == 4 and #U.index(c,p[1]) > 0 and #U.index(c,U.mod(p[1]+1,#base)) > 0 and #U.index(c,p[2]) > 0 and #U.index(c,U.mod(p[2]+1,#base)) > 0 then
	--                    U.dump(p, '?? hit:'..k)
					isin = true
					p2c[i] = k
	--                noridge[k] = nil
					--- get centers
					local c1 = (base[p[1] ] + U.mod(p[2]+1,base))/2
					local c2 = (U.mod(p[1]+1,base) + base[p[2] ])/2
					if floor and floor.top.tip then
						if i == #apair then
							c2 = c2 + floor.top.tip*(c1-c2):normalized()
							af[#af+1] = {v = #av+1, n = 0, u = #av+1}
							af[#af+1] = {v = #av+3, n = 0, u = #av+3}
							af[#af+1] = {v = #av+4, n = 0, u = #av+4}
						end
						if i == 1 then
							c1 = c1 + floor.top.tip*(c2-c1):normalized()
							af[#af+1] = {v = #av+0, n = 0, u = #av+0}
							af[#af+1] = {v = #av+5, n = 0, u = #av+5}
							af[#af+1] = {v = #av+2, n = 0, u = #av+2}
						end
					end

					-- split chunk
	--                aft = M.zip({#av+3, #av+4, #av+6, #av+5}, aft)

					af = M.zip({#av+1, #av+2, #av+4, #av+3}, af)
					av[#av+1] = base[p[1] ]
					av[#av+1] = U.mod(p[1]+1,base)
					av[#av+1] = c1
					av[#av+1] = c2

					af = M.zip({#av+1, #av+2, #av+4, #av+3}, af)
					av[#av+1] = base[p[2] ]
					av[#av+1] = U.mod(p[2]+1,base)
					av[#av+1] = c2
					av[#av+1] = c1
					break
				end
				if #c ~= 4 then
					lo('!! forRidge_WRONG:'..i..':'..k, true)
					iswrong = true
				end
			end
			if not isin then
				table.remove(apair, i)
			end
	--                if i == 2 then break end
		end

	end


--                local u = (base[2]-base[1]):normalized()
--                local v = vec3(-u.y, u.x, 0)

				if false then
					for i=1,8 do
						auv[#auv+1] = {u=0, v=0}
					end
					local u = (U.mod(p[1]+1,base) - base[p[1] ]):normalized()
					local v = vec3(-u.y, u.x, 0)
					local ref = base[p[1] ]
					for j=1,6 do
						local b = av[#av-6+j]
	--                for _,b in pairs(baseext) do
						auv[#auv+1] = {u=(b-ref):dot(u), v=(b-ref):dot(v)}
	--                    auv[#auv+1] = {u=(b-ref):dot(u), v=(b-ref):dot(v)}
					end


					aridge[#aridge+1] = {#baseext+1,#baseext+2}
					p.iridge = {#baseext+1,#baseext+2}
					baseext[#baseext+1] = c1
					baseext[#baseext+1] = c2
	--                    U.dump({p[1], U.mod(p[1]+1,#base), #baseext, #baseext-1}, '?? for_1st:')
	--                    U.dump({p[2], U.mod(p[2]+1,#base), #baseext-1, #baseext}, '?? for_2st:')
					af = M.zip({p[1], U.mod(p[1]+1,#base), #baseext, #baseext-1}, af)
					af = M.zip({p[2], U.mod(p[2]+1,#base), #baseext-1, #baseext}, af)
				end

	av = baseext
	local u = (base[2]-base[1]):normalized()
	local v = vec3(-u.y, u.x, 0)
	local ref = base[1]
	for _,b in pairs(baseext) do
		auv[#auv+1] = {u=(b-ref):dot(u), v=(b-ref):dot(v)}
	end

		av = baseext
		auv = {}
		local u = (base[2]-base[1]):normalized()
		local v = vec3(-u.y, u.x, 0)
		local ref = base[1]
		for _,b in pairs(baseext) do
			auv[#auv+1] = {u=(b-ref):dot(u), v=(b-ref):dot(v)}
		end
]]
--        U.dump(av, '?? av:'..#av)
--    return av,auv,af,aridge
--[[
	if false then
		-- find adjacent parallels
		local ajoin = {}
	--    for k,p in pairs(apair) do
		for k = 1,#apair-1 do
			local p = apair[k]
	--            lo('?? for_PAIR: k='..k..':'..p[1]..':'..p[2]..':'..tostring(apair[k+1][1]-1)..':'..tostring(apair[k+1][2]+1), true)
			if p[1] == apair[k+1][1]-1 and p[2] == apair[k+1][2]+1 then
				ajoin[#ajoin+1] = p
				lo('?? for_JOIN:'..k, true)
				-- s
			end
		end
			U.dump(ajoin, '?? forRidge.ajoin:')
	end
]]


Top.forRidge = forRidge
Top.forGable = forGable
--Top.cut = cut
Top.forChunks = forChunks
Top.pave = pave
Top.pairsUp = pairsUp

Top.inject = inject

return Top