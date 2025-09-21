-- lo('= MESH:')

-- modules
local apack = {
	'/lua/ge/extensions/editor/gen/utils',
}
local U

local function lo(ms)
end

local function reload()
	unrequire(apack[1])
	U = rerequire(apack[1])
end
reload()

local M = {}

local function align(m, pfr, pto, space)
	if not space then space = 0 end
	U.dump(m, '?? align:'..tostring(pfr)..':'..tostring(pto))
--    to()
end

local function log(av, af, auv)
	av = av ~= nil and av or {}
	af = af ~= nil and af or {}
	auv = auv ~= nil and auv or {}

	local an = {}
	an[#an+1] = vec3(-1,-1,0)
	an[#an+1] = vec3(1,-1,0)
	an[#an+1] = vec3(1,1,0)
	an[#an+1] = vec3(-1,1,0)

	af[#af+1] = {v = 0, n = 0, u = 0}
	af[#af+1] = {v = 4, n = 0, u = 4}
	af[#af+1] = {v = 5, n = 1, u = 5}
	af[#af+1] = {v = 0, n = 0, u = 0}
	af[#af+1] = {v = 5, n = 1, u = 5}
	af[#af+1] = {v = 1, n = 1, u = 1}

	af[#af+1] = {v = 1, n = 1, u = 1}
	af[#af+1] = {v = 5, n = 1, u = 5}
	af[#af+1] = {v = 6, n = 2, u = 6}
	af[#af+1] = {v = 1, n = 1, u = 1}
	af[#af+1] = {v = 6, n = 2, u = 6}
	af[#af+1] = {v = 2, n = 2, u = 2}

	af[#af+1] = {v = 2, n = 2, u = 2}
	af[#af+1] = {v = 6, n = 2, u = 6}
	af[#af+1] = {v = 7, n = 3, u = 7}
	af[#af+1] = {v = 2, n = 2, u = 2}
	af[#af+1] = {v = 7, n = 3, u = 7}
	af[#af+1] = {v = 3, n = 3, u = 3}

	af[#af+1] = {v = 3, n = 3, u = 3}
	af[#af+1] = {v = 7, n = 3, u = 7}
	af[#af+1] = {v = 4, n = 0, u = 4}
	af[#af+1] = {v = 3, n = 3, u = 3}
	af[#af+1] = {v = 4, n = 0, u = 4}
	af[#af+1] = {v = 0, n = 0, u = 0}

	av[#av+1] = vec3(0,0,0)
	av[#av+1] = vec3(1,0,0)
	av[#av+1] = vec3(1,1,0)
	av[#av+1] = vec3(0,1,0)

	av[#av+1] = vec3(0,0,5)
	av[#av+1] = vec3(1,0,5)
	av[#av+1] = vec3(1,1,5)
	av[#av+1] = vec3(0,1,5)

	auv[#auv + 1] = {u = 0, v = 0}
	auv[#auv + 1] = {u = 1, v = 0}
	auv[#auv + 1] = {u = 2, v = 0}
	auv[#auv + 1] = {u = 3, v = 0}

	auv[#auv + 1] = {u = 0, v = 5}
	auv[#auv + 1] = {u = 1, v = 5}
	auv[#auv + 1] = {u = 2, v = 5}
	auv[#auv + 1] = {u = 3, v = 5}

	return av,af,auv,an
end


local function uv4poly(base, istart, auv, dbg)
	if not auv then auv = {} end
	local ref = U.mod(istart,base)
	local u = (U.mod(istart+1,base) - ref):normalized()
	local w = u:cross(ref - U.mod(istart-1,base))
  local n = 1
  while w:length() < U.small_val and n < #base do
    n = n + 1
    w = u:cross(ref - U.mod(istart-n,base))
  end
	local v = -(w:cross(u)):normalized()
		if dbg then
			U.dump(base, '?? uv4poly:'..tostring(u)..':'..tostring(v)..':'..tostring(w)..' ref:'..tostring(ref), true)
		end
	for _,b in pairs(base) do
		auv[#auv+1] = {u = (b-ref):dot(u), v = (b-ref):dot(v)}
	end
	return auv
end


local function forNorm(a,b,c,vn)
	vn = vn or (a-b):cross(c-b):normalized()
--        lo('?? forNorm:'..tostring(vn))
	local a = U.vang(vn,vec3(0,0,1))
	return {round(a*10000), round(U.vang(U.proj2D(vn),vec3(1,0,0),true)*10000)},vn
end


local function forUV(base, istart, uvini, w, scale, auv)
	if not scale then scale = {1,1} end
--        U.dump(uvini, '>> forUV:')
--    if not auv then auv = {} end
	local ref = U.mod(istart,base)
	local side = U.mod(istart+1,base) - ref
	local refuv = vec3(0,0,0)
	local sideuv = vec3(side:length(),0,0)
	if uvini then
		refuv = vec3(uvini[1].u, uvini[1].v)
		sideuv = vec3(uvini[2].u-uvini[1].u, uvini[2].v-uvini[1].v)
	end
--            U.dump(uvini, '?? ref:'..tostring(ref)..':'..tostring(refuv)..':'..tostring(sideuv))
	local u = side:normalized()
	if not w then w = u:cross(ref - U.mod(istart-1,base)) end
	local v = (u:cross(w)):normalized()

--        lo('?? forUV_uv:'..tostring(u)..':'..tostring(v))

	local uvu = sideuv/side:length()
	local uvv = vec3(-uvu.y,uvu.x)

	if not auv then auv = {} end
--    for _,b in pairs(base) do
	for i=istart,istart+#base-1 do
		local b = U.mod(i,base)
		local uv = refuv + scale[1]*uvu*(b-ref):dot(u) + scale[2]*uvv*(b-ref):dot(v)
		auv[#auv+1] = {u = uv.x, v = uv.y}
--        auv[#auv+1] = {u = (b-ref):dot(u), v = (b-ref):dot(v)}
	end
	return auv
end


M.forBeam = function(base, L, uvflip, mat, w)
  if not base or #base < 3 then return end
  local a = (base[2]-base[1]):normalized()
  if not w then
    for i=2,#base-1 do
      w = (base[i+1] - base[i]):normalized():cross(a)
--          lo('?? forBeam_w:'..tostring(w))
      if w:length() > U.small_val then break end
    end
  end
--		  U.dump(base,'>> forBeam:'..tostring(L)..':'..tostring(w))
  if not w then return end
  local ai = {}
  local poly = {}
  for i,b in pairs(base) do
    poly[#poly+1] = b
	ai[#ai+1] = #ai + 1
  end
  for i=#base,1,-1 do
    poly[#poly+1] = base[i] + w*L
	ai[#ai+1] = #ai + 1
  end
--  table.insert(ai,1,#ai)
--  table.remove(ai,#ai)
--  table.insert(poly,1,base[1] + w*L)
--		U.dump(poly, '?? poly:')
  local an,auv,af = M.zip2(poly,ai,nil,nil,nil,nil,uvflip)
  return {
    verts = poly,
    normals = an,
    uvs = auv,
    faces = af,
    material = mat or 'WarningMaterial',-- 'm_bricks_01',
  }
end


M.tri2mdata = function(av, ai, istart, dnorm, an, auv, af, iuvini, uvscale)
	if not an then an = {} end
	if not auv then auv = {} end
	if not af then af = {} end
	if not dnorm then dnorm = {} end

--            U.dump(av, '?? for_AV:')
--[[
		if (av[ai[3] ]-av[ai[2] ]):length() == 0 then
			lo('!!_______________________________________________________________________________ ERR_tri2mdata:')
			U.dump(av, '?? AV:')
			U.dump(ai)
		end
]]
--        lo('?? if_NORM:'..tostring(av[ai[1]])..':'..tostring(av[ai[2]])..':'..tostring(av[ai[3]])..':'..tostring((av[ai[1]]-av[ai[2]]):cross(av[ai[3]]-av[ai[2]]):normalized()))
	local akey,vn = forNorm(av[ai[1]], av[ai[2]], av[ai[3]])
--        U.dump(akey, '?? tri2madata:')
	-- TODO: akey[1] may be NaN
	if isnan(akey[1]) then return end
--    if isnan(akey[1])
--            U.dump(akey, '??+++++++++++++++++++++++++ akey:'..tostring(vn)..':'..#an..':'..#av..':'..tostring(ai[1]))
	if not dnorm[akey[1]] then dnorm[akey[1]] = {} end
	local normnew = false
	if not dnorm[akey[1]][akey[2]] then
		-- new normal
		dnorm[akey[1]][akey[2]] = #an
		an[#an+1] = vn
		normnew = true
	end
--    dnorm[akey[1]][akey[2]][#dnorm[akey[1]][akey[2]]+1] = ai

--    local auv = {}
--    auv = forUV({av[ai[1]], av[ai[2]], av[ai[3]]}, 1, auv)
	--uv4poly({av[ai[1]], av[ai[2]], av[ai[3]]}, 1)
  if auv ~= true then
    local tuv = forUV({av[ai[1]], av[ai[2]], av[ai[3]]}, istart,
      iuvini and {auv[iuvini[1]],auv[iuvini[2]]} or nil, nil, uvscale)
  --            U.dump(tuv, '?? for_TUV:'..#auv)
  --    local iuvstart =
    if not iuvini then
      -- append all
      for _,uv in pairs(tuv) do
        auv[#auv+1] = uv
      end
    else
  --        iuvstart = #auv-1
  --        auv[#auv+1] = tuv[U.mod(istart+2,3)]
      auv[#auv+1] = tuv[#tuv] --tuv[U.mod(istart+2,3)]
    end
  end
	-- append new
--    auv[#auv+1] = tuv[U.mod(istart+2,3)]
--    auv[#auv+1] = tuv[#tuv] --tuv[U.mod(istart+2,3)]
--            lo('?? for_TUV2:'..#auv)

  if auv == true then
    af[#af+1] = {v=ai[istart]-1, n=dnorm[akey[1]][akey[2]], u=ai[istart]-1}
    af[#af+1] = {v=ai[U.mod(istart+1,#ai)]-1, n=dnorm[akey[1]][akey[2]], u=ai[U.mod(istart+1,#ai)]-1}
    af[#af+1] = {v=ai[U.mod(istart+2,#ai)]-1, n=dnorm[akey[1]][akey[2]], u=ai[U.mod(istart+2,#ai)]-1}
  else
    af[#af+1] = {v=ai[istart]-1, n=dnorm[akey[1]][akey[2]], u=(iuvini and iuvini[1] or #auv-2)-1}
    af[#af+1] = {v=ai[U.mod(istart+1,#ai)]-1, n=dnorm[akey[1]][akey[2]], u=(iuvini and iuvini[2] or #auv-1)-1}
    af[#af+1] = {v=ai[U.mod(istart+2,#ai)]-1, n=dnorm[akey[1]][akey[2]], u=#auv-1}
  end

--    af[#af+1] = {v=ai[1]-1, n=dnorm[akey[1]][akey[2]], u=0}

	return an,auv,af,dnorm
end


M.zip2 = function(base, ai, an, auv, af, uvscale, uvflip, dbg)
	if #base == 0 then return end
	if not ai then
		ai = {}
		for i=1,#base do
			ai[#ai+1] = i
		end
	end
  if #ai == 0 then return end
	if not an then an = {} end
	if not auv then auv = {} end
	if not af then af = {} end
  local uvkeep = auv == true

--    local auv,an,af = {{u=(base[ai[1]]-base[ai[#ai]]):length(),v=0},{u=0,v=0}}
  if not uvkeep then
    if uvflip then
      auv[#auv+1] = {v=-(base[ai[1]]-base[ai[#ai]]):length(),u=0}
      auv[#auv+1] = {v=0,u=0}
    else
      auv[#auv+1] = {u=(base[ai[1]]-base[ai[#ai]]):length(),v=0}
      auv[#auv+1] = {u=0,v=0}
    end
  end
	local dnorm = {}
  for i,v in pairs(an) do
--        lo('?? z3_n:'..i..':'..tostring(v))
    local akey,vn = forNorm(nil, nil, nil, v)
--          U.dump(akey, '?? zip2_akey:'..tostring(v))
    if not isnan(akey[1]) then
      if not dnorm[akey[1]] then dnorm[akey[1]] = {} end
      if not dnorm[akey[1]][akey[2]] then
        dnorm[akey[1]][akey[2]] = #an-1
      end
    end
--[[
    -- TODO: akey[1] may be NaN
    if isnan(akey[1]) then return end
  --    if isnan(akey[1])
  --            U.dump(akey, '??+++++++++++++++++++++++++ akey:'..tostring(vn)..':'..#an..':'..#av..':'..tostring(ai[1]))
    if not dnorm[akey[1] ] then dnorm[akey[1] ] = {} end
    local normnew = false
    if not dnorm[akey[1] ][akey[2] ] then
      -- new normal
      dnorm[akey[1] ][akey[2] ] = #an
      an[#an+1] = vn
      normnew = true
    end
]]
  end
--      U.dump(dnorm, '?? z2_dndnorm:')

--    local auv,an,af,dnorm = {{u=(base[ai[1]]-base[ai[#ai]]):length(),v=0},{u=0,v=0}}
--            U.dump(auv, '?? zip2.auv:')
	local N = #ai/2
	for i = 1,N-1 do
--        if true or (base[ai[#ai-i]]-base[ai[#ai-i+1]]):length() > 0 then
--        end
			if dbg then
				lo('?? zip2:'..tostring(base[ai[i]])..':'..tostring(base[ai[#ai-i]])..':'..tostring(base[ai[#ai-i+1]]))
			end
		an,auv,af,dnorm = M.tri2mdata(base, {ai[i],ai[#ai-i],ai[#ai-i+1]}, 3, dnorm, an, auv, af, {2*i-1,2*i})--, uvscale)
		an,auv,af,dnorm = M.tri2mdata(base, {ai[#ai-i],ai[i],ai[i+1]}, 1, dnorm, an, auv, af, {2*i+1,2*i})--, uvscale)

--        an,auv,af,dnorm = M.tri2mdata(base, {i,#base-i,#base-i+1}, 3, dnorm, an, auv, af, {2*i-1,2*i})
--        an,auv,af,dnorm = M.tri2mdata(base, {#base-i,i,i+1}, 1, dnorm, an, auv, af, {2*i+1,2*i})
	end
	return an,auv,af
end

--[[
local function zip2_(aiv, av, af)
	if not af then af = {} end
	local dpos = {}
	local dnorm = {}
	local L = #aiv
--    local N = math.floor(L/2)
--        U.dump(aiv, '>> zip:'..#af..' N='..N)
--    local ifr = aiv[1]
--    local inorm = 0
--    local duv = {3,0,2,2,0,1}
	for i = 0,L/2-2 do
		local ai = {aiv[i],aiv[i+1],aiv[L-i]}
		af[#af+1] = {v = ai[1]}
		af[#af+1] = {v = ai[2]}
		af[#af+1] = {v = ai[3]}

		local an = forNorm(av[ai[1] ], av[ai[2] ], av[ai[3] ])
		if not dnorm[an[1] ] then dnorm[an[1] ] = {} end
		if not dnorm[an[1] ][an[2] ] then dnorm[an[1] ][an[2] ] = {} end
		dnorm[an[1] ][an[2] ][#dnorm[an[1] ][an[2] ]+1] = ai

		af[#af+1] = {v = aiv[L-i]}
		af[#af+1] = {v = aiv[i]}
		af[#af+1] = {v = aiv[L-(i+1)]}


		af[#af + 1] = {v = aiv[istart - i]-1} --, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart-i]-1}
		af[#af + 1] = {v = aiv[istart - (i+1)]-1} --, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart - (i+1)]-1}
		af[#af + 1] = {v = aiv[istart+i+1]-1} --, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+1]-1}

		af[#af + 1] = {v = aiv[istart+i+1]-1} --, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+1]-1}
		af[#af + 1] = {v = aiv[istart - (i+1)]-1} --, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart - (i+1)]-1}
		af[#af + 1] = {v = aiv[istart+i+2]-1} --, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+2]-1}

	end
--        U.dump(af, '?? zip2:'..istart)
--        lo('?? zip:'..N..':'..#aiv..':'..#af)
	if #aiv%2 == 1 then
		-- last triangle
		if #aiv == 3 then
			istart = istart + 1
		end
--            lo('?? zip:'..istart..':'..N)
		local i = N-1
		af[#af + 1] = {v = aiv[istart - i] - 1} --, n = 0, u = aiv[istart - i] - 1}
		af[#af + 1] = {v = aiv[U.mod(istart - (i+1),#aiv)] - 1} --, n = 0, u = aiv[U.mod(istart - (i+1),#aiv)]-1}
		af[#af + 1] = {v = aiv[istart+i+1] - 1} --, n = 0, u = aiv[istart+i+1] - 1}
	end
end
]]
--[[
		if withnorm then inorm = i end
		-- withuv and duv[n]+i*4 or
		if withback ~= 2 then
		end
		if withback then
			n = 1
			af[#af + 1] = {v = aiv[istart - i]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart - i]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart+i+1]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+1]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart - (i+1)]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart - (i+1)]-1}
			n=n+1

			af[#af + 1] = {v = aiv[istart+i+1]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+1]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart+i+2]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+2]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart - (i+1)]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart - (i+1)]-1}
		end
]]
--[[
		if withback ~= 2 then
		end
		if withback then
			af[#af + 1] = {v = aiv[istart - i] - 1, n = 0, u = aiv[istart - i]-1}
			af[#af + 1] = {v = aiv[istart+i+1] - 1, n = 0, u = aiv[istart+i+1]-1}
			af[#af + 1] = {v = aiv[U.mod(istart - (i+1),#aiv)] - 1, n = 0, u = aiv[U.mod(istart - (i+1),#aiv)]-1}
		end
]]


local function zip(aiv, af, withback, withnorm, withuv)
	af = af ~= nil and af or {}
--    auv = auv ~= nil and auv or {}

	local N = math.floor(#aiv/2)
--        U.dump(aiv, '>> zip:'..#af..' N='..N)
	local istart = N
	local inorm = 0
	local duv = {3,0,2,2,0,1}
	for i = 0,N-2 do
		if withnorm then inorm = i end
		-- withuv and duv[n]+i*4 or
		local n = 1
		if withback ~= 2 then
			af[#af + 1] = {v = aiv[istart - i]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart-i]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart - (i+1)]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart - (i+1)]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart+i+1]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+1]-1}
			n=n+1

			af[#af + 1] = {v = aiv[istart+i+1]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+1]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart - (i+1)]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart - (i+1)]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart+i+2]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+2]-1}
		end

		if withback then
			n = 1
			af[#af + 1] = {v = aiv[istart - i]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart - i]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart+i+1]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+1]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart - (i+1)]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart - (i+1)]-1}
			n=n+1

			af[#af + 1] = {v = aiv[istart+i+1]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+1]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart+i+2]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart+i+2]-1}
			n=n+1
			af[#af + 1] = {v = aiv[istart - (i+1)]-1, n = inorm, u = withuv and duv[n]+i*4 or aiv[istart - (i+1)]-1}
		end
	end
--        U.dump(af, '?? zip2:'..istart)
--        lo('?? zip:'..N..':'..#aiv..':'..#af)
	if #aiv%2 == 1 then
		-- last triangle
		if #aiv == 3 then
			istart = istart + 1
		end
--            lo('?? zip:'..istart..':'..N)
		local i = N-1
		if withback ~= 2 then
			af[#af + 1] = {v = aiv[istart - i] - 1, n = 0, u = aiv[istart - i] - 1}
			af[#af + 1] = {v = aiv[U.mod(istart - (i+1),#aiv)] - 1, n = 0, u = aiv[U.mod(istart - (i+1),#aiv)]-1}
			af[#af + 1] = {v = aiv[istart+i+1] - 1, n = 0, u = aiv[istart+i+1] - 1}
		end
		if withback then
			af[#af + 1] = {v = aiv[istart - i] - 1, n = 0, u = aiv[istart - i]-1}
			af[#af + 1] = {v = aiv[istart+i+1] - 1, n = 0, u = aiv[istart+i+1]-1}
			af[#af + 1] = {v = aiv[U.mod(istart - (i+1),#aiv)] - 1, n = 0, u = aiv[U.mod(istart - (i+1),#aiv)]-1}
		end
	end
--        U.dump(af, '<< zip:')
	return af
end


local function zip_(base, av, af, auv, ref, orig)
	if not base or #base == 0 then return end
--        U.dump(base, '>> zip:')
	av = av ~= nil and av or {}
	af = af ~= nil and af or {}
	auv = auv ~= nil and auv or {}
	if not orig then orig = base[1] end

	local N = math.floor(#base/2) - 1
	for i = 1,N do
		af[#af + 1] = {v = #av+0, n = 0, u = #av+0}
		af[#af + 1] = {v = #av+1, n = 0, u = #av+1}
		af[#af + 1] = {v = #av+2, n = 0, u = #av+2}
		af[#af + 1] = {v = #av+2, n = 0, u = #av+2}
		af[#af + 1] = {v = #av+1, n = 0, u = #av+1}
		af[#af + 1] = {v = #av+3, n = 0, u = #av+3}

		av[#av + 1] = base[i]
		av[#av + 1] = base[#base - (i-1)]
		av[#av + 1] = base[i+1]
		av[#av + 1] = base[#base - i]

		auv[#auv + 1] = {u = (av[#av-3] - orig):dot(ref[1]), v = (av[#av-3] - orig):dot(ref[2])}
		auv[#auv + 1] = {u = (av[#av-2] - orig):dot(ref[1]), v = (av[#av-2] - orig):dot(ref[2])}
		auv[#auv + 1] = {u = (av[#av-1] - orig):dot(ref[1]), v = (av[#av-1] - orig):dot(ref[2])}
		auv[#auv + 1] = {u = (av[#av] - orig):dot(ref[1]), v = (av[#av] - orig):dot(ref[2])}
	end
	if #base % 2 == 1 then
		-- last
		af[#af + 1] = {v = #av-2, n = 0, u = #av-2}
		af[#af + 1] = {v = #av-1, n = 0, u = #av-1}
		af[#af + 1] = {v = #av+0, n = 0, u = #av+0}
--        af[#af + 1] = {v = #av-1, n = 0, u = #av-1}
--        af[#af + 1] = {v = #av-2, n = 0, u = #av-2}
--        af[#af + 1] = {v = #av+0, n = 0, u = #av+0}

		av[#av + 1] = base[N+2]
--            lo('?? zip_last:'..(N+1))

		auv[#auv + 1] = {u = (av[#av] - orig):dot(ref[1]), v = (av[#av] - orig):dot(ref[2])}
	end

	return av,af,auv
end


local function tri(a, b, c, av, af, auv, ref, orig)
	av = av ~= nil and av or {}
	af = af ~= nil and af or {}
	auv = auv ~= nil and auv or {}
	if not orig then orig = a end

	af[#af + 1] = {v = #av+0, n = 0, u = #av+0}
	af[#af + 1] = {v = #av+1, n = 0, u = #av+1}
	af[#af + 1] = {v = #av+2, n = 0, u = #av+2}
	af[#af + 1] = {v = #av+0, n = 1, u = #av+0}
	af[#af + 1] = {v = #av+2, n = 1, u = #av+1}
	af[#af + 1] = {v = #av+1, n = 1, u = #av+2}


	auv[#auv + 1] = {u = (a - orig):dot(ref[1]), v = (a - orig):dot(ref[2])}
	auv[#auv + 1] = {u = (b - orig):dot(ref[1]), v = (b - orig):dot(ref[2])}
	auv[#auv + 1] = {u = (c - orig):dot(ref[1]), v = (c - orig):dot(ref[2])}

	av[#av + 1] = a
	av[#av + 1] = b
	av[#av + 1] = c

	return av, af, auv
end


local function rc(u, v, av, af, pos, withback)
	av = av ~= nil and av or {}
	af = af ~= nil and af or {}
	pos = pos ~= nil and pos or vec3(0, 0, 0)

	if withback ~= -1 then
		af[#af + 1] = {v = #av+0, n = 0, u = #av+0}
		af[#af + 1] = {v = #av+3, n = 0, u = #av+3}
		af[#af + 1] = {v = #av+1, n = 0, u = #av+1}

		af[#af + 1] = {v = #av+1, n = 0, u = #av+1}
		af[#af + 1] = {v = #av+3, n = 0, u = #av+3}
		af[#af + 1] = {v = #av+2, n = 0, u = #av+2}
	end

	if withback then
		af[#af + 1] = {v = #av+0, n = 1, u = #av+0}
		af[#af + 1] = {v = #av+1, n = 1, u = #av+1}
		af[#af + 1] = {v = #av+3, n = 1, u = #av+3}

		af[#af + 1] = {v = #av+3, n = 1, u = #av+3}
		af[#af + 1] = {v = #av+1, n = 1, u = #av+1}
		af[#af + 1] = {v = #av+2, n = 1, u = #av+2}
	end

	av[#av + 1] = pos
	av[#av + 1] = av[#av] + u
	av[#av + 1] = av[#av] + v
	av[#av + 1] = av[#av] - u

	return av, af
end


local function rect(u, v, av, af, pos, hasfront)
--            lo('?? rect:'..tostring(u)..':'..tostring(v))
	if av == nil then
		av, af = {}, {}
	end
	if pos == nil then
		pos = vec3(0, 0, 0)
	end
	av[#av + 1] = pos
	av[#av + 1] = av[#av] + u
	av[#av + 1] = av[#av] + v
	av[#av + 1] = av[#av] - u

	if hasfront ~= false then
		af[#af + 1] = {v = 0, n = 0, u = 0}
		af[#af + 1] = {v = 3, n = 0, u = 1}
		af[#af + 1] = {v = 1, n = 0, u = 3}

		af[#af + 1] = {v = 1, n = 0, u = 3}
		af[#af + 1] = {v = 3, n = 0, u = 1}
		af[#af + 1] = {v = 2, n = 0, u = 2}
	end

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


M.uvTransform = function(auv, shift, scale)
	if not shift then shift = {0,0} end
	if not scale then scale = {1,1} end
	for i=1,#auv do
		auv[i].u = auv[i].u + shift[1]*scale[1]
		auv[i].v = auv[i].v + shift[2]*scale[2]
	end
	for i=2,#auv do
		auv[i].u = auv[1].u + (auv[i].u-auv[1].u)*scale[1]
		auv[i].v = auv[1].v + (auv[i].v-auv[1].v)*scale[2]
	end
end


local function uv4grid(uvx, uvy, ax, ay, X, Y, auv)
--            U.dump(ax, '?? uv4grid_x:')
--            U.dump(ay, '?? uv4grid_y:')
	local u1, u2 = uvx[1], uvx[2]
	local v1, v2 = uvy[1], uvy[2]
	X = X ~= nil and X or ax[#ax]
	Y = Y ~= nil and Y or ay[#ay]
--    local X,Y = ax[#ax], ay[#ay]
	local auv = auv ~= nil and auv or {}
	for i,y in pairs(ay) do
		local row = {}
		for j,x in pairs(ax) do
			auv[#auv + 1] = {u = u1 + (u2 - u1)*x/X, v = v1 + (v2 - v1)*(Y - y)/Y}
		end
	end
--[[
	local skip,n = {},0
	for i = 1,#ay-1 do
		for j = 1,#ax-1 do
			if j%2 == 0 and i%2 == 0 then
				skip[#skip + 1] = n
			end
			n = n + 1
		end
	end
]]
	return auv
end


local function grid2plate(u, v, ax, ay, askip, av, af, auv, an, uvx, uvy)
	an = an ~= nil and an or {}
	an[#an + 1] = u:cross(v):normalized()
	-- TODO: add inversed for backplate
	av = av ~= nil and av or {}
--    istart = istart ~= nil and istart or #av
	local istart = #av
	askip = askip ~= nil and askip or {}
	local X,Y = u:length(),v:length()

	uvx = uvx == nil and {0, X} or uvx
	uvy = uvy == nil and {0, Y} or uvy
	local u1, u2 = uvx[1], uvx[2]
	local v1, v2 = uvy[1], uvy[2]

	auv = auv ~= nil and auv or {}
	for i,y in pairs(ay) do
		for j,x in pairs(ax) do
			av[#av + 1] = u*x + v*y
			auv[#auv + 1] = {u = u1 + (u2 - u1)*x, v = v1 + (v2 - v1)*(1 - y)}
--            av[#av + 1] = u*x/X + v*y/Y
--            auv[#auv + 1] = {u = u1 + (u2 - u1)*x/X, v = v1 + (v2 - v1)*(Y - y)/Y}
		end
	end

	af = af ~= nil and af or {}
	local n = 0
	for i = 0,#ay - 2 do
		for j = 0,#ax - 2 do
			local toflip, skip
			if askip[i] ~= nil and askip[i][j] ~= nil then
--                lo('??_________ for_TYPE:'..tostring(type(skip[i][j])))
				toflip = type(askip[i][j]) == 'table' and askip[i][j][1] or false
				skip = type(askip[i][j]) == 'table' and askip[i][j][2] or askip[i][j]
					toflip = true
			end
--                toflip = true
			if skip == nil or skip < 0 then
				af[#af + 1] = {
					v = istart+ i*#ax + j,
					u = istart+ i*#ax + j,
					n = #an-1}
				af[#af + 1] = {
					v = istart+ (i+1)*#ax + ((toflip and 1 or 0) + j),
					u = istart+ (i+1)*#ax + ((toflip and 1 or 0) + j),
					n = #an-1}
				af[#af + 1] = {
					v = istart+ i*#ax + (j+1),
					u = istart+ i*#ax + (j+1),
					n = #an-1}
				-- back
				af[#af + 1] = {
					v = istart+ (i+1)*#ax + ((toflip and 1 or 0) + j),
					u = istart+ (i+1)*#ax + ((toflip and 1 or 0) + j),
					n = #an-1}
				af[#af + 1] = {
					v = istart+ i*#ax + j,
					u = istart+ i*#ax + j,
					n = #an-1}
				af[#af + 1] = {
					v = istart+ i*#ax + (j+1),
					u = istart+ i*#ax + (j+1),
					n = #an-1}
			end
			if skip == nil or skip > 0 then
				af[#af + 1] = {
					v = istart+ i*#ax + (j + (toflip and 0 or 1)),
					u = istart+ i*#ax + (j + (toflip and 0 or 1)),
					n = #an-1}
				af[#af + 1] = {
					v = istart+ (i+1)*#ax + j,
					u = istart+ (i+1)*#ax + j,
					n = #an-1}
				af[#af + 1] = {
					v = istart+ (i+1)*#ax + (j+1),
					u = istart+ (i+1)*#ax + (j+1),
					n = #an-1}
				-- back
				af[#af + 1] = {
					v = istart+ (i+1)*#ax + j,
					u = istart+ (i+1)*#ax + j,
					n = #an-1}
				af[#af + 1] = {
					v = istart+ i*#ax + (j + (toflip and 0 or 1)),
					u = istart+ i*#ax + (j + (toflip and 0 or 1)),
					n = #an-1}
				af[#af + 1] = {
					v = istart+ (i+1)*#ax + (j+1),
					u = istart+ (i+1)*#ax + (j+1),
					n = #an-1}
			end
--[[
			if skip[n] == nil or skip[n] < 0 then
--            if skip[n] == nil or skip[n] < 0 then
				-- first triangle
af[#af + 1] = {v = istart+ i*#ax + j, n = 0,        u = istart+ i*#ax+j}
af[#af + 1] = {v = istart+ (i+1)*#ax + j, n = 0,    u = istart+ (i+1)*#ax+j}
af[#af + 1] = {v = istart+ i*#ax + (j+1), n = 0,    u = istart+ i*#ax+(j+1)}
			end
			if skip[n] == nil or skip[n] > 0 then
				-- 2nd triangle
af[#af + 1] = {v = istart+ i*#ax + j + 1, n = 0,        u = istart+ i*#ax+(j+1)}
af[#af + 1] = {v = istart+ (i+1)*#ax + j, n = 0,        u = istart+ (i+1)*#ax+j}
af[#af + 1] = {v = istart+ (i+1)*#ax + (j+1), n = 0,    u = istart+ (i+1)*#ax+(j+1)}
			end
]]
			n = n + 1
		end
	end

	return av,af,auv,an
end


local function fromGrid(u, v, ax, ay, skip, av, af, X, Y, uvstart)
	uvstart = uvstart ~= nil and uvstart or 0
	skip = skip ~= nil and skip or {}
--    local X,Y = ax[#ax],ay[#ay]
	X = u:length()
	Y = v:length()
--    X = X ~= nil and X or ax[#ax]
--    Y = Y ~= nil and Y or ay[#ay]

--    local av, af = {}, {}
	av = av ~= nil and av or {}
	af = af ~= nil and af or {}

	for i,y in pairs(ay) do
		for j,x in pairs(ax) do
			av[#av + 1] = u*x/X + v*y/Y
		end
	end
	local n = 0
	for i = 0,#ay-2 do
		for j = 0,#ax-2 do
--            lo('?? for_skip:'..i..':'..j..':'..n)
			if #U.index(skip, n) == 0 then
				af[#af + 1] = {v = uvstart+i*#ax + j, n = 0, u = uvstart+i*#ax+j}
				af[#af + 1] = {v = uvstart+(i+1)*#ax + j, n = 0, u = uvstart+(i+1)*#ax+j}
				af[#af + 1] = {v = uvstart+i*#ax + (j+1), n = 0, u = uvstart+i*#ax+(j+1)}

				af[#af + 1] = {v = uvstart+i*#ax + j + 1, n = 0, u = uvstart+i*#ax+(j+1)}
				af[#af + 1] = {v = uvstart+(i+1)*#ax + j, n = 0, u = uvstart+(i+1)*#ax+j}
				af[#af + 1] = {v = uvstart+(i+1)*#ax + (j+1), n = 0, u = uvstart+(i+1)*#ax+(j+1)}
			end
			n = n + 1
		end
	end
	return av, af
end


local function grid2mesh(u, v, ax, ay, uext, vext, av, af, auv, skip)
	-- V-F
--            U.dump(vext, '?? grid2mesh_pre:'..#av..':'..#ax..'x'..#ay..':'..u:length())
--            U.dump(ax, '?? grid2mesh_ax:'..#ax)
--            U.dump(ay, '?? grid2mesh_ay:'..#ay)
--        ax = {0,uext[2]/2,uext[2]}
--        ay = {0,vext[2]/2,vext[2]}
	av, af = fromGrid(u, v, ax, ay, skip, av, af, u:length(), v:length(), #auv)
--            U.dump(av, '?? grid2mesh_post:'..#av)
--            U.dump(af, '?? grid2mesh_AF:'..#af)
	-- UV
--    auv = uv4grid(uext, vext, ax, ay, u:length(), v:length(), auv)

	auv = auv ~= nil and auv or {}
	u = u:normalized()
	v = v:normalized()
	local orig = u*uext[1] + v*vext[1] -- vext[1])
	for i = #ax*#ay-1,0,-1 do
		auv[#auv + 1] = {u = (av[#av - i] + orig):dot(u), v = -(av[#av - i] + orig):dot(v)}
	end

	return av, af, auv
end


local slaxml
local slaxdom
if true or U._PRD == 0 then
	slaxml = require('libs/slaxml/slaxml')
	slaxdom = require('libs/slaxml/slaxdom')
end

local afloat = {}
local aint = {}
local amat = {}

local ingeo

local function xml2mesh(node, lvl)
	if lvl == nil then
		lvl = 1
	end
	if node.kids ~= nil then
		for _,kid in pairs(node.kids) do
			if kid.name == 'geometry' then
				lo('??**** IN_GEO:'..kid.attr.id)
				ingeo = kid.attr.id
				afloat[ingeo] = {}
				aint[ingeo] = {}
			elseif kid.name == 'float_array' then
--                lo('?? pre_split:'..#kid.kids)
				if #kid.kids > 0 then
					local astr = U.split(kid.kids[1].value, ' ')
					lo('?? xml2mesh.VNU:'..lvl..':'..kid.attr.id..':'..#astr..'/'..tostring(kid.attr.count)..':'..tostring(ingeo))
					if ingeo ~= nil then
						afloat[ingeo][#afloat[ingeo] + 1] = astr
					end
				end
			elseif ({polylist = 1, triangles = 1})[kid.name] ~= nil then
					lo('?______ for_aint:'..#kid.kids..':'..tostring(kid.attr.count))
				for _,k in pairs(kid.kids) do
--                        lo('?? skid:'..k.name)
					if k.name == 'p' then
						local astr = U.split(k.kids[1].value, ' ')
						lo('?? xml2mesh.FACES:'..tostring(kid.attr.material)..':'..#astr..':'..kid.attr.count..' step:'..(#astr/kid.attr.count))
--                        lo('?? vkid:'..#k.kids..':'..#astr)
--                        afloat[ingeo][#afloat[ingeo] + 1] = astr
						aint[ingeo][#aint[ingeo] + 1] = astr
					end
				end
			end
			xml2mesh(kid, lvl + 1)
			if kid.name == 'geometry' then
				ingeo = nil
			end
		end
	end
	if lvl == 1 then lo('<< xml2mesh:') end
	return node
end


local cgeom,cfloat = 0,0

local function toKids(node, ageom)
	if node.kids ~= nil then
		for _,kid in pairs(node.kids) do
			if kid.name == 'geometry' then
				cgeom = cgeom + 1
				lo('??**** IN_GEO:'..cgeom..':'..kid.attr.id..':'..tostring(ageom[cgeom]))
				ingeo = kid.attr.id
--                afloat[ingeo] = {}
--                aint[ingeo] = {}
			elseif kid.name == 'float_array' then
--                lo('?? pre_split:'..#kid.kids)
				cfloat = cfloat + 1
				if #kid.kids > 0 then
					lo('?? toKids.VNU:'..cfloat..':'..kid.attr.id..':'..tostring(ingeo)..':'..#ageom[cgeom].verts) --..':'..#astr..':'..tostring(ingeo))
					local list,step,count
					if cfloat == 1 then
						list = ageom[cgeom].verts
						step = 3
					elseif cfloat == 2 then
						list = ageom[cgeom].normals
						step = 3
					elseif cfloat == 3 then
						list = ageom[cgeom].uvs
						step = 2
					end
					local str = ''
					for _,v in pairs(list) do
						if step == 3 then
							str = str..v.x..' '..v.y..' '..v.z..' '
						else
							str = str..v.u..' '..v.v..' '
						end
					end
					kid.kids[1].value = str
					for i,a in ipairs(kid.attr) do
						if a.name == 'count' then
							a.value = tostring(#list * step)
							lo('?? for_count:'..a.value)
						end
					end

--                    local astr = U.split(kid.kids[1].value, ' ')
--                    if ingeo ~= nil then
--                        afloat[ingeo][#afloat[ingeo] + 1] = astr
--                    end
				end
			elseif ({polylist = 1, triangles = 1})[kid.name] ~= nil then
					lo('?______ for_aint:'..#kid.kids)
				for _,k in pairs(kid.kids) do
--                        lo('?? skid:'..k.name)
					if k.name == 'p' then
						lo('?? toKids.FACES:'..tostring(kid.attr.material))--..':'..#astr)
						local str = ''
						for o,f in pairs(ageom[cgeom].faces) do
							str = str..f.v..' '..f.n..' '..f.u..' '
						end
						k.kids[1].value = str

--                        local astr = U.split(k.kids[1].value, ' ')
--                        aint[ingeo][#aint[ingeo] + 1] = astr
--                        lo('?? vkid:'..#k.kids..':'..#astr)
--                        afloat[ingeo][#afloat[ingeo] + 1] = astr
					end
				end
				for i,a in ipairs(kid.attr) do
--                    lo('?? name:'.._..':'..kid.name..':'..a.name)
					if a.name == 'count' then
						a.value = tostring(#ageom[cgeom].faces * 3 / 9)
						lo('?? for_count_f:'..a.value)
					end
				end
			end
			toKids(kid, ageom)
			if kid.name == 'geometry' then
				ingeo = nil
			end
		end
	end
	return node

--[[
	if node.kids ~= nil then
		for _,kid in pairs(node.kids) do
			if kid.name == 'float_array' then
				local astr = U.split(kid.kids[1].value, ' ')
				lo('??++++ FOF:'..#astr..':'..(astr[1]+1)) --..tostring(kid.kids[1].value))
			--                lo('?? FOF:'..tostring(#kid.kids[1]))
				afloat[#afloat + 1] = astr
			end

			if kid.name == 'library_materials' then
				lo('?? for_LM:'..#kid.kids)
				local imat = 0
				for _,m in ipairs(kid.kids) do
					if m.name == 'material' then
						imat = imat + 1
						lo('?? lm_kid:'..imat..':'..m.name)
						if m.attr ~= nil then
							for i,a in ipairs(m.attr) do
								if a.name == 'name' then
									if m.name == prop then
										if #U.index(aind, imat) > 0 then
--                                            lo('?? TOSET:'..imat..':'..val)
											a.value = val
										end
									else
										amat[#amat + 1] = a.value
									end
								end
							end
						end
					end
				end
			end

			if kid.name == 'triangles' then
--                lo('?? for_TRI:'..#kid.kids..':'..#kid.attr) --..':'..#astr)
				for i,a in ipairs(kid.attr) do
				end
			end
			kidsList(kid, lvl + 1, prop, aind, val)
---            lo('] kid:'..lvl..':'..pref..':'..kid.name)
		end
	end
	return node
]]
end


local function forNode(node, path, lvl, ind)
	if not lvl then lvl = 1 end
	if not ind then ind = 1 end
	if node.kids ~= nil then
		for _,kid in pairs(node.kids) do
			local nmatch = 0
--				U.dump(path, '?? for_PATH:'..lvl)
--				lo('?? forNode:'.._..':'..kid.name..'/'..path[lvl])
			if kid.name == path[lvl] then
				nmatch = nmatch + 1
			end
			if nmatch == ind then
				if lvl == #path then
					return kid
				elseif lvl < #path then
					return forNode(kid, path, lvl+1, ind)
				end
			end
		end
	end
end
M.forNode = forNode


local function toNode(nd, name, obj)
	local kid = {type='element', name=name, attr={}, kids={}}
	for key,val in pairs(obj) do
		kid.attr[#kid.attr+1] = {name=key, value=val}
	end
	nd.kids[#nd.kids+1] = kid

	return kid
end
M.toNode = toNode


M.ofNode = function(nd, obj)
	for key,val in pairs(obj) do
		if not nd.attr then nd.attr = {} end
		local match
		for i,a in pairs(nd.attr) do
			if a.name == key then
				a.value = val
				match = true
			end
		end
		if not match then
			nd.attr[#nd.attr+1] = {name=key, value=val}
		end
	end
end


local function xmlOn(fname)
	if not slaxml then return end

	local f = io.open(fname)
	local xmlContents = f:read('*all')
	io.close(f)

	return slaxml:dom(xmlContents)
end
M.xmlOn = xmlOn


M.xml2file = function(xml, fname)
	local outputFile = io.open(fname, "w")
--        lo('?? matSet3:'..tostring(outputFile))
	if outputFile then
--      U.lo('?? pres2:'..tostring(doc.root))
		local xmlstr = slaxdom:xml(xml.root)
--      U.lo('?? pres3:'..tostring(xmlstr))
		outputFile:write('<?xml version="1.0" encoding="utf-8"?>\n'..xmlstr)
		outputFile:close()

		print('?? DONE:')
	end
end


local function kidsList(node, lvl, prop, aind, val, dbg)
--    local afloat = {}
--    if dbg then U.lo('>> kidsList:'..lvl..':'..tostring(node.kids and #node.kids or nil)..'>'..tostring(prop)) end
  	local idel = {}
	if node.kids ~= nil then
		for _,kid in pairs(node.kids) do
		if kid.name == prop and not val then
			U.lo('?? check_KID:'.._..':'..prop..':'..tableSize(node.kids))
			table.remove(node.kids, _)
			U.lo('?? deld_KID:'.._..':'..tableSize(node.kids))
			return
		end
--        if dbg then U.lo('?? kid_name:'..tostring(kid.name)) end
--[[
			local pref = ' '
			if lvl == 2 then
				pref = '  '
			elseif lvl == 3 then
				pref = '   '
			elseif lvl == 4 then
				pref = '    '
			elseif lvl == 5 then
				pref = '     '
			elseif lvl == 6 then
				pref = '      '
			elseif lvl == 7 then
				pref = '       '
			elseif lvl == 8 then
				pref = '        '
			elseif lvl == 9 then
				pref = '         '
			elseif lvl == 10 then
				pref = '          '
			elseif lvl == 11 then
				pref = '           '
			elseif lvl == 12 then
				pref = '            '
			end
---            lo('[ kid:'..lvl..':'..pref..':'..kid.name)
			if kid.name == 'float_array' then
				local astr = U.split(kid.kids[1].value, ' ')
				lo('??++++ FOF:'..#astr..':'..(astr[1]+1)) --..tostring(kid.kids[1].value))
--                lo('?? FOF:'..tostring(#kid.kids[1]))
				afloat[#afloat + 1] = astr
			end
]]
	if kid.name == 'float_array' then
		local astr = U.split(kid.kids[1].value, ' ')
--                lo('??++++ FOF:'..#astr..':'..(astr[1]+1)) --..tostring(kid.kids[1].value))
	--                lo('?? FOF:'..tostring(#kid.kids[1]))
		afloat[#afloat + 1] = astr
	end


	if kid.name == 'library_materials' then
--                lo('?? for_LM:'..#kid.kids)
		local imat = 0
		for _,m in ipairs(kid.kids) do
			if m.name == 'material' then
		U.lo('??+++++++++++++++++++++ kid_MAT:'.._)
				imat = imat + 1
--                        lo('?? lm_kid:'..imat..':'..m.name)
				if m.attr ~= nil then
					for i,a in ipairs(m.attr) do
						if a.name == 'name' then
							if m.name == prop then
								if dbg then
									U.lo('?? kid_for:'..prop..':'..imat)
								end
								if not aind then
									U.lo('?? mat_REPL:'..tostring(a.value)..'>'..tostring(val))
									a.value = val
								elseif #U.index(aind, imat) > 0 then
--                                            lo('?? TOSET:'..imat..':'..val)
									a.value = val
								end
							else
								amat[#amat + 1] = a.value
							end
						end
					end
				end
			end
		end
	end

	if kid.name == 'triangles' then
--                lo('?? for_TRI:'..#kid.kids..':'..#kid.attr) --..':'..#astr)
		for i,a in ipairs(kid.attr) do
--                    lo('?? for_attr:'..i..':'..tostring(a.name))
--[[
			if prop ~= nil and a.name == prop and i == ind then
				lo('?? TO_SET:'..i..':'..val)
				a.value = val
			end
			if a.name == 'material' then
				lo('?? for_mat:'..a.value)
				amat[#amat + 1] = a.value
			end
]]
		end
--[[
		for _,k in pairs(kid.kids) do
--                    lo('?? skid:'..k.name)
			if k.name == 'p' then
				local astr = U.split(k.kids[1].value, ' ')
--                        lo('?? vkid:'..#k.kids..':'..#astr)
				aint[#aint + 1] = astr
			end
		end
]]
--                aint[#aint + 1] =  U.split(kid.kids[1].value, ' ')
	end
--        U.lo('?? pre_NEXT:'..tostring(kid))
	local goon = kidsList(kid, lvl + 1, prop, aind, val, dbg)
    if not goon then return end
---            lo('] kid:'..lvl..':'..pref..':'..kid.name)
		end
	end
	return node
end


local function step(base, ind)
	return base[(ind) % #base + 1] - base[(ind - 1) % #base + 1]
end


local function daeParse(fname)
	local xmlContents = io.open(fname):read('*all')
	local doc = slaxml:dom(xmlContents)
	return nil
end


M.matReplace = function(fname, mat)
      U.lo('>> matReplace:'..tostring(fname))
--      print('>> matReplace:'..tostring(slaxml))
	if not slaxml then return end
	local f = io.open(fname)
	local xmlContents = f:read('*all')
	local node = slaxml:dom(xmlContents)
	io.close(f)
--[[
	if node.kids ~= nil then
		for _,kid in pairs(node.kids) do
      print('?? for_kid:'.._)
    end
  end
]]
  for _,s in pairs({'library_images', 'library_materials', 'library_effects'}) do
    kidsList(node, 1, s) --, nil, nil, true)
  end
--    local doc =
--      if true then return end
--  local doc = kidsList(node, 1, 'material', nil, mat, true)
--  local doc = kidsList(node, 1, 'material', {1,2,3,4,5}, mat, true)
--      U.lo('?? pres:'..tostring(doc))
--  fname = '/tmp/bat/to.dae'
	local outputFile = io.open(fname, "w")
--        lo('?? matSet3:'..tostring(outputFile))
	if outputFile then
--      U.lo('?? pres2:'..tostring(doc.root))
		local xmlstr = slaxdom:xml(node.root)
--      U.lo('?? pres3:'..tostring(xmlstr))
		outputFile:write('<?xml version="1.0" encoding="utf-8"?>\n'..xmlstr)
		outputFile:close()
	end
      U.lo('<< matReplace:'..fname)
--      print('<< matReplace:'..fname)
end


local function matSet(fname, mat, aind, xml)
--        ind = 1
--    lo('>> matSet:'..fname..':'..mat..':'..#aind)
	if not slaxml then return end
	local f = io.open(fname)
	local xmlContents = f:read('*all')
	xml = slaxml:dom(xmlContents)
	io.close(f)
--    lo('?? matSet2:'..fname)

--    local doc = kidsList(xml, 1)
	local doc = kidsList(xml, 1, 'material', aind, mat)
--    if true then return end

	local outputFile = io.open(fname, "w")
--        lo('?? matSet3:'..tostring(outputFile))
	if outputFile then
		local xmlstr = slaxdom:xml(doc.root)
		outputFile:write('<?xml version="1.0" encoding="utf-8"?>\n'..xmlstr)
		outputFile:close()
--            lo('?? matSet_DONE:')
--        removeCDATA(file) --remove CDATA flags added by slaxml as this causes warnings when reading collada files
	end
--    lo('<< matSet:')
end


local doctemplate

local function save(ageom, fname) -- doc, fname)
		lo('>> save:'..#ageom..':'..tostring(doctemplate))
	fname = '/assets/_out/'..fname..'.dae'
	toKids(doctemplate.root, ageom)
	local outputFile = io.open(fname, "w")
	if outputFile then
		local xmlstr = slaxdom:xml(doctemplate.root)
		outputFile:write('<?xml version="1.0" encoding="utf-8"?>\n'..xmlstr)
		outputFile:close()
	end
end

-- limit points of crossing the mesh along the (pos,dir) ray
local function dissect(pos, dir, mdata)
	local av = mdata.verts
		lo('>> dissect:'..tostring(pos)..' dir:'..tostring(dir))
	local function forSimplex(i,j,mi,ma)
		local pos2,dir2 = av[mdata.faces[i].v+1], (av[mdata.faces[j].v+1]-av[mdata.faces[i].v+1]):normalized()
		local s,r = closestLinePoints(pos, dir, pos2, dir2)
		local d = (pos + dir*s - (pos2 + dir2*r)):length()
		if d < U.small_dist then
				lo(i..':'..mi..':'..ma..':'..d..':'..s..' pos2:'..tostring(pos2)..' dir2:'..tostring(dir2))
			if s < mi then
				mi = s
			end
			if s > ma then
				ma = s
			end
		end
		return mi,ma
	end
	local ma,mi = 0,math.huge
	for i=1,#mdata.faces,3 do
		mi,ma = forSimplex(i,i+1,mi,ma)
		mi,ma = forSimplex(i+1,i+2,mi,ma)
		mi,ma = forSimplex(i+2,i,mi,ma)
--[[
		local s1 = closestLinePoints(pos, dir, av[mdata.faces[i].v+1], av[mdata.faces[i+1].v+1] - av[mdata.faces[i].v+1])
		local s2 = closestLinePoints(pos, dir, av[mdata.faces[i+1].v+1], av[mdata.faces[i+2].v+1] - av[mdata.faces[i+1].v+1])
		local s3 = closestLinePoints(pos, dir, av[mdata.faces[i+2].v+1], av[mdata.faces[i].v+1] - av[mdata.faces[i+2].v+1])
			lo(s1..':'..s2..':'..s3)
		local smi = math.min(s1,s2,s3)
		local sma = math.max(s1,s2,s3)
		if smi < mi then
			mi = smi
		end
		if sma > ma then
			ma = sma
		end
]]
	end
		lo('?? dirIn:'..tostring(mi)..':'..ma)
end


local function faceHit(am)
	local ray = getCameraMouseRay()
--                local vn = vec3(0,0,1)
--                local d = intersectsRay_Plane(ray.pos, ray.dir, smouse, vn)
	local dmi,imi = math.huge
	for k,m in pairs(am) do
		local av = m.verts
		if m.faces then
			for i = 1,#m.faces,3 do
				local d, bx, by = intersectsRay_Triangle(
					ray.pos, ray.dir,
					av[m.faces[i].v+1], av[m.faces[i+1].v+1], av[m.faces[i+2].v+1]
				)
				if d < dmi then
					dmi = d
					imi = {k,i}
				end
			end
		end
	end
	return imi[1],imi[2]
end


local function forAMat(fname)
	amat = {}

	local xmlContents = io.open(fname):read('*all')
	local doc = slaxml:dom(xmlContents)

	kidsList(doc, 1)

	return amat
end


local function dae2proc(fname, showlist, step)
		U.dump(showlist, '>> dae2proc:'..fname)
--    if showlist == nil then showlist = {} end
	if step == nil then step = 9 end
	local xmlContents = io.open(fname):read('*all')
	local doc = slaxml:dom(xmlContents)
	doctemplate = doc
	xml2mesh(doc)

	local amesh = {}
	local ngeo = 0
	for key,g in pairs(afloat) do
		ngeo = ngeo + 1
		lo('?? dae2proc: geo:'..key..':'..#g..':'..#g[1]..' aint:'..#aint[key]) --..':'..#aint[1]..':'..#aint[2])
		local av,an,auv,af = {},{},{},{}
		for i = 1,#g[1],3 do
--            av[#av + 1] = vec3(g[1][i+1], g[1][i], g[1][i+2])
--            av[#av + 1] = vec3(g[1][i], g[1][i+2], g[1][i+1])
			av[#av + 1] = vec3(g[1][i], g[1][i+1], g[1][i+2])
		end
		if #g > 1 then
			for i = 1,#g[2],3 do
--                an[#an + 1] = vec3(g[2][i+1], g[2][i], g[2][i+2])
--                an[#an + 1] = vec3(g[2][i], g[2][i+2], g[2][i+1])
				an[#an + 1] = vec3(g[2][i], g[2][i+1], g[2][i+2])
			end
		end
		--        an = {vec3(0, 0, 1)}
		if #g > 2 then
			for i = 1,#g[3],2 do
--                auv[#auv + 1] = {u = g[3][i+1], v = g[3][i+0]}
				auv[#auv + 1] = {u = g[3][i], v = g[3][i + 1]}
			end
		else
			auv = {{u = 0, v = 0}}
		end

		for i,list in pairs(aint[key]) do
			lo('?? FOF:'..i..':'..key..':'..ngeo..':'..(#list/step))
			af = {}
			for i = 1,#list,step do
				if step == 9 then
					af[#af + 1] = {v = list[i], n = list[i+1], u = list[i+2]}
					af[#af + 1] = {v = list[i+3], n = list[i+3+1], u = list[i+3+2]}
					af[#af + 1] = {v = list[i+6], n = list[i+6+1], u = list[i+6+2]}

					af[#af + 1] = {v = list[i], n = list[i+1], u = list[i+2]}
					af[#af + 1] = {v = list[i+6], n = list[i+6+1], u = list[i+6+2]}
					af[#af + 1] = {v = list[i+3], n = list[i+3+1], u = list[i+3+2]}
				elseif step == 6 then
					af[#af + 1] = {v = list[i], n = list[i+1], u = 0}
					af[#af + 1] = {v = list[i+2], n = list[i+2+1], u = 0}
					af[#af + 1] = {v = list[i+4], n = list[i+4+1], u = 0}

					af[#af + 1] = {v = list[i], n = list[i+1], u = 0}
					af[#af + 1] = {v = list[i+4], n = list[i+4+1], u = 0}
					af[#af + 1] = {v = list[i+2], n = list[i+2+1], u = 0}
				end
--[[
				af[#af + 1] = {v = list[i], n = list[i+1], u = list[i+2]}
				af[#af + 1] = {v = list[i+3], n = list[i+3+1], u = list[i+3+2]}
				af[#af + 1] = {v = list[i+6], n = list[i+6+1], u = list[i+6+2]}

				af[#af + 1] = {v = list[i], n = list[i+1], u = list[i+2]}
				af[#af + 1] = {v = list[i+6], n = list[i+6+1], u = list[i+6+2]}
				af[#af + 1] = {v = list[i+3], n = list[i+3+1], u = list[i+3+2]}
]]
			end
			if showlist == nil or #U.index(showlist, ngeo) > 0 then
--            if showlist == nil or #U.index(showlist, i) > 0 then
				amesh[#amesh + 1] = {
					verts = av,
					normals = an,
					uvs = auv,
					faces = af,
					material = 'm_bricks_01',
				}
lo('??___ to_MESH:'..key..':'..i..':'..tostring(#amesh[#amesh].verts)..':'..tostring(#amesh[#amesh].normals)..':'..tostring(#amesh[#amesh].uvs)..':'..tostring(#amesh[#amesh].faces))
			end
		end
	end
--            amesh = {amesh[2]}
--    lo('?? dae2proc: vert:'..#afloat[1]) --..':'..#aint[1]..':'..#aint[2])
	lo('<< dae2proc:'..#amesh)
--            amesh = {amesh[1]}
--            amesh = {}
	return amesh
end
--[[
		lo('?? FOF:'..(#g[4]/9))
		local list = g[4]
		for i = 1,#g[4],9 do
			af[#af + 1] = {v = list[i], n = list[i+1], u = list[i+2]}
			af[#af + 1] = {v = list[i+3], n = list[i+3+1], u = list[i+3+2]}
			af[#af + 1] = {v = list[i+6], n = list[i+6+1], u = list[i+6+2]}

			af[#af + 1] = {v = list[i], n = list[i+1], u = list[i+2]}
			af[#af + 1] = {v = list[i+6], n = list[i+6+1], u = list[i+6+2]}
			af[#af + 1] = {v = list[i+3], n = list[i+3+1], u = list[i+3+2]}
		end
		amesh[#amesh + 1] = {
			verts = av,
			normals = an,
			uvs = auv,
			faces = af,
			material = 'm_bricks_01',
		}
]]


local function dae2proc_(fname)
--    lo('>> dae2proc:'..fname)
	local xmlContents = io.open(fname):read('*all')
	local doc = slaxml:dom(xmlContents)

	kidsList(doc, 1)
	local av,an,auv,af = {},{},{},{}
	for i = 1,#afloat[1],3 do
		av[#av + 1] = vec3(afloat[1][i],afloat[1][i+1],afloat[1][i+2])
	end
	for i = 1,#afloat[2],3 do
		an[#an + 1] = vec3(afloat[2][i],afloat[2][i+1],afloat[2][i+2])
	end
--        an = {vec3(0, 0, 1)}
	for i = 1,#afloat[3],2 do
		auv[#auv + 1] = {u = afloat[3][i], v = afloat[3][i + 1]}
	end
--        auv = {{u = 0, v = 0}}
		lo('?? anu:'..#av..':'..#an..':'..#auv)
	for i = 1,#aint[1],12 do
		af[#af + 1] = {v = aint[1][i], n = aint[1][i+1], u = aint[1][i+2]}
		af[#af + 1] = {v = aint[1][i+4], n = aint[1][i+4+1], u = aint[1][i+4+2]}
		af[#af + 1] = {v = aint[1][i+8], n = aint[1][i+8+1], u = aint[1][i+8+2]}

		af[#af + 1] = {v = aint[1][i], n = aint[1][i+1], u = aint[1][i+2]}
		af[#af + 1] = {v = aint[1][i+8], n = aint[1][i+8+1], u = aint[1][i+8+2]}
		af[#af + 1] = {v = aint[1][i+4], n = aint[1][i+4+1], u = aint[1][i+4+2]}
	end
	for i = 1,#aint[2],4 do
--        af[#af + 1] = {v = aint[2][i], n = aint[2][i+1], u = aint[2][i+2]}
	end

	local amesh = {}
	amesh[#amesh + 1] = {
		verts = av,
		faces = af,
		normals = an,
		uvs = auv,
		material = 'm_bricks_01',
	}

	af = {}
	for i = 1,#aint[2],12 do
		af[#af + 1] = {v = aint[2][i], n = aint[2][i+1], u = aint[2][i+2]}
		af[#af + 1] = {v = aint[2][i+4], n = aint[2][i+4+1], u = aint[2][i+4+2]}
		af[#af + 1] = {v = aint[2][i+8], n = aint[2][i+8+1], u = aint[2][i+8+2]}

		af[#af + 1] = {v = aint[2][i], n = aint[2][i+1], u = aint[2][i+2]}
		af[#af + 1] = {v = aint[2][i+8], n = aint[2][i+8+1], u = aint[2][i+8+2]}
		af[#af + 1] = {v = aint[2][i+4], n = aint[2][i+4+1], u = aint[2][i+4+2]}
	end
	for i = 1,#aint[2],4 do
--        af[#af + 1] = {v = aint[2][i], n = aint[2][i+1], u = aint[2][i+2]}
	end
	amesh[#amesh + 1] = {
		verts = av,
		faces = af,
		normals = an,
		uvs = auv,
		material = 'm_metal_brushed',
	}



	for i = 1,#aint[1],12 do
		--        af[#af + 1] = {v = aint[1][i], n = aint[1][i + 1], u = 1}
		--        af[#af + 1] = {v = aint[1][i], n = aint[1][i + 1], u = aint[1][i + 2]}
		--            U.dump(af[#af], '?? for_tri:'..i)
	end
	for i = 1,#aint[2],12 do
--        af[#af + 1] = {v = aint[2][i], n = 0, u = 0}
--        af[#af + 1] = {v = aint[2][i], n = aint[2][i + 1], u = 1}
--        af[#af + 1] = {v = aint[2][i], n = aint[2][i + 1], u = aint[2][i + 2]}
--            U.dump(af[#af], '?? for_tri:'..i)
	end
	lo('<< dae2proc:'..#amesh)
	return amesh -- av,an,auv,af
end


local function update(tmesh)
	local amesh = {}
	for ord,m in pairs(tmesh.data) do
		amesh[#amesh + 1] = m
		if tmesh.buf ~= nil then
			amesh[#amesh + 1] = tmesh.buf[ord]
		end
		if tmesh.trans ~= nil then
			amesh[#amesh + 1] = tmesh.trans[ord]
		end
		if tmesh.sel ~= nil then
			amesh[#amesh + 1] = tmesh.sel[ord]
		end
	end
	tmesh.obj:createMesh({})
	tmesh.obj:createMesh({amesh})
end


local function clone(tmesh)
	local avi,ani,auvi = {},{},{}
	for ord,m in pairs(tmesh.sel) do
		for _,f in pairs(m.faces) do
			if #U.index(avi, f.v) == 0 then
				avi[#avi + 1] = f.v
			end
			if #U.index(ani, f.n) == 0 then
				ani[#ani + 1] = f.n
			end
			if #U.index(auvi, f.u) == 0 then
				auvi[#auvi + 1] = f.u
			end
		end
	end
	table.sort(avi)
	local dvi,dni,duvi = {},{},{}
	local verts,normals,uvs = {},{},{}
	-- TODO: other geometries?
	local m = tmesh.sel[1]
	for o,i in pairs(avi) do
		dvi[i] = o
		verts[#verts+1] = m.verts[i+1] + vec3(-2,0,0)
	end
--            lo('?? vrts:'..tostring(verts[1])..':'..tostring(m.verts[dvi[avi[1]]]))
	for o,i in pairs(ani) do
		dni[i] = o
		normals[#normals+1] = m.normals[i+1]
	end
	for o,i in pairs(auvi) do
		duvi[i] = o
		uvs[#uvs+1] = m.uvs[i+1]
	end
--                U.dump(auvi, '?? UV:')
--                U.dump(duvi, '?? UV:')
--            lo('?? avi:'..#avi..':'..#verts..':'..#normals..':'..#uvs)
	local amesh = {}
	local afaces = {}
	for ord,m in pairs(tmesh.sel) do
		afaces[ord] = {}
		for _,f in pairs(m.faces) do
--                    U.dump(f, '?? for_f:'.._..':'..tostring(dvi[f.v])..':'..tostring(dni[f.n])..':'..tostring(duvi[f.u]))
			afaces[ord][#afaces[ord]+1] = {v = dvi[f.v]-1, n = dni[f.n]-1, u = duvi[f.u]-1}
		end
		amesh[#amesh + 1] = {
			verts = verts,
			normals = normals,
			uvs = uvs,
			material = 's_cyan',
			faces = afaces[ord],
		}
	end
--                U.dump(amesh, '?? AMESH:'..#dmesh[cmesh].sel)
--                U.dump(amesh[1].verts, '?? AMESH1:'..#dmesh[cmesh].sel)
--                U.dump(amesh[1].normals, '?? AMESH2:'..#dmesh[cmesh].sel)
--                U.dump(amesh[1].uvs, '?? AMESH3:'..#dmesh[cmesh].sel)
--                U.dump(amesh[1].faces, '?? AMESH4:'..#dmesh[cmesh].sel)
	return amesh
end


local function move(list, fr, to, mat, mode)
	if mat == nil and list ~= nil then mat = 'WarningMaterial' end
	for ord,m in pairs(fr) do
		if mode ~= -1 and to[ord] == nil then
			to[ord] = {
				verts = m.verts,
				normals = m.normals,
				uvs = m.uvs,
				material = mat,
				faces = {},
			}
		end
--            U.dump(list[ord], '?? move:'..ord)
		if list == nil then
				lo('?? move:'..tostring(fr[ord].material)..':'..tostring(to[ord].material))
			if mat == nil and fr[ord].material ~= to[ord].material then
				-- append part
				to[#to + 1] = fr[ord]
			else
				for _,f in pairs(fr[ord].faces) do
					to[ord].faces[#to[ord].faces + 1] = f
				end
--                fr[ord].faces = {}
			end
			fr[ord] = nil
		elseif list[ord] ~= nil then
			for _,i in pairs(list[ord]) do
				if mode ~= -1 then
					to[ord].faces[#to[ord].faces + 1] = m.faces[i+0]
					to[ord].faces[#to[ord].faces + 1] = m.faces[i+1]
					to[ord].faces[#to[ord].faces + 1] = m.faces[i+2]
					to[ord].faces[#to[ord].faces + 1] = m.faces[i+3]
					to[ord].faces[#to[ord].faces + 1] = m.faces[i+4]
					to[ord].faces[#to[ord].faces + 1] = m.faces[i+5]
				end
				if mode ~= 1 then
					table.remove(m.faces, i+5)
					table.remove(m.faces, i+4)
					table.remove(m.faces, i+3)
					table.remove(m.faces, i+2)
					table.remove(m.faces, i+1)
					table.remove(m.faces, i)
				end
			end
		end
--        local af = list ~= nil and list[ord] or fr[ord].faces
	end
end


local function copy(list, fr, to, mat)
	move(list, fr, to, mat, 1)
end


local function cut(list, fr, to, mat)
	move(list, fr, to, mat, -1)
end


local function mark(mdata, out)
	out.avedit = {}
	for ord,m in pairs(mdata) do
		for _,f in pairs(m.faces) do
			out.avedit[#out.avedit + 1] = m.verts[f.v + 1]
		end
	end
end


local function ifHit(adata, pfr, pto, forall)
	if not pfr then
		local rayCast = cameraMouseRayCast(true)
		pfr = core_camera.getPosition()
		pto = rayCast.pos
	end
--        U.dump(mdata[1], '>> ifHit:'..#mdata..':'..)
	local mi,imi = math.huge,0
	for ord,m in pairs(adata) do
		local av = m.verts
--                U.dump(m.faces, '?? ifHit:'..#m.verts..':'..#m.faces)
		for i  = 1,#m.faces,3 do
--        for i  = 1,#m.faces,6 do
--            local d, bx, by = intersectsRay_Triangle(
--    core_camera.getPosition(), rayCast.pos - core_camera.getPosition(),
			local d, bx, by = intersectsRay_Triangle(
	pfr, pto - pfr,
	av[m.faces[i].v+1], av[m.faces[i+1].v+1], av[m.faces[i+2].v+1]
			)
--                if dbg then
--                    lo('?? for_hit:'..i..' d:'..tostring(d)..':'..tostring(av[m.faces[i].v+1])..':'..tostring(av[m.faces[i+1].v+1])..':'..tostring(av[m.faces[i+2].v+1]))
--                end
			if d ~= math.huge then
				if forall then
					if d < mi then
						mi = d
						imi = ord
					end
				else
--                    lo('?? forVerts_HIT:'..i)
					return ord,d
				end
			end
		end
	end
	return imi,mi
end


local function pop(fr, to)
		lo('>> pop:')
	local sto = to
	local rayCast = cameraMouseRayCast(true)
	local aimi = {}
	local amesh = {}
	local flip = false
	local lastord
	local dmi,imi = math.huge
	for ord,m in pairs(fr) do
		local av = m.verts

		-- check main
	--        local dmi, imi = math.huge
		for i  = 1,#m.faces,6 do
			local d, bx, by = intersectsRay_Triangle(
	core_camera.getPosition(), rayCast.pos - core_camera.getPosition(),
	av[m.faces[i].v+1], av[m.faces[i+1].v+1], av[m.faces[i+2].v+1]
			)
			if d ~= math.huge and d < dmi then
				dmi = d
				imi = i
				lastord = ord
			end
		end

		-- check trans


		-- check select
		local mto = to[ord]
		if mto ~= nil then
			lo('?? INV:'..ord)
			for i = 1,#mto.faces,6 do
				local f = mto.faces[i]
				local d, bx, by = intersectsRay_Triangle(
	core_camera.getPosition(), rayCast.pos - core_camera.getPosition(),
	av[mto.faces[i].v+1], av[mto.faces[i+1].v+1], av[mto.faces[i+2].v+1]
				)
				if d ~= math.huge and d < dmi then
					dmi = d
					imi = i
					flip = true
					lastord = ord
					lo('?? TO_FLIP:'..imi)
				end
			end
		end
		amesh[#amesh + 1] = m
	end
	if lastord == nil then
		lo('!! pop_NOSELECT:')
		return
	end

	table.insert(aimi, lastord, {imi})
			U.dump(aimi, '?? aimi:')
	local path,tri,triord = {},{}
	if flip then
		-- restore
		move(aimi, to, fr)
	else
		-- select
		triord = lastord
		tri[#tri + 1] = fr[lastord].faces[imi+0]
		tri[#tri + 1] = fr[lastord].faces[imi+1]
		tri[#tri + 1] = fr[lastord].faces[imi+2]

		move(aimi, fr, to, 's_yellow')
	end
	amesh[#amesh + 1] = to[lastord]  -- need to be after 'move'

	local center,n = vec3(0,0,0),0
	for _,f in pairs(tri) do
		path[#path + 1] = sto[triord].verts[f.v+1]
		center = center + sto[triord].verts[f.v+1]
	end
	if #tri == 3 then
		path[#path + 1] = sto[triord].verts[tri[1].v+1]
	end
	center = center/3
	--        lo('<< pop:'..#amesh[2].faces)

	return amesh,center,path
end


local function pop___(fr, to)
		lo('>> pop:')
	local sto = to
	local rayCast = cameraMouseRayCast(true)
	local amesh = {}
	local tri,triord = {}
	for ord,m in pairs(fr) do
		local av = m.verts

		-- check main
		local dmi,imi = math.huge
	--        local dmi, imi = math.huge
		local flip = false
		for i  = 1,#m.faces,6 do
			local d, bx, by = intersectsRay_Triangle(
	core_camera.getPosition(), rayCast.pos - core_camera.getPosition(),
	av[m.faces[i].v+1], av[m.faces[i+1].v+1], av[m.faces[i+2].v+1]
			)
			if d ~= math.huge and d < dmi then
				dmi = d
				imi = i
			end
		end

		-- check select
		local mto = to[ord]
		if mto ~= nil then
			lo('?? INV:')
			for i = 1,#mto.ref,6 do
				local f = mto.ref[i]
				local d, bx, by = intersectsRay_Triangle(
	core_camera.getPosition(), rayCast.pos - core_camera.getPosition(),
	av[mto.ref[i].v+1], av[mto.ref[i+1].v+1], av[mto.ref[i+2].v+1]
				)
				if d ~= math.huge and d < dmi then
					dmi = d
					imi = i
					flip = true
					lo('?? TO_FLIP:'..imi)
				end
			end
		end

		if imi ~= nil then
			if flip then
				-- restore
				lo('?? BACK:'..imi)
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi+0]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi+1]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi+2]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi+3]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi+4]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi+5]
				table.remove(to[ord].ref, imi)
				table.remove(to[ord].ref, imi)
				table.remove(to[ord].ref, imi)
				table.remove(to[ord].ref, imi)
				table.remove(to[ord].ref, imi)
				table.remove(to[ord].ref, imi)
			else
				-- select
				triord = ord
				tri[#tri + 1] = m.faces[imi+0]
				tri[#tri + 1] = m.faces[imi+1]
				tri[#tri + 1] = m.faces[imi+2]

				move({{imi}}, fr, to, 's_yellow')

--[[
				if to[ord] == nil then
					to[ord] = {
						verts = m.verts,
						normals = m.normals,
						uvs = m.uvs,
						material = 's_yellow',
						faces = {},
						ref = {},
					}
				end
						lo('?? pop.to_ref:'..imi)
	--            U.dump(m.faces[imi], '?? to>:'..imi)
				local ref = to[ord].ref
				ref[#ref + 1] = m.faces[imi]
				ref[#ref + 1] = m.faces[imi+1]
				ref[#ref + 1] = m.faces[imi+2]
				ref[#ref + 1] = m.faces[imi+3]
				ref[#ref + 1] = m.faces[imi+4]
				ref[#ref + 1] = m.faces[imi+5]
				table.remove(m.faces, imi+5)
				table.remove(m.faces, imi+4)
				table.remove(m.faces, imi+3)
				table.remove(m.faces, imi+2)
				table.remove(m.faces, imi+1)
				table.remove(m.faces, imi)
]]
			end
		end
		-- copy ref to sel
--[[
		to[ord].faces = {}
		for _,f in pairs(to[ord].ref) do
			to[ord].faces[#to[ord].faces + 1] = f
		end
]]
		amesh[#amesh + 1] = m
		if imi ~= nil then
			amesh[#amesh + 1] = to[ord]
		end
	end
	local center,n = vec3(0,0,0),0
	for _,f in pairs(tri) do
		center = center + sto[triord].verts[f.v+1]
	end
	center = center/3
--        lo('<< pop:'..#amesh[2].faces)

	return amesh, center
end


local function pop__(fr, to, oind, ipop)
		lo('>> pop:')
	local sto = to
	local rayCast = cameraMouseRayCast(true)
	local amesh = {}
	local tri,triord = {}
	for ord,m in pairs(fr) do
		local av = m.verts

		-- check main
		local dmi,imi = math.huge
	--        local dmi, imi = math.huge
		local flip = false
		for i  = 1,#m.faces,6 do
			local d, bx, by = intersectsRay_Triangle(
	core_camera.getPosition(), rayCast.pos - core_camera.getPosition(),
	av[m.faces[i].v+1], av[m.faces[i+1].v+1], av[m.faces[i+2].v+1]
			)
			if d ~= math.huge and d < dmi then
				dmi = d
				imi = i
			end
		end

		-- check select
		local mto = to[ord]
		if mto ~= nil then
			lo('?? INV:')
			for _,t in pairs(mto.ref) do
				local d, bx, by = intersectsRay_Triangle(
	core_camera.getPosition(), rayCast.pos - core_camera.getPosition(),
	av[t[1].v+1], av[t[2].v+1], av[t[3].v+1]
				)
				if d ~= math.huge and d < dmi then
					dmi = d
					imi = _
					flip = true
					lo('?? TO_FLIP:'.._)
				end
			end
		end

		if imi ~= nil then
			if flip then
				-- restore
				lo('?? BACK:'..imi)
--[[
				table.insert(fr[ord].faces, imi, to[ord].ref[imi][1])
				table.insert(fr[ord].faces, imi+1, to[ord].ref[imi][2])
				table.insert(fr[ord].faces, imi+1, to[ord].ref[imi][3])
				table.insert(fr[ord].faces, imi+1, to[ord].ref[imi][4])
				table.insert(fr[ord].faces, imi+1, to[ord].ref[imi][5])
				table.insert(fr[ord].faces, imi+1, to[ord].ref[imi][6])
]]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi][1]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi][2]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi][3]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi][4]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi][5]
				fr[ord].faces[#fr[ord].faces + 1] = to[ord].ref[imi][6]
				to[ord].ref[imi] = nil
			else
				-- select
				if to[ord] == nil then
					to[ord] = {
						verts = m.verts,
						normals = m.normals,
						uvs = m.uvs,
						material = 's_yellow', --'WarningMaterial', -- 'm_stone_fence', -- 'm_metal_frame_trim_01',
						faces = {},
						ref = {},
					}
				end
				triord = ord
				tri[#tri + 1] = m.faces[imi+0]
				tri[#tri + 1] = m.faces[imi+1]
				tri[#tri + 1] = m.faces[imi+2]
						lo('?? pop.to_ref:'..imi)
	--            U.dump(m.faces[imi], '?? to>:'..imi)

				to[ord].ref[imi] = {}
				local ref = to[ord].ref[imi]
				ref[#ref + 1] = m.faces[imi]
				ref[#ref + 1] = m.faces[imi+1]
				ref[#ref + 1] = m.faces[imi+2]
				ref[#ref + 1] = m.faces[imi+3]
				ref[#ref + 1] = m.faces[imi+4]
				ref[#ref + 1] = m.faces[imi+5]
				table.remove(m.faces, imi+5)
				table.remove(m.faces, imi+4)
				table.remove(m.faces, imi+3)
				table.remove(m.faces, imi+2)
				table.remove(m.faces, imi+1)
				table.remove(m.faces, imi)
--[[
				m.faces[imi+5] = nil
				m.faces[imi+4] = nil
				m.faces[imi+3] = nil
				m.faces[imi+2] = nil
				m.faces[imi+1] = nil
				m.faces[imi+0] = nil
]]
			end
		end
		to[ord].faces = {}
		for key,list in pairs(to[ord].ref) do
			for _,f in pairs(list) do
				to[ord].faces[#to[ord].faces + 1] = f
			end
		end
		amesh[#amesh + 1] = m
		if imi ~= nil then
			amesh[#amesh + 1] = to[ord]
		end
	end
	local center,n = vec3(0,0,0),0
	for _,f in pairs(tri) do
		center = center + sto[triord].verts[f.v+1]
	end
	center = center/3

	return amesh, center
end
--[[
			for i  = 1,#mto.faces,6 do
				local d, bx, by = intersectsRay_Triangle(
	core_camera.getPosition(), rayCast.pos - core_camera.getPosition(),
	av[mto.faces[i].v+1], av[mto.faces[i+1].v+1], av[mto.faces[i+2].v+1]
				)
				if d ~= math.huge and d < dmi then
					dmi = d
					imi = i
					flip = true
					lo('?? TO_FLIP:'..i)
				end
			end
]]
--[[
				to[ord].faces[#to[ord].faces+1] = m.faces[imi]
				to[ord].faces[#to[ord].faces+1] = m.faces[imi+1]
				to[ord].faces[#to[ord].faces+1] = m.faces[imi+2]
				to[ord].faces[#to[ord].faces+1] = m.faces[imi+3]
				to[ord].faces[#to[ord].faces+1] = m.faces[imi+4]
				to[ord].faces[#to[ord].faces+1] = m.faces[imi+5]
]]
--[[
			local istart = #to[ord].faces + 1
			table.insert(to[ord].faces, istart, m.faces[imi])
			table.insert(to[ord].faces, istart+1, m.faces[imi+1])
			table.insert(to[ord].faces, istart+2, m.faces[imi+2])
			table.insert(to[ord].faces, istart+3, m.faces[imi+3])
			table.insert(to[ord].faces, istart+4, m.faces[imi+4])
			table.insert(to[ord].faces, istart+5, m.faces[imi+5])
]]


local function pop_(fr, to, oind, ipop)
		lo('>> pop:')
	local sto = to
	local rayCast = cameraMouseRayCast(true)
	local amesh = {}
	local tri,triord = {}
	for ord,m in pairs(fr) do
		local av = m.verts

		-- check main
		local dmi,imi = math.huge
--        local dmi, imi = math.huge
		local flip = false
		for i  = 1,#m.faces,6 do
			local d, bx, by = intersectsRay_Triangle(
core_camera.getPosition(), rayCast.pos - core_camera.getPosition(),
av[m.faces[i].v+1], av[m.faces[i+1].v+1], av[m.faces[i+2].v+1]
			)
			if d ~= math.huge and d < dmi then
				dmi = d
				imi = i
			end
		end

		-- check select
		local mto = to[ord]
		if mto ~= nil then
			for i  = 1,#mto.faces,6 do
				local d, bx, by = intersectsRay_Triangle(
	core_camera.getPosition(), rayCast.pos - core_camera.getPosition(),
	av[mto.faces[i].v+1], av[mto.faces[i+1].v+1], av[mto.faces[i+2].v+1]
				)
				if d ~= math.huge and d < dmi then
					dmi = d
					imi = i
					flip = true
					lo('?? TO_FLIP:'..i)
				end
			end
		end
		if flip then
			m = to[ord]
			to = fr
--            fr = sto
		end
--[[
		if oind == nil then
		elseif ord == oind then
			imi = ipop
		end
]]
		if imi ~= nil then
			if to[ord] == nil then
				to[ord] = {
					verts = m.verts,
					normals = m.normals,
					uvs = m.uvs,
					material = 'WarningMaterial', -- 'm_stone_fence', -- 'm_metal_frame_trim_01',
					faces = {},
					ref = {},
				}
			end
				lo('?? to_pop:'..tostring(flip)..':'..imi..':'..#m.faces)
			triord = ord
			tri[#tri + 1] = m.faces[imi+0]
			tri[#tri + 1] = m.faces[imi+1]
			tri[#tri + 1] = m.faces[imi+2]
			U.dump(m.faces[imi], '?? to>:'..imi)
			local istart = #to[ord].faces + 1
--            if flip then istart = 9529 end
			table.insert(to[ord].faces, istart, m.faces[imi])
			table.insert(to[ord].faces, istart+1, m.faces[imi+1])
			table.insert(to[ord].faces, istart+2, m.faces[imi+2])
			table.insert(to[ord].faces, istart+3, m.faces[imi+3])
			table.insert(to[ord].faces, istart+4, m.faces[imi+4])
			table.insert(to[ord].faces, istart+5, m.faces[imi+5])
			if not flip then
				to[ord].ref[istart] = imi
			end
					U.dump(to[ord].ref, '?? REF:')
--[[
			to[ord].faces[#to[ord].faces+1] = m.faces[imi]
--            U.dump(m.faces[imi+1], '?? to>:'..imi)
			to[ord].faces[#to[ord].faces+1] = m.faces[imi+1]
--            U.dump(m.faces[imi+2], '?? to>:'..imi)
			to[ord].faces[#to[ord].faces+1] = m.faces[imi+2]
			to[ord].faces[#to[ord].faces+1] = m.faces[imi+3]
			to[ord].faces[#to[ord].faces+1] = m.faces[imi+4]
			to[ord].faces[#to[ord].faces+1] = m.faces[imi+5]
]]
			table.remove(m.faces, imi+5)
			table.remove(m.faces, imi+4)
			table.remove(m.faces, imi+3)
			table.remove(m.faces, imi+2)
			table.remove(m.faces, imi+1)
			table.remove(m.faces, imi)
				lo('?? post_pop:'..imi..':'..#to[ord].faces..':'..#m.faces)
		end
		amesh[#amesh + 1] = m
		if imi ~= nil then
			amesh[#amesh + 1] = to[ord]
		end
	end
	local center,n = vec3(0,0,0),0
	for _,f in pairs(tri) do
		center = center + sto[triord].verts[f.v+1]
	end
	center = center/3
--[[
	for _,m in pairs(sto) do
--        for i = 1,3 do
--            local f = m.faces[i]
		for i,f in pairs(m.faces) do
			center = center + m.verts[f.v+1]
			n = n + 1
--                lo('?? for_vert:'..n..':'..tostring(m.verts[f.v]))
		end
	end
		lo('?? pop:'..n..':'..tostring(center/n))
]]
	return amesh, center
end


M.rcPave__ = function(ae, aloop, mrc)
	if not mrc then
		-- start, scale
		mrc = {{0,0}, {1,1}}
	end
	local av,ae = {},{}
	-- loops to edges
	for j,loop in pairs(aloop) do

	end
	-- edges to verts and stars
	-- cut off intersections with holes
	for i,e in pairs(ae) do
		for j,loop in pairs(aloop) do

		end
	end
	-- sprouts for convex stars
	-- polygons
	-- to mesh
end


M.rcPave = function(base, aloop, mrc, dbg)
--	    U.lo('>> rcPave:',true)
	if not mrc then
		-- start, scale
		mrc = {{0,0}, {1,1}}
	end
	local av = {} -- {pos, {star}}
	local ae = {}
	local iemax = 1

	local function edgeUp(v1,v2)
--		ae[tableSize(ae)+1] = {v1,v2}
		ae[iemax+1] = {v1,v2}
		iemax = iemax + 1
--		ae[#ae+1] = {v1,v2}
		av[v1].star[v2] = 0
		av[v2].star[v1] = 0
	end
	local function edgeDown(ie)
		av[ae[ie][1]].star[ae[ie][2]] = nil
		av[ae[ie][2]].star[ae[ie][1]] = nil
		ae[ie] = nil
--        table.remove(ae, ie)
	end
	local function v2e(iv, ie)
		-- add
		edgeUp(ae[ie][1],iv)
		edgeUp(ae[ie][2],iv)
		edgeDown(ie)
	end
	-- inital network
	local astick = {} -- vertices which lie on boundary
	for i,b in pairs(base) do
		av[#av+1] = {p=b,star={}}
--		astick[#astick+1] = i
	end
	for i=1,#base do
		edgeUp(i,U.mod(i+1,#base))
--        ae[#ae+1] = {#av,#av+1}
	end

  	local aloopstamp = {}
	local aloopind = {}
	local aestamp = {}
	for _,l in pairs(aloop) do
--[[
    local pth = {}
    for _,iv in pairs(l) do
      pth[#pth+1] = av[iv].p
    end
    local pth,map = U.polyStraighten(pth)
      U.dump(map,'?? straihtned:'..#p..'/'..#pth)
    local iseq = {}
    for j,v in pairs(pth) do
      iseq[#iseq+1] = p[map[j] ]
    end
]]
		local pth = {}
		local p = {}
		for i,b in pairs(l) do
			av[#av+1] = {p=b,star={}}
			p[#p+1] = #av
			pth[#pth+1] = b
		end
		local pth,map = U.polyStraighten(pth)
	--      U.dump(map,'?? straihtned:'..#p..'/'..#pth)
		local iseq = {}
		for j,v in pairs(pth) do
			iseq[#iseq+1] = p[map[j]]
		end
	--        U.dump(iseq, '??++++++ for_loop:'.._)
		aloopstamp[#aloopstamp+1] = U.stamp(iseq)

		local start = #av-#l
--		local nstamp = 1
		for i=start+1,start+#l do
			local stamp = U.stamp({i,start+U.mod(i+1-start,#l)})
--				U.lo('?? for_STAMP:'..stamp..':'..tostring(#U.index(aestamp,stamp))..':'..#aestamp)
			if #U.index(aestamp,stamp) == 0 then
				edgeUp(i,start+U.mod(i+1-start,#l))
				aestamp[#aestamp+1] = stamp
--				aestamp[nstamp+#base] = stamp
--				nstamp = nstamp + 1
--				aestamp[#aestamp+1+#base] = stamp
			end
		end
		aloopind[#aloopind+1] = p
--			U.dump(p, '?? for_P:'.._)
	end
--	      U.dump(aloopind, '?? loopinds:')
--	      U.dump(aestamp, '?? aestamp:')
	--      U.dump(aloopstamp, '?? stamps:')
	--            U.dump(av, '?? av:')
	--            U.dump(ae, '?? ae:')

	local albl = {}
	for i = 1,#av do
		albl[#albl+1] = {av[i].p, i}
	end

	-- restrict loops to base area
	local aebound = {}
	for i=1,#base do
		aebound[#aebound+1] = i
	end

	local function crossBoundary(loop, k)
		for i,ie in pairs(aebound) do
--                lo('?? onBound:'..)
			if ae[ie] then
				local p = U.line2seg(U.mod(k,loop), U.mod(k+1,loop),av[ae[ie][1]].p,av[ae[ie][2]].p)
	--                U.dump(ae[ie],'?? if_p:'..k..':'..i..':'..tostring(p)..':'..tostring(loop[k])..'>'..tostring(U.mod(k+1,loop))..':'..tostring(av[ae[ie][1]].p)..'>'..tostring(av[ae[ie][2]].p))
				if p and U.line2seg(av[ae[ie][1]].p, av[ae[ie][2]].p, U.mod(k,loop), U.mod(k+1,loop)) then
	--				U.dump(ae[ie],'?? HIT:'..tostring(i))
					return p,i
				else
--					U.lo('!! ERR_crossBoundary:'..k)
				end
			end
		end
--      lo('<< onBoundary:'..k)
	end

	local function onBoundary(p)
		if not p then return end
		for i,ie in pairs(aebound) do
--				U.dump(ae[ie],'?? onBoundary:'..i..':'..ie)
			if ae[ie] then
				if U.between(av[ae[ie][1]].p, av[ae[ie][2]].p, {p}) then
					return ie
				end
			end
		end
	end

	local iv = #base
	local start = #base
	for i,loop in pairs(aloop) do
--			U.dump(loop, '?? for_loop:'..i) --..':'..#loop..':'..tableSize(loop))
		local isout,irc
		local onboundary = {}
		local outfirst
		local iecheck = {}
		local aedown = {}
--            U.dump(av,'?? pre_loop:'..i)
		local ifr,ito
		for k=#loop,0,-1  do
			iv = iv + 1
			local b = U.mod(k,loop)
--			local p,ind = onBoundary(loop,k)
--			if p then
--				U.lo('?? on_BD:'..i..':'..k)
--			else

			local ib = onBoundary(loop[k])
--				U.lo('?? if_IN:'..i..':'..k..':'..tostring(U.inRC(b, {base}))..':'..tostring(ib))
			if ib then
--					U.dump('?? on_BOUNDD:'..i..':'..k)
--					if i~=2 or k~=1 then
--						U.dump(ae, '?? ae:')
--						U.dump(aebound, '?? aebound:')
--					end
--				if i~=2 or k~=1 then
--				if i == 1 and k == 2 then
--					U.lo('?? if_bound:'..i..':'..k..':'..tostring(loop[k])..':'..#aebound..':'..tostring(ib))
--				if #aebound < 10 and i < 3 then
				v2e(aloopind[i][k], ib)
				astick[#astick+1] = start + k
				table.remove(aebound, ib)
				aebound[#aebound+1] = #ae-1
				aebound[#aebound+1] = #ae
--						U.dump(aebound, '?? aebound:')
			elseif not U.inRC(b, {base}) then --, nil, true) then
					U.lo('?? is_OUT:'..i..':'..k..':'..tostring(loop[k])..':'..tostring(isout))
--			if not U.inRC(b, {base}) then
--			elseif not U.inRC(b, {base}, nil, true) then
--					U.lo('?? on_B:'..k..':'..tostring(loop[k]))
				if not isout then
--                        lo('?? if_OUT:'..k..':'..tostring(loop[k]))
					local p,ind = crossBoundary(loop,k)
--						U.lo('?? if_P:'..tostring(p))
--                        lo('?? for_OUT:'..k..':'..tostring(p)..':'..ind..':'..#aebound)
					if p then --and (U.mod(k,loop)-p):length()>0 then
						local ie = aebound[ind]
--								U.dump(aebound, '?? on_BOUND:'..k..':'..tostring(U.mod(k,loop))..':'..ind..':'..tostring(p))
						av[start+k].p = p
						v2e(start+k, ie)
						astick[#astick+1] = start+k
--                            U.dump(ae[aebound[ie]],'?? onB:'..k..':'..tostring(p)..'>'..ie..':'..ind..':'..(start+k)) --tostring(av[iv].p))
						-- update aebound
						table.remove(aebound, ind)
						--iemax
						aebound[#aebound+1] = iemax-1
						aebound[#aebound+1] = iemax
--						aebound[#aebound+1] = #ae-1
--						aebound[#aebound+1] = #ae
						ifr = aloopind[i][k]
--						if i == 1 then
--							U.dump(ae, '?? post_CROSS_:'..i..':'..k)
--							U.dump(aebound, '?? post_CROSS_aeb_:'..i..':'..k)
--						end
--							U.lo('?? iFR:'..i..':'..k..':'..tostring(ifr))
--[[
						if true then
	--                            lo('?? outSTART:'..i..' k:'..k..' start:'..start..':'..#av)
	--                        loop[k] = p
						elseif false and k > 0 then
							v2e(aloopind[i][k], ie)
							-- update aebound
							table.remove(aebound, ind)
							aebound[#aebound+1] = #ae-1
							aebound[#aebound+1] = #ae
								U.dump(ae, '?? ae_pp:')
						end
								U.dump(aebound, '?? on_BOUND_out:'..k)
]]
--                            U.dump(ae, '?? AE:')
--                            U.dump(aebound, '?? aebound:')
--                        table.remove(aebound
--					else
--						isout = k
					end
--                elseif k<outfirst-1 then
					--TODO:
--                    table.remove(loop, k-1)
				end
				isout = k
			else
--					U.lo('?? tobound1:'..k..':'..tostring(isout))
				if isout and k+1 == isout then
--					if i == 2 then
--						U.dump(ae, '?? pre_CROSS:'..i..':'..k)
--						U.dump(aebound, '?? pre_CROSS_aeb:'..i..':'..k)
--					end
					local p,ind = crossBoundary(loop,k)
--						U.dump(ae, '?? tobound2_AE:'..i..':'..k..':'..ind) --..':'..(start+k+1))
						U.dump(ae[aebound[ind]], '?? tobound2:'..i..':'..k..':'..tostring(p)) --..':'..ind..':'..ifr..':'..isout) --..(start+k+1))
					ito = aloopind[i][isout]
--						U.dump(aestamp, '?? tobound2:'..i..':'..k..':'..ind..':'..ifr..'>'..ito) --..(start+k+1))
					if p then
						-- remove edge
						local dstamp = U.stamp({ifr,ito})
						local idupe -- = start+U.mod(#loop-k+1,#loop)
						for ii,e in pairs(ae) do
							if dstamp == U.stamp(e) then
								idupe = ii
							end
						end
--                            U.dump(ae[iv],'?? DDOWN:'..start..':'..k..':'..idupe..':'..#ae)
							U.dump(ae[idupe],'?? to_DOWN:')
						edgeDown(idupe)
						-- update position
--                            lo('?? last_OUT:'..ind..':'..iv)
						av[start+k+1].p = p
						astick[#astick+1] = start+k+1
						local ie = aebound[ind]
						v2e(start+k+1, ie)
						-- update aebound
						table.remove(aebound, ind)
						aebound[#aebound+1] = iemax-1
						aebound[#aebound+1] = iemax
--						aebound[#aebound+1] = #ae-1
--						aebound[#aebound+1] = #ae

						if i == 2 then
--							U.dump(ae, '?? post_CROSS:'..i..':'..k)
--							U.dump(aebound, '?? post_CROSS_aeb:'..i..':'..k)
						end

--						isout = nil
--                            U.dump(aebound, '?? aebound:')
					end
				end
			end
		end
		start = start + #loop
--        U.dump(loop, '?? loop_OUT:')
	end
--            U.dump(av, '?? av_post')
--			U.dump(astick, '?? vSTICK:')
--            U.dump(ae, '?? ae_post:')

	local aline = {}
--[[
	for i,e in pairs(ae) do
		aline[#aline+1] = {av[e[1] ].p,av[e[2] ].p}
	end
]]
--    lo('?? STICK:'..#astick)
	------------------------
	-- make sprouts
	------------------------
	local function forSprouts(v)
		if tableSize(v.star) > 2 then return {} end
		return v.star
	end

	for i,v in pairs(av) do
		if U.index(astick,i)[1] then goto continue end
		local p = v.p
		local dmi,pmi,emi,smi=math.huge
--		for iv,_ in pairs(v.star) do
--	        U.dump(forSprouts(v),'?? if_V:'..i)
		local asprout = forSprouts(v)
--			if i == 12 then
--	            U.dump(v.star, '?? for_star:'..i)
--	            U.dump(asprout, '?? for_sprout:'..i)
--			end
		for iv,_ in pairs(asprout) do
			for k,e in pairs(ae) do
--					if i == 12 then
--						U.dump(e, '?? for_eS:'..k)
--					end
				if v.star[e[1]] or v.star[e[2]] or av[iv].star[e[1]] or av[iv].star[e[2]] then
				else
					-- check intersection
					local pc,s = U.line2seg(p,av[iv].p,av[e[1]].p,av[e[2]].p,0.00001)
--							U.dump(e,'?? for_dtep:'..k..':'..tostring(p)..'>'..tostring(pc)..':'..tostring(s)..':'..iv) --..':'..tostring((pc-p):dot(p-av[iv].p)))
--						if i == 12 then -- and k == 5 then
--							U.dump(e,'?? for_12:'..k..':'..tostring(p)..'>'..tostring(pc)..':'..tostring(s)..':'..iv) --..':'..tostring((pc-p):dot(p-av[iv].p)))
--						end
					if pc and (pc-p):dot(p-av[iv].p) > 0 then
						local d = (p-pc):length()
						if d < dmi then
							dmi = d
							pmi = pc
							emi = k
              				smi = s
						end
--						  U.dump(e,'??******* hit: v:'..iv..'>'..i..':'..tostring(d))
--              lo('?? cross_S:'..tostring(s))
					end
				end
			end
		end
		if pmi then
--			U.dump(ae[emi],'?? for_MI:'..i..':'..tostring(pmi)..':'..smi)
			if smi == 0 then
		--          lo('?? pre:'..i..':'..emi..':'..ae[emi][1])
				edgeUp(i,ae[emi][1])
			elseif smi == 1 then
				edgeUp(i,ae[emi][2])
			else
				av[#av+1] = {p=pmi,star={}}
--					if i == 9 then
--						U.dump(av[#av], '?? to_INS:'..i..':'..#av..':'..emi)
--						U.dump(ae, '?? for_AEINS_pre:'..i..'>'..#av..':'..tableSize(ae)..':'..emi)
--					end
--				local v1,v2 = i,#av
--				ae[tableSize(ae)+1] = {v1,v2}
--				ae[#ae+1] = {v1,v2}
--				av[v1].star[v2] = 0
--				av[v2].star[v1] = 0

				edgeUp(i,#av)
				v2e(#av,emi)
--					if i == 12 then
--						U.dump(av[#av], '?? to_INS:'..i..':'..#av..':'..emi)
--						U.dump(ae, '?? for_AEINS_post:'..tableSize(ae))
--				        U.dump(ae, '?? postINS:')
--					end

		--        U.dump(ae, '?? postins:')
		--        U.dump(av[6].star, '?? postins:')
		--        U.dump(av[9].star, '?? postins:')
		--        return albl
			end
--			if i==9 then -- and emi==5 then
--				U.dump(ae, '?? to_EDGE:'..tostring(smi)..':'..emi)
--			end
		end
		::continue::
	end
--        U.dump(ae, '?? ae_post_s:')
  	albl = {}
	for i = 1,#av do
		albl[#albl+1] = {av[i].p, i}
	end
	-- find cycles
  --- oriented graph
  local aeo = {} -- oriented edges
  for i,e in pairs(ae) do
    table.sort(e)
    if not aeo[e[1]] then
      aeo[e[1]] = {}
    end
    aeo[e[1]][e[2]] = 0
--    aeo[e[1]][#aeo[e[1]]+1] = {ib=e[2],done=nil}
  end
--    U.dump(aeo, '?? sorted_AEO:')

  local function ifHole(p)
    local pth = {}
    for _,iv in pairs(p) do
      pth[#pth+1] = av[iv].p
    end
    local pth,map = U.polyStraighten(pth)
--      U.dump(map,'?? straihtned:'..#p..'/'..#pth)
    local iseq = {}
    for j,v in pairs(pth) do
      iseq[#iseq+1] = p[map[j]]
    end
    local stamp = U.stamp(deepcopy(iseq))
    if U.index(aloopstamp, stamp)[1] then
      -- is hole
      return true
    end
    return false
  end

  local function stepNext(a, b)
--      lo('?? stepNext:'..a..'>'..b)
      local dbg = false
--[[
      if a==14 and b==15 then
        dbg = true
      end
]]
--        if dbg then U.dump(av[b].star, '?? star:'..b) end
--      U.dump(av[b].star, '?? stepNext:'..a..'>'..b)
    local ama, ima = math.huge
--    local ama, ima = -math.huge
    -- go over star for maximal left turn angle
    for c,d in pairs(av[b].star) do
      if c ~= a then
        local edge = {b,c}
        table.sort(edge)
--          lo('?? in_star:'..c..'<'..b..':'..aeo[edge[1]][edge[2]])
        local ang = U.vang(av[b].p-av[a].p,av[c].p-av[b].p,true)
--          if dbg then lo('?? for_angg:'..c..':'..d..'>>'..tostring(ang)..' 1:'..tostring(av[b].p-av[a].p)..' 2:'..tostring(av[c].p-av[b].p)) end
--          lo('?? for_ang:'..ang..':'..c..'<'..b..'<'..a..':'..U.vang(av[b].p-av[a].p,vec3(0,-1,0),true))
        if ang < ama then
--        if ang > ama then
          ama = ang
          ima = c
        end
      end
    end
--      lo('?? ama:'..ima..'<'..b..'<'..a..':'..ama..':'..tostring(b<ima and aeo[b][ima] or aeo[ima][b]))
--    if ama then
    if ama and ama < U.small_val then
--    if ama and ama > -U.small_val then
      -- check if step is available
      if b < ima then
        if (not aeo[b][ima] or aeo[b][ima] == 1) then
          return
        else
--          aeo[b][ima] = 1
        end
      elseif b > ima then
        if (not aeo[ima][b] or aeo[ima][b] == -1) then
          return
        else
--          aeo[ima][b] = -1
        end
      end
      return ima
    end
  end

  local apath = {}
        local nia = 0
  for ia,s in pairs(aeo) do
	nia = nia + 1
--        if nia > 3 then break end
--		U.dump(s, '?? for_S:'..ia)
    for ib,d in pairs(s) do
--        lo('??______________________________ in_STAR:'..ia..'>'..ib)
      local step
      if d == 0 then
        step = {ia,ib}
--        s[ib] = 1
      elseif d == -1 then
        step = {ia,ib}
--        s[ib] = nil
      elseif d == 1 then
        step = {ib,ia}
--        s[ib] = nil
      end
      if step then
        -- go for cicle
        local piv,iv = step[1],step[2]
        local istart = piv
        local path = {iv}
            local n = 0
        while iv and iv~=istart and n < 100 do
              n = n + 1
          local civ = stepNext(piv,iv)
			if U.index({9,12,13,16}, piv)[0] or U.index({9,12,13,16}, iv)[0] then
              U.lo('?? sn_got:'..piv..'>'..iv..'>'..tostring(civ)) --..tostring(civ)..'/'..istart)
			end
          if civ then
            path[#path+1] = civ
--                U.dump(aeo, '?? ae_POSTSTEP:'..civ..' n:'..n)
--                break
          end
          piv = iv
          iv = civ
--              n = n + 1
--            break
        end
--            U.dump(path, '?? ae_POSTSTEP: n:'..n..':'..tostring(istart==path[#path])..':'..nia)
        if path[#path] == istart then
          if not ifHole(path) then
            apath[#apath+1] = path -- U.polyStraighten(path)
            for k,ib in pairs(path) do
              local a,b = ib,U.mod(k+1,path)
              if a<b then
  --              if not aeo[a][b] then aeo[a][b] =
                aeo[a][b] = (aeo[a][b] or 0) + 1
                if aeo[a][b]==0 then aeo[a][b]=nil end
              else
                aeo[b][a] = (aeo[b][a] or 0) - 1
                if aeo[b][a]==0 then aeo[b][a]=nil end
              end
            end
--                        U.dump(aeo, '?? AEO:'..#apath)
          end
        end
--            if n == 4 then break end
      end
    end
--        if nia > 1 then break end
--        break
  end
--      U.dump(apath, '?? APTH:')
--      U.dump(aeo, '?? AEO:')
  -------------------------
	-- triangulate discs
  -------------------------
  local morig,mscale = mrc[1],mrc[2] -- mat origin and scale
  local avert,af,auv = {},{},{}
  for i,v in pairs(av) do
    avert[#avert+1] = v.p + vec3(0,0,0.01)
  end
  local an = {-(base[#base]-base[1]):cross(base[2]-base[1]):normalized()}
--      U.dump(base,'??_____________ rcPave:'..tostring(an[1]))
  for i,p in pairs(apath) do
    --- straighten path
    local pth = {}
    for _,iv in pairs(p) do
      pth[#pth+1] = av[iv].p
    end
    local pth,map = U.polyStraighten(pth)
--      U.dump(map,'?? straihtned:'..#p..'/'..#pth)
    local iseq = {}
    for j,v in pairs(pth) do
--      avert[#avert+1] = v
--      iseq[#iseq+1] = #avert
      iseq[#iseq+1] = p[map[j]]
    end
    local stamp = U.stamp(deepcopy(iseq))
    if U.index(aloopstamp, stamp)[1] then
      -- is hole
    else
      --- triangulate
      an,auv,af = M.zip2(avert,iseq, an, true, af)
--          U.lo('?? AF:'..#af)
--          lo('??++++ for_mesh:'..#an..':'..#avert..':'..#af)
    end
--          lo('?? straihtned_stamp:'..U.stamp(deepcopy(iseq)))

--        if i == 4 then break end
  end
--  an = {vec3(0,0,1)}
--    U.dump(avert, '?? preUV:')
  auv = uv4poly(avert, 1) --, nil, true)
--    U.dump(auv, '?? AUV:')
  M.uvTransform(auv, mrc[1], mrc[2])
--    lo('?? mdata:'..#avert..':'..#auv..':'..#af..':'..#an)
--      U.dump(an, '?? AN:')

	local mbody = {
		verts = avert,
		faces = af,
		normals = an,
		uvs = auv,
		material = 'cladding_zinc_bat', --'WarningMaterial', --'cladding_zinc_bat',-- 'm_bricks_01_bat',-- 'WarningMaterial',
	}

	aline = {}
	for i,e in pairs(ae) do
		aline[#aline+1] = {av[e[1]].p,av[e[2]].p}
	end
        if dbg then U.dump(mbody, '<< rcPave') end

	return mbody,nil,albl,aline
end
--[[
        if b < c and (not aeo[b][c] or aeo[b][c] == 1) then
        elseif b > c and (not aeo[c][b] or aeo[c][b] == -1) then
        else
          -- check angle
          local ang = U.vang(av[b].p-av[a].p,av[c].p-av[b].p,true)
            lo('?? for_ang:'..ang..':'..c..'<'..b..'<'..a..':'..U.vang(av[b].p-av[a].p,vec3(0,-1,0),true))
          if ang > ama then
            ama = ang
            imi = c
          end
        end
]]
  --[[
  local istart = 1
  local ne = tableSize(ae)
  for i,e in pairs(ae) do
--    lo('?? ff:'..i)
--    U.dump(e, '?? for_e:'..i)
    local step
    if not e.done then
      step = {e[1],e[2]}
      e.done = 1
    elseif e.done == -1 then
      step = {e[1],e[2]}
      e.done = 0
    elseif e.done == 1 then
      step = {e[2],e[1]}
      e.done = 0
    end
    if step then
      -- got for circle
--        U.dump(step, '?? step:'..i)
      local piv,iv = step[1],step[2]
      local n = 0
      while iv and n < 100 do
        local civ = stepNext(piv,iv)
        piv = iv
        iv = civ
          n = n + 1
      end
      break
    end
  end
  while istart < ne do
    for i=istart,#ae do
      istart = i+1
        U.dump(ae[i], '?? for_i:'..i)
      if ae[i].done ~= 0 then
        --
        lo('?? for_circ:')
        break
      end
    end
  end
  ]]
--[[
					if false then
						loop[k],irc = onBoundary(loop, k)
						-- insert to boundary
						table.insert(pth,U.mod(irc+1,#pth),loop[k])

						av[start+k].p = loop[k]
						lo('?? is_out:'..i..':'..k..' irc:'..tostring(irc))
						edgeUp(start+k,irc)
	--                    iecheck[#iecheck+1] = #ae
						edgeUp(start+k,U.mod(irc+1,#pth))
	--                    iecheck[#iecheck+1] = #ae
	--                    edgeDown(irc)
						iecheck[#iecheck+1] = #ae-1
						iecheck[#iecheck+1] = #ae
	--                    edgeUp(start+k,start+U.mod(k+1,#loop))
						outfirst = k
					end
				if isout and k+1 == isout and false then
					loop[k+1],irc = onBoundary(loop, k)
					av[start+k+1].p = loop[k+1]
						U.dump(iecheck,'?? out_last:'..k..':'..isout..':'..irc)
--                    edgeDown(ie)
					for j,ie in pairs(iecheck) do
--                            U.dump(ae[ie], '?? for_edge:'..j..':'..ie..':'..#ae)
						if U.line2seg(U.mod(k+1,loop),U.mod(k,loop),av[ae[ie][1] ].p,av[ae[ie][2] ].p) then
							U.dump(ae[ie], '?? hit_ie:'..ie..':'..(start+k))
							edgeUp(start+k+1,ae[ie][1])
							edgeUp(start+k+1,ae[ie][2])
							edgeDown(ie)
						end
					end
				end
]]
--[[
				if isout and k+1 == isout then
					loop[k+1],irc = onBoundary(loop, k)
--                    aloopstick[i][k+1] = irc
				end
]]


M.rcPave_ = function(rc, aloop, mrc, apatch, istest)
--        lo('>> rcPave:')
	if not mrc then
		-- start, scale
		mrc = {{0,0}, {1,1}}
	end
--        lo('>> rcPave:', true)
--        if true then return end
--            dump(apatch, '>> rcPave:'..#aloop)
--            dump(rc, '>> rcPave_hull:')
--            indrag = true
	local function onBoundary(loop, k)
		for j =1,#rc do
			local p = U.line2seg(U.mod(k,loop),U.mod(k+1,loop),rc[j],U.mod(j+1,rc))
--                lo('?? onB:'..tostring(p)..':'..j..':'..tostring(U.line2seg(rc[j],U.mod(j+1,rc),loop[k],U.mod(k-1,loop))))
			if p and U.line2seg(rc[j],U.mod(j+1,rc),U.mod(k,loop),U.mod(k+1,loop)) then
--                lo('?? crossed:'..j..':'..tostring(p))
				return p,j
			end
		end
	end
	local out = {}
	-- restrict loop to rc
	local aloopstick = {}
	local irc
	for i,loop in pairs(aloop) do
		local isout
		local onboundary = {}
		local outfirst
		aloopstick[i] = {}
		for k=#loop,0,-1  do
			local b = U.mod(k,loop)
--        for k,b in pairs(loop) do
			if not U.inRC(b, {rc}) then
--                lo('?? is_OUT:'..k..':'..tostring(b))
				if not isout then
--                    onboundary[#onboundary+1] = onBoundary(loop, k)
					loop[k],irc = onBoundary(loop, k)
					outfirst = k
					aloopstick[i][k] = irc
				elseif k<outfirst-1 then
					table.remove(loop, k-1)
				end
				isout = k
			else
--                lo('?? if_out:'..k..':'..tostring(isout))
				if isout and k+1 == isout then
--                    lo('?? to_STICK:'..k..':'..isout)
--                    lo('?? next_CROSS:'..k..':'..tostring(onBoundary(loop, k))..':'..tostring(U.mod(k,loop))..':'..tostring(U.mod(k+1,loop))..':'..tostring(outlast))
					loop[k+1],irc = onBoundary(loop, k)
					aloopstick[i][k+1] = irc
--                    table.insert(loop, k+1, onBoundary(loop, k))
--                    onboundary[#onboundary+1] = onBoundary(loop, k)
				end
			end
		end
--        U.dump(loop, '?? loop_OUT:')
	end
--            U.dump(aloopstick, '?? loopstick:')
--            U.dump(aloop, '?? aloop:')

--[[
]]

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
			v4e[iv] = {p=loop[j], ij={i,j}} -- {vert position, {loop_ind,vert_ind}, containing_edges}
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
	for i,l in pairs(aloopstick) do

	end
--        v4e[7][2] = 3
--        v4e[8][1] = 3
--        table.remove(e4v,7)

--        table.remove(v4e[7],2)
--        table.remove(v4e[8],1)
--        U.dump(v4e, '?? v4e:')
--        U.dump(e4v, '?? e4v:'..#rc)
--            if true then return end
--        U.dump(ae, '?? ae:')
	-- original vertices number
	local nv = #v4e
--        lo('??________________________ rcPave:'..nv..':'..#rc..':'..#aloop)
--        if istest then U.lo('??_____________________ NV:'..nv..':'..#nvert, true) end
	local estamp = {}
	local sense = 0.000001
	------------------------------------------
	-- run over loops verts, make sprouts
	------------------------------------------
	for k=#rc+1,nv do
--    for k=1,nv-#rc do
		local dmi,cmi,imi,isend,isdupe = math.huge

		local function forCross(vinfo, v)
				local dbg = false
--                    vinfo.ij[1] == 2 and vinfo.ij[2] ==1 -- false -- #v4e == 14
--                    vinfo.ij[1] == 1 and vinfo.ij[2] ==2 -- false -- #v4e == 14
				if dbg then
					U.lo('>>************ forCross:'..tostring(v), true)
--                    U.dump(vinfo, '>>************ forCross:'..tostring(v)..':'..#v4e)
				end
			local p = vinfo.p
			local c,d
			for i,e in pairs(ae) do
--                isend = false
				if e.iloop ~= vinfo.ij[1] then
					c = U.line2seg(p, p+v, e[1], e[2], sense)
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
									if dbg then U.lo('??___________ not_END:', true) end
									isend = false
								end
							end
						end
					end
				end
			end
			if dbg then U.lo('<<************ forCross:'..tostring(cmi)..' d:'..tostring(dmi)..' isend:'..tostring(isend), true) end
		end

		local p = v4e[k].p
--            lo('?? for_P:'..tostring(p))

		local loop = aeref[v4e[k].ij[1]]
		local j = v4e[k].ij[2]
			if istest then U.lo('?? pre_CROSS:'..k, true) end
		isdupe = false
		imi = nil
		forCross(v4e[k], (p - U.mod(j-1,loop)):normalized())
		forCross(v4e[k], (p - U.mod(j+1,loop)):normalized())
		if isdupe then
				if istest then U.dump(e4v[imi], '??__________________ DUPE:'..tostring(cmi)..':'..tostring(imi)) end
			v4e[k].isdupe = true
		end
--            lo('?? post_CROSS:'..k..':'..tostring(cmi)..':'..tostring(isend))
--            if v4e[k].ij[1] == 2 and v4e[k].ij[2] ==1 then
--                U.lo('?? for_2:'..k..' cmi:'..tostring(cmi)..':'..tostring(isend))
--            end
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
			local est = U.stamp({k,vn})
--[[
				if est == '3_5' then
					lo('??______________________________________ is_3_5:')
					for i,_ in pairs(estamp) do
						lo('?? if_STAMP:'..i..':'.._)
					end
				end
]]
			local ie = U.index(estamp, est)
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
--                U.lo('!!!!!!!!!!!!!!!!!!!____________m.EXISTS:'..est..':'..k..':'..ie[1])
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
					if istest then U.lo('?? new_VERT:'..#v4e, true) end
				---- to new edge
				v4e[#v4e][#v4e[#v4e]+1] = #ae
				---- to prev edge(s)
				v4e[#v4e][#v4e[#v4e]+1] = imi
			else
				-- new edge to existing vert
				v4e[isend][#v4e[isend]+1] = #e4v
			end
				if istest then U.lo('??+++++++++++++++++++++++++ post_CROSS:'..#v4e, true) end
		else
--            U.lo('?? NO_CROSS:', true)
		end
--        if U.inRC(p, {rc}) then
	end
	for i=1,#e4v do
		table.sort(e4v[i], function(a,b)
			return a.d < b.d
		end)
	end
--            U.dump(e4v, '?? e4v_poststem:'..#rc)
--            U.dump(v4e, '?? v4e_poststem:')
--            if true then return end

--        if istest then U.dump(e4v, '?? e4v:') end
--        lo('?? v4e_post_stem:'..nv..'>'..#v4e)
		if istest then U.dump(v4e, '?? v4e:'..nv..'>'..#v4e) end
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
			if istest then U.lo('??=============================== FOR_RECTS:'..nmax, true) end
	local function stepFrom(n1, n2)
			if istest then U.lo('>> stepFrom:'..n1..'>'..n2, true) end
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
							if dbg then U.lo('?? if_next1:'..e4v[ie][i+1].ind) end
							astar[#astar+1] = {e4v[ie][i+1],ie,i} --.ind
						end
					elseif n.d == 1 then
						if e4v[ie][i-1].ind ~= n1 then
							if dbg then U.lo('?? if_next2:'..e4v[ie][i-1].ind) end
							astar[#astar+1] = {e4v[ie][i-1],ie,i} --.ind
						end
					else
						if dbg then U.lo('?? if_next3:') end
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
			if istest then U.lo('<< stepFrom:'..tostring(ima[1].ind), true) end
		else
			if istest then U.lo('<< stepFrom:'..'NONE', true) end
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

--        U.dump(e4v, '?? e4v:'..#rc)
	local arc = {}
	local togo = true
	while togo and ns < nmax do --151 do
--                U.lo('??================================= for_ns:'..ns)
				ns = ns + 1
		togo = false
		local rc = {}
		for i,list in pairs(e4v) do
--                U.dump(list, '??___________________ for_edge:'..i)
--                U.lo('?? for_edge:'..i)
			for k,n in ipairs(list) do
--                        lo('?? if_n:'..k..':'..tostring(n.done == 0))
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
							ns = 0
					while inext ~= rc[1] and ns < nmax do
								ns = ns + 1
								local npre = inext
						inxt,ext = stepFrom(iprev, inext)
--                            if ns < 10 then
--                                lo('?? nxt: k:'..k..' vert:'..n.ind..' start:'..rc[1]..':'..tostring(npre)..'>'..tostring(inxt))
--                            end
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
--                            U.lo('?? for_node:'..#rc..':'..iprev..'>'..inext)
						end
--                            U.lo('??+++++ next:'..tostring(inxt)..' ext:'..pext..'>'..tostring(ext)..':'..inext)
--                        rc[#rc+1] = inext
						if not inxt then break end
--                        rc[#rc+1] = inxt
						iprev = inext
						inext = inxt
						pext = ext
					end
					if ns >= nmax then
						lo('!! ERR_while:'..i..':'..k)
--                        U.dump(e4v,'!! ERR_while:'..i..':'..k)
--                        U.dump(list,'!! ERR_while:'..i..':'..k)
						break
					end
						if istest then U.dump(rc, '??<<<<<<< for_rc:') end
					if #rc > 2 then
						arc[#arc+1] = rc
--                                U.dump(rc, '??<<<<<<< for_rc:')
--                                togo = false
--                                break
					end
					break
				end
--                        break
			end
--                    if true then break end
			if togo then break end
		end
--        if togo then break end
	end
--        lo('?? for_ARC:'..#arc..':'..ns)
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
	local stbase = U.stamp({1,2,3,4})
--    local stbase = stamp({#aloop*4+1,#aloop*4+2,#aloop*4+3,#aloop*4+4})
	local ast = {}
	for k=2,#aloop+1 do
		ast[#ast+1] = U.stamp({(k-1)*4+1,(k-1)*4+2,(k-1)*4+3,(k-1)*4+4})
	end
		--U.dump(ast, '??___________________ ast:')

	local astamp = {}
	local apath = {}
	if #aloop == 0 then
		ast = {}
		arc = {{1,2,3,4}}
	else
		for k=#arc,1,-1 do
			local st = U.stamp({arc[k][1],arc[k][2],arc[k][3],arc[k][4]})
			local ist = U.index(ast,st)[1]
--                lo('?? for_stamp:'..stamp({arc[k][1],arc[k][2],arc[k][3],arc[k][4]})..':'..tostring(ist), true)
			if not ist then ist = 0 end
			if ist == 1 then
--            if ist == #aloop + 1 then
				-- border rc
				table.remove(arc, k)
			elseif #U.index(astamp, st) > 0 then
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
					U.lo('!! WRONG_PATH:'..#pth)
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
--        auv[#auv+1] = {u = (n.p.x-rc[1].x)*mscale[1] + morig[1], v = (n.p.y-rc[1].y)*mscale[2] + morig[2]}
--        auv[#auv+1] = {u = (n.p.x-rc[1].x)*mscale[1] + morig[1], v = (h - (n.p.y-rc[1].y))*mscale[2] + morig[2]}
	end
--        U.dump(av, '?? for_AV:')
--        lo('?? for_start:'..tostring(av[3])..':'..tostring(rc[1])..':'..tostring(rc[3]))
	-- triangulate
	local af, afhole = {},{}
--            arc = {}
--        U.dump(av, '?? for_AV:'..#av)
--        U.dump(arc, '?? for_RC:'..#arc)
--        table.remove(arc, 10)
	for i,a in pairs(arc) do
		local stamp = U.stamp(deepcopy(a))
--        lo(stamp)
		if stamp == '1_2_3_4' and #aloop > 0 then
			table.remove(arc, i)
		end
	end

		if istest then lo('??^^^^^^^^^^^^^^ for_RC:'..#arc) end

	if true then
--    if not istest then
		for k=#arc,1,-1 do
			local ist = U.index(ast,U.stamp({arc[k][1],arc[k][2],arc[k][3],arc[k][4]}))[1]
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
--                    lo('?? in_HOLE:')
	--                lo('?? to_hole:'..k,true)
				-- hole mesh
				if #U.index(apatch,ist) > 0 then
--                        lo('?? to_patch:'..ist, true)
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
--                    lo('?? in_BODY:')
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
--[[
	local istart = 3
--    auv = forUV(av, istart, {{u=0,v=0},{u=(rc[4]-rc[3]):length(),v=0}}) --uvini, w)
	auv = forUV(av, istart,
		{{u=(rc[4]-rc[3]):length(),v=mrc[1][2]},{u=mrc[1][1],v=mrc[1][2]}},
		(rc[istart+1]-rc[istart]):cross(rc[istart-1]-rc[istart])) --uvini, w)
]]
	local istart = 1
	auv = forUV(av, istart,
		{{u=mrc[1][1],v=-mrc[1][2]},{u=mrc[1][1]+(rc[istart+1]-rc[istart]):length(),v=-mrc[1][2]}}
		,(rc[2]-rc[1]):cross(rc[4]-rc[1]), mrc[2])
--        U.dump(mrc, '?? mrc')
		if istest then U.dump(auv, '?? AF:'..#af..':'..#auv) end
	for i,uv in pairs(auv) do
		auv[i].v = -auv[i].v
	end
	for i,f in pairs(af) do
--        f.u = (f.u - istart + 1) % #auv -- U.mod(f.u+1 + istart,#auv)
		f.u = U.mod(f.u-(istart-1)+1,#auv)-1
	end
--        U.dump(af, '?? for_AF:'..#af)
--        lo('?? for_AF:'..#af)
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
--!!
			mhole = nil

--        U.dump(e4v, '?? e4v:')
--        U.dump(apath, '?? apath:')
--    lo('<< rcPave: av:'..#av..' af:'..#af..' afhole:'..#afhole..'/'..ns, true)

	return mbody,mhole,ae,apath,albl--ae,apath
end


M.valid = function(am)
	for _,m in pairs(am) do
--            U.dump(m, '?? for_m:'.._)
		if not m.faces or #m.faces == 0 then
			lo('!! valid1:')
			return false
		end
		for _,f in pairs(m.faces) do
			if not f.u or not f.v or not f.n then
				U.dump(m.faces,'!! valid2:'.._..':'..#m.faces)
				return false
			end
		end
		if not m.verts then
				lo('!! valid3:')
			return false
		end
		if not m.uvs then
				lo('!! valid4:')
			return false
		end
		if not m.material then
				lo('!! valid5:')
			return false
		end
		if not m.normals then
				lo('!! valid6:')
			return false
		end
	end
	return true
end

local function meshUp(data, nm, group)
	if data == nil or #data == 0 or not M.valid(data) then
		lo('!! meshUp_NODATA:')
		return
	end
--        lo('>> meshUp:'..#data)
--    lo('>> meshUp:'..tostring(group))
--        U.dump(data, '>> meshUp:'..tostring(nm))
	if nm == nil then nm = 'm' end
	if group == nil then group = scenetree.MissionGroup end

	local om = createObject("ProceduralMesh")
	om:setPosition(vec3(0,0,0))
	om.isMesh = true
	om.canSave = false
	om:registerObject('tmp')
	local id = om:getID()
	om:unregisterObject()
	om:registerObject('o_'..nm..'_'..id)
	group:add(om.obj)
	om:createMesh({data})

	return id, om
end


M.align = align
M.rc = rc
M.rect = rect
M.dissect = dissect
M.faceHit = faceHit
M.fromGrid = fromGrid
M.daeParse = daeParse
M.matSet = matSet
M.pop = pop
M.move = move
M.copy = copy
M.cut = cut
M.clone = clone
M.step = step
M.update = update
M.mark = mark
M.save = save
M.meshUp = meshUp

M.dae2proc = dae2proc
M.forAMat = forAMat
M.grid2mesh = grid2mesh
M.forUV = forUV
M.ifHit = ifHit
M.uv4grid = uv4grid
M.uv4poly = uv4poly
M.grid2plate = grid2plate
--M.rcPave = rcPave
M.tri = tri
M.zip = zip
--M.zip_ = zip_
M.log = log

return M

--[[
Mesh.LB = function(a)
	lo('?? LB:'..tostring(a))
end

Mesh.toLineBuff = function(u, v, pos)
--    buf[pos + 0] = u.x
--    buf[pos + 1] = u.y
--    buf[pos + 2] = u.z
--    buf[pos + 3] = v.x
--    buf[pos + 4] = v.y
--    buf[pos + 5] = v.z
	lo('?? toLineBuff31:'..tostring(u)..'::'..tostring(pos))

	buf[0] = 0.0
	buf[1] = 0.0
	buf[2] = 0.0
	buf[3] = -5.0
	buf[4] = 0.0
	buf[5] = 0.0

	return buf
end

Mesh.toLineBuf_ = function(buf, u, v, pos)
--    buf[pos + 0] = u.x
--    buf[pos + 1] = u.y
--    buf[pos + 2] = u.z
--    buf[pos + 3] = v.x
--    buf[pos + 4] = v.y
--    buf[pos + 5] = v.z
	lo('?? toLineBuf1:'..tostring(#buf))

	buf[0] = 0.0
	buf[1] = 0.0
	buf[2] = 0.0
	buf[3] = -5.0
	buf[4] = 0.0
	buf[5] = 0.0

	return buf
end


function test_mesh()
--local function createMesh(type, nodes, fields)
	vertices = {}
	normals = {}
	uvs = {}
	faces = {}

	vertices[#vertices + 1] = {x = 1, y = 0, z = 0}
	vertices[#vertices + 1] = {x = 0, y = 1, z = 0}
	vertices[#vertices + 1] = {x = 0, y = 0, z = 1}

	uvs[#uvs + 1] = {u = 0, v = 1} -- index form 0: csLength + 1
	uvs[#uvs + 1] = {u = 1, v = 1} -- index form 0: csLength + 2
	uvs[#uvs + 1] = {u = 0, v = 0} -- index form 0: csLength + 3
--  uvs[#uvs + 1] = {u = 1, v = 0} -- index form 0: csLength + 4

	normals[#normals + 1] = {x = 1, y = 1, z = 1}

--  faces[#faces + 1] = {v = 0, n = 0, u = 0}

	faces[#faces + 1] = {v = 1, n = 0, u = 1}
	faces[#faces + 1] = {v = 0, n = 0, u = 0}
	faces[#faces + 1] = {v = 2, n = 0, u = 2}


	road = createObject("ProceduralMesh")
	road:setPosition(pos)
--  road:setPosition(vec3(0, 0, 0))
	road.canSave = false
--  road.isMeshRoad = true
	road:registerObject("mesh_obj")
	roadID = road:getID()
	lo('?? createMesh_id:'..tostring(roadID))
	road:unregisterObject()
	road:registerObject("mesh_obj_" .. roadID)
	scenetree.MissionGroup:add(road.obj)

	road:createMesh({{{
		verts = vertices,
		uvs = uvs,
		normals = normals,
		faces = faces,
		material = "WarningMaterial",
	}}})
--  road:createMesh({{{}}})

	return roadID
end

return Mesh
]]
