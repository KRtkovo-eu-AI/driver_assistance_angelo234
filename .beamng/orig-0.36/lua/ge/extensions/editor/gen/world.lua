
--        local p1, p2 = closestLinePoints(vec3(2,0,0), vec3(-1,0,0), vec3(0,1,-1), vec3(0,1,1))
--        lo('?? PP:'..p1..':'..p2)

local _dbdrag = false
local _mode = core_levels.getLevelName(getMissionFilename()) == 'template_tech' and 'BASE' or 'ROAD'
_mode = 'GEN'
local inrecover

-- lo('== WORLD:'.._mode..':'..tostring(scenetree.findObject('theForest'))..' edit:'..tostring(scenetree.findObject('edit'))) --core_levels.getLevelName(getMissionFilename()))


--lo('?? +++*** iDIST:'..tostring(editor.getPreference("gizmos.objectIcons.fadeIconsDistance")))

local W = {}

local stage = nil

-- materials
local dmat = {
	wall = {},
	roof = {},
--    roof = {'','m_metal_claddedzinc','m_roof_slates_rounded'},
	ext = {},
	road = {},
}

W.ui = {
	-- city sector
	sec_margin = 12,
	sec_grid = 16,
	sec_rand = 0.2,
	sec_trim = 6,
	sec_griddir1 = 0,
	sec_griddir2 = 0,

	-- junction
	injunction = false, -- U._PRD == 0 and true or false,
	branch_n = 4,
	exit_r = 0,
	exit_w = 0,
	junction_round = false,
	round_r = 20,
	round_w = 6,

	conf_jwidth = 2,
	conf_margin = 6,
	conf_mslope = 10,
	conf_bank = 2,
	conf_all = false,
	-- 'conf_jwidth', conf_margin, conf_mslope

	xpos = 0,
	ypos = 0,
	zpos = 0,
	building_ang = 0,
	n_floors = 0,
	building_style = 0,
	building_scalex = 1,
	building_scaley = 1,

	basement_toggle = false,
	basement_stairs = false,
	basement_height = 1,

	corner_inset = false,

	floor_inout = 0,
	height_floor = 0,
	ang_floor = 0,
	flip_axis = 0,
	ang_fraction = 4,

	wall_pull = 0,
	wall_spanx = 0,
	wall_spany = 0,
	fringe_height = 0,
	fringe_inout = 0,
	fringe_margin = 0,

	uv_u = 0,
	uv_v = 0,
	uv_scale = 1,

	win_bottom = 0,
	win_left = 0,
	win_space = 0,
	win_scale = 0,

	balc_ind0 = 1,
	balc_ind1 = 2,
	balc_ind2 = 0,
	balc_scale_width = 1,

	pilaster_ind0 = 1,
	pilaster_ind1 = 0,
	pilaster_ind2 = 0,

	door_ind = 0,

	pillar_inout = 0,
	pillar_spany = 1,

	seed = 1,

	top_margin = 0,
	top_tip = 0,
	top_thick = 0,

	ridge_flat = false,
	ridge_width = 20,

	roofborder_width = 0.6,
	roofborder_height = 0.6,
	roofborder_dy = 0,
	roofborder_dz = 0,

	mat_wall = dmat.wall,

	node_r = 80,
	node_n = 8,
}


--stage = '_3_span_copy'

--    stage = 'tst_roof'
--        stage = '3_thin_square'
--stage = '2_side'
--    stage = 'tst_split'
--    stage = 'tst_winmargin'
--    stage = 'tst_stick'
--    stage = 'tst_middle'
--    stage = 'tst_paste'
--    stage = 'tst_paste2'
--    stage = 'tst_split_move'
--    stage = 'tst_scale'
--    stage = 'tst_span2'
--    stage = 'tst_span3'
--    stage = 'tst_roof2'
--    stage = 'tst_height'
--    stage = 'tst_span_paste'
--    stage = 'tst_scale_copy'
--    stage = 'tst_achild'
--    stage = 'tst_achild2'
--    stage = 'tst_paste_alien'
--    stage = 'tst_matmove'
--    stage = 'tst_mirror'
--    stage = 'tst_fringe'
--    stage = 'tst_pillar'
--    stage = 'tst_corner'
--    stage = 'tst_roofrect'
--    stage = 'tst_back'
--    stage = 'tst_addfloor'
--    stage = 'tst_drag'
--    stage = 'tst_topchild'
--    stage = 'tst_paste3'
--    stage = 'tst_child'
--    stage = 'tst_corner3'
--    stage = 'tst_balcony'
--    stage = 'tst_els'
--    stage = 'test_split'
--    stage = 'test_pyr'
--    stage = 'tst_awplus'
--    stage = 'tst_spg'
--    stage = 'tst_side'
--    stage = 'tst_pilaster'
--    stage = 'tst_sc'
--    stage = 'tst_ridge'
--    stage = 'tst_basement'
--    stage = 'tst_border'
--    stage = 'tst_uv'
--    stage = 'tst_shed'
--    stage = 'tst_split'
--    stage = 'tst_mat'
--    stage = 'tst_mesh'
--    stage = 'tst_rooffat'
--    stage = 'tst_split'
--    stage = 'tst_roofshape'
--    stage = 'tst_cut'
--    stage = 'tst_storefront'
--    stage = 'tst_gable'
--    stage = 'tst_roof'
--    stage = 'tst_stairs'
--    stage = 'tst_v'
--    stage = 'tst_copypaste'
--    stage = 'tst_pyramid'
--    stage = 'tst_ridge'
--    stage = 'tst_child'
--    stage = 'tst_roofsplit'
--    stage = 'tst_pave'
--	  stage = 'tst_export'
--    stage = 'tst_rr'
--	stage = 'tst_shot'
--	stage = 'tst_inset'

--scenetree.findObject('theForest')
--local forestName = scenetree.findObject('theForest') and 'theForest' or 'myForest' -- 'theForest'
local forestName = 'myForest' -- 'theForest'

-- modules
local im = ui_imgui
local ffi = require "ffi"
require 'socket'
--local pngImage = require('/lua/ge/extensions/editor/gen/png')

local indrag = false
local inrollback


local U,M,D,Render,T,N,Ter,J = {}


--local function lo(ms, yes)
--	if U._PRD ~= 0 then return end
--	if indrag and not yes then return end
--	print(ms)
--end
local lo = function() end

-- lo('= WORLD:')

--lo('??^^^^^^^^^^^^^^^ world_IFFOREST:'..forestName)

local apack = {
	'/lua/ge/extensions/editor/gen/mesh',
	'/lua/ge/extensions/editor/gen/utils',
	'/lua/ge/extensions/editor/gen/decal',
	'/lua/ge/extensions/editor/gen/render',
	'/lua/ge/extensions/editor/gen/top',
	'/lua/ge/extensions/editor/gen/network',
	'/lua/ge/extensions/editor/gen/terrain',

	'/lua/ge/extensions/editor/gen/jbeam',

	'/lua/ge/extensions/editor/gen/robot',
--    '/lua/ge/extensions/editor/gen/dlltest',
--    '/lua/ge/extensions/editor/gen/png',
}


local function pretest(inworld)
	if U._PRD == 1 then return end
	if true then return end
	lo('>> pretest:')

	unrequire(apack[6])
	N = rerequire(apack[6])


	local direction = vec3(core_camera.getForward())
	local startPoint = vec3(core_camera.getPosition())
	lo('?? pretest:'..tostring(inworld)..':'..tostring(direction)..':'..tostring(startPoint)) --..':'..tostring(color(0,255,255,255).x))
--        if true then return end

--    U.dump(color(0,255,255,255), '?? COLOR:')
	local x, y, z = -3351.02, 3728.87
	x, y = -55.37, -13.65
	x, y = -163.88, 239.00
	x, y, z = -246.85, 112.89, 148.51
	local cpos = {-295.65, 35.83, 245.37, 0.416319, -0.194524, 0.374579, 0.805313}
	cpos = {-180.27, -4.05, 202.81, 0.230399, 0.164714, -0.560714, 0.778065}
	cpos = {-260.81, 45.77, 96.22, 0.129612, 0.0737161, -0.49567, 0.855616}
	cpos = {-197.32, -177.16, 111.40, -0.252278, 0.0638241, -0.232861, -0.937048}
	cpos = {162.04, -86.01, 456.48, 0.220327, 0.29635, -0.747225, 0.552529}
--    cpos = {-326.41, 41.01, 89.41, 0.195361, 0.0401656, -0.202879, 0.958677}
	x, y, z = cpos[1], cpos[2], cpos[3]


	if inworld then
--        U.camSet(cpos)
		scenetree.findObject("thePlayer"):setPosition(vec3(x, y, core_terrain.getTerrainHeight(vec3(x,y))))
		if true then return end
		N.circleSeed(W.ui.node_n, vec3(x, y), W.ui.node_r)
		N.pathsUp()

		N.toDecals()
--        if true then return end

		local R = require('/lua/ge/extensions/editor/gen/region')
--        lo('?? for_R:'..tostring(R))
		local p = vec3(-417.601593,64.60771179,73.03094482)
				p = vec3(-103,-152,50)
		R.forRoads(p, D)

		R.populate(D.out.across, D.out.adec, D, W, nil, p)
		R.onSpacing(0)

--            lo('?? decs:'..tostring(#rdlist))

--        D.ter2road()
	else
		U.camSet(cpos)
	end
--[[
		local rdlist = editor.getAllRoads()
		for id,_ in pairs(rdlist) do
			lo('?? rd:'..tostring(id)..':'..tostring(scenetree.findObjectById(id)))
		end
]]
end


local scope -- = 'building' -- ?md_scope
local inanim = nil
local incorner = false
local incopy = {}
local incontrol
local asel = {} -- Ctrl-selected objects


local camspeed = core_camera.getSpeed()
if camspeed < 20 then
	core_camera.setSpeed(20)
	camspeed = 20
end
--local inedge = nil

local out = {
--    scope = scope,
	wsplit = nil,

	avedit = {},
	aedge = nil,
	asplit = nil,

	inroad = 0,
	inseed = false,
	ctime = os.clock(),
	ccommand = '',

--    defmat = 'm_greybox_base',
	defmat = 'm_greybox_base_bat', --
		--'m_stucco_white',
--    imUtils = require('ui/imguiUtils'),
}


--local Ext = {}

local function reload(mode)
--		print('?? W.reload:'..tostring(mode))
		lo('>> world.reload:'..#apack..':'..tostring(scope))

	unrequire(apack[1])
	M = rerequire(apack[1])

	unrequire(apack[2])
	U = rerequire(apack[2])
	lo = U.lo

	unrequire(apack[4])
	Render = rerequire(apack[4])

	unrequire(apack[5])
	T = rerequire(apack[5])

	T.inject(W)

	if (mode and mode == 'conf') or U._MODE == 'conf' then
		-- DECALS
		unrequire(apack[3])
		D = rerequire(apack[3])
			lo('??+++++++++++++++++++++++++++++++++++ FOR_D:'..tostring(D)..':'..tostring(D.junctionUp))
		W.ui.exit_w = D.out.default.wexit
		W.ui.exit_r = D.out.default.rexit
		out.D = D
--			print('?? for_TER:')
			lo('??^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ for_TER:')
		unrequire(apack[7])
		Ter = rerequire(apack[7])
		out.Ter = Ter
	end
	if mode then
		U._MODE = mode
		U.out._MODE = mode
	end
	if U._PRD == 0 then
		unrequire(apack[6])
		N = rerequire(apack[6])
			lo('??^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ for_TER:')
		unrequire(apack[7])
		Ter = rerequire(apack[7])
		out.Ter = Ter

		unrequire(apack[8])
		J = rerequire(apack[8])
		out.J = J

--		UI.inject({Ter=Ter})
	end

--    unrequire(apack[7])
--    Ext.R = rerequire(apack[7])

--    unrequire(apack[3])
--    D = rerequire(apack[3])

--    unrequire(apack[4])
--    rerequire(apack[4])
end
reload()
W.reload = reload

if U._PRD == 1 then stage = nil end

-- Luca
--U.camSet({-558.94, 1191.07, 110.40, 0.0828153, 0.301844, -0.915906, 0.251293})
-- [-548.58, 1234.23, 1075.13, 0.0958088, 0.310139, -0.903712, 0.279177]
-- [-558.94, 1191.07, 110.40, 0.0828153, 0.301844, -0.915906, 0.251293]

local groupEdit = scenetree.findObject('edit')
if groupEdit then
    scenetree.MissionGroup:removeObject(groupEdit)
    groupEdit:delete()
    groupEdit = nil
      lo('?? edit_DOWN:')
end

--[[
if true and groupEdit == nil then
--    groupEdit = createObject('edit')
--    scenetree.MissionGroup:registerObject('edit')
	groupEdit = createObject("SimGroup")
	groupEdit:registerObject('edit')
	groupEdit = scenetree.findObject('edit')
		lo('?? edit_CREATED:'..tostring(groupEdit))
else
	lo('?? edit_EXISTS:')
end
]]

--        scenetree.findObject('edit'):delete()
--        scenetree.MissionGroup:removeObject(scenetree.findObject('edit'))
--        lo('??+++++++++++++++++++++++++++++++++++++ if_EDIT:'..tostring(scenetree.findObject('edit')))

if U._PRD == 0 and scenetree.MissionGroup and groupEdit then
	scenetree.MissionGroup:removeObject(groupEdit)
	scenetree.MissionGroup:addObject(groupEdit)
end
--    lo('??+++++++++++++++++++++++++++++++++++++ if_EDIT:'..tostring(scenetree.findObject('edit')))


local fsave = '/tmp/save'
local asave = {}  -- save timestamps sec
local csave = 1

local adec = {}
local aref = {}
local L = 4096
local grid = 10

local small_ang = U.small_ang
local small_dist = 0.01
local near_dist = 0.2


local dstyle = {
	residential = {
--        mat_wall = {'m_stonebrick_eroded_01', 'm_stonebrick_mixed_01', 'm_plaster_worn_01'},
		mat_wall = {'m_bricks_01', 'm_stonebrick_eroded_01', 'm_stonebrick_mixed_01', 'm_plaster_worn_01'},
		mat_roof = {},
		mesh_win = {},
		mesh_door = {},
	},
	industrial = {
		mat_wall = {'m_bricks_01'}, -- {'m_metal_brushed', 'metal_plates', 'cladding_zinc', 'm_plaster_raw_dirty_01'},
		mat_roof = {},
		mesh_win = {},
		mesh_door = {},
	},
}


local function materialLoad(path, list, pref)
	pref = pref or '*'
	local matFiles = FS:findFiles(path, pref..'mat.json', -1, true, false)
		lo('>> materialLoad:'..path..':'..#matFiles)
	if not list then list = {} end
--    local list = {}
	for k,v in pairs(matFiles) do
--			lo('?? for_mat_FILE:'..tostring(v))
		loadJsonMaterialsFile(v)
		local objects = extensions.editor_resourceChecker_resourceUtil.getSimObjects(v)
--				lo('?? nmat:'..#objects)
		for _, obj in ipairs(objects) do
				if true or U._PRD == 0 then lo('?? for_mat:'..tostring(obj.name)) end --..tostring(obj.doubleSided))
			obj.doubleSided = true
			local property = 'invertBackFaceNormals'
			local layer = 0-- im.IntPtr(0)
			editor.setMaterialProperty(obj, property, layer, 1)

--            lo('?? materialLoad:'.._..':'..tostring(obj))
			list[#list + 1] = obj
		end
		if pref == 'top.' then
			local wrCommon = {
				'm_stucco_white_bat',
				'm_plaster_raw_dirty_01_bat',
				'm_plaster_float_bat',
				'm_stucco_scraped_01_bat',
				'm_plaster_worn_01_bat',
			}
			for _,nm in pairs(wrCommon) do
				local mo = scenetree.findObject(nm)
--                    lo('?? for_roof_COM:'..tostring(nm)..':'..tostring(mo))
				if mo then
					list[#list + 1] = mo
				end
			end
			table.sort(list, function(a,b)
				return a.name < b.name
			end)
		end

--[[
		objects = extensions.editor_resourceChecker_resourceUtil.getSimObjects(v)
		for _, obj in ipairs(objects) do
			if obj.name == 'm_bricks_01' then
				lo('??+++++ for_BRICKS:'..tostring(obj.name))
				obj.name = 'm_bricks_BAT'
				list[#list + 1] = obj
			end
		end
]]
	end

--	lo('<< materialLoad:'..path..':'..#list)
	return list
end


W.matUp = function()
	if false then
		dmat.wall = materialLoad('/levels/south_france/art/shapes/buildings/')
		materialLoad('/assets/materials/', dmat.wall)
	else
		dmat.wall = materialLoad('/art/shapes/common/building_architect_modules/bat_materials/', nil, 'wall.')
		dmat.roof = materialLoad('/art/shapes/common/building_architect_modules/bat_materials/', nil, 'top.')
	--    dmat.wall = materialLoad('/tmp/bat/', nil, 'wall.')
	--    dmat.roof = materialLoad('/tmp/bat/', nil, 'top.')
--        materialLoad('/art/shapes/objects/')
		if U._PRD == 0 then
			materialLoad('/levels/south_france/art/shapes/buildings/')
			materialLoad('/assets/materials/')
		end
	end
	table.insert(dmat.wall, 1, '- NONE -')
	--    U.dump(dmat.wall, '?? dmat_wall:')
		lo('?? wall_MAT:'..#dmat.wall)
end
--dmat.ext = materialLoad('/assets/mat/')
--dmat.road = materialLoad('/levels/south_france/art/decals/')

--!! dmat.ext = materialLoad('/lua/ge/extensions/editor/gen/assets/')
--!! dmat.road = materialLoad('/levels/smallgrid_aitest/art/road/')

-- meshes
local forest --= core_forest.getForestObject()
local fdata
if forest ~= nil then
--    fdata = forest:getData()
end

local assetPath = '/art/shapes/common/building_architect_modules'
--    '/assets/meshes/buildings/residential/BAT_modules_temp'
--local assetPath = '/lua/ge/extensions/editor/gen/assets'

local daePath = {
	win = {
		assetPath..'/windows/s_R_HS_A_WN_01_0.7x0.7_A.dae',
		assetPath..'/windows/s_R_HS_A_WN_02_0.7x2.2_A.dae',
		assetPath..'/windows/s_R_HS_B_WN_03_1.6x2_A.dae',

--        '/assets/windows/s_R_HS_A_WN_01_0.7x0.7_A.dae',
--        '/assets/windows/s_R_HS_A_WN_02_0.7x2.2_A.dae',
--        '/assets/windows/s_R_HS_B_WN_03_1.6x2_A.dae',

--        '/assets/windows/s_R_HS_B_WN_03_0.6x1.2.dae',
	},
	door = {
		assetPath..'/doors/s_R_HS_A_DR_01_0.9x2.2_A.dae',
		assetPath..'/doors/s_R_HS_B_DR_06_2.1x2.4_A.dae',
	},
	corner = {
		assetPath..'/corner/s_R_HS_A_CO_01.dae',
	},
	balcony = {assetPath..'/balcony/s_R_HS_A_BA_01.dae'},
	pillar = {}, --{assetPath..'/pillar/s_R_HS_A_PL_01.dae'},
--[[
	roofborder = {
--        {'/assets/roof_border/s_RES_S_01_RTR_EO_01_2.dae', {1}}, -- editable material index
--        {'/assets/roof_border/s_RES_S_01_RTR_EEL_01_2.dae', {1}},
		{assetPath..'/roof_border/s_RES_S_01_RTR_EEL_01_2.dae', {1}},
	},
	roofend = {
		{assetPath..'/roof_border/s_RES_S_01_RTR_EEL_01_2.dae', {1}},
	},
	roofcenter = {
		{assetPath..'/roof_border/s_RES_S_01_RTR_EEL_01_2.dae', {1}},
	},
]]
	stringcourse = {},
	_ext = {
		{'/assets/_ext/wooden_watch_tower2.dae'},
--        {'/assets/_out/save_17616.dae'},
--        {'/assets/_ext/chair/chair2.dae', {}, {1,2,4}},
--        {'/assets/_ext/TV_b2.dae', {}, nil, 6},
--        {'/assets/_ext/TV_Stand.dae', {}, {2}, 6},
--        {'/assets/_ext/tv/tv.dae', {}, {2,3}},
--        {'/assets/_ext/chair/modern_chair_11_obj.dae'},
--        {'/assets/_grabbed/t3.dae'},
--        {'/assets/_ext/koltuk/Koltuk2.dae'},
--        {'/assets/_ext/dragon/Dragon 2.5_dae.dae'},
	},
}
local ddae = {}    -- key is the DAE path
local dmesh = {}  -- mesh_id -> mesh_data
local daeloaded = false

--    U.dump(daePath['win'], '?? IF_DAE:')


local function terrainUp(path)
	local terrBlock = TerrainBlock()
		lo('>> terrainUp:'..path..':'..tostring(terrBlock))

	local terrainImpExp = {}
	-- import
	terrainImpExp.terrainName = im.ArrayChar(32, "theTerrain")
	terrainImpExp.metersPerPixel = im.FloatPtr(1)
	terrainImpExp.heightScale = im.FloatPtr(50) -- maxHeight
	terrainImpExp.heightMapTexture = im.ArrayChar(128)
	terrainImpExp.holeMapTexture = im.ArrayChar(128)
	terrainImpExp.textureMaps = {
--        {path = '/levels/south_france/art/terrains/t_asphalt_b.png',
--            material = '', --'m_metal_brushed',
--            selected=false}
--        {path="/levels/gridmap/ter.png", selected=false},
	}
--    local matId, matName = getMtlIdByName("m_metal_brushed")
--        lo('?? for_mat:'..matId)
	table.insert(terrainImpExp.textureMaps,
		{path='/levels/south_france/export/theTerrain_layerMap_7_dirt_rocky.png',
		--"/levels/johnson_valley/export/theTerrain_layerMap_0_rockydirt.png",
		--"/levels/south_france/export/theTerrain_layerMap_0_grass.png",
		selected=true, material='', --"m_metal_brushed",
		materialId = im.IntPtr(0), channel="R", channelId=im.IntPtr(0)})

	terrainImpExp.applyTransform = im.BoolPtr(false)
	-- change values based on terrain size
	-- 1/2 * terrain width, 1/2 * terrain height
	terrainImpExp.transformPos = {
		x = im.FloatPtr(-512),
		y = im.FloatPtr(-512),
		z = im.FloatPtr(0),
	}
	terrainImpExp.flipYAxis = im.BoolPtr(false)
	-- export
	terrainImpExp.exportPath = im.ArrayChar(128)

	terrainImpExp.heightScale[0] = 700
	terrainImpExp.metersPerPixel[0] = 1 --0.75


	local materials = {} --'m_metal_brushed'}
	for _,map in ipairs(terrainImpExp.textureMaps) do
		table.insert(materials, map.material)
	end

		lo('?? ifTB2:'..tostring(scenetree.findClassObjects("TerrainBlock")))
	local terrainBlockProxies = {}
	for _, name in ipairs(scenetree.findClassObjects("TerrainBlock")) do
			lo('?? ifTB:'.._..':'..name) --tostring(scenetree.findClassObjects("TerrainBlock")))
		terrainBlockProxies[name] = {selected = false, id = scenetree.findObject(name):getID()}
	end

	local terrBlockName = ffi.string(terrainImpExp.terrainName)
	for tbName, tbData in pairs(terrainBlockProxies) do
		if string.lower(tbName) == string.lower(terrBlockName) then
--            if debug == true then log('I', '', "Found TerrainBlock with the given name '".. terrBlockName .."'") end
			-- TODO: a TerrainBlock with the same name has been found: can we overwrite it?
			terrBlock = scenetree.findObjectById(tbData.id)
			lo('?? forTB:'..tostring(terrBlock))
--            createNewTerrainBlock = false
		  end
	end

	terrainImpExp.applyTransform[0] = true
	terrainImpExp.transformPos.x[0] = -L
	terrainImpExp.transformPos.y[0] = -L
	terrainImpExp.transformPos.z[0] = -400

	terrBlock:setPosition(vec3(terrainImpExp.transformPos.x[0], terrainImpExp.transformPos.y[0], terrainImpExp.transformPos.z[0]))

	local success = terrBlock:importMaps(path,
		terrainImpExp.metersPerPixel[0],
		terrainImpExp.heightScale[0],
		ffi.string(terrainImpExp.holeMapTexture),
		materials, terrainImpExp.textureMaps,
		terrainImpExp.flipYAxis[0])

		lo('?? impsucc:'..tostring(success))

	local missionGroup = scenetree.MissionGroup
	missionGroup:addObject(terrBlock)

--            if true then return end
--    local block = extensions.editor_terrainEditor.getTerrainBlock()
--    extensions.editor_terrainEditor:attachTerrain(terrBlock)
end


local function meshUp(data, nm, group, dbg)
		lo('>>_______________________ meshUp:'..nm)
	if data == nil or #data == 0 or not M.valid(data) then
		lo('!! meshUp_NODATA:') --..tostring(M.valid(data)))
		return
	end
--        lo('>> meshUp:'..#data)
--    lo('>> meshUp:'..tostring(group))
--        U.dump(data, '>> meshUp:'..tostring(nm))
	if nm == nil then nm = 'm' end
	if group == nil then group = scenetree.MissionGroup end

--	local om = createObject("TSStatic")
--		lo('?? if_obj:'..tostring(om), true)
	local om = createObject("ProceduralMesh")
	om:setPosition(vec3(0,0,0))
	om.isMesh = true
	om.canSave = false
	om:registerObject('tmp_'..tostring(os.clock()))
	local id = om:getID()
--	om:unregisterObject()
		lo('?? if_ID:'..tostring(id)) --..':'..tostring(id and scenetree.findObjectById(id) or nil))
	om:setName('o_'..nm..'_'..id)
--	om:registerObject('o_'..nm..'_'..id)
	group:add(om.obj)
--        lo('?? for_mesh:'..#data..':'..tostring(#data[1].verts)..':'..tostring(#data[1].faces)..':'..tostring(#data[1].uvs)..':'..tostring(#data[1].normals)..':'..tostring(data[1].material), true)
--        if tostring(data[1].material) == 'm_roof_slates_rounded' then
--            U.dump(data[1].faces, '?? FOF:'..#data[1].faces)
--        end
--        lo('?? meshUp:'..tostring(id)..':'..#data[1].verts..':'..#data[1].faces..':'..#data[1].normals..':'..#data[1].uvs)
--        om:createMesh({{data[2]}})
--        om:createMesh({{data[4]}})
--        U.dump(data, '?? meshUp:')
--    U.dump(data[3], '??^^^^^^^^^^^ meshUp 3:')
--    U.dump(data[4], '??^^^^^^^^^^^ meshUp 4:')
--	om:createMesh({{data[3],data[4]}})
--	om:createMesh({{data[4],data[3]}})

	om:createMesh({data})
--        lo('<< meshUp:')
	return id, om
end


--require "lfs"

local function daeImport(path, cls)
		lo('>> daeImport:'..path) --..':'..('dir '..path..[[ /b]])..':'..tostring(io.popen('echo "AAA"')))
	daePath[cls] = {}
	local fl = io.open(path..'/list'):lines() --:read('*all')
	local line = fl()
	line = fl()
	local n = 0
	while line and n < 100 do
--        lo(line)
		daePath[cls][#daePath[cls]+1] = path..'/'..line
		line = fl()
		n = n + 1
	end
		U.dump(daePath[cls])
end


local function daeLoad_(pth, tp)
	local afile = FS:findFiles(pth..tp, '*.dae', -1, true, false)
		lo('>> daeLoad:'..tp)
--        U.dump(afile, '>> daeLoad:')
	if tp == 'windows' then tp = 'win' end
	if tp == 'doors' then tp = 'door' end
	if tp == 'roof_border' then tp = 'roofborder' end
	if tp == 'string_course' then tp = 'stringcourse' end
	if tp == 'store_front' then tp = 'storefront' end
	if tp == 'wall_pilaster' then tp = 'pilaster' end
--    if tp == 'pilaster' then
--        afile = FS:findFiles(pth..'pillar', '*.dae', -1, true, false)
--    end
	daePath[tp] = {}
	for i,p in pairs(afile) do
		if not string.find(p, '_COin_') then
--                lo('?? for_dpath:'..i..':'..p,true)
			local pnew = nil
			local pth = p
			daePath[tp][#daePath[tp]+1] = pth

			local obj = createObject('TSStatic')
			obj:setField('shapeName', 0, pth)
			obj.isMesh = true
			obj.canSave = false
			obj.hidden = true

			local nm = 'dae_'..tp..'_'..i
			local otmp = scenetree.findObject(nm)
			if otmp then otmp:unregisterObject() end
			obj:registerObject(nm) -- needed to get box
			local box = obj:getWorldBox()

--                if tp == 'corner' then
--                    lo('?? dL_tp:'..tp..':'..pth..':'..tostring(box.minExtents)..':'..tostring(box.maxExtents))
--                end

--      local w,h
--          if tp == 'win' then
--          end
--      lo('?? for_dae:'..pth)
      local atkn = U.split(pth, '_',true)
      atkn = U.split(atkn[#atkn],'.',true)
--          lo('?? prespl:'..atkn[1])
      atkn = U.split(atkn[1],'x',true)
--          lo('?? for_w:'..atkn[1]..':'..tonumber(atkn[1]))
      local w,h = atkn[1] and tonumber(atkn[1]) or nil,atkn[2] and tonumber(atkn[2]) or nil
--			ddae[pth] = {type = tp, fr = vec3(0,0,0), to = box.maxExtents, fo = nil, list = {}, w = w and w/100 or nil, h = h and h/100 or nil}
			ddae[pth] = {type = tp, fr = box.minExtents, to = box.maxExtents, fo = nil, list = {}, w = w and w/100 or nil, h = h and h/100 or nil}
			if ({storefront=1,stringcourse=1})[tp] then
	-- detect orientation
				local ma,ima = 0
				for i,c in pairs({'x','y','z'}) do
	--                lo('?? for_SF:'..i..':'..c..':'..ddae[pth].fr[c])
					local L = math.abs(ddae[pth].fr[c]-ddae[pth].to[c])
					if L > ma then
						ma = L
						ima = i
					end
				end
	--                U.dump(ddae[pth], '?? for_MA:'..ima..':'..pth..':'..i)
				if ima == 1 then
					ddae[pth].front = vec3(0,1,0)
					ddae[pth].inout = -ddae[pth].fr.y
					ddae[pth].len = math.abs(ddae[pth].fr.x-ddae[pth].to.x)
				end
				if ima == 2 then
					ddae[pth].front = vec3(-1,0,0)
					ddae[pth].inout = -ddae[pth].to.x
					ddae[pth].len = math.abs(ddae[pth].fr.y-ddae[pth].to.y)
				end
			end

			-- as forest
			local fo = createObject('TSForestItemData')
			if pnew ~= nil then
				fo:setField('shapeFile', 0, pnew)
			else
				fo:setField('shapeFile', 0, pth)
			end
			fo:setField('name', 0, 'f_'..tp)  -- to be found by scenetree.findObject
			fo.useInstanceRenderData = true
			fo.canSave = false
			ddae[pth].fo = fo

		end
	end
		lo('<< daeLoad:'..tp..':'..#afile)
end


local function daeIndex(tp, dae, suf)
	if suf then
		local atoken = U.split(dae, '/', true)
		local nm = U.split(atoken[#atoken], '.', true)[1]
		-- look for in
		dae = ''
		for k=1,#atoken-1 do
			dae = dae..'/'..atoken[k]
		end
		dae = dae..'/'..nm..'in'..'.dae'
	end
	for i,d in pairs(daePath[tp]) do
--            lo('??******** daeIndex:'..tostring(d)..':'..tostring(type(d))..'/'..tostring(nm)..':'..tostring(tp),true)
		local str = type(d) == 'string' and d or d[1]
		if str == dae then
			return i,dae
		end
	end
end

--[[
daeLoad('win')
daeLoad('door')
daeLoad('balcony')
daeLoad('roofborder', true)
U.dump(ddae, '??++ dae:')

daeLoad('_ext')
]]



--local forest = core_forest.getForestObject()
--lo('??+++++++++++++++++++ for:'..tostring(forest)..':'..tostring(im.GetKeyIndex(im.Key_X)))

--*************************************************************

local md_roof = 'flat'

local adesc, cmesh = {}, nil -- mesh_id -> desc, edited mesh ID
local aedit = {mesh = nil, desc = nil} -- mesh_id->(render_data, part_desc)
--local cpart = nil -- current edited mesh part ID
local cij
local cedit = {mesh = nil, part = nil, forest = nil, cval = {}} -- (id, id, item_key, starting_params_vals)

--local cuv = nil

local dforest = {} -- {{item = nil, mesh = nil, type = nil, ind = nil, prn = nil}} -- item_key -> (forest_item, parent_mesh_id, item_semantic_type, index in desc.df, desc of edited part)

local default = {
	floorheight = 3.2,
	winspace = 1.7,
	topmargin = 0.4,
	pillar = {dae = daePath.pillar[1], space = 2, inout = 0, margin = 0, yes = false},
	topthick = 0.1,
	doorbot = 0.001,
	maxundo = 10,
	topmat = U._PRD==0 and 'cladding_zinc' or 'm_stucco_white_bat',
}


local tb

local function forTerrain()
		lo('>>********************** forTerrain:'..tostring(#editor_roadUtils.getMaterialNames())..':'..tostring(scope)..':'..tostring(_mode))
--        local obj = scenetree.findObjectById(28369)
--        U.dump(obj.props, '??********************************** RD:'..tostring(obj.pid))

	tb = extensions.editor_terrainEditor.getTerrainBlock()
	if not tb then return end
		local ext = tb:getObjectBox():getExtents()
	L = ext.x
	out.L = L
		U.dump(aref, '?? for_ext:'..tostring(ext)..':'..L..':'..tableSize(aref))
--        U.dump(ext, '?? tb_SIZE:')

--    core_camera.setPosition(0, vec3(0, 15, 145) - 2*vec3(-10, 8, -1))
--    core_camera.setPosition(0, vec3(-1.38, 8.46, 115.75))
--++
	local cpos = core_camera.getPosition()
	cpos.z = core_terrain.getTerrainHeight(U.proj2D(cpos)) + 10
--    core_camera.setPosition(0, cpos)

	if false and U._PRD == 0 then -- and _mode == 'ROAD' then
		if true then
			-- "near the road"
			U.camSet({-176.84, 1093.29, 35.79, 0.273329, 0.0845505, -0.283164, 0.915402})
--            U.camSet({-189.31, 1006.07, 113.61, 0.284337, -0.0445115, 0.148119, 0.946167})
--            U.camSet({-424.12, 206.56, 367.69, 0.121995, 0.223943, -0.849117, 0.462566})
--                    lo('?? cam_set:')
--            core_camera.setPosition(0, vec3(-486.38, 226.46, 115.75))
--            core_camera.setRotation(0, quat(-0.270526, -0.139889, 0.437503, -0.846072))
		elseif true then
			-- middle cross
			-- [-974.64, 371.03, 70.75, 0.166968, -0.0478266, 0.271186, 0.946727]
			core_camera.setPosition(0, vec3(-974.64, 371.03, 70.75))
			core_camera.setRotation(0, quat(0.166968, -0.0478266, 0.271186, 0.946727))
			--[-991.49, 327.21, 94.67, -0.394102, -0.045285, 0.104788, -0.91195]
--            core_camera.setPosition(0, vec3(-991.49, 327.21, 94.67))
--            core_camera.setRotation(0, quat(-0.394102, -0.045285, 0.104788, -0.91195))
			editor.clearObjectSelection()
		elseif true then
			-- "near crossroad"
--            [-343.33, 167.10, 117.17, 0.330549, 0.263431, -0.564828, 0.708739]
			core_camera.setPosition(0, vec3(-343.33, 167.10, 117.17))
			core_camera.setRotation(0, quat(0.330549, 0.263431, -0.564828, 0.708739))

			editor.clearObjectSelection()
--            editor.selectObjectById(16211) -- TRONROUT0000000027820015_Chemin
		elseif true then
			-- "near the road"
			core_camera.setPosition(0, vec3(-486.38, 226.46, 115.75))
			core_camera.setRotation(0, quat(-0.270526, -0.139889, 0.437503, -0.846072))
		else
			-- for base conform, "close to hill"
			local pos = vec3(15, -72, 10)
			local hter = core_terrain.getTerrainHeight(pos)
			pos.z = pos.z + hter
			local q = quat(0.0490453, 0.179371, -0.947768, 0.259147)
			core_camera.setRotation(0, q)
			core_camera.setPosition(0, pos)
		end
	end
			if true then return end


	W.buildingGen(vec3(-496.4932642,241.80246353,99.30410767))


		lo('?? post_PNG:'..tostring(tb)..':'..core_terrain.getTerrainHeight(vec3(10,10,0))..':'..tostring(scenetree.findObject("thePlayer"):getPosition()))
	scenetree.findObject("thePlayer"):setPosition(vec3(0, 0, 103))
	--        local history = {}
	for x = 1,50 do
		for y = 1,50 do
			tb:setHeightWs(vec3(x, y, 0), 150)
		--                tb:setHeight(x, y, 10)-- max(0, z))
		--                history[#history + 1] = { old = 0, new = 500, x = x, y = y }
		end
	end
	tb:updateGrid()

	local gMin, gMax = Point2I(0,0), Point2I(0,0)
	local te = extensions.editor_terrainEditor.getTerrainEditor()
	te:worldToGridByPoint2I(vec3(0, 0), gMin, tb)
	te:worldToGridByPoint2I(vec3(300, 300), gMax, tb)

		lo('?? MIMA:'..tostring(gMin)..':'..tostring(gMax)..':'..tostring(core_terrain.getTerrainHeight(vec3(10,10,0))))

--[[
--    W.buildingGen(vec3(-11.4932642,23.80246353,99.30410767))
	local gMin, gMax = Point2I(0,0), Point2I(0,0)
	local te = extensions.editor_terrainEditor.getTerrainEditor()
	te:worldToGridByPoint2I(vec3(0, 0), gMin, tb)
	te:worldToGridByPoint2I(vec3(300, 300), gMax, tb)

		lo('?? MIMA:'..tostring(gMin)..':'..tostring(gMax)..':'..tostring(core_terrain.getTerrainHeight(vec3(10,10,0))))

	tb:updateGrid(vec3(0, 0), vec3(200, 200))
]]
end


local function forHeight(flist, i)
	if i == nil then i = #flist end
	local h = 0
	for k = 1,i do
		h = h + flist[k].h
	end
	return h
end


local function base2world(desc, ij, p)
	if not desc then desc = adesc[cedit.mesh] end
	if not desc then return end
	if not p then
		if not (ij[2] and ij[1] and desc.afloor[ij[1]]) then
			lo('!! ERR_base2world_NO:'..tostring(ij[1])..':'..tostring(ij[2])..':'..tostring(#desc.afloor))
		end
		p = U.mod(ij[2], desc.afloor[ij[1]].base)
	end
	--    U.dump(ij, '?? base2world:'..tostring(desc.afloor[ij[1]].base[ij[2]]))
	if not p then return end
	if desc.prn then
		p = p + adesc[desc.prn].pos + vec3(0,0,forHeight(adesc[desc.prn].afloor, desc.floor-1))
	end
	return p + desc.pos + desc.afloor[ij[1]].pos + vec3(0,0,forHeight(desc.afloor, ij[1]-1))
end

-- convert point world coordinates to wall u,v
local function world2wall(p, cw)
	local orig = base2world(adesc[cedit.mesh], cw.ij)
	local un,vn = cw.u:normalized(),cw.v:normalized()

	return vec3((p - orig):dot(un), (p - orig):dot(vn), 0)
end


local function name2type(nm)
--        lo('>> name2type:'..nm)
	return U.split(nm, '_', true)[2]
end


local function ray2segment(ray, a, b)
	local s = closestLinePoints(a, b, ray.pos, ray.pos + ray.dir)
	if 0 <= s and s <= 1 then
		return U.vang(ray.dir, a + (b-a)*s - ray.pos)
	end
	return math.huge
end


local function rayClose(ray, desc, ij, fortop)
	local small_ang = 0.01

	local vup = fortop and vec3(0,0,desc.afloor[ij[1]].h) or vec3(0,0,0)
	local u = desc.afloor[ij[1]].awall[ij[2]].u
	local vp = u:cross(vec3(0,0,1))
	local d = intersectsRay_Plane(ray.pos, ray.dir,
		base2world(desc, ij), vp)
	local p = ray.pos + d*ray.dir
	local s = closestLinePoints(
		base2world(desc, ij)+vup, base2world(desc, {ij[1],ij[2]+1}) + vup,
		p, p + vp)
--        lo('?? rayClose0:'..ij[1]..':'..ij[2]..':'..tostring(p)..':'..d..' s:'..s)
	if 0 <= s and s <= 1 then
--            lo('?? for_close:'..s..':'..U.vang(ray.dir, base2world(desc, ij) + s*u)..':'..tostring(ray.dir))
		if U.vang(ray.dir, base2world(desc, ij) + vup + s*u - ray.pos) < small_ang then
--                lo('?? rayClose:'..ij[1]..':'..ij[2])
			return true,ray
		end
	end
end


local function ray2rect(p, u, v, ray)
	if not ray then ray = getCameraMouseRay() end

	local d = intersectsRay_Plane(ray.pos, ray.dir, p, u:cross(v))
	local t = ray.pos + d*ray.dir
	local pu = (t - p):dot(u:normalized())
	local pv = (t - p):dot(v:normalized())

	return t,pu,pv
end


local function forSel()
	local aid = #asel > 0 and asel or {cedit.mesh}
	local adsc = {}
	for _,id in pairs(aid) do
		adsc[#adsc+1] = adesc[id]
	end
	return adsc
end


local function forBuilding(desc, cb, ij, aij, dbg)
--    local ij = nil
--    if cpart ~= nil then
--        ij = aedit[cpart].desc.ij
--    end
--            U.dump(desc.selection, '?? forBuilding:'..tostring(aij)..':'..tableSize(desc.selection))
	local forscope = ij ~= nil and true or false
	if desc == nil then return end
	ij = ij ~= nil and ij or cij
	if not aij and desc.selection then
		aij = desc.selection
	end
	local cfloor
		if dbg then
			U.dump(ij, '?? forBuilding2:'..tostring(desc))
		end
	local aijside
	if scope == 'side' and ij then
--            U.dump(cij, '?? for_IJ:')
		if #ij == 0 then ij = cij end
		aijside = W.forSide(ij)
--        U.dump(aijside, '?? aijside:')
	end
	for i,f in pairs(desc.afloor) do
		for j,w in pairs(f.awall) do
			local tocall = false
			if ij and not ij[2] and i == cfloor then
				if not ({building=1,floor=1})[scope] then
					break
				end
			end
					if dbg then
						lo('?? FB:'..tostring(aij)..':'..tostring(scope)..':'..tostring(ij and ij[2] or nil)..':'..i..':'..tostring(cfloor),true)
					end
			cfloor = i
			if aij and not forscope then -- ij ~= true then
				if #U.index(aij[i], j) > 0 then
--                if desc.selection[i] and desc.selection[i][j] then
					tocall = true
				end
--[[
				if #U.index(desc.selection, U.stamp({i,j},true)) > 0 then
					tocall = true
--                        U.dump(desc.selection, '?? to_call:'..i..':'..j)
				end
]]
			else
				--            lo('?? forBuilding:'..i..':'..j)
				if scope == 'building' or ij == nil or #ij == 0 or ij == true then
					tocall = true
				elseif scope == 'floor' then
					if i == ij[1] then tocall = true end
				elseif scope == 'side' and aijside then
--                    sameSide()
					if #U.index(aijside[i], j)>0 then tocall = true end
--                        U.dump(ij, '??  FB_for_SIDE:'..i..':'..j..':'..tostring(aijside[i][j])..':'..tostring(tocall))
--                    if j == ij[2] then tocall = true end
				elseif scope == 'wall' then
					if i == ij[1] and j == ij[2] then tocall = true end
				elseif scope == 'top' then
					if i == ij[1] then tocall = true end
				end
			end
				if dbg then
					lo('?? FB2:'..tostring(tocall)..':'..tostring(scope)..':'..tostring(ij and ij[2] or nil)..':'..i..':'..tostring(cfloor),true)
				end
			if tocall then cb(w, {i,j}) end
		end
		cfloor = i
	end
end


W.ctest = function()
  if true then
    M.matReplace('/tmp/bat/'..'t3.dae')
    if true then return false end

    local w = adesc[cedit.mesh].afloor[1].awall[1]
    local pth = w.win
    local obj = createObject('TSStatic')
    obj:setField('shapeName', 0, pth)
    obj.isMesh = true
    obj.canSave = true
    obj.hidden = true
  --        obj.hidden = false -- true
    obj:registerObject('dae_'..w.df[pth][1])
            lo('??^^^^^^^^^^^^^^^^^^^^ for_win:'..pth..':'..tostring(obj.materials))

      local amat = obj:getMeshMaterialNames()
      U.dump(amat, '?? amat:')
    return false
  end
  if true then return true end
  U.dump(cij, '?? ctest:')
  local res = W.ifPairEnd(nil,true)
      lo('?? res:'..tostring(res))
  return false
end


W.daeExport = function()
--    local jdesc = jsonEncode(adesc)
--      lo('??^^^^^^^^^^^^^^^^^ jdesc:'..#jdesc,true)
    lo('>> daeExport:'..tostring(cedit.mesh)..':'..tableSize(adesc))
  local mat
--[[
  if false and U._PRD == 0 then
    local c = '1 0.5 0 1'
    local mat = createObject("Material")
    mat:setField("diffuseColor", 0, c)
--        mat:setField("diffuseMap", 0, c)
    mat:registerObject('mat_'..'dummy')
    if not W.ctest() then return end
  end

    local str = jsonEncode(adesc)
    local jdesc = jsonDecode(str)
    W.recover(jdesc)
            if true then return end
]]
	if not cedit.mesh then
		if U._PRD == 0 then
			for id,d in pairs(adesc) do
				cedit.mesh = id
			end
		else
			return
		end
	else
		-- quit edit
		W.houseUp(adesc[cedit.mesh], cedit.mesh) --, nil, nil, 'mat_'..'dummy')
	end

	local function f2m(pth, fid)
		local obj = createObject('TSStatic')
		obj:setField('shapeName', 0, pth)
		obj.isMesh = true
		obj.canSave = true
		obj.hidden = true
--        obj.hidden = false -- true
		obj:registerObject('dae_'..fid)
		local oid = obj:getID()
--            lo('?? for_idf:'..tostring(oid)..'/'..pth)
		return oid,obj
	end

	local desc = adesc[cedit.mesh]
	editor.clearObjectSelection()
	local aid = {}
	local afi = {}
--		lo('?? pre_build:'..tostring(desc)..':'..tostring(tableSize(editor.selection)))
	editor.selection.forestItem = {}
	local id = 0
	scope = 'building'
	forBuilding(desc, function(w,ij)
--		U.dump(w.df, '?? for_wall:'..ij[1]..':'..ij[2])
		for pth,list in pairs(w.df) do
--			lo('?? for_DAE:'..pth)

--            editor.selectObjectsByRef({ddae[k].fo})
			for _,id in pairs(list) do
				local item = dforest[id].item
				local pos = item:getPosition()
				local scale = item:getScale()
				local trans = item:getTransform()
				local rot = trans:toEuler() -- need Z
--					lo('?? for_fi:'..tostring(item)..':'..tostring(pos)..':'..tostring(scale)..':'..tostring(rot))
--                    U.dump(editor.matrixToTable(item:getTransform()), '?? for_fi:'..tostring(item)..':'..tostring(pos)..':'..tostring(scale)..':'..tostring(rot))
--                    U.dump(trans,'?? for_fi:'..tostring(item)..':'..tostring(pos)..':'..tostring(scale))
				local oid,om = f2m(pth, id)
				if oid then
					groupEdit:add(om.obj)
					aid[#aid+1] = oid
				end
				om:setTransform(trans)
			end
		end
	end) --, {1,1})
	aid[#aid+1] = cedit.mesh
	editor.selectObjects(aid, editor.SelectMode_Add)
		lo('?? daeExport:'..#aid..':'..#afi..':'..tostring(tableSize(editor.selection))) --..':'..tostring(editor.selection.forestItem))--..':'..tostring(aid[1])..':'..tostring(editor.selection.object.id)..':'..tostring(tableSize(editor.selection)))

	scope = nil
	W.markUp()

	editor_fileDialog.saveFile(
		function(data)
			worldEditorCppApi.colladaExportSelection(data.filepath)
				lo('?? editor_fileDialog:'..tostring(data.filepath))
			M.matReplace(data.filepath) --,'mat_dummy')
		end,
		{{"Collada file",".dae"}},
		false,
		"/",
		"File already exists.\nDo you want to overwrite the file?"
	)

--        U.dump(aid, '?? daeExport:'..#aid..':'..tostring(tableSize(editor.selection)))
end
--                om:setPosition(vec3(2,2,0))
--                om:setPosition(pos)
--[[
			id = id + 1
			local obj = createObject('TSStatic')
			obj:setField('shapeName', 0, pth)
			obj.isMesh = true
			obj.canSave = false
			obj.hidden = true
			local nm = 'dae_'..id
			local otmp = scenetree.findObject(nm)
			if otmp then
				lo('?? exists:'..nm)
				otmp:unregisterObject()
			end
]]
--            obj:registerObject(nm)
--            local oid = obj:getID()
--                lo('?? for_id:'..tostring(oid))
--                U.dump(obj:getWorldBox(),'?? fo:'..tostring(pth)..':'..tostring(obj.id))

--[[
			local nm = 'dae_'..tp..'_'..i
			local otmp = scenetree.findObject(nm)
			if otmp then otmp:unregisterObject() end
			obj:registerObject(nm) -- needed to get box
			local box = obj:getWorldBox()
]]
--[[
				local oid,om = f2m(pth, id)
					lo('?? for_idf_:'..tostring(oid))
				if oid then
					groupEdit:add(om.obj)
					aid[#aid+1] = oid
				end
]]

--                obj:registerObject('dae_'..id)
--                local oid = obj:getID()
--                lo('?? ifforest:'..id..':'..tostring(dforest[id]))
--                table.insert(editor.selection.forestItem, dforest[id].item)
--                afi[#afi+1] = dforest[id].item
--                aid[#aid+1] = id
--                break
--            if #aid > 0 then break end

--    for i,o in pairs(adesc) do
--        aid[#aid+1] = o.id
--    end
--[[
	for k,_ in pairs(adesc) do
		if k ~= cedit.mesh then
			aid = {k}
			break
		end
	end
]]
--    editor_forestEditor.selectForestItems(afi)


local function forSide(ij, fromselect)
	if not ij then ij = cij end
--        U.dump(ij, '>> forSide:'..tostring(cij)..':'..tostring(cedit.mesh))
	if not ij or not ij[2] or not cedit.mesh then return end
	local aij = {}
	local desc = adesc[cedit.mesh]
	local base = desc.afloor[ij[1]].base
	local u = U.mod(ij[2]+1,base) - base[ij[2]]
	local buf
	if fromselect then
		buf = {}
		forBuilding(desc, function(w, ij)
			if not buf[ij[1]] then buf[ij[1]] = {} end
			buf[ij[1]][#buf[ij[1]]+1] = ij[2]
		end)
	end
--        U.dump(buf, '?? forSide_buf:')
	for i,f in pairs(desc.afloor) do
		aij[#aij + 1] = {}
		if not buf or buf[i] then

			for j = 1,#f.base do
				if not buf or #U.index(buf[i],j) > 0 then

					local cu = U.mod(j+1,f.base) - f.base[j]
		--                    lo('?? if_side:'..j..':'..U.vang(cu, u))
					if U.vang(cu, u) < small_ang then
						aij[i][#aij[i] + 1] = j
					end

				end
			end

		end
	end
--        lo('<< forSide:'..tableSize(aij))
	return aij
end


local function inSide(ij)
	if not ij then ij = cij end
	if not ij then return end

	local floor = adesc[cedit.mesh].afloor[ij[1]]
	for j = ij[2],ij[2]+1 do
		local upre = U.mod(j, floor.base) - U.mod(j-1, floor.base)
		local upost = U.mod(j+1, floor.base) - U.mod(j, floor.base)
		if U.vang(upre, upost) > small_ang then return false end
	end
	return true
end


local function sameSide(buf)
--        U.dump(buf,'>> sameSide:'..tostring(tableSize(buf)))
	if not buf or tableSize(buf)==0 then return end
	local akey = {}
	for k,r in pairs(buf) do
		akey[#akey+1] = k
	end
	table.sort(akey)
--            U.dump(buf, '?? sameSide2:'..akey[1])
	local aij = forSide({akey[1], buf[akey[1]][1]})
--            U.dump(aij, '?? sameSide:')
--    for _,j in pairs(buf[akey[1]]) do
--!!        for j,_ in pairs(buf[akey[1]]) do
--        aij = forSide({akey[1], j})
--        break
--    end
--        U.dump(aij, '?? sameSide.id_SIDE:')
	for i,row in pairs(buf) do
		if not aij[i] then
--            lo('?? fail'..i, true)
			return false
		end
--        if #U.index(akey, i) == 0 then return false end
		for _,j in pairs(row) do
--            for j,_ in pairs(row) do
			if #U.index(aij[i],j) == 0 then
--                lo('?? fail_j:'..i..':'..j, true)
				return false
			end
		end
	end
	return true
end


local shmap = {}


local function hmapChange(p, h)
--[[
	if not shmap[p.x] then
		shmap[p.x] = {}
	end
	if not shmap[p.x][p.y] then
--        lo('?? for_h:'..p.x..':'..p.y..':'..core_terrain.getTerrainHeight(p))
		shmap[p.x][p.y] = core_terrain.getTerrainHeight(p)
	end
]]
--    tb:setHeight(p.x, p.y, h)
	tb:setHeightWs(p, h)
end


local function restore(tosave)
	if not tosave then
		for j,c in pairs(shmap) do
			for i,h in pairs(c) do
	--            tb:setHeightWs(vec3(j,i), 0)
				tb:setHeightWs(vec3(j,i), h)
			end
		end
	end
--[[
	for j,c in pairs(shmap) do
		for i,h in pairs(c) do
			tb:setHeightWs(vec3(j,i), 0)
--            tb:setHeightWs(vec3(j,i), h)
		end
	end
]]
--        lo('>> restore:')
	if not tb then
		tb = extensions.editor_terrainEditor.getTerrainBlock()
	end
	if not tb then return end
	tb:updateGrid() -- TODO: no arguments makes warning
	out.avedit = {}
	out.apick = {}
	out.apoint = {}
	out.inconform = false
end


local function hmapChange_(p, h)
--    shmap[p.x..'_'..p.y] = core_terrain.getTerrainHeight(p)
	if not out.inconform then
	end
	shmap[#shmap+1] = {p, core_terrain.getTerrainHeight(p)}
--            h = core_terrain.getTerrainHeight(p)
	tb:setHeightWs(p, h)
end


local function restore_()
--        lo('>> restore:')
	for i = #shmap,1,-1 do
--    for _,v in pairs(shmap) do
		local v = shmap[i]
		tb:setHeightWs(v[1], v[2])
	end
	if not tb then
		tb = extensions.editor_terrainEditor.getTerrainBlock()
	end
	tb:updateGrid() -- TODO: no arguments makes warning
	out.avedit = {}
	out.apick = {}
	out.apoint = {}
	out.inconform = false
end


local function terrainBackup(xmm, ymm)
	lo('>> terrainBackup:',true)
	for x = xmm[1],xmm[2] do
		for y = ymm[1],ymm[2] do
			if not shmap[x] then
				shmap[x] = {}
			end
			shmap[x][y] = core_terrain.getTerrainHeight(vec3(x,y))
		end
	end
	return shmap
end


local function confSave()
	out.avedit = {}
	out.apick = {}
	out.apoint = {}

	shmap = {}
end


local function forestClean(desctop, dbg)
	local cforest = {}
	if desctop and desctop.df ~= nil then
		for dae,list in pairs(desctop.df) do
			for i,key in ipairs(list) do
					if dbg then lo('?? fC_key:'..key..':'..tostring(dforest[key].item)) end
				if key == cedit.forest then
					cforest[dae] = i
--                    cforest = {dae, i}
				end
				if dforest[key] then
					editor.removeForestItem(fdata, dforest[key].item)
				end
				dforest[key] = nil
				if key == cedit.forest then
--                    lo('??^^^^^^^^^^____________ cforest_DEL:'..key..':'..tostring(list[1]))
--                    cedit.forest = list[1]
					cforest[dae] = 1
				end
			end
			desctop.df[dae] = {
				scale = desctop.df[dae].scale,
				span = desctop.df[dae].span,
				fitx = desctop.df[dae].fitx}
		end
			if dbg then
				U.dump(desctop.df, '?? fC_done:')
			end
	end
	return cforest
end


W.onQuit = function(forestoff)
      lo('>>^^^^^^^^^^^^^^^^^^^^ onQuit:'..tostring(cedit.mesh)..':'..tostring(adesc and #adesc or nil))
  if cedit.mesh then
    -- quit edit
    W.houseUp(adesc[cedit.mesh],cedit.mesh)
    scope = nil
    W.markUp()
  end
  local oedit = scenetree.findObject('edit')
		lo('?? onQuit:'..tostring(cedit.mesh)..':'..#scenetree.getAllObjects()..' edit:'..tostring(oedit))
  if oedit then
    scenetree.MissionGroup:removeObject(groupEdit)
    oedit:delete()
    groupEdit = nil
--    lo('?? edit_down:'..tostring(scenetree.findObject('edit')))
  end

  if forestoff ~= false then
    for _,s in pairs(dforest) do
      editor.removeForestItem(fdata, s.item)
    end
    for id,d in pairs(adesc) do
      local obj = scenetree.findObjectById(id)
      if obj then
        obj:delete()
        d.id = nil
      end
    end
  end
  if true then return jsonEncode(adesc) end

	if true then return end
	local olist = scenetree.getAllObjects()
	for i,oname in pairs(olist) do
		if false and string.find(oname, 'o_building_') == 1 then
			local obj = scenetree.findObject(oname)
--            lo('?? toclean:'..tostring(obj.name)..':'..tostring(obj:getId())..':'..tostring(adesc[obj:getId()]))
			local desc = adesc[obj:getId()]
			for i,f in pairs(desc.afloor) do
				for j,w in pairs(f.awall) do
					forestClean(w)
				end
			end
			forestClean(desc)
		end
	end
--  return adesc
--[[
	if cedit.mesh then
		local desc = adesc[cedit.mesh]
		forestClean(desc)
		for i,f in pairs(desc.afloor) do
			for j,w in pairs(f.awall) do
				forestClean(w)
			end
		end
	end
]]
end


W.redo = function()
  lo('?? redo:')
end


local function toJSON(desc)
		lo('>>^^^^^^^^^^^^^^^^ toJSON:'..#asave)
--    if #asave > 5 then
--        table.remove(asave, #asave)
--    end
	table.insert(asave, 1, {math.floor(socket.gettime()), deepcopy(desc)})
--  editor.history:commitAction('BATadesc', {items = adesc}, nil, W.redo, true)
--        lo('?? pre_ch:'..asave[1][2].afloor[2].h)
--        desc.afloor[2].h = 5
--        lo('?? post_ch:'..asave[1][2].afloor[2].h)
	if #asave > default.maxundo then
		table.remove(asave, #asave)
	end
--        lo('?? toJSON_done:'..#asave,true)--..':'..fsave) --..tostring(adesc[cedit.mesh]))
--    csave = 1
	if U._PRD == 0 then
		local fname = fsave..'_'..asave[1][1]..'.json'
		jsonWriteFile(fname, desc)
			lo('<< toJSON:'..fname)
	end
end


W.recover = function(jdesc, cb, keep)
    lo('>> recover:'..tostring(keep)..':'..tableSize(adesc))
  if not keep then
    adesc = {}
  end
  local aid = {}
  for id,d in pairs(jdesc) do
      	lo('?? for_desc:'..id..':'..tostring(adesc[tonumber(id)]))
	local cid = tonumber(id)
    adesc[cid] = U.fromJSON(d)
	for i,d in pairs(adesc) do
		if d.idr == cid then
--				lo('?? if_skip:'..tostring(adesc[d.idr].pos)..':'..tostring(d.pos))
			if adesc[d.idr].pos:distance(d.pos) < small_dist then
					lo('?? recover_skip_id:'..cid)
				adesc[cid] = nil
				goto continue
			end
		end
	end
	aid[#aid+1] = cid
	if d.acorner_ then
--			U.dump(d.acorner_, '?? recover_corner:')
		for i,c in pairs(d.acorner_) do
			for _,s in pairs(c.list) do
--					U.dump(s, '?? for_s:')
				for j,p in pairs(s) do
--					lo('?? for_s:'..i..':'..j..':'..tostring(s[j..''])..':'..p..':'..tostring(tonumber(j)))
					if s[j..''] and tonumber(j) then
						s[tonumber(j)] = s[j..'']
						s[j..''] = nil
--						lo('?? ac_replaced:'..i..':'..j..':'..s[j])
					end
--					U.dump(p, '?? for_p:'..i..':'..j)
				end
			end
		end
	end
	for _,f in pairs(d.afloor) do
		f.awplus = {}
	end
	::continue::
--		U.dump(d.afloor[2].awplus, '?? if_AWPLUS:')
  end
  inrecover = cb -- true

  return aid
--[[
    lo('<< recover:'..tableSize(adesc))
    for k,d in pairs(adesc) do
      lo('?? if_desc:'..k..':'..tostring(adesc[k]))
    end
]]
end


local function fromJSON(fname)
	local desc = adesc[cedit.mesh]
--    if not desc or not desc.id then return end
	-- remove current
		lo('>> fromJSON:'..tostring(fname)..':'..tostring(desc)..' asave:'..#asave)
	local function clean()
		if desc then
			cij = nil
			forBuilding(desc, function(w,ij)
	--                U.dump(ij,'?? FC:')
				forestClean(w)
				w.df = {}
			end)
			for i,f in pairs(desc.afloor) do
				forestClean(f)
				f.df = {}
			end
			scenetree.findObject('edit'):deleteAllObjects()
			local obj = scenetree.findObjectById(desc.id)
			if obj then
				obj:delete()
			end

			for _,s in pairs(dforest) do
				editor.removeForestItem(fdata, s.item)
			end
		end
	end
--            if true then return end
--            lo('?? RESTORE:'..csave..':'..tostring(asave[csave])..'-'..desc.id)
	-- restore from history
	if fname then
		clean()
		desc = jsonReadFile(fname)
			lo('?? fromJSON:'..fname..':'..tostring(desc))
		desc = U.fromJSON(desc)
	elseif #asave > 1 then
		clean()
		desc = deepcopy(asave[2][2])
		table.remove(asave, 1)
			lo('??=================== rollback:'..#asave..':'..tostring(desc),true)
	end
	if not desc then return end
	-- post-process description
	desc.id = nil
	desc.selection = nil
	for i,f in pairs(desc.afloor) do
		if f.achild then
			for k,c in pairs(f.achild) do
				c.id = nil
				for ci,cf in pairs(c.afloor) do
					for cj,cw in pairs(cf.awall) do
						cw.id = nil
						for dae,cd in pairs(cw.df) do
							cw.df[dae] = {scale = cd.scale}
						end
						cw.agrid = {}
					end
					for dae,cd in pairs(cf.top.df) do
						cf.top.df[dae] = {scale = cd.scale}
					end
				end
			end
		end
		for j,w in pairs(f.awall) do
			w.id = nil
--            local dscale = {}
			for dae,d in pairs(w.df) do
				w.df[dae] = {scale = d.scale}
			end
			w.agrid = {}
--            w.df = {}
		end
		for dae,d in pairs(f.top.df) do
			f.top.df[dae] = {scale = d.scale}
		end
--        f.top.df = {}
	end
		lo('?? if_corner:'..tostring(desc.acorner_))
--            if i > 1 then break end
--			if s and #s.list>0 then

--            lo('?? fj1:'..tostring(desc.pos)..':'..tostring(desc.id))
--        U.dump(desc.pos, '?? fj1:'..tostring(desc.pos))
	W.houseUp(desc)
	if not fname then
		W.houseUp(nil, desc.id)
	else
		cedit.mesh = nil
	end
			lo('<< fromJSON:'..tostring(desc.id)..':'..tostring(cedit.mesh)..':'..tostring(inrollback))
--            U.dump(desc.afloor[2].base, '?? BASE:')
	return desc.id
end


local restored

W.ifSaved = function()
  local fname = editor.getLevelPath()..'bat.json'
  local file = io.open(fname, "r")
  if file then
    out.saved = true
    file:close()
  end
end

local function up(d, inreload, inact, mode)
--		print('?? W.up:'..tostring(daeloaded)..':'..tostring(mode))
--		lo('>>++++++++++ W.up:'..forestName..':'..tostring(inreload)..':'..tostring(scenetree.findObject(forestName))..':'..tostring(restored)..':'..tostring(inact)..':'..tostring(forest)..' edit:'..tostring(scenetree.findObject('edit'))..':'..tostring(groupEdit)..':'..tostring(D))
	local oedit = scenetree.findObject('edit')
	if inact and D then
		D.inject(inreload)
	end
	if Ter then
		Ter.inject(D)
	end
	if not oedit then
		groupEdit = createObject("SimGroup")
	--      lo('?? sG_up:'..tostring(groupEdit))
		groupEdit:registerObject('edit')
	--      lo('?? e_regd:'..tostring(scenetree.findObject('edit')))
		groupEdit = scenetree.findObject('edit')
		scenetree.MissionGroup:addObject(groupEdit)
--    local sg = createObject("SimGroup")
--    sg:registerObject('edit')
--    oedit = createObject('edit')
--    groupEdit = scenetree.findObject('edit')
        	lo('?? edit_CREADTED:'..tostring(scenetree.findObject('edit'))) --oedit))
 	end
--[[
  if groupEdit == nil then
  --    groupEdit = createObject('edit')
  --    scenetree.MissionGroup:registerObject('edit')
    groupEdit = createObject("SimGroup")
--    groupEdit:registerObject('edit')
--    groupEdit = scenetree.findObject('edit')
      lo('??___________ groupEdit_up:'..tostring(groupEdit))
--  else
--    lo('??_______________ gE_EXISTS:')
  end
  local oedit = scenetree.findObject('edit')
  if not oedit then
--    groupEdit = createObject('edit')
--    groupEdit:registerObject('edit')
--    groupEdit = scenetree.findObject('edit')
      lo('??___________ edit_CREATED:'..tostring(scenetree.findObject('edit')))
  else
    lo('??_______________ edit_EXISTS:')
  end
]]

	W.ifSaved()
--			lo('>>++++++++++ W.up2:')
	--!!	D = d
	if U._PRD == 0 then
		local cancs = editor.getPreference("roadEditor.general.aiRoadsSelectable")
		lo('?? is_CC:'..tostring(cancs))
		if not cancs then
		local aiRoadsPtr = im.BoolPtr(editor.getPreference("roadEditor.general.aiRoadsSelectable"))
		editor.setPreference("roadEditor.general.aiRoadsSelectable", aiRoadsPtr[0])
		end
	end
--            if true then return end
	if U._PRD == 0 and not W.ui.injunction and not ({conf=0})[U._MODE] and scenetree.findObject("thePlayer") then
		scenetree.findObject("thePlayer"):delete()
	end
	if U._PRD == 0 and W.ui.injunction then
		scenetree.findObject("thePlayer"):setPosition(vec3(0, 0, core_terrain.getTerrainHeight(vec3(0,0))))
	end
--    scenetree.findObject("theTerrain"):delete()

--      lo('?? if_forest2:'..tostring(forest)..':'..tostring(forestName),true)
	if not inact or not forest then
--        lo('?? if_forest3:'..tostring(forest)..':'..tostring(forestName),true)
		if forestName ~= 'theForest' then
        		lo('??+++++++ forest_up:'..tostring(forestName),true)
			forest = worldEditorCppApi.createObject("Forest")
			forest:registerObject(forestName)
		else
			forest = core_forest.getForestObject()
		end
	end
	fdata = forest:getData()
	  lo('>>____++++_____ up:'..tostring(scope)..':'..tostring(forest.name)..' fdata:'..tostring(fdata)) --..':'..tostring(editor.getPreference("roadEditor.general.aiRoadsSelectable")))

--			lo('>>++++++++++ W.up3:')
	W.matUp()

	if not daeloaded then
		if true then
--            for _,tp in pairs({'balcony','corner','doors','pillar','wall_pilaster','store_front','stairs','string_course'}) do
--			for _,tp in pairs({'balcony','corner','doors','wall_pilaster','store_front','stairs','string_course','windows','roof_border'}) do
			for _,tp in pairs({'balcony','corner','doors','pillar','wall_pilaster','store_front','stairs','string_course','windows','roof_border'}) do
--            for _,tp in pairs({'balcony','corner','doors','pillar','wall_pilaster','roof_border','store_front','stairs','string_course','windows'}) do
				daeLoad_(assetPath..'/', tp)
			end
			default.pillar.dae = daePath['pillar'][1]
--                U.dump(daePath['pillar'], '?? DDAE:')
		else
	--        daeImport(assetPath..'/windows', 'win')
			daeLoad('win')
	--        daeImport(assetPath..'/doors', 'door')
			daeLoad('door')
			daeLoad('balcony')
			daeLoad('pillar')
			daeLoad('corner')
			daeLoad('roofborder', true)
		end
--                U.dump(ddae, '??=================================== dae:')
				lo('??++ dae:'..tableSize(ddae))

--        if _mode == 'BASE' then daeLoad('_ext') end

		daeloaded = true

		forTerrain()

		local decalPath = {
--            '/lua/ge/extensions/editor/gen/assets/items.level.json',
			'/levels/south_france/main/MissionGroup/decalroads/AI_decalroads/items.level.json',
--            '/levels/ce_test/main/MissionGroup/decalroads/AI_decalroads/items.level.json',
		}
		if U._MODE == 'conf' then
--		if U._PRD == 0 or U._MODE == 'conf' then
			for _,p in pairs(decalPath) do
--				L = 4096 --??
				adec = D.decalsLoad()--, grid)
--					D.decalPlot()
	--            adec,aref = D.decalsLoad(p, L, grid)
			end
		end
	end
	if mode then
		U._MODE = mode
		adec = D.decalsLoad()--, grid)
	end

--			lo('>>++++++++++ W.up33:'..tostring(restored))
	if stage and not restored then
--			lo('>>++++++++++ W.up4:')
		restored = fromJSON(fsave..'/'..stage..'/save')
	end

		lo('?? up.if_rec:'..tostring(inrecover)..':'..tableSize(adesc))
	if inrecover then
		local aid = {}
		for id,d in pairs(adesc) do
		aid[#aid+1] = id
		end

		for i,id in pairs(aid) do
			lo('??^^^^^^^^^^^^^^^^^^^ recover_house:'..id)
	--[[
		local obj = scenetree.findObjectById(id)
		if obj then
			obj:delete()
		end
		d.id = nil
	]]
		W.houseUp(adesc[id])
		adesc[id] = nil
		end
		inrecover()
		inrecover = nil
	end

--        W.test()
	if U._PRD == 0 then
		W.test()
		if inreload then
			W.pretest(true)
		end
		if D then
			D.test()
		end
		if Ter then
			Ter.test()
		end
	end
end


W.floorClear = function(f)
	if f.awplus then
		-- clean up awplus
		for i,wp in pairs(f.awplus) do
			if wp.id then
				local obj = scenetree.findObjectById(wp.id)
				if obj then obj:delete() end
			end
		end
		f.awplus = {}
	end
end


local function clear()
--		if true then return end
--			lo('?? w_CLR:'..#dforest..':'..forestName)
--            lo('>>_____________________________________ W.clear:')
	_dbdrag = false
	if U._PRD == 0 then
--        local console = require('/lua/ge/extensions/ui/console')
--        console.clearConsole() -- not working
	end
	out.avedit = nil
	out.dyell = nil
--	out.ccommand = ''
		lo('?? clear:') --..#editor.getAllRoads())
	restore()
	local fedit = scenetree.findObject('edit')
	if true or U._PRD == 0 then
		if U._MODE == 'ter' then
			Ter.clear()
		elseif W.ui.injunction or ({conf=0})[U._MODE] then
			D.clear()
			if U._PRD == 0 then
				Ter.clear()
			end
		elseif false and U._PRD == 0 then
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
	end
	if fedit then
		fedit:deleteAllObjects()
	end
	if forest then
		fdata = forest:getData()
			lo('??_____________________ clear.for_items: adesc:'..tableSize(adesc)) --..':'..#fdata:getItems())
		for id,desc in pairs(adesc) do
			local obj = scenetree.findObjectById(id)
			if obj ~= nil then
		--            local obj = scenetree.findObject(oname)
		--            lo('?? toclean:'..tostring(obj.name)..':'..tostring(obj:getId())..':'..tostring(adesc[obj:getId()]))
		--            local desc = adesc[obj:getId()]
				for i,f in pairs(desc.afloor) do
					for j,w in pairs(f.awall) do
			--                        U.dump(w.df, '?? to_FCLEAN:'..i..':'..j..':'..tostring(fdata))
						forestClean(w)
					end
				end
				forestClean(desc)
				obj:delete()
			else
				lo('!! ERR_clear_BUILDING_LOST:'..id)
			end
		end
		for _,s in pairs(dforest) do
			editor.removeForestItem(fdata, s.item)
	--        lo('?? f_remove:'.._)
		end

		out = {avedit = {}, wsplit = nil, scope = scope, fdata = fdata}
		if U._PRD == 0 then
			if D then
				D.widthRestore()
			end
			out.ccommand = ''
		end
	end
	lo('<< W.clear:'..tostring(fdata)) --..':'..#editor.getAllRoads())--..':'..out.ccommand)
end

--*********************************************************************

-- extent of height for scope
local function extHeigh(buf)
	local desc = adesc[cedit.mesh]
	if not desc then return end

	local zmi,zma = math.huge,0
	forBuilding(desc, function(w, tij)
		local cp = base2world(desc, {tij[1], 1})
		if cp.z < zmi then
			zmi = cp.z
		end
		if cp.z + desc.afloor[tij[1]].h > zma then
			zma = cp.z + desc.afloor[tij[1]].h
		end
	end)
	return zmi,zma
end


--local borderID = nil
--local function to(tp, pos, ang)
--local function to(daedesc, pos, ang)
local function to(desc, dae, pos, vang, cforest, scale, ind)
	scale = scale ~= nil and scale or 1
--            U.dump(desc.df, '?? to:'..tostring(dae))
	if dae == nil or desc.hidden then return end
	local daedesc = ddae[dae]
	if not daedesc then
		lo('!! ERR.to_NO_daedesc:'..dae)
		return
	end
	local fobj = daedesc.fo
--        lo('?? TO:'..tostring(fobj), true)
	local mtx = MatrixF(true)
--                mtx:setFromEuler(vec3(math.pi,0,0))
	local mrotx = MatrixF(true)
	mrotx:setFromEuler(vec3(vang.x, 0, 0))
	local mroty = MatrixF(true)
--            vang.y = 0.2
	mroty:setFromEuler(vec3(0, vang.y, 0))

	mtx:setFromEuler(vec3(0, 0, vang.z))
	mtx:mul(mrotx)
	mtx:mul(mroty)
--    mtx:scale(vec3(2,1,1))
--[[
	local mrot = MatrixF(true)
	mrot:setFromEuler(vec3(0, 0, ang))
	mtx:setFromEuler(vec3(0.5, 0, 0))
	mrot:mul(mtx)
	mtx = mrot
--    mtx:setFromEuler(vec3(0, 0, ang))
]]
	mtx:setPosition(pos)
	local sc = scale
	if type(scale) ~= 'number' then
		mtx:scale(scale)
--        mtx:scale(vec3(3,1,0.3))
		sc = 1
	end
--        lo('??+++++++++ to.scale:'..tostring(sc), true)
	local item = fdata:createNewItem(fobj, mtx, sc)

--            lo('?? scale:'..tostring(item:getScale()))
--            if borderID ~= nil then
--                item = fdata:createNewItem(borderID, mtx, 1)
--            end
--            lo('?? to:'..tostring(pos)..':'..tostring(item:getScale()))
--    list[#list + 1] = newItem
	local key = item:getKey()
--            if dae == '/assets/doors/s_R_HS_A_DR_01_0.9x2.2_A.dae' then
--                U.dump(item:getData())
--            end
	dforest[key] = {item = item, mesh = nil, type = daedesc.type, prn = desc} -- forest_item<->parent_mesh link

--            lo('?? to.key:'..tostring(key))
	if desc.df == nil then
		desc.df = {}
	end
	if desc.df[dae] == nil then
		desc.df[dae] = {}
	end
--!!    desc.df[dae].scale = scale
	U.push(desc.df[dae], key)
	dforest[key].ind = ind or #desc.df[dae]
--    lo('??*** to_pushed:'..key..'>'..dae)

	if cforest ~= nil then
		if cforest[dae] == #desc.df[dae] then
--        if cforest[1] == dae and cforest[2] == #desc.df[dae] then
			cedit.forest = key
--			lo('?? reset_cforest:'..key)
		end
	end
	return key
end


--local function align(m, pfr, pto, space)
--end


local function wallUp(desc, bdesc, ispany, dbg)
--    U.dump(desc, '>> wallUp:')
--        lo('>> wallUp:'..tostring(tableSize(desc.agrid)))
	if not indrag then
--        lo('>> wallUp:'..desc.mat..':'..tostring(cedit.forest)) --..':'..tostring(desc.door))
	end
--            local base = bdesc.afloor[desc.ij[1]].base
--            desc.u = U.mod(desc.ij[2]+1, base) - U.mod(desc.ij[2], base)
	local an = {desc.u:cross(desc.v):normalized()}
--	local ang = math.atan2(an[1].y, -an[1].x) - math.pi/2
	local ang = math.atan2(-an[1].y, an[1].x) - math.pi/2
--[[
	local auv = {
		{u = desc.uv[1], v = desc.uv[4]},
		{u = desc.uv[1], v = desc.uv[2]},
		{u = desc.uv[3], v = desc.uv[2]},
		{u = desc.uv[3], v = desc.uv[4]},
	}
			if desc.ij[1] == 2 and desc.ij[2] == 3 then
				U.dump(desc.df, '?? for_DF:')
			end
]]
	local av,af,auv = {},{},{}
	local mhole
	local apatch = {}
  local arcext = {} -- hole crossing wall boundary
	if desc.skip or desc.u:length() == 0 then
				lo('?? WALL_VOID:')
		forestClean(desc)
		return av,af,an,auv
	end

--            local nwin = math.floor((desc.u:length() - 2*desc.winleft)/desc.winspace)
--            lo('?? NWIN:'..nwin)
	local X,Y = desc.u:length(),desc.v:length()
	local un, vn = desc.u:normalized(), desc.v:normalized()
--    local forestInd = nil
--    local dscale = {}
  local editkey
  if cedit.forest and dforest[cedit.forest] and dforest[cedit.forest].prn.id == desc.id then
    -- contains selected forest item
--      U.dump(dforest[cedit.forest],'??^^^^^^^^^^^^ for_IND:')
    editkey = dforest[cedit.forest].ind
  end

 	local cforest = forestClean(desc)
	if desc.win and not desc.df[desc.win] then
		desc.df[desc.win] = {}
	end

	if true then
		-- clean forests
--            U.dump(desc.df, '??______________ win_dae:')

--        cforest = forestClean(desc)
		-- create grid
--		local ax, ay = {0}, {0} --forHeight(bdesc.afloor, desc.ij[1]-1)}
		local ax = {0} --forHeight(bdesc.afloor, desc.ij[1]-1)}
		local arc = {}

		local wlen = desc.u:length()
		if desc.storefront and desc.storefront.yes then
-------------------------------
-- STOREFRONT
-------------------------------
			local scale = wlen/desc.storefront.len
			local pos = 0
--                lo('??___________________ for_SF:'..scale..':'..#desc.storefront.adae..':'..wlen..'/'..desc.storefront.len)
			for k,dae in pairs(desc.storefront.adae) do
--                if U._PRD == 0 and k ~= 1 then break end
				local a = U.vang(U.vturn(desc.u, -math.pi/2), ddae[dae].front, true)
--                    local a = U.vang(ddae[dae].front, U.vturn(desc.u, -math.pi/2), true)
--                    U.dump(ddae[dae], '?? for_MESH:'..dae..':'..tostring(desc.u)..' front:'..tostring(ddae[dae].front)..' a:'..tostring(a))
				local scalez = desc.v:length()/math.abs(ddae[dae].fr.z-ddae[dae].to.z)
				local itemkey = to(
					desc, dae,
					desc.pos + un*pos + U.vturn(ddae[dae].front, -a)*ddae[dae].inout, -- (ax[#ax] + 0)/X + desc.v*0/Y,
					vec3(0,0,a), cforest, vec3(1, scale, scalez)) -- un*scale)
--                    if k == 1 then break end
				pos = pos + ddae[dae].len*scale
			end
--[[
			local ltot = 0
			for i,pth in pairs(desc.storefront.adae) do
				local m = ddae[pth]
				local L = math.max(
					math.abs(m.to.x-m.fr.x),
					math.abs(m.to.y-m.fr.y),
					math.abs(m.to.z-m.fr.z))
				if ltot
			end
]]
--            lo('?? for_SF:'..desc.u:length())
		elseif desc.win then
			local margin = 0.06

      local function forW(dae)
			  return (ddae[dae].w and (ddae[dae].w + 2*margin) or (ddae[dae].to.x - ddae[dae].fr.x))
          *(desc.df[dae] and desc.df[dae].scale or 1)
      end
      local function forH(dae)
			  return (ddae[dae].h and (ddae[dae].h + 2*margin) or (ddae[dae].to.z - ddae[dae].fr.z))
          *(desc.df[dae] and desc.df[dae].scale or 1)
      end
		--- win dimensions
--                U.dump(desc.df[desc.win], '?? for_w_size:'..desc.win)
--            lo('?? for_win:'..tostring(desc.win)..':'..tostring(ddae[desc.win]))
--                lo('?? for_WIN:'..tostring(desc.win)..':'..tostring(ddae[desc.win]))
--          lo('?? for_w:'..ddae[desc.win].w..':'..(ddae[desc.win].to.x - ddae[desc.win].fr.x))

--			local winH = (ddae[desc.win].to.z - ddae[desc.win].fr.z)*(desc.df[desc.win] and desc.df[desc.win].scale or 1)
			local winH = forH(desc.win) -- ((ddae[desc.win].h + 2*margin) or (ddae[desc.win].to.z - ddae[desc.win].fr.z))*(desc.df[desc.win] and desc.df[desc.win].scale or 1)
			local winW = forW(desc.win) -- ((ddae[desc.win].w + 2*margin) or (ddae[desc.win].to.x - ddae[desc.win].fr.x))*(desc.df[desc.win] and desc.df[desc.win].scale or 1)
			local scale = desc.df[desc.win] and desc.df[desc.win].scale
			--        local winH = (ddae[desc.win].to.z - ddae[desc.win].fr.z)*(dscale[desc.win] or 1)
			--        local winW = (ddae[desc.win].to.x - ddae[desc.win].fr.x)*(dscale[desc.win] or 1)
			local winspace = desc.winspace
			if desc.df[desc.win].fitx then
					lo('??------ for_SPAN:'..desc.df[desc.win].fitx..':'..tostring(desc.winleft)..':'..tostring(scale), true)
				local newW = desc.u:length() - 2*desc.winleft
				scale = vec3(newW/winW,scale,scale)
				winW = newW
				winspace = winW
			end
      if winspace and winspace < winW then
--          lo('?? wU_respace:',true)
        winspace = winW
--        desc.winspace = winspace
      end

			local doorH, doorW
			--        if desc
			--                winH = 1.2
			--        local winW = 0.8w
			local hasdoor = false
			if desc.doorind ~= nil then
				-- compute door rc pos
				hasdoor = true
				doorH = ddae[desc.door].to.z - ddae[desc.door].fr.z
				doorW = ddae[desc.door].to.x - ddae[desc.door].fr.x
--			            U.dump(ddae[desc.door], '??^^^^^^^^^^^^^^ wU_for_door:'..desc.doorind..':'..desc.door..':'..doorH..':'..doorW)
			end

			local H = forHeight(bdesc.afloor, #bdesc.afloor)
			--        local h = forHeight(bdesc.afloor, desc.ij[1]-1)
			--            h = 0
			--        local uext,vext = {desc.uv[1], desc.uv[3]},{H-desc.uv[4], H-desc.uv[2]}
			local uext,vext = {desc.uv[1], desc.uv[3]},{desc.uv[4], desc.uv[2]}
			--            lo('?? for_wall:'..desc.ij[1]..':'..desc.ij[2]..':'..tostring(cedit.mesh)..' h:'..forHeight(bdesc.afloor, desc.ij[1]-1))
--[[??
			if winspace ~= nil then
				ay[#ay + 1] = ay[#ay] + desc.winbot + margin
				ay[#ay + 1] = ay[#ay] + winH - 2*margin
			end
			ay[#ay + 1] = ay[1] + desc.v:length()
]]

			local function forSkip(L)
			--            lo('?? forSkip:'..L)
				local aind = {}
				for i = 1,L-2,2 do
			--                lo('?? for_i:'..i)
					aind[#aind + 1] = i + L - 1
				end
			--                U.dump(aind, '<< forSkip:'..#aind)
				return aind
			end

			local piW
			if winspace then
				local aval = desc.u:length() - 2*desc.winleft

			--                    lo('?? for_START: aval='..aval..' wspace:'..desc.winspace..' wleft:'..desc.winleft)
				local start = desc.winleft + (aval % winspace)/2
				if desc.pilaster and desc.pilaster.yes then
					piW = math.abs(ddae[desc.pilaster.dae].fr.x-ddae[desc.pilaster.dae].to.x)
--                    while start (winspace - winW)/2
						lo('?? if_REM:'..start..':'..#desc.pilaster.aind..':'..tostring(start - piW*#desc.pilaster.aind/2 + (winspace - winW)/2))
					while start - piW*#desc.pilaster.aind/2 + (winspace - winW)/2 <= 0 and #desc.pilaster.aind > 0 do
--                    while start - piW*#desc.pilaster.aind/2 <= 0 and #desc.pilaster.aind > 0 do
						local irem = #desc.pilaster.aind - 1
						table.remove(desc.pilaster.aind, #desc.pilaster.aind)
							lo('?? to_rem:'..#desc.pilaster.aind..':'..irem)
						W.ui['pilaster_ind'..irem] = 0
					end
					start = start - piW*#desc.pilaster.aind/2
--                        lo('?? pil_W:'..tostring(piW)..':'..start..':'..#desc.pilaster.aind)
				end
--??				local aywin = ay

			--                if U._PRD == 0 then winspace = start + aval end


			--                lo('?? for_w:'..start..':'..aval..':'..(start+aval)..' w:'..winW..' h:'..winH)
			--        desc.awin = {}
			--[[??
				if desc.win ~= nil then
					desc.df[desc.win] = {}
				end
				if hasdoor then
					desc.df[desc.door] = {}
				end
			]]
				desc.agrid = {}
			--        local newForestInd = 1
				-- TODO: handle mesh offset
--            lo('?? for_Z:'..ddae[desc.win].fr.z)
				local winbot = desc.winbot - ddae[desc.win].fr.z*(desc.df[desc.win].scale or 1)
			--            local winbot = desc.winbot - ddae[desc.win].fr.z*(dscale[desc.win] or 1)
			--                lo('?? TO:'..start + aval - desc.winspace)
			--        local skip = {}
			--[[
				local sf_mesh, sf_ifr, sf_ito
				if desc.storefront.yes then
					sf_mesh = ddae[desc.storefront.dae]
					sf_ifr = desc.storefront.ind
						U.dump(sf_mesh, '??************ for_SF:'..sf_ifr)
					local sfL = math.max(
						math.abs(sf_mesh.to.x-sf_mesh.fr.x),
						math.abs(sf_mesh.to.y-sf_mesh.fr.y),
						math.abs(sf_mesh.to.z-sf_mesh.fr.z))
					local nspace = math.floor((sfL-0.001)/winspace) + 1
						lo('??************ for_SFl:'..sfL..':'..winspace..'/'..aval..':'..nspace)
				end
			]]
--          lo('??^^^^^^^^^^^^^^^^^ wU_sh:'..winspace..'/'..winW)
				local wi = 0
				local pilshift = 0
				for p = start,start + aval - winspace,winspace do
					wi = wi + 1
					local winorder = (#ax - 1)/2 + 1
			--            lo('?? for_ax:'..p..':'..#ax)
			--            local toskip = false
					if hasdoor and winorder <= desc.doorind and desc.doorind < winorder + 1 then
						hasdoor = false
			--                            lo('??+++++ place_door:'..winorder)
						ax[#ax + 1] = p
--						if U._PRD == 1 then
--							M.grid2mesh(desc.u, desc.v, ax, ay, uext, vext, av, af, auv, forSkip(#ax))
--						end
			--                        U.dump(auv, '?? for_auv1:'..#auv..':'..#ax..':'..#ay)
			--??                    desc.agrid[#desc.agrid + 1] = {ax[#ax], ay[#ay]}
--						desc.agrid[#desc.agrid + 1] = {ax, ay}
			--                        U.dump(desc.agrid, '??____________ agrid_DOOR_PRE:')

						if desc.doorind % 1 == 0 then
							ax = {p}
							-- door grid
--??							ay = {0, desc.doorbot, desc.doorbot + doorH - margin, desc.v:length()}
--                  lo('?? wU_doorW:'..tostring(doorW))
							ax[#ax + 1] = ax[#ax] + (winspace - doorW)/2 + margin

							-- place door
			--                        lo('?? for_door:'..desc.door..':'..tostring(ddae[desc.door]))
							local doorbot = desc.doorbot - ddae[desc.door].fr.z
--                                    U.dump(ddae[desc.door],'??^^^^^^^^^^^ door_dims:'..desc.doorbot..':'..doorbot..':'..default.doorbot)
							--- add forest item
							local itemkey
							itemkey = to(
								desc, desc.door,
			--                        ddae[desc.door],
								desc.pos + un * (ax[#ax] + doorW/2 - margin) + vn * doorbot,
								vec3(0,0,ang), cforest)
							--- link item
			--                    U.push(desc.df[desc.door], itemkey)
			--                            lo('?? linked:'..desc.door..':'..desc.df[desc.door][1])
			--                    desc.adoor[#desc.adoor + 1] = itemkey
							if desc.doorstairs and desc.doorstairs[desc.doorind] then
--                                    U.dump(ddae[desc.doorstairs[desc.doorind].dae],'?? for_STAIRS:'..desc.doorstairs[desc.doorind].dae, true)
								to(
									desc, desc.doorstairs[desc.doorind].dae,
									desc.pos + un * (ax[#ax] + doorW/2 - margin) + vn * (doorbot-default.doorbot),
									vec3(0,0,ang-math.pi/2), cforest
								)
							end
							ax[#ax + 1] = ax[#ax] + doorW - 2*margin
							arc[#arc+1] = {
								vec3(ax[#ax-1], desc.doorbot,0),
								vec3(ax[#ax], desc.doorbot,0),
								vec3(ax[#ax], desc.doorbot + doorH - margin,0),
								vec3(ax[#ax-1], desc.doorbot + doorH - margin,0),
							}
--                  U.dump(arc, '?? for_ARC:')
							ax[#ax + 1] = p + winspace
							p = ax[#ax] --p + desc.winspace
--??							M.grid2mesh(desc.u, desc.v, ax, ay, uext, vext, av, af, auv, forSkip(#ax))
--??							desc.agrid[#desc.agrid + 1] = {ax, ay}
			--[[
							arc[#arc+1] = {
								vec3(ax[#ax-1]-doorW + margin, desc.doorbot,0),
								vec3(ax[#ax]-doorW/2 + margin, desc.doorbot,0),
								vec3(ax[#ax]-doorW/2 + margin, desc.doorbot + doorH - margin,0),
								vec3(ax[#ax-1]-doorW + margin, desc.doorbot + doorH - margin,0),
							}

			]]
			--??                        desc.agrid[#desc.agrid + 1] = {ax[#ax], ay[#ay]}
			--                                U.dump(desc.agrid, '??____________ agrid_DOOR:')
						else
							p = p + 1
						end

						ax = {p}
--						ay = aywin
						goto continue
					end
			--                    lo('?? AT:'..p..':'..#ax) --..':'..#desc.agrid[1][1])
					-- hole left
					ax[#ax + 1] = p + (winspace - winW)/2 + margin + pilshift

					-- place window
					--- add forest item
			--                    lo('?? is_scale:'..tostring(desc.df[desc.win]))
			--                local scale = desc.df[desc.win] and desc.df[desc.win].scale -- or dscale[desc.win]
          local dae = desc.win
          -- winbot = desc.winbot - fr.z
          local winw, winh, winbotc = winW, winH, winbot
          local ds,dz = 0,0
--              U.dump(desc['win'..'_inf'],'?? if_INF:'..desc.ij[2])
          if true and desc['win'..'_inf'] then -- and desc['win'..'_inf'].ddae[wi] then
            if desc['win'..'_inf'].ddae and desc['win'..'_inf'].ddae[wi] then
              dae = desc['win'..'_inf'].ddae[wi] or dae
              winh = forH(dae)
              winw = forW(dae)
--              winh = ((ddae[desc.win].h + 2*margin) or (ddae[desc.win].to.z - ddae[desc.win].fr.z))*(desc.df[desc.win] and desc.df[desc.win].scale or 1)
--              winw = ((ddae[desc.win].w + 2*margin) or (ddae[desc.win].to.x - ddae[desc.win].fr.x))*(desc.df[desc.win] and desc.df[desc.win].scale or 1)

--              winh = (ddae[dae].h + 2*margin) or ((ddae[dae].to.z - ddae[dae].fr.z)*(desc.df[dae] and desc.df[dae].scale or 1))
--              winw = ddae[dae].w or ((ddae[dae].to.x - ddae[dae].fr.x)*(desc.df[dae] and desc.df[dae].scale or 1))
              ds = (winW-winw)/2
            end
--                U.dump(desc['win'..'_inf'], '??^^^^^^^^^^^^^^ for_INF:')
--                U.dump(dforest[cedit.forest], '?? in_dfor:'..desc.id)
--                U.dump(desc.df,'?? in_DAE:'..tostring(cedit.forest)..':'..dae)
--            key = dforest[dae]
            if not desc['win'..'_inf'].dwinbot then
--              desc['win'..'_inf'].dwinbot = {}
            end
--              desc['win'..'_inf'].dwinbot[wi] = 0
--            end
--                  U.dump(desc['win'..'_inf'], '?? ifinf:'..tostring(desc['win'..'_inf'].dwinbot)..':'..tostring(ddae[dae]),true)
--            if cedit.fscope == 1 and editkey and wi == editkey then
            winbotc = desc.winbot - ddae[dae].fr.z*(desc.df[dae] and desc.df[dae].scale or 1)
            if desc['win'..'_inf'].dwinbot and desc['win'..'_inf'].dwinbot[wi] then
              dz = desc['win'..'_inf'].dwinbot[wi] - desc.winbot
--              winbotc = winbotc - desc.winbot + desc['win'..'_inf'].dwinbot[wi]
            end
--[[
            if desc['win'..'_inf'].dwinbot and desc['win'..'_inf'].dwinbot[wi] then
--                  lo('?? if:'..tostring(desc['win'..'_inf'].dwinbot[wi])..':'..tostring(desc.df[dae])..':'..tostring(ddae[dae]),true)
              local scale = desc.df[dae] and desc.df[dae].scale or 1
              dz = ddae[dae].fr.z*scale
              winbotc = (desc['win'..'_inf'].dwinbot[wi] or winbot) - dz
            end
]]
--            winbot = desc.winbot - ddae[dae].fr.z*(desc.df[dae].scale or 1)
          end
--            lo('?? for_BOT:'..wi..' winbotc:'..winbotc..'/'..winbot..' dz:'..tostring(dz)..' fr.z:'..tostring(ddae[dae].fr.z)..':'..tostring(scale),true)
--            if desc.ij[1] and desc.ij[2] == 1 then
--              lo('?? wU.win:'..wi..':'..tostring(cedit.fscope))
--            end

--              if cedit.fscope == 1 and editkey then
--                lo('?? sing_Z:'..desc.ij[2]..':'..winbotc..'/'..winbot) --..tostring(desc.pos + desc.u*(ax[#ax] + winw/2 - margin + ds)/X + desc.v*winbotc/Y))
--              end
 					local itemkey = to(
						desc, dae,-- desc.win,
						desc.pos + desc.u*(ax[#ax] + winw/2 - margin + ds)/X + desc.v*(winbotc+dz)/Y,
						vec3(0,0,ang), cforest, scale, wi) -- dscale[desc.win])
          if cedit.fscope == 1 and editkey and wi == editkey then -- not out.editkey then
            if desc['win'..'_inf'] then
--            if desc['win'..'_inf'] and desc['win'..'_inf'].ddae[wi] then
              -- save forest key to global for restoring
              out.editkey = itemkey
--                U.dump(desc.df, '??^^^^^^^^^ wU_newkey:'..tostring(itemkey)..':'..tostring(dforest[itemkey]))
            end
          end
--              lo('?? for_key:'..desc.ij[1]..':'..desc.ij[2]..':'..tostring(cedit.forest)..':'..tostring(cedit.fscope)..':'..tostring(itemkey)..':'..wi)
					-- hole right
--              lo('?? winW:'..winw..':'..dae)
					ax[#ax + 1] = ax[#ax] + winw - 2*margin

          -- place pilaster
					if desc.pilaster and desc.pilaster.yes and #U.index(desc.pilaster.aind, wi) > 0 then
						local pilscaleZ = desc.v:length()/(math.abs(ddae[desc.pilaster.dae].fr.z-ddae[desc.pilaster.dae].to.z)+0.001)
--                            lo('?? pill_PUT:'..pilscaleZ)
						to(
							desc, desc.pilaster.dae,
							desc.pos + desc.u*(ax[#ax] + (winW/2+winspace)/2 + 0*ds)/X, -- + desc.v*winbot/Y,
--                            desc.pos + desc.u*(ax[#ax] + winW/2 - margin)/X, -- + desc.v*winbot/Y,
							vec3(0,0,ang), cforest, vec3(1,1,pilscaleZ))
						pilshift = pilshift + piW
					end

			--                ay[#ay + 1] = ay[#ay] + desc.winbot + margin
			--                ay[#ay + 1] = ay[#ay] + winH - 2*margin
					-- holes boundaries
--              lo('?? for_winh:'..desc.ij[2]..':'..wi..':'..winh)

          local wb = desc.winbot + dz -- winbotc -- dz and (winbotc+0*dz) or desc.winbot
--              if cedit.fscope == 1 and editkey then
--                lo('?? for_Z_SING:'..desc.ij[2]..':'..wi..':'..wb..':'..tostring(dz))
--              end
          local dw = 0
          if winw > winspace then
            dw = (winw - winspace)/2
--            lo('??^^^^ respace_SINGLE:'..winw..'/'..winspace..':'..dw)
          end
--              ds = 0
--              dw = 0
--              lo('?? for_arc_wb:'..tostring(wb))
					arc[#arc+1] = {
						vec3(ax[#ax-1]+ds+dw, wb + margin,0),
						vec3(ax[#ax]+ds-dw,wb + margin,0),
						vec3(ax[#ax]+ds-dw,wb + margin + winh - 2*margin,0),
						vec3(ax[#ax-1]+ds+dw,wb + margin + winh - 2*margin,0),
					}
			--[[
						{x=ax[#ax-1], y=desc.winbot + margin},
						{x=ax[#ax], y=desc.winbot + margin},
						{x=ax[#ax], y=desc.winbot + margin + winH - 2*margin},
						{x=ax[#ax-1], y=desc.winbot + margin + winH - 2*margin},
			]]
          if desc.winbot + winH > Y then
            arcext[#arcext+1] = arc[#arc]
          end
					::continue::
				end
			end
      if desc.arcext then
--      if U._PRD==0 and desc.arcext then
--          U.dump(desc.arcext, '??^^^^^^^^^^^^^^^ wU.for_WEXT:'..#desc.arcext..':'..desc.ij[2]..':'..#arc)
        for _,rce in pairs(desc.arcext) do
          arc[#arc+1] = rce
        end
      end
--      				U.dump(ax, '?? AX:')

--[[
			if desc.ij[2]==4 then
				U.dump(ax, '?? AX:')
				U.dump(arc, '?? ARC0:'..desc.ij[2])
			end
]]
			--            U.dump(desc.achild, '??_________________________________________ if_ACHILD:')
			------------------------
			-- CHILDREN
			------------------------
			if desc.achild then
				for _,b in ipairs(desc.achild) do
					if b.yes then
						arc[#arc+1] = b.base
						apatch[#apatch+1] = #arc
						if b.body then
							doorH = ddae[b.body].to.z - ddae[b.body].fr.z
							doorW = ddae[b.body].to.x - ddae[b.body].fr.x
			--                            U.dump(b.base, '??______ for_hole_mesh:'.._..':'..b.body..':'..ang..':'..tostring(desc.pos))

							local w,h = U.rcWH(b.base)
							local scale = vec3((w+0*margin)/doorW, 1, (h+0*margin)/doorH)
							scale.x = (w+2*margin*scale.x)/doorW
							scale.z = (h+2*margin*scale.z)/doorH
			--                                lo('?? WH:'..tostring(w)..':'..tostring(h)..':'..tostring(scale), true)
			--                                scale = vec3(2.5,1,0.5,1)
			--                                U.dump(cforest, '?? cFOR:'..tostring(scale))
							if false and b.body ~= desc.door then
								lo('?? MM:', true)
							else
								to(
									desc, b.body,
									desc.pos + un * (math.min(b.base[1].x,b.base[2].x) + w/2)
										+ vn*math.min(b.base[1].y,b.base[2].y),
			--                            desc.pos + un * (math.min(b.base[1].x,b.base[2].x) + doorW/2 - margin) + vn*math.min(b.base[1].y,b.base[2].y),
									vec3(0,0,ang), cforest, scale)
							end
			--                                lo('?? wu_hole:'..tostring(cedit.forest)..':'..b.body..':'..tostring(desc.door), true)
			--[[
							local doorbot = desc.doorbot - ddae[desc.door].fr.z
							to(
								desc, desc.door,
			--                        ddae[desc.door],
								desc.pos + un * (ax[#ax] + doorW/2 - margin) + vn * doorbot,
								vec3(0,0,ang), cforest)

			]]
						end
			--                    apatch[#arc] = true
					end
				end
			end
			-- PILLARS
			if desc.pillar and desc.pillar.yes and ddae[desc.pillar.dae] then
--					U.dump(ddae[desc.pillar.dae], '?? for_DAE:')
				-- place pillars
				local dpos = vec3(
					(ddae[desc.pillar.dae].to.x - ddae[desc.pillar.dae].fr.x)/2,
					(ddae[desc.pillar.dae].to.y - ddae[desc.pillar.dae].fr.y)/2,
					0
				)
--                    desc.pillar.margin = 0
--                local marginLeft = ((desc.u:length() - 2*desc.pillar.margin) % desc.pillar.space)/2
--                    + desc.pillar.margin + dpos.x
				local marginLeft = desc.pillar.margin + dpos.x
					+ ((desc.u:length() - 2*(desc.pillar.margin + dpos.x)) % desc.pillar.space)/2
					lo('??^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ mLeft:'..marginLeft..' marg:'..tostring(desc.pillar.margin)..' space:'..desc.pillar.space..' u:'..desc.u:length()..' dx:'..tostring(dpos)..':'..tostring(desc.pos)..' span:'..tostring(desc.pillar.span))
				local npillar = math.floor((desc.u:length() - 2*marginLeft)/desc.pillar.space)+1
			--                U.dump(desc.ij,'?? for_pillar:'..tostring(dpos)..':'..tostring(marginLeft)..':'..tostring(desc.pillar.span)..':'..tostring(desc.pillar.space)..':'..npillar)
			--            local npillar = math.floor((desc.u:length() - 2*marginLeft)/desc.pillar.space + 0.5) + 1
				local step = desc.pillar.space
				if desc.pillar.span then
--                    marginLeft = dpos.x
--                    npillar = math.floor((desc.u:length() - 2*marginLeft)/desc.pillar.space)+1
					marginLeft = -(desc.pillar.inout or 0)
						marginLeft = dpos.x
			--                if marginLeft - dpos.x < 0 then
			--                    marginLeft = dpos.x
			--                end
					local spaceAval = desc.u:length() - 2*marginLeft
					npillar = math.floor(spaceAval/desc.pillar.space)
					step = spaceAval/npillar
					npillar = npillar + 1
--[[
]]
				end
				local vn = vec3(-desc.u.y,desc.u.x):normalized()
			--                lo('?? for_pillar:'..desc.pillar.dae..':'..tostring(ddae[desc.pillar.dae].type), true)
			--                U.dump(ddae[desc.pillar.dae].fr, '??________ for_PILLARS:'..marginLeft..' N:'..npillar..':'..desc.pillar.down)
			--            local h = forHeight(adesc[cedit.mesh].afloor, desc.pillar.floor)
				local scale = desc.pillar.down/(ddae[desc.pillar.dae].to.z - ddae[desc.pillar.dae].fr.z)
--                marginLeft = (desc.u:length() - (npillar-1)*desc.pillar.space)/2
				for i = 1,npillar do
			--                    lo('?? in_pillar:'..i..':'..tostring(desc.pillar.down),true)
						lo('?? for_pos:'..i..':'..(marginLeft + (i-1)*step))
					to(
						desc, desc.pillar.dae,
--                        desc.pos + (marginLeft + i*step)*desc.u:normalized()
						desc.pos + (marginLeft + (i-1)*step)*desc.u:normalized()
						- vec3(0,0,desc.pillar.down) - vn*(desc.pillar.inout or 0), -- dpos, -- + vec3(0,2,0),--  + un * (ax[#ax] + doorW/2 - margin) + vn * doorbot,
						vec3(0,0,ang), cforest, vec3(1,1,scale))
				end
			end
			-- BALCONIES
			if desc.balcony and desc.balcony.yes then
			--                U.dump(desc.balcony.ind, '?? for_BALC:'..tostring(desc.ij[2])..':'..#desc.df[desc.win])
			--            local ang = math.atan2()
				local u = desc.u
				local ang = math.atan2(u.y, u.x) + math.pi/2
				local scale = desc.df[desc.balcony.dae] and desc.df[desc.balcony.dae].scale or 1 -- dscale[desc.balcony.dae]
				local ind = 0
				local per = desc.balcony.ind[2]
				if desc.balcony.ind[3] then
					per = per + desc.balcony.ind[3]
				end
			--                lo('?? b_PER:'..per..':'..tostring(scale), true)
				for k,key in ipairs(desc.df[desc.win]) do
					local modper = (k - desc.balcony.ind[1])%per
					if k >= desc.balcony.ind[1] and (modper == 0 or modper == desc.balcony.ind[2]) then
						local wpos = dforest[key].item:getPosition()
			--                lo('?? for_wpos:'..tostring())
						to(desc, desc.balcony.dae, wpos + vec3(0,0,desc.balcony.bottom or 0), vec3(0,0,-ang),
						nil, scale)
			--                        lo('??_________ for_BALC:'..k..':'..tostring(cedit.forest), true)
					end
			--                lo('?? for_pos'..tostring(dforest[key].item:getPosition())
			--                U.dump(dforest[key], '?? for_win:'..key)
				end
			end
			-- STRINGCOURSE
			if desc.stringcourse and desc.stringcourse.yes then
				--M.align(ddae[desc.stringcourse.dae], desc.pos, desc.pos+desc.u)
				local m = ddae[desc.stringcourse.dae]
				local nstep = round(desc.u:length()/m.len)
				local scale = desc.u:length()/(m.len*nstep)
--                    U.dump(desc.stringcourse,'??************* wallUp.stringcourse:'..nstep..':'..desc.u:length()..'/'..m.len..':'..tostring(desc.pos)..' scale:'..scale..':'..tostring(desc.stringcourse.dae))
				local a = U.vang(U.vturn(desc.u, -math.pi/2), m.front, true)
				local s = 0
				local inout = U.vturn(m.front, -a)
--                    lo('?? turned_DIR:'..tostring(inout)..'/'..tostring(m.front))
				for i=1,nstep do
--                        lo('?? for_s:'..i..':'..s..':'..tostring(desc.u))
					to(desc, desc.stringcourse.dae,
						desc.pos+desc.u:normalized()*s - inout*math.abs(m.fr.x-m.to.x)/2 + desc.v:normalized()*(desc.v:length()-math.abs(m.fr.z-m.to.z)+(desc.stringcourse.bot or 0)-0.001),
						vec3(0,0,a), cforest, vec3(1,scale,1))
					s = s + m.len*scale
				end
			end
		end
		ax[#ax + 1] = desc.u:length()
				if dbg then
					U.dump(ax, '?? wU_AX:')
--					U.dump(ay, '?? wU_AY:')
				end
--            if desc.pilaster and desc.pilaster.yes then
--                U.dump(ax, '?? for_AX:')
--            end
		desc.nwin = #ax/2 - 1
--            lo('?? n_win:'..desc.ij[2]..':'..#ax)
--[[
			if false then
--            if desc.ij[1] == 1 and desc.ij[2] == 3 then
					ax = {ax[1],ax[#ax]}
					ay = {ay[1],ay[#ay]}
--                ax = {ax[1],ax[2],ax[3],ax[4]}
--                ax = {ax[1],ax[2],ax[3],ax[4]}
			end
]]
		if not (desc.storefront and desc.storefront.yes) then -- or U._PRD == 0 then
--                lo('??________________________ PAVE:')
--                U.dump(desc.uv, '??______________ for_POS:'..tostring(desc.ij[2]))
--        if U._PRD == 0 and (desc.ij[1] == 1 and desc.ij[2] == 3) then
--                U.dump(av, '?? for_AV:'..#av)
--                if desc.pilaster and desc.pilaster.yes then
--                    U.dump(arc, '??************************* FOR_arc:'..tostring(desc.u)..':'..tostring(desc.v))
--                end
			-- holes array
			desc.arc = arc
			local scale = desc.uvscale or {1,1}
--                    U.dump(desc.uvref, '?? for_UVREF:'..desc.ij[1]..':'..desc.ij[2]..':'..tostring(desc.pos.z))
			local uvref = desc.uvref or {0,(desc.pos.z or 0)*scale[2]}
			local mbody,mh,ae,apath,albl
--[[
			if #arc == 0 then
				av = {
					vec3(0,0,0),
					vec3(desc.u:length(),0,0),
					vec3(desc.u:length(),desc.v:length(),0),
					vec3(0,desc.v:length(),0),
				}
				if true then
					mbody = M.rcPave(av, arc, {uvref,scale}, apatch, true)
				else
					an,auv,af = M.zip2(av, {3,2,1,4}, nil, auv, af)
	--                    U.dump(an, '?? for_AN:')
					an = {desc.u:cross(desc.v):normalized()}
	--                auv = M.forUV(av, 1, {{u=0,v=0}, {u=1,v=0}})
--                        U.dump(av, '?? for_AV:')
--                        U.dump(auv, '?? for_AUV:')
--                        U.dump(af, '?? for_AF:')
				end
--                mbody = {verts = av, faces = af, uvs = auv, normals = an}

--                    U.dump(auv,'!!^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ NO_ARC:'..#af..':'..#mbody.faces)
			else
				mbody,mh,ae,apath,albl = M.rcPave({
					vec3(0,0,0),
					vec3(desc.u:length(),0,0),
					vec3(desc.u:length(),desc.v:length(),0),
					vec3(0,desc.v:length(),0),
				}, arc, {uvref,scale}, apatch) -- {arc[1],arc[2],arc[3],arc[4],arc[5],arc[6]}) -- arc)
			end
				if desc.ij[2]==4 then
					U.dump({
						vec3(0,0,0),
						vec3(desc.u:length(),0,0),
						vec3(desc.u:length(),desc.v:length(),0),
						vec3(0,desc.v:length(),0),
					}, '?? BASE:')
					U.dump(arc, '??__________ wU_topave:')
				end

          if desc.ij[1] == 2 and desc.ij[2] == 3 then
            U.dump(arc, '?? wU.PAVE:'..desc.ij[1]..':'..desc.ij[2])
            U.dump({
              vec3(0,0,0),
              vec3(desc.u:length(),0,0),
              vec3(desc.u:length(),desc.v:length(),0),
              vec3(0,desc.v:length(),0),
            })
          end
]]
--        U.dump(apatch, '??_____ if_PATCH:')
--        if desc.ij[2] == 3 then
--          U.dump(arc, '?? pre_ARC:'..#arc)
--        end
--          if true or (desc.ij[2] == 3 and desc.ij[1] == 1) then
			mbody,mh,ae,apath,albl = M.rcPave({
				vec3(0,0,0),
				vec3(desc.u:length(),0,0),
				vec3(desc.u:length(),desc.v:length(),0),
				vec3(0,desc.v:length(),0),
			}, arc, {uvref,scale}) --, desc.ij[2]==2 and true or false) -- {arc[1],arc[2],arc[3],arc[4],arc[5],arc[6]}) -- arc)
--			}, arc, {uvref,scale}, apatch, desc.ij[2]==1 and true or false) -- {arc[1],arc[2],arc[3],arc[4],arc[5],arc[6]}) -- arc)
--          if desc.ij[2] == 3 then
--            U.dump(mbody, '?? wU_mdata:')
--          end
--            }, arc, {{0,desc.pos.z}, {1,1}}, apatch) -- {arc[1],arc[2],arc[3],arc[4],arc[5],arc[6]}) -- arc)
--                    U.dump(arc, '?? for_WALL:'..#mbody.verts..':'..#mbody.uvs)
--                if #apatch > 0 then
--                    U.dump(mh, '??++++++++++++++++++++++++++ for_PATCH:'..#apatch..':'..#mh.faces)
--                end
--            }, arc, {{0,0}, {1,1}}) -- {arc[1],arc[2],arc[3],arc[4],arc[5],arc[6]}) -- arc)
			local un = desc.u:normalized()
			local vn = desc.v:normalized()
			for k,p in pairs(mbody.verts) do
				mbody.verts[k] = un*p.x + vn*p.y
			end
			mhole = mh
--                    U.dump(mbody, '?? for_AV2:')
--                    U.dump(mbody.verts, '?? for_AV2:')
--                    U.dump(apath, '?? APATH:')
--                    U.dump(ae, '?? AE:')
			av = mbody.verts
			auv = mbody.uvs
			af = mbody.faces
--                if dbg then
--                    U.dump(mbody, '?? if_FACES:'..#af)
--                end
--                af = {}
--            an = {vec3(0,0,1)}
--                    U.dump(an, '??_______________________ AN:')
--                    U.dump(an, '?? for_NORM: u='..tostring(desc.u)..' v='..tostring(desc.v)..':'..#an)

--                    U.dump(af, '?? for_AF2:')
--[[
				if U._PRD==2 and desc.ij[1] == 2 and desc.ij[2] == 3 then
--                        U.dump(mh.faces, '?? _____________faces:')
					local avert = {}
					local zn = (desc.u:cross(desc.v)):normalized()
					for i=1,#mh.faces/6,6 do
						if desc.achild[i].z and desc.achild[i].z ~=0 then
							local apick = {}
							for j = i,i+5 do
								local f = mh.faces[j]
	--                        for _,f in pairs(mh.faces) do
								if #U.index(apick, f.v) == 0 then
									apick[#apick+1] = f.v -- m.verts[f.v+1]
									avert[#avert+1] = mh.verts[f.v+1]
									av[#av+1] = mh.verts[f.v+1] + zn*desc.achild[i].z
		--                            out.avedit[#out.avedit+1] = mh.verts[f.v+1]
								end
							end
						end
					end
--                        U.dump(avert, '?? _____________avert:'..#avert)

					out.fwhite = ae
					for _,e in pairs(out.fwhite) do
						e[1] = e[1] + vec3(-265,1148,1)
						e[2] = e[2] + vec3(-265,1148,1)
					end
					out.flbl = albl
					for _,e in pairs(out.flbl) do
						e[1] = e[1] + vec3(-265,1148,1)
					end
--                    U.dump(mhole, '??++++++++++++++++++++++++++ for_PATCH:'..#apatch..':'..#mh.faces)
				end

			if desc.achild then
				for _,b in ipairs(desc.achild) do
					U.dump(b, '??____________________________ for_hole:')
					for j,n in pairs(b.base) do
						b.base[j] = b.base[j] + vec3(0,0,b.z)
					end
				end
			end
]]
--[[

				out.fwhite = ae
				for _,e in pairs(out.fwhite) do
					e[1] = e[1] + vec3(-265,1148,1)
					e[2] = e[2] + vec3(-265,1148,1)
				end

				av = mbody.verts
				auv = mbody.uvs
				af = mbody.faces
					out.fyell = apath
					for _,e in pairs(out.fyell) do
						for i,n in pairs(e) do
							e[i] = e[i] + vec3(-265,1148,1)
						end
					end
					out.flbl = albl
					for _,e in pairs(out.flbl) do
						e[1] = e[1] + vec3(-265,1148,1)
					end
]]
--                out.m1,out.m2,out.fwhite,out.fyell,out.flbl
--                        out.albl
--[[
			U.rcPave({
				{x=0,y=0},
				{x=desc.u:length(),y=0},
				{x=desc.u:length(),y=desc.v:length()},
				{x=0,y=desc.v:length()},
			}, arc)
]]
		else
			lo('?? place_SF:')
--[[
			if false then
--            if desc.ij[1] == 1 and desc.ij[2] == 3 then
				U.dump(ax, '?? AX:')
				U.dump(ay, '?? AY:')
			end
			M.grid2mesh(desc.u, desc.v, ax, ay, uext, vext, av, af, auv, forSkip(#ax))
]]
		end
--            if dbg then
--            if desc.ij[1] == 1 and desc.ij[2] == 3 then
--                U.dump(av, '?? for_AV_POST:')
--                U.dump(af, '?? for_AF_POST:')
--            end
		-- info for UV moves
--[[??
		if desc.agrid then
			desc.agrid[#desc.agrid + 1] = {ax, ay}
--            desc.agrid[#desc.agrid + 1] = {ax[#ax], ay[#ay]}
		else
			lo('!! ERR_no_agrid:')
		end
]]
	else
		av, af = M.rect(desc.u, desc.v)
	end
--            U.dump(desc.agrid, '??++++++++++++++++++++++ wall_AGRID:'..desc.ij[1]..':'..desc.ij[2])
	-------------------------------------
	-- HANDLE EXTRA FACES at the ROOF
	-------------------------------------
  if false then
    for _,plus in pairs(desc.avplus) do
  --    if desc.avplus ~= nil then
      lo('??____________________________________ desc_plus:'..tostring(desc.plus))
  --        U.dump(desc.avplus, '??*+ for_ATOP:'..#desc.avplus..':'..tostring(desc.pos)) --..'/'..tostring(adesc[cedit.mesh].pos))
  --        U.dump(av, '?? for_AV:')

      if #plus == 3 then
        af[#af + 1] = {v = #av+0, n = 0, u = #av+0}
        af[#af + 1] = {v = #av+2, n = 0, u = #av+2}
        af[#af + 1] = {v = #av+1, n = 0, u = #av+1}
        for _,p in pairs(plus) do
          av[#av + 1] = p -- desc.pos
          auv[#auv + 1] = {u = p:dot(un), v = p:dot(vn)}
  --               auv[#auv + 1] = {u = (rc[1]-ps):dot(un), v = (rc[1]-ps):dot(vn)}
        end
  --            auv = M.forUV(plus, 1, nil, nil, desc.uvscale or {1,1}, auv)
  -- (base, istart, uvini, w, scale)
  --        (av, istart,
  --        {{u=mrc[1][1],v=-mrc[1][2]},{u=mrc[1][1]+(rc[istart+1]-rc[istart]):length(),v=-mrc[1][2]}}
  --        ,(rc[2]-rc[1]):cross(rc[4]-rc[1]), mrc[2])
      end
      if #plus == 4 then
        af[#af + 1] = {v = #av+0, n = 0, u = #av+0}
        af[#af + 1] = {v = #av+3, n = 0, u = #av+3}
        af[#af + 1] = {v = #av+1, n = 0, u = #av+1}

        af[#af + 1] = {v = #av+1, n = 0, u = #av+1}
        af[#af + 1] = {v = #av+3, n = 0, u = #av+3}
        af[#af + 1] = {v = #av+2, n = 0, u = #av+2}

        for _,p in pairs(plus) do
          av[#av + 1] = p -- desc.pos
          auv[#auv + 1] = {u = p:dot(un), v = p:dot(vn)}
        end
      end
    end
  end
	-- set position
	for o,v in pairs(av) do
		av[o] = av[o] + desc.pos -- desc.pos is the bottom-left corner
	end
--        lo('<< wallUp:'..#desc.agrid..' av:'..#av, true)
	return av,af,an,auv,mhole,arcext
end


local function onSide_(p, base)
	for i = 1,#base do
		if math.abs((base[i % #base+1] - p):length() + (p-base[i]):length() - (base[i % #base+1] - base[i]):length()) < 0.01 then
			if (p-base[i]):length() == 0 then
				return i, true
			elseif (base[i % #base+1] - p):length() == 0 then
				-- next edge start, continue
--                return 0
			else
				return i, false
			end
		end
	end
	return 0
end


local function polySplit(base, dbg)
--    local base = out.border[ind]
--            U.dump(base, '>> polySplit:'..#base)
--[[
			if dbg then
				out.avedit = {}
				for _,v in pairs(base) do
					out.avedit[#out.avedit + 1] = v + vec3(0,0,1)
				end
			end
]]

	for i = 1,#base do -- (i - 1) % #base + 1 = i
		local dir = (base[(i + 1)%#base + 1] - base[i%#base + 1]):cross(base[i%#base + 1] - base[i]).z
			if dbg then
				lo('?? pS:'..i..':'..dir)
			end
--                lo('?? vec:'..i..':'..tostring(dir)) -- tostring((base[(i + 1)%#base + 1] - base[i%#base + 1]))..':'..tostring(base[i%#base + 1] - base[i]))
		if dir >= 0 then
					if dbg then
						lo('?? polySplit.for_conc:'..i..':'..dir)
					end
			local v1,v2 = base[i%#base + 1], base[(i + 1)%#base + 1]
			if dir == 0 then
				v2 = v1 + (v1 - base[i]):cross(vec3(0,0,1))
			end
--            out.avedit[#out.avedit + 1] = base[i%#base + 1]
			-- cut
			--- cross with "back" line
			for j = i+3,i + #base - 1 do
--                    lo('?? for_j:'..j)
				local a, b = base[(j-1)%#base + 1], base[(j)%#base + 1]
				local x, y = U.lineCross(v1, v2, a, b)
				local v = vec3(x, y, 0)
						if dbg then lo('?? a_b:'..j..':'..tostring(a)..':'..tostring(b)..':'..tostring(x)..':'..tostring(y)) end
				if (a-b):length() + 0.01 > (a-v):length() + (b-v):length() then
					j = (j-1) % #base + 1
						if dbg then
							lo('?? ps.cross:'..i..'>'..j..' x:'..x..' y:'..y..' v1:'..tostring(v1)..' v2:'..tostring(v2))
						end
					-- cut and break
					local b1,b2 = {},{}
					if j > i then
						--- left poly
						b1[#b1 + 1] = v
						for k = j + 1,#base do
							b1[#b1 + 1] = base[k]
						end
						for k = 1,i+1 do
							b1[#b1 + 1] = base[k]
						end
						--- right poly
						b2[#b2 + 1] = v
						for k = i + 2,j do
							b2[#b2 + 1] = base[k]
						end
					else
						b1[#b1 + 1] = v
						for k = j+1,i+1 do
							b1[#b1 + 1] = base[k]
						end

						b2[#b2 + 1] = v
						for k = i + 2,#base do
							b2[#b2 + 1] = base[k]
						end
						for k = 1,j do
							b2[#b2 + 1] = base[k]
						end
					end
					if dbg then
						U.dump(b1, '?? PS1:')
						U.dump(b2, '?? PS2:')
						out.avedit = {}
						for _,v in pairs(b1) do
							out.avedit[#out.avedit + 1] = v + vec3(0,0,1)
						end
--                        out.apoint = {}
--                        for _,v in pairs(b2) do
--                            out.apoint[#out.apoint + 1] = v + vec3(0,0,0.7)
--                        end
					end
					return b1, b2
				end
			end
		end
	end
	::out::
--    lo('<< polySplit:')
	return nil
end

--polySplit({vec3(-6.884428634,-7.999998467,0), vec3(-5.391410351,-7.999998093,0), vec3(-5.391410351,-7.999998093,0), vec3(0,-8,0), vec3(0,0,0), vec3(-6.884426951,-4.768371582e-07,0)}, true)
--polySplit({vec3(0,0,0), vec3(-6.884426951,-4.768371582e-07,0), vec3(-6.884426198,3.579999523,0), vec3(-12.99999925,3.58,0), vec3(-13,-8,0), vec3(-5.391410351,-7.999998093,0), vec3(-5.391410351,-7.999998093,0), vec3(0,-8,0)}, true)

--polySplit({vec3(-6.884428634,-8,0), vec3(-2.796333313,-8,0), vec3(-2.796333313,-8,0), vec3(0,-8,0), vec3(0,0,0), vec3(-6.884426951,-4.768371582e-07,0)}, true)


local function coverUp(cbase, dbg)
	if not cbase then return end
--            dbg = true
	local lift = 0
--    lift = lift or 0
	local base = {}
	for i,b in pairs(cbase) do
		base[#base+1] = vec3(b.x,b.y)
	end
--    base = U.clone(base)
	for _,p in pairs(base) do
		p.z = lift
	end
--            if dbg then lo('?? coverUp_base:'..#base) end
	local map = {} -- map cleaned up vertices to initial
	for j = #base,1,-1 do
--            if W.ui.dbg ~= nil then
--                lo('?? fCU:'..j..':'..tostring(U.mod(j+1,base))..':'..tostring(U.mod(j-1,base))..':'..tostring(base[j]))
--            end
		if U.vang(U.mod(j+1,base) - base[j],base[j] - U.mod(j-1,base)) < small_ang then
			table.remove(base,j)
		else
			table.insert(map, 1, j)
		end
	end
--        if dbg then U.dump(map, '?? coverUp.map:'..#base) end
	local arc = {}
	local abase = {base}

	local n = 0
	while #abase > 0 and n < 100 do
				if dbg then
					U.dump(abase[#abase], '?? presSPL:'..n..':'..#abase[#abase])
				end
		local sbase = U.clone(abase[#abase])
		local b1, b2 = polySplit(abase[#abase]) --, dbg)
                if dbg then
                    U.dump(b1, '?? split1:')
                    U.dump(b2, '?? split2:')
                end
		if b1 ~= nil and #b1 > 0 then
			abase[#abase] = U.polyStraighten(b1)
			abase[#abase+1] = U.polyStraighten(b2)
--                lo('?? to_abase:'..#abase)
		else
			-- drop rectangle
			local isvalid = true
			for k = 1,#sbase-1 do
				if (sbase[k]-sbase[k+1]):length() < 0.000001 then
					isvalid = false
					break
				end
			end
				if dbg then U.dump(sbase, '?? to_RC:'..#sbase..':'..(sbase[1]-sbase[2]):length()..':'..tostring(isvalid)) end
--                isvalid = true
			if isvalid then
				arc[#arc+1] = sbase -- abase[#abase]
			end
--[[
]]
--            arc[#arc+1] = sbase -- abase[#abase]
			table.remove(abase, #abase)
		end
		n = n + 1
	end
	if not indrag and dbg then U.dump(arc, '<< coverUp: rects='..n..':'..#arc) end
--    if not indrag then U.dump(arc, '<< coverUp: rects='..#arc) end
--    out.border = arc
	return arc,map
end


W.atParent = function(desc)
	if not desc.prn then return vec3(0,0,0) end
	return adesc[desc.prn].pos + vec3(0,0,forHeight(adesc[desc.prn].afloor,desc.floor-1))
end


local function markUp(forestMeshName, forestType)
	if out.inroad and out.inroad > 0 then return end
	if W.ui.injunction then return end
		lo('>> markUp:'..tostring(forestMeshName)..' fscope:'..tostring(cedit.fscope)..':'..tostring(cedit.forest)..':'..tostring(forestType)..':'..tostring(scope))
		if not indrag then
			U.dump(cij, '?? markUp:'..tostring(forestMeshName)..':'..tostring(forestType)..':'..tostring(scope)..':'..tostring(cedit.mesh)..'/'..tostring(cedit.forest)..':'..tostring(adesc[cedit.mesh])) --..' i:'..tostring(cij[1])..' j:'..tostring(cij[2]))
		end
  if cedit.forest and dforest[cedit.forest] then
--    U.dump()
    forestMeshName = U.forOrig(dforest[cedit.forest].item:getData():getShapeFile())
--      lo('?? fmn:'..tostring(forestMeshName))
--    ddae[dae].type
--    dforest[cedit.forest].item:get
  end
	if out.lock then return end
 --   local ij = aedit[cpart].desc.ij
	out.avedit = {}
	out.aforest = {}
	out.apop = {}
	local apth = {}
	out.fyell = nil
	out.dyell = {}
	out.fmtop = nil
	local desc = adesc[cedit.mesh]
--    local rayCast = cameraMouseRayCast(false)
--        U.dump(desc.selection, '?? PRE_B:')
	local aid = #asel > 0 and asel or {cedit.mesh}
--        U.dump(aid, '??______________________ for_AID:'..#aid)
	for _,id in pairs(aid) do
		local dfcorner = {}
		if desc.acorner_ and scope ~= 'top' then
			-- use frame for corners
				U.dump(desc.acorner_, '?? frame_CORNER:')
			for i,c in pairs(desc.acorner_) do
				for j,f in pairs(c.list) do
					if not dfcorner[f[1]] then
--							U.dump(ddae[f.dae], '?? for_MESH:')
						dfcorner[f[1]] = U.polyMargin(desc.afloor[f[1]].base, ddae[f.dae] and ddae[f.dae].to.x or 0.5)
					end
				end
			end
				U.dump(dfcorner, '?? frame_DFCORNER:')
		end
		forBuilding(adesc[id], function(w, ij)
--                    lo('?? mark:'..ij[1]..':'..ij[2], true)
	--            lo('?? to_mark:'..ij[1]..':'..ij[2]..':'..tostring(forestType)..':'..tostring(forestMeshName))
			local pop = false
			local p = base2world(adesc[id], ij)
			if w.u:cross(w.v):dot(p - core_camera.getPosition()) < 0 then
				pop = true
			end

			if forestType then
					lo('?? markUp_for_forest:'..forestType, true)
				for d,list in pairs(w.df) do
					for _,key in ipairs(list) do
						if dforest[key].type == forestType then
							local h = ddae[d].to.z - ddae[d].fr.z
							out.avedit[#out.avedit + 1] = dforest[key].item:getPosition() + vec3(0,0,h)
						end
					end
				end
			elseif forestMeshName == nil then
--                    lo('?? to_mark2:'..ij[1]..':'..ij[2])
				if not ij[2] then return end
				local base = adesc[id].afloor[ij[1]].base
				if not base[ij[2]] then
					lo('?? no_1:', true)
					return
				end
				if not adesc[id].pos then
					lo('?? no_2:')
					return
				end
				if dfcorner[ij[1]] then base = dfcorner[ij[1]] end
	--            local p = base[ij[2]] + adesc[cedit.mesh].pos + adesc[cedit.mesh].afloor[ij[1]].pos
	--                    lo('?? for_POP:'..ij[1]..':'..ij[2]..':'..tostring(pop))
				if ij[1] > 1 then pop = false end
				if scope == 'top' then
	--                lo('?? to_mark:'..tostring(ij[1])..':'..tostring(ij[2]))
	--                U.dump()
	--                out.avedit[#out.avedit + 1] = w.pos + w.v
	--                out.avedit[#out.avedit + 1] = out.avedit[#out.avedit] + w.u
--                elseif scope == 'side' then
--                    U.dump(cij, '?? mark_side:'..ij[1]..':'..ij[2])
				else
--                        lo('?? if_mark:'..ij[1]..':'..ij[2],true)
					if w.spany and w.spany > 1 then
						out.avedit[#out.avedit + 1] = base2world(adesc[id], ij)
						if pop then out.apop[#out.avedit] = true end
						out.avedit[#out.avedit + 1] = base2world(adesc[id], {ij[1],U.mod(ij[2]+1,#base)})
						if pop then out.apop[#out.avedit] = true end
						out.avedit[#out.avedit + 1] = w.pos + w.u + w.v
						out.avedit[#out.avedit + 1] = out.avedit[#out.avedit] - w.u
					else
						if U._PRD == 0 then
							out.avedit[#out.avedit + 1] = w.pos
							if pop then out.apop[#out.avedit] = true end
							out.avedit[#out.avedit + 1] = out.avedit[#out.avedit] + w.u
							if pop then out.apop[#out.avedit] = true end
							out.avedit[#out.avedit + 1] = out.avedit[#out.avedit] + w.v
		--                    if pop then out.apop[#out.avedit] = true end
							out.avedit[#out.avedit + 1] = out.avedit[#out.avedit] - w.u
						end

	--                    if pop then out.apop[#out.avedit] = true end
						local ps, u = desc.pos+base[ij[2]]+vec3(0,0,(w.pos-desc.pos).z), U.mod(ij[2]+1,base)-base[ij[2]]
							lo('?? if_CNR:'..tostring(ps)..'/'..tostring(w.pos)..' desc:'..tostring(desc.pos)..' f:'..tostring(desc.afloor[ij[1]].pos))
--						local ps = dfcorner[ij[1]] and dfcorner[ij[1]][ij[2]] or w.pos
--						local u = dfcorner[ij[1]] and or w.u
						if w.pos then
							local pth = {}
							pth[#pth+1] = ps -- dfcorner[ij[1]] and or w.pos
--                                lo('?? markUp_pos:'..ij[2]..':'..tostring(w.pos))
							pth[#pth+1] = pth[#pth] + u -- w.u
							pth[#pth+1] = pth[#pth] + w.v
							pth[#pth+1] = pth[#pth] - u -- w.u
							pth[#pth+1] = ps -- w.pos
							if not out.dyell[ij[1]] then
								out.dyell[ij[1]] = {len=#base}
							end
							out.dyell[ij[1]][ij[2]] = pth
						else
							lo('!! ERR_markUp_NOWALLPOS:'..ij[1]..':'..ij[2])
						end
--                        out.dyell[ij[1]].len = #base
--                        out.dyell[ij[1]] = 0
--                        if not out.dyell[ij[1]][ij[2]] then out.dyell[ij[1]][ij[2]] = {} end
--                        apth[#apth+1] = pth
					end
--                        lo('?? markUp.walls:'..tostring(w.spany),true)
	--                    lo('?? mark_pos:'..tostring(w.pos))
				end
--                    if scope == 'wall' then
--                        U.dump(out.dyell[2], '?? for_DYELL:'..tostring(out.dyell[2] and #out.dyell[2] or nil)..':'..tableSize(out.dyell[2]))
--                        for _,c in ipairs(out.dyell[2]) do
--                            lo('?? for_DY:'.._)
--                        end
--                    end
			else
--	                lo('??^^^^^^^^^^^ mark_forest:'..tostring(ddae[forestMeshName])..':'..tostring(cedit.fscope)) -- ddae[forestMeshName].type, true)
				if ddae[forestMeshName] ~= nil then
					if ddae[forestMeshName].type == 'win' then
						local h = ddae[forestMeshName].to.z-ddae[forestMeshName].fr.z
			--            lo('?? f_height:'..h)
						if w.df[w.win] then
			              	local vn = w.u:cross(vec3(0,0,1)):normalized()
							for _,key in ipairs(w.df[w.win]) do
								if cedit.fscope ~= 1 or key == cedit.forest then
--                      if _==1 then U.dump(ddae[forestMeshName],'?? fordf:'..tostring(vn)) end
									out.avedit[#out.avedit + 1] = dforest[key].item:getPosition() + vec3(0,0,h) + vn*math.abs(ddae[forestMeshName].fr.y)
--                else
--                  lo('?? mark_single_f:'..key..'/'..tostring(cedit.forest))
								end
							end
						end
						if w['win'..'_inf'] and w['win'..'_inf'].ddae then
			--                  U.dump(w['win'..'_inf'].ddae, '?? for_keys:'..tostring(cedit.fscope)..':'..tostring(cedit.forest))
							for k,dae in pairs(w['win'..'_inf'].ddae) do
				--                  U.dump(w.df, '?? for_DF:')
								-- get key
								if w.df[dae] then
								for _,key in pairs(w.df[dae]) do
									if cedit.fscope ~= 1 or key == cedit.forest then
									out.avedit[#out.avedit + 1] = dforest[key].item:getPosition() + vec3(0,0,h)
									end
								end
								end
				--                if cedit.fscope ~= 1 then --or key == cedit.forest then
				--                end
							end
						end

					elseif ({door=1, stairs=1, storefront=1, pilaster=1, stringcourse=1})[ddae[forestMeshName].type] then
--                    if ddae[forestMeshName].type == 'door' then
						local h = ddae[forestMeshName].to.z-ddae[forestMeshName].fr.z
		--                lo('?? mark_door:'..h..':'..ij[1]..':'..ij[2])
--                            lo('?? mu_fof:'..tostring(h)..':'..tostring(w.df[forestMeshName])..':'..tostring(cedit.forest), true)
						if w.df[forestMeshName] then
							for _,key in ipairs(w.df[forestMeshName]) do
								if cedit.fscope ~= 1 or key == cedit.forest then
--                                        lo('?? if_hit:'..key..':'..tostring(dforest[key].item:getPosition())..':'..h)
--                                    out.avedit[#out.avedit + 1] = dforest[key].item:getPosition() + vec3(0,0,h)
									out.aforest[#out.aforest + 1] = dforest[key].item:getPosition()
									if ddae[forestMeshName].type == 'stairs' then
											U.dump(ddae[forestMeshName], '?? for_STAIRS:')
										local daedata = ddae[forestMeshName]
										local w = vec3(w.u.y, -w.u.x):normalized()
										out.aforest[#out.aforest] = out.aforest[#out.aforest] + w*math.abs(daedata.fr.x-daedata.to.x)/2 -- + daePath[forestMeshName]
									end
								end
							end
						else
--                            U.dump(w.df,'!! ERR_NO_F_ELEMENT:'..tostring(forestMeshName)..':'..ij[1]..':'..ij[2])
						end
--[[
						if w.df[w.door] ~= nil then
							for _,key in ipairs(w.df[w.door]) do
								if cedit.fscope ~= 1 or key == cedit.forest then
									out.avedit[#out.avedit + 1] = dforest[key].item:getPosition() + vec3(0,0,h)
								end
							end
						end
]]
					elseif ({corner=1})[ddae[forestMeshName].type] and desc.acorner_ then
--                        lo('?? mark_CORNER:')
						for i,s in pairs(desc.acorner_) do
							for _,n in pairs(s.list) do
								if n[1] == ij[1] and n[2] == ij[2] then
									out.aforest[#out.aforest + 1] = base2world(desc, ij) --dforest[key].item:getPosition()
								end
							end
						end
					end

					if pop and ({balcony=1, pillar=1})[ddae[forestMeshName].type] and w.df[forestMeshName] then
						for _,key in ipairs(w.df[forestMeshName]) do
							out.aforest[#out.aforest + 1] = dforest[key].item:getPosition()
						end
					end
				else
					lo('!! ERR_markUp.NO_ddae:')
				end
			end
		end) --, nil, nil, true)

	end
--        U.dump(out.dyell, '?? mU_dyell:')
	if forestMeshName == nil and cij then
		if cij[3] then
			out.dyell = {}
--            U.dump(desc.afloor[cij[1]].awplus, '??^^^^^^^^^^^^ markUp_awplus:')
			out.dyell[#out.dyell+1] = {desc.afloor[cij[1]].awplus[cij[3][1]].list[cij[3][2]]}
--            U.dump(out.dyell, '??^^^^^^^^^^^^ markUp_dyell:')
		elseif scope == 'top' then
--                lo('?? mark_TOP:'..#out.dyell)
			if cij[1] == nil then
				lo('!! ERR_markUp.no_cij:')
				return
			end
			if cedit and cedit.mesh then
						U.dump(cij, '?? markUp.for_TOP0:'..tostring(cedit.mesh))
				local floor = adesc[cedit.mesh].afloor[cij[1]]
				local h = forHeight(adesc[cedit.mesh].afloor, cij[1])
--                        lo('?? markUp.for_TOP:'..tostring(cij[1])..' ac:'..#floor.top.achild..' cc:'..tostring(floor.top.cchild)..':'..tostring(cij[2])..' h:'..h)
				--        for i = 1,cij[1] do
				--            h = h + adesc[cedit.mesh].afloor[i].h
				--        end
				local posplus = W.atParent(desc)
--                if desc.prn then
--                    lo('?? for_PRN:'..tostring(desc.prn)..':'..#adesc[desc.prn].afloor)
--                    posplus = adesc[desc.prn].pos + vec3(0,0,forHeight(adesc[desc.prn].afloor,desc.floor-1))
--                end
				local arc = {}
				local pth = {}
				if floor.top.cchild and #floor.top.achild > 0 then
					local child = floor.top.achild[floor.top.cchild]
					if child.base then
						--            local base = child.base --U.polyMargin(child.base, child.margin)
							lo('??**************** if_ch_base: cchild:'..floor.top.cchild..':'..tostring(child.base)..':'..#child.base..':'..tostring(child.margin),true)
						local base = U.polyMargin(child.base, child.margin or 0)
						for _,p in pairs(base) do
--                            out.avedit[#out.avedit + 1] = adesc[cedit.mesh].pos + floor.pos + p + vec3(0,0,h)
							pth[#pth+1] = adesc[cedit.mesh].pos + floor.pos + p + vec3(0,0,h+(child.fat or 0)) + posplus
						end
						pth[#pth+1] = adesc[cedit.mesh].pos + floor.pos + base[1] + vec3(0,0,h+(child.fat or 0)) + posplus
						out.fmtop = {pth}
--                        apth[#apth+1] = pth
					end

		--[[
						pth[#pth+1] = w.pos
						pth[#pth+1] = pth[#pth] + w.u
						pth[#pth+1] = pth[#pth] + w.v
						pth[#pth+1] = pth[#pth] - w.u
					for _,i in pairs(floor.top.achild[floor.top.cchild].list) do
						arc[#arc + 1] = floor.top.body[i]
					end
		]]
--                        U.dump(arc, '?? mark child:'..floor.top.cchild)
				else
		--                    lo('?? for_whole:'..tostring(floor.margin))
--                    local base = U.polyMargin(floor.base, floor.top.margin)
--                    local arc = coverUp(base)
--                        arc = {}
					local achunk = T.forChunks(floor.base)
--                        lo('??______ ARC:', true)
					for k,rc in pairs(achunk) do
						for _,i in pairs(rc) do
							local p = floor.base[i]
		--            arc = floor.top.body
--??                    for k,rc in pairs(arc) do
--??                        for l,p in pairs(rc) do
			--                lo('?? for_vert:'..k..':'..l)
--[[
							out.avedit[#out.avedit + 1] = adesc[cedit.mesh].pos +
								floor.pos + p + vec3(0,0,h)
							if desc.prn then
								out.avedit[#out.avedit] = out.avedit[#out.avedit] +
									adesc[desc.prn].pos + vec3(0, 0, forHeight(adesc[desc.prn].afloor, desc.floor-1))
							end
]]
--                            pth[#pth+1] = adesc[cedit.mesh].pos + floor.pos + p + vec3(0,0,h)
						end
--                        pth[#pth+1] = adesc[cedit.mesh].pos + floor.pos + p + vec3(0,0,h)
					end
--                    pth = {}
					local base = U.polyMargin(floor.base, floor.top.margin or 0)
					for _,p in pairs(base) do
						pth[#pth+1] = adesc[cedit.mesh].pos + floor.pos + p + vec3(0,0,h+(floor.top.fat or 0)) + posplus
					end
					pth[#pth+1] = adesc[cedit.mesh].pos + floor.pos + base[1] + vec3(0,0,h+(floor.top.fat or 0)) + posplus
					out.fmtop = {pth}
				end
			else
				lo('!! ERR_no_cedit:'..tostring(cedit))
			end
		end
	elseif forestMeshName then
		if ({corner=1})[ddae[forestMeshName].type] then
--        if ({corner=1, pillar=1})[ddae[forestMeshName].type] then
--        if ddae[forestMeshName].type == 'corner' then
--                lo('?? MU_corner:', true)
--[[
			if desc.df and desc.df[forestMeshName] then
				for _,key in ipairs(desc.df[forestMeshName]) do
--                    lo('?? for_MESH:'.._..':'..key, true)
					out.aforest[#out.aforest + 1] = dforest[key].item:getPosition() + vec3(0,0,0)
				end
			end
]]
		elseif ddae[forestMeshName].type == 'roofborder' then
--            lo('?? markUp_border:'..tostring(cedit.mesh))
			local desc = adesc[cedit.mesh].afloor[#adesc[cedit.mesh].afloor].top
			if desc.cchild ~= nil then
				desc = desc.achild[desc.cchild]
			end
			for _,key in ipairs(desc.df[desc.roofborder[1]]) do
				out.avedit[#out.avedit + 1] = dforest[key].item:getPosition() + vec3(0,0,0.1)
			end
		end
	end

--    out.fyell = apth
--    lo('<< markUp:'..#out.avedit)
end


local function objDown(desc)
	forestClean(desc)
	if desc.id then
		scenetree.findObjectById(desc.id):delete()
	end
	if desc.achild then
		for i,c in pairs(desc.achild) do
			if c.id then
				scenetree.findObjectById(c.id):delete()
			end
		end
	end
end


local function tipDefualt(floor)
	local base = floor.base
	local dim = (base[1] - base[#base/2+1]):length()
	floor.top.tip = dim/4

	return floor.top.tip
end


local function forBase(base, cb)
	if not base then return end
	for i = 1,#base do
		local v1 = base[i % #base + 1] - base[i]
		local v2 = base[(i + 1) % #base + 1] - base[i % #base + 1]
		if v1:length() == 0 then
			v1 = v2:cross(vec3(0,0,1))
		end
		if v2:length() == 0 then
			v2 = v1:cross(vec3(0,0,1))
		end
		cb(v1, v2, i)
	end
end


local function gablePlate(base, ifirst, tip)
	if not ifirst then ifirst = 1 end
	local ps = base[(ifirst - 1) % #base + 1]
	local u = base[ifirst % #base + 1] - ps
	local v = (base[(ifirst + 2) % #base + 1] - ps)/2 + vec3(0,0,tip/1)

	local rc = {ps}
	rc[#rc + 1] = rc[#rc] + u
	rc[#rc + 1] = rc[#rc] + v
	rc[#rc + 1] = rc[#rc] - u

	return rc,u,v,ps
end


local function roofPlate(p, u, v, h, mskip, fskip)
	v = v + vec3(0, 0, h)
	-- build grid
	local ax, ay = {0}, {0}
	for k,list in pairs(mskip) do
		for s in pairs(list) do
			if k % 2 == 0 then
				if #U.index(ay, s) == 0 then
					ay[#ay + 1] = s
				end
			else
				if #U.index(ax, s) == 0 then
					ax[#ax + 1] = s
				end
			end
		end
	end
		U.dump(ax, '?? AX:')
		U.dump(ay, '?? AY:')

	local rc = {p}
	rc[#rc + 1] = rc[#rc] + u
	rc[#rc + 1] = rc[#rc] + v
	rc[#rc + 1] = rc[#rc] - u
end


local function baseSet(nm)
	U.dump(out.inedge, '?? baseSet:'..nm)
	local floor = adesc[cedit.mesh].afloor[out.inedge[1]]
	if nm == 'square' and #floor.base ~= 4 then
		out.inedge = nil
		out.aedge = nil
		return
	end

	local u = floor.base[U.mod(out.inedge[2]+1,#floor.base)] - floor.base[out.inedge[2]]
	u = floor.awall[1].u:normalized()*u:length()
	local v = vec3(-u.y, u.x)
	floor.base = {floor.base[1]}
	floor.awall[1].u = u

	floor.base[#floor.base + 1] = floor.base[#floor.base] + u
	floor.awall[2].u = v

	floor.base[#floor.base + 1] = floor.base[#floor.base] + v
	floor.awall[3].u = -u

	floor.base[#floor.base + 1] = floor.base[#floor.base] - u
	floor.awall[4].u = -v

	out.inedge = nil
	out.aedge = nil

	W.houseUp(adesc[cedit.mesh], cedit.mesh, true)
	markUp()
end


local function roofSet(nm)
		lo('>> roofSet:'..nm..':'..scope..':'..tostring(cij[1]))
--    if scope == 'top' and cij ~= nil then
	if cij ~= nil then
		local floor = adesc[cedit.mesh].afloor[cij[1]]
			lo('?? roofSet:'..#floor.top.achild..':'..tostring(floor.top.cchild),true)

		local c
		if floor.top.cchild then
			c = floor.top.achild[floor.top.cchild]
		elseif #floor.top.achild == 1 then
			c = floor.top.achild[1]
		end
		if c and c.ridge and c.ridge.on and nm ~= 'ridge' then
			c.ridge.on = false
			if #floor.top.achild == 1 then
				if c.id then
					local obj = scenetree.findObjectById(c.id)
					obj:delete()
				end
				table.remove(floor.top.achild,1)
			end
		end

		if U._PRD == 0 then
		end

--[[
		for i,c in pairs(floor.top.achild) do
			if c.ridge and c.ridge.on and nm ~= 'ridge' then
				c.ridge.on = false
				if c.id then
					local obj = scenetree.findObjectById(c.id)
					obj:delete()
				end
			end
		end
		if floor.awplus then -- and floor.awplus.id then
			-- clean up awplus
			for i,wp in pairs(floor.awplus) do
				if wp.id then
--                        U.dump(wp,'?? aw_DEL:'..wp.id,true)
					local obj = scenetree.findObjectById(wp.id)
					if obj then obj:delete() end
				end
			end
			floor.awplus = {}
--                lo('?? rS_awpus_nil:')
		end
]]
		W.floorClear(floor)
				lo('?? roofSet.for_child:'..tostring(floor.top.cchild)..':'..tostring(floor.top.ridge.on))
--        floor.update = true
		if not floor.top.ridge.on and floor.top.cchild ~= nil then
			local subtop = floor.top.achild[floor.top.cchild]
			subtop.shape = nm
			if floor.awplus[floor.top.cchild] then
				floor.awplus[floor.top.cchild].dirty = true
--                    U.dump(floor.awplus[floor.top.cchild], '?? set_AW_DIRTY:'..floor.top.cchild..':'..tostring(scenetree.findObjectById(floor.awplus[floor.top.cchild].id)))
			end
--                U.dump(subtop, '?? sub_roofSet:'..tostring(floor.top.cchild)..':'..subtop.shape..':'..subtop.tip)
--                W.ui.dbg = true
--            if subtop.id then
--                lo('??++++++++++++++++++++++++++++++++++++++++++++++++++++++ for_ID:'..subtop.id)
--                scenetree.findObjectById(subtop.id):delete()
--            end
		else
			local base = floor.base
			local arc = coverUp(base)
			if #arc == 1 then
				base = arc[1]
			end
--            floor.top.margin = default.topmargin
			if nm == 'flat' then
--                    lo('?? for_flat:'..floor.top.margin)
				-- ??
				if floor.top.cchild ~= nil then
					floor.top.achild[floor.top.cchild].shape = 'flat'
				else
					for _,c in pairs(floor.top.achild) do
						c.shape = 'flat'
					end
					floor.top.shape = 'flat'
				end
--                floor.top.margin = 0
--                floor.top.achild = {}
--                floor.top.cchild = nil
			elseif nm == 'gable' then
--                local desctop,base = W.forTop()
--                desctop.adata = T.forGable(base)
				--??
				if false and floor.top.poly ~= 'V' and #base == 6 and floor.top.cchild == nil then
--                if U._PRD ~= 0 and floor.top.poly ~= 'V' and #base == 6 and floor.top.cchild == nil then
					-- BUILD GABLE PLATES MESHES
					floor = adesc[cedit.mesh].afloor[#adesc[cedit.mesh].afloor]
--                    floor.update = false
--                        U.dump(floor, '?? ___________ L-shape:'..tostring(floor.update)..':'..cedit.mesh)
--                    local base = floor.base
					-- get concave vertex
					local icc
					forBase(floor.base, function(v1, v2, ind)
						if icc ~= nil then return end
						if (v1:cross(v2)).z > 0 then
							icc = ind
						end
					end)
					if icc == nil then return end
--                        lo('??____***** icc:'..tostring(icc))
					-- subdivide into rects
					local v1 = base[icc] - base[(icc-2) % #base + 1]
					local v2 = base[icc % #base + 1] - base[icc]
					local v3 = M.step(base, icc - 2)
					local v4 = M.step(base, icc + 1)

					floor.top.body = {}
					for _,c in pairs(floor.top.achild) do
						-- cleanup
						scenetree.findObjectById(c.id):delete()
						forestClean(c)
					end
					floor.top.achild = {}
					floor.top.ang = floor.top.ang ~= nil and floor.top.ang or math.pi/6
					local body = floor.top.body
						U.dump(body, '?? for_v12:'..icc..':'..tostring(v1)..':'..tostring(v2))
					if v1:length() > v2:length() then

						-- PARENT HAT
						local rc1 = {base[(icc - 2)% #base + 1]}
						rc1[#rc1 + 1] = rc1[#rc1] + v1 + v4
						rc1[#rc1 + 1] = rc1[#rc1] - v3
						rc1[#rc1 + 1] = rc1[#rc1] - (v1 + v4)
						body[#body + 1] = rc1

						local tip = v3:length()/2*math.tan(floor.top.ang)
						local plate1 = U.polyMargin(gablePlate(rc1, 1, tip), floor.top.margin, {3})
								U.dump(plate1, '??+++++**** PLATE1 istart:'..1)
						--- set achild
						local aside = U.mod({icc-1, icc+2, icc+3, icc+4}, #base)
						floor.top.achild[#floor.top.achild + 1] = {
							list = {1}, shape = nm, istart = 1,
							mat = floor.top.mat, margin = floor.top.margin, tip = tip,
							base = rc1,
							ax = {{0, 1}, {0, 1}},
							ay = {{0, 1}, {0, 1}},
							aside = aside} --, ang = ang}

						-- CHILD HAT
						local rc = {base[(icc)% #base + 1]}
						rc[#rc + 1] = rc[#rc] + v4
						rc[#rc + 1] = rc[#rc] - (v2 + v3/2)
						rc[#rc + 1] = rc[#rc] - v4
						body[#body + 1] = rc

						tip = v4:length()/2*math.tan(floor.top.ang)
						local plate2 = U.polyMargin(gablePlate(rc, 2, tip), floor.top.margin, {2})
								U.dump(plate2, '??+++++**** PLATE2 istart:'..2)
						local p1, p2 = closestLinePoints(
							plate1[2], plate1[3], plate2[1], plate2[2])
						local aside = U.mod({icc+1, nil, nil, nil}, #base)
						floor.top.achild[#floor.top.achild + 1] = {
							list = {2}, shape = nm, istart = 2,
							mat = floor.top.mat, margin = floor.top.margin, tip = tip,
							base = rc,
							ax = {{0, p2, 1}, {0, 1}},
							ay = {{0, 1}, {0, 1}},
							skip = {{[0] = {[1] = {true, 1}}}, {}},
							aside = aside, mskip = {{2}, {4}}}
					else
						return
					end
				end
			end
			floor.top.shape = nm
			if floor.awplus[0] then floor.awplus[0].dirty = true end
			-- cleanup ridge
			if floor.top.ridge.on then
				floor.top.ridge.on = false
	--                lo('?? if_CHILD:'..floor.top.achild[1].id)
				for _,c in pairs(floor.top.achild) do
					if c.id then
						local obj = scenetree.findObjectById(c.id)
						obj:delete()
					end
				end
				floor.top.achild = nil
				floor.top.cchild = nil
			end
--                lo('?? for_SHAPE:'..cij[1]..':'..floor.top.shape, true)
		end
		W.forTop().adata = nil
--            lo('?? rS_nil_adata:')
--        floor.update = true
--        W.houseUp(nil, cedit.mesh)
		W.houseUp(adesc[cedit.mesh], cedit.mesh,true)
		markUp()
--            U.dump(floor.top.achild[floor.top.cchild], '??******************** post_SET:')
	end
end


local function isRect(base)
	local yes = true
	forBase(base, function(v1, v2)
--        if math.abs(U.vang(v1, v2) - math.pi/2) > small_ang then
		if U.vang(v1, v2) > small_ang and math.abs(U.vang(v1, v2) - math.pi/2) > small_ang then
--        if math.abs(math.pi/2 - U.vang(v1, v2) % math.pi/2) % math.pi/2 > small_ang then
			yes = false
		end
--[[
		if math.abs(U.vang(v1, v2) - math.pi/2) > 0.01 then
			yes = false
		end
]]
	end)
	return yes
end


local function uvOn(ij, desc)
	desc = desc or adesc[cedit.mesh]
	if not desc then return end
	local cheight = 0
	for i,f in pairs(desc.afloor) do
		cheight = cheight + f.h
		for j,w in pairs(f.awall) do
			if ij ~= nil then
				if i == ij[1] then
					if ij[2] ~= nil then
						if j == ij[2] then
							w.uv = {0, cheight, w.u:length(), cheight - f.h}
						end
					else
						w.uv = {0, cheight, w.u:length(), cheight - f.h}
					end
				end
			else
				w.uv = {0, cheight, w.u:length(), cheight - f.h}
			end
--            lo('?? uvOn:'..i..':'..j..':'..cheight..':'..tostring(w.v:length()))
--            w.update = true
		end
	end
end


local function baseOn(floor, base, i, desc)
	if not i then i = cij[1] end
	if not base then
		base = floor.base
	else
		floor.base = base --?? clone(base)
	end
	if not i then
		lo('!! baseOn_NOCIJ:')
		return
	end
	-- update walls

	for j,w in pairs(floor.awall) do
--			lo('?? baseOn_b:'..j..':'..tostring(base[j])..':'..tostring(U.mod(j+1,base)))
		w.u = U.proj2D(U.mod(j+1,base) - base[j])
		w.ij = {i,j}
	end
--        if true then return end

	uvOn({i,nil},desc)
	-- update top
	floor.top.body = coverUp(floor.base)
end


-- if base edge position of descus matches position of some edge in descthem building
local function inBase(descus, ijus, descthem, i)
	local bus = descus.afloor[ijus[1]].base
	local bthem = descthem.afloor[i].base
	for j = 1,#bthem do
		if (bthem[j] - bus[ijus[2]]):length() < small_dist then
			if (U.mod(j+1,bthem)-U.mod(ijus[2]+1,bus)):length() < small_dist then
				return j
			end
		end
	end
end


local function selectionTurn(ang, center)
--    local center = cedit.cval['DragRot'].center
	local desc = adesc[cedit.mesh]
	local ci
	forBuilding(desc, function(w, ij)
		if ij[1] == ci then return end
		ci = ij[1]

		local floor = desc.afloor[ij[1]]
		local c = center - base2world(desc, {1,1})
		for j = 1,#floor.base do
--                  out.apoint[#out.apoint + 1] = center + U.vturn(base2world(desc, {1,j})-center, 0)
			floor.base[j] = U.proj2D(center + U.vturn(base2world(desc, {ij[1],j})-center, ang) - (desc.pos + floor.pos))
		end
		baseOn(desc.afloor[ci], nil, ci)
	end)
end


local function forSpanY(desc, ij, val)
		U.dump(ij, '>> forSpanY:'..tostring(val))
	local w = desc.afloor[ij[1]].awall[ij[2]]
	if val > #desc.afloor - ij[1] + 1 then
		val = #desc.afloor - ij[1] + 1
	end
	local floor = desc.afloor[ij[1]]
	local ima = 1
	for i = 1,val-1 do
		-- check edges coincidence
		local j = inBase(desc, ij, desc, ij[1]+i)
		if not j then
--                lo('?? drop_at:'..i, true)
			ima = i
			break
		end
		ima = i + 1
			lo('?? if_span: i='..i..':'..j..':'..#desc.afloor[ij[1]+i].awall)
		desc.afloor[ij[1]+i].awall[j].skip = true
	end
	val = ima
	w.spany = val

	return val
end


local function forTop()
	if not cij then return end
	local desctop = adesc[cedit.mesh].afloor[cij[1]].top
	local base = adesc[cedit.mesh].afloor[cij[1]].base
	if desctop and desctop.cchild and desctop.achild then
		desctop = desctop.achild[desctop.cchild]
		if desctop then
			base = desctop.base
		end
	end
	return desctop,base
end


local function roofUp(nm, desc, ifloor, ichild, prn)
	if nm == nil then
		nm = 'flat'
--        nm = md_roof
--    elseif desc == nil then
--        md_roof = nm
--        W.houseUp(adesc[cedit.mesh], cedit.mesh)
--        return
	end
		if true or not indrag then
			lo('>> roofUp:'..nm..':'..tostring(cij)..' scope:'..tostring(scope)..':'..ifloor..':'..tostring(ichild))
		end
	local av,af,an,auv
  	local am = {}
	if desc == nil then
		desc = adesc[cedit.mesh]
	end
--    local floor = desc.afloor[#desc.afloor]
	if ifloor == nil then
		ifloor = #desc.afloor
	end
	local floor = desc.afloor[ifloor]
	local bigbase = floor.base
--    local h = forH
	local h = 0
--    for _,f in pairs(desc.afloor) do
	for i = 1,ifloor do
		h = h + desc.afloor[i].h
	end
	local desctop
	local base,margin,tip,ifirst,mat,arc,aside
	if ichild ~= nil then
--            U.dump(floor.top, '??*************************************** FTOP:')
		local child = floor.top.achild[ichild]
--                U.dump(child, '?? roofUp.for_child:')
		margin = child.margin
		tip = child.tip
		ifirst = child.istart
--            ifirst = 2
		mat = child.mat
		base = child.topext or child.base
		aside = child.aside
		desctop = child
--        base = floor.top.body[child.list[1]]
--        base = child.base
--        child.base = base


--        base = U.clone(floor.top.body[child.list[1]])
		-- for flat
--[[
		arc = {}
		for _,irc in pairs(child.list) do
			arc[#arc + 1] = floor.top.body[irc]
		end
]]
--            U.dump(base, '??*** roofUp.for_child:'..ichild..':'..tostring(mat)..':'..tip)
	else
		margin = floor.top.margin
		tip = floor.top.tip
		ifirst = floor.top.istart
--            ifirst = 2
		mat = floor.top.mat
		base = floor.topext or floor.base
		aside = {}
		for i = 1,#bigbase do
			aside[#aside + 1] = i
		end
		desctop = floor.top
--        base = U.polyMargin(base, margin)

--        base = U.polyMargin(floor.base, floor.top.margin)
--        arc = coverUp(base)

--        afloor[#afloor].top.body = cover
--        base = U.clone(floor.base)
--        base = U.polyMargin(base, margin)
--        arc = floor.top.body
--        margin = floor.top.margin
	end
--        U.dump(base, '?? roofUp_base:')--..tostring(W.ui.dbg))
--[[
	local icc
	if base then
		forBase(base, function(v1, v2, ind)
			if icc ~= nil then return end
			if (v1:cross(v2)).z < 0 then
				icc = ind
			end
		end)
		desctop.isconvex = icc == nil
		base = U.polyStraighten(base)
		local apair,amult = T.pairsUp(base)
		for _,m in pairs(amult) do
			if #m > 1 then
				desctop.ismult = true
				break
			end
		end
		desctop.isridge = T.forRidge(floor,base,nil,nil,true)
	end
]]
--        U.dump(amult, '??^^^^^^^^ rU_for_mult:')

	local arc, b2b = coverUp(base)
	if arc and #arc == 1 then
		--??
		base = arc[1]
	end
--        U.dump(base, '?? roofUp_base_:'..#arc)
--[[
	base = U.clone(base)
	-- drop identical from base
	for i = #base,1,-1 do
		if i > 1 and (base[i] - base[i - 1]):length() == 0 then
			table.remove(base, i)
		end
	end
]]
	local plainbase = base  -- no margin
--    base = U.polyMargin(base, 1)
--    base = U.polyMargin(base, margin)
--    arc = coverUp(base)
	if desctop.ridge and desctop.ridge.on then -- and desctop.ridge.data then
	-- desctop is child
--    if floor.top.ridge.on and desctop.ridge and desctop.ridge.on and desctop.ridge.data then
			lo('?? rU_ridge:'..tostring(floor.top.ridge)..' flat:'..tostring(floor.top.ridge.flat)) --..':'..tostring(desctop.ridge.data.av)..':'..tostring(floor.top.tip))
--            U.dump(desctop, '?? dtop:')
--        lo('?? rU_r1:')
		local valid,data
		ifirst = desctop.istart or floor.top.istart or 1
--            U.dump(floor.top.achild, '?? children:')
		if #floor.top.achild == 1 then
			valid,data = T.forRidge(floor,nil,nil, (ifirst-1)%2)
		else
			valid,data = T.forRidge(floor,desctop.base,ichild,(ifirst-1)%2)
		end
		if not valid or not data then
			return
		end
--        lo('?? rU_r2:')
--[[
		if U._PRD == 0 then
--                        U.dump(floor.top.achild[1].ridge.data, '??******************* for_START:'..ifirst,true)
--                        U.dump(desctop.ridge.data.av, '?? AV:')
		end
]]
		local apair = data.apair
		av = data.av -- desctop.ridge.data.av
		af = data.af --desctop.ridge.data.af
--            U.dump(floor.top, '?? roofUp_RIDGE:'..tostring(floor.top.ridge and floor.top.ridge.flat)..':'..desctop.tip..' af:'..#af..' apair:'..tostring(floor.top.ridge.apair and #floor.top.ridge.apair or nil)..' uv:'..tostring(desctop.uvref),true)
		if not apair or #apair == 0 then
--        if not floor.top.ridge.apair or #floor.top.ridge.apair == 0 then
			lo('!! ERR_roofUp_NORIDGE:'..tostring(ichild))
			return
		end
--            local tauv = M.uv4poly({av[2],av[5],av[4]}, 1)
--                U.dump(tauv, '??+++++++++ tauv:')
		auv = {}

		if not floor.top.ridge.flat then
			-- UV for start triangle
			auv = M.uv4poly({av[1],av[6],av[3]}, 1, auv)
			af[1].u = 0
			af[2].u = 1
			af[3].u = 2
			for i=4,#af do
				af[i].u = af[i].u + 3
			end
		end

		an = {vec3(0,0,1)}
--            U.dump(av, '?? for_AV:'..#av) --..':'..tostring(desctop.ridge.flat))
		for i=1,#av,4 do
			-- lift ridge vertices
			av[i+2] = av[i+2] + vec3(0,0,desctop.tip)
			av[i+3] = av[i+3] + vec3(0,0,desctop.tip)
			-- set uvs for plates
			auv = M.uv4poly({av[i],av[i+1],av[i+2],av[i+3]}, 1, auv)
		end
--            U.dump(av, '?? rU.ridge_av:')
--            U.dump(af, '?? rU.ridge_af:')
--            U.dump(auv, '?? rU.ridge_auv:')
		if floor.top.ridge and floor.top.ridge.flat then
			auv = M.uv4poly({av[#av-4],av[#av-1],av[#av-0],av[#av-5]}, 1, auv)
--            auv = M.uv4poly({av[#av-5],av[#av-4],av[#av-1],av[#av-0]}, 1, auv)
--            local duv = {[10]=18, [11]=19, [14]=16, [15]=17}
--[[
			local duv = {[10]=19, [11]=16, [14]=17, [15]=18}
--            local duv = {[10]=16, [11]=17, [14]=18, [15]=19}
			for i=#af-11,#af-6 do
				af[i].u = duv[af[i].u]
			end
]]
--            local duv = {[10]=16, [11]=17, [14]=18, [15]=19}
			local duv = {[10]=19, [11]=16, [14]=17, [15]=18}
			if false then
				-- double-sided
				for i=#af-11,#af-6 do
					af[i].u = duv[af[i].u]
				end
			else
				-- single-sided
				for i=#af-5,#af do
					af[i].u = duv[af[i].u]
				end
			end
		end

		if not floor.top.ridge.flat then
      -- UV for start triangle
      auv = M.uv4poly({av[6],av[1],av[3]}, 1, auv)
			af[2].u = #auv-3
			af[3].u = #auv-1
			af[1].u = #auv-2
			-- UV for end triangle
			auv = M.uv4poly({av[#av-6],av[#av-3],av[#av-4]}, 1, auv)
			af[#af-2].u = #auv-3
			af[#af-1].u = #auv-1
			af[#af].u = #auv-2
		end
--            lo('?? rU_r3:'..tostring(desctop.uvref)..':'..tostring(floor.top.uvref)..':'..tostring(desctop.uvscale)..':'..tostring(floor.top.uvscale))
		if desctop.uvref or desctop.uvscale then
--        lo('?? rU_toscale:'..tostring(desctop.uvscale)..':'..tostring(floor.top.uvscale),true)
			M.uvTransform(auv,desctop.uvref,desctop.uvscale)
		end
		-- to floor height
		--- delta height
		local ibase = apair[1][1]
--        local ibase = floor.top.ridge.apair[ichild][1]
--        local ibase = floor.top.ridge.apair[ichild][1]
		local cp = floor.base[ibase] + vec3(0,0,h)
--                lo('?? for_dh:'..ibase..':'..ifirst,true)
--        local cp = U.mod(ibase-(ifirst-1)%2,floor.base) + vec3(0,0,h)

--            U.dump(av, '??***************** for_HIT:'..tostring(cp), true)
		local dh = U.linePlaneHit({cp, cp+vec3(0,0,1)}, {
			{av[1]+vec3(0,0,h), av[3]+vec3(0,0,h), av[2]+vec3(0,0,h)},
			{av[1]+vec3(0,0,h), av[3]+vec3(0,0,h), av[6]+vec3(0,0,h)}})
				--TODO:
				if #floor.top.achild > 1 then dh = 0 end
--                lo('?? for_DH:'..((ifirst-1)%2)..':'..dh..':'..h)
--                dh = 0
--        if ifirst%2 == 0 then dh = 0 end
		for i=1,#av do
			av[i] = av[i] + desc.pos + floor.pos + vec3(0,0,h-dh)
		end
--[[
		if floor.top.ridge and floor.top.ridge.flat then
		else
			av[#av-1] = av[#av-1] + vec3(0,0,desctop.tip)
			av[#av] = av[#av] + vec3(0,0,desctop.tip)
			auv = M.uv4poly({av[#av-3],av[#av-2],av[#av-1],av[#av]}, 1, auv, true)
		end
]]
--            U.dump(auv, '?? for_AUV:'..#auv)
--            U.dump(af, '?? for_AF:'..#af)
--            auv = {}

--            U.dump(av, '?? roofUp_RIDGE.AV:')
--            U.dump(auv, '?? roofUp_RIDGE.AUV:')
--            U.dump(af, '?? roofUp_RIDGE.AF:')
--            if true then return end
--        desctop.av = av
--[[
		apair = desctop.ridge.data.apair
		for _,p in pairs(apair) do
			U.dump(p, '??************ for_pair:')
--            if r.iridge then
--                av[r.iridge[1] ] = av[r.iridge[1] ]+vec3(0,0,desctop.tip)
--                av[r.iridge[2] ] = av[r.iridge[2] ]+vec3(0,0,desctop.tip)
--            end
		end

		if false then
			for _,r in pairs(aridge) do
				av[r[1] ] = av[r[1] ]+vec3(0,0,floor.top.tip)
				av[r[2] ] = av[r[2] ]+vec3(0,0,floor.top.tip)
			end
	--        av,auv,af = T.pave(base)
	--            U.dump(av, '?? av:'..#av)
		-- TODO: add backside

			for i=1,#av do
				av[i] = av[i] + desc.pos + floor.pos + vec3(0,0,h)
			end
	--            U.dump(av, '??************* pre_AV:'..#av)
	--            U.dump(af, '??************* pre_AF:'..#af)
	--            U.dump(auv, '??************* pre_AUV:'..#auv)
			desctop.av = av
		end

			if false then
				local u = (U.mod(p[1]+1,base) - base[p[1] ]):normalized()
				local v = vec3(-u.y, u.x, 0)
				local ref = base[p[1] ]
				for j=1,6 do
					local b = av[#av-6+j]
	--                for _,b in pairs(baseext) do
					auv[#auv+1] = {u=(b-ref):dot(u), v=(b-ref):dot(v)}
				end
				auv[#auv+1] = {u=0, v=0}
				auv[#auv+1] = {u=0, v=0}
				auv[#auv+1] = {u=0, v=0}
				auv[#auv+1] = {u=0, v=0}
			end
]]
	elseif nm == 'pyramid' then
    	local ridgeW,ridgeMargin = 0.14,0.08
--		local scorner = base[1]
		local sbase = U.clone(base)
		base = U.polyMargin(base, margin)
		local center = base[1]
		for i = 2,#base do
			center = center + base[i]
		end
		center = center/#base

		local pos = desc.pos + floor.pos
--            U.dump(desctop, '??______________________________ roofUp_pyramid:'..ifloor..':'..tostring(floor.pos)..':'..tostring(h)..':'..ifloor) --, true)
			lo('??______________________________ roofUp_pyramid:'..ifloor..':'..tostring(floor.pos)..':'..tostring(h)..':'..#base..' tip:'..tostring(tip)..' fat:'..tostring(desctop.fat))
--                lo('?? pre_tip:'..tip..':'..tostring(center)..':'..tostring(pos))
--        pos = pos + vec3(0,0,h)
		if not tip then tip = tipDefualt(floor) end
		local top = center + tip*vec3(0,0,1)
		local vtip = center + 1*tip*vec3(0,0,1) + vec3(0,0,h) --+ pos
--                U.dump(base, '?? center: cent:'..tostring(center)..' tip:'..tostring(tip))
		av, af, an, auv = {vtip + pos},{},{},{} -- 1st av is tip!
		for i = 1,#base do

			af[#af + 1] = {v = 0, n = #an, u = #auv + 0}
			af[#af + 1] = {v = #av % #base +1, n = #an, u = #auv + 1}
			af[#af + 1] = {v = #av, n = #an, u = #auv + 2}

			av[#av + 1] = base[i] + pos + vec3(0,0,h)
--                lo('?? for_AV:'..#av..':'..tostring(av[#av])..' h:'..h..' pos:'..tostring(pos))
			local sign = 1 -- (i == 1 or i == 4) and -1 or 1
			an[#an+1] = sign * (base[i]-vtip):cross(base[i % #base + 1] - vtip):normalized()
--                    lo('?? for_norm:'..i..':'..(i % #base + 1)..':'..tostring(an[#an]))
			local side = (base[i % #base + 1] - base[i]):length()
			local d = U.toLine(center, {base[i], U.mod(i+1,base)})
--                U.dump(an,'?? for_NORM:'..tostring(d)..':'..tip)
--                    lo('?? for_U:'..i..':'..side..':'..tostring(math.sqrt(((center-base[i]):length())^2-d^2)),true)
			auv[#auv + 1] = {u = side - math.sqrt(((center-base[i]):length())^2-d^2), v = -vec3(d,tip):length()}
			auv[#auv + 1] = {u = 0, v = 0}
			auv[#auv + 1] = {u = side, v = 0}

			-- build ridge
			if desctop.mat == 'm_roof_slates_ochre_bat' then -- or desctop.mat == 'm_roof_slates_rounded_bat' then
		--      if desctop.mat == 'm_roof_slates_ochre_bat' or desctop.mat == 'm_roof_slates_square_bat' then
				local w = (center-base[i]+vec3(0,0,tip)):normalized()
				local u = w:cross(vec3(0,0,1)):normalized()
				local v = u:cross(w)
		--          lo('?? for_RIDGE:'..tostring(v)..':'..tostring(u)..':'..tostring(w)..':'..tostring(base[i]))
				local poly = {
				-u*ridgeW+base[i]+w*ridgeMargin,
				v*ridgeW*0.6+base[i]+w*ridgeMargin,
				u*ridgeW+base[i]+w*ridgeMargin,
				}
				am[#am+1] = M.forBeam(poly,(base[i]-center+vec3(0,0,tip)):length()-ridgeMargin,true,desctop.mat,w)
			end
--      if i == 1 then
--      end
--      am[#am] =
		end

--            U.dump(av, '?? for_PYR_av:')
--            U.dump(af, '?? for_PYR_af:')
--            U.dump(auv, '?? for_PYR_auv:')
--                lo('?? for_AV2:'..tostring(pb))
		-- adjust height according to intersections
				U.dump(sbase, '?? pyra_base:')
		local idh
		for i = 1,#base do
			local dir = (U.mod(i+2,base)-U.mod(i+1,base)):cross(U.mod(i+1,base)-U.mod(i+0,base)).z
			lo('?? dir:'..i..':'..dir)
			if dir < 0 then
				idh = i+2
				break
			end
		end
		idh = idh or 1
--			idh = 2
			lo('?? idh:'..idh..':'..tostring(sbase[idh]))
		local d1 = intersectsRay_Plane(sbase[idh], vec3(0,0,1), top, (top-U.mod(idh,base)):cross(U.mod(idh+1,base)-U.mod(idh,base)))
		local d2 = intersectsRay_Plane(sbase[idh], vec3(0,0,1), top, (top-U.mod(idh,base)):cross(U.mod(idh-1,base)-U.mod(idh,base)))
--		local d1 = intersectsRay_Plane(scorner, vec3(0,0,1), top, (top-base[1]):cross(base[2]-base[1]))
--		local d2 = intersectsRay_Plane(scorner, vec3(0,0,1), top, (top-base[1]):cross(base[#base]-base[1]))
--                U.dump(base,'??++++ corner:'..tostring(scorner)..' tip:'..tostring(tip)..':'..d1..':'..d2)
		local dh = math.min(d1,d2)
			lo('?? pyramid_dh:'..dh)
		for k = 1,#av do
			local hpre = forHeight(desc.afloor, ifloor-1)
--                    lo('?? hPRE:'..tostring(hpre)..':'..tostring(dh)..':'..tostring(pb), true)
			av[k] = av[k] - vec3(0,0,dh) -- vec3(0,0,dh + hpre) --+ floor.pos --s-h-0.1)
		end
--        U.dump(am,'?? ridge_AM:')
		for _,m in pairs(am) do
			for i=1,#m.verts do
				m.verts[i] = m.verts[i] + pos + vec3(0,0,h-dh+desctop.fat)
			end
		end
		desctop.av = av

--[[
		if not floor.top.ridge.on then
			base = U.polyMargin(base, margin)
		end
]]
--            af[#af + 1] = {v = 0, n = #an, u = #auv + 0}
--            af[#af + 1] = {v = #av, n = #an, u = #auv + 1}
--            af[#af + 1] = {v = #av % #base +1, n = #an, u = #auv + 2}
--            an[#an + 1] = vec3(0, 0, 1)
--            lo('?? n:'..tostring(base[i]-tip)..':'..tostring())
--            auv[#auv + 1] = {u = 0, v = -vec3(d,tip):length()}
--            auv[#auv + 1] = {u = side/2, v = -vec3(d,tip):length()}
--[[
		local pb = base2world(desc, {ifloor, desctop.aside and desctop.aside[1] or 1})
--            lo('??++++ for_PYR_pb:'..tostring(desctop.aside)..':'..tostring(pb)..':'..tostring(av[1])..':'..tostring(h)..' av:'..#av)
		local ma1,mi1 = intersectsRay_Plane(pb, vec3(0,0,1), av[1], (av[2]-av[1]):cross(av[3]-av[2]))
--        local ma2,mi2 = intersectsRay_Plane(pb, vec3(0,0,1), av[1], (av[3]-av[1]):cross(av[4]-av[3]))
		local ma2,mi2 = intersectsRay_Plane(pb, vec3(0,0,1), av[1], (av[#av]-av[1]):cross(av[2]-av[1]))
--                lo('?? for_AV3:'..tostring(av[1]))

--            lo('?? mm:'..tostring(ma1)..':'..tostring(ma2))
--        local ma1,mi1 = intersectsRay_Plane(pb, vec3(0,0,1), av[1], (av[2]-av[1]):cross(av[3]-av[2]))
--        local ma2,mi2 = intersectsRay_Plane(pb, vec3(0,0,1), av[1], (av[3]-av[1]):cross(av[4]-av[3]))
--        local ma2,mi2 = intersectsRay_Plane(pb, vec3(0,0,1), av[1], (av[3]-av[1]):cross(av[4]-av[3]))
--        local ma3,mi3 = intersectsRay_Plane(pb, vec3(0,0,1), av[1], (av[3]-av[1]):cross(av[4]-av[3]))
		local dh = math.min(ma1, ma2) - h
]]
--            dh = -3
--            U.dump(av, '?? mami:'..tostring(ma1)..':'..tostring(ma2)..' dh:'..tostring(dh)..':'..floor.h) --..':'..tostring(mi))
--        local s = closestLinePoints(pb, pb + vec3(0,0,1), av[1], av[2])
--        local x,y = U.lineCross(floor.base[1], floor.base[1] + vec3(0,0,1), base[1], av[1])
--            lo('?? cross:'..tostring(pb)..':'..s..':'..tostring(floor.base[1]+vec3(0,0,1)*s)..':'..(s - h))
--            U.dump(av, '?? pyr_AV:'..tostring(dh)..':'..tostring(ifloor))
--            lo('?? for_DH:'..tostring(av[2])..':'..tostring(dh))
--                U.dump(an, '?? NORM_RESET:'..tostring(tip))
--        U.dump(av)
--        U.dump(af)
--        U.dump(an)
--        U.dump(auv)
--        lo('?? mat:'..floor.top.mat)
--        local id, om = meshUp({mdata}, 'lid', groupEdit)
--[[
			if true then
--                    U.dump(floor.awplus, '?? shed_AWPLUS:'..tostring(ichild))
			else

				local awplus = {}
				local n = 1
				for i = ifirst + 1,ifirst + #base - 1 do
		--        for i,k in pairs(aside) do
		--            lo('??+++++ for_side:'..i..':'..n..':'..#floor.awall..':'..((i- 1 ) % #base + 1)..':'..aside[(i- 1 ) % #base + 1])
					local w = floor.awall[aside[(i - 1 ) % #base + 1] ]
					local wallorig = floor.base[aside[(i - 1 ) % #base + 1] ] - vec3(0,0,floor.h) -- - vec3(0,0,h - floor.h)
					if n == 1 then
						w.avplus[#w.avplus + 1] = {
							base[(i - 1) % #base + 1] - wallorig,
							base[i % #base + 1] - wallorig,
							src[3] - wallorig,
						}
						w.avplus[#w.avplus].ind = (i - 1) % #base + 1
					elseif n == 2 then
						w.avplus[#w.avplus + 1] = {
							base[(i - 1) % #base + 1] - wallorig,
							base[i % #base + 1] - wallorig,
							src[4] - wallorig,
							src[3] - wallorig,
						}
						w.avplus[#w.avplus].ind = (i - 1) % #base + 1
					elseif n == 3 then
						w.avplus[#w.avplus + 1] = {
							base[(i - 1) % #base + 1] - wallorig,
							base[i % #base + 1] - wallorig,
							src[4] - wallorig,
						}
						w.avplus[#w.avplus].ind = (i - 1) % #base + 1
					end
		--                U.dump(w.avplus, '?? APLUS:'..i..' w:'..(aside[(i - 1 ) % #base + 1]))
					n = n + 1
				end

			end
]]
--[[
			v1,v2 = floor.base[b2b[U.mod(ifirst+3,#base)] ], floor.base[b2b[U.mod(ifirst+4,#base)] ]
			awlist[#awlist+1] = {
				v1,
				v2,
				(v1+v2)/2 + vec3(0,0,tip)
			}
]]
	elseif nm == 'shed' then
--        av, af, an, auv = {},{},{},{}
--            U.dump(floor.base, '?? rU_shaed_floor:')
--            U.dump(base, '?? rU_shaed:')
--            U.dump(base,'?? rU.shed:'..#base..':'..tostring(desctop.mdata)..':'..ifirst..':'..desctop.tip,true)
--            U.dump(desctop.imap, '?? rU_shed_imap:'..tostring(ichild))
		if desctop.mdata ~= nil then
			av, af, an, auv = desctop.mdata.av, desctop.mdata.af, desctop.mdata.an, desctop.mdata.auv
		else
			if #base < 3 then return end
--                        ifirst = 6
			local cind --= b2b[U.mod(ifirst+1,#base)]
			local sbase = base
			base = U.polyMargin(base, margin)
				lo('?? rU_base_shed:'..#base..':'..ifirst..':'..tostring(desctop.istart),true)
	--            U.dump(base, '?? flat_BASE1:')
			local arc,map = coverUp(base)
--                    U.dump(arc, '??++++++++++++ roofUp_ARC:'..tostring(margin))
--                    U.dump(map, '?? flat_BASE:'..#arc)
			local u = (U.mod(ifirst+1,base)-U.mod(ifirst,base)):normalized()
			local v = vec3(-u.y, u.x, 0)
			av,af,auv,an = {},{},{},{} --{vec3(0,0,1)}

			local v2i = {}
			local ref = base[1]
			for _,a in pairs(arc) do
				local ai = {}
				for k = 1,#a do
					ai[#ai+1] = #av+k
				end
	--                    U.dump(ai, '?? for_ai:')
				M.zip(ai,af) --,true)
	--                M.zip({#av+1,#av+2,#av+3,#av+4},af)
				for k,b in pairs(a) do
					av[#av+1] = b
					auv[#auv+1] = {u=(b-ref):dot(u), v=-(b-ref):dot(v)}
				end
			end
			--TODO: for concave
			if #arc==1 or U._PRD == 2 then

				-- SUBROOF patches
				local awlist = {}
	--                        floor.awplus = {}
	--                        U.dump(av, '??^^^ av:')
	--                        U.dump(base, '??^^^ base:')
	--                        U.dump(sbase, '??^^^ sbase:')
				local dhp,sdhp
				local asdh = {}
				local p
				local dfit = 0
				for i=1,#av do
					local d = U.toLine(av[i], {U.mod(ifirst,base),U.mod(ifirst+1,base)})
					local dh = d>small_dist and d/10*desctop.tip or 0
					local sdh = U.toLine(sbase[i], {U.mod(ifirst,sbase),U.mod(ifirst+1,sbase)})
					if sdh then
						sdh = (sdh and sdh>small_dist and sdh/10*desctop.tip) or 0
						asdh[#asdh+1] = sdh
--                                lo('?? dDIF:'..i..' dh:'..dh..' d:'..d..':'..#an..':'..(dh-sdh)..':'..(av[i]-sbase[i]):length())
						if dh > 0 and #an==0 then
							-- get normal and shift
							an = {u:cross(av[i]+vec3(0,0,dh)-U.mod(ifirst,base)):normalized()}
							dfit = margin*dh/d
						end
--                                dfit = 1
--                                lo('?? dfit:'..dfit)
--                                dfit = 4*dfit
						av[i] = av[i] + vec3(0,0,dh)
--                        av[i] = av[i] + vec3(0,0,dh-dfit)
						sbase[i] = sbase[i] + vec3(0,0,sdh)
					else
						lo('!! ERR_NO_SDH:'..ifirst..'/'..i..':'..tostring(sbase[i])..':'..tostring(U.mod(ifirst,sbase))..':'..tostring(U.mod(ifirst+1,sbase)),true)
					end
				end
				for i=1,#av do
					av[i] = av[i] + desc.pos + floor.pos + vec3(0,0,h-dfit)
					sbase[i] = sbase[i] + desc.pos + floor.pos + vec3(0,0,h)
					if i > 1 then
						cind = i-1
	--                            lo('?? dhDIFF:'..i..':'..(sdh-dh))
						p = U.polyStraighten({
							sbase[i-1] - vec3(0,0,asdh[i-1]),
							sbase[i] - vec3(0,0,asdh[i]),
							sbase[i],
							sbase[i-1],
						})
						if #p>0 then
	--                                lo('?? to_AWP:'..i..':'..#p) --#awlist[#awlist])
							awlist[#awlist+1] = p
	--                                lo('?? for_CIND:'..cind..':'..desctop.imap[cind])
							awlist[#awlist].ind = ichild and desctop.imap[cind] or cind
	--                            awlist[#awlist].ind = i-1-- ichild and desctop.imap[cind] or cind
						end
					end
				end
				-- last patch
				cind = #av
				p = U.polyStraighten({
					sbase[#av] - vec3(0,0,asdh[#av]),
					sbase[1] - vec3(0,0,asdh[1]),
					sbase[1],
					sbase[#av],
				})
				if #p > 0 then
					awlist[#awlist+1] = p
					awlist[#awlist].ind = ichild and desctop.imap[cind] or cind --#av
				end
				ichild = ichild or 0
	--                    U.dump(awlist, '?? awplus2:')
	--                    table.remove(awlist,5)
				if not floor.awplus[ichild] then
					floor.awplus[ichild] = {list=awlist, id=nil}
				else
					floor.awplus[ichild].list = awlist
				end
			else
				for i=1,#av do
					av[i] = av[i] + desc.pos + floor.pos + vec3(0,0,h)
				end
			end
--                    table.remove(floor.awplus[0].list, 3)
--                    U.dump(sbase, '?? awplus2:')
--                    U.dump(floor.awplus, '?? awplus2:')
		end

		if false then
			av, af, an, auv = {},{},{},{}
			if desctop.mdata ~= nil then
				av, af, an, auv = desctop.mdata.av, desctop.mdata.af, desctop.mdata.an, desctop.mdata.auv
			else
	--                U.dump(base, '??********** for_SHED: ifirst:'..ifirst)
	--                U.dump(desctop, '??********** for_SHED: list:'..ifirst)
	--                U.dump(b2b, '??********** for_SHED: b2b:')
	--                U.dump(floor.base, '??********** for_SHED2:')
				tip = 2*tip/3
				local ps = base[(ifirst - 1) % #base + 1]
				local u = base[ifirst % #base + 1] - ps
				local v = base[(ifirst + 2) % #base + 1] - ps + vec3(0,0,tip)
				local un = u:normalized()
				local vn = v:normalized()
				an[#an + 1] = un:cross(vn)
				local rc = {ps}
				rc[#rc + 1] = rc[#rc] + u
				rc[#rc + 1] = rc[#rc] + v
				rc[#rc + 1] = rc[#rc] - u

				local src = rc
				if not floor.top.ridge.on then
	--                rc = U.polyMargin(rc, margin)
				end
				rc = U.polyMargin(rc, margin)
	--                if not indrag then U.dump(rc, '??_____*************** shed_rc:'..tostring(v)..':'..tip) end
				auv[#auv + 1] = {u = (rc[1]-ps):dot(un), v = -(rc[1]-ps):dot(vn)}
				auv[#auv + 1] = {u = (rc[2]-ps):dot(un), v = -(rc[2]-ps):dot(vn)}
				auv[#auv + 1] = {u = (rc[3]-ps):dot(un), v = -(rc[3]-ps):dot(vn)}
				auv[#auv + 1] = {u = (rc[4]-ps):dot(un), v = -(rc[4]-ps):dot(vn)}
				av,af = M.rc(rc[2]-rc[1], rc[4]-rc[1], av, af, desc.pos + floor.pos + rc[1] + vec3(0,0,h)) --, true)
				desctop.av = av

		--                U.dump(bigbase, '?? BB:')
		--                U.dump(plainbase, '?? base:'..ifirst)
		--                U.dump(aside, '?? aside:')
		--                U.dump(src, '?? rc:')
	--                lo('?? shed_FIRST:'..ifirst)
				local pos = base2world(desc, {floor.ij[1],1})-floor.base[1]
	--                    U.dump(base, '?? iFIRST:'..ifirst..':'..floor.ij[1]..':'..tostring(pos))
				-- SUBROOF patches
				local awlist = {}
				local cind = b2b[U.mod(ifirst+1,#base)]
				local v1,v2 = base[cind], base[b2b[U.mod(ifirst+2,#base)]]
				awlist[#awlist+1] = {
					v1,
					v2,
					v2+vec3(0,0,tip)
				}
				awlist[#awlist].ind = ichild and desctop.imap[cind] or cind -- b2b[U.mod(ifirst+1,#base)]
	--            awlist[#awlist].ind = desctop.imap and desctop.imap[]  or b2b[U.mod(ifirst+1,#base)]
				cind = b2b[U.mod(ifirst+2,#base)]
				v1,v2 = base[cind], base[b2b[U.mod(ifirst+3,#base)]]
				awlist[#awlist+1] = {
					v1,
					v2,
					v2+vec3(0,0,tip),
					v1+vec3(0,0,tip),
				}
				awlist[#awlist].ind = ichild and desctop.imap[cind] or cind -- b2b[U.mod(ifirst+2,#base)]
				cind = b2b[U.mod(ifirst+3,#base)]
				v1,v2 = base[cind], base[b2b[U.mod(ifirst+4,#base)]]
				awlist[#awlist+1] = {
					v1,
					v2,
					v1+vec3(0,0,tip)
				}
				awlist[#awlist].ind = ichild and desctop.imap[cind] or cind --b2b[U.mod(ifirst+3,#base)]

	--                U.dump(awlist, '?? awlist_SHED:'..ichild)
				ichild = ichild or 0
				if not floor.awplus[ichild] then
					floor.awplus[ichild] = {list=awlist, id=nil}
				else
					floor.awplus[ichild].list = awlist
				end
				-- to world
				for _,list in pairs(awlist) do
	--                        U.dump(list, '?? for_AW:'..tostring(pos))
					for i=1,#list do
						list[i] = list[i] + pos + vec3(0,0,floor.h)
					end
				end
					U.dump(floor.awplus, '?? awplus:')
			end
		end
--                local v1,v2 = floor.base[b2b[U.mod(ifirst+1,#base)]], floor.base[b2b[U.mod(ifirst+2,#base)]]
--                v1,v2 = floor.base[b2b[U.mod(ifirst+2,#base)]], floor.base[b2b[U.mod(ifirst+3,#base)]]
--                v1,v2 = floor.base[b2b[U.mod(ifirst+3,#base)]], floor.base[b2b[U.mod(ifirst+4,#base)]]
--                if not floor.awplus then floor.awplus = {list={}} end
--                floor.awplus.list = awlist
--                    floor.awplus.list[#floor.awplus.list+1] = list
--        return nil
	elseif nm == 'gable' then
--        base = floor.top.cchild and base or floor.base
--            U.dump(desctop.base, '?? roofUp_gable_base:'..#base..':'..tostring(floor.top.cchild)..':'..#floor.base..':'..tostring(desctop.tip))
		if not floor.top.tip then floor.top.tip = 2 end
		local tip = desctop.tip or floor.top.tip

		local cbase = floor.top.cchild and base or floor.base
--            ifirst = 2
		local sbase = {}
--        if #cbase ~= 4 then
--            ifirst = 1
--        end
		for i = 1,#cbase do
			sbase[#sbase+1] = U.mod(i+(ifirst or 1)-1,cbase)
		end
		local sdata = T.forGable(sbase)
				if not sdata or #sdata == 0 then
					U.dump(sbase, '??_______________ ERR_NO_sdata:')
--                    _dbdrag = true
				end
		local cbase = U.polyStraighten(U.polyMargin(sbase, margin))
		if #cbase ~= 4 then
			ifirst = 1
		end
				lo('?? rU_gable:'..#sbase..':'..#cbase..':'..ifirst)
		local adata,apair = T.forGable(cbase)
		if adata and #adata==#sdata then
			desctop.adata = adata
--                lo('?? rU_gable:'..#desctop.adata..':'..#sdata..':'..#cbase..':'..#sbase)
	--            U.dump(adata[1],'?? pairs:'..#desctop.adata)
	--            U.dump(adata[2],'?? pairs:'..#desctop.adata)
			local awlist = {}
			local pp = W.atParent(desc)
--                lo('??___________________ rU_pp:'..tostring(pp))
			local dh
			for n=1,#desctop.adata,2 do
				local d = desctop.adata[n]
	--        for n,d in pairs(desctop.adata) do
				for i=1,#d.av do
					d.av[i] = d.av[i] + desc.pos + floor.pos + vec3(0,0,h)
					sdata[n].av[i] = sdata[n].av[i] + desc.pos + floor.pos + vec3(0,0,h)
				end
				d.r = math.abs((d.av[1]-d.av[3]):dot((d.av[1]-d.av[4]):cross(vec3(0,0,1)):normalized()))
	--                lo('?? roof_r:'..n..':'..d.r)
				local ctip = tip * d.r/7.
				for k=2,#d.av,3 do
					d.av[k] = d.av[k] + vec3(0,0,ctip)
					if n == 1 and k == 2 then
						-- fit hight for margins
						local ic = apair[1][1]
						dh = intersectsRay_Plane(sdata[n].av[1], vec3(0,0,1),
							d.av[2], (d.av[2]-d.av[1]):cross(d.av[4]-d.av[1]))
	--                    dh = intersectsRay_Plane(sbase[ic], vec3(0,0,1),
	--                        d.av[k], (d.av[k]-cbase[ic]):cross(U.mod(ic+1,cbase)-cbase[ic]))
	--                        lo('?? for_dh:'..tostring(dh)..':'..ic..' b:'..tostring(sbase[ic])..':'..tostring(d.av[k]))
					end

					sdata[n].av[k] = sdata[n].av[k] + vec3(0,0,ctip-dh)
	--                local v1,v2 = sbase[ind],U.mod(ind+1,sbase)
	--                    lo('?? for_AW:'..tostring(v1)..':'..tostring(v2))
					local ind = k == 2 and U.mod(apair[(n+1)/2][1]-1+ifirst-1,#cbase) or U.mod(apair[(n+1)/2][2]-1+ifirst-1,#cbase)
--                    local ind = k == 2 and U.mod(apair[(n+1)/2][1]-1,#cbase) or U.mod(apair[(n+1)/2][2]-1,#cbase)
					awlist[#awlist+1] = {
	--                    v1,(v1+v2)/2+vec3(0,0,ctip),v2
	--                    d.av[k-1],d.av[k],d.av[k+1]
--                        sdata[n].av[k-1],sdata[n].av[k],sdata[n].av[k+1]
						sdata[n].av[k+1],sdata[n].av[k-1],sdata[n].av[k]
					}
					awlist[#awlist].ind = ind
--                    awlist[#awlist].ind = ichild and desctop.imap[ind]
				end
				d.an[1] = (d.av[4]-d.av[1]):cross(d.av[2]-d.av[1])
			end
			ichild = ichild or 0
--                    U.dump(awlist, '?? rU_gable_awlist:')
--                    U.dump(desctop.imap, '?? imap:')
			if not floor.awplus[ichild] then
				floor.awplus[ichild] = {list=awlist, id=nil}
			else
				floor.awplus[ichild].list = awlist
			end
			-- adjust hight + parent origin
			for n=1,#desctop.adata,2 do
				local d = desctop.adata[n]
				for i=1,#d.av do
					d.av[i] = d.av[i] - vec3(0,0,dh) + pp
				end
			end
		else
			desctop.adata = {}
		end
--                lo('?? gable_OUT:'..tostring(#floor.awplus))
				if true then return end
--                if U._PRD == 0 then return end

--[[
		local cind = U.mod(ifirst-1,#base)
		local v1,v2 = base[cind], base[U.mod(ifirst,#base)]
--                U.dump(b2b, '?? rU_gable:'..tostring(v1)..':'..tostring(v2)..':'..tostring(tip)..':'..tostring(cind)..':'..ifirst)
--                    U.dump(b2b, '?? B2B:'..ichild..':'..ifirst..':'..tostring(base[1])..':'..tostring(v1)..':'..tostring(v2))
		awlist[#awlist+1] = {
			v1,
			v2,
			(v1+v2)/2+vec3(0,0,tip)
		}
		awlist[#awlist].ind = ichild and desctop.imap and desctop.imap[cind] or cind

		cind = U.mod(ifirst+1,#base)
		v1,v2 = base[cind], base[U.mod(ifirst+2,#base)]
		awlist[#awlist+1] = {
			v1,
			v2,
			(v1+v2)/2+vec3(0,0,tip)
		}
		awlist[#awlist].ind = ichild and desctop.imap and desctop.imap[cind] or cind
]]
--        desctop.adata = adata
--            lo('??^^^^^^^^^^^^^^^^^^^^^ rU_gabled:'..#adata)
--[[
		local arc = coverUp(base)
		if arc and #arc == 1 then
			base = arc[1]
		end
]]
--            U.dump(base, '?? GABLE:')
--            U.dump(b2b, '?? ARC:'..#arc)
--            lo('??____ GABLE:'..tostring(floor.top.poly)..':'..floor.top.tip..' ifirst:'..ifirst..':'..#base)
--            U.dump(b2b, '?? b2b:')
		av, af, an, auv = {},{},{vec3(0,0,1), vec3(0,0,-1)},{}

		local function forPlate(plate, av, af, auv, orig, matflip)
			if matflip == nil then matflip = 1 end
--                lo('?? FP0:'..#av)
			av,af,auv = M.tri(plate[1], plate[4], plate[2], av, af, auv,
				{(plate[orig % #plate + 1] - plate[orig]):normalized(), matflip*(plate[orig] - plate[(orig - 2) % #plate + 1]):normalized()},
				plate[orig]
			)
			lo('?? FP1:'..#av)
			av,af,auv = M.tri(plate[2], plate[4], plate[3], av, af, auv,
				{(plate[orig % #plate + 1] - plate[orig]):normalized(), matflip*(plate[orig] - plate[(orig - 2) % #plate + 1]):normalized()},
				plate[orig]
			)
			lo('?? FP2:'..#av)
			return av, af, auv
		end

		if tostring(floor.top.poly) == 'V' then
			local mbase = base -- U.polyMargin(base, margin)
			local vtip = vec3(0,0,floor.top.tip)

			av,af,auv = forPlate(U.polyMargin({
				mbase[1],
				mbase[2],
				(mbase[#mbase - 1] + mbase[2])/2 + vtip,
				(mbase[#mbase] + mbase[1])/2 + vtip,
			}, margin, {2,3}), av, af, auv, 1)
			av,af,auv = forPlate(U.polyMargin({
				mbase[2],
				mbase[3],
				(mbase[3] + mbase[#mbase-2])/2 + vtip,
				(mbase[2] + mbase[#mbase-1])/2 + vtip
			}, margin, {3,4}), av, af, auv, 3, -1)
			av,af,auv = forPlate(U.polyMargin({
				mbase[#mbase-2],
				mbase[#mbase-1],
				(mbase[#mbase-1] + mbase[2])/2 + vtip,
				(mbase[#mbase-2] + mbase[3])/2 + vtip
			}, margin, {2,3}), av, af, auv, 1)
			av,af,auv = forPlate(U.polyMargin({
				mbase[#mbase-1],
				mbase[#mbase],
				(mbase[#mbase] + mbase[1])/2 + vtip,
				(mbase[#mbase - 1] + mbase[2])/2 + vtip
			}, margin, {3,4}), av, af, auv, 3, -1)

--[[
					U.dump(av, '?? av:'..#av)
					U.dump(af, '?? af:'..#af)
					U.dump(auv, '?? auv:'..#auv)

]]

			local worig = base[#base]
			floor.awall[#base].avplus = {{
				base[#base] - worig + vec3(0,0,floor.h),
				base[1] - worig + vec3(0,0,floor.h),
				(base[1] + base[#base])/2 + vtip - worig + vec3(0,0,floor.h)
			}}
			worig = base[3]
			floor.awall[3].avplus = {{
				base[3] - worig + vec3(0,0,floor.h),
				base[4] - worig + vec3(0,0,floor.h),
				(base[3] + base[4])/2 + vtip - worig + vec3(0,0,floor.h)
			}}

			for i,v in pairs(av) do
				av[i] = av[i] + desc.pos + floor.pos + vec3(0,0,h)
			end
			desctop.av = av
--                U.dump(base, '?? for_base:')
		else
			local skip, mskip, ax, ay

	--        skip[0] = {[2] = {true, 1}}
	--        skip[1] = {[]}
	--        skip[0] = {[1] = 1} --, [2] = {true, -1}}
	--        local mskip = desctop.mskip ~= nil and desctop.mskip[1] or {}
	--        if #U.index(mskip, 3) == 0 then mskip[#mskip + 1] = 3 end
	--        mskip = {}
	--        base = U.polyMargin(base, margin, mskip)

			local nv = 0
			local rc,u,v  = gablePlate(base, ifirst, tip)
--                    U.dump(rc, '??_______ to_GABLE: ichild:'..tostring(ichild)..':'..tostring(rc))
			local src = rc
	--                U.dump(rc, '??_________ GABLE:'..ifirst)
	--                U.dump(base, '??_________ GABLE_base:'..ifirst)
			av, af, an, auv = {},{},{},{}

			-- plate I
			local rc,u,v,ps  = gablePlate(base, ifirst, tip)
				lo('?? if_GABLE_child:'..tostring(ichild)..':'..#rc,true)
			local src = rc

			local un = u:normalized()
			local vn = v:normalized()
			an[#an + 1] = un:cross(vn)
			an[#an + 1] = -an[#an]

			local mskip = desctop.mskip ~= nil and desctop.mskip[1] or {}
			if #U.index(mskip, 3) == 0 then mskip[#mskip + 1] = 3 end
	--                U.dump(mskip, '?? for_skip1:')
			rc = U.polyMargin(rc, margin, mskip)
			auv[#auv + 1] = {u = (rc[1]-ps):dot(un), v = -(rc[1]-ps):dot(vn)}
			auv[#auv + 1] = {u = (rc[2]-ps):dot(un), v = -(rc[2]-ps):dot(vn)}
			auv[#auv + 1] = {u = (rc[3]-ps):dot(un), v = -(rc[3]-ps):dot(vn)}
			auv[#auv + 1] = {u = (rc[4]-ps):dot(un), v = -(rc[4]-ps):dot(vn)}
			av,af = M.rc(rc[2]-rc[1], rc[4]-rc[1], av, af, desc.pos + floor.pos + rc[1] + vec3(0,0,h), true)

--                        U.dump(av, '?? GABLE_av1:')
			-- plate II
			local pso = base[(ifirst + 1) % #base + 1] -- opposite corner
			u = -u
			v = (ps - base[(ifirst + 2) % #base + 1])/2 + vec3(0,0,tip/1)
			rc = {pso}
			vn = v:normalized()
			rc[#rc + 1] = rc[#rc] + u
			rc[#rc + 1] = rc[#rc] + v
			rc[#rc + 1] = rc[#rc] - u
	--        local src = rc
			mskip = desctop.mskip ~= nil and desctop.mskip[2] or {}
			if #U.index(mskip, 3) == 0 then mskip[#mskip + 1] = 3 end
	--                U.dump(mskip, '?? for_skip2:')
			rc = U.polyMargin(rc, margin, mskip)
	--                U.dump(rc, '?? ___ for_RC2:')
			auv[#auv + 1] = {u = (rc[1]-pso):dot(un), v = -(rc[1]-pso):dot(vn)}
			auv[#auv + 1] = {u = (rc[2]-pso):dot(un), v = -(rc[2]-pso):dot(vn)}
			auv[#auv + 1] = {u = (rc[3]-pso):dot(un), v = -(rc[3]-pso):dot(vn)}
			auv[#auv + 1] = {u = (rc[4]-pso):dot(un), v = -(rc[4]-pso):dot(vn)}
			av,af = M.rc(rc[2]-rc[1], rc[4]-rc[1], av, af, desc.pos + floor.pos + rc[1] + vec3(0,0,h), true)

			desctop.av = av

			-- SUBROOF patches
			local pos = base2world(desc, {floor.ij[1],1})-floor.base[1]

			local awlist = {}
--            local cind = b2b[U.mod(ifirst-1,#base)]
--            local v1,v2 = base[cind], base[b2b[U.mod(ifirst,#base)]]
			local cind = U.mod(ifirst-1,#base)
			local v1,v2 = base[cind], base[U.mod(ifirst,#base)]
--                U.dump(b2b, '?? rU_gable:'..tostring(v1)..':'..tostring(v2)..':'..tostring(tip)..':'..tostring(cind)..':'..ifirst)
--                    U.dump(b2b, '?? B2B:'..ichild..':'..ifirst..':'..tostring(base[1])..':'..tostring(v1)..':'..tostring(v2))
			awlist[#awlist+1] = {
				v1,
				v2,
				(v1+v2)/2+vec3(0,0,tip)
			}
			awlist[#awlist].ind = ichild and desctop.imap and desctop.imap[cind] or cind

--            cind = b2b[U.mod(ifirst+1,#base)]
--            v1,v2 = base[cind], base[b2b[U.mod(ifirst+2,#base)]]
			cind = U.mod(ifirst+1,#base)
			v1,v2 = base[cind], base[U.mod(ifirst+2,#base)]
			awlist[#awlist+1] = {
				v1,
				v2,
				(v1+v2)/2+vec3(0,0,tip)
			}
			awlist[#awlist].ind = ichild and desctop.imap and desctop.imap[cind] or cind

			ichild = ichild or 0
			if not floor.awplus[ichild] then
				floor.awplus[ichild] = {list=awlist, id=nil}
			else
				floor.awplus[ichild].list = awlist
			end
			-- to world
			for _,list in pairs(awlist) do
--                        U.dump(list, '?? for_AW:'..tostring(pos))
				for i=1,#list do
					list[i] = list[i] + pos + vec3(0,0,floor.h)
				end
			end

--                    awlist = {}
	--        rc = U.polyMargin(rc, margin, mskip)
	--        local mskip = {3}

--                    U.dump(desctop,'?? IF_CCHILD:'..tostring(floor.top.cchild)..':'..tostring(desctop.ax))
			-- TODO: make proper check

		end
	elseif nm == 'flat' then
--	    		U.dump(base,'??^^^^^^^^^^^ for_FLAT:')
		-- TODO:
--            U.dump(desctop.body, '??___________________ flat_BASE:'..#base)
--            lo('??_________________________ for_marg:'..tostring(margin))
--            U.dump(base, '?? flat_BASE0:')
		base = U.polyMargin(base, margin)
--            lo('?? rU_base_flat:'..#base,true)
--            U.dump(base, '?? flat_BASE1:')
		if not base or #base < 3 then return end
		if not floor.top.ridge.on then
--				U.dump(av, '?? av:')
--				U.dump(af, '?? af:')
			av,af,an,auv = {},{},{vec3(0,0,1)},{}
			local arc = coverUp(base)
--                U.dump(arc, '??++++++++++++ roofUp_ARC:'..tostring(margin))
--                U.dump(base, '?? flat_BASE:')
			local u = (base[2]-base[1]):normalized()
			local v = vec3(-u.y, u.x, 0)
			local ref = base[1]
			for _,a in pairs(arc) do
				local ai = {}
				for k = 1,#a do
					ai[#ai+1] = #av+k
				end
--                    U.dump(ai, '?? for_ai:')
				M.zip(ai,af) --,true)
--                M.zip({#av+1,#av+2,#av+3,#av+4},af)
				for _,b in pairs(a) do
					av[#av+1] = b
					auv[#auv+1] = {u=(b-ref):dot(u), v=(b-ref):dot(v)}
				end
			end
--			local mdata = M.rcPave(base, {})
--					U.dump(mdata, '?? mdata:')
--			av,auv,af = T.pave(base)
		else
			av,auv,af = T.pave(base)
				U.dump(av, '?? paved:')
			an = {vec3(0,0,1)}
		end
--            U.dump(av, '?? flat_AV:'..#av)
--            U.dump(auv, '?? AUV:'..#auv)
--            U.dump(af, '?? AF:'..#af)
		for i=1,#av do
			av[i] = av[i] + desc.pos + floor.pos + vec3(0,0,h)
		end
--            U.dump(av, '?? av:'..#av)
--            U.dump(av, '??************* pre_AV:'..#av)
--            U.dump(af, '??************* pre_AF:'..#af)
--            U.dump(auv, '??************* pre_AUV:'..#auv)
					-- floor
	end

	if desc.prn then
		prn = prn or adesc[desc.prn]
		local H = forHeight(prn.afloor, desc.floor-1)
--            lo('?? for_prn_roof:'..H..':'..desc.prn.pos)
    if not av or #av == 0 then return end
		for i,_ in pairs(av) do
			av[i] = av[i] + prn.pos + vec3(0,0,H)
		end
	end

	if not av or #av == 0 then return end
	desctop.av = av
	desctop.af = af
	local mdata = {
		verts = av,
		faces = af,
		normals = an,
		uvs = auv,
		material = mat,
	}
  local amdata = {mdata}
  for _,m in pairs(am) do
    amdata[#amdata+1] = m
  end
--[[
		if nm == 'shed' then
			U.dump(mdata, '!! FOR_SHED:')
			return
		end
]]
--        U.dump(mdata.verts, '<< roofUp_AV:')
--        U.dump(mdata, '<< roofUp:')
		lo('<< roofUp: av:'..#av..' af:'..#af..' an:'..#an..':'..#amdata)
--    if not indrag then U.dump(mdata,'<< roofUp:'..mat..':'..#out.avedit) end
	return amdata
--    markUp()
end
--            lo('?? rU_base_flat2:'..#base,true)
--        desctop.av = av
--[[
			for _,b in pairs(base) do
				av[#av+1] = b
				auv[#auv+1] = {u=(b-ref):dot(u), v=(b-ref):dot(v)}
			end
			local af = {}
			for _,c in pairs(achunk) do
				af = M.zip(c, af)
			end
]]
--[[
		local aridge
		if desctop.ridge and desctop.ridge.on then
			T.forRidge(floor)
--            av,auv,af,aridge = T.forRidge(base)
				U.dump(aridge, '?? roofUp.for_RIDGE:'..#av..':'..#auv..':'..#af..':'..floor.top.tip)
			for _,r in pairs(aridge) do
				av[r[1] ] = av[r[1] ]+vec3(0,0,floor.top.tip)
				av[r[2] ] = av[r[2] ]+vec3(0,0,floor.top.tip)
			end
--                av,auv,af = T.pave(base)
		else
			av,auv,af = T.pave(base)
		end
]]
--[[
		if false and isRect(base) then
			local sbase = U.clone(base)
			av,af,an,auv = {},{},{vec3(0,0,1)},{}
			if false then
					lo('?? for_zip:')
				local ex = (base[2]-base[1]):normalized()
				local ey = vec3(-ex.y, ex.x)
				av,af,auv = M.zip(base, av, af, auv, {ex,ey})
				desctop.av = av
				for i,v in pairs(av) do
					av[i] = av[i] + desc.pos + floor.pos + vec3(0,0,h)
				end
			end

			arc = coverUp(base)
			local sarc = coverUp(sbase)
			if #desc.afloor == floor.ij[1] then
--                lo('?? LAST_floor:')
				sarc = coverUp(sbase)
			end
--            if not indrag then U.dump(arc, '??____ FLAT:'..#arc) end
		--                U.dump(arc, '?? cUPPED:'..#arc)
			-- get extents
			local ps = base[1]
			local un = -(base[2] - ps):normalized()
			local vn = (base[#base] - ps):normalized()
			if arc ~= nil then
				for k,rc in pairs(arc) do
--                        if #desc.afloor == floor.ij[1] then U.dump(sarc[k], '??------ for_rc:'..k) end
					-- ceiling
					auv[#auv + 1] = {u = (rc[1]-ps):dot(un), v = (rc[1]-ps):dot(vn)}
					auv[#auv + 1] = {u = (rc[2]-ps):dot(un), v = (rc[2]-ps):dot(vn)}
					auv[#auv + 1] = {u = (rc[3]-ps):dot(un), v = (rc[3]-ps):dot(vn)}
					auv[#auv + 1] = {u = (rc[4]-ps):dot(un), v = (rc[4]-ps):dot(vn)}
--                        lo('?? RC4:'..tostring(rc[4])..':'..#arc)

					av,af = M.rc(rc[2]-rc[1], rc[4]-rc[1], av, af,
						desc.pos + floor.pos + rc[1] + vec3(0,0,h), true)
							U.dump(av, '??_________________ for_rc:'..k)
					--desc.pos + f.base[1] + rc[1] + vec3(0, 0, cheight))
					--                        U.dump(av, '?? c_av:')
				end
				desctop.av = av
					lo('?? for_flat: arc='..#arc..':'..#av)

			else
				lo('!! ERR.roofUp_norect:')
				return nil
			end
		elseif false then
				lo('?? not_RECT:', true)

			-- TODO:
			arc = coverUp(base)
--                U.dump(arc, '?? cup_post:')
				out.apoint = {}
--                out.apath = {}
				for i,r in pairs(arc) do
					local pth = {}
					if true then
						for _,p in pairs(r) do
							pth[#pth+1] = p + desc.pos + floor.pos
--                            out.apoint[#out.apoint+1] = p + desc.pos + floor.pos
						end
--                        pth[#pth+1] = r[1] + desc.pos + floor.pos
					end
--                    out.apath[#out.apath+1] = pth
				end
--                lo('??__________ NOT_rect:'..#base..':'..tostring(floor.top.poly)..':'..tostring(desc.pos)..':'..tostring(floor.pos))
--                    floor.top.poly = 'V'
			if floor.top.poly == 'V' then
				av,af,auv = M.tri(base[1], base[#base], base[2], av, af, auv, {vec3(1,0,0),vec3(0,1,0)}, base[1])
				av,af,auv = M.tri(base[2], base[#base], base[#base-1], av, af, auv, {vec3(1,0,0),vec3(0,1,0)}, base[1])
				av,af,auv = M.tri(base[2], base[#base-1], base[#base-2], av, af, auv, {vec3(1,0,0),vec3(0,1,0)}, base[1])
				av,af,auv = M.tri(base[2], base[#base-2], base[3], av, af, auv, {vec3(1,0,0),vec3(0,1,0)}, base[1])
			else
				local ex = (base[2]-base[1]):normalized()
				local ey = vec3(-ex.y, ex.x)
				av,af,auv = M.zip(base, av, af, auv, {ex,ey})
--                    U.dump(af, '??____ NO_RECT:'..#base..':'..#av..':'..#af)
			end
--                    U.dump(av, '?? av:')
--                    U.dump(af, '?? af:')
			for i,v in pairs(av) do
				av[i] = av[i] + desc.pos + floor.pos + vec3(0,0,h)
			end
--                lo('?? nRECT2:')
--                U.dump(auv)
		end
]]

--[[
		local mu,mv = 0,0
		for i = 2,#base do
			local udot = (base[i] - base[1]):dot(un)
			local vdot = (base[i] - base[1]):dot(vn)
			if udot > mu then mu = udot end
			if vdot > mv then mv = vdot end
		end
]]
--                    lo('?? EXT:'..mu..':'..mv)
--                    out.avedit = {}
--            for i = 2,2 do
--                local rc = f.top.body[i]


local intopchange = false


local function roofBorderUp(desc, toedit)
	local cforest   --??
	if not desc.afloor[#desc.afloor] then return end
	local desctop = desc.afloor[#desc.afloor].top
	if not desctop then return end
	local rbdae
	local L,dL,dC
	local forChild = false
	if desctop.roofborder ~= nil then
		rbdae = desctop.roofborder[1] or desctop.roofborder
	--            lo('?? FOR_RB:'..tostring(rbdae)..':'..desctop.mat..':'..tostring(desctop.roofborder[2]))
		if toedit == nil and desctop.roofborder[2] then
			--- set material
			M.matSet(U.forBackup(rbdae), desctop.mat, desctop.roofborder[2])  --, ddae[rbdae].xml)
		end
		--- place forest
		---- place
		if desctop.cchild ~= nil then
			desctop = desctop.achild[desctop.cchild]
			forChild = true
		end
		---- cleanup forest lists
		cforest = forestClean(desctop)
--[[
		if desctop.df ~= nil then
			for dae,list in pairs(desctop.df) do
				for i,key in pairs(list) do
					if key == cedit.forest then
						cforest = {dae, i}
					end
					editor.removeForestItem(fdata, dforest[key].item)
					dforest[key] = nil
				end
				desctop.df[dae] = {}
			end
		end
]]
		L,dL,dC = ddae[rbdae].to.x - ddae[rbdae].fr.x, 0.6, 0.105
--            U.dump(desctop.av, '?? for_AV:'..tostring(ddae[rbdae].fr)..':'..tostring(ddae[rbdae].to)..':'..L)
	end
--        U.dump(ddae[rbdae].fr, '?? FR:')
--        U.dump(ddae[rbdae].to, '?? TO:')
	--========================================
	--  ROOF BORDER
	--========================================
	if false and (#desc.afloor[#desc.afloor].base <= 6 or forChild) and desctop.shape == 'gable' then
--    if true and (#desc.afloor[#desc.afloor].base <= 6 or forChild) and desctop.shape == 'gable' then

		local function forSloped(dsc, i1, i2, toflip)
			dC = 0.14
			local u = dsc.av[i2] - dsc.av[i1]
			toflip = toflip == nil and (math.abs(i1 - i2) > 1) or toflip
			local n = math.floor(u:length()/L)
			local scale = (u:length() + dC)/(n*L - (n-1)*dL)
			u = u:normalized()
			--- a
			local an = {u:cross(vec3(0,0,1)):normalized()}
			local ang = math.atan2(-an[1].y, an[1].x) - math.pi/2
			if toflip then
				ang = ang + math.pi
			end
			-- b
			local hyp = (dsc.av[i2] - dsc.av[i1]):length()
			local yng = math.asin((dsc.av[i2].z - dsc.av[i1].z)/hyp)
			if toflip then
				yng = -yng
			end

			local l = L * scale
			local dl = dL * scale
			local p = dsc.av[i1] + (L/2 * scale - dC) * u
				lo('?? forSloped_pre_to:'..i1..':'..i2..':'..n)
			for j = 1,n do
				local itemkey = to(desctop, rbdae, p, vec3(0,yng,ang), cforest, scale)
				p = p + (l - dl)*u
			end
		end

		local function forStraight(dsc, i1, i2, i3)
			dC = 0.17
	--                    for i,v in pairs(dsc.av) do
	--                        lo('?? for_v:'..i..':'..tostring(v-desc.pos))
	--                    end
	--                    U.dump(desctop.av, '?? forStraight:'..i1..':'..i2..':'..i3)
			local u = dsc.av[i2] - dsc.av[i1]
			local n = math.floor(u:length()/L)
			local scale = (u:length() + 2*dC)/(n*L - (n-1)*dL - 0*dC)
			u = u:normalized()
			--- a
			local an = {u:cross(vec3(0,0,1)):normalized()}
			local ang = math.atan2(-an[1].y, an[1].x) - math.pi/2
	--        local bng = 0
	--        local uc = (desctop.av[2] + desctop.av[1])/2
			local hyp = (dsc.av[i3] - dsc.av[i2]):length()
--                    lo('?? for_BANG:'..hyp..':'..dsc.tip)
			local bng = -math.asin((dsc.av[i3].z - dsc.av[i2].z)/hyp)


			local l = L * scale
			local dl = dL * scale
			local p = dsc.av[i1] + (L/2 * scale - dC) * u
			for j = 1,n do
				local itemkey = to(desctop, rbdae, p, vec3(bng,0,ang), cforest, scale)
				p = p + (l - dl)*u
			end
		end

		if desc.afloor[#desc.afloor].top.poly == 'V' then
			U.dump(desctop, '?? border_FOR_V:')

			forStraight(desctop, 1, 3, 6)
			forStraight(desctop, 7, 9, 12)

			forSloped(desctop, 1, 2, true)
			forSloped(desctop, 9, 12, false)

			forStraight(desctop, 13, 15, 18)
			forStraight(desctop, 19, 21, 24)

			forSloped(desctop, 13, 14, true)
			forSloped(desctop, 22, 24, false)

		else
			if desctop.achild and #desctop.achild > 0 then
						lo('?? ****GABLE_TOT:'..#desctop.achild)
	-- L-shaped base only
				-- parent hat
				local c = desctop.achild[1]
	--            for _,c in pairs(desctop.achild) do
--                    U.dump(c, '?? for_child:')
				--- plate I
				forStraight(c, 1, 2, 4)
				forSloped(c, 2, 4, false)
				forSloped(c, 1, 3, true)
				--- plate II
				forStraight(c, 5, 6, 8)
				forSloped(c, 6, 8, false)
				forSloped(c, 5, 7, true)

				-- child hat
				c = desctop.achild[2]
				forStraight(c, 1, 2, 5)
				forSloped(c, 1, 4, true)

				forStraight(c, 7, 8, 10)
				forSloped(c, 8, 10, false)
			else
				forStraight(desctop, 1, 2, 3)
				forStraight(desctop, 5, 6, 7)
				forSloped(desctop, 2, 3)
				forSloped(desctop, 1, 4)
				forSloped(desctop, 6, 7)
				forSloped(desctop, 5, 8)
			end

		end

	end

end

-- (desc) - new building
-- (nil, toedit) - rebuild
-- (desc, toedit, update) - update existing in edit mode
-- (desc, toedit) - quit edit, join mesh
local function houseUp(desc, toedit, update, prn, fordae)
		lo('>>**************** houseUp:'..tostring(desc)..':'..tostring(toedit)..' prn:'..tostring(prn)..':'..tostring(update)..':'..tostring(desc and desc.prn or nil)..' nfloors:'..tostring(desc and #desc.afloor or nil)..':'..tostring(cedit.forest)..':'..tableSize(adesc)) --..tostring(scenetree.findClassObjects('Roads33'))..':'..tostring(scenetree.findObject("Vegetation")))
	local om
	local forsplit = false
	if desc == nil then
		if not toedit then return end
		-- split mesh to material-wise components
		forsplit = true
    if cedit.mesh and scenetree.findObject('edit') then
  		scenetree.findObject('edit'):deleteAllObjects()
    end
		out.avedit = {}
		desc = adesc[toedit]
    if not desc then
      lo('!! houseUp_NODESC:'..tostring(toedit)..':'..tableSize(adesc)..':'..tostring(adesc[tonumber(toedit)])..':'..tostring(adesc[toedit..'']))
--      for k,d in pairs(adesc) do
--      return
    end
		om = scenetree.findObjectById(toedit)
		om:createMesh({})
--        om:createMesh({{{}}})
		cedit.mesh = toedit
--                U.dump(desc, '?? toedit:')
--                return
	end
	local inedit = false
	local dirty = update and true or false
	local cheight = 0
	local am = {}
	local cheight = 0

  if not desc then
    lo('!! nU_NODESC:', true)
    return
  end

	forestClean(desc)

	local mdata
	for i,f in pairs(desc.afloor) do
		forestClean(f.top)
		f.ij = {i}
--            U.dump(f.awplus, '??_____________________+++++++++++ for_AWP:'..i)
--            if i == 2 then U.dump(f.awplus, '??_____________________+++++++++++ for_AWP:') end
		if not f.awplus then f.awplus = {} end
		for k,wp in pairs(f.awplus) do
--                U.dump(wp, '?? for_WP:'..i..':'.._)
			if wp.dirty then
				if wp.id then
--                        lo('?? to_DEL:'..k..':'..tostring(scenetree.findObjectById(wp.id)))
					scenetree.findObjectById(wp.id):delete()
--                    wp.id = nil
				end
--                wp.dirty = false
				f.awplus[k] = nil
			end
		end
--        f.awplus = {}
		if f.update then
				lo('?? dirty1:')
			dirty = true
			f.update = false
		end
--            lo('?? f_dirt:'..i..':'..tostring(dirty))
			lo('??++++ FLOOR: '..i..'/'..#desc.afloor..' walls:'..#f.awall..':'..tostring(prn)..':'..tostring(desc.prn)..' achild:'..tostring((f.top and f.top.achild and #f.top.achild) or nil)) --..':'..tostring(desc.prn and adesc[desc.prn].pos or nil))
		local p = desc.pos + f.pos + f.base[1]
		if desc.prn then
			prn = prn or adesc[desc.prn]
			p = p + prn.pos
		end
		if f.achild then
				lo('?? FLOOR_children:'..#f.achild, true)
			for _,c in pairs(f.achild) do
					lo('?? child_id:'..tostring(c.id)..':'..tostring(c.prn),true)
--                    U.dump(c,'?? child_id:'..tostring(c.id)..':'..tostring(c.prn),true)
--                c.prn = f
				if not c.id then
					-- build mesh
					houseUp(c, nil, nil, desc)
				else
					-- update position
--                        U.dump(c.afloor[1].top, '?? child.for_TOP:')
					houseUp(c)
--                    houseUp(c, c.id, nil, desc)
--[[
						lo('?? repl:'..tostring(c.data))
					for _,m in pairs(c.data) do
						for n = 1,#m.verts do
							m.verts[n] = m.verts[n] + vec3(1,0,0)
						end
					end
					local obj = scenetree.findObjectById(c.id)
					obj:createMesh({})
					obj:createMesh({c.data})
]]
				end
			end
		end

		-- WALLS
    	local botext
		for j,w in pairs(f.awall) do
--                lo('?? w_upd:'..i..':'..j..':'..tostring(w.update)..'/'..tostring(dirty))
			if w.update then
				dirty = true
--            else
--                dirty = false
			end
			w.ij = {i,j}
			w.v = vec3(0, 0, f.h)
			w.pos = p + vec3(0, 0, cheight) -- p changes for walls
			local spillarInout = w.pillar and w.pillar.inout or nil
			if f.pillarinout and w.pillar and not w.pillar.inout then
				w.pillar.inout = f.pillarinout
			end
			if w.pillar and i > 1 then
				w.pillar.down = w.pillar.down or desc.afloor[i-1].h
			end
			if desc.prn then
--                    lo('?? for_wall:'..)
				prn = prn or adesc[desc.prn]
				w.pos = w.pos + vec3(0, 0, forHeight(prn.afloor, desc.floor-1))
			end
			local spany = w.spany or 1
			if spany > 1 then
					lo('?? to_span:'..i..':'..j..' spany:'..spany)
				spany = forSpanY(desc, {i,j}, spany)
					lo('?? can_SPAN:'..spany)
			end
			local H = 0
			for k = 1,spany do
				H = H + desc.afloor[i+k-1].h
			end
			local av,af,an,auv,mhole,arcext
			for k = 1,spany do
--                    lo('?? hu.for_wall:'..i..':'..j..':'..k..' span:'..tostring(w.spany), true)
--                    if w.win and w.df and w.df[w.win] then
--                        lo('?? foff:'..#w.df[w.win])
--                    end
				w.v = vec3(0, 0, H/spany)
				if j > 1 then
--                    w.uv[3] = 2.2
					w.uv[1] = f.awall[U.mod(j-1,#f.base)].uv[3] -- + f.awall[U.mod(j-1,#f.base)].u:length()
					w.uv[3] = w.uv[1] + w.u:length()
				end
--                if i == 2 and j == 2 then
--                end
--                        lo('?? pre_WALL:'..i..':'..j)
        if not w.uvref then
--          U.dump(w.uvref,'?? for_UV_ref:'..i..':'..j)
--          U.dump(w.uvscale,'?? for_UV_scale:'..i..':'..j)
          if i > 1 then
--            w.uvref = {0, forHeight(desc.afloor, i-1)/(w.uvscale[2] or 1)}
            w.uvref = {0, forHeight(desc.afloor, i-1)}
--            U.dump(w.uvref,'?? ht:'..forHeight(desc.afloor, i-1))
          end
        end
				av,af,an,auv,mhole,arcext = wallUp(w, desc, k) --, (i==2 and j==5 and true) or false)
--            U.dump(arcext, '?? if_ARCEXT: wall:'..j..':'..tostring(#arcext))
        if arcext and #arcext>0 then
--        if U._PRD == 0 and #arcext>0 then
--            U.dump(arcext,'??**************** rcEXT:'..#arcext)
          if i < #desc.afloor then
            local fe = desc.afloor[i+1]
            local dfpos = fe.pos-f.pos
            local base = fe.base
--                U.dump(base, '?? uppder_BASE:')
            if not botext then
              -- initialize
              for je,b in pairs(base) do
                  fe.awall[je].arcext = nil
              end
            end
            for je,b in pairs(base) do

              local we = fe.awall[je]
--              we.arcext = nil
--                  if je == 1 and k == 1 then
--                    lo('?? for_ch:'..tostring(rc[1])..':'..tostring(b)..':'..tostring(U.mod(je+1,base)))
--                  end

              local un = we.u:normalized()
              for k,rc in pairs(arcext) do
--                      lo('?? to_check:'..tostring(base[je])..':'..tostring(U.mod(je+1,base))..'<'..tostring(f.base[j]+(w.u:normalized()*rc[1].x)))
                  if U.between(base[je]+dfpos,U.mod(je+1,base)+dfpos,
                    {f.base[j]+(w.u:normalized()*rc[1].x),f.base[j]+(w.u:normalized()*rc[2].x)},U.small_val) then
--                        lo('?? match_wall:'..j..':'..k)
                    local ds = (base[je]-f.base[j]):length()+dfpos:dot(un)
                    if not we.arcext then
                      we.arcext = {}
                    end
                    if not botext then
                      botext = {}
--                      desc.afloor[i+1].botext = deepcopy(desc.afloor[i+1].base)
--                      botext = {} --deepcopy(desc.afloor[i+1].base)
                    end
                    if not botext[je] then
                      botext[je] = {}
                    end
                    for ie,v in pairs(rc) do
                      rc[ie] = rc[ie] - vec3(ds,f.h,0) -- (U.mod(j+1,f.base) - f.base[j]):normalized()*ds
                    end
--                        U.dump(rc, '?? to_WALL:'..je..':'..k..':'..tostring(U.mod(j+1,f.base) - f.base[j]))
                    we.arcext[#we.arcext+1] = rc
                    local vn = -we.u:cross(vec3(0,0,1)):normalized()
                    local dw = 0.2
                    botext[je][#botext[je]+1] = b + un*rc[1].x
                    botext[je][#botext[je]+1] = botext[je][#botext[je]] + vn*dw
                    botext[je][#botext[je]+1] = botext[je][#botext[je]] + un*(rc[2].x-rc[1].x)
                    botext[je][#botext[je]+1] = botext[je][#botext[je]] - vn*dw
--[[
                    local acut = {b + un*rc[1].x}
                    acut[#acut+1] = acut[#acut] + vn*dw
                    acut[#acut+1] = acut[#acut] + un*(rc[2].x-rc[1].x)
                    acut[#acut+1] = acut[#acut] - vn*dw
                    botext[je] = acut
]]
--                    botext[je][#botext[je]+1] = {b+we.u:normalized()*rc[1].x, b+we.u:normalized()*rc[1].x+}
--                    table.insert(botext, U.mod(je+1,))
                  end
              end
--              if we.arcext then
--                desc.afloor[i+1].bottom = false
--              end
--                  if we.arcext then
--                    U.dump(base,'?? for_WEE:'..je..':'..tostring(base[je])..'>'..tostring(base[je+1])..we.ij[1]..':'..we.ij[2]..':'..#we.arcext..':'..tostring(we.u))
--                  end
            end

--              U.dump(botext, '?? if_BE:'..j)

            local function base2ext(base, dext)
              if not dext then return end
              local bote = {}
              for je=1,#base do
                bote[#bote+1] = base[je]
                if dext[je] then
                  for _,v in pairs(dext[je]) do
                    bote[#bote+1] = v
                  end
                end
              end
              return bote
            end

            local bote
            if botext then
--                  U.dump(botext, '?? for_BOTEXT:')
              bote = base2ext(base, botext)
--                  lo('?? if_HCH:'..i..':'..j..':'..tostring(f.top.achild and #f.top.achild or nil)..':'..tostring(bote))
              if f.top.achild and #f.top.achild>0 then
                for k,c in pairs(f.top.achild) do
--                    U.dump(c, '??^^^^^^^^^^^^^^^^^ for_CEST:'..k)
                  local botec
                  for icb,je in pairs(c.imap) do
                    if botext[je] then
--                        U.dump(botext[je],'??__________+++++++++++++++ be_CHECK:'..k..':'..je..':'..icb)
--                        U.dump(c.base, '?? child_BASE:'..tostring(c.base[icb])..':'..tostring(U.mod(icb+1,c.base)))
                      for ie=1,#botext[je],4 do
                        if U.between(c.base[icb],U.mod(icb+1,c.base),
                          {botext[je][ie],botext[je][ie+3]},U.small_val) then
--                                lo('?? match:'..k..':'..ie..':'..icb..':'..tostring(botext[je][ie])..':'..tostring(botec and #botec or nil))
                            if not botec then
                              botec = {}
                            end
                            if not botec[icb] then
                              botec[icb] = {}
                            end
                            botec[icb][#botec[icb]+1] = botext[je][ie]
                            botec[icb][#botec[icb]+1] = botext[je][ie+1]
                            botec[icb][#botec[icb]+1] = botext[je][ie+2]
                            botec[icb][#botec[icb]+1] = botext[je][ie+3]
                        end
--                        bote[#bote+1] = v
                      end
                    end
                  end
--                      U.dump(botec, '?? for_EXBC:')
                  c.topext = base2ext(c.base, botec)
--                      U.dump(c.topext, '??^^^^^^^^^^^^^------------- for_CTOPEXT:'..i..':'..k)
                end
              else
--                    lo('?? for_FULL:'..tostring(bote))
                f.topext = bote
--                    U.dump(f.topext,'??^^^^^^^^^^^^^^^ hU.topext:'..f.ij[1])
              end
--                U.dump(bote, '?? for_BOTE:')
            end
            desc.afloor[i+1].botext = bote
--                lo('?? if_child:'..tostring(f.top.achild and #f.top.achild or nil)..':'..tostring(bote))
          end
        else
          if i<#desc.afloor then
            f.topext = nil
            desc.afloor[i+1].botext = nil
            for k,w in pairs(desc.afloor[i+1].awall) do
              w.arcext = nil
            end
          end
        end
--                        if i==2 and j==5 then
--                            U.dump(w, '??********** for_AF:')
--                        end

--[[
										if i==2 and j == 2 then
											lo('??++++++++++++ test_MAT:'..#av..':'..#af)
											if #av == 4 and #af == 6 then
												W.out.testMAT = {}
											end
										end
						if i==2 and j == 2 then
							U.dump(an, '??++++++++++++ for_WALL_an:'..j..':'..tostring(w.pos)..':'..#av..':'..#af)
							U.dump(av, '?? av:')
							U.dump(af, '?? af:')
							U.dump(auv, '?? auv:')
						end
]]
--                        U.dump(av, '?? for_WALL:'..j..':'..tostring(w.pos))
--                    lo('?? postWU:'..#am..':'..#av)
				if forsplit then
					out.avedit[#out.avedit + 1] = w.pos
					out.avedit[#out.avedit + 1] = out.avedit[#out.avedit] + w.u
					out.avedit[#out.avedit + 1] = out.avedit[#out.avedit] + w.v
					out.avedit[#out.avedit + 1] = out.avedit[#out.avedit] - w.u
				end
				--??
				if w.pillar then w.pillar.inout = spillarInout end
				if toedit ~= nil then
					-- link forest items
					for dae,list in pairs(w.df) do
						for _,key in ipairs(list) do
	--                        lo('?? for_key:'..key..':'..tostring(dforest[key]))
							dforest[key].mesh = toedit
						end
					end
				end
	--                U.dump(desc, '?? desc:')
	--                U.dump(av, '?? av:')
--                    lo('?? postWA2:')
--[[
				-- backside
				if false and #av > 0 then
					local avb, afb = M.rect(w.u, w.v, nil, nil, w.pos, false)
					am[#am + 1] = {
						verts = avb,
						faces = afb,
						normals = {w.u:cross(w.v):normalized()},
						uvs = {
							{u = 0, v = 1},
							{u = 0, v = 0},
							{u = 1, v = 0},
							{u = 1, v = 1},
						},
						material = 'm_metal_brushed',
					}
--                        U.dump(mhole, '??________ MHOLE:')
				end
]]

				-- achild holes
				if mhole and mhole.faces and #mhole.faces > 0 then
					am[#am + 1] = mhole
				end

				if #av > 0 then
--                if #av > 0 and i == 2 and j == 2 then
          if true then
            table.insert(am, 1, {
              verts = av,
              faces = af,
              normals = an,
              uvs = auv,
              material = fordae or w.mat,
            })
          else
            am[#am + 1] = {
              verts = av,
              faces = af,
              normals = an,
              uvs = auv,
              material = w.mat,
            }
          end
--[[
]]
--                        if i == 2 and j == 2 then
--                            lo('??^^^^^^^^^^^^^ test_AM:'..#am) --..':'..tostring(#mhole.faces))
--                        end
				end

				if w.fringe then
					local vp = w.u:cross(w.v):normalized()
					local av, af
	--                base2world(desc, {i,j}, base[j])
					local u = U.mod(j+1,fbase) - U.mod(j,fbase)
					if w.fringe.h < 0 then
--                            lo('?? for_u:'..tostring(u)..':'..tostring(w.u))
						av, af = M.rect(u, w.v:normalized()*w.fringe.h, nil, nil,
--                        av, af = M.rect(w.u, w.v:normalized()*w.fringe.h, nil, nil,
							base2world(desc, {i,j}, fbase[j]))
		--                        M.avShift(av, w.fringe.inout)
					end
					if av then
--                        aedit[w.id].mesh[#aedit[w.id].mesh + 1]
						am[#am + 1] = {
							verts = av,
							faces = af,
							normals = {vp},
							uvs = {
								{u = 0, v = 0},
								{u = u:length(), v = 0},
								{u = u:length(), v = w.fringe.h},
								{u = 0, v = w.fringe.h},
							},
							material = w.fringe.mat,
						}
					end
				end

				if k < spany then
					w.pos = w.pos + vec3(0,0,H/spany)
				end
			end
--                lo('?? postWU2:'..#am..':'..#av..':'..tostring(dirty))
--            wallUp(p + vec3(0, 0, cheight), w, 'o_wall_'..i..'_'..j)
--            wallUp(f.base[j] + vec3(0, 0, cheight), w, 'w'..i..j)
--                    if intopchange then
--                        U.dump(f.base, '?? CHANGED_BASE:'..j)
--                    end
			p = p + (f.base[j % #f.base + 1] - f.base[j])

--          lo('?? pre_cam:'..i..':'..j..':'..#am..':'..spany..':'..tostring(dirty), true)
--            if #av>0 then
--            end
			if dirty then
--                if w.ij[1] == 2 and w.ij[2] == 3 and mhole then
--                    U.dump(mhole, '?? for_holes:')
--                end
--            if dirty or w.update == true then
-- UPDATE MESH
--                inedit = true
--                        lo('?? for_wall:'..tostring(w.id)..':'..i..':'..j)
				local wall = scenetree.findObjectById(w.id)
				if wall ~= nil then
					wall:createMesh({})
					local cam = {}
					for k = 1,spany do
            table.insert(cam, 1, am[1])
--!!
--						table.insert(cam, 1, am[#am-2*k+2])
--                        table.insert(cam, 1, am[#am-2*k+1])
					end
--                        lo('??________ pre_CM:'..i..':'..j..':'..#cam..':'..spany)
--                        U.dump(cam, '??________ pre_CM:'..i..':'..j..':'..#cam)
					if #cam > 0 then
						local isvalid = true
						for _,m in pairs(cam) do
							if not m or #m.faces == 0 or #m.verts == 0 then
								isvalid = false
								break
							end
						end
						if isvalid then
							wall:createMesh({cam})
							aedit[w.id].mesh = am[#am]
							aedit[w.id].mhole = mhole
						else
							U.dump(cam, '!! ERR_houseUp_MESH:'..i..':'..j)
						end
					end
--                        lo('?? post_CM:')

--[[
					if w.spany == 2 then
						wall:createMesh({{am[#am-3], am[#am-2], am[#am-1], am[#am]}})
					else
						wall:createMesh({{am[#am-1], am[#am]}})
					end
]]
					w.update = nil
				else
					lo('!! ERR_WALL:'..tostring(w.id))
				end
--                local none = {verts={},uvs={},normals={},faces={},material=''}
--                local none = {}
			elseif forsplit then
-- SPLIT MESH
				local cam = {}
				for k = 1,spany do
--          cam[#cam+1] =
					table.insert(cam, 1, am[1])
--					table.insert(cam, 1, am[#am-2*k+2])
--!!                    table.insert(cam, 1, am[#am-2*k+1])
				end
--                lo('?? for_edit:'..tostring(groupEdit))
				local id,om = meshUp(cam, 'wall', groupEdit)
--                local id,om = meshUp({am[#am-1], am[#am]}, 'wall', groupEdit)
--                local id,om = meshUp({am[#am]}, 'wall', groupEdit)
				if id then
					w.id = id
					-- TODO: cleanup aedit on building change
					aedit[id] = {mesh = am[#am], desc = w}
				else
					U.dump(cam, '!!ERR_houseUp_MESH:'..i..':'..j)
				end
--                groupEdit:add(om.obj)
			end
		end

-------------------------
-->> TOP
-------------------------
		if f.top ~= nil then
--          lo('??++++++++++++++============ if_EXT:'..i..tostring(f.topext),true)
--                lo('?? shape: '..tostring(f.top.shape),true) --..':'..tostring(#f.top.achild)..':'..tostring(f.top.cchild)) --..':'..tostring(desc.pos)..':'..tostring(f.pos)..':'..tostring(f.base[1]))
			-- ROOF
			f.top.ij = {i}
			if f.top.achild == nil then
				f.top.achild = {}
			end
					if not indrag then lo('?? for_cover: i='..i..' ifchild:'..#f.top.achild..':'..cheight..':'..tostring(forsplit)..':'..tostring(dirty)..':'..#f.top.achild) end
			for _,w in pairs(f.awall) do
				w.avplus = {}
			end
			if false and f.top.ridge.on then
--            if false and f.top.ridge.on then
--                    lo('?? hU_ridge:'..tostring(f.top.ridge.flat),true)
				local auvdim = {}
				for ic,c in pairs(f.top.achild) do
					if c.uvref then
						auvdim[ic] = {c.uvref,c.uvscale}
					end
				end
				local isvalid = T.forRidge(f,nil,nil,(f.top.istart-1)%2)
					lo('??^^^^^^^^^^^^^^ hU.RIDGE:'..i..':'..tostring(isvalid)..':'..#f.top.achild..' istart:'..f.top.istart,true)
--                    U.dump(f.top.achild, '?? chDATA:')
--                    U.dump(f.top.achild[1].ridge.data, '?? chDATA:')
				if not isvalid then
					f.top.ridge.on = false
				else
					for ic,c in pairs(f.top.achild) do
						if auvdim[ic] then
							c.uvref = auvdim[ic][1]
							c.uvscale = auvdim[ic][2]
						end
					end
				end
--                    U.dump(f.top, '??_________________if_RIDGE:')
--                    lo('?? hU_ridge2:'..tostring(f.top.ridge.flat),true)
			end
--                lo('?? for_TOP2:')
--                    lo('?? for_RIDGE1:')
--            for ic,c in pairs(f.top.achild) do
--            if true and #f.top.achild > 0 then
--[[
			local function uvTransform(auv, shift, scale)
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
]]

			local desctop = f.top --forTop()
			local subdata
      local asubdata = {}

			local function forFat(mdata, base, desctop)
--                    U.dump(mdata.verts, '?? forFat.mdata:'..#base..':'..desctop.shape)
--				if true then return end
--                    U.dump(base, '?? forFat.base:'..#base)
				local cbase = {}
				for j=1,#base do
					cbase[#cbase+1] = base2world(desc, {i,1}, base[j])
				end
--                    U.dump(cbase, '?? forFat.mbase:')
				cbase = U.polyStraighten(cbase)
				cbase =  U.polyMargin(cbase, desctop.margin)
--                    U.dump(cbase, '?? forFat.base:')

				-- lower edge index mapping
				local asi = {}
				if desctop.shape == 'ridge' then
--                        U.dump(mdata, '?? fat_RIDGE:',true)
					if #base == 4 then
								asi = {1,6,5,2}
					elseif #base == 6 then
						asi = {1,6,5,13,10,2}
					elseif #base == 8 then
						asi = {1,6,5,13,21,18,17,2}
					else
						return
					end
				elseif desctop and desctop.shape=='gable' and desctop.adata then
					if #desctop.adata == 0 then return end
--					mdata = {verts = desctop.adata[1].av}
--                        faces = desctop.adata[1].af,
--                        normals = desctop.adata[1].an,
--                        uvs = desctop.adata[1].auv,
--                        U.dump(mdata, '?? fF_gable:')
					asi = {1,2,3,6,5,4}
				elseif mdata then
					for j,q in pairs(cbase) do
						for k,p in pairs(mdata.verts) do
							if (U.proj2D(p) - U.proj2D(q)):length() < small_dist then
								asi[#asi+1] = k
							end
						end
					end
				else
					return
				end
				asi[#asi+1] = asi[1]

				-- lower lid
				subdata = deepcopy(mdata)
--                    U.dump(asi, '?? ASI1:'..#subdata.verts)
				-- upper edge
				local asilast = #subdata.verts + 1
				for j=#asi,2,-1 do
--				for j=#asi,1,-1 do
					subdata.verts[#subdata.verts+1] = subdata.verts[asi[j]] + vec3(0,0,desctop.fat)
					asi[#asi+1] = #subdata.verts
				end
				asi[#asi+1] = asilast
--                    U.dump(asi, '?? ASI2:'..#subdata.verts)
				-- lift original
				for i=1,#mdata.verts do
					 mdata.verts[i] = mdata.verts[i] + vec3(0,0,desctop.fat)
				end
--                    U.dump(subdata, '<< forFat:')
--                    U.dump(asi, '<< forFat_ASI:')
--					asi = nil
				return asi
			end
      ---------------
			-- add bottom
      ---------------
			local function polyPave(base, pos)
				local av,af,auv = {},{},{}
				local arc = coverUp(base)

				local u = (base[2]-base[1]):normalized()
				local v = vec3(-u.y, u.x, 0)
				local ref = base[1]
				for _,a in pairs(arc) do
					local ai = {}
					for k = 1,#a do
						ai[#ai+1] = #av+k
					end
					M.zip(ai,af) --,true)
					for _,b in pairs(a) do
						av[#av+1] = b
						auv[#auv+1] = {u=(b-ref):dot(u), v=(b-ref):dot(v)}
					end
				end
				for i=1,#av do
					av[i] = av[i] + pos
				end
				return av,af,auv
			end
--                lo('??***************** for_BOT:'..i..':'..tostring(f.pos)..':'..cheight)
			local hprn = prn and forHeight(prn.afloor, desc.floor-1) or 0
					if prn then lo('??^^^^^^^^^^ BOT_PRN: floor:'..desc.floor..':'..cij[1]..':'..hprn..':'..#prn.afloor..':'..tostring(cheight),true) end
--          U.dump(f.botext, '??__________ if_BE:'..i)
			local av,af,auv = polyPave(f.botext or f.base,
				(prn and prn.pos or vec3(0,0,0))+desc.pos+f.pos
				+vec3(0,0,(hprn + cheight + 0.01)))
--[[
			for i=1,#av do
				lo('??^^^^^^^^^^^^^^^ for_PRN:'..i..':'..tostring(desc.prn)..':'..tostring(prn and prn.pos or nil))
--                av[i] = av[i] +
			end
]]
			local an = {vec3(0,0,1)}
			local mdatabot = {
				verts = av,
				faces = af,
				normals = an,
				uvs = auv,
				material = out.defmat, --'m_greybox_base',
			}
			if true or U._PRD == 1 then
--!!                am[#am + 1] = mdatabot
--                lo('??^^^^^^^^^^^^^^^^^^^^ for_BOT:'..i)
			end
--                    U.dump(mdatabot.verts, '??^^^^^^^^^^^^^^^^^^^^ hU.paved_bot:'..#f.top.achild)

			local function toRender(dsc, adata, lbl, ischild)
--                    lo('>>******** toRender:'..lbl..':'..tostring(forsplit)..':'..tostring(dirty)..' id:'..tostring(dsc.id),true)
--                if forsplit or (ischild and dsc.id == nil) then
				if forsplit or ((ischild or dirty) and dsc.id == nil) then
--                        U.dump(adata,'?? r1:',true)
--                        lo('?? r1:',true)
--                if forsplit then-- or dsc.id == nil then
--                if forsplit then-- or dsc.id == nil then
--                        U.dump(adata,'?? mesh_NEW:'..lbl)
					-- create
					local id, om = meshUp(adata, lbl, groupEdit)
					if id then
	--                        lo('?? for_ID:'..tostring(id),true) --..tostring(id)..':'..tostring(adata[1])..':'..tostring(mdata),true)
						aedit[id] = {mesh = mdata or adata[1], desc = dsc} -- {ij = {i,nil}, type = 'cover'}}
	--                    aedit[id] = {mesh = mdata, desc = f.top} -- {ij = {i,nil}, type = 'cover'}}
						dsc.id = id
					end
				elseif dirty then
--                        lo('?? toRender.mesh_UPD:',true)
					-- update
					local obj = scenetree.findObjectById(dsc.id)
--                        lo('?? r2:',true)
--                        if dsc.ridge.on then
--                            U.dump(adata,'?? r2_data:'..tostring(dsc.id)..':'..tostring(dsc.shape)..':'..tostring(obj)..':'..tostring(dsc.ridge),true)
--                        end
					if obj ~= nil then
						if M.valid(adata) then
							obj:createMesh({})
							obj:createMesh({adata})
						else
							lo('!! ERR_MESH:',true)
						end
					else
						lo('!! ERR.houseUp_NOTOPCHILD:'..tostring(dsc.id),true)
					end
				else
--                        lo('?? r3:',true)
--                        lo('?? mesh_SOLID:'..tostring(dsc.id),true)
					-- to solid building
					for _,d in pairs(adata) do
						am[#am + 1] = d
					end
				end
			end
--[[
			desctop = desctop or forTop()
--                U.dump(desctop, '?? hU_floor:'..i)
			if U._PRD == 0 and desctop and desctop.shape == 'gable' then
					lo('?? hU.if_child:'..tostring(f.top.cchild))
				roofUp('gable', desc, i, f.top.cchild)
					lo('?? hU_gable:'..tostring(desctop.adata)..':'..tostring(subdata),true)
--                    U.dump(desctop.adata, '?? hU_gable:'..tostring(desctop.adata)..':'..tostring(subdata),true)
				local am = {}
				for _,d in pairs(desctop.adata) do
					am[#am+1] = {
						verts = d.av,
						faces = d.af,
						normals = d.an,
						uvs = d.auv,
						material = desctop.mat,
					}
				end
				if subdata then
					am[#am+1] = subdata
				end
				toRender(desctop, am, 'lid')
			end
]]
--                lo('?? for_TOP3:'..tostring(desctop.adata))

			if #f.top.achild > 0 then
--            if #f.top.achild > 0 and not desctop.adata then
--??            if not ({gable=1,shed=1})[f.top.shape] and #f.top.achild > 0 then
					lo('?? with_CHILD:'..i..':'..#f.top.achild..':'..tostring(f.top.ridge.on)..':'..tostring(dirty), true) --..tostring(forsplit)..':'..tostring(dirty)..':'..tostring(f.top.id))
        local asi
				if f.top.id ~= nil then
					local otop = scenetree.findObjectById(f.top.id)
					if otop then
						otop:delete()
					else
						lo('!! ERR_hU.NOTOP:'..f.top.id,true)
					end
					f.top.id = nil
				end
				for ic,c in pairs(f.top.achild) do
							lo('?? for_CHILD:'..ic..':'..tostring(c.shape)..':'..tostring(c.fat))
--                    U.dump(c, '?? for_CHILD:'..ic)
					c = f.top.achild[ic]
--                        lo('?? if_RIDGE:'..ic..':'..tostring(c.ridge and c.ridge.on or nil))
					if c.ridge and c.ridge.on then
						if #f.top.achild == 1 and not c.base then
							c.base = deepcopy(f.base)
						end
						if not c.fat then
							c.fat = f.top.fat or default.topthick
						end
							lo('??+++++++++++++++++++++++ for_child_ridge:'..ic..':'..tostring(c.shape)..' fat:'..tostring(c.fat),true)
--                        U.dump(f.top.achild[1].ridge.data,'??+++++++++++++++++++++++ for_child_ridge:'..ic)
--                        T.forRidge(f, c.base, ic)
					end
--                for ic = 1,1 do
--                            local c = f.top.achild[ic]
--                            U.dump(c.base, '?? pre_rup:'..i..':'..ic..':'..tostring(c.id)..':'..tostring(forsplit)..':'..tostring(dirty)..':'..tostring(c.shape), true)
--                        lo('??^^^^^^^^^^ hU_pre_rU:'..c.shape..':'..c.istart,true)
					local amdata = roofUp(c.shape, desc, i, ic, prn)
          mdata = amdata and amdata[1] or nil
--                        U.dump(mdata, '?? hU_post_rU:'..tostring(mdata)..':'..tostring(c.shape),true)
--                        if c.shape == 'gable' then
--                            U.dump(c,'?? for_TOP_child:'..ic..':'..tostring(c.shape)..':'..tostring(c.adata and #c.adata or nil),true)
--                        end
--                    desctop = c
					if c.fat and c.fat>0 then
--                        if c.shape ~= 'gable' and c.fat and c.fat>0 then
--                        if i == #desc.afloor and c.shape ~= 'gable' and c.fat and c.fat>0 then
--                        if i == #desc.afloor and desctop.shape ~= 'gable' and desctop.fat and desctop.fat>0 then
						-- for last floor thinkness
            local amd = {mdata}
            if c.shape == 'gable' then
              amd = {}
              for k=1,#c.adata,2 do
--                amd[#amd+1] = {verts = c.adata[k].av}
              end
            end
            for k,d in pairs(amd) do
              asi = forFat(d, c.base, c)
              if asi then
                subdata.normals, subdata.uvs, subdata.faces = M.zip2(subdata.verts, asi, subdata.normals, subdata.uvs, subdata.faces)--, dbg)
                subdata.material = out.defmat --'m_greybox_base' -- 'W arningMaterial'
              end
              if c.shape == 'gable' then
                asubdata[#asubdata+1] = subdata
              end
            end
--[[
            if c.shape == 'gable' then
              --TODO: refactor same as for nochild
							asi = forFat({verts=c.adata[1].av}, c.base, c)
            else
							asi = forFat(mdata, c.base, c)
            end
            if c.shape == 'gable' then
              asubdata[#asubdata+1] = subdata
            end

						if c.shape ~= 'gable' or #c.base == 4 then
--                        if (c.shape ~= 'gable' and c.shape ~= 'ridge') or (c.shape == 'gable' and #c.base == 4) then
--                        if (c.shape ~= 'gable' and c.shape ~= 'ridge' and not c.ridge) or (c.shape == 'gable' and #c.base == 4) then
--                                lo('?? preFat:'..#c.base,true)
							asi = forFat({verts=c.adata[1].av}, c.base, c)
							asi = forFat(mdata, c.base, c)
--                                U.dump(asi,'??++++++ post_FAT_asi:')
	--                                    U.dump(asi, '??^^^^^^^^^^^^^^ SDV:'..ic..':'..#c.base..':'..#subdata.verts)
						end
            if asi then
              subdata.normals, subdata.uvs, subdata.faces = M.zip2(subdata.verts, asi, subdata.normals, subdata.uvs, subdata.faces)
              subdata.material = out.defmat --'m_greybox_base' -- 'WarningMaterial'
            end
]]
--                                U.dump(subdata, '??^^^^^^^^^^^^^ SUBDATA:'..tostring(c.id)..':'..tostring(forsplit))
					end

					if c.shape == 'gable' then
--                    if U._PRD == 0 and c.shape == 'gable' then
--                            U.dump(desctop.adata, '?? for_child_Gable:'..#desctop.adata..':'..c.shape..':'..tostring(desctop.id..':'..tostring(dirty)))
--                            desctop.id = nil
            local desctop = c
            local am = {}
            for _,d in pairs(desctop.adata) do
              if desctop.uvref then
                M.uvTransform(d.auv,desctop.uvref,desctop.uvscale)
              end
              am[#am+1] = {
                verts = d.av,
                faces = d.af,
                normals = d.an,
                uvs = d.auv,
                material = desctop.mat,
              }
            end
            for k,sd in pairs(asubdata) do
              am[#am+1] = sd
              am[#am+1] = {
                verts = subdata.verts,
                faces = desctop.adata[k].af,
                normals = desctop.adata[k].an,
                uvs = desctop.adata[k].auv,
                material = out.defmat,
              }
            end

--[[
						local am = {}
						for _,d in pairs(c.adata) do
							am[#am+1] = {
								verts = d.av,
								faces = d.af,
								normals = d.an,
								uvs = d.auv,
								material = c.mat,
							}
						end
--                            U.dump(subdata,'?? gbl_subdata:'..tostring(subdata))
--                            W.out.aedge = {e = subdata.verts}
--                        if subdata and #c.base == 4 then
							lo('??^^^^^^^^^^^ if_subdata:'..ic..':'..tostring(subdata))
						if subdata then
							am[#am+1] = subdata
							-- lower lid plates
							for _,d in pairs(c.adata) do
								if c.uvref or c.uvscale then
									M.uvTransform(d.auv,c.uvref,c.uvscale)
								end
								am[#am+1] = {
									verts = subdata.verts,
									faces = d.af,
									normals = d.an,
									uvs = d.auv,
									material = out.defmat,
								}
							end
						end
]]
						toRender(c, am, 'lid', true)
--                        U.dump(c, '??_______________ for_CHILD:'..ic..':'..tostring(c.shape)..':'..tostring(c.fat)..':'..tostring(mdata))
					elseif mdata then
						if c.uvref then
							M.uvTransform(mdata.uvs,c.uvref,c.uvscale)
						end
--                        if c.ridge and c.ridge.on then
--                            mdata = c.ridge.adata
--                        end
            local data = amdata
            if subdata then data[#data+1] = subdata end
--						local data = (subdata and {mdata,subdata}) or {mdata}
--                        local data = (subdata and c.shape ~= 'ridge' and {mdata,subdata}) or {mdata}
--                        local data = subdata and {mdata,subdata} or {mdata}
--                            lo('?? preRender:'..ic..':'..c.shape..':'..tostring(subdata and tableSize(subdata) or nil)..':'..tostring(data and tableSize(data) or nil))
--                            U.dump(data, '?? preRender_d:'..ic..':'..tostring(data and tableSize(data) or nil))
						toRender(f.top.achild[ic], data, 'lid', true)
--                        toRender(c, data, 'lid', true)
--                            U.dump(c, '?? post_render0:'..ic..':'..tostring(c.id),true)
--                            U.dump(f.top.achild, '?? post_render:'..ic..':'..tostring(c.id),true)

						if c.id then
	--                            lo('?? for_child:'..tostring(c.id), true)
							if not aedit[c.id] then
								aedit[c.id] = {}
							end
							aedit[c.id].mesh = mdata
						end

					end
--                            lo('?? post_rup:'..tostring(c.id)..':'..tostring(forsplit)..':'..tostring(dirty))
				end
				-- render floor
--!!                toRender(desctop, {mdatabot}, 'lid')
--            end
--            if #f.top.achild == 0 then
--                                    U.dump(f.top.achild, '?? post_render101:'..i)

			else
				--            local tp = f.top.cchild ~= nil and f.top.achild[f.top.cchild].shape or f.top.shape -- i < #desc.afloor and 'flat' or md_roof
--                        lo('??********* hU.for_roof:'..i..':'..tostring(f.top.shape)..':'..tostring(forsplit)..' id:'..tostring(f.top.id), true)
--                if f.top.ridge and f.top.ridge.on then
--                    local isvalid = T.forRidge(f,nil,nil,f.top.istart-1)
--                end
				if not f.top.shape then f.top.shape = 'flat' end
--                        U.dump(f.base, '?? hU_base:'..i)
				local amdata = roofUp(f.top.shape, desc, i, nil, prn)
--            U.dump(amdata,'?? hU.post_rU:'..#amdata)
        mdata = amdata and amdata[1] or nil

--                    lo('?? post_RA:'..i..':'..tostring(desctop.fat))
				if i == #desc.afloor and desctop.fat and desctop.fat>0 then
--                if true and desctop.shape ~= 'gable' and i == #desc.afloor and desctop.fat and desctop.fat>0 then
--                            U.dump(mdata, '?? to_FAT:')
					local asi
--                            lo('?? for_GABLE:'..tostring(#f.top.adata))
          local amd = {mdata}
          if f.top.shape == 'gable' then
--                U.dump(f.top.adata, '?? adata:')
            amd = {}
            for k=1,#f.top.adata,2 do
              amd[#amd+1] = {verts = f.top.adata[k].av}
              if k==3 then
              end
            end
          end
          for k,d in pairs(amd) do
            asi = forFat(d, f.base, f.top)
            if asi then
              subdata.normals, subdata.uvs, subdata.faces = M.zip2(subdata.verts, asi, subdata.normals, subdata.uvs, subdata.faces)--, dbg)
              subdata.material = out.defmat --'m_greybox_base' -- 'W arningMaterial'
            end
            if f.top.shape == 'gable' then
              asubdata[#asubdata+1] = subdata
            end
          end
				end
--[[
          if f.top.shape == 'gable' then
--            for k,d in pairs(desctop.)
						asi = forFat(mdata, f.base, f.top)
          else
						asi = forFat(mdata, f.base, f.top)
            asubdata[#asubdata+1] = subdata
          end
]]
--[[
					if f.top.shape ~= 'gable' or #f.top.adata == 2 or U._PRD==0 then
--                    if f.top.shape ~= 'gable' or #f.base == 4 then
--                    if f.top.shape ~= 'gable' or (#f.base == 4 or U._PRD == 0) then
--                            U.dump(f.base, '?? pre_fat:'..#f.base)
						asi = forFat(mdata, f.base, f.top)
            asubdata[#asubdata+1] = subdata
--              U.dump(asi, '?? hU.gable_asi:')
--              U.dump(subdata, '?? hU.gable_mdata:')
					end
					if asi then
						subdata.normals, subdata.uvs, subdata.faces = M.zip2(subdata.verts, asi, subdata.normals, subdata.uvs, subdata.faces)--, dbg)
						subdata.material = out.defmat --'m_greybox_base' -- 'W arningMaterial'
					end
]]
--                    local asi = forFat(mdata, f.base, f.top)
--                            U.dump(asi, '?? to_FAT_asi:'..#f.base..':'..#subdata.verts)
--                            U.dump(subdata.verts, '?? to_FAT_sdverts:'..#f.base..':'..#subdata.verts)
--                            if #asi == 14 then
--                                _dbdrag = true
--                            end

--[[
					subdata.verts = deepcopy(base)
					for l=1,#subdata.verts do
						subdata.verts[l] = subdata.verts[l] + desc.pos + f.pos + vec3(0,0,5)
					end
]]
--[[
							if true then return end

							U.dump(subdata.verts, '?? subdata:'..#subdata.verts)
						for k=1,#bsi do
							asi[#asi+1] = #subdata.verts-#bsi+k
						end
						asi[#asi+1] = #subdata.verts-#bsi+1
							for _,s in pairs(asi) do
								lo('?? for_V:'..s..':'..tostring(subdata.verts[s]))
							end
]]
--                        U.dump(subdata.verts, '??_____________________ for_verts2:')

--                        U.dump(subdata.verts, '??++++++++++++++++++++++++ for_ASI:'..#subdata.verts)
--                        U.dump(asi, '??++++++++++++++++++++++++ for_ASI:'..#subdata.verts)
--                        U.dump(subdata.faces, '??++++++++++++++++++++++++ for_AF:'..#subdata.faces)
--                    M.zip2(subdata.verts, asi)
--                        U.dump(subdata.verts, '?? pre_ZIP:'..#subdata.verts)
--                            lo('?? pre_ZIP:'..#subdata.verts..':'..#desctop.body) --tostring(desctop.body[1][1]))
--                        lo('?? post_ZIP:')
--                        U.dump(subdata.normals, '??<<<<<<<<<<<<<<<<<<<<<<<< for_AN:'..#subdata.normals)
--                        U.dump(subdata.faces, '??<<<<<<<<<<<<<<<<<<<<<<<< for_AF:'..#subdata.faces)
--                    lo('?? PPP:'..tostring(mdata))
--                        U.dump(mdata, '?? HU_data:')
--                        lo('?? for_DT:'..tostring(desctop)..':'..desctop.shape)
--                local desctop = f.top
				if desctop.shape == 'gable' then
--                if U._PRD == 0 and desctop.shape == 'gable' then
					local am = {}
					for _,d in pairs(desctop.adata) do
						if desctop.uvref then
							M.uvTransform(d.auv,desctop.uvref,desctop.uvscale)
						end
						am[#am+1] = {
							verts = d.av,
							faces = d.af,
							normals = d.an,
							uvs = d.auv,
							material = desctop.mat,
						}
					end
          for k,sd in pairs(asubdata) do
						am[#am+1] = sd
            am[#am+1] = {
              verts = subdata.verts,
              faces = desctop.adata[k].af,
              normals = desctop.adata[k].an,
              uvs = desctop.adata[k].auv,
              material = out.defmat,
            }
          end
--[[
					if subdata then
						am[#am+1] = subdata
						-- lower lid plates
						for _,d in pairs(desctop.adata) do
							am[#am+1] = {
								verts = subdata.verts,
								faces = d.af,
								normals = d.an,
								uvs = d.auv,
								material = out.defmat,
							}
						end
					end
]]
--                        U.dump(am, '?? preR:'..#am,true)
					toRender(desctop, am, 'lid')
--                        lo('?? RDd:')
				elseif mdata ~= nil and #mdata.verts > 0 then
--						lo('?? no_child:'..tostring(forsplit)..':'..tostring(dirty)..':'..tostring(f.top.id),true)
					if desctop.uvref then
						M.uvTransform(mdata.uvs,desctop.uvref,desctop.uvscale)
					end
					local data = amdata
          if subdata then data[#data+1] = subdata end
          --and {mdata,subdata} or {mdata}
					if #mdatabot.verts > 0 then
						data[#data+1] = mdatabot
					end

					if true then
--							lo('??^^^^^^^^^^^^ to_LID:'..i..':'..tostring(#data),true)
--                            U.dump(data,'??^^^^^^^^^^^^ to_LID:'..i..':'..tostring(#data),true)
						toRender(desctop, data, 'lid')
					else
						if forsplit then
	--                            lo('?? TOP1:'..tostring(subdata))
	--                        local id, om = meshUp({mdata,subdata}, 'lid', groupEdit)
	--                                lo(''..i,true)
							local id,om = meshUp(data, 'lid', groupEdit)
							aedit[id] = {mesh = mdata, desc = desctop} -- {ij = {i,nil}, type = 'cover'}}
							desctop.id = id
	--                            lo('??+++++ top_id:'..id)
						elseif dirty then
	--                            lo('?? TOP2:'..tostring(subdata))
	--                                lo('?? for_dirty:'..tostring(f.top.id))
							local top = scenetree.findObjectById(desctop.id)
						--                        if not indrag then lo('?? roof_update:'..i..':'..tostring(top)..':'..f.top.mat) end
							if top ~= nil then
								top:createMesh({})
								top:createMesh({data})
	--                            top:createMesh({subdata and {mdata,subdata} or {mdata}})
							else
								desctop.id = meshUp({mdata}, 'lid', groupEdit)
	--                            lo('!! ERR.houseUp_NOTOP:'..tostring(f.top.id))
							end
						else --if U._PRD == 1 then
								lo('??^^^^^^^^^^^^^^^^_________^^^^^^^^^^^^^^^^ TOP3:'..tostring(subdata),true)
	--                        U.dump(mdata, '??______________ for_FLOOR_pre:'..i)
							am[#am + 1] = mdata
							if subdata then
								am[#am + 1] = subdata
							end
						end
						if f.top.id and aedit[f.top.id] then
							aedit[desctop.id].mesh = mdata
						end
					end
				end
--            else
--                lo('??+++++ nil_id:')
--                f.top.id = nil
--                    U.dump(mdata, '??______________ for_FLOOR_pre:'..i)
			end
--                                    U.dump(f.top.achild, '?? post_render10:'..i)


--                U.dump(desc.afloor[2].achild, '?? post_render111:')

			if f.top.border and f.top.border.id then
				local border = scenetree.findObjectById(f.top.border.id)
--                    lo('??+++++++++++++++++++++ if_BORDER:'..tostring(border)..':'..tostring(f.top.border.yes))
				if border then
					border:createMesh({})
				end
			end
			if f.top.shape == 'flat' and f.top.border and f.top.border.yes then
					lo('??^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ hU_border:'..i)
--                    U.dump(f.top.border,'??____________ for_BORDER:'..tostring(#f.top.border.shape)..':'..tostring(f.top.shape), true)

				local bam = {}
				local avert = {}

				local function toBAM(dir, inv)
--                    if dbg then lo('>> toBAM:'..dir) end
					local iseq = {}
					for i = 1,#f.top.border.shape do
						if dir == 1 then
							iseq[#iseq+1] = #avert - 2*#f.top.border.shape + i
						else
							iseq[#iseq+1] = #avert - i - #f.top.border.shape + 1
						end
					end
					for i = 1,#f.top.border.shape do
						if dir == 1 then
							if inv then
								iseq[#iseq+1] = #avert - i + 1
							else
								iseq[#iseq+1] = #avert - #f.top.border.shape + i
							end
--                            iseq[#iseq+1] = #avert - #f.top.border.shape + i
						else
							iseq[#iseq+1] = #avert - i + 1
						end
					end
--                            lo('??^^^^^^^^^^^^^^^^^^^___________________ toBAM:')
					f.top.border.uvscale = f.top.border.uvscale or {1,1}
					local anb,auvb,afb = M.zip2(avert,iseq) --,nil,nil,nil,f.top.border.uvscale)
--                        if dbg then
--                            U.dump(afb, '?? toBAM.AFACE:'..#afb)
--                            U.dump(avert, '?? toBAM.AVERT:')
--                            U.dump(iseq, '?? iseq:')
--                            U.dump(auvb, '?? toBAM.auvb:')
--                        end
					for _,uv in pairs(auvb) do
						uv.u = uv.u*f.top.border.uvscale[1]
						uv.v = uv.v*f.top.border.uvscale[2]
					end
					bam[#bam+1] = {
						verts = avert,
						faces = afb,
						normals = anb,
						uvs = auvb,
						material = f.awall[1].mat, -- f.top.border.mat, --'WarningMaterial', -- w.mat,
--                        material = U._PRD == 1 and f.awall[1].mat or 'WarningMaterial', -- f.top.border.mat, --'WarningMaterial', -- w.mat,
					}
				end


				local base = U.polyMargin(f.base,f.top.margin)
				base = U.polyStraighten(base)

				if #f.top.border.shape == 0 then
					f.top.border.shape = {vec3(0, 0), vec3(1, 0), vec3(1, 1), vec3(0, 1), vec3(0, 0)} --, vec3(0, 1.5)}
--                    f.top.border.shape = {vec3(0, 0), vec3(1, 0), vec3(0.8, 0.5), vec3(1, 1), vec3(0, 1)} --, vec3(0, 1.5)}
				end
--                local av,af,auv = {},{},{}
				local anb,auvb,afb
				local iseq
				local cp = vec3(1,0)
				local upre = f.awall[#base].u
        local dp = vec3(0,0,0)
        if desc.prn then
          dp = prn.pos + vec3(0, 0, forHeight(prn.afloor, desc.floor-1))
        end
				for k,n in pairs(base) do
--                        if k > 4 then break end
					local u = f.awall[k].u:normalized()
					local w = vec3(u.y,-u.x)
					local ifr,ito,step = 1,#f.top.border.shape,1
					if k%2 == 0 then
						ifr,ito,step = #f.top.border.shape,1,-1
					end
					for j=ifr,ito,step do
						local c = f.top.border.shape[j]
--                        local ang = U.vang(upre,u,true)/2
--                            lo('?? for_base:'..i..':'..k)
--                            lo('??++++ ang:'..ang..' w:'..tostring(U.vturn(w,-U.vang(upre,u)/2)),true)
						avert[#avert+1] = base[k] + desc.pos + f.pos + vec3(0,0,forHeight(desc.afloor, f.ij[1]))
						--base2world(desc, {i,k}) --base[k]
						+ (f.top.border.height*(c.y-0.5)+f.top.border.dz)*vec3(0,0,1)
						+ (f.top.border.width*(c.x-0.5)+f.top.border.dy)*U.vturn(w,-U.vang(upre,u,true)/2)+dp
					end
					if k>1 then
						toBAM(k%2+1)
--                        anb,auvb,afb = M.zip2(avert,iseq)
--                        toBAM(k%2==0 and 1 or -1)
					end
					upre = u
				end
				-- last face
				local k = 1
				local u = f.awall[k].u:normalized()
				local w = vec3(u.y,-u.x)
				local ifr,ito,step = 1,#f.top.border.shape,1
				if k%2 == 0 then
					ifr,ito,step = #f.top.border.shape,1,-1
				end
				for i=ifr,ito,step do
					local c = f.top.border.shape[i]
					local ang = U.vang(upre,u)/2
--                            lo('??++++ ang:'..ang..' w:'..tostring(U.vturn(w,-U.vang(upre,u)/2)),true)
					avert[#avert+1] = base[k] + desc.pos + f.pos + vec3(0,0,forHeight(desc.afloor, f.ij[1]))
					+ (f.top.border.height*(c.y-0.5)+f.top.border.dz)*vec3(0,0,1)
					+ (f.top.border.width*(c.x-0.5)+f.top.border.dy)*U.vturn(w,-U.vang(upre,u)/2)+dp
--                    + f.top.border.height*(c.y-0.5)*vec3(0,0,1)
--                    + f.top.border.width*(c.x-0.5)*U.vturn(w,-U.vang(upre,u)/2)
				end
				-- last segment
--                    lo('??************** lastBam:'..#base)
				toBAM(#base%2, true)

				toRender(f.top.border, bam, 'topplus')
--[[
				if true then
				else

					if not f.top.border.id then
	--                        U.dump(bam[1], '?? pre_BORDER:'..#bam)
	--                        lo('??++++++++++++ for_BAM:'..#bam..':'..tostring(forsplit)..':'..tostring(dirty))
						f.top.border.id = meshUp(bam, 'topplus', groupEdit)
					else
						local border = scenetree.findObjectById(f.top.border.id)
	--                        lo('??+++++++++++++++++++++ if_BORDER:'..tostring(border)..':'..tostring(f.top.border.yes))
						if border then
	--                        border:createMesh({})
							if f.top.border.yes then
								border:createMesh({bam})
							end
						elseif f.top.border.yes then
							f.top.border.id = meshUp(bam, 'topplus', groupEdit)
						end
					end
	--                f.top.border.data = bam
					aedit[f.top.border.id] = {mesh = bam, desc = f.top.border}

				end
]]
--                    U.dump(av, '?? for_AV:'..#av)
--                    U.dump(af, '?? for_AF:'..#af..':'..#am)
			end
			f.top.ij = {i}
--                        U.dump(f.top.achild, '?? post_render11:'..i)
		else
			lo('!! ERR_no_cover:')
		end
-------------------------
--<< TOP
-------------------------
		-- for fringe
		local fbase = f.fringe and U.polyMargin(f.base, f.fringe.inout) or f.base

--            U.dump(f.top.achild, '?? post_render22:'..i)

		if f.awplus then
-----------------------------
-- SUBROOF SEGMENTS
-----------------------------
--                U.dump(f.awplus,'??+++++++++++++++++ av_PLUS:'..i..':'..#f.awplus)
--                U.dump(f.awplus, '?? av_PLUS:'..i..':'..forHeight(desc.afloor, f.ij[1])..':'..f.h)
			local uvv
--				U.dump(f.awplus,'?? hU_prePLUS:'..tableSize(f.awplus))
			for k,wp in pairs(f.awplus) do
				if wp.list then
--                        U.dump(wp.list, '?? for_AWP:'..i..':'..tostring(dirty))
					for _,list in pairs(wp.list) do
--							U.dump(list,'??++++++++++++++++ for_WP:')
						for key,p in pairs(list) do
--                                lo('?? if_key:'..tostring(key)..':'..tostring(tonumber(key)))
							if tonumber(key) then
								list[tonumber(key)] = p
--                                list[key] = nil
							end
						end
						uvv = -forHeight(desc.afloor, f.ij[1])*(f.awall[list.ind].uvscale and f.awall[list.ind].uvscale[2] or 1)
--                            U.dump(list, '?? for_list:'.._..':'..tostring(list[1])..':'..tostring(uvv))

						local uvref = f.awall[list.ind].uvref or {0,0}
						local av,auv,af,an = T.pave(list, f.awall[list.ind].uvscale
							, {{u=uvref[1],v=uvv-uvref[2]}, {u=(list[2]-list[1]):length()+uvref[1],v=uvv-uvref[2]}}
						)
						if desc.prn then
							local pp = W.atParent(desc)
							for t=1,#av do
								av[t] = av[t] + pp
							end
						end
						am[#am + 1] = {
							verts = av,
							faces = af,
							normals = an,
							uvs = auv,
							material = f.awall[list.ind].mat,
	--                        material = 'WarningMaterial',
						}
					end
					local cam = {}
					for k=0,#wp.list-1 do
						cam[#cam+1] = am[#am - k]
					end
					if dirty then
--                            lo('?? wp_ID:'..i..':'..tostring(wp.id),true)
						local wallplus = scenetree.findObjectById(wp.id)
		--                    lo('?? for_DIRTY:'..tostring(wallplus)..':'..tostring(f.awplus.id),true)
						if wallplus then
							wallplus:createMesh({})
							wallplus:createMesh({cam})
						else
							wp.id = meshUp(cam, 'wallplus', groupEdit)
						end
					elseif forsplit then
						wp.id = meshUp(cam, 'wallplus', groupEdit)
					end
				end
			end
		end
		cheight = cheight + f.h
	end

	local cforest = forestClean(desc)
--        lo('??_________________ corner_CLEANED:')

	-- add corners
	local function toCorner(list, ifr, ito, mlen)
--            ang = math.pi/4
		for i=ifr,ito do
			local scale = (desc.afloor[list[i][1]].h-0.001)/mlen
--				U.dump(list, '?? for_ac_list:'..i)
			local u = desc.afloor[list[i][1]].awall[list[i][2]].u
			local ang = U.vang(u, vec3(1,0,0), true)+math.pi/2
--                U.dump(list[i],'?? toCorner.in_STEM:'..i..tostring(base2world(desc, list[i]))..':'..list[i].dae)
--                lo('??+++++ toC:'..i)
--                lo('?? toCorner:'..tostring(base2world(desc, list[i]))..':'..ang)
			to(desc, list[i].dae,
				base2world(desc, list[i]),
				vec3(0,0,ang), cforest, vec3(1,1,scale))
		end
	end
--[[
	local function stem2corner(dae, ij, tot, mlen)
--            U.dump(ij, '>> stem2corner:'..tot..':'..tostring(dae))
--        local nmesh = round(tot/mlen)
--        local scale = tot/(mlen*nmesh)
		local scale = (tot-0.001)/mlen
--            lo('?? pre_TO: scale:'..tostring(scale)..' mlen:'..mlen) --..' nmesh:'..nmesh..':'..tostring(base2world(desc, ij)))
		to(desc, dae,
			base2world(desc, ij),
			vec3(0,0,0), cforest, vec3(1,1,scale))
	end
]]
------------------------
-- CORNERS
------------------------
	if desc.acorner_ then
--                U.dump(desc.acorner_, '??>>>>>> for_CORNER:'..tableSize(desc.acorner_))
		for i,s in pairs(desc.acorner_) do
--            if i > 1 then break end
			if s and #s.list>0 then
--          U.dump(s.list[1], '?? for_AC:'..i..':'..tostring(s.list)..':'..s.list[1]['1'])
				-- get stem
--                    U.dump(s.list[1],'??^^^^^^^^^^^^^^^^^^^^^^^^^^^^ for_corner:'..tostring(s.list[1]['1']),true)
				local tot,ci,istart,scale = desc.afloor[s.list[1][1]].h,2,1
				local mlen = ddae[s.list[istart].dae].to.z-ddae[s.list[istart].dae].fr.z
	--                lo('??--- fro_stem0:'..i..':'..ci..':'..#s.list,true)
				while ci <= #s.list do
	--                    U.dump(s.list, '??+++ while_STEM:'..i..':'..ci,true)
--                        lo('?? in_STEM:'..i..':'..ci..':'..tostring(s.list[ci][1] - s.list[ci-1][1]))
					if s.list[ci][1] - s.list[ci-1][1] == 1 and (not s.list[ci].dae or s.list[ci].dae == s.list[ci-1].dae) then
						tot = tot + desc.afloor[s.list[ci][1]].h
					else
						-- end stem
						mlen = ddae[s.list[istart].dae].to.z-ddae[s.list[istart].dae].fr.z
						toCorner(s.list, istart, ci-1, mlen)

--                        toCorner(s.list[istart].dae, s.list[istart], tot, mlen)
--                        stem2corner(s.list[istart].dae, s.list[istart], s.list[ci])
	--[[
						local nmesh = round(tot/mlen)
						scale = tot/(mlen*nmesh)
							lo('?? pre_TO:'..tostring(scale))
						to(desc, s.list[istart].dae,
							base2world(desc,s.list[ci]),
							vec3(0,0,0), nil, vec3(1,1,scale))
	]]
	--                    stem2corner(desc, s.list[istart].dae, nmesh scale)
						-- next stem
						if not s.list[ci].dae then
							s.list[ci].dae = s.list[istart].dae
						end
						istart = ci
						tot = desc.afloor[s.list[istart][1]].h
						mlen = ddae[s.list[istart].dae].to.z-ddae[s.list[istart].dae].fr.z
					end
					ci = ci + 1
				end
				toCorner(s.list, istart, #s.list, mlen)
--[[
				if i == 2 then
						lo('??**************** for_STEM:'..i..':'..istart..':'..#s.list)
					toCorner(s.list, istart, #s.list, mlen)
				end
]]
	--            tot = tot + desc.afloor[s.list[ci-1][1]].h
--                    U.dump(s.list[istart], '?? for_STEM:'..i..':'..ci..'/'..istart..':'..tostring(tot),true)
--                stem2corner(s.list[istart].dae, s.list[istart], tot, mlen)
			end
--                    U.dump(desc.df, '??<<<<<< for_CORNER:'..tableSize(desc.acorner_))
--                    if i > 1 then break end
--                if ci > 3 then break end
		end
--[[
		for i,f in pairs(desc.acorner) do
			for j,c in pairs(f) do
--                    U.dump(c, '?? for_dae:'..tostring(ddae[c.dae])..':'..tostring(#c))
				if #c > 0 and c.dae then
					local L = ddae[c.dae].to.z - ddae[c.dae].fr.z
					local u = desc.afloor[i].awall[j].u
					local ang = math.atan2(u.y, u.x)
					local h = forHeight(desc.afloor, c[#c][1]) - forHeight(desc.afloor, c[1][1]-1)
					local nmesh = h/L % 1 < 0.5 and math.floor(h/L) or math.ceil(h/L)
					if nmesh == 0 then nmesh = 1 end
					local scale = h/(nmesh*L)
--                            U.dump(c, '?? to_corner:'..j..':'..h..'/'..L..' nmesh:'..nmesh..':'..scale..' ang:'..ang)
	--                for k,ij in ipairs(c) do
	--                local n = scale < 1 and 1 or nmesh
					for k = 1,nmesh do
	--                for k = 1,n do
						to(desc, c.dae,
							base2world(desc,{i,j}) + vec3(0,0,(k-1)*L*scale),
							vec3(0,0,-ang), nil, vec3(1,1,scale))
					end
				end
			end
		end
]]
	end
	-- add roof borders
--    roofBorderUp(desc)

--        lo('?? HU:'..tostring(forsplit)..':'..tostring(inedit)..':'..#out.avedit..':'..tostring(desctop.roofborder))
--        lo('?? split_edit:'..tostring(forsplit)..':'..tostring(inedit)..':'..tostring(dirty))
--        lo('?? split_edit:'..tostring(forsplit)..':'..tostring(dirty))
--------------------------
-- RENDER WHOLE
--------------------------
	if not forsplit and not dirty then
--    if not forsplit and not inedit then
		local id
		if toedit ~= nil then
					lo('??---------- houseUp.EDIT_QUIT:'..tostring(out.lock)..':'..#am)
------------------------
-- UPDATE and QUIT EDIT
			scenetree.findObject('edit'):deleteAllObjects()
			id = toedit
			local obj = scenetree.findObjectById(id)
--                    if not indrag then lo('?? to_update:'..id..':'..#out.avedit) end
--            obj:createMesh({{{}}})
			if obj then
				if M.valid(am) then
					obj:createMesh({am})
				else
					lo('!! ERR_MESH_QE:', true)
--                    U.dump(am, '!! mesh:'..#am)
				end
			end
--            indrag = false
--            cuv = nil
			if not out.lock then out.avedit = {} end
		elseif desc.id then
					lo('??---------- to_UPDATE:'..desc.id, true)
			id = desc.id
			local obj = scenetree.findObjectById(id)
			if obj then
				obj:createMesh({})
				obj:createMesh({am})
			end
		else
------------------------
-- CREATE BUILDING
					lo('??----------- houseUp.NEW:'..#am)
--                    U.dump(am, '??----------- houseUp.NEW:')
			local newid, om
			if U._PRD == 0 then
		--        newid,om = meshUp(am,'building')
		--        local ams = am[2]
		--        table.remove(am,2)
		--        table.insert(am,1,ams)
		--[[
				local cam = {}
				for _,m in pairs(am) do
				table.insert(cam,1,deepcopy(m))
		--          cam[#cam+1] = deepcopy(m)
				end
		]]
		--        newid,om = meshUp(cam,'building')
				newid,om = meshUp(am,'building')
		--        newid,om = meshUp({am[5],am[4]},'building')
		--        newid,om = meshUp({am[4],am[3],am[2],am[1]},'building')
			else
				newid,om = meshUp(am,'building')
			end
--			local newid, om = meshUp(am, 'building')
			if newid then
				scenetree.MissionGroup:add(om.obj)
				id = newid
				desc.id = id
				desc.data = am
			else
				lo('!! ERR_houseUp_MESH_BUILDING:')
			end
		end
		--??
		if false then
			for i,f in pairs(desc.afloor) do
				if f.achild then
					for k,c in pairs(f.achild) do
							lo('??^^^^^^^^^^^^^^^^+++++++++++++++ to_CHILD_PRN:'..k,true)
						c.prn = desc.id
					end
				end
			end
		end
		if not id then return end
		adesc[id] = desc
		-- link forest items
		--- for walls
		forBuilding(desc, function(w)
			--- link wins
			if w.df[w.win] then
				for _,key in ipairs(w.df[w.win]) do
					dforest[key].mesh = id
				end
			end
			if w.df[w.door] ~= nil then
				--- link doors
				for _,key in ipairs(w.df[w.door]) do
					dforest[key].mesh = id
				end
			end
		end)
		--- for roof
		local desctop = desc.afloor[#desc.afloor].top
		if desctop and desctop.df ~= nil then
			for dae,list in pairs(desctop.df) do
				for _,key in ipairs(list) do
	--                        lo('??_____to_LINK:'..key..':'..id)
					dforest[key].mesh = id
				end
			end
		end
--        lo('<< houseUp:'..#amesh..':'..tostring(amesh[id]))
--    else
	end
--[[
	if not indrag and not inrollback then
		toJSON(desc)
			lo('??+++++++++++++++++++++++++++++++ SAVED:'..#asave, true)
	end
]]
--            U.dump(desc.afloor[2].top.achild, '?? post_render333:') --..ic..':'..tostring(c.id),true)
  if out.editkey then
    cedit.forest = out.editkey
    out.editkey = nil
  end
			lo('<<------ houseUp:'..tostring(indrag)..' asave:'..#asave..':'..tostring(cedit.forest))
	return desc
--            lo('?? HU3:'..':'..tostring(cedit.forest))
--            U.dump(out.avedit, '<< houseUp:')
end
--[[
							for s=1,#mdata.uvs do
								mdata.uvs[s].u = mdata.uvs[s].u + c.uvref[1]
								mdata.uvs[s].v = mdata.uvs[s].v + c.uvref[2]
							end
]]
--[[
						if true then
--                                lo('??^^^^^^^^^^^^^^^^ to_LID_child:'..i,true)
						else

							if forsplit or c.id == nil then
								local id, om = meshUp(subdata and {mdata,subdata} or {mdata}, 'lid', groupEdit)
									lo('?? for_ID:'..tostring(id))
								aedit[id] = {mesh = mdata, desc = f.top} -- {ij = {i,nil}, type = 'cover'}}
								c.id = id
							elseif dirty then
								local top = scenetree.findObjectById(c.id)
				--                        if not indrag then lo('?? roof_update:'..i..':'..tostring(top)..':'..f.top.mat) end
								if top ~= nil then
		--                            lo('?? to_cm:'..mdata.material)
									local am = subdata and {mdata,subdata} or {mdata}
	--                                    U.dump(am, '??***************** hU.roof_mesh_Update:',true)
									if M.valid(am) then
										top:createMesh({})
										top:createMesh({am})
									else
										lo('!! ERR_MESH:',true)
									end
								else
									lo('!! ERR.houseUp_NOTOPCHILD:'..tostring(c.id),true)
								end
							else
								am[#am + 1] = mdata
								if subdata then
									am[#am + 1] = subdata
								end
							end

						end
]]
	--[[
								f.top.ridge.flat = nil
										U.dump(mdata, '?? ++++++++++++ for_MDATA:'..#f.top.achild)
								if #f.top.achild then
								scenetree.findObjectById(f.top.achild[1].id):delete()
								f.top.achild[1].id = nil
								T.forRidge(f)
								mdata = roofUp(c.shape, desc, i, ic, prn)
	]]


local function fromWall(wdesc, base, j)
	local u = base[j % #base + 1] - base[j]
  local uvref = {0,0}
  if wdesc.uvref then
    uvref[2] = wdesc.uvref[2]
  end
      U.dump(wdesc.uvref,'??^^^^^^^^^^^^^^^^^ for_SCALE:'..wdesc.u:length()..':'..wdesc.ij[2],true)
  if not wdesc.uvref then wdesc.uvref = {0,0} end
  uvref[1] = (wdesc.uvref and wdesc.uvref[1] or 0) + wdesc.u:length() --*(wdesc.uvscale and wdesc.uvscale[1] or 1)
  uvref[2] = wdesc.uvref and wdesc.uvref[2] or 0 --*(wdesc.uvscale and wdesc.uvscale[1] or 1)
--  wdesc.uvref
--      uvref = {0,0}
--      wdesc.uvref = uvref
--      wdesc.uvref = {0,0}
	return {
		ij = {wdesc.ij[1], j},
		u = u, v = wdesc.v,
		mat = wdesc.mat, uv = {0, 0, 1, 1}, -- u:length(), wdesc.v:length()},
		win = wdesc.win, -- daePath.win[windae],
		df = {},
		winspace = wdesc.winspace, winbot = wdesc.winbot, winleft = wdesc.winleft,
		door = wdesc.door, doorbot = wdesc.doorbot, doorwidth = wdesc.doorwidth,
		doorind = nil,
    uvscale = wdesc.uvscale,
    uvref = uvref,
	}
end


local function fromFloor(desc, i)
	local fnew = deepcopy(desc.afloor[i])
	for j,w in pairs(fnew.awall) do
		w.id = nil
		for dae,d in pairs(w.df) do
			w.df[dae] = {scale = d.scale}
			w.skip = nil
			w.spany = nil
		end
--        w.df = {}
	end
	fnew.top.df = {}

	return fnew
end


local function forFloor(base, h, wdesc, floorn)
	local awall = {}
	if floorn == nil then
		floorn = wdesc.ij[1]
	end
	for j = 1,#base do
		awall[#awall + 1] = {
			ij = {floorn, j},
			u = base[j % #base + 1] - base[j], v = vec3(0, 0, h),
			mat = wdesc.mat, uv = {0, 0, 1, 1},
			win = wdesc.win, -- daePath.win[windae],
			df = {},
			winspace = wdesc.winspace, winbot = wdesc.winbot, winleft = wdesc.winleft,
			door = wdesc.door, doorbot = wdesc.doorbot, doorwidth = wdesc.doorwidth,
			doorind = nil,
			pillar = deepcopy(wdesc.pillar),
		}
	end
	return awall
end


local function shape2base(nm)
	local base = {vec3(0,0,0)}
	local vx,vy = vec3(-5,0,0), vec3(0,-5,0)
	if false then
	elseif nm == 'sh_square' then
		base[#base+1] = base[#base] + 2.*vx
		base[#base+1] = base[#base] + 2.*vy
		base[#base+1] = base[#base] - 2.*vx
	elseif nm == 'sh_hexa' then
		for i=0,4 do
			local ang = i*math.pi/3
			base[#base+1] = base[#base] + vx*math.cos(ang) + vy*math.sin(ang)
		end
	elseif nm == 'sh_diamond' then
		base[#base+1] = base[#base] + 1.*vx
		base[#base+1] = base[#base] + 1.*vx*math.cos(math.pi/4) + 1.*vy*math.cos(math.pi/4)
		base[#base+1] = base[#base] + 1.*vy
		base[#base+1] = base[#base] - vx*(1+math.cos(math.pi/4))
	elseif nm == 'sh_cross' then
		for i=0,3 do
			local ang = math.pi/2*i
			base[#base+1] = base[#base] + vx*math.cos(ang) + vy*math.sin(ang)
			base[#base+1] = base[#base] + vx*math.cos(ang+math.pi/2) + vy*math.sin(ang+math.pi/2)
			base[#base+1] = base[#base] + vx*math.cos(ang) + vy*math.sin(ang)
		end
		table.remove(base, #base)

	elseif nm == 'b_shape' then
		base[#base+1] = base[#base] + 2.5*vx
		base[#base+1] = base[#base] + 1.6*vy
		base[#base+1] = base[#base] - 2.5*vx
	elseif nm == 'l_shape' then
		base[#base+1] = base[#base] + 2*vx
		base[#base+1] = base[#base] - vy
		base[#base+1] = base[#base] + vx
		base[#base+1] = base[#base] + 2*vy
		base[#base+1] = base[#base] - 3*vx

--[[
		base[#base+1] = base[#base] - vy
		base[#base+1] = base[#base] + vx
		base[#base+1] = base[#base] + 2*vy
		base[#base+1] = base[#base] - 2*vx
]]
	elseif nm == 'sh_t' then
--    elseif nm == 't_shape' then
		base[#base+1] = base[#base] + vx
		base[#base+1] = base[#base] - vy
		base[#base+1] = base[#base] + vx
		base[#base+1] = base[#base] + vy
		base[#base+1] = base[#base] + vx
		base[#base+1] = base[#base] + vy
		base[#base+1] = base[#base] - 3*vx
	elseif nm == 'sh_u' then
--    elseif nm == 'p_shape' then
		base[#base+1] = base[#base] + 3*vx
		base[#base+1] = base[#base] + 2*vy
		base[#base+1] = base[#base] - vx
		base[#base+1] = base[#base] - vy
		base[#base+1] = base[#base] - vx
		base[#base+1] = base[#base] + vy
		base[#base+1] = base[#base] - vx
--        base[#base+1] = base[#base] - vy
	elseif nm == 's_shape' then
		base[#base+1] = base[#base] + vx
		base[#base+1] = base[#base] + vy
		base[#base+1] = base[#base] + vx
		base[#base+1] = base[#base] + 2*vy
		base[#base+1] = base[#base] - vx
		base[#base+1] = base[#base] - vy
		base[#base+1] = base[#base] - vx
	else
		base[#base+1] = base[#base] + 2.5*vx
		base[#base+1] = base[#base] + 1.6*vy
		base[#base+1] = base[#base] - 2.5*vx
	end
	out.cshape = nm
	return base
end


local function undo()
	inrollback = true
	indrag = false
	if #asave > 1 then
		fromJSON()
--        table.remove(asave, 1)
		if not cij and cedit.mesh then
			scope = 'building'
			cij = {1,1}
		end
			lo('?? undo.restored:'..tostring(cedit.mesh)..':'..tostring(cij))
		markUp()
	end
end


local isfirst = true

local function buildingGen(p, base, intest)
--            p = vec3(0,0,0)
--    if test(p) then return end
	local ingen = (base) and true or false
--    local ingen = (base and not intest) and true or false
		lo('>> buildingGen:'..tostring(p)..':'..tostring(base)..':'..tostring(ingen)..':'..tostring(intest))
	if false then
		local desc = jsonReadFile('/lua/ge/extensions/editor/gen/save.json')
		desc = U.fromJSON(desc)
		desc.id = nil
		for i,f in pairs(desc.afloor) do
			for j,w in pairs(f.awall) do
				w.id = nil
				w.df = {}
			end
			f.top.df = {}
		end
			lo('?? fj1:'..tostring(desc.pos)..':'..tostring(desc.id))
	--        U.dump(desc.pos, '?? fj1:'..tostring(desc.pos))
		houseUp(desc)
				if true then return end
	end

	out.inconform = false
	-- build params
	local len = 6 + math.random(10)
	local w = 5 + math.random(6)

	if out.inseed then
		base = shape2base(W.ui.building_shape)
	elseif not base then

		base = {
			vec3(0, 0, 0),
			vec3(-len, 0, 0),
			vec3(-len, -w, 0),
			vec3(0, -w, 0),
		}
		if U._PRD == 1 or isfirst then
			isfirst = false
			base = out.cshape and shape2base(out.cshape) or {
				vec3(0, 0, 0),
				vec3(-12, 0, 0),
				vec3(-12, -8, 0),
				vec3(0, -8, 0),
			}
--[[
			base = {
				vec3(0, 0, 0),
				vec3(-12, 0, 0),
				vec3(-12, -8, 0),
				vec3(0, -8, 0),
			}
]]
--[[
			base =  {
				4*vec3(1,0,0),
				4*vec3(0,1,0),
				4*vec3(-1,0,0),
				4*vec3(0,-1,0),
			}
]]
			if false and U._PRD == 0 then
				base = {
					vec3(-0.5,0,0),
					vec3(1,0,0),
					vec3(1,2,0),
					vec3(3,4,0),

					vec3(5,4,0),
					vec3(5,7,0),

					vec3(3,6,0),
					vec3(0,3,0),
				}
			end


	--[[
				base = {
		--            vec3(-6.884427697,-8,0),
				vec3(0,0,0),
				vec3(-6.884426951,-4.768371582e-07,0),
		--            vec3(-6.884427697,-8,0),
				vec3(-6.884426198,3.579999523,0),
				vec3(-12.99999925,3.58,0),
				vec3(-13,-8,0),
				vec3(0,-8,0),
			}

			base = {
				vec3(0, 0, 0),
				vec3(-6, 0, 0),
				vec3(-6, 0, 0),
				vec3(-12, 0, 0),
				vec3(-12, -8, 0),
				vec3(0, -8, 0),
			}

			base = {
				vec3(0, 0, 0),
				vec3(-6, 2, 0),
				vec3(-12, 0, 0),
				vec3(-12, -8, 0),
				vec3(-6, -6, 0),
				vec3(0, -8, 0),
			}

			base = {
				vec3(0, 0, 0),
				vec3(-6, 2, 0),
				vec3(-12, 0, 0),
				vec3(-12, -8, 0),
				vec3(-6, -6, 0),
				vec3(0, -8, 0),
			}

			base = {
	--            vec3(-6.884427697,-8,0),
				vec3(0,0,0),
				vec3(-6.884426951,-4.768371582e-07,0),
	--            vec3(-6.884427697,-8,0),
				vec3(-6.884426198,3.579999523,0),
				vec3(-12.99999925,3.58,0),
				vec3(-13,-8,0),
				vec3(0,-8,0),
			}


			base = {
	--            vec3(-6.884427697,-8,0),
				vec3(0,0,0),
				vec3(-6.884426951,-4.768371582e-07,0),
	--            vec3(-6.884427697,-8,0),
				vec3(-6.884426198,4.079999523,0),
				vec3(-9.99999925,4.08,0),
				vec3(-10,-8,0),
				vec3(0,-8,0),
			}

			base = {
	--            vec3(-6.884427697,-8,0),
				vec3(0,0,0),
				vec3(-6.884426951,-4.768371582e-07,0),
	--            vec3(-6.884427697,-8,0),
				vec3(-6.884426198,4.079999523,0),
				vec3(-11.99999925,4.08,0),
				vec3(-12,-8,0),
				vec3(0,-8,0),
			}

			base = {
	--            vec3(-6.884427697,-8,0),
				vec3(0,0,0),
				vec3(-6.884426951,-4.768371582e-07,0),
	--            vec3(-6.884427697,-8,0),
				vec3(-6.884426198,8.079999523,0),
				vec3(-11.99999925,8.08,0),
				vec3(-12,-8,0),
				vec3(0,-8,0),
			}
	]]
		elseif U._PRD == 0 then
			base = {
	--            vec3(-6.884427697,-8,0),
				vec3(0,0,0),
				vec3(-6.884426951,-4.768371582e-07,0),
	--            vec3(-6.884427697,-8,0),
				vec3(-6.884426198,4.08,0),
				vec3(-11.99999925,4.08,0),
				vec3(-12,-8,0),
				vec3(0,-8,0),
			}
		end

	end

	local afloor = {}
	--- ground floor
	afloor[#afloor + 1] = nil

	-- TODO: generate values
	local nfloor = ingen and math.random(2, 4) or 2
	if intest==2 then nfloor = math.random(4, 7) end
	local haspillars
	if ingen and #base == 8 then
		-- add pillars
		haspillars = true -- math.random(1,2) == 1
		if intest==2 then haspillars = false end
	end
	local daewin = 1
	local daedoor = 1
	local daeroofborder = 1
	local winspace, winbot, winleft = 1.7 + 0*math.random(), 1, 0.2
	local doorbot, doorwidth, doorind = default.doorbot, 1.5, 2
	doorind = math.random(2, math.floor((base[2] - base[1]):length()/(winspace + 2)))
--    for i = 1,1 do
	local cheight = 0
--    local amat = {'metal_plates', 'm_roof_slates_ochre'}
	local amat = dmat.roof -- {'m_metal_claddedzinc', 'm_roof_slates_rounded'}
--    local amat = {'m_metal_claddedzinc', 'm_roof_slates_ochre', 'm_roof_slates_rounded'}
--    local amat = {'m_metal_brushed', 'm_plaster_worn_01', 'm_roof_slates_ochre'}
--    local awmat = {'m_wood_oak_raw_01', 'm_wood_frame_trim_01', 'm_bricks_01', 'm_bricks_01'} --, 'm_building_interiors_01','m_bricks_01'}
--    local awmat = {'m_plastic_trim_01', 'm_stonebrick_eroded_01'} --'m_stonebrick_mixed_02'} --, 'm_stonebricks_mixed_01', 'm_stonebricks_mixed_01'}
	local awmat = {'m_bricks_01_bat'} --, 'm_plastic_trim_01_bat'} --, 'm_stonebrick_eroded_01'} --'m_stonebrick_mixed_02'} --, 'm_stonebricks_mixed_01', 'm_stonebricks_mixed_01'}
--	local awmat = {'m_bricks_01', 'm_plastic_trim_01', 'm_stonebrick_eroded_01'} --'m_stonebrick_mixed_02'} --, 'm_stonebricks_mixed_01', 'm_stonebricks_mixed_01'}
--    local awmat = {'m_bricks_01', 'metal_plates', 'm_stonebrick_eroded_01'} --'m_stonebrick_mixed_02'} --, 'm_stonebricks_mixed_01', 'm_stonebricks_mixed_01'}

	if out.inseed then
			lo('?? for_shape:'..tostring(W.ui.building_shape))
--        U.dump(W.ui.mat_wall, '??___________ GEN_mat:')
		if W.ui.building_style == 0 then
			awmat = {out.defmat}
		else
			awmat = W.ui.mat_wall
		end
	end

--    local awmat = {'m_bricks_01', 'm_metal_brushed'} --, 'm_stonebricks_mixed_01', 'm_stonebricks_mixed_01'}
	local aroof = {'flat','shed','gable'}
	local rshape = 'flat'
	if ingen and #base == 4 and nfloor < 4 then
		rshape = 'gable'
		if intest==2 then rshape = 'flat' end
		if not isRect(base) then rshape = 'flat' end

--        rshape = ingen and 'gable' or (aroof)[math.random(1,3)]
--        rshape = ingen and ({'shed','gable'})[math.random(1,2)] or 'flat'
	end
	local margin = ingen and U.rand(0.3, 0.5) or default.topmargin
			if intest==2 then margin = 0 end
	local mat
	mat = ingen and amat[math.random(1,#amat)] or 'WarningMaterial'
	if rshape == 'flat' then
		mat = default.topmat --'m_stucco_white_bat' --'cladding_zinc' --'m_metal_claddedzinc'
--        out.curselect = U.index(dmat.roof, 'm_stucco_white_bat')[1]
--        mat = ingen and 'm_metal_claddedzinc' or 'm_roof_slates_rounded'
--        mat = 'm_roof_slates_rounded' --'m_metal_claddedzinc' --'metal_plates'-- 'm_metal_claddedzinc'
	else
		mat = 'm_roof_slates_rounded' -- 'm_roof_slates_ochre'
	end

	local wmat = (ingen or out.inseed) and tostring(awmat[math.random(1, #awmat)]) or out.defmat -- 'm_greybox_base'
	if U._PRD == 0 then
		wmat = out.defmat -- 'WarningMaterial'
	end
		if intest==2 then
			mat = 'WarningMaterial'
			wmat = out.defmat --'m_greybox_base'
		end

--        lo('??_____________ for_MAT:'..#U.index(dmat.wall, wmat)..':'..wmat)
	local mind = U.index(dmat.wall, wmat)
	if wmat == out.defmat then
		W.out.curselect = 0
	else
		if #mind == 1 then
			W.out.curselect = mind[1] - 1
		end
	end
--[[
	if #mind == 1 then
		W.out.curselect = mind[1] - 1
	end
]]
	for i = 1,nfloor do
		local h = default.floorheight + 0*math.random()
		cheight = cheight + h
--        local mat = 'WarningMaterial'
		local awall = {}
		local ch = h
		local cbase = U.clone(base)
		if i == 1 and #base == 8 and haspillars then
--            lo('?? to_PILLAR:'..#base)
			cbase = {cbase[1],cbase[2],cbase[3], cbase[4]}
			cbase[#cbase + 1] = cbase[#cbase] + (base[5] - base[4])/2
			cbase[#cbase + 1] = cbase[#cbase] + (base[6] - base[5])
			cbase[#cbase + 1] = cbase[#cbase] - (base[5] - base[4])/2
			cbase[#cbase + 1] = base[#base]
		end
		if true then
	--        local mat = 'checkered_line_art'
--           local mat = 'WarningMaterial'
	--        local mat = 'm_sidewalk_curb_trim_01'
	--        local mat = 'm_asphalt_new_01'
	--        mat = 'AsphaltRoad_damage_large_decal_01'
	--        for j = 1,1 do
			for j = 1,#cbase do

				local hasdoor = false
				if i == 1 and j == 1 then -- math.random(#base)
					hasdoor = true
				end

--                local winspace, winbot, winleft = 2, 1, 1.5
				-- TODO: check mesh vertical displacment
--                local windae = 1
				local u,v = cbase[j % #cbase + 1] - cbase[j],vec3(0, 0, h)
				local mwin = daewin

				if (rshape == 'flat' and nfloor == 3 and i == 3) then
					mwin = 3
					ch = 4
				end
				awall[#awall + 1] = {
					ij = {i, j},
					u = u, v = v,
--                    mat = mat, uv = {0, cheight, u:length(), cheight+v:length()}, agrid = {}, -- ax = {}, ay = {},
--                    mat = mat, uv = {0, 0, u:length(), v:length()}, agrid = {}, -- ax = {}, ay = {},
--                    mat = mat, uv = {0, cheight, u:length(), v:length()}, agrid = {}, -- ax = {}, ay = {},
--                    mat = (nfloor == 4 and i == 1) and 'm_metal_frame_trim_01' or wmat,
					mat = (nfloor == 4 and i == 1) and 'm_stonewall_damaged_01' or wmat,
					uv = {0, cheight, u:length(), cheight-v:length()}, agrid = {}, -- ax = {}, ay = {},
--                    mat = mat, uv = {0, 0, 1, 1}, agrid = {}, -- ax = {}, ay = {},
--                    win = daePath.balcony[daewin],
					win = daePath.win[mwin],
--                    awin = {},
					winspace = winspace, winbot = winbot, winleft = winleft,
					balcony = {dae = daePath.balcony[1], ind = {1,2}},
--                    corner = {dae = daePath.corner[1], yes = false},
					pillar = {dae = daePath.pillar[1], space = 2, margin = 0.6, yes = false},
					door = daePath.door[daedoor], --(hasdoor and daePath.door[daedoor] or nil),
					doorbot = doorbot,
					doorwidth = doorwidth,
					doorind = hasdoor and doorind or nil,
					storefront = {adae = {}, yes = false},
					stringcourse = {dae = nil, yes = false},
--                    adoor = {},
					df = {}, -- dictionary dae_name -> forest_items_list
					achild = {},
				}
				local cw = awall[#awall]
        if cw.pillar.dae then
          cw.pillar.inout = -math.abs(ddae[cw.pillar.dae].to.y - ddae[cw.pillar.dae].fr.y)/2
  --                cw.pillar.margin = cw.pillar.inout

          if haspillars and i == 2 and j == 5 then
            awall[#awall].pillar.yes = true
            awall[#awall].pillar.down = afloor[1].h
          end
        end
--                awall[#awall].df[awall[#awall].win] = {}
--                lo('?? for_dorr:'..tostring(awall[#awall].door))
--                awall[#awall].df[awall[#awall].door] = {}
			end
		else
			awall = forFloor(cbase, h, {
				ij = {i},
				mat = mat,
				win = daePath.win[daewin],
				winspace = winspace,
				winbot = winbot,
				winleft = winleft,
				door = (hasdoor and daePath.door[daedoor] or nil),
				doorbot = doorbot,
				doorwidth = doorwidth,
				doorind = doorind,
			})
		end
--            U.dump(awall, '?? awall:')
--            afloor[#afloor + 1] = {base = base, h = h, awall = awall}
		local dim = (base[1] - base[math.floor(#base/2)+1]):length()
		afloor[#afloor + 1] = {base = U.clone(cbase), h = ch, pos = vec3(0,0,0),
			awall = awall,
			top = {ij = {i, nil}, type = 'top', shape = 'flat', poly = '', istart = 1, id = nil, body = nil,
				ridge = {},
--                roofborder = daePath.roofborder[daeroofborder], fdep = {'roofborder'},      -- forest types with dependent materials
				achild = {}, cchild = nil, -- {list= {}, shape = '', mat = '', margin = 0, tip, aside = {}, base={}, av ={}} rc index list
											-- list - aindex of rc in body,,,,,aside - index of outer base sides the vertex belongs, av - corners world pos
				mat = mat, margin = 0, tip = dim/6, df = {}}}
		local mbase = cbase
		if #afloor == nfloor then
--                    afloor[#afloor].top.margin = default.topmargin
--                    afloor[#afloor].top.shape = 'gable'
			afloor[#afloor].top.shape = rshape
			afloor[#afloor].top.margin = margin
			afloor[#afloor].top.fat = default.topthick
--            if intest then afloor[#afloor].top.ridge.on = true end
			mbase = U.polyMargin(cbase, afloor[#afloor].top.margin)
		end
--        local cover = coverUp(mbase)
--                if true then return end
--??        afloor[#afloor].top.body = cover



--        cheight = cheight + 1.5
	end
--            afloor[#afloor].top.poly = 'V'
--            afloor[#afloor].top.shape = 'gable'
	-- render
	return houseUp({pos = p, afloor = afloor, basement = {yes = false}})

--    toJSON(desc)
--        for key,b in pairs(adesc) do
--            U.dump(adesc[key].afloor[1].base, '<< buildingGen:'..key..':'..tostring(adesc[key].pos))
--        end
--        U.dump(adesc[1].afloor[1].base, '<< buildingGen:')
--        lo('<<'..cedit.mesh)
--        U.dump(adesc[cedit.mesh].afloor[1].base, '<< buildingGen:'..tostring(adesc[cedit.mesh].pos))
end


local function test(p)
		lo('>>"""""""""""""""""""""""" test:',true)
--        pretest(true)
		if U._PRD ~= 0 then return end
--		if true then return end
	if true then
		lo('?? test_if_build:'..tableSize(adesc))
		return
	end
    if false then
        local function cb(job)
                lo('?? cb_JOB:')
			local pos,w,h
			for _,d in pairs(adesc) do
				local u = d.afloor[1].awall[1].u
				h = forHeight(d.afloor)
				pos = d.pos + u/2 + h/2*U.vturn(u:normalized(), -math.pi/2) + vec3(0,0,h/2)
				w = u:length()
					lo('?? for_DESC:'..tostring(d.pos)..':'..tostring(pos))
			end
--            local defaultRes = vec3(512, 512, 0)
			local resY = 1024*2
			local resX = math.floor(resY*w/h)
            local defaultRes = vec3(resX, resY, 0)
            local fname = '/tmp/shot_test3.png'
    --        local fname = '/tmp/shot_test'
            local renderView = RenderViewManagerInstance:getOrCreateView('rvTest')
            renderView.luaOwned = true
			local q = quatFromDir(vec3(0,-1,0), vec3(0,0,1))
            local rot = QuatF(0,0,0,1)
				rot = QuatF(q.x,q.y,q.z,q.w)
--				rot = quatFromDir(vec3(-1,0,0), vec3(0,0,1))
            local mat = rot:getMatrix()
			if pos then
	            mat:setPosition(pos)
			end

            renderView.cameraMatrix = mat
            renderView.resolution = Point2I(defaultRes.x, defaultRes.y)
            renderView.viewPort = RectI(0, 0, defaultRes.x, defaultRes.y)

            local aspectRatio = defaultRes.x / defaultRes.y
            local renderOrthogonal = false
            local fov = 90
            local nearPlane = 0.1
            local farClip = 2000
            renderView.frustum = Frustum.construct(renderOrthogonal, math.rad(fov), aspectRatio, nearPlane, farClip)

            renderView.renderCubemap = false
            renderView.namedTexTargetColor = 'rvTest'
            renderView.fov = fov
            renderView.renderEditorIcons = false

			if settings.getValue('GraphicAntialias') == 4 and settings.getValue('GraphicAntialiasType') == "fxaa" then
				settings.setValue('GraphicAntialias', 0)
--				activateAAAfter = true
			end
--            ui_visibility.set(false)
--            gameplay_markerInteraction.setMarkersVisibleTemporary(false)
            job.sleep(0.1)
            renderView:saveToDisk(fname)
                lo('?? for_RV3:'..tostring(renderView))
            RenderViewManagerInstance:destroyView(renderView)
--            gameplay_markerInteraction.setMarkersVisibleTemporary(true)
--            ui_visibility.set(true)
        end

        core_jobsystem.create(cb, nil, nil, nil, nil)
    end
	if true then return end

	if false then
--		local slaxml = require('libs/slaxml/slaxml')
--		local slaxdom = require('libs/slaxml/slaxdom')

		local xml = M.xmlOn(editor.getLevelPath()..'bat/'..'exp_22762.dae')
			lo('?? for_node:'..tostring(xml))
--		for i,k in pairs(xml.kids) do
--			lo(k.name)
--		end
		local nd = M.forNode(xml, {'COLLADA', 'library_materials'})
		if nd then
			local kid
			kid = M.toNode(nd, 'material', {id='m_concrete_plain_01-material', name='m_concrete_plain_01'})
			kid = M.toNode(kid, 'instance_effect', {url='m_concrete_plain_01-effect'})
		end

		nd = M.forNode(xml, {'COLLADA', 'library_geometries', 'geometry', 'mesh', 'triangles'})
		if nd then
			M.ofNode(nd, {material='m_concrete_plain_01-material'})
		end

		nd = M.forNode(xml, {'COLLADA', 'library_visual_scenes', 'visual_scene', 'node', 'instance_geometry', 'bind_material', 'technique_common'})
		if nd then
			lo('?? for_GEO:'..#nd.kids)
			M.toNode(nd, 'instance_material', {symbol="m_concrete_plain_01-material", target='#m_concrete_plain_01-material'})
		end

		M.xml2file(xml, editor.getLevelPath()..'bat/'..'exp_22762_out.dae')
		return
	end

	if false then
--		local j1 = D.junctionUp(vec3(0,-300), 4)
		local aang = nil --{1.53198,1.62402,1.66811,1.51353}
--		local j2 = D.junctionUp(vec3(-170,-310), 4, true, aang)--, {1.602,1.473,1.573,1.579})
--		local j2 = D.junctionUp(vec3(-250,-170), 3)

--		local j1 = D.junctionUp(vec3(220,-56), 4, false)
--		local j1 = D.junctionUp(vec3(262,-48), 4)
--		D.junctionRound()

--		local j2 = D.junctionUp(vec3(96,-293), 3)

		aang = nil -- {1.48960,1.51685,1.57692,1.55452}
		local j3 = D.junctionUp(vec3(-63,-55), 4, true, aang) --, {1.495,1.490,1.622,1.577})
--		D.junctionRound()

--		D.forPlot(true)

--		D.branchMerge(j1.list[1], j3.list[3])

--		D.branchMerge(j2.list[3], j1.list[3])
--		D.branchMerge(j1.list[2], j2.list[4])
--		D.setCar(vec3(-4, -186), 0)

--		D.setCar(vec3(-68, -2), 0, 1)
--		U.camSet({-231.66, -21.47, 87.26, -0.0737859, 0.110495, -0.824252, -0.550413})


		--[-231.66, -21.47, 87.26, -0.0737859, 0.110495, -0.824252, -0.550413]
		-- turn left
		-- lane left
--		D.setCar(vec3(-67, 6), 0)
		-- lane right
--		D.setCar(vec3(-67, -5), 0)
--		D.setCar(vec3(-67, -5), 0)
--		D.setCar(vec3(-60, -97), 2)
--		D.setCar(vec3(-45, -246),0.3)
--+		D.setCar(vec3(-4, -210), 0)

--		D.setCar(vec3(-4, -235), 0)

--		D.setCar(vec3(-12.7, -237), 20)

--		D.junctionUp(vec3(0,0), 4)
--		D.junctionUp(vec3(100,100), 4)

--		D.junctionUp(vec3(366,211), 4)
--		D.junctionUp(vec3(189,9), 4)

--		D.junctionUp(vec3(101,-41), 4)
--		D.junctionUp(vec3(196,110), 4)
		return
	end

	if false then
--		local fname = '/levels/smallgrid/bat/build_in2.json'
		local fname = '/levels/smallgrid/bat/bin1.json'
		local data = jsonReadFile(fname)
			lo('?? from_paul:'..#data)
			indrag = true
		for i=1,10 do
--		for i=1,600 do
--		for i=1,#data-1 do
			local d = data[i]
			local base = {}
			local pos = vec3(d.coords[1][1],d.coords[1][2],0)
			for k=1,#d.coords-1 do
				local p = vec3(d.coords[k][1],d.coords[k][2],0) - pos
				base[#base+1] = p
			end
--				U.dump(pos, '?? for_data:'..tostring(base[2]))
--[[
				base = {
					vec3(0,0,0),
					vec3(10,0,0),
					vec3(10,6,0),
					vec3(0,6,0),
				}SWWWWWWWWWWWWWWWWS
]]
				pos = vec3(0,0,0)
			buildingGen(pos, base)
			if i%50 == 0 then
--				print('?? for_i:'..i)
			end

--		for _,d in pairs(data) do
			if false then
				local base = {
					vec3(0,0,0),
					vec3(10,0,0),
					vec3(10,6,0),
					vec3(0,6,0),
				}
				buildingGen(vec3(d.pos[1],d.pos[2],d.pos[3]), base)
			end
		end
	end

	if false then
		local filepath = '/levels/italy/art/prefabs/'
		local data = jsonReadFile(filepath)
	end

	local base,aloop
	if false then
		base = {
	vec3(0,0,0),
	vec3(10,0,0),
	vec3(10,5,0),
	vec3(0,5,0),
		}
		aloop = {
			{
				vec3(3,0,0),
				vec3(3,5,0),
--				vec3(3,0,0),
			},
			{vec3(9,0,0),vec3(9,5,0)},
			{
				vec3(4,1,0),
				vec3(6,1,0),
				vec3(6,3,0),
				vec3(4,3,0),
			},
--[[

			{
				vec3(2,2,0),
				vec3(3,2,0),
				vec3(3,6,0),
				vec3(2,6,0),
			},
			{
				vec3(7,2,0),
				vec3(8,2,0),
				vec3(8,6,0),
				vec3(7,6,0),
			},
]]
			{
				vec3(7,2,0),
				vec3(8,2,0),
				vec3(8,3,0),
				vec3(7,3,0),
			},
		}
		local mbody,mhole,albl,aedge = M.rcPave(base, aloop)
		if M.valid({mbody}) then
			meshUp({mbody}, 'tst', groupEdit)
--				U.dump(mbody, '?? mbody:')
		end
		if aedge then
			out.fwhite = aedge
			out.fmtop = nil
			out.flbl = albl
		end
		return
	end

  if false then
    local mbody = M.forBeam({
      vec3(0,0,0),
      vec3(1,1,0),
      vec3(2,0,0),
    }, 5, true)
      U.dump(mbody)
--      lo('?? if_TUBE:'..tostring(af))
    if M.valid({mbody}) then
--        U.dump(mbody, '?? test_mdata:'..tostring(aedge))
      meshUp({mbody}, 'tst', groupEdit)
    end
  end

  if false then
    local base = {
vec3(0,0,3),
vec3(10,0,3),
vec3(10,5,3),
vec3(0,5,3),
--[[
vec3(-7.599999905,0,3.200000954),
vec3(-12,0,3.200000954),
vec3(-12,-8,3.200000954),
vec3(-7.599999905,-8,3.200000954),
]]
--[[
vec3(0,0,6.4),
vec3(-6.97370631,0,6.4),
vec3(-12,0,6.4),
vec3(-12,-8,6.4),
vec3(0,-8,6.4),
]]
    }
    local vhit = vec3(2,2,3)
    local inrc = U.inRC(
      vhit,
--      vec3(-9.009117526,-6.076205643,3.200000965),
--      vec3(-2.343889972,8.903316751,6.399999692),
    {base},nil,true)
      lo('?? test_rc:'..tostring(inrc))
  end

  if false then
	local base = {
vec3(0,0,0),
vec3(-5.621447284,-5.548863994,0),
vec3(-12,0,0),
vec3(-12,-8,0),
vec3(0,-8,0),
	}
	local b1, b2 = polySplit(base, true)
		U.dump(b1, '?? split1:')
		U.dump(b2, '?? split2:')
--	local arc = coverUp(base,true)
--		U.dump(arc, '?? test_arc:')
  end

  if false then
    local base = {
vec3(0,0),
vec3(0,4),
vec3(-6,4),
vec3(-6,0),
    }
    local aloop = {
{
vec3(-1,1),
vec3(-1,2),
vec3(-2,2),
vec3(-2,1),
}
    }
        aloop = {}

		local mbody,mhole,albl,aedge
    local am = {}
    mbody,mhole,albl,aedge = M.rcPave(base, aloop)
    mbody.material = 'm_bricks_01_bat'
    if M.valid({mbody}) then
      am[#am+1] = mbody
    end
--        meshUp(am, 'tst', groupEdit)
--        am = {}

    base = {
vec3(6,0),
vec3(6,4),
vec3(0,4),
vec3(0,0),
    }
    aloop = {}
    mbody,mhole,albl,aedge = M.rcPave(base, aloop)
    mbody.material = 'm_bricks_01_bat'
    if M.valid({mbody}) then
      am[#am+1] = mbody
    end

    meshUp(am, 'tst', groupEdit)
  end

	if false then
--[[
66820.93884|I|GELua.print| {
1 = {
	1 = vec3(2.490093408,2.359999952,0),
	2 = vec3(3.109906497,2.359999952,0),
	3 = vec3(3.109906497,3.608793261,0),
	4 = vec3(2.490093408,3.608793261,0), },
2 = {
	1 = vec3(5.690093455,2.359999952,0),
	2 = vec3(6.309906545,2.359999952,0),
	3 = vec3(6.309906545,3.608793261,0),
	4 = vec3(5.690093455,3.608793261,0), },
3 = {
	1 = vec3(8.890093503,2.359999952,0),
	2 = vec3(9.509906592,2.359999952,0),
	3 = vec3(9.509906592,3.608793261,0),
	4 = vec3(8.890093503,3.608793261,0), }, }
66820.94122|I|GELua.print| ?? uppder_BASE:
66820.94123|I|GELua.print| {
1 = vec3(0,0,0),
2 = vec3(-7.599999905,0,0),
3 = vec3(-7.599999905,-8,0),
4 = vec3(0,-8,0), }
]]


		local base = {

vec3(0,0,0),
vec3(7.599999905,0,0),
vec3(7.599999905,3.2,0),
vec3(0,3.2,0),

--[[
vec3(0,0,0),
vec3(12,0,0),
vec3(12,3.2,0),
vec3(0,3.2,0),

vec3(0,0,0),
vec3(10,0,0),
vec3(10,7,0),
vec3(6,7,0),
vec3(6,5,0),
vec3(0,5,0),
]]

--vec3(10,3,0),
--vec3(0,3,0),

--[[
vec3(0,0,0),
vec3(2.96,0,0),
vec3(2.96,1.6,0),
vec3(0,1.6,0),
]]
--vec3(2.96,1.600000024,0),
--vec3(0,1.600000024,0),
		}
		out.fmtop = {base}
--[[ error triang

vec3(1.170093455,1.06,0),
vec3(1.789906545+0.1,1.06,0),
vec3(1.789906545+0.1,2.308793309,0),
vec3(1.170093455,2.308793309,0),

]]
		local aloop = {

{
vec3(0.9400934078,1.06,0),
vec3(1.559906497,1.06,0),
vec3(1.559906497,2.308793309,0),
vec3(0.9400934078,2.308793309,0), },
{
vec3(2.640093408,1.06,0),
vec3(3.259906497,1.06,0),
vec3(3.259906497,2.308793309,0),
vec3(2.640093408,2.308793309,0), },

--[[
3 = {
	1 = vec3(4.340093408,1.06,0),
	2 = vec3(4.959906497,1.06,0),
	3 = vec3(4.959906497,2.308793309,0),
	4 = vec3(4.340093408,2.308793309,0), },
4 = {
	1 = vec3(6.040093408,1.06,0),
	2 = vec3(6.659906497,1.06,0),
	3 = vec3(6.659906497,2.308793309,0),
	4 = vec3(6.040093408,2.308793309,0), },

6 = {
	1 = vec3(5.340093432,-0.8400000477,0),
	2 = vec3(5.959906521,-0.8400000477,0),
	3 = vec3(5.959906521,0.4087932611,0),
	4 = vec3(5.340093432,0.4087932611,0), }
]]
{
vec3(2.640093384,-0.8400000477,0),
vec3(3.259906474,-0.8400000477,0),
vec3(3.259906474,0.4087932611,0),
vec3(2.640093384,0.4087932611,0), },

--[[
{
vec3(2.490093408,2.359999952,0),
vec3(3.109906497,2.359999952,0),
vec3(3.109906497,3.608793261,0),
vec3(2.490093408,3.608793261,0), },
{
vec3(5.690093455,2.359999952,0),
vec3(6.309906545,2.359999952,0),
vec3(6.309906545,3.608793261,0),
vec3(5.690093455,3.608793261,0), },
{
vec3(8.890093503,2.359999952,0),
vec3(9.509906592,2.359999952,0),
vec3(9.509906592,3.608793261,0),
vec3(8.890093503,3.608793261,0), }

{
vec3(0.9400934078,1.06,0),
vec3(1.559906497,1.06,0),
vec3(1.559906497,2.308793309,0),
vec3(0.9400934078,2.308793309,0), },
{
vec3(2.640093408,1.06,0),
vec3(3.259906497,1.06,0),
vec3(3.259906497,2.308793309,0),
vec3(2.640093408,2.308793309,0), },
{
vec3(4.340093408,1.06,0),
vec3(4.959906497,1.06,0),
vec3(4.959906497,2.308793309,0),
vec3(4.340093408,2.308793309,0), },
{
vec3(6.040093408,1.06,0),
vec3(6.659906497,1.06,0),
vec3(6.659906497,2.308793309,0),
vec3(6.040093408,2.308793309,0), },

{
vec3(1.440093455,-0.7399999046,0),
vec3(2.059906545,-0.7399999046,0),
vec3(2.059906545,0.5087934041,0),
vec3(1.440093455,0.5087934041,0), },
{
vec3(4.840093455,-0.7399999046,0),
vec3(5.459906545,-0.7399999046,0),
vec3(5.459906545,0.5087934041,0),
vec3(4.840093455,0.5087934041,0), },
{
vec3(6.540093455,-0.7399999046,0),
vec3(7.159906545,-0.7399999046,0),
vec3(7.159906545,0.5087934041,0),
vec3(6.540093455,0.5087934041,0), },
]]

--[[
    {
vec3(1.2700944,0.2,0),
vec3(2,0.2,0),
vec3(2,1.,0),
vec3(1.2700944,1.,0),
		}
]]
    }
--      aloop = {}

--[[
vec3(1.2700944,1.07,0),
vec3(2,1.07,0),
vec3(2,1.8,0),
vec3(1.2700944,1.8,0),
--vec3(2,2.308793309,0),
--vec3(1.1700944,2.308793309,0),
		},{
vec3(2.2700944,1.07,0),
vec3(2.6,1.07,0),
vec3(2.6,1.8,0),
vec3(2.2700944,1.8,0),
]]
--[[
]]

		for i=1,#aloop do
			out.fmtop[#out.fmtop+1] = aloop[i]
		end
		local mbody,mhole,albl,aedge = M.rcPave(base, aloop)
    if M.valid({mbody}) then
--        U.dump(mbody, '?? test_mdata:'..tostring(aedge))
      meshUp({mbody}, 'tst', groupEdit)
    end

--        local mbody,mhole,ae,apath,albl = M.rcPave2(base, aloop)
		out.flbl = albl
    if aedge then
      out.fwhite = aedge
      out.fmtop = nil
    end
			if true then return end
		if mbody then
			for i=1,#mbody.faces,3 do
				local path = {
					mbody.verts[mbody.faces[i].v+1],
					mbody.verts[mbody.faces[i+1].v+1],
					mbody.verts[mbody.faces[i+2].v+1],
					mbody.verts[mbody.faces[i].v+1],
				}
				out.fmtop[#out.fmtop+1] = path
			end
		end
		return
	end

	if false then
		local base = {
			vec3(0,0,0),
			vec3(20,0,0),
			vec3(20,10,0),
			vec3(0,10,0),
		}
		out.fmtop = {base}
		local aloop = {
			{
				vec3(12,6),vec3(16,6),vec3(16,12),vec3(12,12)
--                vec3(12,-1),vec3(16,-1),vec3(16,4),vec3(12,4)
--                vec3(12,1),vec3(16,1),vec3(16,4),vec3(12,4)
			},
--            {vec3(2,-2),vec3(4,-2),vec3(4,5),vec3(2,5)},
		}
		out.fmtop[#out.fmtop+1] = aloop[1]
		local mbody,mhole,ae,apath,albl = M.rcPave(base, aloop)
--            U.dump(mbody, '?? mbody:')
--            U.dump(apath, '?? mhole:')
		for i=1,#mbody.faces,3 do
			local path = {
				mbody.verts[mbody.faces[i].v+1],
				mbody.verts[mbody.faces[i+1].v+1],
				mbody.verts[mbody.faces[i+2].v+1],
				mbody.verts[mbody.faces[i].v+1],
			}
			out.fmtop[#out.fmtop+1] = path
		end
		return
	end
	if false then
		local base = {
vec3(-4.050905868,1.554117993,6.4),
vec3(-11.62231497,4.458866583,6.4),
vec3(-12,-8,6.4),
vec3(-7.716314831,-8,6.4),
		}
		local i = U.inRC(vec3(-7.651718787,0.1104451204,6.400000774), {base})
			lo('?? if_inrc:'..tostring(i))
		out.apath = {base}
		out.avedit = {vec3(-7.651718787,0.1104451204,6.400000774+0.1)}
		return
	end
	if false then
		local base = {
vec3(0,0,0),
vec3(-6.783516056,0,0),
vec3(-6.783516056,3.400000095,0),
vec3(-11.6042462,3.400000095,0),
vec3(-11.6042462,0,0),
vec3(-15,0,0),
vec3(-15,-10,0),
vec3(-10,-10,0),
vec3(-10,-5,0),
vec3(-8.057351058,-5,0),
vec3(-8.057351058,-7.400000036,0),
vec3(-5,-7.400000036,0),
vec3(-5,-10,0),
vec3(0,-10,0),
--[[
vec3(-8.681075329,10.57470741,0),
vec3(-11.99435381,-4.749814454,0),
vec3(-16.00175891,-3.883382914,0),
vec3(-17.45997995,-10.62792321,0),
vec3(-13.45257486,-11.49435475,0),
vec3(-15.02081845,-18.74776959,0),
vec3(-5.246659455,-20.8610173,0),
vec3(-3.133411747,-11.0868583,0),
vec3(-8.020491246,-10.03023445,0),
vec3(-7.387644178,-7.103200456,0),
vec3(-3.575722075,-7.927367083,0),
vec3(-2.84092402,-4.528791044,0),
vec3(-6.652846122,-3.704624417,0),
vec3(-5.907243538,-0.2560754472,0),
vec3(-1.020164039,-1.312699301,0),
vec3(1.09308367,8.461459697,0),
]]
		}
		local arc = coverUp(base)
		U.dump(arc, '?? test_ARC:')
		out.apath = arc
		out.avedit = {base[1],base[2]}
	end

		if false then
			local base = {
--[[
vec3(-0.08119462677,0.1191920268,0),
vec3(-12.07879471,-0.1207919735,0),
vec3(-11.91880537,-8.119192027,0),
vec3(-6.281633979,-8.006433564,0),
vec3(0.07879470677,-7.879208027,0),
]]


vec3(0.2189276122,0.6524122081,0),
vec3(-12.57083375,0.1405487305,0),
vec3(-12.21892761,-8.652412208,0),
vec3(-6.185458591,-8.410944651,0),
vec3(0.5708337531,-8.140548731,0),
			}
			local adata = T.forGable(base, nil, true)
				lo('?? GABLED:'..tostring(adata and #adata or nil))
--                U.dump(adata, '?? GABLED:')
		end
		if false then
			local base = {
				vec3(0,0,0),
				vec3(-4.597041148,1.489263353,0),
				vec3(-9.569547253,0,0),
				vec3(-15,0,0),
				vec3(-15,-8,0),
				vec3(0,-8,0), }
			local arc = coverUp(base)
				U.dump(arc, '?? test_CU_arc:')
		end

		if false then
-- generate base with parallels
			local cv = vec3(1+math.random(0.5), 1+0.5*math.random())
			local pa = {vec3(0,0,0)}
			local pb = {pa[1]+cv}
			local cdir = U.vturn(cv, -math.pi/2) -- + math.random(0.5))
			for i = 1,3 do
				if i > 1 then
					cdir = U.vturn(cdir, -(0.3+0.2*(math.random()-0*1/2)))
				end
--                cdir = U.vturn(cdir, -0.8*(math.random()-0*1/2))
--                cdir = U.vturn(cv, math.pi/2 + 0.8*math.random())
				pa[#pa+1] = pa[#pa] + cdir*(4+0.8*(math.random()-1/2))
				pb[#pb+1] = pb[#pb] + cdir*(4+0.8*(math.random()-1/2))
			end
			local av = {}
			for i=1,#pa do
				av[#av+1] = pa[i]
			end
			for i=#pb,1,-1 do
				av[#av+1] = pb[i]
			end
--                U.dump(av, '?? base2:'..math.random(0.5))
				local base = {
					vec3(0,0,0),
					vec3(0,3,0),
					vec3(4,3,0),
					vec3(4,5,0),
					vec3(-3,5,0),
					vec3(-3,0,0),
--[[
					vec3(0,0,0),
					vec3(0,3,0),
					vec3(4,3,0),
					vec3(4,5,0),
					vec3(-3,5,0),
					vec3(-3,0,0),

					vec3(0,0,0),
					vec3(0,3,0),
					vec3(4,3,0),
					vec3(4,0,0),

					vec3(0,0,0),
					vec3(-12,0,0),
					vec3(-12,-8,0),
					vec3(0,-8,0),
]]

				}
				base = {vec3(0,0,0)}
				local vx,vy = vec3(-5,0,0), vec3(0,-5,0)
				local vx,vy = vec3(-5,0,0), vec3(0,-5,0)
				base[#base+1] = base[#base] + 3*vx
				base[#base+1] = base[#base] + 2*vy
				base[#base+1] = base[#base] - vx
				base[#base+1] = base[#base] - vy
				base[#base+1] = base[#base] - vx
				base[#base+1] = base[#base] + vy
				base[#base+1] = base[#base] - vx

--[[
				base[#base+1] = base[#base] + vx
				base[#base+1] = base[#base] - vy
				base[#base+1] = base[#base] + vx
				base[#base+1] = base[#base] + vy
				base[#base+1] = base[#base] + vx
				base[#base+1] = base[#base] + vy
				base[#base+1] = base[#base] - 3*vx
]]
--[[
				base[#base+1] = base[#base] + 3*vx
				base[#base+1] = base[#base] + 2*vy
				base[#base+1] = base[#base] - vx
				base[#base+1] = base[#base] - vy
				base[#base+1] = base[#base] - vx
				base[#base+1] = base[#base] + vy
				base[#base+1] = base[#base] - vx

				base = {
					vec3(0,0,0),
					vec3(-15,0,0),
					vec3(-15,-10,0),
					vec3(-10,-10,0),
					vec3(-10,-5,0),
					vec3(-5,-5,0),
					vec3(-4,-10,0),
					vec3(1,-10,0),
				}

]]

				base = {vec3(0,0,0)}
				for i=0,4 do
					local ang = i*math.pi/3
					base[#base+1] = base[#base] + vx*math.cos(ang) + vy*math.sin(ang)
				end

				av = base

				T.forRidge({top = {tip = 2}},base)

			out.fyell = {av}
			out.avedit = {av[1]}
					if true then return end
--                lo('?? 1st:'..tostring(av[1]))

--            local apair,amult = T.pairsUp(av)
--                U.dump(apair, '?? pairs:')
--                U.dump(amult, '?? mults:')
			local adata = T.forGable(av,nil,true) --, apair, W)
--                lo('?? for_am:'..#adata)
			for i,d in pairs(adata) do
				-- set heights
--[[
				for j=1,#d.av do
--                    d[1][j] = d[1][j] + vec3(0,0,1)
				end
				for j=2,#d[1],3 do
--                    d[1][j] = d[1][j] + vec3(0,0,1)
				end
]]
				meshUp({{
					verts = d.av,
					faces = d.af,
					normals = d.an,
					uvs = d.auv,
					material = 'cladding_zinc_bat' --'m_bricks_01',
				}}, 'hat', groupEdit)
			end
		end
------------------------
-- ROBOT
------------------------
		if false then
			local mat
			for t,c in pairs({orange='1 0.5 0 1', green='0 1 0 1', blue='0 0 1 1',yellow='1 1 0 1'}) do
				mat = createObject("Material")
				mat:setField("diffuseColor", 0, c)
		--        mat:setField("diffuseMap", 0, c)
				mat:registerObject('R_mat_'..t)
					lo('?? if_mat:'..tostring(scenetree.findObject('R_mat_'..t)))
			end
		--        lo('?? ifmat:'..tostring(scenetree.findObject('R_512_green')))

					local R = require('/lua/ge/extensions/editor/gen/robot')
					R.clear(groupEdit)
					unrequire('/lua/ge/extensions/editor/gen/robot')
					R = rerequire('/lua/ge/extensions/editor/gen/robot')
			--        R.up()
					R.sceneUp(6, nil, groupEdit) --, dmat.wall)
					R.move(vec3(0,0.2,3.5))
			--        R.voice2job('pick 2 black')
			--        R.voice2job('pick 3 parts')

			--        R.partPick(2)
			--        R.jobUp({{2,1},{3,1}})
			--        R.partPut({{2,1},{3,1}})
			--        R.target(vec3(0,2,2))
					out.R = R
			out.ccommand = ''
		end

		if false then
--            local
			local amat = {
				'cladding_zinc',
--                'm_metal_claddedzinc',
				'm_roof_slates_ochre',
				'm_roof_slates_rounded',
				'm_roof_slates_square',
--[[
				'm_bricks_01', 'm_plaster_raw_dirty_01', 'm_plaster_worn_01',
				'm_stonebrick_eroded_01','m_stonebrick_mixed_01','m_stonebrick_mixed_02',
				'm_stonewall_damaged_01','m_stucco_scraped_01','m_concrete_cinderblock_01',
				'm_plaster_float','m_stucco_white','m_greybox_base',

				'cladding_zinc',
--                'm_metal_claddedzinc',
				'm_roof_slates_ochre',
				'm_roof_slates_rounded',
				'm_roof_slates_square',

				'm_stucco_white_bat',
				'm_plaster_raw_dirty_01_bat',
				'm_plaster_float_bat',
				'm_stucco_scraped_01_bat',
				'm_plaster_worn_01_bat',
]]
			}
	--[[
			local amat = {
				'm_bricks_01', 'm_plaster_raw_dirty_01', 'm_plaster_worn_01',
				'm_stonebrick_eroded_01','m_stonebrick_mixed_01','m_stonebrick_mixed_02',
				'm_stonewall_damaged_01','m_stucco_scraped_01','m_concrete_cinderblock_01',
				'm_plaster_float','m_stucco_white',--'m_greybox_base',
			}
	]]
			local dmat = {}
			local afile = {
--                '/art/shapes/objects/main.materials.json',
--                '/art/shapes/objects/modular_container/main.materials.json',
				'/art/shapes/garage_and_dealership/main.materials.json',
				'/art/shapes/garage_and_dealership/Clutter/main.materials.json',
				'/art/shapes/garage_and_dealership/garage/main.materials.json',
--                '/levels/south_france/art/shapes/buildings/main.materials.json',
			}
			local pref = '*'
			local matFiles = FS:findFiles('/art/shapes/objects/', pref..'materials.json', -1, true, false)
			for _,f in pairs(afile) do
				matFiles[#matFiles+1] = f
			end
			--    local list = {}
			for k,f in pairs(matFiles) do
					lo('?? for_mat_FILE:'..tostring(f))
--                loadJsonMaterialsFile(v)
--            for _,f in pairs(afile) do
				local desc = jsonReadFile(f)
--                    lo('?? for_mat_json:'..tostring(desc))
--                    lo('?? for_mat_json:'..tostring(f)..':'..tostring(desc))
				if desc then
					for k,v in pairs(desc) do
						local ind = U.index(amat, k)[1]
							lo('?? for_mat:'..k..':'..tostring(v.name)..':'..tostring(ind))
						if ind then
							local key = k..'_bat'
							dmat[key] = v
								lo('?? t.for_mat:'..dmat[key].name)
							dmat[key].name = key
							dmat[key].mapTo = key
							dmat[key].activeLayers = 1
							table.remove(amat,ind)
						end
					end
				end
			end
			local pth = '/tmp/bat'
				--'/art/shapes/common/building_architect_modules/bat_materials/'
--            jsonWriteFile(pth..'/wall.materials.json', dmat)
			jsonWriteFile(pth..'/top.materials.json', dmat)
				U.dump(amat,'?? json_DONE:'..tableSize(dmat))

		end

		if true then return end
--[[
		local voicefile = '/tmp/bat/in.txt'
		local file = io.open(voicefile)
		local command = file:read('*all')
		io.close(file)
--        local t = os.execute("echo 'test'")
			lo('?? test:'..command..':'..tostring(os.clock()))
]]
	local base = {
		vec3(0,0,0),
		vec3(0,0,1.5),
		vec3(0,-5,1.5),
		vec3(0,-5,0),
	}
	local an,auv,af = M.zip2(base, {1,2,3,4}) --, {2,3,4,5,6,7})
--            U.dump(af, '?? test_AF:')
--            U.dump(auv, '?? test_AUV:')
	local mdata = {
		verts = base,
		faces = af,
		normals = an,
		uvs = auv,
		material = 'm_bricks_01',-- 'WarningMaterial',
	}
	local id = meshUp({mdata}, 'test', groupEdit)
	W.out.testOM = {id = id, data = mdata}

		if true then return end
	local base = {
		vec3(0,0,0),
		vec3(0,0,1.5),

		vec3(3,0,3),

		vec3(0,0,4),
		vec3(0,-5,4),

		vec3(0,-5,3),

		vec3(0,-5,1.5),
		vec3(0,-5,0),
--        vec3(0,-5,0),
	}

--    an,auv,af,dnorm = M.tri2mdata(base, {2,3,4}, 1, dnorm, an, auv, af, {3,2})

	local an,auv,af = M.zip2(base, {2,3,4,5,6,7})
--            U.dump(an, '?? for_AN:')
--            U.dump(af, '?? for_AF:')
	local mdata = {
		verts = base,
		faces = af,
		normals = an,
		uvs = auv,
		material = 'WarningMaterial',
	}
	meshUp({mdata}, 'test', groupEdit)
		if true then return end

	local base = {
		vec3(0,0,0),
		vec3(1,0,0),
		vec3(1,2,0),
		vec3(3,4,0),

		vec3(5,4,0),
		vec3(5,6,0),

		vec3(3,5,0),
		vec3(0,3,0),
	}
--[[
	base = {
		vec3(-0.3531624084,3.842578263,0),
		vec3(1.427315953,2.237151887,0),
		vec3(3.139744128,3.6,0)
	}
	local av,auv,af = T.pave(base)
			U.dump(av, '?? test_AV:')
			U.dump(af, '?? test_AF:')
			if true then return end
]]
-- ridge test
	base = {
		vec3(0,0,0),
		vec3(1,0,0),
--        vec3(1.5,0,0),
		vec3(1,2,0),
		vec3(3,4,0),

		vec3(8,4,0),
		vec3(8,7,0),

		vec3(3,6,0),
		vec3(0,3,0),
	}
-- split test
	if false then
		base = {
			vec3(0, 0, 0),
	--        vec3(-5, 0.5, 0),
			vec3(-12, 0, 0),
			vec3(-12, -8, 0),
			vec3(0, -8, 0),
		}
	end
	local sbase = deepcopy(base)
-- shape test
	if false then
		base = {
			vec3(0,0,0),
			vec3(2,0,0),
			vec3(2,3,0),
			vec3(4,3,0),
			vec3(4,0,0),
			vec3(6,0,0),
			vec3(6,5,0),
			vec3(0,5,0),
		}
		base = shape2base('p_shape')
		sbase = deepcopy(base)
		base = U.polyMargin(base, 0.4)
			U.dump(base, '?? BASE:')
--        T.pave(base)
--        if true then return end
	end
	if false then
		base = shape2base('sh_t')
		sbase = deepcopy(base)
		base = U.polyMargin(base, 0.4)
			U.dump(base, '?? BASE:')
--            T.forRidge(nil, base)
--            buildingGen(vec3(-266,1144,0), base, true)
--            if true then return end
	end

	local av,auv,af = T.forRidge(nil, base)
--        lo('?? for_ridge:'..#av..':'..#auv..':'..#af)
		U.dump(av, '?? AV:')
		U.dump(af, '?? AF:')
		base = sbase
--        buildingGen(vec3(-266,1144,0), base, true)
		if true then return end

	local achunk = T.forChunks(base)
	-- build mesh
	local av,auv={},{}
	local u = (base[2]-base[1]):normalized()
	local v = vec3(-u.y, u.x, 0)
	local ref = base[1]
	for _,b in pairs(base) do
		av[#av+1] = b
		auv[#auv+1] = {u=(b-ref):dot(u), v=(b-ref):dot(v)}
	end
	local af = {}
	for _,c in pairs(achunk) do
		af = M.zip(c, af)
	end
--		U.dump(af, '?? AF:'..#af)
		if true then return end


	out.m1,out.m2,out.fwhite,out.fyell,out.flbl = M.rcPave({
		vec3(0,0,0), vec3(10,0,0), vec3(10,10,0), vec3(0,10,0)
--[[
		vec3(0,0,0),
		vec3(12,0,0),
		vec3(12,2.8,0),
		vec3(0,2.8,0),
]]
	}, {
--[[
		{vec3(1,1,0), vec3(2,1,0), vec3(2,2,0), vec3(1,2,0)},
--        {vec3(2,3,0), vec3(3,3,0), vec3(3,4,0), vec3(2,4,0)},
		{vec3(1,3,0), vec3(2,3,0), vec3(2,4,0), vec3(1,4,0)},
]]
--        {vec3(1,3,0), vec3(2,3,0), vec3(2,4,0), vec3(1,4,0)},

--[[
		{vec3(1,1,0), vec3(4,1,0), vec3(4,3,0), vec3(1,3,0)},
		--{vec3(2.5,4,0), vec3(3.5,4,0), vec3(3.5,5,0), vec3(2.5,5,0)},
		{vec3(3,4,0), vec3(4,4,0), vec3(4,5,0), vec3(3,5,0)},
		--        {vec3(3,7,0), vec3(8,7,0), vec3(8,9,0), vec3(3,9,0)},
		{vec3(5,2,0), vec3(9,2,0), vec3(9,6,0), vec3(5,6,0)},
]]
		-- sticky test
--[[
		{
			vec3(1,1,0),
			vec3(4,1,0),
			vec3(4,2,0),
			vec3(1,2,0),
		},
		{
			vec3(3,2,0),
			vec3(5,2,0),
			vec3(5,3,0),
			vec3(3,3,0),
		},
]]
		-- border test
		{
			vec3(2,0,0),
			vec3(4,0,0),
			vec3(4,3,0),
			vec3(2,3,0),
		},
--[[
		{
			vec3(2,0,0),
			vec3(4,0,0),
			vec3(4,3,0),
			vec3(2,3,0),
		},
]]
		-- wall test
--[[
		{
			vec3(1.440093455,1.359999952,0),
			vec3(2.059906545,1.359999952,0),
			vec3(2.059906545,2.608793261,0),
			vec3(1.440093455,2.608793261,0), },
		{
			vec3(3.140093455,1.359999952,0),
			vec3(3.759906545,1.359999952,0),
			vec3(3.759906545,2.608793261,0),
			vec3(3.140093455,2.608793261,0), },
		{
			vec3(4.840093455,1.359999952,0),
			vec3(5.459906545,1.359999952,0),
			vec3(5.459906545,2.608793261,0),
			vec3(4.840093455,2.608793261,0), }
]]
	}, nil, nil, true)
		lo('?? OUT:'..tostring(out.fwhite and #out.fwhite or nil)..':'..tostring(out.fyell and #out.fyell or nil), true)
		if false then
			U.dump(out.fwhite, '?? fwhite:')
			U.dump(out.fyell, '?? fyell:')
			U.dump(out.flbl, '?? flbl:')
		end
--        out.fyell = {}
--        out.fyell = nil
		if false then
			out.fwhite,out.fyell,out.flbl = nil,nil,nil
			return true
		end

	for _,e in pairs(out.fwhite) do
		e[1] = e[1] + vec3(-265,1148,1)
		e[2] = e[2] + vec3(-265,1148,1)
	end
	if out.fyell then
		for _,e in pairs(out.fyell) do
			for i,n in pairs(e) do
				e[i] = e[i] + vec3(-265,1148,1)
			end
	--        e[1] = e[1] + vec3(-265,1148,1)
	--        e[2] = e[2] + vec3(-265,1148,1)
		end
	end
	for _,e in pairs(out.flbl) do
		e[1] = e[1] + vec3(-265,1148,1)
	end
--        U.dump(out.fyell, '?? fyell2:')

--        U.dump(out.fwhite[1], '?? fwhite:'..#out.fwhite)
--        out.avedit = {vec3(-265,1148,1)}

		if true then return end
	M.rcPave({{0,0},{10,10}}, {
		{{1,1},{4,3}},
		{{3,7},{8,9}},
		{{5,2},{9,6}},
	})

	local pth = {
		{pos=vec3(0,0,0), width=3},
		{pos=vec3(20,20,0), width=3},
		{pos=vec3(50,60,0), width=4},
		{pos=vec3(100,100,0), width=3},
	}
	for i=1,#pth do
		pth[i].pos = pth[i].pos + vec3(-250,1146,0)
	end
	D.ter2road(pth)
		lo('<<""""""""""""""""""""""""""" test:', true)
		if true then return end


	local av,af,auv,an = M.log()

	for i = 1,#av do
		av[i] = av[i] + p
	end

	local am = {}
	am[#am+1] = {
		verts = av,
		faces = af,
		normals = an,
		uvs = auv,
		material = 'WarningMaterial',
	}

	av,af,auv,an = M.log()

	for i = 1,#av do
		av[i] = av[i] + p + vec3(0,0,5)
	end
	am[#am+1] = {
		verts = av,
		faces = af,
		normals = an,
		uvs = auv,
		material = 'WarningMaterial',
	}

	meshUp(am, 'log')

	return true
end


local function regionUp()
	for i = 1,1 do
		buildingGen()
	end
end


W.forRoof = function(desctop, cb)
--    local lid = desctop
--        lo('?? forRoof:'..tostring(#desctop.achild))
	if desctop.achild and #desctop.achild > 0 then
		if desctop.cchild then
			cb(desctop.achild[desctop.cchild])
		else
			for i,c in pairs(desctop.achild) do
				if cb then cb(c) end
			end
		end
	else
		if cb then cb(desctop) end
	end
end


local function matMove(ds, isrel)
		if _dbdrag then return end
--        _dbdrag = true
--        lo('?? matMove:'..tostring(ds)..':'..tostring(indrag)..':'..tostring(scope), true)
	if scope == 'top' then
--            if true then return end
--            lo('?? for_DS:'..tostring(ds), true)
		local floor = adesc[cedit.mesh].afloor[cij[1]]
		local id,mesh
		if not isrel then
			id = floor.top.id
			if not id then return end
--                lo('?? mmove_topid:'..tostring(id))
			mesh = aedit[id].mesh
		end
--            U.dump(mesh, '?? for_MESH_top:')
			lo('?? for_DM:'..tostring(cedit.cval['DragMat'])..':'..tostring(isrel),true)
		if not cedit.cval['DragMat'] then
			if isrel then
				cedit.cval['DragMat'] = {}
				W.forRoof(floor.top, function(c)
					cedit.cval['DragMat'][#cedit.cval['DragMat']+1] = {c.uvref or {0,0}, c.uvscale or {1,1}}
				end)
			else
				cedit.cval['DragMat'] = deepcopy(mesh.uvs) -- U.clone(mesh.uvs)
				return
			end
--                lo('?? for_DM:'..#cedit.cval['DragMat'])
		end
		if isrel then
--                U.dump(cedit.cval['DragMat'], '?? uv_ROOF:'..tostring(ds)..':'..tostring(floor.top.achild))
			local n = 1
			W.forRoof(floor.top, function(c)
					lo('?? for_lid:'..n..':'..tostring(ds),true)
				c.uvref = {cedit.cval['DragMat'][n][1][1]+ds.x, cedit.cval['DragMat'][n][1][2]+ds.y}
--                U.dump(c, '??______________ for_lid:')
				n = n+1
			end)
--                U.dump(floor.top, '??^^^^^^^^^^^^^^^^^^^^ mM:')
		else
				lo('?? matMove2:',true)
			local suvs = cedit.cval['DragMat']
			local u = (floor.base[2]-floor.base[1]):normalized()
			local v = -u:cross(vec3(0,0,1))
			for i,uv in pairs(mesh.uvs) do
				uv.u = suvs[i].u + ds:dot(u)
				uv.v = suvs[i].v + ds:dot(v)
			end
	--        mesh.uvs = auv
			local obj = scenetree.findObjectById(id)
			obj:createMesh({})
			obj:createMesh({{mesh}})
	--            _dbdrag = true
		end
		return
	end

	if not cedit.cval['DragMat'] then
		cedit.cval['DragMat'] = {}
		cedit.cval['DragRef'] = {}
		forBuilding(adesc[cedit.mesh], function(w, ij)
--            cedit.cval['DragMat'][#cedit.cval['DragMat'] + 1] = U.clone(w.uv)
			cedit.cval['DragMat'][#cedit.cval['DragMat'] + 1] = aedit[w.id].mesh.uvs
			cedit.cval['DragRef'][#cedit.cval['DragRef'] + 1] = w.uvref or {0,w.pos.z*(w.uvscale and w.uvscale[2] or 1)}
		end)
--                    lo('??++++++++++++++++++ to_DragMat:'..#cedit.cval['DragMat'])
--        return
	end
--            _dbdrag = true
	local suv = cedit.cval['DragMat']
	local n = 1
--[[
	local cw = adesc[cedit.mesh].afloor[cij[1] ].awall[cij[2] ]
	if not cw then return end
	local u = cw.u:normalized()
	local v = -cw.v:normalized()
	if isrel then
		ds = u*ds.x+v*ds.y
	end
]]

--        lo('?? MM:'..tostring(u),true)
	forBuilding(adesc[cedit.mesh], function(w, ij)
--        local u = w.u:normalized()
--        local v = w.v:normalized()
		local auv = {}
		local mesh = aedit[w.id].mesh

		local u = w.u:normalized()
		local v = w.v:normalized()
		local cds = vec3(-ds.x,ds.y,ds.z)
		if isrel then
			cds = vec3(-ds.x,ds.y,ds.z)
			cds = u*cds.x+v*cds.y
		end

--[[
		w.uv[1] = suv[n][1] - ds:dot(u)
		w.uv[3] = suv[n][3] - ds:dot(u)
		w.uv[2] = suv[n][2] - ds:dot(v)
		w.uv[4] = suv[n][4] - ds:dot(v)
--            U.dump(w.agrid, '?? for_AGRID:')
		for _,xy in pairs(w.agrid) do
--                lo('?? for_XY1:'..tostring(xy[1])..':'..tostring(xy[2]), true)
--                U.dump(xy[1], '?? for_XY1:')
--                U.dump(xy[2], '?? for_XY2:')
			auv = M.uv4grid({w.uv[1], w.uv[3]}, {w.uv[2], w.uv[4]},
				xy[1], xy[2], w.u:length(), w.v:length(), auv)
		end
]]

--                U.dump(mesh.uvs, '?? for_UV:'..#mesh.verts..':'..#mesh.uvs)
--                auv = {}
		local suvs = cedit.cval['DragMat'][n]
		if not suvs then
			lo('?? NO_SUVS:'..n)
			return
		end
--            lo('?? uv_MOVE:'..tostring(cds)..':'..tostring(cds:dot(u))..':'..tostring(suvs[1].u - cds:dot(u))..':'..tostring(suvs[1].v - cds:dot(v)))
--            lo('?? duvs:'..ij[1]..':'..ij[2]..' suvs:'..#suvs..' verts:'..#mesh.verts)
		for i,p in pairs(mesh.verts) do
			auv[#auv+1] = {}
			auv[#auv].u = suvs[i].u + cds:dot(u)
			auv[#auv].v = suvs[i].v + cds:dot(v)
			w.uvref = {
				cedit.cval['DragRef'][n][1]+cds:dot(u),
				cedit.cval['DragRef'][n][2]+cds:dot(v)
			} -- + {0,w.pos.z*(w.uvscale and w.uvscale[2] or 1)}
--[[
			if true or U._PRD == 0 then
				auv[#auv].v = suvs[i].v - ds:dot(v)
			else
				auv[#auv].v = suvs[i].v + ds:dot(v)
			end
]]
		end
		if not isrel then
			mesh.uvs = auv
			local obj = scenetree.findObjectById(w.id)
			obj:createMesh({})
			obj:createMesh({{mesh}})
		end

		n = n + 1
	end)
end
--            lo('?? u'..tostring(w.u),true)
--            U.dump(w.agrid, '?? AGRID:')
--[[
		local orig = u*uext[1] + v*vext[1] -- vext[1])
		for i = #ax*#ay-1,0,-1 do
			auv[#auv + 1] = {u = (av[#av - i] + orig):dot(u), v = -(av[#av - i] + orig):dot(v)}
		end
]]


W.matScale = function(scale)
	local desc = adesc[cedit.mesh]

	if scope == 'top' then
		local floor = desc.afloor[cij[1]]
		if not cedit.cval['DragMat'] then
			cedit.cval['DragMat'] = {}
			W.forRoof(floor.top, function(c)
				cedit.cval['DragMat'][#cedit.cval['DragMat']+1] = {c.uvref or {0,0}, c.uvscale or {1,1}}
			end)
                lo('??__________________ to_MM:',true)
			matMove(vec3(0,0),true)
		end
		local n = 1
--        U.dump(cedit.cval['DragMat'], '?? DM:')
		W.forRoof(floor.top, function(c)
			c.uvscale = {cedit.cval['DragMat'][n][2][1]*scale[1], cedit.cval['DragMat'][n][2][2]*scale[2]}
--        U.dump(c.uvscale,'?? matScale:'..n..':'..scale[1]..':'..scale[2])
			n = n+1
		end)
--        floor.dirty
		return
	end

	if not cedit.cval['DragMat'] then
		cedit.cval['DragMat'] = {}
		local n = 1
		if scope == 'top' then
			-- TODO: make child-wise
			cedit.cval['DragMat'][1] = desc.afloor[cij[1]].top.uvsclae or {1,1}
		else
			forBuilding(desc, function(w, ij)
				cedit.cval['DragMat'][n] = w.uvscale or {1,1}
				n = n+1
			end)
		end
	end
	local n = 1
--        U.dump(cedit.cval['DragMat'], '?? matScale:'..tostring(scale[1])..':'..tostring(scale[2]))
	if scope == 'top' then
		desc.afloor[cij[1]].top.uvsclae = {cedit.cval['DragMat'][1][1]*scale[1],cedit.cval['DragMat'][1][2]*scale[2]}
	else
		forBuilding(adesc[cedit.mesh], function(w, ij)
			w.uvscale = {cedit.cval['DragMat'][n][1]*scale[1],cedit.cval['DragMat'][n][2]*scale[2]}
			n = n+1
			if desc.afloor[ij[1]].top.border then
				desc.afloor[ij[1]].top.border.uvscale = desc.afloor[ij[1]].awall[1].uvscale or {1,1}
			end
		end)
	end
end


local function matMove_(v, idobj)
	if _dbdrag then return end
--    _dbdrag = true
--    indrag = true
--        lo('>> matMove:'..tostring(v)..':'..idobj)
--        U.dump(aedit[idobj].desc, '?? desc:')
--    U.dump(aedit[idobj].mesh, '?? mesh:')
	if aedit[idobj] == nil then
		lo('!! ERR_matMove:'..idobj)
		return
	end
	-- transform UVs
	local desc = aedit[idobj].desc
	local mesh = aedit[idobj].mesh
	if #cedit.cval == 0 then
--        U.dump(desc, '?? reNEW:'..idobj)
		-- TODO: for roof
		if desc.uv ~= nil then
			cedit.cval = {desc.uv[1], desc.uv[2], desc.uv[3], desc.uv[4]}
		end
	end
	local cuv = cedit.cval
--    if not indrag then
--        cuv = {desc.uv[1], desc.uv[2], desc.uv[3], desc.uv[4]}
--        indrag = true
--    end
	if not desc.u or not desc.v then return end
	local x = v:dot(desc.u:normalized())/desc.u:length()
	local y = v:dot(desc.v:normalized())/desc.v:length()
--            lo('?? xy:'..x..':'..y)

	desc.uv[1] = cuv[1] - (cuv[3] - cuv[1])*x
	desc.uv[3] = cuv[3] - (cuv[3] - cuv[1])*x
	desc.uv[2] = cuv[2] + (cuv[4] - cuv[2])*y
	desc.uv[4] = cuv[4] + (cuv[4] - cuv[2])*y

--[[
]]

--    desc.uv[1] = desc.uv[1] + 0.5
--    desc.uv[3] = cuv[3] - (cuv[3] - cuv[1])*x
--    desc.uv[2] = cuv[2] + (cuv[4] - cuv[2])*y
--    desc.uv[4] = cuv[4] + (cuv[4] - cuv[2])*y


--        _dbdrag = true

	-- rebuild uvs
	local auv = {}
	for _,xy in pairs(desc.agrid) do
		auv = M.uv4grid({desc.uv[1], desc.uv[3]}, {desc.uv[2], desc.uv[4]},
			xy[1], xy[2], desc.u:length(), desc.v:length(), auv)
	end
	mesh.uvs = auv
--        U.dump(auv, '?? AUV:'..#auv..':'..#desc.agrid..':'..#desc.agrid[1][1])
--    mesh.uvs = M.uv4grid({desc.uv[1], desc.uv[3]}, {desc.uv[2], desc.uv[4]}, desc.ax, desc.ay)

--[[
	mesh.uvs = {
		{u = desc.uv[1], v = desc.uv[4]},
		{u = desc.uv[1], v = desc.uv[2]},
		{u = desc.uv[3], v = desc.uv[2]},
		{u = desc.uv[3], v = desc.uv[4]},
	}
]]
--    aedit[idobj]
	local obj = scenetree.findObjectById(idobj)
--    obj:createMesh({{{}}})
	obj:createMesh({})
	obj:createMesh({{mesh}})
end


-- toggle ij presence in buf
local function bufToggle(buf, ij, yes)
	for i,r in pairs(buf) do
		for j,c in pairs(r) do
			for k,cij in pairs(c) do
				if ij[1] == cij[1] and ij[2] == cij[2] then
					if not yes then
						table.remove(c,k)
						return false
					end
				end
			end
		end
	end
	if yes ~= false then
		if not buf[ij[1]] then
			buf[ij[1]] = {}
		end
		if not buf[ij[1]][ij[2]] then
			buf[ij[1]][ij[2]] = {}
		end
		buf[ij[1]][ij[2]][#buf[ij[1]][ij[2]]+1] = ij
	end
	return true
end


local function cornersBuild(desc, dae)
	if not dae then dae = daePath['corner'][1] end
	local acorner = {}
	forBuilding(desc, function(w, ij)
		if desc.basement.yes and ij[1]==1 then
			return
		end
		local v = desc.afloor[ij[1]].base[ij[2]]
		local hit
		for i,s in pairs(acorner) do
			if (s.pos-U.proj2D(v)):length() < small_dist then
				acorner[i].list[#acorner[i].list+1] = ij
				acorner[i].list[#acorner[i].list].dae = acorner[i].list[1].dae
				hit = true
			end
		end
		if not hit then
			local base = desc.afloor[ij[1]].base
			local u1,u2 = base[ij[2]]-U.mod(ij[2]-1,base), U.mod(ij[2]+1,base)-base[ij[2]]
          lo('??^^^^^^^^^^^^ cB.for_ang:'..ij[2]..':'..U.vang(u1,u2),true)
			if math.abs(math.abs(U.vang(u1,u2)-math.pi/2)) < small_ang and u1:cross(u2).z > 0 then
				acorner[#acorner+1] = {pos = U.proj2D(v), list = {ij}}
				acorner[#acorner].list[1].dae = dae
			else
				-- TODO: concave
			end
--[[
			dae = daePath.corner[idae]
			local ang = U.vang(u1,u2) % (math.pi/2)
--                lo('?? for_ang:'..ij[1]..':'..ij[2]..':'..ang..':'..tostring(dae), true)
			if not (math.abs(ang) < small_ang or math.abs(1-ang) < small_ang) then
				dae = nil
			elseif not (math.abs(U.vang(u1,u2)-math.pi/2) < small_ang and u1:cross(u2).z > 0) then
					lo('??______________________ concave:'..ij[1]..':'..ij[2])
				ind,dae = daeIndex('corner', dae, 'in')
			end
]]
		end
	end)
--        U.dump(acorner, '<< cornersBuild_:')
	return acorner
end


--[[
local function cornersBuild(desc, idae)
	local ijskip = {}
	local acorner = {} --desc.acorner or {}
	local yes
	local dae,ind --= daePath.corner[out.curselect]
	forBuilding(desc, function(w, ij)
		if yes == nil then
			yes = bufToggle(acorner,ij)
--                U.dump(acorner, '?? if_YES:'..tostring(yes))
		else
			if #U.index(ijskip, U.stamp(ij,true)) > 0 then return end
			local base = desc.afloor[ij[1] ].base
			local p = base[ij[2] ]
			local u1,u2 = base[ij[2] ]-U.mod(ij[2]-1,base), U.mod(ij[2]+1,base)-base[ij[2] ]
			if math.abs(U.vang(u1,u2)-math.pi/2) < small_ang and u1:cross(u2).z > 0 then
				bufToggle(acorner,ij,yes)
			end
		end
		if yes == false then
			bufToggle(acorner,ij,false)
		else
			if #U.index(ijskip, U.stamp(ij,true)) > 0 then return end
			local base = desc.afloor[ij[1] ].base
			local p = base[ij[2] ]
			local u1,u2 = base[ij[2] ]-U.mod(ij[2]-1,base), U.mod(ij[2]+1,base)-base[ij[2] ]
			dae = daePath.corner[idae]
			local ang = U.vang(u1,u2) % (math.pi/2)
--                lo('?? for_ang:'..ij[1]..':'..ij[2]..':'..ang..':'..tostring(dae), true)
			if not (math.abs(ang) < small_ang or math.abs(1-ang) < small_ang) then
				dae = nil
			elseif not (math.abs(U.vang(u1,u2)-math.pi/2) < small_ang and u1:cross(u2).z > 0) then
					lo('??______________________ concave:'..ij[1]..':'..ij[2])
				ind,dae = daeIndex('corner', dae, 'in')
			end
				lo('?? for_ang2:'..tostring(dae))
			if dae then
				if not acorner[ij[1] ] then
					acorner[ij[1] ] = {}
				end
				acorner[ij[1] ][ij[2] ] = {ij}
				acorner[ij[1] ][ij[2] ].dae = dae
				ijskip[#ijskip+1] = U.stamp(ij,true)
				-- build corner
				forBuilding(desc, function(tw, tij)
	--                    U.dump(ijskip, '?? if_SKIP:'..tij[1]..':'..tij[2]..':'..tostring(yes))
					if #U.index(ijskip, U.stamp(tij,true)) > 0 then return end
					if U.proj2D(base2world(desc, ij) - base2world(desc,tij)):length() < small_dist then
						ijskip[#ijskip+1] = U.stamp(tij,true)
						acorner[ij[1] ][ij[2] ][#acorner[ij[1] ][ij[2] ]+1] = tij -- ijskip[#ijskip]
					end
				end)
			end
		end
	end)
	return acorner
end
]]


local function meshApply(tp, ind) -- ind is 0-based
		lo('>> meshApply:'..tp..':'..#daePath[tp]..':'..tostring(ind)..':'..tostring(cedit.forest)..':'..tostring(cedit.fscope)..':'..tostring(out.inhole), true)
--        U.dump(dforest[cedit.forest], '?? cfor')
	if ind+1 > #daePath[tp] then
		return false
--        ind = #daePath[tp]-1
	end
	-- get index in df
	local iforest, dae, desc
--        iforest = 1
	if out.inhole then
		local child = out.inhole.achild[out.inhole.achild.cur or #out.inhole.achild]
		tp = ddae[child.body].type
		dae = daePath[tp][ind]
--                U.dump(out.inhole.achild, '?? achild:'..tostring(tp)..':'..child.body..':'..dae)
		child.body = dae
--[[
		if true then return end
--        ddae[]
		child.body = dae
]]
--            return
--        out.inhole.achild[out.inhole.achild.cur].body = dae
	else
		tp = dforest[cedit.forest].type
		dae = daePath[tp][ind+1]
--            U.dump(cij, '?? meshApply:'..tp..':'..ind..':'..tostring(cedit.forest-1)..':'..tostring(dae))
		if tp == 'corner' then
			desc = adesc[cedit.mesh]
--                U.dump(desc.acorner,'?? for_CORNER:'..tostring(desc.df)..':'..tostring(cedit.forest))
--                lo('?? ifforr11:'..tostring(iforest))
--            desc.acorner = cornersBuild(desc, ind)
--                    U.dump(desc.df, '?? if_F:')
--                    forestClean(desc, true)
--                    desc.acorner_ = nil
			if dae then
				W.cornerUp(desc, dae, true)
			else
				lo('!! ERR_meshApply_NO_DAE:')
			end
		else
			forBuilding(adesc[cedit.mesh], function(w, ij)
		--        U.dump(w.df, '?? for wall:'..ij[1]..':'..ij[2]..':'..cedit.forest)
		--        U.dump(w.df, '??_+__ DF:'..cedit.forest..':'..tostring(cedit.forest == w.df[dae]))
				if iforest then return end
				for d,list in pairs(w.df) do
					iforest = U.index(w.df[d], cedit.forest)[1]
					if iforest then break end
				end
		--        iforest = U.index(w.df[dae], cedit.forest)[1]
			end)
--                lo('?? iforest2:'..tostring(iforest),true)
			if not iforest then
				lo('!! ERR_NO_FOREST_INDEX:'..tostring(cedit.forest)..':'..tostring(dae))
				return
			end

			local ci = 0
			forBuilding(adesc[cedit.mesh], function(w, ij)
				if tp == 'storefront' then
--                        U.dump(w, '?? for_SF:'..tostring(cedit.forest)..':'..tostring(tp),true)
					local daepre = U.forOrig(dforest[cedit.forest].item:getData():getShapeFile())
--                        lo('?? dae_PRE:'..tostring(daepre)..':'..iforest)
					local ind = U.index(w[tp].adae, daepre)[iforest]
					w[tp].adae[ind] = dae
					w.storefront.len = w.storefront.len - ddae[daepre].len + ddae[dae].len
--                    U.dump(cedit.forest, '?? for_SF:')
--                    local dae = w[tp].adae
				elseif ({win=1,door=1})[tp] then
--            U.dump(w.df,'?? mA_wd:'..ij[1]..':'..ij[2]..':'..tostring(cedit.fscope)..':'..tostring(cedit.mesh))
--                else
              lo('?? meshApply:'..tostring(cedit.fscope),true)
          if cedit.fscope == 1 then
            if not w[tp..'_inf'] then
              w[tp..'_inf'] = {ddae = {}}
            end
            if not w[tp..'_inf'].ddae then
              w[tp..'_inf'].ddae = {}
            end
            local ind = dforest[cedit.forest].ind -- U.index(w.df[tp],cedit.forest)[1]
            if ind then
              w[tp..'_inf'].ddae[ind] = dae
            end
          else
            w[tp..'_inf'] = nil
  					w[tp] = dae -- daePath[tp][ind]
--              U.dump(w[tp..'_inf'], '?? ddae:')
            if tp == 'win' and ddae[w[tp]] then w.winbot = ddae[w[tp]].fr.z end
            if tp == 'door' then
                lo('?? for_DAE:'..w[tp]..':'..tostring(w.doorind))
              w.doorwidth = ddae[w[tp]].to.x - ddae[w[tp]].fr.x + 0.2
              if ij[1] == 2 and w.doorstairs and w.doorind then
                -- adjust basement height
  --                                U.dump(w.doorstairs[w.doorind],'?? for_stairs:'..tostring(w.doorstairs[w.doorind]))
                adesc[cedit.mesh].afloor[1].h = math.abs(ddae[w.doorstairs[w.doorind].dae].fr.z) + ddae[dae].fr.z
              end
            end
            if ij[1] > ci then
				if not (adesc[cedit.mesh].basement and adesc[cedit.mesh].basement.yes and ij[1] == 1) then
					-- update height
					local floor = adesc[cedit.mesh].afloor[ij[1]]
					local hmax = 0
					for _,w in pairs(floor.awall) do
		--                            local dae = w[tp]
						if dae ~= nil then
						if type(dae) == 'table' then
							--TODO: handle better
							if dae.dae then
							dae = dae.dae
							else
							dae = dae[1]
							end
						end
		--                                    U.dump(ddae[dae], '??^^^^^^^^^^^^^^^^ meshApply2:'..tp..':'..tostring(dae))
						if dae then
							if ddae[dae].to.z > hmax then
							hmax = ddae[dae].to.z
							end
						else
							U.dump(w, '!! NO_DAE_FOR_WALL:'..tp)
						end
						end
					end
					if floor.h < hmax then floor.h = hmax + 0.1 end
				end
            end
--            w[tp] = dae
						ci = ij[1]
			--            floor.update = true
					end
				elseif tp == 'stairs' then
					if w.doorind then
						if not w.doorstairs then
								lo('?? doorstairs_NEW:',true)
							w.doorstairs = {}
							w.doorstairs[w.doorind] = {top=0}
						end
						w.doorstairs[w.doorind].dae = dae -- = {dae = dae, top = 0}
						if ij[1] == 2 then
							-- adjust basement height
							adesc[cedit.mesh].afloor[1].h = math.abs(ddae[dae].fr.z)
						end
					end
				else
					if not w[tp] then w[tp] = {} end
					w[tp].dae = dae
				end
		--            U.dump(ddae[w[tp]], '??___________ meshApply:'..w[tp])
		--        w.update = true
			end)
		end
--			lo('?? iforest:'..tostring(iforest)..':'..tostring(cij[1])..':'..tostring(cij[2])) --..':'..dae)
		if tp == 'roofborder' then
			lo('?? meshApply_for_roof:'..tp)
		end

	end
--    houseUp(nil, cedit.mesh)
	houseUp(adesc[cedit.mesh], cedit.mesh, true)
--       U.dump(adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]].df,'?? after_HU:'..tostring(cedit.forest)..' ifor:'..iforest)
--        lo('?? iff333:'..tostring(iforest)..':'..tostring(tp))
	if tp == 'corner' then
		cedit.forest = desc.df[dae][1]
	elseif iforest and cedit.fscope ~= 1 then
		if adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]].df[dae] then
			cedit.forest = adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]].df[dae][iforest]
		else
			lo('!! ERR_NO_DF_DAE:'..tostring(dae),true)
		end
	end
	markUp(dae)
	out.curmselect = ind
	toJSON(adesc[cedit.mesh])
--        U.dump(adesc[cedit.mesh].df, '?? for_DF:')
--        lo('<< meshApply:'..ind..':'..tostring(cedit.forest)..':'..tostring(out.inhole)..':'..tostring(W.ifForest()), true) -- tostring(dforest[cedit.forest].item))
--        W.ui.dbg = true
--            indrag = false
end


local function matApply(nm, ind)
		lo('>> matApply:'..tostring(nm)) --..':'..cedit.forest) --..':'..ind..':'..tostring(cmesh))
--    if not nm then return end
	if U._PRD == 1 and ind == 0 and scope ~= 'top' then
--    if ind == 1 then
		ind = nil
--    if not nm then
		nm = out.defmat -- 'm_greybox_base' --'WarningMaterial'
	end
	out.curselect = ind

--            if ind == 1 then return end
--[[
	if #U.index(dmat.wall,nm) == 0 then
		lo('!! NO_MAT:'..tostring(nm))
		return
	end
]]
	if U._PRD==0 and D and D.forRoad() then
		D.matApply(nm, ind)
		return
	end
--        if true then return end
	local desc = adesc[cedit.mesh]
	if cmesh ~= nil then
		if #dmesh[cmesh].sel == 0 then
			for _,m in pairs(dmesh[cmesh].data) do
				m.material = nm
			end
			dmesh[cmesh].obj:createMesh({})
			dmesh[cmesh].obj:createMesh({dmesh[cmesh].data})
		else
			lo('?? for selection:')
			for _,m in pairs(dmesh[cmesh].sel) do
				m.material = nm
			end
			M.update(dmesh[cmesh])
		end
--            U.dump(dmesh[cmesh].sel, '?? matApply_for_mesh:'..tostring(#dmesh[cmesh].sel))
		return
	end
		lo('>>+++++++ matApply:'..tostring(nm)..':'..tostring(scope)..':'..tostring(cedit.mesh)..':'..tostring(cedit.part)..':'..tostring(cedit.forest), true) --..':'..#desc.afloor)
--    if cedit.forest ~= nil then return end
	if false and nm == 'm_bricks_01' then
--    if nm == 'm_stucco_scraped_BAT' then
		local currentMaterial = scenetree.findObject(nm)
			lo('?? if_CHANNELS:'..tostring(currentMaterial.activeLayers)..':'..tostring(currentMaterial.normalMap)..':'..tostring(currentMaterial.normalMapUseUV))
		currentMaterial.normalMapUseUV = 1
			local info = currentMaterial:getFieldInfo('normalMap', 1)
			for k,v in pairs(info) do
--                lo('?? KV:'..tostring(k)..':'..tostring(v))
			end
			lo('?? for_ID:'..tostring(desc.afloor[2].awall[2].id))
			local wo = scenetree.findObjectById(desc.afloor[2].awall[2].id)
			wo:delete()
--            wo:createMesh({{{}}})
--            U.dump(desc.afloor[2].awall[2],'?? DONE:')
--            U.dump(info, '?? for_INF:'..tostring(info))
--            lo(''..tostring(currentMaterial:getField("normalMap", 0)))
--            U.dump(currentMaterial.normalMap, '?? for_NM:')
--        local fields = currentMaterial:getFields()
--            lo('?? for_FIELDS:'..#fields)
--        local normalMapObj = out.imUtils.texObj(currentMaterial:getNormalMap())
--            dump(currentMaterial)
--            U.dump(currentMaterial, '?? NMO"')
		currentMaterial.activeLayers = 1
--        return
	end
	forBuilding(desc, function(w, ij)
		if w.id == nil then return end
		if scope == 'top' then
--            lo('?? matApply_b:'..ij[1]..':'..ij[2]..':'..scope..':'..cedit.part)
		else
			if nm ~= nil then w.mat = nm end
			if nm ~= 'm_transparent_glass_window' then w.hidden = false end
			w.update = true
--                lo('?? matApply_upd:'..ij[1]..':'..ij[2])
		end
	end)
	if scope == 'top' then
		local floor = desc.afloor[cij[1]]
--            lo('?? for_roof:'..tostring(cedit.part)..':'..cij[1]..':'..tostring(floor.top.cchild)..'/'..#floor.top.achild)
		local desctop
		if floor.top.cchild ~= nil then
			floor.top.achild[floor.top.cchild].mat = nm
			desctop = floor.top.achild[floor.top.cchild]
		elseif #floor.top.achild > 0 then
			for _,c in pairs(floor.top.achild) do
				c.mat = nm
			end
			desctop = floor.top.achild[1]
			floor.top.mat = nm
		else
			floor.top.mat = nm
			desctop = floor.top
		end
		floor.update = true
		if floor.top.roofborder and cij[1] == #desc.afloor then
			-- border material
			local rbdae = floor.top.roofborder[1]
			if rbdae then
				M.matSet(U.forBackup(rbdae), desctop.mat, floor.top.roofborder[2])  --, ddae[rbdae].xml)
			end
--            M.matSet(U.forBackup(rbdae), floor.top.mat, floor.top.roofborder[2])  --, ddae[rbdae].xml)
		end
	end
--[[
	for _,f in pairs(desc.afloor) do
		for o,w in pairs(f.awall) do
			w.mat = nm
			w.update = true
			lo('?? in_aedit:'..tostring(aedit[w.id].desc.mat))
		end
	end
]]
	houseUp(desc, cedit.mesh) --, true)
	markUp()
	toJSON(adesc[cedit.mesh])
end


local function scopeOn(s)
		lo('>> scopeOn:'..tostring(s)..':'..tostring(cedit.forest))
	out.inroad = nil
	local sscope = scope
	scope = s
	out.scope = scope
	out.curmselect = false
	cedit.forest = nil
	cedit.fscope = 0
	if scope == 'top' then
--[[
		local mat = adesc[cedit.mesh].afloor[cij[1] ].top.mat
		for i,m in pairs(dmat.roof) do
--                lo('?? for_mat:'..tostring(m))
			if tostring(m) == mat then
				out.curselect = i-1
				break
			end
		end
]]

		local top,base = forTop()
		if base then
			local icc
			forBase(base, function(v1, v2, ind)
				if icc ~= nil then return end
				if (v1:cross(v2)).z < 0 then
					icc = ind
				end
			end)
			top.isconvex = icc == nil
			base = U.polyStraighten(base)
			local apair,amult = T.pairsUp(base)
			for _,m in pairs(amult) do
				if #m > 1 then
					top.ismult = true
					break
				end
			end
				lo('?? scopeOn_top:'..tostring(top.ismult))
			top.isridge = T.forRidge(adesc[cedit.mesh].afloor[cij[1]],base,nil,nil,true)
		end

--            lo('?? scopeOn:'..mat..':'..#dmat.roof)
--        out.curselect = U.index(dmat.roof, mat)[1]
--        adesc[cedit.mesh].afloor[cij[1]].top.cchild = nil
	elseif scope == 'side' then
		W.ui.ifpilaster = true
		local desc = adesc[cedit.mesh]
		local aij = forSide(cij)
--[[
		if not aij then
			scope = sscope
			return
		end
]]
		if aij then
			local len,pos
			for i,r in pairs(aij) do
				for j,n in pairs(r) do
					if desc.afloor[i].awall[n].doorind then
						W.ui.ifpilaster = false
						break
					end
					if not len then
						len = desc.afloor[i].awall[n].u:length()
						pos = U.proj2D(desc.afloor[i].pos + desc.afloor[i].awall[n].pos)
	--                        lo('?? w_pos:'..tostring(desc.afloor[i].awall[n].pos))
					else
						lo('?? if_DOOR:'..i..':'..n..':'..tostring(desc.afloor[i].awall[n].doorind))
						if math.abs(len - desc.afloor[i].awall[n].u:length()) > small_dist then
							W.ui.ifpilaster = false
							break
						elseif math.abs((pos - U.proj2D(desc.afloor[i].pos + desc.afloor[i].awall[n].pos)):length()) > small_dist then
							W.ui.ifpilaster = false
							break
	--                    elseif desc.afloor[i].awall[n].doorind then
	--                        W.ui.ifpilaster = false
	--                        break
						end
					end
				end
			end

		end

--        lo('?? scopeOn.part'..tostring(cedit.part)..':'..tostring(adesc[cedit.mesh].afloor[#adesc[cedit.mesh].afloor].top.id))
--        cedit.part = adesc[cedit.mesh].afloor[#adesc[cedit.mesh].afloor].top.id
	elseif scope == 'building' and adesc[cedit.mesh] and adesc[cedit.mesh].basement then
		W.ui.basement_toggle = adesc[cedit.mesh].basement.yes
	end
  -- select material
  if true and cedit.mesh and cij and cij[1]<#adesc[cedit.mesh].afloor then
    local dae,imat
    if scope == 'top' then
      local desctop = forTop()
      if desctop then
        dae = desctop.mat
        imat = U.index(dmat.roof, dae)[1]
--            U.dump(dmat.roof, '?? mroof:'..tostring(dae)..':'..tostring(imat))
      end
    elseif cij[2] then
          lo('?? for_cij:'..tostring(cij[1])..':'..tostring(cij[2]))
      dae = adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]].mat
--        U.dump(dmat.wall, '?? mat_wall:'..dae)
      imat = U.index(dmat.wall, dae)[1]
    else
      dae = adesc[cedit.mesh].afloor[cij[1]].awall[1].mat
--        U.dump(dmat.wall, '?? mat_floor:'..dae)
      imat = U.index(dmat.wall, dae)[1]
    end
    if imat then
      out.curselect = imat-1
    end
--      U.dump(cij, '?? sO:'..tostring(dae)..':'..tostring(scope)..':'..tostring(imat))
--    out.curselect = daeIndex('door', dae)
  end
	markUp()
		lo('<< scopeOn:'..tostring(out.curselect)..':'..tostring(cedit.forest)..':'..tostring(cedit.fscope))
--            U.dump(aedit[cpart].desc.ij, '?? scopeOn_ij:')
end


local function isHidden()
			if true then return false end
	local yes = true
	if not adesc[cedit.mesh] then return end
	forBuilding(adesc[cedit.mesh], function(w)
		if not w.hidden then
			yes = false
		end
	end, true)
	return yes
end


local function selectionHide()
		lo('?? selectionHide:'..tostring(scope)..':'..tostring(cedit.forest))
			out.ccommand = ''
			if true then return end
	if cedit.forest then
--        dforest[cedit.forest].mesh
		local dsc = dforest[cedit.forest].prn
		local dae = dforest[cedit.forest].item:getData():getShapeFile()
		for k,key in pairs(dsc.df[dae]) do
			if key == cedit.forest then
				if not dsc.df[dae].skip then
					dsc.df[dae].skip = {}
				end
				U.push(dsc.df[dae].skip, k)
			end
		end
			U.dump(dsc.df[dae].skip,'?? for_skip:') --..dforest[cedit.forest].item:getData():getShapeFile())
--[[
		local df = dsc.df[dforest[cedit.forest].]
		for _,key in
			U.dump(dforest[cedit.forest], '?? for_key:')
		local desc = adesc[dforest[cedit.forest].mesh]
		local cdf = dforest[cedit.forest].prn.df[dforest[cedit.forest].prn[] ]
		if not cdf.skip then
			cdf.skip = {}
		end
		U.push(cdf.skip, cedit.forest, true)
				lo('?? for_skip:'..#cdf.skip)
]]
	else
		local ishidden = isHidden()
		forBuilding(adesc[cedit.mesh], function(w)
			if ishidden then
				w.hidden = false
				w.mat = w.smat
			else
				w.hidden = true
				w.smat = w.mat
			end
		end)
		matApply(not ishidden and 'm_transparent_glass_window' or nil)
	end
end


local function partOn(id)
--    cpart = id
	cedit.part = id
	if id == nil then return end
--[[
	if aedit[id].desc.type == 'cover' then
		lo('?? partOn:'..id..':'..'cover')
	else
	end
]]
	if aedit[id] == nil then
		lo('!! ERR_partOn.NOPART:'..id)
		return
	end
	cij = U.clone(aedit[id].desc.ij)
		U.dump(cij, '>> partOn:'..tostring(id)..':'..tostring(aedit[id]))

--            lo('>> partOn:'..tostring(id)..' i:'..tostring(cij[1]))
--            U.dump(aedit[id].desc, '?? DESC:')
	if aedit[id].desc.type == 'top' and scope ~= 'top' then
		scopeOn('top')
	else
--        lo('?? MUP:1')
--        markUp()
	end
	markUp()
	return cij
--   U.dump(aedit[id], '>> partOn:'..id)
end


local function objEditStop()
	lo('>> objEditStop:'..tostring(cedit.mesh))
	scenetree.findObject('edit'):deleteAllObjects()
--    lo('?? for_edit:'..tostring(cedit.mesh))
	if cedit.mesh ~=nil then
		houseUp(adesc[cedit.mesh], cedit.mesh)
	end
	aedit = {}
end


local function forestEdit(key)
--    scenetree.findObject('edit'):deleteAllObjects()
--        U.dump(dforest, '?? forestEdit:'..tostring(key))
	local meshid = dforest[key] and dforest[key].mesh or nil
	if meshid == nil then
		meshid = cedit.mesh
	end
--            U.dump(dforest[key], '?? for_mesh:')
	local desc = adesc[meshid]
			lo('>> forestEdit:'..key..':'..tostring(dforest[key].type)..':'..tostring(meshid)..':'..tostring(desc)) --..':'..tostring(dforest[key])..':'..tostring(dforest[key].item))
--    cij = nil
	cedit = {mesh = meshid, forest = key, cval = {}}
	forBuilding(desc, function(w, ij)
		for _,akey in pairs(w.df) do
			if #U.index(akey, key) > 0 then
				cij = U.clone(ij)
				break
			end
		end
	end)
		U.dump(cij, '<< forestEdit:'..key)
end


local aint = {} -- top cutting line


local function topSplit()
	local floor = adesc[cedit.mesh].afloor[cij[1]]
		U.dump(aint,'>>__________****** topSplit:'..cij[1]..':'..#aint)--..':'..floor.stop.mat)
	if #aint == 0 then return end
	local base = floor.base
			lo('?? ifchild:'..tostring(floor.top.cchild)..':'..tostring(#floor.top.achild))
	local mat = floor.top.mat
	local tip = floor.top.tip
	local margin = floor.top.margin
	local fat = floor.top.fat
	if floor.top.cchild then
		local c = floor.top.achild[floor.top.cchild]
		base = c.base
--            U.dump(floor.top.achild, '?? for_ACHILD:'..tostring(floor.top.cchild)..':'..tostring(#floor.top.achild))
		mat = c.mat
		tip = c.tip
		margin = c.margin
		fat = c.fat
		if c.id then
			scenetree.findObjectById(c.id):delete()
		end
		table.remove(floor.top.achild, floor.top.cchild)
	else
		floor.top.achild = {}
		floor.top.cchild = 1
	end
	tip = tip/2
--        U.dump(base, '?? for_base:'..#base)
	local achunk,amap = U.polyCut(base, aint[1], aint[2])
--        U.dump(achunk, '?? for_CHUNKS:'..#achunk)
--        U.dump(amap, '?? for_MAP:'..#amap)
  if achunk then
    for i,c in pairs(achunk) do
      table.insert(floor.top.achild, floor.top.cchild, {
        --list = {},
        shape = 'flat', istart = 1,
        imap = amap[i],
  --            list = amap[i], shape = 'flat', istart = 1,
        mat = mat, margin = margin, tip = tip, fat = fat,
        base = c, aside = {} --aside2
      })
    end
  end
		lo('??******************************************************** POST_split:'..floor.top.cchild..'/'..#floor.top.achild..':'..tostring(fat))
--            U.dump(floor.top.achild, '??************************************************************* achild:'..#floor.top.achild)
--            if true then return end
--    floor.update = true
--            W.ui.dbg = true
--                U.dump(floor.top.achild, '?? ACHILD:'..#floor.top.achild)
--!!    houseUp(adesc[cedit.mesh], cedit.mesh)
	for ic,wp in pairs(floor.awplus) do
		if wp.id then
			scenetree.findObjectById(wp.id):delete()
		end
	end
	floor.awplus = nil
	houseUp(adesc[cedit.mesh], cedit.mesh, true)
	toJSON(adesc[cedit.mesh])

--[[
]]
			if true then return end

	local jcut
	local avclose = {}
	local rmatch = 0.6
	local cvHit1,cvHit2 = nil,nil
	local bpos = floor.pos + adesc[cedit.mesh].pos + vec3(0, 0, aint[1][1].z) -- basic outer position
	for j,v in pairs(floor.base) do
		local p = v + bpos
		p.z = aint[1][1].z
		local conc = (base[(j + 0) % #base + 1] - base[(j - 1) % #base + 1]):cross(base[(j - 1) % #base + 1] - base[(j - 2) % #base + 1]).z
--        lo('?? for_dist:'..j..':'..conc) --..p:distanceToLine(aint[1][1], aint[2][1])) --..tostring(aint[1])..':'..tostring(aint[2])..':'..tostring(p)..':'..(aint[1] - p):length()..':'..(aint[2] - p):length())
--            lo('?? if_CLOSE:'..j..':'..tostring(p)..':'..tostring(aint[1][1])..':'..tostring(aint[2][1]))
		if p:distanceToLineSegment(aint[1][1], aint[2][1]) < rmatch then
--            lo('?? for_dist:'..j..':'..tostring((p - aint[1][1]):length())..':'..tostring((p - aint[2][1]):length()))
			if (p - aint[1][1]):length() < rmatch then
				avclose[#avclose + 1] = {j, -1}
				cvHit1 = j
				aint[1][2] = j
			elseif (p - aint[2][1]):length() < rmatch then
				avclose[#avclose + 1] = {j, 1}
				cvHit2 = j
				aint[2][2] = j
			else
				avclose[#avclose + 1] = {j, 0}
			end
		end
	end
		U.dump(avclose, '?? CLOSE:'..tostring(cvHit1)..':'..tostring(cvHit2))
--    lo('?? jcut:'..tostring(jcut))

	-- cutting vertices position
	local cut = {}
	if cvHit1 == nil then
		cut[#cut + 1] = {aint[1][2], aint[1][1] - bpos, false} -- false/true: not_at/at vertex
	elseif (base[(cvHit1 + 0) % #base + 1] - base[(cvHit1 - 1) % #base + 1]):
		cross(base[(cvHit1 - 1) % #base + 1] - base[(cvHit1 - 2) % #base + 1]).z > 0 then
		cut[#cut + 1] = {aint[1][2], base[cvHit1], true}
	end
	for k,v in pairs(avclose) do
		if v[2] == 0 then
			cut[#cut + 1] = {v[1], base[v[1]], true}
		end
		if #cut == 2 then
			break
		end
	end
	if #cut == 1 and not cvHit2 then
		local x,y = U.lineCross(base[aint[2][2]], base[aint[2][2] % #base + 1], cut[1][2], cut[1][2] + (aint[1][1] - aint[2][1]))
--        local x,y = U.lineCross(base[aint[2][2]], base[aint[2][2] % #base + 1], base[cut[1][1]], base[cut[1][1]] + (aint[1][1] - aint[2][1]))
		cut[#cut + 1] = {aint[2][2], vec3(x, y, 0), false}
--        cut[#cut + 1] = {aint[2][2], aint[2][1] - bpos, false}
	end
--        out.avedit = {cut[1][2] + bpos, cut[2][2] + bpos}
	table.sort(cut, function(a, b)
		return a[1] < b[1]
	end)
		lo('?? for_cut:'..#cut)
--        U.dump(cut, '?? CUT:'..#cut)
--        if true then return end

	if #cut ~= 2 then
		U.dump(cut,'!! ERR_CUT:'..#cut)
		return
	end
			out.avedit = {
				cut[1][2]+bpos,
				cut[2][2]+bpos
			}
	local jb, je = cut[2][1], cut[1][1]
	local b1,b2 = {cut[2][2]},{cut[2][2]}
	local aside1 = {jb}
--    b1[#b1 + 1] = cut[2][2]
	for i = jb + 1,jb + #base do
		local ind = (i - 1)%#base + 1
--        lo('?? for_ind:'..ind)
		if ind == je then
			lo('?? for_last:'..tostring(cut[1][3])..':'..#b1)
			if not cut[1][3] then
				b1[#b1 + 1] = base[ind]
				aside1[#aside1 + 1] = ind
				b1[#b1 + 1] = cut[1][2]
				aside1[#aside1 + 1] = cut[1][1]
			elseif U.vang(b1[1] - cut[1][2], cut[1][2] - b1[#b1]) > 0.1 then
				b1[#b1 + 1] = cut[1][2]
				aside1[#aside1 + 1] = cut[1][1]
			end
			break
		end
		b1[#b1 + 1] = base[ind]
		aside1[#aside1 + 1] = ind
	end
			U.dump(b1, '?? B1:'..#b1..':'..jb..'>'..je)
			U.dump(aside1, '?? B1_aside:'..#b1..':'..jb..'>'..je)

--    if U.vang(cut[1][2] - b2[1], base[je])
	local aside2 = {jb}
	for i = je + 1,jb do
		local ind = (i - 1)%#base + 1
		lo('?? for_ind2:'..ind)
		if ind == je + 1 then
			if not cut[1][3] then
				b2[#b2 + 1] = cut[1][2]
				aside2[#aside2 + 1] = cut[1][1]
			elseif U.vang(cut[1][2] - b2[1], base[ind] - cut[1][2]) > 0.1 then
				b2[#b2 + 1] = cut[1][2]
				aside2[#aside2 + 1] = cut[1][1]
			end
		end
		b2[#b2 + 1] = base[ind]
		aside2[#aside2 + 1] = ind
	end
			U.dump(aside2, '?? B2_aside:'..#b2..':'..#b1..':'..jb..'>'..je)
	floor.top.body = {}

	local arc = coverUp(b1)
	local dim = (b1[1] - b1[math.floor(#b1/2)+1]):length()
	floor.top.achild = {{list = {}, shape = 'flat', istart = 1,
		mat = floor.top.mat, margin = floor.top.margin, tip = dim/4,
		base = b1, aside = aside1 -- original base sides indices containing child vertices ??needed??
		}}
	local list = {}
	for i,rc in pairs(arc) do
		floor.top.body[#floor.top.body + 1] = rc
		-- register rc in achild
		list[#list + 1] = #floor.top.body
	end
	floor.top.achild[#floor.top.achild].list = list

	arc = coverUp(b2)
	dim = (b2[1] - b2[#b2/2+1]):length()
	floor.top.achild[#floor.top.achild + 1] = {list = {}, shape = 'flat', istart = 1,
		mat = floor.top.mat, margin = floor.top.margin, tip = dim/4,
		base = b2, aside = aside2}
	list = {}
	for i,rc in pairs(arc) do
		floor.top.body[#floor.top.body + 1] = rc
		-- register rc in achild
		list[#list + 1] = #floor.top.body
	end
	-- list of rc indexes in joint rc list
	floor.top.achild[#floor.top.achild].list = list
	floor.update = true
			U.dump(floor.top.achild, '?? ACHILD:'..#floor.top.achild)

	houseUp(adesc[cedit.mesh], cedit.mesh)
	lo('<< topSplit:'..#floor.top.achild)
--    intopsplit = true
end


W.topClear = function(desctop)
  if not desctop then return end
  if desctop.achild then
    for i,c in pairs(desctop.achild) do
      local cobj = scenetree.findObjectById(c.id)
      if cobj then
        cobj:delete()
      end
    end
  end
  desctop.achild = nil
  desctop.shape = 'flat'
end


local function wallCollapse(desc, buf, redraw)
		lo('>> wallCollapse:'..desc.id..':'..tableSize(buf))
	local jmi = math.huge
	for i,r in pairs(buf) do
		table.sort(r)
		for k = #r,1,-1 do
			local j = r[k]
			if j < jmi then jmi = j end
--        for _,j in pairs(r) do
			local ij = {i,j}
			local floor = desc.afloor[ij[1]]
	--        local base = floor.base
			forestClean(floor.awall[ij[2]])
			if floor.awall[ij[2]].id then
				scenetree.findObjectById(floor.awall[ij[2]].id):delete()
			end
			table.remove(floor.base, ij[2])
			table.remove(floor.awall, ij[2])
      W.floorClear(floor)
      W.topClear(floor.top)
			baseOn(floor)
		end
	end
  if desc.acorner_ and #desc.acorner_ > 0 then
    scopeOn('floor')
    desc.acorner_ = cornersBuild(desc)
--        U.dump(desc.acorner_, '?? wallCollapse_corner:'..tostring(redraw))
  end
	if redraw then
		houseUp(nil, desc.id)
	end
	return jmi
--[[
	for _,ij in pairs(aij) do
	end
	for i,r in pairs(buf) do
		for _,j in pairs(r) do

		end
	end
]]
end


local function inService()
	return im.IsKeyDown(editor.keyModifiers.shift)
end


W.floorSplit = function(f, j, p, shapekeep)
--            lo('>> floorSplit:'..f.ij[1]..' j:'..j, true)
	if not j then j = cij[2] end
--        if not p then p = cp
	-- rebuild base
	local slen = #f.base
	table.insert(f.base, j + 1, p)
--        table.insert(f.base, cij[2] + 1, p)
--                U.dump(f.base, '??******** wallSplit.base:'..cij[2]..':'..(cij[2] % slen + 1))
	-- rebuild walls
	local awall = {}
	for i = 1,j do
		--- walls pre spit
		awall[#awall + 1] = f.awall[i]
	end
	--- update u
	awall[#awall].u = f.base[j % #f.base + 1] - f.base[j]
	--- new walls
	awall[#awall + 1] = fromWall(awall[#awall], f.base, #awall + 1)
	awall[#awall].pos = p
--        awall[#awall + 1] = fromWall(awall[#awall], f.base, #awall + 1)

	--- update ij
	for i = j+2,#f.base do
		--- walls post split
		awall[#awall + 1] = f.awall[i-1]
--            awall[#awall].ij[2] = awall[#awall].ij[2] + 2
	end
	f.awall = awall
	baseOn(f)
	if not shapekeep then
		f.top.shape = 'flat'
		W.floorClear(f)
	end
end


local function wallSplit()
	if cij == nil or cij[1] == nil then return end
		U.dump(out.asplit, '>> wallSplit:'..scope..':'..tostring(out.wsplit)) --..':'..cij[1]..':'..cij[2]..':'..tostring(cedit.part))
		U.dump(adesc[cedit.mesh].afloor[cij[1]].base)
	if not out.asplit then return end
	local floor = adesc[cedit.mesh].afloor[cij[1]]

	if U._PRD == 0 and im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
			lo('?? wallSplit_HOR:')
		if inService() then
			-- split child
		else
			-- split walls
			for i,e in pairs(out.asplit) do
					U.dump(e, '?? u:'..i..':'..e.u..':'..tostring(base2world(adesc[cedit.mesh], e.ij)))
				local w = floor.awall[e.ij[2]]
				if not w.achild then
					w.achild = {}
				end
				local t,u,v = ray2rect(base2world(adesc[cedit.mesh], e.ij), w.u, w.v, getCameraMouseRay())
--                    U.dump({u*w.u:normalized(),v*w.v:normalized()},'?? u_v:'..tostring(u)..':'..tostring(v))
				w.achild[#w.achild+1] = {ij = e.ij, rc = {{0,0},{1,v*w.v:normalized()}}}
				w.achild[#w.achild+1] = {ij = e.ij, rc = {{0,v*w.v:normalized()},{1,1}}}
					U.dump(w.achild, '?? wS.achild:'..i)
			end
		end
		return
	end

	scenetree.findObject('edit'):deleteAllObjects()

	local desc = adesc[cedit.mesh]
	for _,e in pairs(out.asplit) do
		local base = desc.afloor[e.ij[1]].base
		local p = base[e.ij[2]] + e.u*desc.afloor[e.ij[1]].awall[e.ij[2]].u:normalized()
		W.floorSplit(desc.afloor[e.ij[1]], e.ij[2], p)
	end
			U.dump(floor.base, '?? NEW_BASE:')

	uvOn()
  if desc.acorner_ and #desc.acorner_ > 0 then
    desc.acorner_ = cornersBuild(desc)
  end
--    U.dump(desc.acorner_, '?? corner_POST_SPLIT:')
	houseUp(nil, cedit.mesh)

	toJSON(adesc[cedit.mesh])
--    houseUp(adesc[cedit.mesh], cedit.mesh)
	lo('<< wallSplit:'..#out.avedit)
end

-------------------
-- EVENTS
-------------------
local smouse, cscreen


local function forKey(rayCast)
	if rayCast == nil then return end
	local its = fdata:getItemsCircle(rayCast.pos, 0.2)
	if not cij then return end
--        U.dump(cij, '?? forKey:'..#its)
	if #its > 0 then
		return its[1]:getKey()
	else
		U.dump(cij, '!! NO_F_KEY:')
	end
	return nil
end


local newedit = false
local ccenter, cface = nil, nil
--local floorhit = nil
--local screen = nil


local function wallHit(desc, ij)
	local ray = getCameraMouseRay()
	if not ij[1] or not desc.afloor[ij[1]] then return end
	local base = desc.afloor[ij[1]].base
		U.dump(ij, '?? wallHit:'..tostring(desc.afloor[ij[1]].base[ij[2]])..':'..tostring(desc.pos)..':'..tostring(desc.afloor[ij[1]].pos)..':')
	local d = intersectsRay_Plane(ray.pos, ray.dir,
		desc.afloor[ij[1]].base[ij[2]] + desc.pos + desc.afloor[ij[1]].pos,
		(base[ij[2] % #base + 1] - base[ij[2]]):cross(vec3(0,0,1)))
	return ray.pos + d*ray.dir
end


W.ray2rc = function(ray,floor)
	local desc = adesc[cedit.mesh]
	local i = floor.ij[1]
	local p = base2world(desc,{i,1})+vec3(0,0,floor.h)
	local phit = U.ray2plane(ray,p,vec3(0,0,1))
--      lo('>> ray2rc:'..i..':'..tostring(ray.dir)..':'..#floor.top.achild)
--                        U.dump(desctop.base,'??^^^^^^^^^^^^^^^^^^^^^ ifHit:'..tostring(desctop.shape)..':'..tostring(phit)..':'..tostring(p))
	for k,c in pairs(floor.top.achild) do
		local cbase = {}
		local base = c.base or floor.base
		for _,b in pairs(base) do
			cbase[#cbase+1] = base2world(desc,{i,1},b)+vec3(0,0,floor.h)
		end
--            U.dump(cbase, '?? base_PRE:')
		local h = cbase[1].z
		local arc = coverUp(cbase)
		for _,rc in pairs(arc) do
			for j=1,#rc do
				rc[j].z = h
			end
		end
--        local arc = coverUp(cbase,cbase[1].z)
--            U.dump(cbase, '?? base_POST:')
--        local arc = coverUp(U.clone(cbase))
--                lo('?? ray2rc:'..k)
		local inrc = U.inRC(phit, arc)
--			U.dump(arc,'??^^^^^^^^^^^^^^^^^^^^^ r2r:'..k..':'..tostring(phit)..':'..tostring(inrc))
--        local inrc = U.inRC(phit, {cbase})
--                U.dump(cbase,'?? inrc:'..k..':'..tostring(inrc))--..':'..tostring(desctop.shape))
		if inrc then
			return k,(ray.pos-phit):length()
	--        dmi = (ray.pos-phit):length()
	--        ijmi = {i}
	--        return
		end
	end
end


local function forRoofHit(desc, ifloor)
	if desc == nil then
		lo('!! ERR_forRoofHit.nodesc:')
		return nil
	end
	ifloor = ifloor ~= nil and ifloor or #desc.afloor
	local floor = desc.afloor[ifloor]
	local ray = getCameraMouseRay()
	local vn = vec3(0, 0, 1)
	local h = forHeight(desc.afloor, ifloor)
	local d = intersectsRay_Plane(ray.pos, ray.dir, vec3(0, 0, h), vn)
	local p = ray.pos + d*ray.dir - desc.pos - floor.pos
	p.z = 0
			lo('?? forRoofHit1:'..tostring(h)..':'..tostring(d)..':'..tostring(p)) --..':'..#floor.top.body)
--            U.dump(lastFloor.top.body, '?? mup.floors:'..#adesc[cedit.mesh].afloor..':'..tostring(p))
	if not floor.top then return nil end
--    if not floor.top or floor.top.body == nil then return nil end
	local inrc,d1,d2 = U.inRC(p, floor.top.body)
--            lo('?? forRoofHit2:'..tostring(inrc))
--            U.dump(floor.top.body, '?? forRoofHit2:'..tostring(inrc)..':'..tostring(d1)..':'..tostring(d2))
	local id, ichild = floor.top.id, nil
--[[
	local dmi = math.huge
	local function ifHit(desctop, ic)
		local av = desctop.av
		if not av then return end
		for k =1,#desctop.af,3 do
			d = intersectsRay_Triangle(ray.pos, ray.dir,
				desctop.av[desctop.af[k].v+1], desctop.av[desctop.af[k+1].v+1], desctop.av[desctop.af[k+2].v+1])
			if d < dmi then
				dmi = d
				ichild = ic
				id = desctop.id
			end
		end
	end
]]
--        U.dump(floor.top.body,'?? forRoofHit.pre_child:'..#floor.top.achild..' inrc:'..tostring(inrc), true)
	ichild = W.ray2rc(ray, floor)
		lo('?? rH.child:'..tostring(ichild))
--[[
	if #floor.top.achild then
		for i,c in pairs(floor.top.achild) do
			if #U.index(c.list, inrc) > 0 then
				id = c.id
				ichild = i
				break
			else
				ifHit(c, i)
			end
		end
	end
]]
	return id,inrc,ichild
--    if inrc ~= nil then return id,inrc,ichild end
--    return nil
end


W.floorChildrenOff = function(floor)
  if floor.top.achild then
    for i,c in pairs(floor.top.achild) do
      if c.id then
        scenetree.findObject(c.id):delete()
      end
    end
    floor.top.achild = {}
  end
  floor.top.cchild = nil
  roofSet('flat')
end


local function floorClean(floor)
	for j,w in pairs(floor.awall) do
		for dae,r in pairs(w.df) do
			w.df[dae] = {scale = r.scale or 1}
		end
	end
	for dae,r in pairs(floor.top.df) do
		floor.top.df[dae] = {scale = r.scale or 1}
	end
end


local function floorClone(house, ifloor, base)
	local floor = house.afloor[ifloor]
--        lo('?? floorClone:'..tostring(floor.top.shape))
	base = base ~= nil and base or floor.base
	local awall = forFloor(base, floor.h, floor.awall[1], #house.afloor + 1)

	local dim = (base[1] - base[#base/2+1]):length()
	local newfloor = {base = U.clone(base),
		h = floor.h, pos = floor.pos, awall = awall,
		top = {ij = {ifloor+1,nil}, type = 'top', shape = floor.top.shape,
			istart = floor.top.istart,
			roofborder = floor.top.roofborder, fdep = {'roofborder'},
			mat = house.afloor[#house.afloor].top.mat,
			margin = house.afloor[#house.afloor].top.margin, tip = dim/4,
			id = nil, body = nil, achild = {}}}
	local mbase = U.polyMargin(base, floor.top.margin)
	newfloor.top.body = coverUp(mbase)
--    lo('<< floorClone:'..tostring(newfloor.top.shape))

	return newfloor
end
--[[
	local mbase = base
	if #afloor == nfloor then
		afloor[#afloor].top.margin = 0.4
		mbase = U.polyMargin(base, afloor[#afloor].top.margin)
	end
	local cover = coverUp(mbase)
	afloor[#afloor].top.body = cover
]]


local function floorAddMiddle(ind, dir)
--    local ito = ind and ind or cij[1]+1
	if not ind then ind = cij and cij[1] or nil end
	if not dir then dir = 1 end
--    local dir = 1
	local house = adesc[cedit.mesh]
	local floor
	if house.selection then
			U.dump(house.selection, '??******* CLONE SELECTION:')
		local afloor = {}
		local ima = 0
		for i,r in pairs(house.selection) do
			if i > ima then ima = i end
			local f = deepcopy(house.afloor[i])
			afloor[#afloor+1] = f
			for j,w in pairs(f.awall) do
				for dae,_ in pairs(w.df) do
					w.df[dae] = {scale = w.df[dae].scale or 1}
				end
			end
			for dae,_ in pairs(f.top.df) do
				f.top.df[dae] = {scale = f.top.df[dae].scale or 1}
			end
		end
		floor = house.afloor[ima]
		if ima == #house.afloor then
			floor.top.shape = 'flat'
			floor.top.margin = 0
			afloor[#afloor].top.body = floor.top.body
			floor.top.body = coverUp(floor.base)
		end
	--                for k,f in pairs(afloor) do
		for k = #afloor,1,-1 do
			table.insert(house.afloor, ima + 1, afloor[k])
		end
	elseif dir then
			lo('??******************* FLOOR_clone:'..ind,true)
		local ito
		if dir > 0 then
			ito = ind+1
		else
			ito = ind
		end
		floor = adesc[cedit.mesh].afloor[ind]
		local newfloor = deepcopy(floor)
	--            table.insert(house.afloor, ito, newfloor)
		for j,w in pairs(newfloor.awall) do
			for dae,_ in pairs(w.df) do
				w.df[dae] = {scale = w.df[dae].scale or 1}
			end
			if floor.awall[j].spany then
				if dir > 0 then
					floor.awall[j].spany = floor.awall[j].spany + 1
					w.spany = nil
				else
					w.spany = floor.awall[j].spany + 1
					floor.awall[j].spany = nil
				end
			end
			if floor.awall[j].skip then
					lo('?? for_skip:'..j..':'..ind)
				if ind == #house.afloor or not house.afloor[ind+1].awall[j].skip then
					if dir > 0 then
						newfloor.awall[j].skip = nil
					else
						floor.awall[j].skip = nil
					end
				end
			end
			if not w.pillar then
				w.pillar = {}
			end
			w.pillar.yes = false
		end
		for dae,_ in pairs(newfloor.top.df) do
			newfloor.top.df[dae] = {scale = newfloor.top.df[dae].scale or 1}
		end
		table.insert(house.afloor, ito, newfloor)
		if dir > 0 then
			floor.top.shape = 'flat'
			floor.top.margin = 0
			house.afloor[ito].top.body = floor.top.body
			floor.top.body = coverUp(floor.base)
			for i,c in pairs(floor.top.achild) do
				c.ridge = {}
			end
			W.floorClear(floor)
--                lo('?? floor_cleared:', true)
		end
		uvOn()
		cij[1] = ito
	end
end


--local function floorInOut(val)
--end


local function floorAdd()
		lo('>> floorAdd:', true)
	-- clone top floor
	local desc = adesc[cedit.mesh]
	local floor = desc.afloor[#adesc[cedit.mesh].afloor]
--            lo('?? clone_top:') --..cij[1]..':'..floor.top.shape)
	local newfloor = fromFloor(desc, #desc.afloor)
	desc.afloor[#desc.afloor].top.shape = 'flat'
	desc.afloor[#desc.afloor].top.margin = 0
	desc.afloor[#desc.afloor + 1] = newfloor
	newfloor.ij = {#desc.afloor}
	newfloor.top.ij = {#desc.afloor}
  W.floorClear(desc.afloor[#desc.afloor-1])
	for j,w in pairs(newfloor.awall) do
		w.ij[1] = newfloor.ij[1]
		if not w.pillar then
			w.pillar = {}
		end
		w.pillar.yes = false
    w.doorstairs = nil
	end
--[[
	floor.top.shape = 'flat'
	floor.top.margin = 0
	desc.afloor[#desc.afloor].top.body = floor.top.body
	floor.top.body = coverUp(floor.base)
]]

	return newfloor
end


local function floorAppend()
		U.dump(cedit.cval, '>> floorAppend:'..cij[1]..':'..tostring(intopchange))
--        if true then return end
	local desc = adesc[cedit.mesh]
	local u,v = cedit.cval['AltZ'][1],cedit.cval['AltZ'][2]
	local bpos = desc.pos + desc.afloor[cij[1]].pos + vec3(0,0,forHeight(desc.afloor, cij[1]))
	if cedit.cval['AltZ'][3] == nil then
		cedit.cval['AltZ'][3] = 1
	end
	local base = {
		smouse + (u + v) * cedit.cval['AltZ'][3] * 0.2 - bpos,
		smouse + (u - v) * cedit.cval['AltZ'][3] * 0.2 - bpos,
		smouse + (-u - v) * cedit.cval['AltZ'][3] * 0.2 - bpos,
		smouse + (-u + v) * cedit.cval['AltZ'][3] * 0.2 - bpos,
	}
	if cij[1] == #desc.afloor then
		-- add Floor
		local newFloor = floorClone(desc, cij[1], base)
		desc.afloor[#desc.afloor + 1] = newFloor
			U.dump(newFloor, '?? floorAppend:')
			U.dump(newFloor.top, '?? floorAppend_top:')
		uvOn()
	else
		-- append floor
--        table.insert(desc.afloor, cij[1], newFloor)
	end
	cij[1] = cij[1] + 1
--    local floor = desc.afloor[cij[1]]

	out.avedit = {}
	houseUp(adesc[cedit.mesh], cedit.mesh)
			U.dump(cij, '<< floorAppend:')
end


local function forHit(ray, desc)
--      U.dump(cij, '>> forHit:')
	if not desc then return end
	local ij

	local dmi,ijmi,d = math.huge
	local hitwplus
--    lo('>> forHit:')
	for i,f in pairs(desc.afloor) do
--            lo('?? forHit_f:'..i)
		-- tops
--                U.dump(f.top, '?? for_mesh:'..i..' af:'..#f.top.af..' av:'..#f.top.av)
		if f.top then -- and f.top.af then -- maybe nil if having children
--            U.dump(f, '!! ERR_forHit:'..i)

			local function ifHit(desctop,base)
--          U.dump(ijmi,'>> ifHit:'..i..':'..tostring(base),true)
				if base then
					local p = base2world(desc,{i,1})+vec3(0,0,f.h)
					local phit = U.ray2plane(ray,p,vec3(0,0,1))
--                        U.dump(desctop.base,'??^^^^^^^^^^^^^^^^^^^^^ ifHit:'..tostring(desctop.shape)..':'..tostring(phit)..':'..tostring(p))
					local cbase = {}
					for _,b in pairs(base) do
						cbase[#cbase+1] = base2world(desc,{i,1},b)+vec3(0,0,f.h)
					end
					local inrc = U.inRC(phit, {cbase})
--                        U.dump(cbase,'?? ifHit.inrc:'..i..':'..tostring(p)..':'..tostring(inrc)..':'..tostring(desctop.shape)..':'..i..':'..tostring(base2world(desc,{i,1},base[1])))
					if inrc then
--              U.dump(cbase,'?? inRC:'..i..':'..tostring(phit))
						dmi = (ray.pos-phit):length()
						ijmi = {i}
						return
					end
				end
--                    lo('??_____________ for_HIT:',true)
--                    if true then return end
        desctop = f.top
				local av = desctop.av
				if not av then return end
				for k = 1,#desctop.af,3 do
			--                lo('?? for_k:'..k..'/'..#f.top.af)
					d = intersectsRay_Triangle(ray.pos, ray.dir,
						desctop.av[desctop.af[k].v+1], desctop.av[desctop.af[k+1].v+1], desctop.av[desctop.af[k+2].v+1])
					if d < dmi then
						dmi = d
						ijmi = {i}
					end
				end
			end
--[[
				for _,b in pairs(desctop.base) do
					base[#base+1] = base2world(desc,{i,1},b)
				end
				local inrc = U.inRC(phit, {base})
					lo('?? inrc:'..tostring(inrc))
]]
--                lo('??***************** if_ACHILD:'..tostring(#f.top.achild), true)

--      ifHit(f.top,f.base)

			if #f.top.achild > 0 then -- and not f.top.ridge then -- (not f.top.ridge or not f.top.ridge.on) then
				for _,c in pairs(f.top.achild) do
--                        U.dump(c.base,'?? forHit_child:'..i..':'..tostring(c.shape)..':'.._)
					ifHit(c,c.base)
--            U.dump(ijmi,'??^^^^ if_hit_ch:'..i)
--                    lo('?? if_child_af:'..tostring(#c.af)..':'..tostring(ijmi),true)
				end
			else
				ifHit(f.top,f.base)
			end
		end
		if false and f.awplus then
			for j,wp in pairs(f.awplus) do
--                U.dump(wp, '?? for_WP:'..i..':'..j)
				for k,r in pairs(wp.list) do
					local vn = (r[2]-r[1]):cross(r[3]-r[2])
					local phit = U.ray2plane(ray,r[1],vn)
					local d = (ray.pos-phit):length()
--                        lo('?? for_VN:'..k..':'..tostring(vn)..':'..tostring(phit))
					if U.inRC(phit, {r}) then
--                            lo('?? hit_wp:'..k..':'..d..'/'..tostring(dmi))
						if d < dmi then
							dmi = d
							hitwplus = {j,k} -- {floor child index, chunk index}
						end
					end
				end
--                U.ray2plane(ray,p,vec3(0,0,1))
			end
		end
--            U.dump(ijmi,'??************ forHit_prewall:'..i..':'..dmi)
		-- walls
		for j,w in pairs(f.awall) do
      -- local phit = U.ray2plane(ray,p,vec3(0,0,1))
			d = intersectsRay_Triangle(ray.pos, ray.dir,
				base2world(desc, {i,j}),
				base2world(desc, {i}, f.base[j] + vec3(0,0,f.h)),
				base2world(desc, {i,j+1}))
--          lo('?? if_wall_HIT1:'..j..':'..d..'>'..dmi..':'..tostring(d<dmi),true)
			if d < dmi then
				dmi = d
				ijmi = {i,j}
--            U.dump(ijmi,'?? wHIT:'..i..':'..j)
			end
			d = intersectsRay_Triangle(ray.pos, ray.dir,
				base2world(desc, {i,j+1}),
				base2world(desc, {i}, f.base[j] + vec3(0,0,f.h)),
				base2world(desc, {i}, U.mod(j+1,f.base) + vec3(0,0,f.h)))
--          lo('?? if_wall_HIT2:'..j..':'..d,true)
			if d < dmi then
				dmi = d
				ijmi = {i,j}
			end
		end
	end
	if hitwplus and ijmi then ijmi[3] = hitwplus end
--        U.dump(ijmi,'<< forHit:')
	return ijmi
end


local function mdown(rayCast, inject)
--    U.dump(cij, '>> mdown:')
	smouse = rayCast.pos
	cscreen = {im.GetMousePos().x, im.GetMousePos().y}
	local ray = getCameraMouseRay()

	return forHit(ray, adesc[cedit.mesh])
end


local function mdown_(rayCast, inject)
--    local ray = getCameraMouseRay()
--            U.dump(daePath['win'], '?? IF_DAE2:')
--    local hit = getRayCastHit(getCameraMouseRay(), true)
--    if scope == nil then scope = 'building' end
		U.dump(cij, '>>----------- mdown:'..tostring(rayCast.object.name)..'/'..tostring(cameraMouseRayCast(true).object.name)..':'..tostring(scope)..':'..tostring(cij ~= nil and cij[1] or nil)..':'..tostring(cedit.mesh)..':'..tostring(rayCast.pos)..':'..tostring(#editor.getAllRoads())) --..':'..tostring(ray.pos)..':'..tostring(ray.dir)) --..':'..tostring(hit))
--    cmesh = nil
	intopchange = false
	newedit = false
	smouse = rayCast.pos
	cscreen = {im.GetMousePos().x, im.GetMousePos().y}
	if rayCast.object.name == nil then return end
	local nm = rayCast.object.name
	local id = rayCast.object:getID()
	if editor.keyModifiers.ctrl then return end
	if out.axis then return end

	if string.find(nm, 'o_test_') == 1 then return end


	if out.invertedge then
		lo('?? IN_VE:')
		return
	end
	if nm == forestName then
--        lo('?? RH2:')
		local id = forRoofHit(adesc[cedit.mesh])
		if id ~= nil then
			nm = scenetree.findObjectById(id).name
		end
		lo('?? mdown_IF_FOREST: roof:'..tostring(id)..':'..nm)
		partOn(id)
	end

	if nm == 'theTerrain' then
		lo('?? RH3:'..tostring(cedit.mesh))
		local id = forRoofHit(adesc[cedit.mesh])
			lo('?? check_Terr:'..tostring(id))
		if id ~= nil then
			if id ~= cedit.part then
				newedit = true
			end
			cedit.part = id
			partOn(id)
			local part = scenetree.findObjectById(cedit.part)
			if part then
				nm = part.name
			end
--        else
--[[
		elseif editor.keyModifiers.ctrl and not im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
		   lo('?? to_create:')
		   buildingGen(rayCast.pos)
--           scope = 'building'

--                    local amesh = M.dae2proc('/assets/roof_border/s_RES_S_01_RTR_EEL_01_2.dae')
--                    local id, om = meshUp(amesh)
--                    om:setPosition(vec3(0, 0, 3))

		   lo('<< mdown:'..rayCast.object.name)
		   return cedit.mesh
]]
		else
			lo('<< mdown:'..rayCast.object.name)
			return cedit.mesh
		 end
	end
			lo('?? for_nm:'..tostring(nm))
		-- check for hitting roof
--        lo('?? floors:'..#adesc[cedit.mesh].afloor)

	if nm == forestName then -- no fores edit if building not in edit mode
		local key = forKey(rayCast)
			U.dump(cij, '?? for_cij0:'..tostring(key)) --..':'..tostring(dforest[key].mesh)..':'..scenetree.findObjectById(dforest[key].mesh).name)
		if not cij and not cedit.mesh then
			-- to edit mode
			if key and dforest[key].mesh then
				local obj = scenetree.findObjectById(dforest[key].mesh)
				if obj then nm = obj.name end
			end
		else
			-- edit forest
			lo('?? for_key1:'..tostring(key)..'/'..tostring(cedit.forest))
			if key ~= cedit.forest then newedit = true end
			local desc
			if key ~= nil then
				if cedit and cedit.forest == nil and dforest[key] and dforest[key].mesh then
					--  cij
	--                desc = adesc[dforest[key].mesh]
					desc = dforest[key].mesh ~= nil and adesc[dforest[key].mesh] or adesc[cedit.mesh]
					local tp = dforest[key].type
					local ind
					cij = nil
					forBuilding(desc, function(w, ij)
						if ind ~= nil then return end
	--                    lo('?? for_build:'..ij[1]..':'..ij[2])
						for _,akey in pairs(w.df) do
							local cind = U.index(akey, key)
	--                        lo('?? ak:'.._..':'..tostring(#akey)..':'..#cind)
							if #cind > 0 then
	--                            lo('?? ak_found:'..tostring(ind[1]))
								cij = U.clone(ij)
								ind = cind[1]
								break
							end
						end
					end)
						lo('?? f_TYPE:'..tp..':'..tostring(desc)..' key:'..tostring(key)..':'..tostring(dforest[key].mesh)..':'..tostring(cedit.mesh))
	--                    U.dump(cij, '?? for_fc:'..tostring(ind)..':'..tp..'<')
	--                cedit.forest = key
	--                objEditStop()
					--- update edited forest
					local dae
					if string.find(tp, 'roof') == 1 then
						cij = {#desc.afloor, nil}
						desc = desc.afloor[#desc.afloor].top
						-- TODO for cchild
						dae = desc[tp][1]
						for _,akey in pairs(desc.df) do
							local cind = U.index(akey, key)
							if #cind > 0 then
								ind = cind[1]
								break
							end
						end
	--                            lo('?? IS_IND:'..tostring(ind)..':'..cij[1]..':'..tostring(cij))
					elseif cij then
						desc = desc.afloor[cij[1]].awall[cij[2]]
						dae = desc[tp]
					end
	--                    U.dump(desc, '?? for_wall:'..desc[tp])
					if not desc.df then desc.df = {} end
					if desc.df[dae] then
						cedit.forest = desc.df[dae][ind]
					end
	--                    lo('?? for_key2:'..tostring(cedit.forest)..' mesh:'..tostring(cedit.mesh))
				else
					cedit.forest = key
				end
	--                    lo('?? if_c:'..tostring(cij))
	--                lo('?? for_key3:'..cedit.forest..':'..tostring(dforest[cedit.forest].item)..':'..' mesh:'..tostring(cedit.mesh))
	--                editor.removeForestItem(fdata, dforest[cedit.forest].item)
	--                if true then return cedit.mesh end
				forestEdit(cedit.forest)  -- key changed after houseUp
	--            houseUp(adesc[cedit.mesh], cedit.mesh)
	--            markUp(dforest[cedit.forest].item:getData():getShapeFile())
	--                lo('?? fof:'..' mesh:'..tostring(cedit.mesh)..':'..dforest[cedit.forest].type..':'..dforest[cedit.forest].item:getData():getShapeFile())
	--                lo('?? if_cij:'..tostring(cij)..':'..tostring(dforest[cedit.forest].item:getData():getShapeFile()))
					U.dump(cij, '?? for_cij:'..dforest[cedit.forest].type)
				markUp(U.forOrig(dforest[cedit.forest].item:getData():getShapeFile()))
	--            W.houseUp(adesc[cedit.mesh], cedit.mesh)
	--            U.dump(dforest, '?? dforest:')
				return cedit.mesh
			end
		end

--        local its = out.fdata:getItemsCircle(rayCast.pos, 0.2)
--        lo('?? fits:'..tostring(#its))
--        if #its > 0 then
--            local key = its[1]:getKey()
	end
	if string.find(nm, 'o_') == 1 then
		inject.D.unselect()
		cedit.forest = nil
		local id = scenetree.findObject(nm):getID() --rayCast.object:getID()
				lo('?? for_o_:'..id..':'..tostring(scope)..':'..tostring(cedit.mesh)..':'..rayCast.object.name)
		if not cedit.mesh and string.find(nm, 'o_building_') == 1 then
			scope = 'building'
		end
 --               lo('?? for_obj:'..tostring(editor.keyModifiers.alt))
		if editor.keyModifiers.alt then
--        if editor.keyModifiers.shift then
					U.dump(cij, '?? for_split:'..scope)
			if scope == 'top' then
-- TOP SPLIT
					lo('?? for_top:'..tostring(im.IsKeyDown(im.GetKeyIndex(im.Key_Z))))
--                lo('?? rc:'..id..':'..tostring(rayCast.object)) --..rayCast.object.name)
--                houseUp(adesc[cedit.mesh], cedit.mesh)
				if cedit.cval['AltZ'] ~= nil then
--                if im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
					intopchange = true
					floorAppend()
				else
					topSplit()
					intopchange = true
				end
--                cedit.part = id
--                partOn(id)
--                markUp()
				return true
			else
-- WALL SPLIT
--                    U.dump(aedit[id].desc.ij,'?? for_ws:'..cij[1]..':'..cij[2])
--                    lo('?? for_alt:'..tostring(aedit[id]))
				if aedit[id] then
					cij = U.clone(aedit[id].desc.ij)
					wallSplit()
				end
			end
		else
--            local id,inrc = rayCast.object:getID()
					U.dump(cij,'?? hit_id:'..tostring(id)..':'..tostring(scope)..':'..tostring(rayCast.object.name)..'/'..nm)
			local desc = (id and adesc[id]) and adesc[id] or adesc[cedit.mesh] --or adesc[id]
--            for _,f in pairs(desc.afloor) do
--                f.top.cchild = nil
--            end
	--            lo('?? mdown_dim:'..id..':'..tostring(adesc[id].pos)..':'..tostring(obj:getObjectBox():getExtents()))
			if string.find(nm, 'o_building_') == 1 then
				if cedit ~= nil and cedit.mesh ~= nil and id ~= cedit.mesh then
					lo('?? RE_edit:')
					houseUp(adesc[cedit.mesh], cedit.mesh)
				end
				-- to edit
				local obj = scenetree.findObjectById(id)
				local ext = obj:getObjectBox():getExtents()
				-- center for going around
				ccenter = adesc[id].pos + vec3(-ext.x, -ext.y, ext.z)/2
						lo('??+++++ CENTER:'..tostring(ccenter))
--                    U.dump(adesc[id].afloor[2].awall[1].ij, '?? PRE_split:')
				houseUp(nil, id)
--                    U.dump(adesc[cedit.mesh].afloor[2].awall[1].ij, '?? POST_split:')
				-- select part
				rayCast = cameraMouseRayCast(true)
--                    U.dump(cij, '?? RH4:')
					lo('?? DESC:'..tostring(desc))
				id = forRoofHit(desc)
					lo('?? name_after_split:'..rayCast.object.name..':'..tostring(id)) --..':'..tostring(inrc))
--                    lo('?? for_id:'..tostring(desc.afloor[#desc.afloor].top.id))
				if id == nil and rayCast.object.name ~= 'theTerrain' then
					id = rayCast.object:getID()
					if string.find(rayCast.object.name, 'o_wall_') == 1 then
						-- set wall hit world position
						smouse = wallHit(desc, aedit[id].desc.ij)
							lo('?? smouse:'..tostring(smouse)..':'..rayCast.object.name)
					end
				end
			elseif string.find(nm, 'o_wall_') == 1 then
			elseif string.find(nm, 'o_lid_') ~= 1 then
--                    U.dump(desc.afloor[#desc.afloor].top, '?? PRE_roof:'..tostring(cij[1])..':'..tostring(nm))
				local roofid,inrc,ichild = forRoofHit(desc,cij[1])
					lo('?? EDIT_goon:'..tostring(cij[1])..':'..tostring(roofid)..'/'..tostring(id)..':'..tostring(ichild))
				if roofid ~= nil then
					id = roofid
--                    desc.afloor[#desc.afloor].top.cchild = ichild
				end
			end
--                lo('?? ED_OBJ:'..id)
					lo('?? for_part:'..tostring(id)..':'..tostring(scope)..':'..tostring(cedit.part))
			if id ~= cedit.part then
				newedit = true
				if desc then desc.selection = nil end
				cedit.part = id
					U.dump(cij, '?? if_cij1:')
				partOn(id)
			end
				U.dump(cij, '?? if_cij2:')

			return cedit.mesh
		end
	end
	if newedit then
		cedit.cval = {}
	end
	return nil
end


local sdux
local cdrag

local adwin = {}


local function mouseDir(u, v)
	local cmouse = smouse
	-- get alignment directions
	local ray = getCameraMouseRay()
	local dwin = {im.GetMousePos().x - cscreen[1], im.GetMousePos().y - cscreen[2]}
	cscreen = {im.GetMousePos().x, im.GetMousePos().y}
	local camDir = ray.dir:normalized() -- core_camera.getForward():normalized()

	-- cam frame
	local cx, cz = camDir:cross(vec3(0,0,1)):normalized(), camDir
	local cy = cz:cross(cx)

--    local wall = floor.awall[cij[2]]
--    local u, v = wall.u:normalized(), wall.u:cross(vec3(0,0,1)):normalized()
	-- projections to view plane
	local up, vp = (u - u:dot(camDir)*camDir):normalized(), (v - v:dot(camDir)*camDir):normalized()
	-- edges projections in cam coords
	local upcam = vec3(up:dot(cx), up:dot(cy), 0):normalized()
	local vpcam = vec3(vp:dot(cx), vp:dot(cy), 0):normalized()
	local mcam = vec3(dwin[1], dwin[2], 0)
	if mcam:length() == 0 then
		return nil, smouse
	end
--            lo('?? drag:'..tostring(mcam))
	table.insert(adwin, 1, mcam)
	for i = 4,#adwin do
		table.remove(adwin, i)
	end
	if #adwin == 3 then
		mcam = (adwin[1] + adwin[2] + adwin[3])/3
	end
	if mcam:length() == 0 then
		mcam = (adwin[2] + adwin[3])/2
	end
--        lo('?? drag2:'..tostring(mcam)..':'..tostring(u)..':'..tostring(v))
--                    lo('?? cam_frame:'..tostring(cx)..':'..tostring(cy)..' MCAM:'..tostring(mcam))
--                        lo('?? uvm:'..tostring(upcam)..':'..tostring(vpcam)..':'..tostring(mcam))
--    return math.abs(mcam:dot(upcam)) > math.abs(mcam:dot(vpcam))
	local va, vn
	local isperp = false
	if math.abs(mcam:dot(upcam)) > math.abs(mcam:dot(vpcam)) then
--                        lo('?? ALONG')
		va = u
		vn = v
	else
		va = v
		vn = vec3(0,0,1)
		isperp = true
	end
	local d = intersectsRay_Plane(ray.pos, ray.dir, smouse, vn)
	cmouse = ray.pos + d*ray.dir
	local dm = cmouse - smouse

	return dm:dot(va)*va, cmouse, isperp
--    return va, vn
end


local function inCube(meshID, frame)
--        lo('>> inCube:'..tostring(cmesh))
--        out.avedit = {}
	if dmesh[meshID].sel == nil then return end
--            lo('?? sel1:'..#dmesh[meshID].sel[1].faces)
	local p,u,v,w = frame[1],frame[2],frame[3],frame[4]
	local lx,ly,lz = u:length(),v:length(),w:length()
	local un,vn,wn = u:normalized(),v:normalized(),w:normalized()

	local function forFaces(data)
		local dpop = {}
		local dtrans = {}
--        data = cedit.cval['Drag_Z'][2]
		for ord,m in pairs(data) do
	--        for i = 1,6,6 do
			local verts = dmesh[meshID].data[ord].verts

			local faces = U.clone(m)
--            local faces = U.clone(cedit.cval['Drag_Z'][2][ord])
			dpop[ord] = {}
			dtrans[ord] = {}
--            m.faces = U.clone(data[ord])
--                lo('?? inCube_F:'..tostring(#faces))
			for i = 1,#faces,6 do
				local function isIn(vert)
					local x,y,z = (vert - p):dot(un),(vert - p):dot(vn),(vert - p):dot(wn)
					return (0 < x and x < lx and 0 < y and y < ly and 0 < z and z < lz)
				end
				if isIn(verts[faces[i].v+1]) and isIn(verts[faces[i+1].v+1]) and isIn(verts[faces[i+2].v+1]) then
--                if isIn(verts[m.faces[i].v+1]) and isIn(verts[m.faces[i+1].v+1]) and isIn(verts[m.faces[i+2].v+1]) then
--                if isIn(m.verts[m.faces[i].v+1]) and isIn(m.verts[m.faces[i+1].v+1]) and isIn(m.verts[m.faces[i+2].v+1]) then
					if dpop[ord] == nil then
						dpop[ord] = {}
					end
					dpop[ord][#dpop[ord] + 1] = i
				elseif isIn(verts[faces[i].v+1]) or isIn(verts[faces[i+1].v+1]) or isIn(verts[faces[i+2].v+1]) then
					dtrans[ord][#dtrans[ord] + 1] = i
				end
			end
			table.sort(dpop[ord], function(a, b) return a > b end)
--            dmesh[meshID].data[ord].faces = faces
		end
		return dpop, dtrans
	end

	local dpop,dtrans = forFaces(cedit.cval['Drag_Z'].afaces)
	for ord,m in pairs(dmesh[meshID].data) do
		m.faces = U.clone(cedit.cval['Drag_Z'].afaces[ord])
	end
	local dpopsel,dtranssel = forFaces(cedit.cval['Drag_Z'].aselfaces)
	for ord,m in pairs(dmesh[meshID].sel) do
		m.faces = U.clone(cedit.cval['Drag_Z'].aselfaces[ord])
	end
	dmesh[meshID].buf = {}
	for ord,m in pairs(cedit.cval['Drag_Z'].abuffaces) do
		dmesh[meshID].buf[ord] = {
			verts = dmesh[meshID].data[ord].verts,
			normals = dmesh[meshID].data[ord].normals,
			uvs = dmesh[meshID].data[ord].uvs,
			material = 's_magenta',
			faces = {},
		}
		dmesh[meshID].buf[ord].faces = U.clone(cedit.cval['Drag_Z'].abuffaces[ord])
	end
--[[
]]

--[[
	-- dtranssel->dtrans
	for ord,list in pairs(dtranssel) do
		for _,i in pairs(list) do
--            dtrans[ord][#dtrans[ord] + 1] = i
		end
		table.sort(dtrans[ord], function(a, b) return a > b end)
	end
--        U.dump(dtranssel, '?? DTS:')
]]
--        U.dump(dtrans, '?? dtrans:')
	local amesh = {}
--    dmesh[meshID].buf = {}
	dmesh[meshID].trans = {}
	-- dpop->buf
	if false then
		M.move(dpop, dmesh[meshID].data, dmesh[meshID].buf, 's_magenta')
--        M.move(dpopsel, dmesh[meshID].sel, dmesh[meshID].buf, 's_magenta')
--        M.move(dtrans, dmesh[meshID].data, dmesh[meshID].trans, 's_transe')
	else
--        M.copy(dpop, dmesh[meshID].data, dmesh[meshID].buf, 's_magenta')
--        M.copy(dpopsel, dmesh[meshID].sel, dmesh[meshID].buf, 's_magenta')
		-- dpopsel->dpop
--[[
		for ord,list in pairs(dpopsel) do
			for _,i in pairs(list) do
				dpop[ord][#dpop[ord] + 1] = i
			end
--            table.sort(dpop[ord], function(a, b) return a > b end)
		end
]]
--            U.dump(dpop, '?? dpop:')
		-- dpop->
		M.copy(dpop, dmesh[meshID].data, dmesh[meshID].buf, 's_magenta')
		M.copy(dtrans, dmesh[meshID].data, dmesh[meshID].trans, 's_trans')
		-- dtrans->dpop
		for ord,list in pairs(dtrans) do
			for _,i in pairs(list) do
				dpop[ord][#dpop[ord] + 1] = i
			end
			table.sort(dpop[ord], function(a, b) return a > b end)
		end
		-- dpop->
		M.cut(dpop, dmesh[meshID].data)

--        M.move(dpop, dmesh[meshID].data, dmesh[meshID].buf, 's_magenta')
		M.move(dpopsel, dmesh[meshID].sel, dmesh[meshID].buf, 's_magenta')
--        M.cut(dpopsel, dmesh[meshID].sel)
	end

	for ord,m in pairs(dmesh[meshID].data) do
		amesh[#amesh + 1] = m
		amesh[#amesh + 1] = dmesh[meshID].buf[ord]
		amesh[#amesh + 1] = dmesh[meshID].trans[ord]
		amesh[#amesh + 1] = dmesh[meshID].sel[ord]
	end

	dmesh[meshID].obj:createMesh({})
	dmesh[meshID].obj:createMesh({amesh})

	dmesh[meshID].incube = to
end


local function mdrag(rayCast)
--    local nm = rayCast.object.name
			if _dbdrag then return end
--            _dbdrag = true

--            lo('?? mdrag.smouse:'..tostring(smouse))
--        lo('?? for_VEDGE:'..tostring(smouse)..':'..tostring(incorner)..':'..tostring(out.invertedge), true)
	if smouse == nil then
--        lo('!! ERR_mdrag.nosmouse:')
		return
	end
		lo('?? W.DRAG:'..tostring(out.acorner)..':'..tostring(out.invertedge)..':'..tostring(out.inhole)..':'..tostring(cedit.cval['DragRot'])..':'..tostring(out.acorner)) --..' cij:'..tostring(cij[1])..':'..tostring(cij[2]))
--        _dbdrag = true
	local ds = rayCast.pos - smouse
	local dwin = {im.GetMousePos().x - cscreen[1], im.GetMousePos().y - cscreen[2]}
--            lo('?? mdrag:') --..dwin[1]..':'..dwin[2]..':'..rayCast.object.name)
--            lo('?? mdrag:'..tostring(ds)..':'..tostring(cedit.forest)..':'..tostring(dforest[cedit.forest].item:getScale()))
--            U.dump(dforest, '?? mdrag.dforest:')
--		if editor.keyModifiers.shift then
--            lo('??+++++++++++++++++++++++++++ drag_split:'..tostring(out.acorner)..':'..tostring(out.invertedge), true)
--		end
	if out.inhole then
		local id = adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]].id
		local mesh = aedit[id].mesh
			U.dump(mesh.verts,'?? for_mesh:'..#mesh.verts)
				out.inhole = nil
--        U.dump(adesc[cedit.mesh], '?? drag_hole:'..tostring(smouse)..':'..adesc[cedit.mesh].id)
--        lo('?? drag_hole:'..tostring(smouse)..':'..adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]].id)
		return
	end

	indrag = true
	if out.acorner and ({building=1,floor=1,wall=1})[scope] then -- and (not editor.keyModifiers.ctrl and not incorner) then
			lo('?? drag_CORNER:',true) --..tostring(out.invertedge),true)
--      _dbdrag = true
		local handled = true
--    if (out.invertedge or out.acorner) and not editor.keyModifiers.ctrl then
			--lo('?? for_VEDGE2:', true)
		-- local rayCast = cameraMouseRayCast(false, im.flags(SOTTerrain))
--            local rayCast = cameraMouseRayCast(false, im.flags(SOTTerrain))
		local shift = U.proj2D(rayCast.pos - smouse)
		local desc = adesc[cedit.mesh]
		--??
		if false and editor.keyModifiers.alt and out.invertedge then
--[[
------------------------
-- TO V-SHAPE
------------------------
			local ij = out.invertedge.ij
			local floor = desc.afloor[ij[1] ]
			local shift = U.proj2D(rayCast.pos - smouse)
			local desc = adesc[cedit.mesh]
			local ij = out.invertedge.ij
			local icorner = ij[2]
			local floor = scope == 'floor' and desc.afloor[ij[1] ] or desc.afloor[#desc.afloor]
			local ray = getCameraMouseRay()
			local p = base2world(desc, ij)
--                lo('?? for_P:'..tostring(p),true)
			local d = intersectsRay_Plane(ray.pos, ray.dir, p, vec3(0,0,1))
			local t = ray.pos + d*ray.dir
			local across
			if not cedit.cval['DragPos'] then
					U.dump(floor.top.ridge, '??++++++++++++++++++ for_ridge:'..icorner)
				cedit.cval['DragPos'] = {pos = t}
				if floor.top.ridge.on and floor.top.ridge.apair then
					across = {}
					local base = floor.base
					for k,p in pairs(floor.top.ridge.apair) do
						if icorner == p[1] then
								U.dump(p, '?? hit1:'..k)
							across[#across+1] = {p[2],U.mod(p[1]+1,#base),U.mod(p[2]+1,#base)}
--                                across[#across+1] = {p[2],U.mod(p[2]+1,#base)}
						end
						if icorner == U.mod(p[1]+1,#base) then
								U.dump(p, '?? hit2:'..k)
							across[#across+1] = {U.mod(p[2]+1,#base),p[1],p[2]}
						end
						if icorner == p[2] then
								U.dump(p, '?? hit3:'..k)
							across[#across+1] = {p[1],U.mod(p[2]+1,#base),U.mod(p[1]+1,#base)}
						end
						if icorner == U.mod(p[2]+1,#base) then
								U.dump(p, '?? hit4:'..k)
							across[#across+1] = {U.mod(p[1]+1,#base),p[2],p[1]}
						end
					end
						U.dump(across, '??___ across:')
					cedit.cval['DragPos'].across = across
				end
			end

      if not out.acorner then return end
--                out.acorner = nil
			across = cedit.cval['DragPos'].across or {}
				lo('?? for_V:'..#across, true)
--                lo('?? for_P2:'..tostring(t)..':'..tostring(cedit.cval['DragPos'].pos),true)
			local db = t - cedit.cval['DragPos'].pos
--                U.dump(floor.top.ridge, '?? dpos_V:'..ij[2]..':'..tostring(db)..':'..tostring(out.acorner), true)

			if out.acorner then
				for k,e in pairs(out.acorner) do
--                        db = vec3(-0.3,0,0)
					local base = desc.afloor[e.ij[1] ].base
					base[U.mod(e.ij[2],#base)] = U.mod(e.ij[2],base) + db
					-- replace opposite
					local x,y
					if #across == 2 then
--                    if #across == 2 and k == 2 then
--                            U.dump(across, '?? to_cross:'..k, true)
						x,y = U.lineCross(
							base[across[1][1] ],base[across[1][1] ]+base[icorner]-base[across[1][2] ],
							base[across[2][1] ],base[across[2][1] ]+base[icorner]-base[across[2][2] ])
--                            lo('?? crossed:'..tostring(x)..':'..tostring(y),true)
					elseif #across == 1 then
						U.dump(across, '?? ACROSS_1:')
						x,y = U.lineCross(
							base[across[1][1] ],base[across[1][1] ]+base[icorner]-base[across[1][2] ],
							base[across[1][3] ],base[icorner])
					end
					if x and y then
						base[across[1][3] ] = vec3(x,y,0)
					end
	--                desc.afloor[e.ij[1] ].base[e.ij[2] ] = desc.afloor[e.ij[1] ].base[e.ij[2] ] + db
					e.line[1] = e.line[1] + db
					e.line[2] = e.line[2] + db
					baseOn(desc.afloor[e.ij[1] ])
				end
				houseUp(desc, cedit.mesh, true)
				markUp()
				cedit.cval['DragPos'].pos = t
					if not out.ndrag then out.ndrag = 0 end
					out.ndrag = out.ndrag + 1
			end
]]
		elseif U._PRD==0 and editor.keyModifiers.shift then
-- MOVE SPLIT LINE along
			rayCast = cameraMouseRayCast(false)
			if not cedit.cval['DragPos'] then
        local floor = desc.afloor[cij[1]]
				local j = out.acorner[1].ij[2]
--                    lo('?? acorn:'..tostring(out.acorner[1].ij[2]),true)
				local upre = U.mod(j, floor.base) - U.mod(j-1, floor.base)
				local upost = U.mod(j+1, floor.base) - U.mod(j, floor.base)
				cedit.cval['DragPos'] = {
					u = U.vang(upre, upost) < small_ang and upre:normalized() or nil,
				}
				return
			end
			local u = cedit.cval['DragPos'].u
			if u then
				local db = (U.proj2D(rayCast.pos - base2world(desc, out.acorner[1].ij))):dot(u)
				for k,e in pairs(out.acorner) do
					desc.afloor[e.ij[1]].base[e.ij[2]] = desc.afloor[e.ij[1]].base[e.ij[2]] + db*u
					e.line[1] = e.line[1] + db*u
					e.line[2] = e.line[2] + db*u
					baseOn(desc.afloor[e.ij[1]])
				end
				houseUp(desc, cedit.mesh, true)
				markUp()
			end
		elseif not im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
--                lo('?? corner_DRAG:'..tostring(editor.keyModifiers.ctrl), true)
-- MOVE CORNER freehand
			if not out.acorner then return end
--                lo('?? for_CORNER_move:'..tostring(cij[1])..':'..tostring(cij[2]), true)
--                U.dump(out.acorner, '?? acorner:')
--                if true then return end
--                U.dump(desc.afloor[cij[1]].base, '?? BASE:')
			local ray = getCameraMouseRay()
			local pairend,pair,ind,imap
			if editor.keyModifiers.ctrl and scope ~= 'wall' then
				pairend,pair,imap = W.ifPairEnd() --nil,true)
          lo('?? if_PAIREND:'..tostring(pairend)..':'..tostring(cij), true)
--          _dbdrag = true
			end
			local ij = out.acorner[1].ij
			local p = base2world(desc, ij)
			if not cedit.cval['DragPos'] then
				local floor = desc.afloor[cij[1]]
        W.floorChildrenOff(floor)
				local p = base2world(desc, ij)
				local d = intersectsRay_Plane(ray.pos, ray.dir, p, vec3(0,0,1))
				local t = ray.pos + d*ray.dir
				cedit.cval['DragPos'] = {pos = t}
					U.dump(pair,'??+++++++++++++++++++++++++++++ DRAG_freehand:'..tostring(ij[1])..':'..tostring(ij[2])..':'..#out.acorner..':'..tostring(incorner and #incorner or nil)..':'..tostring(pairend)..':'..tostring(scope), true)
--                    _dbdrag = true
--                    pair,ind,map = W.ifPairHit({incorner[1].ij[1],U.mod(incorner[1].ij[2]-1,#floor.base)})
--                    U.dump(pair, '?? PH:'..tostring(ind)..':'..tostring(map))
				if editor.keyModifiers.ctrl then
--??
					if not pairend then
						local insplit = {}
            local inalong
            if scope == 'wall' then
            -- if move edge along the wall
              local w = floor.awall[cij[2]]
--              local w = floor.awall[incorner[1].ij[2]]
              local mdir,cmouse,isperp = mouseDir(w.u:normalized(),w.u:cross(vec3(0,0,1)):normalized())
              if isperp == false then
                inalong = true -- incorner[1].ij[2]
              end
              if not cedit.cval['DragPos'].inalong then
                cedit.cval['DragPos'].inalong = w.u:normalized()
              end
                lo('?? if_DIR:'..tostring(isperp)..':'..tostring(mdir)..':'..tostring(inalong),true)
--              return
            end
                U.dump(incorner, '?? incorner:'..scope..':'..cij[2]..':'..tostring(inalong))
--                _dbdrag = true
            if not inalong then
--------------------
-- V-split
--------------------
              for k,c in pairs(incorner) do

                local ind = c.ij[2]
                floor = desc.afloor[c.ij[1]]
                local base = floor.base
                local u1 = (U.mod(ind-1,base) - base[ind]):normalized()
                local u2 = (U.mod(ind+1,base) - base[ind]):normalized()
                local ang = U.vang(u1,u2,true)
  --                  lo('?? ang0:'..ang)
                ang = ang>0 and -(2*math.pi-ang) or ang
  --                  lo('??************* eB:'..tostring(u1)..':'..tostring(u2)..' ang:'..tostring(ang))
                if math.abs(math.abs(ang)-math.pi)<U.small_val then
                  ang = -math.pi
                end
                local v = U.vturn(u2, -ang/2)
  --									lo('?? pre_SPLIT: floor:'..c.ij[1]..':'..ind..':'..tostring(base[ind])..' v:'..tostring(v)..' ang:'..tostring(ang)..':'..#base,true)
                local mi,imi,vmi=math.huge
                -- corossing with opposite side
                for j,b in pairs(base) do
                  if math.abs(j-ind) > 1 then
    --                                    lo('?? pre_CROSS:'..tostring(base[ind])..':'..tostring(base[ind]+v)..':'..tostring(base[j])..':'..tostring(U.mod(j+1,base)))
                    local x,y = U.lineCross(base[ind],base[ind]+v,base[j],U.mod(j+1,base))
                    local d = (vec3(x,y)-base[ind]):length()
  --											lo('?? for_D:'..j..' b1:'..tostring(base[j])..' b2:'..tostring(U.mod(j+1,base))..' x:'..tostring(x)..'y:'..tostring(y)..':'..tostring(base[ind])..' d:'..tostring(d))
                    if d < mi and U.toLine(base[ind],{base[j],U.mod(j+1,base)}) > small_dist then
  --                                    if not isnan(d) and d < mi and U.toLine(base[ind],{base[j],U.mod(j+1,base)}) > small_dist then
                      mi = d
                      imi = j
                      vmi = vec3(x,y)
                    end
                  end
                end
                if imi then
  --									U.dump(incorner,'??^^^^^^^^^^^^^^^^^ for_IMI:'..imi..':'..tostring(base[imi])..':'..#incorner..' perp:'..tostring(v))
                  local x,y = U.lineCross(base[ind],base[ind]+v, base[imi],U.mod(imi+1,base))
                  local v = vec3(x,y)
                  local ihit
  --                    lo('?? if_HITe:'.. tostring(v)..':'..tostring((v-base[imi]):length())..':'..(v-U.mod(imi+1,base)):length())
                  if (v-base[imi]):length() < small_dist or (v-U.mod(imi+1,base)):length() < small_dist then
  --                    lo('?? hit_EDGE:')
                    ihit = imi
                  elseif (v-U.mod(imi+1,base)):length() < small_dist then
                    ihit = U.mod(imi+1,#base)
                  end
                  if ihit then
                    -- check if opposite sides are parallel
                  else
                    -- if parallel
                    if U.vang(U.mod(imi+1,base)-base[imi],base[ind]-U.mod(ind+1,base)) < small_ang then
                      --- split opposite side
  --											U.dump(base,'?? if_SPLIT:'..imi..':'..#base)
                      W.floorSplit(floor,imi,v-base[1]) --,floor.top.ridge and floor.top.ridge.on)
                          lo('?? post_SPLIT:'..#base,true)
                      -- update incorner
                      if c.ij[2] >= imi then c.ij[2] = c.ij[2] + 1 end

                      insplit[k] = {floor = c.ij[1], fr = c.ij[2], to = imi+1}
  --                        _dbdrag = true
  --                                        U.dump(base,'?? if_SPLIT_post:'..imi..':'..#base)
                    end
                  end
                end

              end

            end
						if tableSize(insplit)>0 then
							cedit.cval['DragPos'].insplit = insplit
                  U.dump(insplit,'?? is_insplit:')
							scenetree.findObject('edit'):deleteAllObjects()
							uvOn()
							houseUp(nil, cedit.mesh)
						end
--[[
										incorner[#incorner+1] = {
											ij = {ij[1],imi},
											line = {},
										}
]]
--[[
						pair = nil
						if math.abs(ang - math.pi) < small_ang then
							-- check split line
							pair,ind,map = W.ifPairHit({incorner[1].ij[1],U.mod(ind-1,#base)})
--                                U.dump(pair,'?? if_split:'..ind)
--                                U.dump(map,'?? map:'..ind)
--                            pairend,pair = W.ifPairEnd({incorner[1].ij[1],U.mod(ind-1,#base)},true)
						else
							-- check inner corner
							pair,ind,map = W.ifPairHit({incorner[1].ij[1],ind})
--                            pairend,pair = W.ifPairEnd({incorner[1].ij[1],ind},true)
						end
						if pair then
							U.dump(pair,'?? if_split:'..ind)
							U.dump(map,'?? map:'..ind)
								U.dump(pair,'??^^^^^^^^^^^^^^^^ V_drag:'..tostring(ang)..':'..tostring(v)..':'..tostring(u1))

							local x,y = U.lineCross({base[ind],base[ind]+v},{
								base[map[pair[ind] ] ],
								base[map[U.mod(pair[ind]+1,#map)] ]
							})
						end
]]
					else
						cedit.cval['DragPos'].pairend = true
					end
				else
          W.floorChildrenOff(floor)
--[[
					if floor.top.achild then
		--                    U.dump(floor.top.achild, '?? achild:')
						for i,c in pairs(floor.top.achild) do
							if c.id then
								scenetree.findObject(c.id):delete()
							end
						end
						floor.top.achild = {}
					end
					floor.top.cchild = nil
					roofSet('flat')
]]
				end
--[[
				if scope == 'building' then
					desc.afloor[#desc.afloor].top.shape = 'flat'
				elseif scope == 'floor' then
					desc.afloor[cij[1] ].top.shape = 'flat'
				end
]]
				return
			end
			if not cedit.cval['DragPos'].pairend then
				local d = intersectsRay_Plane(ray.pos, ray.dir, p, vec3(0,0,1))
				local t = ray.pos + d*ray.dir
	--                        U.dump(incorner, '??^^^^^^^^^^^^^^^^^^^^^ for_corners:'..#incorner)
--	                        _dbdrag = true
				local db = t - cedit.cval['DragPos'].pos
        if cedit.cval['DragPos'].inalong then
          db = cedit.cval['DragPos'].inalong*db:dot(cedit.cval['DragPos'].inalong)
              lo('?? ALONG:'..tostring(db))
--              _dbdrag = true
        end
	--                db = vec3(0,1,0)
	--                lo('?? for_db:'..tostring(db), true)
	--                U.dump(out.acorner, '??++++++++++++++ for_acorner:')
	--                    _dbdrag = true
				if db:length() > 0 then
					for k,e in pairs(incorner) do
	--                    for k,e in pairs(out.acorner) do
		--                    db = vec3(0.1,0,0)
						local base = desc.afloor[e.ij[1]].base
	--                        if db:length() > 0 then
	--                            lo('??^^^^^^^^^^^^ base_UPDATE: baselen:'..#base..':'..tostring(U.mod(e.ij[2],#base))..' base:'..tostring(U.mod(e.ij[2],base))..' db:'..tostring(db)..':'..tostring(cedit.cval['DragPos'].insplit),true)
	--                        end
            base[U.mod(e.ij[2],#base)] = U.mod(e.ij[2],base) + db
    --                desc.afloor[e.ij[1]].base[e.ij[2]] = desc.afloor[e.ij[1]].base[e.ij[2]] + db
            if e.line then
              e.line[1] = e.line[1] + db
              e.line[2] = e.line[2] + db
            end
						baseOn(desc.afloor[e.ij[1]])
					end
	--                    U.dump(cedit.cval['DragPos'].insplit, '?? for_SPLIT:'..tostring(pairend)..':'..tostring(editor.keyModifiers.ctrl))
					if editor.keyModifiers.ctrl then
--                U.dump(cedit.cval['DragPos'].insplit,'?? if_SPLIT:'..tostring(pairend))
--						if pairend then
--							handled = false
--						else
            if cedit.cval['DragPos'].insplit then
	--                            U.dump(cedit.cval['DragPos'].insplit, '??^^^^^^^^^^^^^^^^^^^^^ for_VVVV:'..#incorner)
	--                            _dbdrag = true
							for i,s in pairs(cedit.cval['DragPos'].insplit) do
								local base = desc.afloor[s.floor].base
--                    U.dump(base,'?? for_base:')
								local u1,u2 = U.mod(s.fr,base) - U.mod(s.fr-1,base), U.mod(s.fr,base) - U.mod(s.fr+1,base)
								local x,y = U.lineCross(
									U.mod(s.to+1,base),U.mod(s.to+1,base)+u1,
									U.mod(s.to-1,base),U.mod(s.to-1,base)+u2)
	--                            local vc = vec3(x,y)
								base[s.to] = vec3(x,y)
								baseOn(desc.afloor[s.floor])
								local apair = T.pairsUp(base)
								if apair and #apair==(#base-2)/2 then
									desc.afloor[s.floor].top.poly = 'V'
								end
	--                                lo('?? for_PAIRS:'..s.floor..':'..#apair,true)
	--                                lo('?? newpos:'..tostring(vc))
							end
            elseif pairend then
							handled = false
						end
					end
	--                    U.dump(desc.afloor[2].base, '?? new_BASE:')
					houseUp(desc, cedit.mesh, true)
					markUp()
				end
				cedit.cval['DragPos'].pos = t
			end
		end

		if handled and not cedit.cval['DragPos'].pairend then
      lo('?? handled:',true)
			return
		else
			cedit.cval['DragPos'] = nil
--            cedit.cval['DragRot'] = nil
		end
	-- Z ------------
	elseif im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
		if scope == 'building' then
			if not cedit.cval['DragPos'] then
				lo('?? Z-drag:') --..tostring(cameraMouseRayCast(true).object.name), true)
				cedit.cval['DragPos'] = {
					mouse = cameraMouseRayCast(true).pos,
					pos = adesc[cedit.mesh].pos,
				}
				adesc[cedit.mesh].pinned = true
				return
			end
			local ds = cameraMouseRayCast(true).pos - smouse
				lo('?? Z_dragging:'..tostring(ds)) --..':'..tostring(cameraMouseRayCast(true).object.name), true)
			adesc[cedit.mesh].pos = cedit.cval['DragPos'].pos + ds:dot(vec3(0,0,1))*vec3(0,0,1)
			houseUp(adesc[cedit.mesh], cedit.mesh, true)
			markUp()
		end
		return
	elseif not cedit.forest and not incorner and not editor.keyModifiers.shift and not editor.keyModifiers.ctrl and not editor.keyModifiers.alt then
--            lo('?? indrag1:'..scope..':'..tostring(cedit.cval['DragPos'])..':'..tostring(rayCast.object.name))
--            if true then return end
--        if not (rayCast.object and string.find(rayCast.object.name, 'GroundPlane')==1) then
--            rayCast = cameraMouseRayCast(false, im.flags(SOTTerrain))
--        end
		local rayCast = cameraMouseRayCast(false, im.flags(SOTTerrain))
		if scope == 'building' then
-- MOVE  BUILDING
			if rayCast then
	--            lo('?? drag_pos:'..cameraMouseRayCast(false, im.flags(SOTTerrain)).object.name..':'..cameraMouseRayCast().object.name..':'..tostring(rayCast.pos)..':'..tostring(smouse)..':'..tostring(adesc[cedit.mesh].pos))
				if not cedit.cval['DragPos'] or not cedit.cval['DragPos'].pos then
					lo('?? DRAG_building:', true)
					smouse = rayCast.pos
					cedit.cval['DragPos'] = {pos = adesc[cedit.mesh].pos}
					return
				end
--                    lo('?? for_build:'..tostring(cedit.cval['DragPos'].pos))
				adesc[cedit.mesh].pos = cedit.cval['DragPos'].pos + (rayCast.pos - smouse)
--[[
						if not adesc[cedit.mesh].prn then
							adesc[cedit.mesh].pos = cedit.cval['DragPos'].pos + vec3(-2,0,0)
							indrag = false
							_dbdrag = true
						end
]]
				houseUp(adesc[cedit.mesh], cedit.mesh, true)
				markUp()
				return
			end
		elseif false and scope == 'side' and U._PRD == 0 then
			local desc = adesc[cedit.mesh]
			if not cedit.cval['DragPos'] then
				smouse = rayCast.pos
				cedit.cval['DragPos'] = {}
				for _,f in pairs(desc.afloor) do
					cedit.cval['DragPos'][#cedit.cval['DragPos'] + 1] = U.clone(f.base)
				end
				return
			end
				U.dump(cij, '?? move_SIDE:')
			local shift = U.proj2D(rayCast.pos - smouse)
--                    shift = vec3(3,6,0) --
			local side = forSide()
				U.dump(side, '?? SIDE:'..tostring(cij))
			for i,f in pairs(side) do
				for j,w in pairs(f) do
					local ij = {i, w}
					local base = cedit.cval['DragPos'][ij[1]] --desc.afloor[ij[1]].base
					local cbase = desc.afloor[ij[1]].base
					-- base update
					cbase[ij[2]] = base[ij[2]] + shift
					cbase[ij[2] % #cbase + 1] = base[ij[2] % #base + 1] + shift
					-- u update
					desc.afloor[ij[1]].awall[(ij[2] - 2) % #cbase + 1].u =
						cbase[ij[2]] - cbase[(ij[2] - 2) % #cbase + 1]
					desc.afloor[ij[1]].awall[ij[2]].u = cbase[ij[2] % #cbase + 1] - cbase[ij[2]]
					desc.afloor[ij[1]].awall[(ij[2]) % #cbase + 1].u =
						cbase[(ij[2] + 1) % #cbase + 1] - cbase[ij[2] % #cbase + 1]
				end
			end

--                    _dbdrag = true
			houseUp(adesc[cedit.mesh], cedit.mesh)
			return
		end
		if U._PRD == 0 and scope == 'wall' then
			if not cedit.cval['DragPos'] then
				cedit.cval['DragPos'] = {hmouse = cameraMouseRayCast(true).pos} -- rayCast.pos}
				return
			end
			local c1 = cedit.cval['DragPos'].hmouse
			local c2 = cameraMouseRayCast(true).pos --rayCast.pos
			local vz = (c2-c1):dot(vec3(0,0,1))*vec3(0,0,1)
			local vh = c2 - (c1 + vz)
--                lo('?? for_WALL:'..tostring(c2 - c1)..':'..tostring(vh), true) --..':'..tostring(smouse), true)
			out.ahole = {c1}
			out.ahole[#out.ahole+1] = out.ahole[#out.ahole] + vz
			out.ahole[#out.ahole+1] = out.ahole[#out.ahole] + vh
			out.ahole[#out.ahole+1] = out.ahole[#out.ahole] - vz
			out.ahole[#out.ahole+1] = out.ahole[#out.ahole] - vh
		end
	end

	if cmesh ~= nil then
		out.apath = nil
		if not editor.keyModifiers.shift and not editor.keyModifiers.ctrl then
--                lo('?? MM:')
			if dmesh[cmesh].apick then
				lo('?? move_picked:')
			else
				-- move mesh
				indrag = true
				local dirhit = (rayCast.pos - core_camera.getPosition()):normalized()
				if cedit.cval['Drag'] == nil then
					-- get object center
					local center = U.boxMark(dmesh[cmesh].obj, out)
					local d = intersectsRay_Plane(core_camera.getPosition(), dirhit, center, vec3(0,0,1))
					cedit.cval['Drag'] = {
						terrain = d * dirhit,
						center = center,

						screen = im.GetMousePos(),
						cpos = U.boxMark(dmesh[cmesh].obj, out),
					}
					return
				end
				local cterrain = intersectsRay_Plane(core_camera.getPosition(), dirhit, cedit.cval['Drag'].center, vec3(0,0,1))
				local dv = cterrain * dirhit - cedit.cval['Drag'].terrain
					lo('?? dv:'..tostring(dirhit)..':'..tostring(dv))
				for ord,m in pairs(dmesh[cmesh].data) do
					for i = 1,#m.verts do
						m.verts[i] = m.verts[i] + dv
					end
				end
				cedit.cval['Drag'].terrain = cterrain * dirhit
				M.update(dmesh[cmesh])
				U.boxMark(dmesh[cmesh].obj, out)
		--        local dx = 2 * dirobj * math.sin(da)
			end
			return
		end
		-- SHIFT ------------
		if editor.keyModifiers.shift then
	--    if im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then -- or im.IsKeyDown(im.GetKeyIndex(im.Key_X)) then
			indrag = true
			-- mesh faces selection
	--            lo('?? Z_DRAG:'..tostring(cmesh))
	--                lo('?? mark:'..tostring(im.GetMousePos())..':'..tostring(rayCast.normal)..':'..tostring(core_camera.getForward()))
	--        if true then return end
	--                lo('?? mark2:'..tostring(dirhit)..':'..tostring(rayCast.normal))
	--        local vx = core_camera.getForward():cross(vec3(0,0,1))
	--        local vy = vx:cross(core_camera.getForward())
			if ccenter == nil then return end
			local dirobj = (ccenter - core_camera.getPosition()):normalized()
			local dirhit = (rayCast.pos - core_camera.getPosition()):normalized()
			local dirface = (cface[2]-cface[1]):cross(cface[3]-cface[2]):normalized()
			-- distance to hit point
			local d = intersectsRay_Plane(core_camera.getPosition(), dirhit, ccenter, dirface)
	--        local d = intersectsRay_Plane(core_camera.getPosition(), dirhit, ccenter, dirobj)
			local phit = core_camera.getPosition() + dirhit*d
			if cedit.cval['Drag_Z'] == nil then
				local afaces = {}
					lo('?? z_drag:'..tostring(cmesh)) --..':'..#dmesh[cmesh].data..':'..#dmesh[cmesh].buf..':'..tostring(dmesh))
				if cmesh == nil then return end
	--[[
				if #dmesh[cmesh].buf > 0 then
	--                M.copy(dmesh[cmesh].buf, dmesh[cmesh].data)
	--                M.move(nil, dmesh[cmesh].trans, dmesh[cmesh].data, 'NONE')
	--                    lo('?? moved:'..#dmesh[cmesh].data)
				else
				end
	]]
				for ord,m in pairs(dmesh[cmesh].data) do
					afaces[#afaces+1] = U.clone(m.faces)

					local list = afaces[#afaces]
					-- trans->data_backup
					if dmesh[cmesh].trans[ord] ~= nil then
						for _,f in pairs(dmesh[cmesh].trans[ord].faces) do
							list[#list + 1] = f
						end
					end
	--[[
					if dmesh[cmesh].buf[ord] ~= nil then
						for _,f in pairs(dmesh[cmesh].buf[ord].faces) do
	--                        list[#list + 1] = f
						end
					end
	]]
				end
	--[[
				for _,m in pairs(dmesh[cmesh].trans) do
	--                afaces[#afaces+1] = U.clone(m.faces)
				end
	]]

				local aselfaces = {}
				for _,m in pairs(dmesh[cmesh].sel) do
					aselfaces[#aselfaces+1] = U.clone(m.faces)
				end
				local abuffaces = {}
				for _,m in pairs(dmesh[cmesh].buf) do
					abuffaces[#abuffaces+1] = U.clone(m.faces)
				end
	--            cedit.cval['Drag_Z'] = {phit, afaces, aselfaces, abuffaces}
				cedit.cval['Drag_Z'] = {
					dragstart = phit,
					afaces = afaces,
					aselfaces = aselfaces,
					abuffaces = abuffaces,
				}
	--            out.apoint = {phit}
				return
			end
			local dr = phit - cedit.cval['Drag_Z'].dragstart

			local vx = dirface:cross(vec3(0,0,1)):normalized()
			if dirface:cross(vec3(0,0,1)):length() < 0.001 then
				vx = (rayCast.pos - ccenter):normalized()
			end
			local vz = dirface:cross(vx)
			local w = math.min(math.abs(dr:dot(vx)),math.abs(dr:dot(vz)))*dirface:normalized()

	--        local vx = dirobj:cross(vec3(0,0,1)):normalized()
	--        local vz = dirobj:cross(vx)
	--        local w = math.min(math.abs(dr:dot(vx)),math.abs(dr:dot(vz)))*dirobj:normalized()
			local frame = {cedit.cval['Drag_Z'].dragstart - w/2, dr:dot(vx)*vx, dr:dot(vz)*vz, w}
	--                lo('?? for_frame:'..tostring(dirface)..':'..tostring(vx))
	--                out.apoint = {cedit.cval['Drag_Z'][1]}
	--                out.avedit = {ccenter}
	--                lo('?? sel:'..#dmesh[cmesh].sel[1].faces)
	--                U.dump(dmesh[cmesh].sel[1].faces, '?? sel:')
			inCube(cmesh, frame)
	--                lo('?? zDRAG:'..tostring(#out.apoint))
			out.frame = frame
			return
--[[
		elseif not editor.keyModifiers.ctrl and not im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) and cmesh ~= nil then
			if true then return end
			lo('?? DRAG_SELECTED:'..#dmesh[cmesh].sel)
			-- make procedural
			--- rebuild data
			local amesh = M.clone(dmesh[cmesh])
			local id, om = meshUp(amesh, 'test', groupEdit)
			dmesh[id] = {obj = om, data = amesh, sel = {}}
			-- unselect
					_dbdrag = true
]]
		end
--        return
	end

	if false and editor.keyModifiers.alt and U._PRD == 0 then
--                lo('?? drCtl:'..tostring(ccenter))
		-- fly around
--        indrag = true
		if ccenter == nil then return end
		if #cedit.cval == 0 then
			cedit.cval = {im.GetMousePos().x, core_camera.getPosition()}
		end
		local dx = im.GetMousePos().x - cedit.cval[1]
		local pos = cedit.cval[2]

--        local mr = getCameraMouseRay()
--        local pos = core_camera.getPosition()
		local dir = ccenter - pos
		local newdir = U.vturn(dir, -dx/1000)
--        lo('?? around:'..dx..':'..tostring(core_camera.getForward())..':'..tostring(newdir:normalized())) --..':'..tostring(im.GetMousePos().x))
		local newpos = ccenter - newdir
		core_camera.setPosition(0, newpos)
		local fwd = core_camera.getForward()
		local thresh = 500
		if math.abs(dx) < thresh then
			newdir = (fwd*(thresh - math.abs(dx)) + newdir*math.abs(dx))/thresh
		end
		core_camera.setRotation(0, quatFromDir(newdir, vec3(0,0,1)))
--        lo('?? around:'..tostring(indrag))
		return
	------------------------
	-- CONTROL ------------
	------------------------
	elseif editor.keyModifiers.ctrl then
            lo('?? drag_CTRL:'..tostring(cij)..':'..tostring(cedit.cval['DragRot']),true)
		if adesc[cedit.mesh] == nil then return end
-- ROTATE buiding/floor
		local desc = adesc[cedit.mesh]
		local base = desc.afloor[1].base
--        local ray = getCameraMouseRay()
--        local d = intersectsRay_Plane(ray.pos, ray.dir, smouse, vn)
--        cmouse = ray.pos + d*ray.dir
--            lo('??^^^^^^^^^^^^^^^^^^^^ ctrl_DRAG:'..tostring(cedit.cval['DragRot']))
--            _dbdrag = true
		if not cedit.cval['DragRot'] and cij then
        lo('?? drag_CTRL_0:',true)
--				U.dump(desc.afloor[cij[1]].base, '??+++++++ ROTATE:'..tostring(incorner)..':'..tostring(cedit.mesh)..':'..tostring(ccenter)..':'..tostring(adesc[cedit.mesh])..':'..tostring(desc.afloor[2].ij)..':'..scope..' cij:'..tostring(cij[1])..':'..tostring(cij[2]))
--                _dbdrag = true
			local center = U.polyCenter(base) --+ cedit.cval['DragRot'].base
			center = base2world(desc, {1, 1}, center)
--                    out.apoint = {center + vec3(0,0,1.5)}
      local floor = desc.afloor[cij[1]]
      if floor.top.achild and #floor.top.achild>0 then
        W.floorChildrenOff(floor)
      end
			cedit.cval['DragRot'] = {
				screen = {im.GetMousePos().x},
				center = center}
			if scope == 'floor' then
				cedit.cval['DragRot'].u = (desc.afloor[cij[1]].base[2] - desc.afloor[cij[1]].base[1]):normalized()
					lo('??+++++++++++++++++++ dir_INI:'..tostring(cedit.cval['DragRot'].u)..':'..tostring(cij[1])..':'..tostring(cij[2]), true)
			end
--          lo('?? if_corner0:'..tostring(incorner),true)
			local pairend,pair,imap = W.ifPairEnd() --nil,true)
--                    U.dump(incorner, '?? if_PAIR:'..tostring(pairend)..':'..tostring(incorner))
--          lo('?? if_corner:'..tostring(pairend),true)
--					U.dump(incorner, '??_________ PE:'..tostring(pairend))
			if pairend then
				cedit.cval['DragRot'].pairend = pairend
				cedit.cval['DragRot'].imap = imap
--                cedit.cval['DragRot'].pairind = ind
				local cbase = desc.afloor[cij[1]].base
          U.dump(pair, '?? drag_ctrl_for_pairend:')
				cedit.cval['DragRot'].dir = (U.mod(imap[pair[1]]+1,cbase)-U.mod(imap[pair[1]],cbase)):normalized()
--				cedit.cval['DragRot'].dir = (U.mod(pair[1]+1,cbase)-U.mod(pair[1],cbase)):normalized()
--					U.dump(incorner, '?? if_PE0:'..tostring(pairend)..' dir:'..tostring(cedit.cval['DragRot'].dir),true)
--[[
				local p = base2world(desc, cij)
				local wall = desc.afloor[cij[1] ].awall[cij[2] ]
				local vn = wall.u:cross(wall.v):normalized()
				local d = intersectsRay_Plane(ray.pos, ray.dir, p, vn)
					U.dump(wall, '?? for_wall:'..tostring(p)..' d:'..tostring(d))
				cedit.cval['DragRot'].smouse = ray.pos + d*ray.dir
]]
			end
			return
		end
		local dx = im.GetMousePos().x - cedit.cval['DragRot'].screen[1]
		cedit.cval['DragRot'].screen = {im.GetMousePos().x}
--            _dbdrag = true
		local pe = cedit.cval['DragRot'].pairend
			lo('?? if_PE:'..tostring(pe)..':'..tostring(incorner)..':'..tostring(out.acorner)..':'..tostring(cedit.cval['DragRot'])..':'..cij[2], true)
		local toupdate = true
		if pe then
			local wall = desc.afloor[cij[1]].awall[cedit.cval['DragRot'].imap[pe]]
			local u,v = wall.u:normalized(), wall.u:cross(vec3(0,0,1)):normalized()
			local dpos, cmouse, isperp = mouseDir(u, v)
      if not incorner then
        -- move face
--			if pe == 2 then
--                    lo('?? for_PARALL:', true)
	--                    dpos = vec3(1,0,0)
	--                _dbdrag = true
	--                    cedit.cval['DragRot'].pairend = false
				if dpos then
					local cfloor
					forBuilding(desc, function(w, ij)
						if ij[1] == cfloor then return end
						cfloor = ij[1]
						local floor = desc.afloor[ij[1]]
						local base = floor.base
						local u = U.mod(ij[2]+1,base) - base[ij[2]]
						base[ij[2]] = base[ij[2]] + dpos
						local vnew = base[ij[2]] - U.mod(ij[2]-1,base)
						local x,y = U.lineCross(base[ij[2]], base[ij[2]] + u, U.mod(ij[2]+2,base), U.mod(ij[2]+2,base)+vnew)
--                        local x,y = U.lineCross(base[ij[2]], U.mod(ij[2]+1,base), U.mod(ij[2]+2,base), U.mod(ij[2]+2,base)+vnew)
						--  = U.mod(ij[2]+1,base) - base[ij[2]]
						base[U.mod(ij[2]+1,#base)] = vec3(x,y,0) -- U.mod(ij[2]+1,base) + dpos
	--                        lo('?? fB:'..ij[2])
						baseOn(floor, base, ij[1])
						-- update children
						-- TODO: use childRebase?
						for _,c in pairs(floor.top.achild) do
							if c.imap then
--                                    U.dump(c.imap, '?? for_c_map:'..ij[2])
								for k,m in pairs(c.imap) do
									if m == ij[2] then
--                                        lo('?? to_move_CHILD1:'..k..':'..tostring(c.base[k]))
										c.base[k] = base[ij[2]]
									elseif m == U.mod(ij[2]+1,#base) then
--                                        lo('?? to_move_CHILD2:'..k..':'..tostring(c.base[k]))
										c.base[k] = U.mod(ij[2]+1,base)
									end
								end
							end
						end
--                            U.dump(desc.afloor[ij[1]].top.achild[1], '?? child_POST:')
--[[
						local ang = U.vang(dpos, U.mod(ij[2]+1,base)-base[ij[2] ])
						if true or math.abs(ang) < small_ang or math.abs(ang-math.pi) < small_ang then
	--                    if U.vang(dpos, U.mod(ij[2]+1,base)-base[ij[2] ]) < small_ang then
							--
						else
						end
]]
	--                    desc.afloor[ij[1]].base[ij[2]] = desc.afloor[ij[1]].base[ij[2]] + dpos
					end)
				end
	--            baseOn()
	--            houseUp(nil, cedit.mesh)
	--            houseUp(adesc[cedit.mesh], cedit.mesh, true)
	--            markUp()
      else
        -- move edge
--			elseif pe == 1 and incorner then
              U.dump(incorner,'??^^^^^^^^^^^^^^^^^ PE_1:'..tostring(dpos)..':'..tostring(isperp)..' ifit:'..pe,true)
--              _dbdrag = true
--                local cfloor
--                    lo('?? if_ME:'..tostring(cedit.cval['DragRot'].dir)..':'..tostring(dpos))
				if dpos then
					local ang = U.vang(dpos, cedit.cval['DragRot'].dir)
--                        lo('?? pe_EDGE:'..incorner[1].ij[2]..' ang:'..tostring(ang)..' dp:'..tostring(dpos))
					if isperp then
--                    if math.abs(ang) < small_ang or math.abs(ang-math.pi) < small_ang then
						local ds = cedit.cval['DragRot'].dir*dpos:dot(cedit.cval['DragRot'].dir)
                lo('?? for_ds:'..tostring(ds)..':'..tostring(dpos),true)
						forBuilding(desc, function(w, ij)
		--                    if ij[1] == cfloor then return end
		--                    cfloor = ij[1]
							local floor = desc.afloor[ij[1]]
							for i,c in pairs(incorner) do
								if c.ij[1] == ij[1] and c.ij[2] == ij[2] then
--                                    lo('?? corner_MOVE:'..tostring(ij[2]))

									floor.base[ij[2]] = floor.base[ij[2]] + ds
									baseOn(floor, floor.base, ij[1])
									-- update children
									-- TODO: use childRebase?
									for _,c in pairs(floor.top.achild) do
										if c.imap then
--                                                U.dump(c.imap, '?? for_c_map:'..ij[2])
											for k,m in pairs(c.imap) do
												if m == ij[2] then
													c.base[k] = floor.base[ij[2]]
												end
											end
--[[
											U.dump(c, '?? for_c_b:')
											for j=1,#c.base do
												c.base[j] = floor.base[c.imap[j] ]
											end
]]
										end
									end
									break
								end
							end
						end)
					else
						toupdate = false
					end
				end
			end
			smouse = cmouse
--        elseif incorner then
--            U.dump(incorner, '?? drag_for_V:'..tostring(incorner))
		elseif false and U._PRD == 0 then
			selectionTurn(dx*0.02, cedit.cval['DragRot'].center)
		end
		if toupdate then
			houseUp(adesc[cedit.mesh], cedit.mesh, true)
			markUp()
		end
		return
	end
	if U._PRD == 0 and cedit.forest ~= nil then
-- FOREST MOVE ----------
		-- move forest objs
		indrag = true
		if not cedit.cval['DragPos'] then
			cedit.cval['DragPos'] = {}
		end
--[[
		if not cedit.cval['DragPos'] then
--            lo('?? FOREST:'..tostring(cameraMouseRayCast(true).object.name)..':'..tostring(cameraMouseRayCast(false).object.name)) --..':'..tostring(cedit.cval['DragPos'].mouse-rayCast.pos))
			cedit.cval['DragPos'] = {mouse = cameraMouseRayCast(true).pos}
			lo('?? FOREST:'..tostring(cedit.cval['DragPos'].mouse), true) --..tostring(cameraMouseRayCast(true).object.name)..':'..tostring(cameraMouseRayCast(false).object.name)) --..':'..tostring(cedit.cval['DragPos'].mouse-rayCast.pos))
--            smouse = rayCast.pos
			return
		end
]]
		rayCast = cameraMouseRayCast(true)
		local ds = rayCast.pos - smouse
--            lo('?? FOREST_moving:'..tostring(ds)..':'..tostring(smouse), true)
--                ds = vec3(0,0,0)
		if dforest[cedit.forest] and dforest[cedit.forest].type == 'win' then
-- WINDOWS MOVE
			local wdesc
			if not cedit.cval['DragPos'].mouse then
				cedit.cval['DragPos'].mouse = rayCast.pos
				return
			end
			local dm = rayCast.pos - cedit.cval['DragPos'].mouse
			local cw = adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]]
			local dir = (math.abs(dm:dot(cw.v:normalized())) > math.abs(dm:dot(cw.u:normalized()))) and 'z' or 'x'
--            local dir = (math.abs(ds:dot(cw.v:normalized())) > math.abs(ds:dot(cw.u:normalized()))) and 'z' or 'x'
			forBuilding(adesc[cedit.mesh], function(w, ij)
				if cedit.cval['DragPos'][U.stamp(ij)] == nil then
					-- save initial position
					cedit.cval['DragPos'][U.stamp(ij)] = {bot = w.winbot, left = w.winleft}
					return
				end
--                        lo('?? FOREST2:'..tostring(ds)..':'..U.stamp(ij))
				if dir == 'z' then
					w.winbot = cedit.cval['DragPos'][U.stamp(ij)].bot + ds:dot(cw.v:normalized())
				else
					w.winleft = cedit.cval['DragPos'][U.stamp(ij)].left + ds:dot(cw.u:normalized())
					if w.winleft < 0 then w.winleft = small_dist end
--                        lo('?? w_LEFT:'..w.winleft, true)
				end
				wdesc = w
			end)
--                lo('?? m4:'..tostring(wdesc)..':'..tostring(cedit.mesh),true)
			houseUp(adesc[cedit.mesh], cedit.mesh, true)
--                    lo('?? fofor:'..tostring(cedit.forest))
			if wdesc then
--                lo('?? pre_mark:'..tostring(cedit.mesh),true)
				markUp(wdesc.win)
			end
--            markUp(dforest[cedit.forest].item:getData():getShapeFile())
		end
	elseif rayCast.object.name ~= nil and string.find(rayCast.object.name, 'o_') == 1 then
		-- MESHES -------------
		indrag = true
		if editor.keyModifiers.shift then
--        if editor.keyModifiers.alt then
--            lo('?? mat_drag:')
			-- MATERIAL
			matMove(ds) --, rayCast.object:getID())
		elseif cedit.mesh ~= nil and cij then
			local floor = adesc[cedit.mesh].afloor[cij[1]]
			if #cedit.cval == 0 then
--                cedit.cavl = {floorhit}
--                cedit.cval = {floor.pos, rayCast.object.name}
--                cedit.cval = {im.GetMousePos().x, im.GetMousePos().y, floor.pos}
			end
--            local dx = im.GetMousePos().x - cedit.cval[1]
--            local dy = im.GetMousePos().y - cedit.cval[2]

			if scope == 'floor' and floor then
-- MOVE FLOOR
--                rayCast = cameraMouseRayCast(false, im.flags(SOTTerrain))
--                lo('?? move floor:'..tostring(camDir)..':'..dx..':'..dy..':'..tostring(ds)..':'..tostring(rayCast.pos))
				local wall = floor.awall[cij[2]]
				if wall then
					local u, v = wall.u:normalized(), wall.u:cross(vec3(0,0,1)):normalized()

					local dpos, cmouse = mouseDir(u, v)
	--                    lo('?? for_dpos:'..tostring(dpos),true)
	--                    dpos = vec3(0,0,0)
					if dpos ~= nil then
						floor.pos = floor.pos + dpos
						floor.update = true
						houseUp(adesc[cedit.mesh], cedit.mesh)
						markUp()
					end
					smouse = cmouse
				end
--                cscreen = {im.GetMousePos().x, im.GetMousePos().y}
			end
		end
--        U.dump(cedit, '?? mdrag.cedit:')
	end
end


local function select(desc, ij)
			U.dump(ij, '>> select:')
	if not desc.selection then desc.selection = {} end
	forBuilding(desc, function(w, tij)
		if not desc.selection[tij[1]] then
			desc.selection[tij[1]] = {}
		end
		if #U.index(desc.selection[tij[1]], tij[2]) == 0 then
			desc.selection[tij[1]][#desc.selection[tij[1]]+1] = tij[2]
		end
--                desc.selection[ij[1]][ij[2]] = true
	end, cij)
	if ij then
		forBuilding(desc, function(w, tij)
			if not desc.selection[tij[1]] then
				desc.selection[tij[1]] = {}
			end
			if #U.index(desc.selection[tij[1]], tij[2]) == 0 then
				desc.selection[tij[1]][#desc.selection[tij[1]]+1] = tij[2]
			end
	--                desc.selection[tij[1]][tij[2]] = true
		end, ij)
	end
--    out.selection = desc.selection
--            U.dump(desc.selection, '<< select:')
end


local function forStick()
		lo('>> forStick:', true)
-- ADJUST DROP POSITION, STICK
--                    indrag = false
--            U.dump(cij, '>> forStick:'..tostring(cedit.mesh)..':'..tostring(cedit.part)..':'..tostring(scope))
	local function vert2edge(a, base)
--            lo('>> vert2edge:'..tostring(a))
		local dmi,bmi = near_dist
		for k,q in pairs(base) do
			local d = a:distanceToLine(U.mod(k+1,base), q)
--            local d = a:distanceToLineSegment(U.mod(k+1,base), q)
			if d < dmi and d > 0.0000001 then
--                hit = 'side'
--                        lo('?? HIT_edge:'..dmi, true)
				dmi = d
--                ami = a
				local s = closestLinePoints(base[k], U.mod(k+1,base), a, a+vec3(0,0,1))
				bmi = base[k] + (U.mod(k+1,base) - base[k])*s
					lo('?? vert2edge:'..k..':'..tostring(bmi)..'/'..tostring(a)..':'..d)
--                    imi = j
			end
		end
			lo('<< vert2edge:'..tostring(a))
		return bmi,dmi
	end

	local function forNear(floor, floorpre, child)
		local dmi,d = near_dist
		local a,b,ami,bmi
		local hit = false
		for j,p in pairs(floor.base) do
			a = p + floor.pos
			if child then
				a = a + child.pos
			end
			for k,q in pairs(floorpre.base) do
				b = q + floorpre.pos
--                    lo('?? for_pair:'..j..':'..k)
				d = (a - b):length()
				if d < dmi*2 then
--                if (a - b):length() < near_dist then
--                                lo('?? HIT_vert:'..j..'>'..k..' d:'..d, true)
					dmi = d
					ami = a
					bmi = b
					hit = 'corner'
--                    return a, b
				end
			end

			if hit then break end
--            local dmi,imi = math.huge
			for k,q in pairs(floorpre.base) do
				local d = a:distanceToLineSegment(U.mod(k+1,floorpre.base), q)
				if d < dmi then
					hit = 'side'
--                        lo('?? HIT_edge:'..dmi, true)
					dmi = d
					ami = a
					local s = closestLinePoints(floorpre.base[k], U.mod(k+1,floorpre.base), a, a+vec3(0,0,1))
					bmi = floorpre.base[k] + (U.mod(k+1,floorpre.base) - floorpre.base[k])*s
--                    imi = j
				end
			end
	--                        lo('?? dmi:'..dmi..':'..imi)
--[[
			if dmi < near_dist then
					lo('?? HIT_edge:'..dmi, true)
				local s = closestLinePoints(floorpre.base[imi], U.mod(imi+1,floorpre.base), a, a+vec3(0,0,1))
	--                            lo('?? for_s:'..imi..':'..s)
				return a, floorpre.base[imi] + (U.mod(imi+1,floorpre.base) - floorpre.base[imi])*s
			end
]]
		end
		return ami,bmi,hit
	end

	local desc = adesc[cedit.mesh]
	local floor,floorpre
	local a,b,tp
	if scope == 'building' then
		if desc and desc.prn then
			local ashift = {}
			local ishit
			for i1,f1 in pairs(desc.afloor) do
				for i2,f2 in pairs(adesc[desc.prn].afloor) do
					a,b,tp = forNear(f1, f2, desc)
					if a then
						lo('?? is_hit:'..tp..':'..i1..':'..i2..':'..(b-a):length())
						if tp == 'corner' then
							ishit = true
						end
--[[
						local isperp = true
						for k,v in pairs(ashift) do
								lo('?? if diff:'..i1..':'..i2..':'..k..':'..U.vang(v, b-a)..':'..tostring(U.vang(v, b-a) % math.pi))
							isperp = false
							if math.abs(U.vang(v, b-a) % math.pi - math.pi/2) < small_ang then
								isperp = true
							else
								isperp = false
								break
							end
						end
						if isperp then
							ashift[#ashift+1] = b-a
						end
]]
					end
					if ishit then break end
				end
				if ishit then break end
			end
--[[
				lo('?? for_ashift:'..#ashift)
			floor = desc.afloor[1]
			if not floor then
				U.dump(desc.afloor, '!! NO_FLOOR:')
				return
			end
				lo('?? for_CHILD:'..tostring(desc.pos)..':'..tostring(floor.base[1])..':'..tostring(floor.pos))
			floorpre = adesc[desc.prn].afloor[desc.floor-1]
	--                        lo('?? for_CHILD2:'..tostring(floorpre.pos))
			a,b = forNear(floor, floorpre, desc)
]]
	--                        lo('?? ifhit:'..tostring(a))
		end
	elseif scope == 'floor' and desc and cij then
		floor = desc.afloor[cij[1]]
			lo('?? for_floor:'..tostring(floor.pos), true)
		if cij[1] > 1 then
			floorpre = desc.afloor[cij[1]-1]
				lo('?? floor_check:'..#floor.base..':'..#floorpre.base)
			a,b = forNear(floor, floorpre)
--            floor.pos = floor.pos + (b - a)
		end
	elseif scope == 'wall' and cij then
				lo('?? if_wall_stick:', true)
		floor = desc.afloor[cij[1]]
		forBuilding(desc, function(w, ij)
--            lo('?? ij:'..ij[1]..':'..ij[2])
			local dmi = math.huge
			local function forVert(p)
				local dfmi,bfmi = math.huge
				if ij[1] > 1 then
					-- prev floor
					local cb, cd = vert2edge(p, desc.afloor[ij[1]-1].base)
					if cd < dfmi then
						b = cb
						a = p
						dfmi = cd
					end
				end
				if ij[1] < #desc.afloor then
					local cb, cd = vert2edge(p, desc.afloor[ij[1]+1].base)
					if cd < dfmi then
						b = cb
						a = p
						dfmi = cd
					end
				end
				return dfmi
			end
			local cd = forVert(floor.base[ij[2]])
--                log('?? for_v:'..tostring(floor.base[ij[2]])..':'..cd)
			if cd < dmi then
--                    log
				dmi = cd
--                a = floor.base[ij[2]]
			end
			-- next vert
			cd = forVert(U.mod(ij[2]+1, floor.base))
--                log('?? nxt_ver:'..cd..'/'..dmi)
			if cd <= dmi then
				dmi = cd
--                a = U.mod(ij[2]+1, floor.base)
			end
		end)
--                lo('?? ab:'..tostring(a)..':'..tostring(b))
--        return
	end
	if a then
		if scope == 'wall' then
			if b then
--                    lo('?? to_stick:'..tostring(b)..':'..tostring(a))
				floor.base[cij[2]] = floor.base[cij[2]] + (b - a)
				floor.base[U.mod(cij[2]+1, #floor.base)] = floor.base[U.mod(cij[2]+1, #floor.base)] + (b - a)
				baseOn(floor)
			end
		elseif scope == 'floor' then
			floor.pos = floor.pos + (b - a)
		elseif scope == 'building' then
			desc.pos = desc.pos + (b - a)
		end
		houseUp(desc, cedit.mesh, true)
		markUp()
		indrag = false
		return
	end

end


local objLevel = 0

local function mup(rayCast, inject)
		_dbdrag = false
--		print('?? mup:')
--    out.inhole = false
	 	lo('>> mup:'..tostring(W.ui.injunction)..':'..tostring(out.inroad)..':'..tostring(indrag)..':'..tostring(cedit.forest))
	if W.ui.injunction then return end
	if out.inroad and out.inroad > 0 then return end
	if indrag and incorner and cedit.mesh and cij then
		local floor = adesc[cedit.mesh].afloor[cij[1]]
		local base = floor.base
		local apair = T.pairsUp(base)
		if apair and #apair==(#base-2)/2 then
			floor.top.poly = 'V'
		end
--[[
		lo('?? if_corner:'..tostring(incorner))
]]
	end
--            lo('?? mup.if_control:'..tostring(incontrol), true)
	local sindrag = indrag
	indrag = false
	cedit.cval['DragPos'] = nil
	cedit.cval['DragRot'] = nil
	if incontrol then
--            lo('?? mup_if_pull:'..W.ui.wall_pull..':'..tostring(incontrol))
		if incontrol == 'wall_pull' then
			forStick()
		end
--            U.dump(cedit.cval['DragVal'], '?? mup.control:')
		incontrol = false
		out.inchild = false
		cedit.cval['DragPos'] = nil
		cedit.cval['DragVal'] = nil
		W.ui.wall_pull = 0
		W.ui.building_scalex = 1
		W.ui.building_scaley = 1
		W.ui.building_ang = 0
		W.ui.floor_inout = 0
		W.ui.ang_floor = 0
		W.ui.balc_scale_width = 1
		W.ui.balc_bottom = 0
		W.ui.hole_x = 0
		W.ui.hole_y = 0
		W.ui.hole_pull = 0
		W.ui.uv_u = 0
		W.ui.uv_v = 0
		W.ui.uv_scale = 1
		if cedit.mesh then
			local desc = adesc[cedit.mesh]
			-- backup
			toJSON(desc)
		lo('??******************************** MUP_inc:'..#asave)
--[[
			if #asave > 4 then
				table.remove(asave, #asave)
--                table.remove(asave, 1)
			end
			table.insert(asave, 1, {math.floor(socket.gettime()), deepcopy(desc)})
]]
					lo('>> mup_backup:'..#asave..':'..tostring(cedit.forest)..':'..tostring(out.inseed))
--                    incontrol = 7
		end
		cedit.cval['DragMat'] = nil
		indrag = false
		return
	end
	if im.IsWindowHovered(im.HoveredFlags_AnyWindow) or im.IsAnyItemHovered() then return end
  out.preforest = cedit.forest
	cedit.forest = nil

	if sindrag then
			lo('?? if_prn_drag:'..tostring(cedit.mesh))
		if cedit.mesh and adesc[cedit.mesh] and (adesc[cedit.mesh].prn or (cij and cij[1]>1)) then
			local desc = adesc[cedit.mesh]
			local sfloor = desc.afloor[cij[1]]
			local tfloor = desc.afloor[cij[1]-1]
			-- ADJUST DROP POSITION, STICK
			local function forNear(basea, baseb, pa, pb)
				local dmi,ijmi,ds=math.huge
				for i,a in pairs(basea) do
					for j,b in pairs(baseb) do
						local d = (a+pa-b+pb):length()
						if d<0.3 then
--                            lo('?? to_STICK:'..i..':'..j..':'..d)
							if d<dmi then
								ds = -(a+pa-b+pb)
								d = dmi
--                                ijmi = {i,j}
							end
						end
					end
				end
				if not ds then
					dmi,ds=math.huge,nil
					for i,a in pairs(basea) do
						for j,b in pairs(baseb) do
							local d = U.toLine(a+pa,{b+pb,U.mod(j+1,baseb)+pb})
							if d<0.2 then
--                                lo('?? stick_to_edge:'..i..':'..j)
								if d<dmi then
									ds = d*(U.vturn(b - U.mod(j+1,baseb),math.pi/2)):normalized()
										lo('?? stick_to_edge:'..i..':'..j..':'..tostring(v))
									d = dmi
								end
							end
						end
					end
				end
				return ds
			end

			local ds
			if desc.prn then
					lo('?? for_CHILD:'..tostring(desc.prn)..':'..tostring(desc.floor))
				local dtgt = adesc[desc.prn]
				tfloor = dtgt.afloor[desc.floor-1]
				ds = forNear(sfloor.base,tfloor.base,sfloor.pos+desc.pos,tfloor.pos)
					U.dump(tfloor.base,'?? ds:'..tostring(ds)..':'..tostring(sfloor.pos)..':'..tostring(tfloor.pos)..':'..tostring(desc.pos))
			elseif scope == 'floor' then
					lo('?? for_FLOOR:'..tostring(sfloor.pos)..':'..tostring(tfloor.pos))
				ds = forNear(sfloor.base,tfloor.base,sfloor.pos,tfloor.pos)
--                    lo('?? is_NEAR:'..tostring(ds))
			end
			if ds then
				sfloor.pos = sfloor.pos+ds
				houseUp(adesc[cedit.mesh], cedit.mesh, true)
				markUp()
			end

--[[
			local function forNear(floor, floorpre, child)
				for j,p in pairs(floor.base) do
					local a = p + floor.pos
					if child then
						a = a + child.pos
					end
					local hit = false
					for k,q in pairs(floorpre.base) do
						local b = q + floorpre.pos
						if (a - b):length() < 1 then
--                                lo('?? HIT:'..j..'>'..k)
							hit = true
							return a, b
						end
					end
					if hit then break end
					local dmi,imi = math.huge
					for j,q in pairs(floorpre.base) do
						local d = a:distanceToLineSegment(U.mod(j+1,floorpre.base), q)
						if d < dmi then
							dmi = d
							imi = j
						end
					end
--                        lo('?? dmi:'..dmi..':'..imi)
					if dmi < near_dist then
						local s = closestLinePoints(floorpre.base[imi], U.mod(imi+1,floorpre.base), a, a+vec3(0,0,1))
--                            lo('?? for_s:'..imi..':'..s)
						return a, floorpre.base[imi] + (U.mod(imi+1,floorpre.base) - floorpre.base[imi])*s
					end
				end
			end
			local desc = adesc[cedit.mesh]
			local floor,floorpre
			local a,b
			if scope == 'building' then
				if desc.prn then
					floor = desc.afloor[1]
					if not floor then
						U.dump(desc.afloor, '!! NO_FLOOR:')
						return
					end
						lo('?? for_CHILD:'..tostring(desc.pos)..':'..tostring(floor.base[1])..':'..tostring(floor.pos))
					floorpre = adesc[desc.prn].afloor[desc.floor-1]
--                        lo('?? for_CHILD2:'..tostring(floorpre.pos))
					a,b = forNear(floor, floorpre, desc)
--                        lo('?? ifhit:'..tostring(a))
				end
			elseif scope == 'floor' and desc then
				floor = desc.afloor[cij[1] ]
--                    lo('?? for_floor:'..tostring(floor.pos), true)
				if cij[1] > 1 then
					floorpre = desc.afloor[cij[1]-1]
					a,b = forNear(floor, floorpre)
					floor.pos = floor.pos + (b - a)
				end
			end
			if a then
				floor.pos = floor.pos + (b - a)
				houseUp(desc, cedit.mesh, true)
				markUp()
				indrag = false
				return
			end
]]
--                    indrag = false
--                lo('?? for_DROP:'..tostring(cedit.mesh)..':'..tostring(cedit.part)..':'..tostring(scope), true)
		end
		return
	end

	if out.inseed then
		buildingGen(rayCast.pos)
		W.ui.building_shape = nil
		out.inseed = false
		return
	end
	if indrag then
			U.dump(cij, '>> mup_post_DRAG:'..tostring(cedit.cval['DragPos']))
		if U._PRD == 0 and cedit.cval['DragPos'] and cedit.cval['DragPos'].hmouse then
			local cw = adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]]
			local orig = base2world(adesc[cedit.mesh], cij)
			local un,vn = cw.u:normalized(),cw.v:normalized()
			local rc = {}
				U.dump(out.ahole, '??__________ inhole:'..tostring(base2world(adesc[cedit.mesh], cij))..':'..tostring(cw.u))
			for i=1,4 do -- in pairs(out.ahole) do
				rc[#rc+1] = world2wall(out.ahole[i], cw)
--                rc[#rc+1] = vec3((out.ahole[i] - orig):dot(un), (out.ahole[i] - orig):dot(vn), 0)
			end
			if not cw.achild then
				cw.achild = {}
			end
			out.inhole = cw
			cw.achild[#cw.achild+1] = {base = rc, body = nil, yes = false}
--            cw.achild.cur = #cw.achild
--                U.dump(rc, '?? for_rc:')
		elseif cedit.cval['DragRot'] and scope == 'floor' then
			local desc = adesc[cedit.mesh]
			local cu = (desc.afloor[cij[1]].base[2] - desc.afloor[cij[1]].base[1]):normalized()
				lo('?? mup.DragRot:'..tostring(cedit.cval['DragRot'].u)..':'..tostring(cu), true)
			local ang = U.vang(cedit.cval['DragRot'].u, cu, true) % (2*math.pi)
				lo('?? for_ang:'..tostring(ang)..':'..tostring(cedit.cval['DragRot'].center), true)
			for i = 0,8 do
				local da = math.pi/4*i - ang
						lo('?? da:'..i..':'..da, true)
				if math.abs(da) < 0.07 then
						lo('?? to_stck:'..i..':'..da, true)
					selectionTurn(da, cedit.cval['DragRot'].center)
					houseUp(adesc[cedit.mesh], cedit.mesh, true)
					markUp()
					break
				end
			end
--            selectionTurn(math.pi/2, cedit.cval['DragRot'].center)


--                cedit.cval['DragRot'].u = (desc.afloor[cij[1]].base[2] - desc.afloor[cij[1]].base[1]):normalized()
		end
		if not cedit.cval['DragRot'] then
			forStick()
		end
		smouse = nil
		cedit.cval['DragPos'] = nil
		cedit.cval['DragRot'] = nil
		cedit.cval['DragMat'] = nil
		out.invertedge = nil
		indrag = false
--        if out.invertedge then
--        end
		return true
	end
	rayCast = cameraMouseRayCast(true)  -- true to detect forest
--    rayCast = cameraMouseRayCast(false)
			lo('>> mup0:', true)
	if not rayCast then return end
	local nm = rayCast.object.name
			lo('>>******************* mup:'..tostring(nm)..':'..tostring(scope)..':'..tostring(cedit.mesh), true)
	if not nm then return end
--  if nm == forestName and scope == 'top'
	local id = scenetree.findObject(nm):getID()
			lo('?? for_ID:'..tostring(id)..'/'..tostring(cedit.mesh))
	if scope == 'building' and string.find(nm, 'o_') == 1 and id ~= cedit.mesh and editor.keyModifiers.ctrl then
		-- append to selection
		lo('?? to_APP:'..#asel)
		if #asel == 0 then
			asel[#asel+1] = cedit.mesh
		end
		asel[#asel+1] = id
		markUp()
		return
	else
		asel = {}
	end
	local ray = getCameraMouseRay()

	local ij,desc
	if cedit.mesh then
		desc = adesc[cedit.mesh]
--            lo('?? if_DESC:'..desc.id)
--            if string.find(nm, 'o_lid_') == 1 then
--                lo('??___________ if_AF:'..tostring(#desc.afloor[#desc.afloor].top.achild),true)
--            end
		ij = forHit(ray, adesc[cedit.mesh])
			U.dump(ij, '?? mup_post_HIT:')
		if ij then cij = ij end
		if cij and cij[3] then
			-- hit awplus
			scopeOn('wall')
			markUp()
			return
		end
		--------------------
		-- SHIFT
		--------------------
		if editor.keyModifiers.shift then
			if U._PRD == 0 then
				if out.aedge then
							U.dump(out.aedge, '?? for_edge:'..tostring(out.aedge.top))
					if out.inedge and out.inedge[1] == out.aedge.ij[1] and out.inedge[2] == out.aedge.ij[2] then
						out.inedge = nil
					else
							U.dump(out.aedge.ij, '?? PIN:')
						out.inedge = out.aedge.ij
					end
					return
				elseif out.axis then
		--                    lo('?? axis_HIT:'..tostring())
					if (rayCast.pos - out.axis.pos):length() < 0.4 then
	-- FLIP AROUND by fraction
		--                    lo('?? axis_HIT:')
						if ({floor=1,building=1})[scope] then
							local dir = 1
							local desc = adesc[cedit.mesh]
					--                    U.dump(cij, '?? for_cij:'..tostring(desc))
							local cfloor
							forBuilding(desc, function(w, ij)
					--                        lo('?? for_ij:'..ij[1]..':'..ij[2])
					--                    if ij[1] == cfloor then return end
								if ij[1] ~= cfloor then
					--                            lo('?? for_floor:'..ij[1])
									-- flip base
									cfloor = ij[1]
									local floor = desc.afloor[ij[1]]
									local c = U.proj2D(out.axis.pos - (base2world(desc, {cfloor,1}) - floor.base[1]))
									for i = 1,#floor.base do
										floor.base[i] = c + U.vturn(floor.base[i] - c, dir*2*math.pi/W.ui.ang_fraction)
									end
								end
								-- adjust u's
								w.u = U.vturn(w.u, dir*2*math.pi/W.ui.ang_fraction)
							end)
									lo('?? flipped:'..scope..':'..tostring(cij[1]))
							houseUp(adesc[cedit.mesh], cedit.mesh, true)
							markUp()
					--                out.inaxis = nil
							return
						end

	--[[
						if out.inaxis then
							out.inaxis = nil
						else
							out.inaxis = out.axis.pos
						end
	]]
						return
					end
				end
			end
			if out.middle and rayCast.pos:distanceToLine(out.middle.line[1], out.middle.line[2]) < 0.4 then
-- TO MIDDLE
					lo('?? to_MIDDLE:',true)
				local floor = adesc[cedit.mesh].afloor[out.middle.ij[1]]
				if scope == 'floor' then
					local awall = {}
					local aij = forSide(cij)
					for i,r in pairs(aij) do
						if i == cij[1] then
							local jmi = math.huge
							for _,j in pairs(r) do
								awall[#awall+1] = floor.awall[j]
								if j < jmi then jmi = j end
							end
--                                lo('?? awall:'..#awall..':'..jmi)
							for k = 0,#awall-1 do
								floor.awall[jmi+k] = awall[#awall-k]
							end
							for j = jmi+1,jmi+#awall-1 do
								floor.base[j] = floor.base[j-1] + floor.awall[j-1].u
							end
						end
					end
--                    local abase
				elseif scope == 'wall' then
					local j = out.middle.ij[2]
--                    local floor = adesc[cedit.mesh].afloor[out.middle.ij[1]]
					local base = floor.base
					local pm = (U.mod(j-1,base) + U.mod(j+2, base))/2
		--            local pm = (U.mod(j-2,base) + U.mod(j+3, base))/2
--                        U.dump(base, '?? to_MIDDLE:')
					local um = (U.mod(j+1,base) - base[j])/2
					base[j] = pm - um
					base[U.mod(j+1,#base)] = pm + um
				end
--                    if true then return end
				baseOn(floor)
				houseUp(adesc[cedit.mesh], cedit.mesh, true)
				markUp()
				return
			end
		elseif editor.keyModifiers.alt and not incorner then
			---------------------------
			-- ALT
			---------------------------
-- split
				U.dump(out.wsplit, '?? for_split_alt:'..tostring(scope)..':'..id..':'..tostring(aedit[id]))
			if aedit[id] then
				local floor = desc.afloor[cij[1]]
				if scope == 'top' then
					local cbase = floor.base
					if floor.top.cchild then
						cbase = floor.top.achild[floor.top.cchild].base
					end
					topSplit()
					intopchange = true
--                        U.polyCut(cbase, aint[1][1], aint[2][1])
				else
					cij = U.clone(aedit[id].desc.ij)
					roofSet('flat')
--[[
					floor.top.ridge.on = false
					floor.top.shape = 'flat'
]]
					floor.top.achild = {}
--                        U.dump(floor.top.achild, '?? childs:')
					wallSplit()
				end
			end
			return
--[[
		elseif not ij and editor.keyModifiers.ctrl then
-- BUILDING gen
				lo('?? to_GEN:')
			buildingGen(rayCast.pos)
			return
]]
		elseif editor.keyModifiers.ctrl and ij and not sindrag then
-- APPEND to selection
				U.dump(ij, '?? to_APPEND:')
			select(desc, ij)
--[[
			if not desc.selection then desc.selection = {} end
			forBuilding(desc, function(w, tij)
				if not desc.selection[tij[1] ] then
					desc.selection[tij[1] ] = {}
				end
				desc.selection[tij[1] ][#desc.selection[tij[1] ]+1] = tij[2]
--                desc.selection[ij[1] ][ij[2] ] = true
			end, cij)
			forBuilding(desc, function(w, tij)
				if not desc.selection[tij[1] ] then
					desc.selection[tij[1] ] = {}
				end
				desc.selection[tij[1] ][#desc.selection[tij[1] ]+1] = tij[2]
--                desc.selection[tij[1] ][tij[2] ] = true
			end, ij)
]]
			markUp()
			return ij
		elseif desc then
			desc.selection = nil
		end
	end
		U.dump(ij, '?? mup.for_hit:'..nm..':'..tostring(editor.keyModifiers.ctrl))
	if editor.keyModifiers.ctrl and not sindrag then --and U._MODE ~= 'conf' then
            lo('?? ifGEN:'..tostring(ij)..':'..tostring(nm)..':'..tostring(out.inroad))
		if not ij and (not out.inroad or out.inroad == 0) then
			-- BUILDING gen
				lo('?? to_GEN:'..tostring(W.ui.injunction))
			if W.ui.injunction then
--				local d = D.junctionUp(rayCast.pos, W.ui.branch_n)
			else
				local desc = buildingGen(rayCast.pos)
				toJSON(desc)
			end
--                lo('?? post_GEN:'..#adesc)
			return
		end
	end
  		lo('?? mup_pre:'..tostring(nm)..'/'..tostring(forestName)..':'..tostring(cedit.mesh)..':'..tostring(cij)) --..' cij:'..cij[1]..':'..cij[2])
--    local fkey = forKey(rayCast)
--  		lo('?? mup_pre:'..tostring(fkey)..':'..':'..tostring(nm)..'/'..tostring(forestName)..':'..tostring(cedit.mesh)..':'..tostring(cij)) --..' cij:'..cij[1]..':'..cij[2])
--    if nm ~= forestName then
--        cedit.forest = nil
--    end
	if nm == forestName and cedit.mesh then
		local key = forKey(rayCast)
--        local rCast = cameraMouseRayCast(false)
			lo('?? for_forest:'..tostring(scope)..':'..tostring(key)..'/'..tostring(cedit.forest)..':'..tostring(cedit.fscope)) --..':'..tostring(rCast.object.name)) --..':'..tostring(dforest[key].mesh)..':'..tostring(cedit.fscope))
		if not key then return end
    if scope == 'top' then
      if not ({})[dforest[key].type] then
        return
      end
    end
		if dforest[key] and dforest[key].type == 'balcony' then
			out.inbalcony = true
		end
		if key == out.preforest and cedit.fscope and scope == 'wall' then
--		if cedit.forest == key and cedit.fscope then
			-- switch forest scope
			cedit.fscope = cedit.fscope - 1
					lo('?? tof1:'..tostring(cedit.fscope)..':'..tostring(cedit.forest), true)
			if cedit.fscope == 0 then
				cedit.forest = nil
				markUp()
			else
        cedit.forest = out.preforest
				local dae = U.forOrig(dforest[key].item:getData():getShapeFile())
				markUp(dae)
			end
		elseif not (dforest[key].type == 'door' and scope == 'building') then
				lo('?? tof2_:'..key..':'..dforest[key].type..':'..tostring(dforest[key].mesh)..':'..tostring(cedit.mesh), true)
--                U.dump(dforest[key], '?? tof2:'..dforest[key].type, true)
			local dae = U.forOrig(dforest[key].item:getData():getShapeFile())
			local tp = dforest[key].type
			local fornew
			if cedit.mesh and cedit.mesh ~= dforest[key].mesh and dforest[key].mesh then
--                    lo('?? ')
--                houseUp(nil, id)
				local newmesh = dforest[key].mesh
				-- close previous edit
				houseUp(adesc[cedit.mesh], cedit.mesh)
				aedit = {}

				cedit.mesh = newmesh
				-- start new edit
				fornew = true
--                houseUp(nil, dforest[key].mesh)
--                houseUp(adesc[cedit.mesh], cedit.mesh)
--                markUp()
				-- clean previous edit
--                aedit = {}
			end
			cedit.fscope = 2 -- multiple forest select
			cedit.forest = key
--          lo('?? ce_f:'..tostring(cedit.forest))
--            local dae = U.forOrig(dforest[key].item:getData():getShapeFile())
			local sij
			forBuilding(adesc[cedit.mesh], function(w, ij)
				if sij then
					return
				end
--                    U.dump(w.df[dae], '?? for_key:'..dae)
--                        lo('??_____________________ for_DF:'..tostring(desc.afloor[ij[1]].df))
--                if #U.index(desc.afloor[ij[1]].df[dae], key) > 0 then
--                    sij = ij
--                end
				if #U.index(w.df[dae], key) > 0 then
					sij = ij
				end
				if sij then
					out.curselect = daeIndex('door', dae)
--                    lo('??__________________________ cursel:'..tostring(out.curselect), true)
				end
			end, {})
--                lo('?? found:'..tostring(out.curselect),true)
				U.dump(out.curselect,'?? cs:'..tostring(sij)..':'..dae) --..':'..dforest[key].type)
--                lo('?? di:'..daeIndex(dforest[key].type, dae))
			out.curmselect = daeIndex(tp, dae) - 1
			if sij then cij = sij end
			if fornew then
				houseUp(nil, cedit.mesh)
			end
			markUp(dae)
          lo('?? ce_f2:'..tostring(cedit.forest))
		end
    out.fscope = cedit.fscope
--        if dforest[key] and dforest[key].type == 'balcony' then
--            out.inbalcony = true
--        end
--            lo('?? to_TYPE:'..dforest[key].type)
	elseif string.find(nm, 'o_building_') == 1 then
		-- start edit
			lo('?? if_edit:'..tostring(cedit.mesh)..':'..tostring(id)..':'..tableSize(adesc))
--      for k,d in pairs(adesc) do
--        lo('?? if_DESC:'..k..':'..tostring(adesc[k]))
--      end
		if cedit.mesh and cedit.mesh ~= id then
--[[
			if false then
				-- clean previous edit
				aedit = {}
				--- parts
				groupEdit:deleteAllObjects()
				--- forest
				local desc = adesc[cedit.mesh]
				for i,f in pairs(desc.afloor) do
					for i,w in pairs(f.awall) do
						forestClean(w)
					end
					forestClean(f.top)
				end
--                adesc[cedit.mesh] = nil
			end
]]
			-- clean previous edit
			houseUp(adesc[cedit.mesh], cedit.mesh)
			markUp()
			aedit = {}
		end

		houseUp(nil, id)
--        scope = 'building'
		ij = forHit(ray, adesc[cedit.mesh])
			U.dump(ij, '?? for_desc:'..id..':'..tostring(adesc[id])..':'..tostring(cedit.mesh)..' cursel:'..tostring(out.curselect))
		if ij then
			cij = ij
		end
		scopeOn('building')
--[[
	elseif string.find(nm, 'o_lid_') == 1 then
			U.dump(cij, '?? mup.for_LID:')
		if cij and adesc[cedit.mesh] then
			local floor = adesc[cedit.mesh].afloor[cij[1] ]
			if floor and #floor.top.achild > 0 then
--                    U.dump(floor.top.achild,'?? childs:'..#floor.top.achild..':'..cij[1])
				local scchild = floor.top.cchild
				lo('?? RH1:')
				local _,inrc = forRoofHit(adesc[cedit.mesh], cij[1])
				if inrc ~= nil then
					for i,c in pairs(floor.top.achild) do
						U.dump(c, '?? for_LIST:'..i..':'..inrc..' scch:'..tostring(scchild)..':'..#U.index(c.list, inrc))
						if #U.index(c.list, inrc) > 0 then
--                                lo('?? cc:'..floor.top.cchild)
							if scchild ~= nil then
								floor.top.cchild = nil
							else
								floor.top.cchild = i
							end
							break
						end
					end
				end
					lo('?? inrc:'..tostring(inrc)..':'..tostring(floor.top.cchild))
--                        U.dump(floor.top.achild, '?? achild:')
				markUp()
			end
		end
]]
	else
		out.inbalcony = false
		local scopenew = scope
			U.dump(ij, '??__________________ mup_SCOPE:'..tostring(scope)..':'..tostring(ij))
--            U.dump(cij, '?? cij:')
		if ij then
			local cw = desc.afloor[ij[1]].awall[ij[2]]
			if U._PRD == 0 and cw then
				local ihole = U.inRC(world2wall(rayCast.pos, cw), cw.achild, 'base')
	--                    U.dump(cw.achild, '?? if_HOLE:'..tostring(ihole)..':'..tostring(world2wall(rayCast.pos, cw)))
				if ihole then
					cw.achild.cur = ihole
						U.dump(cw.achild[cw.achild.cur], '?? in_hole:'..ihole)
					out.ahole = {}
					-- dforest[key].type
					for i,p in pairs(cw.achild[ihole].base) do
	--                    lo('?? for_p:'..tostring(cw.pos)..':'..tostring(p)..':'..tostring(cw.u)..':'..tostring(cw.v))
	--                    lo('?? for_p2:'..tostring(vec3(p.x*cw.u:normalized(), p.y*cw.v:normalized(),0)))
						out.ahole[#out.ahole+1] = cw.pos + p.x*cw.u:normalized() + p.y*cw.v:normalized()
					end
					out.ahole[#out.ahole+1] = out.ahole[1]
	--                    U.dump(out.ahole, '?? hole:'..ihole)
					out.inhole = cw
					return
				else
--                    U.dump(cij, '?? cij:')
--                    U.dump(ij, '?? ij:')
--                    U.dump(cw.achild, '?? if_empty:')
					-- clean unfilled holes
					local pw = cij and #cij>1 and desc.afloor[cij[1]].awall[cij[2]] or cw
					if pw.achild and #pw.achild > 0 and not pw.achild[#pw.achild].body then
						table.remove(pw.achild, #pw.achild)
					end
					out.ahole = nil
					out.inhole = nil
				end
			end
--                    out.ahole = nil
--            for _,c in pairs(cw.achild) do
--            end
			cij = ij
			cedit.forest = nil
			if U._PRD == 0 and not U._MODE == 'ter' then
				inject.D.unselect()
			end
				lo('?? if_ij2:'..ij[1]..':'..tostring(ij[2]))
			if not ij[2] then
				scopenew = 'top'
				local floor = adesc[cedit.mesh].afloor[cij[1]]
--            lo('?? if_tCHILD:'..tostring(floor and #floor.top.achild or nil))
				if floor and #floor.top.achild > 0 then
						lo('?? if_child:'..#floor.top.achild)
					local scchild = floor.top.cchild
					local _,inrc,ichild = forRoofHit(adesc[cedit.mesh], cij[1])
						lo('??++++++++++++++++ mup.RH1:'..tostring(inrc)..':'..tostring(ichild))
					if ichild then
						if floor.top.cchild == ichild then
							floor.top.cchild = nil
						else
							floor.top.cchild = ichild
						end
--                            W.ui.dbg = true
					elseif inrc ~= nil then
						for i,c in pairs(floor.top.achild) do
--                            U.dump(c, '?? for_LIST:'..i..':'..inrc..' scch:'..tostring(scchild)..':'..#U.index(c.list, inrc))
							if #U.index(c.list, inrc) > 0 then
	--                                lo('?? cc:'..floor.top.cchild)
								if scchild ~= nil then
									floor.top.cchild = nil
								else
									floor.top.cchild = i
								end
								break
							end
						end
					end
						lo('?? inrc:'..tostring(inrc)..':'..tostring(floor.top.cchild)..':'..scopenew)
	--                        U.dump(floor.top.achild, '?? achild:')
					scopeOn(scopenew)
					markUp()
						W.ifRoof('pyramid')
					return
				end
      -- SCOPE SELECTION
			elseif editor.keyModifiers.shift then -- U._PRD == 0 then
				local wallid = adesc[cedit.mesh].afloor[ij[1]].awall[ij[2]].id
				local issame = (cedit.part == wallid)
				if issame or scope == 'building' then
					local ascope = {'building', 'floor', 'wall'}
					local iscope = U.index(ascope, scope)[1]
					if iscope then
						scopenew = ascope[iscope % #ascope + 1]
					end
				end
				cedit.part = wallid
				if scope == 'top' then
					scopenew = 'floor'
				end
			end
			scopeOn(scopenew)
			local matmatch = true
			if scope ~= 'top' then
				local cmat = out.curselect and tostring(dmat.wall[out.curselect+1]) or nil
--            lo('?? mup_mat:'..tostring(cmat)..':'..tostring(out.curselect))
	--                U.dump(cij, '?? pre_SCOPE:'..scopenew..':'..tostring(out.curselect)..':'..tostring(dmat.wall[out.curselect or 0])..':'..tostring(cmat))
				forBuilding(desc, function(w, ij)
	--                    lo('?? mat_check:'..tostring(ij[1])..':'..tostring(ij[2])..':'..tostring(w.mat)..':'..tostring(dmat.wall[out.curselect])..':'..tostring(cmat==nil))
					if not cmat then
						cmat = w.mat
					end
					if w.mat ~= cmat then
	--                if w.mat ~= tostring(dmat.wall[out.curselect]) then
						matmatch = false
					end
				end)
				if not matmatch then
					out.curselect = false
				else
					for i,m in pairs(dmat.wall) do
						if tostring(m) == cmat then
							out.curselect = i-1
							break
						end
					end
				end
			end
		end
	end
	if U._PRD == 0 and not scope and not ij and not ({conf=0,ter=0})[U._MODE] then
		if im.IsWindowHovered(im.HoveredFlags_AnyWindow) or im.IsAnyItemHovered() then return end
		local rayCast = cameraMouseRayCast(true)
		if rayCast and not N.out.inseed and not W.ui.injunction then
				lo('?? to_SEED:'..tostring(rayCast.pos))
			N.circleSeed(W.ui.node_n, vec3(rayCast.pos.x, rayCast.pos.y), W.ui.node_r)
			N.pathsUp()
--            N.toDecals()
		end
	end
		U.dump(cij, '<< mup:'..tostring(scope)..':'..tostring(ij)..':'..tostring(cedit.mesh)..' forest:'..tostring(cedit.forest)..':'..#adesc..' cursel:'..tostring(out.curselect))
	return ij
end


local function mup_(rayCast, inject)
		lo('>> mup_:'..tostring(rayCast)..':'..tostring(indrag), true)

		if true then
			lo('<< ??mup:')
			return
		end

	if im.IsWindowHovered(im.HoveredFlags_AnyWindow) or im.IsAnyItemHovered() then return end
	smouse = nil
	cedit.cval['DragPos'] = nil
	cedit.cval['DragRot'] = nil
--            _dbdrag = false
	if out.acorner then
--    if out.invertedge then
		out.invertedge = nil
		indrag = false
		return
	end
	if rayCast == nil then return end

	--------------------
	-- SHIFT
	--------------------
	if editor.keyModifiers.shift then
		if out.aedge then
			if out.inedge and out.inedge[1] == out.aedge[1].ij[1] and out.inedge[2] == out.aedge[1].ij[2] then
				out.inedge = nil
			else
					U.dump(out.aedge[1].ij, '?? PIN:')
				out.inedge = out.aedge[1].ij
			end
			return
		elseif out.axis then
--                    lo('?? axis_HIT:'..tostring())
			if (rayCast.pos - out.axis.pos):length() < 0.4 then
--                    lo('?? axis_HIT:')
				if out.inaxis then
					out.inaxis = nil
				else
					out.inaxis = out.axis.pos
				end
				return
			end
		end
		if out.middle and rayCast.pos:distanceToLine(out.middle.line[1], out.middle.line[2]) < 0.4 then
-- TO MIDDLE
			local j = out.middle.ij[2]
			local floor = adesc[cedit.mesh].afloor[out.middle.ij[1]]
			local base = floor.base
			local pm = (U.mod(j-1,base) + U.mod(j+2, base))/2
--            local pm = (U.mod(j-2,base) + U.mod(j+3, base))/2
				U.dump(base, '?? to_MIDDLE:')

			local um = (U.mod(j+1,base) - base[j])/2
			base[j] = pm - um
			base[U.mod(j+1,#base)] = pm + um
--            base[U.mod(j-1,#base)] = base[j]
--            base[U.mod(j+2,#base)] = base[U.mod(j+1,#base)]

--            floor.awall[U.mod(j-2,#base)].u = U.mod(j-1,base) - U.mod(j-2,base)
--            floor.awall[U.mod(j+2,#base)].u = U.mod(j+3,base) - U.mod(j+2,base)
--[[
			-- update u-s
			for j = 1,#floor.base do
				floor.awall[j].u = floor.base[j % #floor.base + 1] - floor.base[j]
			end

--            for i,f in pairs(desc.afloor) do
--            end
			-- update UV's
			uvOn()
]]
			baseOn(floor)
			houseUp(adesc[cedit.mesh], cedit.mesh, true)
			markUp()
			return
		end
	end

	if not editor.keyModifiers.ctrl then
		if D.forRoad() then
			scope = nil
	--        out.scope = nil
	--        inject.D.unselect()
			return
		else
			out.scope = scope
		end
	end

--    rayCast = cameraMouseRayCast(true)
	local nm = rayCast.object.name
		lo('?? mup0:'..tostring(nm)..':'..tostring(scope)..':'..tostring(indrag)..':'..tostring(cedit.part)..':'..tostring(cmesh)..':'..tostring(Engine.Render.DecalMgr.getDecalInstance(1))) --#Engine.Render.DecalMgr.getSet():getObjects())) --..tostring(editor.getDecalTemplates():size())) --..rayCast.object.name)
	if nm == nil then return end

	if nm == 'theTerrain' and not indrag and editor.keyModifiers.ctrl and not im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
		lo('?? to_create:')
		out.apick = nil
		out.apoint = nil
		buildingGen(rayCast.pos)
	--           scope = 'building'

	--                    local amesh = M.dae2proc('/assets/roof_border/s_RES_S_01_RTR_EEL_01_2.dae')
	--                    local id, om = meshUp(amesh)
	--                    om:setPosition(vec3(0, 0, 3))

		lo('<< mdown:'..rayCast.object.name)
		return cedit.mesh
	end

	if string.find(nm, 'o_test_') == 1 and not indrag then
		-- select face
		local id = rayCast.object:getID()
		cmesh = id
		if editor.keyModifiers.shift then
			-- select face
			local amesh,c,path = M.pop(dmesh[id].data, dmesh[id].sel)
				lo('?? FOR_MESH:'..nm..':'..tostring(id)..':'..tostring(rayCast.pos)..':'..tostring(c))
			if amesh == nil or c == nil then
				lo('!! mup_NOSELECT:'..tostring(amesh))
				return
			end
			dmesh[id].obj:createMesh({})
			dmesh[id].obj:createMesh({amesh})
			ccenter = c
			cface = path
			out.apath = {path}
			out.avedit = {}
--[[
			for ord,m in pairs(dmesh[id].sel) do
				for _,f in pairs(m.faces) do
					out.avedit[#out.avedit + 1] = m.verts[f.v + 1]
				end
			end
]]
--            out.apoint = {ccenter}
--            out.avedit = {ccenter}
			local d = (c - core_camera.getPosition()):length()
			local speed = 100*math.pow(1/(1+1/d), 4)
			core_camera.setSpeed(speed)
				lo('?? UPDTD:'..id) --..':'..tostring(c)..':'..speed)
--                U.dump(dmesh[id].sel[1].ref, '?? for_REF:')
		else
			local hit = M.ifHit(dmesh[cmesh].sel)
			if hit > 0 then
				-- mark mdata part
					lo('?? HIT:'..hit..'/'..#dmesh[cmesh].sel)
				local m = dmesh[cmesh].sel[hit]
				out.avedit = {}
				local apick = {}
				for _,f in pairs(m.faces) do
					if #U.index(apick, f.v) == 0 then
						apick[#apick+1] = f.v -- m.verts[f.v+1]
						out.avedit[#out.avedit+1] = m.verts[f.v+1]
					end
				end
				dmesh[cmesh].apick = apick
			else
				dmesh[cmesh].apick = nil
				-- mark object
				U.boxMark(rayCast.object, out)
			end
--[[
			local ob = rayCast.object:getWorldBox()
			out.avedit = {}
			for _,x in pairs({ob.minExtents.x, ob.maxExtents.x}) do
				for _,y in pairs({ob.minExtents.y, ob.maxExtents.y}) do
					for _,z in pairs({ob.minExtents.z, ob.maxExtents.z}) do
						out.avedit[#out.avedit+1] = vec3(x, y, z)
					end
				end
			end
]]
		end
		out.frame = nil
			lo('<< mup_mesh:'..tostring(cmesh))
		return
	elseif indrag then
		if cmesh and not editor.keyModifiers.ctrl then
			-- end selection
			indrag = false
			out.avedit = {} -- {ccenter}
			cedit.cval['Drag_Z'] = nil
			cedit.cval['Drag'] = nil
			if out.frame ~= nil then
				ccenter = vec3(0,0,0)
				for i = 0,1 do
					for j = 0,1 do
						for k = 0,1 do
							ccenter = ccenter + out.frame[1] + (i-1)*out.frame[2] + (j-1)*out.frame[3] + (k-1)*out.frame[4]
						end
					end
				end
				ccenter = ccenter/8
				local d = (ccenter - core_camera.getPosition()):length()
				local speed = 100*math.pow(1/(1+1/d), 4)
				core_camera.setSpeed(speed)
			end
	--        out.frame = nil
	--            U.dump(dmesh[cmesh].incube[1].ref, '?? cube_OFF:')
	--            U.dump(dmesh[cmesh].sel[1].faces, '?? sel:')
	--        dmesh[cmesh].sel = dmesh[cmesh].incube
	--        dmesh[cmesh].incube = {}
	--        cmesh = nil
			return
		elseif adesc[cedit.mesh] then
-- ADJUST DROP POSITION, STICK
--                    indrag = false
					lo('?? for_DROP:'..tostring(cedit.mesh)..':'..tostring(cedit.part)..':'..tostring(scope), true)
			local function forNear(floor, floorpre, child)
				for j,p in pairs(floor.base) do
					local a = p + floor.pos
					if child then
						a = a + child.pos
					end
					local hit = false
					for k,q in pairs(floorpre.base) do
						local b = q + floorpre.pos
						if (a - b):length() < 1 then
--                                lo('?? HIT:'..j..'>'..k)
							hit = true
							return a, b
						end
					end
					if hit then break end
					local dmi,imi = math.huge
					for j,q in pairs(floorpre.base) do
						local d = a:distanceToLineSegment(U.mod(j+1,floorpre.base), q)
						if d < dmi then
							dmi = d
							imi = j
						end
					end
--                        lo('?? dmi:'..dmi..':'..imi)
					if dmi < near_dist then
						local s = closestLinePoints(floorpre.base[imi], U.mod(imi+1,floorpre.base), a, a+vec3(0,0,1))
--                            lo('?? for_s:'..imi..':'..s)
						return a, floorpre.base[imi] + (U.mod(imi+1,floorpre.base) - floorpre.base[imi])*s
					end
				end
			end
			local desc = adesc[cedit.mesh]
			local floor,floorpre
			local a,b
			if scope == 'building' then
				if desc.prn then
					floor = desc.afloor[1]
					if not floor then
						U.dump(desc.afloor, '!! NO_FLOOR:')
						return
					end
						lo('?? for_CHILD:'..tostring(desc.pos)..':'..tostring(floor.base[1])..':'..tostring(floor.pos))
					floorpre = adesc[desc.prn].afloor[desc.floor-1]
--                        lo('?? for_CHILD2:'..tostring(floorpre.pos))
					a,b = forNear(floor, floorpre, desc)
--                        lo('?? ifhit:'..tostring(a))
				end
			elseif scope == 'floor' and desc then
				floor = desc.afloor[cij[1]]
--                    lo('?? for_floor:'..tostring(floor.pos), true)
				if cij[1] > 1 then
					floorpre = desc.afloor[cij[1]-1]
					a,b = forNear(floor, floorpre)
--[[
							b = base2world(desc, {cij[1]-1,1}, b) + vec3(0,0,1)
							out.apoint = {b, base2world(desc, {2, 1})}
--                            out.apick = {b}
							lo('?? for_hit:'..tostring(b))
							return
]]
					floor.pos = floor.pos + (b - a)
				end
			end
			if a then
				floor.pos = floor.pos + (b - a)
				houseUp(desc, cedit.mesh, true)
				markUp()
				indrag = false
				return
			end
		end
	end
--    if indrag then
--    end

	if nm ~= 'theTerrain' then
		cmesh = nil
		if cedit.part ~= nil then
			local obj = scenetree.findObjectById(cedit.part)
			if obj ~= nil then
				nm = obj.name
			end
		end
	elseif cedit.mesh ~= nil then
		-- check for hitting roof
		lo('?? mup_checkhits:')
--        local roofid = forRoofHit(adesc[cedit.mesh])
--        partOn(roofid)
--[[
		if false then
			local lastFloor = adesc[cedit.mesh].afloor[#adesc[cedit.mesh].afloor]
			if forRoofHit(adesc[cedit.mesh]) then
				partOn(lastFloor.top.id)
			end
			local ray = getCameraMouseRay()
			local vn = vec3(0, 0, 1)
			local h = forHeight(adesc[cedit.mesh].afloor)
			local d = intersectsRay_Plane(ray.pos, ray.dir, vec3(0, 0, h), vn)
			local p = ray.pos + d*ray.dir - adesc[cedit.mesh].pos - lastFloor.pos
			p.z = 0
	--            U.dump(lastFloor.top.body, '?? mup.floors:'..#adesc[cedit.mesh].afloor..':'..tostring(p))
			local inrc = U.inRC(p, lastFloor.top.body)
	--            lo('?? INRC:'..tostring(inrc))
			if inrc ~= nil then
				lo('?? ROOF_HIT:'..lastFloor.top.id)
				partOn(lastFloor.top.id)
			end
		end
]]
	end

--    local rayCast = cameraMouseRayCast(true)
	cedit.cval = {}
	if not indrag and not intopchange then
--            lo('?? mup_BODY:')
--[[
		if screen ~= nil then
			screen:delete()
			screen = nil
		end
]]
		if false and editor.keyModifiers.shift then
			lo('?? to_SIDE:')
			scopeOn('side')
			return
		end
--            lo('?? mup_newe:'..tostring(newedit))
		-- check for same
		if nm == forestName then
			local key = forKey(rayCast)
				lo('?? mup_forest:'..tostring(newedit)..':'..tostring(indrag)..':'..tostring(key)..'/'..tostring(cedit.forest))
			if editor.keyModifiers.ctrl then
-- APPEND to FOREST SELECTION
					U.dump(cij, '?? app_FOREST:'..key..':'..dforest[key].type)
				local sij
				local desc = adesc[cedit.mesh]
				forBuilding(desc, function(w, ij)
					for d,list in pairs(w.df) do
						for _,k in ipairs(list) do
							if k == key then
--                                lo('?? hit:')
								sij = U.stamp(ij, true)
							end
						end
					end
				end, true)
				if sij then
					if not desc.selection then
						desc.selection = {U.stamp(cij,true)}
					end
					if #U.index(desc.selection, sij) == 0 then
						desc.selection[#desc.selection+1] = sij
					end
						U.dump(desc.selection, '?? f_sle:')
					markUp(nil, dforest[key].type)
				end
			end
			if cedit.forest == key and not newedit then
				-- cancel edit
					lo('??_____________________________ EDIT_STOP:')
--                cedit.mesh = nil
				cedit.forest = nil
				markUp()
				return true --false
			end
		elseif not newedit then
--                    U.dump(cij, '?? NARROW_DOWN:'..nm)
			if cedit.part ~= nil then
				local obj = scenetree.findObjectById(cedit.part)
				if obj ~= nil and string.find(obj.name, 'o_lid_') == 1 then nm = obj.name end
			end
--                   U.dump(cij, '?? NARROW_DOWN2:'..nm)
--            lo('?? mup_scope:'..objLevel)
			if string.find(nm, '_wall_') then
				if U._PRD and editor.keyModifiers.ctrl and rayCast then
-- APPEND to SELECTION
					local desc = adesc[cedit.mesh]
							U.dump(cij, '?? wall_sel_append:'..rayCast.object:getID()..':'..rayCast.object.name..':'..tostring(desc.selection))
					if not desc.selection then desc.selection = {} end
					forBuilding(desc, function(w, ij)
						if not desc.selection[ij[1]] then
							desc.selection[ij[1]] = {}
						end
						desc.selection[ij[1]][ij[2]] = true
					end, cij)
					local ij = partOn(rayCast.object:getID())
					forBuilding(desc, function(w, ij)
						if not desc.selection[ij[1]] then
							desc.selection[ij[1]] = {}
						end
						desc.selection[ij[1]][ij[2]] = true
					end, ij)
						U.dump(desc.selection, '?? SELED:')
--[[
					if not desc.selection then
						desc.selection = {U.stamp(cij,true)}
					end
					local ij = U.stamp(partOn(rayCast.object:getID()),true)
					if #U.index(desc.selection, ij) == 0 then
						desc.selection[#desc.selection+1] = ij
					end
						U.dump(desc.selection, '?? for_sel:')
--                    scope = nil
]]
					markUp()
				else
					-- CYCLE SCOPE
					objLevel = (objLevel + 1)%3
					local toscope
					if objLevel == 0 then toscope = 'building' end
					if objLevel == 1 then toscope = 'floor' end
					if objLevel == 2 then toscope = 'wall' end
					scopeOn(toscope)
				end
			elseif string.find(nm, 'o_lid_') == 1 then
				if cij and adesc[cedit.mesh] then
					local floor = adesc[cedit.mesh].afloor[cij[1]]
					if floor and #floor.top.achild > 0 then
	--                    U.dump(floor.top.achild,'?? childs:'..#floor.top.achild..':'..cij[1])
						local scchild = floor.top.cchild
						lo('?? RH1:')
						local _,inrc = forRoofHit(adesc[cedit.mesh], cij[1])
						if inrc ~= nil then
							for i,c in pairs(floor.top.achild) do
--                                U.dump(c, '?? for_LIST2:'..i..':'..inrc..' scch:'..tostring(scchild)..':'..#U.index(c.list, inrc))
								if #U.index(c.list, inrc) > 0 then
	--                                lo('?? cc:'..floor.top.cchild)
									if scchild ~= nil then
										floor.top.cchild = nil
									else
										floor.top.cchild = i
									end
									break
								end
							end
						end
							lo('?? inrc:'..tostring(inrc)..':'..tostring(floor.top.cchild))
	--                        U.dump(floor.top.achild, '?? achild:')
						markUp()
					end
				else
					lo('!! NO_desc:'..tostring(cedit.mesh)..':'..tostring(cij)..':'..tableSize(adesc))
				end
			end
		else
			lo('?? mup_ol_reset:'..objLevel..':'..tostring(scope)..':'..tostring(indrag))
			inject.R.out.apick = nil
			inject.R.populate(nil, nil, nil, nil, true)
			inject.D.unselect()
			if scope == 'wall' then objLevel = 2
			elseif scope == 'floor' then objLevel = 1
			elseif scope == 'building' then objLevel = 0
			else
				objLevel = 0
				if not scope then scope = 'building' end
			end
			out.scope = scope
--            editor.clearObjectSelection()
		end
	end
	indrag = false
--!!    intopsplit = false
--        out.avedit = {vec3(0, 0, 6)}
--        U.dump(out.avedit, '<< mup:'..#out.avedit)
		U.dump(cij, '<< mup:'..tostring(cmesh))
	return true
end


--local htot = nil

-- Alt pressed
local function mpointAlt(rayCast)
--            _dbdrag = true
--        lo('?? mpoint:'..tostring(cedit.mesh)..':'..tostring(rayCast)..':'..tostring(_dbdrag))
		if _dbdrag then return end
--    out.wsplit = nil
	if cedit.mesh == nil then return end
--    local rayCast = cameraMouseRayCast(true)
	if rayCast == nil then return end
--??    if cedit.mesh == nil or indrag then return end
	local id = rayCast.object:getID()
	local nm = rayCast.object.name
	if nm == nil or nm == 'theTerrain' then return end
--            lo('?? mpoint2:'..tostring(cedit.mesh)..':'..nm..':'..tostring(intopchange))
	if string.find(nm, 'o_') == 1 then
		if smouse == nil then
			smouse = rayCast.pos
		end
		if _dbdrag then return end
--        if intopsplit then return end
--        indrag = true
--        _dbdrag = true
--        lo('?? point:'..tostring(indrag))
--        lo('?? mpoint:'..id..':'..#cij..':'..tostring(rayCast.pos)..':'..tostring(cedit.part))
		local bdesc = adesc[cedit.mesh]

		if U._PRD == 0 and string.find(nm, 'o_topplus_') == 1 then
-- SPLITTING LINE FOR ROOF BORDER
				_dbdrag = true
			local om = scenetree.findObject(nm)
				lo('??******* forMM:'..id..':'..nm..':'..tostring(om.obj)..':'..tostring(om.vdata)..':'..tostring(#aedit[id].mesh)) --..':'..tostring(om:getMesh())..':'..tostring(aedit[id]), true)
			local part,face = M.faceHit(aedit[id].mesh)
			if not part then return end
				lo('?? PF:'..part..':'..face)
			-- find horizontal direction along the face
			local mdata = aedit[id].mesh[part]
			local pos = mdata.verts[mdata.faces[face].v+1]
			local dir =
(mdata.verts[mdata.faces[face].v+1] - mdata.verts[mdata.faces[face+1].v+1]):cross(mdata.verts[mdata.faces[face+1].v+1] - mdata.verts[mdata.faces[face+2].v+1])
--(mdata.verts[mdata.faces[face].v+1] - mdata.verts[mdata.faces[face].v+2]):cross(mdata.verts[mdata.faces[face].v+2] - mdata.verts[mdata.faces[face].v+3])}
			dir = vec3(-dir.y, dir.x):normalized()
				lo('?? orig:'..tostring(pos)..':'..tostring(dir))
			M.dissect(pos, dir, mdata)
--[[
			local aface = {face}
			for i=1,#mdata.faces,3 do
				if i ~= face then
					local vn =
(mdata.verts[mdata.faces[i].v+1] - mdata.verts[mdata.faces[i+1].v+1]):cross(mdata.verts[mdata.faces[i+1].v+1] - mdata.verts[mdata.faces[i+2].v+1])
					local d = intersectsRay_Plane(orig.pos, orig.dir, mdata.verts[mdata.faces[i].v+1], vn)
					if math.abs(d)<small_dist then
						aface[#aface+1] = i
--                        lo('??+++++++ fit:'..i..':'..tostring(mdata.verts[mdata.faces[i].v+1])..':'..d)
					end
				end
			end
				U.dump(aface, '?? for_VERTS:'..#aedit[id].mesh[1].verts)
]]
		end
--        out.wsplit = nil
		if scope == 'top' then
			if intopchange or not cij then
				return
			end
--            indrag = true

			local floor = adesc[cedit.mesh].afloor[cij[1]]
			-- TODO: check if u not 0
			local u = (floor.base[2] - floor.base[1]):normalized()
			local v = u:cross(vec3(0,0,1)):normalized()
			local cmouse, dpos

			local vn = vec3(0,0,1)
			local ray = getCameraMouseRay()
			local d = intersectsRay_Plane(ray.pos, ray.dir, smouse, vn)
			cmouse = ray.pos + d*ray.dir
			if im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
--            if editor.keyModifiers.ctrl then
--                    lo('?? ptr:'..tostring(cmouse))
				if cedit.cval['AltZ'] == nil then
					cedit.cval['AltZ'] = {u, v}
				end
				if cedit.cval['AltZ'][3] ~= nil then return end
				out.wsplit = nil
				out.avedit = {
					cmouse + (u + v)*0.2,-- + vec3(0,0,floor.top.tip),
					cmouse + (u - v)*0.2,-- + vec3(0,0,floor.top.tip),
					cmouse + (-u - v)*0.2,-- + vec3(0,0,floor.top.tip),
					cmouse + (-u + v)*0.2,-- + vec3(0,0,floor.top.tip),
				}
			else
--                    lo('?? TSL:')
-- TOP SPLIT LINE RENDER
--                        if true then return end
				dpos = mouseDir(u, v)
--                dpos, cmouse = mouseDir(u, v)
				if cedit.cval['AltMove'] == nil then
					lo('??++++____+++ point_start:'..cij[1]..':'..tostring(dpos))
					cedit.cval['AltMove'] = {forHeight(adesc[cedit.mesh].afloor, cij[1])}
					return
				end
				cmouse.z = cedit.cval['AltMove'][1]+adesc[cedit.mesh].pos.z
--                    lo('?? TSL2:'..tostring(dpos)..':'..tostring(cmouse))
				if dpos ~= nil and dpos:length() > 0.001 then
--                        lo('?? for_DPOS:'..tostring(W.ui.dbg)..':'..tostring(dpos))
	--                                lo('?? dpos:'..tostring(u)..':'..tostring(dpos)..':'..tostring(cmouse))
					-- get intersections with boundaries
					aint = {}
					for j = 1,#floor.base do
						local v1,v2 = U.proj2D(floor.base[j] + adesc[cedit.mesh].pos), U.proj2D(floor.base[j % #floor.base + 1] + adesc[cedit.mesh].pos)
						local x,y = U.lineCross(v1, v2, cmouse, cmouse + dpos:cross(vec3(0,0,1)))
						if math.abs((v1 - vec3(x,y,0)):length() + (v2 - vec3(x,y,0)):length() - (v1-v2):length()) < 0.01 then
							aint[#aint + 1] = {vec3(x, y, cedit.cval['AltMove'][1]+adesc[cedit.mesh].pos.z+0.1), j}
						end
					end

					local function stick2base(base, lmouse)
						for i=1,#base do
							local dir = dpos:cross(vec3(0,0,1)):normalized()
							local d = U.toLine(base[i], {lmouse, lmouse+dir})
							if d < 0.2 then
								local v = base[i] - lmouse
								local ds = v - dir*dir:dot(v)
	--                                lo('?? HIT:'..i..':'..tostring(base[i])..':'..tostring(ds))
								lmouse = lmouse + ds
--                                return lmouse + ds
							end
						end
						return lmouse
					end

					out.wsplit = {}
					local base = floor.base
					local margin = forTop().margin
--                            U.dump(base, '?? for_floor:')
					if floor.top.cchild then
						base = floor.top.achild[floor.top.cchild].base
--                                lo('?? for_CHILD:'..#base)
					end
					local mbase = U.polyMargin(base, margin)
						mbase = base
--                        local pos = base2world(adec[cedit.mesh], {cij[1],1})
					-- near vertex sticking
					local pos = U.proj2D(base2world(adec[cedit.mesh], {cij[1],1}) - floor.base[1])
					local lmouse = U.proj2D(cmouse - pos)
					lmouse = stick2base(base, lmouse)
					if cij[1] < #adesc[cedit.mesh].afloor then
						lmouse = stick2base(adesc[cedit.mesh].afloor[cij[1]+1].base, lmouse)
					end

--[[
					if true then
						lmouse = stick2base(base, lmouse)
					else
						for i=1,#base do
							local dir = dpos:cross(vec3(0,0,1)):normalized()
							local d = U.toLine(mbase[i], {lmouse, lmouse+dir})
							if d < 0.2 then
								local v = mbase[i] - lmouse
								local ds = v - dir*dir:dot(v)
	--                                lo('?? HIT:'..i..':'..tostring(base[i])..':'..tostring(ds))
								lmouse = lmouse + ds
							end
						end
					end
]]
--                            lo('?? pre_hit:'..tostring(pos)..':'..tostring(cmouse))
					aint = U.polyCross(base, lmouse, dpos:cross(vec3(0,0,1)))
--                            U.dump(aint, '?? wspl: pos:'..tostring(pos)..' cm:'..tostring(cmouse)..':'..#out.wsplit)
					if #aint == 2 then
						for i=1,#aint do
							if not aint[i] then
								out.wsplit = {}
								break
							end
							out.wsplit[i] = aint[i][2] + pos + vec3(0,0,cedit.cval['AltMove'][1]+adesc[cedit.mesh].pos.z+0.01)
						end
--                        else
--                            out.wsplit = nil
					end

--                    if true or W.ui.dbg == nil then
--                        W.ui.dbg = false
--                    end


--[[
						if W.ui.dbg == nil then
--                            local pleft,pright = U.polyCross(floor.base, cmouse, dpos:cross(vec3(0,0,1)))
--                                lo('?? LR:'..tostring(pleft)..':'..tostring(pright))
							W.ui.dbg = false
						end
--                            U.dump(aint,'?? aint:'..#aint..':'..tostring(adesc[cedit.mesh].pos))
					if #aint == 2 then
						out.wsplit = {aint[1][1],aint[2][1]}
--                        floor.update = true
--                        houseUp(adesc[cedit.mesh], cedit.mesh)
--                        markUp()
					else
--                        lo('!! ERR_nosplit:'..#aint)
					end
]]
--                    if (cmouse - smouse):length() > 0 then
--                        lo('?? wsplit:'..(cmouse - smouse):length())
--                    end
	--                out.wsplit = {aint[1][1],aint[2][1]}
	--                    U.dump(aint, '?? point.aint:')
					-- rerender for mouse being properly detected
	--                    floor.top.update = true
				end
			end

--        if scope == 'top' and not intopsplit then
--            lo('?? cfloor:'..cij[1])
			smouse = cmouse
		elseif not out.acorner then -- and not editor.keyModifiers.ctrl then
--        elseif not out.invertedge then -- and not editor.keyModifiers.ctrl then
-- SPLIT LINE wall RENDER
--                lo('?? forff:'..tostring(id)..':'..rayCast.object.name)
--                lo('?? if_SL:')
			if not aedit[id] or not aedit[id].desc then
				lo('!! NO_desc:')
				return
			end
--                lo('?? if_SL2:'..tostring(indrag)..':'..tostring(intopchange))
			local otype,cij = name2type(rayCast.object.name),aedit[id].desc.ij
--                U.dump(ij, '?? for_desc:')
--                _dbdrag = true

			if intopchange or indrag or incorner then
--                    lo('?? IN_VE:', true)
				out.asplit = nil
				return
			end
			local desc = adesc[cedit.mesh]
--                lo('?? for_SPLIT_LINE:')

			local function line2rect(cp, p, u, v, dir)
--                local p = base2world(desc, ij)
				local un,vn = u:normalized(),v:normalized()
				local pu = (cp - p):dot(un)
				local pv = (cp - p):dot(vn)
				if dir == 1 then
					-- horizontal
					if 0 <= pv and pv <= v:length() then
						return {p + vn*pv, p + vn*pv + u}
					end
				else
					-- vertical
					if 0 <= pu and pu <= u:length() then
						return {p + un*pu, p + un*pu + v}
					end
				end
--                local vp = dir == 1 and u:normalized() or v:normalized()
			end

			if otype == 'wall' then
				local w = desc.afloor[cij[1]].awall[cij[2]]
				local cp = base2world(desc, cij) --w.pos
				local t = ray2rect(cp, w.u, w.v, getCameraMouseRay())
				local buf = forSide(cij, true)
				out.asplit = {}
				forBuilding(desc, function(w, ij)
					local dir = im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) and w.v or w.u
					dir = dir:normalized()
					local u = (t - cp):dot(dir)

					local p = base2world(desc, ij)
--                       local un = dir:normalized()
					local d = (cp - p):dot(dir)
					local line = line2rect(t, base2world(desc, ij), w.u, w.v,
						im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) and 1 or 2)
					if line then
						out.asplit[#out.asplit+1] = {ij = ij, u = d+u, line = line}
					end
				end, nil, buf)
--[[
				if im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
					if ({wall=1})[scope] then
						U.dump(buf,'?? split_HOR:'..cij[2]..':'..#buf)
						forBuilding(desc, function(w, ij)
						end)
					end
				else
--                    out.asplit = {}
				end
]]
--[[
						local p = base2world(desc, ij)
						local un = w.u:normalized()
						local d = (cp - p):dot(un)
						if (d + u) < 0 or (d + u) >= w.u:length() then return end
						p = p + (d + u)*un
						out.asplit[#out.asplit+1] = {ij = ij, u = d+u, line = {p, p + vec3(0,0,desc.afloor[ij[1] ].h)}}
]]
--                    U.dump(buf, '?? split_buf:')
--                    if d < 0 or (d + u) >= w.u:length() then return end
--                    local u,v = rayAlong(p, w.u, w.v, getCameraMouseRay())
--                    p = p + u*un
--                        U.dump(out.asplit, '?? for_split:')
			end
--                        U.dump(out.asplit, '??+++++++++++++++++++++ for_split:')
		end
	end
end


local function set(desc, key, val)
	desc[key] = val
	desc.update = true
end


local function mwheel(dir, rayCast)
			if _dbdrag then return end
--            _dbdrag = true
--            lo('?? mwheel:'..tostring(rayCast)..':'..tostring(cmesh))--..tostring(({scope = 1})['scope']))
	if rayCast == nil then return end
	local nm = rayCast.object.name
--[[
	if false and out.inconform and editor.keyModifiers.ctrl then
		mantle = mantle*(1 + 0.2*dir)
			lo('?? for_conform:'..mantle)
		conform(adesc[cedit.mesh])
		return
	end
]]
--            lo('>> mwheel:'..tostring(nm)..':'..scope)
	if cmesh ~= nil then
--            lo('?? re_SPEED:')
		out.apath = nil
		if editor.keyModifiers.ctrl then
			if dmesh[cmesh].apick then
					lo('?? for_MS:'..#dmesh[cmesh].apick)
					-- TODO: use ord
					local m = dmesh[cmesh].data[1]
				local center = vec3(0,0,0)
				for _,v in pairs(dmesh[cmesh].apick) do
					center = center + m.verts[v+1]
				end
				center = center/#dmesh[cmesh].apick

				local dirobj = center - core_camera.getPosition()
				local dr = dirobj*0.02
				for _,v in pairs(dmesh[cmesh].apick) do
					m.verts[v+1] = m.verts[v+1] + dir*dr
				end

				M.update(dmesh[cmesh])
--                dmesh[cmesh].obj:createMesh({})
--                dmesh[cmesh].obj:createMesh({amesh})
--                _dbdrag = true
			else
				local center,n = vec3(0,0,0),0
				local amesh = dmesh[cmesh].data
				for _,m in pairs(amesh) do
					for _,v in pairs(m.verts) do
						center = center + v
						n = n + 1
					end
				end
				center = center/n
				local dirobj = center - core_camera.getPosition()
				local dr = dirobj*0.02
				for i,m in pairs(amesh) do
					local m = amesh[i]
					for j,v in pairs(m.verts) do
						m.verts[j] = m.verts[j] + dir*dr
					end
				end
	--            dmesh[cmesh].data = amesh
	--                    lo('?? moved:'..cmesh..':'..n..':'..tostring(dirobj)..':'..dir..':'..tostring(amesh[1].verts[1]))
				M.update(dmesh[cmesh])
				U.boxMark(dmesh[cmesh].obj, out)
			end
--            out.apath = nil
		elseif ccenter ~= nil then
			local d = (ccenter - core_camera.getPosition()):length()
			local speed = 100*math.pow(1/(1+1/d), 2)
			core_camera.setSpeed(speed)
		end
		return
	end
	if editor.keyModifiers.alt or editor.keyModifiers.ctrl then
		indrag = true
	end
	------------------------
	-- ALT ---------------
	------------------------
	if U._PRD==0 and editor.keyModifiers.alt then
--            lo('?? resz:'..tostring(cedit.cval['AltZ'] ~= nil))
		if cedit.cval['AltZ'] ~= nil then
--                lo('?? AltZ:')
--            lo('?? weelresz:'..tostring(cedit.cval['AltZ'][3]))
--        if im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
			--            if editor.keyModifiers.shift then
							-- resize apeend mark
--                U.dump(cedit.cval, '?? mark_RESIZE:')
			local u,v = cedit.cval['AltZ'][1],cedit.cval['AltZ'][2]
			if cedit.cval['AltZ'][3] == nil then
				cedit.cval['AltZ'][3] = 1
			end
			cedit.cval['AltZ'][3] = cedit.cval['AltZ'][3] + 0.04 * dir
--                lo('?? to_resz:'..tostring(cedit.cval['AltZ'][3]))
			out.avedit = {
				smouse + (u + v) * cedit.cval['AltZ'][3] * 0.2,
				smouse + (u - v) * cedit.cval['AltZ'][3] * 0.2,
				smouse + (-u - v) * cedit.cval['AltZ'][3] * 0.2,
				smouse + (-u + v) * cedit.cval['AltZ'][3] * 0.2,
			}
			return true
		end
		if cedit.forest ~= nil then
-- WIN SPACING
--        if nm == 'theForest' then
--            local tp = dforest[]
				if dforest[cedit.forest] then lo('?? fof:'..tostring(dforest[cedit.forest].type)) end
			if dforest[cedit.forest] and dforest[cedit.forest].type == 'win' then
				local wdae
				if editor.keyModifiers.shift then
				else
					-- windows space
--                    lo('??_______________________ space:')
					forBuilding(adesc[cedit.mesh], function(w)
	--                    set(w, 'winspace', w.winspace + 0.02 * dir)
						w.winspace = w.winspace + 0.02 * dir
						wdae = w.win
					end)
				end
				--                    lo('?? fofor:'..tostring(cedit.forest))
				houseUp(adesc[cedit.mesh], cedit.mesh)
				markUp(wdae)
			end
		elseif scope == 'top' then
-- ROOF SLOPE
			intopchange = true
			out.wsplit = nil
			local floor = adesc[cedit.mesh].afloor[cij[1]]
			local shape = floor.top.cchild ~= nil and floor.top.achild[floor.top.cchild].shape or floor.top.shape
--                    if true then return end
--                   lo('?? Alt_Wheel:'..tostring(cij[1])..':'..tostring(shape))
			if ({pyramid=1,shed=1,gable=1})[shape] then
--                if ({pyramid=1,shed=1,gable=1})[md_roof] then
--            if md_roof == 'pyramid' or md_roof == 'shed' then
				if cedit.cval['AltWheel'] == nil then
					cedit.cval['AltWheel'] = floor.top.ang
				end

				local floor = adesc[cedit.mesh].afloor[cij[1]]
--                    lo('?? Alt_Wheel2:'..tostring(#floor.top.achild))
				if #floor.top.achild > 0 then
					if floor.top.cchild ~= nil then
						local child = floor.top.achild[floor.top.cchild]
						child.tip = child.tip + 0.02 * dir
					elseif floor.top.ang ~= nil then
--                        lo('??____________ for_WHOLE:'..tostring(floor.top.ang)..':'..floor.top.shape)
						floor.top.ang = floor.top.ang + 0.02 * dir
						roofSet(floor.top.shape)
						return true
					end
				else
					lo('?? for_tip:'..floor.top.tip)
					floor.top.tip = floor.top.tip + 0.02 * dir
					floor.update = true
				end
--                roofSet(floor.top.shape)
				houseUp(adesc[cedit.mesh], cedit.mesh)
				markUp()
			end
--        else
		elseif scope == 'wall' then
-- WALL WIDTH
			if inSide() then
				local floor = adesc[cedit.mesh].afloor[cij[1]]
--                        lo('?? inSIDE:'..cij[2])
				local u = (U.mod(cij[2]+1,floor.base) - floor.base[cij[2]]):normalized()
				floor.base[cij[2]] = floor.base[cij[2]] - u*0.02*dir
				floor.base[U.mod(cij[2]+1,#floor.base)] = floor.base[U.mod(cij[2]+1,#floor.base)] + u*0.02*dir
				baseOn(floor)
				houseUp(adesc[cedit.mesh], cedit.mesh, true)
				markUp()
			end
		elseif scope == 'floor' then
-- FLOOR HIGHT
			local floor = adesc[cedit.mesh].afloor[cij[1]]
			set(floor, 'h', floor.h + dir*0.02) -- 0.2 * dir)
			for _,w in pairs(floor.awall) do
				set(w, 'v', vec3(0,0,floor.h))
				w.v = vec3(0,0,floor.h)
			end
--            floor.h = floor.h + 1.02 * dir
--            floor.update = true
			uvOn()
--                local w = floor.awall[1]
--                U.dump(w.uv,'??++++++++ for_w:'..tostring(w.v))
			houseUp(adesc[cedit.mesh], cedit.mesh)
			markUp()
		elseif scope == 'building' then
--[[
			-- building stores
			local house = adesc[cedit.mesh]
			if house == nil then return end
			if dir > 0 then
				local floor = adesc[cedit.mesh].afloor[cij[1] ]
				local awall = forFloor(house.afloor[#house.afloor].base,
					floor.h, floor.awall[1], #house.afloor + 1)
				house.afloor[#house.afloor + 1] = {base = U.clone(floor.base), h = floor.h, awall = awall}
			end
			if dir < 0 and #house.afloor > 1 then
				local floor = house.afloor[#house.afloor]
				for _,w in pairs(floor.awall) do
					for i,key in pairs(w.df[w.win]) do
--                    for i,key in pairs(w.awin) do
						editor.removeForestItem(fdata, dforest[key].item)
						dforest[key] = nil
					end
				end
				house.afloor[#house.afloor] = nil
				cij[1] = #house.afloor
			end
			houseUp(adesc[cedit.mesh], cedit.mesh)
--            be:reloadCollision()
			markUp()
]]
		end
		return true
	---------------------------
	-- CONTROL -------------
	---------------------------
	elseif U._PRD==0 and editor.keyModifiers.ctrl and not U._MODE == 'ter' then
		local dirty = false
		out.wsplit = nil
		local desc = adesc[cedit.mesh]
		if not desc and not U._MODE == 'ter' then
			lo('!! ERR_NODESC:')
			return
		end
--            U.dump(desc.selection, '?? ctrl_WHEEL:')

		local function forWall(ij)
			local f = desc.afloor[ij[1]]
--                lo('>> forWall:'..tostring(f), true)
			local base = f.base
			if base ~= nil then
				local vn = (base[ij[2] % #base + 1] - base[ij[2]]):cross(vec3(0,0,1)):normalized()
				local dv = vn * 0.02 * dir
				base[ij[2]] = base[ij[2]] + dv
				base[ij[2] % #base + 1] = base[ij[2] % #base + 1] + dv
				--- update u's
				for j,w in pairs(f.awall) do
					w.u = base[j % #base + 1] - base[j]
				end
				--- this wall
				uvOn(ij)
				--- prev wall
				uvOn({ij[1], (ij[2]-2) % #f.base + 1})
				--- next wall
				uvOn({ij[1], ij[2] % #f.base + 1})
				--- update cover
				local mbase = f.base
				if ij[1] == #adesc[cedit.mesh].afloor then
					mbase = U.polyMargin(f.base, f.top.margin)
				end
				f.top.body = coverUp(mbase)
				baseOn(f, base)
			end
		end

		if desc.selection then
			for _,s in pairs(desc.selection) do
				forWall(U.split(s, '_'))
			end
			houseUp(adesc[cedit.mesh], cedit.mesh)
			markUp()
		elseif cedit.forest ~= nil then
-- FOREST SCALE
			if not dforest[cedit.forest] then return end
			local desc = adesc[dforest[cedit.forest].mesh]
				U.dump(desc.afloor[2].awall[1].df, '?? FOREST_scale:'..tostring(dforest[cedit.forest].type))
			if not cedit.cval['DragScale'] then cedit.cval['DragScale'] = {} end
			forBuilding(desc, function(w, ij)
				if not w.df or not w.win then return end
				if not w.df[w.win].scale then w.df[w.win].scale = 1 end
--                if not cedit.cval['DragScale'][U.stamp(ij)] then
--                    cedit.cval['DragScale'][U.stamp(ij)] = w.df[w.win].scale and w.df[w.win].scale or 1
--                    return
--                end
--                    lo('?? pre_scale:'..ij[1]..':'..ij[2]..':'..tostring(w.win), true)
				w.df[w.win].scale = w.df[w.win].scale*(1 + 0.02*dir)
--                    U.dump(desc.afloor[2].awall[1].df, '?? post_scale:'..w.df[w.win].scale)
			end)
--                U.dump(desc.afloor[2].awall[1].df, '?? scaled:')
			houseUp(desc, cedit.mesh, true)
		elseif ({wall=1,side=1})[scope] ~= nil then
-- EXTRUDE
			intopchange = true
			out.wsplit = nil

			if scope == 'side' then
				local asij = forSide()
--                        U.dump(asij, '?? asij:')
				for i,f in pairs(asij) do
					for _,j in pairs(f) do
--                            lo('?? for_wall:'..tostring(f)..':'..tostring(j))
						forWall({i,j})
					end
				end
--[[
						_dbdrag = true
				for i,floor in pairs(adesc[cedit.mesh].afloor) do
--                    forFloor(floor, i)
--                    forWall({i,cij[2]})
				end
]]
			else
				local floor = adesc[cedit.mesh].afloor[cij[1]]
				forWall(cij)
--                forFloor(floor)
			end
--            scenetree.findObject('edit'):deleteAllObjects()
			-- TODO: update roof
--                lo('?? to_EXT:'..tostring(adesc[cedit.mesh].afloor[cij[1]].top.shape), true)
			if adesc[cedit.mesh].afloor[cij[1]].top.shape == 'gable' then
				--?? just houseUp?
				roofSet('gable')
			else
			end
			houseUp(adesc[cedit.mesh], cedit.mesh, true)
			--                houseUp(adesc[cedit.mesh], cedit.mesh)
			markUp()

--[[
		elseif cedit.forest ~= nil then
--        if nm == 'theForest' then
--            local tp = dforest[]
--                lo('?? fof:'..tostring(dforest[cedit.forest].type))
			if dforest[cedit.forest].type == 'win' then
				local wdae
				forBuilding(adesc[cedit.mesh], function(w)
--                    set(w, 'winspace', w.winspace + 0.02 * dir)
					w.winleft = w.winleft + 0.02 * dir
					if w.winleft < 0 then w.winleft = 0.05 end
					wdae = w.win
				end)
--                    lo('?? fofor:'..tostring(cedit.forest))
				markUp(wdae)
			end
]]
		elseif scope == 'top' then
			if im.IsKeyPressed(im.GetKeyIndex(im.Key_Z)) then
--            if editor.keyModifiers.shift then
				-- resize apeend mark
--                U.dump(cedit.cval, '?? mark_RESIZE:')
			else
				local floor = adesc[cedit.mesh].afloor[cij[1]]
				local desc = floor.top
				local base
				if desc.cchild ~= nil then
						lo('?? for_child:')
					desc = desc.achild[desc.cchild]
					base = desc.base
				else
					base = floor.base
				end
				desc.margin = desc.margin + 0.02 * dir
				floor.update = true
					lo('?? dirty2:')
				dirty = true
			end
--                U.dump(aedit[cedit.part].desc, '?? for_margins:'..cedit.part)
--                _dbdrag = true
--            lo('?? marg:')
		elseif scope == 'floor' then
-- FLOOR STRETCH
			local desc = adesc[cedit.mesh]
			local mbase = U.polyMargin(desc.afloor[cij[1]].base, 0.02 * dir)
			baseOn(desc.afloor[cij[1]], mbase)
--            floorInOut(0.02 * dir)
--[[
local function floorInOut(val)
	local desc = adesc[cedit.mesh]
	local mbase = U.polyMargin(desc.afloor[cij[1] ].base, val)
	baseOn(desc.afloor[cij[1] ], mbase)
end

			local desc = adesc[cedit.mesh]
--                lo('?? SS floor:'..cij[1])
			local mbase = U.polyMargin(desc.afloor[cij[1] ].base, 0.02 * dir)
			baseOn(desc.afloor[cij[1] ], mbase)
]]
--            desc.afloor[cij[1]].base = mbase
--                lo('?? dirty3:')
			dirty = true
		end
		if dirty then
			if desc.afloor[cij[1]].top.shape == 'gable' then
				roofSet('gable')
			end
			houseUp(desc, cedit.mesh, true)
			markUp()
		end
		return true
	end
	return false
end

local function floor2level(f, ind)
	for _,w in pairs(f.awall) do
		w.ij[1] = ind
	end
	if f.top then
		f.top.ij[1] = ind
	end
	if f.achild then
--        for _,c in pairs(f.achild) do
--        end
	end
end


local function keyUD(dir)
	lo('?? W.keyUD:'..dir..':'..tostring(cedit.forest))
	if out.inconform and editor.keyModifiers.ctrl then
--            lo('?? for_conform:'..mantle)
		if cedit.cval['TerCtrlUD'] == nil then
			cedit.cval['TerCtrlUD'] = 0
		end
		cedit.cval['TerCtrlUD'] = cedit.cval['TerCtrlUD'] + 0.1*dir
		conform(adesc[cedit.mesh], cedit.cval['TerCtrlUD'])
		return
	end
	if editor.keyModifiers.alt then
		if scope == 'floor' and cij then
-- FLIP FLOORS
			local desc = adesc[cedit.mesh]
			if dir > 0 and cij[1] == #desc.afloor then return end
			if dir < 0 and cij[1] == 1 then return end
					lo('?? to_fflip:'..cij[1]..':'..tostring(cedit.mesh))
			local s_top
			local floor -- source floor
			if dir > 0 then
				if cij[1] + 1 == #desc.afloor then
					s_top = desc.afloor[#desc.afloor].top
				end
				floor = table.remove(desc.afloor, cij[1])
			elseif dir < 0 then
				if cij[1] == #desc.afloor then
					s_top = desc.afloor[#desc.afloor-1].top
				end
				floor = table.remove(desc.afloor, cij[1])
			end
			table.insert(desc.afloor, cij[1]+dir, floor)
			-- update levels
--            floor2level(floor, cij[1] + dir)
--            floor2level(desc.afloor[cij[1]], cij[1])
			for _,w in pairs(floor.awall) do
				w.ij[1] = w.ij[1]+dir
			end
			for _,w in pairs(desc.afloor[cij[1] ].awall) do
				w.ij[1] = w.ij[1]-dir
			end
			if floor.achild then
				for _,c in pairs(floor.achild) do
					c.floor = c.floor + dir
					c.afloor[#c.afloor].top.shape = 'flat'
					c.afloor[#c.afloor].top.margin = 0
				end
			end
			if desc.afloor[cij[1]].achild then
				for _,c in pairs(desc.afloor[cij[1]].achild) do
					c.floor = c.floor - dir
					c.afloor[#c.afloor].top.shape = 'flat'
					c.afloor[#c.afloor].top.margin = 0
				end
			end
			-- flip tops
					U.dump(s_top, '?? s_top:'..tostring(s_top))
			if floor.top and s_top then
				desc.afloor[cij[1]].top = floor.top
				floor.top = s_top
			end
--                    if true then return end
--[[
			for _,w in pairs(floor.awall) do
				w.ij[1] = w.ij[1]+dir
			end
			for _,w in pairs(desc.afloor[cij[1] ].awall) do
				w.ij[1] = w.ij[1]-dir
			end
			if s_top then
				desc.afloor[cij[1] ].top = floor.top
				floor.top = s_top
			end
]]
--                U.dump(desc.afloor[1].top, '?? top1:')
--                U.dump(desc.afloor[2].top, '?? top2:')
			houseUp(desc, cedit.mesh)
			cij[1] = cij[1] + dir
			markUp()
		end
	elseif U._PRD == 1 and editor.keyModifiers.shift then
-- SCALE MATERIAL
		if ({top=1, wall=1, side=1})[scope] ~= nil then
					lo('?? mat_scale:'..scope..':'..tostring(cij[2]), true)
			local wdesc = adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]]
			wdesc.uv[3] = wdesc.uv[3] - dir*0.05
			wdesc.uv[4] = wdesc.uv[4] - dir*0.05
			wdesc.update = true
			houseUp(adesc[cedit.mesh], cedit.mesh)
			markUp()
--            cedit.cval = {desc.uv[1], desc.uv[2], desc.uv[3], desc.uv[4]}
		end
	elseif editor.keyModifiers.ctrl then
			lo('?? for_CTRL:', true)
		local house = adesc[cedit.mesh]
		if house == nil then return end
		local floor
		if scope == 'floor' then
-- CLONE middle FLOOR
			floorAddMiddle()
--[[

]]
			if false then

				if house.selection then
						U.dump(house.selection, '??******* CLONE SELECTION:')
					local afloor = {}
					local ima = 0
					for i,r in pairs(house.selection) do
						if i > ima then ima = i end
						local f = deepcopy(house.afloor[i])
						afloor[#afloor+1] = f
						for j,w in pairs(f.awall) do
							for dae,_ in pairs(w.df) do
								w.df[dae] = {scale = w.df[dae].scale or 1}
							end
						end
						for dae,_ in pairs(f.top.df) do
							f.top.df[dae] = {scale = f.top.df[dae].scale or 1}
						end
					end
					floor = house.afloor[ima]
					if ima == #house.afloor then
						floor.top.shape = 'flat'
						floor.top.margin = 0
						afloor[#afloor].top.body = floor.top.body
						floor.top.body = coverUp(floor.base)
					end
	--                for k,f in pairs(afloor) do
					for k = #afloor,1,-1 do
						table.insert(house.afloor, ima + 1, afloor[k])
					end
				elseif dir > 0 then
						lo('??******************* FLOOR_clone:'..cij[1],true)
					local ito
					if dir > 0 then
						ito = cij[1]+1
					else
						ito = cij[1]
					end
					floor = adesc[cedit.mesh].afloor[cij[1]]
					local newfloor = deepcopy(floor)
		--            table.insert(house.afloor, ito, newfloor)
					for j,w in pairs(newfloor.awall) do
						for dae,_ in pairs(w.df) do
							w.df[dae] = {scale = w.df[dae].scale or 1}
						end
						if floor.awall[j].spany then
							if dir > 0 then
								floor.awall[j].spany = floor.awall[j].spany + 1
								w.spany = nil
							else
								w.spany = floor.awall[j].spany + 1
								floor.awall[j].spany = nil
							end
						end
						if floor.awall[j].skip then
								lo('?? for_skip:'..j..':'..cij[1])
							if cij[1] == #house.afloor or not house.afloor[cij[1]+1].awall[j].skip then
								if dir > 0 then
									newfloor.awall[j].skip = nil
								else
									floor.awall[j].skip = nil
								end
							end
						end
						w.pillar.yes = false
					end
					for dae,_ in pairs(newfloor.top.df) do
						newfloor.top.df[dae] = {scale = newfloor.top.df[dae].scale or 1}
					end
					table.insert(house.afloor, ito, newfloor)
					if dir > 0 then
						floor.top.shape = 'flat'
						floor.top.margin = 0
						house.afloor[ito].top.body = floor.top.body
						floor.top.body = coverUp(floor.base)
					end
					uvOn()
					cij[1] = ito
				end

			end

--[[
			if false then

				local awall = forFloor(floor.base,
				floor.h, floor.awall[1], cij[1] + 1)
				for j,w in pairs(awall) do
					w.mat = floor.awall[j].mat
					w.win = floor.awall[j].win
					w.winspace = floor.awall[j].winspace
					w.winbot = floor.awall[j].winbot
					w.winleft = floor.awall[j].winleft
					w.win = floor.awall[j].win
				end
				table.insert(house.afloor, ito, {base = U.clone(floor.base),
					h = floor.h, pos = floor.pos, awall = awall,
					top = {ij = {}, type = 'top', mat = house.afloor[#house.afloor].top.mat,
						margin = 0, tip = floor.top.tip,
						id = nil, body = nil}})
			-- TODO: update walls ij
			end
			for i = cij[1]+2,#house.afloor do
				local f = house.afloor[i]
				floor2level(f, i)
--                    for j,w in pairs(f.awall) do
--                        w.ij[1] = w.ij[1] + 1
--                    end
			end
]]
--                house.afloor[ito].top.body = floor.top.body
--                floor.top.body = coverUp(floor.base)
		elseif dir > 0 then
			if scope == 'building' or cij[1] == #adesc[cedit.mesh].afloor then
-- APPEND FLOOR to top
				-- clone top floor
				floorAdd()
--[[
				local desc = adesc[cedit.mesh]
				floor = desc.afloor[#adesc[cedit.mesh].afloor]
						lo('?? clone_top:') --..cij[1]..':'..floor.top.shape)
				local newfloor = fromFloor(desc, #desc.afloor)
				house.afloor[#house.afloor].top.shape = 'flat'
				house.afloor[#house.afloor].top.margin = 0
				house.afloor[#house.afloor + 1] = newfloor
				newfloor.ij = {#house.afloor}
				newfloor.top.ij = {#house.afloor}
				for j,w in pairs(newfloor.awall) do
					w.ij[1] = newfloor.ij[1]
				end

				local awall = forFloor(floor.base,
					floor.h, floor.awall[1], #house.afloor + 1)
				-- TODO: use floorClone
				house.afloor[#house.afloor + 1] = floorClone(adesc[cedit.mesh], #adesc[cedit.mesh].afloor, floor.base)
				uvOn()
				forestClean(floor.top)
				local newfloor = house.afloor[#house.afloor]
				newfloor.top = floor.top
				newfloor.top.ij = {#house.afloor, nil}
				floor.top = {ij = {#house.afloor - 1, nil}, type = 'top', shape = 'flat', margin = 0, mat = newfloor.top.mat}
--                floor.top.margin = 0
--                floor.top.shape = 'flat'
				house.afloor[#house.afloor].top.body = floor.top.body
				floor.top.body = coverUp(floor.base)
]]
				if not cij then cij = {} end
				cij[1] = #house.afloor
						lo('?? new_SHAPE:'..cij[1])--house.afloor[#house.afloor].top.shape)
			elseif scope == 'top' then
				floor = adesc[cedit.mesh].afloor[#adesc[cedit.mesh].afloor]
					lo('??____+++___ ifchild:'..tostring(floor.top.cchild)..':'..tostring(floor.top.shape))
				local base
				if floor.top.cchild == nil then
					base = floor.base
				else
					base = floor.top.achild[floor.top.cchild].base
				end
				house.afloor[#house.afloor + 1] =
					floorClone(adesc[cedit.mesh], #adesc[cedit.mesh].afloor, base)
				uvOn()
--                lo('?? ifchild:'..floor.top.cchild)
				local newfloor = house.afloor[#house.afloor]
				newfloor.top.body = coverUp(base)
				newfloor.top.tip = floor.top.tip
				floor.top.margin = 0
				floor.top.shape = 'flat'
					lo('?? RE_shape:'..tostring(newfloor.top.shape))
				cij[1] = cij[1] + 1
			end
--                        local w = house.afloor[#house.afloor-1].awall[1]
--                        lo('?? for_ij: id='..tostring(w.id)..':'..#house.afloor..':'..house.afloor[#house.afloor-1].awall[1].ij[1])
		end
		if dir < 0 and #house.afloor > 1 then
			if scope == 'building' then
-- REMOVE TOP FLOOR
				floor = house.afloor[#house.afloor]
				for _,w in pairs(floor.awall) do
					forestClean(w)
				end
				house.afloor[#house.afloor] = nil
				local floor = house.afloor[#house.afloor]
				local mbase = U.polyMargin(floor.base, floor.top.margin)
				floor.top.body = coverUp(mbase)
				cij[1] = #house.afloor
--            elseif scope == 'floor' then
--                floor = house.afloor[cij[1]]
			end
		end
		houseUp(nil, cedit.mesh)
--        houseUp(adesc[cedit.mesh], cedit.mesh)
		markUp()
	elseif cedit.forest then
-- CHANGE FOREST MESH
		local mtype = dforest[cedit.forest].type
--            lo('?? keyUD.mesh_change:'..tostring(cedit.forest)..':'..tostring(mtype)..':'..dir..':'..#daePath[mtype])
		out.curselect = (out.curselect or 0) - dir
--            lo('?? for_CS:'..tostring(out.curselect))
		if out.curselect < 0 then out.curselect = 0 end
		if meshApply(mtype, out.curselect)==false then
			out.curselect = out.curselect + dir
		end
	end
end


local function keyRL(dir)
--    lo('??----------- keyRL:'..dir..':'..tostring(out.inconform)..':'..mantle) --..tostring(dforest[cedit.forest]))
	if out.inconform and editor.keyModifiers.ctrl then
		mantle = mantle + 0.5*dir
		inmantle = true
--            lo('?? for_conform:'..mantle)
		conform(adesc[cedit.mesh])
		inmantle = false
		return
	end
--        lo('?? keyRL:'..tostring(editor.keyModifiers.ctrl)..':'..tostring(editor.keyModifiers.alt))
--        if editor.keyModifiers.ctrl then return end
--    else
	if editor.keyModifiers.alt then
		if scope == 'top' then
			-- FLIP TOP
			local floor = adesc[cedit.mesh].afloor[cij[1]]
			if floor.top.cchild then
				floor.top.achild[floor.top.cchild].istart = floor.top.achild[floor.top.cchild].istart + 1
			else
				floor.top.istart = floor.top.istart + dir
			end
					lo('?? roof_turn:'..floor.top.istart)
			floor.update = true
			houseUp(adesc[cedit.mesh], cedit.mesh)
		end
	elseif dforest[cedit.forest] ~= nil then
--        lo('?? fof:'..tostring(cedit.part)..':'..dforest[cedit.forest].type..':'..dforest[cedit.forest].mesh)
		if dforest[cedit.forest].type == 'door' then
			local desc = adesc[dforest[cedit.forest].mesh]
--                    U.dump(cij, '?? keyRL_cij:')
			local floor, iwall
			forBuilding(desc, function(w, ij)
--                    U.dump(ij, '?? keyRL_ij:')
				if w.doorind ~= nil then
					w.doorind = w.doorind + dir
					if w.doorind < 1 then w.doorind = 1 end
						lo('??___for_ind:'..ij[1]..':'..tostring(cedit.forest))
					if w.doorind > #w.df[w.win]+1 then
--                        lo('?? for_ind:'..ij[2]..':'..tostring(w.doorind)..':'.. tostring(#w.df[w.win]))
						w.doorind = #w.df[w.win] + 1
--[[
						w.doorind = nil
						floor = adesc[cedit.mesh].afloor[cij[1] ]
						iwall = (ij[2]) % #floor.base + 1
						floor.awall[iwall].df[w.door] = w.df[w.door]
]]
--                        lo('?? iwall:'..tostring(iwall))
					end
				end
			end)
--[[
			if iwall ~= nil then
				local wall = floor.awall[iwall]
				wall.doorind = 1
				wall.df[]
--                lo('?? door_flip:'..iwall..':'..tostring(cedit.forest)..':'..tostring(dforest[cedit.forest].item))
			end
]]
			houseUp(adesc[cedit.mesh], cedit.mesh, true)
					lo('?? dfor:'..tostring(cedit.forest)..':'..tostring(dforest[cedit.forest]))
			markUp(dforest[cedit.forest].item:getData():getShapeFile())

--            U.dump(desc, '?? wdesc:')
--            lo('?? desc:'..tostring(desc.doorind))
		elseif dforest[cedit.forest].type == 'win' then
--                lo('?? for_win_margin:')
			-- win margin
			local wdae
			forBuilding(adesc[cedit.mesh], function(w)
				local nwin = math.floor((w.u:length() - 2*w.winleft)/w.winspace)
				w.winleft = (w.u:length() - (nwin - dir)*w.winspace)/2
				if w.winleft < 0 then w.winleft = 0 end
				wdae = w.win
			end)
			houseUp(adesc[cedit.mesh], cedit.mesh, true)
			markUp(wdae)
		end
	end
end


local function onKey(key)
		lo('>> onKey:'..key)
--    if tostring(key) == '525' then
	if key == im.GetKeyIndex(im.Key_Escape) then
		if cmesh ~= nil then
				lo('?? mesh_ESC:'..#dmesh[cmesh].buf..':'..#dmesh[cmesh].sel)
			if #dmesh[cmesh].buf > 0 then
				-- unselect purple
				M.move(nil, dmesh[cmesh].buf, dmesh[cmesh].data, 'NONE')
				M.move(nil, dmesh[cmesh].trans, dmesh[cmesh].data, 'NONE')
				M.update(dmesh[cmesh])
				out.avedit = {}
				out.apath = nil
				out.frame = nil
			elseif #dmesh[cmesh].sel > 0 then
				-- unselect
					lo('?? unsel:'..#dmesh[cmesh].sel..':'..#dmesh[cmesh].data)
--                for _,m in pairs(dmesh[cmesh].sel) do
--                end
				M.move(nil, dmesh[cmesh].sel, dmesh[cmesh].data, 'NONE')
				M.update(dmesh[cmesh])
				out.avedit = {}
				out.apath = nil
				out.frame = nil
	--            cedit.cval['Drag_Z'] = nil
			end
		end
	elseif key == im.GetKeyIndex(im.Key_Space) then
--    if key == im.Key_Space then
		if cmesh == nil then return end
		local amesh = M.clone(dmesh[cmesh])
				lo('?? to_clone:'..#amesh[1].verts..':'..#amesh[1].faces)
		local id, om = meshUp(amesh, 'test', groupEdit)
		dmesh[id] = {obj = om, data = amesh, sel = {}, buf = {}, trans = {}}
		out.avedit = {}
		out.apat = nil
		out.frame = nil
		M.save(amesh, 'save_'..id)
	elseif key == im.GetKeyIndex(im.Key_Backspace) and cmesh ~= nil then
			lo('?? onBack:'..#dmesh[cmesh].sel)
		if cmesh == nil then return end
		if #dmesh[cmesh].sel == 0 then
			-- remove mesh
			dmesh[cmesh].obj:delete()
			dmesh[cmesh] = nil
			out.avedit = {}
		else
			-- remove selection
			M.move(nil, dmesh[cmesh].trans, dmesh[cmesh].data, 'NONE')
			dmesh[cmesh].sel = {}
			dmesh[cmesh].buf = {}
			M.update(dmesh[cmesh])
			out.avedit = {}
			out.apath = nil
		end
	elseif key == im.GetKeyIndex(im.Key_Enter) then
		lo('??+++++++++++++++++++++++ for_ENT:')
--            jsonEncode()
		out.inaxis = nil
		if out.inconform then
			lo('?? in_CONF:')
			out.avedit = {}
			out.apick = {}
			out.apoint = {}

			shmap = {}
			out.inconform = false
		elseif cmesh ~= nil then
				lo('?? to_persist: buf:'..#dmesh[cmesh].buf..' trans:'..#dmesh[cmesh].trans)
			if editor.keyModifiers.shift then
				-- append transparent
				M.move(nil, dmesh[cmesh].trans, dmesh[cmesh].sel, 'NONE')
				M.move(nil, dmesh[cmesh].buf, dmesh[cmesh].sel, 'NONE')
				M.update(dmesh[cmesh])
				out.avedit = {}
				out.frame = nil
				out.apath = nil
			elseif #dmesh[cmesh].buf == 0 then
				-- sel->data, deselect
	--                    lo('?? deselect:'..#dmesh[cmesh].data)
				M.move(nil, dmesh[cmesh].sel, dmesh[cmesh].data)
	--                    lo('?? mmoved:'..#dmesh[cmesh].data)
				M.update(dmesh[cmesh])
				out.avedit = {}
				out.frame = nil
				out.apath = nil
			else
	--                    lo('?? buf2sel:'..#dmesh[cmesh].data)
	--            out.avedit = {}
				-- buf->sel
				M.move(nil, dmesh[cmesh].buf, dmesh[cmesh].sel, 's_yellow')
				-- trans->unselect
				M.move(nil, dmesh[cmesh].trans, dmesh[cmesh].data, 'NONE')
	--                    lo('?? buf2sel2:'..#dmesh[cmesh].data)
				local amesh = {} --dmesh[cmesh].data
				for _,m in pairs(dmesh[cmesh].data) do
					amesh[#amesh + 1] = m
				end
				for _,m in pairs(dmesh[cmesh].sel) do
					amesh[#amesh + 1] = m
				end
				dmesh[cmesh].obj:createMesh({})
				dmesh[cmesh].obj:createMesh({amesh})
						lo('?? seled:'..tostring(#dmesh[cmesh].data)..':'..tostring(#dmesh[cmesh].sel)..':'..tostring(#dmesh[cmesh].buf))
				M.mark(dmesh[cmesh].sel, out)
				out.frame = nil
			end
		end
--        cedit.cval['Drag_Z'] = nil
	end
end


local function keyAlt(yes)
--        lo('?? keyAlt:'..tostring(yes))
	intopchange = false
	out.wsplit = nil
	out.asplit = nil
	if not yes then
		cedit.cval['AltMove'] = nil
	end
	if U._PRD == 0 then
		core_camera.setSpeed(camspeed)
	end
	markUp()
end


local function keyShift(yes)
	if not yes then
--			lo('?? keyShift:'..tostring(yes))
		intopchange = false
		cedit.cval['AltZ'] = nil
		cedit.cval = {}
		indrag = false
		if U._PRD == 0 then
			core_camera.setSpeed(camspeed)
		end
		if not Ter or not Ter.out.aregion then
			markUp()
		end
--        out.wsplit = nil
	end
--      U.dump(cij,'<< keyShift:')
end


local function goAround(dir)
	if cedit.mesh == nil then return end
	--            indrag = true

--    if ccenter == nil then return end
	if not cedit.cval['CamRot'] then
--                cedit.cval = {im.GetMousePos().x, core_camera.getPosition()}
		local desc = adesc[cedit.mesh]
		local base
		if ({floor=1,top=1,wall=1})[scope] then
			base = desc.afloor[cij[1]].base
		elseif scope == 'building' then
			base = desc.afloor[1].base
		end
		local center -- = U.polyCenter(base) --+ cedit.cval['DragRot'].base
		if cij then
			center = base2world(desc, {cij[1], 1}, center)
			--                out.apoint = {center + vec3(0,0,6)}
					U.dump(base, '?? to_AROUND:'..scope..':'..cij[1]..':'..tostring(center))
			cedit.cval['CamRot'] = {
				camdir = core_camera.getForward(),
				center = center,
				dist = (center - core_camera.getPosition()):length(),
				a = 0}
		else
			lo('!! ERR_goAround_NOCIJ:')
		end
		return
	end
	local a = cedit.cval['CamRot'].a + 0.002
	if a > 1 then a = 1 end
	local center = cedit.cval['CamRot'].center
--                a = 0
	local dirpre = a*(center - core_camera.getPosition()):normalized() + (1-a)*cedit.cval['CamRot'].camdir
	local dirnew = U.vturn(dirpre, -0.00*dir):normalized()
--            lo('?? in_AROUND:'..tostring(dirpre)..'>'..tostring(dirnew)..':'..a)

--                dirnew = core_camera.getPosition() - ccenter
	-- turn camera in direction of dirnew
	core_camera.setRotation(0, quatFromDir(dirnew, vec3(0,0,1)))

	local vp = -dir*(core_camera.getPosition() - center):cross(vec3(0,0,1)):normalized()
	local posnew = center +
(core_camera.getPosition() + vp*0.01*cedit.cval['CamRot'].dist - center):normalized()*cedit.cval['CamRot'].dist
	core_camera.setPosition(0, posnew) -- + dirnew*cedit.cval['CamRot'].dist)

	cedit.cval['CamRot'].camdir = dirnew
	cedit.cval['CamRot'].a = a


--[[
	local pos = core_camera.getPosition()
	local dir = ccenter - pos
	local newdir = U.vturn(dir, 0.02)
	local newpos = ccenter - newdir
--    lo('>> goAround:'..tostring(cedit.mesh)..':'..tostring(pos)..' cent:'..tostring(ccenter)..' dir:'..tostring(dir)..' nd:'..tostring(newdir))

	core_camera.setPosition(0, newpos)
	core_camera.setRotation(0, quatFromDir(newdir, vec3(0,0,1)))
]]

--    local obj = scenetree.findObjectById(cedit.mesh)
--    lo('?? goAround:'..tostring(obj:getPosition()))
--    U.dump(adesc[cedit.mesh], '?? desc:')

--[[
	if false then
		local direction = vec3(core_camera.getForward())
		local startPoint = vec3(core_camera.getPosition())

		lo('?? RA:'..tostring(direction)..':'..tostring(startPoint))
	--    local newPos = U.vturn(U.proj2D(startPoint - direction), 0.05) + vec3(0, 0, startPoint.z)

	--    core_camera.setRotation(0, vec3(0,0,math.pi/4))
	--    core_camera.setRotation(0, quatFromDir(vec3(0, -10, -1),vec3(0,0,1)))
		core_camera.setRotation(0, quatFromDir(vec3(0, -10, 4), vec3(0,0,1)))
	end
]]
end


local function uiUpdate()
	if W.ui.dbg then
		lo('?? to_DBG:'..tostring(cij)..':'..tostring(cedit.mesh), true)
--        W.ui.dbg = false
	end
	if cedit.mesh and cij then
		local desc = adesc[cedit.mesh]
		if not desc then
				lo('!! ERR_uiUpdate_NODESC:'..tostring(cedit.mesh))
			cedit.mesh = nil
			return
		end
		W.ui.xpos = desc.pos.x
		W.ui.ypos = desc.pos.y
		W.ui.zpos = desc.pos.z
--        W.ui.ang_building = 2 -- desc.pos.y
		W.ui.n_floors = #desc.afloor
		if not cij[1] or not desc.afloor[cij[1]] then return end
		W.ui.height_floor = desc.afloor[cij[1]].h
		-- W.ui.ang_floor = 0

		local floor = (cij and cij[1]) and desc.afloor[cij[1]] or nil
		local wall = (cij and cij[1] and cij[2]) and desc.afloor[cij[1]].awall[cij[2]] or nil
		W.ui.wall_spanx = wall and wall.spanx or 1
		W.ui.wall_spany = wall and wall.spany or 1

		W.ui.fringe_height = (wall and wall.fringe) and wall.fringe.h or 0
		W.ui.fringe_inout = (wall and wall.fringe) and wall.inout or 0
		local top = forTop()
		if top then
			W.ui.top_thick = top.fat
		end
--        W.ui.fringe_margin = (floor and floor.fringe) and floor.fringe.inout or 0

		if cij[2] then
      if cij[2] <= #desc.afloor[cij[1]].awall then
        local wall = desc.afloor[cij[1]].awall[cij[2]]
        W.ui.win_bottom = wall.winbot
        W.ui.win_left = wall.winleft
        W.ui.win_space = wall.winspace
        if wall.win and wall.df[wall.win] then
          W.ui.win_scale = wall.df[wall.win].scale and wall.df[wall.win].scale or 1
        end
        W.ui.door_ind = wall.doorind and wall.doorind or 0
        W.ui.door_bot = wall.doorbot or 0
        W.ui.pilaster_ind0 = (wall.pilaster and wall.pilaster.aind and wall.pilaster.aind[1]) or 0
        W.ui.pilaster_ind1 = (wall.pilaster and wall.pilaster.aind and wall.pilaster.aind[2] and wall.pilaster.aind[2]-wall.pilaster.aind[1]) or 0
        W.ui.pilaster_ind2 = (wall.pilaster and wall.pilaster.aind and wall.pilaster.aind[3] and wall.pilaster.aind[3]-wall.pilaster.aind[2]) or 0
      else
        cij[2] = 1
      end
--            if wall.pilaster and wall.pilaster.aind[3] then
--                lo('?? for_PI:'..tostring(W.ui.pilaster_ind2))
--            end
			if W.ui.dbg then
				lo('?? PPPL:'..#wall.pilaster.aind..':'..tostring(cij[2])) --..':'..tostring(W.ui.pilaster_ind0)..':'..tostring(W.ui.pilaster_ind1)..':'..tostring(W.ui.pilaster_ind2), true)
				W.ui.dbg = false
			end
		end
		W.ui.pillar_inout = floor.pillarinout
		W.ui.pillar_space = floor.pillarspace
		if scope == 'wall' and wall and wall.pillar and wall.pillar.inout then
			W.ui.pillar_inout = wall.pillar.inout
			W.ui.pillar_space = wall.pillar.space
		end

		W.ui.top_margin = desc.afloor[cij[1]].top.margin
		W.ui.top_tip = desc.afloor[cij[1]].top.shape == 'flat' and 0 or desc.afloor[cij[1]].top.tip
		if desc.afloor[cij[1]].top.cchild then
			if not desc.afloor[cij[1]].top.achild[desc.afloor[cij[1]].top.cchild] then
				lo('!! ERR_NO_CHILD: cij1:'..tostring(cij[1])..':'..#desc.afloor[cij[1]].top.achild)
				desc.afloor[cij[1]].top.cchild = nil
			else
				W.ui.top_tip = desc.afloor[cij[1]].top.achild[desc.afloor[cij[1]].top.cchild].tip
			end
		end
	end
end


W.floorDel = function(desc, ind)
	if not ind then ind = cij[1] end
	-- cleanup
	local floor = desc.afloor[ind]
	for j,w in pairs(floor.awall) do
		forestClean(w)
		scenetree.findObjectById(w.id):delete()
		aedit[w.id] = nil
	end
	forestClean(floor.top)
  if floor.top.id then
    local otop = scenetree.findObjectById(floor.top.id)
    if otop then
      otop:delete()
    end
    aedit[floor.top.id] = nil
  elseif floor.top.achild then
    for i,c in pairs(floor.top.achild) do
      local cobj = scenetree.findObjectById(c.id)
      if cobj then
        cobj:delete()
        aedit[c.id] = nil
      end
    end
  end
--  floor.top.achild = nil
	table.remove(desc.afloor, ind)
	houseUp(nil, desc.id)
end


W.voice2build = function(cmd)
--    if true then return end
	local atoken = U.split(cmd, ' ', true)
	local val
		lo('?? voice2build:'..tostring(cmd)..':'..#atoken..':'..#adesc)
		lo('DO:'..tostring(cmd))
	if string.find(cmd, 'clear') then
		clear()
	end
	if string.find(cmd, 'scope') then
		for _,t in pairs({'building','floor','wall','top'}) do
			if string.find(cmd, t) then
				scopeOn(t)
			end
		end
		return
	end
	if scope == 'top' then
		for _,t in pairs({'gable','shed','pyramid','flat'}) do
			if string.find(cmd,t) then
				roofSet(t)
			end
		end
	end
	if string.find(cmd, 'building') then
		local desc = buildingGen(vec3(math.random(10),math.random(10)))
			lo('?? created:'..#adesc..':'..tostring(cedit.mesh)..':'..tostring(desc.id)) --..':'..adesc[1].id)
		cedit.mesh = desc.id
		cij = {1,1}
		scope = 'building'
		houseUp(nil, cedit.mesh)
		markUp()
--        cedit.mesh = adesc[1].id
	end
	if string.find(cmd, 'rotate') then
		local ind = U.index(atoken, 'degrees')[1]
		if ind then
			val = atoken[ind-1]
				lo('?? degree:'..tostring(val))
			W.onVal('building_ang', tonumber(val))
		end
	end
	if string.find(cmd, 'floor') then
		if string.find(cmd, 'add') then
			floorAdd()
			houseUp(nil, cedit.mesh)
		else
			local ind = U.index(atoken, 'select')[1] or U.index(atoken, 'choose')[1]
			if ind then
				val = atoken[ind+1]
					lo('?? floor_num:'..tostring(val)) --..':'..tostring(tonumber(val)))
				if val == 'first' then
					cij[1] = 1
				end
				if val == 'second' then
					cij[1] = 2
				end
				if val == 'third' then
					cij[1] = 3
				end
				scopeOn('floor')
				markUp()
			end
		end
	end
	if string.find(cmd, 'height') then
		local ind = U.index(atoken, 'meter')[1] or U.index(atoken, 'meters')[1] or U.index(atoken, 'm')[1]
			lo('for_height1:'..tostring(ind))
		if ind then
			val = tonumber(atoken[ind-1])
			lo('?? for_height2:'..tostring(val))
			if scope == 'floor' then
				W.onVal('height_floor', val)
			end
		end
	end
end


local function onUpdate()
		if _dbdrag then return end

	if U._PRD == 0 and out.R then
		out.R.onUpdate()
	end
	if U._PRD == 0 then
		N.onUpdate()
	end
	if U._MODE == 'conf' then
--	if U._MODE == 'conf' and not scope then
		if Ter and not out.interr then
			out.interr = Ter.onUpdate()
			if out.interr then
				return
			end
		end
		if editor.isWindowVisible('LAT') then
			local inroad = D.onUpdate()
			if inroad then
				lo('?? from_D.upd:'..tostring(inroad))
				out.inroad = inroad
				return
			end
			if D.out.jdesc then
				W.ui.exit_r = D.out.jdesc.r
			end
		end
	elseif U._MODE == 'ter' and Ter and not out.interr then
		out.interr = Ter.onUpdate()
		if out.interr then return end
	end
	uiUpdate()

	if im.IsMouseReleased(0) then
--			lo('?? if_interr:'..tostring(out.interr))
		if out.interr then
			out.interr = nil
		else
			if mup(cameraMouseRayCast(true), {D = D}) then
				return
			end
		end
	end

	local w = im.GetIO().MouseWheel
	if w ~= 0 then
--			lo('?? W.wheel:'..tostring(out.interr))
		if out.interr then
			out.interr = nil
		else
			if mwheel(w, cameraMouseRayCast(true)) then return end
		end
	end

	if W.ui.dbg then
		lo('??++++++++++++++++++++++++++++++++++++++++ to_UPD:'..cij[1]..':'..cij[2])
		W.ui.dbg = false
	end

	local ctime = os.clock()
	if U._PRD == 0 and out.ccommand and ctime and out.ctime and (ctime - out.ctime) > 2 then
		local setfile = '/tmp/bat/set.txt'
		local file = io.open(setfile)
		if file then
			io.close(file)
			local conf = jsonReadFile(setfile)
				U.dump(conf,'?? check:'..tostring(setfile))
			FS:removeFile(setfile)
			local dset = {o=0,y=0,g=0,b=0}
			for _,list in pairs(conf.set) do
				for j,p in pairs(list) do
					if p == 'a' then
						dset['o'] = dset['o'] + 1
					elseif p == 'b' then
						dset['y'] = dset['y'] + 1
					elseif p == 'c' then
						dset['g'] = dset['g'] + 1
					elseif p == 'd' then
						dset['b'] = dset['b'] + 1
					end
				end
			end
				U.dump(dset, '?? for_DSET:')
			out.R.clear(groupEdit)
			out.R.sceneUp(nil, dset, groupEdit)
			out.R.up()
		--      out.R.move(vec3(0,0.2,3.5))
		--      io.flush(setfile)
		end
		if false then
			local voicefile = '/tmp/bat/in.txt'
			local file = io.open(voicefile)
			local command = file:read('*all')
			io.close(file)
		--                lo('?? for_COMMAND:'..tostring(#command)..':'..tostring(#out.ccommand))
			if command ~= out.ccommand and #command > 2 then
				lo('?? for_COMMAND2:'..tostring(#command)..':'..tostring(#out.ccommand))
		--        if not out.ccommand or command ~= out.ccommand then
		--            lo('?? COMMAND:'..command)
				if out.R then
					out.R.voice2job(command)
					local outputFile = io.open(voicefile, "w")
					if outputFile then
						outputFile:write('')
						outputFile:close()
						out.ccommand = ''
					end
				end
		--            W.voice2build(command)
		--        out.ccommand = command
			end
		end
		out.ctime = ctime
--            lo('?? test:'..command..':'..tostring(os.clock()))
	end
--        local t = os.execute("echo 'test'")

	out.asplit = nil
	if not indrag then
--        out.acorner = nil
		out.invertedge = nil
	end
--    incorner = false
--------------------
-- KB events
--------------------
	if editor.keyModifiers.ctrl and im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
--            lo('?? to_restoere:'..#asave..':'..tostring(cedit.mesh)..':'..tostring(inrollback))
		if not inrollback then
--            inrollback = true
			if U._MODE == 'conf' then
				D.undo()
			else
					lo('?? to_restore:'..#asave, true) --..':'..tostring(cedit.mesh)..':'..tostring(inrollback))
				undo()
			end
--                lo('?? restored:'..#asave..':'..tostring(inrollback))
--[[
			if #asave > 1 then
				table.remove(asave, 1)
				inrollback = true
				fromJSON()
				markUp()
					lo('?? restored:'..tostring(cedit.mesh))
			end
]]
			return
		end
	end
	if im.IsKeyReleased(im.GetKeyIndex(im.Key_Z)) then
--        lo('?? Z_up:')
		inrollback = false
	end
	if im.IsKeyReleased(im.GetKeyIndex(im.Key_ModCtrl)) then
--            lo('??^^^^^^^^^^^^^^ rb_BACK:',true)
		inrollback = false
	end
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_Enter)) then
			lo('?? to_SAVE:')
		local desc = adesc[cedit.mesh]
		if desc and desc.prn then desc = adesc[desc.prn] end
--        lo('?? for_save:')
--            for j,f in pairs(desc.afloor) do
--                lo('?? for_child:'..j..':'..tostring(f.achild))
--            end
		toJSON(desc)
--[[
		table.insert(asave, 1, {math.floor(socket.gettime()), deepcopy(desc)})
--        asave[#asave+1] = math.floor(socket.gettime())
			lo('?? for_JSON:'..#asave..':'..fsave) --..tostring(adesc[cedit.mesh]))
		jsonWriteFile(fsave..'_'..asave[1][1]..'.json', desc)
		csave = 1
]]
	end
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_Delete)) then
		if out.inhole then
--                lo('?? HOLE:'..tostring(#out.inhole.achild))
			if out.inhole.achild then
--                U.dump(out.inhole.achild[#out.inhole.achild], '?? HOLE:')
				local cw = out.inhole
				cw.achild[#cw.achild].yes = true
				houseUp(adesc[cedit.mesh], cedit.mesh, true)
			end
--                lo('?? HOLE2:'..tostring(#out.inhole.achild))
--            out.inhole = nil
			return
		end
			lo('??----------------- to_del:'..tostring(cedit.mesh)..':'..tostring(out.inhole), true)
--            if true then return end
		local desc = adesc[cedit.mesh]
		if not desc then return end
		if scope == 'top' then
			local floor = desc.afloor[cij[1]]
			if floor.top.cchild then
--                    U.dump(floor.top.achild, '?? achild:')
				floor.top.cchild = nil
				for i,c in pairs(floor.top.achild) do
					if c.id then
						scenetree.findObject(c.id):delete()
					end
				end
				floor.top.achild = {}
			end
				U.dump(floor.awplus, '?? fAW:')
			roofSet('flat')
		elseif scope == 'building' then
-- DELETE BUILDING
			if desc.prn then
					lo('?? for_child:'..desc.floor) --#desc.prn.afloor[desc.floor].achild)
				for k,c in pairs(adesc[desc.prn].afloor[desc.floor-1].achild) do
--                    lo('?? for_child:'..k..':'..tostring(c.id))
					if c.id == cedit.mesh then
						table.remove(adesc[desc.prn].afloor[desc.floor-1].achild, k)
						break
					end
				end
			end
			for i,f in pairs(desc.afloor) do
				for j,w in pairs(f.awall) do
					forestClean(w)
				end
				forestClean(f.top)
			end
				lo('?? DEL_building:'..cedit.mesh..':'..tostring(desc.id))
			scenetree.findObject('edit'):deleteAllObjects()
			scenetree.findObjectById(cedit.mesh):delete()
			adesc[cedit.mesh] = nil
			cedit.mesh = nil
			out.avedit = {}
--            out.fyell = nil
			out.dyell = nil
			out.fmtop = nil
		elseif scope == 'floor' then
				lo('?? del_FLOOR:'..cij[1])
-- DELETE FLOOR
			W.floorDel(desc)
      if cij[1] == #desc.afloor+1 then
        cij[1] = cij[1]-1
      end
--[[
			-- cleanup
			local floor = desc.afloor[cij[1] ]
			for j,w in pairs(floor.awall) do
				forestClean(w)
				scenetree.findObjectById(w.id):delete()
				aedit[w.id] = nil
			end
			forestClean(floor.top)
			scenetree.findObjectById(floor.top.id):delete()
			aedit[floor.top.id] = nil
			table.remove(desc.afloor, cij[1])
			houseUp(nil, desc.id)
]]
--            out.fyell = nil
			out.dyell = nil
			out.fmtop = nil
		elseif scope == 'wall' then -- and false then
-- DELETE SELECTION
			local buf = {}
			if desc.selection and tableSize(desc.selection) > 0 then
				buf = desc.selection
			else
				buf[cij[1]] = {cij[2]}
			end
			wallCollapse(adesc[cedit.mesh], buf, true)
			markUp()
		end
    toJSON(desc)
	end


-----------------------------
-- MARKING
-----------------------------
	if out.fwhite then
--            lo('?? for_fwhite:'..#out.fwhite)
		for _,v in pairs(out.fwhite) do
			debugDrawer:drawLine(v[1], v[2], ColorF(1,1,1,1), 4)
		end
	end
	-- top marking
	if out.fmtop then
		for _,pth in pairs(out.fmtop) do
			for i=1,#pth-1 do
        debugDrawer:drawLineInstance(pth[i], pth[i+1], 4, ColorF(1,1,0.5,0.1))
--				debugDrawer:drawLine(pth[i], pth[i+1], ColorF(1,1,0,10))
--				debugDrawer:drawLine(pth[i], pth[i+1], ColorF(1,0,1,10))
			end
		end
	end
--[[
	if false and out.dyell then
		for i,row in pairs(out.dyell) do
			for j,pth in pairs(row) do
				local incorner

				for i=1,#pth-1 do
					local clr = ColorF(1,1,0.5,0.1)
					local w = 4
					if incorner then
						clr = ColorF(1,1,1,0.1)
						w = 2
					end
--                    if out.acorner and out.acorner[i][j] then
--                        debugDrawer:drawLineInstance(pth[i], pth[i+1], 4, ColorF(0.5,1,0.5,0.1))
--                    end
					debugDrawer:drawLineInstance(pth[i], pth[i+1], w, clr)
	--                debugDrawer:drawLine(pth[i], pth[i+1], ColorF(1,1,0.5,0.5), 6)
				end
			end
		end
	end
]]
--[[
				if out.acorner then
--                        lo('?? ACORN0:')
						U.dump(out.acorner, '?? acorn:')
					for i,c in pairs(out.acorner) do
						if c.ij[1] == i and c.ij[2] == j then
							incorner = true
						end
					end
				end
]]


	if out.fyell then
--            lo('?? for_fwhite:'..#out.fwhite)
		for _,pth in pairs(out.fyell) do
--            Render.path(pth, color(255,255,150,155), 4)

--            Render.path(pth, color(255,0,0,255), 2)
			for i=1,#pth-1 do
				debugDrawer:drawLineInstance(pth[i], pth[i+1], 4, ColorF(1,1,0.5,0.1))
--                debugDrawer:drawLine(pth[i], pth[i+1], ColorF(1,1,0.5,0.5), 6)
			end
--            debugDrawer:drawLine(v[1], v[2], ColorF(1,0,0,1), 6)
		end
	end
	if out.ahole and #out.ahole > 1 then
		for i=2,#out.ahole do
--        for _,v in pairs(out.ahole) do
			debugDrawer:drawLine(out.ahole[i-1], out.ahole[i], ColorF(1,1,0,1), 4)
		end
--        Render.path(out.ahole, color(255,255,0,255), 4)
	end

	if out.aforest then
		for _,s in pairs(out.aforest) do
			local r = 0.02*math.sqrt((s-core_camera.getPosition()):length())
			debugDrawer:drawSphere(s, r, ColorF(1,1,0,1),false)
--            if W.out.apop and W.out.apop[_] then
--            else
--                debugDrawer:drawSphere(s, r, ColorF(1,1,0,1),true)
--            end
	--        debugDrawer:drawSphere(s, 0.2, ColorF(1,1,0,1))
		end
	end
	if out.inseed then
		local rayCast = cameraMouseRayCast(false, im.flags(SOTTerrain))
		if rayCast then
			local r = 0.01*math.sqrt((rayCast.pos-core_camera.getPosition()):length())
			debugDrawer:drawSphere(rayCast.pos + vec3(0,0,0.25), r*4, ColorF(0.3,1,0.2,.6), false)
			Render.path({rayCast.pos, rayCast.pos + vec3(0,0,0.2)}, color(0,255,255,255), 3)
		end
	end
	if out.aedge then
		for i = 1,#out.aedge.e-1 do
--        for _,d in pairs(out.aedge.e) do
			debugDrawer:drawLine(out.aedge.e[i], out.aedge.e[i+1], ColorF(1,1,1,1), 4)
--      ffi.C.BNG_DBG_DRAW_Line(
--            float x1, float y1, float z1,
--            float x2, float y2, float z2, color(255, 255, 255, 255), bool useZ);

--            debugDrawer:drawLine(d.e[1], d.e[2], ColorF(1,1,1,1))
--            Render.path(out.aedge.e, color(255, 255, 255, 255), 2)
		end
	end
	if out.flbl then
		for _,l in pairs(out.flbl) do
			if l[2] then
				debugDrawer:drawText(l[1], String(tostring(l[2])), ColorF(0,0,1,1))
			end
		end
	end

--[[
	if out.asplit then
--            U.dump(out.asplit, '?? mark_split:')
		for _,e in pairs(out.asplit) do
			debugDrawer:drawLine(e[1], e[2], ColorF(1,1,0,1))
		end
	end
]]

--            if editor.keyModifiers.alt then
--                lo('?? for_ALT:'..tostring(cameraMouseRayCast(false).object.name))
--            end

	local rayCast,rayCastHit = cameraMouseRayCast(false)
	if rayCast then
		if editor.keyModifiers.alt and not indrag then
			mpointAlt(rayCast)
		end
	end

	if out.dyell then
		for i,row in pairs(out.dyell) do
			for j,pth in pairs(row) do
				if j == 'len' then
					goto continue
				end
				local incorner
				if out.acorner then
--          if indrag then
--  					goto continue
--          end
--                        lo('?? ACORN0:')
--                        U.dump(out.acorner, '?? acorn:')
					for _,c in pairs(out.acorner) do
						if i==c.ij[1] then
							if j==c.ij[2] then
								incorner = 4
							end
							if U.mod(j+1,row.len)==c.ij[2] then
								incorner = 2
							end
						end
					end
				end

				for i=1,#pth-1 do
					local clr = ColorF(1,1,0.5,0.1)
					local w = 4
					if incorner and i==incorner then
--                            lo('?? inCORN:'..i..':'..j) --..':'..incorner)
--                            U.dump(pth, '?? atCORN:'..#pth..':'..i..':'..j)
--                        clr = ColorF(0,1,1,0.1)
						w = 1
					end
--                    if out.acorner and out.acorner[i][j] then
--                        debugDrawer:drawLineInstance(pth[i], pth[i+1], 4, ColorF(0.5,1,0.5,0.1))
--                    end
					debugDrawer:drawLineInstance(pth[i], pth[i+1], w, clr)
	--                debugDrawer:drawLine(pth[i], pth[i+1], ColorF(1,1,0.5,0.5), 6)
				end
				::continue::
			end
		end
	end

--    out.acorner = nil

	if im.IsWindowFocused(im.FocusedFlags_AnyWindow) or im.IsWindowHovered(im.HoveredFlags_AnyWindow) or im.IsAnyItemHovered() then return end

	if not out.inedge then
		out.aedge = nil
	end
	out.axis = nil
	out.middle = nil
--    if not out.inaxis then
--    end
--[[
	if im.IsMouseReleased(0) then
		lo('?? W_mup:')
	end
	if im.IsMouseDragging(0) then
		lo('?? W_drag:')
	end
]]

-- UI VALUES
--    uiUpdate()

	if rayCast then
--                if editor.keyModifiers.shift then
--                    lo('?? for_obj_hit:'..rayCast.object.name)
--                end
--        if im.IsMouseClicked(0) then
--            lo('?? for_RCSDSS:'..tostring(cameraMouseRayCast(true).object.name)..':'..tostring(cameraMouseRayCast(false).object.name))
--        end
		if im.IsMouseDragging(0) then
--                lo('?? unupd_drag:',true)
--            mdrag(rayCast)
--            lo('?? for_VE:'..tostring(out.invertedge))
--            return
		end
		-- mouse event
--        if true then
		if not indrag then
			local desc = adesc[cedit.mesh]
			-- on-the-fly object id
			local id = rayCast.object:getID()

			if ({building=1,floor=1,wall=1,side=1})[scope] then
--            if (scope == 'building' or scope == 'side') then
-- CORNER LINE render
				if rayCast.object and rayCast.object.name then
					local otype = name2type(rayCast.object.name)
					if otype == 'wall' then
--                            lo('?? for_hit:')
						local ray = getCameraMouseRay()
						local ishit
						forBuilding(desc, function(w, ij)
							local floor = desc.afloor[ij[1]]
							for k,p in pairs({
								base2world(desc, ij),
								base2world(desc, {ij[1], ij[2]+1})}) do
								if ray2segment(ray, p, p+vec3(0,0,floor.h)) < 0.01 then
--                                        lo('?? edge_CLOSE:')
									if w.u:cross(w.v):dot(ray.dir) < 0 then
										ishit = {ij={ij[1],U.mod(ij[2]+k-1,#floor.base)}, line={p, p+vec3(0,0,floor.h)}}
										break
									end
								end
							end
						end)
						if ishit then
              if ({building=1,floor=1})[scope] and editor.keyModifiers.ctrl then
                cij = ishit.ij
              end
--                                lo('?? ishit:'..tostring(ishit.ij[2]))
--!!                            out.invertedge = ishit
							out.acorner = {ishit}
--                                lo('?? in_AC:'..out.acorner[1].ij[2]..':'..#desc.afloor[2].base)
							forBuilding(desc, function(w, ij)
								if ij[1] == ishit.ij[1] then return end
								local p = base2world(desc,ij)
								if U.proj2D(ishit.line[1] - p):length() < small_dist then
									--to list
									out.acorner[#out.acorner + 1] = {ij = ij,
										line = {p, p + vec3(0,0,desc.afloor[ij[1]].h)}}
								end
							end)
--                                lo('?? for_corners:'..#out.acorner..':'..tostring(out.invertedge))
							incorner = out.acorner
--    							out.acorner = nil
						else
							incorner = false
							out.acorner = nil
						end
					end
				end
      else
        out.acorner = nil
			end
--                        if true then return end
--[[

					if false and string.find(rayCast.object.name, 'o_wall_') == 1 then
						-- check for close border
						if desc then
							local ray = getCameraMouseRay()
							local ij = aedit[id].desc.ij
							local floor = desc.afloor[ij[1] ]
							local base = floor.base
							local a = base2world(desc, ij) --base[ij[2] ] + desc.pos +
							local b = base2world(desc, {ij[1], ij[2] % #base + 1}) --base[ij[2] % #base + 1] + desc.pos
			--                    lo('?? ab:'..tostring(a)..':'..tostring(b)..':'..tostring(desc.pos)..':'..tostring(rayCast.pos))
							for k,p in pairs({a, b}) do
								local campos = core_camera.getPosition()
								local s = closestLinePoints(p, p + vec3(0,0,floor.h), campos, campos + ray.dir)
								local ps = p + vec3(0,0,floor.h*s)
									if editor.keyModifiers.ctrl then
										local vp = p - core_camera.getPosition()
--                                        lo('?? for dir:'..s) --..tostring(ray.dir)..':'..tostring(vp))
									end
								local isin
								if U.vang(ray.dir, ps - core_camera.getPosition()) < 0.01 then
--                                if rayCast.pos:distanceToLine(p, p + vec3(0,0,1))/(core_camera.getPosition() - rayCast.pos):length() < 0.02 then
--                                        U.dump(aedit[id].desc.ij, '?? for_CORNER:'..id)
--                                        _dbdrag = true
									ij = {ij[1], k == 1 and ij[2] or (ij[2] % #base + 1)}
									forBuilding(desc, function(w, tij)
										if tij[1] == ij[1] and tij[2] == ij[2] then
											-- is in scope
											isin = true
										end
									end)
									if isin then
										-- get line bottom
										local zmi,zma = extHeigh(desc)
										p.z = zmi
										local H = zma - zmi
										out.invertedge = {
											ij = ij,
											line = {p, p + vec3(0,0,H)}}
	--                                        line = {p, p + vec3(0,0,forHeight(desc.afloor))}}
										incorner = true
									end
									break
								end
								if not isin then
									out.invertedge = nil
									incorner = false
--                                        lo('?? out_CORNER:', true)
								end
								if rayCast.pos:distanceToLine(p, p + vec3(0,0,1)) < 0.2 then
			--                        lo('?? close:')
			--                        out.apath = {{p, p + vec3(0,0,forHeight(desc.afloor))}}
			--                        out.invertedge = {{p, p + vec3(0,0,forHeight(desc.afloor))}}
								end
							end
						end
					else
--                        out.invertedge = nil
--                        incorner = false
					end
	--                    _dbdrag = true
				else
	--                lo('!! W.NO_RC_name:')
]]

			if im.IsWindowFocused(im.FocusedFlags_AnyWindow) or im.IsWindowHovered(im.HoveredFlags_AnyWindow) or im.IsAnyItemHovered() then
			else

	--                    lo('?? if_CO__1:'..tostring(rayCast.object.name))
				if editor.keyModifiers.shift and not out.inedge then
	--                        lo('?? for_obj_hit:'..rayCast.object.name)
					local otype,ij = name2type(rayCast.object.name)
	--[[
					if otype == 'lid' then
						ij[1] = #desc.afloor
					elseif otype == 'wall' then
						ij = aedit[id].desc.ij
					end
	]]
					if aedit[id] and aedit[id].desc.ij then
	-- for BASE BORDER
						local ij = aedit[id].desc.ij
						local ifloor = ij[1]

						local floor
						if desc then
							floor = desc.afloor[aedit[id].desc.ij[1]]
						end
	--                        U.dump(aedit[id].desc.ij, '?? for_ij:'..tostring(otype))

						if floor then

	--                lo('?? look for borders:'..tostring())
							local base = floor.base
							local ijhit
	--                            U.dump(ij, '?? if_hit:')
							local istop
							local ray = getCameraMouseRay()
							local lift = vec3(0,0,0)
							if otype == 'lid' then
								for j = 1,#floor.base do
									if rayClose(ray, desc, {ij[1],j}, true) then
	--                                    lo('?? is_hit_top_lid:'..ij[1])
										ijhit = {ij[1],j}
										istop = true
										lift = vec3(0,0,floor.h)
										break
									end
								end
							else
								if rayClose(ray, desc, ij) then
	--                                lo('?? is_hit:'..ij[1]..':'..ij[2])
									ijhit = ij
								elseif rayClose(ray, desc, ij, true) then
	--                                lo('?? is_hit_top:'..ij[1]..':'..ij[2])
									ijhit = ij
									istop = true
									lift = vec3(0,0,floor.h)
								end
							end
							if U._PRD==0 and ijhit then
	--                                U.dump(ijhit, '?? ijhit:'..tostring(istop))
								out.aedge = {e = {}, ij = ijhit, top = istop}
								if scope == 'wall' then
									out.aedge.e = {
	base2world(desc, ijhit) + lift,
	base2world(desc, {ij[1],U.mod(ijhit[2]+1,#base)}) + lift}
								else
									local jma = 0
									for j = 1,#base do
	--[[
										if floor.awall[j].u:cross(vec3(0,0,1)):dot(ray.dir) < 0 then
											out.aedge.e[#out.aedge.e+1] =
	base2world(desc, {ij[1],j}) + (istop and vec3(0,0,floor.h) or vec3(0,0,0))
											jma = j
										end
	]]
										out.aedge.e[#out.aedge.e+1] =
	base2world(desc, {ij[1],j}) + lift
									end
									out.aedge.e[#out.aedge.e+1] =
	base2world(desc, {ij[1],1}) + lift
								end
							else
								out.aedge = nil
							end

	--                        if scope == 'floor' then
	--                            for i,_ in pairs(base) do
	--                            end
	--                        end
							if false and #floor.base == 4 then
								for i = 1,#base do
		--                        lo('?? for_p:'..tostring(base2world(desc, ij)))
		--                        _dbdrag = true
									local a,b = base2world(desc, ij), base2world(desc, {ij[1],U.mod(i+1,#base)})
									if a and b then
										local s = rayCast.pos:distanceToLine(a, b)
										if s < 0.2 then
				--                            lo('?? HIT:'..ij[1]..':'..ij[2])
											out.aedge = {{e = {a, b}, ij = ij}}
											break
										end
									end
								end
							end
	-- FLOOR CENTER
							local c = U.polyCenter(base)
							c = c + base2world(desc, {ifloor, 1}) - base[1] + vec3(0,0,floor.h)
	--[[
						local c = vec3(0,0,0)
						local s = 0
						for j = 1,#base do
							local ds = (U.mod(j+1,base) - U.mod(j,base)):length() + (U.mod(j,base) - U.mod(j-1,base)):length()
							c = c + base2world(desc, {ifloor, j})*ds
							s = s + ds
						--                    c = c + base2world(desc, {ifloor, j}) + vec3(0,0,floor.h)
						end
						c = c/s + vec3(0,0,floor.h)
	]]
							--                c = c/#base
							if U._PRD == 0 then
								out.axis = {pos = c}
							end
						end
					else
	--                    lo('!! no_IJ:'..id..':'..tostring(aedit[id]))
					end

	-- SPLITTED WALL MIDDLE
					if scope == 'wall' and U._PRD == 0 then
						local base = desc.afloor[cij[1]].base
						local pm = (base2world(desc, cij) + base2world(desc, {cij[1],U.mod(cij[2]+1,#base)}))/2
	--                        local pm = base2world(desc, base[aside[cij[1]][1]]) + base2world(desc, U.mod(aside[cij[1]][3]+1,base))
						out.middle = {line = {pm, pm + vec3(0,0,desc.afloor[cij[1]].h)}, ij = cij}
					elseif scope == 'floor' and U._PRD == 0 then
	--                if ({wall=1,floor=1})[scope] then
	--                if scope == 'wall' then
						-- check for middle
						local aside = forSide(cij)
						local u = desc.afloor[cij[1]].awall[cij[2]].u:normalized()
						local base = desc.afloor[cij[1]].base
						local pmi,pma,mi,ma = math.huge,0
						for i,r in pairs(aside) do
							if i == cij[1] then
								for _,j in pairs(r) do
									local prj = (U.mod(j,base) - base[cij[2]]):dot(u)
									if prj < pmi then
										pmi = prj
										mi = j
									end
									if prj > pma then
										pma = prj
										ma = j
									else
										prj = (U.mod(j+1,base) - base[cij[2]]):dot(u)
										if prj > pma then
											pma = prj
											ma = U.mod(j+1,#base)
										end
									end
	--                                if j < mi then mi = j end
	--                                if j > ma then ma = j end
								end
							end
						end
	--                        U.dump(aside, '?? PM:'..tostring(mi)..':'..tostring(ma)..':'..tostring(u)..':'..cij[2])
						if mi and ma then
	--                        ma = U.mod(ma+1,#base)
							local p = (U.mod(mi,base) - base[cij[2]] + U.mod(ma,base) - base[cij[2]]):dot(u)
							local jp = cij[2]
							for k,j in pairs(aside[cij[1]]) do
								local p1,p2 = (base[j] - base[cij[2]]):dot(u),(U.mod(j+1,base) - base[cij[2]]):dot(u)
								if p1 <= p and p <= p2 then
									jp = j
								end
							end
	--                            lo('?? for_JP:'..jp)
							local pm = base2world(desc,cij, base[jp] + u*(base[cij[2]]-base[jp]):dot(u) + u*p/2)
							out.middle = {line = {pm, pm + vec3(0,0,desc.afloor[cij[1]].h)}, ij = cij}
	--                            lo('?? PM2:'..tostring(pm)..':'..mi..':'..ma..':'..tostring(u)..':'..cij[2], true)
						end
	--                       U.dump(aside, '?? for_side_m: i='..cij[1]..':'..mi..':'..ma)
	--                    local pm = (base2world(desc, {cij[1],mi}) + base2world(desc, {cij[1],U.mod(ma+1,#base)}))/2
	--                        local pm = base2world(desc, base[aside[cij[1]][1]]) + base2world(desc, U.mod(aside[cij[1]][3]+1,base))
	--[[
						if #aside[cij[1] ] == 3 and cij[2] == aside[cij[1] ][2] then
							local base = desc.afloor[cij[1] ].base
							local pm = (base2world(desc, {cij[1],aside[cij[1] ][1]}) + base2world(desc, {cij[1],U.mod(aside[cij[1] ][3]+1,#base)}))/2
					--                        local pm = base2world(desc, base[aside[cij[1] ][1] ]) + base2world(desc, U.mod(aside[cij[1] ][3]+1,base))
							out.middle = {line = {pm, pm + vec3(0,0,desc.afloor[cij[1] ].h)}, ij = cij}
					--                            U.dump(aside, '?? for_side:'..cij[1]..':'..#aside[cij[1] ]..':'..tostring(pm))
					--                            _dbdrag = true
						end
	]]
					end

				end

			end
		end
		if im.IsMouseDragging(0) then
--                lo('?? unupd_drag:',true)
			mdrag(rayCast)
--            lo('?? for_VE:'..tostring(out.invertedge))
--            return
		end

--        if string.find(rayCast.object.name, 'o_') ~= 1 then
--            out.wsplit = nil
--        end
	end
	-- wall marking
--                    if out.acorner then
--                        U.dump(out.acorner, '?? acorn0:')
--                    end
	------------------------
	-- ANIMS
	------------------------
	local anistep = 0.1
	if inanim then
		indrag = true
		if not inanim.c then inanim.c = 0 end
		if inanim.c < 1 then
			inanim.c = inanim.c + anistep
			if inanim.c > 1 then inanim.c = 1 end
			inanim.cb(inanim.ain, inanim.amm, inanim.c)
		else
			inanim.ondone()
			inanim = nil
			indrag = false
		end
	end

	--------------------
	-- KB events
	--------------------
	if im.IsKeyPressed(im.GetKeyIndex(im.Key_Backspace)) then
--        if csave > #asave then return end

		fromJSON(U._PRD == 0 and fsave..'_'..asave[csave][1]..'.json' or nil)
--[[
		local desc = adesc[cedit.mesh]
		if not desc or not desc.id then return end
		-- remove current
		forBuilding(desc, function(w,ij)
			forestClean(w)
			w.df = {}
		end)
		for i,f in pairs(desc.afloor) do
			forestClean(f)
			f.df = {}
		end
		scenetree.findObject('edit'):deleteAllObjects()
		scenetree.findObjectById(desc.id):delete()
				lo('?? RESTORE:'..csave..':'..tostring(asave[csave])..'-'..desc.id)
		-- restore from history
		desc = jsonReadFile(fsave..'_'..asave[csave]..'.json')
		desc = U.fromJSON(desc)
		desc.id = nil
		for i,f in pairs(desc.afloor) do
			for j,w in pairs(f.awall) do
				w.id = nil
				w.df = {}
			end
			f.top.df = {}
		end
--            lo('?? fj1:'..tostring(desc.pos)..':'..tostring(desc.id))
	--        U.dump(desc.pos, '?? fj1:'..tostring(desc.pos))
		houseUp(desc)
]]
--        csave = csave + 1
	end
	if editor.keyModifiers.ctrl then
--            lo('?? for_CTRL:'..tostring(im.IsKeyDown(im.GetKeyIndex(im.Key_J))))
		local desc = adesc[cedit.mesh]

		if im.IsKeyPressed(im.GetKeyIndex(im.Key_C)) then
-- TO COPY BUFFER
				if desc.prn then
					U.dump(adesc[desc.prn].selection, '?? to_COPY_fromparent:')
				else
					U.dump(desc.selection, '?? to_COPY:')
				end
			if desc.selection then
				incopy[cedit.mesh] = adesc[cedit.mesh].selection
				adesc[cedit.mesh].selection = nil
			else
				incopy[cedit.mesh] = {}
				forBuilding(desc, function(w, ij)
					if not incopy[cedit.mesh][ij[1]] then
						incopy[cedit.mesh][ij[1]] = {}
					end
					if #U.index(incopy[cedit.mesh][ij[1]], ij[2]) == 0 then
						incopy[cedit.mesh][ij[1]][#incopy[cedit.mesh][ij[1]]+1] = ij[2]
					end
--!!                    incopy[cedit.mesh][ij[1]][ij[2]] = true
				end)
			end
				U.dump(incopy[cedit.mesh], '?? copied:')
--        elseif im.IsKeyPressed(im.GetKeyIndex(im.Key_V)) then
		elseif im.IsKeyPressed(im.GetKeyIndex(im.Key_V)) then
-- PASTE
					U.dump(incopy[cedit.mesh], '??+++++++++++++++++++++++++++++++ to_PASTE:'..tableSize(incopy)..':'..tostring(out.invertedge))
			-- TODO: for between buildings
			local dsrc = desc
--                            if not out.invertedge then return end
			if false and U._PRD == 0 and not incopy[cedit.mesh] then
				if tableSize(incopy) == 1 then
					for id,buf in pairs(incopy) do
						local akey = {}
						for k,r in pairs(buf) do
							akey[#akey + 1] = k
						end
						table.sort(akey)
							U.dump(akey, '?? sorted:')
						local dsrc = adesc[id]
							U.dump(buf, '?? for_them:'..tostring(dsrc)..':'..tostring(scope))
						local ftgt = desc.afloor[cij[1]]
						local afloor = {}
						for _,k in pairs(akey) do
--                        for i,r in pairs(buf) do
							local r = buf[k]
							if #r ~= #dsrc.afloor[k].base then
								-- incomplete floor
								return
							end
							afloor[#afloor + 1] = deepcopy(dsrc.afloor[k])
						end
							lo('?? af:'..#afloor)
						if cij[1] == #desc.afloor then
							-- append floors
							for _,f in pairs(afloor) do
								floorClean(f)
								desc.afloor[#desc.afloor + 1] = f
							end
							-- update tops
							ftgt.top.shape = 'flat'
							ftgt.top.margin = 0
							afloor[#afloor].top.body = ftgt.top.body
							ftgt.top.body = coverUp(ftgt.base)
--                                lo('?? forf:'..#desc.afloor)
							houseUp(nil, cedit.mesh)
							return
						end
					end
				end
			else
					lo('?? paste_SAME:'..tableSize(incopy)..tostring(cedit.mesh)..':'..tostring(out.acorner)..':'..tostring(scope))
				local buf = incopy[cedit.mesh]
				if not buf then
					if tableSize(incopy) == 1 then
						for id,b in pairs(incopy) do
							buf = b
							dsrc = adesc[id]
						end
					end
				end
				local akey = {}
				for k,r in pairs(buf) do
					akey[#akey+1] = k
				end
--                local desc = adesc[cedit.mesh]
				local floor = desc.afloor[cij[1]]
--                if out.invertedge then
				if U._PRD == 0 and scope ~= 'top' then

					if out.acorner then
	--- INSERT
						if #akey == 1 and sameSide(buf) then
							local fsrc = dsrc.afloor[akey[1]]
							local ij = out.invertedge.ij
	--                            U.dump(buf, '?? INSERT:'..ij[2])
							local ftgt = desc.afloor[ij[1]]
							local u = ftgt.awall[ij[2]].u:normalized()
	--                        local p = floor.base[ij[2]]
							local awall = {}
							local L = 0
							local jmi = math.huge
							for _,j in pairs(buf[akey[1]]) do
	--                            for j,_ in pairs(buf[akey[1]]) do
								awall[#awall+1] = deepcopy(dsrc.afloor[akey[1]].awall[j])
								L = L + fsrc.awall[j].u:length()
								if j < jmi then jmi = j end
							end
							local psrc = fsrc.base[jmi]
								lo('?? for_L:'..L..' jmi:'..jmi..':'..#awall, true)
	--[[
							for _,j in pairs(buf[akey[1] ]) do
	--                            for j,_ in pairs(buf[akey[1] ]) do
	end
	for _,j in pairs(buf[akey[1] ]) do
	--                                    for j,_ in pairs(buf[akey[1] ]) do
	end
	]]
							-- handle base
							local p = ftgt.base[ij[2]] --table.remove(floor.base,ij[2])
							local dir = U.less(p, psrc, u)
	--                            U.dump(ftgt.base, '?? base_PRE:'..ij[2])
	--                            if true then return end

							local function forShift(ain, amm, c)
	--                            local c = amm.L[3] and amm.L[3] or 0
	--                            local base = U.clone(ain.base)
								local L = amm.L[1] + (amm.L[2] - amm.L[1])*c
								for j,q in pairs(ain.base) do
									if ain.dir < 0 then
										--?? <=
	--                                    if j == #ain.base then
	--                                        lo('?? for4:'..tostring(U.less(q, ain.p, ain.u)),true)
	--                                    end
										if U.less(q, ain.p, ain.u) <= small_dist then
											--- shift back
											if j ~= ij[2] then
	--                                        if ain.u:dot(fsrc.awall[jmi].u) < 0 then
	--                                                lo('?? shifting:'..j, true)
												ain.baseout[j] = ain.base[j] - ain.u*L
											end
										end
									else
										if U.less(q, ain.p, ain.u) >= 0 then
											--- shift forth
											if j ~= ij[2] then
												ain.baseout[j] = ain.base[j] + ain.u*L
											end
										end
									end
								end
								houseUp(desc, cedit.mesh)
							end

							local function onShifted()
	--                                lo('>> onShifted:'..#ftgt.base, true)
								if true then
									if dir < 0 then
										ftgt.base[ij[2]] = ftgt.base[ij[2]] - u*L
									else
										ftgt.base[ij[2]] = ftgt.base[ij[2]] + u*L
									end
									p = ftgt.base[ij[2]]
										U.dump(ftgt.base, '?? base_SHIFTED:'..ij[2])

									-- to walls
									local n = 0
									for _,j in pairs(buf[akey[1]]) do
	--                                    for j,_ in pairs(buf[akey[1]]) do
										local w = awall[n+1] -- deepcopy(dsrc.afloor[akey[1]].awall[j])
										for dae,r in pairs(w.df) do
											w.df[dae] = {scale = r.scale}
										end
	--                                    w.df = {}
	--                                    w.ij[1] = ij[1]
										-- to walls
											lo('?? ins_wall:'..(ij[2]+n)..':'..w.u:length(), true)
										table.insert(ftgt.awall, ij[2]+n, w)
										n = n + 1
									end

									-- to base
									n = 1
									for _,j in pairs(buf[akey[1]]) do
	--                                    for j,_ in pairs(buf[akey[1]]) do
	--                                        lo('?? inserting:'..(ij[2]+n), true)
										p = p + u*awall[n].u:length()
										table.insert(ftgt.base, ij[2]+n, p)
	--                                    if n < #awall then
	--                                        p = p + u*awall[n+1].u:length()
	--                                    end
	--                                    p = p + u*fsrc.awall[U.mod(j+1,#fsrc.base)].u:length()
										n = n + 1
									end
	--                                    lo('?? ins_POST:'..#ftgt.base, true)
	--                                    U.dump(ftgt.base, '?? for_BASE2:')
									baseOn(ftgt)
	--                                    U.dump(ftgt.base, '?? insed:'..#ftgt.base..':'..#ftgt.awall, true)
	--                                    U.dump(ftgt.base, '?? for_BASE3:')
								end
								houseUp(desc, cedit.mesh)
							end
	--                            indrag = false
							-- for animated shift
							inanim = {ain = {p = p, psrc = psrc, dir = dir, u = u, base = U.clone(ftgt.base), baseout = ftgt.base, L = L},
								amm = {L = {0, L}}, cb = forShift, ondone = onShifted}
						end
						return
					elseif scope == 'floor' then
							U.dump(buf, '?? for_FLOOR:'..tostring(buf))
						if #akey == 1 then
							local buflen = tableSize(buf[akey[1]])
							if buflen == 0 or buflen == #dsrc.afloor[akey[1]].awall then
	-- FLOOR REPLACE
									lo('?? paste_FLOOR:'..floor.top.id)
								-- cleanup
								for j,w in pairs(floor.awall) do
									forestClean(w)
									scenetree.findObjectById(w.id):delete()
								end
								forestClean(floor.top)
								scenetree.findObjectById(floor.top.id):delete()

								local newfloor = deepcopy(desc.afloor[akey[1]])
								if cij[1] == #desc.afloor then
									newfloor.top = floor.top
								end
								for j,w in pairs(newfloor.awall) do
									for dae,r in pairs(w.df) do
										w.df[dae] = {scale = r.scale or 1}
									end
								end
								for dae,r in pairs(newfloor.top.df) do
									newfloor.top.df[dae] = {scale = r.scale or 1}
								end
								desc.afloor[cij[1]] = newfloor
								baseOn(newfloor)
								houseUp(nil, cedit.mesh)

								return
	--                            houseUp(desc, cedit.mesh)
							end
						end
	--                        return
					elseif scope == 'wall' then
							U.dump(buf,'?? REPLACE:'..#akey)
						if #akey == 1 then
	--- WALL REPLACE
							if sameSide(buf) then
										U.dump(cij, '?? SSide:'..cij[1])
										U.dump(desc.selection, '?? if_sel:'..tableSize(desc.selection))

								local jmi = math.huge
								if tableSize(desc.selection) == 1 and sameSide(desc.selection) then
									local buf = {}
									for i,r in pairs(desc.selection) do
										table.sort(r)
										buf[i] = r
										jmi = table.remove(buf[i],1)
									end
										U.dump(buf, '?? to_coll:')
									wallCollapse(desc, buf)

									if false then

										local buftgt = desc.selection
										if tableSize(buftgt) > 1 then return end
										local ftgt
										-- remove
										local acol = {}
												U.dump(buftgt, '?? BT:')
										for i,r in pairs(buftgt) do
											ftgt = desc.afloor[i]
												lo('?? for_row:'..i..':'..tostring(desc)..':'..tostring(ftgt))
											for j,_ in pairs(r) do
												acol[#acol+1] = j
												if j < jmi then
													jmi = j
												end
											end
										end
											lo('?? ftgt_pre:'..#acol..':'..jmi..':'..#ftgt.base)
										for o,j in pairs(acol) do
											if o > 1 then
												table.remove(ftgt.base, jmi+1)
												local w = table.remove(ftgt.awall, jmi+1)
												forestClean(w)
													lo('?? to_del:'..w.id)
												scenetree.findObjectById(w.id):delete()
											end
										end
											U.dump(ftgt.base, '?? ftgt_post:'..#acol..':'..#ftgt.base..':'..#ftgt.awall)
										-- insert
										for n,j in pairs(acol) do

										end
										baseOn(ftgt)

									end
	--                                houseUp(desc, cedit.mesh, true)
	--[[
									local akeytgt = {}
									for k,r in pairs(buftgt) do
										akeytgt[#akeytgt+1] = k
									end
									table.sort(akeytgt)
										U.dump(akeytgt, '?? keytgt:')
	]]
								else
									jmi = cij[2]
								end
									lo('?? paste_TO:'..jmi)
	--                                houseUp(nil, cedit.mesh)
	--                                    if true then return end
	--                            local s =
								if true then

									local fsrc = dsrc.afloor[akey[1]]
									local floor = desc.afloor[cij[1]]
										U.dump(floor.base, '?? base_pre:')
									-- go over source side
									local L, jpre = 0
									local awall = {}
									for _,j in pairs(buf[akey[1]]) do
		--                            for j,_ in pairs(buf[akey[1]]) do
											lo('?? for_wall:'..j)
										if jpre and j > jpre + 1 then
											lo('?? check_len:')
											if (U.mod(j,fsrc.base)-U.mod(j-1,fsrc.base)):length() == 0 then
													lo('?? emp_wall:'..j)
												awall[#awall+1] = deepcopy(fsrc.awall[j-1])
											end
										end
										awall[#awall+1] = deepcopy(fsrc.awall[j])
										local w = awall[#awall]
										for dae,d in pairs(w.df) do
											w.df[dae] = {scale = d.scale}
										end
		--                                awall[#awall].df = {}
		--                                forestClean(awall[#awall])
										awall[#awall].ij[1] = cij[1]
										L = L + (U.mod(j+1,fsrc.base)-U.mod(j,fsrc.base)):length()
										jpre = j
									end
									local u = floor.awall[jmi].u:normalized()
									local c = floor.awall[jmi].u:length()/L
										lo('?? nwalls:'..#awall..':'..L..':'..jmi..':'..c)

									-- wall down
									scenetree.findObjectById(floor.awall[jmi].id):delete()
									forestClean(floor.awall[jmi])
									table.remove(floor.awall,jmi)

		--                                local wnew = deepcopy(fsrc.awall[1])
		--                                wnew.df = {}
		--                                table.insert(floor.awall,cij[2],wnew)
		--                                    U.dump(fsrc.awall[1], '?? fsrc:'..#fsrc.awall)

									local iind = jmi
									local p = table.remove(floor.base,jmi)
		--                                    lo('?? iind:'..iind)
									for j,w in pairs(awall) do
										-- to base
										table.insert(floor.base, iind+j-1, p)
										p = p + u*w.u:length()*c
									end
									for j = #awall,1,-1 do
										-- to walls
										table.insert(floor.awall, iind, awall[j])
		--                                    floor.awall[#floor.awall].ij = {cij[1],}
									end

		--                                U.dump(fsrc.awall[1], '?? w_src:')
		--                                U.dump(floor.awall[4], '?? w_tgt:')
		--                                U.dump(floor.base, '?? new_base:'..#floor.awall)
		--                            baseOn(fsrc)
									baseOn(floor)
									houseUp(desc, cedit.mesh)

								end
	--                            houseUp(desc, cedit.mesh, true)
							end
						end
						return
					end
				elseif scope == 'top' then

					table.sort(akey) -- floors index list
	-- PASTE child to floor
						U.dump(buf, '?? to_PASTE_f:'..cij[1]..':'..tostring(floor.pos))
						U.dump(akey, '?? for_topaste:'..tostring(floor.pos)..':'..tostring(scope))
	--                    U.dump(desc.afloor[2].top, '?? 2nd_floor_PRE:'..#desc.afloor[2].awall)
					--- make building
					local dnew = {afloor = {}, pos = floor.pos, prn = desc.id, floor = nil}

					local newfloor
					if scope == 'floor' then
						if #U.index(akey, cij[1]) == 1 then
							newfloor = akey[1]
						else
							newfloor = cij[1] + 1
						end
					elseif scope == 'top' then
						newfloor = cij[1] + 1
					end
					dnew.floor = newfloor
					--TODO: specify criterion
					local samefloor = (akey[1] == newfloor and akey[#akey] == #dsrc.afloor)
							lo('?? newfloor:'..tostring(samefloor)..':'..tostring(newfloor)..'/'..#desc.afloor..':'..tostring(smouse))
					if newfloor then --and newfloor < #desc.afloor then
						-- duplicate on same floor
						local H = 0
						for i = #akey,1,-1 do
	--                        lo('?? for_key:'..akey[i]..':'..tostring(buf[akey[i]]))
							if #buf[akey[i]] == #dsrc.afloor[akey[i]].base then
	--                                U.dump(desc.afloor[akey[1]].top, '?? whole:'..i..':'..akey[i])
								table.insert(dnew.afloor, 1, deepcopy(dsrc.afloor[akey[i]]))
								-- remove forest
								dnew.afloor[1].top.df = {}
								dnew.afloor[1].top.ij[1] = #akey - #dnew.afloor + 1
								for _,w in pairs(dnew.afloor[1].awall) do
									for dae,r in pairs(w.df) do
										w.df[dae] = {scale = w.df[dae].scale}
									end
	--                                w.df = {}
									-- reset ij
	--                                w.ij[1] = #akey - #dnew.afloor + 1
								end
								H = H + desc.afloor[akey[i]].h

								if samefloor then
									-- remove current floor elements
									if i == 1 then
										for j,w in pairs(desc.afloor[akey[i]].awall) do
											objDown(w)
										end
										objDown(desc.afloor[akey[i]].top)
										desc.afloor[akey[1]].awall = {}
										desc.afloor[akey[1]].top = nil
										desc.afloor[akey[1]].h = H
									else
										table.remove(desc.afloor, akey[i])
									end
								end
							end
						end
						if not floor.achild then
							floor.achild = {}
						end
						if samefloor then
							floor.achild[#floor.achild+1] = dnew
						end
	--                        lo('?? newpos_pre:'..tostring(rayCast.pos)..':'..tostring(desc.afloor[newfloor].pos))
						local posnew = U.proj2D(rayCast.pos - (desc.pos + desc.afloor[newfloor].pos + desc.afloor[newfloor].base[1]))
						if true then
							floor.achild[#floor.achild+1] = deepcopy(dnew)
							floor.achild[#floor.achild].pos = posnew -- floor.achild[#floor.achild].pos + vec3(4,0,0)
							-- update by-reference object
							floor.achild[#floor.achild].prn = dnew.prn
						end
							lo('??****** NEWPOS:'..tostring(posnew)..'/'..tostring(desc.pos)..':'..tostring(desc.afloor[newfloor].pos)..':'..tostring(desc.afloor[newfloor].base[1])..':'..#desc.afloor[2].awall)

						houseUp(desc, cedit.mesh, true)
					end
					incopy[cedit.mesh] = nil

				end
			end
		end
	end
	if editor.keyModifiers.shift and not out.inaxis then
-- FLY AROUND
--            lo('?? for_SHIFT:')
		local dir
		if im.IsKeyDown(im.GetKeyIndex(im.Key_RightArrow)) then
			dir = 1
		end
		if im.IsKeyDown(im.GetKeyIndex(im.Key_LeftArrow)) then
			dir = -1
		end
--            lo('?? if_ARO:'..tostring(dir))
		if dir then goAround(dir) end
	end

----------------------------------
-- MARKUPS
----------------------------------
--[[
	if out.invertedge then
--            lo('?? for_p:'..tostring(out.invertedge.line))
		local p = out.invertedge.line
--            lo('?? for_oive:'..tostring(p))
		for i = 2,2 do
--            debugDrawer:drawLine(p[i-1], p[i], ColorF(1,1,0,1), 2)
		end
	end
]]
	if out.acorner and not indrag then
--        lo('?? AC2:')
--        U.dump(out.acorner, '?? AC2:')
		for _,e in pairs(out.acorner) do
			debugDrawer:drawLine(e.line[1], e.line[2], ColorF(1,1,0,1), 2)
		end
	elseif out.asplit then
--            U.dump(out.asplit, '?? mark_split:')
		for _,e in pairs(out.asplit) do
			debugDrawer:drawLine(e.line[1], e.line[2], ColorF(1,1,0,1))
		end
	elseif out.wsplit ~= nil then -- and inpoint then
--        lo('?? LINE:')
		debugDrawer:drawLine(W.out.wsplit[1], W.out.wsplit[2], ColorF(1,1,0,1))
	end

--        lo('?? for_FW:'..tostring(out.fwhite))
	if out.fcyan then
		for _,v in pairs(out.fcyan) do
			debugDrawer:drawLine(v[1], v[2], ColorF(0,1,1,1), 4)
		end
	end

	-- AXIS
	local axis
	if out.inaxis then
		axis = out.inaxis
	elseif out.axis then
		axis = out.axis.pos
	end
--[[
	if out.axis then
		axis = out.axis.pos
	elseif out.inaxis then
		axis = out.inaxis
	end
]]
	if axis then
		Render.circle(axis + vec3(0,0,0.01), 0.6, color(255,255,255,255), 2)
--        Render.circle(axis + vec3(0,0,0.01), 0.41, color(255,255,255,255))
		Render.path({axis, axis + vec3(0,0,0.5)}, color(255,255,255,200), 2)
--        debugDrawer:drawLine(axis, axis + vec3(0,0,0.5), ColorF(1,1,1,0.6))
	end

	-- MIDDLE
	if out.middle then
--            lo('?? for_MID:'..tostring(out.middle.line[1])..':'..tostring(out.middle.line[2]))
		debugDrawer:drawLine(out.middle.line[1], out.middle.line[2], ColorF(1,1,1,1))
	end

	U.markUp(out.mcyan, 0.1, ColorF(0,1,1,1))
end

--------------------------
-- CONTROLS EVENTS
--------------------------

local function windowsToggle()
	lo('>> windowsToggle:')
	if cij[3] then
		local wplus = adesc[cedit.mesh].afloor[cij[1]].awplus[cij[3][1]].list[cij[3][2]]
		wplus.win = {dae = daePath.win[1]}
			U.dump(wplus, '?? wT_wplus:')
	else
		forBuilding(adesc[cedit.mesh], function(w, ij)
			if w.winspace ~= nil then
				w.winspace = nil
			else
				w.winspace = default.winspace
			end
	--        lo('?? newind:'..tostring(w.doorind))
		end)
	end
--    houseUp(adesc[cedit.mesh], cedit.mesh, true)
end


local function doorToggle()
	U.dump(cij, '>>--------- doorToggle:'..tostring(cedit.part)..':'..scope)
	local wdesc
	if not out.inhole then
		forBuilding(adesc[cedit.mesh], function(w, ij)
	--            U.dump(w, '?? doorToggle:')
			if w.doorind ~= nil then
				w.doorind = nil
			elseif w.df[w.win] then
				w.doorind = math.ceil(#w.df[w.win]/2)
					U.dump(cij, '??____ DOOR_IND:'..w.doorind)
				wdesc = w
			end
	--        lo('?? newind:'..tostring(w.doorind))
		end)
	else
		wdesc = adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]]
				U.dump(wdesc.achild, '?? aCHILD:'..tostring(wdesc.door))
--        wdesc.df[wdesc.door] = {}
		local child = wdesc.achild[wdesc.achild.cur or #wdesc.achild]
		if child then
			child.yes = true
			child.body = wdesc.door
		end
--        out.inhole = nil
--        out.ahole = nil
--        wdesc.achild
	end
	houseUp(adesc[cedit.mesh], cedit.mesh, true)
	uiUpdate()

	if wdesc ~= nil then
--			U.dump(wdesc.df, '?? for_DF:'..tostring(wdesc.door))
		local list = wdesc.df[wdesc.door]
		if list then
			cedit.forest = list[#list]
		else
			lo('!! ERR_NO_DOOR:', true)
		end
	end
	markUp(wdesc ~= nil and wdesc.door or nil)
		lo('<< doorToggle:'..tostring(cedit.forest), true)
end


local function balcToggle()
		indrag = false
		lo('>> balcToggle:')
	out.inbalcony = not out.inbalcony
	local desc = adesc[cedit.mesh]
	forBuilding(desc, function(w, ij)
		if not w.balcony then
			w.balcony = {dae = daePath.balcony[1]}
		end
		w.balcony.ind = {W.ui.balc_ind0,W.ui.balc_ind1}
		w.balcony.yes = out.inbalcony -- not w.balcony.yes
		if W.ui.balc_ind2 > 0  then
			w.balcony.ind[#w.balcony.ind+1] = W.ui.balc_ind2
		end
	end)
	return true
--        return false
end


local function pillarSpan(n)
		indrag = false
		U.dump(cij, '>> pillarSpan:'..n)
	forBuilding(adesc[cedit.mesh], function(w, ij)
--        if not w.pillar then
--            w.pillar = deepcopy(default.pillar)
--        end
		if ij[1] - 1 - n < 0 then
			n = ij[1] - 1
			W.ui['pillar_spany'] = n
		end
		w.pillar.down = forHeight(adesc[cedit.mesh].afloor, ij[1] - 1) - forHeight(adesc[cedit.mesh].afloor, ij[1] - 1 - n)
--            U.dump(w.ij, '?? for_down:'..w.pillar.down..':'..n)
	end)
end


local function pillarToggle()
		U.dump(cij, '>> pillarToggle:')
	if cij[1] == 1 then return end
	forBuilding(adesc[cedit.mesh], function(w, ij)
		if not w.pillar then
			w.pillar = deepcopy(default.pillar)
--                U.dump(default.pillar,'?? p_new:'..ij[2]..':'..tostring(w.pillar.yes))
		end
		w.pillar.yes = not w.pillar.yes
		if not w.pillar.yes or not ddae[w.pillar.dae] then return end
--                lo('?? pT:'..tostring(w.pillar.dae)..':'..tostring(ddae[w.pillar.dae]))
		w.pillar.inout = -math.abs(ddae[w.pillar.dae].to.y - ddae[w.pillar.dae].fr.y)/2
--        w.pillar.margin = -w.pillar.inout
--            lo('?? for_pillar:'..ij[2])
		w.pillar.down = adesc[cedit.mesh].afloor[ij[1]-1].h
	end)
	out.curselect = 1
end


local function atticToggle()
	lo('>> atticToggle:')
end


local function corniceToggle()
	lo('>> corniceToggle:')
end


W.cornerUp = function(desc, dae, toreplace)
	local acorner = cornersBuild(desc, dae)
--        U.dump(acorner, '??^^^^^^^^^^^^^^^^^^ CB:')
--        U.dump(desc.acorner_, '??^^^^^^^^^^^^^^^^^^ CB_dac:')
	if desc.acorner_ then
--            U.dump(desc.acorner_, '?? to_SUBTRACT:', true)
		for n,p in pairs(acorner) do
--            local isext
			local hit
			for m,q in pairs(desc.acorner_) do

--                local hit
				for i,a in pairs(p.list) do
					for j,b in pairs(q.list) do
						if a[1] == b[1] and a[2] == b[2] then
							hit = true
--                                lo('?? is_HIT:'..m..':'..n)
--                                    lo('??--------------- to_REM:'..a[1]..':'..a[2])
							local sdae = q.list[1].dae
							if toreplace then
								q.list[j].dae = dae
							else
								table.remove(q.list,j)
								-- reset dae
								if #q.list >= j then
									q.list[j].dae = sdae
								end
							end
							break
						end
					end
					if not hit and (p.pos-q.pos):length() < small_dist then
						hit = true
						q.list[#q.list+1] = a
						table.sort(q.list, function(a, b)
							return a[1] < b[1]
						end)
						break
--                                    U.dump(q.list, '??++++++++++++++++++++++ to_ADD:')
					end
				end
--[[
				if false and (p.pos-q.pos):length() < small_dist then
					isext = true
--                        U.dump(p, '?? HIT_fr:')
--                        U.dump(q, '?? HIT_to:')
					local hit
					for i,a in pairs(p.list) do
						for j,b in pairs(q.list) do
							if a[1] == b[1] and a[2] == b[2] then
								hit = true
									lo('?? is_HIT:'..m..':'..n)
--                                    lo('??--------------- to_REM:'..a[1]..':'..a[2])
								local sdae = q.list[1].dae
								if toreplace then
									q.list[j].dae = dae
								else
									table.remove(q.list,j)
									-- reset dae
									if #q.list >= j then
										q.list[j].dae = sdae
									end
								end
								break
							end
						end
						if not hit then
							q.list[#q.list+1] = a
							table.sort(q.list, function(a, b)
								return a[1] < b[1]
							end)
--                                    U.dump(q.list, '??++++++++++++++++++++++ to_ADD:')
						end
					end
				end
]]
			end
			if not hit then
				desc.acorner_[#desc.acorner_+1] = p
			end
--            if not isext then
--                desc.acorner_[#desc.acorner_+1] = p
--            end
--                lo('?? if_EXT:'.._..':'..tostring(isext),true)
		end
		for k=#desc.acorner_,1,-1 do
			if #desc.acorner_[k].list == 0 then
				table.remove(desc.acorner_, k)
			end
		end
--                U.dump(desc.acorner_, '??****************************** post_REM:'..tostring(dae))
--                    desc.acorner_ = nil
--            return
	else
		desc.acorner_ = cornersBuild(desc)
	end
end


local function toggle(tp)
		lo('>> toggle:'..tp..':'..daePath[tp][1])
	if false then
	elseif tp == 'door' then
		doorToggle()
	elseif tp == 'corner' then
		local desc = adesc[cedit.mesh]
--            U.dump(desc.acorner, '?? toggle_corner:'..scope, true)
		out.curselect = 1
		W.cornerUp(desc)
--            U.dump(desc.acorner_, '?? CB:')
--            desc.acorner_ = nil
	elseif tp == 'pilaster' then
			indrag = false
--            U.dump(daePath[tp], '?? toggle_PIL:')
		forBuilding(adesc[cedit.mesh], function(w, ij)
--            lo('?? to_PIL:'..ij[1]..':'..ij[2])
			if not w.pilaster then
				w.pilaster = {
					dae = daePath[tp][1],
					yes = false,
					aind = {1},
				}
			end
			w.pilaster.yes = not w.pilaster.yes
			if not w.pilaster.yes then
				w.pilaster = nil
			end
		end)
	elseif tp == 'stringcourse' then
		forBuilding(adesc[cedit.mesh], function(w, ij)
			if not w.stringcourse then
				w.stringcourse = {}
			end
			w.stringcourse.yes = not w.stringcourse.yes
			if w.stringcourse.yes then
				if not w.stringcourse.dae then
					w.stringcourse.dae = daePath[tp][1]
				end
			else
				w.stringcourse = nil
			end
		end)
	elseif tp == 'storefront' then
			lo('?? toggle_storefront:'..tostring(cedit), true)
		forBuilding(adesc[cedit.mesh], function(w, ij)
			if not w.storefront then w.storefront = {adae = {}, yes = false} end
			w.storefront.yes = not w.storefront.yes
			if w.storefront.yes then w.storefront.adae = {} end
				lo('?? for_SF:'..tostring(w.storefront.yes)..':'..#w.storefront.adae)
			if w.storefront.yes and #w.storefront.adae == 0 then
				local ltot = 0
				local lwall = w.u:length()
				w.storefront.adir = {}
				if daePath[tp] then
					local list = U.perm(deepcopy(daePath[tp]))
						if U._PRD == 0 then list = deepcopy(daePath[tp]) end
					for i,pth in pairs(list) do
						local m = ddae[pth]
						if math.abs(lwall - ltot) < m.len/2 then
							-- scale
							break
						end
						w.storefront.adae[#w.storefront.adae+1] = pth
						ltot = ltot + m.len
					end
				end
				w.storefront.len = ltot
					lo('?? SF_LEN:'..w.storefront.len)
--[[
						local L = math.max(
							math.abs(m.to.x-m.fr.x),
							math.abs(m.to.y-m.fr.y),
							math.abs(m.to.z-m.fr.z))
						-- TODO: get long direction
						w.storefront.adir[#w.storefront.adir+1] = vec3(0,1,0)
]]
--                    U.dump(w.storefront.adae, '??++++++++++++++ for_PERM:'..lwall)
--                w.storefront.ind = w.doorind and w.doorind+1 or 1
--                U.dump(ddae[daePath[tp][1]], '?? for_dae:')
--                w.storefront.dae =
			end
		end)
	end
end
--[[
		local acorner = cornersBuild(desc, out.curselect)
			U.dump(acorner, '?? cornerToggle.acorner:')
		desc.acorner = acorner
				desc.acorner = {}
]]
--[[
		local acorner = cornersBuild_(desc)
			U.dump(acorner, '?? CB:')
		if desc.acorner_ then
--            U.dump(desc.acorner_, '?? to_SUBTRACT:', true)
			for _,p in pairs(acorner) do
				for _,q in pairs(desc.acorner_) do
					if (p.pos-q.pos):length() < small_dist then
--                        U.dump(p, '?? HIT_fr:')
--                        U.dump(q, '?? HIT_to:')
						local hit
						for i,a in pairs(p.list) do
							for j,b in pairs(q.list) do
								if a[1] == b[1] and a[2] == b[2] then
									hit = true
--                                    lo('??--------------- to_REM:'..a[1]..':'..a[2])
									local sdae = q.list[1].dae
									table.remove(q.list,j)
									-- reset dae
									if #q.list >= j then
										q.list[j].dae = sdae
									end
									break
								end
							end
							if not hit then
								q.list[#q.list+1] = a
								table.sort(q.list, function(a, b)
									return a[1] < b[1]
								end)
--                                    U.dump(q.list, '??++++++++++++++++++++++ to_ADD:')
							end
						end
					end
				end
			end
--                U.dump(desc.acorner_, '??****************************** post_REM:')
--                    desc.acorner_ = nil
--            return
		else
			desc.acorner_ = cornersBuild_(desc)
		end
]]


local function forScope()
	return scope,cij
end


local function forDesc()
	return adesc[cedit.mesh]
end


local function forDAE(tp)
--        U.dump(daePath['win'], '?? forDAE:'..tp)
--    return U.clone(ddae[tp])
	return U.clone(daePath[tp])
end


local function ifForest(atp, dbg)
	if out.inhole then
--                indrag = true
		local dae = out.inhole.achild[out.inhole.achild.cur or #out.inhole.achild].body
--            if atp then lo('?? ifForest.for_hole:'..tostring(dae)) end

--            lo('?? if_hole:'..tostring(atp)..':'..tostring(ddae[dae]))
--            if atp then indrag = true end
		if ddae[dae] and atp and #U.index(atp, ddae[dae].type) > 0 then
--            lo('?? ifForest.for_hole:')
			return true
		end
	end
	if cedit.forest then
			if W.ui.dbg and atp[1] == 'corner' then
				U.dump(atp, '?? ifForest:'..tostring(atp)..':'..tostring(cedit.forest)..':'..tostring(dforest[cedit.forest]))
			end
		if dbg then
			lo('?? iffor:'..tostring(dforest[cedit.forest]), true)
		end
--            lo('?? ififForest:'..tostring(dforest[cedit.forest].type)..':'..tostring(atp))
		if atp == nil then
			return true
		elseif dforest[cedit.forest] and #U.index(atp, dforest[cedit.forest].type) > 0 then
			return true
		else
			if out.inhole and out.inhole.achild.cur then
				local dae = out.inhole.achild[out.inhole.achild.cur].body
--                    lo('?? if_hole:'..tostring(atp)..':'..tostring(ddae[dae]))
				if ddae[dae] and ddae[dae].type == atp then
					return true
				end
			end
			return false
		end
	end
--    if atp == nil and cedit.forest ~= nil then return true end
--        if dbg then
--            lo('?? ifforest:'..tostring(cedit.forest)..':'..tostring(dforest[cedit.forest]), true) --..':'..tostring(dforest[cedit.forest].type)..':'..tostring(atp[1]))
--        end
--            if not atp or atp[1] == 'balcony' then
--                return out.inbalcony
--            end
	if atp and atp[1] == 'balcony' then
		return out.inbalcony
	end
--            if incontrol == 7 then
--                lo('?? iff:'..tostring(cedit.forest)..':'..tostring(dforest[cedit.forest]))
--            end
--    if cedit.forest ~= nil then return true end
	if cedit.forest == nil or dforest[cedit.forest] == nil then return false end
	if #U.index(atp, dforest[cedit.forest].type) > 0 then
--    if dforest[cedit.forest].type == tp then
		return true
	end
	return false
end


W.ifPairHit = function(ij)
	local floor = adesc[cedit.mesh].afloor[ij[1]]
	local base,map = U.polyStraighten(floor.base)
	local apair = T.pairsUp(base)
	for i,p in pairs(apair) do
		if ij[2] == p[1] then
			return p,1,map
		elseif ij[2] == p[2] then
			return p,2,map
		end
	end
end


W.ifPairEnd = function(ij,dbg)
--      U.dump(cij, '>> ifPairEnd:')
	local forhit
	if not ij then
		ij = U.clone(cij)
	else
		forhit = true
	end
    if dbg then U.dump(incorner, '>> ifPairEnd:'..tostring(forhit)..':'..tostring(incorner)..' cij:'..tostring(cij and cij[1] or nil)..':'..tostring(cij and cij[2] or nil), true) end
	if not ij then
      lo('!! ERR_ifPairEnd_NOIJ:',true)
    return false
  end

	if scope == 'wall' or incorner then -- or scope == 'side' then
--    if scope == 'wall' or (scope == 'floor' and incorner) then -- or scope == 'side' then
		local floor = adesc[cedit.mesh].afloor[ij[1]]
    local base,imap = U.polyStraighten(floor.base)
    for k,v in pairs(imap) do
      if v == ij[2] then
        ij[2] = k
        break
      end
    end
        if dbg then U.dump(imap, '?? iPE_base:'..cij[2]..'/'..ij[2]..' base_orig:'..#floor.base) end

		local function isFit(i, pair)
			if not i then return end
			return (U.mod(i-1,#base) == pair[1] and U.mod(i+1,#base) == pair[2])
        or (U.mod(i-1,#base) == pair[2] and U.mod(i+1,#base) == pair[1])
--			return (U.mod(i-1,#floor.base) == pair[1] and U.mod(i+1,#floor.base) == pair[2]) or (U.mod(i-1,#floor.base) == pair[2] and U.mod(i+1,#floor.base) == pair[1])
		end
		local function isHit(i,pair)
			return  i == pair[1] or i == pair[2]
		end

--                base = U.polyStraighten(base)
    local apair = T.pairsUp(base)
      if dbg then
        U.dump(apair, '?? ifPairEnd_PAIRS:'..tostring(forhit))
      end
--            local adata,apair = T.forGable(floor.base)
                if dbg then U.dump(apair,'?? if_pairs_end:'..tostring(forhit)..':'..ij[2]..':'..tostring(isHit(ij[2],apair[1]))) end
    if apair then
      for i,p in pairs(apair) do
        local fit
        if forhit then
          fit = isHit(ij[2],apair[i])
          if fit then
            return nil,apair[i],imap
          end
        elseif ij[2] then
          local ifit
          if isFit(ij[2],apair[i]) then
            ifit = ij[2]
          elseif incorner and isFit(U.mod(ij[2]-1,#base), apair[i]) then
            ifit = U.mod(ij[2]-1,#base)
          end
              if dbg then U.dump(apair[i],'?? if_FIT:'..ij[2]..':'..tostring(ifit)) end
          if ifit then
            if i == 1 then
              return ifit,apair[1],imap
            elseif i == #apair then
              return ifit,apair[#apair],imap
            end
          end
        end
      end
    end

	end
end
--[[
					local fit = forhit and isHit(ij[2],apair[i]) or isFit(ij[2],apair[i])
							if dbg then U.dump(apair[i],'?? for_FIT:'..tostring(fit)..':'..ij[2]) end
					if fit then
						if i == 1 then
							return (incorner and 1 or 2),apair[1]
						elseif i == #apair then
							return (incorner and 1 or 2),apair[#apair]
						else
							return nil,apair[i]
						end
					end
]]
--[[
				if isFit(cij[2],apair[1]) then
					return (incorner and 1 or 2),apair[1]
				elseif isFit(cij[2],apair[#apair]) then
	--                U.dump(apair, '?? isPairEnd:'..cij[1]..':'..tostring(cij[2]), true)
					return (incorner and 1 or 2),apair[#apair]
--                else
--                    return nil,apair
				end
]]


local function ifRoof(nm)

	if cij == nil or cij[1] == nil or cedit.mesh == nil then
		return false
	end
--        if true then return true end

--        lo('?? ifRoof:'..tostring(nm))
	local floor = adesc[cedit.mesh].afloor[cij[1]]
	if not floor then return false end

	if nm == 'ridge' then
		return forTop().isridge
	end
--        if true then return true end
--    local floor = adesc[cedit.mesh].afloor[#adesc[cedit.mesh].afloor]
	local base

	if ({flat=1,shed=1,pyramid=1})[nm] then -- nm == 'flat' or nm == 'shed' then
		if nm == 'shed' then
			if (floor.top.cchild and not floor.top.achild[floor.top.cchild].isconvex) or not floor.top.isconvex then
				return false
			end
		end
		return true
	end
			if W.ui.dbg then
				lo('??^^^^^^^^^^^^^^^^^^^^^^ child_BASE0:'..nm..':'..tostring(floor.top.cchild)..':'..#floor.top.achild)
--                W.ui.dbg = nil
			end

	if floor.top.cchild ~= nil and #floor.top.achild >= floor.top.cchild then
		base = floor.top.achild[floor.top.cchild].base
			if W.ui.dbg then
				U.dump(base, '??^^^^^^^^^^^^^^^^^^^^^^ child_BASE:')
				W.ui.dbg = nil
			end
	else
--        if cij[1] < #adesc[cedit.mesh].afloor then
--            -- no shapes for the whole lower floor
--            return false
--        end
		base = floor.base
	end
--[[
		if W.ui.dbg then --and nm ~= 'flat' then
			U.dump(base, '?? ifRoof_pyr:')
			W.ui.dbg = false
			return false
		end
		if W.ui.dbg == false then
			return false
		end
]]
	local arc = coverUp(base)
	if arc and #arc == 1 then
		base = arc[1]
	end

	if nm == 'pyramid' then
--[[
		if cedit.mesh == nil or adesc[cedit.mesh] == nil then
			lo('!! ERR_ifRoof:'..nm..':'..tostring(cedit.mesh))
			return false
		end
]]
		if base and (#base == 4 or U._PRD==0) then
			return true
		end
		return false
	elseif nm == 'shed' then
		if U._PRD == 0 then return true end
		if base and #base == 4 and isRect(base) then
			return true
		end
		return false
	elseif nm == 'gable' then
--        if _dbdrag then return end
--        _dbdrag = true
--        lo('??*** ifgable:'..tostring(isRect(base)))
--[[
		if not base then
			lo('!! no_BASE:')
		elseif #base ~= 4 and #base ~= 6 then
			lo('!! no_BASE2:'..#base)
		elseif not isRect(base) then
			lo('!! no_RECT:')
		end
		if not (base and (#base == 4 or #base == 6) and isRect(base)) then
			lo('!! noBB:'..#base..':'..tostring(isRect(base)))
		end
]]
		local desctop = forTop()
		if desctop.ismult then
			return false
		end
		if desctop.isridge then
			return true
		end
		if (base and ((#base == 4 or #base == 6 or #base == 8) and isRect(base))) or floor.top.poly == 'V' then
--        if U._PRD == 0 or (base and ((#base == 4 or #base == 6) and isRect(base))) or floor.top.poly == 'V' then
			return true
		end
--            if true then return true end
		return false
	end
end


local function ifTopRect()
		return true
--[[
	local desc = adesc[cedit.mesh]

	if not cij or not cij[1] then return end
	local floor = desc.afloor[cij[1] ]
--    local desctop = floor.top
	local base = floor.base
	if floor.top.cchild then
		base = floor.top.achild[floor.top.cchild].base
	end
	return #base == 4 and isRect(base)
]]
--[[
--        desctop = floor.top.achild[floor.top.cchild]
			if out.inridge then
				U.dump(desctop, '??___________________________________________ for_DESC:'..tostring(floor.top.cchild))
				out.inridge = false
		--        indrag = true
			end
]]
--        U.dump(floor.top, '>> ifTopRect:'..tostring(cij[1]))
end


local function ifValid(tp)
	if false then
	elseif tp == 'attic' then
		local desc = adesc[cedit.mesh]
		if desc == nil then return false end
		local yes = cij ~= nil and cij[1] == #desc.afloor and scope == 'wall'
		local floor = desc.afloor[#desc.afloor]
		if floor.top.cchild then
			return yes and ({shed=1,gable=1})[floor.top.achild[floor.top.cchild].shape] ~= nil
		end
		return yes and ({shed=1,gable=1})[floor.top.shape] ~= nil
	elseif tp == 'pilaster' and cij and cij[2] then
		if scope == 'wall' and not adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]].doorind then
			return true
		elseif scope == 'side' and W.ui.ifpilaster then return true end
	end
end


W.childRebase = function(achild, val, sbase, j, cbase, toemu)
	if not achild then return end
	local v = -U.perp(U.mod(j+1, sbase) - sbase[j]):normalized()
	local vb,ve = sbase[U.mod(j,#sbase)], sbase[U.mod(j+1,#sbase)]
	for k,c in pairs(achild) do
		-- for top children
		for _,ib in pairs({j,U.mod(j+1,#sbase)}) do
			-- if wall ends are in the child vertex mapping
--                U.dump(c, '?? tostring:'..ib)
			local aind = U.index(c.imap, ib)
			if #aind > 0 then
--                            U.dump(c, '?? for_child: floor:'..i..' wall:'..j..':'..#aind)
				for _,ic in pairs(aind) do
					local pchild = cbase[k][ic] --c.base[ic]
--                                lo('?? if_at:'..tostring(pchild)..':'..tostring(base[j])..':'..tostring(U.mod(j+1,base)))
--                        lo('?? if_at:'..tostring(pchild)..':'..tostring(vb)..':'..tostring(ve))
					-- if child vertex belongs to wall edge
					if math.abs((vb-pchild):length()+(ve-pchild):length()-(vb-ve):length()) < small_dist then
--                            if k == 2 then
--                                lo('?? move_c_base: child:'..k..' ind:'..ic..':'..ib..'/'..j..':'..tostring(pchild)..' v:'..tostring(v)..':'..val)
--                            end
--                            U.dump(cbase,'?? move_c_base: child:'..k..':'..ib..'/'..j..':'..ic..':'..tostring(pchild))
						if toemu then
							if not toemu[k] then toemu[k] = {} end
							if not toemu[k][ic] then toemu[k][ic] = vec3(0,0,0) end
							toemu[k][ic] = toemu[k][ic] + v*val
						else
							c.base[ic] = cbase[k][ic] + v*val
						end
					end
				end
			end
		end
	end
end


local function extrude(buf, val, sdata, rebased)
--local function extrude(buf, val, sbase, rebased)
	local asbase = sdata.abase
	local acbase = sdata.cbase
--        U.dump(buf, '>> extrude:')
		lo('>> extrude:'..tostring(rebased), true)
	if not sameSide(buf) then return end
--        lo('?? extrude:')
--    if tableSize(buf) > 1 or not sameSide(buf) then return end
--            U.dump(buf, '>> extrude:'..tostring(rebased))
--            U.dump(asbase, '?? asbase:')
--    local floor = adesc[cedit.mesh].afloor[cij[1]]
--    local base = floor.base
	local rebuild
	for i,row in pairs(buf) do
		local floor = adesc[cedit.mesh].afloor[i]
		local base = floor.base

		local aj = U.clone(row)
		table.sort(aj)
		local mi,ma = math.min(unpack(row)), math.max(unpack(row))

		for _,r in pairs(row) do
--            local v = -U.perp(U.mod(r+1, base) - base[r]):normalized()
		end

		local v = -U.perp(U.mod(row[1]+1, base) - base[row[1]]):normalized()

		if not rebased then
--                    lo('?? pre_INS:'..#floor.awall,true)
			aj[#aj + 1] = U.mod(ma+1,#base)
			local tbuf = {}
			tbuf[i] = aj
	--            U.dump(tbuf, '?? mm:'..mi..':'..ma)
			if sameSide(tbuf) then
				-- insert
					lo('?? to_ins_at_ma:'..ma,true)
				table.insert(floor.base, ma+1, U.mod(ma+1,floor.base))
				local w = deepcopy(floor.awall[ma])
				w.id = nil
				for dae,d in pairs(w.df) do
					w.df[dae] = {scale = d.scale}
				end
--                w.df = {}
				table.insert(floor.awall, ma+1, w)
				rebuild = true
			end
			table.remove(aj, #aj)
			table.insert(aj, 1, U.mod(mi-1,#base))
			tbuf[i] = aj
--                U.dump(tbuf, '?? mm2:'..mi..':'..ma)
			if sameSide(tbuf) then
				-- insert
				table.insert(floor.base, mi, floor.base[mi])
--                    U.dump(floor.base, '?? to_ins_at_mi:'..i..':'..mi)
				local w = deepcopy(floor.awall[mi])
				w.id = nil
				for dae,d in pairs(w.df) do
					w.df[dae] = {scale = d.scale}
				end
--                w.df = {}
				table.insert(floor.awall, mi, w)
				rebuild = true
				for j = 1,#row do
					row[j] = row[j] + 1
				end
				table.insert(asbase[i], mi, vec3(0,0,0))
			end
--                    lo('?? post_INS:'..#floor.awall,true)
--            U.dump(floor.base, '?? new_base:'..tostring(v))
--                U.dump(r, '?? newbuf:')
		end
--                U.dump(r, '?? extr_FLOOR:'..i..':'..tostring(val))
		-- move base
		for _,j in pairs(row) do
			-- walls to move
--                    lo('?? to_move:'..tostring(i)..':'..tostring(asbase[i]))
--                lo('?? to_move:'..j, true)
--                U.dump(asbase[i], '??_______________ bases:')
			base[U.mod(j,#base)] = asbase[i][U.mod(j,#asbase[i])] + v*val
			if #U.index(row,U.mod(j+1,#base)) == 0 then
				-- move wall end
				base[U.mod(j+1,#base)] = asbase[i][U.mod(j+1,#asbase[i])] + v*val
			end
			W.childRebase(floor.top.achild, val, asbase[i], j, acbase[i])
--[[
			-- wall edge
			local vb,ve = asbase[i][U.mod(j,#asbase[i])],asbase[i][U.mod(j+1,#asbase[i])]
			for k,c in pairs(floor.top.achild) do
				-- for top children
				for _,ib in pairs({j,U.mod(j+1,#base)}) do
--                    U.dump(c.imap, '?? in_map:'..ib..' j:'..j)
					-- if wall ends are in the child vertex mapping
					local aind = U.index(c.imap, ib)
					if #aind > 0 then
--                            U.dump(c, '?? for_child: floor:'..i..' wall:'..j..':'..#aind)
						for _,ic in pairs(aind) do
							local pchild = acbase[i][k][ic] --c.base[ic]
--                                lo('?? if_at:'..tostring(pchild)..':'..tostring(base[j])..':'..tostring(U.mod(j+1,base)))
								lo('?? if_at:'..tostring(pchild)..':'..tostring(vb)..':'..tostring(ve))
							-- if child vertex belongs to wall edge
							if math.abs((vb-pchild):length()+(ve-pchild):length()-(vb-ve):length()) < small_dist then
									U.dump(acbase[i][k],'?? move_c_base:'..ib..':'..ic..':'..tostring(pchild))
								c.base[ic] = acbase[i][k][ic] + v*val
							end
	--                        c.base[c.imap[ic] ] =
						end
					end
				end
--                U.dump()
			end
]]
		end
		baseOn(floor)
--            U.dump(floor.base, '?? after_SPLIT:')
	end
	return rebuild
end


--[[
				U.dump(out.ahole, '?? for_cur:')
			if false then
				local desc = adesc[cedit.mesh]
				local w = desc.afloor[cij[1] ].awall[cij[2] ]
				-- get mesh
				local m = aedit[w.id].mhole
	--                U.dump(m, '?? for_cur:'..cur)
				local av = {}
				for _,f in pairs(m.faces) do
					if #U.index(av, f.v) == 0 then
						av[#av+1] = f.v
					end
				end
				local un = w.u:normalized()
					U.dump(av, '?? h_IVERTS:'..tostring(w.u)..':'..tostring(dmesh[w.id]))
				for _,v in pairs(av) do
	--                    lo('?? for_v:'..tostring(m.verts[v+1]))
					m.verts[v+1] = m.verts[v+1] + un*val
				end
				local obj = scenetree.findObjectById(w.id)
				obj:createMesh({})
				obj:createMesh({{m}})
			end
]]

W.mat2xml = function(fname)
	local xml = M.xmlOn(fname)
--	local xml = M.xmlOn(editor.getLevelPath()..'bat/'..'exp_22762.dae')
		lo('?? for_node:'..tostring(xml))
--		for i,k in pairs(xml.kids) do
--			lo(k.name)
--		end
	local nd = M.forNode(xml, {'COLLADA', 'library_materials'})
	local basemat = 'm_plaster_worn_01_bat'
	if nd then
		local kid
		kid = M.toNode(nd, 'material', {id=basemat..'-material', name=basemat})
		kid = M.toNode(kid, 'instance_effect', {url=basemat..'-effect'})
	end

	nd = M.forNode(xml, {'COLLADA', 'library_geometries', 'geometry', 'mesh', 'triangles'})
	if nd then
		M.ofNode(nd, {material=basemat..'-material'})
	end

	nd = M.forNode(xml, {'COLLADA', 'library_visual_scenes', 'visual_scene', 'node', 'instance_geometry', 'bind_material', 'technique_common'})
	if nd then
		lo('?? for_GEO:'..#nd.kids)
		M.toNode(nd, 'instance_material', {symbol=basemat.."-material", target='#'..basemat..'-material'})
	end

	M.xml2file(xml, fname)
--	M.xml2file(xml, editor.getLevelPath()..'bat/'..'exp_22762_out.dae')
end


W.persist = function()
    local dirname = editor.getLevelPath()..'bat'
		lo('>> persist:'..tableSize(adesc)..'>'..dirname, true)
	scope = 'building'
	local outputFile = io.open(editor.getLevelPath()..'/main/MissionGroup/bat/items.level.json', "w")
	local list = {}
	for i,desc in pairs(adesc) do
		editor.clearObjectSelection()
		editor.selection.forestItem = {}
		local aid = {}
		local afi = {}
		--		lo('?? pre_build:'..tostring(desc)..':'..tostring(tableSize(editor.selection)))
--			lo('?? for_desc:'..tostring())
		aid[#aid+1] = desc.id
		editor.selectObjects(aid, editor.SelectMode_Add)
--		lo('?? daeExport:'..#aid..':'..#afi..':'..tostring(tableSize(editor.selection))) --..':'..tostring(editor.selection.forestItem))--..':'..tostring(aid[1])..':'..tostring(editor.selection.object.id)..':'..tostring(tableSize(editor.selection)))
--		scope = nil
		local fpath = dirname..'/exp_'..desc.id..'.dae'
		worldEditorCppApi.colladaExportSelection(fpath)
		W.mat2xml(fpath)
-- {"class":"TSStatic","persistentId":"0ee68748-c2ce-461e-805f-743500c4f401","__parent":"Statics","isRenderEnabled":true,"shapeName":"/levels/smallgrid/bat/exp_21550.dae"}
		list[#list+1] = {
			class='TSStatic',
			persistentId='0ee68748-c2ce-461e-805f-743500c4f'..i,
			__parent='bat',
			isRenderEnabled=true,
			shapeName=fpath,
		}
		if i%50 == 0 then
--			print('?? saved:'..i)
		end
		-- to prefab
		if false then
			local id = 0
			forBuilding(desc, function(w,ij)
			--		U.dump(w.df, '?? for_wall:'..ij[1]..':'..ij[2])
				for pth,list in pairs(w.df) do
					for _,id in pairs(list) do
						local item = dforest[id].item
						local pos = item:getPosition()
						local scale = item:getScale()
						local trans = item:getTransform()
						local rot = trans:toEuler() -- need Z
			--					lo('?? for_fi:'..tostring(item)..':'..tostring(pos)..':'..tostring(scale)..':'..tostring(rot))
			--                    U.dump(editor.matrixToTable(item:getTransform()), '?? for_fi:'..tostring(item)..':'..tostring(pos)..':'..tostring(scale)..':'..tostring(rot))
			--                    U.dump(trans,'?? for_fi:'..tostring(item)..':'..tostring(pos)..':'..tostring(scale))
						local oid,om = f2m(pth, id)
						if oid then
							groupEdit:add(om.obj)
							aid[#aid+1] = oid
						end
						om:setTransform(trans)
					end
				end
			end)
		end
	end
	local jdata = jsonEncode(list)
	jdata = jdata:gsub('},{', '}\r\n{')
	outputFile:write(string.sub(jdata, 2, #jdata-1))
		lo('?? jDATA:'..#list,true) --..':'..dirname..'/items.level.json'..':'..jdata, true)
	outputFile:close()
	-- missiongroup file
	outputFile = io.open(editor.getLevelPath()..'/main/MissionGroup/items.level.json', "r")
	local cnt = outputFile:read('*all')
	outputFile:close()
		lo('?? mgf:'..#cnt)
	outputFile = io.open(editor.getLevelPath()..'/main/MissionGroup/items.level.json', "w")
	outputFile:write(cnt..'\r\n'..'{"name":"bat","class":"SimGroup","__parent":"MissionGroup"}')
	outputFile:close()
	--{"name":"bat","class":"SimGroup","__parent":"MissionGroup"}

	scope = nil
		lo('<< persist:')
end


---------------------------
-- UI events
---------------------------
local function onVal(key, val)
--		lo('?? onVal:'..key..':'..tostring(val))
		if not incontrol then U.dump(cij, '?? W.onVal:'..tostring(key)..':'..tostring(val)) end --..':'..tostring(out.inseed)..':')
	local sval = W.ui[key]
	W.ui[key] = val

	if false then
	elseif key == 'junction' then
		W.ui.injunction = not W.ui.injunction
		return
	elseif ({b_road=0})[key] then
		return Ter.onVal(key, val)
	elseif ({sec_margin=0,sec_grid=0,sec_rand=0,sec_griddir1=0,sec_griddir2=0})[key] then
		return Ter.onVal(key,val)
	elseif ({b_conform=0, b_junction=0})[key] then
		return D.onVal(key, val)
	elseif ({conf_jwidth=0, conf_margin=0, conf_mslope=0, conf_bank=0, conf_all=0})[key] then
		D.onVal(key, val)
		incontrol = true
		return
	elseif ({exit_r=0, exit_w=0, junction_round=0, branch_n=0, round_r=0, round_w=0})[key] then
		D.onVal(key, val)
		incontrol = true
		return
  	elseif key == 'session_save' then
--			lo('?? ss:'..U._PRD)
		if U._PRD == 0 then
--			return W.persist()
		end
		local jdata = jsonEncode({jdata = adesc})
			lo('?? ss:'..#jdata..':'..tostring(editor.getLevelPath()))

		local dirname = editor.getLevelPath()..'bat'
	--    local dir = FS:openDirectory(dirname)
	--    local ifdir = FS:findFiles(dirname, "*", -1, true, false)
	--      lo('?? ifdir:'..tostring(ifdir))
	--    if not ifdir then
	--      FS:directoryCreate(dirname)
	--      lo('?? created:'..dirname)
	--    end
		editor_fileDialog.saveFile(
			function(data)
					lo('?? to_path:'..tostring(data.filepath))
				local fname = data.filepath -- editor.getLevelPath()..'bat.json'
				local outputFile = io.open(fname, "w")
				if outputFile then
					outputFile:write(jdata)
					outputFile:close()
				end
		--        worldEditorCppApi.colladaExportSelection(data.filepath)
		--        lo('?? editor_fileDialog:'..tostring(data.filepath))
		--        M.matReplace(data.filepath) --,'mat_dummy')
			end,
			{{"JSON file",".json"}},
			false,
			dirname,
		--      editor.getLevelPath(),
			"File already exists.\nDo you want to overwrite the file?"
		)

		if false then
			local fname = editor.getLevelPath()..'bat.json'
			local outputFile = io.open(fname, "w")
			if outputFile then
				outputFile:write(jdata)
				outputFile:close()
			end
		end
		return
	elseif key == 'dae_exp' then
		return W.persist()
	elseif key == 'base_load' then
		local dirname = editor.getLevelPath()..'bat'
		editor_fileDialog.openFile(
			function(fdata)
				local fname = fdata.filepath
				local data = jsonReadFile(fname)
					lo('?? from_paul:'..#data)
					indrag = true
		--		for i=1,10 do
		--		for i=1,600 do
				for i=1,#data-1 do
					local d = data[i]
					local base = {}
					local pos = vec3(d.coords[1][1],d.coords[1][2],0)
					for k=1,#d.coords-1 do
						local p = vec3(d.coords[k][1],d.coords[k][2],0) - pos
						base[#base+1] = p
					end
--						pos = vec3(0,0,0)
					pos.z = core_terrain.getTerrainHeight(U.proj2D(pos))
					buildingGen(pos, base)
					if i%50 == 0 then
--						print('?? loaded:'..i)
					end
				end
			end, {{"JSON file", ".json"}}, false, dirname)

	elseif key == 'session_load' then

		local dirname = editor.getLevelPath()..'bat'
		editor_fileDialog.openFile(
			function(data)
				local fname = data.filepath
				local file = io.open(fname, "r")
				if file then
					local jbody = file:read('*all')
					local jdesc = jsonDecode(jbody)
						lo('?? desc_loaded:'..tostring(tableSize(jdesc['jdata'])))
					file:close()
			--        if true then return end
					local istart = adesc and tableSize(adesc) or 0
					local aid = W.recover(jdesc['jdata'], function()
						lo('?? recovered:'..tostring(#jdesc))
			--        unrequire(apack[2])
			--        UI = rerequire(apack[2])
			--        editor.showWindow('BAT')
					end, true)
						U.dump(aid, '?? post_recover:'..tableSize(adesc)..':'..#aid) --istart)
--[[
					local aid = {}
					local i = 1
					for id,d in pairs(adesc) do
						if i > istart then
							d.id = nil
							aid[#aid+1] = id
						end
						i = i + 1
					end
]]
					for i,id in pairs(aid) do
							lo('??^^^^^^^^^^^^^^^^^^^ recover_house:'..id)
						adesc[id].id = nil
						adesc[id].idr = id
						W.houseUp(adesc[id])
						adesc[id] = nil
					end
					inrecover = nil

					return
				end
			end, {{"JSON file", ".json"}}, false, dirname)

		if false then
			local fname = editor.getLevelPath()..'bat.json'
			local file = io.open(fname, "r")
			if file then
				local jbody = file:read('*all')
				local jdesc = jsonDecode(jbody)
				lo('?? desc_saved:'..tostring(tableSize(jdesc['jdata'])))
				file:close()
		--        if true then return end
				W.recover(jdesc['jdata'], function()
				lo('?? recovered:'..tostring(#jdesc))
		--        unrequire(apack[2])
		--        UI = rerequire(apack[2])
		--        editor.showWindow('BAT')
				end)
				lo('?? post_recover:'..tableSize(adesc))
				local aid = {}
				for id,d in pairs(adesc) do
				d.id = nil
				aid[#aid+1] = id
				end
				for i,id in pairs(aid) do
					lo('??^^^^^^^^^^^^^^^^^^^ recover_house:'..id)
				W.houseUp(adesc[id])
				adesc[id] = nil
				end
				inrecover = nil
				return
			end
		end

	elseif key == 'nodes2roads' then
		N.toDecals()
		return
	elseif key == 'node_n' then
		N.circleSeed(val, vec3(x, y), W.ui.node_r)
		N.pathsUp()
		return
	elseif key == 'node_r' then
		N.circleSeed(W.ui.node_n, vec3(x, y), val)
		N.pathsUp()
		return
	elseif key == 'seed' then
		if val < 1 then
			val = 1
			W.ui[key] = val
		end
	elseif key == 'building_style' then
--        incontrol = true
		if val == 1 then
			W.ui.mat_wall = dstyle['residential'].mat_wall
		elseif val == 2 then
			W.ui.mat_wall = dstyle['industrial'].mat_wall
		else
			W.ui.mat_wall = dmat.wall
		end
		return
	elseif key == 'building_shape' then
		if val == sval then
			out.inseed = not out.inseed
			if not out.inseed then
				W.ui[key] = nil
			end
		else
			out.inseed = true
--      scope = nil
		end
		return
	elseif key == 'building_seed' then
		out.inseed = not out.inseed
			lo('?? in_seed:'..tostring(out.inseed))
--        W.ui[key] = out.inseed
--        incontrol = true
		return
	end

			if false and key == 'uv_u' then
		--        incontrol = true
				lo('?? for_TEST:'..tostring(W.out.testOM.id))
				local obj = scenetree.findObjectById(W.out.testOM.id)
				obj:createMesh({{}})
				for i=1,#W.out.testOM.data.uvs do
					local uv = W.out.testOM.data.uvs[i]
					uv.u = uv.u + 0.1
				end
				obj:createMesh({{W.out.testOM.data}})
			end

	if not cedit.mesh then
		lo('!! ERR_onVal_NOEDIT:')
		return
	end
	local dae
	local desc = adesc[cedit.mesh]
	if incontrol then
		indrag = true
	end


	if string.find(key, 'scope_') then
		local scp = U.split(key, '_', true)[2]
--        lo('?? for_SCOPE:'..scp, true)
		if scp then
			scopeOn(scp)
			return
		end
	end

	if false then
	elseif key == 'dae_export' then
		W.daeExport()
	elseif key == 'child_height' then
		local floor = desc.afloor[cij[1]]
		if not cedit.cval['DragVal'] then
			cedit.cval['DragVal'] = {h = floor.h}

			local ftgt = desc.afloor[cij[1]-1]
			if ftgt then
				ftgt.achild = {}
					lo('?? onVal.child_height:'..desc.afloor[cij[1]].top.cchild..':'..cij[1])
				local achild = desc.afloor[cij[1]].top.achild
				local cchild = achild[desc.afloor[cij[1]].top.cchild]
				W.floorClear(floor)
				-- top childs to floor childs
				for _,c in pairs(floor.top.achild) do
			-- new floor child
					local dnew = {afloor = {}, pos = vec3(0,0,0), prn = desc.id, floor = floor.ij[1]}
					local child = deepcopy(floor)
					child.base = deepcopy(c.base)
					dnew.afloor = {child}
	--            			U.dump(c,'?? c_BASE:'..cij[1])
					child.awall = {}
					for i,b in pairs(child.base) do
						child.awall[#child.awall+1] = deepcopy(floor.awall[c.imap[i]])
					end
					baseOn(child, nil, 1, dnew)

					child.top.df = {}
					child.top.ij[1] = 1
					child.top.achild = {}
					child.top.shape = 'flat'
	--                child.h = val
	--				baseOn(child)

					for _,w in pairs(child.awall) do
						for dae,r in pairs(w.df) do
							w.df[dae] = {scale = w.df[dae].scale}
						end
					end
					ftgt.achild[#ftgt.achild+1] = dnew
					if _ == desc.afloor[cij[1]].top.cchild then
						cedit.cval['DragVal'].tochild = _
					end
		--                    if _ == 1 then
		--                    end
				end
				for j,w in pairs(floor.awall) do
					objDown(w)
				end
		--                U.dump(floor.top,'?? if_TOP:'..tostring(floor.top.id))
				objDown(floor.top)
				W.floorClear(floor)
				table.remove(desc.afloor, cij[1])
	--            cij = {cij[1]-1,1}

				-- clean previous edit
				houseUp(adesc[cedit.mesh], cedit.mesh)
	--                markUp()
				aedit = {}
	--            lo('?? to_CHILD_id:'..tostring(ftgt.achild[cedit.cval['DragVal'].tochild].id))
				houseUp(nil, ftgt.achild[cedit.cval['DragVal'].tochild].id)
				cij = {1}
				out.inchild = true
			else
				lo('!! child_height_NO_TARGET:'..tostring(cij[1]))
			end
--                    lo('?? new_EDIT:'..tostring(cedit.mesh))
			return
		end
		if cedit.cval['DragVal'].tochild then
			adesc[cedit.mesh].afloor[cij[1]].h = cedit.cval['DragVal'].h + val
		end
--            floor.h = H
--[[
			floor.awall = {}
			floor.top = nil -- {ridge={}}
				U.dump(floor,'??_____ floor_CLEANED:')
			-- get common verts
			local amatch = {}
			out.avedit = {}
			for i,a in pairs(cchild.base) do
				for _,c in pairs(achild) do
					if _ ~= desc.afloor[cij[1] ].top.cchild then
						for j,b in pairs(c.base) do
							if (a-b):length() < small_dist then
								lo('?? match:'..i..':'..j..':'..tostring(a))
								out.avedit[#out.avedit+1] = a
								amatch[#amatch+1] = i
							end
						end
					end
				end
			end
]]
	elseif key == 'top_thick' then
		local desctop = forTop()
--            U.dump(desctop, '?? DT:')
		if desctop.ridge and desctop.ridge.on and #desctop.base ~= 4 then
			return
		end
		desctop.fat = val
--        desc.afloor[cij[1]].top.fat = val
	elseif key == 'uv_u' then
--        incontrol = true
--        lo('?? for_TEST:'..tostring(W.out.testID))
		if false and U._PRD == 0 then
			matMove(vec3(0,0,0), true)
		else
			matMove(vec3(val,0,0), true)
		end
--        return
	elseif key == 'uv_v' then
--        incontrol = true
		matMove(vec3(0,-val,0),true)
--        return
	elseif key == 'uv_scale' then
--        incontrol = true
		W.matScale({1/val,1/val})
--[[
		forBuilding(desc, function(w, ij)
			if not w.uvscale then
				w.uvscale = {1,1}
			end
			w.uvscale = {w.uvscale[1]*val,w.uvscale[2]*val}
		end)
]]
--        return
	elseif key == 'stairs_toggle' then
--            indrag = false
		local w = desc.afloor[cij[1]].awall[cij[2]]
		forBuilding(desc, function(w, ij)
			if w.doorind then
--                    lo('?? for_DOOR:'..ij[1]..':'..ij[2]..':'..w.doorind..':'..tostring(daePath['stairs']))
				if not w.doorstairs then
					w.doorstairs = {}
				end
				if w.doorstairs[w.doorind] then
					w.doorstairs[w.doorind] = nil
				else
					w.doorstairs[w.doorind] = {dae = daePath['stairs'][1], top = 0}
					local dae = ddae[w.doorstairs[w.doorind].dae]
--                        U.dump(dae, '?? stairs_toggle:', true)
					if ij[1] == 2 then
						-- adjust basement height
						desc.afloor[1].h = math.abs(dae.fr.z)
					end
				end
			end
		end)
	elseif key == 'basement_toggle' then
--            indrag = false
		desc.basement.yes = not desc.basement.yes
			lo('??______ for_basement:'..tostring(desc.basement.yes))
		if desc.basement.yes then
			local floorpre = desc.afloor[1]
	--            U.dump(desc.afloor[1].awall[1], '?? floor_PRE:'..tostring(desc.afloor[2].pos))
			local floor = {ij = {1}, base = deepcopy(desc.afloor[1].base), pos = floorpre.pos,
				h = W.ui.basement_height,
				top = deepcopy(floorpre.top), awall = {},
			}
			for j=1,#floor.base do
				floor.awall[#floor.awall+1] = {
					ij = {1, j},
					u = floorpre.awall[j].u,
					v = vec3(0,0,floor.h),
					df = {},
					uv = deepcopy(floorpre.awall[j].uv),
					mat = out.defmat -- 'm_greybox_base', -- 'm_stonewall_damaged_01',
				}
			end
			floor.top.shape = 'flat'
			floor.top.achild = nil
			floor.top.margin = 0
			floor.top.fat = 0
			for i=2,#desc.afloor do
				local f = desc.afloor[i]
				f.ij[1] = f.ij[1]+1
				for j,w in pairs(f.awall) do
					w.ij[1] = w.ij[1]+1
				end
			end
			table.insert(desc.afloor, 1, floor)
			cij[1] = 1
			if desc.acorner_ then
--                U.dump(desc.acorner_, '?? corners_shift:')
				for i,b in pairs(desc.acorner_) do
					for j,f in pairs(b.list) do
						f[1] = f[1] + 1
					end
				end
			end
		else
			if desc.acorner_ then
				for i,b in pairs(desc.acorner_) do
					for j,f in pairs(b.list) do
						f[1] = f[1] - 1
					end
				end
			end
      if desc.afloor[2] then
        for _,w in pairs(desc.afloor[2].awall) do
          w.doorstairs = nil
        end
      end
			W.floorDel(desc, 1)
		end
		houseUp(nil, cedit.mesh)
		if desc.basement.yes then
			scopeOn('floor')
		else
      cij[1] = 1
			scopeOn('building')
		end
		markUp()
		return
	elseif key == 'ridge_width' then
		local floor = desc.afloor[cij[1]]
		if #floor.base ~= 4 then
			return
		end
		floor.top.ridge.flat = val
	elseif key == 'ridge_flat' then
--            indrag = false
		local floor = desc.afloor[cij[1]]
--        if #floor.base
		if val then
			floor.top.ridge.flat = W.ui.ridge_width
		else
			floor.top.ridge.flat = nil
		end
			lo('?? pre_ridged:'..tostring(floor.top.id)..':'..#floor.top.achild..':'..tostring(floor.top.achild[1]))
--            U.dump(floor.top.achild[1], '?? pre_ridged:'..tostring(floor.top.id))
		if #floor.top.achild == 1 then
			local obj = scenetree.findObjectById(floor.top.achild[1].id)
			if obj then
				obj:delete()
			end
			floor.top.achild[1].id = nil
			T.forRidge(floor)
			lo('?? ridged:')
		else
			lo('!! ERR_ridge_flat:')
		end
--            U.dump(floor.top,'?? ridge_flat:')
	elseif key == 'stringcourse_bottom' then
		forBuilding(desc, function(w, ij)
			if w.stringcourse and w.stringcourse.yes then
				w.stringcourse.bot = val
				dae = w.stringcourse.dae
			end
		end)
--        lo('?? ifSC:'..tostring(ifForest({'stringcourse'})))
	elseif key == 'pilaster_ind0' then
		local desc = adesc[cedit.mesh]
		forBuilding(desc, function(w, ij)
			if w.pilaster then
--                lo('?? nwin:'..w.nwin..'/'..val)
				if val > w.nwin then
					val = w.nwin
					W.ui[key] = val
				elseif val == 0 then
					val = 1
					W.ui[key] = val
				end
				w.pilaster.aind[1] = val
				dae = w.pilaster.dae
			end
		end)
	elseif key == 'pilaster_ind1' then
--            indrag = false
--            lo('?? pi1:'..val)
		local desc = adesc[cedit.mesh]
		forBuilding(desc, function(w, ij)
			if w.pilaster then
				local ind = w.pilaster.aind[1] + val
				if ind > w.nwin then
					val = val - 1
					W.ui[key] = val
				elseif val == 0 and #w.pilaster.aind > 1 then
					table.remove(w.pilaster.aind, 2)
					W.ui.dbg = true
				else
					w.pilaster.aind[2] = ind
				end
				dae = w.pilaster.dae
--                    U.dump(w.pilaster.aind, '??___________ AIND2:')
			end
		end)
	elseif key == 'pilaster_ind2' then
--            indrag = false
--            lo('?? pi2:'..val)
		local desc = adesc[cedit.mesh]
		forBuilding(desc, function(w, ij)
			if w.pilaster and w.pilaster.aind[2] then
				local ind = w.pilaster.aind[2] + val
				if ind > w.nwin then
					val = val - 1
					W.ui[key] = val
				elseif val == 0 and #w.pilaster.aind > 2 then
					table.remove(w.pilaster.aind, 3)
				else
					w.pilaster.aind[3] = ind
				end
				dae = w.pilaster.dae
					U.dump(w.pilaster.aind, '??___________ AIND:'..ij[1]..':'..ij[2]..tostring(cedit.mesh)..':'..tostring(cij[2]))
			end
		end)
			W.ui.dbg = true
			lo('?? cij:'..tostring(cij[1])..':'..tostring(cij[2]))
	elseif key == 'mesh_apply' then
--        lo('?? MA:'..tostring(dforest[cedit.forest].type), true)
		meshApply(dforest[cedit.forest].type, val)
		return
	elseif key == 'corner_inset' then
			indrag = false
			lo('?? for_ci:'..tostring(val))
		return
	elseif key == 'floor_inout' then
--            indrag = false
--            lo('?? floor_inout:')
		local desc = adesc[cedit.mesh]
		local floor = desc.afloor[cij[1]]
		if not cedit.cval['DragPos'] or not cedit.cval['DragPos'].base then
			local cbase = {} -- children bases
			if floor.top.achild then
				for j,c in pairs(floor.top.achild) do
					cbase[#cbase+1] = U.clone(c.base)
				end
			end
			cedit.cval['DragPos'] = {base = deepcopy(desc.afloor[cij[1]].base), cbase = cbase}
--            return
		end
--            U.dump(cedit.cval['DragPos'].cbase, '?? child_BASES:')
		local dmove = {}
		for j=1,#floor.base do
			W.childRebase(floor.top.achild, val, cedit.cval['DragPos'].base, j, cedit.cval['DragPos'].cbase, dmove)
--            for k=1,cedit.cval['DragPos'].cbase
		end
--            U.dump(dmove, '?? to_move:')
		for ic,c in pairs(cedit.cval['DragPos'].cbase) do
			for ib,b in pairs(c) do
				floor.top.achild[ic].base[ib] = b + dmove[ic][ib]
			end
			if floor.awplus[ic] then
--                U.dump(floor.awplus[ic], '?? for_AW:')
				if floor.awplus[ic].id then
--                        lo('?? to_rem:'..floor.awplus[ic].id)
					scenetree.findObjectById(floor.awplus[ic].id):delete()
				end
--                floor.top.achild[ic].shape = 'flat'
--                floor.awplus[ic].dirty = true
			end
		end
		local mbase = U.polyMargin(cedit.cval['DragPos'].base, val)
		baseOn(desc.afloor[cij[1]], mbase)
	elseif key == 'ang_top' then
		-- FLIP TOP
		local dir = 1
		local floor = adesc[cedit.mesh].afloor[cij[1]]
--            lo('?? ANG_TOP:'..tostring(floor.top.cchild), true)
		local id
		if floor.top.cchild then
			if floor.awplus[floor.top.cchild] then
--                U.dump(floor.awplus[floor.top.cchild], '?? for_AWP:')
				floor.awplus[floor.top.cchild].dirty = true
			else
				lo('!! ERR_ang_top_NOCHILD:', true)
			end
			floor.top.achild[floor.top.cchild].istart = (floor.top.achild[floor.top.cchild].istart or 1) + 1
					lo('?? ang_top:'..floor.top.cchild..':'..floor.top.achild[floor.top.cchild].istart,true)
--            id = floor.awplus[floor.top.cchild].id
--            floor.awplus[floor.top.cchild].id = nil
		else
			if floor.awplus[0] then
				floor.awplus[0].dirty = true
			end
--            id = floor.awplus[0].id
--            floor.awplus[0].id = nil
			floor.top.istart = floor.top.istart + dir
		end
--        if id then
--            scenetree.findObjectById(id):delete()
--        end
	elseif key == 'roofborder_width' then
		local floor = desc.afloor[cij[1]]
		floor.top.border.width = val
	elseif key == 'roofborder_height' then
		local floor = desc.afloor[cij[1]]
		floor.top.border.height = val
	elseif key == 'roofborder_dy' then
		local floor = desc.afloor[cij[1]]
		floor.top.border.dy = val
	elseif key == 'roofborder_dz' then
		local floor = desc.afloor[cij[1]]
		floor.top.border.dz = val
	elseif key == 'roof_border' then
--            indrag = false
		local floor = desc.afloor[cij[1]]
--            lo('?? roof_border:'..cij[1])
--        local floor = desc.afloor[#desc.afloor]
		local desctop = forTop()
		if desctop then
			if not desctop.border then
				desctop.border = {
					width=W.ui.roofborder_width,
					height=W.ui.roofborder_height,
					dy=W.ui.roofborder_dy,
					dz=W.ui.roofborder_dz,
					shape={},
					mat = out.defmat -- 'm_greybox_base',
				}
			end
			desctop.border.yes = not desctop.border.yes
		end
--            lo('?? roof_border:'..tostring(floor.top.border.yes))
	elseif key == 'roof_ridge' then
--            indrag = false
--        out.inridge = not out.inridge --and false or true
--        local floor = desc.afloor[cij[1]]
--        local floor = desc.afloor[#desc.afloor]
--        floor.top.ridge.on = not floor.top.ridge.on
--        T.forRidge(floor)
--[[
		if floor.awplus and floor.awplus.id then
			-- clean up awplus
			local obj = scenetree.findObjectById(floor.awplus.id)
			if obj then obj:delete() end
			floor.awplus.list = {}
		end

		if not floor.top.ridge.on then
			floor.top.ridge.on = true
		else
--            floor.top.cchild = nil
			roofSet('flat')
			return
		end
]]
		local floor = desc.afloor[cij[1]]

		local desctop
		if #floor.top.achild == 0 then
			W.floorClear(floor)
			T.forRidge(floor)
		else
			if floor.top.cchild then
				lo('?? child_RIDGE:',true)
				desctop = forTop()
			elseif #floor.top.achild == 1 then
				desctop = floor.top.achild[1]
			end
			if not desctop.ridge then
				desctop.ridge = {on = false}
			end
			if desctop then
				desctop.ridge.on = true -- not desctop.ridge.on
				desctop.shape = 'ridge'
				floor.top.shape = 'ridge'
				W.floorClear(floor)
			end
		end
				lo('??____^^^^^^^ ridge_UP:'..#floor.top.achild,true)
--[[
		if U._PRD == 0 then
		else
			if floor.top.ridge.on then
				roofSet('flat')
				return
			else
				if floor.awplus then
					-- clean up awplus
					for _,aw in pairs(floor.awplus) do
						if aw.id then
							local obj = scenetree.findObjectById(aw.id)
							if obj then obj:delete() end
						end
					end
					floor.awplus = {}
				end
				floor.top.ridge.on = true
			end
		end
				if not desctop.ridge.on then
					roofSet('flat')
					return
				else
					W.floorClear(floor)
--                    T.forRidge(floor)
				end
]]
--[[
		local desctop = forTop()
		local istop = desctop.id == floor.top.id
			lo('?? if_floor:'..tostring(floor.top.id)..'/'..tostring(desctop.id))

		if not desctop.ridge then
			desctop.ridge = {on = false}
		end
		desctop.ridge.on = not desctop.ridge.on
		if not desctop.ridge.on then
			roofSet('flat')
			return
		else
			W.floorClear(floor)
		end
]]
--[[!!
		if floor.top.ridge.on then
			roofSet('flat')
			return
		else
			if floor.awplus then
				-- clean up awplus
				for _,aw in pairs(floor.awplus) do
					if aw.id then
						local obj = scenetree.findObjectById(aw.id)
						if obj then obj:delete() end
					end
				end
				floor.awplus = {}
			end
			floor.top.ridge.on = true
		end
]]
--[[
			local desctop = forTop()
			if not desctop.ridge then
				desctop.ridge = {on = false}
			end
				lo('?? if_RDG:'..tostring(floor.top.ridge.on)..':'..tostring(desctop.ridge.on))
			if not desctop.ridge.on then
				desctop.ridge.on = true
			else
				desctop.ridge.on = false
	--            floor.top.ridge.on = false
	--                U.dump(floor.top.achild,'?? if_RDG:'..tostring(floor.top.ridge.on)..':'..tostring(desctop.ridge.on))
				roofSet('flat')
				return
			end
		floor.top.ridge.on = not floor.top.ridge.on
--            lo('?? oV_ifridge:'..tostring(floor.top.ridge.on))
		if floor.top.ridge.on then
		else
				U.dump(floor.top, '?? if_MESH:'..tostring(floor.top.id), true)
			floor.top.achild = {}
			roofSet('flat')
			return
		end
]]
--            U.dump(desctop, '?? post_RIDGE:')
--        local av,auv,af = T.forRidge(floor.base)
--            lo('?? in_RIDGE:'..tostring(out.inridge)..':'..#av..':'..#af)
	elseif key == 'hole_pull' then
		local cw = out.inhole
		local b = cw.achild[cw.achild.cur or #cw.achild]
		b.z = val
	elseif key == 'hole_fitmesh' then
--            indrag = false
		local cw = out.inhole
		local b = cw.achild[cw.achild.cur or #cw.achild]
--                U.dump(b.base, '?? for_b:')
		if b.body then
			local doorH = ddae[b.body].to.z - ddae[b.body].fr.z
			local doorW = ddae[b.body].to.x - ddae[b.body].fr.x
--                lo('?? hole_fitmesh.WH:'..tostring(doorH)..':'..tostring(doorW))
--                            U.dump(b.base, '??______ for_hole_mesh:'.._..':'..b.body..':'..ang..':'..tostring(desc.pos))

			local w,h = U.rcWH(b.base)
			local scale = vec3(doorW/w, 1, doorH/h)
--                U.dump('?? scale:'..tostring(scale))
			b.base[1].y = b.base[2].y + h*scale.z
			b.base[4].y = b.base[3].y + h*scale.z
			local xmid = (b.base[1].x + b.base[4].x)/2
			b.base[1].x = xmid - doorW/2
			b.base[2].x = xmid - doorW/2
			b.base[3].x = xmid + doorW/2
			b.base[4].x = xmid + doorW/2
--                U.dump(b.base, '?? for_b2:')
			-- reset frame
			out.ahole = {}
			for i,p in pairs(b.base) do
				out.ahole[#out.ahole+1] = cw.pos + p.x*cw.u:normalized() + p.y*cw.v:normalized()
			end
			out.ahole[#out.ahole+1] = out.ahole[1]
		end
	elseif key == 'hole_x' then
--            indrag = false
		if out.inhole and out.inhole.achild then
--                U.dump(out.inhole.arc, '?? hole_x:'..cedit.mesh..':'..tostring(aedit[cedit.mesh]))
			local ahole = out.inhole.achild
			local cur = ahole.cur or #ahole
			if not cedit.cval['DragPos'] or not cedit.cval['DragPos'].hole then
				cedit.cval['DragPos'] = {hole = deepcopy(ahole[cur].base), frame = deepcopy(out.ahole)}
				incontrol = true
			end
			cedit.cval['DragPos'].shole = deepcopy(ahole[cur].base)
			cedit.cval['DragPos'].sframe = deepcopy(out.ahole)
			local w = out.inhole
			local base,frame = cedit.cval['DragPos'].hole,cedit.cval['DragPos'].frame
--                    local irc = U.inRC(base[1] + val*vec3(1,0,0), {out.inhole.arc[6]})
--                    U.dump(out.inhole.arc[6], '?? if_RC:'..tostring(irc)..':'..tostring(base[1]))
--                    U.dump(out.inhole.arc[6], '?? if_RC:'..tostring(irc)..':'..tostring(base[1]), true)
--                        lo('?? if_in:'..i..':'..tostring(ind)..':'..#out.inhole.arc)
--                        lo('?? rollback:'..U.inRC(base[i], out.inhole.arc), true)
			-- check if inside
			local rollback
			for i = 1,#base do
				local ind = U.inRC(base[i] + val*vec3(1,0,0), out.inhole.arc)
				if ind and ind ~= #out.inhole.arc - #out.inhole.achild + cur then
					rollback = true
				end
				local rc = {
					vec3(0,0,0),
					vec3(w.u:length(),0,0),
					vec3(w.u:length(),w.v:length(),0),
					vec3(0,w.v:length(),0)}
				if not U.inRC(base[i] + val*vec3(1,0,0), {rc}) then
					rollback = true
				end
				if rollback then
					ahole[cur].base = cedit.cval['DragPos'].shole
					out.ahole = cedit.cval['DragPos'].sframe
					return
				end
			end

			for i = 1,#base do
				ahole[cur].base[i] = base[i] + val*vec3(1,0,0)
			end
--            local w = adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]]
			local un = w.u:normalized()
			for i = 1,#out.ahole do
				out.ahole[i] = frame[i] + un*val
			end
		end
	elseif key == 'hole_y' then
		if out.inhole and out.inhole.achild then
			local ahole = out.inhole.achild
			local cur = ahole.cur or #ahole
			if not cedit.cval['DragPos'] or not cedit.cval['DragPos'].hole then
				cedit.cval['DragPos'] = {hole = deepcopy(ahole[cur].base), frame = deepcopy(out.ahole)}
				incontrol = true
			end
			cedit.cval['DragPos'].shole = deepcopy(ahole[cur].base)
			cedit.cval['DragPos'].sframe = deepcopy(out.ahole)
			local w = out.inhole
			local base,frame = cedit.cval['DragPos'].hole,cedit.cval['DragPos'].frame
			-- check if inside
			local rollback
--                    U.dump(base, '?? if_IN:'..val)
			for i = 1,#base do
				local ind = U.inRC(base[i] + val*vec3(0,1,0), out.inhole.arc)
				if ind and ind ~= #out.inhole.arc - #out.inhole.achild + cur then
					rollback = true
				end
				local rc = {
					vec3(0,0,0),
					vec3(w.u:length(),0,0),
					vec3(w.u:length(),w.v:length(),0),
					vec3(0,w.v:length(),0)}
				local p = base[i] + val*vec3(0,1,0)
				if not U.inRC(base[i] + val*vec3(0,1,0), {rc}) then
					rollback = true
				end
				if rollback then
					ahole[cur].base = cedit.cval['DragPos'].shole
					out.ahole = cedit.cval['DragPos'].sframe
					return
				end
			end

			for i = 1,#base do
				ahole[cur].base[i] = base[i] + val*vec3(0,1,0)
			end
--            local w = adesc[cedit.mesh].afloor[cij[1]].awall[cij[2]]
			local vn = w.v:normalized()
			for i = 1,#out.ahole do
				out.ahole[i] = frame[i] + vn*val
			end
		end
	elseif key == 'zpos' then
		desc.pos.z = val
	elseif key == 'xpos' then
		desc.pos.x = val
	elseif key == 'ypos' then
		desc.pos.y = val
	elseif ({roof_flat=1,roof_pyramid=1,roof_shed=1,roof_gable=1})[key] then
--        indrag = false
		local floor = desc.afloor[#desc.afloor]
--                lo('?? flat_PRE:'..#floor.top.achild)
		roofSet(U.split(key, '_',true)[2])
--                lo('?? flat_POST:'..#floor.top.achild)
		incontrol = true
		return
	elseif key == 'pilaster_toggle' then
		toggle('pilaster')
	elseif key == 'pillar_spany' then
		pillarSpan(val)
	elseif key == 'pillar_toggle' then
		pillarToggle()
	elseif key == 'win_toggle' then
		windowsToggle()
	elseif key == 'door_toggle' then
		toggle('door')
	elseif key == 'stringcourse_toggle' then
		toggle('stringcourse')
	elseif key == 'storefront_toggle' then
		toggle('storefront')
	elseif key == 'balc_bottom' then
		forBuilding(desc, function(w, ij)
			w.balcony.bottom = val + 0.1 < desc.afloor[ij[1]].h and val or (desc.afloor[ij[1]].h-0.1)
		end)
	elseif key == 'balc_scale_width' then
		forBuilding(desc, function(w, ij)
			w.df[w.balcony.dae].scale = vec3(val,1,1)
		end)
	elseif key == 'balc_ind0' then
		if val < 1 then
			val = 1
			W.ui[key] = val
		end
		forBuilding(desc, function(w, ij)
			w.balcony.ind[1] = val
		end)
	elseif key == 'balc_ind1' then
		if val < 1 then
			val = 1
			W.ui[key] = val
		end
		forBuilding(desc, function(w, ij)
			w.balcony.ind[2] = val
		end)
	elseif key == 'balc_ind2' then
		if val < 0 then
			val = 0
			W.ui[key] = val
		end
		forBuilding(desc, function(w, ij)
			w.balcony.ind[3] = val
		end)
	elseif key == 'balc_toggle' then
		if not balcToggle() then return end
	elseif key == 'corner_toggle' then
		toggle('corner')
--        return
--        cornerToggle()
	elseif key == 'n_floors' then
		if val < 1 then
			W.ui[key] = 1
			return
		end
    if val == 1 and desc.basement and desc.basement.yes then
      val = 2
      W.ui[key] = val
      return
    end
		if val > #desc.afloor then
			for i =1,val-#desc.afloor do
				floorAdd()
			end
		else
--                lo('?? to_DEL:',true)
			local top = deepcopy(desc.afloor[#desc.afloor].top)
			for i = 1,#desc.afloor-val do
				local floor = desc.afloor[#desc.afloor]
				for _,w in pairs(floor.awall) do
					forestClean(w)
					scenetree.findObjectById(w.id):delete()
				end
				forestClean(desc.afloor[#desc.afloor].top)
				scenetree.findObjectById(floor.top.id):delete()

				table.remove(desc.afloor, #desc.afloor)
			end
			desc.afloor[#desc.afloor].top = top
			cij[1] = #desc.afloor
--[[
			house.afloor[#house.afloor] = nil
			local floor = house.afloor[#house.afloor]
			local mbase = U.polyMargin(floor.base, floor.top.margin)
			floor.top.body = coverUp(mbase)
]]
		end
		uvOn()
		houseUp(nil, cedit.mesh)
			U.dump(cij,'?? for_nfl:'..#desc.afloor)
	elseif key == 'building_ang' or key == 'ang_floor' then
		if not cedit.cval['DragVal'] then
				lo('??******************** drag_ROT:', true)
			cedit.cval['DragVal'] = {}
			for i,f in pairs(desc.afloor) do
				cedit.cval['DragVal'][#cedit.cval['DragVal'] + 1] = {base = U.clone(f.base), achild = {}}
				local achild = cedit.cval['DragVal'][#cedit.cval['DragVal']].achild
				for i,c in pairs(f.top.achild) do
--                    cedit.cval['DragVal'][#cedit.cval['DragVal']].achild[cedit.cval['DragVal'][#cedit.cval['DragVal']].achild+1] = U.clone(c.base)
					achild[#achild+1] = U.clone(c.base)
				end

				if f.achild then
					local fchild = {}
					for i,d in pairs(f.achild) do
						fchild[i] = {}
						for j,fc in pairs(d.afloor) do
							fchild[i][#fchild[i]+1] = U.clone(fc.base)
--                            fchild[#fchild+1] = U.clone(fc.base)
						end
					end
					cedit.cval['DragVal'][#cedit.cval['DragVal']].fchild = fchild
				end
			end
		end
		local aij = {}
		if scope == 'floor' then aij = {cij[1]} end
		local center = U.polyCenter(cedit.cval['DragVal'][1].base)
		local cfloor
		forBuilding(desc, function(w, ij)
--                lo('?? for_floor:'..ij[1]..':'..tostring(desc.afloor[ij[1]].top.achild), true)
			if cfloor == ij[1] then return end
			cfloor = ij[1]
			local floor = desc.afloor[ij[1]]
			local base = cedit.cval['DragVal'][ij[1]].base
			for j,_ in pairs(base) do
				desc.afloor[ij[1]].base[j] = center + U.vturn(base[j]-center, 2*math.pi*val/360)
			end
--                lo('?? post_FLOOR:'..ij[1]..':'..ij[2]..':',true)
			if floor.top.achild then
--            if ij[1] == #desc.afloor and floor.top.achild then
				for k,c in pairs(floor.top.achild) do
--                    U.dump(c, '?? for_child:'..k)
--                        U.dump(basechild, '?? building_ang.for_child:'..k)
--                    base = c.base
--                        lo('?? if_RIDGE:'..tostring(c.ridge))
--                        if c.ridge then
--                            lo('?? if_RIDGE2:'..tostring(c.ridge.on))
--                        end
					local basechild = cedit.cval['DragVal'][ij[1]].achild[k]
					if basechild then
						for j=1,#c.base do
							c.base[j] = center + U.vturn(basechild[j]-center, 2*math.pi*val/360)
						end
					end
--                        U.dump(c.base, '?? child_ROT:'..k)
--[[
					if c.ridge and c.ridge.on then
						lo('?? ridge_ROT:')
					else
						local basechild = cedit.cval['DragVal'][ij[1] ].achild[k]
						if basechild then
							for j=1,#c.base do
								c.base[j] = center + U.vturn(basechild[j]-center, 2*math.pi*val/360)
							end
						end
					end
]]
				end
			end
			if floor.achild then
				for i,d in pairs(floor.achild) do
					local basechild = cedit.cval['DragVal'][ij[1]].fchild[i]
					for j,f in pairs(d.afloor) do
--                            U.dump(basechild, '?? fo_fch:'..i..':'..j)
--                        local basechild = cedit.cval['DragVal'][ij[1]].achild[k]
						for k=1,#f.base do
							f.base[k] = center + U.vturn(basechild[j][k]-center, 2*math.pi*val/360)
						end
						baseOn(f)
					end
				end
			end
			baseOn(desc.afloor[ij[1]])
		end, aij)
		if desc.afloor[cij[1]].top.shape == 'gable' then
			roofSet('gable')  -- calls houseUp
		end
--            _dbdrag = true
--                U.dump(desc.afloor[2].top, '?? post_rs:')
--[[
			local u = (base[2]-base[1]):normalized()
			for j,_ in pairs(base) do
				desc.afloor[ij[1] ].base[j] = base[j] + u*(base[j]-center):dot(u)*(val-1)
--                base[j] = center + (base[j]-center)*val
			end
]]
	elseif key == 'building_scalex' then
		if not cedit.cval['DragVal'] then
			cedit.cval['DragVal'] = {}
			for i,f in pairs(desc.afloor) do
				cedit.cval['DragVal'][#cedit.cval['DragVal'] + 1] = U.clone(f.base)
			end
		end
		local center = U.polyCenter(cedit.cval['DragVal'][1])
		forBuilding(desc, function(w, ij)
--                lo('?? for_floor:'..ij[1], true)
			local base = cedit.cval['DragVal'][ij[1]] -- desc.afloor[ij[1]].base
			local u = (base[2]-base[1]):normalized()
			for j,_ in pairs(base) do
				desc.afloor[ij[1]].base[j] = base[j] + u*(base[j]-center):dot(u)*(val-1)
--                base[j] = center + (base[j]-center)*val
			end
			baseOn(desc.afloor[ij[1]])
		end, {})
	elseif key == 'building_scaley' then
		if not cedit.cval['DragVal'] then
			cedit.cval['DragVal'] = {}
			for i,f in pairs(desc.afloor) do
				cedit.cval['DragVal'][#cedit.cval['DragVal'] + 1] = U.clone(f.base)
			end
		end
		local center = U.polyCenter(cedit.cval['DragVal'][1])
		forBuilding(desc, function(w, ij)
--                lo('?? for_floor:'..ij[1], true)
			local base = cedit.cval['DragVal'][ij[1]] -- desc.afloor[ij[1]].base
			local u = U.perp(base[2]-base[1]):normalized()
			for j,_ in pairs(base) do
				desc.afloor[ij[1]].base[j] = base[j] + u*(base[j]-center):dot(u)*(val-1)
--                base[j] = center + (base[j]-center)*val
			end
			baseOn(desc.afloor[ij[1]])
		end, {})
--[[
	elseif key == 'ang_floor' then
		if not cedit.cval['DragVal'] then
				lo('??******************** drag_ROT:', true)
			cedit.cval['DragVal'] = {}
			for i,f in pairs(desc.afloor) do
				cedit.cval['DragVal'][#cedit.cval['DragVal'] + 1] = U.clone(f.base)
			end
		end
		local center = U.polyCenter(cedit.cval['DragVal'][1])
		forBuilding(desc, function(w, ij)
				lo('?? for_floor:'..ij[1]..':'..tostring(desc.afloor[ij[1] ].top.achild), true)
				local floor = desc.afloor[ij[1] ]
				local base = cedit.cval['DragVal'][ij[1] ]
				for j,_ in pairs(base) do
					desc.afloor[ij[1] ].base[j] = center + U.vturn(base[j]-center, 2*math.pi*val/360)
				end
				if ij[1] == #desc.afloor and floor.top.achild then
					for k,c in pairs(floor.top.achild) do
						U.dump(c, '?? for_child:'..k)
					end
				end
				baseOn(desc.afloor[ij[1] ])
			end, {cij[1]})
			if desc.afloor[cij[1] ].top.shape == 'gable' then
				roofSet('gable')
			end
]]
	elseif key == 'floor_clone' then
		floorAddMiddle()
		houseUp(nil, cedit.mesh)
--        indrag = false
--        incontrol = false
--        return
	elseif key == 'height_floor' then

		desc.afloor[cij[1]].h = val
		baseOn(desc.afloor[cij[1]])

--    elseif key == 'flip_left' then
--    elseif key == 'flip_right' then
	elseif key == 'fringe_inout' then
		local ij = scope == 'floor' and {cij[1]} or cij
			U.dump(ij, '?? fringe_io:'..tostring(scope))
		forBuilding(desc, function(w, ij)
			if not desc.afloor[ij[1]].fringe then
				desc.afloor[ij[1]].fringe = {}
			end
			desc.afloor[ij[1]].fringe.inout = val
--            w.fringe = {h = val, inout = 0, margin = 0, mat = 'WarningMaterial', mesh = nil}
--            w.fringe.inout = val
		end, ij)
--            return
	elseif key == 'fringe_height' then
--        U.dump(out.aedge, '?? fringe:')
		forBuilding(desc, function(w, ij)
			if not w.fringe then
				w.fringe = {h = 0, inout = 0, margin = 0, mat = 'WarningMaterial', mesh = nil}
			end
			w.fringe.h = val
		end)
	elseif key == 'flip_axis' then
--            lo('?? flip_ax:'..key..':'..tostring(sval)..':'..val, true)
		if val > sval then
			W.ui[key] = sval + 90
		else
			W.ui[key] = sval - 90
		end
	elseif key == 'wall_pull' then
		if not incontrol then
			if not desc.selection then
				desc.selection = {}
				-- to buffer
				select(desc)
			end
--                U.dump(desc.selection, '?? for_DSEL:')
			local abase = {}
			local cbase = {} -- children bases
			for f,_ in pairs(desc.selection) do
				abase[f] = U.clone(desc.afloor[f].base)
				local childhit
				if desc.afloor[f].top.achild then
					cbase[f] = {}
					for j,c in pairs(desc.afloor[f].top.achild) do
						-- check consistency with top children
--                            U.dump(c.base, '??^^^^^^^^^^^ for_child:'..j..':'..f)
--                            U.dump(abase[f], '??^^^^^^^^^^^ for_fbase:'..j..':'..f)
						for a,isel in pairs(desc.selection[f]) do
							if c.base then
								for b,ic in pairs(c.base) do
										lo('?? if_cross:'..a..':'..isel..':'..b..':'..tostring(abase[f][isel]))
									local e1,e2 = c.base[b],U.mod(b+1, c.base)
									local m = abase[f][isel]
									if (e1-m):length() + (e2-m):length() - (e1-e2):length()<small_dist then
	--                                    lo('?? hit_child:',true)
										if (e1-m):length() > U.small_val and (e2-m):length() > U.small_val then
											childhit = j
										end
									end
									m = U.mod(isel+1,abase[f])
									if (e1-m):length() + (e2-m):length() - (e1-e2):length()<small_dist then
	--                                    lo('?? hit_child:',true)
										if (e1-m):length() > U.small_val and (e2-m):length() > U.small_val then
											childhit = j
										end
									end
								end
							end
						end
						cbase[f][#cbase[f]+1] = U.clone(c.base)
					end
				end
				if childhit then
--                        lo('?? to_FLATTEN:'..f)
					local floor = desc.afloor[f]
					floor.top.shape = 'flat'
					for _,c in pairs(floor.top.achild) do
						scenetree.findObjectById(c.id):delete()
					end
					floor.top.achild = nil
					floor.top.cchild = nil
					W.floorClear(floor)
				end
			end
			cedit.cval['DragExt'] = {abase = abase, cbase = cbase, awall = {}} --, awall = desc.afloor[cij[1]].awall}

--                cedit.cval['DragExt'] = {base = U.clone(desc.afloor[cij[1]].base), awall = desc.afloor[cij[1]].awall}
		end
--                incontrol = true
--                if true then return end
--            U.dump(desc.selection, '?? wp:'..cij[1])
		if desc.afloor[cij[1]].awplus then
--                U.dump(desc.afloor[cij[1]].top.achild, '?? AW_children:')
			for _,wp in pairs(desc.afloor[cij[1]].awplus) do
				wp.dirty = true
			end
--                U.dump(desc.afloor[cij[1]].awplus, '?? AW_clean:')
		end
--            [floor.top.cchild].dirty = true
--            U.dump(desc.selection,'?? to_pull2:'..tostring(desc.afloor[cij[1]].top.shape),true)
--            U.dump(desc.selection, '?? to_extr:'..val)
--            indrag = false
		if extrude(desc.selection, val, cedit.cval['DragExt'], incontrol) then
				lo('?? exted:')
--            if extrude(desc.selection, val, cedit.cval['DragExt'].base, incontrol) then
			-- clear previous
			for j,w in pairs(cedit.cval['DragExt'].awall) do
				if w.id then
					scenetree.findObjectById(w.id):delete()
				end
--                lo('?? to_del:'..j..':'..tostring(w.id), true)
--                scenetree.findObjectById(w.id):delete()
			end
				lo('?? REBUILD:',true)
--!!                scenetree.findObjectById(desc.afloor[cij[1]].top.id):delete()
--            roofSet('gable')
			houseUp(nil, cedit.mesh)    -- to rebuild
		end
		if desc.afloor[cij[1]].top.shape == 'gable' then
			roofSet('gable')
		end
--        if desc.afloor[cij[1]].top.shape == 'shed' then
--            roofSet('shed')
--        end
--            indrag = true
--        return
	elseif key == 'wall_spanx' then
			U.dump(cij, '?? spanX:')
		--TODO:
		local w = desc.afloor[cij[1]].awall[cij[2]]
		w.spanx = val
	elseif key == 'wall_spany' then
		val = forSpanY(desc, cij, val)
		W.ui[key] = val
	elseif key == 'win_fitx' then
		forBuilding(desc, function(w, ij)
			if cedit.fscope == 1 then
				lo('?? single_WIN:',true)
				if w.df[w.win].fitx == 1 then
					w.df[w.win].fitx = nil
				else
					w.df[w.win].fitx = 1
				end
			end
		end)
--            indrag = false
--            U.dump(desc.afloor[2].awall[3].df, '?? win_fitx:')
	elseif key == 'win_bottom' then
		forBuilding(desc, function(w, ij)
			local winHeight = ddae[w.win].to.z - ddae[w.win].fr.z
--                lo('??_______________ win_bottom:'..val..':'..winHeight..'/'..desc.afloor[ij[1]].h, true)
--                U.dump(ddae[w.win], '??_______________ win_bottom:'..val..':'..winHeigth..':'..desc.afloor[ij[1]].h)
--[[!!
			if val + winHeight > desc.afloor[ij[1] ].h then
				val = desc.afloor[ij[1] ].h - winHeight
				W.ui[key] = val
			end
]]
      if cedit.fscope == 1 then
--          U.dump(w['win'..'_inf'], '?? for_winf:'..tostring(cedit.forest)..':'..ij[1]..':'..ij[2])
--          U.dump(w.df, '?? for_winfDF:'..tostring(cedit.forest))
--            lo('?? if_IND:'..dforest[cedit.forest].ind,true)
        if not w['win'..'_inf'] then
          w['win'..'_inf'] = {}
        end
        if not w['win'..'_inf'].dwinbot then
          w['win'..'_inf'].dwinbot = {}
        end
        if dforest[cedit.forest] then
          w['win'..'_inf'].dwinbot[dforest[cedit.forest].ind] = val
        end
--[[
        if w['win'..'_inf'] then
          for i,dae in pairs(w['win'..'_inf'].ddae) do
            for j,k in pairs(w.df[dae]) do
--                U.dump(dforest[k], '?? in_dforest:'..j)
              if k == cedit.forest then
--              if dforest[k].item:getKey() == cedit.forest then
--                lo('?? MOVE:')
                if not w['win'..'_inf'].dwinbot then
                  w['win'..'_inf'].dwinbot = {}
                end
                w['win'..'_inf'].dwinbot[dforest[k].ind] = val
              end
            end
          end
        end
]]
--            U.dump(w['win'..'_inf'], '?? post_set:')
      else
	    	w.winbot = val
      end
		end)
	elseif key == 'win_space' then
--            val = 2.5
--            W.ui[key] = val
--[[
		local w = desc.afloor[cij[1] ].awall[cij[2] ]
		local winW = (ddae[w.win].to.x - ddae[w.win].fr.x)*(w.df[w.win] and w.df[w.win].scale or 1)
		local aval = w.u:length() - 2*w.winleft
			lo('??^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ for_WSpace:'..aval..':'..(winW+val))
		if aval <= (winW + val) then
			W.ui[key] = sval
			return
		end
]]
--        local nwin = aval % desc.winspace
--        if nwin < 1 then
--        local start = desc.winleft + (aval % desc.winspace)/2

		local lim
		forBuilding(desc, function(w, ij)
--            if lim then return end
			local winW = (ddae[w.win].to.x - ddae[w.win].fr.x)*(w.df[w.win] and w.df[w.win].scale or 1)
				lo('?? if_LEFT:'..tostring(w.winleft))
			local aval = w.u:length() - 2*(w.winleft or 0)
--                lo('??^^^^^^^^^^^^^^ for_WSpace:'..val..':'..ij[1]..':'..ij[2]..':'..aval..':'..(winW+val))
			if aval <= (winW + val) then
				lim = true
				return
			end
			w.winspace = val
		end)
		if lim then
			W.ui[key] = sval
		end
	elseif key == 'win_left' then
--[[
		local w = desc.afloor[cij[1] ].awall[cij[2] ]
		local winW = (ddae[w.win].to.x - ddae[w.win].fr.x)*(w.df[w.win] and w.df[w.win].scale or 1)
		local aval = w.u:length() - 2*val
			lo('??^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ for_WLeft:'..aval..':'..(winW+w.winspace))
		if aval <= (winW + w.winspace) then
			W.ui[key] = sval
			return
		end
]]
		local lim
		forBuilding(desc, function(w, ij)
--            if lim then return end
			local winW = (ddae[w.win].to.x - ddae[w.win].fr.x)*(w.df[w.win] and w.df[w.win].scale or 1)
			local aval = w.u:length() - 2*val
--                lo('??^^^^^^^^^^^^^^ for_WLeft:'..ij[1]..':'..ij[2]..':'..aval..':'..(winW+w.winspace))
			if aval <= (winW + (w.winspace or 0)) then
--                W.ui[key] = sval
				lim = true
				return
			end
			w.winleft = val
		end)
		if lim then
			W.ui[key] = sval
		end
	elseif key == 'win_scale' then
			U.dump(cij, '?? wscale:')
		forBuilding(desc, function(w, ij)
			if not w.df or not w.win or not w.df[w.win] then
				return
			end
			w.df[w.win].scale = val
		end)
	elseif key == 'door_ind' then
		forBuilding(desc, function(w, ij)
			if val < 1 then
				val = 1
			else
				local nwin = math.floor((w.u:length() - 2*w.winleft)/w.winspace)
				if val > nwin then
					val = nwin
				end
			end
			if w.doorstairs and w.doorstairs[w.doorind] then
--                    U.dump(w.doorstairs[w.doorind], '?? for_STAIRS:'..ij[1]..':'..ij[2])
				w.doorstairs[val] = w.doorstairs[w.doorind]
				w.doorstairs[w.doorind] = nil
			end
			if w.doorind then
				w.doorind = val
			end
			W.ui[key] = val
		end)
	elseif key == 'door_bot' then
		forBuilding(desc, function(w, ij)
			if not w.doorind then return end
			local doorH = ddae[w.door].to.z - ddae[w.door].fr.z
			local ma = desc.afloor[ij[1]].h - doorH
			if val > ma then
				val = ma
				W.ui[key] = val
				lo('?? MAX:'..val)
			end
			w.doorbot = val
		end)
	elseif key == 'pillar_span' then
		if scope == 'floor' then
			desc.afloor[cij[1]].pillarspan = not desc.afloor[cij[1]].pillarspan
		elseif scope == 'wall' then
			forBuilding(desc, function(w, ij)
				if w.pillar.span then
					w.pillar.span = nil
				else
					w.pillar.span = true
				end
			end)
		end
	elseif key == 'pillar_space' then
		if true then
			if scope == 'floor' then
				desc.afloor[cij[1]].pillspace = val
			elseif scope == 'wall' then
				local w = desc.afloor[cij[1]].awall[cij[2]]
				local spacemax = w.u:length()/2 --w.u:length() - 2*w.pillar.margin -- w.u:length()/2 --(w.u:length() - 2*w.pillar.margin)/2
				if val >= spacemax then
					val = spacemax
					W.ui[key] = val
				end
	--                    val = 5.1
	--                    W.ui[key] = val
	--[[
	]]
				forBuilding(desc, function(w, ij)
					if val == 0 then
						w.pillar.space = nil
					else
						w.pillar.space = val
					end
				end)
			end
		else
			if scope == 'floor' then
				forBuilding(desc, function(w, ij)
					local spacemax = w.u:length()/2 --w.u:length() - 2*w.pillar.margin -- w.u:length()/2 --(w.u:length() - 2*w.pillar.margin)/2
					if val >= spacemax then
						val = spacemax
						W.ui[key] = val
					end
					if val == 0 then
						w.pillar.space = nil
					else
						w.pillar.space = val
					end
				end)
			end
		end
	elseif key == 'pillar_inout' then
		if scope == 'floor' then
			desc.afloor[cij[1]].pillarinout = val
		elseif scope == 'wall' then
			forBuilding(desc, function(w, ij)
				if val == 0 then
					w.pillar.inout = nil
				else
					w.pillar.inout = val
				end
			end)
		end
	elseif key == 'top_ridge' then
		if not desc.afloor[cij[1]].top.ridge then
			desc.afloor[cij[1]].top.ridge = 0
		end
		desc.afloor[cij[1]].top.ridge = desc.afloor[cij[1]].top.ridge + val
			desc.afloor[cij[1]].top.ridge = val
	elseif key == 'top_margin' then
--            lo('?? top_marg:'..val, true)
--            if true then return end
		local floor = desc.afloor[cij[1]]
--        lo('?? if_child:'..tostring(floor.top.cchild), true)
		if floor.top.cchild then
			floor.top.achild[floor.top.cchild].margin = val
--        else
--            floor.top.margin = val
		end
		floor.top.margin = val
	elseif key == 'top_tip' then
		if val == 0 then
			val = 0.01
			W.ui[key] = val
--            roofSet('flat')
		end
		forTop().tip = val
		if desc.afloor[cij[1]].top.ridge.on then
			desc.afloor[cij[1]].top.tip = val
--            lo('?? top_tip_RIDGE:'..val, true)
		end
--        desc.afloor[cij[1]].top.tip = val
	end
--        lo('?? onVal_out:'..tostring(cedit.forest)..':'..tostring(ifForest({'stringcourse'})), true)
--        lo('??^^^^^^^^^^^^^^^ onV_HU:')
	houseUp(desc, cedit.mesh, true)
--        lo('?? onVal_out2:'..tostring(cedit.forest)..':'..tostring(W.ifForest({'win'}))..':'..tostring(W.ifForest({'pillar'}, true)), true)
		lo('<< onVal:'..tostring(key),true) --..tostring(out.inbalcony)..':'..tostring(ifForest({'stringcourse'},true))..':'..tostring(cedit.forest), true)
	if out.inbalcony then
		forBuilding(desc, function(w, ij)
			if w.df[w.balcony.dae] and #w.df[w.balcony.dae] > 0 then
				cedit.forest = w.df[w.balcony.dae][1]
				dae = w.balcony.dae
			end
		end)
--[[
		markUp(dae)
	else
		markUp()
]]
	end
	markUp(dae)
--[[
		if key == 'win_space' then
			local iffor = ifForest({'win'},true)
			lo('?? iff_win:'..tostring(iffor)..':'..tostring(cedit.forest),true)
		end
]]
	--..tostring(dforest[cedit.forest].type)..':'
--    indrag = false
	incontrol = key
end





W.up = up
W.clear = clear
W.forestEdit = forestEdit
W.forestClean = forestClean
W.goAround = goAround
W.houseUp = houseUp
W.markUp = markUp
W.matApply = matApply
W.meshApply = meshApply
W.matMove = matMove
W.partOn = partOn
--W.regionUp = regionUp
W.scopeOn = scopeOn
W.selectionHide = selectionHide
W.isHidden = isHidden
W.wallSplit = wallSplit
W.conform = conform
W.confSave = confSave
W.restore = restore

W.onKey = onKey
W.keyUD = keyUD
W.keyRL = keyRL
W.keyAlt = keyAlt
W.keyShift = keyShift
W.mdown = mdown
W.mdrag = mdrag
W.mpoint = mpoint
W.mup = mup
W.mwheel = mwheel

W.windowsToggle = windowsToggle
W.doorToggle = doorToggle
--W.balconyToggle = balconyToggle
W.pillarToggle = pillarToggle
W.atticToggle = atticToggle
W.corniceToggle = corniceToggle
W.baseSet = baseSet
W.roofSet = roofSet

W.forDesc = forDesc
W.forScope = forScope
W.ifValid = ifValid
--W.ifAttic = ifAttic
--W.ifPillaster = ifPillaster
W.ifRoof = ifRoof
W.ifTopRect = ifTopRect
W.ifForest = ifForest
W.forDAE = forDAE
W.buildingGen = buildingGen
W.forHeight = forHeight
W.forSel = forSel
W.forTop = forTop
W.forSide = forSide
W.base2world = base2world

W.undo = undo

W.onVal = onVal

W.onUpdate = onUpdate

-- props
W.adesc = adesc
-- W.cedit = cedit
W.dmat = dmat
W.out = out

W.test = test
W.pretest = pretest

return W

