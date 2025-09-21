local im = ui_imgui
--lo('== UI:'..tostring(im.IoFontsGetCount())) --..':'..tostring(im.GetColorU321(im.Col_WindowBg))..':'..tostring(im.Col_WindowBg))

for i = 1,im.IoFontsGetCount() do
--    lo('?? for_font:'..ffi.string(im.IoFontsGetName(i)))
end

local _dbgone = false

local function lo(ms)
end

-- HELP
--[[
Mesh
	Click - mark ext box
		Yellow_selection - mark selection mdata part
	Shift-Click - toggle face selection
	Shift-Drag - group selection
	Drag - move
		Marked_yellow_selection - move
	Ctrl-Wheel - move back/force
		Yellow_selection_marked - for selection
	Esc
	Backspace
	Enter - sel->data
		Shift - append to selection (trans+buf)->sel
Roadconform
	Click - select
		Click node - node select
	L-R - change width
	U-D - add ending node (for roads merging)
Inconform
	Ctrl
		L-R - mantle size
		U-D - base height
Drag
	Forest
		Window - move
>        Door - move
	Floor - move horizontally
	L-R
		Door - index
		Window - leftmargin
	Z
		face selection box
Alt-
	Drag - cam around
	MMove - splitter position wall/top
	Z - floor appendix position
		Wheel - floor appendix resize
		Click - floor append
	Drag - move UV
>    Up-Down - scale UV
	Wheel
		Window - space
		Floor - height
		Roof - tip
Ctrl-
	L-R
		Roof - rotate orientation
	Up-Down
		Top-Child - add floor
		Floor - clone floor
		Building - clone/remove last floor
	Wheel -
		Floor - stretch/shrink
		Roof - margins
		Wall,Side - in/out-set
	Click - select face
Shift
	Drag - rotate building
<    MMove - splitter position wall/top
Del
>    Floor - delete floor
]]

--?? LOAD icons
--local icon_door = icon(file, var.iconSize, im.GetStyleColorVec4(im.Col_ButtonActive))

local M = {}

local apack = {
	'/lua/ge/extensions/editor/gen/mesh',
	'/lua/ge/extensions/editor/gen/utils',
	'/lua/ge/extensions/editor/shortcutLegend',
    '/lua/ge/extensions/editor/gen/decal',
}
local U, D, SCLegend
local editMode

local count_inp
local header = 'Building Architect' --'City Editor'

local ashape = {'sh_square', 'sh_hexa', 'sh_diamond', 'sh_u', 'sh_t', 'sh_cross'}
local inPairEnd

local conf = {
  inputMargin = 94,
}

local aicon = {
--    {'t_shape'}
--    't_shape',
--    s_block = {im.ImVec2(0.12,0.12), im.ImVec2(0.86,.86)},
--    tetromino = {im.ImVec2(0.12,0.12), im.ImVec2(0.86,0.86)},

	sh_square = {im.ImVec2(0.96,0.98), nil, 'b_shape'},
	sh_hexa = {im.ImVec2(0.96,0.98), nil, 'b_shape'},
	sh_diamond = {im.ImVec2(0.96,0.98), nil, 'b_shape'},
	sh_u = {im.ImVec2(1.25,1.25), nil, 'p_shape'},
	sh_t = {im.ImVec2(1.25,1.25), nil, 't_shape'},
	sh_cross = {im.ImVec2(0.96,0.98), nil, 'b_shape'},
--[[
	b_shape = {im.ImVec2(-0.2,0), im.ImVec2(1.2,0.9)},
	l_shape = {im.ImVec2(0.3,0.3), im.ImVec2(0.7,0.7)},
	t_shape = {im.ImVec2(0.3,0.3), im.ImVec2(0.7,0.7)},
	p_shape = {im.ImVec2(0.3,0.3), im.ImVec2(0.7,0.7)},
	s_shape = {im.ImVec2(0.3,0.3), im.ImVec2(0.7,0.7)},
]]

	balcony = {im.ImVec2(0,0), im.ImVec2(1,1)},
	corner = {im.ImVec2(0,0), im.ImVec2(1,1)},
	pillar = {im.ImVec2(0,0), im.ImVec2(1,1)},
	opened_door = {im.ImVec2(0,0), im.ImVec2(1,1)},
	pilaster = {im.ImVec2(0,0), im.ImVec2(1,1)},
	stringcourse = {im.ImVec2(0,0), im.ImVec2(1,1)},
	storefront = {im.ImVec2(0,0), im.ImVec2(1,1)},
	stairs = {im.ImVec2(0,0), im.ImVec2(1,1)},
--    stairs = {im.ImVec2(0.8,0.8), nil},
--    stairs = {im.ImVec2(-0.2,0.2), im.ImVec2(1.2,1.2)},
	window3 = {im.ImVec2(0.1,0.1), im.ImVec2(0.9,0.9)},
--[[
	roof_flat = {im.ImVec2(1,1), nil, 'roof_flat'},
	roof_pyramid = {im.ImVec2(1,1), nil},
	roof_shed = {im.ImVec2(1,1), nil},
	roof_gable = {im.ImVec2(1,1), nil},
]]

	roof_flat = {im.ImVec2(0,0), im.ImVec2(1,1)},
	roof_pyramid = {im.ImVec2(0,0), im.ImVec2(1,1)},
	roof_shed = {im.ImVec2(0,0), im.ImVec2(1,1)},
	roof_gable = {im.ImVec2(0,0), im.ImVec2(1,1)},


	roof_ridge = {im.ImVec2(0,0), im.ImVec2(1,1)},
	roof_border = {im.ImVec2(0,0), im.ImVec2(1,1)},

	roof_chimney = {im.ImVec2(0,0), im.ImVec2(1,1)},

	width_fit = {im.ImVec2(0,0), im.ImVec2(1,1)},


--    window = {im.ImVec2(-0.2,-0.2), im.ImVec2(1.2,1.2)},
--    win2 = {im.ImVec2(0,0), im.ImVec2(1,1), suf = '.jpg'},
--    window = {im.ImVec2(0,0), im.ImVec2(1,1)},
--    t_shape = {im.ImVec2(0.12,0.12), im.ImVec2(0.86,0.86)},
--    t_shape = {im.ImVec2(0.2,0.2), im.ImVec2(0.8,0.8)},
--    "/art/dynamicDecals/textures/00_color_palette_test.png",
}
local dicon = {
}

local SCNow, SCPre = {}, {}
local HTree,HTreeR,HTreeTer = {},{},{}

--editor.unregisterWindow("BAT")

local function reload()
	if editor and editor.unregisterWindow then
		editor.unregisterWindow("BAT")
		if not editor.isWindowRegistered('BAT') then
			editor.registerWindow("BAT") --, im.ImVec2(200, 700))
		end

		editor.unregisterWindow("LAT")
		if not editor.isWindowRegistered('LAT') then
			editor.registerWindow("LAT") --, im.ImVec2(200, 700))
		end
	end
--    unrequire(apack[1])
--    M = rerequire(apack[1])
	unrequire(apack[2])
	U = rerequire(apack[2])
	unrequire(apack[3])
	SCLegend = rerequire(apack[3])
--        lo('?? forlegend:'..SCLegend)
--    unrequire(apack[4])
--    D = rerequire(apack[4])
    	U.lo('>>++++++++++++++++ UI.reload:'..#apack..':'..tostring(D)) --..':'..tostring(D.forRoad()))

	if editor.texObj then
		for key,p in pairs(aicon) do
--                U.dump(aicon[key], '?? for_icon:'..tostring(aicon[key]))
			dicon[key] = editor.texObj('/lua/ge/extensions/editor/gen/assets/_icon/'..(aicon[key][3] or key)..(p.suf or '.png'))
--            dicon[key] = editor.texObj('/lua/ge/extensions/editor/gen/assets/_icon/'..key..(p.suf or '.png'))
	--        dicon[key] = editor.texObj('/lua/ge/extensions/editor/gen/assets/_icon/'..key..'.png')
	--        dicon[p] = editor.texObj('/lua/ge/extensions/editor/gen/assets/_icon/'..p..'.png')
	--        dicon[p[1]] = editor.texObj('/lua/ge/extensions/editor/gen/assets/_icon/'..p[1]..'.png')
		end
	end
--	M.forHelp()
		lo('<<++++++++++++++++++_________________________ reload:'..tableSize(dicon))
end
reload()
--print('=??????????????????? in_UI:')

local W,D,R


M.inject = function(olib)
--		print('?? UI.inject:')
	if olib.U then
		U = olib.U
--		print('??******** UI.inject:'..U._MODE..':'..U.out._MODE)
	end
	if olib.W then
		W = olib.W
	end
end


local scope,cij
local croad, cforest

local function forHelp(W, R)
	----------------------------------------------
	----------- HELP -----------------------------
	----------------------------------------------
	-- NETWORK
	local insector = W.out.Ter and W.out.Ter.out.insector
	HTreeTer = {{}, {}, {}, {}}
	local row = HTreeTer[1]
	row[#row + 1] = {'Click', insector and 'pick road/junction'}
	row[#row + 1] = {'Shift-Click', insector and 'pick sector' or 'add/rmove center'}
--	row[#row + 1] = {'Alt-Click', 'road/exit '..('start/end')}

	-- ROAD
	HTreeR = {{}, {}, {}, {}}
	local row = HTreeR[1]
	row[#row + 1] = {'Ctrl-Click', 'append to selection'}
	row[#row + 1] = {'Alt-Click', 'road/exit '..('start/end')}

	-- BAT
	scope,cij = W.forScope()
	HTree = {{}, {}, {}, {}}
	local row = HTree[1]
	row[#row + 1] = {'Ctrl-Click', 'new building'}
	if W.out.axis then
		row[#row + 1] = {'Shift-Click', 'pick center'}
	elseif U._PRD == 0 then
		row[#row + 1] = {'Shift-Click', 'pick region'}
	end
	row[#row + 1] = {'Ctrl-X', 'wipe-out'}
	if scope and ({wall=1,floor=1,building=1})[scope] then
		row[#row + 1] = {'Del', ''}
	end
	if scope then
		row = HTree[2]
--        row[#row + 1] = {'Alt-Drag', 'fly around'}
		if W.ifForest() then
			row[#row + 1] = {'Up/Down', 'switch meshes'}
			if U._PRD == 0 then
				row[#row + 1] = {'Ctrl-Wheel', 'scale'}
			end
			if W.ifForest({'door'}) then
				row[#row + 1] = {'Left-Right', 'door position'}
			elseif U._PRD==0 and W.ifForest({'win'}) then
				row[#row + 1] = {'NEXT', 'line'}
				row[#row + 1] = {'Drag', 'position'}
				row[#row + 1] = {'Alt-Wheel', 'spacing'}
			end
		else
			if U._PRD == 0 and scope == 'building' then
				row[#row + 1] = {'Drag', 'position'}
				row[#row + 1] = {'Z-Drag', 'z-position'}
			elseif scope == 'top' then
--                row[#row + 1] = {'Z-Left/Right', 'flip'}
			end
			if U._PRD == 0 then
				row[#row + 1] = {'Alt', '...'}
			else
				if not W.out.invertedge then
					row[#row + 1] = {'Alt-Click', 'split '..tostring(scope)}
          if ({building=1,floor=1,wall=1})[scope] then
            row[#row + 1] = {'Shift-Click', 'switch scope'}
          end
				end
			end
			if U._PRD==0  then
				row[#row + 1] = {'Shift', '...'}
			else
				if ({floor=1,wall=1})[scope] then
					row[#row + 1] = {'Ctrl', '...'}
				end
			end
			if editor.keyModifiers.alt then
				row = HTree[3]
				local splittable
				if ({wall=1, building=1})[scope] then
					splittable = 'wall'
				elseif scope == 'floor' then
					if U._PRD == 0 then
						row[#row + 1] = {'Alt-Wheel', 'height'}
					end
					row[#row + 1] = {'Alt-Up/Down', 'flip floors'}
					splittable = 'wall'
				elseif scope == 'side' then
					splittable = 'side'
				elseif scope == 'top' then
					splittable = 'roof'
					if U._PRD == 0 then
						row[#row + 1] = {'Alt-Wheel', 'slope'}
					end
					local top = W.forTop()
					if U._PRD == 0 and top and ({gable=1, shed=1})[top.shape] then
						row[#row + 1] = {'Alt-Left/Right', 'flip'}
					end
				end
--[[
				if pairEnd == nil then pairEnd = W.ifPairEnd() end
				if W.out.invertedge or pairEnd then
					if U._PRD == 0 then
						row[#row + 1] = {'Alt-Drag', 'keep parallels'}
					end
--                    row[#row + 1] = {'Alt-Drag', 'V-shape'}
				else
					row[#row + 1] = {'Alt-Click', 'split '..tostring(splittable)}
				end
]]
				if U._PRD == 0 and not W.out.invertedge then
					row[#row + 1] = {'Alt-Click', 'split '..tostring(splittable)}
				end
				if U._PRD == 0 and scope == 'top' then
					row[#row + 1] = {'Z', '...'}
					if im.IsKeyDown(im.GetKeyIndex(im.Key_Z)) then
						row = HTree[4]
						row[#row + 1] = {'Click', 'place floor'}
						row[#row + 1] = {'Wheel', 'in/out-set'}
					end
				end
				if U._PRD == 0 and false then
					row[#row + 1] = {'Alt-Drag', 'fly around'}
				end
			end
			if editor.keyModifiers.ctrl then
				row = HTree[3]

				if inPairEnd == nil then inPairEnd = W.ifPairEnd() end
				if W.out.acorner or inPairEnd then
--				if W.out.invertedge or inPairEnd then
					row[#row + 1] = {'Ctrl-Drag', 'keep parallels'}
--					if U._PRD == 0 then
--					end
--                    row[#row + 1] = {'Alt-Drag', 'V-shape'}
				elseif W.out.inconform then
					row[#row + 1] = {'Ctrl-Up/Down', 'height'}
					row[#row + 1] = {'Ctrl-Left/Right', 'conformation width'}
				else
					if U._PRD == 0 and ({floor=1,building=1})[scope] then
						row[#row + 1] = {'Ctrl-Drag', 'rotate'}
					end
					if ({wall=1,floor=1,side=1})[scope] then
						row[#row + 1] = {'Ctrl-Click', 'add selection'}
					end
					if U._PRD == 0 and scope ~= 'building' then
						row[#row + 1] = {'Ctrl-Wheel', 'in/out-set'}
					end
--[[
					if scope == 'wall' then
					elseif scope == 'side' then
						row[#row + 1] = {'Ctrl-Wheel', 'in/out-set'}
					elseif scope == 'floor' then
						row[#row + 1] = {'Ctrl-Wheel', 'in/out-set'}
]]
					if scope == 'floor' then
						row[#row + 1] = {'Ctrl-Up', 'clone floor'}
					elseif U._PRD==0 and scope == 'building' then
						row[#row + 1] = {'Ctrl-Up/Down', 'clone/remove top floor'}
					elseif U._PRD == 0 and scope == 'top' then
						row[#row + 1] = {'Ctrl-Wheel', 'roof margin'}
					end
				end
			else
				inPairEnd = nil
			end
			if editor.keyModifiers.shift then
				row = HTree[3]
				if W.out.invertedge then
					row[#row + 1] = {'Shift-Drag', 'split position'}
				elseif U._PRD==0 then
					row[#row + 1] = {'Shift-Drag', 'texture position'}
				end
				if W.out.inaxis then
					row[#row + 1] = {'Shift-Left/Right', 'flip around center'}
				elseif U._PRD==0 then
					row[#row + 1] = {'Shift-Left/Right', 'fly around'}
				end
				if scope == 'wall' and W.out.middle then
					row[#row + 1] = {'Shift-Click', 'center the wall'}
				end
			end
		end
	end
	if R.out.inpave then
		row = HTree[2]
		row[#row + 1] = {'Enter', 'build'}
	end
end
--M.forHelp = forHelp


local function buttonSelector(list, cb)
	for i,v in pairs(list) do
		im.SameLine()
--        ifroof = W.ifRoof('pyramid')
--        color = ifroof and im.ImVec4(0.5, 0.7, 0.5, 1) or im.ImVec4(0.5, 0.7, 0.5, 0.5)
		if editor.uiIconImageButton(v.icon, --editor.icons.forest_select,
			im.ImVec2(34, 34), color,
			nil, nil, 'MultiSelectButton') then
--            if ifroof then W.roofSet('pyramid') end
		end
		if list.text then im.tooltip(list.text) end
	end
end

local env


local function labelHide(shift, lbl)
	if not shift then
    if lbl then
      shift = -im.CalcTextSize(lbl).x - 6
    else
      shift = -30
    end
  end
	im.SameLine()
	local cur = im.GetCursorPos()
	local topLeft = im.GetWindowPos()
	topLeft.x = topLeft.x + cur.x + shift -- padding
	topLeft.y = topLeft.y + cur.y
	local bottomRight = im.ImVec2(topLeft.x + 100, topLeft.y + 20)
	local color = im.GetColorU321(im.Col_WindowBg)
	im.ImDrawList_AddRectFilled(im.GetWindowDrawList(), topLeft, bottomRight, color)
	im.Dummy(im.ImVec2(0,0))
end


local function combo(lbl, key, list, isel)
--    if not env or not env.ui[key] then return end
	if not isel then
		isel = env and env.ui[key] or 0
	end
--        im.Text(lbl)
--        if true then return end

	local comboItems = im.ArrayCharPtrByTbl(list)
--    if im.Combo2(lbl, editor.getTempInt_NumberNumber(isel), "aaaaaa") then
  im.Text(lbl)
  im.SameLine()
  im.Indent(conf.inputMargin)
	count_inp = count_inp + 1
	if im.Combo1(tostring(count_inp), editor.getTempInt_NumberNumber(isel), comboItems, nil, nil) then
		local o = editor.getTempInt_NumberNumber()
--            lo('?? combo:'..lbl..':'..key..':'..isel)
--            lo('?? seld:'..tostring(o))
		env.onVal(key, o)
	end
	labelHide()
  im.Unindent(conf.inputMargin)
end


local function inputLeft(lbl, key, isint, step)
	if not step then step = 1 end
	local tp = isint and 'Int' or 'Float'

	im.Text(lbl)
--        if true then return end
	im.SameLine()
	local ptr = im[tp..'Ptr'](env and env.ui[key] or 0)
--    im['Input'..tp]('sadas', ptr, step)
	count_inp = count_inp + 1
	if im['Input'..tp](tostring(count_inp), ptr, step) then
		env.onVal(key, ptr[0])
	end

	-- cover ID-label
	labelHide()
end
--    local padding = 2
--        lo('?? TL:'..topLeft.x..':'..topLeft.y)
--        topLeft = im.ImVec2(-461,277)
--    topLeft.x = topLeft.x  - 50
--    local bottomRight = im.ImVec2(topLeft.x + sz.x + 2*padding, topLeft.y + 20)
--    local color = im.GetColorU322(im.ImVec4(1.0, 0.0, 0.0, 1.0))


local function input(lbl, key, isint, step, nolabel)
	if not step then step = 1 end
	local tp = isint and 'Int' or 'Float'
--    if not env or not env.ui[key] then return end

	local ptr = im[tp..'Ptr'](env.ui[key] or 0)
	im.Dummy(im.ImVec2(0, 0))

  im.Text(lbl)
  im.SameLine()
  im.Indent(conf.inputMargin)
	if im['Input'..tp]("##" .. lbl, ptr, step) then
			lo('?? input:'..key)
		env.onVal(key, ptr[0])
	end
	if true or nolabel then
--		labelHide(nil, lbl)
--        im.SameLine()
--        im.Indent(140)
--        im.ColorButton('ib'..lbl, im.ImVec4(0.6, 0.0, 0.0, 1))
	end
  im.Unindent(conf.inputMargin)
end


local function slider(lbl, key, mm, isint, format)
	local nolabel
	if not lbl then
		nolabel = true
		count_inp = count_inp + 1
		lbl = tostring(count_inp)
	end
--  count_inp = count_inp + 1
--  local clbl = tostring(count_inp)
	local tp = isint and 'Int' or 'Float'
	if not format then
		format = tp == 'Float' and '%.1f' or '%d'
	end

--[[
	if not env.ui[key] then
		lo('!! ERR_slider_noval:'..tostring(key))
		return
	end
]]
	local ptr = im[tp..'Ptr'](env.ui[key] or 0)
	im.Dummy(im.ImVec2(0, 0))

--  im.SameLine()
	if not nolabel then
		im.Text(lbl)
		im.SameLine()
		im.Indent(conf.inputMargin)
	end
	if im['Slider'..tp]("##" .. lbl, ptr, mm[1], mm[2], format) then
		env.onVal(key, ptr[0])
	end
--	if true or nolabel then
--		labelHide(nil, lbl)
--	end
	if not nolabel then
		im.Unindent(conf.inputMargin)
	end
end

local colorOn = im.ImVec4(0.5, 0.7, 0.5, 1)
local colorOff = im.ImVec4(0.5, 0.5, 0.5, 0.7)
local colorList = im.ImVec4(0.5, 0.5, 0.55, 1)
local bgActive = im.ImVec4(0.3, 0.3, 0.3, 1)

local function button(lbl, key, icon, c, isactive)
	if not c then c = im.ImVec4(0.5, 0.7, 0.5, 1) end
	local bgcolor = isactive and bgActive or nil
	if editor.uiIconImageButton(editor.icons[icon], --editor.icons.forest_select,
		im.ImVec2(30, 30), c,
		nil, bgcolor, 'MultiSelectButton') then
			if c ~= colorOff then
				env.onVal(key)
			end
	end
	if lbl then
		im.tooltip(lbl)
	end
end

-- text button
local function buttonT(text, key, ctxt, cbg, dim, center, shift)

	cbg = cbg or im.ImVec4(0.7, 0.7, 0., 1)
	dim = dim or im.ImVec2(62,22)
--    cbg = im.ImVec4(0.7, 0.7, 0., 1)
	if editor.uiIconImageButton(editor.icons.forest_select,
	dim, im.ImVec4(0.0, 0.0, 0.0, 0), nil, cbg, 'MultiSelectButton') then
		if ctxt == colorOff then return end
		env.onVal(key, text)
--          lo('?? ui.buttonT.matsel:'..tostring(W.out.curselect))
	end

	-- overlay text
	im.SameLine()
--      center = nil
  if center then
    local len = im.CalcTextSize(text).x
    local ds = (dim.x - len)/2 + (shift or 0)
    im.Indent(-30+ds)
    im.Unindent(-30)
  else
    im.Indent(-24)
    im.Unindent(-30)
  end
--    im.Text(text)
	im.TextColored(ctxt, text)
end

-- custom image
local function buttonCC(lbl, key, c, dim, cbg)
	im.BeginGroup()
	-- invisible button
	if editor.uiIconImageButton(editor.icons.forest_select, --editor.icons.forest_select,
	dim, im.ImVec4(0.0, 0.0, 0.0, 0), nil, cbg, 'MultiSelectButton') then
		if c == colorOff then return end
		env.onVal(key, lbl)
	end
	im.SameLine()
	im.Indent(-30)
	im.Unindent(-30)
	-- overlay icon
	if dicon and dicon[lbl] then
		im.Image(dicon[lbl].tex:getID(),
			dim,
			aicon[lbl][1], aicon[lbl][2], c
		)
	end
	im.EndGroup()
end


local function buttonC(lbl, key, c, dim, cbg)
--        path = 't_shape'
--    local width = im.CalcTextSize(t).x
	--                if im.ColorButton('ib'..i, im.ImVec4(0.6, 0.0, 0.0, 1)) then
	if not dim then dim = im.ImVec2(37,37) end
--    if not cbg then cbg = im.ImVec4(0.2, 0.2, 0.2, 1) end
--    local bgcolor = nil
	if not c then
		im.Dummy(im.ImVec2(0, 0))
		if im.Button(lbl) then
			env.onVal(key, lbl)
		end
	else
		im.BeginGroup()
			if editor.uiIconImageButton(editor.icons.forest_select, --editor.icons.forest_select,
				dim, im.ImVec4(0.0, 0.0, 0.0, 0),
				nil, cbg, 'MultiSelectButton') then

        if c == colorOff then return end
        env.onVal(key, lbl)
			end
	--[[
			if im.InvisibleButton(lbl, im.ImVec2(30, 30)) then
				env.onVal(key, lbl)
	--            lo('?? click:')
			end
	]]
--                dim = im.ImVec2(64,64)
			im.SameLine()
			im.Indent()
			im.Unindent()
			if dicon and dicon[lbl] then
				im.Image(dicon[lbl].tex:getID(),
				dim,
		--        im.ImVec2(0,0), im.ImVec2(0.5,0.5))
				aicon[lbl][2] and aicon[lbl][1] or im.ImVec2(aicon[lbl][1].x-1,aicon[lbl][1].y-1),
				aicon[lbl][2] and aicon[lbl][2] or im.ImVec2(2-aicon[lbl][1].x,2-aicon[lbl][1].y),
				c,
--                aicon[lbl][1], aicon[lbl][2], c,
				im.ImVec4(0.6, 1.0, 0.0, 0.01)
				)
			end
		im.EndGroup()
	end
--    im.TextColored(lbl, lbl)
end


local function check(lbl, key)
  if env.ui[key] == nil then return end
--  count_inp = count_inp + 1
--  local clbl = tostring(count_inp)

  local ptr = im.BoolPtr(env.ui[key])
  im.Text(lbl)
  im.SameLine()
  im.Indent(conf.inputMargin)

  if im.Checkbox("##" .. lbl, ptr) then
--    lo('?? check:'..tostring(ptr[0])..':'..tostring(env))
  	env.onVal(key, ptr[0])
  end
--  labelHide(nil)
  im.Unindent(conf.inputMargin)
end


local function columnRight(width, border)
	if not border then border = false end
--        border = true
	im.Columns(2, 'right_'..width, border)
	im.SetColumnWidth(0, im.GetWindowWidth() - width)
	im.SetColumnWidth(1, width)
end


local function control(list, w, dec, reg)
	count_inp = 200
	if not W then forHelp(w, reg) end
	W = w
	env = W
	D = dec
--		U.lo('?? control:'..tostring(D))
	R = reg
	list = scope == 'top' and W.dmat.roof or W.dmat.wall --W.ui.mat_wall  or {}

	local handled = false

	scope,cij = W.forScope()
--	scope = W.out.scope
--	forHelp()
	croad = U._PRD == 0 and D.forRoad() or nil
	cforest = W.ifForest()
	local desc = W.forDesc()
--	if croad then scope = nil end
	local colorNav = im.ImVec4(0.4, 0.6, 0.8, 1)

	local inconform = W.out.D and (W.out.D.forRoad() or W.out.D.out.inall) or nil -- or scope == 'building'
	local injunction = W.out.D and W.out.D.out.injunction or nil
	local insector = W.out.Ter and W.out.Ter.out.insector
--		print('?? if_LAT:'..tostring(editor.beginWindow('LAT', 'WIP_Landscape_Gen')))

	if editor.beginWindow('LAT', 'WIP_Landscape_Gen') then
--	if editor.beginWindow('LAT', 'Landscape Editor') then
--		local ctxt = im.ImVec4(0.9, 0.9, 0.9, 1)
--		buttonT('BASES2', 'base_load3', ctxt, im.ImVec4(0.47, 0.45, 0.45, 1), nil, true, -3)
		local bgcolor, color
		local scapeColor = im.ImVec4(0.1, 0.6, 0.9, 0.8)

		if ({conf=0,ter=0})[U.out._MODE] then
--			if W.out.D.forRoad() or W.out.D.out.inall then
--								slider('Junction width', 'conf_jwidth', {1,8})
--			end
			columnRight(64)

			if injunction then
--				input('Branches', 'branch_n', true)
--				slider('Radius', 'exit_r', {5, 50})
				slider('Exit radius', 'exit_r', {5, 50})
				slider('Exit width', 'exit_w', {2, 10})
--								im.Dummy(im.ImVec2(0, 6))
				check('Roundabout', 'junction_round')
				slider('Radius', 'round_r', {10, 20*3})
				slider('Circle width', 'round_w', {4, 10})
			elseif inconform then
				slider('Margin', 'conf_margin', {1,20})
				slider('Max slope', 'conf_mslope', {2,45})
				slider('Banking rate', 'conf_bank', {0,5})
				W.ui.conf_all = W.out.D.out.inall or false
				im.Dummy(im.ImVec2(0, 6))
				check('For all', 'conf_all')
			elseif insector  then
				slider('Margin', 'sec_margin', {6,50})
				slider('Grid spacing', 'sec_grid', {6,200})
				slider('Grid axis 1', 'sec_griddir1', {-30,30})
				slider('Grid axis 2', 'sec_griddir2', {-30,30})
				slider('Random rate', 'sec_rand', {0,0.4}, false, "%.02f")
--				slider('Margin', 'sec_trim', {6,50})
			end

			im.NextColumn()
			-- conform button
			if W.out.D.out.inconform then
				bgcolor = bgActive
			else
				bgcolor = nil
			end
			color = (inconform or (injunction and injunction.r)) and scapeColor or colorOff
			local title = 'Conform'
			if W.out.D.forRoad() then
				if W.out.D.out.inall then
					title = title..' all'
				else
					title = title..' road'
				end
			elseif scope == 'building' then
				title = title..' base'
			end
			button(title, 'b_conform', 'terrain_tools', color)
			-- junction button
			color = injunction and im.ImVec4(0.5, 0.7, 0.5, 1) or colorOff
			button('build junction', 'b_junction', 'twoRoadsCrossAdd', color)

			-- road place button
			color = im.ImVec4(0.4, 0.4, 1, 1) --W.out.D.out.inplot and im.ImVec4(0.5, 0.7, 0.5, 1) or colorOff
			button('place roads', 'b_road', 'timeline', color)

			im.Indent(10)
			im.Columns(1)
			local padding = 2

	        im.Dummy(im.ImVec2(0, 26))
			im.TextColored(colorNav, 'Hint')

			local insector = W.out.Ter and W.out.Ter.out.insector
			im.Text('Shift-Click: '..(W.out.Ter.out.inplace and 'pick sector' or 'add/rmove center'))
			if not W.out.Ter.out.inplace then
				im.Text('Ctrl-Wheel: '..(insector and '' or 'region resize'))
			end
			im.Text('Click: pick road/junction')
			if inconform then
				im.Text('Ctrl-Click: append to selection')
				im.Text('Alt-Click: road/exit '..('start/end'))
			end
			if insector then
--				M.tree2ui(HTreeTer)
--				print('?? help ter')
			else
				M.tree2ui(HTreeR)
			end
		end
	end

--					color = (W.out.D.forRoad() or W.out.D.out.inall or scope == 'building') and scapeColor or colorOff
--							color = scapeColor
--[[
			if editor.uiIconImageButton(editor.icons.terrain_tools,
				im.ImVec2(30, 30), color,
				nil, bgcolor, 'MultiSelectButton') then
					U.lo('?? ui.to_conform:'..tostring(W.out.D.forRoad()))
				W.out.avedit = {}
				if W.out.D.forRoad() or W.out.D.out.inall then --== 1 then
					if U._MODE == 'conf' then
						W.out.D.road2ter()
					elseif D.out.inconform then
--??                        D.restore()
--								W.out.D.ter2road()
					else
--								W.out.D.ter2road()
					end
				elseif scope == 'building' then
					if W.out.inconform then
						W.restore()
					else
						local adsc = W.forSel()
						for _,dsc in pairs(adsc) do
							W.conform(dsc)
--                                    W.restore(true)
							if #adsc > 1 then
								W.confSave()
							end
						end
						if #adsc > 1 then
--                                    W.confSave()
						end
					end
				end
			end
--                    title = title + (D.forRoad() and ' road' or ' base') -- (D.out.inconform or W.out.inconformm) and 'Restore' or 'Conform'
			im.tooltip(title)

]]

--    if editor.beginWindow("BAT", "City Editor") then
	if editor.beginWindow("BAT", header) then --, im.WindowFlags_NoCollapse) then
--    if editor.beginWindow("PANE", "City Editor") then
		im.Dummy(im.ImVec2(0, 0))
		im.SameLine()
		----------------------------------------------
		----------- BUILDING SEED --------------------
		----------------------------------------------
--        im.TextColored(colorNav, 'Shape type')
--		columnRight(174)

		local bsize = 60
		im.Indent(8)
		local nbut = 3
		im.Columns(nbut+1, 'right_'..bsize, false)
		im.SetColumnWidth(0, im.GetWindowWidth() - (24 + bsize*nbut))
		for i=1,nbut do
			im.SetColumnWidth(i, bsize)
		end
--		im.SetColumnWidth(1, bsize)
--		im.SetColumnWidth(2, bsize)
--		im.SetColumnWidth(3, bsize)

--		im.SetColumnWidth(4, bsize)
--		im.SetColumnWidth(5, bsize)


--        columnRight(174)
--    im.Indent(12)
		im.PushFont3("cairo_regular_medium")
--        im.PushFont3("cairo_semibold_large")
		im.Text('Shape type')
		im.PopFont()
--    im.Unindent(12)
		im.PushFont3("segoeui_regular")

		local ctxt, opacity -- = (scope == 'building') and im.ImVec4(0.9, 0.9, 0.9, 1) or colorOff

		im.NextColumn()
		ctxt = im.ImVec4(0.9, 0.9, 0.9, 1) --W.out.saved and im.ImVec4(0.9, 0.9, 0.9, 1) or colorOff
		opacity = 1 --W.out.saved and 1 or 0.3
		if false then
			buttonT('BASES', 'base_load', ctxt, im.ImVec4(0.47, 0.45, 0.45, opacity), nil, true, -3)
			im.NextColumn()
			buttonT('TODAE', 'dae_exp', ctxt, im.ImVec4(0.47, 0.45, 0.45, opacity), nil, true, -3)
			im.NextColumn()
		end
    	buttonT('Load', 'session_load', ctxt, im.ImVec4(0.47, 0.45, 0.45, opacity), nil, true, -3)

		im.NextColumn()
    ctxt = true and im.ImVec4(0.9, 0.9, 0.9, 1) or colorOff
    opacity = true and 1 or 0.3
    buttonT('Save', 'session_save', ctxt, im.ImVec4(0.47, 0.45, 0.45, opacity), nil, true, -3)

		im.NextColumn()
    ctxt = (scope == 'building') and im.ImVec4(0.9, 0.9, 0.9, 1) or colorOff
    opacity = (scope == 'building') and 1 or 0.3
    buttonT('Export', 'dae_export', ctxt, im.ImVec4(0.47, 0.45, 0.45, opacity), nil, true, -4)

    im.Unindent(44)
		im.Columns(0)

--        im.PushFont3("cairo_semibold_small")
--        im.Dummy(im.ImVec2(0, 2))
		im.Dummy(im.ImVec2(0, 0))
		for k,s in pairs({'rectangle','hexagon','corner_cut','u_shape','t_shape','x_shape'}) do
--        for k,s in pairs(ashape) do
			local c = W.ui.building_shape == s and im.ImVec4(0.6, 1.0, 0, 0.6) or colorList
			local bgcolor = W.ui.building_shape == ashape[k] and im.ImVec4(0.9, 0.5, 0, 1) or nil -- im.ImVec4(1, 0.7, 0, 0.7) or nil -- im.ImVec4(1., 1.0, 0, 0.6) or nil
      		local clr = W.ui.building_shape == ashape[k] and im.ImVec4(0.2, 0.2, 0.2, 0.8) or colorList
--			local bgcolor = W.ui.building_shape == ashape[k] and bgActive or nil
			im.SameLine()
			local dim = im.ImVec2(37,37)
			local key = ashape[k]
			if editor.uiIconImageButton(editor.icons[s], --editor.icons.forest_select,
				dim, clr, -- im.ImVec4(1.0, 1.0, 1.0, 1),
				nil,
				bgcolor, -- bg color
				'MultiSelectButton') then
--                    if c == colorOff then return end
					if env then
						env.onVal('building_shape', key)
--                            lo('?? for_shape:'..tostring(W.ui.building_shape))
					else
						lo('!! NO_ENV:')
					end
			end
--            im.Indent(-4)


--            buttonC(s, 'building_shape', c) -- 'tetromino') -- 't_shape')
			im.tooltip(s)
--            im.Unindent(-4)
		end

--[[
]]
--        im.Dummy(im.ImVec2(10, 0))

--		im.Indent(10)
--		combo('Var.type', 'building_subshape', {'','SUB_SHAPE'})
--		im.Unindent(10)

--[[
--!!		im.Dummy(im.ImVec2(0, 4))
--		local opacity = (scope == 'building') and 1 or 0.4
--        local ctxt = (scope == 'building') and im.ImVec4(0.1, 0.1, 0.1, 1) or colorOff
--    buttonT('Clone', 'floor_clone', im.ImVec4(0.9, 0.9, 0.9, 1), im.ImVec4(0.47, 0.45, 0.45, 1), nil, true) --, nil, im.ImVec2(67,47))

--!!    im.Unindent(10)

--    im.SameLine()


    ctxt = true and im.ImVec4(0.9, 0.9, 0.9, 1) or colorOff
    opacity = true and 1 or 0.4
    buttonT('Save', 'session_save', ctxt, im.ImVec4(0.47, 0.45, 0.45, opacity), nil, true)
    im.SameLine()
    ctxt = (scope == 'building') and im.ImVec4(0.9, 0.9, 0.9, 1) or colorOff
    opacity = (scope == 'building') and 1 or 0.4
    buttonT('Export', 'dae_export', ctxt, im.ImVec4(0.47, 0.45, 0.45, opacity), nil, true)
]]
--		buttonT('Export', 'dae_export', ctxt, im.ImVec4(0.25, 0.25, 0.25, 1), im.ImVec2(50,20))
--[[
		im.Dummy(im.ImVec2(0, 6))
		combo('Style', 'building_style', {'','traditional/regional','modern', 'classic/monumental','functional','contemporary','decorative','historical'})
		im.Dummy(im.ImVec2(0, 41))
		inputLeft('Seed', 'seed', true)
]]
--    im.Unindent(42)
--		im.Columns(0)
--        im.Dummy(im.ImVec2(0, 12))

--[[
		if im.TreeNodeEx1('Shape type', im.TreeNodeFlags_DefaultOpen) then
--        if im.TreeNode1('Building seed') then
			columnRight(174)
				for k,s in pairs({'b_shape', 'l_shape','t_shape','p_shape','s_shape'}) do
					local c = W.ui.building_shape == s and im.ImVec4(0.6, 1.0, 0, 0.6) or colorList
					buttonC(s, 'building_shape', c) -- 'tetromino') -- 't_shape')
					im.SameLine()
				end
				im.NextColumn()
				combo('Style', 'building_style', {'','residential','industrial'})
			im.Columns(0)
			im.Dummy(im.ImVec2(0, 12))
			im.TreePop()
		end
]]
		im.Dummy(im.ImVec2(0, 8))
--        im.Separator()
--        im.TextColored(colorNav, 'SHAPES222:')
--        im.Separator()

		----------------------------------------------
		----------- SCOPE SELCTOR --------------------
		----------------------------------------------
		local rightPanelWidth = U._PRD ==0 and 90 or 2
		columnRight(rightPanelWidth)
--        im.Columns(2, 'rScope', false)
--        im.SetColumnWidth(0, im.GetWindowWidth() - rightPanelWidth)
--        im.SetColumnWidth(1, rightPanelWidth)

--            im.SameLine()
			im.Dummy(im.ImVec2(4, 0))
			im.SameLine()
			if W.out.inseed == true then
--                lo('?? seeeed:')
			end

--            button('Seed building', 'building_seed', 'add_location', nil, W.out.inseed == true)
--            im.SameLine()
--            im.Dummy(im.ImVec2(8, 0))
			im.Separator()
--            im.SameLine()
--            im.TextColored(colorNav, 'Scope:')
			im.Dummy(im.ImVec2(8, 0))
			im.Indent(8)

      		im.PushFont3("cairo_regular_medium")
			im.Text('Scope')
      		im.PopFont()
      		im.PushFont3("segoeui_regular")

--			im.Text('Scope')
			im.Unindent(8)
--            im.SameLine()
			im.Indent(6)
			im.Dummy(im.ImVec2(6, 0))

			for i,t in pairs({'Building', 'Floor', 'Wall', 'Side', 'Top'}) do
--            for i,t in pairs({'Building', 'Floor', 'Side', 'Wall', 'Top'}) do
				if U._PRD == 1 then
					local scp = string.lower(t)
					if i > 1 then
						im.SameLine()
					end
--                    im.Indent()
--                    im.Unindent()
					im.BeginGroup()
					local ctxt = (scp == scope) and im.ImVec4(0.1, 0.1, 0.1, 1) or im.ImVec4(1, 1, 1, 1)
					local cbg = (scp == scope) and im.ImVec4(0.9, 0.5, 0, 1) or im.ImVec4(0.47, 0.45, 0.45, 1)
					buttonT(t, 'scope_'..scp, ctxt, cbg, nil, true)
					im.EndGroup()

--                    W.scopeOn(scope)
--                    button(t, 'pillar_span', 'compare_arrows')
				else

					im.SameLine()

					local colText = im.ImVec4(0.5, 0.5, 0.5, 1)
					if scope == string.lower(t) then
						colText = im.ImVec4(0.9, 0.9, 0.9, 1)
					end
		--            if editor.uiHighlightedText(t..'1', t..'2', im.ImVec4(0.6, 0.0, 0.0, 1)) then
		--            im.Indent(10)
		--            im.Dummy(im.ImVec2(10, 0))

					im.BeginGroup()
						local width = im.CalcTextSize(t).x
		--                if im.ColorButton('ib'..i, im.ImVec4(0.6, 0.0, 0.0, 1)) then
						if im.InvisibleButton('ib'..i, im.ImVec2(width + 5, 20)) then
		--                    lo('?? cclick'..i..':'..t)
							scope = string.lower(t)
							W.scopeOn(scope)
						end
						im.SameLine()
						im.Indent(-30)
						im.Unindent(-30)
						im.TextColored(colText, t)
		--                editor.uiHighlightedText(t, t, colText)
		--                im.Unindent(30)
					im.EndGroup()

				end
	--            if editor.uiHighlightedText(t, t, colText) then
	--                lo('???')
	--            end
			end
			im.Unindent(6)
--            im.Indent(8)
--        im.SameLine()

		im.NextColumn()
		if U._PRD == 0 then

  --                im.SameLine()
  --                im.Dummy(im.ImVec2(6, 0))
  --            im.Indent(2)

          im.BeginGroup()
            im.Indent(6)
    --                im.SameLine()
    --                im.Dummy(im.ImVec2(6, 0))
            local ishidden = W.isHidden()
            if editor.uiIconImageButton(ishidden and editor.icons.visibility_off or editor.icons.visibility,
              im.ImVec2(24, 24), im.ImVec4(0.7, 0.7, 0.7, 0.8),
              nil, nil, 'MultiSelectButton') then
              W.selectionHide()
            end

            im.SameLine()
            im.Dummy(im.ImVec2(0, 0))
    --                im.Indent(6)
            im.SameLine()
            if editor.uiIconImageButton(editor.icons.backspace,
              im.ImVec2(24, 24), im.ImVec4(0.9, 0.3, 0.3, 0.8),
              nil, nil, 'MultiSelectButton') then
              W.out.avedit = {}
  --                        W.undo()
              W.daeExport()
    --                    W.clear()
            end
            im.tooltip('Ctrl-Z')
    --                im.tooltip('Clean-up!')
          im.EndGroup()

  --            im.Unindent()
    end
    im.Columns(1)

		----------------------------------------------
		----------- MAT/MESH LIST --------------------
		----------------------------------------------
		if _dbgone then return end
		im.Dummy(im.ImVec2(0, 10))
		im.Dummy(im.ImVec2(2, 0))

		im.Indent(8)

		im.SameLine()

--        im.Columns(1, "mSelRow1", false)
--        im.Columns(1)
		im.PushItemWidth(-1)


		local meshType = nil
		-- build list
		local mlist
		for _,tp in pairs({'win','door','storefront','stairs','corner','stringcourse','balcony','pillar','pilaster','roofborder'}) do
			if W.ui.dbg then
				lo('?? ui_DBG:')
				W.ui.dbg = false
			end
			if W.ifForest({tp}) then
				-- build list of meshes
				mlist = W.forDAE(tp)
				if mlist and  #mlist > 0 and type(mlist[1]) == 'table' then
					for i = 1,#mlist do
						--TODO: handle better
						mlist[i] = mlist[i][1]
					end
				end
--                    U.dump(list, '??*********** for_list:'..tp..':'..tostring(#list[1])..':'..tostring(type(list[1])))
--                    _dbgone = true
				if mlist ~= nil then
					meshType = tp
				elseif not _dbgone then
					lo('!! ERR_ui.NO_DAE:'..tostring(tp))
					_dbgone = true
				end
				break
			end
		end
--            U.dump(list, '??+++++++++++++++++++++++++++++++++++++++++++++++ mlist:')
--            _dbgone = true
--[[
		if meshType ~= nil then
			-- use list of meshes, otherwise list of materials
			for i = 1,#list do
				list[i] = U.split(list[i], '/', true)
	--            lo('?? for_L:'..i..':'..#list[i])
				list[i] = list[i][#list[i] ]
			end
		end
]]
		if U._PRD == 0 and D.forRoad() then
			-- road materials
			list = {
				'road1_concrete',
				'road_asphalt_unmarked',
			}
--            list = W.dmat.road
		end

--            lo('?? for_list:'..list[1])
		if true and U._PRD == 1 then

--                U.dump(list)
			im.Text('Material')
--            im.Text(W.ifForest() and 'Mesh' or 'Material')
--          if 9 == W.out.curselect then
--            lo('?? ui.ucs:'..9)
--          end
			local selected = W.out.curselect or 0
			for i,s in pairs(list) do
				list[i] = tostring(list[i])
			end
--            table.insert(list, 1, 'aaa')

			local comboItems = im.ArrayCharPtrByTbl(list)
			if im.Combo1('', editor.getTempInt_NumberNumber(selected), comboItems, nil, nil) then

				local o = editor.getTempInt_NumberNumber()
					lo('?? UI.sel1:'..o..':'..tostring(W)..':'..tostring(meshType))
				local s = list[o+1]
				if false and meshType ~= nil then
					W.meshApply(meshType, o)
				elseif s then
					if D.forRoad() then
						D.matApply(tostring(s))
					else
						W.matApply(tostring(s), o)
					end
				end
			end

		else

			if im.BeginListBox('', im.ImVec2(-1, 160)) then --, im.ImVec2(333, 180), im.WindowFlags_ChildWindow) then
	--        if im.BeginListBox('', im.ImVec2(-1, 200)) then --, im.ImVec2(333, 180), im.WindowFlags_ChildWindow) then
	--        if im.BeginListBox('', im.ImVec2(333, 180), im.WindowFlags_ChildWindow) then
	--            im.Columns(6, "jctListBoxColumns", true)

		--            lo('?? FL:'..#list)
				-- render list
				for o,s in pairs(list) do
		--            lo('?? for_mat:'..tostring(s))
		--            im.TextColored(im.ImVec4(0.5, 0.5, 0.7, 1), tostring(s))
		--            im.SameLine()
					local selected = (o == W.out.curselect)
					if im.Selectable1(tostring(s), selected) then
							lo('?? UI.sel2:'..o..':'..tostring(W)..':'..tostring(meshType))
						if D.forRoad() then
							D.matApply(tostring(s))
						else
							W.matApply(tostring(s), o)
						end
--[[
						if meshType ~= nil then
							W.meshApply(meshType, o)
						else
							if D.forRoad() then
								D.matApply(tostring(s))
							else
								W.matApply(tostring(s), o)
							end
						end
]]
						return true
					end
				end
				im.EndListBox()
				im.PopItemWidth()
			end

		end

		im.Dummy(im.ImVec2(0, 2))
		im.Columns(3, 'UV', false)
			im.Text('UV u')
		im.NextColumn()
			im.Text('UV v')
		im.NextColumn()
			im.Text('Scale')
		im.NextColumn()
			slider(nil, 'uv_u', {-1,1})
		im.NextColumn()
			slider(nil, 'uv_v', {-1,1})
		im.NextColumn()
			slider(nil, 'uv_scale', {0.2,5})

		im.Columns(1)

		im.Unindent(8)

--[[
				'AsphaltRoad_damage_sml_decal_01',
				'asphalt_patches',
--                'ground_parts_decal',
				'm_dirty_ground',
				'nat_decals_rocks_01',
--                'a_asphalt_01_a',
]]
		im.Dummy(im.ImVec2(0, 2))
		im.Separator()

--        im.Dummy(im.ImVec2(0, 4))
		im.Dummy(im.ImVec2(0, 2))

--        im.Dummy(im.ImVec2(0, 0))
--        im.SameLine()
		if W.out.inedge then
			----------------------------------------------
			----------- BASE SHAPE SELCTOR ---------------
			----------------------------------------------
			local color = colorOn

			if editor.uiIconImageButton(editor.icons.check_box_outline_blank,
				im.ImVec2(34, 34), colorOn,
				nil, nil, 'MultiSelectButton') then
				W.baseSet('square')
			end
			im.tooltip('Square')

			im.SameLine()
			color = colorOff
			if editor.uiIconImageButton(editor.icons.data_usage,
				im.ImVec2(34, 34), color,
				nil, nil, 'MultiSelectButton') then
				if color == colorOn then
					W.baseSet('circle')
				end
			end
			im.tooltip('Circle')
		elseif W.forScope() == 'top' then
			----------------------------------------------
			----------- ROOF SHAPE SELCTOR ---------------
			----------------------------------------------
--        if W.out.scope == 'roof' then
			-- TODO: type selection color

--            im.Columns(2, 'topParts', false)
--            local roofTypeWidth = 400
--            im.SetColumnWidth(0, 00) --im.GetWindowWidth() - roofTypeWidth)
--            im.SetColumnWidth(1, 400) -- roofTypeWidth)

			columnRight(180)

--            local color = W.ifTopRect() and true and colorOn or colorOff
			local color = W.ifRoof('ridge') and colorOn or colorOff
--                    W.ifTopRect()
			im.SameLine()
			buttonC('roof_ridge', 'roof_ridge', color, im.ImVec2(30, 30))
			im.tooltip('Ridge toggle')

			im.SameLine()
			color = (desc and cij and desc.afloor[cij[1]].top.shape == 'flat' and not desc.afloor[cij[1]].top.cchild and colorOn)
        or colorOff
--            color = scope == 'top' and W.ifRoof('flat') and colorOn or colorOff
			buttonC('roof_border', 'roof_border', color, im.ImVec2(30, 30))
			im.tooltip('Border toggle')

			if U._PRD == 0 then
				im.SameLine()
				buttonC('roof_chimney', 'roof_chimney', colorOff, im.ImVec2(30, 30))
				im.tooltip('Chimney toggle')
			end

			im.NextColumn()

			color = im.ImVec4(0.7, 0.7, 0.75, 1) -- colorList
			for _,s in pairs({'flat','pyramid','shed','gable'}) do
				color = W.ifRoof(s) and im.ImVec4(0.7, 0.7, 0.75, 1) or colorOff
--                    color = im.ImVec4(0.7, 0.7, 0.75, 1)
				buttonCC('roof_'..s, 'roof_'..s, color, im.ImVec2(30, 30))
				im.tooltip(s)
				im.SameLine()
			end

			im.Columns(1)
--[[
			if false then

				local dim = im.ImVec2(30, 30)
				local cbg = nil
				local key = 'roof_flat'
				local lbl = key
				local c = nil

--                im.SameLine()
				im.BeginGroup()
				if editor.uiIconImageButton(editor.icons.forest_select, --editor.icons.forest_select,
				dim, im.ImVec4(0.0, 0.0, 0.0, 0), nil, cbg, 'MultiSelectButton') then
					if c == colorOff then return end
					env.onVal(key, lbl)
				end
				im.SameLine()
				im.Indent(-30)
				im.Unindent(-30)
				if dicon and dicon[lbl] then
					im.Image(dicon[lbl].tex:getID(),
						dim,
						aicon[lbl][1], aicon[lbl][2], c
					)
				end
				im.EndGroup()

				im.SameLine()
				im.BeginGroup()
				key = 'roof_gable'
				lbl = key
				if editor.uiIconImageButton(editor.icons.forest_select, --editor.icons.forest_select,
				dim, im.ImVec4(0.0, 0.0, 0.0, 0), nil, cbg, 'MultiSelectButton') then
					if c == colorOff then return end
					env.onVal(key, lbl)
				end
				im.SameLine()
				im.Indent(-30)
				im.Unindent(-30)
				if dicon and dicon[lbl] then
					im.Image(dicon[lbl].tex:getID(),
						dim,
						aicon[lbl][1], aicon[lbl][2], c
					)
				end
				im.EndGroup()

			else
--                for _,s in pairs({'flat', 'gable'}) do
			end
			if false then

				for _,s in pairs({'flat','pyramid','shed','gable'}) do
					if not scope then break end

					color = W.ifRoof(s) and colorOn or colorOff -- im.ImVec4(0.5, 0.7, 0.5, 0.5)
					im.SameLine()
					buttonC('roof_'..s, 'roof_'..s, color, im.ImVec2(30, 30))
					im.tooltip(s)
				end

			end

			local color = im.ImVec4(0.5, 0.7, 0.5, 1)
			if editor.uiIconImageButton(editor.icons.forest_select,
				im.ImVec2(34, 34), color,
				nil, nil, 'MultiSelectButton') then
				W.roofSet('flat')
			end
			im.tooltip('Flat')

			im.SameLine()
			local ifroof = W.ifRoof('pyramid')
			color = ifroof and im.ImVec4(0.5, 0.7, 0.5, 1) or im.ImVec4(0.5, 0.7, 0.5, 0.5)
			if editor.uiIconImageButton(editor.icons.forest_select,
				im.ImVec2(34, 34), color,
				nil, nil, 'MultiSelectButton') then
				if ifroof then W.roofSet('pyramid') end
			end
			im.tooltip('Pyramid')

			im.SameLine()
			ifroof = W.ifRoof('shed')
			color = ifroof and im.ImVec4(0.5, 0.7, 0.5, 1) or im.ImVec4(0.5, 0.7, 0.5, 0.5)
			if editor.uiIconImageButton(editor.icons.forest_select,
				im.ImVec2(34, 34), color,
				nil, nil, 'MultiSelectButton') then
				if ifroof then W.roofSet('shed') end
			end
			im.tooltip('Shed')

			im.SameLine()
			ifroof = W.ifRoof('gable')
			color = ifroof and im.ImVec4(0.5, 0.7, 0.5, 1) or im.ImVec4(0.5, 0.7, 0.5, 0.5)
			if editor.uiIconImageButton(editor.icons.forest_select,
				im.ImVec2(34, 34), color,
				nil, nil, 'MultiSelectButton') then
				if ifroof then W.roofSet('gable') end
			end
			im.tooltip('Gable')
]]
		else
			----------------------------------------------
			----------- ELEMENTS SELECTOR ----------------
			----------------------------------------------
			local rightPanelWidth = U._PRD == 0 and 180 or 90
			local color, bgcolor = nil, nil -- scope == 'wall' and im.ImVec4(0.5, 0.7, 0.5, 1) or im.ImVec4(0.5, 0.7, 0.5, 0.5)
--            im.SameLine()
--            im.Dummy(im.ImVec2(0, 0))
--            im.Dummy(im.ImVec2(0, 60))
--            im.SameLine()
--            im.Separator()
--            im.Dummy(im.ImVec2(-10, -10))
			im.Indent(10)
      im.PushFont3("cairo_regular_medium")
			im.Text('Elements')
      im.PopFont()
      im.PushFont3("segoeui_regular")
--			im.Text('Elements')
			im.Unindent(10)

			im.Dummy(im.ImVec2(10, -10))

			im.Columns(2, 'rParts', U._PRD == 0 and true or false)
	--            im.SetColumnWidth(1, 70)

				im.SetColumnWidth(0, im.GetWindowWidth() - rightPanelWidth)
				im.SetColumnWidth(1, rightPanelWidth)
	--            im.SetColumnWidth(2, 0)
	--            im.SetColumnWidth(1, 170)
	--            im.Columns(1, "eSelRow1", true)
        im.Dummy(im.ImVec2(2, 0))
        if true then
--				if not croad then
-------------------------
-- PARTS TYPES
-------------------------
--                    buttonC('window', 'win_toggle', colorOn, im.ImVec2(34, 34))

--[[
					color = colorOn --im.ImVec4(0.5, 0.7, 0.5, 1)
						color = im.ImVec4(0.5, 0.7, 0.5, 0)
					if editor.uiIconImageButton(editor.icons.web_asset, --icons.picture_in_picture, --.forest_select,
						im.ImVec2(34, 34), color,
						nil, bgcolor, 'MultiSelectButton') then
							W.windowsToggle()
						end
]]
					-- windows
					im.SameLine()
					color = (not scope) and colorOff or colorOn
					bgcolor = W.ifForest({'win'}) and bgActive or nil
					buttonC('window3', 'win_toggle', color, im.ImVec2(30, 30), bgcolor)
					im.tooltip('Window toggle')

					-- door
					im.SameLine()
					color = (not scope or scope == 'building') and colorOff or colorOn
					bgcolor = W.ifForest({'door'}) and bgActive or nil
					buttonC('opened_door', 'door_toggle', color, im.ImVec2(30, 30), bgcolor)
					im.tooltip('Door toggle')

					if U._PRD == 0 then
						-- storefront
						im.SameLine()
						color = (cij and cij[1] == 1) and colorOn or colorOff
						bgcolor = W.ifForest({'storefront'}) and bgActive or nil
						buttonC('storefront', 'storefront_toggle', color, im.ImVec2(30, 30), bgcolor)
						im.tooltip('Store front toggle')
					end

					im.SameLine()
					color = (not scope) and colorOff or colorOn
					bgcolor = W.ifForest({'balcony'}) and bgActive or nil
					buttonC('balcony', 'balc_toggle', color, im.ImVec2(30, 30), bgcolor)
					im.tooltip('Balcony toggle')

					im.SameLine()
					color = (not scope or scope == 'wall') and colorOff or colorOn
					bgcolor = W.ifForest({'corner'}) and bgActive or nil
					buttonC('corner', 'corner_toggle', color, im.ImVec2(30, 30), bgcolor)
					im.tooltip('Corner toggle')

					-- stringcourse
					if true or U._PRD == 0 then
						im.SameLine()
						color = scope and colorOn or colorOff --(not scope or scope == 'wall') and colorOff or colorOn
						bgcolor = W.ifForest({'stringcourse'}) and bgActive or nil
						buttonC('stringcourse', 'stringcourse_toggle', color, im.ImVec2(30, 30), bgcolor)
						im.tooltip('String course toggle')
					end

					im.SameLine()
					color = ({wall=1,floor=1})[W.forScope()] ~= nil and colorOn or colorOff
					bgcolor = W.ifForest({'pillar'}) and bgActive or nil
					buttonC('pillar', 'pillar_toggle', color, im.ImVec2(30, 30), bgcolor)
					im.tooltip('Pillar togglee')

					if U._PRD == 0 then
						-- pilaster
						im.SameLine()
						color = W.ifValid('pilaster') and colorOn or colorOff
	--                        color = ({wall=1,floor=1,building=1})[W.forScope()] ~= nil and colorOn or colorOff
						bgcolor = W.ifForest({'pilaster'}) and bgActive or nil
						buttonC('pilaster', 'pilaster_toggle', color, im.ImVec2(30, 30), bgcolor)
						im.tooltip('Pilaster toggle')
					end


					-- stairs
					im.SameLine()
					color = W.ifForest({'door','stairs'}) and colorOn or colorOff
					if not desc or not desc.basement or not desc.basement.yes or cij[1] ~= 2 then
						color = colorOff
					end
--                    color = (scope == 'building' and desc.basement and desc.basement.yes) and colorOn or colorOff
--                    color = (scope == 'wall' and cij and cij[1] == 1) and colorOn or colorOff
					bgcolor = W.ifForest({'stairs'}) and bgActive or nil
					buttonC('stairs', 'stairs_toggle', color, im.ImVec2(30, 30), bgcolor)
					im.tooltip('Stair toggle')


--                    color = not scope and colorOff or colorOn -- (not scope or scope == 'wall') and colorOff or colorOn
--                    button('Balcony toggle', 'balc_toggle', 'fg_barrier', color, W.ifForest({'balcony'}))


--[[
					color = colorOn
		--            color = (({wall=1})[scope] ~= nil or bgcolor ~= nil) and im.ImVec4(0.5, 0.7, 0.5, 1) or colorOff
					if editor.uiIconImageButton(editor.icons.forest_select,
						im.ImVec2(34, 34), color,
						nil, bgcolor, 'MultiSelectButton', nil, nil) then
		--                    lo('?? DOOR:'..scope)
						W.doorToggle()
						if scope == 'wall' then
						end
					end

					if true then
						-- balcony
						im.SameLine()
						color = not scope and colorOff or colorOn -- (not scope or scope == 'wall') and colorOff or colorOn
--                        bgcolor = W.ifForest({'corner'}) and bgActive or nil
--                            bgcolor = bgActive
						button('Balcony toggle', 'balc_toggle', 'fg_barrier', color, W.ifForest({'balcony'}))
					end

					if true or U._PRD == 0 then
						-- corners
						im.SameLine()
						color = (not scope or scope == 'wall') and colorOff or colorOn
--                        bgcolor = W.ifForest({'corner'}) and bgActive or nil
--                            bgcolor = bgActive
						button('Corner toggle', 'corner_toggle', 'details', color, W.ifForest({'corner'}))
					end

					if true or U._PRD == 0 then
					end
					-- pillars
					im.SameLine()
					color = ({wall=1,floor=1})[W.forScope()] ~= nil and im.ImVec4(0.5, 0.7, 0.5, 1) or colorOff
					bgcolor = nil
					if editor.uiIconImageButton(editor.icons.account_balance, --.forest_select,
						im.ImVec2(34, 34), color,
						nil, bgcolor, 'MultiSelectButton', nil, nil, function()
						end) then
							if ({wall=1,floor=1})[W.forScope()] then
			--                    lo('?? DOOR:')
								W.pillarToggle()
							end
					end
					im.tooltip('Pillar toggle')
]]




					--attick
					if false then
						im.SameLine()
			--            im.NextColumn()

						bgcolor = W.ifForest({'attick'}) and bgActive or nil
						color = W.ifValid('attic') and colorOn or colorOff -- ({top=1})[scope] ~= nil and colorOn or colorOff
			--            bgcolor = W.ifForest('cornice') and bgActive or nil
						if editor.uiIconImageButton(editor.icons.account_balance,
							im.ImVec2(34, 34), color,
							nil, bgcolor, 'MultiSelectButton', nil, nil, function()
							end) then
								if color == colorOn then
									W.atticToggle()
								end
						end
						im.tooltip('Attick toggle')
					end

				else
					-- placeholder
					im.InvisibleButton('ib_TYPES', im.ImVec2(34, 34))
				end

				im.NextColumn()
				local scapeColor = im.ImVec4(0.1, 0.6, 0.9, 0.8)
				if U._PRD == 0 then

	--                im.SameLine()
	--                im.Dummy(im.ImVec2(142, 0))
--                    im.SameLine()
					if false and U._PRD == 0 then
						color = (W.ui.injunction or W.out.D.out.injunction) and im.ImVec4(0.5, 0.7, 0.5, 1) or colorOff
						button('JAT', 'junction', 'twoRoadsCrossAdd', color)
						im.SameLine()

						color = scapeColor
						if editor.uiIconImageButton(editor.icons.roadRefPathDecal, --urbanRoad3To2Merge02, --.terrain_tools,
							im.ImVec2(30, 30), color,
							nil, bgcolor, 'MultiSelectButton') then
							W.onVal('nodes2roads')
						end
						im.tooltip('Roads')
						im.SameLine()
					end

					-- POPULATE
					bgcolor = R.out.inpave and bgActive or nil
					color = R.out.inpick and scapeColor or colorOff
					if editor.uiIconImageButton(editor.icons.domain,
						im.ImVec2(30, 30), color,
						nil, bgcolor, 'PopulateButton') then
							if color ~= colorOff then
									lo('?? ui.to_pop:'..tostring(R))
								W.out.avedit = {}
								handled = R.populate(D.out.across, D.out.adec, D, W)
									handled = true
							end
					end
					local title = 'Populate' -- (D.out.inconform or W.out.inconformm) and 'Restore' or 'Conform'
					im.tooltip(title)
	--                im.SameLine()
	--                im.NextColumn()
	--                im.Dummy(im.ImVec2(0, 0))
		--            im.Dummy(im.ImVec2(182, 0))
					-- CONFORM
					im.SameLine()
--[[
					bgcolor = (D.out.inconform or W.out.inconformm) and bgActive or nil
					if U._MODE == 'conf' and W.out.D.out.inconform then
						bgcolor = bgActive
					else
						bgcolor = nil
					end
					color = (W.out.D.forRoad() or W.out.D.out.inall or scope == 'building') and scapeColor or colorOff
--							color = scapeColor
					if false and editor.uiIconImageButton(editor.icons.terrain_tools,
						im.ImVec2(30, 30), color,
						nil, bgcolor, 'MultiSelectButton') then
							U.lo('?? ui.to_conform:'..tostring(W.out.D.forRoad()))
						W.out.avedit = {}
						if W.out.D.forRoad() or W.out.D.out.inall then --== 1 then
							if U._MODE == 'conf' then
								W.out.D.road2ter()
							elseif D.out.inconform then
		--??                        D.restore()
--								W.out.D.ter2road()
							else
--								W.out.D.ter2road()
							end
						elseif scope == 'building' then
							if W.out.inconform then
								W.restore()
							else
								local adsc = W.forSel()
								for _,dsc in pairs(adsc) do
									W.conform(dsc)
--                                    W.restore(true)
									if #adsc > 1 then
										W.confSave()
									end
								end
								if #adsc > 1 then
--                                    W.confSave()
								end
							end
						end
					end
					local title = 'Conform'
					if W.out.D.forRoad() then
						if W.out.D.out.inall then
							title = title..' all'
						else
							title = title..' road'
						end
					elseif scope == 'building' then
						title = title..' base'
					end
--                    title = title + (D.forRoad() and ' road' or ' base') -- (D.out.inconform or W.out.inconformm) and 'Restore' or 'Conform'
					im.tooltip(title)
]]
				end

				if false and U._MODE == 'conf' then
					if W.out.D.out.inconform then
						bgcolor = bgActive
					else
						bgcolor = nil
					end
					color = (W.out.D.forRoad() or W.out.D.out.inall or scope == 'building') and scapeColor or colorOff

--					color = (W.out.D.forRoad() or W.out.D.out.inall or scope == 'building') and scapeColor or colorOff
	--							color = scapeColor
					if editor.uiIconImageButton(editor.icons.terrain_tools,
						im.ImVec2(30, 30), color,
						nil, bgcolor, 'MultiSelectButton') then
							U.lo('?? ui.to_conform:'..tostring(W.out.D.forRoad()))
						W.out.avedit = {}
						if W.out.D.forRoad() or W.out.D.out.inall then --== 1 then
							if U._MODE == 'conf' then
								W.out.D.road2ter()
							elseif D.out.inconform then
		--??                        D.restore()
	--								W.out.D.ter2road()
							else
	--								W.out.D.ter2road()
							end
						elseif scope == 'building' then
							if W.out.inconform then
								W.restore()
							else
								local adsc = W.forSel()
								for _,dsc in pairs(adsc) do
									W.conform(dsc)
	--                                    W.restore(true)
									if #adsc > 1 then
										W.confSave()
									end
								end
								if #adsc > 1 then
	--                                    W.confSave()
								end
							end
						end
					end
					local title = 'Conform'
					if W.out.D.forRoad() then
						if W.out.D.out.inall then
							title = title..' all'
						else
							title = title..' road'
						end
					elseif scope == 'building' then
						title = title..' base'
					end
	--                    title = title + (D.forRoad() and ' road' or ' base') -- (D.out.inconform or W.out.inconformm) and 'Restore' or 'Conform'
					im.tooltip(title)
				end


				if false then
					-- balconies
					im.SameLine()
					color = im.ImVec4(0.5, 0.7, 0.5, 1)
					bgcolor = nil
					if editor.uiIconImageButton(editor.icons.forest_select,
						im.ImVec2(34, 34), color,
						nil, bgcolor, 'MultiSelectButton') then
							W.balconyToggle()
						end
					im.tooltip('Balcony toggle')
					-- cornice
					im.SameLine()
					bgcolor = W.ifForest({'roofborder'}) and bgActive or nil
					color = colorOn -- ({top=1})[scope] ~= nil and colorOn or colorOff
		--            bgcolor = W.ifForest('cornice') and bgActive or nil
					if editor.uiIconImageButton(editor.icons.account_balance,
						im.ImVec2(34, 34), color,
						nil, bgcolor, 'MultiSelectButton', nil, nil, function()
						end) then
							W.corniceToggle()
					end
					im.tooltip('Сornice toggle')
				end
			im.Columns(0)
-- MESHES
			im.Dummy(im.ImVec2(0, 10))
			im.Indent(8)
			if not croad and meshType and mlist then
				local selected = W.out.curmselect or 0
				for i,s in pairs(mlist) do
					mlist[i] = tostring(mlist[i])
				end
				for i = 1,#mlist do
					mlist[i] = U.split(mlist[i], '/', true)
					mlist[i] = mlist[i][#mlist[i]]
				end
				combo('Mesh', 'mesh_apply', mlist, selected)
--[[
				local comboItems = im.ArrayCharPtrByTbl(mlist)
				if im.Combo1('m', editor.getTempInt_NumberNumber(selected), comboItems, nil, nil) then
					local o = editor.getTempInt_NumberNumber()
						lo('?? UI.sel:'..o..':'..tostring(W)..':'..tostring(meshType))
					local s = mlist[o+1]
					W.meshApply(meshType, o)
				end
]]
			end
--            im.
--            im.Indent(-14)

		end


		--=========================================
		-- ELEMENTS CONTROLS
		--=========================================
		local desc = W.forDesc()
		local vspace = U._PRD == 1 and 480+60 or 570+60
--        local vspace = U._PRD == 1 and 320 or 460
		-- 320
		im.BeginChild1('CONTROLS', im.ImVec2(im.GetWindowWidth(), im.GetWindowHeight() - vspace), false)
--            im.Indent(8)
			if R.out.inpave then
				im.Columns(3, 'rSpacing', false)
				local margin = im.GetWindowWidth()/6
				local marginInCol = 12
				im.SetColumnWidth(0, margin)
				im.SetColumnWidth(1, im.GetWindowWidth() - 2*margin)
				im.SetColumnWidth(2, margin)

				im.NextColumn()
				im.Dummy(im.ImVec2(0, 6))
				im.Dummy(im.ImVec2(2, 0))
				im.SameLine()
				local v_spacing = im.FloatPtr(R.ui.spacing)
	--            if im.InputFloat("Spacing###", v_spacing, 0.1, 20.0) then
				im.PushItemWidth(im.GetWindowWidth() - 2*(margin + marginInCol + 40))
				if im.SliderFloat('Spacing', v_spacing, 0.1, 20.0, '%.1f') then
	--            if im.SliderFloat("", v_spacing, 0.1, 20.0, "Buildings Spacing = %.1f") then
						--                 R.ui.spacing = v_spacing[0]
					R.onSpacing(v_spacing[0])
				end
	--            im.Columns(1)
			else
--                local croad = D.forRoad()
				local mat = croad ~= nil and croad:getField("material", "") or nil
				if mat == nil or ({road_invisible = 1, WarningMaterial = 1})[mat] then mat = 'NONE' end
		--        if true then
				if croad and mat ~= 'NONE' then -- and D.out.inconform then
-- ROAD CONTROLS
		--                    lo('?? mat:'..tostring(croad:getField("material", "")))
		--            im.SameLine()

					-- road controls
					im.Dummy(im.ImVec2(0, 10))
					im.Dummy(im.ImVec2(2, 0))
					im.SameLine()
					im.BeginGroup()

						im.Columns(2, 'SSS', false) -- false - no borders
						im.SetColumnWidth(0, 175)
						im.SetColumnWidth(1, 175)

						local laneL,laneR = im.IntPtr(D.ui.laneL),im.IntPtr(D.ui.laneR)-- D.forLanes()
			--            im.IntPtr(D.laneL)
			--            local laneR = im.IntPtr(D.laneR)

						if im.InputInt('Lanes L', laneL, 1) then
							if laneL[0] < 0 then
								D.ui.laneL = 0
							else
								D.ui.laneL = laneL[0]
							end
							D.laneSet()
						end
			--            im.SameLine()
						im.NextColumn()
						if im.InputInt('Lanes R', laneR, 1) then
							if laneR[0] < 0 then
								D.ui.laneR = 0
							else
								D.ui.laneR = laneR[0]
							end
							D.laneSet()
						end
						-- middle params
						im.Columns(1)
						im.Dummy(im.ImVec2(0, 6))
						im.TextColored(colorNav, 'Middle Line:')
						local middleYellow = im.BoolPtr(D.ui.middleYellow)
						local middleDashed = im.BoolPtr(D.ui.middleDashed)
		--                local oldVal = roadMgr.middleYellow[0]
						im.SameLine()
						im.Dummy(im.ImVec2(12, 0))
						im.SameLine()
						if im.Checkbox("Yellow", middleYellow)  then
							D.ui.middleYellow = middleYellow[0] and true or false
							D.middleUp()
						end
						im.SameLine()
						im.Dummy(im.ImVec2(12, 0))
						im.SameLine()
						if not D.ui.middleYellow and im.Checkbox("Dashed", middleDashed) then
							D.ui.middleDashed = middleDashed[0] and true or false
							D.middleUp()
						end
					im.EndGroup()

		--                im.NextColumn()
		--            im.EndListBox()
				else
--                    im.Separator()
					im.Dummy(im.ImVec2(0, 0))
					im.Dummy(im.ImVec2(0, 0))
--                    im.Indent(-4)
--                    im.Indent(10)
-- BUILDING CONTROLS
					env = W
					if W.out.inhole and W.out.inhole.achild[#W.out.inhole.achild].body then
						slider('X', 'hole_x', {-5, 5})
						slider('Y', 'hole_y', {-5, 5})
						if U._PRD == 0 then
							slider('Ex/In-trude', 'hole_pull', {-5, 5})
							slider('Width', 'hole_w', {-5, 5})
							slider('Height', 'hole_h', {-5, 5})
						end
						buttonC('width_fit', 'hole_fitmesh', colorOn, im.ImVec2(30, 30))
						im.tooltip('Match mesh size')

--                        button('Match width', 'pillar_span', 'width_fit')
--                        im.tooltip('Match mesh size')
--                        buttonC('width_fit', 'hole_fitmesh', nil, im.ImVec2(30, 30))
					elseif cforest then
-- ELEMENTS CONTROLS
--                        im.Unindent(4)
--                        im.Dummy(im.ImVec2(20, 0))
						if false then
						elseif W.ifForest({'stringcourse'}) then
							slider('Bottom margin', 'stringcourse_bottom', {-0.5, 0.5})
						elseif W.ifForest({'pilaster'}) then
							im.Text('Positioning')
							im.Indent(-6)
							im.Columns(3, 'PILA_IND', true) -- false - no borders
--                            im.SetColumnWidth(0, 120)
--                            im.SetColumnWidth(1, 120)
--                            im.SetColumnWidth(2, 120)

							im.Text('From left corner')
							im.NextColumn()
							im.Text('Next spacing')
							im.NextColumn()
							if true or U._PRD == 0 then
								im.Text('Next spacing')
							end

							im.NextColumn()
							input('a', 'pilaster_ind0', true, nil, true)
							im.NextColumn()
							input('b', 'pilaster_ind1', true, nil, true)
							im.NextColumn()
							if true or U._PRD == 0 then
								input('c', 'pilaster_ind2', true, nil, true)
							end

							im.Columns(1)
						elseif U._PRD == 0 and W.ifForest({'corner'}) then
							check('Use inward corner', 'corner_inset')
						elseif W.ifForest({'balcony'}) then
--                                lo('?? for_BALC:')
--                            slider('Bottom margin', 'balc_bottom', {0.05, 5})
							slider('Bottom margin', 'balc_bottom', {-0.2, 5})
							slider('Scale width', 'balc_scale_width', {0.2, 5})

--                            columnRight(70)
--                            im.Columns(2, 'right_'..200, true)
--                            im.SetColumnWidth(0, im.GetWindowWidth() - width)
--                            im.SetColumnWidth(1, width)
							im.Dummy(im.ImVec2(0, 10))
							im.Text('Balcony spacing')
							im.Indent(-6)
							im.Columns(3, 'BALC_IND', true) -- false - no borders
--                            im.SetColumnWidth(0, 120)
--                            im.SetColumnWidth(1, 120)
--                            im.SetColumnWidth(2, 120)

							im.Text('From left corner')
							im.NextColumn()
							im.Text('Next spacing')
							im.NextColumn()
							im.Text('Next spacing')

							im.NextColumn()
							input('a', 'balc_ind0', true, nil, true)
							im.NextColumn()
							input('b', 'balc_ind1', true, nil, true)
							im.NextColumn()
							input('c', 'balc_ind2', true, nil, true)

							im.Columns(1)
							im.Dummy(im.ImVec2(0, 0))
							im.Dummy(im.ImVec2(0, 0))

						elseif W.ifForest({'win'}) then
--							im.Unindent(6)
--							columnRight(60)
							-- z-pos
							slider('Bottom margin', 'win_bottom', {0, 5})
							im.NextColumn()
							im.NextColumn()
              if W.out.fscope ~= 1 then
                -- left
                slider('Left margin', 'win_left', {0.0, 10})
                im.NextColumn()
                im.NextColumn()
                -- space
                slider('Spacing', 'win_space', {0.1, 10})
                im.NextColumn()
                im.NextColumn()
                -- scaling
                slider('Scale', 'win_scale', {0.2, 10})
              end

							if false and U._PRD == 0 then
								im.NextColumn()
								buttonC('width_fit', 'win_fitx', nil, im.ImVec2(30, 30))
	--                            button('Match width', 'pillar_span', 'compare_arrows')
								im.tooltip('Match width')
							end

--							im.Columns(0)
						elseif W.ifForest({'door'}) then
							input('Position', 'door_ind', true)
							slider('Bottom margin', 'door_bot', {0.1, 2})
--                            input('Bottom margin', 'door_bot')
						elseif W.ifForest({'pillar'}) then
							im.Unindent(6)

							columnRight(60)
--                            im.Unindent(10)
							slider('Spacing', 'pillar_space', {0.1, 10})
--                            im.Indent(10)
							im.NextColumn()
--                            check('Fill width', 'pillar_span')
							buttonCC('width_fit', 'pillar_span', nil, im.ImVec2(30, 30))
--                            buttonC('Fill width', 'pillar_span', nil, im.ImVec2(30, 30))
--                            button('Match width', 'pillar_span', 'compare_arrows')
							im.tooltip('Match width')

							im.NextColumn()
							slider('In-Out', 'pillar_inout', {-4, 4})

							im.NextColumn()

							im.NextColumn()
 --                           im.Unindent(10)
--                            im.Indent(10)
							input('Z-Span', 'pillar_spany', true)

							im.Columns(0)
						end
					elseif scope == 'building' then
						-- X-Y input + slider
--                        im.Columns(2, 'C_POS', false)

--                        slider('X', 'xpos', {-4000,4000})
						input('X', 'xpos')
--                        im.NextColumn()
						input('Y', 'ypos')
--                        im.Columns(0)
						input('Z', 'zpos', nil, 0.1)

						slider('X-Scale', 'building_scalex', {0.1, 10})
						slider('Y-Scale', 'building_scaley', {0.1, 10})

						slider('Rotation', 'building_ang', {-360, 360})
						input('Floors', 'n_floors', true)

--[[
						if im.TreeNodeEx1('Shape type', im.TreeNodeFlags_DefaultClosed) then
							im.SameLine()
							im.Text('CONT')
							im.TreePop()
						end
]]
				--        if im.TreeNode1('Building seed') then
--[[
							columnRight(174)
							im.Text('C1')

								for k,s in pairs({'b_shape', 'l_shape','t_shape','p_shape','s_shape'}) do
									local c = W.ui.building_shape == s and im.ImVec4(0.6, 1.0, 0, 0.6) or colorList
									buttonC(s, 'building_shape', c) -- 'tetromino') -- 't_shape')
									im.SameLine()
								end
								im.NextColumn()
								im.Text('C1')
--                                combo('Style', 'building_style', {'','residential','industrial'})
							im.Columns(0)
							im.Dummy(im.ImVec2(0, 12))

]]

						-- BASEMENT
--                        im.Dummy(im.ImVec2(0, 6))
--                        im.Separator()
						im.Dummy(im.ImVec2(0, 4))

						check('Basement', 'basement_toggle')
--						check('Basement toggle', 'basement_toggle')
						if false and desc.basement.yes then
							slider('Height', 'basement_height', {0,5})
							slider('In/out-set', 'basement_inout', {-2,5})
						end
--[[
						im.Columns(2) --, 'right_'..width)
						im.SetColumnWidth(0, im.GetWindowWidth()/2)
						im.SetColumnWidth(1, im.GetWindowWidth()/2)

						check('Basement toggle', 'basement_toggle')
						im.NextColumn()
						check('Basement stairs toggle', 'basement_stairs')

						im.Columns(0)
]]


--[[
						local laneL,laneR = im.IntPtr(D.ui.laneL),im.IntPtr(D.ui.laneR)
						if im.InputFloat('Lanes R', laneR, 1) then
							if laneR[0] < 0 then
								D.ui.laneR = 0
							else
								D.ui.laneR = laneR[0]
							end
						end
]]
						-- z-pos
						-- rotate
						-- n-floors
					elseif scope == 'floor' then
						-- height
						slider('Height', 'height_floor', {0.1, 10})
						slider('Ex/In-trude', 'floor_inout', {-10, 10})
--                        input('Height', 'height_floor')
						-- rotate
						slider('Rotation', 'ang_floor', {-180, 180})
						if U._PRD == 0 then
							input('Rotation fraction (of 2Pi)', 'ang_fraction', true)
						end
            im.Dummy(im.ImVec2(0, 4))
            im.Indent(conf.inputMargin)
            if cij[1] ~= 1 or not (desc.basement and desc.basement.yes) then
  						buttonT('Clone', 'floor_clone', im.ImVec4(0.9, 0.9, 0.9, 1), im.ImVec4(0.47, 0.45, 0.45, 1), nil, true) --, nil, im.ImVec2(67,47))
            end
            im.Unindent(conf.inputMargin)
--						buttonC('Clone', 'floor_clone', nil, im.ImVec2(67,47))
--                        if W.out.inaxis then
--                            input('Flip around', 'flip_axis', {-360, 360}, true)
--                        end
					elseif ({wall= 1, side=1})[scope] then
						-- extrusion
						slider('Ex/In-trude', 'wall_pull', {-10, 10})
--[[
						if im.TreeNode1('Patches') then
							input('Z-Span', 'wall_spany', true)
--                            input('X-Span', 'wall_spanx', true)
						end
]]
					elseif scope == 'top' then
						im.Indent(6)
							if W.ui.dbg  then
								lo('??^^^^^^^^^^^^^^ UI_dbg:'..tostring(desc)..':'..tostring(desc.afloor[#desc.afloor].top))
								W.ui.dbg = false
							end
						-- margin
						slider('Margins', 'top_margin', {0, 4})
						slider('Thickness', 'top_thick', {0, 1})
						if cij then
							local floor = desc.afloor[cij[1]]
							if cij[1] > 1 and (floor.top.achild and #floor.top.achild and floor.top.cchild) or W.out.inchild then
								slider('Height', 'child_height', {-2, 2})
							end
							local desctop = floor.top
	--                            desctop = desc.afloor[#desc.afloor].top
							if desc and cij and desc.afloor[cij[1]] and desc.afloor[cij[1]].top then
--                            if desc and desc.afloor[#desc.afloor].top then
								local hasridge = floor.top.ridge and floor.top.ridge.on
--                                if not floor.top
								if floor.top.cchild then
									desctop = floor.top.achild[floor.top.cchild] or desctop
								end
								local top,base = W.forTop()
								if top then

								if top.achild and #top.achild==1 then
									desctop = top.achild[1]
									base = desctop.base
				--                                        lo('?? for_DT:'..tostring(desctop.ridge.on)..':'..tostring(base))
								else
									desctop = top
								end
								if desctop.shape ~= 'flat' or hasridge or (desctop.ridge and desctop.ridge.on)  then -- or W.out.inridge then
					--                            if desc.afloor[#desc.afloor].top.shape ~= 'flat' or desc.afloor[#desc.afloor].top.ridge.on  then -- or W.out.inridge then
									-- slope
									slider('Tip height', 'top_tip', {0, 10})
								elseif cij and desc.afloor[cij[1]].top.border and desc.afloor[cij[1]].top.border.yes then
					--                                im.Text('BBDD')
									slider('Border width', 'roofborder_width', {0.01, 3})
									slider('Border height', 'roofborder_height', {0, 3})
									slider('Border in/out', 'roofborder_dy', {-3, 3})
									slider('Border lifting', 'roofborder_dz', {-3, 3})
								end
								im.Dummy(im.ImVec2(0, 6))
								if U._PRD == 0 then
									if desc.afloor[cij[1]].top.shape == 'gable' then
									-- ridge position
									slider('Ridge centering', 'top_ridge', {-3, 3})
									end
								end
								if desctop.ridge and desctop.ridge.on then -- and base and #base==4 then
				--                                if desc.afloor[cij[1]].top.ridge.on and base and #base==4 then
									if #floor.base == 4 then
									check('Flat top toggle', 'ridge_flat')
									slider('Ridge thickness', 'ridge_width', {0,75})
									end
									hasridge = true
					--                                    im.Text('RIDGE')
								end
								local canRotate = top and (({gable=1, shed=1})[top.shape] or hasridge)
								local color = canRotate and im.ImVec4(0.7, 0.7, 0.75, 1) or colorOff
					--                            button('Rotate', 'ang_top', 'rotate_left', color)
								button('Rotate', 'ang_top', 'rotate_left', color)

								end

--[[
								local top,base = W.forTop()
--                                if #top.achild == 1 then top = top.achild
								if top.achild and #top.achild==1 then
									desctop = top.achild[1]
									base = desctop.base
--                                        lo('?? for_DT:'..tostring(desctop.ridge.on)..':'..tostring(base))
								else
									desctop = top
								end
]]
--                                if top and (({gable=1, shed=1})[top.shape] or hasridge) then
--                                    button('Rotate', 'ang_top', 'rotate_left', color)
	--                                input('Rotation', 'ang_top', true)
	--                                slider('Rotation', 'ang_top', {-180, 180}, true)
--                                end
							end
						end
					else
						if U._PRD == 0 then
--							if U._MODE == 'conf' and W.out.D.out.inconform then
							if false and (W.out.D.forRoad() or W.out.D.out.inall) then
--								slider('Junction width', 'conf_jwidth', {1,8})
								slider('Margin', 'conf_margin', {1,20})
								slider('Max slope', 'conf_mslope', {2,45})
								slider('Banking rate', 'conf_bank', {0,5})
								W.ui.conf_all = W.out.D.out.inall or false
								im.Dummy(im.ImVec2(0, 6))
								check('For all', 'conf_all')
							elseif W.ui.injunction then
								input('Branches', 'branch_n', true)
								slider('Exit radius', 'exit_r', {5, 50})
								slider('Exit width', 'exit_w', {2, 10})
--								im.Dummy(im.ImVec2(0, 6))
								check('Roundabout', 'junction_round')
								slider('Radius', 'round_r', {20/2, 20*3})
								slider('Width', 'round_w', {4, 10})
							else
								slider('Radius', 'node_r', {40,200})
								slider('Amount', 'node_n', {3,30}, true)
							end
						end
					end
					if U._PRD == 0 and W.out.inedge then
						if im.TreeNode1('Patches') then
							input('Z-Span', 'wall_spany', true)
--                            input('X-Span', 'wall_spanx', true)
							im.TreePop()
						end
						if im.TreeNode1('Fringe') then
							slider('Height', 'fringe_height', {-5,5})
							slider('In-Out', 'fringe_inout', {-5,5})
--                            slider('Margin', 'fringe_margin', {-3,3})
							im.TreePop()
						end
					end
				end

			end

		im.EndChild()


		----------------------------------------------
		----------- HELP -----------------------------
		----------------------------------------------
		im.Indent(10)
		im.Columns(1)

--        im.Dummy(im.ImVec2(0, 26))
		im.TextColored(colorNav, 'Hint') --..tableSize(HTree[3]))

		im.Unindent(4)

--        im.SameLine()
--        im.Dummy(im.ImVec2(12, 10))
--        im.SameLine()

		M.tree2ui(HTree)
--[[
		local padding = 2
		for i,row in pairs(HTree) do
--            im.Indent(10)
			if tableSize(row) > 0 then
--                im.SameLine()
				if i == 1 then
					im.Dummy(im.ImVec2(0, 0))
				else
					im.Dummy(im.ImVec2(0, 24))
				end

				im.Separator()
				im.Dummy(im.ImVec2(0, 0))
				im.Dummy(im.ImVec2(0, 0))   -- add vertical space of 1pt

				local nextline = false
				for _,c in pairs(row) do
					if _ == 1 then
--                        im.SameLine()
--                        im.Dummy(im.ImVec2(0, 30))
--                        im.SameLine()
					end
					local k,v = c[1],c[2]
					if k ~= 'NEXT' then
						if not nextline then
							im.SameLine()
						else
							im.Unindent(-6)
							im.Text('')
							im.Dummy(im.ImVec2(0, 0))
						end
	--                    im.Dummy(im.ImVec2(0, 4))
	--                    im.SameLine()
						local sz = im.CalcTextSize(k)
						local cur = im.GetCursorPos()
						local topLeft = im.GetWindowPos()
--                            lo('?? for_pos:'..topLeft.x..':'..topLeft.y)
--                            topLeft = im.ImVec2(-461,377)
						topLeft.x = topLeft.x + cur.x - padding
						topLeft.y = topLeft.y + cur.y
--                        local color = im.GetColorU322(im.ImVec4(1.0, 0.0, 0.0, 1.0))
						local color = im.GetColorU321(im.Col_FrameBg)
						local bottomRight = im.ImVec2(topLeft.x + sz.x + 2*padding, topLeft.y + 20)
						im.ImDrawList_AddRectFilled(im.GetWindowDrawList(),
							im.ImVec2(topLeft.x, topLeft.y), bottomRight,
--                            topLeft, bottomRight,
							color)--, 2, nil, 2)
						im.Text(k)
						im.SameLine()
						im.Text(v)

						if not nextline then
							im.SameLine()
						else
							nextline = false
						end

						im.SameLine()
						im.Dummy(im.ImVec2(4, 0))
						im.SameLine()
					else
						nextline = true
					end
--                    im.Dummy(im.ImVec2(8, 0))

--                    im.SameLine()

	--                im.SameLine()
	--                im.Dummy(im.ImVec2(0, 0))
	--                im.SameLine()
				end
	--            im.SameLine()
--                im.Dummy(im.ImVec2(0, 0))
			end
--        im.Dummy(im.ImVec2(0, 10))
--            im.SameLine()

--            im.Dummy(im.ImVec2(0, 2))
--            im.Separator()
--            im.Dummy(im.ImVec2(0, 2))

--            im.Text("Clients:")
--            im.Text("Clients:"..tostring(im.GetCursorPos().x)..':'..tostring(im.GetCursorPos().y))
		end
]]

	end

--        im.Separator()
--        im.Text("Clients")
--[[
		im.Columns(1)
		im.Dummy(im.ImVec2(0, 20))
--        im.Separator()
		local colorHelp = im.ImVec4(0.7, 0.7, 0.1, 0.6)
		im.Dummy(im.ImVec2(0, 6))
		im.Dummy(im.ImVec2(2, 0))
		im.SameLine()
		im.TextColored(colorNav, 'Hint:')
		im.SameLine()
		im.Dummy(im.ImVec2(3, 0))
		im.SameLine()
--        im.TextColored(colorHelp, 'ALT-TAP: new   TAP: select')
		im.TextColored(colorHelp, 'ALT-CLICK: new  SHIFT-DRAG: turn  CTRL-DRAG: around')
--        im.TextColored(colorHelp, 'ALT-TAP: new   TAP: select  SHIFT-DRAG: turn  CTRL-DRAG: around')
--        im.SameLine()
--        im.TextColored(colorHelp, 'Alt-Click: new')
		-- scope
		im.TextColored(colorHelp, ' ')
		im.SameLine()
		if D.forRoad() then
			im.TextColored(colorHelp, 'UP/DOWN: road width  LEFT/RIGHT: mantle width')
		elseif W.ifForest() then
			if W.ifForest({'win'}) then
				im.TextColored(colorHelp, 'DRAG: up/down  ALT-WHEEL: space  CTRL-WHEEL: margin')
			end
			if W.ifForest({'door'}) then
				im.TextColored(colorHelp, 'RIGHT/LEFT: position')
			end
		elseif W.out.scope == 'building' then
			im.TextColored(colorHelp, 'CTRL-UP/DOWN: add/remove floor')
		elseif W.out.scope == 'floor' then
			if editor.keyModifiers.alt then
				im.TextColored(colorHelp, 'WHEEL: height')
			elseif editor.keyModifiers.ctrl then
				im.TextColored(colorHelp, 'WHEEL: in/out-set  UP: clone floor')
			else
				im.TextColored(colorHelp, 'CTRL  ALT  DRAG')
			end

--            im.TextColored(colorHelp, 'Ctrl-Up: clone  Ctrl-Wheel: in/out-set  Alt-Wheel: height  Drag:')
		elseif W.out.scope == 'wall' then
			if editor.keyModifiers.alt then
				im.TextColored(colorHelp, 'CLICK: split  WHEEL: in/out-set')
			else
				im.TextColored(colorHelp, 'ALT')
			end
		elseif W.out.scope == 'top' then
			if editor.keyModifiers.alt then
				im.TextColored(colorHelp, 'CLICK: split  WHEEL: slope angle  L/R: turn')
				im.Dummy(im.ImVec2(3, 0))
				im.SameLine()
				im.TextColored(colorHelp, 'Z: spot select, Z+WHEEL: spot resize')
			elseif editor.keyModifiers.ctrl then
				im.TextColored(colorHelp, 'WHEEL: margin size')
			else
				im.TextColored(colorHelp, 'CTRL  ALT')
			end
		else
			im.TextColored(colorHelp, ' ')
		end
]]

--        local spacing = im.ImVec2(500, 20)
--        im.Dummy(spacing)

--    editor.endWindow()
--    editor.showWindow("BAT")

--[[
	if false and editMode then
--        lo('?? for_HELP:')
		SCNow = {}
		local scope = W.forScope()

		SCNow['Ctrl-Click'] = 'new building'..'::'..tostring(scope) --..':'..tostring(W.out.scope)
		if scope then
			if U._PRD == 0 then
				SCNow['Ctrl-Drag'] = 'fly around'
			end
			SCNow['Shift-Drag'] = 'rotate'
			if ({wall=1, floor=1, building=1})[scope] then
				SCNow['Alt-Click'] = 'split wall'
			elseif scope == 'side' then
				SCNow['Alt-Click'] = 'split side'
			elseif scope == 'top' then
				SCNow['Alt-Click'] = 'split roof'
			end
		end

		local toupdate = false
		for k,v in pairs(SCNow) do
			if SCPre[k] ~= SCNow[k] then
				toupdate = true
				break
			end
		end
		if toupdate then
				lo('?? SC_UPDATE:'..tostring(W.forScope()))
			toupdate = false
			SCPre = SCNow
			editMode.auxShortcuts = SCNow
--            SCLegend.onEditorEditModeChanged()
			extensions.hook("onEditorEditModeChanged", nil, nil)
		end

	end
]]
--[[
		editMode.auxShortcuts = {}
		editMode.auxShortcuts['Alt-Click'] = 'new building'..'::'..tostring(W.out.scope)
--        editMode.auxShortcuts[editor.AuxControl_Copy] = nil -- "Copy objects"
		if W.forScope() then
			editMode.auxShortcuts['Ctrl-Drag'] = 'fly around'
			editMode.auxShortcuts['Shift-Drag'] = 'rotate'
		end
]]
--    editor.editModes.cityEditMode.auxShortcuts['Alt-Click'] = 'new building'


	return handled
end


M.tree2ui = function(htree)
	if not htree then return end

	local padding = 2
	for i,row in pairs(htree) do
--            im.Indent(10)
		if tableSize(row) > 0 then
--                im.SameLine()
			if i == 1 then
				im.Dummy(im.ImVec2(0, 0))
			else
				im.Dummy(im.ImVec2(0, 24))
			end

			im.Separator()
			im.Dummy(im.ImVec2(0, 0))
			im.Dummy(im.ImVec2(0, 0))   -- add vertical space of 1pt

			local nextline = false
			for _,c in pairs(row) do
				if _ == 1 then
--                        im.SameLine()
--                        im.Dummy(im.ImVec2(0, 30))
--                        im.SameLine()
				end
				local k,v = c[1],c[2]
				if k ~= 'NEXT' then
					if not nextline then
						im.SameLine()
					else
						im.Unindent(-6)
						im.Text('')
						im.Dummy(im.ImVec2(0, 0))
					end
--                    im.Dummy(im.ImVec2(0, 4))
--                    im.SameLine()
					local sz = im.CalcTextSize(k)
					local cur = im.GetCursorPos()
					local topLeft = im.GetWindowPos()
--                            lo('?? for_pos:'..topLeft.x..':'..topLeft.y)
--                            topLeft = im.ImVec2(-461,377)
					topLeft.x = topLeft.x + cur.x - padding
					topLeft.y = topLeft.y + cur.y
--                        local color = im.GetColorU322(im.ImVec4(1.0, 0.0, 0.0, 1.0))
					local color = im.GetColorU321(im.Col_FrameBg)
					local bottomRight = im.ImVec2(topLeft.x + sz.x + 2*padding, topLeft.y + 20)
					im.ImDrawList_AddRectFilled(im.GetWindowDrawList(),
						im.ImVec2(topLeft.x, topLeft.y), bottomRight,
--                            topLeft, bottomRight,
						color)--, 2, nil, 2)
					im.Text(k)
					im.SameLine()
					im.Text(v)

					if not nextline then
						im.SameLine()
					else
						nextline = false
					end

					im.SameLine()
					im.Dummy(im.ImVec2(4, 0))
					im.SameLine()
				else
					nextline = true
				end
--                    im.Dummy(im.ImVec2(8, 0))

--                    im.SameLine()

--                im.SameLine()
--                im.Dummy(im.ImVec2(0, 0))
--                im.SameLine()
			end
--            im.SameLine()
--                im.Dummy(im.ImVec2(0, 0))
		end
--        im.Dummy(im.ImVec2(0, 10))
--            im.SameLine()

--            im.Dummy(im.ImVec2(0, 2))
--            im.Separator()
--            im.Dummy(im.ImVec2(0, 2))

--            im.Text("Clients:")
--            im.Text("Clients:"..tostring(im.GetCursorPos().x)..':'..tostring(im.GetCursorPos().y))
	end

end


local function hint(emode)
--		lo('>> hint:'..tostring(emode))
	editMode = emode
--    editMode.auxShortcuts = {}
--    SCLegend.onEditorEditModeChanged()
end

--[[
	if editor.beginWindow('Materials', 'Materials') then
		im.BeginListBox('', im.ImVec2(233, 180), im.WindowFlags_ChildWindow)
		im.EndListBox()
	end
	editor.endWindow()
	editor.showWindow('Materials')
]]
-- control()
--[[
			local colText = W.isHidden() and im.ImVec4(1, 0.4, 0.4, 1) or im.ImVec4(1, 0.4, 0.4, 0.4)
			local t, i = 'Hidden', 6
			local width = im.CalcTextSize(t).x
			if im.InvisibleButton('ib'..i, im.ImVec2(width + 5, 20)) then
				scope = string.lower(t)
				W.selectionHide()
			end
			im.SameLine()
			im.Indent(-30)
			im.Unindent(-30)
			im.TextColored(colText, t)

]]
--        local spacing = im.ImVec2(500, 20)


M.control = control
M.hint = hint

return M

--[[
			if false then
				im.BeginGroup()
				if editor.uiIconImageButton(editor.icons.ab_asset_jbeam,
					im.ImVec2(10, 10), im.ImVec4(0.5, 0.7, 0.5, 1),
					nil, nil, 'b_scope_'..i,
					colText, nil) then
	--            if im.Button(t) then
	--                lo('?? pressed:'..i..':'..t)
					W.scopeOn(string.lower(t))
				end
				im.SameLine()
				im.Indent(-10)
	--            im.Space(20)
	--            im.Dummy(im.ImVec2(10, 0))
	--            im.SameLine()
				im.EndGroup()
			end
]]