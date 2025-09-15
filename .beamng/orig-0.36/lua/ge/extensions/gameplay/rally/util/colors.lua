-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local M = {}

--
-- colors
--
M.clr_white       = {1.0, 1.0, 1.0}
M.clr_black       = {0.0, 0.0, 0.0}

M.clr_green       = {0.0, 1.0, 0.0}
M.clr_green_dark  = {0.0, 0.5, 0.0}
M.clr_light_green = {0.5, 1.0, 0.5}
M.clr_lime_green  = {0.66, 1.0, 0.0}
M.clr_green_pastel = {0.718, 1, 0.455}

M.clr_red         = {1.0, 0.0, 0.0}
M.clr_red_dark    = {0.5, 0.0, 0.0}
M.clr_light_red   = {1.0, 0.5, 0.5}
M.clr_red_pastel = {1, 0.494, 0.635}

M.clr_teal        = {0.0, 0.76, 0.55}
M.clr_teal_2      = {0.0,  1.0, 0.58}

M.clr_yellow      = {1.0, 1.0, 0.0}
M.clr_blue        = {0.0, 0.0, 1.0}
M.clr_blue_light  = {0.392,0.612,1}

M.clr_orange      = {1.0, 0.64, 0.0}
M.clr_pink        = {1.0, 0.0, 1.0}
M.clr_aqua        = {0.0, 0.8, 0.8}

M.clr_grey         = {0.5, 0.5, 0.5}
M.clr_grey_light   = {0.75, 0.75, 0.75}
M.clr_grey_light_2 = {0.8, 0.8, 0.8}
M.clr_grey_dark    = {0.2, 0.2, 0.2}

M.clr_purple       = {0.73, 0, 1}

M.clr_error    = im.ImVec4(1, 0.2, 0.2, 1)
M.clr_no_error = im.ImVec4(0.2, 1, 0.2, 1)

------------------------------------------------------------------------------

--
-- snaproads.lua
--
M.snaproads_radius = 0.7
M.snaproads_radius_recce = 0.5
M.snaproads_radius_driving = 0.2
M.snaproads_alpha = 0.8
M.snaproads_alpha_driving = 0.2
-- M.snaproads_clr = M.clr_red
M.snaproads_clr_recce = M.clr_red
M.snaproads_clr_route = M.clr_orange

M.recce_alpha = M.snaproads_alpha
M.recce_driveline_clr = M.snaproads_clr_recce
M.recce_cut_clr = M.clr_teal

M.pacenote_adjacent_radius_factor = 0.5

M.cam_last_pos_radius = M.snaproads_radius * 2
M.cam_last_pos_alpha = M.snaproads_alpha

local clr_at = M.clr_blue
local clr_at_adjacent = M.clr_blue_light
local clr_di = M.clr_orange
local clr_label_fg = M.clr_white
local clr_label_bg = M.clr_black

--
-- pacenote.lua
--
-- M.pacenote_alpha_shape_normal = 0.25
-- M.pacenote_alpha_txt_normal = 0.5
--
-- M.pacenote_alpha_shape_selected = 0.8
-- M.pacenote_alpha_txt_selected = 1.0
--
-- M.pacenote_alpha_shape_previous = 0.25
-- M.pacenote_alpha_txt_previous = 0.5

M.pacenote_alpha_interlink = 0.5

M.pacenote_shapeAlpha_factor = 0.8

M.pacenote_base_alpha_background = 0.5
M.pacenote_base_alpha_no_sel = 0.5

M.pacenote_base_alpha_prev   = 0.5
M.pacenote_base_alpha_next   = 0.5
M.pacenote_base_alpha_selected = 0.5

M.pacenote_alpha_text_selected = 1.0

M.pacenote_alpha_recce = 0.1

M.pacenote_linkHeightRadiusShinkFactor = 0.5
M.pacenote_linkFromWidth = 1.5
M.pacenote_linkToWidth = 0.25

M.pacenote_clr_interlink = M.clr_pink
M.pacenote_clr_interlink_txt = M.clr_black
M.pacenote_clr_cs_to_ce_direct = M.clr_grey_light
M.pacenote_clr_di = clr_di
M.pacenote_clr_di_txt = M.clr_black
M.pacenote_clr_link_fg = clr_label_fg
M.pacenote_clr_link_bg = clr_label_bg

--
-- pacenoteWaypoint.lua
--
M.waypoint_clr_cs = M.clr_green
M.waypoint_clr_cs_adjacent = M.clr_green_pastel
M.waypoint_clr_ce = M.clr_red
M.waypoint_clr_ce_adjacent = M.clr_red_pastel
M.waypoint_clr_di = clr_di
M.waypoint_clr_at = clr_at
M.waypoint_clr_at_adjacent = clr_at_adjacent
M.waypoint_clr_txt_fg = clr_label_fg
M.waypoint_clr_txt_bg = clr_label_bg
M.waypoint_clr_sphere_hover = M.clr_white
M.waypoint_clr_sphere_selected = M.clr_white
M.waypoint_clr_background = M.clr_grey_dark

M.waypoint_alpha_text_background = 0.3
M.waypoint_alpha_shape_background = 0.3
M.waypoint_alpha_selected = 0.9

M.waypoint_shapeAlpha_hover = 0.95
M.waypoint_textAlpha_hover = 1.0
M.waypoint_shapeAlpha_intersectPlane = 0.97
-- M.waypoint_sphereAlphaReducionFactor = 0.6
M.waypoint_planeAlphaFactor = 3.2
-- M.waypoint_shapeAlpha_arrowAdjustFactor = 1.25
-- M.waypoint_intersectPlaneAlphaReductionFactor = 0.66

--
-- rallyEditor/pacenotes.lua
--
M.new_pacenote_cursor_clr_link = M.clr_white
M.new_pacenote_cursor_clr_cs = M.clr_light_green
M.new_pacenote_cursor_clr_ce = M.clr_light_red
M.new_pacenote_cursor_alpha = 0.8
M.new_pacenote_cursor_linkHeightRadiusShinkFactor = 0.5
M.new_pacenote_cursor_linkFromWidth = 1.0
M.new_pacenote_cursor_linkToWidth = 0.25

-- M.segments_alpha = 0.5
-- M.segments_clr_assigned = M.clr_white
-- M.segments_clr = M.clr_aqua

return M
