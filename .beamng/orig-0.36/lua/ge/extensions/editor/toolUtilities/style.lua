-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This file contains the style settings for various spline-based tools.

local M = {}

-- Module constants.
local im = ui_imgui


-- Gets the UI style properties.
local function getStyle()
  return {
    -- Colours.
    color_mousePos          = color(130, 40, 130, 128),     -- The color of the mouse position sphere (cursor).
    color_node              = color(30, 15, 200, 255),      -- The color of the spline node spheres.
    color_rib_handle        = color(180, 130, 125, 255),    -- The color of the rib handle spheres.
    color_bar_handle        = color(180, 130, 125, 255),    -- The color of the bar handle spheres.
    color_highlight         = color(130, 130, 130, 128),    -- The color of the sphere highlight around a selected node, rib, etc.

    color_spline            = color(0, 30, 70, 255),        -- The color of the spline line segments when selected.
    color_spline_dull       = color(100, 100, 110, 64),     -- The color of the spline line segments when not selected.
    color_spline_linked     = color(207, 29, 29, 100),      -- The color of the linked spline line segments.
    color_node_dull         = color(30, 90, 240, 200),      -- The color of the spline node spheres when not selected.
    color_layer_wire        = color(255, 255, 255, 180),    -- The color of the wire frame (eg for layers).
    color_rib_line          = color(80, 130, 200, 255),     -- The color of the rib lines (the line segment between the two rib handles).
    color_bar_line          = color(200, 130, 80, 255),     -- The color of the bar lines (the line segment between the bar and the ground).
    color_ground            = color(0, 30, 70, 255),        -- The color of the ground line when selected (for 3D splines).
    color_ground_dull       = color(100, 100, 110, 64),     -- The color of the ground line when not selected (for 3D splines).
    color_drop              = color(150, 150, 160, 180),    -- The color of the drop line when selected (for 3D splines).
    color_drop_thicker      = color(235, 235, 255, 220),    -- The color of thicker drop lines when selected (for 3D splines).
    color_drop_dull         = color(100, 100, 110, 64),     -- The color of the drop line when not selected (for 3D splines).
    color_normal            = color(180, 180, 180, 100),    -- The color of the normal lines.
    color_ref_normal        = color(130, 130, 130, 100),    -- The color of the reference normal lines.
    color_arc_seg           = color(255, 255, 0, 100),      -- The color of the arc segments.

    color_not_selected_surf = color(150, 150, 150, 150),    -- The color of surfaces which are not selected.
    color_active_surf       = color(200, 200, 220, 255),    -- The color of an 'active' surface.

    color_graph_clear_node  = color(140, 230, 150, 160),    -- Colours for the nav graph visualisation.
    color_nav_graph_node    = color(100, 120, 100, 200),
    color_path_node         = color(80, 250, 123, 255),
    color_path_node_big     = color(80, 250, 123, 255),
    color_nav               = color(50, 50, 50, 255),
    color_path              = color(152, 255, 152, 255),

    text_foreground         = color(0, 0, 0, 255),          -- The text foreground color.
    text_background         = color(255, 255, 255, 255),    -- The text background color.

    -- Line thicknesses.
    spline_thickness        = 7,      -- The thickness of the spline line segments when selected.
    spline_thickness_dull   = 3,      -- The thickness of the spline line segments when not selected.
    rib_thickness           = 4,      -- The thickness of the rib lines.
    bar_thickness           = 4,      -- The thickness of the bar lines.
    wire_thickness          = 4,      -- The thickness of the wire frame.
    active_seg_thickness    = 10,     -- The thickness of the active segment.
    ground_thickness        = 5,      -- The thickness of the ground line when selected (for 3D splines).
    ground_thickness_dull   = 3,      -- The thickness of the ground line when not selected (for 3D splines).
    drop_thickness          = 2,      -- The thickness of the drop line when selected (for 3D splines).
    drop_thickness_thicker  = 5,      -- The thickness of the drop line when selected (for 3D splines).
    drop_thickness_dull     = 1,      -- The thickness of the drop line when not selected (for 3D splines).
    normal_thickness        = 4,      -- The thickness of the normal lines.
    normal_ref_thickness    = 2,      -- The thickness of the reference normal lines.
    arc_seg_thickness       = 2,      -- The thickness of the arc segments.

    -- Sphere scale factors.
    sphere_mousePos         = 0.1,    -- The scale factor of the mouse position sphere.
    sphere_node             = 0.15,   -- The scale factor of the spline node spheres when selected.
    big_node                = 0.3,    -- The scale factor of the big graph path node spheres.
    sphere_node_dull        = 0.15,   -- The scale factor of the spline node spheres when not selected.
    sphere_node_hover       = 0.3,    -- The scale factor of the spline node sphere highlight.
    sphere_rib              = 0.1,    -- The scale factor of the rib spheres.
    sphere_bar              = 0.1,    -- The scale factor of the bar spheres.
  }
end

-- Blue Rose.
local function getBlueRose()
  return {
    fullWhite = im.ImVec4(0.95, 1.0, 0.95, 0.8),
    dullWhite = im.ImVec4(0.6, 0.8, 0.6, 0.3),
    purpleB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    greenB = im.ImVec4(0.35, 0.75, 0.35, 1.0),
    greenD = im.ImVec4(0.2, 0.45, 0.2, 1.0),
    blueB = im.ImVec4(0.3, 0.6, 0.8, 1.0),
    blueD = im.ImVec4(0.2, 0.4, 0.5, 1.0),
    redB = im.ImVec4(0.6, 0.2, 0.2, 1.0),
    yellowB = im.ImVec4(0.4, 1.0, 0.4, 1.0),
  }
end

-- Vapour Static.
local function getVapourStatic()
  return {
    fullWhite = im.ImVec4(1.0, 1.0, 1.0, 0.7),
    dullWhite = im.ImVec4(1.0, 1.0, 1.0, 0.25),
    purpleB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    blueB = im.ImVec4(0.29, 0.62, 0.64, 1.0),
    blueD = im.ImVec4(0.18, 0.36, 0.38, 1.0),
    greenB = im.ImVec4(0.91, 0.45, 0.65, 1.0),
    greenD = im.ImVec4(0.50, 0.23, 0.36, 1.0),
    redB = im.ImVec4(0.76, 0.17, 0.13, 1.0),
    yellowB = im.ImVec4(0.4, 1.0, 0.4, 1.0),
  }
end

-- Canyon Clay.
local function getCanyonClay()
  return {
    fullWhite = im.ImVec4(0.98, 0.95, 0.9, 0.7),
    dullWhite = im.ImVec4(0.7, 0.6, 0.5, 0.3),
    purpleB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    greenB = im.ImVec4(0.45, 0.65, 0.4, 1.0),
    greenD = im.ImVec4(0.25, 0.35, 0.2, 1.0),
    blueB = im.ImVec4(0.3, 0.5, 0.6, 1.0),
    blueD = im.ImVec4(0.2, 0.3, 0.4, 1.0),
    redB = im.ImVec4(0.75, 0.35, 0.2, 1.0),
    yellowB = im.ImVec4(0.4, 1.0, 0.4, 1.0),
  }
end

-- Terminal Aurora.
local function getTerminalAurora()
  return {
    fullWhite = im.ImVec4(0.9, 0.9, 0.95, 0.7),
    dullWhite = im.ImVec4(0.6, 0.6, 0.65, 0.25),
    purpleB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    blueB = im.ImVec4(0.2, 0.9, 0.75, 1.0),
    blueD = im.ImVec4(0.1, 0.5, 0.4, 1.0),
    greenB = im.ImVec4(0.35, 0.6, 0.3, 1.0),
    greenD = im.ImVec4(0.2, 0.35, 0.2, 1.0),
    redB = im.ImVec4(0.7, 0.25, 0.3, 1.0),
    yellowB = im.ImVec4(0.4, 1.0, 0.4, 1.0),
  }
end

-- Summer of Love.
local function getSummerOfLove()
  return {
    fullWhite = im.ImVec4(1, 1, 1, 0.7),
    dullWhite = im.ImVec4(1, 1, 1, 0.25),
    purpleB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    greenB = im.ImVec4(0.93, 0.73, 0.32, 1.0),
    greenD = im.ImVec4(0.53, 0.43, 0.10, 1.0),
    blueB = im.ImVec4(0.91, 0.39, 0.56, 1.0),
    blueD = im.ImVec4(0.55, 0.20, 0.30, 1.0),
    redB = im.ImVec4(0.99, 0.30, 0.00, 1.0),
    yellowB = im.ImVec4(0.4, 1.0, 0.4, 1.0),
  }
end

-- Deer Park.
local function getDeerPark()
  return {
    fullWhite = im.ImVec4(1, 1, 1, 0.7),
    dullWhite = im.ImVec4(1, 1, 1, 0.25),
    purpleB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    greenB = im.ImVec4(0.40, 0.75, 0.45, 1.0),
    greenD = im.ImVec4(0.18, 0.35, 0.20, 1.0),
    blueB = im.ImVec4(0.85, 0.75, 0.55, 1.0),
    blueD = im.ImVec4(0.50, 0.45, 0.25, 1.0),
    redB = im.ImVec4(0.65, 0.35, 0.20, 1.0),
    yellowB = im.ImVec4(0.4, 1.0, 0.4, 1.0),
  }
end

-- Dream Machine.
local function getDreamMachine()
  return {
    fullWhite = im.ImVec4(1.0, 1.0, 1.0, 0.7),
    dullWhite = im.ImVec4(0.65, 0.65, 0.65, 0.25),
    purpleB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    blueB = im.ImVec4(0.9, 0.6, 0.85, 1.0),
    blueD = im.ImVec4(0.4, 0.3, 0.5, 1.0),
    greenB = im.ImVec4(0.55, 0.85, 0.75, 1.0),
    greenD = im.ImVec4(0.25, 0.5, 0.45, 1.0),
    redB = im.ImVec4(1.0, 0.5, 0.5, 1.0),
    yellowB = im.ImVec4(0.4, 1.0, 0.4, 1.0),
  }
end

-- Headlights.
local function getHeadlights()
  return {
    fullWhite = im.ImVec4(1, 1, 1, 0.7),
    dullWhite = im.ImVec4(1, 1, 1, 0.25),
    purpleB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    greenB = im.ImVec4(0.90, 0.90, 0.90, 1.0),
    greenD = im.ImVec4(0.50, 0.50, 0.50, 1.0),
    blueB = im.ImVec4(1.0, 1.0, 0.55, 1.0),
    blueD = im.ImVec4(0.60, 0.60, 0.20, 1.0),
    redB = im.ImVec4(1.0, 0.35, 0.35, 1.0),
    yellowB = im.ImVec4(0.4, 1.0, 0.4, 1.0),
  }
end

-- Crystal.
local function getCrystal()
  return {
    fullWhite = im.ImVec4(0.95, 0.95, 1.0, 0.7),
    dullWhite = im.ImVec4(0.6, 0.6, 0.7, 0.7),
    purpleB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    blueB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    blueD = im.ImVec4(0.25, 0.35, 0.45, 1.0),
    greenB = im.ImVec4(0.9, 0.9, 0.4, 1.0),
    greenD = im.ImVec4(0.4, 0.4, 0.2, 1.0),
    redB = im.ImVec4(1.0, 0.4, 0.5, 1.0),
    yellowB = im.ImVec4(0.4, 1.0, 0.4, 1.0),
  }
end

-- Neo-Industrial.
local function getNeoIndustrial()
  return {
    fullWhite = im.ImVec4(0.95, 0.95, 0.95, 0.75),
    dullWhite = im.ImVec4(0.95, 0.95, 0.95, 0.2),
    purpleB = im.ImVec4(0.6, 0.8, 1.0, 1.0),
    greenB = im.ImVec4(0.4, 0.75, 0.5, 1.0),
    greenD = im.ImVec4(0.2, 0.4, 0.25, 1.0),
    blueB = im.ImVec4(0.35, 0.6, 0.9, 1.0),
    blueD = im.ImVec4(0.15, 0.3, 0.6, 1.0),
    redB = im.ImVec4(0.9, 0.3, 0.3, 1.0),
    yellowB = im.ImVec4(0.4, 1.0, 0.4, 1.0),
  }
end

-- Return the imgui colour palette for the given theme.
local function getImguiCols(theme)
  if theme == "blueRose" then
    return getBlueRose()
  elseif theme == "vapourStatic" then
    return getVapourStatic()
  elseif theme == "canyonClay" then
    return getCanyonClay()
  elseif theme == "terminalAurora" then
    return getTerminalAurora()
  elseif theme == "summerOfLove" then
    return getSummerOfLove()
  elseif theme == "deerPark" then
    return getDeerPark()
  elseif theme == "dreamMachine" then
    return getDreamMachine()
  elseif theme == "headlights" then
    return getHeadlights()
  elseif theme == "crystal" then
    return getCrystal()
  elseif theme == "neoIndustrial" then
    return getNeoIndustrial()
  end
  return getCrystal() -- Default theme.
end


-- Public interface.
M.getStyle =                                            getStyle
M.getImguiCols =                                        getImguiCols

return M