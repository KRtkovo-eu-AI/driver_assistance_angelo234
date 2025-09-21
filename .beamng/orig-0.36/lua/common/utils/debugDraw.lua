-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- GE do calls `onPreRender()`
-- VEH do calls on `onDebugDraw()`


local M = {}

local DEFAULTTEXTCOL = color(255,255,255,255)
local DEFAULTTEXTCOLBG = color(196,196,196,127)

local ffifound, ffi = pcall(require, 'ffi')
if not ffifound then
  log("E", "parse", "ffi missing")
end

M.Sphere = ffifound and ffi.C.BNG_DBG_DRAW_Sphere or nop
M.Cylinder = ffifound and ffi.C.BNG_DBG_DRAW_Cylinder or nop
M.Line = ffifound and ffi.C.BNG_DBG_DRAW_Line or nop
M.Text = ffifound and ffi.C.BNG_DBG_DRAW_Text or nop
M.LineInstance_MinArg = ffifound and ffi.C.BNG_DBG_DRAW_LineInstance_MinArg or nop
M.SquarePrism = ffifound and ffi.C.BNG_DBG_DRAW_SquarePrism or nop
M.TextAdvanced = ffifound and ffi.C.BNG_DBG_DRAW_TextAdvanced or nop
M.TriSolid = ffifound and ffi.C.BNG_DBG_DRAW_TriSolid or nop

M.LineInstance_MinArgBatch = ffifound and ffi.C.BNG_DBG_DRAW_LineInstance_MinArgBatch or nop
M.TriSolidBatch = ffifound and ffi.C.BNG_DBG_DRAW_TriSolidBatch or nop

--- accept vec3, only difference to `debugDrawer` is using packed color `int` instead of mismatched `ColorF|I`
--- avoid using those conversion function, use FFI directly if possible
M.drawSphere = function (pos,r,packedCol,useZ)
  if useZ == nil then useZ=true end
  M.Sphere(pos.x,pos.y,pos.z,r,packedCol,useZ)
end
M.drawLineInstance_MinArg = function (posA,posB,w,packedCol)
  M.LineInstance_MinArg(posA.x,posA.y,posA.z,posB.x,posB.y,posB.z,w,packedCol)
end
M.drawSquarePrism = function (base,tip,baseSize,tipSize,packedCol,useZ)
  if useZ == nil then useZ=true end
  M.SquarePrism(base.x,base.y,base.z,tip.x,tip.y,tip.z,baseSize.x,baseSize.y,tipSize.x,tipSize.y,packedCol,useZ)
end
M.drawTextAdvanced = function (pos,txt,packedCol,useAdvancedText,twod,bgColorPacked,shadow,useZ)
  if packedCol == nil then packedCol=DEFAULTTEXTCOL end
  if useAdvancedText == nil then useAdvancedText=false end
  if twod == nil then twod=false end
  if bgColorPacked == nil then bgColorPacked=defaultTextColBg end
  if shadow == nil then shadow=false end
  if useZ == nil then useZ=true end
  M.TextAdvanced(pos.x,pos.y,pos.z,txt,packedCol,useAdvancedText,twod,bgColorPacked,shadow,useZ)
end
M.drawTriSolid = function (posA,posB,posC,packedCol,useZ)
  if useZ == nil then useZ=true end
  M.TriSolid(posA.x,posA.y,posA.z,posB.x,posB.y,posB.z,posC.x,posC.y,posC.z,packedCol,useZ)
end

return M