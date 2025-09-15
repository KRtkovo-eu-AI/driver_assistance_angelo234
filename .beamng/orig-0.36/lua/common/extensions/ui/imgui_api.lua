-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt


-- do not use this file/extensions directly, use ui_imgui instead

-- this file needs to be in sync with imgui_api.h

local M = {}

M.ctx = nil -- global lua imgui context

if IMGUI_LUAINTF then
  require('/common/extensions/ui/imgui_gen_luaintf')(M)
  require('/common/extensions/ui/imgui_custom_luaintf')(M)
else
  require('/common/extensions/ui/imgui_gen')(M)
  require('/common/extensions/ui/imgui_custom')(M)
end

return M