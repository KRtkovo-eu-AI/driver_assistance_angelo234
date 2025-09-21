-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- only works with imgui-luaintf gameengine branch for now
-- start with -imguiluaintf if you want to test
if Engine then
  local cmdArgs = Engine.getStartingArgs()
  local i = 1
  while i <= #cmdArgs do
    local v = cmdArgs[i]
    if v == '-imguiluaintf' then
      IMGUI_LUAINTF = true
    end
    i = i + 1
  end
end

if IMGUI_LUAINTF then
  print('!!!IMGUI IS USING WIP LUAINTF MODE!!!')
end

require('common/cdefDebugDraw')
require('common/cdefImgui')
require('common/cdefMath')
