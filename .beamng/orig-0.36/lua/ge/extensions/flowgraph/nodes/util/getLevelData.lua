-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Get Level Data'
C.description = "Gets information about the current level."
C.color = im.ImVec4(0.03, 0.41, 0.64, 0.75)
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'out', type = 'string', name = 'devName', description = "Name of the level." },
  { dir = 'out', type = 'string', name = 'directory', hidden = true, description = "Directory for the level." },
  { dir = 'out', type = 'string', name = 'path', hidden = true, description = "Full path of the level." },
}

C.tags = {'gameplay', 'level', 'utils'}

function C:work()
  local levelPath = getMissionFilename()
  local dir, _, _ = path.split(levelPath)
  local devName = string.gsub(dir, "(.*/)(.*)/", "%2")

  self.pinOut.devName.value = tostring(devName)
  self.pinOut.directory.value = tostring(dir)
  self.pinOut.path.value = tostring(levelPath)
end

return _flowgraph_createNode(C)
