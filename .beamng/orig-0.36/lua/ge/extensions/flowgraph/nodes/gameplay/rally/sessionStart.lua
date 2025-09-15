-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
local logTag = ''

C.name = 'Rally Mode Session Start'
C.description = 'Do necessary loading for Rally Mode.'
C.color = rallyUtil.rally_flowgraph_color
C.tags = {'rally'}
C.category = 'once_instant'

C.pinSchema = {
  -- { dir = 'in', type = 'number', name = 'vehId', description = 'Vehicle id.'},
}

-- gets called when the project of this node stops execution.
function C:_executionStopped()
  -- log('D', logTag, "stopped rallyStage execution")
  self:unloadExt()
  extensions.hook('onRallySessionEnd')
end

function C:unloadExt()
  -- log('D', logTag, 'unloading gameplay_rally')
  if extensions.isExtensionLoaded('gameplay_rally') then
    extensions.unload('gameplay_rally')
  end
end

function C:loadExt()
  -- log('D', logTag, 'loading gameplay_rally')
  if not extensions.isExtensionLoaded('gameplay_rally') then
    extensions.load('gameplay_rally')
  end
end

function C:workOnce()
  -- log('D', logTag, 'loading rally mode')
  -- self:unloadExt()
  self:loadExt()

  local missionId, missionDir, err = rallyUtil.detectMissionIdHelper()
  if err then
    log('E', logTag, '')
    log('E', logTag, '===============================================')
    log('E', logTag, '= RallyStage flowgraph: failed to detect missionId')
    log('E', logTag, '= Did you select a Rally stage in the mission editor? And open the race file?')
    log('E', logTag, '===============================================')
    log('E', logTag, '')
    return
  end

  gameplay_rally.loadMission(missionId, missionDir)
  extensions.hook('onRallySessionStart')
end

return _flowgraph_createNode(C)
