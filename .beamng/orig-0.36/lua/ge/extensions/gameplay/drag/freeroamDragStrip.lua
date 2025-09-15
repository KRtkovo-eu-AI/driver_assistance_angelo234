-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
M.dependencies = {"gameplay_drag_general"}
local levelDragStrips = {}
local dragTetherRange = 4 --meter
local tether

local function openHistoryScreen(facility)
  local dragPos = freeroam_facilities.getClosestDoorPositionForFacility(facility)
  if career_career.isActive() and dragPos then
    tether = career_modules_tether.startSphereTether(dragPos, dragTetherRange, M.closeMenu)
  end
  guihooks.trigger('ChangeState', {state = 'dragHistory', params = {id = facility.id, name = facility.name, level = facility.level}})
end
M.openHistoryScreen = openHistoryScreen

local function onHistoryScreenClosed()
  if tether then tether.remove = true tether = nil end
end
M.onMenuClosed = onHistoryScreenClosed

local function closeMenu()
  career_career.closeAllMenus()
end
M.closeMenu = closeMenu

local function getDragDataForLevel(levelIdentifier)
  if not levelDragStrips[levelIdentifier] then
    levelDragStrips[levelIdentifier] = {}

    -- TODO: build this list dynamically
    local levelDir = core_levels.getLevelByName(levelIdentifier).dir
    local fileList = FS:findFiles(levelDir.."/dragstrips/", "*.dragData.json", -1, true, false)

    for i, file in ipairs(fileList) do
      local dragData = gameplay_drag_general.loadDragStripData(file)
      local _, fn, ext = path.split(file, true)
      dragData._originFile = file
      dragData._fnWithoutExt = string.sub(fn, 1, string.len(fn) - string.len(ext)-1)
      dragData._index = i
      table.insert(levelDragStrips[levelIdentifier], dragData)
    end
  end
  return levelDragStrips[levelIdentifier]
end

local function onGetRawPoiListForLevel(levelIdentifier, elements)
  if career_career.isActive() or settings.getValue("enableDragRaceInFreeroam") then
    for _, data in pairs(getDragDataForLevel(levelIdentifier)) do
      for i, lane in ipairs(data.strip.lanes) do
        local pos = vec3(lane.waypoints.stage.transform.pos)
        local poi = {
          id = string.format("drag##%s-%s", data._fnWithoutExt, lane.shortName),
          markerInfo = {
            invisibleTrigger = {
              pos = pos,
              radius = 6,
              onInside = function(interactData)
                if not gameplay_drag_general.getDragIsStarted() then
                  --dump("Starting Drag from Marker")
                  gameplay_drag_general.setDragRaceData(deepcopy(data))
                  gameplay_drag_general.startDragRaceActivity(i)
                end
              end,
            }
          }
        }
        table.insert(elements, poi)
      end
    end
  end
end
M.onGetRawPoiListForLevel = onGetRawPoiListForLevel

return M