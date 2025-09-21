-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.dependencies = {"core_vehicleTriggers", "core_funstuff"}
local im = ui_imgui

local debugAddVehicleTriggers = false

local playerVehicleDefaultPath = "/root/playerVehicle/general/"
local sandboxDefaultPath = "/root/sandbox/quick/"
local missionDefaultPath = "/root/sandbox/mission/"
local careerDefaultPath = "/root/sandbox/career/"

local slowMoFactor = 0.01
local maxSlotAmount = 8

-- holds all known menu items
local menuTree = {}
local menuTreeCopy
local menuTreeCopyForUI

local iconTags = jsonReadFile("ui/assets/iconMappings/iconTags.json")
local iconTagsWarned = {}
local updateIcon = function(icon, title)
  if not icon then return end
  if iconTags[icon] then
    if not iconTagsWarned[icon] then
      log("W", "", string.format("Using outdated icon %s, please update to %s for %s", icon, iconTags[icon], title))
      iconTagsWarned[icon] = true
    end
    return iconTags[icon]
  else
    return icon
  end
end

local callStack = {} -- contains the history of menus so one can go 'back' or 'up' the menus again

-- transitional state: these values can change whereas the UI is not visible yet, use uiVisible to check if the UI is shown
local currentLevel = nil -- level that the menu is in right now
local currentMenuItems = nil -- items that are displaying
local uiVisible = false
local gameStateBefore = nil
local vehicleMenuTrees = {}
local currentUiState
local simTimeBefore
--local possibleTopLevels = {"sandbox", "playerVehicle", "favorites", "multi"}

local safeFile = "settings/radialFavorites.json"
local recentActionsFile = "settings/radialRecentActions.json"
local dynamicSlotSettingsFile = "settings/radialDynamicSlotSettings.json"

-- Track recent actions with uniqueIDs, organized by top-level category
local recentActions = {
  all = {}
}

-- Set different max lengths for each category
local maxRecentActionsLimits = {
  all = 10
}

-- Configuration for recent actions behavior
local recentActionConfig = {
  mode = "preserveOrder" -- Options: "preserveOrder" or "moveToTop"
}

local dynamicSlotSettings = {
  root_up = {
    breadcrumbs = {"Radial Menu Up","Dynamic Slot Configuration"},
    mode = "recentActions",
    category = "all",
    priority = 1
  },
  root_down = {
    breadcrumbs = {"Radial Menu Down","Dynamic Slot Configuration"},
    mode = "empty",
    category = "all",
    priority = 2,
  },
  root_upLeft = {
    breadcrumbs = {"Radial Menu Up Left","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 3,
    level = "/root/sandbox/vehicles/",
    uniqueID = "switchVehiclesNext",
  },
  root_upRight = {
    breadcrumbs = {"Radial Menu Up Right","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 4,
    level = "/root/playerVehicle/vehicleFeatures/",
    uniqueID = "toggleWalkingMode",
  },
  root_downLeft = {
    breadcrumbs = {"Radial Menu Down Left","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 5,
  },
  root_downRight = {
    breadcrumbs = {"Radial Menu Down Right","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 6,
  },



  sandbox_left = {
    breadcrumbs = {"Sandbox Right","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 1,
    level = "/root/sandbox/repair/",
    uniqueID = "repairVehicle",
  },
  sandbox_upLeft = {
    breadcrumbs = {"Sandbox Up Left","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 2,
  },
  sandbox_up = {
    breadcrumbs = {"Sandbox Up","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 3,
    level = "/root/sandbox/vehicles/",
    uniqueID = "switchVehiclesNext",
  },
  sandbox_upRight = {
    breadcrumbs = {"Sandbox Up Right","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 4,
  },
  sandbox_right = {
    breadcrumbs = {"Sandbox Right","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 5,
    level = "/root/sandbox/vehicles/",
    uniqueID = "vehicle_selector",
  },
  sandbox_downRight = {
    breadcrumbs = {"Sandbox Down Right","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 6,
  },
  sandbox_down = {
    breadcrumbs = {"Sandbox Down","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 7,
    level = "/root/sandbox/",
    uniqueID = "traffic",
  },
  sandbox_downLeft = {
    breadcrumbs = {"Sandbox Down Left","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 8,
  },


  vehicle_left = {
    breadcrumbs = {"Vehicle Left","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 1,
    uniqueID = "toggle_left_signal",
    level = "/root/playerVehicle/lights/signals/",
  },
  vehicle_upLeft = {
    breadcrumbs = {"Vehicle Up Left","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 2,
  },
  vehicle_up = {
    breadcrumbs = {"Vehicle Up","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 3,
    uniqueID = "toggleWalkingMode",
    level = "/root/playerVehicle/vehicleFeatures/",
  },
  vehicle_upRight = {
    breadcrumbs = {"Vehicle Up Right","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 4,
  },
  vehicle_right = {
    breadcrumbs = {"Vehicle Right","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 5,
    uniqueID = "toggle_right_signal",
    level = "/root/playerVehicle/lights/signals/",
  },
  vehicle_downRight = {
    breadcrumbs = {"Vehicle Down Right","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 6,
  },
  vehicle_down = {
    breadcrumbs = {"Vehicle Down","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 7,
    uniqueID = "toggleIgnitionLevel",
    level = "/root/playerVehicle/vehicleFeatures/",
  },
  vehicle_downLeft = {
    breadcrumbs = {"Vehicle Down Left","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 8,
  },

  funstuff_left = {
    breadcrumbs = {"Funstuff Left","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 1,
    pathFilter = "/root/sandbox/funStuff/"
  },
  funstuff_upLeft = {
    breadcrumbs = {"Funstuff Up Left","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    level = "/root/sandbox/funStuff/destruction/",
    uniqueID = "funBoom",
    priority = 2,
    pathFilter = "/root/sandbox/funStuff/"
  },
  funstuff_up = {
    breadcrumbs = {"Funstuff Up","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 3,
    level = "/root/sandbox/funStuff/forces/",
    uniqueID = "boost",
    pathFilter = "/root/sandbox/funStuff/"
  },
  funstuff_upRight = {
    breadcrumbs = {"Funstuff Up Right","Dynamic Slot Configuration"},
    mode = "uniqueAction",
    priority = 4,
    level = "/root/sandbox/funStuff/destruction/",
    uniqueID = "funBreak",
    pathFilter = "/root/sandbox/funStuff/"
  },
  funstuff_right = {
    breadcrumbs = {"Funstuff Right","Dynamic Slot Configuration"},
    mode = "empty",
    priority = 5,
    pathFilter = "/root/sandbox/funStuff/"
  },


}

local vehicleWaitFrames = 0 -- counter for timeout waiting for vehicle menu items

local initialized = false

-- if its shown
local function isEnabled()
  return uiVisible
end

local function saveRecentActions()
  jsonWriteFile(recentActionsFile, recentActions, true, nil, true)
end

local function loadRecentActions()
  local savedRecentActions = jsonReadFile(recentActionsFile)
  if savedRecentActions then
    for category, actions in pairs(savedRecentActions) do
      recentActions[category] = {}
      for key, action in pairs(actions) do
        local idx = tonumber(key) -- sometimes the key is a number, sometimes it's a string. stupid json
        recentActions[category][idx] = action
      end
    end

    log("I", "", "Loaded radial menu recent actions")
  end
end
local function saveDynamicSlotSettings()
  local savedDynamicSlotSettings = {}
  for key, value in pairs(dynamicSlotSettings) do
    savedDynamicSlotSettings[key] = {
      mode = value.mode,
      category = value.category,
      uniqueID = value.uniqueID,
      level = value.level,
    }
  end
  jsonWriteFile(dynamicSlotSettingsFile, savedDynamicSlotSettings, true, nil, true)
end

local baseDynamicSlotSettings = {}
for key, value in pairs(dynamicSlotSettings) do
  baseDynamicSlotSettings[key] = {
    mode = value.mode,
    category = value.category,
    uniqueID = value.uniqueID,
    level = value.level,
  }
end

local function loadDynamicSlotSettings(reset)
  local savedDynamicSlotSettings
  if reset then
    savedDynamicSlotSettings = baseDynamicSlotSettings
  else
    savedDynamicSlotSettings = jsonReadFile(dynamicSlotSettingsFile)
  end

  if savedDynamicSlotSettings then
    for key, value in pairs(savedDynamicSlotSettings) do
      local slot = dynamicSlotSettings[key]
      if slot then
        slot.mode = value.mode
        slot.category = value.category
        slot.uniqueID = value.uniqueID
        slot.level = value.level
      end
    end
  end
end

local function resetDynamicSlotSettings()
  loadDynamicSlotSettings(true)
  saveDynamicSlotSettings()
end
M.resetDynamicSlotSettings = resetDynamicSlotSettings

local validRoots = {
  ["/root/playerVehicle/"] = {icon = "steeringWheelSporty", title="Vehicle Actions", expanded = true},
  ["/root/sandbox/"] = {icon = "terrain", title="Freeroam Actions", expanded = true},
}
local function buildTree(flatData)
  local tree = {items = {}, path = "/"}

  for path, value in pairs(flatData) do
    -- Check if path starts with any valid root
    local isValidRoot = nil
    for root in pairs(validRoots) do
      if string.find(path, "^" .. root) then
        isValidRoot = root
        break
      end
    end
    if not isValidRoot then goto continue end
    local currentNode = tree
    currentNode.items = currentNode.items or {}
    local currentPath = "/"
    --log("I", "", "Building tree for path " .. path)
    for segment in string.gmatch(path, "[^/]+") do
      currentPath = currentPath .. segment .. "/"
      --log("I", "", "Current path: " .. currentPath)
      if not currentNode.items[segment] then
        --log("I", "", "Adding segment " .. segment .. " to path " .. currentPath)
        currentNode.items[segment] = {items = {}, path = currentPath, niceName = segment}
        if isValidRoot then
          tableMerge(currentNode.items[segment], validRoots[isValidRoot])
        end
      end
      currentNode = currentNode.items[segment]
    end

    -- Add the final value to the leaf node
    for k, v in pairs(value) do
      v.level = path -- Assign the full path to the leaf node
      v.action = true
      local title = v.title or (v.level .. k)
      if not v.dynamicSlot then
        currentNode.items[title] = v
      end
    end
    ::continue::
  end

  return tree
end

local function addTitleToTreeItems(tree)
  for key, value in pairs(tree.items) do
    if value.path then
      for key2, value2 in pairs(tree.items) do
        if value2["goto"] and value2["goto"] == value.path then
          tableMerge(value, value2)
          tree.items[key2] = nil

          break
        end
      end
    end
    if value.items then
      addTitleToTreeItems(value)
    end
  end
end

local function sortCategories(a, b)
  if a.startSlot and b.startSlot then
    return a.startSlot < b.startSlot
  end
  if a.categoryOrder and b.categoryOrder then
    return a.categoryOrder < b.categoryOrder
  end
  return tostring(a._key) < tostring(b._key)
end
local function filterValidDynamicSlotItems(tree)
  local sortedItems = {}
  for key, value in pairs(tree.items) do
    value._key = key
    table.insert(sortedItems, value)
  end
  table.sort(sortedItems, sortCategories)
  tree.items = sortedItems
  for _, value in ipairs(tree.items) do
    if value.items then
      filterValidDynamicSlotItems(value)
    end
    if not value.uniqueID then
      value.invalid = true
    end
    if value.dynamicSlot then
      value.invalid = true
    end
  end
end

local function getActionInfo(level, uniqueID)
  if not menuTreeCopy or not menuTreeCopy[level] then return nil end
  for _, actionInfo in ipairs(menuTreeCopy[level]) do
    if actionInfo.uniqueID == uniqueID then return actionInfo end
  end
  return nil
end

local dynamicSlotKeyToConfigure = nil
local function openDynamicSlotConfigurator(dynamicSlotKey)
  dynamicSlotKeyToConfigure = dynamicSlotKey
  guihooks.trigger('OpenDynamicSlotConfigurator')
end


local function getDynamicSlotConfigurationData()
  local menuTreeCopyForUI = deepcopy(menuTreeCopy)
  local dynamicSlotSettingsData = dynamicSlotSettings[dynamicSlotKeyToConfigure]
  local filter = dynamicSlotSettingsData.pathFilter or nil
  if filter then
    local newMenuTreeCopyForUI = {}
    for key, value in pairs(menuTreeCopyForUI) do
      if string.find(key, dynamicSlotSettingsData.pathFilter) then
        newMenuTreeCopyForUI[key] = value
      end
    end
    menuTreeCopyForUI = newMenuTreeCopyForUI
  end
  for key, value in pairs(menuTreeCopyForUI) do
    for _, action in pairs(value or {}) do
      action.icon = updateIcon(action.icon, action.title)
    end
  end

  menuTreeCopyForUI = buildTree(menuTreeCopyForUI)
  addTitleToTreeItems(menuTreeCopyForUI)
  filterValidDynamicSlotItems(menuTreeCopyForUI)

  local rootItems = {}
  if filter then
    while not string.find(menuTreeCopyForUI.path, filter) do
      menuTreeCopyForUI = menuTreeCopyForUI.items[1]
    end
    for _, item in ipairs(menuTreeCopyForUI.items) do
      table.insert(rootItems, item)
    end
  else
    -- break out root menus
    for _, item in ipairs(menuTreeCopyForUI.items[1].items) do
      if validRoots[item.path] then
        table.insert(rootItems, item)
      end
    end
  end
  local recentActionsItem = {title = "Recent Actions", icon = "redo", items = {}}
  if not filter then
    table.insert(recentActionsItem.items, { title = "Most Recent Action", mode = "recentActions", category = "all", uniqueID = "dynamic_recent",icon = "redo", desc = "Shows the most recently used action"})
  end
  local recentActions = M.getMostRecentAvailableActions("all", 3)
  table.sort(recentActions, function(a, b) return a.timestamp > b.timestamp end)
  for _, action in ipairs(recentActions) do
    if filter and not string.find(action.level, filter) then
      goto continue
    end
    table.insert(recentActionsItem.items, action)
    ::continue::
  end
  table.insert(rootItems, 1, recentActionsItem)
  table.insert(rootItems, 1, {title = "Empty", mode = "empty", uniqueID = "dynamic_empty", icon = "circleSlashed", desc = "Leaves this slot empty"})
  menuTreeCopyForUI.items = rootItems

  local dynamicSlotKey = dynamicSlotKeyToConfigure
  local data = {
    items = menuTreeCopyForUI,
    dynamicSlotKey = dynamicSlotKey,
    dynamicSlotData = dynamicSlotSettings[dynamicSlotKey],
  }
  if dynamicSlotSettings[dynamicSlotKey].mode == "uniqueAction" then
    data.uniqueAction = getActionInfo(dynamicSlotSettings[dynamicSlotKey].level, dynamicSlotSettings[dynamicSlotKey].uniqueID)
  end
  return data
end

M.setDynamicSlotConfiguration = function(dynamicSlotKey, data)
  local setting = dynamicSlotSettings[dynamicSlotKey]
  if data.mode == "uniqueAction" then
    setting.mode = "uniqueAction"
    setting.uniqueID = data.uniqueID
    setting.level = data.level
    setting.category = nil
  elseif data.mode == "recentActions" then
    setting.mode = "recentActions"
    setting.uniqueID = nil
    setting.level = nil
    setting.category = data.category
  elseif data.mode == "empty" then
    setting.mode = "empty"
    setting.uniqueID = nil
    setting.level = nil
    setting.category = nil
  end
  saveDynamicSlotSettings()
  M.reload()
end



local function toNiceName(str)
  -- Replace underscores with spaces
  local niceName = str:gsub("_", " ")

  -- Add space before each capital letter except the first one
  niceName = niceName:gsub("(%l)(%u)", "%1 %2")

  -- Capitalize the first letter of each word
  niceName = niceName:gsub("(%a)(%w*)", function(first, rest)
    return first:upper() .. rest:lower()
  end)

  return niceName
end


local sortCategoriesFun = function(a, b) return a.categoryOrder < b.categoryOrder end
local function sort_categories(input_list)
  -- Create a lookup table for quick index access

  for i, category in ipairs(input_list) do
    category.categoryOrder = category.categoryOrder or (1000 + i)
  end

  -- Sort the input list based on the category order
  table.sort(input_list, sortCategoriesFun)
end

local function getAllLevels(path)
  local levels = {}
  local currentPath = "/"

  for segment in path:gmatch("/([^/]+)") do
    currentPath = currentPath .. segment .. "/"
    table.insert(levels, currentPath)
  end

  return levels
end

local function gotoButtonExists(gotoLevel, prevLevel, menuTreeCopy)
  if not menuTreeCopy[prevLevel] then return false end
  for _, item in ipairs(menuTreeCopy[prevLevel]) do
    if item["goto"] == gotoLevel then return true end
  end
end

--[[
- definition:
 * items = items inside a menu
 * entries : a single thing that should produce one ore more menu entries
 * Slot positions on the radial menu:
  * 1 = Left, 3 = Up, 5 = Right, 7 = Down
  * (Values 2,4,6,8 are the diagonals in between)
]]

-- this function adds a new menu entry
local function addEntry(_args)
  local args = deepcopy(_args) -- do not modify the outside table by any chance
  if  type(args.generator) ~= 'function' and (type(args.title) ~= 'string' or (type(args.onSelect) ~= 'function' and type(args["goto"]) ~= 'string')) then
    -- TODO: add proper warning/error
    log('W', 'quickaccess', 'Menu item needs at least a title and an onSelect function callback: ' .. dumps(args))
    --return false
  end

  -- defaults
  if args.level == nil then args.level = '/root/sandbox/general/' end
  if args.desc == nil then args.desc = '' end
  if not (string.startswith(args.level, "/root/")) then
    args.level = '/root/sandbox' .. args.level
  end

  if type(args.level) ~= 'string' then
    log('E', 'quickaccess', 'Menu item level incorrect, needs to be a string: ' .. dumps(args))
    return false
  end
  if string.sub(args.level, string.len(args.level)) ~= '/' then args.level = args.level .. '/' end -- make sure there is always a trailing slash in the level

  if menuTree[args.level] == nil then
    -- add new level if not existing
    menuTree[args.level] = {}
  end

  if args.uniqueID then
    -- make this entry unique in this level
    local replaced = false
    for k, v in pairs(menuTree[args.level]) do
      if v.uniqueID == args.uniqueID then
        menuTree[args.level][k] = args
        replaced = true
        break
      end
    end
    if not replaced then
      table.insert(menuTree[args.level], args)
    end
  else
    -- always insert
    table.insert(menuTree[args.level], args)
  end

  return true
end


local function getAiPath(path, index)
  local aiPath = {}
  local veh = be:getObject(index)
  if not veh then return nil end
  local firstWpAdded = false
  local vehPos = veh:getPosition()
  local vehFwd = veh:getDirectionVector()
  local bestWp = nil
  local bestDist = math.huge
  local prevDot = nil
  local prevWp = nil
  local startDist = path[1].distToTarget
  local firstValidWp = nil
  -- find the first valid waypoint
  for _, marker in ipairs(path) do
    if marker.wp then
      firstValidWp = marker.wp
      if not firstWpAdded then
        local toWp = (marker.pos - vehPos):normalized()
        local dot = toWp:dot(vehFwd)
        -- If we have a previous dot product and this one is lower, we found our local maximum
        if prevDot and dot < prevDot and prevDot > 0 then
          --log('I', 'quickaccess', 'Adding waypoint ' .. prevWp .. ' because it is the local maximum, dot: ' .. dot .. ', prevDot: ' .. prevDot .. ', startDist: ' .. startDist .. ', marker.distToTarget: ' .. marker.distToTarget)
          table.insert(aiPath, prevWp)
          firstWpAdded = true
        end
        prevDot = dot
        prevWp = marker.wp
      else
        table.insert(aiPath, marker.wp)
      end
    end
  end
  -- If we haven't found a local maximum but have a positive dot product, use the last one
  if not firstWpAdded and prevDot and prevDot > 0 then
    table.insert(aiPath, prevWp)
    firstWpAdded = true
  end
  -- fallback: if no waypoint is in front of vehicle, use the first waypoint that's 25m away and all other waypoints
  if not firstWpAdded then
    for _, marker in ipairs(path) do
      if marker.wp and startDist - marker.distToTarget > 25 then
        table.insert(aiPath, marker.wp)
      end
    end
  end
  return aiPath
end


local function registerRootMenus()
  -- multi menu
  addEntry({ level = '/root/', generator = function(entries)
      table.insert(entries, {title = "Vehicle Actions", ["goto"] = playerVehicleDefaultPath, icon = "steeringWheelSporty", startSlot = 1, endSlot = 1, isLTabAction = true, desc = "Interact with the vehicle itself, like engine, electrics, etc.", ignoreAsRecentAction = true})

      table.insert(entries, {startSlot = 3, endSlot = 3, dynamicSlot = {id="root_up"}})
      table.insert(entries, {startSlot = 7, endSlot = 7, dynamicSlot = {id="root_down"}})
      table.insert(entries, {startSlot = 4, endSlot = 4, dynamicSlot = {id="root_upRight"}})
      table.insert(entries, {startSlot = 2, endSlot = 2, dynamicSlot = {id="root_upLeft"}})
      table.insert(entries, {startSlot = 6, endSlot = 6, dynamicSlot = {id="root_downRight"}})
      table.insert(entries, {startSlot = 8, endSlot = 8, dynamicSlot = {id="root_downLeft"}})

    end
  })


end

--TODO: Felix: automatically filter empty categories if there's no actions or items inside. then, move this condition back into "root/sandbox/ai/"
--TODO: Felix: gray out empty categories or something similar
local function registerSandboxMenus()
  -- root menu entry

  addEntry({ level = '/root/', generator = function(entries)
    local title = "Freeroam Actions"
    local icon = "terrain"
    local desc = "Recover your vehicle, interact with the environment, manage vehicles and traffic, etc."
    local gotoPath = sandboxDefaultPath

    if career_career.isActive() then
      title = "Career Actions"
      icon = "cup"
      desc = "Interact with career related systems like taxi and recovery."
      gotoPath = careerDefaultPath
    end
    if gameplay_missions_missionManager.getForegroundMissionId() then
      title = "Challenge Actions"
      icon = "flag"
      desc = "Interact with the challenge related systems like recovery and submitting attempts."
      gotoPath = missionDefaultPath
    end

    table.insert(entries, {title = title, ["goto"] = gotoPath, icon = icon, startSlot = 5, endSlot = 5 , uniqueID = 'sandbox', isRTabAction = true, desc = desc, ignoreAsRecentAction = true})
  end})

  -- categories
  local sandboxCategories = {
    { title = 'Other',                  ["goto"] = '/root/sandbox/other/',     icon = nil,            uniqueID = 'other',     categoryOrder = -30 },
    { title = 'Fun Stuff',              ["goto"] = '/root/sandbox/funStuff/',  icon = 'magicWand',    uniqueID = 'funStuff',  categoryOrder = -20 },
    { title = 'Repair',                 ["goto"] = '/root/sandbox/repair/',    icon = 'wrench',       uniqueID = 'repair',    categoryOrder = -10 },
    { title = 'Quick Actions',          ["goto"] = '/root/sandbox/quick/',     icon = 'charge',                               categoryOrder =   0 },
    { title = 'Challenge',              ["goto"] = '/root/sandbox/mission/',   icon = 'flag',         uniqueID = 'challenge', categoryOrder =   1 },
    { title = 'Career',                 ["goto"] = '/root/sandbox/career/',    icon = 'cup',          uniqueID = 'career',    categoryOrder =   2 },
    { title = 'Manage Vehicles',        ["goto"] = '/root/sandbox/vehicles/',  icon = 'carPlus',      uniqueID = 'manageVehicles',  categoryOrder =  10 },
    { title = 'ui.radialmenu2.traffic', ["goto"] = '/root/sandbox/traffic/',   icon = 'trafficLight', uniqueID = 'traffic',   categoryOrder =  20 },
    { title = 'Ai',                     ["goto"] = '/root/sandbox/ai/',        icon = 'AIMicrochip',  uniqueID = 'sandboxAi', categoryOrder =  30 },
  }

  for _, category in ipairs(sandboxCategories) do
    local res = tableMerge(category, {level = '/root/sandbox/'})
    addEntry(res)
  end
  addEntry({ level = '/root/sandbox/mission/', generator = function(entries) end })
  addEntry({ level = '/root/sandbox/mission/', generator = function(entries) end })

  local envBlacklist = {['scenario'] = true, ['mission'] = true, ['garage'] = true}
  addEntry({ level = '/root/sandbox/other/', generator = function(entries)
    if getPlayerVehicle(0) and not core_input_actionFilter.isActionBlocked("reload_vehicle") then
      table.insert(entries, {
        title = "ui.radialmenu2.Save", icon = "saveMesh", priority = 90, startSlot = 1, uniqueID = 'beamstateSave',
        onSelect = function()
          getPlayerVehicle(0):queueLuaCommand("beamstate.save()")
          return {"hide"}
        end
      })

      table.insert(entries, {
        title = "ui.radialmenu2.Load", icon = "loadMesh", priority = 91, startSlot = 2, uniqueID = 'beamstateLoad',
        onSelect = function()
          getPlayerVehicle(0):queueLuaCommand("beamstate.load()")
          extensions.hook('trackVehReset')
          return {"hide"}
        end
      })
    end

    if not core_input_actionFilter.isActionBlocked("photomode") then
      table.insert(entries, {
        title = "ui.dashboard.photomode", icon = "photo", priority = 92, startSlot = 3, folder = true, uniqueID = 'photomode',
        onSelect = function()
          M.setEnabled(false)
          guihooks.trigger('ChangeState', {state = 'menu.photomode'})
          return {}
        end
      })
    end

    if not envBlacklist[core_gamestate.state.menuItems] then
      table.insert(entries, {
        title = 'ui.environment.tireMarks', icon = 'ESC',  uniqueID = 'tiremarks', ["goto"] = '/root/sandbox/other/tiremarks/', ignoreAsRecentAction = true,
      })
      --time of day
      table.insert(entries, {
        title = 'ui.environment.timeOfDay', icon = 'sunRise',  uniqueID = 'timeofday', ["goto"] = '/root/sandbox/other/timeofday/', ignoreAsRecentAction = true,
      })

      if not career_career.isActive() then
        table.insert(entries, {
          title = 'ui.dashboard.environment', icon = 'weather',  uniqueID = 'environment',
          onSelect = function()
            M.setEnabled(false)
            guihooks.trigger('ChangeState', {state = 'menu.environment'})
            return {}
          end
        })
      end
    end
  end})
  addEntry({ level = '/root/sandbox/other/timeofday/', generator = function(entries)
    if not envBlacklist[core_gamestate.state.menuItems] then
      -- Get current time from environment service
      local timeOfDay = core_environment.getTimeOfDay()
      local currentTime = timeOfDay and timeOfDay.time or 0
      local seconds = ((currentTime + 0.5) % 1) * 86400
      local hours = math.floor(seconds / 3600)
      local mins = math.floor(seconds / 60 - (hours * 60))
      local timeStr = string.format("%02d:%02d", hours, mins)

      -- Morning (6:30 AM)
      table.insert(entries, {
        title = 'ui.quickrace.tod.sunrise', icon = 'sunRise', uniqueID = 'timeMorning',
        desc = "Set time to 6:30 AM",
        originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "timeofday"},
        startSlot = 2, endSlot = 2,
        onSelect = function()
          core_environment.setTimeOfDay({time = 0.75}) -- 0.75 = 6:30 AM (3/4 through cycle)
          return {'hide'}
        end
      })

      -- Noon (12:00 PM)
      table.insert(entries, {
        title = 'ui.quickrace.tod.noon', icon = 'day', uniqueID = 'timeNoon',
        desc = "Set time to 12:00 PM",
        originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "timeofday"},
        startSlot = 3, endSlot = 3,
        onSelect = function()
          core_environment.setTimeOfDay({time = 0}) -- 0 = noon
          return {'hide'}
        end
      })

      -- Afternoon (3:00 PM)
      table.insert(entries, {
        title = 'ui.quickrace.tod.evening', icon = 'sunDown', uniqueID = 'timeAfternoon',
        desc = "Set time to 5:00 PM",
        originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "timeofday"},
        startSlot = 4, endSlot = 4,
        onSelect = function()
          core_environment.setTimeOfDay({time = 0.2084}) -- 0.20833333333333334 = 5:00 PM
          return {'hide'}
        end
      })

      -- -1h
      table.insert(entries, {
        title = '-1h', icon = 'fastBackward', uniqueID = 'timeMinus1h',
        desc = "Go back 1 hour (Current: " .. timeStr .. ")",
        originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "timeofday"},
        startSlot = 1, endSlot = 1,
        onSelect = function()
          core_environment.setTimeOfDay({time = (currentTime - 1/24) % 1})
          return {'reload'}
        end
      })

      -- +1h
      table.insert(entries, {
        title = '+1h', icon = 'fastForward', uniqueID = 'timePlus1h',
        desc = "Go forward 1 hour (Current: " .. timeStr .. ")",
        originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "timeofday"},
        startSlot = 5, endSlot = 5,
        onSelect = function()
          core_environment.setTimeOfDay({time = (currentTime + 1/24) % 1})
          return {'reload'}
        end
      })

      -- -10m
      table.insert(entries, {
        title = '-10m', icon = 'arrowSmallLeft', uniqueID = 'timeMinus10m',
        desc = "Go back 10 minutes (Current: " .. timeStr .. ")",
        originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "timeofday"},
        startSlot = 8, endSlot = 8,
        onSelect = function()
          core_environment.setTimeOfDay({time = (currentTime - 10/1440) % 1})
          return {'reload'}
        end
      })

      -- +10m
      table.insert(entries, {
        title = '+10m', icon = 'arrowSmallRight', uniqueID = 'timePlus10m',
        desc = "Go forward 10 minutes (Current: " .. timeStr .. ")",
        originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "timeofday"},
        startSlot = 6, endSlot = 6,
        onSelect = function()
          core_environment.setTimeOfDay({time = (currentTime + 10/1440) % 1})
          return {'reload'}
        end
      })

      -- Play/Pause
      local isPlaying = timeOfDay and timeOfDay.play or false
      table.insert(entries, {
        title = isPlaying and 'ui.environment.pause' or 'ui.environment.play',
        icon = isPlaying and 'pause' or 'play',
        uniqueID = 'timePlayPause',
        desc = "Toggle time progression (Current: " .. timeStr .. ", " .. (isPlaying and "Playing" or "Paused") .. ")",
        originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "timeofday"},
        startSlot = 7, endSlot = 7,
        onSelect = function()
          local timeOfDay = core_environment.getTimeOfDay()
          if timeOfDay then
            timeOfDay.play = not isPlaying
            core_environment.setTimeOfDay(timeOfDay)
          end
          return {'reload'}
        end
      })
    end
  end})
  addEntry({ level = '/root/sandbox/other/tiremarks/', generator = function(entries)
    table.insert(entries, {
      title = 'ui.environment.tireMarks.reset', icon = 'broom',  uniqueID = 'tiremarksReset', originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "tiremarks"},
      startSlot = 2.5, endSlot = 3.5,
      onSelect = function() be:resetTireMarks() return {'hide'} end
    })
    table.insert(entries, {
      title = 'ui.environment.tireMarks.save', icon = 'import',  uniqueID = 'tiremarksSave', originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "tiremarks"},
      startSlot = 0.5, endSlot = 1.5,
      onSelect = function() be:saveTireMarks("tiremarks.dat") return {'hide'} end
    })
    table.insert(entries, {
      title = 'ui.environment.tireMarks.load', icon = 'export',  uniqueID = 'tiremarksLoad', originalActionInfo = {level = "/root/sandbox/other/", uniqueID = "tiremarks"},
      startSlot = 4.5, endSlot = 5.5,
      onSelect = function() be:loadTireMarks("tiremarks.dat") return {'hide'} end
    })
  end})

  extensions.core_funstuff.registerFunstuffActions()


  addEntry({ level = '/root/sandbox/repair/', generator = function(entries)
    local playerVeh = getPlayerVehicle(0)
    if playerVeh then

      if not core_input_actionFilter.isActionBlocked("loadHome") then
        table.insert(entries, {
          title = 'ui.radialmenu2.Manage.Home', icon = 'garage02', priority = 80, startSlot = 5, endSlot = 5, uniqueID = 'loadHome', ignoreAsRecentActionForCategory = "sandbox",
          onSelect = function()
            extensions.hook('trackVehReset')
            playerVeh:queueLuaCommand("recovery.loadHome()")
            return {'hide'}
          end
        })
      end
      if not core_input_actionFilter.isActionBlocked("recover_to_last_road") then
        table.insert(entries, {
          title = 'ui.radialmenu2.flipVehicleUpright', icon = 'carToWheels', priority = 83, startSlot = 3, endSlot = 3, uniqueID = 'flipVehicleUpright', ignoreAsRecentActionForCategory = "sandbox",
          onSelect = function()
            spawn.safeTeleport(playerVeh, playerVeh:getPosition(), quatFromDir(playerVeh:getDirectionVector()), nil, nil, nil, nil, false )
            return {'hide'}
          end
        })
      end
      if not core_input_actionFilter.isActionBlocked("reload_vehicle") then
        table.insert(entries, {
          title = "ui.radialmenu2.repairVehicle", icon = "wrench", priority = 90, startSlot = 0, endSlot = 2, uniqueID = 'repairVehicle', ignoreAsRecentActionForCategory = "sandbox",
          onSelect = function()
            getPlayerVehicle(0):resetBrokenFlexMesh()
            spawn.safeTeleport(getPlayerVehicle(0), getPlayerVehicle(0):getPosition(), quatFromDir(getPlayerVehicle(0):getDirectionVector(), getPlayerVehicle(0):getDirectionVectorUp()), nil, nil, nil, nil, true)
            return {"hide"}
          end
        })
      end
      if not core_input_actionFilter.isActionBlocked("reset_all_physics") then
        table.insert(entries, {
          title = "ui.radialmenu2.resetAllVehicles", icon = "carsWrench", priority = 90, startSlot = 4, endSlot = 4, uniqueID = 'resetAllVehicles',
          onSelect = function()
            extensions.hook('trackVehReset')
            resetGameplay(-1)
            return {"hide"}
          end
        })
      end
      if not core_input_actionFilter.isActionBlocked("recover_to_last_road") then
        table.insert(entries, {
          title = 'ui.radialmenu2.recoverToLastRoad', icon = 'road', priority = 82, startSlot = 7, endSlot = 7, uniqueID = 'recoverToLastRoad', ignoreAsRecentActionForCategory = "sandbox", -- Down
          onSelect = function()
            spawn.teleportToLastRoad(playerVeh, {resetVehicle = false})
            return {'hide'}
          end
        })
      end
      if not core_input_actionFilter.isActionBlocked("saveHome") then
        table.insert(entries, {
          title = 'ui.radialmenu2.Manage.Set_home', icon = 'plusGarage1', priority = 81, startSlot = 6, endSlot = 6, uniqueID = 'saveHome', ignoreAsRecentActionForCategory = "sandbox",
          onSelect = function()
            playerVeh:queueLuaCommand("recovery.saveHome()")
            return {'hide'}
          end
        })
      end
    end
  end})

  addEntry({ level = '/root/sandbox/quick/', generator = function(entries)

    table.insert(entries, {startSlot = 1, endSlot = 1, dynamicSlot = {id="sandbox_left"}})
    table.insert(entries, {startSlot = 2, endSlot = 2, dynamicSlot = {id="sandbox_upLeft"}})
    table.insert(entries, {startSlot = 3, endSlot = 3, dynamicSlot = {id="sandbox_up"}})
    table.insert(entries, {startSlot = 4, endSlot = 4, dynamicSlot = {id="sandbox_upRight"}})
    table.insert(entries, {startSlot = 5, endSlot = 5, dynamicSlot = {id="sandbox_right"}})
    table.insert(entries, {startSlot = 6, endSlot = 6, dynamicSlot = {id="sandbox_downRight"}})
    table.insert(entries, {startSlot = 7, endSlot = 7, dynamicSlot = {id="sandbox_down"}})
    table.insert(entries, {startSlot = 8, endSlot = 8, dynamicSlot = {id="sandbox_downLeft"}})
  end})

  addEntry({ level = '/root/sandbox/vehicles/', generator = function(entries)
    if not core_input_actionFilter.isActionBlocked("vehicle_selector") then
      local e = { title = 'ui.radialmenu2.Manage.Select', icon = 'vehicleFeatures01', startSlot = 1, folder = true, uniqueID = 'vehicle_selector',
        onSelect = function()
          M.setEnabled(false)
          guihooks.trigger('ChangeState', {state = 'menu.vehicles'})
          return {}
        end
      }
      table.insert(entries, e)
    end
    if not core_input_actionFilter.isActionBlocked("parts_selector") then
      local e = { title = 'ui.radialmenu2.Manage.Partsconfig', icon = 'engine', startSlot = 2, folder = true, uniqueID = 'partsconfig',
        onSelect = function()
          M.setEnabled(false)
          guihooks.trigger('ChangeState', {state = 'menu.vehicleconfig'})
          return {}
        end
      }
      table.insert(entries, e)
    end



    if not core_input_actionFilter.isActionBlocked("switch_next_vehicle") then

      table.insert(entries,{
        level = '/vehicles/', title = 'ui.radialmenu2.Manage.Remove', icon = 'trashBin1', startSlot = 7,
        enabled = be:getObjectCount() > 0 and getPlayerVehicle(0) ~= nil, uniqueID = 'removeVehicle',
        onSelect = function()
          core_vehicles.removeCurrent()
          extensions.hook("trackNewVeh")
          return {'hide'}
        end
      })
      table.insert(entries,{
        level = '/vehicles/', title = 'ui.radialmenu2.Manage.Clone', icon = 'copy', startSlot = 6, uniqueID = 'cloneVehicle',
        enabled = be:getObjectCount() > 0 and getPlayerVehicle(0) ~= nil,
        onSelect = function()
          core_vehicles.cloneCurrent()
          extensions.hook("trackNewVeh")
          return {'hide'}
        end
      })
      local currentVehicle = be:getPlayerVehicle(0)
      local vehName = {txt = 'ui.radialmenu2.Manage.yourVehicle', context = {name = M.getVehicleName(currentVehicle)}}
      table.insert(entries, {
        title = 'ui.radialmenu2.Manage.Switch', icon = 'arrowsReplace', startSlot = 3, uniqueID = 'switchVehiclesNext',
        enabled = be:getObjectCount() > 1,
        desc = vehName,
        onSelect = function()
          be:enterNextVehicle(0, 1)
          return {'reload'}
        end
      })
      table.insert(entries, {
        title = 'ui.radialmenu2.Manage.SwitchEllipsis', icon = 'carsChange', startSlot = 4, uniqueID = 'switchVehicles', ["goto"] = '/root/sandbox/vehicles/switch_vehicles/', ignoreAsRecentAction = true,
        desc = vehName,
        enabled = be:getObjectCount() > 1,
      })
    end
  end})

  addEntry({ level = '/root/sandbox/vehicles/switch_vehicles/', icon = 'cars', generator = function(entries)
    if be:getObjectCount() == 0 or core_input_actionFilter.isActionBlocked("switch_next_vehicle") then return end
    local vid = be:getPlayerVehicleID(0) or -1 -- matches all

    local function switchToVehicle(objid)
      local veh = getObjectByID(objid)
      if veh then
        be:enterVehicle(0, veh)
        return true
      end
    end

    for i = 0, be:getObjectCount()-1 do
      local veh = be:getObject(i)
      local targetVehId = veh:getId()
      --if targetVehId ~= vid then
        local vicon = "carNumber" .. (i%9)+1
        local vehKey = veh.JBeam
        local vehMainInfo = core_vehicles.getModel(vehKey)

        if vehMainInfo then
          local vehConfig = veh.partConfig
          local config_key = string.match(vehConfig, "vehicles/".. vehKey .."/(.*).pc")
          local configInfo = vehMainInfo.configs[config_key] or vehMainInfo.model
          if configInfo["Type"]=="PropParked" or configInfo["Type"]=="PropTraffic" then goto skipObj end
        end

        table.insert(entries, {
          title = M.getVehicleName(veh),
          icon = vicon,
          originalActionInfo = {level = "/root/sandbox/vehicles/", uniqueID = "switchVehicles"},
          folder = targetVehId == vid,
          color = targetVehId == vid and "#ff6600" or nil,
          onSelect = function()
            switchToVehicle(targetVehId)
            return {'reload'}
          end
        })
      --end
      ::skipObj::
    end
  end})

  addEntry({ level = "/root/sandbox/ai/", generator = function(entries)
    if not core_input_actionFilter.isActionBlocked("toggleAITraffic") and getPlayerVehicle(0) then
      table.insert(
        entries,
        {
          title = "ui.radialmenu2.ai.stop",
          desc = "ui.radialmenu2.ai.stop.desc",
          priority = 61,
          startSlot = 1,
          endSlot = 1,
          enabled = be:getObjectCount() > 1,
          icon = "parkingIndicator",
          uniqueID = 'aiStop',
          originalActionInfo = {level = "/root/sandbox/", uniqueID = "sandboxAi"},
          onSelect = function()
            core_vehicleBridge.executeAction(getPlayerVehicle(0),'setOtherVehiclesAIMode', "stop")
            return {"hide"}
          end
        }
      )
      table.insert(
        entries,
        {
          title = "ui.radialmenu2.ai.random",
          desc = "ui.radialmenu2.ai.random.desc",
          priority = 62,
          startSlot = 2,
          endSlot = 2,
          enabled = be:getObjectCount() > 1,
          icon = "arrowsShuffle",
          uniqueID = 'aiRandom',
          originalActionInfo = {level = "/root/sandbox/", uniqueID = "sandboxAi"},
          onSelect = function()
            core_vehicleBridge.executeAction(getPlayerVehicle(0),'setOtherVehiclesAIMode', "random")
            return {"hide"}
          end
        }
      )
      table.insert(
        entries,
        {
          title = "ui.radialmenu2.ai.flee",
          desc = "ui.radialmenu2.ai.flee.desc",
          priority = 64,
          startSlot = 3,
          endSlot = 3,
          enabled = be:getObjectCount() > 1,
          icon = "carsFlee",
          uniqueID = 'carsFlee',
          originalActionInfo = {level = "/root/sandbox/", uniqueID = "sandboxAi"},
          onSelect = function()
            core_vehicleBridge.executeAction(getPlayerVehicle(0),'setOtherVehiclesAIMode', "flee")
            return {"hide"}
          end
        }
      )
      table.insert(
        entries,
        {
          title = "ui.radialmenu2.ai.chase",
          desc = "ui.radialmenu2.ai.chase.desc",
          priority = 65,
          startSlot = 4,
          endSlot = 4,
          enabled = be:getObjectCount() > 1,
          icon = "carsChase",
          uniqueID = 'aiChase',
          originalActionInfo = {level = "/root/sandbox/", uniqueID = "sandboxAi"},
          onSelect = function()
            core_vehicleBridge.executeAction(getPlayerVehicle(0),'setOtherVehiclesAIMode', "chase")
            return {"hide"}
          end
        }
      )
      table.insert(
        entries,
        {
          title = "ui.radialmenu2.ai.follow",
          desc = "ui.radialmenu2.ai.follow.desc",
          priority = 66,
          startSlot = 5,
          endSlot = 5,
          enabled = be:getObjectCount() > 1,
          icon = "carsFollow",
          uniqueID = 'aiFollow',
          originalActionInfo = {level = "/root/sandbox/", uniqueID = "sandboxAi"},
          onSelect = function()
            core_vehicleBridge.executeAction(getPlayerVehicle(0),'setOtherVehiclesAIMode', "follow")
            return {"hide"}
          end
        }
      )

      -- Todo: if no route it set, generate a random route (and say so in the description)
      local rp = core_groundMarkers.routePlanner
      local enabled = true
      if not rp or not rp.path or not rp.path[1] then
        enabled = false
      end
      local followRoute = {
        title = "ui.radialmenu2.ai.followRoute",
        desc = "ui.radialmenu2.ai.followRoute.desc",
        enabled = enabled,
        priority = 67,
        startSlot = 6,
        endSlot = 6,
        icon = "routeSimple",
        uniqueID = 'aiFollowRoute',
        onSelect = function()
          local playerVeh = be:getPlayerVehicle(0)
          if not playerVeh then return end
          local playerVehId = playerVeh:getID()
          local affectedVehicleCount = 0
          for i = 0, be:getObjectCount()-1 do
            local veh = be:getObject(i)
            if veh:getPosition():distance(playerVeh:getPosition()) > 50 then goto continue end
            if veh:getID() == playerVehId then goto continue end
            local aiPath = getAiPath(rp.path, i)
            local str = '{wpTargetList = '..serialize(aiPath)
            str = str..', noOfLaps = 1, aggression = 0.3, avoidCars = "on", driveInLane = "on", speedMode = "limit"}'
            affectedVehicleCount = affectedVehicleCount + 1
            veh:queueLuaCommand('ai.driveUsingPath('..str..')')
            ::continue::
          end
          return {"hide"}
        end
      }
      table.insert(entries, followRoute)

      local raceRoute = {
        title = "ui.radialmenu2.ai.raceRoute",
        desc = "ui.radialmenu2.ai.raceRoute.desc",
        enabled = enabled,
        priority = 67,
        startSlot = 7,
        endSlot = 7,
        icon = "routeSimpleFlag",
        uniqueID = 'aiRaceRoute',
        onSelect = function()
          local playerVeh = be:getPlayerVehicle(0)
          if not playerVeh then return end
          local playerVehId = playerVeh:getID()
          local affectedVehicleCount = 0
          for i = 0, be:getObjectCount()-1 do
            local veh = be:getObject(i)
            if veh:getPosition():distance(playerVeh:getPosition()) > 50 then goto continue end
            if veh:getID() == playerVehId then goto continue end
            local aiPath = getAiPath(rp.path, i)
            local str = '{wpTargetList = '..serialize(aiPath)
            str = str..', noOfLaps = 1, aggression = 0.9, avoidCars = "on"}'
            local veh = be:getObject(i)
            veh:queueLuaCommand('ai.driveUsingPath('..str..')')
            veh:queueLuaCommand('ai.setRacing(true)')
            affectedVehicleCount = affectedVehicleCount + 1
            ::continue::
          end
          return {"hide"}
        end
      }
      table.insert(entries, raceRoute)

      table.insert(
        entries,
        {
          title = "ui.radialmenu2.ai.disableMyself",
          desc = "ui.radialmenu2.ai.disableMyself.desc",
          priority = 67,
          startSlot = 8,
          endSlot = 8,
          icon = "circleSlashed",
          uniqueID = 'aiDisableMyself',
          originalActionInfo = {level = "/root/sandbox/", uniqueID = "sandboxAi"},
          onSelect = function()
            core_vehicleBridge.executeAction(getPlayerVehicle(0),'setAIMode', "disable")
            return {"hide"}
          end
        }
      )
    end
  end})

  addEntry({ level = '/root/sandbox/traffic/', generator = function(entries)
    if not core_input_actionFilter.isActionBlocked("toggleTraffic") then
      table.insert(entries, { title = 'ui.radialmenu2.traffic.stop', desc = "ui.radialmenu2.traffic.stop.desc", priority = 61, icon = 'pause',
        originalActionInfo = {level = "/root/sandbox/", uniqueID = "traffic"},
        uniqueID = 'trafficStop',
        onSelect = function()
        extensions.gameplay_traffic.deactivate(true)
        extensions.hook("stopTracking", ({Name = "TrafficEnabled"}))
        return {"hide"}
      end})
      table.insert(entries, { title = 'ui.radialmenu2.traffic.remove', desc = "ui.radialmenu2.traffic.remove.desc", priority = 62, icon = 'trashBin1',
        originalActionInfo = {level = "/root/sandbox/", uniqueID = "traffic"},
        uniqueID = 'trafficRemove',
        onSelect = function()
          extensions.gameplay_parking.deleteVehicles()
          extensions.gameplay_traffic.deleteVehicles()
        extensions.hook("stopTracking", ({Name = "TrafficEnabled"}))
        return {"hide"}
      end})
      table.insert(entries, { title = 'ui.radialmenu2.traffic.spawnNormal', desc = "ui.radialmenu2.traffic.spawnNormal.desc", priority = 63, icon = 'cars',
        originalActionInfo = {level = "/root/sandbox/", uniqueID = "traffic"},
        uniqueID = 'trafficSpawnNormal',
        onSelect = function()
          extensions.gameplay_traffic.setupTrafficWaitForUi(false)
          extensions.hook("startTracking", ({Name = "TrafficEnabled"}))
          return {"hide"}
        end})
      table.insert(entries, { title = 'ui.radialmenu2.traffic.spawnPolice', desc = "ui.radialmenu2.traffic.spawnPolice.desc", priority = 64, icon = 'carChase01',
        originalActionInfo = {level = "/root/sandbox/", uniqueID = "traffic"},
        uniqueID = 'trafficSpawnPolice',
        onSelect = function()
          extensions.gameplay_traffic.setupTrafficWaitForUi(true)
          extensions.hook("startTracking", ({Name = "TrafficEnabled"}))
          return {"hide"}
      end})
      table.insert(entries, { title = 'ui.radialmenu2.traffic.start', desc = "ui.radialmenu2.traffic.start.desc", priority = 65, icon = 'play',
        originalActionInfo = {level = "/root/sandbox/", uniqueID = "traffic"},
        uniqueID = 'trafficStart',
        onSelect = function()
          extensions.gameplay_traffic.activate()
          extensions.gameplay_traffic.setTrafficVars({aiMode = "traffic", enableRandomEvents = true})
          extensions.hook("startTracking", ({Name = "TrafficEnabled"}))
          return {"hide"}
      end})
    end
  end})

end

local function registerPlayerVehicleMenus()
  -- see also quickAccess.lua in vlua folder
  addEntry({ level = '/root/playerVehicle/helperSystems/', generator = function(entries)
    if getPlayerVehicle(0) and not core_input_actionFilter.isActionBlocked("switch_camera_next") then
      local desc = ""
      if not settings.getValue('GraphicDynMirrorsEnabled') then
        desc = "Enable Dynamic Mirrors in the Graphics section of the Settings menu to use this feature."
      end
      table.insert(entries, {
        title = "ui.radialmenu2.Mirrors", icon = "mirrorInteriorMiddle", priority = 95,
        uniqueID = "toggleMirrors",
        enabled = settings.getValue('GraphicDynMirrorsEnabled'),
        desc = desc,
        onSelect = function()
          M.setEnabled(false)
          guihooks.trigger('ChangeState', {state = 'menu.vehicleconfig.tuning.mirrors', params = {exitRoute = "play"}})
          return {}
        end
      })
    end
  end})
  addEntry({ level = '/root/playerVehicle/general/', generator = function(entries)
    table.insert(entries, {dynamicSlot = {id = "vehicle_left"}, startSlot = 1, endSlot = 1, ignoreAsRecentActionForCategory = "playerVehicle" })
    table.insert(entries, {dynamicSlot = {id = "vehicle_upLeft"}, startSlot = 2, endSlot = 2, ignoreAsRecentActionForCategory = "playerVehicle" })
    table.insert(entries, {dynamicSlot = {id = "vehicle_up"}, startSlot = 3, endSlot = 3, ignoreAsRecentActionForCategory = "playerVehicle" })
    table.insert(entries, {dynamicSlot = {id = "vehicle_upRight"}, startSlot = 4, endSlot = 4, ignoreAsRecentActionForCategory = "playerVehicle" })
    table.insert(entries, {dynamicSlot = {id = "vehicle_right"}, startSlot = 5, endSlot = 5, ignoreAsRecentActionForCategory = "playerVehicle" })
    table.insert(entries, {dynamicSlot = {id = "vehicle_downLeft"}, startSlot = 6, endSlot = 6, ignoreAsRecentActionForCategory = "playerVehicle" })
    table.insert(entries, {dynamicSlot = {id = "vehicle_down"}, startSlot = 7, endSlot = 7, ignoreAsRecentActionForCategory = "playerVehicle" })
    table.insert(entries, {dynamicSlot = {id = "vehicle_downRight"}, startSlot = 8, endSlot = 8, ignoreAsRecentActionForCategory = "playerVehicle" })

  end})
  addEntry({ level = '/root/playerVehicle/vehicleFeatures/', generator = function(entries)
    -- Add vehicle entry/exit action
    if gameplay_walk.isTogglingEnabled() and not core_input_actionFilter.isActionBlocked("toggleWalkingMode") then
      local vehicleInFront = gameplay_walk.getVehicleInFront()
      local title = gameplay_walk.isWalking() and
      (vehicleInFront and "ui.radialmenu2.enterVehicle" or "ui.radialmenu2.noVehicleToEnter") or
      "ui.radialmenu2.exitVehicle"

      local e = {
        title = title, icon = gameplay_walk.isWalking() and "seatArrowInLeft" or "seatArrowOut", startSlot = 1, priority = 94, ignoreAsRecentActionForCategory = "playerVehicle",
        enabled = not gameplay_walk.isWalking() or vehicleInFront ~= nil, uniqueID = "toggleWalkingMode",
        onSelect = function()
          gameplay_walk.toggleWalkingMode()
          return {'hide'}
        end
      }
      if not gameplay_walk.isAtParkingSpeed() then
        e.enabled = false
        e.disableReason = "Vehicle needs to be stopped."
      end
      table.insert(entries, e)
    end
  end})
end

local function registerDefaultMenus()
  registerRootMenus()
  registerPlayerVehicleMenus()
  registerSandboxMenus()


  if debugAddVehicleTriggers then
    addEntry({ level = '/root/playerVehicle/general/', generator = function(entries)
      if not getPlayerVehicle(0) then return end
      local driverNode = core_camera.getDriverData(getPlayerVehicle(0)) or 0
      local driverPos = getPlayerVehicle(0):getPosition() + getPlayerVehicle(0):getNodePosition(driverNode)
      local vehIds = {}
      local triggerEntries = {}

      for i = 0, be:getObjectCount() - 1 do
        local veh = be:getObject(i)
        local vehId = veh:getId()
        local vData = extensions.core_vehicle_manager.getVehicleData(vehId)
        if vData and vData.vdata and type(vData.vdata.triggers) == 'table' then
          for _, trg in pairs(vData.vdata.triggers or {}) do
            local trigger = veh:getTrigger(trg.abid)
            local triggerPos = trigger:getCenter()
            if triggerPos:distance(driverPos) < 2 then
              vehIds[vehId] = true
              local e = {title = trg.name, titleWithVehicleName = vData.vdata.information.name .. " " .. trg.name,
              onSelect = function()
                core_vehicleTriggers.triggerEvent("action0", 1, trg.abid, vehId, vData.vdata)
                return {'temporaryHide'}
              end,
              onDeselect = function()
                core_vehicleTriggers.triggerEvent("action0", 0, trg.abid, vehId, vData.vdata)
                return {'temporaryUnhide'}
              end,
              onSecondarySelect = function()
                core_vehicleTriggers.triggerEvent("action2", 1, trg.abid, vehId, vData.vdata)
                return {'temporaryHide'}
              end,
              onSecondaryDeselect = function()
                core_vehicleTriggers.triggerEvent("action2", 0, trg.abid, vehId, vData.vdata)
                return {'temporaryUnhide'}
              end}
              table.insert(triggerEntries, e)
            end
          end
        end
      end

      -- if there are triggers by multiple vehicles nearby, add the name of the vehicle as well
      for _, triggerEntry in ipairs(triggerEntries) do
        if tableSize(vehIds) > 1 then
          triggerEntry.title = triggerEntry.titleWithVehicleName
        end
        triggerEntry.titleWithVehicleName = nil
        table.insert(entries, triggerEntry)
      end
    end})
  end
end

local function countLevels(path)
  local count = 0
  for _ in path:gmatch("/[^/]+") do
    count = count + 1
  end
  return count
end

local function getParentPaths(path)
  local paths = {}
  local pattern = "(.+)/[^/]+/"

  -- Add all parent paths
  while true do
    table.insert(paths, path)
    local parentPath = path:match(pattern)
    if not parentPath then break end
    path = parentPath .. "/"
  end

  -- Reverse the order so shortest paths come first
  for i = 1, math.floor(#paths / 2) do
    paths[i], paths[#paths - i + 1] = paths[#paths - i + 1], paths[i]
  end

  return paths
end

local slotSize = 1 / maxSlotAmount

local function calculateGapsBetweenButtons()
  local gaps = {}
  if #currentMenuItems > 0 then
    local positionedItems = {}
    for _, item in ipairs(currentMenuItems) do
      if item.position then
        table.insert(positionedItems, {
          startPos = (item.position - item.size/2) % 1,
          endPos = (item.position + item.size/2) % 1
        })
      end
    end

    if #positionedItems == 0 then
      -- No positioned buttons, add single gap from 0 to 1
      table.insert(gaps, {
        startPos = 1 - slotSize / 2,
        size = 1
      })
    else
      -- Sort items by position for gap calculation
      table.sort(positionedItems, function(a,b) return a.startPos < b.startPos end)

      -- Find gaps between consecutive items
      for i = 1, #positionedItems do
        local curr = positionedItems[i]
        local next = positionedItems[i % #positionedItems + 1]

        local gapStart = curr.endPos
        local gapEnd = next.startPos

        -- Handle wrap-around case
        if next.startPos < curr.endPos then
          gapEnd = next.startPos + 1
        end

        table.insert(gaps, {
          startPos = gapStart,
          size = gapEnd - gapStart
        })
      end
    end
  end
  return gaps
end

local function isUsingOccupiedSlots(startSlot, endSlot, occupiedSlots)
  endSlot = endSlot or startSlot

  -- Normalize slots to be between 1 and maxSlotAmount
  startSlot = ((startSlot - 1) % maxSlotAmount) + 1
  endSlot = ((endSlot - 1) % maxSlotAmount) + 1

  -- Handle wrap-around case
  if endSlot < startSlot then
    endSlot = endSlot + maxSlotAmount
  end

  -- Check each occupied slot to see if it falls within our range
  for slot, _ in pairs(occupiedSlots) do
    local normalizedSlot = ((slot - 1) % maxSlotAmount) + 1
    if normalizedSlot >= startSlot and normalizedSlot <= endSlot then
      return true
    end
    -- Handle wrap-around case for occupied slots
    if normalizedSlot + maxSlotAmount >= startSlot and normalizedSlot + maxSlotAmount <= endSlot then
      return true
    end
  end

  -- If no conflicts, mark our slots as occupied
  local occupiedSlotsThisAction = {}
  for slot = startSlot, endSlot do
    local normalizedSlot = ((slot - 1) % maxSlotAmount) + 1
    occupiedSlotsThisAction[normalizedSlot] = true
  end

  tableMerge(occupiedSlots, occupiedSlotsThisAction)
  return false
end

local uiData
-- we got all the data required, show the menu
local function _assembleMenuComplete()
  if not currentMenuItems then return end

  local objID = be:getPlayerVehicleID(0)
  local pathBeforeCategory = currentLevel:match("^/([^/]+)/")
  local currentFirstLevelId = currentLevel:match("^/([^/]+)/")
  local currentSecondLevelId = currentLevel:match("^/[^/]+/([^/]+)/")
  local currentThirdLevelId = currentLevel:match("^/[^/]+/[^/]+/([^/]+)/")

  local backButtonIndex
  if #callStack > 0 then
    local mid = math.floor(#currentMenuItems/2)+1
    backButtonIndex = mid
  end

  -- convert slots to positions and sizes
  local occupiedSlots = {} -- Track occupied slots
  for _, e in ipairs(currentMenuItems) do
    if e.enabled == nil then e.enabled = true end
    if e.startSlot then
      local slotPosition = e.startSlot
      if e.endSlot then
        local normalizedEndSlot = (e.endSlot + (e.startSlot > e.endSlot and maxSlotAmount or 0))
        slotPosition = (e.startSlot + normalizedEndSlot) / 2
        if slotPosition > maxSlotAmount then slotPosition = slotPosition - maxSlotAmount end
        e.size = math.abs(e.startSlot - normalizedEndSlot) + 1
      else
        e.size = 1
      end
      if not isUsingOccupiedSlots(e.startSlot, e.endSlot, occupiedSlots) then
        e.position = (slotPosition - 1) % maxSlotAmount / maxSlotAmount
        e.size = e.size / maxSlotAmount
      else
        log("W", "", "two radial buttons have overlapping slots. Removing slots from one of them")
      end
    end
    e.icon = updateIcon(e.icon, e.title)
    if e["goto"] then
      e.folder = true
    end
  end

  -- calculate gaps between positioned buttons
  local gaps = calculateGapsBetweenButtons(slotSize)

  -- if there are no gaps, remove all positions and sizes and place all buttons dynamically
  if tableIsEmpty(gaps) then
    for _, e in ipairs(currentMenuItems) do
      e.position = nil
      e.size = nil
    end
    gaps = {{
      startPos = 1 - slotSize / 2,
      size = 1
    }}
  end

  -- gap button sizes
  local gapButtonSizes = {}
  for i, gap in ipairs(gaps) do
    gapButtonSizes[i] = slotSize
  end

  -- calculate how many buttons are not placed yet
  local buttonsToBePlaced = 0
  for _, item in ipairs(currentMenuItems) do
    if not item.position then
      buttonsToBePlaced = buttonsToBePlaced + 1
    end
  end

  -- calculate how many buttons can be placed in each gap
  for i, gap in ipairs(gaps) do
    local maxButtonsInGap = math.floor(gap.size / gapButtonSizes[i])
    gap.fittingButtonsAmount = math.min(maxButtonsInGap, buttonsToBePlaced)
    buttonsToBePlaced = buttonsToBePlaced - gap.fittingButtonsAmount
  end

  -- place remaining buttons in gaps that minimize button size changes
  while buttonsToBePlaced > 0 do
    local bestGap = 1
    local biggestButtonSize = 0

    -- find gap where adding a button causes smallest change to button sizes
    for i, gap in ipairs(gaps) do
      local newButtonSize = gap.size / (gap.fittingButtonsAmount + 1)

      if newButtonSize > biggestButtonSize then
        biggestButtonSize = newButtonSize
        bestGap = i
      end
    end

    gaps[bestGap].fittingButtonsAmount = gaps[bestGap].fittingButtonsAmount + 1
    buttonsToBePlaced = buttonsToBePlaced - 1
  end

  -- try to find the first fitting empty space for each leftover button
  for _, e in ipairs(currentMenuItems) do
    if not e.position then
      -- Find first gap that can fit another button
      for gapIndex, gap in ipairs(gaps) do
        if gap.fittingButtonsAmount > 0 then
          -- Calculate button size to fill gap evenly
          local buttonSize = math.min(slotSize, gap.size / gap.fittingButtonsAmount)

          -- Calculate position within gap
          local position = (gap.startPos + (buttonSize / 2)) % 1

          -- Place button and update gap capacity
          e.position = position
          e.size = buttonSize

          gap.fittingButtonsAmount = gap.fittingButtonsAmount - 1
          gap.startPos = gap.startPos + buttonSize
          gap.size = gap.size - buttonSize
          break
        end
      end
    end
  end

  table.sort(currentMenuItems, function(a, b)
    if not a.position then return false end
    if not b.position then return true end
    return a.position < b.position
  end)

  local categoriesList = {}
  if currentSecondLevelId then
    pathBeforeCategory = pathBeforeCategory .. "/" .. currentSecondLevelId
    local categoryItems = menuTreeCopy["/" .. pathBeforeCategory .. "/"]

    for _, item in ipairs(categoryItems or {}) do
      local id = item["goto"]:match("^/[^/]+/" .. currentSecondLevelId .. "/([^/]+)/")
      if not item.title then item.title = toNiceName(str) end
      item.id = id

      table.insert(categoriesList, item)
    end

    sort_categories(categoriesList)
  else
    pathBeforeCategory = nil
  end

  -- get the parent paths of the current level to get the breadcrumbs
  local levelPaths = getParentPaths(currentLevel)
  table.remove(levelPaths, 1)
  local breadcrumbs = {}
  for _, path in ipairs(levelPaths) do
    -- remove the last level from path to get parent path
    local parentPath = path:gsub("/[^/]+/$", "/")
    local menuItems = menuTreeCopy[parentPath]
    if menuItems then
      for _, item in ipairs(menuItems) do
        if (path == "/root/playerVehicle/" and item["goto"] == playerVehicleDefaultPath) or
           (path == "/root/sandbox/" and item["goto"] == sandboxDefaultPath) or
           (path == "/root/sandbox/" and item["goto"] == careerDefaultPath) or
           (path == "/root/sandbox/" and item["goto"] == missionDefaultPath) or
           item["goto"] == path then
          table.insert(breadcrumbs, item.title or toNiceName(item["goto"]:match("[^/]+$")))
          break
        end
      end
    end
  end
  local menuIcon = "beamNG" -- default fallback icon
  -- Get the first segment after root in the current path
  local firstSegment = currentLevel:match("^/root/([^/]+)/")
  if firstSegment then
    -- Look for matching item in root level
    local rootItems = menuTreeCopy["/root/"] or {}
    for _, item in ipairs(rootItems) do
      if item["goto"] and item["goto"]:match("^/root/" .. firstSegment .. "/") then
        menuIcon = item.icon or menuIcon
        break
      end
    end
  end

  if currentSecondLevelId == "dynamicConfig" then
    categoriesList = {}
    breadcrumbs = {}
    local setting = dynamicSlotSettings[currentThirdLevelId]
    if setting then
      breadcrumbs = setting.breadcrumbs
    end
  end

  uiData = {
    canGoBack = #callStack > 0,
    items = currentMenuItems,
    categories = categoriesList,
    pathBeforeCategory = pathBeforeCategory,
    selectedCategory = currentThirdLevelId,
    backButtonIndex = backButtonIndex,
    currentLevel = currentLevel,
    breadcrumbs = breadcrumbs,
    menuIcon = menuIcon,
    hasLRShoulderButtons = currentFirstLevelId == "root" and currentSecondLevelId == nil,
  }

  if simTimeBefore == nil then
    if simTimeAuthority.get() < slowMoFactor then
      simTimeBefore = false
    else
      simTimeBefore = simTimeAuthority.get()
      simTimeAuthority.set(slowMoFactor)
    end
  end

  core_sounds.setAudioBlur(1)

  if not uiVisible then
      -- Store current game state before changing it
    if gameStateBefore ~= nil then
      log("E", "quickAccess", "gameStateBefore is not nil")
    else
      gameStateBefore = deepcopy(core_gamestate.state)
    end
    guihooks.trigger('ChangeState', {state ='Radial'})
    core_gamestate.setGameState(nil,'radial')
  end
  guihooks.trigger('radialMenuUpdated')

  uiVisible = true
end

local function getUiData()
  return uiData
end

local function isMenuEmpty(level)
  local entries = deepcopy(menuTreeCopy[level] or {}) -- make a copy, the generators modify the menu below, this should not be persistent
  local menuItems = {}

  for _, e in ipairs(entries) do
    if type(e) == 'table' then
      if type(e.generator) == 'function' then
        e.generator(entries)
      else
        table.insert(menuItems, e)
      end
    end
  end

  for _, menuItem in ipairs(menuItems) do
    if not (menuItem["goto"] or menuItem.generator) then
      return false
    end
  end

  for _, menuItem in ipairs(menuItems) do
    if menuItem["goto"] and not isMenuEmpty(menuItem["goto"]) then
      return false
    end
  end

  return true
end

local function generateCompleteTree(level)
  local menuTreeCopy = deepcopy(menuTree)
  currentMenuItems = {}

  for path, items in pairs(menuTreeCopy) do
    for _, e in ipairs(items) do
      if type(e) == "table" then
        if type(e.generator) == "function" then
          e.generator(items)
        end
      end
    end
  end

  for path, items in pairs(menuTreeCopy) do
    for i = #items, 1, -1 do
      local item = items[i]
      if item.generator then
        table.remove(items, i)
      end
    end
  end

  return menuTreeCopy
end

-- recursively remove empty levels from menuTreeCopy
local function removeEmptyLevels(menuTreeCopy)
  local hasChanges = false

  -- Protected paths from favoriteActionsPaths
  local protectedPaths = {
    [sandboxDefaultPath] = true,
    [playerVehicleDefaultPath] = true,
    [careerDefaultPath] = career_career.isActive(),
    [missionDefaultPath] = gameplay_missions_missionManager.getForegroundMissionId() ~= nil,
  }
  --for _, path in pairs(favoriteActionsPaths) do
  --  protectedPaths[path] = true
  --end

  -- First pass: check all levels for emptiness
  for level, items in pairs(menuTreeCopy) do
    if level ~= "/root/" and not protectedPaths[level] then
      local isEmpty = true
      for _, item in ipairs(items) do
        if not item.ignoreForCheckingIfMenuIsEmpty then
          if not item["goto"] then
            isEmpty = false
            break
          elseif item["goto"] and not isMenuEmpty(item["goto"]) then
            isEmpty = false
            break
          end
        end
      end

      if isEmpty then
        --log("I", "quickAccess", "removing level " .. dumps(level))
        menuTreeCopy[level] = nil
        hasChanges = true
      end
    end
  end

  -- If we removed any levels, we need to do another pass to clean up goto references
  if hasChanges then
    for level, items in pairs(menuTreeCopy) do
      for i = #items, 1, -1 do
        local item = items[i]
        if item["goto"] and not menuTreeCopy[item["goto"]] and not protectedPaths[item["goto"]] then
          if item.dynamicSlot then
            table.remove(items, i)
          else
            table.remove(items, i)
          end
        end
      end
    end
  end

  return menuTreeCopy
end


local function getMostRecentAvailableActions(category, limit, dynamicItems)
  if limit == nil or limit == 0 then
    return {}
  end
  local skipActions = {}
  for _, item in ipairs(dynamicItems or {}) do
    if item.dynamicSlotSetting.mode == "uniqueAction" then
      table.insert(skipActions, {uniqueID = item.dynamicSlotSetting.uniqueID, level = item.dynamicSlotSetting.level})
    end
  end
  if dynamicItems and not limit then
    limit = #dynamicItems - #skipActions
  end


  -- Default limit if not provided
  limit = limit or maxRecentActionsLimits[category] or 4

  -- Copy the actions from the category
  local actions = {}
  for i, action in ipairs(recentActions[category] or {}) do
    action._sortIdx = i
    table.insert(actions, action)
  end

  -- Sort by timestamp (most recent first)
  table.sort(actions, function(a, b)
    return (a.timestamp or 0) > (b.timestamp or 0)
  end)

  -- Filter to available actions and limit the count
  local result = {}
  for _, action in ipairs(actions) do
    -- Check if action is available (exists in menuTreeCopy)
    local actionInfo = getActionInfo(action.level, action.uniqueID)
    if actionInfo then
      -- compare the action with the skipActions
      local compare = {uniqueID = action.uniqueID, level = action.level}
      if actionInfo.originalActionInfo then
        compare = {uniqueID = actionInfo.originalActionInfo.uniqueID, level = actionInfo.originalActionInfo.level}
      end
      local skip = false
      for _, skipAction in ipairs(skipActions) do
        if (skipAction.uniqueID == compare.uniqueID and skipAction.level == compare.level) then
          skip = true
          break
        end
      end

      if not skip and #result < limit then
        table.insert(result, action)
      end

      -- Stop once we have enough actions
      if #result >= limit then
        break
      end
    end
  end

  table.sort(result, function(a, b)
    return a._sortIdx < b._sortIdx
  end)


  return result
end

local function show(level, getVehicleItems)
  if getVehicleItems == nil then getVehicleItems = true end

  if type(level) ~= 'string' then
    if getPlayerVehicle(0) then
      level = playerVehicleDefaultPath
    else
      level = sandboxDefaultPath
    end
    level = "/root/"
  end


  -- TODO disabled this for testing
  -- if the level is a category, set the callstack to {}
  --[[ if countLevels(level) == 3 then
    callStack = {}
  end ]]

  currentLevel = level

  -- now ask the active vehicle for any items
  local vehicle = getPlayerVehicle(0)
  if vehicle and getVehicleItems then
    vehicle:queueLuaCommand('extensions.core_quickAccess.requestItems("' .. tostring(currentLevel) .. '")')
    -- we give the vehicle 4 gfx frames to add items
    vehicleWaitFrames = 4
    return
  end

  extensions.hook("onBeforeRadialOpened")

  menuTreeCopy = generateCompleteTree(currentLevel)


  local playerVehId = be:getPlayerVehicleID(0)
  if playerVehId ~= -1 then
    for k, vehicleLevel in pairs(vehicleMenuTrees[playerVehId]) do
      if menuTreeCopy[k] then
        for _, item in ipairs(vehicleLevel) do
          table.insert(menuTreeCopy[k], item)
        end
      else
        menuTreeCopy[k] = vehicleLevel
      end
    end
  end

  -- add any missing "goto" buttons
  ::gotoButtonsStart::
  for levelPath, levelInfo in pairs(menuTreeCopy) do
    if countLevels(levelPath) >= 2 then
      local allLevels = getAllLevels(levelPath)
      for i, subLevel in ipairs(allLevels) do
        if i > 1 then
          local nextLevel = allLevels[i+1]
          if nextLevel then
            if not gotoButtonExists(nextLevel, subLevel, menuTreeCopy) then
              local item = {
                ["goto"] = nextLevel,
                icon = "material_traffic",
                priority = 53,
                title = toNiceName(nextLevel:match("/([^/]+)/?$"))
              }
              menuTreeCopy[subLevel] = menuTreeCopy[subLevel] or {}
              table.insert(menuTreeCopy[subLevel], item)
              goto gotoButtonsStart
            end
          end
        end
      end
    end
  end
  for level, tree in pairs(menuTreeCopy) do
    local dynamicItems = {}
    local recentActionItems = 0
    for _, item in ipairs(tree or {}) do
      if item.dynamicSlot then
        item.dynamicSlotSetting = deepcopy(dynamicSlotSettings[item.dynamicSlot.id])
        table.insert(dynamicItems, item)
        if item.dynamicSlotSetting.mode == "recentActions" then
          recentActionItems = recentActionItems + 1
        end
      end
    end
    if not next(dynamicItems)  then
      goto noDynamicItems
    end

    table.sort(dynamicItems, function(a, b)
      return a.dynamicSlotSetting.priority < b.dynamicSlotSetting.priority
    end)

    local recentActions = getMostRecentAvailableActions("all", recentActionItems, dynamicItems)
    local recentActionsIdx = 1

    for _, item in ipairs(dynamicItems) do
      item.marking = "invisible"
      item.contextAction = function() M.openDynamicSlotConfigurator(item.dynamicSlot.id) end
      item.enabled = false
      item.marking = "outline"

      local recentAction = recentActions[recentActionsIdx]
      local actionInfo = nil
      local originalActionInfo = nil

      if item.dynamicSlotSetting.mode == "recentActions" then
        item.marking = "solid"
        if recentAction then
          recentActionsIdx = recentActionsIdx + 1
          actionInfo = deepcopy(getActionInfo(recentAction.level, recentAction.uniqueID))
          originalActionInfo = {level = recentAction.level, uniqueID = recentAction.uniqueID}
        else
          item.desc = "(This slot shows recently used actions.)"
          item.icon = "redo"
        end
      elseif item.dynamicSlotSetting.mode == "uniqueAction" then
        item.marking = "star"
        actionInfo = deepcopy(getActionInfo(item.dynamicSlotSetting.level, item.dynamicSlotSetting.uniqueID))
        originalActionInfo = {level = item.dynamicSlotSetting.level, uniqueID = item.dynamicSlotSetting.uniqueID}
        if actionInfo and actionInfo.originalActionInfo then
          originalActionInfo = actionInfo.originalActionInfo
        end
        if not actionInfo then
          item.desc = "(The action assigned to this slot is not available right now.)"
          item.icon = "unlink"
        end
      end
      if actionInfo then
        if actionInfo.enabled ~= false then
          item.enabled = true
        end
        actionInfo.startSlot = nil
        actionInfo.endSlot = nil
        tableMerge(item, actionInfo)
        item.isRecentAction = true
        item.originalActionInfo = originalActionInfo
      else
        item.desc = item.desc or "(This slot can be configured to show a custom or recent action.)"
        item.ignoreForCheckingIfMenuIsEmpty = true
      end
    end
    ::noDynamicItems::
  end

  -- Remove empty levels from menuTreeCopy
  -- TODO this will remove the quick menu if it is empty, not sure if that should stay that way
  menuTreeCopy = removeEmptyLevels(menuTreeCopy)

  for _, item in ipairs(menuTreeCopy[currentLevel] or {}) do
    table.insert(currentMenuItems, item)
  end

  _assembleMenuComplete()

  return true
end

local function vehicleItemsCallback(objID, level, vehicleMenuTree)
  if currentLevel == nil then
    return
  end

  vehicleMenuTrees[objID] = vehicleMenuTree

  -- no need to wait anymore
  vehicleWaitFrames = 0
  show(level, false)
end

-- go to another level, saving the history
local function gotoLevel(level)
  if currentLevel ~= nil then
    table.insert(callStack, currentLevel)
  end
  return show(level)
end
M.gotoLevel = gotoLevel

local function hide()
  extensions.hook("onHideRadialMenu")
  if uiVisible then
    -- Restore previous game state
    --dump("Hide: gameStateBefore")
    --dump(gameStateBefore)
    if gameStateBefore ~= nil then
      guihooks.trigger('ChangeState', {state ='play'})
      core_gamestate.setGameState(gameStateBefore.state, gameStateBefore.appLayout, gameStateBefore.menuItems, gameStateBefore.options)
      gameStateBefore = nil
    end
  end
  currentMenuItems = nil
  currentLevel = nil
  vehicleWaitFrames = 0
  uiVisible = false

  if simTimeBefore then
    simTimeAuthority.set(simTimeBefore)
  end
  simTimeBefore = nil
  core_sounds.setAudioBlur(0)
end

local function reload()
  if currentLevel then show(currentLevel) end
end

local function back()
  if currentLevel == nil then return end -- not visible: no way to go back, return
  if not callStack or #callStack == 0 then
    -- at top to the history: close?
    --return hide()
    return false
  end

  local oldLevel = callStack[#callStack]
  table.remove(callStack, #callStack)
  show(oldLevel)
end

local function temporaryHide()
  guihooks.trigger('RadialTemporaryHide', true)
  simTimeAuthority.set(1)
  core_sounds.setAudioBlur(0)
end

local function temporaryUnhide()
  guihooks.trigger('RadialTemporaryHide', false)
  simTimeAuthority.set(slowMoFactor)
  core_sounds.setAudioBlur(1)
end

local function itemSelectCallback(actionResult)
  log('D', 'quickaccess.itemSelectCallback', 'called: ' .. dumps(actionResult))
  if type(actionResult) ~= 'table' then
    log('E', 'quickaccess.itemSelectCallback', 'invalid item result args: ' .. dumps(actionResult))
    return
  end
  if actionResult[1] == 'hide' then
    hide()
  elseif actionResult[1] == 'reload' then
    reload()
  elseif actionResult[1] == 'goto' then
    gotoLevel(actionResult[2])
  elseif actionResult[1] == 'back' then
    back()
  elseif actionResult[1] == 'hideMeOnly' then
    hide()
  elseif actionResult[1] == 'temporaryHide' then
    temporaryHide()
  elseif actionResult[1] == 'temporaryUnhide' then
    temporaryUnhide()
  elseif actionResult[1] == 'hideAndIgnoreSlowmotion' then
    simTimeBefore = nil
    hide()
  end
end

local function clearRecentActions()
  log("I", "", "clearing recent actions")
  recentActions = {
    playerVehicle = {},
    sandbox = {},
    other = {},  -- For any actions that don't fit the main categories
    all = {}     -- Combined list of all recent actions
  }
end
M.clearRecentActions = clearRecentActions

local function trackRecentAction(item, currentLevel)
  if not (item.uniqueID or item.originalActionInfo) or item.ignoreAsRecentAction then
    return
  end

  -- Create a record of the action
  local actionRecord = {
    uniqueID = item.uniqueID,
    level = currentLevel,
    title = item.title,
    timestamp = Engine.Platform.getSystemTimeMS(),
    icon = item.icon
  }
  if item.originalActionInfo then
    actionRecord.level = item.originalActionInfo.level
    actionRecord.uniqueID = item.originalActionInfo.uniqueID
  end
  --log("I", "trackRecentAction", "actionRecord: " .. dumps(actionRecord))

  if not actionRecord.uniqueID or not actionRecord.level then
    log("E", "trackRecentAction", "actionRecord is missing uniqueID or level: " .. dumps(actionRecord))
    return
  end

  -- Determine which category this action belongs to
  --local category = currentLevel:match("^/root/([^/]+)/") or "other"
  local recentActionCategories = {'all'}
  --log("I","","Tracking recent action: " .. actionRecord.title .. " in category: " .. category .. " and 'all'")

  for _, category in ipairs(recentActionCategories) do
    local categoryActions = recentActions[category]
    if item.ignoreAsRecentActionForCategory == category then
      goto continue
    end

    while #categoryActions > maxRecentActionsLimits[category] do
      table.remove(categoryActions, #categoryActions)
    end

    -- Check if this action already exists in the category list
    local existsInCategory = false
    local existingIndex = nil
    for i = 1, #categoryActions do
      if categoryActions[i].uniqueID == actionRecord.uniqueID and categoryActions[i].level == actionRecord.level then
        existsInCategory = true
        existingIndex = i
        break
      end
    end

    if recentActionConfig.mode == "preserveOrder" then
      -- Mode 1: Preserve order, just update timestamp if exists
      if existsInCategory then
        -- If action exists, just update its timestamp without changing position
        categoryActions[existingIndex].timestamp = actionRecord.timestamp
      else

        -- If the list exceeds the maximum size, remove the oldest non-pinned action
        if #categoryActions >= maxRecentActionsLimits[category] then
          local oldestIndex = nil
          local oldestTimestamp = nil

          -- Find the oldest non-pinned action based on timestamp
          for i = 1, #categoryActions do
            if not categoryActions[i].pinned then
              if oldestTimestamp == nil or categoryActions[i].timestamp < oldestTimestamp then
                oldestTimestamp = categoryActions[i].timestamp
                oldestIndex = i
              end
            end
          end

          -- If we found a non-pinned action, remove it
          if oldestIndex then
            --local actionInfo = getActionInfo(categoryActions[oldestIndex].level, categoryActions[oldestIndex].uniqueID)
            --local item = categoryActions[oldestIndex]
            --log('I', 'quickaccess', 'Removed oldest non-pinned action in category ' .. category ..": " .. string.format("%s %s (%d) (%s)", item.level, item.uniqueID, item.timestamp - Engine.Platform.getSystemTimeMS(), actionInfo and actionInfo.title or "MISSING"))
            -- replace the oldest action with the new one
            table.remove(categoryActions, oldestIndex)
            table.insert(categoryActions, oldestIndex, actionRecord)
          else
            -- If all actions are pinned, we can't remove any
            -- Just keep all of them (exceeding the limit)
            log('W', 'quickaccess', 'All recent actions in category ' .. category .. ' are pinned, exceeding limit')
          end
          -- Add to the end of the category list
        else
          -- Add to the end of the category list
          table.insert(categoryActions, actionRecord)
        end

      end
    else -- "moveToTop" mode
      -- Mode 2: Move to top, remove if exists and add to top
      if existsInCategory then
        -- Remove the existing entry
        table.remove(categoryActions, existingIndex)
      end

      -- Add to the beginning of the list
      table.insert(categoryActions, 1, actionRecord)

      -- Trim the list if it exceeds the maximum size, but respect pinned items
      while #categoryActions > maxRecentActionsLimits[category] do
        -- Find the first non-pinned action from the end of the list
        local foundNonPinned = false
        for i = #categoryActions, 1, -1 do
          if not categoryActions[i].pinned then
            table.remove(categoryActions, i)
            foundNonPinned = true
            break
          end
        end

        -- If all remaining actions are pinned, stop trimming
        if not foundNonPinned then
          log('W', 'quickaccess', 'All remaining recent actions in category ' .. category .. ' are pinned, exceeding limit')
          break
        end
      end
    end
    ::continue::
  end
  --[[
    local recent = getMostRecentAvailableActions("all", maxRecentActionsLimits.all)
    dump("All")
    for i, item in ipairs(recent) do
      local actionInfo = getActionInfo(item.level, item.uniqueID)
      dump(string.format("recent[%d] = %s %s (%d) (%s)", i, item.level, item.uniqueID, item.timestamp - Engine.Platform.getSystemTimeMS(), actionInfo and actionInfo.title or "MISSING"))
    end
  ]]
end

local function itemAction(item, buttonDown, actionIndex)
  if item == nil then return end

  -- Track action if it has a uniqueID and is being selected (not deselected)
  if buttonDown then
    trackRecentAction(item, currentLevel)
  end

  -- remote item? call vehicle then
  if item.objID then
    local veh = getObjectByID(item.objID)
    if not veh then
      log('E', 'quickaccess', 'unable to select item. vehicle got missing: ' .. tostring(objID) .. ' - menu item: ' .. dumps(item))
      return
    end
    veh:queueLuaCommand('extensions.core_quickAccess.selectItem(' .. serialize(item.id) .. ', ' .. serialize(buttonDown) .. ', ' .. serialize(actionIndex) .. ')')
    return
  end

  if buttonDown then
    -- goto = dive into this new sub menu
    if type(item["goto"]) == 'string' and actionIndex == 1 then
      itemSelectCallback({'goto', item["goto"]})
      return true
    elseif type(item.onSelect) == 'function' and actionIndex == 1 then
      itemSelectCallback(item.onSelect(item))
    elseif type(item.onSecondarySelect) == 'function' and actionIndex == 2 then
      itemSelectCallback(item.onSecondarySelect(item))
    elseif type(item.onTertiarySelect) == 'function' and actionIndex == 3 then
      itemSelectCallback(item.onTertiarySelect(item))
    end
  else
    if type(item.onDeselect) == 'function' and actionIndex == 1 then
      itemSelectCallback(item.onDeselect(item))
    elseif type(item.onSecondaryDeselect) == 'function' and actionIndex == 2 then
      itemSelectCallback(item.onSecondaryDeselect(item))
    elseif type(item.onTertiaryDeselect) == 'function' and actionIndex == 3 then
      itemSelectCallback(item.onTertiaryDeselect(item))
    end
  end
  return true
end

local function onInit()
  if not initialized then
    registerDefaultMenus()
    initialized = true
  end
  hide()
end

-- callback from the ui
local function selectItem(id, buttonDown, actionIndex)
  if type(id) ~= 'number' then return end
  if currentMenuItems == nil then return end
  local m = currentMenuItems[id]
  if m == nil then
    log('E', 'quickAccess.selectItem', 'item not found: ' .. tostring(id))
  end
  actionIndex = actionIndex or 1
  itemAction(m, buttonDown, actionIndex)
end

local function contextAction(id, buttonDown, actionIndex)
  if type(id) ~= 'number' then return end
  if currentMenuItems == nil then return end
  local m = currentMenuItems[id]
  if m == nil then
    log('E', 'quickAccess.contextAction', 'item not found: ' .. tostring(id))
  end
  if m.contextAction and type(m.contextAction) == 'function' then
    m.contextAction()
  end
end

local function setEnabled(enabled, level, force)
  if enabled then
    if force or not currentUiState or currentUiState == "play" or uiVisible then
      if not uiVisible then
        callStack = {} -- reset the callstack
      end
      show(level)
    end
  else
    hide()
  end
end

local function toggle(level)
  if isEnabled() then
    setEnabled(false)
  else
    setEnabled(true, level)
  end
end

local lastTimeMoved = 0
local function getMovedRadialLastTimeMs()
  return lastTimeMoved
end

local function moved()
  lastTimeMoved = Engine.Platform.getSystemTimeMS()
end

local function renderTree(node, path)
  for key, value in pairs(node) do
    local currentPath = (path and (path .. "/" .. key) or key)
    if type(value) == "table" then
      local isLowestLevel = true
      local title = key -- Default title as key

      -- Check if the current node is a lowest-level table and extract the title if available
      for subKey, subValue in pairs(value) do
        if type(subValue) == "table" then
          isLowestLevel = false
          break
        elseif subKey == "title" then
          title = subValue
        end
      end

      if isLowestLevel then
        im.Text(title)
        im.SameLine()
        if im.Button("Add ##" .. currentPath) then
          addActionLink("/" .. path .. "/", title, favoriteSelectionIndex)
        end
      else
        if im.CollapsingHeader1(key) then
          im.Indent() -- Add indentation for child elements
          renderTree(value, currentPath)
          im.Unindent() -- Remove indentation after rendering child elements
        end
      end
    end
  end
end

local function renderImGuiTreeWindow()
  if im.Begin("Favorite Selection") then
    renderTree(menuTreeCopyForUI, nil)

    if im.Button("Cancel") then
      favoriteSelectionIndex = nil
    end
  end
  im.End()
end

local function onUpdate()
  -- logic for the menu assembling timeout
  if vehicleWaitFrames > 0 then
    vehicleWaitFrames = vehicleWaitFrames - 1
    if vehicleWaitFrames == 0 then
      log('E', 'quickaccess', 'vehicle didn\'t respond in time with menu items, showing menu anyways ...')
      _assembleMenuComplete()
    end
  end

  --[[ if favoriteSelectionIndex then
    renderImGuiTreeWindow()
  end ]]
end

local function vehicleItemSelectCallback(objID, args)
  log('D', 'quickAccess.vehicleItemSelectCallback', 'got result from id: ' .. tostring(objID) .. ' : ' .. dumps(args))
  --we don't need objID for now
  itemSelectCallback(args)
end

local function onVehicleSwitched()
  -- if switchign vehicles while the menu is show, reload it
  if uiVisible and currentLevel then
    reload()
  end
end

local function onExtensionLoaded()
  extensions.hook("onQuickAccessLoaded")
  loadRecentActions()
  loadDynamicSlotSettings()
end

local function onExtensionUnloaded()
  saveRecentActions()
  saveDynamicSlotSettings()
end

local function onExit()
  saveRecentActions()
  saveDynamicSlotSettings()
end

local function onUiChangedState(toState)
  currentUiState = toState
end

local function onSerialize()
  setEnabled(false)
  saveRecentActions()
  saveDynamicSlotSettings()
end

local function onBeforeBigMapActivated()
  setEnabled(false)
end

local function getVehicleName(veh)
  if not veh then return "No vehicle" end
  local vehKey = veh.JBeam
  local vehConfig = veh.partConfig
  local vehicleNameSTR = {veh.JBeam}
  local vehMainInfo = core_vehicles.getModel(vehKey)

  if vehMainInfo then
    table.clear(vehicleNameSTR)
    local config_key = string.match(vehConfig, "vehicles/".. vehKey .."/(.*).pc")
    local configInfo = vehMainInfo.configs[config_key] or vehMainInfo.model

    -- skip prop traffic

    -- build name
    table.insert(vehicleNameSTR, vehMainInfo.model["Brand"])
    table.insert(vehicleNameSTR, vehMainInfo.model["Name"])
    if vehMainInfo.configs[config_key] then
      table.insert(vehicleNameSTR, configInfo["Configuration"] or "")
    end

    -- set icon
    if configInfo["Type"] then
      if configInfo["Type"]== "Trailer" then vicon = "smallTrailer" end
      if configInfo["Type"]== "Prop" then vicon = "trafficCone" end
    end
  end
  local vehicleName = table.concat(vehicleNameSTR, " ")
  return vehicleName
end
M.getVehicleName = getVehicleName


-- binding app interface
local actions = {
  toggleBigMap = function()
    freeroam_bigMapMode.enterBigMap()
  end,
  toggleRadialMenuMulti = function()
    M.toggle()
  end,
  recoverVehicle = function()
    extensions.hook('trackVehReset')
    local veh = be:getPlayerVehicle(0)
    if veh then
      veh:resetBrokenFlexMesh()
      spawn.safeTeleport(veh, veh:getPosition(),quatFromDir(veh:getDirectionVector(), veh:getDirectionVectorUp()))
    end
  end,
  resetVehicle = function()
    extensions.hook('trackVehReset')
    resetGameplay(0)
  end,
  activateStarterMotor = function()
    local veh = be:getPlayerVehicle(0)
    if veh then
      core_vehicleBridge.executeAction(veh,'setIgnitionLevel', 3)
    end
  end,


}
local function tryAction(action)
  if core_input_actionFilter.isActionBlocked(action) then
    log("D", "quickAccess", "try Action: " .. action .. " - blocked")
    return
  end
  if actions[action] then
    actions[action]()
  else
    log("E", "quickAccess", "tryAction: " .. action .. " - not found")
  end
end

M.tryAction = tryAction
M.getMostRecentAvailableActions = getMostRecentAvailableActions

-- public interface
M.onInit = onInit
M.vehicleItemsCallback = vehicleItemsCallback
M.vehicleItemSelectCallback = vehicleItemSelectCallback
M.onUpdate = onUpdate
M.onVehicleSwitched = onVehicleSwitched
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.onExit = onExit
M.onUiChangedState = onUiChangedState
M.onSerialize = onSerialize
M.onBeforeBigMapActivated = onBeforeBigMapActivated
M.getUiData = getUiData
M.openDynamicSlotConfigurator = openDynamicSlotConfigurator


-- public API
M.addEntry = addEntry
M.registerMenu = function() log('E', 'quickAccess', 'registerMenu is deprecated. Please use quickAccess.addEntry: ' .. debug.traceback()) end

-- API towards the UI
M.selectItem = selectItem
M.contextAction = contextAction
M.back = back
M.isEnabled = isEnabled
M.moved = moved
M.getMovedRadialLastTimeMs = getMovedRadialLastTimeMs
M.reload = reload
M.getDynamicSlotConfigurationData = getDynamicSlotConfigurationData
M.toNiceName = toNiceName

-- input map
M.setEnabled = setEnabled
M.toggle = toggle

return M

-- vehicle triggers test code

