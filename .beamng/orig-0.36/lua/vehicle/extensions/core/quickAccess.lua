-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- this is a proxy for the GE module quickaccess

local M = {}

local constants = {
  mphToMs = 1 / 2.237,
  kmhToMs = 1 / 3.6
}
local menuTree = {}
local menuTreeCopy

local actionCallbackArgs
local actionCallbackCountdown

-- same as GE/core/quickAccess/addEntry - sync with it!
local function addEntry(_args)
  local args = deepcopy(_args) -- do not modify the outside table by any chance
  if type(args.generator) ~= "function" and (type(args.title) ~= "string" or (type(args.onSelect) ~= "function" and type(args["goto"]) ~= "string")) then
    -- TODO: add proper warning/error
    log("W", "quickaccess", "Menu item needs at least a title and an onSelect function callback: " .. dumps(args))
  --return false
  end
  -- defaults
  if args.level == nil then
    args.level = "/root/playerVehicle/general/"
  end

  if not (string.startswith(args.level, "/root/playerVehicle/") or string.startswith(args.level, "/root/sandbox/")) then
    args.level = "/root/playerVehicle" .. args.level
  end

  if type(args.level) ~= "string" then
    log("E", "quickaccess", "Menu item level incorrect, needs to be a string: " .. dumps(args))
    return false
  end
  if string.sub(args.level, string.len(args.level)) ~= "/" then
    args.level = args.level .. "/"
  end -- make sure there is always a trailing slash in the level

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
  args.origin = debug.tracesimple()
  return true
end

local function generateCompleteTree(level)
  menuTreeCopy = deepcopy(menuTree)

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
      else
        item.objID = obj:getId()
        item.id = path .. item.title .. i
      end
    end
  end
end

-- open the menu in a specific level
local function requestItems(level)
  generateCompleteTree(level)

  obj:queueGameEngineLua("core_quickAccess.vehicleItemsCallback(" .. obj:getId() .. "," .. serialize(level) .. "," .. serialize(menuTreeCopy) .. ")")
  return true
end

local function itemSelectCallback(args)
  if args == nil then
    -- no result = hide by default
    args = {"hide"}
  elseif args[1] == "reload" then
    actionCallbackArgs = args
    actionCallbackCountdown = 2
    return
  end
  obj:queueGameEngineLua("core_quickAccess.vehicleItemSelectCallback(" .. obj:getId() .. "," .. serialize(args) .. ")")
end

local function selectItem(id, buttonDown, actionIndex)
  local item
  for path, items in pairs(menuTreeCopy) do
    for i = 1, #items do
      local _item = items[i]
      if _item.id == id then
        item = _item
        goto itemFound
      end
    end
  end
  ::itemFound::
  if item == nil then
    log("E", "core_quickAccess.selectItem", "item not found: " .. tostring(id))
    return itemSelectCallback({"error", "item_not_found"})
  end

  -- actual action implementation
  if buttonDown then
    -- goto = dive into this new sub menu
    if type(item["goto"]) == "string" and actionIndex == 1 then
      return itemSelectCallback({"goto", item["goto"]})
    elseif type(item.onSelect) == "function" and actionIndex == 1 then
      itemSelectCallback(item.onSelect(item))
    elseif type(item.onSecondarySelect) == "function" and actionIndex == 2 then
      itemSelectCallback(item.onSecondarySelect(item))
    elseif type(item.onTertiarySelect) == "function" and actionIndex == 3 then
      itemSelectCallback(item.onTertiarySelect(item))
    end
  else
    if type(item.onDeselect) == "function" and actionIndex == 1 then
      itemSelectCallback(item.onDeselect(item))
    elseif type(item.onSecondaryDeselect) == "function" and actionIndex == 2 then
      itemSelectCallback(item.onSecondaryDeselect(item))
    elseif type(item.onTertiaryDeselect) == "function" and actionIndex == 3 then
      itemSelectCallback(item.onTertiaryDeselect(item))
    end
  end
  return true
end

local function updateGFX(dt)
  if actionCallbackCountdown then
    actionCallbackCountdown = actionCallbackCountdown - 1
    if actionCallbackCountdown <= 0 then
      obj:queueGameEngineLua("core_quickAccess.vehicleItemSelectCallback(" .. obj:getId() .. "," .. serialize(actionCallbackArgs) .. ")")
      actionCallbackCountdown = nil
    end
  end
end

local function onInit()
  -- check if there are powered wheels
  local hasPoweredWheels = tableSize(powertrain.getPoweredWheelNames()) > 0

  -- only add cruise control if there are powered wheels
  -- todo: display the cc target speed in the description
  if hasPoweredWheels then
    -- cruise control
    addEntry(
      {
        level = "/root/playerVehicle/cruise_control/",
        generator = function(entries)
          local cruiseEnabled = false
          local cruiseSetSpeed = 0
          if extensions.isExtensionLoaded("cruiseControl") then
            local config = extensions.cruiseControl.getConfiguration()
            cruiseEnabled = config.isEnabled
            cruiseSetSpeed = config.targetSpeed
          end

          local targetSpeedDesc = {txt = "ui.radialmenu2.cruiseControl.targetSpeed", context = {speed = cruiseSetSpeed}}
          if cruiseSetSpeed == 0 or not cruiseEnabled then
            targetSpeedDesc = {txt = "ui.radialmenu2.cruiseControl.notSet"}
          end

          local e

          -- Toggle cruise on/off
          e = {
            title = "ui.radialmenu2.cruiseControl.toggle",
            priority = 51,
            desc = targetSpeedDesc,
            icon = "cruiseEnable",
            startSlot = 4.5,
            endSlot = 5.5,
            originalActionInfo = {level = "/root/playerVehicle/", uniqueID = "cruiseControl"},
            uniqueID = "cruiseControl_toggle",
            onSelect = function()
              extensions.cruiseControl.setEnabled(not cruiseEnabled)
              return {"reload"}
            end
          }
          if cruiseEnabled then
            e.color = "#33ff33"
            e.icon = "cruiseEnable"
          end
          table.insert(entries, e)

          -- Resume/+
          e = {
            title = "ui.radialmenu2.cruiseControl.plusResume",
            priority = 52,
            desc = targetSpeedDesc,
            icon = "plusSet",
            startSlot = 2.5,
            endSlot = 3.5,
            originalActionInfo = {level = "/root/playerVehicle/", uniqueID = "cruiseControl"},
            uniqueID = "cruiseControl_plusResume",
            onSelect = function()
              if cruiseEnabled then
                local units = settings.getValue("uiUnitLength")
                local delta = units == "imperial" and constants.mphToMs or constants.kmhToMs

                extensions.cruiseControl.changeSpeed(delta)
              else
                extensions.cruiseControl.setSpeed(cruiseSetSpeed)
              end
              return {"reload"}
            end
          }
          table.insert(entries, e)

          -- Set/-
          e = {
            title = "ui.radialmenu2.cruiseControl.minusSet",
            priority = 53,
            desc = targetSpeedDesc,
            icon = "minusRes",
            startSlot = 6.5,
            endSlot = 7.5,
            originalActionInfo = {level = "/root/playerVehicle/", uniqueID = "cruiseControl"},
            uniqueID = "cruiseControl_minusSet",
            onSelect = function()
              if cruiseEnabled then
                local units = settings.getValue("uiUnitLength")
                local delta = units == "imperial" and constants.mphToMs or constants.kmhToMs

                extensions.cruiseControl.changeSpeed(-delta)
              else
                extensions.cruiseControl.holdCurrentSpeed()
              end
              return {"reload"}
            end
          }
          table.insert(entries, e)

          -- Cancel
          e = {
            title = "ui.radialmenu2.cruiseControl.cancel",
            priority = 54,
            desc = targetSpeedDesc,
            icon = "cruiseDisable",
            startSlot = 0.5,
            endSlot = 1.5,
            originalActionInfo = {level = "/root/playerVehicle/", uniqueID = "cruiseControl"},
            uniqueID = "cruiseControl_cancel",
            onSelect = function()
              extensions.cruiseControl.setEnabled(false)
              return {"reload"}
            end
          }
          table.insert(entries, e)
        end
      }
    )
  end
end

local function onExtensionLoaded()
  -- Vehicle features categories
  local vehicleFeatureCategories = {
    { title = 'Helper Systems',   ["goto"] = '/root/playerVehicle/helperSystems/',   icon = 'shieldHandPlus',    uniqueID = 'helperSystems', categoryOrder = -20 },
    { title = 'Lights',           ["goto"] = '/root/playerVehicle/lights/',          icon = 'highBeam',          uniqueID = 'lights', categoryOrder = -10 },
    { title = 'Quick Actions',    ["goto"] = '/root/playerVehicle/general/',         icon = 'charge',  categoryOrder = 0  },
    { title = 'Vehicle Features', ["goto"] = '/root/playerVehicle/vehicleFeatures/', icon = 'vehicleFeatures03', uniqueID = 'vehicleFeatures', categoryOrder = 10 },
    { title = 'Cruise Control',   ["goto"] = '/root/playerVehicle/cruise_control/',  icon = 'cruiseEnable',      uniqueID = 'cruiseControl', categoryOrder = 20 }
  }

  for _, category in ipairs(vehicleFeatureCategories) do
    local res = tableMerge(category, {level = '/root/playerVehicle/'})
    addEntry(res)
  end
end

-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.onInit = onInit
M.updateGFX = updateGFX

-- interface for GE lua
M.requestItems = requestItems
M.selectItem = selectItem -- no extension message hook, thus no 'on'

-- interface for the vehicle lua
M.addEntry = addEntry
M.registerMenu = function()
  log("E", "quickAccess", "registerMenu is deprecated. Please use core_quickAccess.addEntry: " .. debug.traceback())
end

return M
