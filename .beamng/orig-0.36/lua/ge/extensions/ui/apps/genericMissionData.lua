-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.onInit = function() setExtensionUnloadMode(M, "manual") end

local debug = false

-- Mission data storage
local missionData = {
  elements = {},  -- Stores all elements with their categories
  displayOrder = {}  -- Stores the order of elements to display
}

-- Function to clear all mission data
local function clearData()
  missionData = {
    elements = {},
    displayOrder = {}
  }
  guihooks.trigger('SetGenericMissionDataResetAll')
end

-- Function to get all current data
local function sendAllData()
  -- Send all elements in order
  for i, item in ipairs(missionData.displayOrder) do
    guihooks.trigger('SetGenericMissionData', {
      element = item.element,
      index = i
    })
  end
end

-- Function to update mission data
local function setData(args)
  if not args.category then args.category = 'default' end
  if not args.order then args.order = -(1 + tableSize(missionData.elements)) end

  local matchedCategories = {}

  -- Try to match the category as a lua pattern
  for category, _ in pairs(missionData.elements) do
    if string.match(category, args.category) then
      table.insert(matchedCategories, category)
    end
  end

  -- If no category was found, use the category name directly
  if #matchedCategories == 0 then
    table.insert(matchedCategories, args.category)
  end

  -- Update all matched categories
  for _, category in ipairs(matchedCategories) do
    if args.clear then
      missionData.elements[category] = nil
    elseif args.msg ~= "" then
      missionData.elements[category] = {
        title = args.title,
        txt = args.txt,
        order = args.order,
        style = args.style,
        category = args.category,
        minutes = args.minutes,
        seconds = args.seconds,
        milliseconds = args.milliseconds
      }
    end
  end

  -- Update display order
  local newOrder = {}
  for category, element in pairs(missionData.elements) do
    table.insert(newOrder, {
      category = category,
      element = element
    })
  end
  table.sort(newOrder, function(a, b)
    return a.element.order < b.element.order
  end)

  -- Check if order changed
  local orderChanged = #newOrder ~= #missionData.displayOrder
  if not orderChanged then
    for i, item in ipairs(newOrder) do
      if missionData.displayOrder[i] ~= item.category then
        orderChanged = true
        break
      end
    end
  end

  missionData.displayOrder = newOrder

  -- Send updates to UI
  if orderChanged then
    -- If order changed, send all elements
    for i, item in ipairs(missionData.displayOrder) do
      guihooks.trigger('SetGenericMissionData', {
        element = item.element,
        index = i
      })
    end
  else
    -- If order didn't change, only send updated elements
    for _, category in ipairs(matchedCategories) do
      local element = missionData.elements[category]
      if element then
        -- Find the index of this element
        for i, item in ipairs(missionData.displayOrder) do
          if item.category == category then
            guihooks.trigger('SetGenericMissionData', {
              element = element,
              index = i
            })
            break
          end
        end
      end
    end
  end
end

-- Debug UI
local im
if debug then
  im = ui_imgui

  local function onUpdate()
    if not im then return end

    im.Begin("Generic Mission Data Debug")

    -- Show current data
    im.Text("Current Mission Data:")
    for i, item in ipairs(missionData.displayOrder) do
      local element = item.element
      im.BulletText(string.format("[%d] %s: %s (order: %d)", i, item.category, element.txt, element.order))
    end


    im.SameLine()
    if im.Button("Clear All") then
      clearData()
    end

    im.End()
  end

  M.onUpdate = onUpdate
end

-- Public interface
M.setData = setData
M.clearData = clearData
M.sendAllData = sendAllData


return M
