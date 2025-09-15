local M = {}

local Constants = extensions.ui_router_constants

M.isUiTypeValidByFilterAll = function(uiTypesCompleted, uiTypes)
  local allComplete = true

  for _, uiType in ipairs(uiTypes) do
    if not uiTypesCompleted[uiType] then
      allComplete = false
      break
    end
  end

  return allComplete
end

M.isUiTypeValidByFilterAny = function(uiTypesCompleted, uiTypes)
  for _, uiType in ipairs(uiTypes) do
    if uiTypesCompleted[uiType] then
      return true
    end
  end

  return false
end

M.isUiTypeValidByFilterExcept = function(uiTypesCompleted, uiTypes)
  for _, uiType in ipairs(uiTypes) do
    if not uiTypesCompleted[uiType] then
      return true
    end
  end

  return false
end

M.isUiTypeValidByFilterOnly = function(uiTypesCompleted, uiTypes)
  dump("isUiTypeValidByFilterOnly", uiTypesCompleted, uiTypes)
  local onlyComplete = true

  for uiType, completed in pairs(uiTypesCompleted) do
    if completed and not tableContains(uiTypes, uiType) then
      onlyComplete = false
      break
    end
  end

  return onlyComplete
end

M.isRouteChangeValid = function(uiTypesCompleted, uiTypes, uiTypesFilter)
  dump("isRouteChangeValid", uiTypesCompleted, uiTypes, uiTypesFilter)
  if uiTypesFilter == Constants.UiTypesFilter.all then
    return M.isUiTypeValidByFilterAll(uiTypesCompleted, uiTypes)
  elseif uiTypesFilter == Constants.UiTypesFilter.any then
    return M.isUiTypeValidByFilterAny(uiTypesCompleted, uiTypes)
  elseif uiTypesFilter == Constants.UiTypesFilter.except then
    return M.isUiTypeValidByFilterExcept(uiTypesCompleted, uiTypes)
  elseif uiTypesFilter == Constants.UiTypesFilter.only then
    return M.isUiTypeValidByFilterOnly(uiTypesCompleted, uiTypes)
  end
end

M.isRelativeRoute = function(route)
  if type(route) ~= "string" then
    return false
  end

  -- Check for Angular style (^.)
  if string.sub(route, 1, 1) == "^" then
    return true
  end

  -- Check for Vue style (./ or ../)
  if string.sub(route, 1, 1) == "." then
    return true
  end

  return false
end

M.resolveRelativeRoute = function(route, currentState)
  if not M.isRelativeRoute(route) then
    return route -- Already absolute
  end

  -- Get current state parts
  local currentParts = {}
  for part in string.gmatch(currentState, "[^%.]+") do
    table.insert(currentParts, part)
  end

  -- Detect which style of relative path and process accordingly
  local upCount = 0
  local remaining = route

  -- Handle Angular style (^.)
  if string.sub(route, 1, 1) == "^" then
    while string.sub(remaining, 1, 1) == "^" do
      upCount = upCount + 1
      remaining = string.sub(remaining, 2) -- Remove the ^
    end

    if string.sub(remaining, 1, 1) == "." then
      remaining = string.sub(remaining, 2) -- Remove leading dot if present
    end

  -- Handle Vue style (./ or ../)
  elseif string.sub(route, 1, 1) == "." then
    -- Handle current level (./)
    if string.sub(route, 1, 2) == "./" then
      remaining = string.sub(route, 3) -- Skip ./ and keep at current level
      upCount = 0
    else
      -- Count ../ sequences for parent navigation
      remaining = route
      while string.sub(remaining, 1, 3) == "../" do
        upCount = upCount + 1
        remaining = string.sub(remaining, 4) -- Remove ../
      end

      -- Also handle .. without trailing slash
      while string.sub(remaining, 1, 2) == ".." and (string.len(remaining) == 2 or string.sub(remaining, 3, 3) == "." or string.sub(remaining, 3, 3) == "/") do
        upCount = upCount + 1
        remaining = string.sub(remaining, 3) -- Remove ..

        -- Remove separator if present
        if string.sub(remaining, 1, 1) == "/" then
          remaining = string.sub(remaining, 2)
        end
      end
    end

    -- Convert slashes to dots for internal consistency
    remaining = string.gsub(remaining, "/", ".")
  end

  -- Remove parts based on upCount
  for i = 1, upCount do
    if #currentParts > 0 then
      table.remove(currentParts)
    end
  end

  -- Build the full path
  local parentPath = table.concat(currentParts, ".")

  -- Handle edge cases with empty strings
  if remaining == "" then
    return parentPath
  elseif parentPath == "" then
    return remaining
  else
    return parentPath .. "." .. remaining
  end
end

return M

