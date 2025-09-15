local M = {}
local Constants = extensions.ui_router_constants

M.dependencies = {"ui_router_routes_default", "ui_router_routes_menu", "ui_router_routes_career"}

-- put your route modules here
-- do not add the default route module here, it will be loaded automatically
M.RouteModules = {"ui_router_routes_menu", "ui_router_routes_career"}

local function mergeConfig(oldConfig, newConfig)
  for _, uiType in ipairs(oldConfig.uiTypes) do
    local found = false

    for _, existingUiType in ipairs(newConfig.uiTypes) do
      if existingUiType == uiType then
        found = true
        break
      end
    end

    if not found then
      table.insert(oldConfig.uiTypes, uiType)
    end
  end
end

local function insertRouteToModule(routeTokens, config, moduleRef)
  local actualRoute = routeTokens[#routeTokens]
  local routes = moduleRef.Routes

  table.remove(routeTokens, #routeTokens)

  for _, routeToken in ipairs(routeTokens) do
    local subRoute = routes[routeToken]

    if not subRoute then
      subRoute = {
        children = {}
      }
      routes[routeToken] = subRoute
    end

    routes = subRoute.children
  end

  if routes[actualRoute] then
    mergeConfig(routes[actualRoute], config)
  else
    routes[actualRoute] = config
  end
end

local function getRouteInternal(routeTokens, routeModuleRef)
  local routes = routeModuleRef.Routes
  local settings = deepcopy(routeModuleRef.ModuleSettings)
  local foundRoute = nil

  -- if only the module name is provided, return the default/root route of the module
  if #routeTokens == 0 then
    foundRoute = routes[""]
  else
    -- local actualRoute = routeTokens[#routeTokens]
    -- table.remove(routeTokens, #routeTokens)

    -- traverse the route tokens to find the route
    -- merge the settings from top to the actual route's settings if available
    for _, routeToken in ipairs(routeTokens) do
      foundRoute = routes[routeToken]
      routes = foundRoute.children

      if not foundRoute then
        break
      end

      -- if not foundRoute then
      --   routes = nil
      --   break
      -- end

      tableMerge(settings, foundRoute)

      -- routes = route.children
    end

    -- if routes then
    --   foundRoute = routes[actualRoute]
    -- end
  end

  if not foundRoute then
    return nil
  end

  foundRoute = deepcopy(foundRoute)
  tableMerge(foundRoute, settings)

  return foundRoute
end

M.RouteModuleRefs = {}

M.getRoute = function(route)
  local routeTokens = string.split(route, "[^%.]+")
  local routeModuleRef = M.RouteModuleRefs[routeTokens[1]]

  dump("getRoute", routeTokens, routeModuleRef)

  if routeModuleRef then
    table.remove(routeTokens, 1)
    return getRouteInternal(routeTokens, routeModuleRef)
  elseif #routeTokens == 1 and M.RouteModuleRefs["default"] then
    routeModuleRef = M.RouteModuleRefs["default"]
    return routeModuleRef.Routes[routeTokens[1]]
  end
end

-- start initialization
local function loadRouteModule(fullModuleName)
  dump("loadRouteModule", fullModuleName)

  -- if extensions.isExtensionLoaded(fullModuleName) then
  --   log("W", "[routeManager] module " .. fullModuleName .. " already loaded. ignoring...")
  --   return
  -- end

  -- -- manually load the extension
  -- extensions.load(fullModuleName)

  local routeModule = extensions[fullModuleName]
  local moduleName = routeModule.ModuleName

  -- check if ModuleName is configured. this will be used as key for the module references
  if moduleName then
    moduleName = trim(moduleName)

    if moduleName == "" then
      moduleName = nil
    end
  end

  -- if moduleName is not configured, use the file name as the module name
  if not moduleName then
    local moduleNameParts = string.split(fullModuleName, "[^_]+")
    moduleName = moduleNameParts[#moduleNameParts]
  end

  -- configure module settings. merge default settings with configured settings
  local settings = routeModule.ModuleSettings or {}
  local completeSettings = deepcopy(Constants.ModuleSettingsDefaults)
  tableMerge(completeSettings, settings)

  routeModule.ModuleSettings = completeSettings

  -- configure module runtime properties
  routeModule.IsModule = true
  M.RouteModuleRefs[moduleName] = routeModule
end

local function loadDefaultRouteModule()
  log("D", "", "loadDefaultRouteModule")
  dump("loadDefaultRouteModule")
  -- if not extensions.isExtensionLoaded("ui_router_routes_default") then
  --   extensions.load("ui_router_routes_default")
  -- end

  -- if not extensions["ui_router_routes_default"] then
  --   log("D", "", "[routeManager] ui_router_routes_default not found")
  --   dump("[routeManager] ui_router_routes_default not found")
  -- end

  local defaultModule = extensions["ui_router_routes_default"]
  if not defaultModule or not defaultModule.Routes then
    log("D", "", "[routeManager] ui_router_routes_default not found or no routes defined")
    dump("[routeManager] ui_router_routes_default not found or no routes defined")
    return
  end

  M.RouteModuleRefs["default"] = defaultModule
end

local function loadRouteModules()
  dump("loadRouteModules", M.RouteModules)
  for _, routeModule in ipairs(M.RouteModules) do
    loadRouteModule(routeModule)
    dump("routeModuleRefs", M.RouteModuleRefs)
  end
end
-- end initialization

M.onExtensionLoaded = function()
  dump("[routeManager] onExtensionLoaded")
  loadDefaultRouteModule()
  loadRouteModules()
end

return M
