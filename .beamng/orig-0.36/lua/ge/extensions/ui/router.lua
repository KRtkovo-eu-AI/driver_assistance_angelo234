local M = {}

local MODULE_NAME = "ui_router"

M.dependencies = {"ui_router_config", "ui_router_routeManager"}

local Constants = extensions.ui_router_constants
local Utils = extensions.ui_router_utils
local Config = extensions.ui_router_config
local RouteManager = extensions.ui_router_routeManager

local function notifyListeners(hookName, ...)
  local eventName = MODULE_NAME .. "_" .. hookName
  extensions.hook(eventName, ...)
  guihooks.trigger(eventName, ...)
end

local RouterState = {
  -- waiting for ui to load
  waitingForUi = 0,
  -- all UIs are loaded and the router is waiting for a command
  idle = 1,
  -- a route transition is in progress
  transitioning = 2,
  -- the route has changed and the UI is active/displayed
  active = 3,
  -- an error occurred during route transition
  error = 4
}

local RouterChangeAction = {
  push = 1,
  replace = 2,
  back = 3,
  forward = 4
}

-- start ui state
local uiState = {}
for _, uiType in ipairs(Config.RequiredUiTypes) do
  uiState[uiType] = false
end

local function hasLoadedUi()
  for _, loaded in pairs(uiState) do
    if not loaded then
      return false
    end
  end

  return true
end

-- method to call from UI to confirm that it has completed bootstrapping
-- once all required UIs are loaded, the route will navigate to the main route
M.loadComplete = function(uiType)
  log("D", "", "loadComplete: " .. uiType)
  dump("loadComplete", uiType)

  if uiState[uiType] == nil then
    log("E", "", "uiLoaded: uiType not found in uiState: " .. uiType)
    return
  end

  uiState[uiType] = true
  notifyListeners("uiLoaded", uiType)

  -- navigate to main route if all ui libs are loaded
  if hasLoadedUi() then
    M.state.state = RouterState.idle
    notifyListeners("stateChanged", M.state)

    M.push(extensions.ui_router_config.MainRoute)
  end
end
-- end ui state

-- start transition manager
local TransitionManager = {
  transitionTime = 0,
  transitionTimeout = extensions.ui_router_config.TransitionTimeout
}

TransitionManager.startTransition = function()
  M.state.state = RouterState.transitioning
  M.state.transitionTime = 0
end

TransitionManager.updateTransition = function(dtReal, dtSim, dtRaw)
  if M.state.state ~= RouterState.transitioning then
    return
  end

  M.state.transitionTime = M.state.transitionTime + dtReal
  TransitionManager.validateTransition()
end

TransitionManager.endTransition = function()
  M.state.state = RouterState.active
  M.state.transitionTime = 0
end

TransitionManager.timeoutTransition = function()
  M.state.state = RouterState.error
  M.state.transitionTime = 0
end

TransitionManager.validateTransition = function()
  if M.state.state == RouterState.transitioning and M.state.transitionTime > TransitionManager.transitionTimeout then
    TransitionManager.timeoutTransition()
  end
end

TransitionManager.isTransitioning = function()
  return M.state.state == RouterState.transitioning
end
-- end transition manager

-- start router state
local RouterStateHooks = {
  -- emitted when lua wants to send request to UI to change route
  routeChange = "routeChange",
  -- emitted when lua wants to send request to UI to change route but it fails
  routeChangeError = "routeChangeError",
  -- emitted when UI has successfully changed route
  routeChangeSuccess = "routeChangeSuccess"
}

local RouterManager = {}

RouterManager.error = function(route, error)
  M.state.state = RouterState.error
  M.state.transitionTime = nil
  notifyListeners(RouterStateHooks.routeChangeError, {
    route = route,
    error = error
  })
end

RouterManager.routeChange = function(toRoute, fromRoute, callbackFn)
  -- toRoute - name, params, config
  log("D", "", "changing route to " .. toRoute.name .. " from " .. (fromRoute and fromRoute.name or "nil"))
  dump("changing route", toRoute, fromRoute, callbackFn)

  M.Current = {
    toRoute = toRoute,
    fromRoute = fromRoute,
    callbackFn = callbackFn,
    uiTypesCompleted = {}
  }

  for _, uiType in ipairs(toRoute.uiTypes) do
    M.Current.uiTypesCompleted[uiType] = false
  end

  TransitionManager.startTransition()
  notifyListeners(RouterStateHooks.routeChange, {
    toRoute = toRoute,
    fromRoute = fromRoute
  })
end

M.Current = nil
M.History = {}
M.ForwardStack = {}

M.routeChangeComplete = function(uiType)
  dump("routeChangeComplete", uiType)

  if not TransitionManager.isTransitioning() then
    log("W", "", "routeChangeComplete: transition is not in progress")
    return
  end

  M.Current.uiTypesCompleted[uiType] = true

  dump("uiTypesCompleted", M.Current.uiTypesCompleted)

  local isValid = Utils.isRouteChangeValid(M.Current.uiTypesCompleted, M.Current.toRoute.uiTypes,
      M.Current.toRoute.uiTypesFilter)

  if not isValid then
    log("D", "", "routeChangeComplete: route change is not valid. waiting for ui types to complete.")
    dump("routeChangeComplete: route change is not valid. waiting for ui types to complete.")
    return
  end

  TransitionManager.endTransition()

  if M.Current.callbackFn then
    M.Current.callbackFn()
  end

  notifyListeners(RouterStateHooks.routeChangeSuccess, M.Current.toRoute)
end

M.validateRouteState = function()
  if M.state.state == RouterState.transitioning and M.state.transitionTime > Config.TransitionTimeout then
    RouterManager.error(M.state.current, "route transition timeout")
  end
end
-- end router state

-- start navigation methods
M.push = function(route, params)
  dump("push", route, params)
  -- check if state is transitioning if so ignore or cancel the transition
  -- for now just ignore the request
  if TransitionManager.isTransitioning() then
    dump("push", "route transition is already in progress, ignoring request")
    return
  end

  if M.Current then
    route = Utils.resolveRelativeRoute(route, M.Current.toRoute.name)
  end

  local routeConfig = RouteManager.getRoute(route)

  -- check if route is valid
  if not routeConfig then
    log("E", "", "navigate: route not found in config: " .. route)
    RouterManager.error(route, "route not found in config")
    return
  end

  local fromRoute = #M.History > 0 and M.History[#M.History] or nil
  local callback = function()
    -- disable duplicate history entries
    if not fromRoute or fromRoute.name ~= M.Current.toRoute.name then
      M.Current.callbackFn = nil
      table.insert(M.History, M.Current.toRoute)
    end

    M.ForwardStack = {}
  end

  local toRoute = {
    params = params,
    name = route
  }
  tableMerge(toRoute, routeConfig)

  RouterManager.routeChange(toRoute, fromRoute, callback)
end

M.replace = function(route, params)
  dump("replace", route, params)
  -- check if state is transitioning if so ignore or cancel the transition
  -- for now just ignore the request
  if TransitionManager.isTransitioning() then
    dump("replace", "route transition is already in progress, ignoring request")
    return
  end

  local routeConfig = RouteManager.Routes[route]
  if not routeConfig then
    log("E", "", "navigate: route not found in config: " .. route)
    RouterManager.error(route, "route not found in config")
    return
  end

  routeConfig.meta = getMetaProperties(route)

  local fromRoute = #M.state.history > 0 and M.state.history[#M.state.history] or nil
  local fromRouteName = fromRoute and fromRoute.name
  local eventData = getEventData(fromRouteName, fromRoute, route, routeConfig)

  local callback = function()
    M.sstate.current.callbackFn = nil
    M.state.history[#M.state.history] = M.state.current
    M.state.forwardStack = {}
  end

  RouterManager.routeChange(route, params, routeConfig, eventData, callback)
end

M.back = function()
  dump("back")
  -- ignore for now. this can be updated later to cancel the current transition and go back to previous route
  if TransitionManager.isTransitioning() then
    dump("back", "route transition is already in progress, ignoring request")
    return
  end

  if #M.state.history <= 1 then
    dump("back", "no history to go back to, ignoring request")
    return
  end

  local fromRoute = M.state.history[#M.state.history]
  local toRoute = M.state.history[#M.state.history - 1]
  local eventData = getEventData(fromRoute.name, fromRoute, toRoute.name, toRoute)

  local callback = function()
    table.remove(M.state.history, #M.state.history)
    table.insert(M.state.forwardStack, fromRoute)
  end

  RouterManager.routeChange(toRoute.name, toRoute.params, toRoute, eventData, callback)
end

M.forward = function()
  dump("forward")
  -- ignore for now. this can be updated later to cancel the current transition and go back to previous route
  if TransitionManager.isTransitioning() then
    dump("forward", "route transition is already in progress, ignoring request")
    return
  end

  if #M.state.forwardStack == 0 then
    dump("forward", "no forward stack to go forward to, ignoring request")
    return
  end

  local fromRoute = M.state.history[#M.state.history]
  local toRoute = M.state.forwardStack[1]
  local eventData = getEventData(fromRoute.name, fromRoute, toRoute.name, toRoute)

  local callback = function()
    table.remove(M.state.forwardStack, 1)
    table.insert(M.state.history, toRoute)
  end

  RouterManager.routeChange(toRoute.name, toRoute.params, toRoute, eventData, callback)
end
-- end navigation methods

M.getRoute = function(route)
  return RouteManager.Routes and RouteManager.Routes[route]
end

M.getRoutes = function()
  return RouteManager.Routes
end

-- start route/config methods
M.addOrUpdateRoute = function(route, config, options)
  -- dump("addOrUpdateRoute", route, config, options)
  local merge = options and options.merge or false
  local routeConfig = RouteManager.Routes[route]

  if not routeConfig or not merge then
    RouteManager.Routes[route] = config
  elseif merge then
    -- add config.uiTypes to routeConfig.uiTypes if not existing
    for _, uiType in ipairs(config.uiTypes) do
      local found = false

      for _, existingUiType in ipairs(routeConfig.uiTypes) do
        if existingUiType == uiType then
          found = true
          break
        end
      end

      if not found then
        table.insert(routeConfig.uiTypes, uiType)
      end
    end
  end
end
-- end route/config methods

-- start extension lifecycle hooks
M.state = {
  current = nil,
  history = {},
  forwardStack = {},
  state = nil,
  transitionTime = nil
}

M.onExtensionLoaded = function()
  log("D", "", "onExtensionLoaded")
  dump("[ui_router] onExtensionLoaded")
  -- disable automatic unload
  setExtensionUnloadMode(M, "manual")
end

M.onGuiUpdate = function(dtReal, dtSim, dtRaw)
  TransitionManager.updateTransition(dtReal, dtSim, dtRaw)
end

M.onSerialize = function()
  log("D", "", "onSerialize")
end

M.onReload = function()
  log("D", "", "onReload")
  dump("[ui_router] onReload")
end

M.onRefresh = function()
  log("D", "", "onRefresh")
  dump("[ui_router] onRefresh")
end

-- end extension lifecycle hooks

return M
