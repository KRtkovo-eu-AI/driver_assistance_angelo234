local M = {}
local Constants = extensions.ui_router_constants

-- other routes that aren't modules, you can define them here
-- if you want to use nested routes, you should add it to a module instead
M.Routes = {
  ["credits"] = {},
  ["play"] = {
    uiTypes = {"angular"},
    uiTypesFilter = Constants.UiTypesFilter.only,
    uiApps = {
      shown = true
    }
  }
}

return M
