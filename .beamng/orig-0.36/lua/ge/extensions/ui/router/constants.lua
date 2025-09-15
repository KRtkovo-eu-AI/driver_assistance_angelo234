local M = {}

M.UiTypesFilter = {
  only = "only", -- only the ui types specified in the route should be loaded
  except = "except", -- all ui types except the ones specified in the route should be loaded
  all = "all", -- all ui types must be loaded
  any = "any" -- at least one of the ui types specified in the route must be loaded
}

M.ModuleSettingsDefaults = {
  uiTypes = {"angular", "vue"},
  uiTypesFilter = M.UiTypesFilter.any,
  meta = {
    infoBar = M.InfoBarDefaults
  }
}

M.InfoBarDefaults = {
  visible = true,
  showSysInfo = true
}

M.InfoBarDefaultsHidden = {
  visible = false,
  showSysInfo = false
}

M.UiAppsDefaults = {
  shown = false
}

return M
