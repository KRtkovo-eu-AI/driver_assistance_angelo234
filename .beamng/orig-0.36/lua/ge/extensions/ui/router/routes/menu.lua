local M = {}
local Constants = extensions.ui_router_constants

-- ModuleName(optional) by default is the name of the file unless specified otherwise
-- Uncomment and set to a different name if you want to specify a different name for the route module
-- local ModuleName = nil

-- ModuleSettings(optional) by default is empty
-- This will be the default settings to use for all the child routes unless
-- specified otherwise in the child route settings

local ModuleSettings = {
  uiTypes = {"angular"},
  meta = {
    infoBar = Constants.InfoBarDefaults
  },
}

local Routes = {
  ["campaigns"] = {},
  ["levels"] = {
    uiTypes = {"angular"},
    uiTypesFilter = Constants.UiTypesFilter.only,
    children = {
      -- ["details"] = {}
    }
  },
  -- TODO: this needs to be setup in angular to have a child-parent relationship between levels and levelDetails
  -- currently, they are setup as siblings
  ["levelDetails"] = {},
  ["mainmenu"] = {
    uiTypes = {"vue"},
    uiTypesFilter = Constants.UiTypesFilter.only
  },
  ["mods"] = {
    uiTypesFilter = Constants.UiTypesFilter.only,
    children = {
      ["local"] = {},
      ["repository"] = {},
      ["automation"] = {}
    }
  },
  ["options"] = {
    uiTypesFilter = Constants.UiTypesFilter.only,
    children = {
      ["audio"] = {},
      ["camera"] = {},
      ["controls"] = {
        children = {
          ["bindings"] = {},
          ["ffb"] = {},
          ["hardware"] = {}
        }
      },
      ["display"] = {},
      ["gameplay"] = {},
      ["graphics"] = {},
      ["help"] = {},
      ["language"] = {},
      ["licenses"] = {},
      ["other"] = {},
      ["performance"] = {},
      ["stats"] = {},
      ["userInterface"] = {}
    }
  },
  ["quickraceOverview"] = {
    uiTypesFilter = Constants.UiTypesFilter.only
  },
  ["quickraceLevelselect"] = {
    uiTypesFilter = Constants.UiTypesFilter.only
  },
  ["quickraceTrackselect"] = {
    uiTypesFilter = Constants.UiTypesFilter.only
  },
  ["scenarios"] = {},
  ["vehicleconfig"] = {
    children = {
      ["parts"] = {
        uiTypes = {"angular", "vue"},
        uiTypesFilter = Constants.UiTypesFilter.all
      }
    }
  },
  ["vehicles"] = {
    uiTypesFilter = Constants.UiTypesFilter.only
  },
  ["vehiclesdetails"] = {
    uiTypesFilter = Constants.UiTypesFilter.only
  },
}

M.ModuleSettings = ModuleSettings
M.Routes = Routes

return M
