local M = {}
local Constants = extensions.ui_router_constants

-- ModuleName(optional) by default is the name of the file unless specified otherwise
local ModuleName = "career"
local ModuleSettings = {
  uiTypes = {"vue"},
  uiTypesFilter = Constants.UiTypesFilter.only,
  infoBar = Constants.InfoBarDefaults,
}

local Routes = {
  ["pauseBigMiddlePanel"] = {
    name = "career.pauseBigMiddlePanel",
  },
  ["logbook"] = {
    name = "career.logbook",
  },
  ["milestones"] = {
    name = "career.milestones",
  },
  ["computer"] = {
    name = "career.computer",
  },
  ["vehicleInventory"] = {
    name = "career.vehicleInventory",
  },
  ["vehiclePerformance"] = {
    name = "career.vehiclePerformance",
  },
  ["tuning"] = {
    name = "career.tuning",
  },
  ["painting"] = {
    name = "career.painting",
  },
  ["repair"] = {
    name = "career.repair",
  },
  ["partShopping"] = {
    name = "career.partShopping",
  },
  ["partInventory"] = {
    name = "career.partInventory",
  },
  ["vehiclePurchase"] = {
    name = "career.vehiclePurchase",
  },
  ["vehicleShopping"] = {
    name = "career.vehicleShopping",
  },
  ["insurancePolicies"] = {
    name = "career.insurancePolicies",
  },
  ["cargoDeliveryReward"] = {
    name = "career.cargoDeliveryReward",
  },
  ["cargoDropOff"] = {
    name = "career.cargoDropOff",
  },
  ["cargoOverview"] = {
    name = "career.cargoOverview",
  },
  ["myCargo"] = {
    name = "career.myCargo",
  },
  ["progressLanding"] = {
    name = "career.progressLanding",
  },
  ["domainSelection"] = {
    name = "career.domainSelection",
  },
  ["branchPage"] = {
    name = "career.branchPage",
  },
  ["profiles"] = {
    name = "career.profiles",
  },
}

M.ModuleSettings = ModuleSettings
M.ModuleName = ModuleName
M.Routes = Routes

return M
