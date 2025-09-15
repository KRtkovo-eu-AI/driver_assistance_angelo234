--[[
This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
If a copy of the bCDDL was not distributed with this
file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
This module contains a set of functions which manipulate behaviours of vehicles.
]]

local M = {}

local jbeamUtils = require("jbeam/utils")

local supportedFileVersion = 2

local function process(vehicle)
  profilerPushEvent('jbeam/interaction.process')

  local interactionGroups = {}
  local actionCategories = {}
  local actions = {}


  local interactionFilenamesVehicle = FS:findFiles(vehicle.vehicleDirectory, '*.interaction.json', -1, false, false)
  local interactionFilenames = FS:findFiles('/vehicles/common/', '*.interaction.json', -1, false, false)
  arrayConcat(interactionFilenames, interactionFilenamesVehicle)
  --dump{'interactionFilenames', interactionFilenames}

  for _, filename in ipairs(interactionFilenames) do
    --log('I', 'events', 'loaded interaction file: ' .. tostring(filename))
    local j = jsonReadFile(filename)
    if j.fileversion ~= supportedFileVersion then
      log('E', 'interaction', 'interaction file wrong version. Supported version ' .. tostring(supportedFileVersion) .. ' - found version ' .. tostring(j.fileversion) .. '. Ignoring file ' .. tostring(filename))
      goto continue
    end

    tableMerge(interactionGroups, j.interactionGroups or {})
    tableMerge(actionCategories, j.actionCategories or {})
    for k,action in pairs(j.actions or {}) do
      action.source = filename
    end
    tableMerge(actions, j.actions or {})

    ::continue::
  end

  --dump{'interactionGroups', interactionGroups}
  --dump{'actionCategories', actionCategories}
  --dump{'actions', actions}

  -- store them for the otehr subsystems in the vehicle data
  -- i.e. used in lua/ge/extensions/core/input/actions.lua

  vehicle.interactionGroups = interactionGroups
  vehicle.actionCategories = actionCategories
  vehicle.inputActions = actions

  profilerPopEvent('jbeam/interaction.process')
end

M.process = process

return M