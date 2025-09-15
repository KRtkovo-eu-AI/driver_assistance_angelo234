--[[
This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
If a copy of the bCDDL was not distributed with this
file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
This module contains a set of functions which manipulate behaviours of vehicles.
]]

local M = {}


local function process(objID, vehicleObj, config, activePartsData)
  profilerPushEvent('jbeam/licenseplatesSkins.process')

  -- set license plate
  local useLicensePlate = false
  local licenseplatePath = ''
  local licensePlateUsedFormats = {}

  for partName, part in pairs(activePartsData) do
    if part.slotType then

      local isLicenseplate = false
      local isLicenseplateDesign = false
      local isSkin = false

      -- handling of slottypes properly with backward compatibility
      if type(part.slotType) == 'string' then
        isLicenseplate = part.slotType:find('_licenseplate')
        isLicenseplateDesign = part.slotType:find('licenseplate_design')
        isSkin = part.slotType:find('skin_') or part.slotType == 'paint_design'
      elseif type(part.slotType) == 'table' then
        for _, slotType in ipairs(part.slotType) do
          isLicenseplate = isLicenseplate or (type(slotType) == 'string' and slotType:find('_licenseplate'))
          isLicenseplateDesign = isLicenseplateDesign or (type(slotType) == 'string' and slotType:find('licenseplate_design'))
          isSkin = isSkin or (type(slotType) == 'string' and slotType:find('skin_') or slotType == 'paint_design')
        end
      end


      if part.licenseplateFormat or isLicenseplate then
        useLicensePlate = true
        if part.licenseplateFormat and not tableContains(licensePlateUsedFormats,part.licenseplateFormat) then
          table.insert(licensePlateUsedFormats, part.licenseplateFormat)
        elseif not part.licenseplateFormat and not tableContains(licensePlateUsedFormats,"30-15") then
          table.insert(licensePlateUsedFormats, "30-15")
        end
      end

      -- license plates setup
      if isLicenseplateDesign and part.licenseplate_path then
        licenseplatePath = part.licenseplate_path
      end

      -- skin setup
      if isSkin then
        local skinSlot = (type(part.skinType) == 'string' and part.skinType ~= '') and part.skinType or part.slotType
        if skinSlot == 'paint_design' or type(skinSlot) ~= 'string' then skinSlot = '' end
        vehicleObj:setSkin(skinSlot .. '.' .. (part.skinName or part.globalSkin or ''))
        if part.default_color ~= nil then
          extensions.core_vehicle_manager.setVehicleColorsNames(objID, {part.default_color, part.default_color_2, part.default_color_3})
        end
      end
    end
  end

  if useLicensePlate then
    extensions.core_vehicles.setPlateText((config and config.licenseName) or false, objID, licenseplatePath, licensePlateUsedFormats)
  end

  profilerPopEvent('jbeam/licenseplatesSkins.process')
end

M.process = process

return M