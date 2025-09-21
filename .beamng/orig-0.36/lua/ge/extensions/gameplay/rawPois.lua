-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local rawPoiGeneration = 0
local rawPoiListByLevel = {}

local function validateRawPoiElement(element)
  local valid = true
  if not element.data then
    log("E","","Element does not contain a data field! Not keeping it in the element list.")
    dumpz(element,2)
    valid = false
  end
  if element.data and not (element.data.id or element.data.missionId) then
    log("E","","Element.data does not contain neither id nor missionId! Not keeping it in the element list.")
    dumpz(element,2)
    valid = false
  end
  if not element.clusterType then
    log("E","","Element has no clusterType! Not keeping it in the element list.")
    dumpz(element,2)
  end
  return valid
end

local function getRawPoiListByLevel(levelIdentifier)
  -- when in the tutorial, only add the desired elements
  if career_career.isActive()  and levelIdentifier == "west_coast_usa" then
    -- show tutorial step specific pois
    if not career_modules_linearTutorial.getTutorialFlag("arrivedAtFuelstation") then
      local gasStation = freeroam_facilities.getGasStation("apex")
      local elements = {
        freeroam_gasStations.formatGasStationPoi(gasStation)
      }
      return elements, rawPoiGeneration
    elseif  not career_modules_linearTutorial.getTutorialFlag("completedTutorialMission") then
      local elements = {}
      gameplay_missions_missions.formatMissionToRawPoi(gameplay_missions_missions.getMissionById("west_coast_usa/arrive/005-ArriveTutorial"), elements, levelIdentifier)
      return elements, rawPoiGeneration
    elseif  not career_modules_linearTutorial.getTutorialFlag("purchasedFirstCar") then
      local elements = {}
      local dealer = freeroam_facilities.getDealership("quarrysideAutoSales")
      freeroam_facilities.walkingMarkerFormatFacility(dealer, elements)
      return elements, rawPoiGeneration
    elseif  not career_modules_linearTutorial.getTutorialFlag("modifiedFirstCar") then
      local elements = {}
      local garage = freeroam_facilities.getFacility("computer", "servicestationGarageComputer")
      freeroam_facilities.formatFacilityToRawPoi(garage, elements)
      return elements, rawPoiGeneration
    end

    -- only show dealership when car not bought
    if not career_career.hasBoughtStarterVehicle() then
      local elements = {}
      local dealer = freeroam_facilities.getDealership("quarrysideAutoSales")
      freeroam_facilities.walkingMarkerFormatFacility(dealer, elements)
      return elements, rawPoiGeneration
    end
  end

  -- otherwise create poi list as usual
  if not rawPoiListByLevel[levelIdentifier] then
    local elementsUnchecked, elements = {}, {}
    -- call all extensions to add their POIs
    extensions.hook("onGetRawPoiListForLevel",levelIdentifier, elementsUnchecked)
    for _, e in ipairs(elementsUnchecked) do
      -- sanity check

      -- make clustering true by default
      if e.markerInfo.bigmapMarker and e.markerInfo.bigmapMarker.cluster == nil then
        e.markerInfo.bigmapMarker.cluster = true
      end

      if (not career_modules_testDrive or not career_modules_testDrive.isActive() or (e.data and e.data.type == "testDriveEnd")) then
        table.insert(elements, e)
      end
    end
    rawPoiListByLevel[levelIdentifier] = elements
  end
  return rawPoiListByLevel[levelIdentifier], rawPoiGeneration
end

M.getRawPoiGeneration = function() return rawPoiGeneration end
M.getRawPoiListByLevel = getRawPoiListByLevel
M.clear = function()
  rawPoiListByLevel = {}
  rawPoiGeneration = rawPoiGeneration + 1
  log("D","","Raw Poi Lists Cleared. New Generation: " .. rawPoiGeneration)
end
M.showMissionMarkersToggled = M.clear
M.onModManagerReady = M.clear
return M
