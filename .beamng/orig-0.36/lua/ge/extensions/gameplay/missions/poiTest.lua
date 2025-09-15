-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

-- poi list stuff
local function onGetRawPoiListForLevel(levelIdentifier, elements)
  for _, ps in ipairs(gameplay_sites_sitesManager.loadSites("gameplay/parkingSpotTests.sites.json").parkingSpots.sorted) do
    local id = string.format("Test-PS-%d", ps.id)
    table.insert(elements,  {
      id = id,
      -- this is located in
      pos = ps.pos,
      rot = ps.rot,
      scl = ps.scl,
      radius = 3,
      visibleInPlayMode = true,
      interactableInPlayMode = true,
      clusterType = "parkingMarker",

      data = {
        type = "poiTest",
        id = id,
        name = ps.name,
        description = "To Test the Poi System",
      } -- maybe put in some data for which testdrive is currently active
    })
  end
end

local function onPoiDetailPromptOpening(elemData, promptData)
  local poiName = false
  for _, elem in ipairs(elemData) do
    if elem.type == "poiTest" then
      poiName = elem.name
    end
  end
  if poiName then
    local ret = {}
    ret.label = poiName
    ret.buttonText = "Do the Test"
    ret.buttonFun = function() print("Test Successfull!") end
    table.insert(promptData, ret)
  end
end

M.onGetRawPoiListForLevel = onGetRawPoiListForLevel
M.onPoiDetailPromptOpening = onPoiDetailPromptOpening

M.onExtensionLoaded = function()
  gameplay_markerInteraction.clearCache()
end



return M