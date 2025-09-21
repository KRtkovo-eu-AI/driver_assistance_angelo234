-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local cachedPriceData
local levelUnit
local groupLC


local function _set7CharDisplay(objName, char)
  local obj = scenetree.findObject(objName)
  if not obj then log("E","setPrices",dumps(objName).."was not found");return end
  obj:preApply()
  -- obj:setField('shapeName', 0, "art/shapes/quarter_mile_display/display_".. char ..".dae")
  obj:setHidden(true)
  obj:postApply()
  local clone =  createObject('TSStatic')
  -- clone:preApply()
  clone:setField('shapeName', 0, "art/shapes/quarter_mile_display/display_".. char ..".dae")
  clone:setTransform(obj:getTransform())
  clone:setScale(obj:getScale())
  clone:setCanSave(false)
  -- clone:postApply()
  clone:registerObject(tostring(objName).."_localCopy")
  groupLC:addObject(clone)
end

local function setDisplayPrices()
  local levelName = getCurrentLevelIdentifier()
  if not levelName or levelName == '' then log("E","","Tried to get facility without level!") return false end

  local levelInfoData = core_levels.getLevelByName(levelName)
  if levelInfoData then
    levelUnit = levelInfoData.localUnits
  end

  local facilitiesData = freeroam_facilities.getFacilities(levelName)
  if not facilitiesData then
    log("E","fixedPrice", "facilitiesData invalid")
    return false
  end
  if not facilitiesData.gasStations then
    log("I","fixedPrice", "no gasStations")
    return false
  end
  local didWork = false
  groupLC = scenetree.findObject("fuelPrice_localCopies")
  if not groupLC then
    groupLC = createObject("SimGroup")
    groupLC:registerObject("fuelPrice_localCopies")
    groupLC.canSave = false
  else
    groupLC:deleteAllObjects()
  end
  cachedPriceData = {}
  for k,v in pairs(facilitiesData.gasStations) do
    if not v.prices then goto continueStation end
    for fuelType,v2 in pairs(v.prices) do
      if v2.disabled and v2.displayObjects then
        for i=1, #v2.displayObjects  do
          for _,objName in ipairs(v2.displayObjects[i]) do
            _set7CharDisplay(objName, "-")
          end
        end
      else
        didWork = true
        local price = v2.priceBaseline
        if v2.priceRandomnessGain and v2.priceRandomnessBias then
          price = price + v2.priceRandomnessGain * (math.random()-v2.priceRandomnessBias)
        end
        v2.price = price
        log("D","price",dumps(v.id).."\t"..dumps(fuelType) .. "\t".. dumps(price))
        if not v2.displayObjects then goto continueFuelType end
        if levelUnit and levelUnit[fuelType]=="gallonUS" then
          price = price * 3.78541 --US GAL
        end
        local priceStr = string.format("%.3f", price):gsub("%.", "")
        for i=1, #v2.displayObjects  do
          local char = priceStr:sub(i, i)
          -- force 9/10 in US signs
          if i ==4 and v2.us_9_10_tax then
            char = "9"
          end
          for _,objName in ipairs(v2.displayObjects[i]) do
            _set7CharDisplay(objName, char)
          end
        end
      end
      ::continueFuelType::
    end
    cachedPriceData[v.id] = v
    ::continueStation::
  end
  if not didWork then cachedPriceData=nil; return false end
  return true
end

local function getFuelPrice(stationId, fuelType) --always metric unit
  if not cachedPriceData then return nil end

  if cachedPriceData[stationId] and cachedPriceData[stationId].prices and cachedPriceData[stationId].prices[fuelType] then
    return cachedPriceData[stationId].prices[fuelType].price
  end
  return nil
end

local function onClientStartMission(levelPath)
  if not setDisplayPrices() then
    -- log("E","onClientStartMission","no price data")
    extensions.unload("freeroam_facilities_fuelPrice")
  end
end

-- local function onExtensionLoaded()
--   log("E","onExtensionLoaded","--------------------")
-- end
-- local function onExtensionUnloaded()
--   log("E","onExtensionUnloaded","--------------------")
-- end

local function onSerialize()
  local d = {}
  d.cachedPriceData = cachedPriceData
  d.currentLevel = getCurrentLevelIdentifier()
  d.levelUnit = levelUnit
  return d
end

local function onDeserialized(data)
  if data.currentLevel and data.currentLevel == getCurrentLevelIdentifier() and data.cachedPriceData then
    cachedPriceData = data.cachedPriceData
    levelUnit = data.levelUnit
  else
    onClientStartMission(getCurrentLevelIdentifier())
  end
end

-- freeroam_facilities_fuelPrice.restoreSign() if you need to modify placement
-- then add `true` as arg before commit to avoid change later
local function restoreSign(hide)
  if groupLC then
    groupLC:deleteAllObjects()
  end
  for k,v in pairs(cachedPriceData) do
    if not v.prices then goto continueStation end
    for fuelType,v2 in pairs(v.prices) do
      if v2.displayObjects then
        for i=1, #v2.displayObjects  do
          for _,objName in ipairs(v2.displayObjects[i]) do
            local obj = scenetree.findObject(objName)
            if not obj then log("E","setPrices",dumps(objName).."was not found");return end
            obj:preApply()
            obj:setField('shapeName', 0, "art/shapes/quarter_mile_display/display_8.dae")
            obj:setHidden(hide or false)
            obj:postApply()
          end
        end
      end
    end
    ::continueStation::
  end
end

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized
-- M.onExtensionLoaded = onExtensionLoaded
-- M.onExtensionUnloaded = onExtensionUnloaded

M.onClientStartMission = onClientStartMission

M.setDisplayPrices = setDisplayPrices
M.getFuelPrice = getFuelPrice

M.restoreSign = restoreSign

return M