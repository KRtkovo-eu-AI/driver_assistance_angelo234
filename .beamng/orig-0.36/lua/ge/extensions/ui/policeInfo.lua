-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local lastPursuitData = {
  pursuitLevel = 0,
  sightValue = 0,
  arrest = 0,
  evade = 0
}
local info = {
  {txt = 0, key = 'duration'},
  {txt = 0, key = 'policeCount'},
  {txt = 0, key = 'uniqueOffensesCount'}
}

M.enabled = true

local function resetPursuitTable() -- resets values to zero
  for k, v in pairs(lastPursuitData) do
    if type(v) == 'number' then
      lastPursuitData[k] = 0
    end
  end
  lastPursuitData.info = ''
  lastPursuitData.alert = ''
  guihooks.trigger('PoliceInfoUpdate', lastPursuitData)
end

local function onVehicleSwitched(_, id)
  if M.enabled then
    resetPursuitTable()
  end
end

local function onPursuitOffense(vehId, name, data)
  if M.enabled and vehId == be:getPlayerVehicleID(0) then
    lastPursuitData.alert = {txt = 'ui.apps.police.offenseAlert', context = {value = translateLanguage('ui.traffic.infractions.'..name, name)}}
  end
end

local function onGuiUpdate(dt)
  if not be:getEnabled() or not M.enabled then return end

  local pursuit = gameplay_police.getPursuitData() -- player vehicle pursuit data
  if not pursuit then
    if lastPursuitData.pursuitScore ~= 0 then
      resetPursuitTable()
    end
    return
  end

  local pd = lastPursuitData
  pd.pursuitLevel = pursuit.mode
  pd.sightValue = pursuit.sightValue
  pd.arrest = lerp(pd.arrest, pursuit.timers.arrestValue, 0.5)
  pd.evade = lerp(pd.evade, pursuit.timers.evadeValue, 0.5)
  -- lerp is used to make the progress bar act fancy when values get reset to zero

  if pursuit.timers.main > 0 then
    pursuit.duration = pursuit.timers.main
  end

  if pursuit.mode == 0 then
    pd.info = ''
  else
    pd.info = info
    for _, v in ipairs(pd.info) do
      if type(v) == 'table' then
        v.txt = pursuit[v.key] or ''
        if type(v.txt) == 'number' and v.key == 'duration' then
          local minutes = math.floor(v.txt / 60)
          local seconds = math.floor(v.txt - minutes * 60)
          v.txt = string.format("%02d:%02d", minutes, seconds)
        end
      end
    end
  end

  guihooks.trigger('PoliceInfoUpdate', pd)
  pd.alert = nil -- alert gets pinged for one frame
end

M.onVehicleSwitched = onVehicleSwitched
M.onPursuitOffense = onPursuitOffense
M.onGuiUpdate = onGuiUpdate

return M