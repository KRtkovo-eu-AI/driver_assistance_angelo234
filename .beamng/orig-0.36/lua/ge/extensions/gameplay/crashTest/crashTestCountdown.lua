-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local timer = 0
local old = 0
local callback = nil

local function startNewCountdown(duration, callback_)
  timer = duration
  old = duration
  callback = callback_
end

local function triggerDisplayUpdate()
  local msg = "Go!"
  if timer > 0 then
    msg = tostring(math.ceil(timer))
  end

  guihooks.trigger('ScenarioFlashMessage', {{msg, 0.95, "", true}})
end

local function triggerSoundUpdate()

  if math.ceil(timer) > 0 then
    Engine.Audio.playOnce('AudioGui', 'event:UI_Countdown1')
  else
    Engine.Audio.playOnce('AudioGui', 'event:UI_CountdownGo')
    callback()
  end
end

local function updateCoundown(dtSim)
  timer = timer - dtSim

  if math.floor(timer) ~= old then
    old = math.floor(timer)
    triggerDisplayUpdate()
    triggerSoundUpdate()
  end
end

local function onUpdate(dtReal, dtSim)
  if timer > 0 then
    updateCoundown(dtSim)
  end
end

M.onUpdate = onUpdate

M.startNewCountdown = startNewCountdown
return M
