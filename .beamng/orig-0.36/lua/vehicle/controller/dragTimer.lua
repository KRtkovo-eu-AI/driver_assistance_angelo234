-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.relevantDevice = "gearbox"

local sixtyFeet = 18.288 --m
local eighthMile = 201.168 --m
local quarterhMile = 402.336 --m

local state = nil
local countdownTimer = 0
local startingPosition = nil
local timer = 0
local sixtyFeetTime = 0
local eighthMileTime = 0
local eighthMileSpeed = 0
local quarterMileTime = 0
local quarterMileSpeed = 0

local function updateGFX(dt)
end

local function update(dt)
  local throttle = electrics.values.throttle or 0
  if state == "ready" then
    local transbrakeActive = electrics.values.transbrake or false
    if throttle > 0 and transbrakeActive then
      state = "countdown"
      countdownTimer = 3
      guihooks.message("Drag timer armed...", 4, "vehicle.dragtimer.status")
    end
  elseif state == "countdown" then
    local transbrakeActive = electrics.values.transbrake or false
    countdownTimer = countdownTimer - dt
    if countdownTimer <= 0 then
      state = "measuring"
      controller.getController("transbrake").setTransbrake(false)
      countdownTimer = 0
      startingPosition = obj:getPosition()
      timer = 0
      sixtyFeetTime = 0
      eighthMileTime = 0
      eighthMileSpeed = 0
      quarterMileTime = 0
      quarterMileSpeed = 0
      guihooks.message("Go!", 1, "vehicle.dragtimer.status")
    end
    if throttle <= 0 or not transbrakeActive then
      state = "ready"
    end
  elseif state == "measuring" then
    timer = timer + dt
    local currentPos = obj:getPosition()
    local currentDistance = (currentPos - startingPosition):length()
    local currentVelocity = obj:getVelocity():length()
    if currentDistance >= sixtyFeet and sixtyFeetTime <= 0 then
      sixtyFeetTime = timer
    end

    if currentDistance >= eighthMile and eighthMileTime <= 0 then
      eighthMileTime = timer
      eighthMileSpeed = currentVelocity
    end

    if currentDistance >= quarterhMile and quarterMileTime <= 0 then
      quarterMileTime = timer
      quarterMileSpeed = currentVelocity
      state = "done"
      timer = 0
    end

    if timer >= 25 then
      state = "done"
    end
  elseif state == "done" then
    guihooks.message("Results are in!", 2, "vehicle.dragtimer.status")
    print("Drag results:")
    print("60ft time: " .. sixtyFeetTime .. "s")
    print("1/8 time: " .. eighthMileTime .. "s")
    print("1/8 speed: " .. (eighthMileSpeed * 2.2369362920544) .. "mph")
    print("1/4 time: " .. quarterMileTime .. "s")
    print("1/4 speed: " .. (quarterMileSpeed * 2.2369362920544) .. "mph")

    state = "ready"
  end
end

local function init(jbeamData)
  state = "ready"
  countdownTimer = 0
  startingPosition = nil
  timer = 0
  sixtyFeetTime = 0
  eighthMileTime = 0
  eighthMileSpeed = 0
  quarterMileTime = 0
  quarterMileSpeed = 0
end

M.init = init
M.updateGFX = updateGFX
M.update = update

return M
