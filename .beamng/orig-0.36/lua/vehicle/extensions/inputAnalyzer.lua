-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local max = math.max
local min = math.min
local abs = math.abs

local inputCheckTimer = 0
local inputCheckTime = 1 / 30

local inputString = ""
local inputs = {}
local lastInputs = {}

local registeredInputStrings = {}

local iV = {
  u = "throttle_input",
  d = "brake_input",
  l = "steering_input",
  r = "steering_input",
  a = "clutch_input",
  b = "parkingbrake_input"
}

local absMin = function(number)
  return abs(min(number, 0))
end

local absMax = function(number)
  return abs(max(number, 0))
end

local iF = {
  u = abs,
  d = abs,
  l = absMin,
  r = absMax,
  a = abs,
  b = abs
}

local function checkInput(dt)
  for k, v in pairs(iV) do
    lastInputs[k] = inputs[k]
    inputs[k] = iF[k](electrics.values[v], 0) > 0.01
  end

  local keyPressed = false
  for k, v in pairs(inputs) do
    if v and v ~= lastInputs[k] then
      keyPressed = true
      break
    end
  end

  if keyPressed then
    for k, v in pairs(inputs) do
      if v and v ~= lastInputs[k] then
        inputString = inputString .. k
      end
    end

    local inputLength = inputString:len()
    for _, is in ipairs(registeredInputStrings) do
      if inputLength >= is.stringLength then
        local subStr = inputString:sub(inputLength - is.stringLength + 1)
        if subStr == is.string then
          is.callback(inputString)
        end
      end
    end
    if inputLength > 16 then
      inputString = inputString:sub(2)
    end
  end
  --print(inputString)
end

local function updateGFX(dt)
  inputCheckTimer = inputCheckTimer + dt
  if inputCheckTimer >= inputCheckTime then
    checkInput(inputCheckTimer)
    inputCheckTimer = inputCheckTimer - inputCheckTime
  end
end

local function registerInputString(str, callback)
  local strLength = str:len()
  if strLength < 8 or strLength > 16 then
    return
  end
  local duplicate = false
  for _, v in ipairs(registeredInputStrings) do
    if v.string == string and v.callback == callback then
      duplicate = true
      break
    end
  end

  if not duplicate then
    table.insert(registeredInputStrings, {string = str, stringLength = strLength, callback = callback})
  end

  if tableIsEmpty(registeredInputStrings) then
    M.updateGFX = nop
  else
    M.updateGFX = updateGFX
  end
end

local function unregisterInputString(string, callback)
  local indexToRemove
  for k, v in ipairs(registeredInputStrings) do
    if v.string == string and v.callback == callback then
      indexToRemove = k
      break
    end
  end
  if indexToRemove then
    table.remove(registeredInputStrings, indexToRemove)
  end
  if tableIsEmpty(registeredInputStrings) then
    M.updateGFX = nop
  else
    M.updateGFX = updateGFX
  end
end

local function onReset()
end

local function onInit()
end

M.onInit = onInit
M.onReset = onReset
M.updateGFX = updateGFX

M.registerInputString = registerInputString
M.unregisterInputString = unregisterInputString

return M
