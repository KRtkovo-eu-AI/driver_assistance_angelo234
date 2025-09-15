-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local keywordWhiteList = {"not", "true", "false", "nil", "and", "or", "dt"}
local keyworkdWhiteListLookup = nil

local customElectricsEnv
local pwmEnv
local updateCustomElectricsFunction

local customDefaultValues

--function used as a case selector, input can be both int and bool as the first argument, any number of arguments after that
--in case it's a bool, it works like a ternary if, returning the second param if true, the third if false
--if the selector is an int n, it simply returns the nth+1 param it was given, if n > #params it returns the last given param
local function case(selector, ...)
  local index = 0
  local selectorType = type(selector)

  if selectorType == "boolean" then
    index = selector and 1 or 0
  elseif selectorType == "number" then
    index = math.floor(selector) --make sure we have an int for table access
  else
    log("E", "jbeam.expressionParser.parse", "Only booleans and numbers are supported as case selectors! Defaulting to last argument... Type: " .. selectorType)
  end

  local arg = {...}
  return arg[index] or arg[select("#", ...)] --fetch value from given index or from the last index
end

function pwm(input, frequency, dutyCycle, timeOffset)
  -- Calculate the sine wave normalized to [0, 1] with a given frequency and phase (time) offset
  local sineWave = (sin(2 * pi * frequency * pwmTime + (timeOffset or 0)) + 1) / 2
  --Generate a pwm signal based on the duty cycle
  return (sineWave > (1 - dutyCycle)) and input or 0
end

local function buildBaseEnv()
  --we build our custom environment for the parsed lua from jbeam variables
  --we also include a list of variables from the pc file so that we can keep backwards compatibility with (now) removed variables
  customElectricsEnv = {}
  --include all math functions and constants
  for k, v in pairs(math) do
    customElectricsEnv[k] = v
  end

  --also include a few of our own math functions
  customElectricsEnv.dump = dump
  customElectricsEnv.dumpz = dumpz
  customElectricsEnv.round = round
  customElectricsEnv.square = square
  customElectricsEnv.clamp = clamp
  customElectricsEnv.smoothstep = smoothstep
  customElectricsEnv.smootherstep = smootherstep
  customElectricsEnv.smoothmin = smoothmin
  customElectricsEnv.sign = sign
  customElectricsEnv.case = case
  customElectricsEnv.pwm = pwm
  customElectricsEnv.logValue = function(value, label)
    print(string.format("Custom Electric: %s = %s", label or "<no label>", tostring(value)))
    return value
  end
  customElectricsEnv.dump = dump
  customElectricsEnv.electrics = {}

  --build our kweyword lookup table
  keyworkdWhiteListLookup = {}
  for _, v in pairs(keywordWhiteList) do
    keyworkdWhiteListLookup[v] = true
  end

  pwmEnv = {sin = math.sin, pi = math.pi, pwmTime = 0}
  --pwm function needs access to a global time variable, hence it also gets a custom environment
  setfenv(customElectricsEnv.pwm, pwmEnv)

  --dump(customElectricsEnv)
end

local function updateEnvElectrics(dt)
  table.clear(customElectricsEnv.electrics)
  --TODO invesitgate if we maybe only want to update the values that are actually used within the custom value functions
  for electricsName, electricsValue in pairs(electrics.values) do
    customElectricsEnv.electrics[electricsName] = electricsValue
  end
  --pwm time counts up to 1h and then resets to 0, this means we don't support any pwm frequencies lower than that
  pwmEnv.pwmTime = (pwmEnv.pwmTime + dt) % 3600
end

local function updateElectrics(dt)
  for electricsName, customValue in pairs(customElectricsEnv.electrics) do
    electrics.values[electricsName] = customValue
  end
  --dump(customElectricsEnv.electrics)
end

local function updateGFX(dt)
  --copy our main electrics into the sandboxed environment
  updateEnvElectrics(dt)

  --this method runs with its own sandboxed environment
  updateCustomElectricsFunction(dt)

  --apply chnages from the custom update back into the main electrics
  updateElectrics(dt)
end

local function resetCustomValues()
  --print("resetCustomValues")
  --dump(customDefaultValues)
  for electricsName, value in pairs(customDefaultValues) do
    electrics.values[electricsName] = value
  end
end

local function parse(expr)
  --check if we find a *single standalone* "=" sign and abort parsing if found. >=, <=, == and ~= are allowed to support boolean operations

  if expr:find("[^<>~=]=[^=]") then
    log("E", "electricsCustomValueParser.parse", "Assignments are not supported inside expressions!")
    return nil
  end

  --special case: since we use components for the data storage, tables don't merge correctly.
  --this means that we can end up with multiple table headers in our data, this check here tries to identify that and skips
  if expr == "electricsFunction" then
    return nil
  end

  --find all literals (single letter, then >= 0 letters, numbers or _, so supported variable names in jbeam for this consist of [a-zA-Z0-9_])
  -- for s in expr:gmatch("%a[%a%d_%.]*") do
  --   print("Literal: " .. s)
  --   --   --we need to check if the literal exists in the env (functions mostly) or if it's in the whitelist (for stuff like true/false/etc)
  --   --   --if it's not in either table, it's forbidden and we abort parsing
  --   s = s:sub(1, (s:find("%.") or (s:len() + 1)) - 1)
  --   if not (customElectricsEnv[s] or keyworkdWhiteListLookup[s]) then
  --     log("E", "jbeam.expressionParser.parse", "Found illegal literal in expression: " .. s)
  --     return nil
  --   end
  -- end
  --load the now sanitized and sandbox checked code with our custom environment
  local exprFunc, message = load("return " .. expr, nil, "t", {})
  if exprFunc then
    return exprFunc
  else
    --syntax error most likely
    log("E", "electricsCustomValueParser.parse", "Parsing expression failed, message: " .. message)
    return nil
  end
end

local function sanitizeCustomExpression(expr)
  return expr or "" --return empty string if no expression exists (this is needed for custom values that need the reset but no updates)
end

local function compileCustomValueUpdates(customValueData)
  buildBaseEnv()
  --print("base env")
  --dump(customElectricsEnv)

  --print("customValueData")
  --dump(customValueData)

  customDefaultValues = {}

  local customValueUpdateStrings = {}
  for _, customValue in ipairs(customValueData) do
    local sanitizedExpression = sanitizeCustomExpression(customValue.electricsFunction)
    local parseResult = parse(sanitizedExpression)
    if parseResult then
      --only add to updates if there is a function (empty functions are still valid, but don't need updates here)
      if customValue.electricsFunction then
        table.insert(customValueUpdateStrings, customValue.electricsName)
        table.insert(customValueUpdateStrings, sanitizedExpression)
      end
      if customValue.electricsDefault then
        customDefaultValues[customValue.electricsName] = customValue.electricsDefault

        --init custom value in normal electrics
        electrics.values[customValue.electricsName] = customValue.electricsDefault
      else
        log("D", "electricsCustomValueParser.compileCustomValueUpdates", string.format("No default value for custom electrics value: %s", customValue.electricsName))
      end
    end
  end

  --print("customDefaultValues")
  --dump(customDefaultValues)

  --print("customValueUpdateStrings")
  --dump(customValueUpdateStrings)

  local customValueUpdates = {}
  for i = 1, #customValueUpdateStrings, 2 do
    --check length of code string and only add to updates if it's not empty (empty update strings are still valid, but don't need updates here)
    if #customValueUpdateStrings[i + 1] > 0 then
      table.insert(customValueUpdates, string.format("electrics.%s = %s", customValueUpdateStrings[i], customValueUpdateStrings[i + 1]))
    end
  end

  --print("customValueUpdates")
  --dump(customValueUpdates)

  local customValueUpdateString = "return function(dt) " .. table.concat(customValueUpdates, ";") .. " end"

  --print("customValueUpdateString")
  --dump(customValueUpdateString)
  local customValuesFunc, message = load(customValueUpdateString, nil, "t", customElectricsEnv)
  if customValuesFunc then
    updateCustomElectricsFunction = customValuesFunc()
  else
    --syntax error most likely
    log("E", "electricsCustomValueParser.compileCustomValueUpdates", "Compiling custom electrics values failed, message: " .. message)
    return nil
  end
end

M.compileCustomValueUpdates = compileCustomValueUpdates
M.resetCustomValues = resetCustomValues
M.updateGFX = updateGFX

return M
