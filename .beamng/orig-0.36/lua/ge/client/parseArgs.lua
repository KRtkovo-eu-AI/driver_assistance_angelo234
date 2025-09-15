-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.args = {}

-- Support functions used to manage the directory list
local function pushFront(list, token, delim)
  if list ~= "" then
    return token .. delim .. list
  end
  return token
end

local function pushBack(list, token, delim)
  if list ~= "" then
    return list .. delim .. token
  end
  return token
end

local function popFront(list, delim)
  --TODO:
  return nextToken(list, unused, delim)
end

-- The default global argument parsing
local function defaultParseArgs()
  M.args = {}
  local argumentCount = tonumber(getConsoleVariable("$Game::argc"))
  -- log('I','parse','$Game::argv = '..getConsoleVariable("$Game::argv")..'     argumentCount: '..tostring(argumentCount))
  if argumentCount then
    for i = 0, argumentCount - 1 do
      local arg = getConsoleVariable("$Game::argv".. tostring(i))
      local nextArg = getConsoleVariable("$Game::argv".. tostring(i + 1))
      local hasNextArg = (argumentCount - i) > 1
      -- log('I','parse',"    $Game::argv".. tostring(i).."= "..tostring(arg))

      if arg == "-log" then
        if hasNextArg == true then
          -- Turn on console logging
          if nextArg ~= 0 then
            -- Dump existing console to logfile first.
            nextArg = nextArg + 4
          end
          setLogMode(nextArg)
          setConsoleVariable("$logModeSpecified", true)
          i = i + 1
        else
          log("E", "", "Error: Missing Command Line argument. Usage: -log <Mode: 0,1,2>")
        end
      elseif arg == "-cefdev" then
        enableCEFDevConsole(true)
      elseif arg == "-fullscreen" then
        setFullScreen(true)
      elseif arg == "-windowed" then
        setFullScreen(false)
      elseif arg == "-vehicleConfig" then
        if hasNextArg then
          M.args.vehicleConfig = nextArg
          i = i + 1
        else
          log("E", "", "Error: Missing Command Line argument. Usage: -vehicleConfig \"pickup/myConfig.pc\"")
        end
      elseif arg == "-vehicle" then
        if hasNextArg then
          setConsoleVariable("$beamngVehicleArgs", nextArg)
          i = i + 1
        else
          log("E", "", "Error: Missing Command Line argument. Usage: -vehicle <vehicle arg>")
        end
      elseif arg == "-luafile" then
        if hasNextArg then
          require(nextArg)
          i = i + 1
        else
          log("E", "", "Error: Missing Command Line argument. Usage: -luafile <lua file arg>")
        end
      elseif arg == "-lua" then
        if hasNextArg then
          LuaExecuteQueueString(nextArg)
          i = i + 1
        else
          log("E", "", "Error: Missing Command Line argument. Usage: -lua <lua arg>")
        end
      elseif arg == "-onLevelLoad_ext" then
        if hasNextArg then
          queueCmdlineLevelLoadExtension(nextArg)
          i = i + 1
        else
          log("E", "", "Error: Missing Command Line argument. Usage: -onLevelLoad_ext <argument>")
        end
      elseif arg == "-level" then
        if hasNextArg then
          setConsoleVariable("$levelToLoad", nextArg)
          i = i + 1
        else
          log("E", "", "Error: Missing Command Line argument. Usage: -level <level file name (no path), with or without extension>")
        end
      elseif arg == "-worldeditor" then
        setConsoleVariable("$startWorldEditor", true)
      elseif arg == "-tcom" then
        extensions.load('tech/techCore')
        tech_techCore.openServer()
      end
    end
  end
end

M.defaultParseArgs = defaultParseArgs

return M