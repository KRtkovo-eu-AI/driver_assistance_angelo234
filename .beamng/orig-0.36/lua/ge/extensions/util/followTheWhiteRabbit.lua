-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- extensions.load('util_followTheWhiteRabbit')

local M = {}

local function onExtensionLoaded()
  consoleSetPrintLogTimeAndOrigin(false)
  consoleAddAvailableContext('neo')
  consoleSetContext('neo')
  consoleSetPrintLogTimeAndOrigin(false)

  print('> You enter a wide open space. Grey tiles on the ground, cubic objects in the horizon. A big banner reads "GRIDMAP" ahead of you. You are sitting inside a blue pickup. What do you do? < throttle / exit vehicle / change vehicle>')
end

local function onConsoleExecuteCommand(context, cmd)
  if context ~= 'neo' then return end
  if cmd == 'throttle' then
    print('You crash. Game over.')
  else
    -- here be dragons
  end
end

M.onExtensionLoaded = onExtensionLoaded
M.onConsoleExecuteCommand = onConsoleExecuteCommand

return M
