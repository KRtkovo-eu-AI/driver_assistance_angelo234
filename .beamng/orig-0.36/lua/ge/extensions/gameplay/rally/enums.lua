-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local logTag = ''

local M = {}

M.pacenoteAudioMode = {
  auto = 1,
  freeform = 2,
  structuredOnline = 3,
  structuredOffline = 4,
  custom = 5,
}

M.pacenoteAudioModeNames = {
  [M.pacenoteAudioMode.auto] = 'auto',
  [M.pacenoteAudioMode.freeform] = 'freeform',
  [M.pacenoteAudioMode.structuredOnline] = 'structuredOnline',
  [M.pacenoteAudioMode.structuredOffline] = 'structuredOffline',
  [M.pacenoteAudioMode.custom] = 'custom',
}

M.drivelineMode = {
  recce = 1,
  route = 2,
}

M.drivelineModeNames = {
  [M.drivelineMode.recce] = 'recce',
  [M.drivelineMode.route] = 'route',
}

M.triggerType = {
  dynamic = 1,
  csImmediate = 15,
  -- csStatic = 20,
  -- csHalf = 30,
  -- ceMinus5 = 40,
  -- ceStatic = 50,
}

M.triggerTypeName = {
  [M.triggerType.dynamic] = 'dynamic (default)',
  [M.triggerType.csImmediate] = 'immediate (only use for first pacenote)',
  -- [M.triggerType.csStatic] = 'at corner start',
  -- [M.triggerType.csHalf] = 'halfway between corner start and corner end',
  -- [M.triggerType.ceMinus5] = '5m before corner end',
  -- [M.triggerType.ceStatic] = 'at corner end',
}

M.slowCornerReleaseType = {
  csStatic = 20,
  csHalf = 30,
  ceMinus5 = 40,
  ceStatic = 50,
}

M.slowCornerReleaseTypeName = {
  [M.slowCornerReleaseType.csStatic] = 'at corner start',
  [M.slowCornerReleaseType.csHalf] = 'halfway between corner start and corner end (default)',
  [M.slowCornerReleaseType.ceMinus5] = '5m before corner end',
  [M.slowCornerReleaseType.ceStatic] = 'at corner end',
}

return M