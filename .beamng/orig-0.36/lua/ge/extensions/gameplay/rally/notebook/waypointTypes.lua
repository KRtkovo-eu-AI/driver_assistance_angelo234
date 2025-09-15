-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local wpTypeFwdAudioTrigger = "fwdAudioTrigger"
local wpTypeRevAudioTrigger = "revAudioTrigger"
local wpTypeCornerStart = "cornerStart"
local wpTypeCornerEnd = "cornerEnd"

local shortener_map = {
  [wpTypeFwdAudioTrigger] = "At",
  [wpTypeRevAudioTrigger] = "Ar",
  [wpTypeCornerStart] = "Cs",
  [wpTypeCornerEnd] = "Ce",
}

local function shortenWaypointType(wpType)
  return shortener_map[wpType]
end

M.wpTypeFwdAudioTrigger = wpTypeFwdAudioTrigger
M.wpTypeRevAudioTrigger = wpTypeRevAudioTrigger
M.wpTypeCornerStart = wpTypeCornerStart
M.wpTypeCornerEnd = wpTypeCornerEnd

M.shortenWaypointType = shortenWaypointType

return M
