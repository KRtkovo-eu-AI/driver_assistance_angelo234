-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

M.qualityLevels = {
  Lowest = {
    caseSensitive = 1,
  },
  Low = {
    caseSensitive = 1,
  },
  Normal = {
    caseSensitive = 1,
  },
  High = {
    caseSensitive = 1,
  }
}

M.qualityLevels.Lowest["$pref::Shadows::textureScalar"] = 0.25
M.qualityLevels.Lowest["$pref::Shadows::disable"] = 2

M.qualityLevels.Low["$pref::Shadows::textureScalar"] = 0.5
M.qualityLevels.Low["$pref::Shadows::disable"] = 1

M.qualityLevels.Normal["$pref::Shadows::textureScalar"] = 1.0
M.qualityLevels.Normal["$pref::Shadows::disable"] = 0

M.qualityLevels.High["$pref::Shadows::textureScalar"] = 2.0
M.qualityLevels.High["$pref::Shadows::disable"] = 0

return M