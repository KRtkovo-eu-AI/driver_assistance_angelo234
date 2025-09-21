-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

log("E", "", "Something tried to use the deprecated 'lua/vehicle/motionSim.lua' file. If you are the programmer/modder responsible for that attempt, please check https://go.beamng.com/protocols")
print(debug.tracesimple())
return {}
