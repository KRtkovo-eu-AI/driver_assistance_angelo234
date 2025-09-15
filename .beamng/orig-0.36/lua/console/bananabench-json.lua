-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local bench = require("lua/console/bananabench")

--dump(args)

local outputFilename = 'bananabench.json'

if args and #args > 1 then
    outputFilename = args[2]
end

local res = bench.physics()
--dump(res)
jsonWriteFile(outputFilename, res, true)
