-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

package.path = 'lua/console/?.lua;lua/gui/?.lua;lua/common/?.lua;lua/common/socket/?.lua;lua/?.lua;?.lua'
package.cpath = ''

extensions = require("extensions")
extensions.addModulePath("lua/ge/extensions/")
extensions.addModulePath("lua/common/extensions/")

require('luaCore')
require('common/cdefs')
require('utils')
require('mathlib')

Engine = { Profiler = { pushEvent = nop, popEvent = nop}}
