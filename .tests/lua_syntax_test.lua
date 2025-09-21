local laura = require('laura')
local describe = laura.describe
local it = laura.it
local expect = laura.expect

local dirs = {'scripts', 'lua'}

-- gather all lua files under specified directories
local files = {}
for _, dir in ipairs(dirs) do
  local p = io.popen('find '..dir..' -name "*.lua"')
  for file in p:lines() do
    table.insert(files, file)
  end
  p:close()
end

table.sort(files)

describe('Lua syntax check', function()
  for _, file in ipairs(files) do
    it('parses '..file, function()
      expect(function()
        local chunk, err = loadfile(file)
        if not chunk then
          error(err, 0)
        end
      end).notToFail()
    end)
  end
end)

