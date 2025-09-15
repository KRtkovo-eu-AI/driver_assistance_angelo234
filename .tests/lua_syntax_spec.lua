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
      local chunk, err = loadfile(file)
      assert.is_truthy(chunk, err)
    end)
  end
end)

