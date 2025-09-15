-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- require('introspection').gather_methods()

local M = {}

-- Function to gather methods from a given table
local function gather_methods()
  local methods = {}

  -- Helper function to gather methods from a table
  local function gather_from_table(tbl, class_name, is_static)
    for key, value in pairs(tbl) do
      if type(key) == "string" and type(value) == "function" then
        -- Ignore keys starting with "__"
        if not key:match("^__") then
          local info = debug.getinfo(value, "S")
          local signature = key
          if info.what then
            signature = signature .. " (" .. info.what .. ")"
          end
          -- Store method details in the methods table
          table.insert(methods, {
            class_name = class_name,
            method_name = key,
            signature = signature,
            is_static = is_static
          })
        end
      end
    end
  end

  -- Iterate over global variables
  for global_key, global_value in pairs(_G) do
    -- Ignore the global variable named "extensions"
    if global_key ~= "extensions" then
      if type(global_key) == "string" then
        if type(global_value) == "table" then
          -- Check if the table has a ___type field indicating a class
          if global_value.___type then
            local class_name = global_value.___type:match("class<(.+)>") or global_value.___type:match("static_class<(.+)>")
            if class_name then
              local is_static = global_value.___type:match("static_class<") ~= nil
              gather_from_table(global_value, class_name, is_static)
              -- Gather methods from the metatable, if any
              local mt = getmetatable(global_value)
              if mt then
                gather_from_table(mt, class_name, is_static)
              end
            end
          end
        else
          -- Check the metatable of global_value for class information
          local mt = getmetatable(global_value)
          if mt then
            if mt.___type then
              local class_name = mt.___type:match("class<(.+)>") or mt.___type:match("static_class<(.+)>")
              if class_name then
                local is_static = mt.___type:match("static_class<") ~= nil
                gather_from_table(mt, class_name, is_static)
              end
            end
          end
        end
      end
    end
  end

  -- Gather methods from built-in modules
  local built_in_modules = { "table", "math", "string", "coroutine", "os", "io", "debug", "package" }
  for _, module_name in ipairs(built_in_modules) do
    local module = _G[module_name]
    if type(module) == "table" then
      gather_from_table(module, module_name, true)
    end
  end

  return methods
end

M.gather_methods = gather_methods

return M