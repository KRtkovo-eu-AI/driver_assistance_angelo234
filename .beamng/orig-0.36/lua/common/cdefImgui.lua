-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- reads the content of a file
function readFile(filename)
  local f = io.open(filename, "r")
  if f == nil then
    return nil
  end
  local content = f:read("*all")
  f:close()
  return content
end

if ffi then
  -- base requirement for imgui_gen.h
  ffi.cdef([[
  typedef struct ImVector {
    int Size;
    int Capacity;
    const void* Data;
  } ImVector;
  typedef struct ImVec2 {
    float x;
    float y;
  } ImVec2;
  typedef struct ImVec4 {
    float x;
    float y;
    float z;
    float w;
  } ImVec4;
  typedef struct ImGuiContext {} ImGuiContext;
  typedef struct ImDrawListSharedData {} ImDrawListSharedData;
  typedef struct ImFontBuilderIO {} ImFontBuilderIO;
  ]])

  if IMGUI_LUAINTF then
    ffi.cdef("int ImGuiInputTextCallbackLua(const void* data);") -- only to prevent errors with current lua
  end
  if not IMGUI_LUAINTF then
    ffi.cdef(readFile('lua/common/extensions/ui/imgui_gen.h'))
    ffi.cdef(readFile('lua/common/extensions/ui/imgui_custom.h'))
    ffi.cdef(readFile('lua/common/extensions/ui/flowgraph/editor_api.h'))
    ffi.cdef("int ImGuiInputTextCallbackLua(const ImGuiInputTextCallbackData* data);")
  end
end
