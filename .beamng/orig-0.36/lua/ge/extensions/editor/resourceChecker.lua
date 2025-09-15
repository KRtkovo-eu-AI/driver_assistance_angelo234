-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local im = ui_imgui
local resourceUtil = extensions.editor_resourceChecker_resourceUtil

local pos = im.ImVec2(0, 0)

local toolWindowName = 'resourceChecker'
local imageWindowName = 'resourceChecker_imagePrev'
local duplicateResolverWindowName = 'resourceChecker_duplicatePrev'

local verifier
local function onWindowMenuItem()
  editor.showWindow(toolWindowName)
end

local function getProgress()
  local progress = resourceUtil.getProgress()
  return progress
end

local function getSimObjects(fileName)
  local ret = {}
  local objs = scenetree.getAllObjects()
  --log('E', '', '# objects existing: ' .. tostring(#scenetree.getAllObjects()))
  for _, objName in ipairs(objs) do
    local o = scenetree.findObject(objName)
    if o then
      if o:getFileName() == fileName then
        table.insert(ret, o)
      end
    end
  end
  return ret
  --log('E', '', '# objects left: ' .. tostring(#scenetree.getAllObjects()))
end

local function getPlayerVehiclePath()
  local vehid = be:getPlayerVehicleID(0)
  if vehid ~= -1 then
    local veh = getObjectByID(vehid)
    if veh then
      return "/vehicles/" .. veh.JBeam .. "/"
    end
  end
  return nil
end

local function drawRectBg(text, color, hover, smol)
  local textSize = im.CalcTextSize(text)
  local leftT = im.GetCursorScreenPos()
  leftT = im.ImVec2(leftT.x - 2, leftT.y - 1)
  if hover == 1 then
    leftT = im.ImVec2(leftT.x - 2, (leftT.y - 1)-textSize.y-1)
  end
  local window = im.GetWindowWidth()
  local rightB
  if smol == 1 then
    rightB = im.ImVec2(leftT.x+textSize.x+3, leftT.y+textSize.y+4)
  else
    rightB = im.ImVec2(leftT.x + window, leftT.y + textSize.y+1)
  end
  im.ImDrawList_AddRectFilled(im.GetWindowDrawList(), leftT, rightB, im.GetColorU322(color), 0, nil)
end

local imageToPreview = nil
local function imagePreview()
  if editor.beginWindow(imageWindowName, "Image Preview",  im.flags(im.WindowFlags_NoScrollbar, im.WindowFlags_NoDocking)) then
    if not imageToPreview then im.Text("Image is not selected")
    else
      local windowSize = im.GetWindowSize()
      local img = nil
      local ratio = nil
      img = editor.getTempTextureObj(imageToPreview)
      if img and img.size.x > 0 and img.size.y > 0 then
        ratio = img.size.y / img.size.x
        local sizex = windowSize.y / ratio  - (2 * im.uiscale[0])
        local sizey = windowSize.y - (2 * im.uiscale[0])
        im.Image(
          img.tex:getID(),
          im.ImVec2(sizex, sizey),
          nil, nil, nil,
          editor.color.white.Value
        )
      end
    end
  end
  editor.endWindow()
end

local function texHovered(res)
  if res and not FS:fileExists(res) and string.endswith(res, ".png" ) then
    res = string.gsub(res, ".png", ".dds")
  end
  if res and FS:fileExists(res) then
    local cases = {".dds", ".png", ".bmp", ".jpg", ".jpeg", ".tga"}
    local isTexture = false
    for _,b in pairs(cases) do
      if string.lower(res):find(b) then isTexture = true goto safeCnt end
    end
    ::safeCnt::
    if isTexture == true then
      im.BeginTooltip()
      local img = nil
      local ratio = nil
      img = editor.getTempTextureObj(res)
      if img and img.size.x > 0 and img.size.y > 0 then
        ratio = img.size.y / img.size.x
        local sizex = (64 / ratio) * im.uiscale[0]
        local sizey = 64 * im.uiscale[0]
        im.Image(
          img.tex:getID(),
          im.ImVec2(sizex, sizey),
          nil, nil, nil,
          editor.color.white.Value
        )
      end
      im.EndTooltip()
    end
  end
end

local duplicateTable = nil
local duplicateName = nil
local copiedData = {}

local function duplicateContextMenu(res, count)
  if im.BeginPopup("Popup_" .. res[2]..count) then
    if im.Selectable1("Copy material data##"..res[2]) then
      copiedData = {}
      copiedData = res[3]
      im.CloseCurrentPopup()
    end
    if im.Selectable1("Paste material data##"..res[2]) then
      if copiedData and copiedData ~= {} and copiedData ~= "removed" then
        duplicateTable[res[1]][res[2]] = copiedData
        duplicateTable[res[1]].modified = true
      end
      im.CloseCurrentPopup()
    end
    if im.Selectable1("Remove duplicate##"..res[2]) then
      duplicateTable[res[1]][res[2]] = "removed"
      duplicateTable[res[1]].modified = true
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end
end

local function duplicateResolver()
  if editor.beginWindow(duplicateResolverWindowName, "Duplicate data",  im.flags(im.WindowFlags_NoDocking)) then
    if not duplicateName then im.Text("Nothing to check")
    elseif duplicateName and not duplicateTable then im.Text("Checking in progress")
    elseif duplicateName and tableIsEmpty(duplicateTable) then im.Text("Found 0 duplicates")
    else
      local windowSize = im.GetContentRegionAvail()
      im.Text("Manage duplicates of selected material")
      im.BeginChild1("##duplicateChild", im.ImVec2(0, windowSize.y-60* im.uiscale[0], false, im.WindowFlags_ChildWindow))
      local fontPushed = im.PushFont3("robotomono_regular")
      local count = 0
      for k,v in pairs(duplicateTable) do
        count = count + 1
        if (count % 2 == 0) then
          drawRectBg(tostring(k), im.ImVec4(1, 1, 1, 0.06))
        end
        if im.TreeNode1("File: "..k) then
          for l,b in pairs(v) do
            count = count + 1
            duplicateContextMenu({k,l,b}, count)
            if (count % 2 == 0) then
              drawRectBg(tostring(l), im.ImVec4(1, 1, 1, 0.06))
            end
            if l ~= 'modified' then
              if b == "removed" then
                im.Text("  Material: "..l.." - Deleted")
              else
                if im.TreeNode1("  Material: "..l) then
                  if im.IsItemHovered() and im.IsMouseClicked(1) then
                    im.OpenPopup("Popup_"..l..count)
                  end
                  for j,c in pairs(b) do
                    count = count + 1
                    duplicateContextMenu({k,l,b}, count)
                    if (count % 2 == 0) then
                      drawRectBg(tostring(j), im.ImVec4(1, 1, 1, 0.06))
                    end
                    if j ~= "Stages" then im.Text(j..": "..tostring(c)) end
                    if im.IsItemHovered() and im.IsMouseClicked(1) then
                      im.OpenPopup("Popup_"..l..count)
                    end
                    if j == "Stages" then
                      if im.TreeNode1("Stages:") then
                        local stage = 0
                        for s,t in pairs(c) do
                          stage = stage + 1
                          if not tableIsEmpty(t) then
                            count = count + 1
                            duplicateContextMenu({k,l,b}, count)
                            if (count % 2 == 0) then
                              drawRectBg(tostring(stage), im.ImVec4(1, 1, 1, 0.06))
                            end
                            if im.TreeNode1("Stage "..stage..":") then
                              for m,x in pairs(t) do
                                count = count + 1
                                duplicateContextMenu({k,l,b}, count)
                                if (count % 2 == 0) then
                                  drawRectBg(tostring(x), im.ImVec4(1, 1, 1, 0.06))
                                end
                                if type(x) == "table" then
                                  local txtstring = m..": "
                                  for a,f in pairs(x) do
                                    txtstring = txtstring..tostring(f)..", "
                                  end
                                  if string.endswith(txtstring, ", " ) then
                                    local stringend = ", "
                                    txtstring = string.sub(txtstring, 1, #txtstring - #stringend)
                                  end
                                  im.Text(txtstring)
                                else
                                  im.Text(m..": "..tostring(x))
                                  if im.IsItemHovered() and type(x) == "string" then
                                    texHovered(x)
                                    if im.IsMouseClicked(1) then
                                      im.OpenPopup("Popup_"..l..count)
                                    end
                                  end
                                end
                                if im.IsItemHovered() and im.IsMouseClicked(1) then
                                  im.OpenPopup("Popup_"..l..count)
                                end
                              end
                              im.TreePop()
                            end
                            if im.IsItemHovered() and im.IsMouseClicked(1) then
                              im.OpenPopup("Popup_"..l..count)
                            end
                          end
                        end
                        im.TreePop()
                        if im.IsItemHovered() and im.IsMouseClicked(1) then
                          im.OpenPopup("Popup_"..l..count)
                        end
                      end
                      if im.IsItemHovered() and im.IsMouseClicked(1) then
                        im.OpenPopup("Popup_"..l..count)
                      end
                    end
                  end
                  im.TreePop()
                  if im.IsItemHovered() and im.IsMouseClicked(1) then
                    im.OpenPopup("Popup_"..l..count)
                  end
                end
                if im.IsItemHovered() and im.IsMouseClicked(1) then
                  im.OpenPopup("Popup_"..l..count)
                end
              end
            end
            im.Separator()
          end
          im.TreePop()
        end
        im.Separator()
      end
      if fontPushed then
        im.PopFont()
      end
      im.EndChild()
      im.Separator()
      im.Spacing()
      im.Dummy(im.ImVec2(im.GetContentRegionAvailWidth()-220* im.uiscale[0], 0))
      im.SameLine()
      if im.Button("Save changes and close window", im.ImVec2(220* im.uiscale[0],0)) then
        for k,v in pairs(duplicateTable) do
          if FS:fileExists(k) and v.modified == true then
            log('D', '', 'Detected modified material file '..k )
            local materialFile = jsonReadFile(k) or {}
            for l,b in pairs(v) do
              if materialFile[l] and b ~= "removed" then
                materialFile[l] = b
              end
              if materialFile[l] and b == "removed" then
                materialFile[l] = nil
              end
            end
            log('I', '', 'Saved materials to '..k )
            jsonWriteFile(k, materialFile, true)
            resourceUtil.resaveMaterial(k)
          end
        end
        editor.hideWindow(duplicateResolverWindowName)
      end
    end
  end
  editor.endWindow()
end

local shapePreview = ShapePreview()
local dimRdr = RectI(0, 0, 128, 128)
shapePreview:setRenderState(false,false,false,false,false,false)
shapePreview:setCamRotation(0.6, 3.9)
shapePreview:renderWorld(dimRdr)
shapePreview.mBgColor = ColorI(28,38,51,255)
local function shapeHovered(res)
  local shapePreviewSize = 128 * im.uiscale[0]
  if res and FS:fileExists(res) then
    shapePreview:setObjectModel(res)
    shapePreview:fitToShape()
    im.BeginTooltip()
    dimRdr:set(0, 0, shapePreviewSize, shapePreviewSize)
    shapePreview:renderWorld(dimRdr)
    shapePreview:ImGui_Image(shapePreviewSize, shapePreviewSize)
    im.EndTooltip()
  end
end

local matPreview = ShapePreview()
local dimRdr = RectI(0, 0, 128, 128)
matPreview:setObjectModel("/art/shapes/material_preview/cube_1m.dae")
matPreview:setRenderState(false,false,false,false,false,false)
matPreview:setCamRotation(0.6, 3.9)
matPreview:renderWorld(dimRdr)
matPreview:fitToShape()
matPreview.mBgColor = ColorI(28,38,51,255)
local function matHovered(res)
  local mat = scenetree.findObject(res)
  local matPreviewSize = 128 * im.uiscale[0]
  if mat and mat.___type == "class<Material>" then
    matPreview:setMaterial(mat)
    im.BeginTooltip()
    dimRdr:set(0, 0, matPreviewSize, matPreviewSize)
    matPreview:renderWorld(dimRdr)
    matPreview:ImGui_Image(matPreviewSize, matPreviewSize)
    im.EndTooltip()
  end
end

local resExplorer

local function resourceContextMenu(res, count, type, filepath)
  if im.BeginPopup("Popup_" .. res..count) then
    local dir, basefilename, ext = path.splitWithoutExt(res)
    if basefilename and basefilename ~= "" then
      im.Text('['..basefilename..']')
    else
      im.Text('['..tostring(res)..']')
    end
    if im.Selectable1("Open in Explorer##"..res..count) then
      if type and type == 'material' or type == 'duplicates' or type == 'usedMat' then
        local o = scenetree.findObject(res)
        if o and o.getFileName then
          if FS:fileExists(o:getFileName()) then Engine.Platform.exploreFolder(o:getFileName()) else log('E', '', 'Path :'..tostring(o:getFileName())..' does not exist' ) end
        end
      else
        if not string.startswith(res, '/') then
          res = '/'..res
        end
        if FS:fileExists(res) then Engine.Platform.exploreFolder(res) else log('E', '', 'Path :'..tostring(res)..' does not exist' ) end
      end
      im.CloseCurrentPopup()
    end
    if string.lower(res):find(".json") == nil and string.lower(res):find(".cs") == nil then
      if im.Selectable1("Preview##"..res..count) then
      if not type then
        if not string.startswith(res, '/') then
          res = '/'..res
        end
        if FS:fileExists(res) then
          local cases = {".dds", ".png", ".bmp", ".jpg", ".jpeg", ".tga"}
          for _,b in pairs(cases) do
            if string.lower(res):find(b) then imageToPreview = res editor.showWindow(imageWindowName) end
          end
          local cases = {".dae", ".dts", ".cdae", ".cached.dts"}
          for _,b in pairs(cases) do
            if string.lower(res):find(b) then editor_shapeEditor.showShapeEditorLoadFile(res) end
          end
        else log('E', '', 'Path :'..tostring(res)..' does not exist' ) end
      end
      if type and type == 'material' or type == 'duplicates' or type == 'usedMat' then
        if editor_materialEditor then
          editor_materialEditor.showMaterialEditor()
          editor_materialEditor.selectMaterialByName(res, true)
        end
      end
      im.CloseCurrentPopup()
      end
    end
    if type and type == 'duplicates' then
      if im.Selectable1("Duplicates Resolver##"..res..count) then
        duplicateName = res
        duplicateTable = nil
        resourceUtil.duplicateData(res)
        editor.showWindow(duplicateResolverWindowName)
        im.CloseCurrentPopup()
      end
    end
    if type and type == 'usedMat' then
      if im.Selectable1("Save all usages data##"..res..count) then
        local path = filepath.."/resourceChecker_matData_"..res..".json"
        jsonWriteFile(path, resExplorer[6][res], true)
        if FS:fileExists(path) then Engine.Platform.exploreFolder(path) else log('E', '', 'Path :'..path..' does not exist' ) end
        im.CloseCurrentPopup()
      end
    end
    if not type or type ~= 'material' and type ~= 'duplicates' and type ~= 'usedMat' then
      if im.Selectable1("Copy path to clipboard##"..res..count) then
        if not string.startswith(res, '/') then
          res = '/'..res
        end
        if FS:fileExists(res) then im.SetClipboardText(res) else log('E', '', 'Path :'..tostring(res)..' does not exist' ) end
        im.CloseCurrentPopup()
      end
      if im.Selectable1("Dump asset##"..res..count) then
        if not string.startswith(res, '/') then
          res = '/'..res
        end
        if FS:fileExists(res) then dump(FS:stat(res)) else log('E', '', 'Path :'..tostring(res)..' does not exist' ) end
        im.CloseCurrentPopup()
      end
    end
    im.EndPopup()
  end
end

local isDoubleClicked = {}
local isSelected = {}
local function popUp(name)
  local win = im.GetMainViewport()
  pos.x = win.Pos.x + win.Size.x / 2
  pos.y = win.Pos.y + win.Size.y / 2

  im.SetNextWindowPos(pos, im.ImGuiCond_Always, im.ImVec2(0.5, 0))

  if im.BeginPopupModal(name, nil, im.WindowFlags_AlwaysAutoResize+im.WindowFlags_NoResize+im.WindowFlags_NoMove+im.WindowFlags_NoCollapse+im.WindowFlags_NoDocking+im.WindowFlags_NoTitleBar) then
    isDoubleClicked = {}
    local cancel
    if cancel == 1 then
      im.Text("Cancelling job")
    else
      im.Text("Checking files")
    end
    if getProgress() then
      im.ProgressBar(getProgress()/100, im.ImVec2(300* im.uiscale[0], 0))
      im.SameLine()
      if im.Button("Cancel") then
        resourceUtil.stopProgress()
        cancel = 1
      end
    end
    if getProgress() == 100 or getProgress() == nil then
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end
end

local btnpress = 3
local function warning(name, level, count, unusType, selected)
  local win = im.GetMainViewport()
  pos.x = win.Pos.x + win.Size.x / 2
  pos.y = win.Pos.y + win.Size.y / 2

  im.SetNextWindowPos(pos, im.ImGuiCond_Always, im.ImVec2(0.5, 0))

  if im.BeginPopupModal(name, nil, im.WindowFlags_AlwaysAutoResize+im.WindowFlags_NoResize+im.WindowFlags_NoMove+im.WindowFlags_NoCollapse+im.WindowFlags_NoDocking) then
    im.Text("This action will remove all files that were listed ("..count..").\nThis cannot be undone.\nMake sure to backup your work.\nThis feature only works for unpacked content.")
    popUp("Progress")
    if im.Button("Ok ("..btnpress..")") then
      if btnpress > 1 then
        btnpress = btnpress - 1
      else
        btnpress = 3
        im.OpenPopup("Progress")
        resExplorer = resourceUtil.removeUnused(level, unusType, selected)
        im.CloseCurrentPopup()
      end
    end
    im.SameLine()
    if im.Button("Cancel") then
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end
end

local searchTxt = im.ArrayChar(256, "")
local searchFilter = false
local skipK = {}

local itemsList = {}
local maxLines = 1000
local function resultUI(testName, filepath, text, data, isAdvanced, unusType, level, itms)
  isSelected = {}
  itemsList = data
  local windowSize = im.GetContentRegionAvail()
  im.Text(text)
  local textSize = im.CalcTextSize(text)
  im.Separator()
  if editor.uiInputSearch(nil, searchTxt, 400* im.uiscale[0]) then
    if ffi.string(searchTxt) ~= nil then
      searchFilter = true
    end
  end
  im.Separator()
  im.BeginChild1("##childResults", im.ImVec2(0, windowSize.y-textSize.y-(im.GetFontSize()*3.5)-30), false, im.WindowFlags_ChildWindow+im.WindowFlags_HorizontalScrollbar)
  local fontPushed = im.PushFont3("robotomono_regular")
  im.Spacing()
  if isAdvanced == 1 then
    local foundi = true
    local foundd = true
    local foundao = true
    local count = 0
    for k,v in pairs(data) do
      if count < maxLines then
        if k and v then
          if (foundi == false or foundd == false or foundao == false) and searchFilter == true then
            if not string.match(string.lower(k), string.lower(ffi.string(searchTxt))) then
              goto skipk
            end
          end
          count = count + 1
          if (count % 2 == 0) then
            drawRectBg(tostring(k), im.ImVec4(1, 1, 1, 0.06))
          end
          im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), k..": ")
          if im.IsItemHovered() then
            drawRectBg(tostring(k), im.ImVec4(0.2, 0.24, 0.31, 0.78), 1)
          end
          for i,h in pairs(v) do
            if count < maxLines then
              if i and h then
                if (foundd == false or foundao == false) and searchFilter == true then
                  if not string.match(string.lower(i), string.lower(ffi.string(searchTxt))) then
                    foundi = true
                    goto skipi
                  else
                    foundi = false
                  end
                end
                count = count + 1
                resourceContextMenu(i, count)
                if (count % 2 == 0) then
                  if isDoubleClicked[i] == true then
                    isSelected[i] = true
                    drawRectBg(tostring(i), im.ImVec4(0.8, 0.4, 0.1, 1))
                    im.TextColored(im.ImVec4(1, 1, 1, 1), '  '..i..": ")
                  else
                    drawRectBg(tostring(i), im.ImVec4(1, 1, 1, 0.06))
                    im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), '  '..i..": ")
                  end
                else
                  if isDoubleClicked[i] == true then
                    isSelected[i] = true
                    drawRectBg(tostring(i), im.ImVec4(0.8, 0.4, 0.1, 1))
                    im.TextColored(im.ImVec4(1, 1, 1, 1), '  '..i..": ")
                  else
                    drawRectBg(tostring(i), im.ImVec4(0, 0, 0, 0.1))
                    im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), '  '..i..": ")
                  end
                end
                if im.IsItemHovered() and im.IsMouseClicked(1) then
                  im.OpenPopup("Popup_" .. i..count)
                end
                if im.IsItemHovered() and im.IsMouseClicked(0) then
                  if not im.IsKeyDown(im.GetKeyIndex(im.Key_ModShift)) then
                    isDoubleClicked = {}
                  end
                  if isDoubleClicked[i] == true then
                    isDoubleClicked[i] = false
                  else
                    isDoubleClicked[i] = true
                  end
                elseif im.IsItemHovered() then
                  drawRectBg(tostring(i), im.ImVec4(0.2, 0.24, 0.31, 0.78), 1)
                  texHovered(i)
                end
                for d,g in pairs(h) do
                  if count < maxLines then
                    if d and g then
                      if foundao == false and searchFilter == true then
                        if not string.match(string.lower(d), string.lower(ffi.string(searchTxt))) then
                          foundd = true
                          goto skipd
                        else
                          foundd = false
                        end
                      end
                      count = count + 1
                      resourceContextMenu(d, count, 'material')
                      if (count % 2 == 0) then
                        if isDoubleClicked[d] == true then
                          isSelected[d] = true
                          drawRectBg(tostring(d), im.ImVec4(0.8, 0.4, 0.1, 1))
                          im.TextColored(im.ImVec4(1, 1, 1, 1), '    '..d..": ")
                        else
                          drawRectBg(tostring(d), im.ImVec4(1, 1, 1, 0.06))
                          im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), '    '..d..": ")
                        end
                      else
                        if isDoubleClicked[d] == true then
                          isSelected[d] = true
                          drawRectBg(tostring(d), im.ImVec4(0.8, 0.4, 0.1, 1))
                          im.TextColored(im.ImVec4(1, 1, 1, 1), '    '..d..": ")
                        else
                          drawRectBg(tostring(d), im.ImVec4(0, 0, 0, 0.1))
                          im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), '    '..d..": ")
                        end
                      end
                      if im.IsItemHovered() and im.IsMouseClicked(1) then
                        im.OpenPopup("Popup_" .. d..count)
                      end
                      if im.IsItemHovered() and im.IsMouseClicked(0) then
                        if not im.IsKeyDown(im.GetKeyIndex(im.Key_ModShift)) then
                          isDoubleClicked = {}
                        end
                        if isDoubleClicked[d] == true then
                          isDoubleClicked[d] = false
                        else
                          isDoubleClicked[d] = true
                        end
                      elseif im.IsItemHovered() then
                        drawRectBg(tostring(d), im.ImVec4(0.2, 0.24, 0.31, 0.78), 1)
                        matHovered(d)
                      end
                      for a,o in pairs(g) do
                        if count < maxLines then
                          if a and o then
                            if searchFilter == true then
                              if not string.match(string.lower(o), string.lower(ffi.string(searchTxt))) then
                                foundao = true
                                goto skipao
                              else
                                foundao = false
                              end
                            end
                            count = count + 1
                            local path = o:match("^(%S+)%s+(.+)")
                            if not string.startswith(path, '/') then
                              path = '/'..path
                            end
                            resourceContextMenu(path, count)
                            if (count % 2 == 0) then
                              if isDoubleClicked[o] == true then
                                isSelected[o] = true
                                drawRectBg(tostring(o), im.ImVec4(0.8, 0.4, 0.1, 1))
                                im.TextColored(im.ImVec4(1, 1, 1, 1), '      '..a..": "..o)
                              else
                                drawRectBg(tostring(o), im.ImVec4(1, 1, 1, 0.06))
                                im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), '      '..a..": "..o)
                              end
                            else
                              if isDoubleClicked[o] == true then
                                isSelected[o] = true
                                drawRectBg(tostring(o), im.ImVec4(0.8, 0.4, 0.1, 1))
                                im.TextColored(im.ImVec4(1, 1, 1, 1), '      '..a..": "..o)
                              else
                                drawRectBg(tostring(o), im.ImVec4(0, 0, 0, 0.1))
                                im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), '      '..a..": "..o)
                              end
                            end
                            if im.IsItemHovered() and im.IsMouseClicked(1) then
                              if path then
                                im.OpenPopup("Popup_" .. path..count)
                              end
                            end
                            if im.IsItemHovered() and im.IsMouseClicked(0) then
                              if not im.IsKeyDown(im.GetKeyIndex(im.Key_ModShift)) then
                                isDoubleClicked = {}
                              end
                              if isDoubleClicked[o] == true then
                                isDoubleClicked[o] = false
                              else
                                isDoubleClicked[o] = true
                              end
                            elseif im.IsItemHovered() then
                              drawRectBg(tostring(o), im.ImVec4(0.2, 0.24, 0.31, 0.78), 1)
                              texHovered(path)
                            end
                          end
                        end
                      end
                      ::skipao::
                    end
                  end
                end
                ::skipd::
              end
            end
          end
          ::skipi::
        end
      end
    end
    ::skipk::
    if count >= maxLines then
      im.TextColored(im.ImVec4(1, 1, 1, 1), "Can't display more than "..maxLines.." lines. Please check game log for more info.")
    end
  else
    im.Columns(2)
    im.SetColumnWidth(0, 60)
    im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), "No.")
    im.NextColumn()
    --im.PushItemWidth(200)
    im.Spacing()
    im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), "Issue")
    im.Spacing()
    im.Separator()
    im.Separator()
    im.Spacing()
    im.NextColumn()
    local count = 0
    for k,v in pairs(data) do
      if searchFilter == true and skipK[k] then
        goto skipEntr
      end
      count = count + 1
      if count < maxLines then
        if (count % 2 == 0) then
          if isDoubleClicked[k] == true then
            drawRectBg(tostring(k), im.ImVec4(0.8, 0.4, 0.1, 1))
            im.TextColored(im.ImVec4(1, 1, 1, 1), tostring(k))
          else
            drawRectBg(tostring(k), im.ImVec4(1, 1, 1, 0.06))
            im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), tostring(k))
          end
        else
          if isDoubleClicked[k] == true then
            drawRectBg(tostring(k), im.ImVec4(0.8, 0.4, 0.1, 1))
            im.TextColored(im.ImVec4(1, 1, 1, 1), tostring(k))
          else
            im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), tostring(k))
          end
        end
      end
      ::skipEntr::
    end
    skipK = {}
    im.NextColumn()
    local count = 0
    for k,v in pairs(data) do
      if searchFilter == true and not string.match(string.lower(v), string.lower(ffi.string(searchTxt))) then
        skipK[k] = true
        goto skipLine2
      end
      count = count + 1
      local path = nil
      if testName == "matVersion" or testName == "unusedMat" then
        local matName = tostring(v):match("^(%S+)%s+(.+)")
        resourceContextMenu(matName, count, 'material')
      elseif testName == "duplicates" then
        resourceContextMenu(v, count, 'duplicates')
      elseif testName == "usedMat" then
        local matName = tostring(v):match("^(%S+)%s+(.+)")
        resourceContextMenu(matName, count, 'usedMat', filepath)
      else
        path = tostring(v):match(".*. (.+)")
        if path and FS:fileExists(path) then
        elseif tostring(v) and string.match(tostring(v), " ") then
          path = tostring(v):match("^(%S+)%s+(.+)")
          if path and FS:fileExists(path) then end
        else
          path = tostring(v)
        end
        if not string.startswith(path, '/') then
          path = '/'..path
        end
        resourceContextMenu(path, count)
      end
      if count < maxLines then
        if (count % 2 == 0) then
          if isDoubleClicked[k] == true then
            isSelected[v] = true
            drawRectBg(tostring(v), im.ImVec4(0.8, 0.4, 0.1, 1))
            im.TextColored(im.ImVec4(1, 1, 1, 1), tostring(v))
          else
            drawRectBg(tostring(v), im.ImVec4(1, 1, 1, 0.06))
            im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), tostring(v))
          end
        else
          if isDoubleClicked[k] == true then
            isSelected[v] = true
            drawRectBg(tostring(v), im.ImVec4(0.8, 0.4, 0.1, 1))
            im.TextColored(im.ImVec4(1, 1, 1, 1), tostring(v))
          else
            drawRectBg(tostring(v), im.ImVec4(0, 0, 0, 0.1))
            im.TextColored(im.ImVec4(0.5, 0.9, 1, 1), tostring(v))
          end
        end
      end
      if im.IsItemHovered() and im.IsMouseClicked(1) then
        if testName == "matVersion" or testName == "unusedMat" or testName == "usedMat" then
          local matName = tostring(v):match("^(%S+)%s+(.+)")
          im.OpenPopup("Popup_" .. matName..count)
        elseif testName == "duplicates" then
          im.OpenPopup("Popup_" .. v..count)
        else
          im.OpenPopup("Popup_" .. path..count)
        end
      end
      if im.IsItemHovered() and im.IsMouseClicked(0) then
        if not im.IsKeyDown(im.GetKeyIndex(im.Key_ModShift)) then
          isDoubleClicked = {}
        end
        if isDoubleClicked[k] == true then
          isDoubleClicked[k] = false
        else
          isDoubleClicked[k] = true
        end
      elseif im.IsItemHovered() then
        drawRectBg(tostring(v), im.ImVec4(0.2, 0.24, 0.31, 0.78), 1)
        if testName == "matVersion" or testName == "unusedMat" or testName == "usedMat" then
          matHovered(tostring(v):match("^(%S+)%s+(.+)"))
        elseif testName == "duplicates" then
          matHovered(tostring(v))
        elseif testName == "tsstatics" or testName == "forestitems" or testName == "unusedMesh" or testName == "missingMat" or testName == "colData" then
          if FS:fileExists(tostring(v)) then
            shapeHovered(tostring(v))
          else
            shapeHovered(tostring(v):match("^(%S+)%s+(.+)"))
          end
        else
          texHovered(tostring(v))
        end
      end
      ::skipLine2::
    end
    if count >= maxLines then
      im.TextColored(im.ImVec4(1, 1, 1, 1), "Can't display more than "..maxLines.." lines. Please check game log for more info.")
    end
  end
  if fontPushed then
    im.PopFont()
    --im.SetWindowFontScale(1)
  end
  im.EndChild()
  im.Separator()
  im.Spacing()
  im.Dummy(im.ImVec2(5* im.uiscale[0], 0))
  im.SameLine()
  drawRectBg("LMB", im.ImVec4(0, 0, 0, 0.3), 0, 1)
  im.Text("LMB")
  im.SameLine()
  im.Text(" Select row")
  im.SameLine()
  im.Dummy(im.ImVec2(50* im.uiscale[0], 0))
  im.SameLine()
  drawRectBg("RMB", im.ImVec4(0, 0, 0, 0.3), 0, 1)
  im.Text("RMB")
  im.SameLine()
  im.Text(" Context Menu")
  if unusType ~= nil and level ~= nil then
    warning("Remove all unused files", level, itms, unusType)
    im.SameLine()
    local buttonWidth = 160* im.uiscale[0]
    local cnt = 0
    if not tableIsEmpty(isSelected) then
      for k,v in pairs(isSelected) do
        cnt = cnt + 1
      end
      warning("Remove only selected files", level, cnt, unusType, isSelected)
      im.Dummy(im.ImVec2(im.GetContentRegionAvailWidth()-buttonWidth*3-11* im.uiscale[0], 0))
    else
      im.Dummy(im.ImVec2(im.GetContentRegionAvailWidth()-buttonWidth-5* im.uiscale[0], 0))
    end
    im.SameLine()
    local removeButtonCol = im.ImVec4(1, 0, 0, 0.7)
    im.PushStyleColor2(im.Col_Button, removeButtonCol)
    if not tableIsEmpty(isSelected) then
      if im.Button("Remove selected files ("..cnt..")", im.ImVec2(buttonWidth,0)) then
        im.OpenPopup("Remove only selected files")
      end
      if im.IsItemHovered() then
        im.BeginTooltip()
        im.Text("Permanently removes selected files from unpacked level")
        im.EndTooltip()
      end
    end
    if not tableIsEmpty(isSelected) then
      im.SameLine()
      if im.Button("Invert selection" , im.ImVec2(buttonWidth,0)) then
        local tempSel = isSelected
        for k,v in pairs(itemsList) do
          if not tempSel[v] then
            isDoubleClicked[k] = true
          else
            isDoubleClicked[k] = nil
          end
        end
      end
    end
    im.SameLine()
    if im.Button("Remove all unused files", im.ImVec2(buttonWidth,0)) then
      im.OpenPopup("Remove all unused files")
    end
    im.PopStyleColor()
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Permanently removes unused files from unpacked level")
      im.EndTooltip()
    end
  end
  if filepath and testName then
    im.Dummy(im.ImVec2(im.GetContentRegionAvailWidth()-200* im.uiscale[0], 0))
    im.SameLine()
    if im.Button("Save output to userfolder", im.ImVec2(200* im.uiscale[0],0)) then
      local path = filepath.."/resourceChecker_"..testName..".json"
      jsonWriteFile(path, data, true)
      local addText = "//"..text:gsub('\n','').."\n"
      local f = io.open(path, "r")
      if f then
        for line in f:lines() do
          addText = addText..line..'\n'
        end
        f:close()
        local f = io.open(path, "w")
        f:write(addText)
        f:close()
      end
      if FS:fileExists(path) then Engine.Platform.exploreFolder(path) else log('E', '', 'Path :'..path..' does not exist' ) end
    end
  end
end

local btnText = "error"
local useVeh = false
local skipCommon = true

local function matTab()
  local convertdata
  local getlevel = getCurrentLevelIdentifier()
  if not getlevel then
    im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Level is not loaded")
  else
    local playerVehiclePath = getPlayerVehiclePath()
    if playerVehiclePath then
      if useVeh == true then btnText = "Check current level" else btnText = "Check current vehicle" end
      if im.Button(btnText, im.ImVec2(220* im.uiscale[0],0)) then
        if useVeh == false then useVeh = true else useVeh = false end
      end
    else
      useVeh = false
    end
    local matdata = "/levels/"..getlevel.."/"
    if useVeh == true then matdata = playerVehiclePath end
    im.Text("Checking: "..matdata)
    im.SameLine()
    im.Dummy(im.ImVec2(im.GetContentRegionAvailWidth()-180* im.uiscale[0], 0))
    im.SameLine()
    local skipCommonEnabled = im.BoolPtr(skipCommon)
    if im.Checkbox("Skip common folders", skipCommonEnabled) then
      skipCommon = skipCommonEnabled[0]
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Skips common folders for duplicate checking")
      im.EndTooltip()
    end
    popUp("Progress")
    im.Text("Select one of these tools to generate information about materials loaded by this level.")
    if im.Button("Check materials version", im.ImVec2(152* im.uiscale[0],0)) then
      im.OpenPopup("Progress")
      convertdata = matdata
      verifier = resourceUtil.verifyVersion(convertdata)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Checks for a materials version")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Verify duplicates", im.ImVec2(120* im.uiscale[0],0)) then
      im.OpenPopup("Progress")
      convertdata = matdata
      verifier = resourceUtil.verifyDuplicate(convertdata, skipCommon)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Checks for duplicated materials")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Remove pid", im.ImVec2(121* im.uiscale[0],0)) then
      im.OpenPopup("Progress")
      convertdata = matdata
      verifier = resourceUtil.fixPID(convertdata, skipCommon)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Removes deprecated Persistent Ids from materials")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Convert to PNG", im.ImVec2(121* im.uiscale[0],0)) then
      editor_fileDialog.openFile(
        function(data)
          im.OpenPopup("Progress")
          convertdata = matdata
          verifier = resourceUtil.textureExporter(convertdata, data.path)
        end,
        {{"All Folders","*"}},
        true,
        '/'
      )
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Converts old DDS files to PNG")
      im.EndTooltip()
    end
    if im.Button("Check texture map", im.ImVec2(131* im.uiscale[0],0)) then
      im.OpenPopup("Progress")
      convertdata = matdata
      verifier = resourceUtil.checkMatTex(convertdata)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Validates texture mappings in materials")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Check texture files", im.ImVec2(131* im.uiscale[0],0)) then
      im.OpenPopup("Progress")
      convertdata = matdata
      verifier = resourceUtil.checkTex(convertdata)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Checks materials file for texture issues")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Check missing mats", im.ImVec2(131* im.uiscale[0],0)) then
      im.OpenPopup("Progress")
      convertdata = matdata
      verifier = resourceUtil.checkmissingMats(convertdata)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Checks for missing materials mapping in currently loaded models")
      im.EndTooltip()
    end
    if useVeh == true then
      im.SameLine()
      if im.Button("Remove dummy mats", im.ImVec2(140* im.uiscale[0],0)) then
        im.OpenPopup("Progress")
        convertdata = matdata
        verifier = resourceUtil.removeDummy(convertdata, skipCommon)
      end
      if im.IsItemHovered() then
        im.BeginTooltip()
        im.Text("Removes all dummy materials from a vehicle")
        im.EndTooltip()
      end
    end
    im.Spacing()
    im.Separator()
    im.Spacing()
    if not verifier and getProgress() == nil then
      im.Text("Select a verifier above to see results!")
    elseif not verifier and getProgress() ~= nil then
      im.Text("Working...")
    elseif verifier and verifier[1] == 2 and verifier[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Verifying done")
      im.Spacing()
      resultUI("matVersion", matdata, "Detected ("..verifier[2]..") V0 materials in : ", verifier[3])
    elseif verifier and verifier[1] == 3 and verifier[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Verifying done")
      resultUI("duplicates", matdata, "Detected duplicated materials ("..verifier[2].."),\nit's required to avoid doing that as it can lead into issues.", verifier[4])
    elseif verifier and verifier[1] == 5 and verifier[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Finished removing persistentIds")
      resultUI("persistentid", matdata, "Removed ("..verifier[2]..") persistentIds in files: ", verifier[4])
    elseif verifier and verifier[1] == 6 and verifier[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Finished checking texture maps\nFound: "..verifier[2]+verifier[3]+verifier[6].." issues")
      resultUI("textureMap", matdata, "There is ("..verifier[2]..") issues with wrong path to files.\nDetected ("..verifier[3]..") missing textures in materials.\nFound ("..verifier[6]..") issues with Texture Cooker files.\nPlease use .png file extension in Texture Cooker materials to make sure everything works correctly.\n*Numbers next to map name correspond to material layer.", verifier[4], 1)
    elseif verifier and verifier[1] == 7 and verifier[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Finished checking texture files\nFound: "..verifier[2]+verifier[3]+verifier[6].." issues")
      resultUI("textureFiles", matdata, "There is ("..verifier[2]..") issues with incorrect file format.\nDetected ("..verifier[3]..") textures that are not power of 2.\nFound ("..verifier[6]..") textures that cannot be cooked.\n*Numbers next to map name correspond to material layer.", verifier[4], 1)
    elseif verifier and verifier[1] == 8 and verifier[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Verifying done")
      resultUI("missingMat", matdata, "Detected ("..verifier[2]..") missing materials: ", verifier[4])
    elseif verifier and verifier[1] == 9 and verifier[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Removed dummy mats")
      resultUI("remDummy", matdata, "Removed ("..verifier[2]..") dummy materials: ", verifier[4])
    elseif verifier and verifier[1] == 10 and verifier[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Exported textures")
      resultUI("textureExporter", matdata, "Exported ("..verifier[2]..") textures: ", verifier[4])
    elseif verifier and verifier[5] == 2 then
      im.TextColored(im.ImVec4(1, 0, 0, 1), "Verification failed")
    end
  end
end

local function resTab()
  local getlevel = getCurrentLevelIdentifier()
  if not getlevel then
    im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Level is not loaded")
  else
    im.Text("Checking: ".."/levels/"..getlevel.."/")
    popUp("Progress")
    im.Text("Select one of these tools to generate information about assets loaded by this level.")
    local buttonSize = 140* im.uiscale[0]
    if im.Button("Loaded TSStatics", im.ImVec2(buttonSize,0)) then
      im.OpenPopup("Progress")
      resExplorer = resourceUtil.checkStatic()
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Generates a list of currently loaded TSStatics")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Available ForestItems", im.ImVec2(buttonSize,0)) then
      im.OpenPopup("Progress")
      resExplorer = resourceUtil.checkForest()
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Generates a list of available Forest Meshes")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Loaded Terrains", im.ImVec2(buttonSize,0)) then
      im.OpenPopup("Progress")
      resExplorer = resourceUtil.checkTerrains()
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Generates a list of used terrains")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Used Materials", im.ImVec2(buttonSize,0)) then
      im.OpenPopup("Progress")
      resExplorer = resourceUtil.checkUsedMats(getlevel)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Generates a list of used materials and its data")
      im.EndTooltip()
    end
    if im.Button("Unused Materials", im.ImVec2(buttonSize,0)) then
      im.OpenPopup("Progress")
      resExplorer = resourceUtil.checkUnusedMats(getlevel)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Generates a list of unused materials")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Unused Meshes", im.ImVec2(buttonSize,0)) then
      im.OpenPopup("Progress")
      resExplorer = resourceUtil.checkUnusedModels(getlevel)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Generates a list of unused meshes")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Unused Textures", im.ImVec2(buttonSize,0)) then
      im.OpenPopup("Progress")
      resExplorer = resourceUtil.unusedTextures(getlevel)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Generates a list of unused textures")
      im.EndTooltip()
    end
    im.SameLine()
    if im.Button("Collision Data", im.ImVec2(buttonSize,0)) then
      im.OpenPopup("Progress")
      resExplorer = resourceUtil.checkColData(getlevel)
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      im.Text("Generates a list with data about collisions in the level")
      im.EndTooltip()
    end
    im.Spacing()
    im.Separator()
    im.Spacing()
    if not resExplorer and getProgress() == nil then
      im.Text("Select a verifier above to see results!")
    elseif not resExplorer and getProgress() ~= nil then
      im.Text("Working...")
    elseif resExplorer and resExplorer[1] == 1 and resExplorer[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Task complete")
      resultUI("tsstatics", "/levels/"..getlevel.."/", "There is currently ("..resExplorer[2]..") TSStatics loaded in scene with ("..resExplorer[3].. ") instances.\nTotal Collada size: "..resExplorer[6].." MB\nTotal cache size: "..resExplorer[7].." MB", resExplorer[4])
    elseif resExplorer and resExplorer[1] == 2 and resExplorer[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Task complete")
      resultUI("forestitems", "/levels/"..getlevel.."/", "There is currently ("..resExplorer[2]..") ForestItems available in the level.\nTotal Collada size: "..resExplorer[6].." MB\nTotal cache size: "..resExplorer[7].." MB", resExplorer[4])
    elseif resExplorer and resExplorer[1] == 3 and resExplorer[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Task complete")
      resultUI("terrains", "/levels/"..getlevel.."/", "There is currently ("..resExplorer[2]..") TerrainBlocks loaded in scene that have a total size of "..resExplorer[3].." MB: ", resExplorer[4])
    elseif resExplorer and resExplorer[1] == 4 and resExplorer[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Task complete")
      resultUI("unusedMat", "/levels/"..getlevel.."/", "There is currently ("..resExplorer[2]..") unused Materials in the level: ", resExplorer[4], 0, 1, getlevel, resExplorer[2])
    elseif resExplorer and resExplorer[1] == 5 and resExplorer[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Task complete")
      resultUI("unusedMesh", "/levels/"..getlevel.."/", "There is currently ("..resExplorer[2]..") unused Meshes in the level that have a total size of "..resExplorer[3].." MB: ", resExplorer[4], 0, 2, getlevel, resExplorer[2])
    elseif resExplorer and resExplorer[1] == 6 and resExplorer[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Task complete")
      resultUI("unusedTex", "/levels/"..getlevel.."/", "There is currently ("..resExplorer[2]..") unused textures in the level that have a total size of "..resExplorer[3].." MB: ", resExplorer[4], 0, 3, getlevel, resExplorer[2])
    elseif resExplorer and resExplorer[1] == 7 and resExplorer[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Task complete, removed: ("..resExplorer[2]..") files with total size of "..resExplorer[3].." MB")
    elseif resExplorer and resExplorer[1] == 8 and resExplorer[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Task complete")
      resultUI("usedMat", "/levels/"..getlevel.."/", "There is currently ("..resExplorer[2]..") used Materials in the level that have a total size of "..resExplorer[3].." MB: ", resExplorer[4], 0, nil, getlevel, resExplorer[2])
    elseif resExplorer and resExplorer[1] == 9 and resExplorer[5] == 1 then
      im.TextColored(im.ImVec4(1, 1, 0.2, 1), "Task complete")
      resultUI("colData", "/levels/"..getlevel.."/", "There is currently ("..resExplorer[2]..") used static collision meshes in the level that have a total of "..resExplorer[3][1].." polygons.\nThere is ("..resExplorer[3][3]..") static meshes instances that use Visible Mesh Collisions and have total of "..resExplorer[3][2].." polygons: ", resExplorer[4], 0, nil, getlevel, resExplorer[2])
    elseif resExplorer and resExplorer[5] == 2 then
      im.TextColored(im.ImVec4(1, 0, 0, 1), "Task failed")
    end
  end
end

local function onEditorGui()
  if editor.beginWindow(toolWindowName, "Resources Checker") then
    im.Text("Check resources of current level")
    im.Spacing()

    if im.BeginTabBar("tabs") then
      if im.BeginTabItem("Materials Verification", nil, im.TabItemFlags_None) then
        matTab()
        im.EndTabItem()
      end
      if im.BeginTabItem("Resources Explorer", nil, im.TabItemFlags_None) then
        resTab()
        im.EndTabItem()
      end
    end
    --im.Spacing()
  end
  imagePreview()
  duplicateResolver()
  editor.endWindow()
end

local function jobData(type, data)
  if type == 2 then
    verifier = data
  end
  if type == 3 then
    resExplorer = data
  end
end

local function updateDuplicateTable(data)
  duplicateTable = data
end

local function onEditorActivated()

end

local function onEditorDeactivated()

end

local function onEditorInitialized()
  editor.addWindowMenuItem("Resources Checker", onWindowMenuItem)
  editor.registerWindow(toolWindowName, im.ImVec2(500, 300))
  editor.registerWindow(imageWindowName, im.ImVec2(256, 256))
  editor.registerWindow(duplicateResolverWindowName, im.ImVec2(500, 300))
end

M.jobData = jobData
M.updateDuplicateTable = updateDuplicateTable
M.onEditorGui = onEditorGui
M.onWindowMenuItem = onWindowMenuItem
M.onEditorInitialized = onEditorInitialized
M.onEditorActivated = onEditorActivated
M.onEditorDeactivated = onEditorDeactivated

return M