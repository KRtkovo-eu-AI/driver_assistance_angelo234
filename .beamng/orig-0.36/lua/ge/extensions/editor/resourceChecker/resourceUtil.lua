-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local objects = nil

local luaType = type
local im = ui_imgui
local ffi = require("ffi")

local function onExtensionUnloaded()
  extensions.unload('extensions.editor_resourceChecker_resourceUtil')
end

--get scene tree all objects
local function getSimObjects(fileName)
  local ret = {}
  local objs = scenetree.getAllObjects()
  --log('E', '', '# objects existing: ' .. tostring(#scenetree.getAllObjects()))
  for _, objName in ipairs(objs) do
    local o = scenetree.findObject(objName)
    if o and o.getFileName then
      if o:getFileName() == fileName then
        table.insert(ret, o)
      end
    end
  end
  return ret
  --log('E', '', '# objects left: ' .. tostring(#scenetree.getAllObjects()))
end

local function resaveMaterial(file)
  if file and FS:fileExists(file) then
    local persistenceMgr = PersistenceManager()
    persistenceMgr:registerObject('matFixOrder_PersistMan')
    loadJsonMaterialsFile(file)
    local objects = getSimObjects(file)
    if not tableIsEmpty(objects) then
      for _, obj in ipairs(objects) do
        if obj.___type == "class<Material>" then
          obj.persistentId = ""
          persistenceMgr:setDirty(obj, '')
        end
      end
      persistenceMgr:saveDirty()
      --for _, obj in ipairs(objects) do
        --obj:delete()
      --end
    end
    persistenceMgr:delete()
  end
end

--check of pow2
local function powerOfTwo(x)
  return((math.log(x)/math.log(2)) % 1 == 0)
end

local function removeFromForestJson(shape, foresData)
  local forestContent = jsonReadFile(foresData)
  if forestContent then
    local forestItem
    for k,v in pairs(forestContent) do
      if v.shapeFile == shape then
        forestItem = k
      end
    end
    if forestItem then
      log('I', '', 'Removing unused forestItem '..forestItem )
      forestContent[forestItem] = nil
      jsonWriteFile(foresData, forestContent, true)
    else
      log('W', '', 'Could not find '..shape )
    end
  end
end

--get material layers fields
local function getMaterialTexFields(mat)
  local fields = {}
  if mat and mat.___type == "class<Material>" then
    local layers = 1
    local version = mat:getField("version",0)
    if version == "0" or version == "1" then
      layers = 4
    elseif version == "1.5" then
      layers = mat:getField("activeLayers",0)
    end
    local layer = 0
    for i=1, layers do
      for k,v in pairs(mat:getFields()) do
        if v["type"] == "filename" then
          local filepth = mat:getField(k,layer)
          if filepth and filepth ~= "" then
            fields[k.."."..layer] = filepth
          end
        end
      end
      layer = layer + 1
    end
    return fields
  else
    log('E', '', 'Material not found' )
  end
end

local duplicatedM = {}

--look for duplicates
local function findDuplicates(duplicatelist)
  local seen1 = {}
  local seen1file = {}
  local seen2 = {}
  local seen2file = {}
  for k,v in pairs(duplicatelist) do
    if seen1[v[1]] then
      duplicatedM[v[1]] = true
    else
      seen1[v[1]] = true
      seen1file[v[1]] = v[3]
    end
    if v[2] and v[2] ~= "unmapped_mat" then
      if seen2[v[2]] then
        duplicatedM[v[1]] = true
      else
        seen2[v[2]] = true
        seen2file[v[2]] = v[3]
      end
    end
  end
end

--materials verifiers
local verifyVersionworkJob

local function verifyVersionwork(job, convertdata)
  local isDone
  local verifydata = convertdata
  local count0 = 0
  local countPBR = 0
  local type = 2
  local isOld = {}
  local checkedMats = {}
  local output = {}
  job.progress = 0
  job.stop = nil

  if not verifydata then
    log('E', '', 'There is no material path' )
    isDone = 2
  elseif not string.match(verifydata, "/") then
    log('E', '', 'Incorrect path' )
    isDone = 2
  else
    log('I', '', 'Verifying materials version' )

    local materialFiles = FS:findFiles(verifydata, "*.cs\t*materials.json", -1, true, false)

    job.progress = 5
    job.sleep(0.001)

    log('D', '', dumps(materialFiles))

    for _, fn in ipairs(materialFiles) do
      if job.stop == true then
        do return end
      end
      local dir, basefilename, ext = path.splitWithoutExt(fn)

      if string.find(fn, 'materials.cs$') then
        job.yield()
        TorqueScript.exec(fn)
        objects = extensions.editor_resourceChecker_resourceUtil.getSimObjects(fn)
      elseif string.find(fn, 'materials.json$') then
        job.yield()
        loadJsonMaterialsFile(fn)
        objects = extensions.editor_resourceChecker_resourceUtil.getSimObjects(fn)
      end
      if not tableIsEmpty(objects) then
        log('I', '', 'parsing all materials file: ' .. tostring(fn))

        for _, obj in ipairs(objects) do
          if job.stop == true then
            do return end
          end
          -- the old material files can also contain other stuff ...
          if job.progress < 75 then
            job.progress = job.progress + 0.01
          end
          if obj.___type == "class<Material>" then
            job.yield()
            log('I', '', ' * ' .. tostring(obj:getClassName()) .. ' - ' .. tostring(obj:getName()) .. ' - version: ' .. tostring(obj:getField('version', 0)) )
            local version = tonumber(obj:getField('version', 0))
            --PBR check
            if version and version < 1.5 then
              job.yield()
              if not checkedMats[obj:getName()] == true then
                checkedMats[obj:getName()] = true
                count0 = count0 + 1
              end
              isOld[obj:getName()] = obj:getFileName()
            elseif version == 1.5 then
              job.yield()
              if not checkedMats[obj:getName()] == true then
                checkedMats[obj:getName()] = true
                countPBR = countPBR + 1
              end
            end
          end
        end
      end
    end
    job.sleep(0.001)
    job.progress = 75
    for k,v in pairs(isOld) do
      job.yield()
      table.insert(output, k.."  in: "..v)
    end
    table.sort(output, function(a,b) return string.upper(a) < string.upper(b) end)
    job.sleep(0.001)
    log('I', '', 'Found ' ..tostring(count0).. ' old materials' )
    log('I', '', 'Found ' ..tostring(countPBR).. ' PBR materials' )
    isDone = 1
    job.progress = 100
  end
  local data = {type, count0, output, countPBR, isDone}
  extensions.editor_resourceChecker.jobData(2, data)
end

local function verifyVersion(convertdata)
  verifyVersionworkJob = extensions.core_jobsystem.create(verifyVersionwork, 1, convertdata)
end

local verifyDuplicateworkJob

local function verifyDuplicatework(job, convertdata, skipCommon)
  local isDone
  local verifydata = convertdata
  local countduplicate = 0
  local duplicated = {}
  local type = 3
  local matTable = {}
  duplicatedM = {}
  job.progress = 0
  job.stop = nil

  if not verifydata then
    log('E', '', 'There is no material path' )
    isDone = 2
  elseif not string.match(verifydata, "/") then
    log('E', '', 'Incorrect path' )
    isDone = 2
  else
    log('I', '', 'Verifying materials duplicates' )

    local duplicatelist = {}

    --V2, shortcode much more efficient, checks all types of files at once
    --we have to check for common art too...
    local materialFiles = FS:findFiles(verifydata, "*.cs\t*materials.json", -1, true, false)
    if skipCommon == false then
      local commonVeh = FS:findFiles("/vehicles/common", "*.cs\t*materials.json", -1, true, false)
      local commonArt = FS:findFiles("/art", "*.cs\t*materials.json", -1, true, false)
      local commonCore = FS:findFiles("/core", "*.cs\t*materials.json", -1, true, false)
      for k,v in pairs(commonVeh) do
        table.insert(materialFiles, v)
      end
      for k,v in pairs(commonArt) do
        table.insert(materialFiles, v)
      end
      for k,v in pairs(commonCore) do
        table.insert(materialFiles, v)
      end
    end
    job.sleep(0.001)
    job.progress = 10
    for _, fn in ipairs(materialFiles) do
      if job.stop == true then
        do return end
      end
      local dir, basefilename, ext = path.splitWithoutExt(fn)
      if FS:fileSize(fn) > 0 then
        if string.find(fn, 'materials.cs$') then
          log('I', '', 'Loading cs material file '..fn )
          local f = io.open(fn, "r")
          if f then
            matTable[fn] = {}
            local titleS = nil
            for line in f:lines() do
              local title = line:match('%b()')
              local key = line:match("(.+)=(.+)")
              local value = line:match('%b""')
              if title then
                title = title:gsub('%(', '')
                title = title:gsub('%)', '')
                --print("title "..title)
                matTable[fn][title] = {}
                matTable[fn][title].name = title
                titleS = title
              end
              if key then
                key = key:gsub(' ', "")
                if value then
                  value = value:gsub('"', "")
                  --print("val  "..value)
                  matTable[fn][titleS][key] = value
                end
              end
            end
            f:close()
          end
        elseif string.find(fn, 'materials.json$') then
          log('I', '', 'Loading json material file '..fn )
          matTable[fn] = jsonReadFile(fn) or {}
        end
        job.yield()
      end
    end
    --dump(matTable)
    if not tableIsEmpty(matTable) then
      log('I', '', 'parsing all materials')
      if job.stop == true then
        do return end
      end
      for k,v in pairs(matTable) do
        local path = k
        for k,v in pairs(v) do
          local mat = v
          if mat and mat.name then
            local matname = mat.name
            log('I', '', ' * ' .. tostring(matname) .. ' - mapTo: ' .. tostring(mat.mapTo) )
            local matID = math.random(0, 100000000)
            if duplicatelist[matID] then matID = math.random(0, 100000000) end
            duplicatelist[matID] = {matname, mat.mapTo, path}
            if job.progress < 50 then
              job.progress = job.progress + 0.01
            end
          elseif mat and not mat.name then
            log('W', '', 'Corrupted or incompatible material found '..k)
          end
          job.yield()
        end
      end
    end
    job.progress = 50
    job.sleep(0.001)
    extensions.editor_resourceChecker_resourceUtil.findDuplicates(duplicatelist)
    job.sleep(0.001)
    if job.stop == true then
      do return end
    end

    job.progress = 90
    job.sleep(0.001)
    for k,v in pairs(duplicatedM) do
      countduplicate = countduplicate + 1
      table.insert(duplicated, k)
      --duplicated["Duplicated Mapping"][k] = v
    end
    table.sort(duplicated, function(a,b) return string.upper(a) < string.upper(b) end)
    log('I', '', 'Found ' ..tostring(countduplicate).. ' duplicates' )
    job.sleep(0.001)
    isDone = 1
    job.progress = 100
  end
  local data = {type, countduplicate, "dummy", duplicated, isDone}
  extensions.editor_resourceChecker.jobData(2, data)
end

local function verifyDuplicate(convertdata, skipCommon)
  verifyDuplicateworkJob = extensions.core_jobsystem.create(verifyDuplicatework, 1, convertdata, skipCommon)
end

local fixPIDworkJob

local function fixPIDwork(job, convertdata, skipCommon)
  local isDone
  local verifydata = convertdata
  local type = 5
  local matTable = {}
  local editedFiles = {}
  local outdatedFiles = {}
  local count = 0
  job.stop = nil
  job.progress = 0
  job.sleep(0.001)

  if not verifydata then
    log('E', '', 'There is no material path' )
    isDone = 2
  elseif not string.match(verifydata, "/") then
    log('E', '', 'Incorrect path' )
    isDone = 2
  else
    log('I', '', 'Removing PID' )

    --V2, shortcode much more efficient, checks all types of files at once
    --we have to check for common art too...
    local materialFiles = FS:findFiles(verifydata, "*materials.json", -1, true, false)
    if skipCommon == false then
      local commonArt = FS:findFiles("/art", "*materials.json", -1, true, false)
      local commonCore = FS:findFiles("/core", "*materials.json", -1, true, false)
      if string.match(verifydata, "vehicles/") then
        local commonVeh = FS:findFiles("/vehicles/common", "*materials.json", -1, true, false)
        for k,v in pairs(commonVeh) do
          table.insert(materialFiles, v)
        end
      end
      for k,v in pairs(commonArt) do
        table.insert(materialFiles, v)
      end
      for k,v in pairs(commonCore) do
        table.insert(materialFiles, v)
      end
    end

    job.progress = 10
    job.sleep(0.001)
    if job.stop == true then
      do return end
    end

    for _, fn in ipairs(materialFiles) do
      local dir, basefilename, ext = path.splitWithoutExt(fn)
      if FS:fileSize(fn) > 0 then
        log('I', '', 'Loading json material file '..fn )
        matTable[fn] = jsonReadFile(fn) or {}
        if job.stop == true then
          do return end
        end
      end
      job.yield()
    end
    job.sleep(0.001)
    job.progress = 20
    --dump(matTable)
    if not tableIsEmpty(matTable) then
      log('I', '', 'parsing all materials')
      for k,v in pairs(matTable) do
        local path = k
        for k,v in pairs(v) do
          if job.stop == true then
            do return end
          end
          local mat = v
          if mat and mat.persistentId then
            job.yield()
            outdatedFiles[path] = true
            count = count + 1
          elseif mat and not mat.persistentId then
            log('W', '', 'Corrupted or incompatible material found '..k)
          end
        end
      end
    end

    job.progress = 65
    job.sleep(0.001)
    for k,v in pairs(outdatedFiles) do
      log('I', '', 'Saved materials to '..k )
      resaveMaterial(k)
      table.insert(editedFiles, k)
    end
    table.sort(editedFiles, function(a,b) return string.upper(a) < string.upper(b) end)
    log('I', '', 'Removed '..count..' persistendIds' )
    job.progress = 100
    job.sleep(0.001)
    isDone = 1
  end
  local data = {type, count, "", editedFiles, isDone}
  extensions.editor_resourceChecker.jobData(2, data)
end

local function fixPID(convertdata, skipCommon)
  fixPIDworkJob = extensions.core_jobsystem.create(fixPIDwork, 1, convertdata, skipCommon)
end

local checkMatTexworkJob

local function checkMatTexwork(job, convertdata)
  local isDone
  local verifydata = convertdata
  local type = 6
  local fileIsMissing = {}
  local countmissing = 0
  local incorrectPath = {}
  local countpath = 0
  local incorrectPathCooker = {}
  local countcooker = 0
  local issuesTab = {}
  local matData = {}
  job.progress = 0
  job.sleep(0.001)
  job.stop = nil
  if not verifydata then
    log('E', '', 'There is no material path' )
    isDone = 2
  elseif not string.match(verifydata, "/") then
    log('E', '', 'Incorrect path' )
    isDone = 2
  else
    log('I', '', 'Checking texture maps' )

    --V2, shortcode much more efficient, checks all types of files at once
    local materialFiles = FS:findFiles(verifydata, "*.cs\t*materials.json", -1, true, false)

    job.progress = 10
    job.sleep(0.001)
    if job.stop == true then
      print("STOPPING")
      do return end
    end

    for _, fn in ipairs(materialFiles) do
      if job.stop == true then
        print("STOPPING")
        do return end
      end
      job.yield()
      local dir, basefilename, ext = path.splitWithoutExt(fn)
      matData[fn] = {}

      if string.find(fn, 'materials.cs$') then
        TorqueScript.exec(fn)
        objects = extensions.editor_resourceChecker_resourceUtil.getSimObjects(fn)
      elseif string.find(fn, 'materials.json$') then
        loadJsonMaterialsFile(fn)
        objects = extensions.editor_resourceChecker_resourceUtil.getSimObjects(fn)
      end

      if not tableIsEmpty(objects) then
        log('I', '', 'parsing all materials file: ' .. tostring(fn))
        job.yield()
        for _, obj in ipairs(objects) do
          if job.stop == true then
            print("STOPPING")
            do return end
          end
          job.yield()
          -- the old material files can also contain other stuff ...
          if obj.___type == "class<Material>" then
            local texfields = extensions.editor_resourceChecker_resourceUtil.getMaterialTexFields(obj)
            if texfields then
              matData[fn][obj:getName()] = {}
              for k,v in pairs(texfields) do
                matData[fn][obj:getName()][k] = v
              end
            end
          end

          if obj.___type == "class<TerrainMaterial>" then
            local texfields = {}
            if texfields then
              for k,v in pairs(obj:getFields()) do
                if v["type"] == "filename" then
                  texfields[k] = obj:getField(k,0)
                end
              end
              matData[fn][obj:getName()] = {}
              for k,v in pairs(texfields) do
                matData[fn][obj:getName()][k] = v
              end
            end
          end
        end
      end

      local cases = {".color.png", ".normal.png", ".data.png", ".color.dds", ".normal.dds", ".data.dds", ".dds", ".png", ".bmp", ".jpg", ".jpeg", ".tga"}
      for e,t in pairs(matData) do
        fileIsMissing[e] = {}
        incorrectPath[e] = {}
        incorrectPathCooker[e] = {}
        for k,v in pairs(t) do
          if job.progress < 75 then
            job.progress = job.progress + 0.001
          end
          fileIsMissing[e][k] = {}
          incorrectPath[e][k] = {}
          incorrectPathCooker[e][k] = {}
          for m,d in pairs(v) do
            job.yield()
            local dir, basefilename, ext = path.splitWithoutExt(d)
            if d ~= "" and d ~= nil then
              for _,b in pairs(cases) do
                if d:find(b) then
                  if d:find(".color.png") or d:find(".data.png") or d:find(".normal.png") then
                    if FS:fileExists(dir..basefilename..".png") or FS:fileExists(dir..basefilename..".dds") then
                    else fileIsMissing[e][k][m] = d.."   Reason: File not found" end
                  elseif dir then
                    if not FS:fileExists(dir..basefilename..b) then fileIsMissing[e][k][m] = d.."   Reason: File not found" end
                  end
                  if d:find(".color.dds") or d:find(".data.dds") or d:find(".normal.dds") then
                    if FS:fileExists(dir..basefilename..".png") or FS:fileExists(dir..basefilename..".dds") then
                      incorrectPathCooker[e][k][m] = d.."   Reason: cannot be cooked, wrong postfix, use png in texture cooker files"
                    else fileIsMissing[e][k][m] = d.."   Reason: File not found" end
                  end
                  if d:find(".color") or d:find(".data") or d:find(".normal") then
                    if not dir then incorrectPath[e][k][m] = d.."   Reason: Path does not contain directory" end
                    if not ext then incorrectPath[e][k][m] = d.."   Reason: Path does not contain extension" end
                  end
                  if dir then
                    if not string.lower(dir):find(string.lower(verifydata):gsub('/levels/','levels/'):gsub('/vehicles/','vehicles/')) then
                      if dir:find("levels/") then
                        incorrectPath[e][k][m] = d.."   Reason: Path leads to a different level, might cause issues"
                      elseif dir:find("vehicles/") and not dir:find("vehicles/common/") then
                        incorrectPath[e][k][m] = d.."   Reason: Path leads to a different vehicle, might cause issues"
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
    end
    if job.stop == true then
      print("STOPPING")
      do return end
    end
    local tempTable = {}
    job.progress = 75
    job.sleep(0.001)
    for k,v in pairs(incorrectPathCooker) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k] = {}
            countcooker = countcooker + 1
          end
        end
      end
    end
    for k,v in pairs(incorrectPathCooker) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k][e] = b
          end
        end
      end
    end
    incorrectPathCooker = tempTable
    local tempTable = {}
    for k,v in pairs(incorrectPath) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k] = {}
            countpath = countpath + 1
          end
        end
      end
    end
    for k,v in pairs(incorrectPath) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k][e] = b
          end
        end
      end
    end
    incorrectPath = tempTable
    local tempTable = {}
    for k,v in pairs(fileIsMissing) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k] = {}
            countmissing = countmissing + 1
          end
        end
      end
    end
    for k,v in pairs(fileIsMissing) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k][e] = b
          end
        end
      end
    end
    job.progress = 90
    job.sleep(0.001)
    fileIsMissing = tempTable
    issuesTab["Incorrect Path for Texture Cooker"] = {}
    issuesTab["Incorrect Path"] = {}
    issuesTab["Missing File"] = {}
    for k,v in pairs(incorrectPathCooker) do
      issuesTab["Incorrect Path for Texture Cooker"][k] = v
    end
    for k,v in pairs(incorrectPath) do
      issuesTab["Incorrect Path"][k] = v
    end
    for k,v in pairs(fileIsMissing) do
      issuesTab["Missing File"][k] = v
    end
    job.progress = 100
    job.sleep(0.001)
    log('I', '', 'Checked all materials textures mapping' )
    isDone = 1
  end

  local data = {type, countpath, countmissing, issuesTab, isDone, countcooker}
  extensions.editor_resourceChecker.jobData(2, data)
end

local function checkMatTex(convertdata)
  checkMatTexworkJob = extensions.core_jobsystem.create(checkMatTexwork, 1, convertdata)
end

local checkTexworkJob

local function checkTexwork(job, convertdata)
  local ffi = require("ffi")
  local isDone
  local verifydata = convertdata
  local type = 7
  local countp2 = 0
  local pow2 = {}
  local countcooker = 0
  local cooker = {}
  local issuesTab = {}
  local countext = 0
  local fileext = {}
  local matData = {}

  job.progress = 0
  job.sleep(0.001)
  job.stop = nil

  if not verifydata then
    log('E', '', 'There is no material path' )
    isDone = 2
  elseif not string.match(verifydata, "/") then
    log('E', '', 'Incorrect path' )
    isDone = 2
  else
    log('I', '', 'Checking texture maps' )

    --V2, shortcode much more efficient, checks all types of files at once
    local materialFiles = FS:findFiles(verifydata, "*.cs\t*materials.json", -1, true, false)
    local sorted = {}
    for k,v in pairs(materialFiles) do
      if string.find(v, '/terrains/') and string.find(v, '/terrain/') then
      else
        sorted[k] = v
      end
    end
    materialFiles = sorted
    job.progress = 10
    job.sleep(0.001)

    for _, fn in ipairs(materialFiles) do
      if job.stop == true then
        do return end
      end
      job.yield()
      local dir, basefilename, ext = path.splitWithoutExt(fn)
      matData[fn] = {}

      if string.find(fn, 'materials.cs$') then
        TorqueScript.exec(fn)
        objects = extensions.editor_resourceChecker_resourceUtil.getSimObjects(fn)
      elseif string.find(fn, 'materials.json$') then
        loadJsonMaterialsFile(fn)
        objects = extensions.editor_resourceChecker_resourceUtil.getSimObjects(fn)
      end
      if not tableIsEmpty(objects) then
        log('I', '', 'parsing all materials file: ' .. tostring(fn))

        for _, obj in ipairs(objects) do
          job.yield()
          -- the old material files can also contain other stuff ...
          if obj.___type == "class<Material>" then
            local texfields = extensions.editor_resourceChecker_resourceUtil.getMaterialTexFields(obj)
            if texfields then
              matData[fn][obj:getName()] = {}
              for k,v in pairs(texfields) do
                matData[fn][obj:getName()][k] = v
              end
            end
          end
        end
      end
      for e,t in pairs(matData) do
        pow2[e] = {}
        fileext[e] = {}
        cooker[e] = {}
        for k,v in pairs(t) do
          if job.progress < 75 then
            job.progress = job.progress + 0.001
          end
          pow2[e][k] = {}
          fileext[e][k] = {}
          cooker[e][k] = {}
          for m,d in pairs(v) do
            if job.stop == true then
              do return end
            end
            job.yield()
            local dir, basefilename, ext = path.splitWithoutExt(d)
            if d ~= "" and d ~= nil then
              if FS:fileExists(d) then
                local tex = im.ImTextureHandler(d)
                local size = tex:getSize()
                local format = ffi.string(tex:getFormat())
                if extensions.editor_resourceChecker_resourceUtil.powerOfTwo(size.x) == false or extensions.editor_resourceChecker_resourceUtil.powerOfTwo(size.y) == false then
                  pow2[e][k][m] = d.." Format: "..format.."   Reason: is not a power of 2"
                end
                if d:find("bmp") or d:find("jpg") or d:find("jpeg") or d:find("tga") then
                  fileext[e][k][m] = d.." Format: "..format.."   Reason: not optimal format for textures"
                end
                if not d:find("color.png") and not d:find("normal.png") and not d:find("data.png") then
                  if d:find("png") then
                    cooker[e][k][m] = d.." Format: "..format.."   Reason: cannot be cooked, missing postfix"
                  end
                end
                if d:find("color.png") or d:find("normal.png") or d:find("data.png") then
                  if extensions.editor_resourceChecker_resourceUtil.powerOfTwo(size.x) == false or extensions.editor_resourceChecker_resourceUtil.powerOfTwo(size.y) == false then
                    cooker[e][k][m] = d.." Format: "..format.."   Reason: cannot be cooked, is not a power of 2"
                  end
                  if format:find("R16G16B16") then
                    cooker[e][k][m] = d.." Format: "..format.."   Reason: cannot be cooked, is a 16 bit PNG"
                  end
                end
              end
            end
          end
        end
      end
    end

    job.progress = 75

    job.sleep(0.001)
    local tempTable = {}
    for k,v in pairs(cooker) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k] = {}
            countcooker = countcooker + 1
          end
        end
      end
    end
    for k,v in pairs(cooker) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k][e] = b
          end
        end
      end
    end
    cooker = tempTable
    local tempTable = {}
    for k,v in pairs(fileext) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k] = {}
            countext = countext + 1
          end
        end
      end
    end
    for k,v in pairs(fileext) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k][e] = b
          end
        end
      end
    end
    fileext = tempTable
    local tempTable = {}
    for k,v in pairs(pow2) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k] = {}
            countp2 = countp2 + 1
          end
        end
      end
    end
    for k,v in pairs(pow2) do
      job.yield()
      if not tableIsEmpty(v) then
        for e,b in pairs(v) do
          if not tableIsEmpty(b) then
            tempTable[k][e] = b
          end
        end
      end
    end
    if job.stop == true then
      do return end
    end
    job.progress = 90
    job.sleep(0.001)
    pow2 = tempTable
    issuesTab["Incorrect File for Texture Cooker"] = {}
    issuesTab["Incorrect File Format"] = {}
    issuesTab["Incorrect Resolution"] = {}
    for k,v in pairs(cooker) do
      issuesTab["Incorrect File for Texture Cooker"][k] = v
    end
    for k,v in pairs(fileext) do
      issuesTab["Incorrect File Format"][k] = v
    end
    for k,v in pairs(pow2) do
      issuesTab["Incorrect Resolution"][k] = v
    end
    job.progress = 100
    job.sleep(0.001)
    log('I', '', 'Checked all texture files' )
    isDone = 1
  end

  local data = {type, countext, countp2, issuesTab, isDone, countcooker}
  extensions.editor_resourceChecker.jobData(2, data)
end

local function checkTex(convertdata)
  checkTexworkJob = extensions.core_jobsystem.create(checkTexwork, 1, convertdata)
end

local checkmissingMatsworkJob

local function checkmissingMatswork(job, convertdata)
  local luaType = type
  local isDone
  local verifydata = convertdata
  local type = 8
  local objmatTable = {}
  local mapping = {}
  local missingMat = {}
  local count = 0

  job.progress = 0
  job.sleep(0.001)
  job.stop = nil

  if not verifydata then
    log('E', '', 'There is no material path' )
    isDone = 2
  elseif not string.match(verifydata, "/") then
    log('E', '', 'Incorrect path' )
    isDone = 2
  else
    log('I', '', 'Checking missing materials mapping' )
    log('I', '', 'Checking material files' )
    --V2, shortcode much more efficient, checks all types of files at once
    --we have to check for common art too...
    local commonVeh = FS:findFiles("/vehicles/common", "*.cs\t*materials.json", -1, true, false)
    local commonArt = FS:findFiles("/art", "*.cs\t*materials.json", -1, true, false)
    local commonCore = FS:findFiles("/core", "*.cs\t*materials.json", -1, true, false)
    local materialFiles = FS:findFiles(verifydata, "*.cs\t*materials.json", -1, true, false)
    for k,v in pairs(commonVeh) do
      table.insert(materialFiles, v)
    end
    for k,v in pairs(commonArt) do
      table.insert(materialFiles, v)
    end
    for k,v in pairs(commonCore) do
      table.insert(materialFiles, v)
    end
    job.progress = 20
    job.sleep(0.001)

    for _, fn in ipairs(materialFiles) do
      if job.stop == true then
        do return end
      end
      job.yield()
      local dir, basefilename, ext = path.splitWithoutExt(fn)

      if string.find(fn, 'materials.cs$') then
        TorqueScript.exec(fn)
        objects = extensions.editor_resourceChecker_resourceUtil.getSimObjects(fn)
      elseif string.find(fn, 'materials.json$') then
        loadJsonMaterialsFile(fn)
        objects = extensions.editor_resourceChecker_resourceUtil.getSimObjects(fn)
      end
      if not tableIsEmpty(objects) then

        job.yield()
        log('I', '', 'parsing all materials file: ' .. tostring(fn))

        for _, obj in ipairs(objects) do
          if job.progress < 50 then
            job.progress = job.progress + 0.001
          end
          job.yield()
          -- the old material files can also contain other stuff ...
          if obj.___type == "class<Material>" then
            mapping[obj:getField("mapTo",0)] = true
          end
        end
      end
    end
    job.progress = 50
    job.sleep(0.001)
    if job.stop == true then
      do return end
    end
    log('I', '', 'Checking meshes for materials' )
    local meshFiles = FS:findFiles(verifydata, "*.dae\t*.dts\t*.cdae\t*.cached.dts", -1, true, false)
    for k,v in ipairs(meshFiles) do
      local dir, basefilename, ext = path.splitWithoutExt(v)
      if job.progress < 75 then
        job.progress = job.progress + 0.01
      end
      job.yield()
      local shapeLoader
      if not shapeLoader then
        shapeLoader = ShapePreview()
      end
      shapeLoader:setObjectModel(v)
      log('I', '', 'Checking mesh '.. v)
      table.insert(objmatTable, {shapeLoader:getMaterialNames(), v})
      shapeLoader:clearShape()
    end
    job.progress = 75
    job.sleep(0.001)
    for k,v in pairs(objmatTable) do
      if job.progress < 90 then
        job.progress = job.progress + 0.01
      end
      job.yield()
      if (luaType(v[1]) == "table") then
        for g,j in pairs(v[1]) do
          if not mapping[j] then
            log('I', '', 'Found missing mat '..j.. ' in: '..v[2] )
            table.insert(missingMat, j.."   Mesh: "..v[2])
            count = count + 1
          end
        end
      else
        log("E","", "Is not a table???")
      end
    end
    if job.stop == true then
      do return end
    end
    table.sort(missingMat, function(a,b) return string.upper(a) < string.upper(b) end)
    job.progress = 100
    job.sleep(0.001)
    isDone = 1
  end
  local data = {type, count, "dummy", missingMat, isDone}
  extensions.editor_resourceChecker.jobData(2, data)
end

local function checkmissingMats(convertdata)
  checkmissingMatsworkJob = extensions.core_jobsystem.create(checkmissingMatswork, 1, convertdata)
end

--resource explorer
local checkStaticworkJob

local function checkStaticwork(job)
  log('I', '', 'Checking TSStatics' )
  local type = 1
  local isDone
  local countduplicate = 0
  local countScene = 0
  local size = 0
  local sizecache = 0
  job.progress = 0
  job.stop = nil
  job.sleep(0.001)
  local meshNames = scenetree.findClassObjects('TSStatic')
  local shapeList = {}
  job.progress = 20
  job.sleep(0.001)
  for i,v in ipairs(meshNames) do
    if job.stop == true then
      do return end
    end
    if job.progress < 50 then
      job.progress = job.progress + 0.01
    end
    job.yield()
    local m = scenetree.findObject(v)
    if not m then log("E", "", "TSStatic object broken "..dumps(v))
    else
      shapeList[m.shapeName] = true
      countScene = countScene + 1
    end
  end
  job.progress = 50
  job.sleep(0.001)
  local shapes = {}
  local shapesprepare = {}
  for k,v in pairs(shapeList) do
    if job.stop == true then
      do return end
    end
    if job.progress < 90 then
      job.progress = job.progress + 0.01
    end
    job.yield()
    log('I', '', 'Found shape '..k )
    local fsize = FS:fileSize(k)
    if fsize > 0 and fsize > -1 then
      size = size + fsize
    end
    local cacheSize = 0
    if FS:fileExists(k:gsub('.dae','.cdae')) then
      local fsize = FS:fileSize(k:gsub('.dae','.cdae'))
      cacheSize = fsize
      if fsize > 0 and fsize > -1 then
        sizecache = sizecache + fsize
      end
    else
      if FS:fileExists('/temp/'..k:gsub('.dae','.cdae')) then
        local fsize = FS:fileSize('/temp/'..k:gsub('.dae','.cdae'))
        cacheSize = fsize
        if fsize > 0 and fsize > -1 then
          sizecache = sizecache + fsize
        end
      end
    end
    table.insert(shapesprepare, {k,fsize,cacheSize})
    countduplicate = countduplicate + 1
  end
  table.sort(shapesprepare, function(a,b) return tonumber(a[3]) > tonumber(b[3]) end)
  for k,v in pairs(shapesprepare) do
    local size = string.format("%.2f", v[2] / 1048576)
    local cachesize = string.format("%.2f", v[3] / 1048576)
    table.insert(shapes, v[1].." Collada size: "..size.." MB. Cache size: "..cachesize.." MB")
  end
  job.progress = 90
  job.sleep(0.001)
  job.progress = 100
  job.sleep(0.001)
  isDone = 1
  size = string.format("%.2f", size/1048576)
  sizecache = string.format("%.2f", sizecache/1048576)
  local data = {type, countduplicate, countScene, shapes, isDone, size, sizecache}
  extensions.editor_resourceChecker.jobData(3, data)
end

local function checkStatic()
  checkStaticworkJob = extensions.core_jobsystem.create(checkStaticwork, 1)
end

local checkForestworkJob

local function checkForestwork(job)
  log('I', '', 'Checking TSForestItemData' )
  local type = 2
  local isDone
  local countduplicate = 0
  job.progress = 0
  job.stop = nil
  job.sleep(0.001)
  local meshNames = scenetree.findClassObjects('TSForestItemData')
  local shapeList = {}
  local size = 0
  local sizecache = 0
  job.progress = 20
  job.sleep(0.001)
  for i,v in ipairs(meshNames) do
    if job.stop == true then
      do return end
    end
    if job.progress < 50 then
      job.progress = job.progress + 0.01
    end
    job.yield()
    local m = scenetree.findObject(v)
    if not m then log("E", "", "ForestItem object broken "..dumps(v))
    else
      shapeList[m:getField("shapeFile",0)] = true
    end
  end
  job.progress = 50
  job.sleep(0.001)
  local shapes = {}
  local shapesprepare = {}
  for k,v in pairs(shapeList) do
    if job.stop == true then
      do return end
    end
    if job.progress < 90 then
      job.progress = job.progress + 0.01
    end
    job.yield()
    log('I', '', 'Found ForestItem '..k )
    local fsize = FS:fileSize(k)
    if fsize > 0 and fsize > -1 then
      size = size + fsize
    end
    local cacheSize = 0
    if FS:fileExists(k:gsub('.dae','.cdae')) then
      local fsize = FS:fileSize(k:gsub('.dae','.cdae'))
      cacheSize = fsize
      if fsize > 0 and fsize > -1 then
        sizecache = sizecache + fsize
      end
    else
      if FS:fileExists('/temp/'..k:gsub('.dae','.cdae')) then
        local fsize = FS:fileSize('/temp/'..k:gsub('.dae','.cdae'))
        cacheSize = fsize
        if fsize > 0 and fsize > -1 then
          sizecache = sizecache + fsize
        end
      end
    end
    table.insert(shapesprepare, {k,fsize,cacheSize})
    countduplicate = countduplicate + 1
  end
  table.sort(shapesprepare, function(a,b) return tonumber(a[3]) > tonumber(b[3]) end)
  for k,v in pairs(shapesprepare) do
    local size = string.format("%.2f", v[2] / 1048576)
    local cachesize = string.format("%.2f", v[3] / 1048576)
    table.insert(shapes, v[1].." Collada size: "..size.." MB. Cache size: "..cachesize.." MB")
  end
  job.progress = 90
  job.sleep(0.001)
  job.progress = 100
  job.sleep(0.001)
  isDone = 1
  size = string.format("%.2f", size/1048576)
  sizecache = string.format("%.2f", sizecache/1048576)
  local data = {type, countduplicate, "dummy", shapes, isDone, size, sizecache}
  extensions.editor_resourceChecker.jobData(3, data)
end

local function checkForest()
  checkForestworkJob = extensions.core_jobsystem.create(checkForestwork, 1)
end

local checkTerrainsworkJob

local function checkTerrainswork(job)
  log('I', '', 'Checking TerrainBlocks' )
  local type = 3
  local isDone
  local countduplicate = 0
  job.progress = 0
  job.stop = nil
  job.sleep(0.001)
  local meshNames = scenetree.findClassObjects('TerrainBlock')
  local shapeList = {}
  local size = 0
  job.progress = 20
  job.sleep(0.001)
  for i,v in ipairs(meshNames) do
    if job.stop == true then
      do return end
    end
    job.yield()
    local m = scenetree.findObject(v)
    if not m then log("E", "", "TerrainBlock object broken "..dumps(v))
    else
      shapeList[m:getField("terrainFile",0)] = true
    end
  end
  job.progress = 50
  job.sleep(0.001)
  local shapes = {}
  for k,v in pairs(shapeList) do
    if job.stop == true then
      do return end
    end
    job.yield()
    table.insert(shapes, k)
    log('I', '', 'Found terrain '..k )
    local fsize = FS:fileSize(k)
    if fsize > 0 and fsize > -1 then
      size = size + fsize
    end
    countduplicate = countduplicate + 1
  end
  table.sort(shapes, function(a,b) return string.upper(a) < string.upper(b) end)
  job.progress = 90
  job.sleep(0.001)
  job.progress = 100
  job.sleep(0.001)
  isDone = 1
  size = string.format("%.2f", size/1048576)
  local data = {type, countduplicate, size, shapes, isDone}
  extensions.editor_resourceChecker.jobData(3, data)
end

local function checkTerrains()
  checkTerrainsworkJob = extensions.core_jobsystem.create(checkTerrainswork, 1)
end

local matstoRemove = {}

local checkUnusedMatsworkJob

local function checkUnusedMatswork(job, levelname, removal)
  local luaType = type
  local type = 4
  local isDone
  local countduplicate = 0
  local unused = {}
  local shapes = {}
  job.progress = 0
  job.sleep(0.001)
  job.stop = nil
  if not levelname then
    log('E', '', 'There is no level name' )
    isDone = 2
  else
    log('I', '', 'Checking for unused materials' )
    log('I', '', 'Checking Prefabs' )
    local shapeList = {}
    job.progress = 5
    local prefabs = FS:findFiles("/levels/"..levelname.."/", "*.prefab\t*.prefab.json", -1, true, false)
    local missionPrefabs = FS:findFiles("/gameplay/missions/"..levelname.."/", "*.prefab\t*.prefab.json", -1, true, false)
    for k,v in pairs(missionPrefabs) do
      table.insert(prefabs, v)
    end
    for _, fn in ipairs(prefabs) do
      if job.stop == true then
        do return end
      end
      job.yield()
      local dir, basefilename, ext = path.splitWithoutExt(fn)
      if FS:fileSize(fn) > 0 then
        if string.find(fn, 'prefab$') then
          log('I', '', 'Loading ts prefab file '..fn )
          local f = io.open(fn, "r")
          if f then
            for line in f:lines() do
              job.yield()
              if line:match('shapeName') then
                line = line:gsub('shapeName', '')
                line = line:gsub('"', "")
                line = line:gsub(' ', "")
                line = line:gsub(';', "")
                line = line:gsub('=', "")
                shapeList[line] = true
              end
            end
            f:close()
          end
        elseif string.find(fn, 'prefab.json$') then
          log('I', '', 'Loading json prefab file '..fn )
          local f = io.open(fn, "r")
          for line in f:lines() do
            job.yield()
            local data = json.decode(line)
            if data.shapeName then
              shapeList[data.shapeName] = true
            end
          end
          f:close()
        end
      end
    end
    log('I', '', 'Checking TSForestItemData' )
    local meshNames = scenetree.findClassObjects('TSForestItemData')
    local objmatTable = {}
    local mats = {}
    job.progress = 10
    job.sleep(0.001)
    for k,v in pairs(meshNames) do
      if job.stop == true then
        do return end
      end
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "ForestItem object broken "..dumps(v))
      else
        shapeList[m:getField("shapeFile",0)] = true
      end
    end
    job.progress = 15
    job.sleep(0.001)
    for k,v in pairs(shapeList) do
      if job.stop == true then
        do return end
      end
      job.yield()
      if FS:fileExists(k) then
        local shapeLoader
        if not shapeLoader then
          shapeLoader = ShapePreview()
        end
        shapeLoader:setObjectModel(k)
        table.insert(objmatTable, shapeLoader:getMaterialNames())
        shapeLoader:clearShape()
      end
    end
    job.progress = 20
    job.sleep(0.001)
    log('I', '', 'Checking TSStatics' )
    local meshNames = scenetree.findClassObjects('TSStatic')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "TSStatic object broken "..dumps(v))
      else
        table.insert(objmatTable, m:getMaterialNames())
      end
    end
    job.progress = 25
    job.sleep(0.001)
    for k,v in pairs(objmatTable) do
      job.yield()
      if (luaType(v) == "table") then
        for k,v in pairs(v) do
          mats[v] = true
        end
      else
        log("E","", "Is not a table???")
      end
    end
    job.progress = 30
    job.sleep(0.001)
    local terrainMats = {}
    log('I', '', 'Checking TerrainBlocks' )
    local meshNames = scenetree.findClassObjects('TerrainBlock')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "TerrainBlock object broken "..dumps(v))
      else
        table.insert(terrainMats, m:getMaterials())
      end
    end
    job.progress = 35
    job.sleep(0.001)
    for k,v in pairs(terrainMats) do
      job.yield()
      for k,v in pairs(v) do
        mats[v:getInternalName()] = true
      end
    end
    job.progress = 40
    job.sleep(0.001)
    log('I', '', 'Checking GroundPlanes' )
    local meshNames = scenetree.findClassObjects('GroundPlane')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "GroundPlane object broken "..dumps(v))
      else
        mats[m:getField("Material",0)] = true
      end
    end
    job.progress = 45
    job.sleep(0.001)
    log('I', '', 'Checking GroundCovers' )
    local meshNames = scenetree.findClassObjects('GroundCover')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "GroundCover object broken "..dumps(v))
      else
        mats[m:getField("Material",0)] = true
      end
    end
    job.progress = 50
    job.sleep(0.001)
    log('I', '', 'Checking DecalRoads' )
    local meshNames = scenetree.findClassObjects('DecalRoad')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "DecalRoad object broken "..dumps(v))
      else
        mats[m:getField("Material",0)] = true
      end
    end
    job.progress = 55
    job.sleep(0.001)
    log('I', '', 'Checking MeshRoads' )
    local meshNames = scenetree.findClassObjects('MeshRoad')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "MeshRoad object broken "..dumps(v))
      else
        mats[m:getField("topMaterial",0)] = true
        mats[m:getField("sideMaterial",0)] = true
        mats[m:getField("bottomMaterial",0)] = true
      end
    end
    job.progress = 60
    job.sleep(0.001)
    log('I', '', 'Checking DecalData' )
    local meshNames = scenetree.findClassObjects('DecalData')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "DecalData object broken "..dumps(v))
      else
        mats[m:getField("Material",0)] = true
      end
    end
    job.progress = 65
    job.sleep(0.001)
    if job.stop == true then
      print("STOPPING")
      do return end
    end
    local materialFiles
    if removal == 1 then
      materialFiles = FS:findFiles("/levels/"..levelname.."/", "*materials.json", -1, true, false)
    else
      materialFiles = FS:findFiles("/levels/"..levelname.."/", "*.cs\t*materials.json", -1, true, false)
    end
    local blacklist = {"cubemaps"}
    local matTable = {}

    for _, fn in ipairs(materialFiles) do
      job.yield()
      for _,b in ipairs(blacklist) do
        if fn:find(b) then
          -- log("E","onClientPreStartMission","skipped = "..dumps(v))
          goto skipFile
        end
      end
      if job.stop == true then
        print("STOPPING")
        do return end
      end
      local dir, basefilename, ext = path.splitWithoutExt(fn)
      if FS:fileSize(fn) > 0 then
        if string.find(fn, 'materials.json$') then
          log('I', '', 'Loading json material file '..fn )
          matTable[fn] = jsonReadFile(fn) or {}
        end
      end
      ::skipFile::
    end
    job.progress = 70
    job.sleep(0.001)
    local materialFilesdata = {}
    if job.stop == true then
      print("STOPPING")
      do return end
    end
    if not tableIsEmpty(matTable) then
      log('I', '', 'parsing all materials')
      for k,v in pairs(matTable) do
        job.yield()
        local path = k
        for k,v in pairs(v) do
          local mat = v
          if mat and mat.name and mat.mapTo and not mat.internalName then
            materialFilesdata[mat.name] = mat.mapTo
          elseif mat and mat.name and mat.internalName then
            materialFilesdata[mat.internalName] = mat.internalName
          elseif mat and not mat.name or mat and not mat.internalName then
            log('W', '', 'Corrupted or incompatible material found '..k)
          end
        end
      end
    end
    job.progress = 75
    job.sleep(0.001)

    local tmpMats = {}
    for k,v in pairs(mats) do
      job.yield()
      k = string.lower(k)
      tmpMats[k] = true
    end
    mats = tmpMats
    for k,v in pairs(materialFilesdata) do
      job.yield()
      if mats[string.lower(k)] or mats[string.lower(v)] then
        --print("is used")
      else
        log('I', '', 'Found unused material '..v )
        unused[k] = v
      end
    end
    job.progress = 85
    job.sleep(0.001)
    for k,v in pairs(unused) do
      job.yield()
      local m = scenetree.findObject(k)
      if m and m:getFileName() then
        table.insert(shapes, k.."  in: "..m:getFileName())
        countduplicate = countduplicate + 1
      end
    end
    table.sort(shapes, function(a,b) return string.upper(a) < string.upper(b) end)
    job.progress = 100
    job.sleep(0.001)
    isDone = 1
  end
  local data = {type, countduplicate, "dummy", shapes, isDone}
  if removal == 1 then
    local toRemove = {}
    for k,v in pairs(unused) do
      job.yield()
      local m = scenetree.findObject(k)
      if m and m:getFileName() then
        toRemove[k] = m:getFileName()
      end
    end
    extensions.editor_resourceChecker_resourceUtil.matstoRemove = toRemove
  else
    extensions.editor_resourceChecker.jobData(3, data)
  end
end

local function checkUnusedMats(levelname, removal)
  checkUnusedMatsworkJob = extensions.core_jobsystem.create(checkUnusedMatswork, 1, levelname, removal)
end

local checkUsedMatsworkJob

local function checkUsedMatswork(job, levelname)
  local luaType = type
  local type = 8
  local isDone
  local countduplicate = 0
  local sizeTotal = 0
  local shapes = {}
  local usages = {}
  job.progress = 0
  job.sleep(0.001)
  job.stop = nil
  if not levelname then
    log('E', '', 'There is no level name' )
    isDone = 2
  else
    log('I', '', 'Checking for used materials' )
    log('I', '', 'Checking Prefabs' )
    local shapeList = {}
    local matsInObjects = {}
    job.progress = 5
    local prefabs = {}
    local prefabInstances = scenetree.findClassObjects('Prefab')
    for k,v in pairs(prefabInstances) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "Prefab object broken "..dumps(v))
      else
        if m:getField('filename',0) then
          table.insert(prefabs, m:getField('filename',0))
        end
      end
    end
    for _, fn in ipairs(prefabs) do
      if job.stop == true then
        do return end
      end
      job.yield()
      local dir, basefilename, ext = path.splitWithoutExt(fn)
      if FS:fileSize(fn) > 0 then
        if string.find(fn, 'prefab$') then
          log('I', '', 'Loading ts prefab file '..fn )
          local f = io.open(fn, "r")
          if f then
            for line in f:lines() do
              job.yield()
              if line:match('shapeName') then
                line = line:gsub('shapeName', '')
                line = line:gsub('"', "")
                line = line:gsub(' ', "")
                line = line:gsub(';', "")
                line = line:gsub('=', "")
                shapeList[line] = true
              end
            end
            f:close()
          end
        elseif string.find(fn, 'prefab.json$') then
          log('I', '', 'Loading json prefab file '..fn )
          local f = io.open(fn, "r")
          for line in f:lines() do
            job.yield()
            local data = json.decode(line)
            if data.shapeName then
              shapeList[data.shapeName] = true
            end
          end
          f:close()
        end
      end
    end
    log('I', '', 'Checking TSForestItemData' )
    local meshNames = scenetree.findClassObjects('TSForestItemData')
    local objmatTable = {}
    local mats = {}
    local allMatsUsages = {}
    job.progress = 10
    job.sleep(0.001)
    for k,v in pairs(meshNames) do
      if job.stop == true then
        do return end
      end
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "ForestItem object broken "..dumps(v))
      else
        shapeList[m:getField("shapeFile",0)] = true
      end
    end
    job.progress = 15
    job.sleep(0.001)
    for k,v in pairs(shapeList) do
      if job.stop == true then
        do return end
      end
      job.yield()
      if FS:fileExists(k) then
        local shapeLoader
        if not shapeLoader then
          shapeLoader = ShapePreview()
        end
        shapeLoader:setObjectModel(k)
        table.insert(objmatTable, shapeLoader:getMaterialNames())
        matsInObjects[k] = shapeLoader:getMaterialNames()
        shapeLoader:clearShape()
      end
    end
    job.progress = 20
    job.sleep(0.001)
    log('I', '', 'Checking TSStatics' )
    local meshNames = scenetree.findClassObjects('TSStatic')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "TSStatic object broken "..dumps(v))
      else
        table.insert(objmatTable, m:getMaterialNames())
        matsInObjects[m:getModelFile()] = m:getMaterialNames()
      end
    end
    job.progress = 25
    job.sleep(0.001)
    for k,v in pairs(objmatTable) do
      job.yield()
      if (luaType(v) == "table") then
        for k,v in pairs(v) do
          mats[v] = true
          table.insert(allMatsUsages, v)
        end
      else
        log("E","", "Is not a table???")
      end
    end
    job.progress = 30
    job.sleep(0.001)
    local terrainMats = {}
    log('I', '', 'Checking TerrainBlocks' )
    local meshNames = scenetree.findClassObjects('TerrainBlock')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "TerrainBlock object broken "..dumps(v))
      else
        table.insert(terrainMats, m:getMaterials())
      end
    end
    job.progress = 35
    job.sleep(0.001)
    for k,v in pairs(terrainMats) do
      job.yield()
      for k,v in pairs(v) do
        mats[v:getInternalName()] = true
        table.insert(allMatsUsages, v:getInternalName())
      end
    end
    job.progress = 40
    job.sleep(0.001)
    log('I', '', 'Checking GroundPlanes' )
    local meshNames = scenetree.findClassObjects('GroundPlane')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "GroundPlane object broken "..dumps(v))
      else
        mats[m:getField("Material",0)] = true
        table.insert(allMatsUsages, m:getField("Material",0))
        if not matsInObjects['GroundPlanes'] then matsInObjects['GroundPlanes'] = {} end
        table.insert(matsInObjects['GroundPlanes'], m:getField("Material",0))
      end
    end
    job.progress = 45
    job.sleep(0.001)
    log('I', '', 'Checking GroundCovers' )
    local meshNames = scenetree.findClassObjects('GroundCover')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "GroundCover object broken "..dumps(v))
      else
        mats[m:getField("Material",0)] = true
        table.insert(allMatsUsages, m:getField("Material",0))
        if not matsInObjects['GroundCovers'] then matsInObjects['GroundCovers'] = {} end
        table.insert(matsInObjects['GroundCovers'], m:getField("Material",0))
      end
    end
    job.progress = 50
    job.sleep(0.001)
    log('I', '', 'Checking DecalRoads' )
    local meshNames = scenetree.findClassObjects('DecalRoad')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "DecalRoad object broken "..dumps(v))
      else
        mats[m:getField("Material",0)] = true
        table.insert(allMatsUsages, m:getField("Material",0))
        if not matsInObjects['DecalRoads'] then matsInObjects['DecalRoads'] = {} end
        table.insert(matsInObjects['DecalRoads'], m:getField("Material",0))
      end
    end
    job.progress = 55
    job.sleep(0.001)
    log('I', '', 'Checking MeshRoads' )
    local meshNames = scenetree.findClassObjects('MeshRoad')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "MeshRoad object broken "..dumps(v))
      else
        mats[m:getField("topMaterial",0)] = true
        mats[m:getField("sideMaterial",0)] = true
        mats[m:getField("bottomMaterial",0)] = true
        table.insert(allMatsUsages, m:getField("topMaterial",0))
        table.insert(allMatsUsages, m:getField("sideMaterial",0))
        table.insert(allMatsUsages, m:getField("bottomMaterial",0))
        if not matsInObjects['MeshRoads'] then matsInObjects['MeshRoads'] = {} end
        table.insert(matsInObjects['MeshRoads'], m:getField("topMaterial",0))
        table.insert(matsInObjects['MeshRoads'], m:getField("sideMaterial",0))
        table.insert(matsInObjects['MeshRoads'], m:getField("bottomMaterial",0))
      end
    end
    job.progress = 60
    job.sleep(0.001)
    log('I', '', 'Checking DecalData' )
    local meshNames = scenetree.findClassObjects('DecalData')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "DecalData object broken "..dumps(v))
      else
        mats[m:getField("Material",0)] = true
        table.insert(allMatsUsages, m:getField("Material",0))
        if not matsInObjects['DecalData'] then matsInObjects['DecalData'] = {} end
        table.insert(matsInObjects['DecalData'], m:getField("Material",0))
      end
    end
    job.progress = 65
    job.sleep(0.001)
    if job.stop == true then
      print("STOPPING")
      do return end
    end
    for k,v in pairs(mats) do
      job.yield()
      local mat = scenetree.findObject(k)
      local matSizeCheck = {}
      if mat and mat.___type == "class<Material>" then
        local texfields = extensions.editor_resourceChecker_resourceUtil.getMaterialTexFields(mat)
        local matSize = 0
        local matName = mat:getFileName()
        local totalSizeCheck = {}
        if texfields then
          for g,h in pairs(texfields) do
            local file = h
            if file:find(".color.png") or file:find(".data.png") or file:find(".normal.png") then
              if FS:fileExists(file:gsub('.png', '.dds')) then
                file = file:gsub('.png', '.dds')
              else
                if FS:fileExists('/temp/'..file:gsub('.png', '.dds')) then
                  file = '/temp/'..file:gsub('.png', '.dds')
                end
              end
            end
            if FS:fileExists(file) then
              local fileData = FS:stat(file)
              if not matSizeCheck[file] == true then
                matSizeCheck[file] = true
                matSize = matSize + fileData.filesize
              end
              if not totalSizeCheck[file] == true then
                totalSizeCheck[file] = true
                sizeTotal = sizeTotal + fileData.filesize
              end
            elseif matName and not FS:fileExists(file) then
              local dir, basefilename, ext = path.splitWithoutExt(matName)
              if FS:fileExists(dir..file) then
                local fileData = FS:stat(dir..file)
                if not matSizeCheck[dir..file] == true then
                  matSizeCheck[dir..file] = true
                  matSize = matSize + fileData.filesize
                end
                if not totalSizeCheck[dir..file] == true then
                  totalSizeCheck[dir..file] = true
                  sizeTotal = sizeTotal + fileData.filesize
                end
              end
            else
              log('W', '', 'File not found '..file )
            end
          end
        end
        local countMat = 0
        for g,h in pairs(allMatsUsages) do
          if h == k then
            countMat = countMat + 1
          end
        end
        mats[k] = {matSize, countMat}
      end
    end
    job.progress = 85
    job.sleep(0.001)
    for k,v in pairs(mats) do
      job.yield()
      if v and (v == true or v == false) then
        mats[k] = nil
      end
    end
    for k,v in pairs(mats) do
      job.yield()
      table.insert(shapes, {k.."  used: "..tostring(v[2]).." times. Textures memory usage: "..string.format("%.2f", v[1]/1048576).." MB", v[1]})
      countduplicate = countduplicate + 1
    end
    table.sort(shapes, function(a,b) return tonumber(a[2]) > tonumber(b[2]) end)
    for k,v in pairs(shapes) do
      shapes[k] = v[1]
    end
    sizeTotal = string.format("%.2f", sizeTotal/1048576)
    for k,v in pairs(matsInObjects) do
      if (luaType(v) == "table") then
        for n,p in pairs(v) do
          if not usages[p] then usages[p] = {} end
          if not tableContains(usages[p], k) then table.insert(usages[p], k) end
        end
      end
    end
    job.progress = 100
    job.sleep(0.001)
    isDone = 1
  end
  local data = {type, countduplicate, sizeTotal, shapes, isDone, usages}
  extensions.editor_resourceChecker.jobData(3, data)
end

local function checkUsedMats(levelname, removal)
  checkUsedMatsworkJob = extensions.core_jobsystem.create(checkUsedMatswork, 1, levelname)
end

local checkColDataworkJob

local function checkColDatawork(job, levelname)
  local luaType = type
  local shapes = {}
  local type = 9
  local isDone
  local polyStats = {0, 0, 0}
  local countduplicate = 0
  job.progress = 0
  job.sleep(0.001)
  job.stop = nil
  if not levelname then
    log('E', '', 'There is no level name' )
    isDone = 2
  else
    log('I', '', 'Checking for used colmeshes' )
    log('I', '', 'Checking Prefabs' )
    job.progress = 5
    local staticInstances = {}
    local prefabs = {}
    local prefabInstances = scenetree.findClassObjects('Prefab')
    for k,v in pairs(prefabInstances) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "Prefab object broken "..dumps(v))
      else
        if m:getField('filename',0) then
          table.insert(prefabs, m:getField('filename',0))
        end
      end
    end
    for _, fn in pairs(prefabs) do
      if job.stop == true then
        do return end
      end
      job.yield()
      local dir, basefilename, ext = path.splitWithoutExt(fn)
      if FS:fileSize(fn) > 0 then
        if string.find(fn, 'prefab.json$') then
          log('I', '', 'Loading json prefab file '..fn )
          local f = io.open(fn, "r")
          for line in f:lines() do
            job.yield()
            local data = json.decode(line)
            if data.shapeName then
              if not staticInstances[data.shapeName] then
                staticInstances[data.shapeName] = {count = 0, collision = {}, ColPolygons = 0, VisPolygons = 0}
              end
              staticInstances[data.shapeName].count = staticInstances[data.shapeName].count + 1
              if not data.collisionType then
                table.insert(staticInstances[data.shapeName].collision, 'Collision Mesh')
              else
                table.insert(staticInstances[data.shapeName].collision, data.collisionType)
              end
            end
          end
          f:close()
        end
      end
    end
    log('I', '', 'Checking Forest Object' )
    local forestObject = nil
    if core_forest and core_forest.getForestObject() then
      forestObject = core_forest.getForestObject()
    end
    job.progress = 10
    job.sleep(0.001)
    if forestObject then
      job.yield()
      for k,v in pairs(forestObject:getData():getItems()) do
        if not staticInstances[v:getData():getShapeFile()] then
          staticInstances[v:getData():getShapeFile()] = {count = 0, collision = {}, ColPolygons = 0, VisPolygons = 0}
        end
        staticInstances[v:getData():getShapeFile()].count = staticInstances[v:getData():getShapeFile()].count + 1
        table.insert(staticInstances[v:getData():getShapeFile()].collision, 'Collision Mesh')
      end
    end
    job.progress = 15
    job.sleep(0.001)
    log('I', '', 'Checking TSStatics' )
    local meshNames = scenetree.findClassObjects('TSStatic')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "TSStatic object broken "..dumps(v))
      else
        if not staticInstances[m:getModelFile()] then
          staticInstances[m:getModelFile()] = {count = 0, collision = {}, ColPolygons = 0, VisPolygons = 0}
        end
        staticInstances[m:getModelFile()].count = staticInstances[m:getModelFile()].count + 1
        table.insert(staticInstances[m:getModelFile()].collision, m:getField('collisionType',0))
      end
    end
    for k,v in pairs(staticInstances) do
      if job.stop == true then
        do return end
      end
      job.yield()
      if FS:fileExists(k) then
        local shapeLoader
        if not shapeLoader then
          shapeLoader = ShapePreview()
        end
        shapeLoader:setObjectModel(k)
        shapeLoader.mFixedDetail = true
        shapeLoader:setCurrentDetail(0)
        shapeLoader:renderWorld(RectI(0,0,256,256))
        staticInstances[k].ColPolygons = shapeLoader.mColPolys
        staticInstances[k].VisPolygons = shapeLoader.mDetailPolys
        shapeLoader:clearShape()
      end
    end
    for k,v in pairs(staticInstances) do
      job.yield()
      local colMeshInst = 0
      local visMeshInst = 0
      local totalCount = 0
      local slow = false
      if v and v.ColPolygons > 0 then
        local count = 0
        for i,c in pairs(v.collision) do
          if c == "Collision Mesh" then count = count + 1 end
        end
        colMeshInst = count
      end
      if v and v.VisPolygons > 0 then
        local count = 0
        for i,c in pairs(v.collision) do
          if (c == "Visible Mesh" or c == "Visible Mesh Final")  then count = count + 1 end
        end
        visMeshInst = count
      end
      if visMeshInst > 0 then
        slow = true
      end
      local colMeshTotSize = colMeshInst*v.ColPolygons
      local visMeshTotSize = visMeshInst*v.VisPolygons
      local totalColSize = colMeshTotSize+visMeshTotSize
      local totalCount = visMeshInst+colMeshInst
      if slow == true then
        table.insert(shapes, {k.."   used: "..totalCount.." times. ColPolys: "..v.ColPolygons..". Visible Mesh ColPolys: "..v.VisPolygons..". Total ColPolys: "..totalColSize..". Warning: This mesh is using Visible Mesh Collisions which might be cause performance issues", totalColSize})
      else
        table.insert(shapes, {k.."   used: "..totalCount.." times. ColPolys: "..v.ColPolygons..". Total ColPolys: "..totalColSize, totalColSize})
      end
      polyStats[1] = polyStats[1] + totalColSize
      polyStats[2] = polyStats[2] + visMeshTotSize
      polyStats[3] = polyStats[3] + visMeshInst
      countduplicate = countduplicate + totalCount
    end
    table.sort(shapes, function(a,b) return tonumber(a[2]) > tonumber(b[2]) end)
    for k,v in pairs(shapes) do
      shapes[k] = v[1]
    end
    job.progress = 100
    job.sleep(0.001)
    isDone = 1
  end
  local data = {type, countduplicate, polyStats, shapes, isDone}
  extensions.editor_resourceChecker.jobData(3, data)
end

local function checkColData(levelname, removal)
  checkColDataworkJob = extensions.core_jobsystem.create(checkColDatawork, 1, levelname)
end

local shapestoRemove = {}

local checkUnusedModelsworkJob

--merged unused TSStatic and Forest Items + new functionality
local function checkUnusedModelswork(job, levelname, removal)
  local luaType = type
  local type = 5
  local isDone
  local countduplicate = 0
  local unused = {}
  local models = {}
  local shapes = {}
  local forestShapes = {}
  local size = 0
  job.progress = 0
  job.stop = nil
  job.sleep(0.001)
  if not levelname then
    log('E', '', 'There is no level name' )
    isDone = 2
  else
    log('I', '', 'Checking for unused models' )
    log('I', '', 'Checking Prefabs' )
    job.progress = 5
    local prefabs = FS:findFiles("/levels/"..levelname.."/", "*.prefab\t*.prefab.json", -1, true, false)
    local missionPrefabs = FS:findFiles("/gameplay/missions/"..levelname.."/", "*.prefab\t*.prefab.json", -1, true, false)
    for k,v in pairs(missionPrefabs) do
      table.insert(prefabs, v)
    end
    for _, fn in ipairs(prefabs) do
      if job.stop == true then
        do return end
      end
      job.yield()
      local dir, basefilename, ext = path.splitWithoutExt(fn)
      if FS:fileSize(fn) > 0 then
        if string.find(fn, 'prefab$') then
          log('I', '', 'Loading ts prefab file '..fn )
          local f = io.open(fn, "r")
          if f then
            for line in f:lines() do
              job.yield()
              if line:match('shapeName') then
                line = line:gsub('shapeName', '')
                line = line:gsub('"', "")
                line = line:gsub(' ', "")
                line = line:gsub(';', "")
                line = line:gsub('=', "")
                models[line] = true
              end
            end
            f:close()
          end
        elseif string.find(fn, 'prefab.json$') then
          log('I', '', 'Loading json prefab file '..fn )
          local f = io.open(fn, "r")
          for line in f:lines() do
            job.yield()
            local data = json.decode(line)
            if data.shapeName then
              models[data.shapeName] = true
            end
          end
          f:close()
        end
      end
    end
    job.progress = 15
    job.sleep(0.001)
    log('I', '', 'Checking TSStatics' )
    local meshNames = scenetree.findClassObjects('TSStatic')
    for k,v in pairs(meshNames) do
      if job.stop == true then
        do return end
      end
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "TSStatic object broken "..dumps(v))
      else
        models[m:getField("shapeName",0)] = true
      end
    end
    job.progress = 25
    job.sleep(0.001)
    log('I', '', 'Checking TSForestItemData' )
    local meshNames = scenetree.findClassObjects('TSForestItemData')
    local forestModels = {}
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "ForestItem object broken "..dumps(v))
      else
        local i = m:getField("internalName",0)
        if not i then log("E", "", "ForestItem object broken")
        else
          forestModels[i] = m:getField("shapeFile",0)
        end
      end
    end
    job.progress = 45
    job.sleep(0.001)
    log('I', '', 'Checking Forest Folder' )
    local forestInternals = {}
    local forestFiles = FS:findFiles("/levels/"..levelname.."/forest/", "*forest4.json", -1, true, false)
    for k,v in pairs(forestFiles) do
      job.yield()
      local dir, basefilename, ext = path.splitWithoutExt(v)
      if FS:fileSize(v) > 0 then
        forestInternals[basefilename:gsub('.forest4', '')] = true
      end
    end
    job.progress = 50
    job.sleep(0.001)
    log('I', '', 'Checking GroundCovers' )
    local meshNames = scenetree.findClassObjects('GroundCover')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "GroundCover object broken "..dumps(v))
      else
        local type = 0
        for i=1, 8 do
          if m:getField("shapeFilename",type) then models[m:getField("shapeFilename",type)] = true end
          type = type + 1
        end
      end
    end
    job.progress = 55
    job.sleep(0.001)
    for k,v in pairs(forestModels) do
      job.yield()
      if forestInternals[k] then
        models[v] = true
      end
    end
    local modelsNoExt = {}
    job.progress = 60
    job.sleep(0.001)
    for k,v in pairs(models) do
      job.yield()
      local dir, basefilename, ext = path.splitWithoutExt(k)
      basefilename = string.lower(basefilename)
      modelsNoExt[basefilename] = true
    end
    local tempMdl = {}
    for k,v in pairs(models) do
      job.yield()
      k = string.lower(k)
      tempMdl[k] = true
    end
    models = tempMdl
    job.progress = 65
    local meshFiles = FS:findFiles("/levels/"..levelname.."/", "*.dae\t*.dts\t*.cdae", -1, true, false)
    for k,v in pairs(meshFiles) do
      job.yield()
      local dir, basefilename, ext = path.splitWithoutExt(v)
      if models[string.lower(v)] then
      elseif ext == "cdae" and modelsNoExt[string.lower(basefilename)] then
      else
        log('I', '', 'Found unused model '..v )
        unused[v] = true
      end
    end
    if job.stop == true then
      do return end
    end
    job.progress = 75
    for k,v in pairs(forestModels) do
      forestShapes[string.lower(v)] = true
    end
    job.sleep(0.001)
    for k,v in pairs(unused) do
      job.yield()
      if forestShapes[string.lower(k)] then
        table.insert(shapes, k.."   Warning: This is an active forest item, but not used in the level")
      else
        table.insert(shapes, k)
      end
      local fsize = FS:fileSize(k)
      if fsize > 0 and fsize > -1 then
        size = size + fsize
      end
      countduplicate = countduplicate + 1
    end
    table.sort(shapes, function(a,b) return string.upper(a) < string.upper(b) end)
    job.progress = 100
    job.sleep(0.001)
    isDone = 1
    size = string.format("%.2f", size/1048576)
  end
  local data = {type, countduplicate, size, shapes, isDone}
  if removal == 1 then
    local toRemove = {}
    for k,v in pairs(unused) do
      job.yield()
      if forestShapes[string.lower(k)] then
        table.insert(toRemove, k.." /levels/"..levelname.."/art/forest/managedItemData.json")
      else
        table.insert(toRemove, k)
      end
    end
    extensions.editor_resourceChecker_resourceUtil.shapestoRemove = toRemove
  else
    extensions.editor_resourceChecker.jobData(3, data)
  end
end

local function checkUnusedModels(levelname, removal)
  checkUnusedModelsworkJob = extensions.core_jobsystem.create(checkUnusedModelswork, 1, levelname, removal)
end

local textoRemove = {}

local unusedTexturesworkJob

local function unusedTextureswork(job, levelname, removal)
  local type = 6
  local isDone
  local countduplicate = 0
  local unused = {}
  local textures = {}
  local shapes = {}
  local size = 0
  job.progress = 0
  job.sleep(0.001)
  job.stop = nil

  if not levelname then
    log('E', '', 'There is no level name' )
    isDone = 2
  else
    log('I', '', 'Checking for unused textures' )

    log('I', '', 'Checking Materials' )
    local meshNames = scenetree.findClassObjects('Material')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "Material broken "..dumps(v))
      else
        local texfields = extensions.editor_resourceChecker_resourceUtil.getMaterialTexFields(m)
        if texfields then
          for k,v in pairs(texfields) do
            textures[v] = true
          end
        end
      end
    end
    job.progress = 10
    job.sleep(0.001)
    log('I', '', 'Checking TerrainMaterials' )
    local meshNames = scenetree.findClassObjects('TerrainMaterial')
    for k,v in pairs(meshNames) do
      if job.stop == true then
        do return end
      end
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "TerrainMaterial broken "..dumps(v))
      else
        for k,v in pairs(m:getFields()) do
          job.yield()
          if v["type"] == "filename" then
            textures[m:getField(k,0)] = true
          end
        end
      end
    end
    job.progress = 15
    job.sleep(0.001)
    log('I', '', 'Checking WaterPlanes' )
    local meshNames = scenetree.findClassObjects('WaterPlane')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "WaterPlane broken "..dumps(v))
      else
        for k,v in pairs(m:getFields()) do
          job.yield()
          if v["type"] == "filename" then
            textures[m:getField(k,0)] = true
          end
        end
      end
    end
    job.progress = 20
    job.sleep(0.001)
    log('I', '', 'Checking WaterBlocks' )
    local meshNames = scenetree.findClassObjects('WaterBlock')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "WaterBlock broken "..dumps(v))
      else
        for k,v in pairs(m:getFields()) do
          job.yield()
          if v["type"] == "filename" then
            textures[m:getField(k,0)] = true
          end
        end
      end
    end
    job.progress = 25
    job.sleep(0.001)
    log('I', '', 'Checking Rivers' )
    local meshNames = scenetree.findClassObjects('River')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "River broken "..dumps(v))
      else
        for k,v in pairs(m:getFields()) do
          job.yield()
          if v["type"] == "filename" then
            textures[m:getField(k,0)] = true
          end
        end
      end
    end
    job.progress = 30
    job.sleep(0.001)
    log('I', '', 'Checking CloudLayers' )
    local meshNames = scenetree.findClassObjects('CloudLayer')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "CloudLayer broken "..dumps(v))
      else
        textures[m:getField("texture",0)] = true
      end
    end
    job.progress = 40
    job.sleep(0.001)
    log('I', '', 'Checking ScatterSkies' )
    local meshNames = scenetree.findClassObjects('ScatterSky')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "ScatterSky broken "..dumps(v))
      else
        for k,v in pairs(m:getFields()) do
          job.yield()
          if v["type"] == "filename" then
            textures[m:getField(k,0)] = true
          end
        end
      end
    end
    log('I', '', 'Checking Cubemaps' )
    local meshNames = scenetree.findClassObjects('CubemapData')
    for k,v in pairs(meshNames) do
      job.yield()
      local m = scenetree.findObject(v)
      if not m then log("E", "", "Cubemap broken "..dumps(v))
      else
        for k,v in pairs(m:getFields()) do
          job.yield()
          if v["type"] == "filename" then
            textures[m:getField(k,0)] = true
            textures[m:getField(k,1)] = true
            textures[m:getField(k,2)] = true
            textures[m:getField(k,3)] = true
            textures[m:getField(k,4)] = true
            textures[m:getField(k,5)] = true
          end
        end
      end
    end
    log('I', '', 'Checking Info' )
    local meshNames = jsonReadFile("/levels/"..levelname.."/info.json")
    if meshNames then
      for k,v in pairs(meshNames) do
        job.yield()
        if k == "previews" then
          for i,t in pairs(v) do
            textures[t] = true
          end
        end
        if k == "spawnPoints" then
          for i,t in pairs(v) do
            for i,m in pairs(t) do
              if i == "preview" then
                textures[m] = true
              end
            end
          end
        end
      end
    end
    job.progress = 50
    job.sleep(0.001)
    local texTemp = {}
    for k,v in pairs(textures) do
      job.yield()
      if k ~= ""  and k ~= nil then
        local dir, filename, ext = path.split(k)
        if filename then
          local txt = string.lower(filename:gsub('.'..ext, ''))
          texTemp[txt] = true
        end
      end
    end
    if job.stop == true then
      do return end
    end
    job.progress = 65
    job.sleep(0.001)
    local texFiles = FS:findFiles("/levels/"..levelname.."/", ".png\t*.dds", -1, true, false)
    local blacklist = {"buslines", "quickrace", "scenarios", "scenarios", "lights", "export", "import", "minimap"}
    for k,v in pairs(texFiles) do
      job.yield()
      for _,b in ipairs(blacklist) do
        if v:find(b) then
          -- log("E","onClientPreStartMission","skipped = "..dumps(v))
          goto skipTex
        end
      end
      local dir, filename, ext = path.split(v)
      local txt = string.lower(filename:gsub('.'..ext, ''))
      if texTemp[txt] then
      elseif filename:find("ter.depth") or filename:find("minimap") or filename:find("annotation") or filename:find("preview") or filename:find("imposter") or filename:find("spawn") then
      else
        log('I', '', 'Found unused texture '..v )
        unused[v] = true
      end
      ::skipTex::
    end
    if job.stop == true then
      do return end
    end
    job.progress = 75
    job.sleep(0.001)

    for k,v in pairs(unused) do
      job.yield()
      table.insert(shapes, k)
      local fsize = FS:fileSize(k)
      if fsize > 0 and fsize > -1 then
        size = size + fsize
      end
      countduplicate = countduplicate + 1
    end
    table.sort(shapes, function(a,b) return string.upper(a) < string.upper(b) end)
    job.progress = 100
    job.sleep(0.001)
    isDone = 1
    size = string.format("%.2f", size/1048576)
  end
  local data = {type, countduplicate, size, shapes, isDone}
  if removal == 1 then
    local toRemove = {}
    for k,v in pairs(unused) do
      job.yield()
      table.insert(toRemove, k)
    end
    extensions.editor_resourceChecker_resourceUtil.textoRemove = toRemove
  else
    extensions.editor_resourceChecker.jobData(3, data)
  end
end

local function unusedTextures(levelname, removal)
  unusedTexturesworkJob = extensions.core_jobsystem.create(unusedTextureswork, 1, levelname, removal)
end

local removeUnusedworkJob

local function removeUnusedwork(job, levelname, item, selected)
  local type = 7
  local count = 0
  local isDone
  local size = 0
  job.progress = 0
  job.sleep(0.001)
  job.stop = nil
  local materialsToRemove = {}
  local shapesToRemove = {}
  local texturesToRemove = {}
  if not levelname then
    log('E', '', 'There is no level name' )
    isDone = 2
  else
    log('I', '', 'Removing unused files' )
    if item == 1 then
      if not tableIsEmpty(selected) then
        for k,v in pairs(selected) do
          local entry = k:gsub(' ','')
          entry = entry:gsub('in:',';')
          local count = 0
          local location
          local mat
          for w in entry:gmatch("([^;]+)") do
            count = count + 1
            if (count % 2 == 0) then
              location = w
            else
              mat = w
            end
          end
          materialsToRemove[mat] = location
        end
      else
        extensions.editor_resourceChecker_resourceUtil.checkUnusedMats(levelname, 1)
        while checkUnusedMatsworkJob.running do
          job.sleep(0.1)
        end
        materialsToRemove = extensions.editor_resourceChecker_resourceUtil.matstoRemove
      end
      job.progress = 5
      if not tableIsEmpty(materialsToRemove) then
        for k,v in pairs(materialsToRemove) do
          if string.find(v, levelname) then
            count = count + 1
            log('I', '', 'Removing unused material '..k..' in '..v )
            editor.removeMaterialFromJson(k, v)
          end
        end
      end
      job.progress = 15
    end
    if item == 2 then
      if not tableIsEmpty(selected) then
        for k,v in pairs(selected) do
          table.insert(shapesToRemove, k)
        end
      else
        extensions.editor_resourceChecker_resourceUtil.checkUnusedModels(levelname, 1)
        while checkUnusedModelsworkJob.running do
          job.sleep(0.1)
        end
        shapesToRemove = extensions.editor_resourceChecker_resourceUtil.shapestoRemove
      end
      job.progress = 20
      if not tableIsEmpty(shapesToRemove) then
        for k,v in pairs(shapesToRemove) do
          local file
          if string.match(v, "managedItemData.json") then
            file = v:gsub(' /levels/'..levelname..'/art/forest/managedItemData.json','')
            extensions.editor_resourceChecker_resourceUtil.removeFromForestJson(file, "/levels/"..levelname.."/art/forest/managedItemData.json")
          elseif string.match(v, "   Warning: This is an active forest item, but not used in the level") then
            file = v:gsub('   Warning: This is an active forest item, but not used in the level','')
            extensions.editor_resourceChecker_resourceUtil.removeFromForestJson(file, "/levels/"..levelname.."/art/forest/managedItemData.json")
          else
            file = v
          end
          log('I', '', 'Removing unused shape '..file )
          local fsize = FS:fileSize(file)
          local rem = FS:removeFile(file)
          if rem == 0 then
            count = count + 1
            if fsize > 0 and fsize > -1 then
              size = size + fsize
            end
          end
          if rem == -1 then
            local realPath = FS:getUserPath()
            local fileRealPath = FS:getFileRealPath(file)
            local modFilepath = fileRealPath:gsub(realPath, '')
            if FS:fileExists(modFilepath) then
              local rem = FS:removeFile(modFilepath)
              if rem == 0 then
                count = count + 1
                if fsize > 0 and fsize > -1 then
                  size = size + fsize
                end
              end
            else
              log('W', '', 'Could not remove shape '..file )
            end
          end
        end
      end
      job.progress = 45
    end
    if item == 3 then
      if not tableIsEmpty(selected) then
        for k,v in pairs(selected) do
          table.insert(texturesToRemove, k)
        end
      else
        extensions.editor_resourceChecker_resourceUtil.unusedTextures(levelname, 1)
        while unusedTexturesworkJob.running do
          job.sleep(0.1)
        end
        texturesToRemove = extensions.editor_resourceChecker_resourceUtil.textoRemove
      end
      job.progress = 50
      if not tableIsEmpty(texturesToRemove) then
        for k,v in pairs(texturesToRemove) do
          log('I', '', 'Removing unused texture '..v )
          local fsize = FS:fileSize(v)
          local rem = FS:removeFile(v)
          if rem == 0 then
            count = count + 1
            if fsize > 0 and fsize > -1 then
              size = size + fsize
            end
          end
          if rem == -1 then
            local realPath = FS:getUserPath()
            local fileRealPath = FS:getFileRealPath(v)
            local modFilepath = fileRealPath:gsub(realPath, '')
            if FS:fileExists(modFilepath) then
              local rem = FS:removeFile(modFilepath)
              if rem == 0 then
                count = count + 1
                if fsize > 0 and fsize > -1 then
                  size = size + fsize
                end
              end
            else
              log('W', '', 'Could not remove texture '..v )
            end
          end
        end
      end
    end
    job.progress = 100
    job.sleep(0.001)
    isDone = 1
    size = string.format("%.2f", size/1048576)
  end
  local data = {type, count, size, "nothing", isDone}
  extensions.editor_resourceChecker.jobData(3, data)
end

local function removeUnused(levelname, item, selected)
  removeUnusedworkJob = extensions.core_jobsystem.create(removeUnusedwork, 1, levelname, item, selected)
end

local duplicateDataworkJob

local function duplicateDatawork(job, material)
  local verifydata = material
  local duplicatelist = {}

  if not verifydata then
    log('E', '', 'There is no material' )
  else
    log('I', '', 'Searching materials' )
    local matTable = {}
    local maplist = {}
    --V2, shortcode much more efficient, checks all types of files at once
    --we have to check for common art too...
    local materialFiles = {}
    local commonLevels = FS:findFiles("/levels", "*.cs\t*materials.json", -1, true, false)
    local commonVeh = FS:findFiles("/vehicles", "*.cs\t*materials.json", -1, true, false)
    local commonArt = FS:findFiles("/art", "*.cs\t*materials.json", -1, true, false)
    local commonCore = FS:findFiles("/core", "*.cs\t*materials.json", -1, true, false)
    for k,v in pairs(commonLevels) do
      table.insert(materialFiles, v)
    end
    for k,v in pairs(commonVeh) do
      table.insert(materialFiles, v)
    end
    for k,v in pairs(commonArt) do
      table.insert(materialFiles, v)
    end
    for k,v in pairs(commonCore) do
      table.insert(materialFiles, v)
    end
    for _, fn in ipairs(materialFiles) do
      local dir, basefilename, ext = path.splitWithoutExt(fn)
      if FS:fileSize(fn) > 0 then
        if string.find(fn, 'materials.cs$') then
          log('I', '', 'Loading cs material file '..fn )
          local f = io.open(fn, "r")
          if f then
            matTable[fn] = {}
            local titleS = nil
            for line in f:lines() do
              local title = line:match('%b()')
              local key = line:match("(.+)=(.+)")
              local value = line:match('%b""')
              if title then
                title = title:gsub('%(', '')
                title = title:gsub('%)', '')
                --print("title "..title)
                matTable[fn][title] = {}
                matTable[fn][title].name = title
                titleS = title
              end
              if key then
                key = key:gsub(' ', "")
                if value then
                  value = value:gsub('"', "")
                  --print("val  "..value)
                  matTable[fn][titleS][key] = value
                end
              end
            end
            f:close()
          end
        elseif string.find(fn, 'materials.json$') then
          --log('I', '', 'Loading json material file '..fn )
          matTable[fn] = jsonReadFile(fn) or {}
        end
        job.yield()
      end
    end
    --dump(matTable)
    if not tableIsEmpty(matTable) then
      log('I', '', 'parsing all materials')
      for k,v in pairs(matTable) do
        for l,b in pairs(v) do
          local mat = b
          if mat and mat.name then
            if mat.name == verifydata or mat.mapTo == verifydata or l == verifydata then
              if mat.mapTo and mat.mapTo ~= "" and mat.mapTo ~= "unmapped_mat" then maplist[mat.mapTo] = true end
              if not duplicatelist[k] then duplicatelist[k] = {} end
              if duplicatelist[k] then
                if not duplicatelist[k][l] then duplicatelist[k][l] = {} end
                if duplicatelist[k][l] then duplicatelist[k][l] = b end
              end
            end
          elseif mat and not mat.name then
            log('W', '', 'Corrupted or incompatible material found '..k)
          end
          job.yield()
        end
      end
      for k,v in pairs(matTable) do
        for l,b in pairs(v) do
          local mat = b
          if mat and mat.name then
            if mat.mapTo and mat.mapTo ~= "" and mat.mapTo ~= "unmapped_mat" then
              if maplist[mat.mapTo] then
                if not duplicatelist[k] then duplicatelist[k] = {} end
                if duplicatelist[k] then
                  if not duplicatelist[k][l] then duplicatelist[k][l] = {} end
                  if duplicatelist[k][l] then duplicatelist[k][l] = b end
                end
              end
            end
          elseif mat and not mat.name then
            log('W', '', 'Corrupted or incompatible material found '..k)
          end
          job.yield()
        end
      end
      --dumpz(duplicatelist)
    end
    extensions.editor_resourceChecker.updateDuplicateTable(duplicatelist)
  end
end

local function duplicateData(material)
  duplicateDataworkJob = extensions.core_jobsystem.create(duplicateDatawork, 1, material)
end


local removeDummyworkJob

local function removeDummywork(job, convertdata, skipCommon)
  local luaType = type
  local isDone
  local verifydata = convertdata
  local type = 9
  local matTable = {}
  local resultTable = {}
  local count = 0

  job.progress = 0
  job.sleep(0.001)
  job.stop = nil

  if not verifydata then
    log('E', '', 'There is no material path' )
    isDone = 2
  elseif not string.match(verifydata, "/") then
    log('E', '', 'Incorrect path' )
    isDone = 2
  else
    log('I', '', 'Checking material files' )
    --V2, shortcode much more efficient, checks all types of files at once
    --we have to check for common art too...
    local materialFiles = FS:findFiles(verifydata, "*materials.json", -1, true, false)
    if skipCommon == false then
      local commonVeh = FS:findFiles("/vehicles/common", "*materials.json", -1, true, false)
      for k,v in pairs(commonVeh) do
        table.insert(materialFiles, v)
      end
    end
    job.progress = 20
    job.sleep(0.001)
    local dummyMat = {}
    for _, fn in ipairs(materialFiles) do
      if FS:fileSize(fn) > 0 then
        if string.find(fn, 'materials.json$') then
          matTable[fn] = jsonReadFile(fn) or {}
        end
        job.yield()
      end
    end
    job.progress = 50
    job.sleep(0.001)
    --dump(matTable)
    if not tableIsEmpty(matTable) then
      log('I', '', 'parsing all materials')
      for k,v in pairs(matTable) do
        for l,b in pairs(v) do
          local mat = b
          if mat and mat.name then
            if not mat.Stages or tableIsEmpty(mat.Stages) or tableIsEmpty(mat.Stages[1]) then
              count = count + 1
              log('I', '', 'Found dummy material: '..mat.name.. ' in: '..k)
              if not dummyMat[k] then dummyMat[k] = {} end
              if dummyMat[k] then
                dummyMat[k][l] = true
              end
            end
          elseif mat and not mat.name then
            log('W', '', 'Corrupted or incompatible material found '..k)
          end
          job.yield()
          if job.stop == true then
            do return end
          end
        end
      end
      log('I', '', 'Found: '.. count ..' dummy materials')
    end
    job.progress = 85
    job.sleep(0.001)
    for k,v in pairs(dummyMat) do
      if FS:fileExists(k) then
        local materialFile = jsonReadFile(k) or {}
        for l,b in pairs(v) do
          if materialFile[l] then materialFile[l] = nil end
          table.insert(resultTable, l.. ' in: '..k)
        end
        log('I', '', 'Saved materials to '..k )
        jsonWriteFile(k, materialFile, true)
        job.yield()
        if job.stop == true then
          do return end
        end
      end
    end
    if job.stop == true then
      do return end
    end
    table.sort(resultTable, function(a,b) return string.upper(a) < string.upper(b) end)
    job.progress = 100
    job.sleep(0.001)
    isDone = 1
  end
  local data = {type, count, "dummy", resultTable, isDone}
  extensions.editor_resourceChecker.jobData(2, data)
end

local function removeDummy(convertdata, skipCommon)
  removeDummyworkJob = extensions.core_jobsystem.create(removeDummywork, 1, convertdata, skipCommon)
end

local textureExporterworkJob

local function textureExporterwork(job, convertdata, exportpath)
  local isDone
  local verifydata = convertdata
  local type = 10
  local resultTable = {}
  local count = 0
  job.progress = 0
  job.sleep(0.001)
  if not verifydata then
    log('E', '', 'There is no material path' )
    isDone = 2
  elseif not string.match(verifydata, "/") then
    log('E', '', 'Incorrect path' )
    isDone = 2
  else
    log('I', '', 'Exporting textures to PNG' )

    --V2, shortcode much more efficient, checks all types of files at once
    local meshFiles = FS:findFiles(verifydata, "*.dds", -1, true, false)
    for k,v in ipairs(meshFiles) do
      if job.progress < 98 then
        job.progress = job.progress + 0.1
      end
      job.yield()
      if job.stop == true then
        do return end
      end
      if v and FS:fileExists(v:gsub('.dds', '.png')) == false then
        local dir, basefilename, ext = path.splitWithoutExt(v)
        local filepathIn = v
        local filepath = exportpath..dir..basefilename..".png"
        if not convertDDSToPNG(filepathIn, filepath) then
          log('E', 'Unable to convert dds to png: ' .. tostring(filepathIn))
        end
        log('I', 'Converted dds to png: ' .. tostring(filepath))
        table.insert(resultTable, filepath)
        count = count + 1
      else
        log('I', 'PNG file already exists: ' .. tostring(v:gsub('.dds', '.png')))
      end
    end
    table.sort(resultTable, function(a,b) return string.upper(a) < string.upper(b) end)
    job.progress = 100
    job.sleep(0.001)
    isDone = 1
  end
  local data = {type, count, "dummy", resultTable, isDone}
  extensions.editor_resourceChecker.jobData(2, data)
end

local function textureExporter(convertdata, exportpath)
  textureExporterworkJob = extensions.core_jobsystem.create(textureExporterwork, 1, convertdata, exportpath)
end

--interface
local function getProgress()
  if verifyVersionworkJob and verifyVersionworkJob.running then
    return verifyVersionworkJob.progress
  end
  if verifyDuplicateworkJob and verifyDuplicateworkJob.running then
    return verifyDuplicateworkJob.progress
  end
  if fixPIDworkJob and fixPIDworkJob.running then
    return fixPIDworkJob.progress
  end
  if checkMatTexworkJob and checkMatTexworkJob.running then
    return checkMatTexworkJob.progress
  end
  if checkTexworkJob and checkTexworkJob.running then
    return checkTexworkJob.progress
  end
  if checkmissingMatsworkJob and checkmissingMatsworkJob.running then
    return checkmissingMatsworkJob.progress
  end
  if checkStaticworkJob and checkStaticworkJob.running then
    return checkStaticworkJob.progress
  end
  if checkForestworkJob and checkForestworkJob.running then
    return checkForestworkJob.progress
  end
  if checkTerrainsworkJob and checkTerrainsworkJob.running then
    return checkTerrainsworkJob.progress
  end
  if checkUnusedMatsworkJob and checkUnusedMatsworkJob.running then
    return checkUnusedMatsworkJob.progress
  end
  if checkUsedMatsworkJob and checkUsedMatsworkJob.running then
    return checkUsedMatsworkJob.progress
  end
  if checkUnusedModelsworkJob and checkUnusedModelsworkJob.running then
    return checkUnusedModelsworkJob.progress
  end
  if unusedTexturesworkJob and unusedTexturesworkJob.running then
    return unusedTexturesworkJob.progress
  end
  if checkColDataworkJob and checkColDataworkJob.running then
    return checkColDataworkJob.progress
  end
  if removeUnusedworkJob and removeUnusedworkJob.running then
    return removeUnusedworkJob.progress
  end
  if removeDummyworkJob and removeDummyworkJob.running then
    return removeDummyworkJob.progress
  end
  if textureExporterworkJob and textureExporterworkJob.running then
    return textureExporterworkJob.progress
  end
end

local function stopProgress()
  if verifyVersionworkJob and verifyVersionworkJob.running then
    verifyVersionworkJob.stop = true
  end
  if verifyDuplicateworkJob and verifyDuplicateworkJob.running then
    verifyDuplicateworkJob.stop = true
  end
  if fixPIDworkJob and fixPIDworkJob.running then
    fixPIDworkJob.stop = true
  end
  if checkMatTexworkJob and checkMatTexworkJob.running then
    checkMatTexworkJob.stop = true
  end
  if checkTexworkJob and checkTexworkJob.running then
    checkTexworkJob.stop = true
  end
  if checkmissingMatsworkJob and checkmissingMatsworkJob.running then
    checkmissingMatsworkJob.stop = true
  end
  if checkStaticworkJob and checkStaticworkJob.running then
    checkStaticworkJob.stop = true
  end
  if checkForestworkJob and checkForestworkJob.running then
    checkForestworkJob.stop = true
  end
  if checkTerrainsworkJob and checkTerrainsworkJob.running then
    checkTerrainsworkJob.stop = true
  end
  if checkUnusedMatsworkJob and checkUnusedMatsworkJob.running then
    checkUnusedMatsworkJob.stop = true
  end
  if checkUsedMatsworkJob and checkUsedMatsworkJob.running then
    checkUsedMatsworkJob.stop = true
  end
  if checkUnusedModelsworkJob and checkUnusedModelsworkJob.running then
    checkUnusedModelsworkJob.stop = true
  end
  if unusedTexturesworkJob and unusedTexturesworkJob.running then
    unusedTexturesworkJob.stop = true
  end
  if checkColDataworkJob and checkColDataworkJob.running then
    checkColDataworkJob.stop = true
  end
  if removeUnusedworkJob and removeUnusedworkJob.running then
    removeUnusedworkJob.stop = true
  end
  if removeDummyworkJob and removeDummyworkJob.running then
    removeDummyworkJob.stop = true
  end
  if textureExporterworkJob and textureExporterworkJob.running then
    textureExporterworkJob.stop = true
  end
end

local function onExtensionLoaded()
end

-- interface
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.getSimObjects = getSimObjects
M.resaveMaterial = resaveMaterial
M.powerOfTwo = powerOfTwo
M.removeFromForestJson = removeFromForestJson
M.findDuplicates = findDuplicates
M.getMaterialTexFields = getMaterialTexFields
M.verifyVersion = verifyVersion
M.verifyDuplicate = verifyDuplicate
M.fixPID = fixPID
M.checkMatTex = checkMatTex
M.checkTex = checkTex
M.checkmissingMats = checkmissingMats
M.checkStatic = checkStatic
M.checkForest = checkForest
M.checkTerrains = checkTerrains
M.matstoRemove = matstoRemove
M.checkUnusedMatsworkJob = checkUnusedMatsworkJob
M.checkUnusedMats = checkUnusedMats
M.checkUsedMatsworkJob = checkUsedMatsworkJob
M.checkUsedMats = checkUsedMats
M.shapestoRemove = shapestoRemove
M.checkUnusedModelsworkJob = checkUnusedModelsworkJob
M.checkUnusedModels = checkUnusedModels
M.textoRemove = textoRemove
M.unusedTexturesworkJob = unusedTexturesworkJob
M.unusedTextures = unusedTextures
M.removeUnused = removeUnused
M.checkColDataworkJob = checkColDataworkJob
M.checkColData = checkColData
M.duplicateData = duplicateData
M.duplicateDataworkJob = duplicateDataworkJob
M.removeDummyworkJob = removeDummyworkJob
M.removeDummy = removeDummy
M.textureExporterworkJob = textureExporterworkJob
M.textureExporter = textureExporter
M.getProgress = getProgress
M.stopProgress = stopProgress

return M