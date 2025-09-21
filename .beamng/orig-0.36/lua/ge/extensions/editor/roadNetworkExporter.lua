-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Window->Tech->Road Network Exporter, visible with .tech license

local M = {}

local im = ui_imgui

local toolWindowName = 'RoadNetworkExporter'
local exportedFilename = nil
local osmExportedFilename = nil
local sumoExportedFilename = nil
local sumo_filename = nil

local function onWindowMenuItem()
  editor.showWindow(toolWindowName)
end

local function onEditorInitialized()
  if not ResearchVerifier.isTechLicenseVerified() then return end
  editor.registerWindow(toolWindowName, im.ImVec2(600,600))
  editor.addWindowMenuItem('Road Network Exporter', onWindowMenuItem, {groupMenuName = 'Tech'})
end

local function onEditorGui()
  if not ResearchVerifier.isTechLicenseVerified() then return end
  if editor.beginWindow(toolWindowName, 'Road Network Exporter', im.WindowFlags_NoCollapse + im.WindowFlags_AlwaysAutoResize) then

    -- OpenDrive export section
    im.Text("OpenDrive (.xodr) Export:")
    im.TextWrapped("Exports current level road network as an OpenDrive .xodr file.\nBeamNG might look frozen while the export is in progress.")

    if not levelLoaded then
      im.Text("No level currently loaded.")
    else
      if im.Button("Export OpenDrive '" .. getCurrentLevelIdentifier() .. "'...") then
        exportedFilename = nil
        extensions.editor_fileDialog.saveFile(
          function(data)
            local filename = data.filepath
            extensions.tech_openDriveExporter.export(filename:sub(1, -(#".xodr" + 1)))
            exportedFilename = FS:virtual2Native(filename)
          end,
          {{"OpenDrive",".xodr"}},
          false,
          "/",
          "File already exists.\nDo you want to overwrite the file?"
        )
      end
      if exportedFilename ~= nil then
        im.Text("Exported to '" .. exportedFilename .. "'.")
      end
    end

    im.Separator()

    -- OpenStreetMap export section
    im.Text("OpenStreetMap (.osm) Export:")
    im.TextWrapped("Exports current level road network as an OpenStreetMap .osm file.\nBeamNG might look frozen while the export is in progress.")

    if not levelLoaded then
      im.Text("No level currently loaded.")
    else
      if im.Button("Export OpenStreetMap '" .. getCurrentLevelIdentifier() .. "'...") then
        osmExportedFilename = nil
        extensions.editor_fileDialog.saveFile(
          function(data)
            local filename = data.filepath
            extensions.tech_openStreetMapExporter.export(filename:sub(1, -(#".osm" + 1)))
            osmExportedFilename = FS:virtual2Native(filename)
          end,
          {{"OpenStreetMap",".osm"}},
          false,
          "/",
          "File already exists.\nDo you want to overwrite the file?"
        )
      end
      if osmExportedFilename ~= nil then
        im.Text("Exported to '" .. osmExportedFilename .. "'.")
      end
    end

    im.Separator()


    -- OpenStreetMap export section
    im.Text("SUMO (.xml) Export:")
    im.TextWrapped("Exports current level road network as an SUMO .xml file.\nBeamNG might look frozen while the export is in progress.")

    if not levelLoaded then
      im.Text("No level currently loaded.")
    else
      if im.Button("Export SUMO '" .. getCurrentLevelIdentifier() .. "'...") then
        sumoExportedFilename = nil
        extensions.editor_fileDialog.saveFile(
          function(data)
            local filename = data.filepath
            sumo_filename = filename
            extensions.tech_sumoExporter.export(sumo_filename:sub(1, -(#".xml" + 1)))
            sumoExportedFilename = FS:virtual2Native(sumo_filename)
          end,
          {{"SUMO",".xml"}},
          false,
          "/",
          "File already exists.\nDo you want to overwrite the file?"
        )
      end
      if sumoExportedFilename ~= nil then
        im.Text("Exported files:")
        im.Text("  " .. sumo_filename .. ".edg.xml")
        im.Text("  " .. sumo_filename .. ".nod.xml")
      end
    end


  end
  editor.endWindow()
end


M.onEditorInitialized = onEditorInitialized
M.onEditorGui = onEditorGui

return M
