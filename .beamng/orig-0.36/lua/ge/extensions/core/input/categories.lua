-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

return {
  vehicle_specific = { order = 1, icon = "local_taxi",     title = "ui.options.controls.bindings.VehicleSpecific" },
  vehicle_specific_unusable = { order = 1.5, icon = "local_taxi",     title = "ui.options.controls.bindings.VehicleSpecificUnusable" },
  vehicle          = { order = 2, icon = "directions_car", title = "ui.options.controls.bindings.Vehicle" },
  general          = { order = 3, icon = "language",       title = "ui.options.controls.bindings.General" },
  gameplay         = { order = 4, icon = "extension",      title = "ui.options.controls.bindings.Gameplay" },
  camera           = { order = 5, icon = "videocam",       title = "ui.options.controls.bindings.Camera" },
  menu             = { order = 6, icon = "web",            title = "ui.options.controls.bindings.MenuNavigation" },
  menuExtra        = { order = 6.5,icon = "web",           title = "ui.options.controls.bindings.MenuNavigationExtra" },
  slowmotion       = { order = 7, icon = "timer",          title = "ui.options.controls.bindings.SlowMotion" },
  replay           = { order = 8, icon = "local_movies",   title = "ui.options.controls.bindings.Replay" },
  editor           = { order = 9, icon = "editor",         title = "ui.options.controls.bindings.Editor" },
  flowgraph        = { order =10, icon = "editor",         title = "Flowgraph Editor" },
  debug            = { order =11, icon = "bug_report",     title = "ui.options.controls.bindings.GeneralDebug" },
  vehicle_debug    = { order =12, icon = "settings",       title = "ui.options.controls.bindings.VehicleDebug" },
}
