-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'In Zone'
C.description = 'Detects if a veh is in the zone'
C.color = ui_flowgraph_editor.nodeColors.sites
C.category = 'repeat_instant'
C.pinSchema = {
    { dir = 'in', type = 'number', name = 'vehId', description = "Veh id."},
    { dir = 'in', type = 'table', name = 'sitesData', description = 'Sites data'},
    { dir = 'out', type = 'bool', name = 'inside', description = 'Is inside'},
}

function C:work(args)
    if self.pinIn.flow.value then
        if not self.pinIn.sitesData.value then return end

        local isInside = true

        local sites = self.pinIn.sitesData.value
        local veh = scenetree.findObjectById(self.pinIn.vehId.value)
        local oobb = veh:getSpawnWorldOOBB()
        
        for i = 0, 8 do 
          local test = oobb:getPoint(0)
          local zones = sites:getZonesForPosition(test)
          if #zones == 0 then
            isInside = false
          end
        end

        self.red[2] = math.abs(math.sin(Engine.Platform.getRuntime()*2))
        self.red[3] = self.red[2]
        for _, zone in pairs(sites.zones.objects) do
          zone:drawDebug(nil, not isInside and self.red or self.white, 2, -0.5, isInside)
        end
        self.pinOut.inside.value = isInside
    end
end

function C:_executionStarted()
    self.red = {1,0,0}
    self.white = {1,1,1}
end

return _flowgraph_createNode(C)
