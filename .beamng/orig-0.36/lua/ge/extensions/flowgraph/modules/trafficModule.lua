-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
C.moduleOrder = 0 -- low first, high later

function C:init()
  self.trafficActive = false
  self.parkedCarsActive = false
  self.keepTrafficState = false
end

-- trafficActive and parkedCarsActive should only get updated if the flowgraph itself alters the traffic state
function C:updateTrafficState()
  self.trafficActive = next(gameplay_traffic.getTrafficData()) and true or false
end

function C:updateParkedCarsState()
  self.parkedCarsActive = next(gameplay_parking.getParkedCarsData()) and true or false
end

function C:insertTraffic(id)
  gameplay_traffic.insertTraffic(id)
  self:updateTrafficState()
end

function C:removeTraffic(id)
  gameplay_traffic.removeTraffic(id)
  self:updateTrafficState()
end

function C:activateTraffic(vehList)
  gameplay_traffic.activate(vehList)
  self:updateTrafficState()
end

function C:deactivateTraffic()
  if self.trafficActive then
    gameplay_traffic.deactivate()
    self:updateTrafficState()
  end
end

function C:activateParkedCars(vehList)
  gameplay_parking.processVehicles(vehList)
  self:updateParkedCarsState()
end

function C:deactivateParkedCars()
  if self.parkedCarsActive then
    gameplay_parking.processVehicles()
    self:updateParkedCarsState()
  end
end

function C:executionStopped()
  if not self.keepTrafficState then
    if self.trafficActive then
      gameplay_traffic.setTrafficVars()
      gameplay_police.setPursuitVars()
    end
    if self.parkedCarsActive then
      gameplay_parking.setParkingVars()
    end

    self:deactivateTraffic()
    self:deactivateParkedCars()
  end
  self.keepTrafficState = false
end

return _flowgraph_createModule(C)