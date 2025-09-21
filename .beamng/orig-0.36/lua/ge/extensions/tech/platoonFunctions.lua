-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt


local M = {}
-- M.dependecies = {'tech_platoonArch'}

local vid
local sensorId
local sensorIdRadar
local WIDTH = 100
local HEIGHT = 100
local resolution = {WIDTH, HEIGHT}
local targetSpeed
local data
local debug
local prevDistanceToCars = {}
local prevSpeed
local prevSpeed2
local prevDistanceToCars2 = {}
local zeroSpeed = 0
local speedcalc = 0
local vehicleID
local noCars
local vehiclesData= {}
local launched = false
local platoons = {}
local currentSpeed = 0
local speedIncrement = 1
local simTimeTotal = 0
local leaderMode





local function changeSpeed(speed)
  targetSpeed = speed
end


-- ----------------------------------start Platoon Architecure file replacement code-----------------------------
local function createPlatoon(leaderID, sensorID)
  -- Create a new platoon table with the leader's ID as the key
  platoons[leaderID] = {
  platoonID = leaderID,       -- Use leader's ID as platoonID
  vehicles = {leaderID},
  ultrasonics = {sensorID} }      -- The first vehicle added is the leader}
end

local function getVehicleIndex(leaderID, vehicleID)
  print("getting vehicle Index of platoon: "..leaderID.." of vehicle's"..vehicleID)
  if platoons[leaderID] then
    -- print("in if in getVehiclesIndex")
    local vehicles = platoons[leaderID].vehicles
    -- Iterate through the vehicles list to find the index of the vehicle
    for i, id in ipairs(vehicles) do
      if id == vehicleID then
        local vehicle = tostring(vehicleID) 
        be:sendToMailbox(tostring(vehicle), lpack.encodeBinWorkBuffer(i))
        return i  -- Return the index of the vehicle in the platoon
      end
    end
   
  else
    -- print("Platoon with leader ID " .. leaderID .. " does not exist in getVehicleOdx function.")
    -- return nil
  end
end

-- Function to add a vehicle to the platoon based on the leader's ID
local function addVehicleToPlatoon(leaderID, vehicleID, sensorID)
  -- Ensure the platoon with the leader ID exists
  if platoons[leaderID] then
    -- Check if the vehicleID is already the leader
    if vehicleID == leaderID then
      ui_message("This vehicle is already the leader", 5, "Tech", "forward")
    else
      -- Add the vehicle ID to the platoon
      table.insert(platoons[leaderID].vehicles, vehicleID)
      table.insert(platoons[leaderID].ultrasonics, sensorID)
    end
  else
    -- print("Platoon with leader ID " .. leaderID .. " does not exist.")
  end
  getVehicleIndex(leaderID,vehicleID)
end

local function addVehicleInPlatoon(leaderID, index, vehicleID, sensorID)
  -- Ensure the platoon with the leader ID exists
  if platoons[leaderID] then
    -- Check if the vehicleID is already the leader
    if vehicleID == leaderID then
      ui_message("This vehicle is already the leader", 5, "Tech", "forward")
    else
      -- Add the vehicle ID to the platoon
      table.insert(platoons[leaderID].vehicles, index, vehicleID)
      table.insert(platoons[leaderID].ultrasonics, index, sensorID)
    end
  else
    -- print("Platoon with leader ID " .. leaderID .. " does not exist.")
  end
  getVehicleIndex(leaderID,vehicleID)
end

local function getRelayVehiclesID(leaderID, vehicleIndex)

  vid = platoons[leaderID].vehicles[vehicleIndex]
  vehicleInfront = platoons[leaderID].vehicles[vehicleIndex-1]
  print("vehicleID infront: "..vehicleInfront)
  local mailBoxNameVehicleInfront = "vehicleInfront"..vid
  be:sendToMailbox(mailBoxNameVehicleInfront, lpack.encodeBinWorkBuffer(vehicleInfront))



end


-- Function to remove a vehicle from a platoon based on the leader's ID
local function removeVehicleFromPlatoon(vehiclesData) --problem when a vehicle is in distant --check for NFSA
  local vehiclesInfo = lpack.decode(vehiclesData)
  local leaderID = vehiclesInfo.leaderID
  local vehicleID = vehiclesInfo.vehicleID
  local isLastVehicle = false
  -- Ensure the platoon with the leader ID exists
  if platoons[leaderID] then
    for i, id in ipairs(platoons[leaderID].vehicles) do
      if id == vehicleID then
        local vehiclesList = platoons[leaderID].vehicles
        if i == #vehiclesList then
          isLastVehicle = true
        end
        extensions.tech_sensors.removeSensor(platoons[leaderID].ultrasonics[i])
        table.remove(platoons[leaderID].vehicles, i)  -- Remove the vehicle ID from the platoon
        table.remove(platoons[leaderID].ultrasonics, i) --remove the ultrasonic sensor from the list


        if isLastVehicle == false then
          for i, v in ipairs(vehiclesList) do
            if i > 1 then
              local vehicle = tostring(v) 
              getRelayVehiclesID(leaderID, i)
              be:sendToMailbox(vehicle, lpack.encodeBinWorkBuffer(i))
            end
          end 
        end
      end
    end
  else

  end
end

local function changePlatoonLeaderID(oldleaderID)
  --getting the platoon data corresponding to the current leaderID
  local platoon = platoons[oldleaderID]
  print(platoon)
  --checking if the platoon exists
  if platoon then
    --checking if the platoon has more than two vehicles
    if #platoon.vehicles > 2 then      
      local newLeader = platoons[oldleaderID].vehicles[2]
      extensions.tech_sensors.removeSensor(platoons[leaderID].ultrasonics[1])
      table.remove(platoon.vehicles,1)
      table.remove(platoon.ultrasonics,1)
      -- print("newLeader: "..newLeader)
      createPlatoon(newLeader)
      platoons[newLeader] = platoon
      platoons[oldleaderID] = nil
    else
      -- print("ending platoon for NFSA here to implement")
    end

  end
end


local function printPlatoons()
  for leaderID, platoon in pairs(platoons) do
    for i, vehicleID in ipairs(platoon.vehicles) do
    end
  end
end






-- ---------------------------------end of Platoon Architecure file replacement code--------------------------------
local function loadWithID(leaderID, vid, speed, debugFlag)
  loaded = true
  print("loading")
  local ultrasonicArgs = {updateTime = 0.1, isVisualised = true}
  local ultrasonicIDLeader = extensions.tech_sensors.createUltrasonic(leaderID,ultrasonicArgs)
  print("vehicle: "..leaderID.." ultrasonicID: "..ultrasonicIDLeader)
  createPlatoon(leaderID, ultrasonicIDLeader)
  Engine.Annotation.enable(true)
  AnnotationManager.setInstanceAnnotations(true)
  local mailBoxName = "currentLeader"..vid
  mailboxNameUltrasonic = vid.."UltrasonicReading"
  local ultrasonicID = extensions.tech_sensors.createUltrasonic(vid,ultrasonicArgs)
  be:sendToMailbox(mailBoxName, lpack.encodeBinWorkBuffer(leaderID))
  print("vehicle: "..vid.." ultrasonicID: "..ultrasonicID)
  addVehicleToPlatoon(leaderID,vid,ultrasonicID)
  vehicleIndex = getVehicleIndex(leaderID, vid)
  print("vid: "..vid.." vehicleIndex: "..vehicleIndex)
  getRelayVehiclesID(leaderID, vehicleIndex) --send it to the mailbox of the vehicle to be able to calulate the distance ahead


  ACCFunctionCall = "extensions.tech_platooning.formPlatoon("..leaderID..","..vid..","..speed..")" 
  be:queueObjectLua(vid, ACCFunctionCall)

  relaysJoin = "extensions.tech_platooning.vehicleJoined("..vid..")" 
  be:queueObjectLua(leaderID, relaysJoin)

end

local function joinWithID(leaderID, vid, speed, debugFlag)
  loaded = true
  Engine.Annotation.enable(true)
  AnnotationManager.setInstanceAnnotations(true)
  local ultrasonicArgs = {updateTime = 0.1, isVisualised = true}
  local ultrasonicID = extensions.tech_sensors.createUltrasonic(vid,ultrasonicArgs)
  print("vehicle: "..vid.." ultrasonicID: "..ultrasonicID)
  addVehicleToPlatoon(leaderID,vid,ultrasonicID)
  vehicleIndex = getVehicleIndex(leaderID, vid)
  getRelayVehiclesID(leaderID, vehicleIndex) --send it to the mailbox of the vehicle to be able to calulate the distance ahead
  local mailBoxName = "currentLeader"..vid --check will cause issue with multiple platoons
  local mailboxNameUltrasonic = vid.."UltrasonicReading"
  be:sendToMailbox(mailBoxName, lpack.encodeBinWorkBuffer(leaderID))
  printPlatoons()
  print("joining function ".."leaderID: "..leaderID.." followerID: "..vid)
  local ACCFunctionCall = "extensions.tech_platooning.joinPlatoon("..leaderID..","..vid..","..speed..")" 
  be:queueObjectLua(vid, ACCFunctionCall)

  relaysJoin = "extensions.tech_platooning.vehicleJoined("..vid..")" 
  be:queueObjectLua(leaderID, relaysJoin)
end


local function leavePlatoon(leaderID, vid)
  loaded = false
  local ACCFunctionCall = "extensions.tech_platooning.leavePlatoon("..leaderID..","..vid..")" --added leavePlatoon for NFSA
  be:queueObjectLua(vid, ACCFunctionCall)

end

local function launchPlatoon(leaderID, leaderMode, speed)
  launched = true
  print("Launched "..speed)
  local launchFunctonCall = "extensions.tech_platooning.launchPlatoon("..leaderID..","..leaderMode..","..speed..")"
  be:queueObjectLua(leaderID, launchFunctonCall)
  
end

local function endPlatoon(platoonID) --edit for NFSA
  launched = false
  local leaderID = platoonArch.getLeader(platoonID)
  local launchFunctonCall = "extensions.tech_platooning.endPlatoon("..leaderID..")"
  be:queueObjectLua(leaderID, launchFunctonCall)
  local vehiclesList = platoonArch.getRelayVehicles()
  for i, v in ipairs(vehiclesList) do
    local launchFunctonCall = "extensions.tech_platooning.endPlatoon("..v..")"
    be:queueObjectLua(v, launchFunctonCall)
  end 
  platoonArch.emptydata(platoonID)
end


--------------------ADDED for NFSA
local function leaderExitPlatoon(leaderID)
  
  local newLeaderID = platoons[leaderID].vehicles[2] --NFSA getting the new LeaderID
  print("newLeader NFSA: "..newLeaderID)
  local leaderExitFunctionCall = "extensions.tech_platooning.leaderExitPlatoon("..leaderID..")" --old leader exiting the platoon
  be:queueObjectLua(leaderID, leaderExitFunctionCall) --
  local newLeaderFunctionCall = "extensions.tech_platooning.reassignLeaderNFSA("..newLeaderID..")" --asssigning new leader to follow --FSA something is wrong here
  be:queueObjectLua(newLeaderID, newLeaderFunctionCall)
  local newLeaderFunctionCall = "extensions.tech_platooning.changeLeader("..newLeaderID..")" --asssigning new leader to follow
  be:queueObjectLua(newLeaderID, newLeaderFunctionCall)
  changePlatoonLeaderID(leaderID) -- NFSA changing the new LeaderID in the DB
  print(printPlatoons())
  local vehiclesList = platoons[newLeaderID].vehicles
  
  for i, v in ipairs(vehiclesList) do
    if i > 1 then
      local vehicle = tostring(v) 
      local mailBoxName = "currentLeader"..vehicle
      print(type(vehicle))
      be:sendToMailbox(vehicle, lpack.encodeBinWorkBuffer(i))
      local launchFunctonCall = "extensions.tech_platooning.updateLeaderToFollow("..newLeaderID..")" --changing the leader to follow for the rest of the vehicles in the platoon
      be:queueObjectLua(v, launchFunctonCall)
      be:sendToMailbox(mailBoxName, lpack.encodeBinWorkBuffer(newLeaderID))
    end
  end 
end  


local function splitPlatoon(leaderID,vehicleID)                    --vehicleID is the vehicle the user would want to pass infront of
  local platoonID = platoonArch.getPlatoonID(leaderID)
  local vehiclesList = platoonArch.getRelayVehiclesWtihID(platoonID)
  local vehicleIndex = platoonArch.getRelayVehicleIndexById(vehicleID)
  local firstPlatoonVehicles
  local secondPlatoonVehicles

  platoonArch.splitPlatoon(leaderID,vehicleID)

  
end

local function joinInBetween(leaderID, relayVehicleID, externalVehicleID, speed, debugFlag)
  loaded = true
  Engine.Annotation.enable(true)
  AnnotationManager.setInstanceAnnotations(true)
  -- platoonArch.addVehicleToPlatoon(leaderID,vid)
  local ultrasonicArgs = {updateTime = 0.1, isVisualised = true}
  local ultrasonicID = extensions.tech_sensors.createUltrasonic(externalVehicleID ,ultrasonicArgs)
  vehicleIndex = getVehicleIndex(leaderID, relayVehicleID) --get index of vehicle in platoon that we'll enter infront of
  addVehicleInPlatoon(leaderID, vehicleIndex, externalVehicleID, ultrasonicID) --change to inplatoon another function for joining in the middle
  updatedVehicleIndex = vehicleIndex + 1 --after inserting the outsider vehicle, the vehicle's omdex behind it changed
  getRelayVehiclesID(leaderID, updatedVehicleIndex) --send it to the mailbox of the vehicle already in platoon with the new vehicle's ID to be able to calulate the distance ahead
  getRelayVehiclesID(leaderID, vehicleIndex) --send to mailbox of newly added vehicle to platoon with the vehicle infront of it
  local mailBoxName = "currentLeader"..externalVehicleID --check will cause issue with multiple platoons
  local mailboxNameUltrasonic = externalVehicleID.."UltrasonicReading"
  be:sendToMailbox(mailBoxName, lpack.encodeBinWorkBuffer(leaderID))
  local ACCFunctionCall = "extensions.tech_platooning.joinPlatoon("..leaderID..","..externalVehicleID..","..speed..")" 
  be:queueObjectLua(externalVehicleID, ACCFunctionCall)
end

local function onUpdate(dtReal, dtSim, dtRaw)
  local distance 

    

  for platoonID, platoon in pairs(platoons) do  -- Loop over all platoons
    for i, vehicleID in ipairs(platoon.vehicles) do  -- Loop over vehicles (index-based)
      local sensorID = platoon.ultrasonics[i]  -- Get corresponding sensor ID
      sensorData = extensions.tech_sensors.getUltrasonicReadings(sensorID)
      for k, v in pairs(sensorData) do
        if k == "distance" then
          distance = v
          local mailboxNameUltrasonic = vehicleID.."UltrasonicReading"
          if distance <= 8 then
            be:sendToMailbox(mailboxNameUltrasonic, lpack.encodeBinWorkBuffer(distance))
          end
        end
      end
      
    end
  end
end
  
  
-- Public interface
M.onUpdate                    = onUpdate
M.onExtensionLoaded           = function() log('I', 'ACC', 'adaptiveCruiseControlWithRadar extension loaded') end
M.onExtensionUnloaded         = unload
M.leavePlatoon                = leavePlatoon
M.load                        = load
M.loadWithID                  = loadWithID
M.joinWithID                  = joinWithID
M.changeSpeed                 = changeSpeed
M.launchPlatoon               = launchPlatoon
M.endPlatoon                  = endPlatoon
M.leaderExitPlatoonByPlatoonID= leaderExitPlatoonByPlatoonID
M.leaderExitPlatoonByLeaderID = leaderExitPlatoonByLeaderID
M.leaderExitPlatoon           = leaderExitPlatoon
M.splitPlatoon                = splitPlatoon
M.createPlatoon               = createPlatoon
M.addVehicleToPlatoon         = addVehicleToPlatoon
M.removeVehicleFromPlatoon    = removeVehicleFromPlatoon
M.changePlatoonLeaderID       = changePlatoonLeaderID
M.printPlatoons               = printPlatoons
M.getVehicleIndex             = getVehicleIndex
M.joinInBetween               = joinInBetween
M.getRelayVehiclesID          = getRelayVehiclesID
M.addVehicleInPlatoon         = addVehicleInPlatoon 


return M