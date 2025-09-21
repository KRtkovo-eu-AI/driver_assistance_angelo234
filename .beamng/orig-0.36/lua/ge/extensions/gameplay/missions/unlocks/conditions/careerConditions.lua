-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.branchLevel = {
  info = 'The user has to have a certain level in a career branch.',
  editorFunction = "displayBranchLevel",
  getLabel = function(self) return {txt = "missions.missions.unlock.attributeLevel.atLeast", context = {branchName = career_career and career_branches.getBranchByPath(self.branchId).name or self.branchId, level = self.level}} end,
  conditionMet = function(self) return ((not career_career) or (not career_career.isActive()) or (not career_branches)) or career_branches.getBranchLevelByPath(self.branchId) >= self.level end
}

M.league = {
  info = "The user has to have a league unlocked.",
  editorFunction = "displayLeague",
  getLabel  = function(self) return "League" end,
  conditionMet = function(self)
    if not career_career.isActive() or not career_modules_branches_leagues then return false end
    local leagueUnlocked = career_modules_branches_leagues.isLeagueUnlocked(self.leagueId)
    return leagueUnlocked
  end
}

return M