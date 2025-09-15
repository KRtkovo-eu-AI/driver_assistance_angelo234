-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- from core/scripts/gui/cursors.cs
local platform = getConsoleVariable("$platform")
local cursor = createObject("GuiCursor")
cursor.renderOffset = Point2F(0, 0)
if platform == "macos" then
  cursor.hotSpot = Point2I(4, 4)
  cursor.bitmapName = "/core/art/gui/images/macCursor"
else
  cursor.hotSpot = Point2I(1, 1)
  cursor.bitmapName = "/core/art/gui/images/defaultCursor.png"
end
cursor:registerObject("DefaultCursor")

-- from /core/art/gui/profiles.cs
-- GuiDefaultProfile
local guiDefaultProfile = createObject("GuiControlProfile")
guiDefaultProfile.tab = false
guiDefaultProfile.canKeyFocus = false
guiDefaultProfile.hasBitmapArray = false
guiDefaultProfile.mouseOverSelected = false
-- fill color
guiDefaultProfile.opaque = false
guiDefaultProfile.fillColor     = ColorI(242, 241, 240, 255)
guiDefaultProfile.fillColorHL   = ColorI(228, 228, 235, 255)
guiDefaultProfile.fillColorSEL  = ColorI( 98, 100, 137, 255)
guiDefaultProfile.fillColorNA   = ColorI(255, 255, 255, 255)
-- border color
guiDefaultProfile.border = 0
guiDefaultProfile.borderColor   = ColorI(100, 100, 100, 255)
guiDefaultProfile.borderColorHL = ColorI( 50,  50,  50, 50)
guiDefaultProfile.borderColorNA = ColorI( 75,  75,  75, 255)
-- font
guiDefaultProfile.fontType = "Arial"
guiDefaultProfile.fontSize = 14
guiDefaultProfile.fontCharset = ANSI
guiDefaultProfile.fontColor     = ColorI(0, 0, 0, 255)
guiDefaultProfile.fontColorHL   = ColorI(0, 0, 0, 255)
guiDefaultProfile.fontColorNA   = ColorI(0, 0, 0, 255)
guiDefaultProfile.fontColorSEL  = ColorI(255, 255, 255, 255)
-- bitmap information
guiDefaultProfile.bitmap = ""
guiDefaultProfile.bitmapBase = ""
guiDefaultProfile.textOffset = Point2I(0, 0)
-- used by guiTextControl
guiDefaultProfile.modal           = true
guiDefaultProfile.justify         = 0 -- AlignmentType enum in C++, 0 = LeftJustify
guiDefaultProfile.autoSizeWidth   = false
guiDefaultProfile.autoSizeHeight  = false
guiDefaultProfile.returnTab       = false
guiDefaultProfile.numbersOnly     = false
guiDefaultProfile.cursorColor   = ColorI(0, 0, 0, 255)
guiDefaultProfile:registerObject("GuiDefaultProfile")

-- GuiSolidDefaultProfile
local guiSolidDefaultProfile = scenetree.findObject("GuiSolidDefaultProfile")
if not guiSolidDefaultProfile then
guiSolidDefaultProfile = createObject("GuiControlProfile")
guiSolidDefaultProfile.opaque = true
guiSolidDefaultProfile.border = 1
guiSolidDefaultProfile.category = "Core"
guiSolidDefaultProfile:registerObject("GuiSolidDefaultProfile")
end

local guiTransparentProfile = scenetree.findObject("GuiTransparentProfile")
if not guiTransparentProfile then
guiTransparentProfile = createObject("GuiControlProfile")
guiTransparentProfile.opaque = false
guiTransparentProfile.border = 0
guiTransparentProfile.category = "Core"
guiTransparentProfile:registerObject("GuiTransparentProfile")
end

local guiGroupBorderProfile = scenetree.findObject("GuiGroupBorderProfile")
if not guiGroupBorderProfile then
guiGroupBorderProfile = createObject("GuiControlProfile")
guiGroupBorderProfile.border = 0
guiGroupBorderProfile.opaque = false
guiGroupBorderProfile.hasBitmapArray = true
guiGroupBorderProfile.bitmap = "./images/group-border"
guiGroupBorderProfile.category = "Core"
guiGroupBorderProfile:registerObject("GuiGroupBorderProfile")
end

local guiTabBorderProfile = scenetree.findObject("GuiTabBorderProfile")
if not guiTabBorderProfile then
guiTabBorderProfile = createObject("GuiControlProfile")
guiTabBorderProfile.border = 0
guiTabBorderProfile.opaque = false
guiTabBorderProfile.hasBitmapArray = true
guiTabBorderProfile.bitmap = "./images/tab-border"
guiTabBorderProfile.category = "Core"
guiTabBorderProfile:registerObject("GuiTabBorderProfile")
end

-- GuiToolTipProfile
local guiToolTipProfile = createObject("GuiControlProfile")
guiToolTipProfile.fillColor   = ColorI(239, 237, 222, 255)
guiToolTipProfile.borderColor = ColorI(138, 134, 122, 255)
guiToolTipProfile.fontType    = "Arial"
guiToolTipProfile.fontSize    = 14
guiToolTipProfile.fontColor   = ColorI(0, 0, 0, 255)
guiToolTipProfile.category    = "Core"
guiToolTipProfile:registerObject("GuiToolTipProfile")

-- GuiModelessDialogProfile
local guiModelessDialogProfile = createObject("GuiControlProfile")
guiModelessDialogProfile.modal = false
guiModelessDialogProfile.category = "Core"
guiModelessDialogProfile:registerObject("GuiModelessDialogProfile")

-- GuiFrameSetProfile
local guiFrameSetProfile = createObject("GuiControlProfile")
guiFrameSetProfile.fillcolor = ColorI(255, 255, 255, 255)
guiFrameSetProfile.borderColor = ColorI(246, 245, 244, 255)
guiFrameSetProfile.border = 1
guiFrameSetProfile.opaque = true
guiFrameSetProfile.category = "Core"
guiFrameSetProfile:registerObject("GuiFrameSetProfile")

-- GuiWindowProfile
local guiWindowProfile = createObject("GuiControlProfile")
guiWindowProfile.opaque = false
guiWindowProfile.border = 2
guiWindowProfile.fillColor = ColorI(242, 241, 240, 255)
guiWindowProfile.fillColorHL = ColorI(221, 221, 221, 255)
guiWindowProfile.fillColorNA = ColorI(200, 200, 200, 255)
guiWindowProfile.fontColor = ColorI(50, 50, 50, 255)
guiWindowProfile.fontColorHL = ColorI(0, 0, 0, 255)
guiWindowProfile.bevelColorHL = ColorI(255, 255, 255, 255)
guiWindowProfile.bevelColorLL = ColorI(0, 0, 0, 255)
guiWindowProfile.text = "untitled"
guiWindowProfile.bitmap = "./images/window"
guiWindowProfile.textOffset = Point2I(8, 4)
guiWindowProfile.hasBitmapArray = true
guiWindowProfile.justify = 0 -- AlignmentType enum in C++, 0 = LeftJustify
guiWindowProfile.category = "Core"
guiWindowProfile:registerObject("GuiWindowProfile")

-- GuiInputCtrlProfile
local guiInputCtrlProfile = createObject("GuiControlProfile")
guiInputCtrlProfile.tab = true
guiInputCtrlProfile.canKeyFocus = true
guiInputCtrlProfile.category = "Core"
guiInputCtrlProfile:registerObject("GuiInputCtrlProfile")

-- GuiTextProfile
local guiTextProfile = createObject("GuiControlProfile")
guiTextProfile.justify = 0 -- AlignmentType enum in C++, 0 = LeftJustify
guiTextProfile.fontColor = ColorI(20, 20, 20, 255)
guiTextProfile.category = "Core"
guiTextProfile:registerObject("GuiTextProfile")

local guiTextRightProfile = createObject("GuiControlProfile")
guiTextRightProfile:inheritParentFields(guiTextProfile)
guiTextRightProfile.justify = 1 -- AlignmentType enum in C++, 1 = RightJustify
guiTextRightProfile.category = "Core"
guiTextRightProfile:registerObject("GuiTextRightProfile")

-- GuiAutoSizeTextProfile
local guiAutoSizeTextProfile = createObject("GuiControlProfile")
guiAutoSizeTextProfile.fontColor = ColorI(0, 0, 0, 255)
guiAutoSizeTextProfile.autoSizeWidth = true
guiAutoSizeTextProfile.autoSizeHeight = true
guiAutoSizeTextProfile.category = "Core"
guiAutoSizeTextProfile:registerObject("GuiAutoSizeTextProfile")

--GuiMediumTextProfile
local guiMediumTextProfile = createObject("GuiControlProfile")
guiMediumTextProfile:inheritParentFields(guiTextProfile)
guiMediumTextProfile.fontSize = 24
guiMediumTextProfile.category = "Core"
guiMediumTextProfile:registerObject("GuiMediumTextProfile")

--GuiBigTextProfile
local guiBigTextProfile = createObject("GuiControlProfile")
guiBigTextProfile:inheritParentFields(guiTextProfile)
guiBigTextProfile.fontSize = 36
guiBigTextProfile.category = "Core"
guiBigTextProfile:registerObject("GuiBigTextProfile")

--GuiMLTextProfile
local guiMLTextProfile = createObject("GuiControlProfile")
guiMLTextProfile.fontColorLink = ColorI(100, 100, 100, 255)
guiMLTextProfile.fontColorLinkHL = ColorI(255, 255, 255, 255)
guiMLTextProfile.autoSizeWidth = true
guiMLTextProfile.autoSizeHeight = true
guiMLTextProfile.border = 0
guiMLTextProfile.category = "Core"
guiMLTextProfile:registerObject("GuiMLTextProfile")

-- GuiTextArrayProfile
local guiTextArrayProfile = createObject("GuiControlProfile")
guiTextArrayProfile:inheritParentFields(guiTextProfile)
guiTextArrayProfile.fontColor = ColorI(50, 50, 50, 255)
guiTextArrayProfile.fontColorHL = ColorI(0, 0, 0, 255)
guiTextArrayProfile.fontColorSEL = ColorI(0, 0, 0, 255)
guiTextArrayProfile.fillColor = ColorI(200, 200, 200, 255)
guiTextArrayProfile.fillColorHL = ColorI(228, 228, 235, 255)
guiTextArrayProfile.fillColorSEL = ColorI(200, 200, 200, 255)
guiTextArrayProfile.border = 0
guiTextArrayProfile.category = "Core"
guiTextArrayProfile:registerObject("GuiTextArrayProfile")

-- GuiTextEditProfile
local guiTextEditProfile = createObject("GuiControlProfile")
guiTextEditProfile.opaque = true
guiTextEditProfile.bitmap = "./images/textEdit"
guiTextEditProfile.hasBitmapArray = true
guiTextEditProfile.border = -2 -- fix to display textEdit img
-- guiTextEditProfile.borderWidth = "1"  // fix to display textEdit img
-- guiTextEditProfile.borderColor = "100 100 100"
guiTextEditProfile.fillColor = ColorI(242, 241, 240, 0)
guiTextEditProfile.fillColorHL = ColorI(255, 255, 255, 255)
guiTextEditProfile.fontColor = ColorI(0, 0, 0, 255)
guiTextEditProfile.fontColorHL = ColorI(255, 255, 255, 255)
guiTextEditProfile.fontColorSEL = ColorI(98, 100, 137, 255)
guiTextEditProfile.fontColorNA = ColorI(200, 200, 200, 255)
guiTextEditProfile.textOffset = Point2I(4, 2)
guiTextEditProfile.autoSizeWidth = false
guiTextEditProfile.autoSizeHeight = true
guiTextEditProfile.justify = 0 -- AlignmentType enum in C++, 0 = LeftJustify
guiTextEditProfile.tab = true
guiTextEditProfile.canKeyFocus = true
guiTextEditProfile.category = "Core"
guiTextEditProfile:registerObject("GuiTextEditProfile")

-- GuiProgressProfile
local guiProgressProfile = createObject("GuiControlProfile")
guiProgressProfile.opaque = false
guiProgressProfile.fillColor = ColorI(0, 162, 255, 200);
guiProgressProfile.border = 1
guiProgressProfile.borderColor   = ColorI(50, 50, 50, 200);
guiProgressProfile.category = "Core"
guiProgressProfile:registerObject("GuiProgressProfile")

-- GuiProgressBitmapProfile
local GuiProgressBitmapProfile = createObject("GuiControlProfile")
GuiProgressBitmapProfile.border = 0
GuiProgressBitmapProfile.hasBitmapArray = true
GuiProgressBitmapProfile.bitmap = "./images/loadingbar"
GuiProgressBitmapProfile.category = "Core"
GuiProgressBitmapProfile:registerObject("GuiProgressBitmapProfile")

-- GuiProgressTextProfile
local guiProgressTextProfile = createObject("GuiControlProfile")
guiProgressTextProfile.fontSize = 14
guiProgressTextProfile.fontType = "Arial"
guiProgressTextProfile.fontColor = ColorI(0, 0, 0, 255)
guiProgressTextProfile.justify = 2 -- AlignmentType enum in C++, 2 = CenterJustify
guiProgressTextProfile.category = "Core"
guiProgressTextProfile:registerObject("GuiProgressTextProfile")

-- GuiButtonProfile
local guiButtonProfile = createObject("GuiControlProfile")
guiButtonProfile.opaque = true
guiButtonProfile.border = 1
guiButtonProfile.fontColor = ColorI(50, 50, 50, 255)
guiButtonProfile.fontColorHL = ColorI(0, 0, 0, 255)
guiButtonProfile. fontColorNA = ColorI(200, 200, 200, 255)
-- guiButtonProfile.fontColorSEL =ColorI(0, 0, 0, 255)
guiButtonProfile.fixedExtent = false
guiButtonProfile.justify = 2 -- AlignmentType enum in C++, 2 = CenterJustify
guiButtonProfile.canKeyFocus = false
guiButtonProfile.bitmap = "./images/button"
guiButtonProfile.hasBitmapArray = false
guiButtonProfile.category = "Core"
guiButtonProfile:registerObject("GuiButtonProfile")

-- GuiMenuButtonProfile
local guiMenuButtonProfile = createObject("GuiControlProfile")
guiMenuButtonProfile.opaque = true
guiMenuButtonProfile.border = 0
guiMenuButtonProfile.fontSize = 18
guiMenuButtonProfile.fontType = "Arial Bold";
guiMenuButtonProfile.fontColor = ColorI(50, 50, 50, 255);
guiMenuButtonProfile.fontColorHL = ColorI(0, 0, 0, 255)
guiMenuButtonProfile.fontColorNA = ColorI(200, 200, 200, 255)
-- guiMenuButtonProfile.fontColorSEL = ColorI(0, 0, 0, 255)
guiMenuButtonProfile.fixedExtent = false
guiMenuButtonProfile.justify = 2 -- AlignmentType enum in C++, 2 = CenterJustify
guiMenuButtonProfile.canKeyFocus = false
guiMenuButtonProfile.bitmap = "./images/selector-button"
guiMenuButtonProfile.hasBitmapArray = false
guiMenuButtonProfile.category = "Core"
guiMenuButtonProfile:registerObject("GuiMenuButtonProfile")

-- GuiButtonTabProfile
local guiButtonTabProfile = createObject("GuiControlProfile")
guiButtonTabProfile.opaque = true
guiButtonTabProfile.border = 1
guiButtonTabProfile.fontColor = ColorI(50, 50, 50, 255)
guiButtonTabProfile.fontColorHL = ColorI(0, 0, 0, 255);
guiButtonTabProfile.fontColorNA = ColorI(0, 0, 0, 255);
guiButtonTabProfile.fixedExtent = false;
guiButtonTabProfile.justify = 2 -- AlignmentType enum in C++, 2 = CenterJustify
guiButtonTabProfile.canKeyFocus = false;
guiButtonTabProfile.bitmap = "./images/buttontab"
guiButtonTabProfile.category = "Core"
guiButtonTabProfile:registerObject("GuiButtonTabProfile")

-- GuiCheckBoxProfile
local guiCheckBoxProfile = createObject("GuiControlProfile")
guiCheckBoxProfile.opaque = false
guiCheckBoxProfile.fillColor = ColorI(232, 232, 232, 255)
guiCheckBoxProfile.border = 0
guiCheckBoxProfile.borderColor = ColorI(100, 100, 100, 255)
guiCheckBoxProfile.fontSize = 14;
guiCheckBoxProfile.fontColor = ColorI(20, 20, 20, 255)
guiCheckBoxProfile.fontColorHL = ColorI(80, 80, 80, 255)
guiCheckBoxProfile.fontColorNA = ColorI(200, 200, 200, 255)
guiCheckBoxProfile.fixedExtent = true
guiCheckBoxProfile.justify = 0 -- AlignmentType enum in C++, 0 = LeftJustify
guiCheckBoxProfile.bitmap = "./images/checkbox"
guiCheckBoxProfile.hasBitmapArray = true
guiCheckBoxProfile.category = "Core"
guiCheckBoxProfile:registerObject("GuiCheckBoxProfile")

-- GuiScrollProfile
local guiScrollProfile = createObject("GuiControlProfile")
guiScrollProfile.opaque = true;
guiScrollProfile.fillcolor = ColorI(255, 255, 255, 255)
guiScrollProfile.fontColor = ColorI(0, 0, 0, 255)
guiScrollProfile.fontColorHL = ColorI(150, 150, 150, 255)
-- guiScrollProfile.borderColor = GuiDefaultProfile.borderColor;
guiScrollProfile.border = 1
guiScrollProfile.bitmap = "./images/scrollBar"
guiScrollProfile.hasBitmapArray = true
guiScrollProfile.category = "Core"
guiScrollProfile:registerObject("GuiScrollProfile")

-- GuiOverlayProfile
local guiOverlayProfile = createObject("GuiControlProfile")
guiOverlayProfile.opaque = true;
guiOverlayProfile.fillcolor = ColorI(255, 255, 255, 255)
guiOverlayProfile.fontColor = ColorI(0, 0, 0, 255)
guiOverlayProfile.fontColorHL = ColorI(255, 255, 255, 255)
guiOverlayProfile.fillColor = ColorI(0, 0, 0, 100)
guiOverlayProfile.category = "Core"
guiOverlayProfile:registerObject("GuiOverlayProfile")

-- GuiSliderProfile
local guiSliderProfile = createObject("GuiControlProfile")
guiSliderProfile.bitmap = "./images/slider"
guiSliderProfile.category = "Core"
guiSliderProfile:registerObject("GuiSliderProfile")

-- GuiSliderBoxProfile
local guiSliderBoxProfile = createObject("GuiControlProfile")
guiSliderBoxProfile.bitmap = "./images/slider-w-box"
guiSliderBoxProfile.category = "Core"
guiSliderBoxProfile:registerObject("GuiSliderBoxProfile")

-- GuiPopupMenuItemBorder
local guiPopupMenuItemBorder = createObject("GuiControlProfile")
guiPopupMenuItemBorder:inheritParentFields(guiButtonProfile)
guiPopupMenuItemBorder.opaque = true
guiPopupMenuItemBorder.border = 1
guiPopupMenuItemBorder.fontColor = ColorI(0, 0, 0, 255)
guiPopupMenuItemBorder.fontColorHL = ColorI(0, 0, 0, 255)
guiPopupMenuItemBorder.fontColorNA = ColorI(255, 255, 255, 255)
guiPopupMenuItemBorder.fixedExtent = false
guiPopupMenuItemBorder.justify = 2 -- AlignmentType enum in C++, 2 = CenterJustify
guiPopupMenuItemBorder.canKeyFocus = false
guiPopupMenuItemBorder.bitmap = "./images/button"
guiPopupMenuItemBorder.category = "Core"
guiPopupMenuItemBorder:registerObject("GuiPopupMenuItemBorder")

-- GuiPopUpMenuDefault
local guiPopUpMenuDefault = createObject("GuiControlProfile")
guiPopUpMenuDefault:inheritParentFields(guiDefaultProfile)
guiPopUpMenuDefault.opaque = true
guiPopUpMenuDefault.mouseOverSelected = true
guiPopUpMenuDefault.textOffset = Point2I(3, 3)
guiPopUpMenuDefault.border = 0
guiPopUpMenuDefault.borderThickness = 0
guiPopUpMenuDefault.fixedExtent = true
guiPopUpMenuDefault.bitmap = "./images/scrollbar"
guiPopUpMenuDefault.hasBitmapArray = true
guiPopUpMenuDefault.profileForChildren = "GuiPopupMenuItemBorder"
guiPopUpMenuDefault.fillColor = ColorI(242, 241, 240, 255)
guiPopUpMenuDefault.fillColorHL = ColorI(228, 228, 235, 255)
guiPopUpMenuDefault.fillColorSEL = ColorI(98, 100, 137, 255)
-- font color is black
guiPopUpMenuDefault.fontColorHL = ColorI(0, 0, 0, 255)
guiPopUpMenuDefault.fontColorSEL = ColorI(255, 255, 255, 255)
guiPopUpMenuDefault.borderColor = ColorI(100, 100, 100, 255)
guiPopUpMenuDefault.category = "Core";
guiPopUpMenuDefault:registerObject("GuiPopUpMenuDefault")

-- GuiPopUpMenuProfile
local guiPopUpMenuProfile = createObject("GuiControlProfile")
guiPopUpMenuProfile:inheritParentFields(guiPopUpMenuDefault)
guiPopUpMenuProfile.textOffset         = Point2I(6, 4)
guiPopUpMenuProfile.bitmap             = "./images/dropDown";
guiPopUpMenuProfile.hasBitmapArray     = true
guiPopUpMenuProfile.border             = 1
guiPopUpMenuProfile.profileForChildren = "GuiPopUpMenuDefault"
guiPopUpMenuProfile.category = "Core"
guiPopUpMenuProfile:registerObject("GuiPopUpMenuProfile")

-- GuiTabBookProfile
local guiTabBookProfile = createObject("GuiControlProfile")
guiTabBookProfile.fillColorHL = ColorI(100, 100, 100, 255)
guiTabBookProfile.fillColorNA = ColorI(150, 150, 150, 255)
guiTabBookProfile.fontColor = ColorI(30, 30, 30, 255)
guiTabBookProfile.fontColorHL = ColorI(0, 0, 0, 255)
guiTabBookProfile.fontColorNA = ColorI(50, 50, 50, 255)
guiTabBookProfile.fontType = "Arial"
guiTabBookProfile.fontSize = 14
guiTabBookProfile.justify = 2 -- AlignmentType enum in C++, 2 = CenterJustify
guiTabBookProfile.bitmap = "./images/tab"
guiTabBookProfile.tabWidth = 64
guiTabBookProfile.tabHeight = 24
guiTabBookProfile.tabPosition = 0 -- 0 = GuiTabBookCtrl::TabPosition::AlignTop
guiTabBookProfile.textOffset = Point2I(0, -3)
guiTabBookProfile.tab = true
guiTabBookProfile.cankeyfocus = true
guiTabBookProfile.category = "Core"
guiTabBookProfile:registerObject("GuiTabBookProfile")

-- GuiTabPageProfile
local guiTabPageProfile = createObject("GuiControlProfile")
guiTabPageProfile:inheritParentFields(guiDefaultProfile)
guiTabPageProfile.fontType = "Arial"
guiTabPageProfile.fontSize = 10
guiTabPageProfile.justify  = 2 -- AlignmentType enum in C++, 2 = CenterJustify
guiTabPageProfile.bitmap = "./images/tab"
guiTabPageProfile.opaque = false
guiTabPageProfile.fillColor = ColorI(240, 239, 238, 255)
guiTabPageProfile.category = "Core"
guiTabPageProfile:registerObject("GuiTabPageProfile")

-- GuiConsoleProfile
local guiConsoleProfile = createObject("GuiControlProfile")
if platform == "macos" then
  guiConsoleProfile.fontType = "Monaco"
  guiConsoleProfile.fontSize = 13
else
  guiConsoleProfile.fontType = "Lucida Console"
  guiConsoleProfile.fontSize = 12
end
guiConsoleProfile.fontColor = ColorI(255, 255, 255, 255)
guiConsoleProfile.fontColorHL = ColorI(0, 255, 255, 255)
guiConsoleProfile.fontColorNA = ColorI(255, 0, 0, 255)
guiConsoleProfile:setField("fontColors", 6, "100 100 100 255")
guiConsoleProfile:setField("fontColors", 7, "100 100 0 255")
guiConsoleProfile:setField("fontColors", 8, "0 0 100 255")
guiConsoleProfile:setField("fontColors", 9, "0 100 0 255")
guiConsoleProfile.category = "Core"
guiConsoleProfile:registerObject("GuiConsoleProfile")

-- GuiConsoleTextProfile
local guiConsoleTextProfile = createObject("GuiControlProfile")
guiConsoleTextProfile.fontColor = ColorI(0, 0, 0, 255)
guiConsoleTextProfile.autoSizeWidth = true
guiConsoleTextProfile.autoSizeHeight = true
guiConsoleTextProfile.textOffset = Point2I(2, 2)
guiConsoleTextProfile.opaque = true
guiConsoleTextProfile.fillColor = ColorI(255, 255, 255, 255)
guiConsoleTextProfile.border = 1
guiConsoleTextProfile.borderThickness = 1
guiConsoleTextProfile.borderColor = ColorI(0, 0, 0, 255)
guiConsoleTextProfile.category = "Core"
guiConsoleTextProfile:registerObject("GuiConsoleTextProfile")

-- ConsoleScrollProfile
local consoleScrollProfile = createObject("GuiControlProfile")
consoleScrollProfile:inheritParentFields(guiScrollProfile)
consoleScrollProfile.opaque = true
consoleScrollProfile.fillColor = ColorI(0, 0, 0, 175)
consoleScrollProfile.border = 1
-- consoleScrollProfile.borderThickness = 0;
consoleScrollProfile.borderColor = ColorI(0, 0, 0, 255)
consoleScrollProfile.category = "Core"
consoleScrollProfile:registerObject("ConsoleScrollProfile")

-- ConsoleTextEditProfile
local consoleScrollProfile = createObject("GuiControlProfile")
consoleScrollProfile:inheritParentFields(guiTextEditProfile)
consoleScrollProfile.fillColor = ColorI(242, 241, 240, 255)
consoleScrollProfile.fillColorHL = ColorI(255, 255, 255, 255)
consoleScrollProfile.category = "Core"
consoleScrollProfile:registerObject("ConsoleTextEditProfile")

-- // ----------------------------------------------------------------------------
-- // Radio button control
-- // ----------------------------------------------------------------------------

-- GuiRadioProfile
local guiRadioProfile = createObject("GuiControlProfile")
guiRadioProfile.fontSize = 14
guiRadioProfile.fillColor = ColorI(232, 232, 232, 255)
guiRadioProfile.fontColor = ColorI(20, 20, 20, 255)
guiRadioProfile.fontColorHL = ColorI(80, 80, 80, 255)
guiRadioProfile.fixedExtent = true
guiRadioProfile.bitmap = "./images/radioButton"
guiRadioProfile.hasBitmapArray = true
guiRadioProfile.category = "Core"
guiRadioProfile:registerObject("GuiRadioProfile")

-- GuiMonospace
local guiMonospace = createObject("GuiControlProfile")
guiMonospace:inheritParentFields(guiDefaultProfile)
guiMonospace.fillColor = ColorI(242, 241, 240, 200)
guiMonospace.fontType = "Courier New"
guiMonospace.opaque = true
guiMonospace.fontSize = 16
guiMonospace:registerObject("GuiMonospace")

-- GuiCEFProfile
local guiMonospace = createObject("GuiControlProfile")
guiMonospace.textOffset = Point2I(4, 2)
guiMonospace.autoSizeWidth = false
guiMonospace.autoSizeHeight = true
guiMonospace.justify = 0 -- AlignmentType enum in C++, 0 = LeftJustify
guiMonospace.tab = true
guiMonospace.canKeyFocus = true
guiMonospace.category = "Core"
guiMonospace.fontColors[0] = ColorI(255, 0, 6, 255)
guiMonospace.fontColor = ColorI(255, 0, 6, 255, 255)
guiMonospace:registerObject("GuiCEFProfile")


