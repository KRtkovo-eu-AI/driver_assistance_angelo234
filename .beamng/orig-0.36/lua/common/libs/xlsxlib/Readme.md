# XLSX Parsing Module for Lua

This module provides functionality to parse XLSX files in Lua. It allows you to read data from Excel sheets, including handling shared strings, cell references, and ranges.

## Features

- **Parse XLSX Files**: Load and parse `.xlsx` files to extract data.
- **Handle Shared Strings**: Correctly interpret shared strings used in Excel files.
- **Cell Reference Parsing**: Supports parsing cell references like `A1`, `B2`, `$C$6`.
- **Range Extraction**: Extract data from specific ranges like `A1:C3`, `B2`, or entire rows/columns.
- **Sheet Access**: Retrieve data from specific sheets by name.

## Dependencies

- **[slaxdom](https://github.com/Phrogz/SLAXML)**: A pure Lua SAX-like streaming XML parser with DOM support.
- **`ZipArchive` Class**: A Lua class or library capable of reading ZIP files.

## Installation

1. **Copy the Module**

   Save the provided Lua code into a file, e.g., `xlsxParser.lua`, and place it in your project directory.

2. **Install Dependencies**

   - **slaxdom**: Ensure `slaxdom.lua` is available in your Lua module path. You can download it from [SLAXML GitHub Repository](https://github.com/Phrogz/SLAXML).
   - **ZipArchive**: You need a Lua library that provides ZIP archive functionality. One common choice is [LuaZip](https://keplerproject.github.io/luazip/).

## Usage

### Loading an XLSX File

```lua
local xlsxParser = require('xlsxParser')

-- Load the XLSX file
local xlsx = xlsxParser.loadFileXLSX('path/to/your/file.xlsx')
```

### Retrieving Data from a Sheet

```lua
-- Get data from a specific sheet by name
local sheetName = 'Sheet1'
local data = xlsxParser.getSheetData(xlsx, sheetName)

-- Print the data
for rowIndex, row in ipairs(data) do
  for colIndex, value in ipairs(row) do
    print(string.format("Row %d, Col %d: %s", rowIndex, colIndex, tostring(value)))
  end
end
```

### Retrieving Data from a Specific Range

```lua
-- Specify a range in Excel notation (e.g., "A1:C3")
local range = 'A1:C3'
local dataInRange = xlsxParser.getSheetData(xlsx, sheetName, range)

-- Process the data as needed
for rowIndex, row in ipairs(dataInRange) do
  for colIndex, value in ipairs(row) do
    print(string.format("Row %d, Col %d: %s", rowIndex, colIndex, tostring(value)))
  end
end
```

### Example: Reading a Single Cell

```lua
-- Get the value of cell B2
local cellValue = xlsxParser.getSheetData(xlsx, sheetName, 'B2')
print("Value of B2:", cellValue)
```

## API Reference

### `loadFileXLSX(filepath)`

Loads an XLSX file and parses shared strings and sheet relationships.

- **Parameters:**
  - `filepath` (string): The path to the XLSX file.

- **Returns:**
  - A table containing:
    - `filepath`: The path to the XLSX file.
    - `sharedStrings`: A table of shared strings.
    - `sheets`: A table mapping sheet names to their file paths.

### `getSheetData(xlsx, sheetName, range)`

Retrieves data from a specific sheet, optionally within a specified range.

- **Parameters:**
  - `xlsx` (table): The XLSX data returned by `loadFileXLSX`.
  - `sheetName` (string): The name of the sheet from which to retrieve data.
  - `range` (string, optional): An Excel range notation (e.g., `"A1:C3"`). If omitted, the entire sheet is returned.

- **Returns:**
  - **Single Cell**: If a single cell is specified, returns the value of that cell.
  - **Range of Cells**: If a range is specified, returns a table of rows, each containing a table of cell values.
  - **Entire Sheet**: If no range is specified, returns the entire sheet data as a table.

## Error Handling

The module uses `error()` to raise exceptions when encountering issues such as:

- Invalid cell references.
- Sheet not found.
- Failed to read or parse XML content.

Ensure to wrap calls in `pcall` or use appropriate error handling mechanisms in your code.

## Example

```lua
local xlsxParser = require('xlsxParser')

-- Load the XLSX file
local xlsx = xlsxParser.loadFileXLSX('example.xlsx')

-- Get data from "Sheet1"
local data = xlsxParser.getSheetData(xlsx, 'Sheet1')

-- Print the data
for rowIndex, row in ipairs(data) do
  for colIndex, value in ipairs(row) do
    io.write(tostring(value), "\t")
  end
  io.write("\n")
end

-- Get data from a specific range
local rangeData = xlsxParser.getSheetData(xlsx, 'Sheet1', 'A1:B2')

-- Print the range data
for rowIndex, row in ipairs(rangeData) do
  for colIndex, value in ipairs(row) do
    print(string.format("Row %d, Col %d: %s", rowIndex, colIndex, tostring(value)))
  end
end
```

## Notes

- The module currently does not support formula evaluation. Cells containing formulas will return `nil` or the formula string, depending on implementation.
- Date and time formats are returned as raw values. Additional processing may be required to convert them to readable formats.
- The module assumes that the XLSX file conforms to the standard Open XML format used by Excel.

## License

Copyright 2024 BeamNG GmbH, Thomas Fischer <tfischer@beamng.gmbh>

**Disclaimer**: This module is provided "as is" without warranty of any kind. Use it at your own risk.

## Acknowledgments

- **slaxdom**: For XML parsing capabilities.
- **BeamNG GmbH**: Original copyright.

## Contact

tfischer@beamng.gmbh