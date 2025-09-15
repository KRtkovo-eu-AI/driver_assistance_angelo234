-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

if ffi then
  ffi.cdef[[
    typedef struct { float x, y, z; } Vector3;
    void BNG_DBG_DRAW_Sphere(float x, float y, float z, float radius, unsigned int packedCol, bool useZ);
    void BNG_DBG_DRAW_Cylinder(float x1, float y1, float z1, float x2, float y2, float z2, float radius, unsigned int packedCol, bool useZ);
    void BNG_DBG_DRAW_Line(float x1, float y1, float z1, float x2, float y2, float z2, unsigned int packedCol, bool useZ);
    void BNG_DBG_DRAW_Text(float x1, float y1, float z1, const char * text, unsigned int packedCol);
    void BNG_DBG_DRAW_LineInstance_MinArg(float x1, float y1, float z1, float x2, float y2, float z2, float w, unsigned int packedCol);
    void BNG_DBG_DRAW_SquarePrism(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float x4, float y4, unsigned int packedCol, bool useZ);
    void BNG_DBG_DRAW_TextAdvanced(float x1, float y1, float z1, const char* text, unsigned int packedCol, bool useAdvancedText, bool twod, unsigned int bgColorPacked, bool shadow, bool useZ);
    void BNG_DBG_DRAW_TriSolid(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3, unsigned int packedCol, bool useZ);
    void BNG_DBG_DRAW_LineInstance_MinArgBatch(const float &data, unsigned int lineCount, float w1, unsigned int packedCol);
    void BNG_DBG_DRAW_TriSolidBatch(const float &data, unsigned int triCount, unsigned int packedCol, bool useZ);
  ]]
end
