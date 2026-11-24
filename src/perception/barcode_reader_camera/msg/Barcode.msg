# One decoded symbol.
string format        # QRCode/DataMatrix/Code128/Code39/EAN-13/PDF417/Aztec/ITF
string code          # decoded text (UTF-8)
string raw_hex       # raw payload bytes as hex, e.g. "31 32 33"
string symbology_id  # ISO/IEC 15424 identifier, e.g. ]Q1 ]d1 ]C0
float32 center_x     # original image pixels
float32 center_y
float32[8] corners   # x0,y0,x1,y1,x2,y2,x3,y3 in original image pixels
string source        # decode strategy that first produced it (WARP-2d, ROI-raw, PFULL-1d-otsu, ...)
