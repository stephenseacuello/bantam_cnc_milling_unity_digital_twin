(Face milling 3x3 inch block - 6061-T6 Aluminum)
(Tool: 1/4" 2-flute HSS end mill)
(Workpiece: 76.2mm x 76.2mm x 50.8mm, origin at front-left-top)
G90 G21 (Absolute, metric)
G17 (XY plane)

(Roughing pass: 0.5mm DOC, 50% stepover = 3.175mm)
M3 S16000 (Spindle on, 16000 RPM)
G0 Z5.0 (Safe height)
G0 X-5.0 Y0.0

(Pass 1: Z = -0.5mm)
G1 Z-0.5 F500
G1 X81.2 F1000
G0 Z1.0
G0 Y3.175
G1 Z-0.5 F500
G1 X-5.0 F1000
G0 Z1.0
G0 Y6.35
G1 Z-0.5 F500
G1 X81.2 F1000
G0 Z1.0
G0 Y9.525
G1 Z-0.5 F500
G1 X-5.0 F1000
G0 Z1.0
G0 Y12.7
G1 Z-0.5 F500
G1 X81.2 F1000
G0 Z1.0
G0 Y15.875
G1 Z-0.5 F500
G1 X-5.0 F1000
G0 Z1.0
G0 Y19.05
G1 Z-0.5 F500
G1 X81.2 F1000
G0 Z1.0
G0 Y22.225
G1 Z-0.5 F500
G1 X-5.0 F1000
G0 Z1.0
G0 Y25.4
G1 Z-0.5 F500
G1 X81.2 F1000
G0 Z1.0

(Continue across full 76.2mm width...)
G0 Y28.575
G1 Z-0.5 F500
G1 X-5.0 F1000
G0 Z1.0
G0 Y31.75
G1 Z-0.5 F500
G1 X81.2 F1000
G0 Z1.0
G0 Y34.925
G1 Z-0.5 F500
G1 X-5.0 F1000
G0 Z1.0
G0 Y38.1
G1 Z-0.5 F500
G1 X81.2 F1000
G0 Z1.0

(Finishing pass: Z = -0.7mm, 15% stepover, slower feed)
G0 X-5.0 Y0.0
G1 Z-0.7 F300
G1 X81.2 F600
G0 Z1.0
G0 Y0.95
G1 Z-0.7 F300
G1 X-5.0 F600
G0 Z1.0

M5 (Spindle off)
G0 Z10.0 (Retract)
G0 X0 Y0 (Home XY)
M30 (Program end)
