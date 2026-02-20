(Full demonstration program - Combined roughing and finishing)
(Tool: 1/4" 2-flute HSS end mill)
(Workpiece: 76.2mm x 76.2mm x 50.8mm 6061-T6 Aluminum)
(Operations: Face → Pocket → Contour → Finish)
G90 G21
G17

(=== OPERATION 1: Face Milling ===)
M3 S16000
G0 Z5.0
G0 X-5.0 Y0.0

(Face top, 0.5mm DOC)
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

(=== OPERATION 2: Rectangular Pocket ===)
G0 Z5.0
G0 X20.0 Y20.0

(Pocket Layer 1: Z=-1.5)
G1 Z-1.5 F300
G1 X56.2 F800
G1 Y56.2
G1 X20.0
G1 Y20.0
G1 X23.175 Y23.175
G1 X53.025
G1 Y53.025
G1 X23.175
G1 Y23.175
G1 X26.35 Y26.35
G1 X49.85
G1 Y49.85
G1 X26.35
G1 Y26.35
G0 Z1.0

(Pocket Layer 2: Z=-3.0)
G0 X20.0 Y20.0
G1 Z-3.0 F300
G1 X56.2 F800
G1 Y56.2
G1 X20.0
G1 Y20.0
G0 Z1.0

(=== OPERATION 3: Circular Contour ===)
G0 Z5.0
G0 X38.1 Y23.1 (Circle center at block center, R=15)

G1 Z-2.0 F300
G2 X38.1 Y23.1 I0.0 J15.0 F600

G1 Z-4.0 F300
G2 X38.1 Y23.1 I0.0 J15.0 F600

(=== OPERATION 4: Finishing Pass ===)
(Re-trace pocket walls at final depth with fine feed)
G0 Z5.0
G0 X20.0 Y20.0
G1 Z-3.0 F200
G1 X56.2 F400
G1 Y56.2
G1 X20.0
G1 Y20.0
G0 Z1.0

M5 (Spindle off)
G0 Z10.0 (Retract)
G0 X0 Y0 (Return home)
M30 (Program end)
