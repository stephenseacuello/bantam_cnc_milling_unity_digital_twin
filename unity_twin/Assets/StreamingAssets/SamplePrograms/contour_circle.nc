(Circular contour pass - 40mm diameter circle centered on block)
(Tool: 1/4" 2-flute HSS end mill)
(Cut depth: 5mm, 1mm per pass)
G90 G21
G17

M3 S18000
G0 Z5.0
G0 X38.1 Y18.1 (Start at 3 o'clock: center=38.1,38.1 R=20)

(Pass 1: Z = -1.0mm)
G1 Z-1.0 F300
G2 X38.1 Y18.1 I0.0 J20.0 F600 (Full circle CW, center offset I=0 J=+20)

(Pass 2: Z = -2.0mm)
G1 Z-2.0 F300
G2 X38.1 Y18.1 I0.0 J20.0 F600

(Pass 3: Z = -3.0mm)
G1 Z-3.0 F300
G2 X38.1 Y18.1 I0.0 J20.0 F600

(Pass 4: Z = -4.0mm)
G1 Z-4.0 F300
G2 X38.1 Y18.1 I0.0 J20.0 F600

(Pass 5: Z = -5.0mm)
G1 Z-5.0 F300
G2 X38.1 Y18.1 I0.0 J20.0 F600

(Spring pass - repeat at final depth)
G2 X38.1 Y18.1 I0.0 J20.0 F400

M5
G0 Z10.0
G0 X0 Y0
M30
