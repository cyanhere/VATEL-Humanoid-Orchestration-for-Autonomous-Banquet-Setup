# R1 sensor helmet v0.11 + cable box v2.8 — CAD (print-ready)

Concept-stage 3D model of the clamp-on sensor helmet (Hesai JT128 + RealSense D435if) for the Unitree R1, and of
the cable box on the robot's back. **Cable box v2.8 is laser-cut acrylic** (sections below): the box is five 3 mm
acrylic panels between two printed end caps, every part is screwed to something, and the hub's ports face out of
the box. **v0.11 makes P7 one piece:** v0.10's cable former never touched the rest of P7. It's gone, and P7 now prints flat in PETG.
**v0.10 routes the cables:** P7 becomes a cradle for the JT128's right-angle plug,
P7b is a new clip on the rear clamp, and the new cable box (now `R1_cable_box_v2.8_ASSEMBLY.step`, parts `B1`–`B14`,
built by `../cable_box_cad.py`) holds the USB hub and takes up the head cables. Every other helmet part is
byte-for-byte the same as v0.9. **v0.9 mounted the real JT128:** it changed P4 and P7 and added P4b (a lidar adapter
plate). v0.8 answered the SolidWorks review of v0.7 and the first heat-set insert test (P2, P3,
P4, P5, P6, ear ring pads; Intel's own D435 model). Earlier: v0.7 was the first revision after fitting the printed helmet on the
robot (skull screws, 1 mm clamp gap, ear pad preload, cable outlets); v0.6 made the P2 / P3 print face flat; v0.5
fixed the P1 pod support; v0.4 made every part print-ready. The 2D drawings in `../drawings/` are generated from this
CAD's own tessellation, so the drawings and the STEP files show the same parts.

## Cable box v2.8: one 627ZZ

You have one NSK 627ZZ, so the pulley now runs on a single bearing. **Reprint `B3_takeup_carriage`,
`B3b_carriage_cheek` and `B4_takeup_pulley`** (the v2.7 ones are replaced); every other file is byte for byte the same
as v2.7. The axle is still the M5 × 35 with its nut.

- **Where the bearing sits:** 7.38–14.38 mm in from B4's back face, centred midway between the two
  cable grooves. Both cables carry the same tension (they hang on the same carriage), so their combined pull passes
  through the bearing's middle and doesn't try to tip it. If one cable goes slack, the tipping load is about 15 N·mm,
  which a deep-groove bearing takes easily, and the cheeks sit 0.9 mm from the pulley's faces, so it can't wobble far.
- **B4 pulley:** one Ø22.15 × 7 seat, entered from the front face, with the bearing pressed down onto a Ø18 shoulder.
  As B4 prints (back face down) that shoulder faces up, so there's no overhang inside the bore. The pulley's outside,
  grooves and width are unchanged.
- **B3 / B3b:** the bosses become Ø10 spacer tubes that reach in to the inner ring (8.1 mm long on B3, 12.4 mm on B3b),
  each ending in a Ø6.85 × 3 mm spigot. The two spigots meet inside the bearing's 7 mm bore, 1 mm apart; the M5 runs
  through both.
- **Fit** (same 627 data as v2.7; measured on the exported parts):

| | Drawn | 627 (SKF / Schaeffler 627-2Z; NSK 627DD) | Result |
|---|---|---|---|
| Seat in B4 | Ø22.15 × 7, onto a Ø18 shoulder | D 22, B 7; housing abutment ≤ 19 | light press fit once printed; the shoulder backs the outer ring all round |
| Spacer tubes on B3 / B3b | Ø10 | shaft abutment 9.4–10.5 | touch only the inner ring |
| Spigots | Ø6.85 × 3 from each side, 1 mm apart | bore 7 | slip fit; together they centre the bearing on the M5 |
| End float | 0.15 / 0.15 (measured) | – | the inner ring is never squeezed |
| Load | about 5 N | C0 ≈ 1.4 kN | far below the rating |

## Cable box v2.7: NSK 627ZZ bearings in the pulley

You found NSK 627ZZ bearings (7 × 22 × 7) to use instead of the 625ZZ (5 × 16 × 5). **Reprint `B3_takeup_carriage`,
`B3b_carriage_cheek` and `B4_takeup_pulley` only**: every other file (prints, part STEPs, laser DXFs) is byte for byte
the same as v2.6. The axle stays an M5 × 35 with its nut.

- **B4 pulley:** seats Ø22.15 × 7.0 (they were Ø16.15 × 5.0), one from each face, with a Ø18 shoulder between them.
  The pulley's outside, its two grooves and its width (26 mm) are unchanged, and there is still 14 mm of plastic
  between a seat and the groove root.
- **B3 / B3b cheeks:** the 627's bore is 7 mm and the axle is M5, so each cheek's boss now carries a Ø6.85 × 6 mm
  spigot (0.4 lead-in) that fills the bore of the bearing next to it. The M5 runs through the spigots. The bosses grow
  from Ø7.4 to Ø10, to suit the bigger inner ring. Both spigots point up as the cheeks print, so they need no support.
- **Fit,** checked against the 627-2Z data from SKF and Schaeffler, NSK's 627DD limits, and measured on the exported parts:

| | Drawn | 627 (SKF / Schaeffler 627-2Z; NSK 627DD) | Result |
|---|---|---|---|
| Seat in B4 (× 2) | Ø22.15 × 7.0 deep | D 22, B 7, ra ≤ 0.3 | 0.075 a side as drawn. Printed holes come out 0.1–0.2 small, so it ends up a light press fit; the bearing sits flush |
| Shoulder between the seats | Ø18 bore | housing abutment Da ≤ 19.6 (SKF), ≤ 19 (NSK) | backs the outer ring all round, and stays clear of the inner ring (d1 ≈ 12.2) |
| Bosses on B3 / B3b | Ø10 × 0.75 | shaft abutment da 9.4–12.1 (SKF), ≤ 10.5 (NSK) | inside every maker's range: they touch only the inner ring |
| Spigots | Ø6.85 × 6.0, 1 mm short of the bearing's far face | bore d 7 | 0.075 a side as drawn; printed pins come out a little big, so a slip fit. Each one centres its bearing on the M5 (Ø5.3 hole, 0.78 mm wall: it only locates, the bolt carries the load) |
| End float | 0.15 / 0.15 each side (measured) | – | the bearings are never squeezed, whatever the print tolerances |
| Load | about 5 N on the pulley | C0 ≈ 1.4 kN | far below the rating |

## Cable box v2.6: hook fillet, assembly colours, bearing fit

- **B5 hooks:** the plate and the arm were rounded separately, which left the plate's front edge square where it
  meets the arm. They are now one body, rounded together:
  - the plate's front corner is R2.5;
  - an R2.5 fillet fills the inside corner between the plate and the arm;
  - the R1.5 top edges run on from the plate onto the arm.
  **Reprint `B5_handle_hook_L` and `_R` only**: every other file (prints, part STEPs, laser DXFs) is byte for byte
  the same as v2.5.
- **Colours in the assembly:** screws silver, nuts and washers dark grey, heat-set inserts brass, magnets red, 625ZZ
  bearings blue.
- **Bearing fit,** checked against SKF's data for the 625-2Z (= 625ZZ) and measured on the exported parts:

| | Drawn | 625-2Z (SKF) | Result |
|---|---|---|---|
| Seat in B4 (× 2) | Ø16.15 × 5.0 deep, square inside corner | D 16 (0 / −0.007), B 5 (0 / −0.06), chamfer r 0.3 | 0.075 a side as drawn. Printed holes come out 0.1–0.2 small, so it ends up a light press fit. The bearing sits flush; the square corner is under the allowed fillet (ra ≤ 0.3) |
| Shoulder between the seats | Ø12 bore | housing abutment Da ≤ 13.6; shield recess D2 ≈ 13.2 | the shoulder reaches in past Ø13.6, so it backs the outer ring all the way round |
| Bosses on B3 / B3b | Ø7.4 | shaft abutment da 7.4–8.3; inner ring shoulder d1 ≈ 8.4 | inside the allowed range: they touch only the inner ring, never the shield or the outer ring |
| End float | 0.15 each side (measured) | – | the pulley can shift 0.3 mm; the bearings are never squeezed, whatever the print tolerances |
| Bore on the axle | Ø5 bore on the M5 × 35 | d 5 (0 / −0.007); M5 6g major Ø4.83–4.98 | 0.02–0.17 clearance: a slip fit. The nut pulls B3b onto B3's tenons, so the inner rings float between the bosses rather than being clamped. The far bearing sits on the bolt's thread (an M5 × 35 has 22 mm of thread); for an inner ring that stands still that is fine |
| Load | about 5 N on the pulley | C0 = 0.38 kN | far below the rating |

## Cable box v2.5: answers to the v2.4 review

| Review point | v2.5 |
|---|---|
| Do B4 (grey) and B13 (blue) turn? | **B4 turns.** One NSK 627ZZ pressed into it (since v2.8) runs on the M5 axle (drawn in the assembly as `bearing_627ZZ_1`), and the whole carriage slides up and down as well. Both happen together: when the head pulls 2 mm of cable out, the pulley turns and the carriage rises 1 mm; when the head comes back, the spring pulls it down. **B13 doesn't turn, and doesn't need to.** It only stores the JT128 lead's spare length, wound on it once at assembly. It sits past the tie point on B11's rib, so nothing on it ever moves. |
| Is there room for the lead round B13? (3.2 mm to the wall) | The 3.2 mm you measured is to the spool's back lip. What fills the column is the wound lead itself: 3 turns of the Ø5.5 lead on the R25 tube reach R30.5. In v2.4 that left 1.1 mm to the side panel and 3.0 to the divider. The spool is now centred in its column, so **the wound lead has 2.0 mm each side**. Wind it on the bench: take B13 out (its 2 screws go in through its open centre), wind the 3 turns, and screw it back. |
| Countersinks and counterbores close to the edges | **B5:** the tongue is now 13.5 × 13.5 mm and the front hole moved 2 mm inboard. Each countersink has ≥ 3.4 mm of plate round it (≥ 2.85 mm right at the surface, where the R1.5 edge rounds off; v2.4 had about 1.2). **B14:** the two counterbores were 0.8 mm apart and 0.5 mm from the front face. They are now on a diagonal (x −111.5 / z −11 and x −119 / z −17): 3.4 mm between them and ≥ 2.9 mm to every edge (≥ 2.35 at the surface). The ear is 3 mm longer at the back, and B10's corner blocks reach 3 mm further back to take the inserts. |
| B3's rounding looks odd | It was. The back cheek was rounded first and the bridges were added after, so each bridge met a rounded edge and left a step. Now the cheek and both bridges are one body, rounded together: R1 over the outside and R1 in the inside corners where the bridges meet the cheek. The back face and the bridge ends (which sit on B3b) stay sharp. |
| Relief holes at the acrylic corners | Added, R0.6 (Ø1.2): one in each corner of every tab slot in B1 and B2, and one at both root corners of every tab on the side panels and the divider (including the divider's top tab). A laser leaves a small radius in an inside corner; the reliefs let a square tab go all the way in and the panel's edge sit flat. |

**What to print and cut for v2.5** (compared with v2.4):

| Files | v2.5 |
|---|---|
| `B10_bottom_cap`, `B13_cable_spool`, `B14_strap_ear_L`, `B14_strap_ear_R`, `B3_takeup_carriage`, `B5_handle_hook_L`, `B5_handle_hook_R`, `B9_top_cap` | **reprint** |
| all five `laser_cut/` panels | **recut**: relief holes; the side panels also have the ear holes moved |
| `B11_uturn_former`, `B12_hub_cradle`, `B3b_carriage_cheek`, `B4_takeup_pulley`, `B7_spring_drum` | unchanged, byte for byte |

## Cable box v2.4: checked against the robot measurements

Your measurements and photos of the R1 were: upper holder Ø8, side holders Ø6.5, 25 mm from the holder's top down
to the top of the Jetson's ports, hub 121.5 × 32.4 × 14.0 mm, and 137 mm of hub cable to the plug tip. Checked
against v2.3, they showed three problems, all fixed in v2.4:

| Check | v2.3 | v2.4 |
|---|---|---|
| Hub USB-C → Jetson Type-C | **did not reach.** The Jetson's Type-C is on the robot's right (y ≈ −33) and the hub sat in the left column, so its cable would have had to cross the whole port deck: about 150 mm against 137 | **the whole box is mirrored:** hub, jumper window and spool on the right, take-up and cable entry on the left. The cable now needs about 114 mm (with a 25 mm plug), so **about 23 mm spare** |
| Hooks vs the Jetson's ports | the lips hung in front of the holder's crossbar at y ±21–43, right over the USB-A (y ≈ +27) and Type-C (y ≈ −33) plugs | **corner hooks:** each sits on a back corner of the U-shaped holder, nothing at \|y\| < 42 (the ports and plugs are within \|y\| ≤ 40) |
| Hub | assumed 115 × 36 × 12.5 | measured 121.5 × 32.4 × 14.0: B12's pocket (121.8 × 32.7 × 14.3), the side panel's notch, the RJ45 notch and B13 follow it |

**Does the hub reach the Jetson?** The build models the cable as a smooth path and checks its length against the
137 mm you measured. The path:
- leaves the hub's top end through B12's cable gap;
- bends out through B1's window (an S-bend, R14);
- rises between B1 and the holder's crossbar;
- goes over the crossbar in one half circle, 1 mm clear of it;
- goes down into the Type-C plug.

It stays at least 0.9 mm from the box, the hooks and the screw heads, and 1.1 mm from the R1 mesh.

| Jetson Type-C at | Cable needed (incl. plug) | Spare of 137 mm | Bend over the crossbar |
|---|---|---|---|
| x -82 (6 mm further back) | 108 mm | 29 mm | R8 |
| x -76 (from your photos) | 114 mm | 23 mm | R11 |
| x -70 (6 mm further forward) | 123 mm | 14 mm | R14 |

The Jetson and its ports are not in the R1 meshes. Their position comes from your photos and the 25 mm you
measured: ports facing up, port tops at z 195, about 9 mm in front of the crossbar. The model also assumes the hub's
USB-C plug has a 25 mm rigid part. A shorter plug leaves more spare.

**The new hooks.** In the R1 mesh the upper holder is a U: the Ø8 crossbar runs behind the Jetson (axis x −89.6),
and its side arms (\|y\| 48–56) go forward to the torso through R8 corners.
- **Arm:** each B5 hook's 21 × 22 mm arm rests on a corner and its side arm, and carries the box.
- **Outer lip:** a 4 mm lip outside the side arm.
- **Inner tooth:** a 2.7 mm tooth inside the corner. Its back follows the corner, so the box's top can't pull back
  off the holder; that pull is about 2 N from the box's overhang.
- **Clearances:** 1.5 mm at the corner and 1.8 mm beside the side arm (the real holder may sit about 1 mm off the
  mesh); the arm is 2 mm over the holder's top until the box settles.
- **The two hooks are mirror images** (`B5_handle_hook_L` / `_R`).
- **Fixing:** each screws to B9 with 2 × M3 × 10 countersunk. One screw is in a tongue inboard of the cable entry
  slot and one in B9's front block, which now reaches down to z 212.
- **B1 → B9 screws:** moved 8.5 mm lower, under the hook arms.
- **Strength,** in v2.2's design case (the whole box plus the cable pull on **one** hook, × 4 for knocks, about
  50 N):
  - arm root about 0.7 MPa, along the print layers;
  - the front hook screw about 130 N, against several hundred newtons for an M3 insert;
  - the tooth about 2 MPa at 4 × the box's backward pull.
  - Print them as before: 100 % PETG, top face on the bed.

**What to print and cut for v2.4:**

| Files | v2.4 |
|---|---|
| `B5_handle_hook_L`, `_R` | **new** corner hooks (one of each; they replace the two `B5_handle_hook`) |
| `B12_hub_cradle` | **reprint**: for the measured hub, and mirrored |
| `B3`, `B3b`, `B9`, `B10`, `B11`, `B13` | **reprint**: mirror images of v2.3 (B9 also has the hook tongues and the deeper front blocks) |
| `laser_cut/B1_front_panel` | **recut**: mirrored (window on the right), B1 → B9 holes 8.5 mm lower |
| `laser_cut/B6_side_panel_R` | **recut**: now the hub panel (notch for the measured hub + tie slots) |
| `laser_cut/B6_side_panel_L` | same cut as v2.3's `B6_side_panel_R`, just turned over: if you already cut that one, use it here |
| `B2_lid`, `B8_divider`, `B14_strap_ear_L` / `_R` | **unchanged**, byte for byte |
| `B4_takeup_pulley`, `B7_spring_drum` | same shape (they only moved to the other column); keep any you printed |

## Cable box v2.3: tolerances, hardware and fits

A check of every fit in the box against the hardware it takes and how printed and laser-cut parts come out. Four fits
were too tight and are fixed (B3, B3b, B9, B10 and B11 changed; every other box file is the same as v2.2):

| Fit | v2.2 | v2.3 |
|---|---|---|
| Pulley between the carriage's bearing bosses | 0 mm axial clearance: the bosses (2 × 0.9) and the pulley (26) exactly filled the 27.8 mm between the bridges, so a pulley printed 0.1 mm long would have squeezed the bearings | bosses 0.75 mm: 0.15 mm float each side (measured from the STEPs) |
| Printed slots that take the acrylic divider (B10's floor ribs, the slot under B11) | 3.3 mm wide: too tight once printed (FDM slots come out 0.1–0.2 small) and for a 3.2 mm sheet | 3.6 mm (0.3 a side) |
| Caps between the side panels | line to line: a cap printed a little wide would push the panels out of B1's slots | 0.15 mm a side (the panel screws pull them in) |
| Magnet pockets in the caps | Ø6.15, no lead-in, in holes that print horizontally (~0.1–0.2 small) | Ø6.2 with a 0.4 mm lead-in: a firm press fit |

**Every fit (v2.3).** Clearances are per side unless noted; printed holes usually come out 0.1–0.2 mm small, and the
laser's kerf (about 0.1–0.2 mm) opens holes and slots in the acrylic.

| Fit | Drawn | Goes in | Result |
|---|---|---|---|
| Heat-set inserts (25, all M3) | Ø4.0 × 5.5 deep; Ø4.0 × 4.5 in B3's lower bridge | M3 short insert, OD ≈ 4.6 (the H19 size already used in P4) | melted in; the 5.5 holes take inserts 3–5 mm long, B3's takes 3–4 mm. Not the 5.7 mm H15 type |
| M3 clearance holes (printed and acrylic) | Ø3.4 | M3 screws | ISO 273 medium; the acrylic ones open to about Ø3.5 |
| Button heads sunk in the B14 ears | Ø6.2 × 4 counterbore | ISO 7380 M3 head Ø5.7 × 1.65 | 0.25 |
| Countersinks in B5 | 90°, Ø6.8 | ISO 10642 M3 head Ø6.72 | flush |
| B5 hooks on the R1's upper holder (v2.4) | – | Ø8 holder, mesh geometry | 1.5 at the corner, 1.8 beside the side arm, 2 over the top (settles) |
| M5 axle head in B3 | Ø9.2 × 5.5 counterbore | ISO 4762 M5 head Ø8.5 × 5 | 0.35, head 0.5 below the face |
| M5 axle | Ø5.3 through B3 / B3b and their spigots; 627ZZ bore 7 | M5 × 35 | the 627ZZ sits on both cheeks' Ø6.85 spigots (slip); the nut pulls B3b onto B3's tenons, and its inner ring floats 0.15 a side between the spacer tubes (v2.8 table); 4.5 of the nut's 4.7 mm engaged |
| M5 nut pocket in B3b | 8.2 across flats × 5.4 deep | ISO 4032 M5 nut, 8.0 AF × 4.7 | 0.1 (draw it in with the bolt) |
| 627ZZ seat in B4 (v2.8, one) | Ø22.15 × 7.0, entered from the front face | NSK 627ZZ, 22 × 7 | light press fit once printed; pressed down onto the Ø18 shoulder, which backs the outer ring |
| Spacer tubes + spigots (B3, B3b, v2.8) | Ø10 tubes to the inner ring, spigots Ø6.85 × 3 | 627 inner ring, bore 7 | bosses inside the shaft abutment range (9.4–10.5): only the inner ring is touched; spigots slip in the bore; 0.15 axial float |
| Pulley rims to the cheeks | – | – | 0.9 |
| Tenons in mortises (B3 → B3b) | 10 × 2.5 / 3.0 into 10.4 × 2.9 / 3.4 | – | 0.2 |
| Spring drum | B10 fork Ø4.3, drum bore Ø4.4, 0.5 to each fork cheek | M4 × 30 + washer + nyloc | the drum turns on the M4 |
| Magnets in the caps | Ø6.2 × 3.2 + 0.4 lead-in | Ø6 × 3 N42 | press fit; press flush with a vise, same pole out on all four |
| Magnets in the lid (acrylic) | Ø6.0 | Ø6 × 3 N42 | slip fit after the kerf: glue them (epoxy or gel CA). Don't press magnets into acrylic: it cracks |
| Side panel and divider tabs in B1 | slots 3.15 × 16.15 | 3.0 mm sheet | about 0.15 with the kerf |
| Slot corners and tab roots (v2.5) | R0.6 relief holes | the tab's square corner | the tab seats fully, the panel edge sits flat |
| Tabs in the lid | slots 3.4 × 16.4 | 3.0 mm sheet | about 0.3: the lid drops on |
| Caps between the side panels | – | – | 0.15 |
| Divider in B10's ribs and B11's slot | 3.6 wide | 3.0 mm sheet | 0.3 (still fits a 3.2 mm sheet) |
| Carriage in its column | – | – | 0.6 to each of the four walls |
| B12's end lips in the hub panel's notch (right since v2.4) | – | – | 0.15 |
| Hub in B12 | pocket +0.3 | hub, 121.5 × 32.4 × 14.0 (measured, v2.4) | 0.15 |
| Cables in the pulley / former grooves | 6.0 and 5.5 wide | JT128 Ø5.5, RealSense Ø4.5 | 0.25 and 0.5 |

Different sheet: if your acrylic isn't 2.9–3.1 mm, set `ACR_T` in `../cable_box_cad.py`; every slot follows it.

**Hardware: all standard metric parts.**
- **Screws:**
  - ISO 7380 button head M3 × 8 (16), M3 × 12 (4) and M3 × 5 (1)
  - ISO 10642 countersunk M3 × 10 (4)
  - ISO 4762 socket head M5 × 35 and M4 × 30
- **Nuts and washers:** ISO 4032 M5 nut, ISO 10511 M4 nyloc, M3 and M4 washers.
- **Bearings, magnets, inserts:** NSK 627ZZ (×1, v2.8), 6 × 3 mm N42 discs (×8), 25 M3 short heat-set inserts.

**Mounting points:** the build checks all 27 fasteners against the solids:
- every hole lines up through every part it passes
- the heads seat
- each insert has at least 1 mm of plastic round it
- thread engagement is 4.0–5.2 mm
- every screw tip stops short of the bottom of its hole

## Cable box v2.2: thicker hooks

The two B5 hooks carry the whole box (about 0.96 kg with the hub and cables, plus up to 2.6 N of cable pull). Only
B5 changed; every other box file is the same as v2.1.

| | v2.1 | v2.2 |
|---|---|---|
| Arm and lip width | 16 mm (24 at the plate, with a neck between) | 22 mm all the way, no neck |
| Top plate | 5 mm | 6 mm (the M3 × 10 countersunk screws still engage 4 mm) |
| Lip | 6 mm, 7 mm in front of the handle bar | 10 mm, 5 mm in front of it |
| Inside corner (arm to lip) | R1.5 | R3 |
| Volume | 13.5 cm³ | 20.4 cm³ (26 g each, 100 % infill) |

**How strong.** Design case: the whole box plus the cable pull (12 N) on **one** hook, times 4 for knocks (about 50 N).
- **Arm root,** where the arm leaves the box (bending, along the print layers): 0.35 MPa (v2.1: 0.53).
- **Lip root,** if the box swings and the same 50 N pushes the lip (across the layers, PETG's weak direction): 0.7 MPa
  (v2.1: 2.6).
- **PETG strength:** printed PETG holds roughly 40 MPa along its layers and 15–20 MPa across them, so even the old hook
  had a large margin. v2.2 has more than 20 times the design load everywhere.
- **Hook screws:** two M3 heat-set inserts per hook pull out at several hundred newtons each, against
  about 30 N per screw in the design case (counting the hook's lever about the box's top edge).
- **Print:** keep 100 % infill and PETG. They print on their top face, so the arm bends along the layers.

## Cable box v2.1: answers to the v2.0 review

| Review point | v2.1 |
|---|---|
| B5 hook: the rear countersink is almost at the plate's edge | The plate runs 5 mm further back (5 mm of plastic behind the countersink, was 0.1). The hooks also moved 4 mm inwards (y ±32), so the longer plate clears the cable slot's bell mouth; the lips now sit 8 mm clear of the handle's fixings. |
| B3b: the countersinks break its top and bottom edges | The two self-tapping screws are gone. B3's bridge ends carry 10 mm tenons that drop into mortises in B3b (1.3 mm walls; B3b's inner face is left sharp so the walls are full). The M5 axle's nut, now on 1.4 mm of B3b's own plastic, pulls B3b onto them. B3b's outer face has only the nut pocket. |
| How is the B3 / B4 carriage held? Nothing fixes it to the walls | It isn't fixed, on purpose: it is the moving part of the take-up (a "dancer"). The two cables loop **under** its pulley and pull it up, the constant-force spring on the B7 drum (4 N) and the carriage's own weight (1.2 N) pull it down, and the four walls round it (front panel, lid, side panel, divider) leave 0.6 mm a side, so it can only slide up and down. The head pulling cable out lifts it; the spring pulls it back down and takes up the slack. If the cables are unhooked the spring just lowers it onto the drum; with the lid off it can tip backwards, so fit the lid before moving the robot. |
| Does the spring pull on the head or the sensor ports? | A little, and it is meant to: it keeps the span taut. The spring (4 N) plus the carriage's weight (1.2 N) hang on four cable strands, so each cable carries about 1.3 N, and the two strands that go to the head pull about 2.6 N in all (up to about 4 N while the head is pulling cable out, with friction at the slot). On the neck that is at most 0.5 N m, against the R1's 33 N m head joints (`R1.urdf`): under 2 %. The pull stops at P7b, where both cables are zip-tied; between P7b and each plug the cables hang slack and carry nothing. **What can hurt the plugs is the end stop:** if the head turns past what the take-up stores, the carriage hits the former and the neck motor pulls straight on the cables. Keep the ±80° head-yaw limit in software (the joint itself goes to ±115°). |
| If the lid is held only by magnets, can the take-up push it off? | No. The cables and the spring pull the carriage up and down in the plane of the pulley; the lid is only one of its four guide walls. The loads that do reach the lid are under 1 N from the cables at the slot, and the carriage's inertia: 1.2 N per g of forward acceleration, so 3.6 N even at 3 g. The four Ø6 × 3 magnet pairs hold roughly 30–40 N, eight times that or more. If the lid did come off (a fall onto the box), nothing falls out: the cradle, spool, former, drum and caps are screwed; the hub sits on Dual Lock with the spool 3.5 mm behind it; the carriage stays hanging on its cables and spring. Only the divider would then slide back if pulled, since the lid holds its back edge. |
| What is B14 for? | A strap anchor. The box hangs from its two hooks at the top; a 20 mm velcro strap through each ear's slot goes round the R1's U-frame bar beside it and back onto itself, so the bottom of the box can't swing away from the robot. The bar is vertical above z 55 and bends inwards below; v2.0's slot (z 10–32) was below most of the bend, so it now sits at z 22–44, level with it. |
| Hub ethernet: is there room for a plug, and does its cable block the window? | v2.0 had none: the RJ45 is on the hub's bottom end and sat on the cradle's ledge with the coil ring under it. Now the hub is 8 mm lower, the ledge has a notch under the port, and there are 38 mm below it for the plug and boot (checked). The cable bends under the cradle and runs up a 20 × 16 mm channel beside the divider. The jumper window is now **58 × 23 mm** (was 36 × 15), so it takes all five cables (hub USB-C, RealSense, JT128 RJ45 + power, hub ethernet) and a plug can pass while the others are in. |
| What are the three holes in B11? | Zip-tie windows. Below the U-turn, each head cable is zip-tied to the rib through two windows (JT128 between 1 and 2, RealSense between 2 and 3). The spring keeps the cables in tension; tied here, nothing pulls on the cable from the Jetson side. They are now all 3.2 mm wide (v2.0 had one 1.6 mm window, too narrow for a 2.5 mm tie), and the rib is 4 mm longer to make room. |
| Is B9's entry slot big enough for both cables? | Yes. It is 11 mm wide and 30 mm long (open at the back, the lid closes it). The JT128 lead (Ø5.5) and the RealSense cable (Ø4.5) sit 11.75 mm apart along it, in line with their pulley grooves, with 2.75 and 3.25 mm either side. The bell mouth is now 2 × 45° (was 1.2) for the cables that come in at an angle. |

To make room for the ethernet plug the spare JT128 lead moved off the floor: the new **B13 cable spool** (3 turns, R25)
screws to the back of the hub cradle with 2 screws, behind the hub. The splitter now sits low on the divider, behind
the ethernet channel, and the velcro-tie slots for the branch hanks moved down beside it.

## Cable box v2.0: laser-cut acrylic, every part fixed, hub ports outside

*Since v2.4 the box is mirrored: in this section the hub column is on the robot's right and the take-up on the left.*

v2.0 answers the review of v1.0. That review asked how B3b mounts, how the M5 axle mounts, whether the pulley is
printed, whether the hub's ports are usable, whether the box can be laser-cut acrylic, and asked for a nicer lid. It
also asked for every mounting point to be checked.

**What was wrong in v1.0:**
- **B3b doesn't bolt to the box, and isn't meant to.** It is the front cheek of the moving carriage and bolts to B3.
  The README never said so.
- **M5 axle:** it did mount (head in B3, nut in B3b), but nothing explained how.
- **Hub ports:** only the dongle's port was usable. The splitter covered the lower ports, and the 29 mm in front of
  the rest is too short for a normal plug and its cable.
- **Spring drum (B7):** its M4 axle had no nut, so the drum could walk off it.
- **Cable entry:** the slot in the top was closed, and 22 × 11 mm is too small to feed the JT128's M8 plug through.
- **U-turn former:** the top of its grooves ran 0.1 mm under the top wall. The cables would have needed threading
  through a closed channel, and they pressed on the wall by 0.15 mm.
- **Strap ears:** the strap slot ran into the side wall, so a strap couldn't pass through it.
- **Spring screw:** its insert sat at the edge of a 5.5 mm bridge, and the hole went right through.
- **Pulley:** the 625ZZ seats were Ø15.95. A printed hole comes out undersize, so the bearings wouldn't have pressed in.

**Construction.** The box is about 145 × 261 × 49 mm, made from 3 mm acrylic laser-cut from the DXFs in `laser_cut/`.
- **Acrylic panels:** B1 front panel (faces the robot), B2 lid, B6 left and right side panels, B8 divider.
- **End caps:** the side panels and divider stand between B1 and B2. Two printed caps, B9 (top) and B10 (bottom),
  fill the ends of the acrylic tube.
- **Screws:** B1 and both side panels screw into the caps with M3 button heads into heat-set inserts.
- **Tabs:** the tabs on the side panels' and divider's edges sit in slots in B1 and B2. They keep the panels straight
  between the caps, which matters because the carriage slides on them.
- **No glue,** and the box comes apart again.
- **Printed parts inside:** B11 U-turn former + rib and B12 hub cradle screw to B1 from the robot side; B13 cable
  spool screws to the back of B12 (v2.1).

**How every part is fixed** (v2.1; all 27 fasteners are checked in the build, see *Build checks*):

| Part | Held by | Into |
|---|---|---|
| B9 top cap, B10 bottom cap | B1: 2 × C16 M3 × 8 each, from the robot side. Each side panel: 2 × C16 into B9, 1 × C16 + 2 × C17 into B10 | C6 inserts in the caps' corner blocks |
| B1 front panel | the caps (above) + the slots that take the side panels' and divider's front tabs | – |
| B6 side panels | front-edge tabs in B1's slots, back-edge tabs in B2's slots, and screws into both caps (above) | – |
| B8 divider | front tabs in B1, back tabs in B2, bottom edge between two ribs on B10, top tab in the slot under B11 (it slides in from the back) | – |
| B2 lid | its slots drop over the 13 tabs on the side panels and the divider; 4 C7 magnet pairs hold it to the caps | magnets glued in the lid and the caps |
| B5 hooks (v2.4: corner hooks, L and R) | 2 × C8 M3 × 10 countersunk each, from above | C6 inserts in B9: one in a tongue behind the front rail, one in the front block |
| B11 former + rib | 2 × C16 through B1 | C6 inserts in its front face |
| B12 hub cradle | 2 × C16 through B1; its two end lips sit in the ends of the right panel's notch | C6 inserts in its rail and bottom ledge (+ 2 in the back of its rail for B13) |
| B13 cable spool | 2 × C16 from behind, through its disc (reach in through the spool's open centre); fit it after the hub | C6 inserts in the back of B12's rail |
| B14 strap ears | 2 × C17 M3 × 12 each, heads sunk in the ear, through the side panel. Use: a 20 mm velcro strap (C10) through the slot, round the U-frame bar beside it, back onto itself | C6 inserts in B10 (the same screws hold the side panel) |
| B7 spring drum | C4 M4 × 30 + washer + nyloc through the fork on B10 | nyloc (snug, the drum spins) |
| B3 + B3b carriage, B4 pulley | **not fixed to the box: it is the moving part.** The cables loop under the pulley and pull it up, the spring pulls it down, and it slides in the left column, 0.6 mm from the four acrylic walls (front panel, lid, side panel, divider) | – |
| B3b → B3 | the C3 M5 × 35 axle (head sunk in B3's back cheek, nut sunk in B3b); two tenons on B3's bridge ends sit in mortises in B3b so it can't turn | – |
| B4 pulley | one 627ZZ pressed in from the front face onto the Ø18 shoulder; it sits on both cheeks' spigots round the M5 axle, its inner ring between their spacer tubes (0.15 float each side), and the pulley turns on its balls | – |
| spring → carriage | C9 M3 × 5 + washer through the spring's end hole, from below | C6 insert in B3's lower bridge |
| hub | Dual Lock (C11) to B1's inside face; the cradle's ledges hold it up and down, its rail takes the push of a plug going in, and the end lips take the pull of one coming out. Its RJ45 (bottom end) has 38 mm below it for the plug | – |
| splitter | Dual Lock low on the divider, at the back | – |
| cables | zip-tied to B11's rib after the U-turn, through its three 3.2 mm windows; branch-cable hanks on velcro ties (C15) through the slots low in the right panel; spare JT128 lead on B13 | – |

**The lid.** B2 has no screws. Its slots drop over the tabs on the side panels' and divider's back edges, which
locate it to 0.2 mm, and four pairs of Ø6 × 3 magnets hold it to the caps. It sits flush, with a finger notch in its
bottom edge. With the lid off, the cable slot in B9 is open at the back, so both head cables can be laid in without
feeding a plug through. The lid closes it.

**The hub's ports face out.** The hub sits against B1 with its port face in a notch in the right panel, 1.5 mm inside
the panel's outer face, held at both ends by B12's lips. All ports (3 × USB-A, HDMI, USB-C PD) can be used from outside with the box closed, and the WiFi dongle sits
outside the box. The check pushes the whole port face 45 mm out from the panel, enough for an HDMI plug and its cable
bend. That zone is clear of the box, and the arms can't reach it without first hitting the torso.

**The take-up** is unchanged in principle: a 2:1 loop, pitch R28.
- **Longer travel:** the box is 8 mm taller at the bottom, so the carriage travels 117 mm and
  stores 235 mm of cable (v1.0: 230).
- **Former:** it sits 3 mm lower, so the cables clear the top cap by 5 mm and can be laid in from behind.
- **Pulley (B4):** printed in PETG. It only has to keep the two cables' bend at R28 and carry about 5 N; the 627ZZ
  bearing does the turning.
- **Spring drum:** it turns on an M4 in a fork on B10.

## What changed in v0.11: P7 in one piece

- **The bug:** v0.10's P7 was two separate bodies. Its cable former began 0.5 mm past the end of the plug's cable,
  and the shelf stopped 1 mm short of it, so the two never touched. The STL held two loose pieces. The build now checks
  that every part is a single solid.
- **The former is gone.** It isn't needed: P7b on the rear clamp ties both cables down, and the pull from the cable box
  stops there. Between the plug and its first tie, the JT128 lead only has to bend down gently. Leave it a loose loop
  (bend radius ≥ 28 mm, 5 × its diameter) and don't pull it tight round P4's edge.
- **What's left is the plug cradle:** two tabs on P4's guard inserts (same H9 screws), the bridge between them, the
  shelf under the plug, and the rear and end walls that stop the plug's body turning on its socket. The shelf no
  longer reaches down over P4's edge, so the whole part stands on one flat bottom, the face that sits on P4.
- **Print it in PETG** (6 g, no support, flat bottom on the bed). It is screwed down, and TPU would creep under the
  screw heads and let the screws loosen. Nothing on it needs to flex. P7b stays TPU.
- **Reprint `P7_connector_guard (PETG).stl`**, or the `_mirror` file if your plug's cable leaves to the right. Every
  other file is byte for byte the same as v0.10.

## What changed in v0.10: cable routing and the cable box

**Where the cables go.**
- **JT128:** its splitter cable has a right-angle M8 plug (manual Fig. 13), whose cable leaves sideways to the robot's
  left. The new **P7** holds the plug: a shelf and two walls round its body, and a quarter former that turns the
  cable straight down past P4's rim at R28 (the manual's minimum bend, 5 × OD). The cable is zip-tied to the former
  through its two windows, so the plug's joint (rated under 40 N sideways) never takes the pull. It still uses P4's
  two guard inserts and the H9 screws. If your plug's cable leaves to the right, print
  `P7_connector_guard_mirror (TPU).stl` instead. **(Superseded by v0.11: the former is gone, P7 is PETG, and P7b takes the pull.)**
- **Down the head:** the lead runs down the left of the head on stick-on zip-tie bases. The RealSense USB-C runs
  back along the right side the same way.
- **P7b clip:** both cables meet at P7b, a clip on the rear clamp flanges' back face. Two zip ties wrap the flange
  block; each runs down one cable groove, so it holds the clip and clamps its cable together. The two grooves are
  the same size (Ø6.2, for the JT128 lead), so each cable takes the groove on its own side, with either P7, and the
  two never cross.
- **To the box:** from P7b the two cables run side by side to the slot in the box's top. That span is the only cable
  that moves with the head. It is left unsleeved: the cables slide through the slot into the take-up as the span
  shortens, so a sleeve would bunch up at the slot. P7b and the slot hold the two cables together at both ends.

**The cable box as first built (v1.0, printed; replaced by v2.0 above).** It was about 144 × 253 × 48 mm, on the robot's back
behind the Jetson. Its assembly uses the same frame as the helmet assembly, so the two line up in SolidWorks.
- **Mounting:** it hangs on the upper handle by two B5 hooks and is strapped to the U-frame tubes with velcro
  through the ears at its lower corners. To remove it, undo the two straps, lift it off the handle and unplug the
  jumpers. The B2 lid comes off with 4 thumbscrews.
- **Right column, the take-up:** both head cables enter through the top, run down round the B4 pulley (two grooves,
  pitch R28) on the B3/B3b carriage, and go back up over a fixed U-turn former. A constant-force spring on the B7
  drum pulls the carriage down. The carriage travels 115 mm, which stores 230 mm of cable, at about 3 N of cable
  tension. After the U-turn the cables are tied to a rib and no longer move.
- **Left column:**
  - **Hub:** the UGREEN 6-in-1 (70411, 115 × 36 × 12.5) sits in a cradle against the front wall, held by ledges
    and Dual Lock tape, with its cable end up. The WiFi dongle plugs into its USB-A.
  - **Spare JT128 lead:** about 0.5 m is left over; it coils 2–3 turns round the ring at the bottom (R25, so the cable
    bends at its R27.5 minimum). One layer stays clear of the floor, the hub and the splitter (checked).
  - **Splitter:** stands upright on Dual Lock against the divider, below the dongle. The lead comes up into its bottom
    end; the RJ45 and power branches leave its top and run up behind the dongle to the window.
  - **Spare branch cable:** the branches are 800 mm long and the GNSS pigtail isn't used, so hank the spare lengths
    behind the hub and hold them with velcro ties through the two bridges on the side wall.
- **Jumpers:** four leave through the window in the front face, below the handle, into the Jetson panel: hub →
  USB-C, RealSense → USB-A, JT128 RJ45 → RJ45, JT128 power → XT30 through an XT30 → DC5521 adapter.
  **Measure the XT30's voltage first: the JT128 takes 9–32 V.**

**Checked against the R1's motion** (R1 meshes and URDF joint ranges; `../build/cable_box_report.json`): see
*Build checks* below.

## What changed in v0.9

| Found | v0.9 |
|---|---|
| The lidar mount doesn't fit: our JT128 has no flange | Right. Up to v0.8, S1 was a placeholder, not this unit: a Ø50 body with three M4 lugs on top and a connector housing. The JT128 user manual (`../assets/HESAI JT128 Documentation.pdf`, sections 1.4 and 2.1) shows a Φ62.5 × 17.1 mm base under a Φ~60 body, 73 mm tall. It mounts from **below**, with 3 × M3-6H tapped holes 6 mm deep on Φ56, at 30°, 155° and 280° from the connector (top view, anticlockwise). The holes with drill-point cones in the bottom view (Fig. 7) are the tapped ones; the other two are a round pin hole and a slot. The M8 socket sits 8.5 mm up the base and reaches 38.75 mm from the axis. **S1 is now modelled from those drawings** (`parts/S1_JT128.step`), with the connector pointing back. |
| Why a new part (P4b) and not just a new P4 | The lidar's screws have to go in from underneath, and its Φ62.5 base covers the four H6 screws that hold P4 to P1 (r 32). Screwed straight onto P4, the lidar could only go on before H6, and from below, which is P1. **P4b** is a 4 mm adapter plate, which is what Hesai's own recommended installation (Fig. 8) shows: the lidar screws to it from below with 3 × ISO 10642 M3 × 8, countersunk flush, 4 mm into its 6 mm holes (the manual asks for 3–5 mm). P4b then sits on P4 and its three ears screw down from above at r 35.5, clear of the base, with 3 × ISO 7380 M3 × 6 into M3 short inserts in P4. The ears are at 0°, 130° and 230°, so P4b fits only one way round, and the lidar's own hole pattern fits P4b only one way: the connector always points back. |
| P4 | The three M4 insert holes under the old lugs become three M3 short insert holes (Ø4.0, through) for P4b at r 35.5, 2.5 mm from the rim. The outline, the H6 countersinks, the guard inserts and the hoop holes are unchanged. |
| P7 | Its walls ran into the bigger base, so the clearance cut is now Ø67.2 (round P4b) instead of Ø51.2. The plug outlet's floor drops from 6 to 4 mm: the real plug axis is 12.5 mm above P4, against 17.7 mm on the placeholder. |
| Unchanged | P1, P2, P3, P5, P6, P8 and P9. The lidar seat, tilt (24°) and hoop are where they were, and the lidar now sits 4 mm higher on P4b. Its top is 5.9 mm under the hoop. |
| P7 (superseded by v0.10) | v0.9's P7 modelled a straight plug; the real right-angle plug would hit its side walls. v0.10 replaces it with a plug cradle (above). |

The manual also settles the field of view that the v0.8 CAD had to assume: the lowest channel is **−4.4°** (vertical
FOV −4.4° to +90.5°), measured from an optical centre 60.6 mm up the lidar. v0.8 assumed −9° from a lower point, so
the real margins are larger (*Build checks*).

## What changed in v0.8

| Review point | v0.8 |
|---|---|
| P8c ear ring pad: is the shape intended? It isn't smooth, and goes up and down round the circle | Neither was intended. **The rings:** the pad's profile was traced from the scanned ear disc as a 16-step polyline, and every step became its own cone face. Under the pad the disc is a plain 23.5° cone (the scanned profile is within 0.12 mm of a straight line), so the pad and its pocket in the cup are now **one cone each**; the pad has 4 faces. It keeps the 0.5 mm preload. **The up and down** is the cup's rim on the head side (the skirt). It follows the head round the ear disc, which is not flat (the head drops about 5 mm from one side of the cup to the other), and it had traced the scan's ripples. It is now **one smooth wave** (a 4-harmonic fit of the head), at least 0.7 mm off the head everywhere (1.7 mm on average). |
| P2 / P3: the extrusion from the side piece (the skull-screw lug) is a weird shape and hard to print | It was a column standing off the arm: its outer side faced the bed, so it needed support, and it met the arm in steps and scoops. It is now a **tab**: the crown arm itself widens forward to the screw, with the arm's own inner and outer surfaces (no step), R4 / R2 inside corners, R5 front corners and R1.2 edges like the rest of the arm. A **foot** under the tab reaches the skull (0.2 mm off it round the hole, 1.2 mm further out). Every wall of the tab and foot runs along the print's vertical (the ear cup axis), so it prints **without support**. The only overhangs are two short bridges: the Ø3.8 screw hole and the 2.5 mm flat roof of the Ø8.5 counterbore. The washer sits on the counterbore's flat floor, square to the screw. The screw, washer and hole B are unchanged (M3 × 16, tip 5.9 mm below the hole's centre). |
| P6: the USB-C doesn't go into the camera there (`reference/D435_Solid.SLDPRT`) | Right: v0.7 put the plug at the camera's mid-depth. Intel's model (opened in SolidWorks and saved as `reference/D435_Solid.step`) has the USB-C receptacle in a notch at the **back** of the camera's robot-right end, 3.8 mm in front of its back face, and the plug goes in along the camera. **S2 is now that model.** The P6 outlet is now a notch in the right cheek's back edge (14.5 × 10.5 mm, R5 corners), open towards P5, so the hood slides on over the plugged cable. The plug runs over where the hood's right ear and the right cradle screw head were, so **P6 has only its left ear** (the hood hugs the camera, and the left screw stops it sliding off), and **P5's right cradle screw is a countersunk M3 × 10 that sits flush**. The plug is assumed to be a 12.5 × 6.5 mm overmold, 22 mm long (red in the assembly): it clears P5 by 0.6 mm, the same as on Intel's camera. |
| P4: the holes are not for M3 / M4 brass inserts; the melted plastic jammed the threads | They were tap-drill holes (Ø3.3 / Ø2.5), meant for tapping the plastic. They are now **heat-set insert holes**: 3 × Ø5.6 for M4 (lidar) and 2 × Ø4.0 for M3 (guard), through the 4 mm plate with a 0.4 mm lead-in. P4 is only 4 mm thick, so use **short inserts: M4 × 4 (OD 6.3) and M3 × 3 or × 4 (OD 4.6)**, e.g. CNC Kitchen "M4 short" / "M3 short" or ruthex M4 short / M3 × 4.0. The standard M4 × 8.1 and M3 × 5.7 are longer than the plate. The guard holes were 0.65 mm from the rim, so the plate has two R4.5 lobes there (2.5 mm wall). |

The clamp is unchanged by the new tab: `../clamp_fe.py` on the v0.8 side piece gives thumbscrew 29 N, ear cup 27 N (19 N with the thumbscrew loose), skull screw 36 N and a peak stress of 7.0 MPa (v0.7: 27 / 26 / 19 / 36 N, 6.9 MPa).

`v08_changes.png` shows the changes (v0.7 left, v0.8 right where it helps).

## What changed in v0.7

| Found on the robot | v0.7 |
|---|---|
| The whole helmet comes off when pulled backwards; it relied on clamping alone | **Each side piece is now screwed to the skull.** A lug on the crown arm sits over the R1's own threaded hole (the middle pair, "B", `R1_skull_holes.png`) and takes an ISO 4762 M3 × 16 socket-head screw with a washer (H17 + H18), vertically, from above. The lug's foot follows the skull (0.2 mm off it round the hole) and its seat is flat, square to the screw. With both side pieces screwed down, the two hinges lock the crest too, so the helmet is held by two screws and no longer by friction. |
| The 3 mm clamp gap bent the side pieces ("the headphone shape changes") | The design gap is now **1.0 mm**. The thumbscrew only snugs the ring now. FE (`../clamp_fe.py`, with the skull screw) gives about 27 N of thumbscrew tension with the flanges touching, and the side pieces barely deflect. |
| The ear cup goes over the ear disc but is loose (TPU 95A doesn't compress) | The TPU pad was never meant to give way; the cup was simply never pushed onto the disc. In the v0.6 FE the rear pads took 44/52 N of the clamp and the ear only 13 N, and with the thumbscrew loose the cup was free. The **ear ring pads are now 0.5 mm thicker** towards the disc. With the side piece screwed down, the crown arm flexes to take that 0.5 mm, so the cup is pressed onto the disc's 24° cone, which centres it. FE: about 19 N per ear from the screw alone, 26 N with the flanges touching. If that is too tight or too loose, scale the pad's thickness in the slicer (Z): each 0.1 mm changes it by about 4 N. |
| The RealSense hood needs a cable outlet (USB-C on the side) | **P6 has a U-shaped outlet** in its robot-right cheek, 18 mm wide, open at the bottom and reaching 9 mm above the camera's mid-height, so the hood goes on with the cable in. Plug the cable in **after** the hood's H8 screws (the plug sits in front of the right H8 head). |
| ... and the lidar | The JT128 connector points straight back from the base (manual, Fig. 6/7). **P7's side walls now splay** from 22 mm apart (at the H9 screws) to 27 mm at the back, and its rear wall is **open above a 6 mm bumper bar**, so the mated plug runs out of the back and the cable bends down behind the bar. The plug's size isn't in the manual: the CAD assumes up to 23 × 20 mm (red envelope in the assembly). |

Assumptions to check on the robot (also under *Still to verify*):
- **Hole B** position and axis come from the R1 head model (vertical bore at x 20.49, y ±42.0 mm, entering a ~45° slope).
  The lug's Ø3.8 hole allows ±0.4 mm. Before printing, screw an M3 into the hole by hand: it must go at least **7 mm**
  deep (the M3 × 16 reaches 6.3 mm below the hole's centre).
- **Plugs:** USB-C overmold up to 13 × 8 mm; JT128 plug up to 23 × 20 mm (v0.9: an M8 plug, assumed Ø12 × 40 mm). Measure yours.

`v07_changes.png` shows the v0.7 changes in place (v0.8 replaced the lug, the USB-C outlet and the pad shape).

## What changed in v0.6

| Review point | v0.6 |
|---|---|
| P2 / P3: the rear arm (to the back of the head) sticks out where it joins the ear cup | It did. The rear arm is a band that follows the head, and its front end ran 10 mm into the ear cup as a square-cut block that stood up to 3.3 mm (P3: 3.4 mm) proud of the cup's outer face. The arm is now trimmed flush with that face before it is rounded, so it runs into the cup level with the face, and its new edges get the same R1.5 as the rest of the arm. |
| Print P2 / P3 on the ear cup's outer face | That face is now one flat face of 20 cm² (the cup face plus the flush end of the arm), and nothing stands proud of it. The build checks this: highest point above the plane +0.000 mm on both pieces. `print_prep.py` now lays P2 / P3 on it, which also needs less support than before (20 cm² instead of 37 cm²). |

Nothing else on P2 / P3 changed: the hinge knuckle and bore, the ear socket, the clamp flange, M5 hole and nut pocket,
and the ear-ring and rear pad pockets are identical, and so are the pads. The trim removed 0.17 cm³ (P2) and 0.19 cm³ (P3), at the
rear arm's root. `P2_v05_vs_v06.png` shows the junction before and after. The clamp is unaffected: `../clamp_fe.py` on the same mesh gives
17.4 N/mm (v0.5: 17.5), i.e. about 52 N per side at the 3 mm gap, with a peak stress of 6.6 MPa (v0.5: 6.5), and that
peak is at the knuckle, not at the trimmed root.

## What changed in v0.5

| Review point | v0.5 |
|---|---|
| P1: the pod taper is asymmetric (its right side differs from the left) | It was. The taper was a ruled loft from the collar's bottom circle to an ellipse on the beam. OCC's loft step split the circle into front and rear halves but split the ellipse at a side point, so the rear of the collar was ruled onto one quarter of the foot and the surface twisted to the robot's right (up to 9 mm off its mirror image). It is replaced by a **web**: the beam's own side profile rises up into the collar, at the beam's 32 mm width, with its rear face blended into the beam by an R8 arc and R1.5 edges like the rest of the beam. The web and beam are one 2D extrusion, so they are symmetric by construction (mirror check: no point of the surface more than 0.3 mm from its mirror image). |
| P1: extra material at the side might block the hinge pin | It did on the right. The twisted flank came down to 1.6 mm from the pin axis just behind the rear lug, which is the path of the punch that pushes the dowel out. The web stays inside the beam width (\|y\| ≤ 16 mm). Behind the rear lug there is now at least 7 mm of clearance around the pin axis on both sides. In front of the front lug (the insertion side) nothing changed: 47 mm is clear for the 32 mm dowel. |
| P1: notch in the seat rim beside the front lugs | v0.4 cut a box out of the collar there to trim a thin sliver that lay over the front leaf. The box removed only 0.02 cm³ and left a step in the rim, so it is gone and the collar edge runs round unbroken. |

The two round notches in the sides of the collar are the coves for the roll-hoop shaft collars (H11). They are
intentional and symmetric: the hoop legs run at R42, just outside the Ø76 collar. P1 is 4 cm³ lighter (145.7 vs
149.8 cm³), and its centroid is back on the centre line (it was 0.5 mm to the right). The sliver audit finds 6 faces
under 0.5 mm² out of 224. The two new ones are 0.05 mm² each, where the rear lug barrel meets the collar, and are far
below what a printer resolves.

## What changed in v0.4

| Review point | v0.4 |
|---|---|
| P1: insert bosses stuck out of the pod as "bullet" stubs | Gone. The Ø76 seat disc now runs straight down 14 mm as a solid **collar**, and all four H15 inserts sit inside it with at least 4 mm of wall. Below the collar the pod tapered into the beam (one ruled loft; replaced by the web in v0.5). |
| P1: elliptical scoops on the pod flank | Gone. They came from the H1 pin's insertion path behind the rear lug. The **dowel now goes in from the front** (the brim flares leave a clear path), so nothing is cut out of the pod. |
| P1: ledges and steps beside the knuckles | Gone. Each lug is now a flat **hinge leaf** (the beam profile widened to ±27 mm over the lug span) with the Ø12 barrel half sunk underneath, like a normal 3-knuckle hinge leaf. The side piece's knuckle runs in a plain slot between the leaves (flat roof, 0.9 mm clearance). |
| P1: faceted nose and stubs behind the face plate | The brim is a flat 5 mm plate that flares into the face with R34 blends. The keel that carries the forehead pad is a flat 2D profile. The face plate is 8 mm thick at the ends, so the H8 inserts need no bosses, and 4 mm in the middle. The visor top is one continuous 30° bevel. |
| Fillets / chamfers on all parts | Every printed part is rounded (table below), plate corners have radii, and hole mouths have lead-in chamfers (heat-set inserts self-centre). |
| Printability | `../print_prep.py` lays every printed part on its best flat face and writes bed-ready STLs to `stl_print/`, with the support area. P1 was audited for slivers: 5 faces under 0.5 mm² remain, out of 218, all below what a printer resolves. |

Unchanged from v0.3: the closed-ring clamp (3.0 mm gap, roughly 40–50 N per side, see *Clamp force* below), all fastener types and engagement lengths,
the measured ear-disc sockets, pads, and every interface position (seat plane, camera face, hinge axis, pad positions).

**Rounding per part.** OCC builds every rounding in this list. A rounding it can't build is logged in
`cad_build_report.json` under `roundings_not_built`.

| Part | Rounding |
|---|---|
| P1 crest | beam + pod web + leaf outline edges R1.5 (leaf-end tops R0.4, where the barrel top is 0.5 mm under), web-to-beam R8 profile blend, barrel ends 0.6 chamfer, seat rim 0.8 chamfer, collar bottom R5, face plate corners R6 and edges R0.8, brim edges R2/R1.5, keel edges R1.5, lead-ins on every hole |
| P2 / P3 side pieces | crown arm + knuckle + skull tab outline R1.2 (tab corners R5 / R4 / R2, foot edges R1, foot-to-tab corner R0.8, counterbore rim R0.8), rear arm R1.5 (including the edges where v0.6 trims it flush with the cup face), clamp flange R2, cup skirt R1.5, ear socket bore R0.8, window R1, lead-ins on the pin bore and M5 hole. The clamp mating face is left as a plain 90° edge. |
| P4 / P4b / P5 plates | P4: R3 blends between the disc and hoop ears (no inside corners). P4b: R3 blends between the disc and its three ears. P5: R4 corners. All: 0.4 mm edge break top and bottom. |
| P6 hood | one U-profile with R3 / R1.5 / R1 corners, front edge R1, ears R1, USB-C outlet edges R0.8 |
| P7 plug cradle (v0.11) | tabs, bridge, shelf and walls fused first, then every edge R0.6 except the flat bottom |
| P7b cable clip | every edge R1.5 |
| P8 pads | 0.3 chamfer on the edges that go into the pockets (not on the ear rings) |
| B1 / B2 / B6 / B8 acrylic (v2.0) | R4 outer corners on B1 and B2, R3 jumper window, R2 inside corners on the right panel's hub notch, obround tie slots, R0.6 relief holes at every slot corner and tab root (v2.5); laser-cut edges |
| B9 / B10 caps | corner blocks R1.5, B9 rail R1, divider ribs R0.8, drum fork R0.8; 2 × 45° bell mouth on the cable slot (the faces against the acrylic stay sharp so they sit flush) |
| B11 former + rib | 0.4 chamfer on the flange rims, each groove one 45° wall; rib front edge at 45° |
| B12 hub cradle | back edges R0.8, end-lip corners R1.85 (matching the notch's R2) |
| B13 spool / B14 ears | spool disc rim and lip 0.6 chamfer; ears every edge R1.5 except the face on the panel, strap slot R1.4 |
| B3 / B3b carriage | B3: cheek and bridges rounded as one body (v2.5), R1 outside and in the inside corners; its back face (on the bed) and the bridge ends stay sharp. B3b: edges R1 except its outer face and inner face (full walls round the mortises). Tenon ends 0.4 chamfer |
| B4 pulley | 0.4 chamfer on both rims; each groove has one 45° wall so it prints without support |
| B5 hooks (v2.6) | plate and arm rounded as one body: plate corners and the inside corner at the arm R2.5, every other edge R1.5; lips R1; the faces on B9, against B1 and towards the holder stay sharp |

## Printing

**Bill of materials:** `BOM.md` (or `BOM.csv` for a spreadsheet) lists every print, every screw and insert to buy,
and the tools. For the fit-check build every custom part is printed, including P4, P5 and P9, which are aluminium and
steel rod in the final build.

Bed-ready STLs are in `stl_print/` (already rotated, z = 0 is the bed), each named with the material to print it
in, e.g. `P2_side_piece_L (PETG).stl`. P1 says (PLA) for the fit-check; reprint the final P1 in ASA. `print_orientation.png` shows each part on
the bed, with the faces that need support in red. Numbers come from `print_orientation.json` (support = area facing
more than 45° downward).

| Part | Material | Face on the bed | Support | Bed contact | Size on the bed (mm) | Settings |
|---|---|---|---|---|---|---|
| P1_crest | ASA (PLA fit-check) | camera face; front (tilted 30 deg) | 36.1 cm² | 38.2 cm² | 124 x 128 x 140 | 4 walls, 30 % gyroid, 0.2 mm layers; tree supports where shown red |
| P2_side_piece_L | PETG | ear cup outer face; robot left side (tilted 8 deg) | 21.9 cm² | 20.1 cm² | 136 x 122 x 61 | 4 walls, 40 % gyroid (the clamp force assumes this), 0.2 mm layers |
| P3_side_piece_R | PETG | ear cup outer face; robot right side (tilted 8 deg) | 21.9 cm² | 20.1 cm² | 136 x 122 x 61 | 4 walls, 40 % gyroid (the clamp force assumes this), 0.2 mm layers |
| P4_lidar_seat_plate | PETG with heat-set inserts (final option: CNC Al 6061, tapped) | seat face (bottom, on P1); countersinks open upwards; bottom (tilted 24 deg) | 0.1 cm² | 50.0 cm² | 98 x 80 x 4 | 100 % infill, 0.2 mm layers; press the M3 short heat-set inserts (H19, H20) in flush from the top |
| P4b_lidar_adapter | PETG (final option: CNC Al 6061; Hesai recommends a grounded metal mount) | underside (sits on P4); the lidar countersinks face the bed (45 deg cones, no support); bottom (tilted 24 deg) | 0.3 cm² | 33.6 cm² | 66 x 73 x 4 | 100 % infill, 0.2 mm layers; no inserts (the JT128 screws go into the lidar's own threads) |
| P5_camera_cradle | PETG fit-check (final: CNC Al 6061) | camera face (front); countersinks open upwards; front (tilted 30 deg) | 1.0 cm² | 35.3 cm² | 120 x 31 x 4 | 100 % infill, 0.2 mm layers |
| P6_brow_bumper | TPU 95A | back face (against P5); back (tilted 30 deg) | 0.5 cm² | 7.4 cm² | 112 x 30 x 31 | 3 walls, 25 % gyroid, slow (TPU) |
| P7_connector_guard | PETG | bottom (the face on P4) on the bed; bottom (tilted 24 deg) | 0.0 cm² | 8.0 cm² | 47 x 28 x 16 | 4 walls, 40 % gyroid, 0.2 mm layers; no support |
| P7_connector_guard_mirror | PETG | bottom (the face on P4) on the bed; bottom (tilted 24 deg) | 0.0 cm² | 8.0 cm² | 47 x 28 x 16 | 4 walls, 40 % gyroid, 0.2 mm layers; no support; **print this instead of P7 only if your plug's cable leaves to the right** |
| P7b_cable_clip | TPU 95A | back face (the cable grooves) up; front | 0.7 cm² | 3.2 cm² | 21 x 20 x 9 | 3 walls, 25 % gyroid, slow (TPU) |
| P8a_crown_pad_1 | TPU 95A | flat pocket end; top (tilted 28 deg) | 0.0 cm² | 0.7 cm² | 10 x 10 x 7 | 100 % infill (solid pads), slow (TPU) |
| P8a_crown_pad_2 | TPU 95A | flat pocket end; top (tilted 28 deg) | 0.0 cm² | 0.7 cm² | 10 x 10 x 7 | 100 % infill (solid pads), slow (TPU) |
| P8a_crown_pad_3 | TPU 95A | flat pocket end; top (tilted 20 deg) | 0.0 cm² | 0.7 cm² | 10 x 10 x 8 | 100 % infill (solid pads), slow (TPU) |
| P8a_crown_pad_4 | TPU 95A | flat pocket end; bottom (tilted 21 deg) | 0.0 cm² | 0.8 cm² | 11 x 10 x 8 | 100 % infill (solid pads), slow (TPU) |
| P8b_forehead_pad | TPU 95A | flat pocket end; robot right side | 0.0 cm² | 0.7 cm² | 13 x 12 x 10 | 100 % infill (solid pads), slow (TPU) |
| P8c_ear_ring_pad_L | TPU 95A | lying flat, cone axis vertical, on supports; robot right side (tilted 8 deg) | 9.3 cm² | 0.0 cm² | 42 x 42 x 6 | 100 % infill (solid pads), slow (TPU) |
| P8c_ear_ring_pad_R | TPU 95A | lying flat, cone axis vertical, on supports; robot right side (tilted 8 deg) | 9.3 cm² | 0.0 cm² | 42 x 42 x 6 | 100 % infill (solid pads), slow (TPU) |
| P8d_rear_pad_L | TPU 95A | flat pocket end; robot left side (tilted 45 deg) | 0.0 cm² | 0.7 cm² | 10 x 8 x 10 | 100 % infill (solid pads), slow (TPU) |
| P8d_rear_pad_R | TPU 95A | flat pocket end; front (tilted 45 deg) | 0.0 cm² | 0.6 cm² | 10 x 8 x 10 | 100 % infill (solid pads), slow (TPU) |
| P9_roll_hoop | PETG fit-check (final: Ø6 steel rod bent R42) | lying flat in its plane, on supports; front (tilted 24 deg) | 9.6 cm² | 2.2 cm² | 90 x 103 x 6 | 100 % infill, brim; supports under the round rod |
| B3_takeup_carriage | PETG | back cheek (counterbore side) on the bed; back | 0.6 cm² | 47.3 cm² | 64 x 75 x 38 | 4 walls, 40 % gyroid, 0.2 mm; 1 M3 short insert (spring, from below) |
| B3b_carriage_cheek | PETG | outer face on the bed (nut pocket and countersinks open downwards; the axle boss up); front | 0.4 cm² | 47.3 cm² | 64 x 75 x 22 | 4 walls, 40 % gyroid, 0.2 mm layers |
| B4_takeup_pulley | PETG | back face (flat side of both grooves) on the bed; back | 4.3 cm² | 25.9 cm² | 61 x 61 x 26 | 4 walls, 40 % gyroid, 0.15 mm layers (bearing seats) |
| B5_handle_hook_L | PETG (TPU tape on the bar face optional) | top face (plate and arm) on the bed; the lips point up; top | 1.8 cm² | 10.3 cm² | 32 x 61 x 32 | 100 % infill, 0.2 mm layers |
| B5_handle_hook_R | PETG (TPU tape on the bar face optional) | top face (plate and arm) on the bed; the lips point up; top | 1.8 cm² | 10.3 cm² | 32 x 61 x 32 | 100 % infill, 0.2 mm layers |
| B7_spring_drum | PETG | flange face; front | 3.1 cm² | 4.4 cm² | 24 x 24 x 13 | 100 % infill, 0.15 mm layers; support from the bed under the top flange |
| B9_top_cap | PETG | top face on the bed (blocks and rail up); top | 2.1 cm² | 51.7 cm² | 133 x 43 x 26 | 4 walls, 40 % gyroid, 0.2 mm layers; 10 M3 short inserts (4 hooks, 2 front panel, 4 side panels), 2 magnets |
| B10_bottom_cap | PETG | bottom face on the bed (fork up); bottom | 1.9 cm² | 56.6 cm² | 133 x 43 x 23 | 4 walls, 40 % gyroid, 0.2 mm layers; 8 M3 short inserts (2 front panel, 6 side panels / ears), 2 magnets |
| B11_uturn_former | PETG | front face (inserts) on the bed; the grooves' 45 deg walls print without support; front | 4.1 cm² | 12.6 cm² | 58 x 44 x 39 | 4 walls, 30 % gyroid, 0.2 mm layers; 2 M3 short inserts (front face) |
| B12_hub_cradle | PETG | front face (inserts) on the bed; the end lips and ledges rise from it; front | 2.5 cm² | 16.4 cm² | 45 x 136 x 18 | 4 walls, 30 % gyroid, 0.2 mm layers; 4 M3 short inserts (2 in the front face, 2 in the back of the rail for B13) |
| B13_cable_spool | PETG | disc face on the bed; the 45 deg lip prints without support; front | 2.1 cm² | 27.0 cm² | 60 x 60 x 23 | 4 walls, 30 % gyroid, 0.2 mm layers; no inserts (2 M3 screws through its disc into B12) |
| B14_strap_ear_L | PETG | front face on the bed (the strap slot vertical); front | 1.4 cm² | 4.3 cm² | 8 x 71 x 20 | 100 % infill, 0.2 mm layers (the strap pulls on the outer bar) |
| B14_strap_ear_R | PETG | front face on the bed (the strap slot vertical); front | 1.4 cm² | 4.3 cm² | 8 x 71 x 20 | 100 % infill, 0.2 mm layers (the strap pulls on the outer bar) |

- **P1** stands on its camera face (the flattest, most accurate datum). The lidar seat is then near-vertical, so it
  needs no support and prints true. Use tree supports for the beam arc and the brim. It is 140 mm tall, so check
  your Z height.
- **P2 / P3** lie on the ear cup's outer face (flat since v0.6). The two arms rise from it on tree supports; the skull tab needs none. Ream the hinge bore to Ø4 H8 after printing.
- **Cable box (v2.8):** the box panels are laser-cut (`laser_cut/`, 3 mm cast acrylic; measure the sheet and set
  `ACR_T` in `../cable_box_cad.py` if it isn't 2.9–3.1 mm). Printed parts, all PETG:
  - the caps print on their outer faces, with the blocks up;
  - the former, cradle and ears print on their front faces, the spool on its disc;
  - the B5 hooks (L and R, mirror images) stand on their top face with the lips up;
  - B3b goes on its flat outer face;
  - B7 is the only part that needs support, from the bed under its top flange.
  - **Heat-set inserts:** 25 short M3 inserts (C6), all Ø4.0 × 5.5 holes.
  - **Magnets:** press the 4 cap magnets in flush (same pole out); glue the lid's 4 into its holes with the opposite
    pole facing them.
- **P7** (PETG, v0.11): flat bottom on the bed, no support.
- **TPU** (P6, P7b, pads): slow, no retraction. Print the ear ring pads flat on tree supports. The other pads need none.
- **Heat-set inserts** (H15 in P1, H19 in P4): 245–260 °C for ASA, about 230–245 °C for PETG. Push straight in, using the lead-in chamfers to centre them, until flush with the top. The P4 holes go through, so a little plastic may push out underneath: trim it flush.
- **P4 / P4b / P5 / P9 printed for the fit-check:**
  - P4 and P5 print solid, countersinks up. P4 takes 5 short heat-set inserts (H19: M3 × 3–4); tighten the screws only snug.
  - P4b prints solid on its underside: the lidar countersinks face the bed and print as 45° cones, with no support.
  - The printed P9 only checks the fit and the lidar clearance; it isn't a roll hoop.
- Fit-check P1 in PLA, but print P2/P3 in PETG straight away: they flex to make the clamp force, and PLA would clamp
  about twice as hard. The final structural prints are ASA (P1, near the lidar's heat) and PETG (P2/P3).

## Opening in SolidWorks

1. **File → Open**, set the file type to *STEP*, and pick `R1_sensor_helmet_v0.11_ASSEMBLY.step` (and, if you
   want the cable box with it, insert `R1_cable_box_v2.8_ASSEMBLY.step` at the origin: same frame; it includes every screw, insert, nut, magnet and bearing,
   coloured by kind: screws silver, nuts dark grey, inserts brass, magnets red, 627ZZ blue).
2. The assembly opens with these groups:
   - `helmet`: parts, `P8_pads`, `hardware`
   - `sensors` (the JT128 modelled from its manual's drawings, outside only; the D435 is Intel's model)
   - `R1_reference`: the decimated head
3. If 3D Interconnect opens the parts read-only, use **Break Link** (or turn it off under *Tools → Options → Import*).
4. Nothing is mated; everything is fixed where the design puts it. The side pieces are shown at the **design gap**:
   flanges 1.0 mm apart, rear pads just touching, ear ring pads 0.5 mm into the ear discs (their designed preload).
5. The red bodies in `sensors` are **assumed** cable-plug envelopes (JT128 M8 plug, USB-C plug).

**Frame (all 3D files):** +X = robot left, +Y = up, +Z = robot forward. The origin is on the neck yaw axis
(R1 `head_yaw_link`). Every part file uses the same frame, so a part inserted at the origin lands in place.

**Interference Detection in SolidWorks:** leave *"treat coincidence as interference"* off. Pads, inserts, nuts,
washers and screw heads seat face-to-face on their parts by design. Threads are drawn at the minor diameter.

## Assembly order and fasteners

| Step | What | Fastener → what it screws into | Engagement | Tool |
|---|---|---|---|---|
| 0 | bench: heat-set the inserts | H15: 6× brass M3×5.7 (OD 4.6) into the Ø4.0 holes, 4 in the P1 collar and 2 in the face-plate ends | – | soldering iron |
| 0b | bench: heat-set the P4 inserts | H19: 5× brass M3 × 3–4 short into the Ø4.0 holes, flush with the top | – | soldering iron |
| 1 | P4 plate → P1 | H6: 4× ISO 10642 M3×10 flat head → H15 inserts (do this first: P4b covers them) | 5.7 mm (full insert) | 2 mm hex, from above |
| 2a | bench: JT128 → P4b | H4: 3× ISO 10642 M3×8 flat head, from below P4b → the JT128's M3 holes, **0.5–0.6 Nm** | 4.0 mm (manual: 3–5) | 2 mm hex, from below |
| 2b | P4b (with the lidar) → P4 | H21: 3× ISO 7380 M3×6 through the ears → H19 inserts in P4; fits one way only | 3.0 mm | 2 mm hex, from above |
| 3 | P7 plug cradle → P4 | H9: 2× ISO 7380 M3×6 → H19 M3 inserts in P4; plug the JT128 cable in (coupling nut 0.4 Nm), its body in the cradle; let the cable bend down in a loose loop (≥ R28) to the first zip-tie base | 3.0 mm | 2 mm hex, from above |
| 4 | hoop P9 through the P4 ears | H11: 2× Ø6 shaft collars underneath (they sit in the coves in the collar's sides) | – | set screws |
| 5 | camera → P5 (bench) | H7: 2× ISO 10642 M3×6 flat head from behind → D435if M3 | 2.0 mm (max 3) | 2 mm hex |
| 6 | P5 + P6 hood → P1 face | H8: right ISO 10642 M3×10 countersunk in P5; left ISO 7380 M3×12 + H16 washer through the P6 ear → H15 inserts | 5.7 / 4.5 mm | 2 mm hex, from the front |
| 7 | crest on the head, side pieces on the hinges | H1: Ø4×32 dowels **pressed in from the front** of the front lug; H2: 4× M3×4 grubs → tapped M3 in the P1 lug barrels | 4.0 mm | 1.5 mm hex, from the side |
| 7b | screw the side pieces to the skull | H17: 2× ISO 4762 M3×16 + H18 ISO 7089 washer, vertically through the counterbore in each skull tab → the R1's threaded hole B | ~6 mm (check ≥ 7 mm of thread first) | 2.5 mm hex, from above |
| 8 | close the clamp | H3: DIN 464 M5×25 knurled thumbscrew through P2 → H14 ISO 4032 M5 nut, captive in P3. **Tighten until the flanges touch** (1 mm). | 4.7 mm (full nut) | by hand |
| 9 | cables on the helmet | plug the D435if's USB-C in **after** the H8 screws (it runs over the right one), through the notch at the back of P6's right cheek, then back along the right side; the JT128 lead from P7 down the left side; both on stick-on zip-tie bases (about 3 a side, keep bends ≥ R28 for the JT128) | – | – |
| 10 | P7b clip | on the rear clamp flanges' back face, each cable in the groove on its own side; 2 zip ties round the flange block (top, front, bottom), each down one groove over its cable | – | – |
| 11 | cable box (bench) | heat-set the 25 C6 inserts (B9 10, B10 8, B11 2, B12 4, B3 1); glue the 8 C7 magnets (caps: same pole out; lid: opposite pole facing); the 627ZZ into B4 from its front face, down onto the shoulder; B5 hooks → B9 (4 × C8) | – | soldering iron, 2 mm hex |
| 11b | box shell | hold B11 and B12 against B1's inside face and screw them on from the robot side (4 × C16), then both caps (4 × C16). Slide the divider in from the back (front tabs into B1, bottom edge between B10's ribs, top tab into B11's slot). Fit the side panels' front tabs into B1 and screw them to the caps (6 × C16; at the bottom front the B14 ears go on with 4 × C17). The right panel's notch goes round B12's end lips | 4 mm into each insert | 2 mm hex |
| 11c | hub, spool, take-up | hub into B12 from the back on Dual Lock, dongle into a USB-A from outside, ethernet plug into its RJ45 from below if used; then B13 spool → back of B12 (2 × C16, through the spool's centre). Splitter on Dual Lock low on the divider. Spring coil on B7, B7 in B10's fork on the C4 M4 × 30 + washer + nyloc (snug, it must spin) | – | 2 / 3 mm hex, 7 mm spanner |
| 12 | cables and lid | lay both head cables into B9's slot from the back, over B4's grooves, and assemble the carriage round them (B4 between B3's cheeks on the C3 axle, B3b onto B3's tenons, nut into B3b's pocket); spring tape → B3 with C9 from below. Cables up over B11 and zip-tie them to its rib with the head at zero pose and the pulley centre 49 mm above its lowest point. JT128 lead: unscrew B13, wind 3 turns on it, screw it back (2 × C16 through its open centre), then down to the splitter. Hang the box on the upper holder: each hook's arm on a back corner, the outer lip outside the side arm, the tooth inside the corner. Strap the ears to the U-frame bars. Hub USB-C out through B1's window, up behind the crossbar, over it and into the Jetson's Type-C; jumpers and ethernet out through the window too; lid on | – | by hand |

To take a hinge apart: loosen both grubs, then push the dowel out from the back with a punch of Ø3 mm or less (behind
the rear lug there is at least 7 mm of clearance around the pin axis), or press it out forwards.

**Clamp tuning.** The skull screws hold the helmet; the clamp only closes the ring (about 29 N on the thumbscrew
with the flanges touching). The ear cups get about 27 N each (about 19 N from the thicker ear pads
alone). For a snugger or looser ear fit, print the ear ring pads thicker or thinner (slicer Z scale): each 0.1 mm
is about 4 N. Don't use the thumbscrew to set the force; it is only there to close the joint.

## Contents

| File | What |
|---|---|
| `R1_sensor_helmet_v0.11_ASSEMBLY.step` | full helmet assembly |
| `R1_cable_box_v2.8_ASSEMBLY.step` | cable box with every screw, insert, nut and magnet, and placeholders for the hub, dongle, splitter and spring; same frame as the helmet |
| `laser_cut/*.dxf` | the cable box's acrylic panels (B1, B2, B6 L/R, B8), 1:1 mm cut lines, and all five nested on one sheet |
| `parts/*.step` | every part, pad and sensor in the assembly frame (`S1_JT128.step` = from the Hesai manual's drawings, `S2_D435if.step` = Intel's model); `B*.step` = cable box parts (acrylic and printed) |
| `BOM.md`, `BOM.csv` | bill of materials: prints, hardware to buy, tools |
| `stl_print/*.stl` | the printed parts rotated onto the bed, ready for the slicer; the material is in the name, e.g. `P2_side_piece_L (PETG).stl` |
| `reference/R1_head_reference_mm.stl` | full-resolution R1 head, mm, same frame |
| `reference/D435_Solid.SLDPRT`, `.step` | Intel's D435 model (yours) and the STEP SolidWorks saved from it, which the build reads |

The build's working files (raw STLs in the assembly frame, DXFs of P4 / P4b / P5, `mesh_robot_frame.npz`,
`cad_build_report.json`, `print_orientation.json` / `.png`) go to `../build/`, which git ignores.

## Materials

| Part | Fit-check | Final |
|---|---|---|
| P1 crest | PLA | **ASA** (sits under the lidar's 7.5–10 W); solid model, 30 % gyroid, 4 walls |
| P2/P3 side pieces | PLA | **PETG**, 4 walls, 40 % gyroid (the clamp force assumes E ≈ 1.5 GPa) |
| P4 lidar plate, P4b lidar adapter, P5 cradle (all 4 mm) | printed PETG, solid (`stl_print/`), P4 with short heat-set inserts | **CNC aluminium 6061**: for an aluminium P4, drill the 5 insert holes Ø2.5 and tap M3 at the same centres; break edges 0.4. An aluminium P4b is also the grounded metal mount Hesai recommends for the lidar |
| P6 hood | – | TPU 95A |
| P7 plug cradle (v0.11) | PETG | PETG |
| P8 pads | – | TPU 95A printed solid (conformal STLs), CA-glued into their pockets |
| P9 hoop | printed PETG (fit and clearance only) | Ø6 spring-steel / 304 rod bent R42, matte-black sleeve |

Hole sizes in the printed parts:
- heat-set inserts M3 (P1): Ø4.0 × 8 deep, with a 0.5 lead-in;
- short heat-set inserts in P4: Ø4.0 (M3), through the 4 mm plate, 0.4 lead-in;
- P4b: Ø3.4 with a 90° countersink Ø6.8 underneath (JT128 screws), Ø3.4 with a Ø6.5 × 1 counterbore on top (ears);
- tapped M3: Ø2.5;
- M5 clearance: Ø5.5;
- the M5 nut pocket: 8.1 mm across flats;
- hinge bores: Ø4.0; ream them to fit the dowel.

## Still to verify

- **Cable box: what is still assumed** (the hub is measured since v2.4):
  - **Jetson port deck:** not in the R1 meshes.
    - Ports placed from your photos at x ≈ −76, y +27 / +7 / −11 / −33 (USB-A / XT30 / RJ45 / Type-C), port tops
      25 mm below the holder's top.
    - The hooks keep \|y\| ≥ 42 clear, and the hub cable has at least 14 mm spare anywhere from 6 mm behind to
      6 mm in front of that.
    - Worth checking: the gap from the Type-C plug's outer side to the holder's side arm should be at least 6 mm (about 9
      from the photos); the hook's tooth sits in it.
  - **Upper holder:** from the R1 mesh: side arms \|y\| 48–56 (104 mm between their inner faces), corners R8, crossbar Ø8.
    The hooks leave 1.5–1.8 mm, so a holder more than about 1.5 mm off that won't take them. Measure between the side
    arms' inner faces before printing B5.
  - **Things on the holder:** your photos show a stand pole and a printed clamp on the holder. The box needs the space
    behind the Jetson, and the hooks need the holder's two back corners free.
  - **Hub:** its RJ45 assumed in the middle of its bottom end, its USB-C cable in the middle of its top end.
  - **Hub cable plug:** assumed 25 mm rigid.
  - **Acrylic:** sheet thickness 3.0 mm.
  - **Splitter:** 56 × 20 × 15 mm.
  - **Dongle's port:** height along the hub.
- **Sensors.** S2 is Intel's D435 model (`reference/D435_Solid.step`). S1 is modelled from the JT128 manual: the base,
  height and screw holes are dimensioned there; the body and dome outline was traced from its side view (±0.5 mm), and
  the hole angles were measured off the bottom view (±0.3°, about ±0.15 mm on the Φ56 circle; P4b's Ø3.4 holes allow
  ±0.2 mm). Print P4b first (18 g) and screw it to the lidar to check before printing P4 and P7. Hesai supplies a STEP model on request;
  swap it in at `lidar_base_centre_robot_frame` in `cad_build_report.json` if you get it.
- **Head geometry.** The head-fitting surfaces come from the URDF simulation mesh. Do the PLA fit-check first.
- **Skull hole B.** Position and axis come from the R1 head model. Screw an M3 in by hand before printing P2/P3:
  it must go at least 7 mm deep. If a tab's hole doesn't line up, open it to Ø4.5; the washer still covers it.
- **Plug sizes.** JT128 M8 plug assumed Ø12 × 40 mm from the socket's hex; USB-C overmold assumed 12.5 × 6.5 mm and 22 mm long. A USB-C
  overmold up to 7.5 mm thick still fits (the notch in Intel's camera allows that); a longer one just runs further
  out past P5 (red envelopes in the assembly).
- **Clamp force.** `../clamp_fe.py` (PETG E = 1.5 GPa, solid TPU pads, the skull screw as a fixed support) gives, with
  the flanges touching and the ear pads preloaded: thumbscrew 29 N, ear cup 27 N, rear pad 14 N,
  skull-screw load 36 N, peak stress 7.0 MPa (on the crown arm's front edge just under the skull
  tab). With the thumbscrew loose the ear cup still gets about 19 N from the pad preload. Per mm of
  flange closure (no preload): 20.9 N/mm. The FE is
  mesh-sensitive (see v0.4–v0.6 notes: up to ±25 %), so treat these as ±25 %.

## Regenerating

```bash
conda activate r1_helmet_cad
python helmet_cad.py        # builds, checks, exports cad/ (~11 min)
python cable_box_cad.py     # cable box: builds, motion checks against the R1, exports cad/ (after helmet_cad.py)
python print_prep.py        # orients the printed parts -> cad/stl_print/, print_orientation.*
python cad_preview.py       # renders cad/preview_assembly.png
python fastener_sections.py # renders cad/fastener_sections.png
python helmet_drawings.py   # regenerates the drawing PDF from the CAD
python clamp_fe.py          # contact FE of the rear clamp, ~13 min (needs: pip install gmsh scikit-fem)
```

## Build checks (last run)

Helmet build v0.11: lidar tilt 24 deg, seat centre [-14.0, 0.0, 100.84] (robot frame, mm); JT128 base 4 mm above it on P4b.

| Check | Result |
|---|---|
| B-rep validity (19 bodies) | all valid, each part one solid |
| R1 head-mesh vertices inside structural parts (all vertices) | none |
| Pads on the head (shaped to touch; ear ring pads preloaded 0.5 mm) | max contact depth 0.49 mm |
| Ear disc (L) surface-of-revolution fit | rms 0.016 mm; skirt gap to head min/mean 0.72/1.69 mm; skirt overlap on the rim min/mean 0.63/1.51 mm |
| Interference, every pair of bodies (parts, pads, sensors, hardware; 2080 pairs) | none (> 0.05 mm³) |
| Thread engagement (21 threaded fasteners, measured from the solids) | all OK: M3 insert 2.9-5.7 mm; JT128 M3 4.0 mm (manual: 3-5); D435if M3 2.0-2.0 mm; M3 tapped ASA 3.7-3.7 mm; M5 nut 4.7-4.7 mm |
| Rear clamp + skull screws (contact FE) | 1.0 mm design gap, flanges touching, ear pads preloaded: thumbscrew ~29 N, ear cup 27 N (19 N with the thumbscrew loose), rear pad 14 N, skull screw 36 N; FE ±25 % (see *Clamp force* above) |
| Edge roundings | all built |
| P2 / P3 print base (ear cup outer face) | P2_side_piece_L: flat, nothing proud of it; P3_side_piece_R: flat, nothing proud of it |
| Tool access (25 fasteners, each at its assembly step; H4 from below P4b on the bench) | all clear |
| Hinge swing 15° | P2_side_piece_L: crest overlap 0.0 mm³, head points 0; P3_side_piece_R: crest overlap 0.0 mm³, head points 0 |
| Lidar FOV (JT128 optics: lowest channel −4.4° from its optical centre) | highest body P6_brow_bumper at −21.1° (then P5 −24.6°, P1 −24.7°, S2 −25.2°, P7 −46.1°, P7b −83.2°) |
| What the 24° tilt buys (JT128 manual optics) | optical centre 1.315 m up; lowest beam −28.4° ahead, +19.6° behind; floor seen from 2.46 m ahead, 0.762 m table tops from 1.05 m (from the neck axis) |
| Camera depth frustum (87×58°) | P6_brow_bumper: 0, P1_crest: 0, P5_camera_cradle: 0 vertices inside |

Cable box v2.8 (`../cable_box_cad.py`, `../build/cable_box_report.json`), checked against the R1's own meshes and the
URDF joint ranges, with the box mounted on the waist (it turns and leans with the torso). The box is built with the hub on
+y and mirrored before the robot checks; the checks inside the box don't change under a mirror.

| Check | Result |
|---|---|
| B-rep validity (5 acrylic panels, 13 printed parts, after the mirror) | all valid; no faces under 1 mm² or edges under 0.2 mm |
| Interference, every pair of parts (panels, printed parts, hub, dongle, splitter, spring), with the carriage, pulley and the 627ZZ at the bottom, middle and top of their travel | none (> 0.05 mm³) |
| Fasteners (27: every screw, axle and insert) | 27 OK: clear holes through every part they pass, heads seated, ≥ 1 mm of plastic round every insert, tips clear of the hole bottom; thread engagement 4.0–4.0 mm |
| Holes to the edges (v2.5) | B5 countersinks ≥ 3.4 mm of plate in plan (≥ 2.85 at the surface); B14 counterbores ≥ 2.9 mm to every edge (≥ 2.35 at the surface), 3.4 mm apart |
| Fits measured between the exported parts | pulley float 0.15 each side; carriage 0.60 to all four walls; caps 0.15 to the side panels; spool 2.55 to the right panel and to the divider (the wound lead 2.0) |
| Lid magnets | 4 pairs, lid hole and cap pocket on one axis, ≥ 0.8 mm pocket walls |
| Laser DXFs | B1 145 × 261, B2 145 × 261, B6 L and R 49 × 261, B8 49 × 223 mm; closed outlines, each matches its solid |
| Cables in the box (JT128 lead Ø5.5, RealSense Ø4.5): entry slot → pulley → former → rib, carriage at 3 heights | nothing in the way; they sit in the pulley's and former's grooves and are ≥ 0.8 mm from everything else |
| Spare JT128 lead: about 3 turns on the B13 spool (r 25–30.5) | clear of everything |
| Hub RJ45: 38 mm plug + boot under the hub, the cable's bend, and the channel up to the window | clear of every part |
| Hub ports: the port face pushed 45 mm out of the right panel | clear of every part |
| Take-up | carriage travel 117.4 mm → stores 235 mm of cable (2:1 loop); 0.6 mm to each acrylic wall |
| R1 torso, upper holder and U-frame vs the box, hooks, ears and screw heads, as mounted (dense surface samples) | 0 points inside; closest 0.9 mm, between the hooks and the holder (the hooks' clearance to it) |
| **Jetson port deck** (assumed from the photos): the plugs in front of the crossbar (y −40…37, up to z 221), the jumpers rising between B1 and the crossbar and passing over it | clear of the box, the hooks and every screw head |
| **Hub USB-C → Jetson Type-C** (137 mm measured) | needs 114 mm → 23 mm spare (14–29 for the port 6 mm either way); bends R14 and R11; ≥ 0.9 mm from the box, ≥ 1.1 mm from the R1 mesh |
| Head + helmet over the neck range (pitch ±36°, yaw ±115°) | min 14.1 mm |
| Pelvis + hips over the waist range (roll ±30°, yaw ±150°) | min 28.4 mm |
| Arms: 6384 poses per arm over the shoulder and elbow ranges, Revo 2 hand as an r 40 capsule; box **plus the plugs in the hub's ports** | 0 / 0 poses (left / right) touch them without also hitting the torso |
| Head-cable span, P7b clip → top slot (straight line) | 31–260 mm over the full neck range; 115 mm at zero pose |

**How far the head may turn.** The span depends mostly on head yaw (pitch ±36° throughout):

| Head yaw limit | Slack the take-up must store (it stores 235 mm) | Span's gap to the neck / torso |
|---|---|---|
| ±60° | 193 mm | 26.9 / 25.9 mm |
| ±70° | 201 mm | 18.6 / 18.5 mm |
| ±80° | 209 mm | 10.5 / 11.3 mm |
| ±90° | 216 mm | 2.0 / 4.7 mm |
| ±100° | 222 mm | 0.8 / 0.9 mm |
| ±115° (joint limit) | 229 mm | 0.4 / 0.9 mm |

Past about ±90° the cables lie on the neck and wrap round it, which takes more cable than the straight line. Near the
joint limit the take-up would reach its end stop and the cable would pull on P7b and the plug. **Limit head yaw to ±80°
in software:** the cables then stay at least 10.5 mm off the neck and torso, with about
6 mm of carriage travel spare at each end (set the carriage as in assembly step 12).
