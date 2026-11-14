# R1 sensor helmet v0.11 + cable box v2.8 — bill of materials

This BOM is for the **fit-check build**, where every custom part is printed. The *Final* column says what each part
becomes once the fit is verified. Quantities, specs and print settings come from the v0.10 CAD (`cad_build_report.json`,
`cable_box_report.json`, `print_orientation.json`). The same list is in `BOM.csv` for a spreadsheet or shopping list.

## What to print for v0.11 (P7 fix)

Helmet: compared with v0.10, print **1 file**: every other file is byte for byte the same as v0.10.

| File | Why |
|---|---|
| `P7_connector_guard (PETG).stl` | **redone**: v0.10's P7 was two loose pieces (its cable former never touched the shelf). The former is gone (P7b takes the cable's pull), and the cradle now stands on one flat bottom: PETG, no support, 6 g. Don't print the v0.10 `(TPU)` file. Print `P7_connector_guard_mirror (PETG).stl` instead only if your plug's cable leaves to the right |

## What to print for v0.10 (cable routing + cable box)

Helmet: compared with v0.9, print **2 files**; every other helmet file is byte-for-byte the same as v0.9 (P4 and
P4b included, so the v0.9 prints stay valid):

| File | Why |
|---|---|
| `P7_connector_guard (TPU).stl` | **superseded by v0.11** (above). v0.10: cradle for the JT128's right-angle plug + R28 former that turns the cable down (strain relief). Print `P7_connector_guard_mirror (TPU).stl` instead only if your plug's cable leaves to the right |
| `P7b_cable_clip (TPU).stl` | **new**: clip on the rear clamp flanges where the JT128 and RealSense cables meet |

Cable box **v2.8**: only **one** NSK 627ZZ, centred under the two cables. **Reprint `B3_takeup_carriage`, `B3b_carriage_cheek` and `B4_takeup_pulley`** (they replace the v2.7 ones); every other file is byte for byte the same as v2.7. The axle stays M5 × 35 (see the README).

Cable box **v2.7**: NSK 627ZZ bearings (7 × 22 × 7) in the pulley instead of 625ZZ. **Reprint `B3_takeup_carriage`, `B3b_carriage_cheek` and `B4_takeup_pulley`**; every other file (prints and laser DXFs) is byte for byte the same as v2.6. The axle stays M5 × 35: each cheek now has a spigot that fills its bearing's 7 mm bore (see the README).

Cable box **v2.6**: only `B5_handle_hook_L` and `_R` changed (the plate and arm are rounded as one body, no square edge where they meet): **reprint those two**. Every other file is byte for byte the same as v2.5. The assembly now colours the hardware by kind, and the bearing fit is checked against SKF's 625-2Z data (see the README).

Cable box **v2.5** (answers to the v2.4 review, see the README): hook and ear holes moved away from the edges, B3 rounded as one body, the spool centred in its column, R0.6 relief holes at every acrylic slot corner and tab root. **Reprint** `B10_bottom_cap`, `B13_cable_spool`, `B14_strap_ear_L`, `B14_strap_ear_R`, `B3_takeup_carriage`, `B5_handle_hook_L`, `B5_handle_hook_R`, `B9_top_cap`; **recut all five panels**. Unchanged: `B11_uturn_former`, `B12_hub_cradle`, `B3b_carriage_cheek`, `B4_takeup_pulley`, `B7_spring_drum`. Hardware is unchanged.

Cable box **v2.4** (checked against your robot measurements, see the README). The box is mirrored, so the hub's USB-C reaches the Jetson's Type-C on the robot's right (about 23 mm spare of the 137 mm). The hooks now sit on the holder's back corners, clear of the Jetson's ports, and B12 fits the measured 121.5 × 32.4 × 14.0 hub. **Print / cut:**

| File | v2.4 |
|---|---|
| `B5_handle_hook_L (PETG).stl`, `B5_handle_hook_R (PETG).stl` | **new**, one of each (replace the two `B5_handle_hook`) |
| `B12_hub_cradle (PETG).stl` | **reprint**: measured hub, mirrored |
| `B3`, `B3b`, `B9`, `B10`, `B11`, `B13` | **reprint**: mirror images of v2.3 (B9 also: hook tongues, deeper front blocks) |
| `laser_cut/B1_front_panel` | **recut**: mirrored, B1 → B9 holes 8.5 mm lower |
| `laser_cut/B6_side_panel_R` | **recut**: now the hub panel |
| `laser_cut/B6_side_panel_L` | the same cut as v2.3's `B6_side_panel_R`, turned over |
| `B2_lid`, `B8_divider`, `B14_strap_ear_L / _R` | unchanged, byte for byte |
| `B4_takeup_pulley`, `B7_spring_drum` | same shape (moved to the other column) |

Cable box **v2.3** (tolerance review, see the README): `B3_takeup_carriage`, `B3b_carriage_cheek` (0.15 mm axial float for the pulley), `B9_top_cap`, `B10_bottom_cap` (0.15 mm to the side panels, press-fit magnet pockets with a lead-in; B10 also wider divider ribs) and `B11_uturn_former` (wider divider slot) changed from v2.2. **Reprint those five**; every other box file, the laser DXFs included, is byte-for-byte the same as v2.2.

Cable box **v2.2**: only `B5_handle_hook (PETG).stl` changed from v2.1 (**reprint ×2**: 22 mm wide, 6 mm plate, 10 mm lip, R3 inside corner); every other box file is byte-for-byte the same as v2.1.

Cable box **v2.1** (answers to the v2.0 review; see the README). Compared with v2.0:

| File | v2.1 |
|---|---|
| `B3_takeup_carriage (PETG).stl` | **reprint**: tenons on the bridge ends instead of the two pilot holes |
| `B3b_carriage_cheek (PETG).stl` | **reprint**: mortises instead of the two countersunk screw holes; nut pocket 5.4 deep (the nut sits on B3b's plastic) |
| `B5_handle_hook (PETG).stl` | **reprint ×2**: plate 5 mm longer behind the rear countersink |
| `B9_top_cap (PETG).stl` | **reprint**: hook inserts at y ±32 (was ±36); 2 mm bell mouth on the cable slot |
| `B11_uturn_former (PETG).stl` | **reprint**: three equal 3.2 mm tie windows, rib 4 mm longer |
| `B12_hub_cradle (PETG).stl` | **reprint**: hub 8 mm lower, notch under the hub's RJ45, 2 inserts on the back for B13 |
| `B13_cable_spool (PETG).stl` | **new**, replaces `B13_coil_ring` (don't print that): spool for the spare JT128 lead, behind the hub |
| `B14_strap_ear_L / _R (PETG).stl` | **reprint ×2**: taller, strap slot at z 22–44 (level with the U-frame bar's bend) |
| `B4_takeup_pulley, B7_spring_drum, B10_bottom_cap` | same as v2.0, byte for byte |
| `laser_cut/B1_front_panel` | **recut**: jumper window 58 × 23 (was 36 × 15), the ring's 3 holes gone, the cradle's 2 holes moved down 8 mm |
| `laser_cut/B6_side_panel_L` | **recut**: hub notch 8 mm lower, front tabs and velcro-tie slots moved |
| `laser_cut/B2_lid, B6_side_panel_R, B8_divider` | same as v2.0 |

Cable box **v2.0** (sections 1b and 1c): the box itself is now **laser-cut 3 mm acrylic** (`laser_cut/`) between
two printed end caps. The v1.0 printed body B1 and lid B2 are gone; don't print them. Every v1.0 box print changed
except B7, so print the whole section 1b list:

| v1.0 file | v2.0 |
|---|---|
| `B1_cable_box_body`, `B2_cable_box_lid` | **gone**: acrylic B1 front panel and B2 lid (plus B6 side panels, B8 divider) |
| `B3_takeup_carriage`, `B3b_carriage_cheek` | 0.6 mm to each acrylic wall (was 0.4), bearing bosses Ø7.4 (inner race only), spring screw recentred in a blind insert hole |
| `B4_takeup_pulley` | bearing seats Ø16.15 (was 15.95: a printed hole comes out undersize, the bearings would not press in) |
| `B5_handle_hook` | screw holes 8 mm apart (was 7: the countersinks nearly met) to suit B9 |
| `B7_spring_drum` | same shape |
| new | `B9_top_cap`, `B10_bottom_cap`, `B11_uturn_former`, `B12_hub_cradle`, `B13_coil_ring`, `B14_strap_ear_L` / `_R` |

## What to print for v0.9 (the real JT128)

v0.8 was drawn round a placeholder lidar with a flange (three M4 lugs on top). The real JT128 has no flange: it
screws on from **below**, with 3 × M3 into tapped holes in its base, and its Φ62.5 base would cover P4's H6 screws.
So it now sits on a small adapter plate (P4b) that screws down onto P4. Compared with v0.8, **3 files** changed;
every other print file is byte-for-byte the same as v0.8:

| File | Why | Print now? |
|---|---|---|
| `P4b_lidar_adapter (PETG).stl` | **new**: the JT128 screws to it from below (countersunk, flush); it screws to P4 from above, outside the lidar | yes, first: screw it to the lidar to check the holes |
| `P4_lidar_seat_plate (PETG).stl` | the 3 M4 lug inserts under the lidar become 3 M3 inserts for P4b, at r 35.5 | yes |
| `P7_connector_guard (TPU).stl` | opened round the bigger base and P4b; plug outlet floor 2 mm lower | superseded by v0.10's plug cradle |

New hardware: H4 is now 3 × ISO 10642 M3 × 8 countersunk (not M4 × 8 button heads), plus 3 × ISO 7380 M3 × 6 (H21) and
3 more M3 short inserts (H19). The M4 short inserts are no longer used.

## What changed for v0.8

Compared with v0.7, reprint these **7 files**; every other print file is byte-for-byte the same as v0.7:

| File | Why |
|---|---|
| `P2_side_piece_L (PETG).stl`, `P3_side_piece_R (PETG).stl` | skull tab instead of the lug (prints without support); smooth ear-cup rim; pad pocket is one cone |
| `P8c_ear_ring_pad_L (TPU).stl`, `P8c_ear_ring_pad_R (TPU).stl` | one smooth cone instead of 16 steps (same 0.5 mm preload) |
| `P6_brow_bumper (TPU).stl` | USB-C outlet moved to the back of the right cheek; right ear removed |
| `P5_camera_cradle (PETG).stl` | right cradle screw countersunk (the USB-C plug passes over it) |
| `P4_lidar_seat_plate (PETG).stl` | heat-set insert holes (Ø5.6 / Ø4.0, through) and two lobes round the guard holes |

If you never printed v0.7, `P7_connector_guard (TPU).stl` (changed in v0.7) is also new for you.

New hardware: H19 short heat-set inserts (3 × M4 × 4, 2 × M3 × 3–4) for P4, and 1 × ISO 10642 M3 × 10 for the
right cradle screw (the same screw as H6). One H8 (M3 × 12) and one H16 washer fewer.

## 1. Printed parts (18 prints)

All files are in `stl_print/`, already rotated so z = 0 is the bed. Each file name ends with the material to print
it in: P1 says (PLA) for this fit-check, and the final P1 is reprinted in ASA.

| ID | Part | File | Qty | Fit-check material | Final | Settings | Face on the bed | Support | ≈ Filament |
|---|---|---|---|---|---|---|---|---|---|
| P1 | Crest (spine) | `P1_crest (PLA).stl` | 1 | PLA | ASA | 4 walls, 30 % gyroid, 0.2 mm; tree supports | camera face | 36 cm² | 106 g |
| P2 | Side piece L | `P2_side_piece_L (PETG).stl` | 1 | **PETG** | PETG | 4 walls, 40 % gyroid, 0.2 mm; tree supports | ear cup outer face | 22 cm² | 46 g |
| P3 | Side piece R | `P3_side_piece_R (PETG).stl` | 1 | **PETG** | PETG | 4 walls, 40 % gyroid, 0.2 mm; tree supports | ear cup outer face | 22 cm² | 47 g |
| P4 | Lidar seat plate | `P4_lidar_seat_plate (PETG).stl` | 1 | PETG + H19 inserts | CNC Al 6061, 4 mm (tapped) | 100 % infill, 0.2 mm | seat face (countersinks up) | none | 26 g |
| P4b | Lidar adapter plate | `P4b_lidar_adapter (PETG).stl` | 1 | PETG | CNC Al 6061, 4 mm (Hesai: grounded metal mount) | 100 % infill, 0.2 mm | underside (lidar countersinks on the bed) | none | 18 g |
| P5 | Camera cradle | `P5_camera_cradle (PETG).stl` | 1 | PETG | CNC Al 6061, 4 mm | 100 % infill, 0.2 mm | camera face (countersinks up) | none | 19 g |
| P6 | Brow bumper (hood) | `P6_brow_bumper (TPU).stl` | 1 | TPU 95A | TPU 95A | 3 walls, 25 % gyroid, slow | back face | 0 cm² | 18 g |
| P7 | Plug cradle (v0.11) | `P7_connector_guard (PETG).stl` (or `_mirror`) | 1 | **PETG** | PETG | 4 walls, 40 % gyroid, 0.2 mm | bottom (the face on P4) | none | 6 g |
| P7b | Cable clip on the rear clamp (v0.10) | `P7b_cable_clip (TPU).stl` | 1 | TPU 95A | TPU 95A | 3 walls, 25 % gyroid, slow | front face (grooves up) | none | 4 g |
| P8a | Crown pads | `P8a_crown_pad_1…4 (TPU).stl` | 4 (all different) | TPU 95A | TPU 95A | 100 % infill, slow | flat pocket end | none | 1 g each |
| P8b | Forehead pad | `P8b_forehead_pad (TPU).stl` | 1 | TPU 95A | TPU 95A | 100 % infill, slow | flat pocket end | none | 1 g |
| P8c | Ear ring pads | `P8c_ear_ring_pad_L (TPU).stl`, `_R (TPU).stl` | 1 + 1 | TPU 95A | TPU 95A | 100 % infill, slow | lying flat | 9 cm² each (tree) | 3 g each |
| P8d | Rear pads | `P8d_rear_pad_L (TPU).stl`, `_R (TPU).stl` | 1 + 1 | TPU 95A | TPU 95A | 100 % infill, slow | flat pocket end | none | 1 g each |
| P9 | Roll hoop | `P9_roll_hoop (PETG).stl` | 1 | PETG | Ø6 steel rod bent R42 | 100 % infill, brim | lying flat in its plane | 10 cm² | 9 g |

**Filament.** The estimates are part only, from the CAD volume and surface area at these settings; your slicer gives
the exact figures.

| Material | Parts | Estimate | With supports and a margin |
|---|---|---|---|
| PLA | P1 | ≈ 110 g | ≈ 150 g |
| PETG | P2–P5, P4b, P7, P9 | ≈ 171 g | ≈ 226 g |
| TPU 95A | P6, P7b, P8 | ≈ 25 g | ≈ 40 g |

One spool of each is plenty.

- **P2 / P3 in PETG, even for the fit-check.** These pieces flex to make the clamp force. PLA is roughly twice as
  stiff, so it would clamp about twice as hard, and it can crack under the preload.
- **P1 in PLA is fine for the fit-check.** It is the rigid spine. Reprint it in ASA for use on the robot, because it
  sits under the lidar's 7.5–10 W.
- The P5 slicer preview shows 0.9 cm² of overhang. That is only the 0.4 mm edge chamfers, so no support is needed.

## 1b. Cable box prints (v2.5, all PETG)

The box sits on the robot's back, behind the Jetson: about 145 × 261 × 49 mm (front panel and lid 145 wide, side
panels 139 apart).

| ID | Part | File | Qty | Settings | Face on the bed | ≈ Filament |
|---|---|---|---|---|---|---|
| B3 | Take-up carriage (back cheek + bridges + tenons) | `B3_takeup_carriage (PETG).stl` | 1 | 4 walls, 40 % gyroid; 1 M3 short insert (spring, from below) | back cheek | 33 g |
| B3b | Carriage front cheek (mortises) | `B3b_carriage_cheek (PETG).stl` | 1 | 4 walls, 40 % gyroid | outer face (nut pocket down) | 27 g |
| B4 | Take-up pulley (2 grooves) | `B4_takeup_pulley (PETG).stl` | 1 | 4 walls, 40 % gyroid, 0.15 mm (bearing seats) | back face | 45 g |
| B5 | Corner hook L / R (v2.4: on the holder's back corners) | `B5_handle_hook_L (PETG).stl`, `B5_handle_hook_R (PETG).stl` | 1 + 1 | 100 % infill | top face (lips up) | 25 g each |
| B7 | Spring drum | `B7_spring_drum (PETG).stl` | 1 | 100 % infill, 0.15 mm; support under the top flange | flange face | 3 g |
| B9 | Top cap | `B9_top_cap (PETG).stl` | 1 | 4 walls, 40 % gyroid; 10 M3 short inserts, 2 magnets | top face | 29 g |
| B10 | Bottom cap (with the drum fork) | `B10_bottom_cap (PETG).stl` | 1 | 4 walls, 40 % gyroid; 8 M3 short inserts, 2 magnets | bottom face | 24 g |
| B11 | U-turn former + tie rib | `B11_uturn_former (PETG).stl` | 1 | 4 walls, 30 % gyroid; 2 M3 short inserts | front face | 32 g |
| B12 | Hub cradle (measured 121.5 × 32.4 × 14 hub; end lips in the right panel) | `B12_hub_cradle (PETG).stl` | 1 | 4 walls, 30 % gyroid; 4 M3 short inserts (2 front, 2 back for B13) | front face | 21 g |
| B13 | Cable spool (behind the hub) | `B13_cable_spool (PETG).stl` | 1 | 4 walls, 30 % gyroid; no inserts | disc face | 15 g |
| B14 | Strap ear L / R (v2.5: 3 mm longer, screws on a diagonal) | `B14_strap_ear_L (PETG).stl`, `B14_strap_ear_R (PETG).stl` | 1 + 1 | 100 % infill | front face (strap slot vertical) | 13 g each |

About 280 g of PETG (the v1.0 printed box was 470 g).

## 1c. Cable box laser-cut panels (v2.0)

Cut from **3 mm cast acrylic** (clear or tinted; cast cuts cleaner than extruded). The DXFs in `laser_cut/` are the
cut lines at 1:1 in mm, holes included; `B_all_panels_nested (3mm acrylic).dxf` has all five on one 452 × 261 mm
sheet. **Measure your sheet first:** the slots are drawn for 3.0 mm. If it is not 2.9–3.1 mm, set `ACR_T` in
`cable_box_cad.py` and rerun it. That regenerates the slots, the caps and the DXFs.

| ID | Part | File | Qty | Size (mm) | Holes |
|---|---|---|---|---|---|
| B1 | Front panel (faces the robot) | `B1_front_panel (3mm acrylic).dxf` | 1 | 145 × 261 | 8 × Ø3.4, 12 tab slots, jumper window 58 × 23 |
| B2 | Lid | `B2_lid (3mm acrylic).dxf` | 1 | 145 × 261 | 4 × Ø6 (magnets), 13 tab slots, finger notch |
| B6 | Side panel L / R | `B6_side_panel_L (3mm acrylic).dxf`, `B6_side_panel_R (3mm acrylic).dxf` | 1 + 1 | 49 × 261 (with tabs) | 5 × Ø3.4 each; R (v2.4): hub notch + 4 velcro-tie slots |
| B8 | Divider | `B8_divider (3mm acrylic).dxf` | 1 | 49 × 223 (with tabs) | – |

About 370 g of acrylic. With the hub and cables the box adds roughly 0.8 kg to the torso (v1.0: about 0.6 kg).

## 2b. Cable box and cable-routing hardware

| ID | Item | Spec | Qty | Goes into | Notes |
|---|---|---|---|---|---|
| C1 | Constant-force spring | ≈ 4 N (0.4 kgf; 3–5 N is fine), strip ≤ 10 mm wide, free (natural) coil ID 11.3–12.3 mm, extension ≥ 150 mm. The coil must grip the Ø13.5 B7 drum: makers size the drum 10–20 % over the spring's free ID, so a 13–14 mm ID spring would sit loose | 1 | B7 drum → B3 lower bridge | with the carriage's 1.2 N it gives ≈ 1.3 N per cable, ≈ 2.6 N on the head (under 2 % of the neck joints' torque); a stronger spring only adds pull on the head |
| C2 | Ball bearing | **NSK 627ZZ** (7 × 22 × 7; v2.8: one) | 1 | B4 pulley | press in from the front face, down onto the shoulder; it sits on both cheeks' spigots; the pulley turns on it |
| C3 | Pulley axle | ISO 4762 M5 × 35 + ISO 4032 M5 nut | 1 + 1 | B3 (head sunk) → B4's bearings → B3b (nut sunk) | the 627ZZ sits on both cheeks' Ø6.85 spigots round the bolt; the nut pulls B3b onto B3's tenons and its inner race floats 0.15 a side between the two spacer tubes; the pulley spins on the balls |
| C4 | Drum axle | ISO 4762 M4 × 30 + M4 washer + M4 nyloc | 1 + 1 + 1 | fork on B10 → B7 → fork | nyloc only snug: the drum must spin |
| C6 | Heat-set inserts, short | brass M3 × 3–5 long, OD ≈ 4.6 (Ø4.0 holes, 5.5 deep; B3's is 4.5 deep: 3–4 mm), same as H19 | 25 | B9: 10, B10: 8, B11: 2, B12: 4, B3: 1 | not the 5.7 mm H15 type |
| C7 | Lid magnets | Ø6 × 3 mm N42 disc | 8 | 4 pressed into the caps' pockets (press fit, lead-in), 4 glued into the B2 lid's holes | cap magnets all with the same pole out; each lid magnet with the opposite pole facing it. Don't press them into the acrylic |
| C8 | Hook screws | ISO 10642 M3 × 10 | 4 | B5 L / R → B9 (inserts: one in a tongue, one in the front block, per hook) | |
| C9 | Spring screw | ISO 7380 M3 × 5 + M3 washer | 1 | spring end → B3 insert (from below) | |
| C10 | Velcro straps | 20 mm wide, ≥ 200 mm long | 2 | through each B14 ear, round the R1's U-frame bar beside it | stop the box's bottom swinging away from the robot |
| C11 | Dual Lock / velcro tape | 3M Dual Lock or hook-and-loop, 25 mm | ~15 cm | hub ↔ B1 inside face, splitter ↔ divider | |
| C12 | Spiral wrap (optional) | 10 mm, ~200 mm | 1 | the four jumpers, B1 window → Jetson panel | not on the head span: those two cables slide through the top slot |
| C13 | Zip ties + stick-on bases | 2.5–3.6 mm ties; ~6 adhesive tie bases | as needed | P7b (2), B11 rib (2, through its 3.2 mm windows), helmet runs | |
| C14 | JT128 power adapter | XT30 male → DC5521 male (or a pigtail) | 1 | Jetson XT30 → splitter's DC5521 | **measure the XT30's voltage first: JT128 takes 9–32 V** |
| C15 | Velcro cable ties | reusable, 10–12 mm wide, ~150 mm | 2 | spare branch cable hanks ↔ the slots in the right panel | |
| C16 | Panel screws | ISO 7380 M3 × 8 button head | 16 | B1 → caps (4), side panels → caps (6), B1 → B11 / B12 (4), B13 → B12 (2) | 2 mm hex; 4 mm into each insert |
| C17 | Ear screws | ISO 7380 M3 × 12 button head | 4 | B14 ear + side panel → B10 (heads sunk 4 mm in the ear) | 2 mm hex |
| C18 | Acrylic sheet | 3 mm cast acrylic, ≥ 460 × 270 mm | 1 | B1, B2, B6 L/R, B8 | measure its thickness before cutting |
| C19 | Glue for the magnets | 2-part epoxy or gel CA | – | C7 | |

## 2. Hardware to buy

| ID | Item | Spec | Qty | Buy | Goes into | Notes |
|---|---|---|---|---|---|---|
| H1 | Hinge pin | dowel pin ISO 2338 Ø4 h8 × 32, steel or A1 | 2 | 4 | P1 lugs + P2/P3 knuckles | pressed in from the front of the front lug |
| H2 | Pin retainer | grub screw ISO 4026 M3 × 4, flat point | 4 | 10 | tapped M3 in the P1 lug barrels | 1.5 mm hex |
| H3 | Clamp thumbscrew | knurled thumb screw DIN 464 M5 × 25 | 1 | 2 | P2 flange → H14 nut | tighten until the flanges touch |
| H4 | Lidar screws | countersunk ISO 10642 M3 × 8, class ≥ 4.8 (Hesai) | 3 | 5 | P4b (from below) → the JT128's own M3 holes | 4 mm into the 6 mm holes (manual: 3–5 mm past the plate); **0.5–0.6 Nm** (aluminium threads, ~10 re-tightenings); 2 mm hex |
| H6 | Plate screws | countersunk ISO 10642 M3 × 10 | 4 + 1 | 8 | P4 → H15 inserts in the P1 collar; + 1 for the right cradle screw (H8) | 2 mm hex |
| H7 | Camera screws | countersunk ISO 10642 M3 × 6 | 2 | 4 | P5 (from behind) → D435if | **max 3 mm into the camera** |
| H8 | Cradle screws | left: button head ISO 7380 M3 × 12; right: countersunk ISO 10642 M3 × 10 (counted under H6) | 1 + 1 | 2 | left through the P6 ear + P5, right flush in P5 → H15 inserts in the P1 face | left with the H16 washer |
| H9 | Guard screws | button head ISO 7380 M3 × 6 | 2 | 4 | P7 → H19 M3 inserts in P4 | |
| H11 | Hoop retainers | shaft collar, Ø6 bore, OD ≤ 12, width ≤ 5, set screw | 2 | 2 | P9 legs, under the P4 ears | only if you fit the hoop |
| H14 | Clamp nut | hex nut ISO 4032 M5 | 1 | 2 | captive in the P3 flange pocket | |
| H15 | Heat-set inserts | brass M3 × 5.7, OD 4.6 (for a Ø4.0 hole), e.g. ruthex M3×5.7 | 6 | 12 | 4 in the P1 collar, 2 in the P1 face ends | |
| H16 | Washer | ISO 7089 M3 (3.2 / 7 / 0.5) | 1 | 4 | under the left H8, on the TPU hood ear | |
| H17 | Skull screws | socket head ISO 4762 M3 × 16 | 2 | 4 | P2/P3 skull tab → the R1's threaded hole B (middle pair on the skull) | 2.5 mm hex; check the hole takes ≥ 7 mm of screw first |
| H18 | Washers | ISO 7089 M3 (3.2 / 7 / 0.5), same as H16 | 2 | 4 | under H17, in the tab's counterbore | |
| H19 | Heat-set inserts, **short** | brass M3 × 3.0 or × 4.0, OD 4.6 (Ø4.0 hole), e.g. CNC Kitchen M3 short, ruthex M3 × 4.0 | 3 + 2 | 8 | printed P4: 3 under the P4b ears, 2 under the P7 guard | P4 is 4 mm thick: the standard M3 × 5.7 is too long |
| H21 | Adapter screws | button head ISO 7380 M3 × 6 (same as H9) | 3 | 5 | P4b ears → H19 inserts in P4 | 3 mm engaged; tip 1 mm above P4's underside; 2 mm hex |
| — | Clamp shims | M5 shim washers 0.5 mm (e.g. DIN 988 5 × 10 × 0.5) | 0–2 | 4 | between the rear flanges | only to reduce the clamp force |
| H10 | Cable anchors | zip ties + stick-on zip-tie bases / P-clips | as needed | 1 pack | P1 rear, torso handle | |

Not needed for the fit-check:
- **H5:** lidar locating pins. The JT128's base has a round pin hole and a slot (at 40° and 270° from the connector),
  but the manual gives no size, and the countersunk H4 screws centre the lidar anyway. Add pins only if you see it shift.
- **H12:** sleeve for the steel hoop.

## 3. Sensors (you already have these)

| ID | Item | Mounts with |
|---|---|---|
| S1 | Hesai JT128 lidar | H4 on P4b (connector pointing back) |
| S2 | Intel RealSense D435if | H7 on P5 |

## 4. Tools and consumables

- **Soldering iron with an M3 heat-set tip** (no M4 inserts since v0.9). Use about 200–220 °C in the PLA P1,
  230–245 °C in the PETG P4, and 245–260 °C later in ASA.
- **M3 tap and a tap wrench**, for the 4 P1 lug barrels (H2 grubs). P4 no longer needs tapping (H19 inserts).
- **Ø4.0 mm drill bit, or a Ø4 H8 reamer**, for the hinge bores in P1 and in the P2/P3 knuckles. Printed holes come out
  undersize, so clear them before pressing in the dowels.
- **Hex keys** 1.5, 2 and 2.5 mm.
- **Ø3 mm pin punch and a small hammer or vise**, to press the dowels in and punch them out.
- **CA glue**, for the P8 pads in their pockets.

## 5. Fit-check notes (printed P4 / P5 / P9)

- **P4:** printed solid. Press the H19 short inserts in flush from the top (the holes go through; trim any plastic
  pushed out underneath), and tighten H21 / H9 only snug.
- **Lidar order:** P4 onto P1 first (H6, while they are still uncovered). On the bench, screw the JT128 to P4b from
  below (H4, 0.5–0.6 Nm); its hole pattern only lines up one way, with the connector between P4b's two rear ears.
  Then set P4b on P4 and screw its three ears down (H21). The ears (0 / 130 / 230°) also fit only one way, so the
  connector always ends up pointing back, between the P7 walls.
- **P9:** a printed hoop only checks the fit through the P4 ears and the lidar clearance. It is not a roll hoop until
  it is steel. You can leave the hoop and H11 off for the first fit.
- **Skull screws first, clamp last:** with the side pieces swung down, fit H17 + H18 into hole B on each side, then
  close the clamp until the flanges touch. Plug the USB-C cable in after the H8 screws: it runs over the right one,
  out through the notch at the back of P6's right cheek.
- **Final build, after the fit is verified:**
  - P1 in ASA;
  - P4 / P4b / P5 CNC-cut from 4 mm Al 6061 (for an aluminium P4, drill the insert holes Ø2.5 instead and tap M3);
    an aluminium P4b also gives the lidar the grounded metal mount Hesai recommends;
  - P9 from about 300 mm of Ø6 steel or 304 rod, bent R42, with an H12 sleeve;
  - add H5.
