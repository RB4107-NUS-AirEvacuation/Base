RB4107 SAFE EVACUATION ROBOT - FABRICATION PACK REV C2   (2026-11-03)
=====================================================================
Rev C2 = Rev C1 (below) + the changes in this box. Where the text below disagrees, THIS BOX wins.

WHAT CHANGED (3 Nov 2026)
  P12 deck                Al 6061-T6 5 mm -> CAST ACRYLIC 5 mm, same geometry (countersinks cut with a countersink bit).
  P28 side walls (NEW)    2x cast acrylic 3 mm, 350 x 65, 76 x 32 notch (P13 lifts up to 3 mm for stage-2 belt tension). Glued onto the deck top along both long edges
                          (outer face 3 mm in from the edge) and to the corner posts; top flush with the tray.
                          NOT glued to the tray. Glue them last, after the belts are tensioned. Consumable C4 solvent cement.
  P24 rail angles         20 x 20 x 1 stock as bought - full legs, only cut to 183. Plate-leg holes use the UPPER M3 hole
                          row of the side plates (11.8 from the corner edge); rail-leg holes Ø3.4. Split into P24 LEFT x2
                          (rear A + front B) and P24 RIGHT x2 (front A + rear B), each with its own sheet, holes-only DXF
                          and STEP (datum = corner edge = outer face of the other leg, dotted line; + hinge end).
  P12 tail-cradle holes   the 4 deck holes of the motor tail cradles P10 are now slots 3.4 x 6 (were Ø3.4), so the cradle
                          slides with the motor (motor-bracket slots 7 x 3.4 were already there) to tension the stage-1
                          belt: 1.8 mm towards tension, 0.8 back. Stage 2 = P27 bracket slots (vertical).

WHERE THINGS ARE NOW
  RB4107 Fabrication request - Base/   metal parts (P11, profiles, shafts, P24) + subfolder "RB4107 Acrylic parts - laser
                                       cut" (P1, P2, P9, P12, P13, P16, P28). Changes.pdf = the new / changed part drawings only (P12, P28, P24 L + R).
                                       ONE order sheet for acrylic + metal: "Fabrication order - Base, acrylic + metal
                                       (Rev C2, 3 Nov 2026)" (.xlsx + .pdf) in this folder (A-D acrylic, E-G metal).
                                       Zip: "RB4107 Fabrication request - Base - Newest.zip".
  1 Drawings/   current: Part detail drawings - Base (Rev C2), Acrylic parts - drawings (Rev C2), Detailed assembly
                drawing (Rev C2), Changes.pdf, 3D print request (Rev C1, unchanged).
                The 19 Oct "Assembly drawings (Rev C)" and "Fabrication drawings (Rev C)" sets are fully superseded and
                were moved to 1 Drawings/_superseded/.
  2 Laser cut DXF/, 3 STEP 3D models/   P12 = acrylic version, P28 added, P24 holes-only DXF added.
  5 Parts list/  BOM + full checklist updated for C2 (P12 acrylic, P28 x2, C4). The Track + Base checklists are older.

FUSION MODEL (updated 3 Nov)
  "Redesign Rev C - 29Oct (printed P17)": deck P12 material Acrylic, side walls P28 (base feature "Rev C3 side walls on deck").
  "Redesign + Track 19Oct": references it; P24 angles with full 20 mm legs and holes / screws in the upper row.
  Deck tail-cradle slots: feature "Rev C2 tail-cradle P10 slots (motor tension)" in the Base tray component.


RB4107 SAFE EVACUATION ROBOT - FABRICATION PACK REV C1   (2026-10-29)
=====================================================================
Rev C1 = Rev C below + the changes in this box. Where the Rev C text below disagrees, THIS BOX wins.

WHAT CHANGED (29 Oct 2026)
  P1 / P2 side plates     Al 6061-T6 5 mm -> CAST ACRYLIC 5 mm, laser cut. Same holes; three 3 mm bosses on the top
                          edge (R8 over the two M5 corner-bracket holes, R7 over the M3 tensioner-tab hole).
  P13 shaft supports      Al 4 mm -> cast acrylic 4 mm.        P9 motor shims   Al 2 mm -> cast acrylic 2 mm.
  Bearing seats           P1 / P2 / P13: no ream, no press fit - finished Ø16.0-16.1 slide fit + epoxy (ease by hand if tight).
  P12 deck                7 weight-saving cut-outs removed (machining cost, +129 g); cut-outs renumbered S1-S10.
  P25 tensioner tab       to side plate with M3 x 8 (was M3 x 6): F1 8 -> 12, F2 24 -> 20.
  Acrylic rules           tap by hand, snug screws, washer under every head, NO thread-locker / solvent / CA glue near it.
  Robot mass target       3 kg.

WHERE THINGS ARE NOW
  RB4107 Fabrication request - Base/           what goes to the workshop: metal parts (deck, sector gear, profiles,
                                               angles, shafts) + subfolder "RB4107 Acrylic parts - laser cut" (P1, P2,
                                               P9, P13, P16) - each with its own order sheet, drawings,
                                               DXF and STEP. Zip: "RB4107 Fabrication request - Base (Rev C1, 29 Oct 2026).zip"
  1 Drawings/   current: Part detail drawings - Base (Rev C1), Acrylic parts - drawings (Rev C1), Detailed assembly
                drawing (Rev C1, change sheet first), 3D print request (Rev C1). The 19 Oct Assembly + Fabrication
                drawing sets have a C1 notice page in front and a red banner on every sheet.
  2 Laser cut DXF/, 3 STEP 3D models/   P1, P2, P9, P13 replaced by the acrylic versions; P12 without cut-outs.
  5 Parts list/  BOM + checklists updated for C1 (pictures kept). The two Track + Base checklists are from 20/21 Oct -
                 use the BOM for current quantities.
  Old versions: 1 Drawings/_superseded/.

FUSION MODEL (updated 29 Oct)
  "Redesign Rev C - 29Oct (printed P17)" v8: P1 / P2 bosses (sketch + extrude "C1 bosses - ..."), deck cut-outs removed
  (Delete Face "C1 deck - weight-saving cut-outs removed"), P1 / P2 / P9 / P13 material = Acrylic, deck = Aluminum 6061.
  "Redesign + Track 19Oct" v32: reference updated to v8.
  The C1 DXF + STEP files of P1, P2, P9, P13 and P12 are EXPORTED FROM THIS MODEL (body section + body STEP), with the DXF
  layers added by hole size. Raw export: "9 Working files/Fusion export Rev C1 (29 Oct)".
  Full assembly: "3 STEP 3D models/RB4107 full assembly Rev C1 (robot + track + levelling + chair).step".


RB4107 SAFE EVACUATION ROBOT - FABRICATION PACK REV C   (2026-10-20)
=====================================================================
Made from the Fusion models "Redesign" (v39) and "Redesign + Track 19Oct"
(robot + new track parts + levelling system + chair, all assembled).

WHAT IS IN THIS FOLDER
  1 Drawings/
      RB4107 Assembly drawings (Rev C).pdf   8 sheets, A3 landscape, exploded views
        with numbered balloons, a parts-in-view table (with quantities) and assembly /
        joint notes on every sheet - same style as Rev B2:
        A1 general arrangement (overall views + size)   A2 back segment
        A3 front segment   A4 hinge axle stack   A5 hinge drive + sector gear
        A6 drive train   A7 base tray + electronics   A8 track tensioner + rail carriers (new)
      RB4107 Fabrication drawings (Rev C).pdf   28 sheets, A4 landscape:
        cover / index / changes, overall views + space envelope,
        SPACING + TOLERANCES (gaps between parts, room for fabrication error, fits,
        built-in adjustment), joint / fastener schedule + assembly order,
        part drawings with hole tables, cut list, printed parts, bill of materials.
      The levelling system and the chair are done by the levelling team: they appear in
      the overall views and the 3D files for position only, and are NOT in the BOM or
      the assembly drawings. Our side of the interface = 8x M2 tapped holes in the tray.
  2 Laser cut DXF/      one file per flat part, in mm, all lines on layer CUT.
                        Bearing seats are drawn at 15.8 on purpose: ream to 16 H7 after cutting.
                        Deck corner holes are 6.4 (countersink 13 on the underside after cutting).
                        P11 sector gear: 14 teeth (decided 2026-10-22), cut from a bought m1 100T gear (BOM B24).
  3 STEP 3D models/     full assembly + one STEP per made / changed part.
  4 STL 3D print/       printed parts (P10, P22, P23 tail cradle, cover, ESP32 sled; L1-L5 levelling kept for reference).
  5 Parts list/         RB4107 BOM (Rev C).xlsx - green = new, yellow = changed, red = removed.
  9 Working files/      pictures and data used to make the drawings (not needed for building).

PART NUMBERS
  P1-P26  parts you make (P18, P19 removed; P24 rail carrier angle, P25 tensioner tab,
          P26 tensioner slider are new; P17 is now two flat plates P17a + P17b)
  M1-M8   bought parts that need a small change (bore, cross hole, dimple)
  L1-L5   levelling system parts - 3D files only, for reference (levelling team)
  B / F / E / C   bought parts, fasteners, electronics, consumables (see the BOM)
  "A" / "B" in a name = left / right hand of a mirrored pair.

COORDINATES (all CAD files)
  Origin = hinge axis.  +X = towards the nose (front segment),  +Y = up,
  Z = across the robot (A side at Z < 0).  Everything is already in its final position.

MAIN CHANGES FROM REV B
  - New track parts merged in: track tensioners at all 4 idlers, one aluminium angle
    per rail instead of 24 small clips, rails trimmed to 186.
  - Side plates re-drilled for the new angles + tensioner, longer idler slot.
  - Hinge shafts plain round with spring pins (no flats, no clips); stage-2 pulley is
    the same type as stage 1.
  - Hinge clearance fixes (sector bracket corner rounded, deck slot wider, hinge motor
    moved 1.5 mm) - at least 2 mm clearance through the whole 35 degree fold.
  - Tray plate drilled for the levelling system (8 x M2 tapped holes).
  - Weight-saving cut-outs in the deck (-129 g) and tray (-180 g).
  - Bought cast corner brackets instead of cut-angle brackets: MISUMI LBSBK8-3030-SET x4 (rear,
    3030 bars) and LBSBK6-2020 x4 (front, 2020 bars) - decided 2026-10-23. One tapped M5 hole per
    bracket in P1/P2 at Y 10 (P2: 17.5 from the bar face, P1: 10.7 from the bar face).
  - Hinge shaft bearing supports: the one-piece L part is now two flat laser-cut plates
    (bearing plate P17a 5 mm + foot plate P17b 6 mm) joined by 2 x M3x12 screws into square
    nuts held in slots in the foot plate. Same parts on both sides; the side-B foot points
    inboard to keep clear of the track belt. Mounting screws are now M5x12.
  - Drive train drawings now follow the model (8 mm intermediate shafts, F688ZZ,
    74 x 22 shaft supports, sector gear cut from a bought 100-tooth gear).

STILL OPEN (decide / check before building)
  1. (Levelling team) seat underside needs 8 holes for their seat brackets.
  2. DONE 2026-10-22: sector gear = 14 teeth; pinion 22T, centre distance 61.0 mm checked in the model.
  3. Stage-2 belt runs only 1.0 mm under the deck plate - check or add a relief.
  4. Battery has only 0.6-1.1 mm around it - allow about 2 mm.
  5. (Levelling team) tilt range of the 30 mm actuator vs the steepest stairs.
  6. If the rails were already drilled for the old clips, move the new pilot holes 3 mm.

Superseded: the separate folder "RB4107 Rev C DXF - side plates" (one level up) was a
working copy - use the DXFs in this pack instead.
