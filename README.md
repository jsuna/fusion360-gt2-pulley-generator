# Pulley Generator for Fusion 360

An Autodesk Fusion 360 add-in that generates timing, V-belt, multi-V (ribbed) and round belt / rope pulleys. Pick the pulley type and the dialog shows that type's settings, with standard presets and a Custom option. It adds a **Pulley Generator** button to **Solid → Create**, next to Spur Gear.

![GT2 pulley generated with flanges, hub and M8 set screws](docs/screenshot.png)

## Install

1. Download or clone this repo.
2. Copy the `PulleyGenerator` folder into Fusion's AddIns folder:
   - **Windows:** `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\`
   - **macOS:** `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/`
3. In **Utilities → Add-Ins → Scripts and Add-Ins → Add-Ins**, select **PulleyGenerator**, click **Run** and tick **Run on Startup**.

The button then appears under **Solid → Create → Pulley Generator**. If you installed the earlier GT2-only version, stop it and remove its `GT2Pulley` folder; the timing type covers everything it did.

## Pulley types

| Type | Presets | You set |
| --- | --- | --- |
| Timing (toothed) | GT2 2 mm, GT 3 mm, GT 5 mm, HTD 3M / 5M / 8M, MXL, XL, L, T2.5, T5, T10 | Teeth, pulley width, groove clearance, flanges |
| V-belt | Z, A / 4L, B / 5L, SPZ, SPA, SPB | Datum diameter, number of grooves |
| Multi-V (ribbed) | PH, PJ, PK, PL, PM | Effective diameter, number of ribs |
| Round belt / rope (U-groove) | Round belt, wire rope sheave | Belt or rope diameter, pitch diameter (rope centre), edge fillet, rim width |

Every type also has **Custom**: it starts from the last preset's values and lets you edit the groove dimensions directly. For timing pulleys you also choose the groove shape (GT-style arcs, trapezoid or round).

The **Result** box shows the pitch / datum / effective diameter, OD, root diameter and width as you change settings, plus a warning below the standard minimum diameter. A live preview shows the pulley before you click OK.

## Shared options

| Option | What it does |
| --- | --- |
| Bore diameter | Shaft bore; 0 for none. |
| Hub (boss) | Hub on top of the pulley (or the top flange), with diameter and height. |
| Set screws | None, in the hub, or through the teeth / rim. Through-teeth holes on timing pulleys are centred on a groove. |
| Thread standard / size | Metric M2–M8, or imperial #4-40 to 1/4-20. |
| Hole | Plain hole at tap drill size, cosmetic thread, or modeled thread. |
| Number of screws / spacing | 1–4 screws; 90° default, 180° for opposed screws. |
| Flanges (timing only) | Top and/or bottom, with OD offset from the pulley OD and thickness. |

## Geometry and sources

**Timing** pulleys are drawn in the face plane and extruded. The pitch diameter is N × pitch / π, and the OD is the pitch diameter minus twice the pitch line differential. Groove clearance enlarges the groove without changing the OD, so the pitch stays exact.

- **GT2 2 mm:** nominal 2GT profile (R0.555 tip, R1.00 flanks centred on the OD circle, R0.15 fillets, 0.75 mm deep, PLD 0.254). Flank centres are solved exactly at every tooth count.
- **MXL, XL, L:** straight-sided pulley grooves from ISO 5294.
- **GT 3 / 5 mm, HTD, T-series (approximate):** these pulley groove shapes are proprietary or not published freely. They are built from published belt tooth heights and pitch line differentials (SDP/SI technical section): the GT construction scaled to the 3 / 5 mm tooth height, a near-semicircular groove for HTD scaled from the 5M figures, and a 40° groove with clearance around the T-series belt tooth. Fine for 3D printing and prototypes; check fit before machining.

**V-belt, multi-V and rope** pulleys are half cross-sections revolved around the axis.

- **V-belt:** narrow sections from DIN 2211, classical sections from ISO 4183. The groove is 34° up to the standard changeover diameter and 38° above it. OD = datum diameter + 2b.
- **Multi-V:** ISO 9982 rib pitch, 40° groove angle, tip and bottom radii and edge distance. Effective diameter = OD + 2 × effective line differential.
- **Rope / round belt:** groove radius 0.535 d, depth 1.5 d and a 40° throat for wire rope (Unirope sheave guidance); 0.52 d radius, 0.75 d depth and a 30° throat for round belts.

The full tables are in [`PulleyGenerator/standards.py`](PulleyGenerator/standards.py) and the geometry in [`PulleyGenerator/profiles.py`](PulleyGenerator/profiles.py).
