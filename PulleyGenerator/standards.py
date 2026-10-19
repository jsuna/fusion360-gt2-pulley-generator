"""Groove and tooth tables for the pulley generator. All dimensions in mm.

Sources
-------
Timing, straight-sided pulley grooves (MXL, XL, L, H):
    ISO 5294 (Synchronous belt drives - Pulleys), Table 2.
Timing, belt tooth heights and pitch line differentials (HTD, GT, T-series):
    SDP/SI Technical Section, Figure 19 and Table 4.
GT2 2 mm groove: nominal 2GT belt profile (R0.555 tip, R1.00 flanks
    0.40 off centre, R0.15 fillets, 0.75 deep, PLD 0.254).
V-belt grooves, narrow (SPZ, SPA, SPB): DIN 2211 (via SIT pulley catalogue).
V-belt grooves, classical (Z, A, B): ISO 4183, Table 2. The 34/38 degree
    changeover diameters are taken to match the narrow section with the same
    datum width (DIN 2211 / DIN 2217).
Multi-V (V-ribbed) grooves: ISO 9982, Table 1 (pitch, angle, tip and
    bottom radii, edge distance), effective line differential and minimum
    effective diameters (Table 2).
Wire rope sheaves: groove radius 0.535 d, depth 1.5 d, throat 35-45 degrees
    (Unirope sheave guidance).

Profiles marked approximate are built from published belt dimensions rather
than an official pulley groove definition (the GT and HTD shapes are
proprietary, and the T-series pulley gap is derived from the belt tooth).
"""

# ---------------------------------------------------------------- timing
#
# Every timing profile has 'family', 'pitch' and 'pld' (pitch line
# differential: OD = PD - 2 * pld). Family-specific keys:
#
#   arc    (GT style)   h, r_tip, r_flank, b, r_fillet
#   trap   (trapezoid)  bw (groove bottom width), hg (groove depth),
#                       phi (flank angle from radial, deg), rb, rt
#   round  (HTD style)  h (depth), r (groove radius), beta (included flank
#                       angle, deg), rf (edge fillet)


def _gt_scaled(pitch, h, pld):
    """GT 3 mm / 5 mm: the 2 mm GT2 construction scaled to the published
    tooth height. Approximate."""
    k = h / 0.75
    return dict(family='arc', pitch=pitch, pld=pld, h=h, r_tip=0.555 * k,
                r_flank=1.00 * k, b=0.40 * k, r_fillet=0.15 * k)


def _htd(pitch, h, pld):
    """HTD: near-semicircular groove scaled from the 5M figures
    (R1.49 tooth, R0.43 root fillet). Approximate."""
    k = pitch / 5.0
    return dict(family='round', pitch=pitch, pld=pld, h=h, r=1.49 * k,
                beta=0.0, rf=0.43 * k)


def _t_series(pitch, ht, s, rad, pld, depth_extra, top_extra):
    """T-series: straight 40 degree groove from the belt tooth (base width s,
    height ht) plus clearance. Approximate."""
    import math
    hg = ht + depth_extra
    top = s + top_extra
    bw = top - 2 * hg * math.tan(math.radians(20))
    return dict(family='trap', pitch=pitch, pld=pld, bw=bw, hg=hg, phi=20.0,
                rb=rad, rt=rad)


TIMING = [
    ('GT2 2 mm', dict(family='arc', pitch=2.0, pld=0.254, h=0.75, r_tip=0.555,
                      r_flank=1.00, b=0.40, r_fillet=0.15)),
    ('GT 3 mm (approx.)', _gt_scaled(3.0, 1.14, 0.381)),
    ('GT 5 mm (approx.)', _gt_scaled(5.0, 1.93, 0.5715)),
    ('HTD 3M (approx.)', _htd(3.0, 1.22, 0.381)),
    ('HTD 5M (approx.)', _htd(5.0, 2.06, 0.5715)),
    ('HTD 8M (approx.)', _htd(8.0, 3.38, 0.686)),
    ('MXL', dict(family='trap', pitch=2.032, pld=0.254, bw=0.84, hg=0.69,
                 phi=20.0, rb=0.25, rt=0.13)),
    ('XL', dict(family='trap', pitch=5.08, pld=0.254, bw=1.32, hg=1.65,
                phi=25.0, rb=0.41, rt=0.64)),
    ('L', dict(family='trap', pitch=9.525, pld=0.381, bw=3.05, hg=2.67,
               phi=20.0, rb=1.19, rt=1.17)),
    ('T2.5 (approx.)', _t_series(2.5, 0.7, 1.5, 0.2, 0.3, 0.05, 0.1)),
    ('T5 (approx.)', _t_series(5.0, 1.2, 2.65, 0.4, 0.5, 0.05, 0.2)),
    ('T10 (approx.)', _t_series(10.0, 2.5, 5.3, 0.6, 1.0, 0.10, 0.3)),
]

# ---------------------------------------------------------------- V-belt
#
#   wd  datum width       b  groove above datum line (OD = dd + 2b)
#   h   groove below datum line                     e  groove spacing
#   f   edge distance (face to first groove centre)
#   dd_34  datum diameter up to which the groove is 34 deg (38 deg above)
#   dd_min minimum recommended datum diameter

V_BELT = [
    ('Z', dict(wd=8.5, b=2.0, h=7.0, e=12.0, f=7.0, dd_34=80, dd_min=50)),
    ('A / 4L', dict(wd=11.0, b=2.75, h=8.7, e=15.0, f=9.0, dd_34=118, dd_min=75)),
    ('B / 5L', dict(wd=14.0, b=3.5, h=10.8, e=19.0, f=11.5, dd_34=190, dd_min=125)),
    ('SPZ', dict(wd=8.5, b=2.0, h=9.0, e=12.0, f=8.0, dd_34=80, dd_min=63)),
    ('SPA', dict(wd=11.0, b=2.8, h=11.0, e=15.0, f=10.0, dd_34=118, dd_min=90)),
    ('SPB', dict(wd=14.0, b=3.5, h=14.0, e=19.0, f=12.5, dd_34=190, dd_min=140)),
]

# ---------------------------------------------------------------- multi-V
#
#   e   rib pitch      angle  groove angle (deg)
#   rt  rib tip radius (min)  rb  groove bottom radius (max)
#   be  effective line differential (effective dia = OD + 2 be)
#   f   edge distance (face to first groove centre, min)
#   de_min minimum effective diameter

MULTI_V = [
    ('PH', dict(e=1.6, angle=40.0, rt=0.15, rb=0.3, be=0.8, f=1.3, de_min=13)),
    ('PJ', dict(e=2.34, angle=40.0, rt=0.2, rb=0.4, be=1.2, f=1.8, de_min=20)),
    ('PK', dict(e=3.56, angle=40.0, rt=0.25, rb=0.5, be=2.0, f=2.5, de_min=45)),
    ('PL', dict(e=4.7, angle=40.0, rt=0.4, rb=0.4, be=3.0, f=3.3, de_min=75)),
    ('PM', dict(e=9.4, angle=40.0, rt=0.75, rb=0.75, be=4.0, f=6.4, de_min=180)),
]

# ---------------------------------------------------------------- rope / U
#
# Factors of the rope (or round belt) diameter d:
#   r   groove radius     depth  groove depth from OD
#   beta  included throat (flare) angle, deg

ROPE = [
    ('Round belt', dict(r=0.52, depth=0.75, beta=30.0)),
    ('Wire rope sheave', dict(r=0.535, depth=1.5, beta=40.0)),
]


def lookup(table, name):
    for n, data in table:
        if n == name:
            return dict(data)
    raise KeyError(name)
