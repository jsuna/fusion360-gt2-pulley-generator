"""Pure 2D geometry for the pulley generator (no Fusion imports). Units: mm.

Outlines are lists of segments, each either
    ('line', a, b)        straight from a to b
    ('arc',  a, m, b)     three-point arc from a through m to b
and each segment starts where the previous one ends.

Timing pulleys are built in the face plane (x, y), centre at the origin, and
extruded. V-belt, multi-V and rope pulleys are half cross-sections in the
(z, r) plane - z along the axis, r the radius - and revolved about z.
"""

import math


# ---------------------------------------------------------------- vectors

def _add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _mul(a, k):
    return (a[0] * k, a[1] * k)


def _len(a):
    return math.hypot(a[0], a[1])


def _unit(a):
    d = _len(a)
    return (a[0] / d, a[1] / d)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _mirror(p):
    return (-p[0], p[1])


def _mid_of(c, t1, t2, r):
    """Midpoint of the minor arc t1 -> t2 about centre c."""
    return _add(c, _mul(_unit(_add(_sub(t1, c), _sub(t2, c))), r))


def _circle_intersections(c0, r0, c1, r1):
    (x0, y0), (x1, y1) = c0, c1
    dx, dy = x1 - x0, y1 - y0
    d = math.hypot(dx, dy)
    if d == 0 or d > r0 + r1 or d < abs(r0 - r1):
        return []
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    xm, ym = x0 + a * dx / d, y0 + a * dy / d
    return [(xm + h * dy / d, ym - h * dx / d),
            (xm - h * dy / d, ym + h * dx / d)]


def _line_circle(p, d, radius):
    """Intersections of the line p + t d (d unit) with the circle about the
    origin. Returns the t values."""
    b = _dot(p, d)
    c = _dot(p, p) - radius * radius
    disc = b * b - c
    if disc < 0:
        return []
    s = math.sqrt(disc)
    return [-b - s, -b + s]


def _reverse(segs):
    out = []
    for s in reversed(segs):
        if s[0] == 'line':
            out.append(('line', s[2], s[1]))
        else:
            out.append(('arc', s[3], s[2], s[1]))
    return out


def _mirror_segs(segs):
    out = []
    for s in segs:
        out.append((s[0],) + tuple(_mirror(p) for p in s[1:]))
    return out


# ---------------------------------------------------------------- fillets

def _fillet_corner(p_prev, p, p_next, r):
    """Round the corner p between straight edges. Returns
    (t1, mid or None, t2, setback)."""
    if r <= 1e-9:
        return p, None, p, 0.0
    u = _unit(_sub(p, p_prev))
    v = _unit(_sub(p_next, p))
    cos_t = max(-1.0, min(1.0, _dot(_mul(u, -1), v)))
    theta = math.acos(cos_t)                  # interior angle at p
    if theta < 1e-6 or abs(theta - math.pi) < 1e-6:
        return p, None, p, 0.0
    t = r / math.tan(theta / 2)
    t1 = _sub(p, _mul(u, t))
    t2 = _add(p, _mul(v, t))
    bis = _unit(_add(_mul(u, -1), v))
    c = _add(p, _mul(bis, r / math.sin(theta / 2)))
    mid = _sub(c, _mul(bis, r))
    return t1, mid, t2, t


def filleted_polygon(verts):
    """Closed polygon [(point, fillet radius), ...] -> segments."""
    n = len(verts)
    corners = []
    for i in range(n):
        corners.append(_fillet_corner(verts[i - 1][0], verts[i][0],
                                      verts[(i + 1) % n][0], verts[i][1]))
    for i in range(n):                        # fillets must fit their edges
        a, b = verts[i][0], verts[(i + 1) % n][0]
        if corners[i][3] + corners[(i + 1) % n][3] > _len(_sub(b, a)) + 1e-7:
            raise ValueError('A fillet radius is too large for the groove.')
    segs = []
    for i in range(n):
        t1, mid, t2, _ = corners[i]
        if mid is not None:
            segs.append(('arc', t1, mid, t2))
        nxt = corners[(i + 1) % n][0]
        if _len(_sub(nxt, t2)) > 1e-9:
            segs.append(('line', t2, nxt))
    return segs


# ---------------------------------------------------------------- timing
#
# Each family gives ONE groove in a local frame: +y is the groove centreline
# (radial, origin at the pulley centre), +x points toward the next groove.
# Segments run from the left OD point to the right OD point.

def timing_diameters(p, n):
    pd = n * p['pitch'] / math.pi
    od = pd - 2 * p['pld']
    return pd, od


def timing_depth(p):
    return p['hg'] if p['family'] == 'trap' else p['h']


def _groove_arc(p, ro, c):
    """GT style: tip arc + flank arcs centred on the OD circle + fillets."""
    h = p['h']
    rt = p['r_tip'] + c
    rfl = p['r_flank'] + c
    rfi = p['r_fillet']
    d = p['r_flank'] - p['r_tip']             # centres for internal tangency

    ct = (0.0, ro - (h - p['r_tip']))
    sols = _circle_intersections((0.0, 0.0), ro, ct, d)
    if not sols:
        raise ValueError('Could not solve the flank arcs; check the profile.')
    cf_r = min(sols, key=lambda q: q[0])      # left centre swings right flank
    t_r = _add(cf_r, _mul(_unit(_sub(ct, cf_r)), rfl))

    if rfi > 1e-9:
        sols = _circle_intersections((0.0, 0.0), ro - rfi, cf_r, rfl + rfi)
        if not sols:
            raise ValueError('Could not solve the groove fillets.')
        cfi_r = max(sols, key=lambda q: q[0])
        o_r = _mul(_unit(cfi_r), ro)
        f_r = _add(cf_r, _mul(_unit(_sub(cfi_r, cf_r)), rfl))
        right = [('arc', t_r, _mid_of(cf_r, t_r, f_r, rfl), f_r),
                 ('arc', f_r, _mid_of(cfi_r, f_r, o_r, rfi), o_r)]
    else:
        sols = _circle_intersections((0.0, 0.0), ro, cf_r, rfl)
        o_r = max(sols, key=lambda q: q[0])
        right = [('arc', t_r, _mid_of(cf_r, t_r, o_r, rfl), o_r)]

    t_l = _mirror(t_r)
    tip = ('arc', t_l, (0.0, ct[1] - rt), t_r)
    return _reverse(_mirror_segs(right)) + [tip] + right


def _groove_trap(p, ro, c):
    """Straight-sided groove: flat bottom, flanks at phi, rb / rt fillets."""
    phi = math.radians(p['phi'])
    tphi = math.tan(phi)
    yb = ro - (p['hg'] + c)
    hwb = p['bw'] / 2 + c / math.cos(phi) - c * tphi
    if hwb <= 0:
        raise ValueError('Groove bottom width must be greater than 0.')
    w = (math.sin(phi), math.cos(phi))        # right flank, outward
    nrm = (math.cos(phi), -math.sin(phi))     # toward material
    p_br = (hwb, yb)

    ts = _line_circle(p_br, w, ro)
    if not ts:
        raise ValueError('Groove flank does not reach the OD.')
    p_tr = _add(p_br, _mul(w, max(ts)))

    # bottom fillet between the bottom line and the flank
    rb = p['rb'] + c
    tb1, mb, tb2, sb = _fillet_corner((-hwb, yb), p_br, p_tr, rb)
    if tb1[0] < -1e-9:
        raise ValueError('Bottom radius is too large for the groove width.')

    # top fillet between the flank and the OD circle
    rt = p['rt']
    if rt > 1e-9:
        q = _add(p_br, _mul(nrm, rt))
        ts = _line_circle(q, w, ro - rt)
        if not ts:
            raise ValueError('Top radius does not fit.')
        fc = _add(q, _mul(w, max(ts)))
        tw = _sub(fc, _mul(nrm, rt))
        to = _mul(_unit(fc), ro)
        top = [('arc', tw, _mid_of(fc, tw, to, rt), to)]
        if _dot(_sub(tw, p_br), w) < sb - 1e-9:
            raise ValueError('Top and bottom radii overlap on the flank.')
    else:
        tw = p_tr
        top = []

    right = []
    if mb is not None:
        right.append(('arc', tb1, mb, tb2))
    right.append(('line', tb2, tw))
    right += top

    left = _reverse(_mirror_segs(right))
    mid = []
    if tb1[0] > 1e-9:
        mid = [('line', _mirror(tb1), tb1)]
    return left + mid + right


def _groove_round(p, ro, c):
    """Round (HTD style / U) groove: arc bottom, flanks tangent at an
    included angle beta, edge fillet rf onto the OD circle."""
    R = p['r'] + c
    cy = ro - p['h'] + p['r']                 # arc centre (fixed by nominal)
    half = math.radians(p['beta']) / 2
    rf = p['rf']
    ct = (0.0, cy)
    bottom = (0.0, cy - R)

    tan_pt = (R * math.cos(half), cy - R * math.sin(half))
    w = (math.sin(half), math.cos(half))
    nrm = (math.cos(half), -math.sin(half))

    right = []
    t_arc = None
    if tan_pt[1] < ro:                        # a straight flank exists
        if rf > 1e-9:
            q = _add(tan_pt, _mul(nrm, rf))
            ts = _line_circle(q, w, ro - rf)
            if ts and max(ts) >= 0:
                fc = _add(q, _mul(w, max(ts)))
                tw = _sub(fc, _mul(nrm, rf))
                to = _mul(_unit(fc), ro)
                t_arc = tan_pt
                right = [('line', tan_pt, tw),
                         ('arc', tw, _mid_of(fc, tw, to, rf), to)]
        else:
            ts = _line_circle(tan_pt, w, ro)
            t_arc = tan_pt
            right = [('line', tan_pt, _add(tan_pt, _mul(w, max(ts))))]
    if t_arc is None:                         # arc runs straight into the OD
        if rf > 1e-9:
            sols = _circle_intersections((0.0, 0.0), ro - rf, ct, R + rf)
            if not sols:
                raise ValueError('Edge fillet does not fit the groove.')
            fc = max(sols, key=lambda q: q[0])
            t_arc = _add(ct, _mul(_unit(_sub(fc, ct)), R))
            to = _mul(_unit(fc), ro)
            right = [('arc', t_arc, _mid_of(fc, t_arc, to, rf), to)]
        else:
            sols = _circle_intersections((0.0, 0.0), ro, ct, R)
            if not sols:
                raise ValueError('Groove does not reach the OD.')
            t_arc = max(sols, key=lambda q: q[0])
            right = []

    arc = ('arc', _mirror(t_arc), bottom, t_arc)
    return _reverse(_mirror_segs(right)) + [arc] + right


_GROOVES = {'arc': _groove_arc, 'trap': _groove_trap, 'round': _groove_round}


def _rotate(pt, th):
    """Local (x tangential, y radial) -> global, groove centreline at th."""
    x, y = pt
    c, s = math.cos(th), math.sin(th)
    return (y * c - x * s, y * s + x * c)


def timing_outline(p, n, clearance=0.0):
    """Closed toothed outline for n teeth. Returns (segments, info)."""
    pd, od = timing_diameters(p, n)
    ro = od / 2.0
    if ro <= timing_depth(p) + 0.5:
        raise ValueError('Too few teeth for this profile.')
    local = _GROOVES[p['family']](p, ro, clearance)

    left, right = local[0][1], local[-1][-1]
    half_r = math.atan2(right[0], right[1])
    half_l = math.atan2(-left[0], left[1])
    step = 2 * math.pi / n
    if half_l + half_r >= step - 0.02 / ro:
        raise ValueError('Grooves overlap at %d teeth; reduce clearance or '
                         'radii.' % n)

    out = []
    for k in range(n):
        th = math.pi / 2 + k * step
        for s in local:
            out.append((s[0],) + tuple(_rotate(q, th) for q in s[1:]))
        a = _rotate(right, th)
        b = _rotate(left, th + step)
        out.append(('arc', a, _mid_of((0.0, 0.0), a, b, ro), b))
    root = od - 2 * (timing_depth(p) + clearance)
    return out, dict(pd=pd, od=od, root=root)


# ---------------------------------------------------------------- revolved

def _rim(width, r_in, r_o, groove_verts):
    """Close a (z, r) cross-section: bore line, right face, grooves listed
    right to left along the OD, left face."""
    return ([((0.0, r_in), 0.0), ((width, r_in), 0.0), ((width, r_o), 0.0)]
            + groove_verts + [((0.0, r_o), 0.0)])


def v_angle(p, dd):
    if p.get('angle'):
        return p['angle']
    return 34.0 if dd <= p['dd_34'] else 38.0


def v_profile(p, dd, grooves, r_in):
    """Classical / narrow V-belt pulley. dd = datum diameter."""
    angle = v_angle(p, dd)
    t = math.tan(math.radians(angle) / 2)
    r_d = dd / 2.0
    r_o = r_d + p['b']
    r_bot = r_d - p['h']
    wt = p['wd'] / 2 + p['b'] * t
    wb = p['wd'] / 2 - p['h'] * t
    if wb <= 0:
        raise ValueError('Groove is too deep for its width and angle.')
    if p['e'] < 2 * wt - 1e-9:
        raise ValueError('Groove spacing is smaller than the groove width.')
    if p['f'] < wt - 1e-9:
        raise ValueError('Edge distance is smaller than half the groove width.')
    if r_bot <= r_in + 0.5:
        raise ValueError('Datum diameter is too small for this bore.')
    width = (grooves - 1) * p['e'] + 2 * p['f']
    verts = []
    for i in range(grooves - 1, -1, -1):
        z = p['f'] + i * p['e']
        verts += [((z + wt, r_o), 0.0), ((z + wb, r_bot), 0.0),
                  ((z - wb, r_bot), 0.0), ((z - wt, r_o), 0.0)]
    segs = filleted_polygon(_rim(width, r_in, r_o, verts))
    return segs, dict(od=2 * r_o, root=2 * r_bot, width=width, angle=angle,
                      pd=dd)


def multiv_profile(p, de, ribs, r_in):
    """V-ribbed (multi-V) pulley. de = effective diameter."""
    half = math.radians(p['angle']) / 2
    k = 1 / math.sin(half) - 1
    r_o = de / 2.0 - p['be']
    r_apex = r_o + p['rt'] * k                # sharp ridge before rounding
    r_vb = r_apex - (p['e'] / 2) / math.tan(half)   # sharp groove bottom
    r_bot = r_vb + p['rb'] * k
    dz = (r_o - r_vb) * math.tan(half)
    if r_bot <= r_in + 0.5:
        raise ValueError('Effective diameter is too small for this bore.')
    if p['f'] < dz - 1e-9:
        raise ValueError('Edge distance is too small for the groove.')
    width = (ribs - 1) * p['e'] + 2 * p['f']
    z = [p['f'] + i * p['e'] for i in range(ribs)]
    verts = [((z[-1] + dz, r_o), p['rt'])]
    for i in range(ribs - 1, -1, -1):
        verts.append(((z[i], r_vb), p['rb']))
        if i > 0:
            verts.append(((z[i] - p['e'] / 2, r_apex), p['rt']))
    verts.append(((z[0] - dz, r_o), p['rt']))
    segs = filleted_polygon(_rim(width, r_in, r_o, verts))
    return segs, dict(od=2 * r_o, root=2 * r_bot, width=width, pd=de)


def rope_groove_width(d, R, depth, beta):
    """Groove width at the OD (before the edge fillet)."""
    r_c = -depth + R                          # arc centre below the OD
    half = math.radians(beta) / 2
    ty = r_c - R * math.sin(half)
    if ty < 0:
        return 2 * (R * math.cos(half) + (-ty) * math.tan(half))
    return 2 * math.sqrt(max(R * R - r_c * r_c, 0.0))


def rope_profile(d, pd, R, depth, beta, rf, width, r_in):
    """U-groove for a rope or round belt. pd = pitch diameter (rope centre),
    d = rope diameter, R = groove radius, depth from OD to groove bottom."""
    r_root = pd / 2.0 - d / 2.0
    r_o = r_root + depth
    r_c = r_root + R
    if r_root <= r_in + 0.5:
        raise ValueError('Pitch diameter is too small for this bore.')
    if R < d / 2:
        raise ValueError('Groove radius is smaller than the rope.')
    mouth = rope_groove_width(d, R, depth, beta)
    if width < mouth + 2 * rf + 0.2:
        raise ValueError('Rim width %.2f mm is narrower than the groove '
                         '(%.2f mm).' % (width, mouth + 2 * rf))
    zc = width / 2.0
    half = math.radians(beta) / 2

    # Right half of the groove, from the land down to the bottom, in (z, r).
    tan_pt = (zc + R * math.cos(half), r_c - R * math.sin(half))
    right = []                                # ordered land -> bottom
    t_arc = None
    if tan_pt[1] < r_o:
        corner = (tan_pt[0] + (r_o - tan_pt[1]) * math.tan(half), r_o)
        t1, mid, t2, sb = _fillet_corner((width, r_o), corner, tan_pt, rf)
        if sb <= _len(_sub(corner, tan_pt)) + 1e-9:
            t_arc = tan_pt
            right = [('line', (width, r_o), t1)]
            if mid is not None:
                right.append(('arc', t1, mid, t2))
            right.append(('line', t2, tan_pt))
    if t_arc is None:
        if rf > 1e-9:
            fz = math.sqrt((R + rf) ** 2 - (r_o - rf - r_c) ** 2)
            fc = (zc + fz, r_o - rf)
            t_land = (zc + fz, r_o)
            t_arc = _add((zc, r_c), _mul(_unit(_sub(fc, (zc, r_c))), R))
            right = [('line', (width, r_o), t_land),
                     ('arc', t_land, _mid_of(fc, t_land, t_arc, rf), t_arc)]
        else:
            dz = math.sqrt(max(R * R - (r_o - r_c) ** 2, 0.0))
            t_arc = (zc + dz, r_o)
            right = [('line', (width, r_o), t_arc)]

    def mirror_z(q):
        return (2 * zc - q[0], q[1])

    left = []
    for s in reversed(right):                 # bottom -> land, mirrored
        if s[0] == 'line':
            left.append(('line', mirror_z(s[2]), mirror_z(s[1])))
        else:
            left.append(('arc', mirror_z(s[3]), mirror_z(s[2]), mirror_z(s[1])))

    arc = ('arc', t_arc, (zc, r_root), mirror_z(t_arc))
    segs = [('line', (0.0, r_in), (width, r_in)),
            ('line', (width, r_in), (width, r_o))]
    segs += right + [arc] + left
    segs.append(('line', segs[-1][-1], (0.0, r_in)))
    segs = [s for s in segs if s[0] == 'arc' or _len(_sub(s[2], s[1])) > 1e-9]
    return segs, dict(od=2 * r_o, root=2 * r_root, width=width, pd=pd,
                      mouth=mouth)
