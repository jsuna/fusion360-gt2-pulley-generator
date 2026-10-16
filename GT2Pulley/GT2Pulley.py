"""GT2 Pulley generator for Autodesk Fusion.

Builds a 2 mm pitch GT2 timing pulley body in a new component, with an
optional hub (boss) and optional radial set screw holes.

Geometry (nominal 2GT belt profile, mm):
    PD   = N * 2 / pi                  pitch diameter
    OD   = PD - 2 * 0.254              outside diameter (PLD = 0.254)
    root = OD - 2 * 0.75               groove bottom (belt tooth height 0.75)
    tip arc    R0.555, centre on groove centreline at r_OD - 0.195
    flank arcs R1.00, centres ON the OD circle, ~0.40 either side of the
               centreline, each swinging the flank on the OPPOSITE side,
               internally tangent to the tip arc
    fillets    R0.15 between each flank and the OD

The flank centres are solved exactly (on the OD circle AND tangent to the
tip arc), so the profile stays tangent at any tooth count.

Clearance enlarges the tip and flank radii (groove grows into the pulley);
the OD is never reduced, so the effective pitch is unchanged.

Layout along the axis (z):
    bottom flange   -t .. 0
    teeth            0 .. W           (W = pulley width, excluding hub)
    top flange       W .. W + t
    hub              on top of the top flange (or the teeth if no top flange)
Flanges and hub are bored through. Flange OD = pulley OD + offset
(diameter offset, so the lip above the OD is offset / 2 per side).

Set screws: radial holes from the bore outward, one wall only.
    In hub         - centred on the hub height
    Through teeth  - centred on the pulley width, each hole on a groove
                     centreline (spacing snapped to the nearest groove)
The first hole is on groove 0 (+Y). Extra holes are spaced by the
spacing angle (90 deg default; 180 deg for opposed screws).

Thread: metric or imperial size, and a hole type:
    Plain hole       - drilled at the tap drill size, tap it yourself
    Cosmetic thread  - tap drill hole + Fusion cosmetic thread (drawings/CAM)
    Modeled thread   - real thread geometry (e.g. for 3D printing)

Install as an add-in (adds a "GT2 Pulley" button to Solid > Create):
    copy the GT2Pulley folder (this file + GT2Pulley.manifest) into Fusion's
    API/AddIns folder, then in Utilities > Add-Ins > Scripts and Add-Ins >
    Add-Ins, Run it and tick "Run on Startup". See README.md.
"""

import math
import traceback

import adsk.core
import adsk.fusion

CMD_ID = 'GT2PulleyCmd'

PITCH = 2.0       # mm
PLD = 0.254       # pitch line differential, mm
H = 0.75          # belt tooth height = groove depth, mm
R_TIP = 0.555     # mm
R_FLANK = 1.00    # mm
R_FILLET = 0.15   # mm

SCREW_NONE = 'None'
SCREW_HUB = 'In hub'
SCREW_TEETH = 'Through teeth'

STD_METRIC = 'Metric (ISO)'
STD_IMPERIAL = 'Imperial (UNC/UNF)'

HOLE_PLAIN = 'Plain hole (tap drill size)'
HOLE_COSMETIC = 'Cosmetic thread'
HOLE_MODELED = 'Modeled thread'

# name: (major dia mm, pitch mm, tap drill mm)
THREADS = {
    STD_METRIC: [
        ('M2', 2.0, 0.40, 1.60),
        ('M2.5', 2.5, 0.45, 2.05),
        ('M3', 3.0, 0.50, 2.50),
        ('M4', 4.0, 0.70, 3.30),
        ('M5', 5.0, 0.80, 4.20),
        ('M6', 6.0, 1.00, 5.00),
        ('M8', 8.0, 1.25, 6.80),
    ],
    STD_IMPERIAL: [
        ('#4-40', 0.112 * 25.4, 25.4 / 40, 2.26),
        ('#6-32', 0.138 * 25.4, 25.4 / 32, 2.71),
        ('#8-32', 0.164 * 25.4, 25.4 / 32, 3.45),
        ('#10-24', 0.190 * 25.4, 25.4 / 24, 3.80),
        ('#10-32', 0.190 * 25.4, 25.4 / 32, 4.04),
        ('1/4-20', 0.250 * 25.4, 25.4 / 20, 5.11),
    ],
}
FUSION_THREAD_TYPE = {
    STD_METRIC: 'ISO Metric profile',
    STD_IMPERIAL: 'ANSI Unified Screw Threads',
}


def thread_entry(std, name):
    for e in THREADS[std]:
        if e[0] == name:
            return e
    raise ValueError('Unknown thread size %s' % name)


def hole_diameter(entry, hole_type):
    """Hole drilled for the screw. Modeled threads start from the basic minor
    diameter (D - 1.0825 P) so Fusion can cut the full thread form."""
    _, major, pitch, tap = entry
    if hole_type == HOLE_MODELED:
        return major - 1.0825 * pitch
    return tap

_app = None
_ui = None
_handlers = []


# ---------------------------------------------------------------- geometry

def diameters(n):
    pd = n * PITCH / math.pi
    od = pd - 2 * PLD
    root = od - 2 * H
    return pd, od, root


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


def _toward(c, p, r):
    """Point at distance r from c in the direction of p."""
    dx, dy = p[0] - c[0], p[1] - c[1]
    d = math.hypot(dx, dy)
    return (c[0] + r * dx / d, c[1] + r * dy / d)


def _mirror(p):
    return (-p[0], p[1])


def groove_local(n, clearance=0.0):
    """One groove in a local frame: +y is the groove centreline (radial,
    origin at pulley centre), +x points toward the next groove.

    Returns (arcs, ro) where arcs is a list of (start, end, centre, radius)
    running oL -> fL -> tL -> tR -> fR -> oR along the groove.
    """
    _, od, _ = diameters(n)
    ro = od / 2.0
    rt = R_TIP + clearance
    rfl = R_FLANK + clearance
    rfi = R_FILLET
    d = R_FLANK - R_TIP                    # centre spacing for internal tangency

    ct = (0.0, ro - (H - R_TIP))           # tip arc centre
    sols = _circle_intersections((0.0, 0.0), ro, ct, d)
    if not sols:
        raise ValueError('Could not solve flank centres for N = %d' % n)
    cf_r = min(sols, key=lambda p: p[0])   # x < 0: swings the +x (right) flank
    t_r = _toward(cf_r, ct, rfl)           # flank / tip tangency

    sols = _circle_intersections((0.0, 0.0), ro - rfi, cf_r, rfl + rfi)
    if not sols:
        raise ValueError('Could not solve groove fillets for N = %d' % n)
    cfi_r = max(sols, key=lambda p: p[0])  # fillet centre, right side
    o_r = _toward((0.0, 0.0), cfi_r, ro)   # fillet / OD tangency
    f_r = _toward(cf_r, cfi_r, rfl)        # fillet / flank tangency

    cf_l, t_l, cfi_l = _mirror(cf_r), _mirror(t_r), _mirror(cfi_r)
    o_l, f_l = _mirror(o_r), _mirror(f_r)

    arcs = [
        (o_l, f_l, cfi_l, rfi),   # left fillet
        (f_l, t_l, cf_l, rfl),    # left flank (centre on the right)
        (t_l, t_r, ct, rt),       # tip
        (t_r, f_r, cf_r, rfl),    # right flank (centre on the left)
        (f_r, o_r, cfi_r, rfi),   # right fillet
    ]
    return arcs, ro


def _rotate(p, th):
    """Local (x tangential, y radial) -> global, groove centreline at angle th."""
    x, y = p
    c, s = math.cos(th), math.sin(th)
    return (y * c - x * s, y * s + x * c)


def _arc_mid(a, b, c, r):
    """Midpoint of the minor arc a->b about centre c."""
    vx = (a[0] - c[0]) + (b[0] - c[0])
    vy = (a[1] - c[1]) + (b[1] - c[1])
    v = math.hypot(vx, vy)
    return (c[0] + r * vx / v, c[1] + r * vy / v)


def pulley_outline(n, clearance=0.0):
    """Closed outline as a list of three-point arcs (start, mid, end), in mm."""
    arcs_local, ro = groove_local(n, clearance)
    o_r = arcs_local[-1][1]
    half = math.atan2(o_r[0], o_r[1])          # groove half-width as an angle
    if 2 * half >= 2 * math.pi / n - 0.02 / ro:
        raise ValueError('Clearance %.2f mm makes neighbouring grooves overlap '
                         'at %dT; reduce it.' % (clearance, n))
    out = []
    for k in range(n):
        th = math.pi / 2 + 2 * math.pi * k / n
        th_next = th + 2 * math.pi / n
        for a, b, c, r in arcs_local:
            ga, gb, gc = _rotate(a, th), _rotate(b, th), _rotate(c, th)
            out.append((ga, _arc_mid(ga, gb, gc, r), gb))
        # OD land from this groove's right edge to the next groove's left edge
        a = _rotate(arcs_local[-1][1], th)
        b = _rotate(arcs_local[0][0], th_next)
        out.append((a, _arc_mid(a, b, (0.0, 0.0), ro), b))
    return out


def screw_angles(n, mode, count, spacing):
    """Angles (radians, from +X) of the set screw holes.

    The first hole sits on groove 0 (+Y). Through-teeth holes are snapped to
    the nearest groove centreline so the hole is centred in a groove.
    """
    base = math.pi / 2
    step = 2 * math.pi / n
    out = []
    for i in range(count):
        a = i * spacing
        if mode == SCREW_TEETH:
            a = round(a / step) * step
        out.append(base + a)
    return out


# ---------------------------------------------------------------- Fusion

def _p3(p):
    return adsk.core.Point3D.create(p[0] / 10.0, p[1] / 10.0, 0)   # mm -> cm


def _pt(x, y, z):
    return adsk.core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)  # mm -> cm


def _cylinder(tbm, p0, p1, r_mm):
    return tbm.createCylinderOrCone(p0, r_mm / 10.0, p1, r_mm / 10.0)


def _add_temp_bodies(design, comp, temp_bodies):
    """Put temporary BRep bodies into the component, parametric or direct."""
    if design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        for b in temp_bodies:
            comp.bRepBodies.add(b, bf)
        bf.finishEdit()
        return [bf.bodies.item(i) for i in range(bf.bodies.count)]
    return [comp.bRepBodies.add(b) for b in temp_bodies]


def _combine(comp, target, tools, operation):
    coll = adsk.core.ObjectCollection.create()
    for t in tools:
        coll.add(t)
    ci = comp.features.combineFeatures.createInput(target, coll)
    ci.operation = operation
    ci.isKeepToolBodies = False
    comp.features.combineFeatures.add(ci)


def _num(s):
    try:
        return float(s)
    except ValueError:
        if '/' in s:
            a, b = s.split('/', 1)
            return float(a) / float(b)
        raise


def _thread_info(tf, std, entry):
    """Find the matching Fusion thread designation and class for an entry."""
    name, major, pitch, _ = entry
    ttype = FUSION_THREAD_TYPE[std]
    q = tf.threadDataQuery
    desig = None
    for size in q.allSizes(ttype):
        s = size.strip().lstrip('#')
        if std == STD_METRIC:
            try:
                ok = abs(_num(s) - major) < 1e-3
            except ValueError:
                ok = False
        else:
            ok = s == name.split('-')[0].lstrip('#')
        if not ok:
            continue
        for d in q.allDesignations(ttype, size):
            if std == STD_METRIC:
                p = d.lower().split('x')[-1].split()[0]
                try:
                    if abs(float(p) - pitch) < 1e-3:
                        desig = d
                except ValueError:
                    pass
            else:
                tpi = round(25.4 / pitch)
                if ('-%d' % tpi) in d:
                    desig = d
            if desig:
                break
        if desig:
            break
    if not desig:
        ok, d, cls = q.recommendThreadData(major / 10.0, True, ttype)
        if not ok:
            raise ValueError('Fusion has no thread data for %s.' % name)
        desig = d
    classes = list(q.allClasses(True, ttype, desig))
    want = '6H' if std == STD_METRIC else '2B'
    cls = want if want in classes else (classes[0] if classes else '')
    return tf.createThreadInfo(True, ttype, desig, cls), desig


def _radial_hole_faces(body, r_cm):
    """Cylindrical faces of the radial set screw holes (axis perpendicular to Z)."""
    out = []
    for f in body.faces:
        cyl = adsk.core.Cylinder.cast(f.geometry)
        if cyl and abs(cyl.radius - r_cm) < 1e-4 and abs(cyl.axis.z) < 1e-6:
            out.append(f)
    return out


def _add_threads(comp, body, r_cm, std, entry, modeled):
    """Thread every radial screw hole. Returns how many holes failed."""
    tf = comp.features.threadFeatures
    info, _ = _thread_info(tf, std, entry)
    faces = _radial_hole_faces(body, r_cm)
    if not faces:
        return 0

    def add(face_list):
        coll = adsk.core.ObjectCollection.create()
        for f in face_list:
            coll.add(f)
        ti = tf.createInput(coll, info)
        ti.isFullLength = True
        ti.isModeled = modeled
        tf.add(ti)

    try:
        add(faces)                     # all holes in one feature
        return 0
    except Exception:
        pass
    failed = 0                         # fall back to one hole at a time
    for i in range(len(faces)):
        remaining = _radial_hole_faces(comp.bRepBodies.item(0), r_cm)
        if not remaining:
            break
        try:
            add([remaining[0]])
        except Exception:
            failed += 1
    return failed


def build_pulley(n, width, bore, clearance,
                 flange_bottom=False, flange_top=False, flange_od_offset=4.0, flange_t=1.0,
                 hub=False, hub_dia=0.0, hub_h=0.0,
                 screw_mode=SCREW_NONE, thread_std=STD_METRIC, thread_size='M4',
                 hole_type=HOLE_PLAIN, screw_count=2, screw_spacing=math.pi / 2):
    """Builds the pulley. Returns a warning string ('' if none)."""
    entry = thread_entry(thread_std, thread_size)
    screw_dia = entry[1]               # major diameter, for fit checks
    hole_dia = hole_diameter(entry, hole_type)
    design = adsk.fusion.Design.cast(_app.activeProduct)
    if not design:
        raise RuntimeError('Open a Fusion design first.')

    _, od, root = diameters(n)
    if bore >= root - 2 * clearance - 1.0:
        raise ValueError('Bore %.2f mm is too large for a %dT pulley '
                         '(root %.2f mm).' % (bore, n, root))
    flanges = flange_bottom or flange_top
    if flanges:
        if flange_od_offset <= 0:
            raise ValueError('Flange OD offset must be greater than 0.')
        flange_od = od + flange_od_offset
        if flange_t <= 0:
            raise ValueError('Flange thickness must be greater than 0.')
    if hub:
        if hub_h <= 0:
            raise ValueError('Hub height must be greater than 0.')
        if hub_dia <= bore + 1.0:
            raise ValueError('Hub diameter must be at least 1 mm larger than the bore.')
    if screw_mode != SCREW_NONE:
        if bore <= 0:
            raise ValueError('Set screws need a bore.')
        if screw_mode == SCREW_HUB:
            if not hub:
                raise ValueError('Set screws "In hub" need the hub turned on.')
            if screw_dia >= hub_h:
                raise ValueError('Screw hole %.1f mm does not fit in a %.1f mm '
                                 'hub height.' % (screw_dia, hub_h))
        if screw_mode == SCREW_TEETH and screw_dia >= width:
            raise ValueError('Screw hole %.1f mm does not fit in a %.1f mm '
                             'pulley width.' % (screw_dia, width))

    occ = design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = 'GT2 %dT' % n

    # ---- toothed body
    sketch = comp.sketches.add(comp.xYConstructionPlane)
    sketch.name = 'GT2 %dT profile' % n
    sketch.isComputeDeferred = True

    outline = pulley_outline(n, clearance)
    pts = sketch.sketchPoints
    arcs = sketch.sketchCurves.sketchArcs
    first = pts.add(_p3(outline[0][0]))
    start = first
    for i, (a, m, b) in enumerate(outline):
        end = first if i == len(outline) - 1 else pts.add(_p3(b))
        arcs.addByThreePoints(start, _p3(m), end)
        start = end

    if bore > 0:
        sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), bore / 20.0)

    sketch.isComputeDeferred = False

    want_loops = 2 if bore > 0 else 1
    prof = None
    for p in sketch.profiles:
        if p.profileLoops.count == want_loops:
            prof = p
            break
    if prof is None:
        raise RuntimeError('Pulley profile did not close; try a different clearance.')

    ext = comp.features.extrudeFeatures.addSimple(
        prof, adsk.core.ValueInput.createByReal(width / 10.0),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = ext.bodies.item(0)

    tbm = adsk.fusion.TemporaryBRepManager.get()

    def tube(z0, z1, dia):
        t = _cylinder(tbm, _pt(0, 0, z0), _pt(0, 0, z1), dia / 2.0)
        if bore > 0:
            hole = _cylinder(tbm, _pt(0, 0, z0 - 1), _pt(0, 0, z1 + 1), bore / 2.0)
            tbm.booleanOperation(t, hole,
                                 adsk.fusion.BooleanTypes.DifferenceBooleanType)
        return t

    # ---- flanges and hub, bored through
    hub_z0 = width + (flange_t if flange_top else 0.0)
    adds = []
    if flange_bottom:
        adds.append(tube(-flange_t, 0.0, flange_od))
    if flange_top:
        adds.append(tube(width, width + flange_t, flange_od))
    if hub:
        adds.append(tube(hub_z0, hub_z0 + hub_h, hub_dia))
    if adds:
        tools = _add_temp_bodies(design, comp, adds)
        _combine(comp, body, tools, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        body = comp.bRepBodies.item(0)

    # ---- set screw holes (bore outward, one wall each)
    if screw_mode != SCREW_NONE:
        if screw_mode == SCREW_HUB:
            z = hub_z0 + hub_h / 2.0
            reach = hub_dia / 2.0 + 1.0
        else:
            z = width / 2.0
            reach = od / 2.0 + 1.0
        holes = []
        for a in screw_angles(n, screw_mode, screw_count, screw_spacing):
            holes.append(_cylinder(tbm, _pt(0, 0, z),
                                   _pt(reach * math.cos(a), reach * math.sin(a), z),
                                   hole_dia / 2.0))
        tools = _add_temp_bodies(design, comp, holes)
        _combine(comp, body, tools, adsk.fusion.FeatureOperations.CutFeatureOperation)
        body = comp.bRepBodies.item(0)

    body.name = 'GT2 %dT' % n

    warning = ''
    if screw_mode != SCREW_NONE and hole_type != HOLE_PLAIN:
        try:
            failed = _add_threads(comp, body, hole_dia / 20.0, thread_std, entry,
                                  hole_type == HOLE_MODELED)
            if failed:
                warning = ('%d of the %s holes could not be threaded; they were '
                           'left at %.2f mm.' % (failed, thread_size, hole_dia))
        except Exception as e:
            warning = ('Threads could not be added (%s); the holes were left '
                       'at %.2f mm.' % (e, hole_dia))
    return warning


def _info_text(n):
    pd, od, root = diameters(n)
    return ('PD %.3f mm\nOD %.3f mm\nRoot %.3f mm\nAngular pitch %.3f deg'
            % (pd, od, root, 360.0 / n))


def _update_visibility(inputs):
    fl_on = inputs.itemById('flangeBottom').value or inputs.itemById('flangeTop').value
    inputs.itemById('flangeOD').isVisible = fl_on
    inputs.itemById('flangeThk').isVisible = fl_on
    hub_on = inputs.itemById('hub').value
    inputs.itemById('hubDia').isVisible = hub_on
    inputs.itemById('hubHeight').isVisible = hub_on
    screws_on = inputs.itemById('screws').selectedItem.name != SCREW_NONE
    for i in ('threadStd', 'threadSize', 'holeType', 'screwCount', 'screwSpacing'):
        inputs.itemById(i).isVisible = screws_on


def _fill_sizes(inputs):
    std = inputs.itemById('threadStd').selectedItem.name
    items = inputs.itemById('threadSize').listItems
    items.clear()
    default = 'M4' if std == STD_METRIC else '#8-32'
    for e in THREADS[std]:
        items.add(e[0], e[0] == default)


class _CreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            inputs = cmd.commandInputs
            vi = adsk.core.ValueInput.createByString

            inputs.addIntegerSpinnerCommandInput('teeth', 'Teeth', 8, 500, 1, 60)
            inputs.addValueInput('width', 'Pulley width (excluding hub)', 'mm', vi('7 mm'))
            inputs.addValueInput('bore', 'Bore diameter (0 = none)', 'mm', vi('5 mm'))
            inputs.addValueInput('clearance', 'Groove clearance', 'mm', vi('0 mm'))
            inputs.addTextBoxCommandInput('info', 'Diameters', _info_text(60), 4, True)

            inputs.addBoolValueInput('flangeBottom', 'Bottom flange', True, '', False)
            inputs.addBoolValueInput('flangeTop', 'Top flange', True, '', False)
            inputs.addValueInput('flangeOD', 'Flange OD = pulley OD +', 'mm', vi('4 mm'))
            inputs.addValueInput('flangeThk', 'Flange thickness', 'mm', vi('1 mm'))

            inputs.addBoolValueInput('hub', 'Hub (boss)', True, '', False)
            inputs.addValueInput('hubDia', 'Hub diameter', 'mm', vi('16 mm'))
            inputs.addValueInput('hubHeight', 'Hub height', 'mm', vi('7 mm'))

            dd = inputs.addDropDownCommandInput(
                'screws', 'Set screws', adsk.core.DropDownStyles.TextListDropDownStyle)
            dd.listItems.add(SCREW_NONE, True)
            dd.listItems.add(SCREW_HUB, False)
            dd.listItems.add(SCREW_TEETH, False)
            text = adsk.core.DropDownStyles.TextListDropDownStyle
            std = inputs.addDropDownCommandInput('threadStd', 'Thread standard', text)
            std.listItems.add(STD_METRIC, True)
            std.listItems.add(STD_IMPERIAL, False)
            inputs.addDropDownCommandInput('threadSize', 'Thread size', text)
            _fill_sizes(inputs)
            ht = inputs.addDropDownCommandInput('holeType', 'Hole', text)
            ht.listItems.add(HOLE_PLAIN, True)
            ht.listItems.add(HOLE_COSMETIC, False)
            ht.listItems.add(HOLE_MODELED, False)
            inputs.addIntegerSpinnerCommandInput('screwCount', 'Number of screws', 1, 4, 1, 2)
            inputs.addValueInput('screwSpacing', 'Screw spacing', 'deg', vi('90 deg'))

            _update_visibility(inputs)

            on_exec = _ExecuteHandler()
            cmd.execute.add(on_exec)
            _handlers.append(on_exec)
            on_change = _ChangedHandler()
            cmd.inputChanged.add(on_change)
            _handlers.append(on_change)
            on_destroy = _DestroyHandler()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


class _ChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            inputs = args.inputs
            if args.input.id == 'teeth':
                n = inputs.itemById('teeth').value
                inputs.itemById('info').text = _info_text(n)
            if args.input.id == 'threadStd':
                _fill_sizes(inputs)
            if args.input.id in ('hub', 'screws', 'flangeBottom', 'flangeTop'):
                _update_visibility(inputs)
        except Exception:
            pass


class _ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            inputs = args.command.commandInputs
            mm = lambda i: inputs.itemById(i).value * 10.0      # cm -> mm
            n = inputs.itemById('teeth').value
            clearance = mm('clearance')
            if clearance < 0 or clearance > 0.2:
                raise ValueError('Clearance should be between 0 and 0.2 mm.')
            pick = lambda i: inputs.itemById(i).selectedItem.name
            warning = build_pulley(
                n, mm('width'), mm('bore'), clearance,
                flange_bottom=inputs.itemById('flangeBottom').value,
                flange_top=inputs.itemById('flangeTop').value,
                flange_od_offset=mm('flangeOD'), flange_t=mm('flangeThk'),
                hub=inputs.itemById('hub').value,
                hub_dia=mm('hubDia'), hub_h=mm('hubHeight'),
                screw_mode=pick('screws'),
                thread_std=pick('threadStd'),
                thread_size=pick('threadSize'),
                hole_type=pick('holeType'),
                screw_count=inputs.itemById('screwCount').value,
                screw_spacing=inputs.itemById('screwSpacing').value)   # radians
            if warning:
                _ui.messageBox('GT2 Pulley: ' + warning)
        except Exception as e:
            _ui.messageBox('GT2 Pulley: {}'.format(e))


class _DestroyHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        pass                       # add-in stays loaded; nothing to clean up


PANEL_ID = 'SolidCreatePanel'      # Solid tab > Create (where Spur Gear lives)


def run(context):
    global _app, _ui
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface
        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
        cmd_def = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, 'GT2 Pulley', 'Creates a 2 mm pitch GT2 timing pulley.')
        on_created = _CreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        panel = _ui.allToolbarPanels.itemById(PANEL_ID)
        if panel and not panel.controls.itemById(CMD_ID):
            panel.controls.addCommand(cmd_def)
    except Exception:
        if _ui:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def stop(context):
    try:
        if not _ui:
            return
        panel = _ui.allToolbarPanels.itemById(PANEL_ID)
        control = panel.controls.itemById(CMD_ID) if panel else None
        if control:
            control.deleteMe()
        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
    except Exception:
        pass
