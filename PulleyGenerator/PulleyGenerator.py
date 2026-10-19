"""Pulley Generator add-in for Autodesk Fusion.

Adds a "Pulley Generator" button to Solid > Create. Pick a pulley type and
the dialog shows that type's settings:

    Timing        GT2, GT 3/5 mm, HTD 3M/5M/8M, MXL, XL, L, T2.5/T5/T10, Custom
    V-belt        Z, A / 4L, B / 5L, SPZ, SPA, SPB, Custom
    Multi-V       PH, PJ, PK, PL, PM (V-ribbed), Custom
    Round / rope  round belt or wire rope U-groove, Custom

Every type shares the bore, hub and set screw (with thread) options; timing
pulleys can also have flanges. Choosing Custom starts from the last preset's
values. Groove data and sources are in standards.py; geometry in profiles.py.

Layout along the axis (z):
    bottom flange   -t .. 0        (timing only)
    pulley           0 .. W        (W = pulley / rim width, excluding hub)
    top flange       W .. W + t    (timing only)
    hub              on top of the top flange or the pulley

Install: copy the PulleyGenerator folder into Fusion's API/AddIns folder,
then in Utilities > Add-Ins > Scripts and Add-Ins > Add-Ins, Run it and tick
"Run on Startup".
"""

import math
import traceback

import adsk.core
import adsk.fusion

from . import profiles
from . import standards

CMD_ID = 'PulleyGeneratorCmd'
PANEL_ID = 'SolidCreatePanel'

TYPE_TIMING = 'Timing (toothed)'
TYPE_V = 'V-belt'
TYPE_MULTIV = 'Multi-V (ribbed)'
TYPE_ROPE = 'Round belt / rope (U-groove)'
TYPES = [TYPE_TIMING, TYPE_V, TYPE_MULTIV, TYPE_ROPE]
CUSTOM = 'Custom'

FAM_ARC = 'GT style (arcs)'
FAM_TRAP = 'Trapezoid (straight sides)'
FAM_ROUND = 'Round (HTD style)'
FAMILIES = {FAM_ARC: 'arc', FAM_TRAP: 'trap', FAM_ROUND: 'round'}

SCREW_NONE = 'None'
SCREW_HUB = 'In hub'
SCREW_RIM = 'Through teeth / rim'

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

_app = None
_ui = None
_handlers = []


class _Inputs(object):
    """Every dialog input by id, including those inside groups, so lookups
    never depend on which collection an event hands over."""

    def __init__(self):
        self._by_id = {}

    def collect(self, coll):
        for i in range(coll.count):
            inp = coll.item(i)
            self._by_id[inp.id] = inp
            group = adsk.core.GroupCommandInput.cast(inp)
            if group:
                self.collect(group.children)

    def itemById(self, id_):
        return self._by_id[id_]


_dialog = _Inputs()


# ================================================================ dialog spec
#
# Custom-field ids, with the preset key each one mirrors and its unit.

TIMING_CUSTOM = [            # id, key, label, unit, families
    ('cPitch', 'pitch', 'Pitch', 'mm', 'arc trap round'),
    ('cPld', 'pld', 'Pitch line differential', 'mm', 'arc trap round'),
    ('cH', 'h', 'Groove depth', 'mm', 'arc round'),
    ('cRtip', 'r_tip', 'Tip radius', 'mm', 'arc'),
    ('cRflank', 'r_flank', 'Flank radius', 'mm', 'arc'),
    ('cRfillet', 'r_fillet', 'Edge fillet', 'mm', 'arc'),
    ('cBw', 'bw', 'Groove bottom width', 'mm', 'trap'),
    ('cHg', 'hg', 'Groove depth ', 'mm', 'trap'),
    ('cPhi', 'phi', 'Flank angle (from radial)', 'deg', 'trap'),
    ('cRb', 'rb', 'Bottom radius', 'mm', 'trap'),
    ('cRt', 'rt', 'Top radius', 'mm', 'trap'),
    ('cR', 'r', 'Groove radius', 'mm', 'round'),
    ('cBeta', 'beta', 'Included flank angle', 'deg', 'round'),
    ('cRf', 'rf', 'Edge fillet ', 'mm', 'round'),
]

V_CUSTOM = [
    ('vWd', 'wd', 'Datum width', 'mm'),
    ('vB', 'b', 'Groove above datum', 'mm'),
    ('vH', 'h', 'Groove below datum', 'mm'),
    ('vAngle', 'angle', 'Groove angle', 'deg'),
    ('vE', 'e', 'Groove spacing', 'mm'),
    ('vF', 'f', 'Edge distance', 'mm'),
]

MV_CUSTOM = [
    ('mE', 'e', 'Rib pitch', 'mm'),
    ('mAngle', 'angle', 'Groove angle', 'deg'),
    ('mRt', 'rt', 'Rib tip radius', 'mm'),
    ('mRb', 'rb', 'Groove bottom radius', 'mm'),
    ('mBe', 'be', 'Effective line differential', 'mm'),
    ('mF', 'f', 'Edge distance', 'mm'),
]

ROPE_CUSTOM = [              # factors of the rope diameter in presets
    ('rR', 'r', 'Groove radius', 'mm'),
    ('rDepth', 'depth', 'Groove depth', 'mm'),
    ('rBeta', 'beta', 'Throat angle (included)', 'deg'),
]


def _vi(text):
    return adsk.core.ValueInput.createByString(text)


def _get(inputs, id_, unit):
    """Read a value input in mm or degrees."""
    v = inputs.itemById(id_).value
    return math.degrees(v) if unit == 'deg' else v * 10.0


def _set(inputs, id_, unit, value):
    inputs.itemById(id_).value = math.radians(value) if unit == 'deg' else value / 10.0


def _pick(inputs, id_):
    return inputs.itemById(id_).selectedItem.name


# ================================================================ parameters

def _timing_params(inputs):
    name = _pick(inputs, 'tProfile')
    if name != CUSTOM:
        return name, standards.lookup(standards.TIMING, name)
    fam = FAMILIES[_pick(inputs, 'cFamily')]
    p = {'family': fam}
    for id_, key, _, unit, fams in TIMING_CUSTOM:
        if fam in fams.split():
            p[key] = _get(inputs, id_, unit)
    return 'Custom', p


def _v_params(inputs):
    name = _pick(inputs, 'vSection')
    if name != CUSTOM:
        return name, standards.lookup(standards.V_BELT, name)
    p = {key: _get(inputs, id_, unit) for id_, key, _, unit in V_CUSTOM}
    p['dd_34'] = 0
    p['dd_min'] = 0
    return 'Custom', p


def _mv_params(inputs):
    name = _pick(inputs, 'mSection')
    if name != CUSTOM:
        return name, standards.lookup(standards.MULTI_V, name)
    p = {key: _get(inputs, id_, unit) for id_, key, _, unit in MV_CUSTOM}
    p['de_min'] = 0
    return 'Custom', p


def _rope_params(inputs):
    d = _get(inputs, 'rD', 'mm')
    name = _pick(inputs, 'rPreset')
    if name != CUSTOM:
        f = standards.lookup(standards.ROPE, name)
        return name, dict(r=f['r'] * d, depth=f['depth'] * d, beta=f['beta'])
    return 'Custom', {key: _get(inputs, id_, unit) for id_, key, _, unit in ROPE_CUSTOM}


def _profile(inputs):
    """Work out the pulley outline from the dialog.

    Returns dict with kind ('extrude' or 'revolve'), segs (mm), info, width,
    name, n_teeth (timing) and warn."""
    ptype = _pick(inputs, 'ptype')
    bore = _get(inputs, 'bore', 'mm')
    r_in = bore / 2.0
    warn = ''
    if ptype == TYPE_TIMING:
        name, p = _timing_params(inputs)
        n = inputs.itemById('teeth').value
        clearance = _get(inputs, 'tClear', 'mm')
        if clearance < 0 or clearance > 0.3:
            raise ValueError('Groove clearance should be between 0 and 0.3 mm.')
        segs, info = profiles.timing_outline(p, n, clearance)
        if bore and bore >= info['root'] - 1.0:
            raise ValueError('Bore is too large for this pulley (root %.2f mm).'
                             % info['root'])
        width = _get(inputs, 'tWidth', 'mm')
        if width <= 0:
            raise ValueError('Pulley width must be greater than 0.')
        info['width'] = width
        if 'approx' in name:
            warn = 'Approximate profile (see README).'
        return dict(kind='extrude', segs=segs, info=info, width=width, warn=warn,
                    name='%s %dT' % (name.replace(' (approx.)', ''), n), n_teeth=n)

    if ptype == TYPE_V:
        name, p = _v_params(inputs)
        dd = _get(inputs, 'vDd', 'mm')
        n = inputs.itemById('vGrooves').value
        segs, info = profiles.v_profile(p, dd, n, r_in)
        if dd < p.get('dd_min', 0):
            warn = 'Below the %g mm minimum datum diameter for %s.' % (p['dd_min'], name)
        return dict(kind='revolve', segs=segs, info=info, width=info['width'],
                    warn=warn, name='%s %g x%d' % (name, dd, n), n_teeth=0)

    if ptype == TYPE_MULTIV:
        name, p = _mv_params(inputs)
        de = _get(inputs, 'mDe', 'mm')
        n = inputs.itemById('mRibs').value
        segs, info = profiles.multiv_profile(p, de, n, r_in)
        if de < p.get('de_min', 0):
            warn = 'Below the %g mm minimum effective diameter for %s.' % (p['de_min'], name)
        return dict(kind='revolve', segs=segs, info=info, width=info['width'],
                    warn=warn, name='%s %g x%d' % (name, de, n), n_teeth=0)

    name, p = _rope_params(inputs)
    d = _get(inputs, 'rD', 'mm')
    pd = _get(inputs, 'rPd', 'mm')
    rf = _get(inputs, 'rRf', 'mm')
    width = _get(inputs, 'rWidth', 'mm')
    if width <= 0:                            # auto: groove plus 1.5 mm lands
        width = profiles.rope_groove_width(d, p['r'], p['depth'], p['beta']) + 2 * rf + 3.0
    segs, info = profiles.rope_profile(d, pd, p['r'], p['depth'], p['beta'], rf,
                                       width, r_in)
    return dict(kind='revolve', segs=segs, info=info, width=width, warn=warn,
                name='Rope %g PD %g' % (d, pd), n_teeth=0)


def _info_text(inputs):
    try:
        pr = _profile(inputs)
    except Exception as e:
        return 'Error: %s' % e
    i = pr['info']
    if pr['kind'] == 'extrude':
        lines = ['PD %.3f mm   OD %.3f mm' % (i['pd'], i['od']),
                 'Root %.3f mm   Pitch angle %.3f deg' % (i['root'], 360.0 / pr['n_teeth'])]
    else:
        label = {TYPE_V: 'Datum', TYPE_MULTIV: 'Effective'}.get(_pick(inputs, 'ptype'), 'Pitch')
        lines = ['%s %.2f mm   OD %.2f mm' % (label, i['pd'], i['od']),
                 'Root %.2f mm   Width %.2f mm' % (i['root'], i['width'])]
        if 'angle' in i:
            lines[-1] += '   Angle %g deg' % i['angle']
    if pr['warn']:
        lines.append(pr['warn'])
    return '\n'.join(lines)


# ================================================================ Fusion build

def _pt(x, y, z):
    return adsk.core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)   # mm -> cm


def _cylinder(tbm, p0, p1, r_mm):
    return tbm.createCylinderOrCone(p0, r_mm / 10.0, p1, r_mm / 10.0)


def _draw(sketch, segs, to_sk):
    """Draw a closed chain of segments; to_sk maps an (a, b) mm pair to a
    sketch-space Point3D."""
    pts = sketch.sketchPoints
    lines = sketch.sketchCurves.sketchLines
    arcs = sketch.sketchCurves.sketchArcs
    first = pts.add(to_sk(segs[0][1]))
    start = first
    for i, s in enumerate(segs):
        end = first if i == len(segs) - 1 else pts.add(to_sk(s[-1]))
        if s[0] == 'line':
            lines.addByTwoPoints(start, end)
        else:
            arcs.addByThreePoints(start, to_sk(s[2]), end)
        start = end


def _largest_profile(sketch):
    best, area = None, -1.0
    for i in range(sketch.profiles.count):
        p = sketch.profiles.item(i)
        a = p.areaProperties().area
        if a > area:
            best, area = p, a
    if best is None:
        raise RuntimeError('The pulley profile did not close.')
    return best


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


def screw_angles(n_teeth, mode, count, spacing):
    """Hole angles (radians from +X). The first sits on +Y (groove 0 for
    timing pulleys). Through-teeth holes snap to the nearest groove."""
    base = math.pi / 2
    out = []
    for i in range(count):
        a = i * spacing
        if mode == SCREW_RIM and n_teeth:
            step = 2 * math.pi / n_teeth
            a = round(a / step) * step
        out.append(base + a)
    return out


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
                if ('-%d' % round(25.4 / pitch)) in d:
                    desig = d
            if desig:
                break
        if desig:
            break
    if not desig:
        ok, d, _ = q.recommendThreadData(major / 10.0, True, ttype)
        if not ok:
            raise ValueError('Fusion has no thread data for %s.' % name)
        desig = d
    classes = list(q.allClasses(True, ttype, desig))
    want = '6H' if std == STD_METRIC else '2B'
    cls = want if want in classes else (classes[0] if classes else '')
    return tf.createThreadInfo(True, ttype, desig, cls)


def _hole_face(body, r_cm, angle):
    """The cylindrical face of the radial hole pointing at angle."""
    ax, ay = math.cos(angle), math.sin(angle)
    for f in body.faces:
        cyl = adsk.core.Cylinder.cast(f.geometry)
        if (cyl and abs(cyl.radius - r_cm) < 1e-4 and abs(cyl.axis.z) < 1e-6
                and abs(abs(cyl.axis.x * ax + cyl.axis.y * ay) - 1) < 1e-6):
            return f
    return None


def _add_threads(comp, r_cm, angles, std, entry, modeled):
    """Thread each radial hole on its own. Returns how many failed."""
    tf = comp.features.threadFeatures
    info = _thread_info(tf, std, entry)
    failed = 0
    for a in angles:
        face = _hole_face(comp.bRepBodies.item(0), r_cm, a)
        try:
            if face is None:
                raise RuntimeError('hole not found')
            coll = adsk.core.ObjectCollection.create()
            coll.add(face)
            ti = tf.createInput(coll, info)
            ti.isFullLength = True
            ti.isModeled = modeled
            tf.add(ti)
        except Exception:
            failed += 1
    return failed


def build_pulley(inputs):
    """Build the pulley from the dialog. Returns a warning string."""
    design = adsk.fusion.Design.cast(_app.activeProduct)
    if not design:
        raise RuntimeError('Open a Fusion design first.')

    pr = _profile(inputs)
    info = pr['info']
    width = pr['width']
    od = info['od']
    bore = _get(inputs, 'bore', 'mm')
    timing = pr['kind'] == 'extrude'

    flange_bottom = timing and inputs.itemById('flBottom').value
    flange_top = timing and inputs.itemById('flTop').value
    flange_od = od + _get(inputs, 'flOD', 'mm')
    flange_t = _get(inputs, 'flThk', 'mm')
    hub = inputs.itemById('hub').value
    hub_dia = _get(inputs, 'hubDia', 'mm')
    hub_h = _get(inputs, 'hubHeight', 'mm')
    screw_mode = _pick(inputs, 'screws')
    std = _pick(inputs, 'threadStd')
    entry = thread_entry(std, _pick(inputs, 'threadSize'))
    hole_type = _pick(inputs, 'holeType')
    screw_count = inputs.itemById('screwCount').value
    spacing = inputs.itemById('screwSpacing').value          # radians
    hole_dia = hole_diameter(entry, hole_type)

    # ---- checks
    if (flange_bottom or flange_top):
        if _get(inputs, 'flOD', 'mm') <= 0:
            raise ValueError('Flange OD offset must be greater than 0.')
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
            if entry[1] >= hub_h:
                raise ValueError('%s does not fit in a %.1f mm hub height.'
                                 % (entry[0], hub_h))
        elif entry[1] >= width:
            raise ValueError('%s does not fit in a %.1f mm pulley width.'
                             % (entry[0], width))

    occ = design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = pr['name']

    # ---- pulley body
    if timing:
        sketch = comp.sketches.add(comp.xYConstructionPlane)
        sketch.name = '%s profile' % pr['name']
        sketch.isComputeDeferred = True
        _draw(sketch, pr['segs'], lambda q: adsk.core.Point3D.create(q[0] / 10.0, q[1] / 10.0, 0))
        if bore > 0:
            sketch.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(0, 0, 0), bore / 20.0)
        sketch.isComputeDeferred = False
        want = 2 if bore > 0 else 1
        prof = None
        for i in range(sketch.profiles.count):
            if sketch.profiles.item(i).profileLoops.count == want:
                prof = sketch.profiles.item(i)
                break
        if prof is None:
            raise RuntimeError('Pulley profile did not close; try another clearance.')
        ext = comp.features.extrudeFeatures.addSimple(
            prof, adsk.core.ValueInput.createByReal(width / 10.0),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        body = ext.bodies.item(0)
    else:
        sketch = comp.sketches.add(comp.xZConstructionPlane)
        sketch.name = '%s section' % pr['name']
        sketch.isComputeDeferred = True
        _draw(sketch, pr['segs'],
              lambda q: sketch.modelToSketchSpace(_pt(q[1], 0, q[0])))   # (z, r)
        sketch.isComputeDeferred = False
        revs = comp.features.revolveFeatures
        ri = revs.createInput(_largest_profile(sketch), comp.zConstructionAxis,
                              adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ri.setAngleExtent(False, _vi('360 deg'))
        body = revs.add(ri).bodies.item(0)

    tbm = adsk.fusion.TemporaryBRepManager.get()

    def tube(z0, z1, dia):
        t = _cylinder(tbm, _pt(0, 0, z0), _pt(0, 0, z1), dia / 2.0)
        if bore > 0:
            hole = _cylinder(tbm, _pt(0, 0, z0 - 1), _pt(0, 0, z1 + 1), bore / 2.0)
            tbm.booleanOperation(t, hole, adsk.fusion.BooleanTypes.DifferenceBooleanType)
        return t

    # ---- flanges and hub
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

    # ---- set screws
    angles = []
    if screw_mode != SCREW_NONE:
        if screw_mode == SCREW_HUB:
            z = hub_z0 + hub_h / 2.0
            reach = hub_dia / 2.0 + 1.0
        else:
            z = width / 2.0
            reach = od / 2.0 + 1.0
        angles = screw_angles(pr['n_teeth'], screw_mode, screw_count, spacing)
        holes = [_cylinder(tbm, _pt(0, 0, z),
                           _pt(reach * math.cos(a), reach * math.sin(a), z),
                           hole_dia / 2.0) for a in angles]
        tools = _add_temp_bodies(design, comp, holes)
        _combine(comp, body, tools, adsk.fusion.FeatureOperations.CutFeatureOperation)
        body = comp.bRepBodies.item(0)

    body.name = pr['name']

    warning = ''
    if angles and hole_type != HOLE_PLAIN:
        try:
            failed = _add_threads(comp, hole_dia / 20.0, angles, std, entry,
                                  hole_type == HOLE_MODELED)
            if failed:
                warning = ('%d of the %s holes could not be threaded; they were '
                           'left at %.2f mm.' % (failed, entry[0], hole_dia))
        except Exception as e:
            warning = ('Threads could not be added (%s); the holes were left at '
                       '%.2f mm.' % (e, hole_dia))
    return warning


# ================================================================ dialog

GROUPS = {TYPE_TIMING: 'gTiming', TYPE_V: 'gV', TYPE_MULTIV: 'gMV', TYPE_ROPE: 'gRope'}


def _dropdown(inputs, id_, label, items, selected):
    dd = inputs.addDropDownCommandInput(id_, label,
                                        adsk.core.DropDownStyles.TextListDropDownStyle)
    for name in items:
        dd.listItems.add(name, name == selected)
    return dd


def _fill_custom(inputs, spec, preset):
    """Copy a preset's values into the (hidden) custom fields."""
    for row in spec:
        id_, key, _, unit = row[:4]
        if key in preset:
            _set(inputs, id_, unit, preset[key])


def _on_preset(inputs, changed_id):
    """When a preset is picked, mirror it into the custom fields so that
    switching to Custom starts from it."""
    if changed_id == 'tProfile':
        name = _pick(inputs, 'tProfile')
        if name != CUSTOM:
            p = standards.lookup(standards.TIMING, name)
            _fill_custom(inputs, TIMING_CUSTOM, p)
            fam = {v: k for k, v in FAMILIES.items()}[p['family']]
            for item in inputs.itemById('cFamily').listItems:
                item.isSelected = item.name == fam
    elif changed_id == 'vSection':
        name = _pick(inputs, 'vSection')
        if name != CUSTOM:
            p = standards.lookup(standards.V_BELT, name)
            p['angle'] = profiles.v_angle(p, _get(inputs, 'vDd', 'mm'))
            _fill_custom(inputs, V_CUSTOM, p)
    elif changed_id == 'mSection':
        name = _pick(inputs, 'mSection')
        if name != CUSTOM:
            _fill_custom(inputs, MV_CUSTOM, standards.lookup(standards.MULTI_V, name))
    elif changed_id in ('rPreset', 'rD'):
        name = _pick(inputs, 'rPreset')
        if name != CUSTOM:
            _, p = _rope_params(inputs)
            _fill_custom(inputs, ROPE_CUSTOM, p)


def _update_visibility(inputs):
    ptype = _pick(inputs, 'ptype')
    for t, gid in GROUPS.items():
        inputs.itemById(gid).isVisible = t == ptype

    custom = _pick(inputs, 'tProfile') == CUSTOM
    inputs.itemById('cFamily').isVisible = custom
    fam = FAMILIES[_pick(inputs, 'cFamily')]
    for id_, _, _, _, fams in TIMING_CUSTOM:
        inputs.itemById(id_).isVisible = custom and fam in fams.split()

    fl_on = inputs.itemById('flBottom').value or inputs.itemById('flTop').value
    inputs.itemById('flOD').isVisible = fl_on
    inputs.itemById('flThk').isVisible = fl_on

    for sel_id, spec in (('vSection', V_CUSTOM), ('mSection', MV_CUSTOM),
                         ('rPreset', ROPE_CUSTOM)):
        on = _pick(inputs, sel_id) == CUSTOM
        for row in spec:
            inputs.itemById(row[0]).isVisible = on

    hub_on = inputs.itemById('hub').value
    inputs.itemById('hubDia').isVisible = hub_on
    inputs.itemById('hubHeight').isVisible = hub_on
    screws_on = _pick(inputs, 'screws') != SCREW_NONE
    for i in ('threadStd', 'threadSize', 'holeType', 'screwCount', 'screwSpacing'):
        inputs.itemById(i).isVisible = screws_on


def _fill_sizes(inputs):
    std = _pick(inputs, 'threadStd')
    items = inputs.itemById('threadSize').listItems
    items.clear()
    default = 'M4' if std == STD_METRIC else '#8-32'
    for e in THREADS[std]:
        items.add(e[0], e[0] == default)


class _CreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.setDialogInitialSize(420, 640)
            inputs = cmd.commandInputs

            _dropdown(inputs, 'ptype', 'Pulley type', TYPES, TYPE_TIMING)

            # ---- timing
            g = inputs.addGroupCommandInput('gTiming', 'Timing pulley').children
            _dropdown(g, 'tProfile', 'Profile',
                      [n for n, _ in standards.TIMING] + [CUSTOM], 'GT2 2 mm')
            _dropdown(g, 'cFamily', 'Groove shape', list(FAMILIES), FAM_ARC)
            for id_, key, label, unit, _ in TIMING_CUSTOM:
                g.addValueInput(id_, label, unit, _vi('0 ' + unit))
            g.addIntegerSpinnerCommandInput('teeth', 'Teeth', 8, 500, 1, 20)
            g.addValueInput('tWidth', 'Pulley width (excluding hub)', 'mm', _vi('7 mm'))
            g.addValueInput('tClear', 'Groove clearance', 'mm', _vi('0 mm'))
            g.addBoolValueInput('flBottom', 'Bottom flange', True, '', False)
            g.addBoolValueInput('flTop', 'Top flange', True, '', False)
            g.addValueInput('flOD', 'Flange OD = pulley OD +', 'mm', _vi('4 mm'))
            g.addValueInput('flThk', 'Flange thickness', 'mm', _vi('1 mm'))

            # ---- V-belt
            g = inputs.addGroupCommandInput('gV', 'V-belt pulley').children
            _dropdown(g, 'vSection', 'Belt section',
                      [n for n, _ in standards.V_BELT] + [CUSTOM], 'A / 4L')
            g.addValueInput('vDd', 'Datum diameter', 'mm', _vi('100 mm'))
            g.addIntegerSpinnerCommandInput('vGrooves', 'Grooves', 1, 12, 1, 1)
            for id_, key, label, unit in V_CUSTOM:
                g.addValueInput(id_, label, unit, _vi('0 ' + unit))

            # ---- multi-V
            g = inputs.addGroupCommandInput('gMV', 'Multi-V (ribbed) pulley').children
            _dropdown(g, 'mSection', 'Belt section',
                      [n for n, _ in standards.MULTI_V] + [CUSTOM], 'PJ')
            g.addValueInput('mDe', 'Effective diameter', 'mm', _vi('50 mm'))
            g.addIntegerSpinnerCommandInput('mRibs', 'Ribs', 1, 50, 1, 6)
            for id_, key, label, unit in MV_CUSTOM:
                g.addValueInput(id_, label, unit, _vi('0 ' + unit))

            # ---- rope / round belt
            g = inputs.addGroupCommandInput('gRope', 'Round belt / rope pulley').children
            _dropdown(g, 'rPreset', 'Groove', [n for n, _ in standards.ROPE] + [CUSTOM],
                      'Round belt')
            g.addValueInput('rD', 'Belt / rope diameter', 'mm', _vi('6 mm'))
            g.addValueInput('rPd', 'Pitch diameter (rope centre)', 'mm', _vi('50 mm'))
            for id_, key, label, unit in ROPE_CUSTOM:
                g.addValueInput(id_, label, unit, _vi('0 ' + unit))
            g.addValueInput('rRf', 'Edge fillet', 'mm', _vi('0.5 mm'))
            g.addValueInput('rWidth', 'Rim width (0 = auto)', 'mm', _vi('0 mm'))

            inputs.addTextBoxCommandInput('info', 'Result', '', 3, True)

            # ---- shared: bore, hub, set screws
            g = inputs.addGroupCommandInput('gShaft', 'Bore, hub and set screws').children
            g.addValueInput('bore', 'Bore diameter (0 = none)', 'mm', _vi('5 mm'))
            g.addBoolValueInput('hub', 'Hub (boss)', True, '', False)
            g.addValueInput('hubDia', 'Hub diameter', 'mm', _vi('16 mm'))
            g.addValueInput('hubHeight', 'Hub height', 'mm', _vi('7 mm'))
            _dropdown(g, 'screws', 'Set screws', [SCREW_NONE, SCREW_HUB, SCREW_RIM],
                      SCREW_NONE)
            _dropdown(g, 'threadStd', 'Thread standard', [STD_METRIC, STD_IMPERIAL],
                      STD_METRIC)
            g.addDropDownCommandInput('threadSize', 'Thread size',
                                      adsk.core.DropDownStyles.TextListDropDownStyle)
            _dropdown(g, 'holeType', 'Hole', [HOLE_PLAIN, HOLE_COSMETIC, HOLE_MODELED],
                      HOLE_PLAIN)
            g.addIntegerSpinnerCommandInput('screwCount', 'Number of screws', 1, 4, 1, 2)
            g.addValueInput('screwSpacing', 'Screw spacing', 'deg', _vi('90 deg'))

            global _dialog
            _dialog = _Inputs()
            _dialog.collect(inputs)
            _fill_sizes(_dialog)
            for cid in ('tProfile', 'vSection', 'mSection', 'rPreset'):
                _on_preset(_dialog, cid)
            _update_visibility(_dialog)
            _dialog.itemById('info').text = _info_text(_dialog)

            for event, handler in ((cmd.execute, _ExecuteHandler()),
                                   (cmd.executePreview, _PreviewHandler()),
                                   (cmd.inputChanged, _ChangedHandler()),
                                   (cmd.destroy, _DestroyHandler())):
                event.add(handler)
                _handlers.append(handler)
        except Exception:
            _ui.messageBox('Pulley Generator failed:\n{}'.format(traceback.format_exc()))


class _ChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            inputs = _dialog
            cid = args.input.id
            if cid == 'threadStd':
                _fill_sizes(inputs)
            _on_preset(inputs, cid)
            if cid == 'vDd' and _pick(inputs, 'vSection') != CUSTOM:
                _on_preset(inputs, 'vSection')
            _update_visibility(inputs)
            inputs.itemById('info').text = _info_text(inputs)
        except Exception:
            pass


class _PreviewHandler(adsk.core.CommandEventHandler):
    """Live preview. The preview is rolled back afterwards and OK rebuilds,
    so any thread warning still gets reported."""
    def notify(self, args):
        try:
            build_pulley(_dialog)
        except Exception:
            pass                   # the Result box already shows the error


class _ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            warning = build_pulley(_dialog)
            if warning:
                _ui.messageBox('Pulley Generator: ' + warning)
        except Exception as e:
            args.executeFailed = True
            _ui.messageBox('Pulley Generator: {}'.format(e))


class _DestroyHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        pass                       # add-in stays loaded


def run(context):
    global _app, _ui
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface
        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
        cmd_def = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, 'Pulley Generator',
            'Creates timing, V-belt, multi-V and round belt / rope pulleys.')
        on_created = _CreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)
        panel = _ui.allToolbarPanels.itemById(PANEL_ID)
        if panel and not panel.controls.itemById(CMD_ID):
            panel.controls.addCommand(cmd_def)
    except Exception:
        if _ui:
            _ui.messageBox('Pulley Generator failed:\n{}'.format(traceback.format_exc()))


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
