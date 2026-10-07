#!/usr/bin/env python3
"""Clean up the Fusion 360 export (fusion360descriptor) into vector_cad.urdf.xacro.

  cad_urdf.py                      # vector_cad_raw.urdf.xacro -> vector_cad.urdf.xacro
  cad_urdf.py --mass 3.0 --no-symmetry

After every export from Fusion, save it as urdf/vector_cad_raw.urdf.xacro (meshes into
meshes/) and run this. It:

- moves each revolute joint onto its servo's output spline (the exporter puts some of them
  12 mm off, on the servo body instead of the shaft); the meshes don't move, they are
  exported in assembly coordinates
- makes the six legs symmetric: each leg (servo mount, coxa servo and everything after it)
  is shifted so the hips sit on a mirror-symmetric pattern at one height
- renames the joints to vector.urdf.xacro's (L1_coxa, ..., R3_tibia) with the same signs,
  zeros and limits (legs.yaml), so the gait's joint_states drive this model too: coxa about
  +z with 0 pointing out along the mounting yaw, femur and tibia about -y so a positive angle
  lifts, femur 0 horizontal, tibia 0 in line with the femur
- adds base_link (centred between the hips, at femur-axis height, like vector.urdf.xacro)
  and an <leg>_foot frame at each tibia tip
- scales every mass and inertia so the robot weighs --mass kg (Fusion has no materials yet)

It prints the geometry it finds, in legs.yaml's terms.
"""
import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.join(HERE, '..', 'ros2_ws', 'src', 'vector_description')
XACRO_NS = 'http://www.ros.org/wiki/xacro'
LEGS = ('L1', 'L2', 'L3', 'R1', 'R2', 'R3')
# mounting yaw of each leg (hip to foot, seen from above), as in legs.yaml
YAW = {'L1': math.pi / 4, 'L2': math.pi / 2, 'L3': 3 * math.pi / 4,
       'R1': -math.pi / 4, 'R2': -math.pi / 2, 'R3': -3 * math.pi / 4}


# ---- transforms ----

def rot(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    m = np.eye(4)
    m[:3, :3] = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
    return m


def from_origin(o):
    m = np.eye(4)
    if o is None:
        return m
    x, y, z = (float(v) for v in o.get('xyz', '0 0 0').split())
    r, p, w = (float(v) for v in o.get('rpy', '0 0 0').split())
    m = rot((0, 0, 1), w) @ rot((0, 1, 0), p) @ rot((1, 0, 0), r)
    m[:3, 3] = (x, y, z)
    return m


def set_origin(parent, m):
    o = parent.find('origin')
    if o is None:
        o = ET.SubElement(parent, 'origin')
        parent.remove(o)
        parent.insert(0, o)
    R = m[:3, :3]
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        r, w = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
    else:  # pitch +-90 deg: put everything in roll
        r, w = math.atan2(-R[1, 2], R[1, 1]), 0.0
    f = lambda v: ' '.join(f'{x:.9g}' for x in v)
    o.set('xyz', f(m[:3, 3]))
    o.set('rpy', f((r, p, w)))


def read_stl(path):
    d = open(path, 'rb').read()
    n = int.from_bytes(d[80:84], 'little')
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].reshape(-1, 3).astype(float)


class Model:
    def __init__(self, path):
        ET.register_namespace('xacro', XACRO_NS)
        self.tree = ET.parse(path)
        self.root = self.tree.getroot()
        self.links = {l.get('name'): l for l in self.root.findall('link')}
        self.meshdir = os.path.join(DESC, 'meshes')
        self._mesh = {}
        self.reindex()

    def reindex(self):
        self.joints = {j.get('name'): j for j in self.root.findall('joint')}
        self.by_child = {j.find('child').get('link'): j for j in self.joints.values()}
        self.kids = {}
        for j in self.joints.values():
            self.kids.setdefault(j.find('parent').get('link'), []).append(j)
        self._w = {}

    def world(self, link):
        """Link frame in the root frame, all joints at zero."""
        if link not in self._w:
            j = self.by_child.get(link)
            self._w[link] = np.eye(4) if j is None else \
                self.world(j.find('parent').get('link')) @ from_origin(j.find('origin'))
        return self._w[link]

    def subtree(self, link):
        out = [link]
        for j in self.kids.get(link, []):
            out += self.subtree(j.find('child').get('link'))
        return out

    def axis(self, j):
        return np.array([float(v) for v in j.find('axis').get('xyz').split()])

    def mesh_points(self, link):
        """All visual mesh vertices of a link, in the root frame."""
        pts = []
        for v in self.links[link].findall('visual'):
            m = v.find('geometry/mesh')
            if m is None:
                continue
            f = os.path.join(self.meshdir, os.path.basename(m.get('filename')))
            if f not in self._mesh:
                self._mesh[f] = read_stl(f)
            s = np.array([float(x) for x in m.get('scale', '1 1 1').split()])
            M = self.world(link) @ from_origin(v.find('origin'))
            pts.append((M[:3, :3] @ (self._mesh[f] * s).T).T + M[:3, 3])
        return np.vstack(pts) if pts else np.zeros((0, 3))

    def move_joint_frame(self, j, new_origin):
        """Give joint j a new origin and re-express its child's contents so nothing moves."""
        child = j.find('child').get('link')
        D = np.linalg.inv(new_origin) @ from_origin(j.find('origin'))
        set_origin(j, new_origin)
        link = self.links[child]
        for tag in ('visual', 'collision', 'inertial'):
            for e in link.findall(tag):
                set_origin(e, D @ from_origin(e.find('origin')))
        for k in self.kids.get(child, []):
            set_origin(k, D @ from_origin(k.find('origin')))
        self._w = {}


def hull2d(p):
    """Indices of the convex hull of 2D points (monotone chain)."""
    idx = sorted(range(len(p)), key=lambda i: (p[i][0], p[i][1]))
    cross = lambda o, a, b: (p[a][0] - p[o][0]) * (p[b][1] - p[o][1]) - (p[a][1] - p[o][1]) * (p[b][0] - p[o][0])
    lo, hi = [], []
    for i in idx:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], i) <= 0:
            lo.pop()
        lo.append(i)
    for i in reversed(idx):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], i) <= 0:
            hi.pop()
        hi.append(i)
    return lo[:-1] + hi[:-1]


def simplify2d(P, n):
    """A convex polygon cut down to n corners, dropping the corner that adds the least area."""
    P = [tuple(p) for p in P]
    area = lambda a, b, c: abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))
    while len(P) > n:
        k = min(range(len(P)), key=lambda i: area(P[i - 1], P[i], P[(i + 1) % len(P)]))
        del P[k]
    return np.array(P)


def thin3d(P, n):
    """n of the points, spread out (farthest-point sampling)."""
    if len(P) <= n:
        return P
    pick = [int(np.argmax(np.linalg.norm(P - P.mean(0), axis=1)))]
    d = np.linalg.norm(P - P[pick[0]], axis=1)
    while len(pick) < n:
        pick.append(int(np.argmax(d)))
        d = np.minimum(d, np.linalg.norm(P - P[pick[-1]], axis=1))
    return P[pick]


def write_silhouette(m, legs, report, path, unit=5e-4, min_size=0.004, corners=16):
    """Every visible part as a convex outline, for a top-down drawing.

    Body parts: a 2D polygon in base_link. Leg parts: points in their segment's frame (the
    joint conventions of vector.urdf.xacro, joints at zero), chosen so their 2D hull is the
    part's outline seen from above at any coxa, femur or tibia angle. Lengths in `unit` m.
    """
    def parts(link):
        for v in m.links[link].findall('visual'):
            mesh = v.find('geometry/mesh')
            if mesh is None:
                continue
            f = os.path.join(m.meshdir, os.path.basename(mesh.get('filename')))
            if f not in m._mesh:
                m._mesh[f] = read_stl(f)
            s = np.array([float(x) for x in mesh.get('scale', '1 1 1').split()])
            M = m.world(link) @ from_origin(v.find('origin'))
            P = (M[:3, :3] @ (m._mesh[f] * s).T).T + M[:3, 3]
            if np.ptp(P, axis=0).max() >= min_size:  # skip screws and stickers
                yield P

    q = lambda P: np.round(np.asarray(P) / unit).astype(int).tolist()
    in_leg = set()
    out = {'unit': unit, 'legs': {}, 'body': [], 'parts': {}}
    for n, (c, f, t) in legs.items():
        r = report[n]
        sub = {k: set(m.subtree(j.find('child').get('link'))) for k, j in (('coxa', c), ('femur', f), ('tibia', t))}
        seg = {'coxa': sub['coxa'] - sub['femur'], 'femur': sub['femur'] - sub['tibia'], 'tibia': sub['tibia']}
        in_leg |= sub['coxa']
        yaw = YAW[n]
        R = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        hip = np.array([r['x'], r['y'], 0.0])
        origin = {'coxa': hip, 'femur': hip + R @ [r['coxa'], 0, 0], 'tibia': hip + R @ [r['coxa'] + r['femur'], 0, 0]}
        out['legs'][n] = {'x': round(r['x'], 4), 'y': round(r['y'], 4), 'yaw': round(yaw, 6),
                          'coxa': round(r['coxa'], 4), 'femur': round(r['femur'], 4), 'tibia': round(r['tibia'], 4)}
        out['parts'][n] = {}
        for k in ('coxa', 'femur', 'tibia'):
            out['parts'][n][k] = []
            for link in sorted(seg[k]):
                for P in parts(link):
                    L = (P - origin[k]) @ R  # into the segment frame
                    if k == 'coxa':  # only turns about z: the outline from above is enough
                        out2 = simplify2d(L[hull2d(L[:, :2].tolist()), :2], corners)
                        pts = np.c_[out2, np.full(len(out2), L[:, 2].max())]
                    else:  # pitches about y: the corners of its 3D hull outline it at any angle
                        pts = thin3d(L[ConvexHull(L).vertices], 2 * corners)
                    out['parts'][n][k].append(q(pts))
    for link in m.links:
        if link in in_leg:
            continue
        for P in parts(link):
            out['body'].append({'z': q([P[:, 2].max()])[0], 'p': q(simplify2d(P[hull2d(P[:, :2].tolist()), :2], corners))})
    out['body'].sort(key=lambda b: b['z'])
    with open(path, 'w') as fh:
        json.dump(out, fh, separators=(',', ':'))
    n = len(out['body']) + sum(len(v) for leg in out['parts'].values() for v in leg.values())
    print(f'wrote {os.path.relpath(path)}: {n} parts, {os.path.getsize(path) // 1024} kB')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--input', default=os.path.join(DESC, 'urdf', 'vector_cad_raw.urdf.xacro'))
    ap.add_argument('--output', default=os.path.join(DESC, 'urdf', 'vector_cad.urdf.xacro'))
    ap.add_argument('--legs', default=os.path.join(DESC, 'config', 'legs.yaml'))
    ap.add_argument('--mass', type=float, default=3.0, help='total mass, kg')
    ap.add_argument('--no-symmetry', action='store_true', help='leave the legs where Fusion put them')
    ap.add_argument('--silhouette', default=os.path.join(DESC, 'config', 'silhouette.json'),
                    help="top-down outline of every part, for the operator page's robot view")
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.legs))
    m = Model(args.input)

    revs = [j for j in m.joints.values() if j.get('type') == 'revolute']
    if len(revs) != 18:
        sys.exit(f'expected 18 revolute joints, found {len(revs)}')

    # 1. every revolute joint onto the output spline of the nearest servo
    splines = []
    for name in m.links:
        if not name.startswith('ServoMG996R'):
            continue
        for v in m.links[name].findall('visual'):
            if v.find('geometry/mesh').get('filename').endswith('__Body.stl'):  # Fusion's name for the spline
                M = m.world(name) @ from_origin(v.find('origin'))
                f = os.path.join(m.meshdir, os.path.basename(v.find('geometry/mesh').get('filename')))
                P = read_stl(f) * 0.001
                splines.append(((M[:3, :3] @ P.T).T + M[:3, 3]).mean(0))
    if len(splines) != 18:
        sys.exit(f'expected 18 servo splines (ServoMG996R*__Body.stl), found {len(splines)}')
    moved = 0
    for j in sorted(revs, key=lambda j: len(m.subtree(j.find('child').get('link'))), reverse=True):
        W = m.world(j.find('child').get('link'))
        p, a = W[:3, 3], W[:3, :3] @ m.axis(j)
        a /= np.linalg.norm(a)
        c = min(splines, key=lambda c: np.linalg.norm(np.cross(c - p, a)))
        off = (c - p) - np.dot(c - p, a) * a  # perpendicular offset from the axis to the spline
        if np.linalg.norm(off) > 1e-4:
            Wn = W.copy()
            Wn[:3, 3] += off
            parent = m.world(j.find('parent').get('link'))
            m.move_joint_frame(j, np.linalg.inv(parent) @ Wn)
            moved += 1
    print(f'{moved} of 18 joints moved onto their servo spline')

    # 2. name the legs: coxa = revolute with no revolute above it, then femur, tibia
    def first_rev_below(link):
        for k in m.kids.get(link, []):
            if k.get('type') == 'revolute':
                return k
            r = first_rev_below(k.find('child').get('link'))
            if r is not None:
                return r
        return None
    rev_children = {j.find('child').get('link') for j in revs}
    def has_rev_above(j):
        link = j.find('parent').get('link')
        while link in m.by_child:
            if link in rev_children:
                return True
            link = m.by_child[link].find('parent').get('link')
        return False
    root = next(n for n in m.links if n not in m.by_child)
    body = m.by_child  # noqa
    coxas = [j for j in revs if not has_rev_above(j)]
    legs = {}
    for c in coxas:
        p = m.world(c.find('child').get('link'))[:3, 3]
        name = ('L' if p[1] > 0 else 'R') + ('1' if p[0] > 0.04 else '3' if p[0] < -0.04 else '2')
        f = first_rev_below(c.find('child').get('link'))
        t = first_rev_below(f.find('child').get('link'))
        legs[name] = (c, f, t)
    if sorted(legs) != sorted(LEGS):
        sys.exit(f'could not tell the legs apart: {sorted(legs)}')

    # the links that belong to each leg: everything fixed to the body whose subtree holds the
    # leg's coxa, plus body parts (servo mount, coxa servo bracket) nearest that hip
    body_root = next(j.find('child').get('link') for j in m.kids[root])
    hip = {n: m.world(c.find('child').get('link'))[:3, 3] for n, (c, f, t) in legs.items()}
    roots = {n: [] for n in LEGS}
    for k in m.kids.get(body_root, []):
        sub = m.subtree(k.find('child').get('link'))
        owner = [n for n, (c, f, t) in legs.items() if c.find('child').get('link') in sub]
        if owner:
            roots[owner[0]].append(k)
            continue
        pts = np.vstack([m.mesh_points(l) for l in sub])
        if not len(pts):
            continue
        cen = pts.mean(0)
        n, d = min(((n, np.linalg.norm((cen - h)[:2])) for n, h in hip.items()), key=lambda x: x[1])
        if d < 0.06:
            roots[n].append(k)

    # 3. symmetric hips: same |x| front/rear, same |y| per row, one height
    def leg_geometry(n):
        c, f, t = legs[n]
        Wc, Wf, Wt = (m.world(j.find('child').get('link')) for j in (c, f, t))
        h, up = Wc[:3, 3], Wc[:3, :3] @ m.axis(c)
        up = up / np.linalg.norm(up) * np.sign(up[2])
        fa = Wf[:3, :3] @ m.axis(f)
        out = np.cross(fa, up)
        out = out - np.dot(out, up) * up
        out /= np.linalg.norm(out)
        if np.dot(out[:2], h[:2]) < 0:
            out = -out
        side = np.cross(up, out)
        def plane(p, a):  # where the joint's axis crosses the leg plane
            a = a / np.linalg.norm(a)
            return p - np.dot(p - h, side) / np.dot(a, side) * a
        pf = plane(Wf[:3, 3], fa)
        pt = plane(Wt[:3, 3], Wt[:3, :3] @ m.axis(t))
        pts = np.vstack([m.mesh_points(l) for l in m.subtree(t.find('child').get('link'))])
        rel = pts - pt
        foot = pts[np.argmax(np.linalg.norm(rel - np.outer(rel @ side, side), axis=1))]
        foot = foot - np.dot(foot - h, side) * side
        return dict(hip=h, up=up, out=out, side=side, femur=pf, tibia=pt, foot=foot)

    geo = {n: leg_geometry(n) for n in LEGS}
    if not args.no_symmetry:
        fz = np.mean([g['femur'][2] for g in geo.values()])
        corner = [geo[n]['hip'] for n in ('L1', 'L3', 'R1', 'R3')]
        cx, cy = np.mean([abs(p[0]) for p in corner]), np.mean([abs(p[1]) for p in corner])
        my = np.mean([abs(geo[n]['hip'][1]) for n in ('L2', 'R2')])
        for n in LEGS:
            sx = 0.0 if n[1] == '2' else cx * (1 if n[1] == '1' else -1)
            sy = (my if n[1] == '2' else cy) * (1 if n[0] == 'L' else -1)
            target = np.array([sx, sy, geo[n]['hip'][2] + fz - geo[n]['femur'][2]])
            delta = target - geo[n]['hip']
            for k in roots[n]:
                o = from_origin(k.find('origin'))
                pw = m.world(k.find('parent').get('link'))
                shift = np.eye(4)
                shift[:3, 3] = np.linalg.inv(pw)[:3, :3] @ delta
                set_origin(k, shift @ o)
            m._w = {}
            print(f'{n}: shifted {np.linalg.norm(delta) * 1000:.2f} mm')
        geo = {n: leg_geometry(n) for n in LEGS}

    # 4. base_link: between the hips, at femur-axis height, x forward, z up (the export's frame)
    centre = np.mean([g['hip'] for g in geo.values()], axis=0)
    centre[2] = np.mean([g['femur'][2] for g in geo.values()])
    for n, g in geo.items():
        g['hip_b'] = g['hip'] - centre

    # 5. joints: names, signs, zeros and limits of vector.urdf.xacro
    lim = cfg['limits']
    report = {}
    for n, (c, f, t) in legs.items():
        g = geo[n]
        vf, vt = g['tibia'] - g['femur'], g['foot'] - g['tibia']
        ang = lambda v: math.atan2(np.dot(v, g['up']), np.dot(v, g['out']))
        yaw = math.atan2(g['out'][1], g['out'][0])
        zero = {'coxa': (yaw - YAW[n] + math.pi) % (2 * math.pi) - math.pi,
                'femur': ang(vf), 'tibia': ang(vt) - ang(vf)}
        want = {'coxa': g['up'], 'femur': -g['side'], 'tibia': -g['side']}  # -y of the coxa frame
        # the tibia link frame in the Fusion pose; folding the zeros in below keeps it there at
        # joint = zero, so the foot is placed against this frame
        tl = t.find('child').get('link')
        Wt = m.world(tl)
        for kind, j in (('coxa', c), ('femur', f), ('tibia', t)):
            W = m.world(j.find('child').get('link'))
            a = m.axis(j)
            s = 1.0 if np.dot(W[:3, :3] @ a, want[kind]) > 0 else -1.0
            a = s * a
            j.find('axis').set('xyz', ' '.join(f'{x:.9g}' for x in a))
            # joint value now = s * fusion value + zero; fold the zero into the origin
            o = from_origin(j.find('origin')) @ rot(a, -zero[kind])
            set_origin(j, o)
            j.set('name', f'{n}_{kind}')
            l = j.find('limit')
            l.set('lower', f"{lim[kind][0]}")
            l.set('upper', f"{lim[kind][1]}")
            l.set('effort', f"{lim['effort']}")
            l.set('velocity', f"{lim['velocity']}")
        m._w = {}
        # foot frame on the tibia link, x along the tibia
        foot = ET.SubElement(m.root, 'link', name=f'{n}_foot')
        fj = ET.SubElement(m.root, 'joint', name=f'{n}_foot_joint', type='fixed')
        ET.SubElement(fj, 'parent', link=tl)
        ET.SubElement(fj, 'child', link=f'{n}_foot')
        F = np.eye(4)
        F[:3, 0], F[:3, 1], F[:3, 2] = vt / np.linalg.norm(vt), g['side'], np.cross(vt / np.linalg.norm(vt), g['side'])
        F[:3, 3] = g['foot']
        ET.SubElement(fj, 'origin')
        set_origin(fj, np.linalg.inv(Wt) @ F)
        report[n] = dict(x=g['hip_b'][0], y=g['hip_b'][1], yaw=yaw,
                         coxa=np.linalg.norm((g['femur'] - g['hip'])[:2]),
                         femur=np.linalg.norm(vf), tibia=np.linalg.norm(vt), zero=zero)

    # base_link replaces the exporter's dummy root
    old_root = m.links[root]
    rj = m.kids[root][0]
    old_root.set('name', 'base_link')
    rj.find('parent').set('link', 'base_link')
    rj.set('name', 'base_link_cad_joint')
    shift = np.eye(4)
    shift[:3, 3] = -centre
    set_origin(rj, shift @ from_origin(rj.find('origin')))

    # 6. masses
    links = [l for l in m.root.findall('link') if l.find('inertial/mass') is not None]
    total = sum(float(l.find('inertial/mass').get('value')) for l in links)
    k = args.mass / total
    for l in links:
        ms = l.find('inertial/mass')
        ms.set('value', f"{float(ms.get('value')) * k:.9g}")
        it = l.find('inertial/inertia')
        for a in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
            it.set(a, f"{float(it.get(a)) * k:.9g}")

    m.reindex()
    m.links = {l.get('name'): l for l in m.root.findall('link')}
    write_silhouette(m, legs, report, args.silhouette)
    ET.indent(m.tree, space='   ')
    with open(args.output, 'w') as fh:
        fh.write("<?xml version='1.0' encoding='utf-8'?>\n")
        fh.write('<!-- Generated by software/tools/cad_urdf.py from vector_cad_raw.urdf.xacro. Do not edit. -->\n')
        fh.write(ET.tostring(m.root, encoding='unicode'))
        fh.write('\n')

    print(f'mass {total:.2f} kg in the export, scaled to {args.mass:.2f} kg')
    print('\nlegs.yaml terms (base_link at femur-axis height, between the hips):')
    print(' leg     x        y      yaw   coxa   femur  tibia   zero offsets (coxa/femur/tibia, deg)')
    for n in LEGS:
        r = report[n]
        z = r['zero']
        print(f" {n}  {r['x']:+.4f}  {r['y']:+.4f}  {math.degrees(r['yaw']):+6.1f}  {r['coxa']:.4f} {r['femur']:.4f} "
              f"{r['tibia']:.4f}   {math.degrees(z['coxa']):+.1f} / {math.degrees(z['femur']):+.1f} / {math.degrees(z['tibia']):+.1f}")
    print(f'\nwrote {os.path.relpath(args.output)}')


if __name__ == '__main__':
    main()
