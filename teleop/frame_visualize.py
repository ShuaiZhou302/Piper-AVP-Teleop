#!/usr/bin/env python3
"""
3D frame visualizer for debugging end-effector pose conventions.

Usage examples:
  # Show only base frame and a target at xyz=(0.1,0,0.2), rpy(rad)=(0,0,0)
  python frame_visualize.py --xyz 0.1 0 0.2 --rpy 0 0 0

  # Same but treat input rpy as degrees
  python frame_visualize.py --xyz 0.1 0 0.2 --rpy 0 0 90 --deg

  # Compose with the IK ee-offset (Trans_z(0.1) * Rz(-90deg)), see difference
  # of two multiplication orders side-by-side
  python frame_visualize.py --xyz 0.06 0.0 0.21 --rpy 138 86.4 130.4 --deg \
      --offset_xyz 0 0 0.1 --offset_rpy 0 0 -90 --offset_deg

Conventions:
  - rpy uses tf 'sxyz' convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  - "main" frame = world rotation by rpy + translation by xyz
  - "main * offset (R then T)" = main_T * SE3(R_off, t_off): rotate first in
    main-local frame by R_off, then translate by t_off in main-local frame
    (this is what pinocchio's pin.SE3(R, t) does)
  - "main * offset (T then R)" = main_T * (Trans then Rot): translate first,
    then rotate -- different result if R_off and t_off don't commute
"""

import argparse
import math
import sys

import numpy as np
import matplotlib

# If user passes --save, switch to non-interactive backend BEFORE importing pyplot
if '--save' in sys.argv:
    matplotlib.use('Agg')

import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401, E402


def rpy_to_R(rpy):
    """sxyz: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(rpy)
    T[:3, 3] = np.array(xyz, dtype=float)
    return T


def compose_R_then_T(T_main, R_off, t_off):
    """Apply rotation first, then translation, both in main-local frame.
    Equivalent to T_main @ [[R_off, t_off],[0,1]] (pin.SE3 convention)."""
    T_off = np.eye(4)
    T_off[:3, :3] = R_off
    T_off[:3, 3] = t_off
    return T_main @ T_off


def compose_T_then_R(T_main, R_off, t_off):
    """Apply translation first, then rotation, both in main-local frame."""
    T_t = np.eye(4)
    T_t[:3, 3] = t_off
    T_r = np.eye(4)
    T_r[:3, :3] = R_off
    return T_main @ T_t @ T_r


def draw_frame(ax, T, label, axis_len=0.05, lw=2.0, alpha=1.0):
    o = T[:3, 3]
    R = T[:3, :3]
    colors = ['r', 'g', 'b']
    names = ['x', 'y', 'z']
    for i in range(3):
        d = R[:, i] * axis_len
        ax.quiver(
            o[0], o[1], o[2],
            d[0], d[1], d[2],
            color=colors[i], linewidth=lw, alpha=alpha, arrow_length_ratio=0.2,
        )
        ax.text(o[0] + d[0] * 1.15, o[1] + d[1] * 1.15, o[2] + d[2] * 1.15,
                names[i], color=colors[i], fontsize=8)
    if label:
        ax.text(o[0], o[1], o[2] + axis_len * 0.4, label,
                color='black', fontsize=9)


def set_equal_axes(ax, points, pad=0.05):
    pts = np.array(points)
    mins = pts.min(axis=0) - pad
    maxs = pts.max(axis=0) + pad
    span = (maxs - mins).max()
    mid = (mins + maxs) / 2.0
    ax.set_xlim(mid[0] - span / 2, mid[0] + span / 2)
    ax.set_ylim(mid[1] - span / 2, mid[1] + span / 2)
    ax.set_zlim(mid[2] - span / 2, mid[2] + span / 2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


def save_html(path, base_T, T_main, T_RT, T_TR, have_offset, axis_len):
    """Save an interactive 3D plotly figure as standalone HTML."""
    import plotly.graph_objects as go

    def add_frame_traces(fig, T, label, length, opacity=1.0):
        o = T[:3, 3]
        R = T[:3, :3]
        colors = ['red', 'green', 'blue']
        names = ['x', 'y', 'z']
        for i in range(3):
            d = R[:, i] * length
            tip = o + d
            fig.add_trace(go.Scatter3d(
                x=[o[0], tip[0]],
                y=[o[1], tip[1]],
                z=[o[2], tip[2]],
                mode='lines',
                line=dict(color=colors[i], width=8),
                name='%s.%s' % (label, names[i]),
                opacity=opacity,
                showlegend=(i == 0),
                legendgroup=label,
            ))
            # cone arrowhead
            fig.add_trace(go.Cone(
                x=[tip[0]], y=[tip[1]], z=[tip[2]],
                u=[d[0] * 0.3], v=[d[1] * 0.3], w=[d[2] * 0.3],
                showscale=False,
                colorscale=[[0, colors[i]], [1, colors[i]]],
                anchor='tip',
                sizemode='absolute',
                sizeref=length * 0.25,
                opacity=opacity,
                hoverinfo='skip',
                showlegend=False,
            ))
        # label at origin
        fig.add_trace(go.Scatter3d(
            x=[o[0]], y=[o[1]], z=[o[2]],
            mode='text',
            text=[label],
            textposition='top center',
            showlegend=False,
            hoverinfo='skip',
        ))

    fig = go.Figure()
    add_frame_traces(fig, base_T, 'base', axis_len * 1.4)
    add_frame_traces(fig, T_main, 'target', axis_len)
    if have_offset:
        add_frame_traces(fig, T_RT, 'main*(R-then-T) [SE3]', axis_len, opacity=0.85)
        add_frame_traces(fig, T_TR, 'main*(T-then-R)', axis_len, opacity=0.55)

    fig.update_layout(
        scene=dict(
            xaxis_title='X', yaxis_title='Y', zaxis_title='Z',
            aspectmode='data',
        ),
        title='Frame visualizer (red=X green=Y blue=Z) — drag to rotate, scroll to zoom',
        margin=dict(l=0, r=0, t=40, b=0),
    )
    fig.write_html(path, include_plotlyjs='cdn')


def parse_args():
    p = argparse.ArgumentParser(description='3D frame visualizer')
    p.add_argument('--xyz', type=float, nargs=3, required=True,
                   help='Target xyz in base frame (m)')
    p.add_argument('--rpy', type=float, nargs=3, required=True,
                   help='Target rpy (sxyz) in base frame, default rad')
    p.add_argument('--deg', action='store_true', help='Interpret --rpy as degrees')
    p.add_argument('--offset_xyz', type=float, nargs=3, default=None,
                   help='Optional offset translation in target-local frame')
    p.add_argument('--offset_rpy', type=float, nargs=3, default=None,
                   help='Optional offset rotation (sxyz) in target-local frame')
    p.add_argument('--offset_deg', action='store_true',
                   help='Interpret --offset_rpy as degrees')
    p.add_argument('--axis_len', type=float, default=0.05,
                   help='Axis arrow length for the drawn frames (m)')
    p.add_argument('--save', type=str, default=None,
                   help='Save figure to this path (PNG). If set, no interactive window.')
    p.add_argument('--multi_view', action='store_true',
                   help='Save 4 views (iso/top/front/side) in one PNG (used with --save)')
    p.add_argument('--html', type=str, default=None,
                   help='Save interactive HTML to this path (rotate/zoom in browser).')
    return p.parse_args()


def main():
    args = parse_args()

    rpy = np.array(args.rpy, dtype=float)
    if args.deg:
        rpy = np.deg2rad(rpy)
    T_main = make_T(args.xyz, rpy)

    have_offset = args.offset_xyz is not None and args.offset_rpy is not None
    if have_offset:
        off_rpy = np.array(args.offset_rpy, dtype=float)
        if args.offset_deg:
            off_rpy = np.deg2rad(off_rpy)
        R_off = rpy_to_R(off_rpy)
        t_off = np.array(args.offset_xyz, dtype=float)
        T_RT = compose_R_then_T(T_main, R_off, t_off)   # SE3 convention
        T_TR = compose_T_then_R(T_main, R_off, t_off)
    else:
        T_RT = None
        T_TR = None

    base = np.eye(4)
    points = [base[:3, 3], T_main[:3, 3]]
    if have_offset:
        points.append(T_RT[:3, 3])
        points.append(T_TR[:3, 3])

    # Interactive HTML output (browser, no display server needed)
    if args.html:
        save_html(args.html, base, T_main, T_RT, T_TR, have_offset, args.axis_len)
        print('Saved interactive HTML to:', args.html)
        print('Open it with: xdg-open', args.html, '  (or just double-click)')
        return

    def populate(ax):
        # World/base frame at origin (slightly larger axes for reference)
        draw_frame(ax, base, 'base', axis_len=args.axis_len * 1.4, lw=2.5)
        # Main target frame
        draw_frame(ax, T_main, 'target (xyz, rpy)', axis_len=args.axis_len, lw=2.0)
        if have_offset:
            draw_frame(ax, T_RT, 'main * (R then T)  [SE3]', axis_len=args.axis_len, lw=2.0, alpha=0.8)
            draw_frame(ax, T_TR, 'main * (T then R)', axis_len=args.axis_len, lw=2.0, alpha=0.5)
        set_equal_axes(ax, points, pad=max(0.08, args.axis_len * 2))

    if args.save and args.multi_view:
        fig = plt.figure(figsize=(12, 10))
        views = [
            ('iso',  30, -60),
            ('top',  90,  -90),
            ('front', 0, -90),
            ('side',  0,   0),
        ]
        for i, (name, elev, azim) in enumerate(views, 1):
            ax = fig.add_subplot(2, 2, i, projection='3d')
            populate(ax)
            ax.view_init(elev=elev, azim=azim)
            ax.set_title('%s view (red=X green=Y blue=Z)' % name)
        plt.tight_layout()
    else:
        fig = plt.figure(figsize=(8, 7))
        ax = fig.add_subplot(111, projection='3d')
        populate(ax)
        ax.set_title('Frame visualizer (red=X green=Y blue=Z)')
        plt.tight_layout()

    # Print numeric summary so user can compare with /puppet/end_pose
    np.set_printoptions(precision=6, suppress=True)
    print('--- Main target ---')
    print('xyz:', T_main[:3, 3])
    print('R  :')
    print(T_main[:3, :3])
    if have_offset:
        print('--- main * (R then T)  [== pinocchio SE3 convention] ---')
        print('xyz:', T_RT[:3, 3])
        print('R  :')
        print(T_RT[:3, :3])
        print('--- main * (T then R) ---')
        print('xyz:', T_TR[:3, 3])
        print('R  :')
        print(T_TR[:3, :3])

    if args.save:
        fig.savefig(args.save, dpi=120, bbox_inches='tight')
        print('Saved figure to:', args.save)
    else:
        plt.show()


if __name__ == '__main__':
    main()
