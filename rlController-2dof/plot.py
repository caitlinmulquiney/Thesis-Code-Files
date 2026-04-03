import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
from dataclasses import dataclass
from typing import Optional


@dataclass
class ModelData:
    t: np.ndarray        # (N,)
    F: np.ndarray        # (N,12)  [eta(0:6) | nu(6:12)]
    u: np.ndarray        # (N,7)
    label: str           # legend label e.g. "LQR" or "PPO"


def plotComparison(models: list[ModelData], figsize=(18, 22)):
    """
    Plot up to 4 models on shared axes — one subplot row per signal.

    Layout:
        Rows 0–3  : position states   z, φ, θ, ψ
        Rows 4–7  : velocity states   w, p, q, r
        Rows 8–14 : control inputs    sail β, sail twist, L foil,
                                      stbd T, stbd rudder, port T, port rudder
    """
    assert 1 <= len(models) <= 4, "Pass between 1 and 4 ModelData objects."

    pos_signals   = [(2, r'$z$',        r'$m$'),
                     (3, r'$\phi$',     r'$rad$'),
                     (4, r'$\theta$',   r'$rad$'),
                     (5, r'$\psi$',     r'$rad$')]

    vel_signals   = [(2, r'$w$',   r'$m/s$'),
                     (3, r'$p$',   r'$rad/s$'),
                     (4, r'$q$',   r'$rad/s$'),
                     (5, r'$r$',   r'$rad/s$')]

    ctrl_labels   = ['Sail $\\beta$', 'Sail twist', 'L foil',
                     'Stbd T foil', 'Stbd rudder',
                     'Port T foil', 'Port rudder']

    n_rows = len(pos_signals) + len(vel_signals) + len(ctrl_labels)
    fig, axes = plt.subplots(n_rows, 1, figsize=figsize, sharex=False)
    fig.subplots_adjust(hspace=0.45)

    colors = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e']

    def _plot_row(ax, t, y, model_idx, model_label, ylabel, section_title=None):
        ax.plot(t, y, color=colors[model_idx], label=model_label, linewidth=1.2)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.grid(True, linewidth=0.4, alpha=0.6)
        if section_title:
            ax.set_title(section_title, fontsize=10, fontweight='bold', pad=4)

    # ── Position rows ──────────────────────────────────────────
    for row, (col, name, unit) in enumerate(pos_signals):
        ax = axes[row]
        for mi, m in enumerate(models):
            _plot_row(ax, m.t, m.F[:, col], mi, m.label, f'{name}  $({unit})$',
                      section_title='Position' if row == 0 else None)
        if row == len(pos_signals) - 1:
            ax.set_xlabel(r'Time $(s)$', fontsize=9)

    # ── Velocity rows ──────────────────────────────────────────
    offset = len(pos_signals)
    for row, (col, name, unit) in enumerate(vel_signals):
        ax = axes[offset + row]
        for mi, m in enumerate(models):
            _plot_row(ax, m.t, m.F[:, 6 + col], mi, m.label, f'{name}  $({unit})$',
                      section_title='Velocity' if row == 0 else None)
        if row == len(vel_signals) - 1:
            ax.set_xlabel(r'Time $(s)$', fontsize=9)

    # ── Control input rows ─────────────────────────────────────
    offset += len(vel_signals)
    for row, ctrl_label in enumerate(ctrl_labels):
        ax = axes[offset + row]
        for mi, m in enumerate(models):
            _plot_row(ax, m.u[:,0], m.u[:, row+1], mi, m.label, r'$rad$',
                      section_title='Control inputs' if row == 0 else None)
        ax.set_title(ctrl_label, fontsize=9, pad=3)
        if row == len(ctrl_labels) - 1:
            ax.set_xlabel(r'Time $(s)$', fontsize=9)

    # ── Single shared legend at the top ───────────────────────
    handles = [plt.Line2D([0], [0], color=colors[i], linewidth=2, label=m.label)
               for i, m in enumerate(models)]
    fig.legend(handles=handles, loc='upper center', ncol=len(models),
               fontsize=10, frameon=True, bbox_to_anchor=(0.5, 0.995))

    plt.show()
    return fig

def load_ppo_episode(path, label):
    d = np.load(path)
    t       = d['t']              # (N,)
    states  = d['states']         # (N,10)  z,roll,pitch,yaw,vx,vy,vz,p,q,r
    actions = d['actions']        # (N,7)

    # Build F to match your (N,12) convention: [eta(0:6) | nu(6:12)]
    # eta: pad x=0, y=0, then z, roll, pitch, yaw
    # nu:  pad u=vx, v=vy, then w, p, q, r
    N  = len(t)
    F  = np.zeros((N, 12))
    F[:, 2] = states[:, 0]   # z
    F[:, 3] = states[:, 1]   # roll
    F[:, 4] = states[:, 2]   # pitch
    F[:, 5] = states[:, 3]   # yaw
    F[:, 8] = states[:, 4]   # vx  (surge — index 6 would be u)
    F[:, 9] = states[:, 5]   # vy
    F[:,10] = states[:, 6]   # w
    F[:, 9] = states[:, 7]   # p
    F[:,10] = states[:, 8]   # q
    F[:,11] = states[:, 9]   # r

    return ModelData(t=t, F=F, u=actions, label=label)

# ── Load data ──────────────────────────────────────────────────
def load_model(path, label):
    mat = loadmat(path)
    return ModelData(
        t     = mat['t'].squeeze(),
        F     = mat['F'],
        u     = mat['uMatrix'],
        label = label
    )

lqr      = load_model('../AdaptationforFoilingCatamaran/lqr.mat',     'LQR')
lqr_gain = load_model('../AdaptationforFoilingCatamaran/lqrGain.mat', 'LQR gain scheduled')
ppo_base_base = load_ppo_episode("ppo_base.npz", "PPO base")

plotComparison([lqr, lqr_gain, ppo_base_base])
