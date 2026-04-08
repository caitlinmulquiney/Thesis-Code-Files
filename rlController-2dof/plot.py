import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
from dataclasses import dataclass
from typing import Optional
import matplotlib.ticker as ticker


@dataclass
@dataclass
class ModelData:
    label:     str
    t:         np.ndarray
    F:         np.ndarray
    u:         np.ndarray
    # vmg_log:   np.ndarray   # shape (N, 2): [t, vmg]
    # vmg_e_log: np.ndarray   # shape (N, 2): [t, vmg_e]

def plotVmgComparison(models: list[ModelData], figsize=(18, 10)):
    """
    Plot VMG and VMG error for up to 4 models on shared axes.
    """
    assert 1 <= len(models) <= 4, "Pass between 1 and 4 ModelData objects."

    fig, axes = plt.subplots(1, 1, figsize=figsize, sharex=True)
    fig.subplots_adjust(hspace=0.35)

    colors = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e']

    # ── VMG ────────────────────────────────────────────────────
    ax = axes
    for mi, m in enumerate(models):
        t   = m.vmg_log[:, 0]
        vmg = m.vmg_log[:, 1]
        ax.plot(t, vmg, color=colors[mi], label=m.label, linewidth=1.2)

    ax.set_ylabel(r'VMG $(knots)$', fontsize=9)
    ax.set_title('Velocity Made Good', fontsize=10, fontweight='bold', pad=4)
    ax.grid(True, linewidth=0.4, alpha=0.6)

    # ── Target VMG ──────────────────────────────────────────────
    ax = axes
    twa_deg = np.arange(40, 151)
    t_target = (twa_deg - 40) * 2
    twa = np.deg2rad(twa_deg)
    target_vmg = (
        -0.00000272 * np.rad2deg(np.pi - np.abs(twa))**4
        + 0.001025 * np.rad2deg(np.pi - np.abs(twa))**3
        - 0.12778 * np.rad2deg(np.pi - np.abs(twa))**2
        + 6.012 * np.rad2deg(np.pi - np.abs(twa))
        - 74
    )
    ax.plot(t_target, target_vmg, color='black', linewidth=2, label='Target VMG')

    # ax.set_ylabel(r'Target VMG $(knots)$', fontsize=9)
    # ax.set_title('Target VMG', fontsize=10, fontweight='bold', pad=4)
    # ax.set_xlabel(r'Time $(s)$', fontsize=9)
    # ax.grid(True, linewidth=0.4, alpha=0.6)

    # ── Legend ─────────────────────────────────────────────────
    handles = [plt.Line2D([0], [0], color=colors[i], linewidth=2, label=m.label)
               for i, m in enumerate(models)]
    ax.legend(loc='upper center', ncol=len(models)+1)
    def sec_to_deg(x, pos):
        return x / 2 + 40 # 2 seconds = 1 degree

    ax.xaxis.set_major_formatter(ticker.FuncFormatter(sec_to_deg))
    ax.set_xlabel('TWA (deg)')

    plt.show()
    return fig

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

    # pos_signals   = [(2, r'$z$',        r'$m$'),
    #                  (3, r'$\phi$',     r'$rad$'),
    #                  (4, r'$\theta$',   r'$rad$'),
    #                  (5, r'$\psi$',     r'$rad$')]

    # vel_signals   = [(2, r'$w$',   r'$m/s$'),
    #                  (3, r'$p$',   r'$rad/s$'),
    #                  (4, r'$q$',   r'$rad/s$'),
    #                  (5, r'$r$',   r'$rad/s$')]

    ctrl_labels   = ['Sail $\\beta$', 'Sail twist', 'L foil',
                     'Stbd T foil', 'Stbd rudder',
                     'Port T foil', 'Port rudder']

    n_rows = len(ctrl_labels) #len(pos_signals) + len(vel_signals) + 
    fig, axes = plt.subplots(n_rows, 1, figsize=figsize, sharex=False)
    fig.subplots_adjust(hspace=0.45)

    colors = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e']

    def _plot_row(ax, t, y, model_idx, model_label, ylabel, section_title=None):
        ax.plot(t, y, color=colors[model_idx], label=model_label, linewidth=1.2)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.grid(True, linewidth=0.4, alpha=0.6)
        if section_title:
            ax.set_title(section_title, fontsize=10, fontweight='bold', pad=4)

    # # ── Position rows ──────────────────────────────────────────
    # for row, (col, name, unit) in enumerate(pos_signals):
    #     ax = axes[row]
    #     ax.set_xlim(0, 150)
    #     for mi, m in enumerate(models):
    #         _plot_row(ax, m.t, m.F[:, col], mi, m.label, f'{name}  $({unit})$',
    #                   section_title='Position' if row == 0 else None)
    #     if row == len(pos_signals) - 1:
    #         ax.set_xlabel(r'Time $(s)$', fontsize=9)

    # target_pos_values = [-1.3, 2.6*np.pi/180,-0.5*np.pi/180,0*np.pi/180]
    # for mi, m in enumerate(models):
    #     print(f"\n{m.label} RMSE:")

    #     for i, (col, name, unit) in enumerate(pos_signals):
    #         signal = m.F[:, col]
    #         target = target_pos_values[i]

    #         rmse = np.sqrt(np.mean((signal - target)**2))

    #         print(f"  {name}: {rmse:.4f} {unit}")
    # # ── Velocity rows ──────────────────────────────────────────
    # offset = len(pos_signals)
    # for row, (col, name, unit) in enumerate(vel_signals):
    #     ax = axes[offset + row]
    #     for mi, m in enumerate(models):
    #         _plot_row(ax, m.t, m.F[:, 6 + col], mi, m.label, f'{name}  $({unit})$',
    #                   section_title='Velocity' if row == 0 else None)
    #     if row == len(vel_signals) - 1:
    #         ax.set_xlabel(r'Time $(s)$', fontsize=9)

    # ── Control input rows ─────────────────────────────────────
    offset = 0 #len(vel_signals)
    for row, ctrl_label in enumerate(ctrl_labels):
        ax = axes[offset + row]
        for mi, m in enumerate(models):
            _plot_row(ax, m.u[:,0], m.u[:, row+1], mi, m.label, r'$rad$',
                      section_title='Control inputs' if row == 0 else None)
        ax.set_xlim(0, 150)
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
    # vmg_e_log = d['vmg_e_log']
    # vmg_log = d['vmg_log']

    # Build F to match your (N,12) convention: [eta(0:6) | nu(6:12)]
    # eta: pad x=0, y=0, then z, roll, pitch, yaw
    # nu:  pad u=vx, v=vy, then w, p, q, r
    N  = len(t)
    F  = np.zeros((N, 12))
    F[:, 2] = states[:, 0]   # z
    F[:, 3] = states[:, 1]   # roll
    F[:, 4] = states[:, 2]   # pitch
    F[:, 5] = states[:, 3]   # yaw
    F[:, 6] = states[:, 4]   # vx  (surge — index 6 would be u)
    F[:, 7] = states[:, 5]   # vy
    F[:, 8] = states[:, 6]   # w
    F[:, 9] = states[:, 7]   # p
    F[:,10] = states[:, 8]   # q
    F[:,11] = states[:, 9]   # r

    return ModelData(t=t, F=F, u=actions, label=label) #,vmg_log=vmg_log, vmg_e_log=vmg_e_log

# ── Load data ──────────────────────────────────────────────────
def load_model(path, label):
    mat = loadmat(path)
    return ModelData(
        t     = mat['t'].squeeze(),
        F     = mat['F'],
        u     = mat['uMatrix'],
        label = label,
        # vmg_log     = mat['vmgMatrix'],
        # vmg_e_log     = mat['vmgEMatrix']
    )

# lqr      = load_model('../AdaptationforFoilingCatamaran/lqr.mat',     'LQR')
# lqr_gain = load_model('../AdaptationforFoilingCatamaran/lqrGain_wind_variation.mat', 'LQR gain scheduled')
# ppo_base_base = load_ppo_episode("ppo_actuate_wind_2%.npz", "PPO Actuate")
# ppo_base_base_1 = load_ppo_episode("ppo_actuate_wind_1%_1.npz", "PPO Actuate 1")
lqr      = load_model('analysis/lqr_base.mat',     'LQR')
lqr_gain = load_model('analysis/lqrGain_base.mat', 'LQR Gain Scheduled')
ppo_base = load_ppo_episode("analysis/ppo_base_base.npz", "PPO Base")
ppo_delay = load_ppo_episode("analysis/ppo_delay_base.npz", "PPO Delay")
plotComparison([lqr, lqr_gain, ppo_base, ppo_delay])
#plotVmgComparison([lqr_vmg, lqr_gain_vmg, ppo_base_vmg_validation, ppo_delay_vmg_validation])
