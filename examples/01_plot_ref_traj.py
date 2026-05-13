"""Plot joint positions and velocities from collected reference trajectory data."""

import argparse
import numpy as np
import matplotlib.pyplot as plt
import glob
import os


def plot_npz(path, freq):
    data = np.load(path)
    has_hf = 'hf_qvel' in data

    ncols = 3 if has_hf else 2
    fig, axes = plt.subplots(7, ncols, figsize=(6 * ncols, 14))

    t = np.arange(data['qpos'].shape[0]) / freq
    for j in range(7):
        axes[j, 0].plot(t, data['qpos'][:, j], label='actual')
        axes[j, 0].plot(t, data['qdes'][:, j], label='desired', linestyle='--')
        axes[j, 0].set_ylabel(f'Joint {j}')
        if j == 0:
            axes[j, 0].set_title('Joint Position')
            axes[j, 0].legend()
        if j == 6:
            axes[j, 0].set_xlabel('Time (s)')

        axes[j, 1].plot(t, data['qvel'][:, j])
        axes[j, 1].set_ylabel(f'Joint {j}')
        if j == 0:
            axes[j, 1].set_title(f'Joint Velocity ({freq} Hz)')
        if j == 6:
            axes[j, 1].set_xlabel('Time (s)')

        if has_hf:
            sample_freq = float(data['sample_freq'])
            t_hf = np.arange(data['hf_qvel'].shape[0]) / sample_freq
            axes[j, 2].plot(t_hf, data['hf_qvel'][:, j])
            axes[j, 2].set_ylabel(f'Joint {j}')
            if j == 0:
                axes[j, 2].set_title(f'Joint Velocity ({int(sample_freq)} Hz)')
            if j == 6:
                axes[j, 2].set_xlabel('Time (s)')

    fig.tight_layout()
    out = path.replace('.npz', '.png')
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved {out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--freq", required=True, type=float, help="Control frequency used during collection")
    parser.add_argument("--dir", required=True, type=str, help="Directory containing .npz files")
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.dir, "*.npz")))
    if not files:
        print(f"No .npz files found in {args.dir}")
    for f in files:
        plot_npz(f, args.freq)
