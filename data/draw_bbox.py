import os
import matplotlib
# Use non-interactive backend if no display available
if os.environ.get('DISPLAY') is None:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Lists for measurements
m_cx, m_cy, m_w, m_h = [], [], [], []
# Lists for predictions
p_cx, p_cy, p_w, p_h = [], [], [], []
# Lists for updated states
z_cx, z_cy, z_w, z_h = [], [], [], []

with open('./data/kalman-bbox.csv', 'r') as f:
    for line in f:
        line = line.rstrip()
        # Skip empty lines and test framework output
        if not line or not ';' in line:
            continue
        data = line.split(';')
        # Skip header line
        if data[0] == 'measurement cx':
            continue
        try:
            # Measurements
            m_cx.append(float(data[0]))
            m_cy.append(float(data[1]))
            m_w.append(float(data[2]))
            m_h.append(float(data[3]))
            # Predictions
            p_cx.append(float(data[4]))
            p_cy.append(float(data[5]))
            p_w.append(float(data[6]))
            p_h.append(float(data[7]))
            # Updated states
            z_cx.append(float(data[8]))
            z_cy.append(float(data[9]))
            z_w.append(float(data[10]))
            z_h.append(float(data[11]))
        except (ValueError, IndexError):
            continue  # Skip malformed lines

dpi = 100
fig, axes = plt.subplots(2, 2, figsize=(1440/dpi, 960/dpi), dpi=dpi)

# Top-left plot: Center trajectories (similar to kalman-2d)
ax1 = axes[0, 0]
ax1.plot(m_cx, m_cy, 'r', marker='o', markersize=3, label='Measurement')
ax1.plot(p_cx, p_cy, 'g', marker='s', markersize=3, label='Prediction')
ax1.plot(z_cx, z_cy, 'y', marker='^', markersize=3, label='Updated state')
ax1.legend(loc="upper left")
ax1.grid(alpha=0.2)
ax1.invert_yaxis()
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title("Center Trajectories", fontsize=12, fontweight='bold')

# Top-right plot: Bounding boxes at selected frames
ax2 = axes[0, 1]

# Select frames to visualize (every N-th frame to avoid clutter)
n_frames = len(m_cx)
step = max(1, n_frames // 8)  # Show ~8 frames
frame_indices = list(range(0, n_frames, step))

# Color progression for temporal visualization
colors_m = plt.cm.Reds([0.3 + 0.7 * i / len(frame_indices) for i in range(len(frame_indices))])
colors_p = plt.cm.Greens([0.3 + 0.7 * i / len(frame_indices) for i in range(len(frame_indices))])
colors_z = plt.cm.YlOrBr([0.3 + 0.7 * i / len(frame_indices) for i in range(len(frame_indices))])

for idx, frame in enumerate(frame_indices):
    # Measurement bbox (red)
    rect_m = patches.Rectangle(
        (m_cx[frame] - m_w[frame]/2, m_cy[frame] - m_h[frame]/2),
        m_w[frame], m_h[frame],
        linewidth=1.5, edgecolor=colors_m[idx], facecolor='none',
        linestyle='-', label='Measurement' if idx == 0 else None
    )
    ax2.add_patch(rect_m)
    ax2.plot(m_cx[frame], m_cy[frame], 'o', color=colors_m[idx], markersize=4)

    # Prediction bbox (green)
    rect_p = patches.Rectangle(
        (p_cx[frame] - p_w[frame]/2, p_cy[frame] - p_h[frame]/2),
        p_w[frame], p_h[frame],
        linewidth=1.5, edgecolor=colors_p[idx], facecolor='none',
        linestyle='--', label='Prediction' if idx == 0 else None
    )
    ax2.add_patch(rect_p)
    ax2.plot(p_cx[frame], p_cy[frame], 's', color=colors_p[idx], markersize=4)

    # Updated state bbox (yellow/orange)
    rect_z = patches.Rectangle(
        (z_cx[frame] - z_w[frame]/2, z_cy[frame] - z_h[frame]/2),
        z_w[frame], z_h[frame],
        linewidth=1.5, edgecolor=colors_z[idx], facecolor='none',
        linestyle=':', label='Updated state' if idx == 0 else None
    )
    ax2.add_patch(rect_z)
    ax2.plot(z_cx[frame], z_cy[frame], '^', color=colors_z[idx], markersize=4)

    # Add frame number annotation
    ax2.annotate(f'{frame}', (m_cx[frame], m_cy[frame] - m_h[frame]/2 - 5),
                 fontsize=7, ha='center', color=colors_m[idx])

ax2.legend(loc="upper left")
ax2.grid(alpha=0.2)
ax2.invert_yaxis()
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_title("Bounding Boxes (selected frames)", fontsize=12, fontweight='bold')
ax2.set_aspect('equal', adjustable='datalim')
ax2.autoscale()

# Bottom-left plot: Width over time
ax3 = axes[1, 0]
frames = list(range(len(m_w)))
ax3.plot(frames, m_w, 'r', label='Measurement', alpha=0.7)
ax3.plot(frames, p_w, 'g', label='Prediction', alpha=0.7)
ax3.plot(frames, z_w, 'y', label='Updated state', alpha=0.7)
ax3.legend(loc="upper left")
ax3.grid(alpha=0.2)
ax3.set_xlabel('Frame')
ax3.set_ylabel('Width')
ax3.set_title("Width over time", fontsize=12, fontweight='bold')

# Bottom-right plot: Height over time
ax4 = axes[1, 1]
ax4.plot(frames, m_h, 'r', label='Measurement', alpha=0.7)
ax4.plot(frames, p_h, 'g', label='Prediction', alpha=0.7)
ax4.plot(frames, z_h, 'y', label='Updated state', alpha=0.7)
ax4.legend(loc="upper left")
ax4.grid(alpha=0.2)
ax4.set_xlabel('Frame')
ax4.set_ylabel('Height')
ax4.set_title("Height over time", fontsize=12, fontweight='bold')

plt.suptitle("Kalman BBox", fontsize=15, fontweight='bold')
plt.tight_layout()
plt.savefig('./data/kalman-bbox.png', dpi=dpi)
if os.environ.get('DISPLAY') is not None:
    plt.show()
