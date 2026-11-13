"""Plot the offline plan_return CSV; requires matplotlib. No vehicle I/O."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("csv", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
with args.csv.open() as stream:
    rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
if not rows:
    raise SystemExit("No successfully solved trajectory samples")
def values(name):
    return [row[name] for row in rows]
t = values("time_s")
fig = plt.figure(figsize=(12, 8), layout="constrained")
ax = fig.add_subplot(2, 2, 1, projection="3d")
ax.plot(values("x_m"), values("y_m"), values("z_m"), color="#2474a8", label="Predicted path")
ax.scatter([rows[0]["x_m"]], [rows[0]["y_m"]], [rows[0]["z_m"]], color="#cc7100", label="Captured state")
ax.scatter([0], [0], [0], marker="*", s=90, color="#198f62", label="Local origin")
ax.set(xlabel="Local X (m)", ylabel="Local Y (m)", zlabel="Local Z (m)", title="Return from received position")
ax.legend(fontsize=8)
ax = fig.add_subplot(2, 2, 2)
for name, label in [("x_m", "X"), ("y_m", "Y"), ("z_m", "Z")]:
    ax.plot(t, values(name), label=label)
ax.set(xlabel="Predicted time (s)", ylabel="Position (m)", title="Position feedback from the ideal plant")
ax.legend(); ax.grid(alpha=.25)
ax = fig.add_subplot(2, 2, 3)
ax.plot(t, [100 * v for v in values("throttle_norm")], color="#2474a8", label="Normalized ESC command")
ax.axhline(69.7214, color="#cc7100", linestyle="--", label="Assumed hover throttle")
ax.set(xlabel="Predicted time (s)", ylabel="Throttle (%)", ylim=(0, 100), title="Combined thrust mapped through assumed linear curve")
ax.legend(fontsize=8); ax.grid(alpha=.25)
ax = fig.add_subplot(2, 2, 4)
ax.plot(t, values("tilt_deg"), label="Vehicle tilt")
ax.plot(t, values("gimbal_x_deg"), label="TVC X")
ax.plot(t, values("gimbal_y_deg"), label="TVC Y")
ax.set(xlabel="Predicted time (s)", ylabel="Angle (degrees)", title="Attitude and TVC commands")
ax.legend(); ax.grid(alpha=.25)
fig.suptitle("Return to local origin — offline MPC prediction, no hardware commands", fontsize=14)
fig.savefig(args.output, dpi=160)
print(args.output)
