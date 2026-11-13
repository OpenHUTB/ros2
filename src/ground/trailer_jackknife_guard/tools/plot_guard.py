import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from trailer_jackknife_guard.model import TrailerParams
from trailer_jackknife_guard.simulation import run_scenario
import math

p = TrailerParams()
delta = math.radians(6.0)
unguarded = run_scenario("reverse_unguarded", -1.2, delta, p, duration=20.0,
                         guarded=False)
guarded = run_scenario("reverse_guarded", -1.2, delta, p, duration=30.0,
                       guarded=True)

fig, ax = plt.subplots(figsize=(7.5, 4.5), dpi=150)
ax.plot([t for t, *_ in unguarded.trace], [phi for _, phi, *_ in unguarded.trace],
        label="reverse, no guard")
ax.plot([t for t, *_ in guarded.trace], [phi for _, phi, *_ in guarded.trace],
        label="reverse, guarded")
ax.axhline(math.degrees(p.jackknife_angle), color="red", linestyle="--",
           label="jackknife limit 75 deg")
ax.axhline(math.degrees(p.jackknife_angle) * 0.85, color="orange",
           linestyle=":", label="guard margin 63.75 deg")
ax.set_xlabel("time [s]")
ax.set_ylabel("hitch angle [deg]")
ax.set_title("trailer jackknife guard: 1.2 m/s reverse, 6 deg steer")
ax.grid(True, linestyle="--", alpha=0.5)
ax.legend(loc="lower right")
fig.savefig("/mnt/c/Windows/Temp/trailer_guard_demo.png", bbox_inches="tight")
print("saved", "unguarded_jackknife_at", unguarded.time_to_jackknife,
      "guarded_max", round(guarded.max_abs_phi_deg, 2))
