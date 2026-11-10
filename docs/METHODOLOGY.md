# Methodology and implementation audit

## Geometric model

Standard DH transforms map four revolute joints plus a fixed auxiliary link into the base frame. The base translation is 0.14 m; non-zero link geometry is a1=0.08 m, d3=0.045 m, d3_fixed=0.11 m and a4=0.085 m. The zero-angle end-effector position is A=[0.165,0.110,0.185] m. Workspace analysis uses 15 samples per joint. The end-effector and auxiliary-link-midpoint Jacobians share the first three revolute axes; joint 4 does not affect that midpoint.

## Planner and differential control

A scalar cubic displacement parameterises each straight Cartesian segment with zero endpoint velocities. The simulation uses 0–8–16 s, the supplied physical controller 0–12–24 s. Forward Euler updates are clipped by joint bounds; joint velocities are clipped separately. Pseudoinverse control is undamped: near singularities, clipping bounds commands but does not establish stability or maintain the exact null-space decomposition.

The original error array compares desired position at t_i with forward kinematics at q_(i−1), while the logged simulated actual position is evaluated after q_i is integrated. This convention is preserved. Thus the error array is not exactly the norm of the plotted desired-minus-post-update positions. Historical table values follow the original error array.

## Redundancy and obstacle clearance

The local objective is d(q)=norm(p_mid(q)−centre)−radius, with gradient J_mid.T @ unit_direction. An indicator activates it when local clearance is below 0.06 m. Its projected contribution is (I−Jplus J) @ ka grad(d). The independent SymPy derivation and numerical comparison are retained.

Global clearance takes the minimum sphere-to-line-segment clearance across the chain. It is not the optimised objective and excludes physical link thickness. A positive minimum in a sampled trajectory does not prove swept-volume collision freedom. Saturation and finite steps can compromise ideal task decoupling.

## Gain search discrepancy

The presentation describes selecting the lowest qualifying primary gain. The code instead ranks valid candidates using `(gain <= 10, -mean_error)` with `max`: candidates <=10 are preferred, then minimum mean error determines the selection. This implemented behaviour is preserved. The secondary search maximises local minimum clearance among trials meeting clearance and joint-margin constraints, with distance-only and unconstrained fallbacks if needed. Selected saved gains are 10 and 1.5; distance-only selection is 3, which saturates q2.

## Physical execution

The physical script contains no secondary obstacle objective. It uses k=3 and a vertical factor of 1.5, encoder feedback, a 12/36 first-joint ratio and non-blocking EV3 position targets. Ports A/B/C use large motors, D a medium motor; A and D are inverted. Nominal speed is 15 (EV3 percentage-style speed setting), motor-1 speed is 45. Joint-velocity limiting is a different quantity: 15 degrees/s. Integration dt is measured with time.time and clipped to [1e-4,0.10] s; increments are limited to 3 degrees.

The measured encoder state, rather than the previous command, is used in each target update. Cartesian feedback is kinematically reconstructed from those encoders. It is not an independent Cartesian measurement and cannot quantify model calibration error or structural flexure.

Encoder reset does not home the arm. It establishes a software reference that must match the physical zero posture. Original reset/polarity order is preserved; confirm behaviour on the actual ev3dev version. Wall-clock timing is not monotonic, and clipping dt means logs do not retain complete elapsed timestamps. The experimental time axis reconstructed by cumulative clipped dt can underestimate duration.

## Corrections and their scope

- Removed the invalid leading plus before the script shebang.
- Moved ev3dev imports, motor construction and polarity writes into main(). Importing geometry functions does not access devices.
- Added each successfully constructed motor to the cleanup list immediately. A finally block now attempts to brake every motor even if setup, control or unexpected errors fail; a failure stopping one motor does not prevent attempts on the others.
- Normal, interrupted and unexpected-error exits use consistent CENCI3v log filenames. Unlike the original inconsistent interrupted filenames, this permits the same loader to read partial logs. Existing same-name logs can be overwritten; use a fresh working directory per run.
- Cleanup runs before log saving. If file saving fails, motors have already received stop attempts. A physical power loss or process kill can still prevent Python cleanup. No hardware-level safety guarantee is made.
- Added the missing pointSegmentClosestPoint helper before its first simulation use. The source notebook called it without defining it; the implementation uses the same clipped projection as the existing segment-distance function.
- Translated Markdown and plot/console labels; removed Italian comments and redundant prose. Numeric constants, gain grids and control equations are preserved.
- Experimental Qdes/Qreal logs are radians, confirmed from the supplied controller and recorded increments. New analysis explicitly converts them to degrees for display. Old degree-labelled plots consequently should not be used as quantitative joint-angle evidence.
- Experimental timing logs are required rather than silently labelling sample indices as seconds. Original shortest-array alignment and cumulative clipped-dt reconstruction remain.

Safety and lifecycle corrections require a new supervised hardware verification. Historical physical results belong to the original controller run, not the modified script. No controller was executed on hardware during this task.

## Experimental evidence

The complete local log directory contains six trial prefixes. Four named trials are used in the README. CENCI3, CENCI3z and CENCI3v vertical multipliers are annotations from the supplied analysis; only CENCI3v has its controller version supplied. The logs reproduce the saved Cartesian summary values. The presentation's preference for a lower gain reflects qualitative motion regularity, not a formal stability test or repeated-trial significance analysis. Raw logs are deliberately not published.
