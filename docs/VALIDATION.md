# Validation record

The original simulation notebook, physical main.py, experimental notebook, complete 17-slide PowerPoint and 17-page PDF were inspected. The presentation versions agree on geometry, reported simulation outcomes and qualitative physical-trial conclusions. The PowerPoint was inspected as OOXML, and the PDF text was read with pypdf. Project images were extracted directly from the presentation; no full slide deck is redistributed.

## Executed checks

- Both translated notebooks' Python cells parse successfully and execute sequentially in a hardware-free environment.
- The complete simulation, workspace sampling, gain tuning, analytic and SymPy comparisons, global-distance diagnostics and aggressive-gain analysis were run.
- Selected simulation reproduces local minimum clearance 0.649 cm, mean logged error 0.007 mm, maximum logged error 0.014 mm and q2 maximum 29.16 degrees, matching rounded historical outputs.
- Aggressive ka=3 reproduces local minimum clearance 1.172 cm, mean error 0.237 mm and zero joint margin. Global selected clearance matches 0.636 cm.
- The experimental notebook executes on the original local log files. Cartesian summary values match the original saved table to three decimals in cm. Radian-to-degree conversion is applied to joint plots.
- End-effector Jacobian agrees with centred finite differences at a non-zero test configuration (absolute tolerance 1e-8).
- The hardware module was imported without ev3dev2 or motor access. Its Jacobian matches the simulation, and first-joint command conversion maps 1 degree to 3 motor degrees.
- Notebook structure, source syntax and local Markdown image references were checked. Selected English plot figures were regenerated from the original calculations/logs and visually reviewed.

## Limits

The physical controller was never run or called. Motor polarity after reset, brake behaviour, exception handling on a real brick, gearing and timing require supervised hardware verification. No new physical experiment or independent position measurement was performed. Proposed dependency ranges are not an exact recreation of the original software environment. Source logs and the full presentation are omitted from publication; experimental reruns require the original logs.
