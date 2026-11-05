# Execution ledger

Authorization: user accepted full eight-step route and instructed continue. This increment implements the foundational algorithm (steps1–4), plus pure deck mapping. Independent simulation/calibration/control acceptance follow code, not precede it.
Ruling: use NumPy/SciPy scientific runtime already present, read-only; no installing into old project.
Ruling: augment six kinematic states with jerk/curvature-rate drivers so future samples remain temporally coherent; actual derivatives include damping and are audited.
Ruling: start generic configurable numeric limits; they are test bounds, not real hardware identification.

Task1: model tests RED (missing module) then 7/7 GREEN; analytic line/acceleration/circle/reverse, Markov semigroup and actual derivative limits.
Task2: filter tests RED then total15/15 GREEN; covariance reconstruction, linear update, offset Jacobian and low-speed uncertainty.
Task3: IMM tests RED then total23/23 GREEN; weights, lifecycle, immutable forecast and mode retention.
Task4: deck tests RED then total26/26 GREEN; pure rigid geometry only, no integration acceptance.
Independent core review: all26 pass, one P2 found: broad yaw sigma support aliases through 2pi to falsely tiny covariance. Reproduced with two failing tests (broad prior rejection, transactional prediction), then fixed before wrapping; forecast propagation failure regression also RED->GREEN. No other actionable important core findings. Original review predates new file I/O; I/O self-reviewed and covered by five tests.
File I/O: missing module RED -> GREEN. Added complete packet parameter snapshots and strict pose-only reader. Five tests check CLI, output safety, schema and unchanged prefixes. Analytic circle particle and all-rejection tests cover future sampler.
Final suite:36/36 PASS; exact command and log in verification/tests.txt. Fixture has11 hand-written analytical poses and6 outputs; not a new simulation or experiment.
Ruling: local-angle Gaussian support is explicit (sqrt(8Ppsi)<pi/2); overwide initialization/update errors require caller handling. No broad-angle performance claimed.
Ruling: finite physical trajectory rejection does not produce calibrated future model probabilities; posterior labels retained without renormalizing unsampled modes. No deterministic reachable outer bound supplied.
No source commits or uploads performed; local standalone deliverable. No new simulator, calibration, monitoring or closed-loop work started under user's step-by-step constraint.
