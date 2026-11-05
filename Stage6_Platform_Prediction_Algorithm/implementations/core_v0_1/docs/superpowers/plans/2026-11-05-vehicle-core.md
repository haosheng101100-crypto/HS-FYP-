# Vehicle Prediction Core Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans; native execution follows user instruction to continue the agreed route.

**Goal:** Implement actual vehicle-model UKF/IMM prediction core before independent simulation.
**Architecture:** Explicit 8D nonholonomic dynamics, nonlinear pose filter, CTMC model interaction, trajectory-preserving stochastic forecast and rigid deck map.
**Tech Stack:** Python 3.12, NumPy >=2.0, SciPy >=1.14, unittest. Existing scientific runtime used read-only.
**Spec:** ../specs/2026-11-05-vehicle-core-design.md

## Global Constraints
Online input only stamped received poses and covariance. No future route/control. No old files edited or shared branch writes. State order x,y,psi,v,a,kappa,j,eta. RK4 max .05s, minimum5 poses/.4s, stale .2s, reset gap1s. Four models never merge future branches. No statistical or reachable-set guarantee.

## Review Focus
- Heading wrap, signed reverse speed and low-speed curvature ambiguity.
- Observation delay/ordering, initialization offset covariance, stale/restart.
- Positive covariance and log probability at very small likelihoods.
- True model derivatives must obey constraints, not merely hidden j/eta.
- Forecast immutability/local RNG and explicit unsampled modes.

### Task 1: Dynamics
Files stage6/model.py, tests/test_model.py.
Interfaces: Mode, Limits; step(state8,dt,mode)->state8; violations(state,mode,limits)->list; transition_matrix(dt)->4x4.
- [x] RED tests against analytic motion, mode damping derivatives, input checks, transition semigroup.
- [x] Implement vectorized RK4 for all sigma points; immutable configs.
- [x] GREEN full unittest and ledger evidence.

### Task 2: UKF
Files stage6/filter.py, stage6/observation.py, tests/test_filter.py.
Interfaces: Observation; UKF(mean,cov,mode,offset); predict(dt); update(obs)->log_likelihood; initialize(obs,mode,offset).
- [x] RED analytical linear update, covariance, wrap, stationary uncertainty, offset Jacobian.
- [x] Implement UT, circular moments, PSD checks, process update, pose likelihood.
- [x] GREEN full suite and record results.

### Task 3: IMM and future output
Files stage6/predictor.py, stage6/forecast.py, tests/test_predictor.py, tests/test_forecast.py.
Interfaces: VehiclePredictor.update(Observation); forecast(now,...)->serializable dict. Forecast branches sample full trajectories; rejected samples/counts explicit.
- [x] RED mode interaction and between-mode covariance, causality, fresh/stale/gap, no mutation/RNG, branch preservation.
- [x] Implement continuous-time Markov switching, stable log weights, timestamp lifecycle and bounded sample forecasts.
- [x] GREEN entire suite, JSON serialization, independent whole-core review.

### Task 4: Deck map and deliverable
Files stage6/deck.py, tests/test_deck.py, README.md, 算法推导.md, verification/.
- [x] RED rotational offset velocity and rectangle corners; implement pure map preserving samples.
- [x] GREEN analytic tests, source compilation and independent review; fix important findings with failing regression tests first.
- [x] Deliver reproducible source, config, analytical check outputs and honest stage status. No simulation acceptance claim.
