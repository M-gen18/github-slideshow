# Implementation Checklist - RFT Violation Fix

## Pre-Implementation

- [ ] Backup your current MATLAB project
- [ ] Note your current simulation results for comparison
- [ ] Verify MATLAB version (R2018b or later recommended)

## Implementation Steps

### Step 1: Update summarize_log_uav.m

- [ ] Open `summarize_log_uav.m` in MATLAB editor
- [ ] Locate the function declaration (line 1)
- [ ] Update input parser section (lines 1-11):
  - [ ] Add: `p.addParameter('RFT_VIOL_TOL', -1e-5);`
  - [ ] Add: `viol_tol = p.Results.RFT_VIOL_TOL;`
- [ ] Locate RFT metrics section (around line 67)
- [ ] Remove hardcoded thresholds:
  - [ ] Delete: `RFT_ACTIVE_TOL = 1e-4;`
  - [ ] Delete: `RFT_VIOL_TOL = -1e-5;`
- [ ] Update violation calculation (around line 80):
  - [ ] Change `abs(m) <= RFT_ACTIVE_TOL` to `abs(m) <= tol`
  - [ ] Change `m < RFT_VIOL_TOL` to `m < viol_tol`
- [ ] Save the file

### Step 2: Update run_RFT_study.m

- [ ] Open `run_RFT_study.m` in MATLAB editor
- [ ] Verify line 33 has: `RFT_VIOL_TOL = -0.015;`
- [ ] Locate single-mission summary section (around line 166)
- [ ] Update 3-4 `summarize_log_uav` calls to add parameter:
  - [ ] `S_pid` call: Add `'RFT_VIOL_TOL',RFT_VIOL_TOL,`
  - [ ] `S_lmpc` call: Add `'RFT_VIOL_TOL',RFT_VIOL_TOL,`
  - [ ] `S_rft` call: Add `'RFT_VIOL_TOL',RFT_VIOL_TOL,`
  - [ ] `S_fuzzy` call (if applicable): Add `'RFT_VIOL_TOL',RFT_VIOL_TOL,`
- [ ] Save the file

## Validation Steps

### Step 3: Test the Fix

- [ ] Run the simulation: `run_RFT_study`
- [ ] Wait for completion (may take 30-60 minutes for Monte Carlo runs)
- [ ] Check console output for safety violation percentages

### Step 4: Verify Results

Expected output in Monte Carlo summary:

- [ ] PID Controller: Safety_Violation_Pct ≈ 100%
- [ ] LMPC Controller: Safety_Violation_Pct ≈ 100%
- [ ] LMPC+RFT Controller: Safety_Violation_Pct ≈ 0-5% (low or zero)

### Step 5: Review Detailed Metrics

- [ ] Check Single Mission Summary table
- [ ] Verify `RFT_violation_pct` column shows realistic values
- [ ] Review RFT margin plots for LMPC+RFT
- [ ] Confirm margins are properly classified

## Troubleshooting

### If violations are still 0% for all controllers:

- [ ] Verify `RFT_VIOL_TOL = -0.015` in run_RFT_study.m (line 33)
- [ ] Check that parameter is being passed in all summarize_log_uav calls
- [ ] Ensure both files were updated (not just one)
- [ ] Clear workspace: `clear all; close all; clc;`
- [ ] Re-run the script

### If violations are 100% for LMPC+RFT:

- [ ] Check `mpc_r.use_rft_constraint = true` (should be enabled)
- [ ] Verify RFT specification: `mpc_r.RFT_spec_s = Treq_s`
- [ ] Review RFT margin values in output logs
- [ ] Check if MPC solver is functioning correctly

### If getting errors:

- [ ] **"Unrecognized parameter RFT_VIOL_TOL"**
  - summarize_log_uav.m wasn't updated correctly
  - Re-check Step 1

- [ ] **"Too many input arguments"**
  - Check function call syntax in run_RFT_study.m
  - Ensure proper comma placement

- [ ] **"Index exceeds array dimensions"**
  - Likely unrelated to this fix
  - Check your simulation setup and data

## Post-Implementation

- [ ] Document your results
- [ ] Compare before/after violation percentages
- [ ] Save updated figures and tables
- [ ] Archive the updated code version

## Rollback Plan

If you need to revert changes:

- [ ] Restore `summarize_log_uav.m` from backup
- [ ] Restore `run_RFT_study.m` from backup
- [ ] Clear workspace and re-run original version

## Success Criteria

✓ Simulation runs without errors
✓ PID shows ~100% violations (no RFT protection)
✓ LMPC shows ~100% violations (no RFT protection)
✓ LMPC+RFT shows 0-5% violations (RFT protection working)
✓ Results are consistent across Monte Carlo runs
✓ RFT margins are properly classified

---

**Estimated Time**: 10-15 minutes for implementation + 30-60 minutes for testing

**Difficulty Level**: Beginner-Intermediate

**Required Tools**: MATLAB R2018b+ with Optimization Toolbox

**Files Modified**: 2 files only (minimal invasive)
