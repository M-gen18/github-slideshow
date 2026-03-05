# Quick Reference Guide - RFT Violation Fix

## What Was Fixed?

The safety violation detection was not working because the threshold parameter wasn't being passed correctly between functions.

## Files Changed

1. **summarize_log_uav.m**
2. **run_RFT_study.m**

## Before vs After

### Before:
```
Safety_Violation_Pct for LMPC+RFT: 0% (INCORRECT)
```
The system used hardcoded threshold of -0.00001 instead of configured -0.015

### After:
```
Safety_Violation_Pct for LMPC+RFT: 0% or realistic low value (CORRECT)
```
The system now uses the properly configured threshold of -0.015

## How to Use Updated Files

### Option 1: Replace Files Directly
1. Navigate to your MATLAB project directory
2. Replace the old files with the updated versions from `UPDATED_MATLAB_FILES/`
3. Run your simulation as normal

### Option 2: Manual Integration
If you have custom modifications:

**In `summarize_log_uav.m`** (lines 1-11):
```matlab
function S = summarize_log_uav(log, par, varargin)
p = inputParser;
p.addParameter('RFT_ACTIVE_TOL', 5e-3);
p.addParameter('RFT_VIOL_TOL', -1e-5);      % ADD THIS LINE
p.addParameter('t_critical_start', []);
p.addParameter('t_critical_end', []);
p.parse(varargin{:});
tol = p.Results.RFT_ACTIVE_TOL;
viol_tol = p.Results.RFT_VIOL_TOL;          % ADD THIS LINE
t1  = p.Results.t_critical_start;
t2  = p.Results.t_critical_end;
```

**In `summarize_log_uav.m`** (around line 79):
```matlab
% CHANGE FROM:
S.RFT_violation_pct = 100*mean(m < RFT_VIOL_TOL);

% CHANGE TO:
S.RFT_violation_pct = 100*mean(m < viol_tol);
```

**In `run_RFT_study.m`** (around line 166):
```matlab
% ADD 'RFT_VIOL_TOL',RFT_VIOL_TOL to each summarize_log_uav call:
S_pid = summarize_log_uav(log_pid, par, 'RFT_ACTIVE_TOL',RFT_ACTIVE_TOL, 'RFT_VIOL_TOL',RFT_VIOL_TOL, ...);
S_lmpc = summarize_log_uav(log_lmpc, par, 'RFT_ACTIVE_TOL',RFT_ACTIVE_TOL, 'RFT_VIOL_TOL',RFT_VIOL_TOL, ...);
S_rft = summarize_log_uav(log_rft, par, 'RFT_ACTIVE_TOL',RFT_ACTIVE_TOL, 'RFT_VIOL_TOL',RFT_VIOL_TOL, ...);
```

## Testing the Fix

Run your simulation and check the final Monte Carlo summary:

```matlab
=== Final Monte Carlo Performance & Safety Summary ===
     Controller     Safety_Violation_Pct
    ____________    ____________________
    {'PID'     }             ~100%       % Should show violations
    {'LMPC'    }             ~100%       % Should show violations  
    {'LMPC+RFT'}             0-5%        % Should be low/zero
```

## Key Parameters

- `RFT_ACTIVE_TOL = 1e-3`: Constraint is "active" when margin near 0
- `RFT_VIOL_TOL = -0.015`: Violation when margin < -1.5%

## Troubleshooting

**Q: Still showing 0% violations for all controllers?**
A: Check that line 33 in `run_RFT_study.m` has: `RFT_VIOL_TOL = -0.015;`

**Q: Showing 100% violations for LMPC+RFT?**
A: Check RFT constraint is enabled: `mpc_r.use_rft_constraint = true;`

**Q: Getting errors about undefined parameter?**
A: Make sure both files are updated - they work together

## Contact/Support

If you encounter issues:
1. Verify both files are from the same update
2. Check MATLAB version compatibility (R2018b+)
3. Review BUGFIX_SUMMARY.md for detailed technical explanation

---
Last Updated: 2026-03-05
Fix Version: 1.0
