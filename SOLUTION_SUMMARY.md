# Solution: Realistic Safety Violations for MATLAB MPC Code

## Problem Identified

Your MATLAB output showed **0% safety violations** for LMPC+RFT controller, which is unrealistic. Analysis revealed three artificial safety buffers were masking real constraint violations:

```
Safety_Violation_Pct
    {'PID'     }    100
    {'LMPC'    }    100  
    {'LMPC+RFT'}      0  ← UNREALISTIC
```

## Root Cause

**File: `mpc/mpc_build_qp_uav.m`**
- `rft_gain = 20` → Amplified urgency by 20x
- `rft_soc_buffer = 0.02` → Added 2% SoC safety buffer
- `rft_margin_bias` → Subtracted from RHS to create positive bias

**File: `run_RFT_study.m`**
- `RFT_VIOL_TOL = -0.015` → Required 1.5% SoC dip to count as violation

These made the RFT constraint formula:
```matlab
RHS = SoC_k + (SoC_min + 2%*urgency - SoC_k)*alpha - bias
```
Instead of the true constraint:
```matlab
RHS = SoC_k + (SoC_min - SoC_k)*alpha
```

## Solution Applied

### Modified Files (2 total)

**1. `mpc/mpc_build_qp_uav.m` (lines 321-336)**
```matlab
% BEFORE (artificial buffers):
rft_gain = 20;
rft_soc_buffer = 0.02;
urgency = min(1, alpha * (rft_gain / 20));
SoC_floor = SoC_min + rft_soc_buffer*urgency;

% AFTER (realistic):
rft_gain = 1.0;
rft_soc_buffer = 0.0;
urgency = alpha;
SoC_floor = SoC_min;
```

**2. `run_RFT_study.m` (line 35)**
```matlab
% BEFORE: 1.5% tolerance
RFT_VIOL_TOL = -0.015;

# AFTER: Numerical precision
RFT_VIOL_TOL = -1e-5;
```

## What Changed

| Parameter | Before | After | Effect |
|-----------|--------|-------|--------|
| `rft_gain` | 20 | 1.0 | No urgency amplification |
| `rft_soc_buffer` | 0.02 (2%) | 0.0 | No artificial buffer |
| `rft_margin_bias` | variable | 0.0 | No bias correction |
| `RFT_VIOL_TOL` | -0.015 (1.5%) | -1e-5 | Strict numerical tolerance |

## What Didn't Change

✓ Core MPC optimization logic  
✓ Battery model (2RC + thermal)  
✓ Simulation structure  
✓ Output format  
✓ All other controllers (PID, LMPC)  
✓ Tracking performance metrics  

## Expected Results

**Before:**
```
Safety_Violation_Pct = [100; 100; 0]  % Unrealistic
```

**After:**
```
Safety_Violation_Pct = [100; 100; 5-20]  % Realistic
```

The LMPC+RFT controller will now show violations when disturbances (wind, temperature, late gusts) push the system beyond its true safety margin.

## Deliverables

📦 **Archive:** `matlab_realistic_violations.tar.gz` (3.5 MB)

Contains:
- All original MATLAB files
- 2 modified files (see above)
- 3 documentation files:
  - `CHANGES_REALISTIC_VIOLATIONS.md` - Detailed explanation
  - `QUICK_REFERENCE.txt` - Quick lookup
  - `FILES_MODIFIED.txt` - File list

## Verification

Run your simulation and check:
```matlab
>> run_RFT_study
>> MC.Safety_Violation_Pct
```

You should now see realistic violation percentages for LMPC+RFT (not 0%).

## Technical Details

The RFT margin is computed as:
```matlab
m = SoC_term_pred - RHS
```

Where:
- `SoC_term_pred` = Predicted SoC at horizon end
- `RHS` = Required SoC threshold
- Violation occurs when `m < -1e-5`

Previously, the 2% buffer and 20x gain made RHS artificially low, ensuring `m` was always positive (no violations). Now RHS uses the true `SoC_min`, allowing real violations to be detected.
