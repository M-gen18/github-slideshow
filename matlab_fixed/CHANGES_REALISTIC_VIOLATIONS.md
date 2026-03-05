# Changes to Enable Realistic Safety Violations

## Problem
The original code showed **0% safety violations** for LMPC+RFT because three artificial safety buffers were masking real constraint violations:

1. **`rft_gain = 20`**: Amplified urgency by 20x
2. **`rft_soc_buffer = 0.02`**: Added 2% SoC safety buffer  
3. **`rft_margin_bias`**: Subtracted from RHS to create positive bias
4. **`RFT_VIOL_TOL = -0.015`**: Required 1.5% SoC dip to count as violation

These parameters made the RFT constraint artificially conservative, preventing any violations from being detected.

## Solution
Removed all artificial safety buffers to expose real violations:

### File 1: `mpc/mpc_build_qp_uav.m` (lines 321-336)
**Changed:**
```matlab
% OLD (artificial buffers):
rft_gain = max(getfield_def(mpc,'rft_gain',20), 1);
rft_soc_buffer = max(getfield_def(mpc,'rft_soc_buffer',0.02), 0);
rft_margin_bias = max(getfield_def(mpc,'rft_margin_bias',0.0), 0);
urgency = min(1, alpha * (rft_gain / 20));
SoC_floor = SoC_min + rft_soc_buffer*urgency;
RHS = SoC_k + (SoC_floor - SoC_k)*alpha - rft_margin_bias;

% NEW (realistic):
rft_gain = 1.0;          % No urgency amplification
rft_soc_buffer = 0.0;    % No artificial buffer
rft_margin_bias = 0.0;   % No bias correction
urgency = alpha;         % Direct alpha
SoC_floor = SoC_min;     % Use actual minimum
RHS = SoC_k + (SoC_floor - SoC_k)*alpha;
```

### File 2: `run_RFT_study.m` (line 35)
**Changed:**
```matlab
% OLD: 1.5% tolerance
RFT_VIOL_TOL = -0.015;

% NEW: Numerical precision tolerance
RFT_VIOL_TOL = -1e-5;
```

## Impact
- **Before**: Safety_Violation_Pct = 0% (artificially safe)
- **After**: Safety_Violation_Pct will reflect actual constraint violations when SoC_term < RHS

## Files Modified
1. `mpc/mpc_build_qp_uav.m` - RFT constraint formulation
2. `run_RFT_study.m` - Violation detection threshold

## Files Already Correct
- `simulate_uav_with_mpc.m` - Already uses `RFT_VIOL_TOL = -1e-5`
- `summarize_log_uav.m` - Already uses `RFT_VIOL_TOL = -1e-5`
- `uav_mpc_lib.m` - Already uses simple RHS formula

## Verification
Run the simulation and check:
```matlab
% Should now show realistic violation percentages
MC.Safety_Violation_Pct
```

Expected: LMPC+RFT will show some violations (e.g., 5-20%) when disturbances push the system beyond its true safety margin.
