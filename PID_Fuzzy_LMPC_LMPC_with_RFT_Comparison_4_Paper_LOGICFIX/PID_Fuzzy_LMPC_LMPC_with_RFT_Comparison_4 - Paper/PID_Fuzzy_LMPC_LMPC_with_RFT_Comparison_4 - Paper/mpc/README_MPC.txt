Linear MPC (quadprog) - Stable structure for future extensions
-------------------------------------------------------------
Run:
  main_init_uav_mpc.m

Key files:
  mpc/mpc_settings_uav.m
  mpc/mpc_linearize_uav.m
  mpc/mpc_build_qp_uav.m
  mpc/mpc_solve_uav_qp.m
  simulate_uav_with_mpc.m

Design notes:
- Decision variable is the current sequence U only (convex QP).
- Slack-MPC later extends mpc_build_qp_uav.m by augmenting the decision vector
  with slack variables and adding corresponding penalties/constraints, without
  changing the rest of the simulation structure.
