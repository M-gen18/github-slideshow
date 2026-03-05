function x0 = batt_init_state(par)
%BATT_INIT_STATE Initialize state struct.
x0 = struct('SoC',0.88,'V1',0.012,'V2',0.005,'T',25+273.15,'SoH',0.94);
end
