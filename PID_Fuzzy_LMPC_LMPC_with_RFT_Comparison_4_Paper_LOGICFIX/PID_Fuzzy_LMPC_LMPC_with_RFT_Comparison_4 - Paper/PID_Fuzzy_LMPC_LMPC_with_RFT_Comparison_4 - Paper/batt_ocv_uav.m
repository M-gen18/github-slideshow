function Voc = batt_ocv_uav(SoC, par)
%BATT_OCV_UAV Pack open-circuit voltage model.
s = min(max(SoC,0),1);
Voc = par.ocv_a + par.ocv_b*s + par.ocv_c*tanh(par.ocv_d*(s - par.ocv_e));
end
