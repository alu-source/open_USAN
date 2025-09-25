function [v_corr] = Helper_apply_correction_matrix(correction_matrix, meas_points)
%HELPER_APPLY_CORRECTION_MATRIX
v_corr(:,1) = interp2([-40.5:0.45:40.5],[-40.5:0.45:40.5],correction_matrix(:,:,1),meas_points(:,2),meas_points(:,1)); 
v_corr(:,2) = interp2([-40.5:0.45:40.5],[-40.5:0.45:40.5],correction_matrix(:,:,2),meas_points(:,2),meas_points(:,1)); 



end

 