function [U] = page_matrix(u,L)
% =========================================================================
%               Generate a Page matrix of order L
% =========================================================================
m = size(u,1);
T = size(u,2);
u = u(:,1:floor(T/L)*L);

U = reshape(u,m*L,floor(T/L));

end

