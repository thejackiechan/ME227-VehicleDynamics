function [ indices, curvatures ] = find_indices( path )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% initialize indices to return
indices = zeros(17,1);
curvatures = zeros(17,1);

%start on a straight
indices(1) = 1;
curvatures(1) = 0;

index = 2;

N = length(path.k_1pm);
%path.k_1pm

for i = 2:(N-2)
    if(path.k_1pm(i)== 0 && path.k_1pm(i+1) ~= 0)
        %line to clothoid
        indices(index) = i+1;
        curvatures(index) = path.k_1pm(i+1);
        index = index + 1;
    elseif (path.k_1pm(i)~= 0 && path.k_1pm(i+1) == 0)
        %clothoid to line
        indices(index) = i+1;
        curvatures(index) = path.k_1pm(i+1);
        index = index + 1;
    elseif ((path.k_1pm(i)~= 0 && path.k_1pm(i+1) ~= 0) && (path.k_1pm(i+2) == path.k_1pm(i+1))&& (path.k_1pm(i)~= path.k_1pm(i+1)) )
        %clothoid to arc
        indices(index) = i+1;
        curvatures(index) = path.k_1pm(i+1);
        index = index + 1;
        
    elseif ((path.k_1pm(i-1)- path.k_1pm(i) == 0) && path.k_1pm(i+1) ~= path.k_1pm(i))
        %arc to clothoid
        indices(index) = i+1;
        curvatures(index) = path.k_1pm(i+1);
        index = index + 1;
    end
    
    
end

end

