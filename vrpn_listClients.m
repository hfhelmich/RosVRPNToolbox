%%  vrpn_listClients.m
%   Function will scan the current rostopic list for any vrpn trackers that
%   are present and display the names of each one for easy viewing in
%   MATLAB.
%
%   Output:
%       - num - amount of trackers running
%
%
%   Harrison Helmich; 9 Aug 2022
%
function [output, num] = vrpn_listClients

    output = "";

    num = 0;
    list = rostopic("list");

    for i = 1:numel(list)
        if contains(list(i), '/vrpn_client_node/')
            num = num + 1;

            a = strjoin(list(i));
            a = strsplit(a, '/');
            output(num) = a(3);
        end
    end

    %disp(newline);
    if isempty(output(1))
        warning('No VRPN objects are visible.');
    else
        %disp('OptiTrack sees the following:');
        %for i = 1:numel(output)
        %    disp(output(i));
        %end
    end
end