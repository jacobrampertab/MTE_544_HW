function P_carrot = calculate_carrot(pstart, pend, probot, r)
% start is the x-y location of the start position
% end is the x-y location of the end position
% robot is the x-y location of the robot position
% r is the distance ahead on the line that is desired to track

p1_to_x = probot - pstart;
line_segment = pend - pstart;
line_segment_unit = line_segment/norm(line_segment);
projection = dot(line_segment,p1_to_x) / norm(line_segment)^2 * line_segment;

P_strict_carrot = pstart + projection + r * line_segment_unit;

x_to_p2 = pend - probot;
if(sqrt(sum(x_to_p2.*x_to_p2)) < r)
    P_carrot = pend;
else
    P_carrot = P_strict_carrot;
end