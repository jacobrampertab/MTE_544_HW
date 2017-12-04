function collisionDetected = checkCollisionsOG(p1, p2, map)
% p1 is the first point of the line
% p2 is the second point of the line
% map is the occupancy grid to check against

collisionDetected = 0;

[x,y] = bresenham(p1(1), p1(2), p2(1), p2(2));
for i=1:length(x)
   if(map(x(i), y(i)) == 1)
       collisionDetected = 1;
       break;
   end
end