simDuration = 5;
fps = 60;
numFrames = simDuration * fps;

Q = readmatrix('state2.csv');

quat = Q(:, 2:5);


N = size(quat,1);

idx = round(linspace(1, N, min(numFrames, N)));

figure;
ax = axes;
grid(ax, 'on');
axis(ax, [-1 1 -1 1 -1 1]);
view(3);
title('Quaternion Orientation Animation');

% Initial pose at origin
h = poseplot(quaternion(eye(3), 'rotmat', 'frame'), [0 0 0], 'Parent', ax);

v = VideoWriter('C:\Users\abhay\Videos\WorkingCPP.avi');
v.FrameRate = fps;
open(v);

for k = 1:length(idx)
    q = quat(idx(k), :);
    % quaternion constructor in MATLAB expects [w x y z]
    q_orientation = quaternion(q);
    set(h, 'Orientation', q_orientation, 'Position', [0 0 0]);
    drawnow limitrate;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);