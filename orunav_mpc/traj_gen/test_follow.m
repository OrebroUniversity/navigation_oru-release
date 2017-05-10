load trajectory_02.txt

states = trajectory_02(:, 1:4);
controls = trajectory_02(:, 5:6);
sim_states = [];
dt = 0.01;
dt_ref = 0.06;
len = 0.68;

state = states(1,:);
%controls(1,2) = (states(2,4) - states(1,4))/dt_ref;
for i = 1:size(trajectory_02,1);
    v = controls(i,1);
    w = controls(i,2);

    for j = 1:6;
        state = [
            state(1) + cos(state(3)) * v * dt
            state(2) + sin(state(3)) * v * dt
            state(3) + (tan(state(4)) * v * dt)/len
            state(4) + w * dt]';
        sim_states = [sim_states; state];
    end
end
