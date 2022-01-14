load('static_env_trondheim_harbour.mat');

figure(680)
clf(680)
hold on;
grid on;
axis('equal')

plot_static_obs(static_obs, 680);