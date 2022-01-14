time_steps = settings.t_sim/settings.dt;
empty_agent = agents;

agent_data = [empty_agent; empty_agent];
while size(agent_data,1) <= time_steps/2
     agent_data= [agent_data;agent_data];
end
diff = time_steps - size(agent_data,1);
agent_data = [agent_data; agent_data(1:diff,:)];



% Clear variables that are no longer in use;
clear('empty_agents', 'diff', 'time_steps');