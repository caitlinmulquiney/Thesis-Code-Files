from stable_baselines3 import PPO
from delayEnv import HydrofoilEnv
import numpy as np
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

eval_env = DummyVecEnv([lambda: HydrofoilEnv()])
eval_env = VecNormalize.load("ppo_hydrofoil_4dof_delay_prob_2.pkl", eval_env)
eval_env.training = False
eval_env.norm_reward = False

model = PPO.load("ppo_hydrofoil_4dof_delay_prob_2")

obs = eval_env.reset()
done = False
total_reward = 0

# ── Pre-allocate collectors ────────────────────────────────
t_log      = []   # (N,)
state_log  = []   # (N, 10)  raw obs: z, roll, pitch, yaw, vx, vy, vz, p, q, r
action_log = []   # (N, 7)
reward_log = []   # (N,)
vmg_log = []
vmg_e_log = []

for step in range(3000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, _ = eval_env.step(action)

    raw_obs = eval_env.get_original_obs()[0]   # (10,)
    actual_action = eval_env.envs[0].unwrapped.last_action
    action_log.append(actual_action)

    t_log.append(step * eval_env.envs[0].unwrapped.sim.dt)
    state_log.append(raw_obs)
    #action_log.append(action[0])
    reward_log.append(float(reward[0]))
    vmg_log.append(eval_env.envs[0].unwrapped.vmg)
    vmg_e_log.append(eval_env.envs[0].unwrapped.vmg_e)

    total_reward += float(reward[0])

    if step % 10 == 0:
        height = raw_obs[0]
        pitch  = np.rad2deg(raw_obs[2])
        roll   = np.rad2deg(raw_obs[1])
        yaw    = np.rad2deg(raw_obs[3])
        print(f"  Step {step:5d}: z={height:.3f}  pitch={pitch:.2f}°  roll={roll:.3f}°  yaw={yaw:.3f}°")

    if terminated[0]:
        break

    path = f"ppo_delay_wind_variation.npz"
    np.savez(path,
        t      = np.array(t_log),
        states = np.array(state_log),
        actions = np.column_stack([np.array(t_log), np.array(action_log)]),
        rewards= np.array(reward_log),
        vmg_log = np.column_stack([np.array(t_log), np.array(vmg_log)]),
        vmg_e_log = np.column_stack([np.array(t_log), np.array(vmg_e_log)])
    )

eval_env.close()