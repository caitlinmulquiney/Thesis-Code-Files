from stable_baselines3 import PPO
from env import HydrofoilEnv
import numpy as np
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

eval_env = DummyVecEnv([lambda: HydrofoilEnv()])
eval_env = VecNormalize.load("vecnormalize_stats_4dof_base.pkl", eval_env)
eval_env.training = False
eval_env.norm_reward = False

model = PPO.load("ppo_hydrofoil_4dof_base")

obs = eval_env.reset()
done = False
total_reward = 0

# ── Pre-allocate collectors ────────────────────────────────
t_log      = []   # (N,)
state_log  = []   # (N, 10)  raw obs: z, roll, pitch, yaw, vx, vy, vz, p, q, r
action_log = []   # (N, 7)
reward_log = []   # (N,)

for step in range(12000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, _ = eval_env.step(action)

    raw_obs = eval_env.get_original_obs()[0]   # (10,)

    t_log.append(step * eval_env.envs[0].unwrapped.sim.dt)
    state_log.append(raw_obs)
    action_log.append(action[0])
    reward_log.append(float(reward[0]))

    total_reward += float(reward[0])

    if step % 10 == 0:
        height = raw_obs[0]
        pitch  = np.rad2deg(raw_obs[2])
        roll   = np.rad2deg(raw_obs[1])
        yaw    = np.rad2deg(raw_obs[3])
        print(f"  Step {step:5d}: z={height:.3f}  pitch={pitch:.2f}°  roll={roll:.3f}°  yaw={yaw:.3f}°")

    if terminated[0]:
        break

    # ── Save episode to .npz ───────────────────────────────────
    path = f"ppo_base.npz"
    np.savez(path,
        t      = np.array(t_log),
        states = np.array(state_log),
        actions = np.column_stack([np.array(t_log), np.array(action_log)]),
        rewards= np.array(reward_log),
    )

eval_env.close()