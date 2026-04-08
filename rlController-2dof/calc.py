import numpy as np

def MonteCarloTest(controller):
    N = 50  # number of trials

    results = []

    for i in range(N):
        # --- Randomise conditions ---
        params = {
            "z0": np.random.uniform(-2.0, -0.5),
            "roll0": np.random.uniform(-5, 5) * np.pi/180,
            "pitch0": np.random.uniform(-5, 5) * np.pi/180,
            "yaw0": np.random.uniform(-10, 10) * np.pi/180,
            "wind": np.random.uniform(8, 16),
            "lift_scale": np.random.normal(1.0, 0.1),
            "drag_scale": np.random.normal(1.0, 0.1),
            "delay": np.random.uniform(0.0, 0.3),
        }

        # --- Run simulation ---
        sim_data = run_evaluation(controller, params)

        # --- Compute metrics ---
        z = sim_data.F[:, 0]
        target = -1.3

        rmse = np.sqrt(np.mean((z - target)**2))

        # Failure condition
        failed = np.any(np.abs(z - target) > 0.5)

        results.append({
            "rmse": rmse,
            "failed": failed
        })