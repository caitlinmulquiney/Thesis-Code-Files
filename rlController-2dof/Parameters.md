Parameters to care about:
n_steps: The number of steps to run for each environment per update
        (i.e. rollout buffer size is n_steps * n_envs where n_envs is number of environment copies running in parallel)
        NOTE: n_steps * n_envs must be greater than 1 (because of the advantage normalization)
        See https://github.com/pytorch/pytorch/issues/29372
    small = high bias/low variation
    large = low bias/ high variation  
gamma: Discount factor- how far into the future the agent cares about results learning_rate: The learning rate, it can be a function of the current progress remaining (from 1 to 0)
    lower = increased stability
policy_kwargs: additional arguments to be passed to the policy on creation. See :ref:`ppo_policies`
    change network size, layers, activation function, might need to be larger to deal with mutiple states
ent_coef: Entropy coefficient for the loss calculation
    increase in this encourages exploration

ep_len_mean: Mean episode length (HIGHER IS GOOD)
ep_rew_mean: Mean episodic training reward (HIGHER IS GOOD)
approx_kl: how much policy changes (should be between 0.01-0.05)
clip_fraction: fraction that hits clip boundary (should be between 0.05-0.15)
entropy_loss: policy randomness (SHOULD INCREASE)
explained_variance: Fraction of the return variance explained by the value function, see https://scikit-learn.org/stable/modules/model_evaluation.html#explained-variance-score (ev=0 => might as well have predicted zero, ev=1 => perfect prediction, ev<0 => worse than just predicting zero) (SHOULD BE NEAR 1)
value_loss: critic prediction error (SHOULD DECREASE OVER TIME)
std: how random the policy is (SHOULD DECREASE)
