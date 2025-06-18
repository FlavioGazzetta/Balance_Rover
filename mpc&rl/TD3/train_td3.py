import numpy as np
import torch
import gym
import argparse
import os
import custom_env2  # Ensure this is correctly imported

import utils
import TD3


def flatten_state(state):
    """Utility function to flatten nested states into a single 1D array."""
    if isinstance(state, dict):
        flattened_values = [flatten_state(v) for v in state.values() if v is not None and len(v) > 0]
        if not flattened_values:
            raise ValueError("State dictionary contains no valid elements to flatten.")
        return np.concatenate(flattened_values)
    elif isinstance(state, (list, tuple)):
        flattened_values = [flatten_state(s) for s in state if s is not None and len(s) > 0]
        if not flattened_values:
            raise ValueError("State list/tuple contains no valid elements to flatten.")
        return np.concatenate(flattened_values)
    elif isinstance(state, np.ndarray):
        return state.flatten()
    else:
        return np.array(state).flatten()


def eval_policy(policy, env, seed, eval_episodes=10):
    avg_reward = 0.0
    for _ in range(eval_episodes):
        print("Resetting environment in eval_policy...")
        state = env.reset()

        # Flatten the state to ensure it is a 1D Numpy array
        state = flatten_state(state).astype(np.float32)
        done = False

        while not done:
            # Pass the flattened state array to the policy's select_action method
            action = policy.select_action(state)

            # Step the environment and unpack the new API's return values
            next_state, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

            # Flatten the new state after each step
            state = flatten_state(next_state).astype(np.float32)

            avg_reward += reward

    avg_reward /= eval_episodes

    print("---------------------------------------")
    print(f"Evaluation over {eval_episodes} episodes: {avg_reward:.3f}")
    print("---------------------------------------")

    return avg_reward


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", default="TD3")  # Policy name (TD3)
    parser.add_argument("--env", default="CustomEnv2-v0")  # Custom Environment ID
    parser.add_argument("--seed", default=0, type=int)  # Sets Gym, PyTorch and Numpy seeds
    parser.add_argument("--start_timesteps", default=25e3, type=int)  # Time steps initial random policy is used
    parser.add_argument("--eval_freq", default=5e3, type=int)  # How often (time steps) we evaluate
    parser.add_argument("--max_timesteps", default=1e6, type=int)  # Max time steps to run environment
    parser.add_argument("--expl_noise", default=0.1, type=float)  # Std of Gaussian exploration noise
    parser.add_argument("--batch_size", default=256, type=int)  # Batch size for both actor and critic
    parser.add_argument("--discount", default=0.99, type=float)  # Discount factor
    parser.add_argument("--tau", default=0.005, type=float)  # Target network update rate
    parser.add_argument("--policy_noise", default=0.2, type=float)  # Noise added to target policy during critic update
    parser.add_argument("--noise_clip", default=0.5, type=float)  # Range to clip target policy noise
    parser.add_argument("--policy_freq", default=2, type=int)  # Frequency of delayed policy updates
    parser.add_argument("--save_model", action="store_true")  # Save model and optimizer parameters
    parser.add_argument("--load_model", default="")  # Model load file name, "" doesn't load, "default" uses file_name
    args = parser.parse_args()

    file_name = f"{args.policy}_{args.env}_{args.seed}"
    print("---------------------------------------")
    print(f"Policy: {args.policy}, Env: {args.env}, Seed: {args.seed}")
    print("---------------------------------------")

    if not os.path.exists("./results"):
        os.makedirs("./results")

    if args.save_model and not os.path.exists("./models"):
        os.makedirs("./models")

    # Initialize the environment
    env = gym.make(args.env)

    # Set seeds
    env.reset(seed=args.seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    # Extract environment information
    state_sample = env.reset()
    flattened_state = flatten_state(state_sample)
    state_dim = flattened_state.shape[0]
    action_dim = env.action_space.shape[0]
    max_action = float(env.action_space.high[0])

    # Prepare TD3 parameters
    kwargs = {
        "state_dim": state_dim,
        "action_dim": action_dim,
        "max_action": max_action,
        "discount": args.discount,
        "tau": args.tau,
    }

    # Initialize policy
    if args.policy == "TD3":
        # Target policy smoothing is scaled wrt the action scale
        kwargs["policy_noise"] = args.policy_noise * max_action
        kwargs["noise_clip"] = args.noise_clip * max_action
        kwargs["policy_freq"] = args.policy_freq
        policy = TD3.TD3(**kwargs)
    else:
        raise ValueError(f"Unsupported policy: {args.policy}")

    if args.load_model != "":
        policy_file = file_name if args.load_model == "default" else args.load_model
        policy.load(f"./models/{policy_file}")

    replay_buffer = utils.ReplayBuffer(state_dim, action_dim)

    print("Starting evaluation of untrained policy...")
    evaluations = [eval_policy(policy, env, args.seed)]

    print("Starting training loop...")
    state = env.reset()
    state = flatten_state(state).astype(np.float32)
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0
    done = False

    for t in range(1, int(args.max_timesteps) + 1):
        episode_timesteps += 1

        # Select action randomly or according to policy
        if t < args.start_timesteps:
            action = env.action_space.sample()
        else:
            action = (
                policy.select_action(state)
                + np.random.normal(0, max_action * args.expl_noise, size=action_dim)
            ).clip(-max_action, max_action)

        # Perform action
        next_state, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated

        # Flatten the next state
        next_state_flat = flatten_state(next_state).astype(np.float32)

        # Determine whether episode is done
        done_bool = float(done)

        # Store data in replay buffer
        replay_buffer.add(state, action, next_state_flat, reward, done_bool)

        state = next_state_flat
        episode_reward += reward

        # Train agent after collecting sufficient data
        if t >= args.start_timesteps:
            policy.train(replay_buffer, args.batch_size)

        if done:
            print(
                f"Total Timesteps: {t} Episode Num: {episode_num + 1} Episode Timesteps: {episode_timesteps} Reward: {episode_reward:.3f}"
            )
            state = env.reset()
            state = flatten_state(state).astype(np.float32)
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1
            done = False

        # Evaluate the policy at specified intervals
        if t % args.eval_freq == 0:
            evaluations.append(eval_policy(policy, env, args.seed))
            np.save(f"./results/{file_name}", evaluations)
            if args.save_model:
                policy.save(f"./models/{file_name}")
