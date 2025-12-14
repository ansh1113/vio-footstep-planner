#!/usr/bin/env python3
import numpy as np
from stable_baselines3 import PPO
from quadruped_ppo import QuadrupedEnv
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True, help='Path to model')
    parser.add_argument('--episodes', type=int, default=10)
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args()
    
    model = PPO.load(args.model)
    env = QuadrupedEnv(render=args.render)
    
    rewards = []
    for ep in range(args.episodes):
        obs = env.reset()
        done = False
        ep_reward = 0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = env.step(action)
            ep_reward += reward
        rewards.append(ep_reward)
        print(f"Episode {ep+1}: {ep_reward:.2f}")
    
    print(f"\nMean reward: {np.mean(rewards):.2f} ± {np.std(rewards):.2f}")

if __name__ == "__main__":
    main()
