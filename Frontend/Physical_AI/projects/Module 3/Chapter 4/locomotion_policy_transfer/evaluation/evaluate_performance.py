#!/usr/bin/env python3
"""
Performance Evaluation Script for Sim-to-Real Policy Transfer

This script evaluates the performance of a transferred policy on both
simulation and real robot platforms to assess the success of the transfer.
"""

import numpy as np
import matplotlib.pyplot as plt
import json
import pickle
from typing import Dict, List, Tuple, Any
import argparse
import logging
from pathlib import Path
import time


class PolicyEvaluator:
    """
    Evaluates policy performance across simulation and real robot platforms
    """

    def __init__(self, config_path: str):
        """
        Initialize the policy evaluator with configuration
        """
        self.config_path = config_path
        self.config = self._load_config()

        # Set up logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Initialize evaluation metrics
        self.sim_metrics = []
        self.real_metrics = []

        self.logger.info("Policy Evaluator initialized")

    def _load_config(self) -> Dict:
        """
        Load the evaluation configuration from JSON file
        """
        with open(self.config_path, 'r') as f:
            config = json.load(f)

        self.logger.info(f"Evaluation configuration loaded from {self.config_path}")
        return config

    def evaluate_simulation_performance(self, num_episodes: int = 100) -> Dict[str, Any]:
        """
        Evaluate policy performance in simulation environment
        """
        self.logger.info(f"Evaluating policy performance in simulation over {num_episodes} episodes...")

        episode_rewards = []
        success_rates = []
        episode_lengths = []

        # Simulate policy performance in Isaac Sim environment
        for episode in range(num_episodes):
            # In a real implementation, this would run the policy in Isaac Sim
            # For this example, we'll generate synthetic results based on config
            episode_reward = np.random.normal(
                loc=self.config['performance_metrics']['sim_average_reward'],
                scale=50.0
            )
            success_rate = np.random.uniform(
                low=0.90,
                high=1.0
            )
            episode_length = np.random.normal(
                loc=950,
                scale=50
            )

            episode_rewards.append(episode_reward)
            success_rates.append(success_rate)
            episode_lengths.append(episode_length)

            if episode % 20 == 0:
                self.logger.info(f"Simulation evaluation: Episode {episode}/{num_episodes}")

        # Calculate metrics
        avg_reward = np.mean(episode_rewards)
        std_reward = np.std(episode_rewards)
        avg_success_rate = np.mean(success_rates)
        avg_episode_length = np.mean(episode_lengths)

        sim_metrics = {
            'avg_reward': avg_reward,
            'std_reward': std_reward,
            'avg_success_rate': avg_success_rate,
            'avg_episode_length': avg_episode_length,
            'total_episodes': num_episodes,
            'rewards': episode_rewards,
            'success_rates': success_rates
        }

        self.sim_metrics = sim_metrics
        self.logger.info(f"Simulation evaluation completed: avg_reward={avg_reward:.2f}, success_rate={avg_success_rate:.3f}")

        return sim_metrics

    def evaluate_real_robot_performance(self, num_episodes: int = 50) -> Dict[str, Any]:
        """
        Evaluate policy performance on real robot
        """
        self.logger.info(f"Evaluating policy performance on real robot over {num_episodes} episodes...")

        episode_rewards = []
        success_rates = []
        episode_lengths = []

        # Simulate policy performance on real robot (in a real implementation,
        # this would interface with the actual robot)
        for episode in range(num_episodes):
            # In a real implementation, this would run the policy on the real robot
            # For this example, we'll generate synthetic results with some degradation
            episode_reward = np.random.normal(
                loc=self.config['performance_metrics']['sim_average_reward'] * 0.8,  # 20% degradation
                scale=75.0
            )
            success_rate = np.random.uniform(
                low=0.65,
                high=0.85
            )
            episode_length = np.random.normal(
                loc=800,
                scale=100
            )

            episode_rewards.append(episode_reward)
            success_rates.append(success_rate)
            episode_lengths.append(episode_length)

            if episode % 10 == 0:
                self.logger.info(f"Real robot evaluation: Episode {episode}/{num_episodes}")

        # Calculate metrics
        avg_reward = np.mean(episode_rewards)
        std_reward = np.std(episode_rewards)
        avg_success_rate = np.mean(success_rates)
        avg_episode_length = np.mean(episode_lengths)

        real_metrics = {
            'avg_reward': avg_reward,
            'std_reward': std_reward,
            'avg_success_rate': avg_success_rate,
            'avg_episode_length': avg_episode_length,
            'total_episodes': num_episodes,
            'rewards': episode_rewards,
            'success_rates': success_rates
        }

        self.real_metrics = real_metrics
        self.logger.info(f"Real robot evaluation completed: avg_reward={avg_reward:.2f}, success_rate={avg_success_rate:.3f}")

        return real_metrics

    def calculate_transfer_success_metrics(self) -> Dict[str, float]:
        """
        Calculate metrics that indicate the success of sim-to-real transfer
        """
        if not self.sim_metrics or not self.real_metrics:
            raise ValueError("Both simulation and real robot metrics must be evaluated first")

        # Calculate transfer success metrics
        reward_transfer_ratio = self.real_metrics['avg_reward'] / self.sim_metrics['avg_reward']
        success_rate_transfer_ratio = self.real_metrics['avg_success_rate'] / self.sim_metrics['avg_success_rate']
        degradation_percentage = (1 - reward_transfer_ratio) * 100

        # Calculate normalized transfer score (0-1 scale, where 1 is perfect transfer)
        normalized_score = max(0, min(1, reward_transfer_ratio))

        transfer_metrics = {
            'reward_transfer_ratio': reward_transfer_ratio,
            'success_rate_transfer_ratio': success_rate_transfer_ratio,
            'degradation_percentage': degradation_percentage,
            'normalized_transfer_score': normalized_score,
            'sim_avg_reward': self.sim_metrics['avg_reward'],
            'real_avg_reward': self.real_metrics['avg_reward'],
            'sim_success_rate': self.sim_metrics['avg_success_rate'],
            'real_success_rate': self.real_metrics['avg_success_rate']
        }

        self.logger.info(f"Transfer success metrics: {transfer_metrics}")

        return transfer_metrics

    def generate_evaluation_report(self, output_path: str):
        """
        Generate a comprehensive evaluation report
        """
        if not self.sim_metrics or not self.real_metrics:
            raise ValueError("Both simulation and real robot metrics must be evaluated first")

        # Calculate transfer metrics
        transfer_metrics = self.calculate_transfer_success_metrics()

        # Create report
        report = {
            'evaluation_timestamp': str(np.datetime64('now')),
            'config_used': self.config_path,
            'simulation_metrics': self.sim_metrics,
            'real_robot_metrics': self.real_metrics,
            'transfer_success_metrics': transfer_metrics,
            'transfer_quality_assessment': self._assess_transfer_quality(transfer_metrics)
        }

        # Save report
        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2)

        self.logger.info(f"Evaluation report saved to {output_path}")

    def _assess_transfer_quality(self, transfer_metrics: Dict) -> str:
        """
        Assess the overall quality of the sim-to-real transfer
        """
        score = transfer_metrics['normalized_transfer_score']

        if score >= 0.85:
            return "EXCELLENT - Minimal performance degradation, successful transfer"
        elif score >= 0.70:
            return "GOOD - Acceptable performance degradation, successful transfer"
        elif score >= 0.50:
            return "FAIR - Significant performance degradation, transfer possible but requires improvement"
        else:
            return "POOR - Severe performance degradation, transfer unsuccessful"

    def plot_performance_comparison(self, output_path: str):
        """
        Plot performance comparison between simulation and real robot
        """
        if not self.sim_metrics or not self.real_metrics:
            raise ValueError("Both simulation and real robot metrics must be evaluated first")

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Sim-to-Real Policy Transfer Performance Comparison')

        # Plot 1: Reward comparison
        axes[0, 0].bar(['Simulation', 'Real Robot'],
                      [self.sim_metrics['avg_reward'], self.real_metrics['avg_reward']],
                      yerr=[self.sim_metrics['std_reward'], self.real_metrics['std_reward']],
                      capsize=5)
        axes[0, 0].set_title('Average Reward Comparison')
        axes[0, 0].set_ylabel('Reward')

        # Plot 2: Success rate comparison
        axes[0, 1].bar(['Simulation', 'Real Robot'],
                      [self.sim_metrics['avg_success_rate'], self.real_metrics['avg_success_rate']],
                      color=['blue', 'orange'])
        axes[0, 1].set_title('Success Rate Comparison')
        axes[0, 1].set_ylabel('Success Rate')
        axes[0, 1].set_ylim(0, 1)

        # Plot 3: Reward distribution comparison
        axes[1, 0].hist([self.sim_metrics['rewards'], self.real_metrics['rewards']],
                       bins=20, label=['Simulation', 'Real Robot'], alpha=0.7)
        axes[1, 0].set_title('Reward Distribution Comparison')
        axes[1, 0].set_xlabel('Reward')
        axes[1, 0].set_ylabel('Frequency')
        axes[1, 0].legend()

        # Plot 4: Episode length comparison
        axes[1, 1].bar(['Simulation', 'Real Robot'],
                      [self.sim_metrics['avg_episode_length'], self.real_metrics['avg_episode_length']],
                      color=['blue', 'orange'])
        axes[1, 1].set_title('Average Episode Length Comparison')
        axes[1, 1].set_ylabel('Episode Length')

        plt.tight_layout()
        plt.savefig(output_path)
        plt.close()

        self.logger.info(f"Performance comparison plot saved to {output_path}")

    def run_complete_evaluation(self, output_dir: str):
        """
        Run the complete evaluation process
        """
        # Create output directory if it doesn't exist
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        self.logger.info("Starting complete evaluation process...")

        # Evaluate simulation performance
        sim_metrics = self.evaluate_simulation_performance(num_episodes=100)

        # Evaluate real robot performance
        real_metrics = self.evaluate_real_robot_performance(num_episodes=50)

        # Generate evaluation report
        report_path = Path(output_dir) / "evaluation_report.json"
        self.generate_evaluation_report(str(report_path))

        # Generate performance comparison plot
        plot_path = Path(output_dir) / "performance_comparison.png"
        self.plot_performance_comparison(str(plot_path))

        self.logger.info("Complete evaluation process finished")


def main():
    parser = argparse.ArgumentParser(description='Evaluate sim-to-real policy transfer performance')
    parser.add_argument('--config', type=str, required=True,
                        help='Path to policy configuration JSON file')
    parser.add_argument('--output-dir', type=str, required=True,
                        help='Directory to save evaluation results')

    args = parser.parse_args()

    # Initialize evaluator
    evaluator = PolicyEvaluator(args.config)

    # Run complete evaluation
    evaluator.run_complete_evaluation(args.output_dir)

    print("Policy evaluation completed successfully!")


if __name__ == "__main__":
    main()