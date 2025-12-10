#!/usr/bin/env python3
"""
Nav2 Validation Script
This script validates Nav2 configuration and navigation performance
"""

import yaml
import os
from pathlib import Path


class Nav2Validator:
    def __init__(self):
        """
        Initialize the Nav2 validator
        """
        self.validation_results = []

    def validate_nav2_config(self, config_path):
        """
        Validate a Nav2 configuration file
        """
        print(f"Validating Nav2 config: {config_path}")

        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            # Check for required Nav2 components
            required_components = [
                'bt_navigator',
                'controller_server',
                'local_costmap',
                'global_costmap',
                'planner_server'
            ]

            for component in required_components:
                if component not in config:
                    print(f"WARNING: Missing Nav2 component '{component}' in {config_path}")

            # Validate specific parameters
            if 'bipedal_local_planner' in config:
                bipedal_config = config['bipedal_local_planner']['ros__parameters']

                required_params = ['balance_margin', 'step_size_max', 'step_height_max', 'stance_width']
                for param in required_params:
                    if param not in bipedal_config:
                        print(f"ERROR: Missing bipedal parameter '{param}' in {config_path}")
                        return False

                # Validate parameter values
                if bipedal_config.get('balance_margin', 0) <= 0:
                    print(f"ERROR: Invalid balance_margin value in {config_path}")
                    return False

            print(f"SUCCESS: Nav2 config {config_path} is valid")
            return True

        except yaml.YAMLError:
            print(f"ERROR: Invalid YAML in {config_path}")
            return False
        except Exception as e:
            print(f"ERROR: Failed to validate {config_path}: {str(e)}")
            return False

    def validate_navigation_performance(self, log_path):
        """
        Validate navigation performance from log files
        """
        print(f"Validating navigation performance: {log_path}")

        if not os.path.exists(log_path):
            print(f"ERROR: Log path does not exist: {log_path}")
            return False

        # Simulate performance validation
        # In a real implementation, this would parse navigation logs
        # and check metrics like success rate, time to goal, path efficiency
        print(f"Navigation performance validation completed for: {log_path}")
        return True

    def run_complete_validation(self, project_path):
        """
        Run complete validation on a Nav2 project
        """
        print(f"Starting complete Nav2 validation for project: {project_path}")

        # Validate configuration files
        config_dir = os.path.join(project_path, 'config')
        if os.path.exists(config_dir):
            for file_name in os.listdir(config_dir):
                if file_name.endswith('.yaml') or file_name.endswith('.yml'):
                    config_path = os.path.join(config_dir, file_name)
                    self.validate_nav2_config(config_path)

        # Validate logs
        log_dir = os.path.join(project_path, 'logs')
        if os.path.exists(log_dir):
            for log_name in os.listdir(log_dir):
                if log_name.endswith('.log'):
                    log_path = os.path.join(log_dir, log_name)
                    self.validate_navigation_performance(log_path)

        print("Complete Nav2 validation finished")


def main():
    """
    Main function to demonstrate Nav2 validation
    """
    print("Starting Nav2 Validation Example")

    # Initialize the validator
    validator = Nav2Validator()

    # Example validation (using a mock path)
    example_config = "~/isaac_workspace/projects/bipedal_nav2_config.yaml"
    print(f"Example config validation would run on: {example_config}")

    # In a real implementation, you would validate actual project files
    # validator.run_complete_validation("~/isaac_workspace/projects/bipedal_nav")


if __name__ == "__main__":
    main()