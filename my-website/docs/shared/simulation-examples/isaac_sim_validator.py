#!/usr/bin/env python3
"""
Isaac Sim Validation Script
This script validates Isaac Sim configurations and synthetic data generation
"""

import json
import os
from pathlib import Path


class IsaacSimValidator:
    def __init__(self):
        """
        Initialize the Isaac Sim validator
        """
        self.validation_results = []

    def validate_environment_config(self, config_path):
        """
        Validate an Isaac Sim environment configuration file
        """
        print(f"Validating environment config: {config_path}")

        try:
            with open(config_path, 'r') as f:
                config = json.load(f)

            # Check required fields
            required_fields = ['name', 'description', 'assets', 'lighting']
            for field in required_fields:
                if field not in config:
                    print(f"ERROR: Missing required field '{field}' in {config_path}")
                    return False

            # Validate assets
            assets = config.get('assets', [])
            for asset in assets:
                if 'name' not in asset or 'type' not in asset:
                    print(f"ERROR: Asset missing required fields in {config_path}")
                    return False

            # Validate lighting
            lighting = config.get('lighting', {})
            if 'type' not in lighting:
                print(f"ERROR: Lighting missing type in {config_path}")
                return False

            print(f"SUCCESS: Environment config {config_path} is valid")
            return True

        except json.JSONDecodeError:
            print(f"ERROR: Invalid JSON in {config_path}")
            return False
        except Exception as e:
            print(f"ERROR: Failed to validate {config_path}: {str(e)}")
            return False

    def validate_synthetic_data(self, dataset_path):
        """
        Validate synthetic data generation results
        """
        print(f"Validating synthetic dataset: {dataset_path}")

        if not os.path.exists(dataset_path):
            print(f"ERROR: Dataset path does not exist: {dataset_path}")
            return False

        # Check for expected subdirectories
        expected_dirs = ['rgb', 'depth', 'segmentation', 'labels']
        for dir_name in expected_dirs:
            dir_path = os.path.join(dataset_path, dir_name)
            if not os.path.exists(dir_path):
                print(f"WARNING: Expected directory missing: {dir_path}")

        # Count files in dataset
        rgb_dir = os.path.join(dataset_path, 'rgb')
        if os.path.exists(rgb_dir):
            rgb_files = [f for f in os.listdir(rgb_dir) if f.endswith(('.png', '.jpg', '.jpeg'))]
            print(f"Found {len(rgb_files)} RGB images in dataset")

        print(f"Dataset validation completed for: {dataset_path}")
        return True

    def run_complete_validation(self, project_path):
        """
        Run complete validation on an Isaac Sim project
        """
        print(f"Starting complete validation for project: {project_path}")

        # Validate environment configs
        config_dir = os.path.join(project_path, 'configs')
        if os.path.exists(config_dir):
            for file_name in os.listdir(config_dir):
                if file_name.endswith('.json'):
                    config_path = os.path.join(config_dir, file_name)
                    self.validate_environment_config(config_path)

        # Validate datasets
        dataset_dir = os.path.join(project_path, 'datasets')
        if os.path.exists(dataset_dir):
            for dataset_name in os.listdir(dataset_dir):
                dataset_path = os.path.join(dataset_dir, dataset_name)
                if os.path.isdir(dataset_path):
                    self.validate_synthetic_data(dataset_path)

        print("Complete validation finished")


def main():
    """
    Main function to demonstrate Isaac Sim validation
    """
    print("Starting Isaac Sim Validation Example")

    # Initialize the validator
    validator = IsaacSimValidator()

    # Example validation (using a mock path)
    example_config = "~/isaac_workspace/projects/basic_office.json"
    print(f"Example config validation would run on: {example_config}")

    # In a real implementation, you would validate actual project files
    # validator.run_complete_validation("~/isaac_workspace/projects/basic_office")


if __name__ == "__main__":
    main()