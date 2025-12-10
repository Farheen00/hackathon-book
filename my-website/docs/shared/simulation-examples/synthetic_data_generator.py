#!/usr/bin/env python3
"""
Synthetic Data Generator for Isaac Sim
This script demonstrates how to generate synthetic datasets for AI model training
"""

import omni
from omni.isaac.synthetic_utils import SyntheticDataHelper
import numpy as np
import os


class SyntheticDataGenerator:
    def __init__(self, resolution=(640, 480)):
        """
        Initialize the synthetic data generator
        """
        self.resolution = resolution
        self.synthetic_data = SyntheticDataHelper()
        self.synthetic_data.set_resolution(*resolution)

        # Enable different output types
        self.synthetic_data.enable_rgb_output()
        self.synthetic_data.enable_depth_output()
        self.synthetic_data.enable_segmentation_output()
        self.synthetic_data.enable_normals_output()

    def generate_dataset(self, num_samples=100, environment_config="basic_office.json", output_dir="~/isaac_workspace/datasets"):
        """
        Generate a synthetic dataset with the specified parameters
        """
        print(f"Generating {num_samples} samples for dataset")
        print(f"Environment config: {environment_config}")
        print(f"Output directory: {output_dir}")

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        dataset_info = {
            "name": f"synthetic_dataset_{num_samples}samples",
            "num_samples": num_samples,
            "resolution": self.resolution,
            "output_types": ["rgb", "depth", "segmentation", "normals"],
            "environment_config": environment_config
        }

        print(f"Dataset info: {dataset_info}")

        # Simulate the data generation process
        for i in range(num_samples):
            if i % 20 == 0:
                print(f"Generated {i}/{num_samples} samples...")

            # In a real implementation, this would capture data from Isaac Sim
            # For this example, we'll just simulate the process
            pass

        print(f"Completed generation of {num_samples} samples")
        return dataset_info


def main():
    """
    Main function to demonstrate synthetic data generation
    """
    print("Starting Isaac Sim Synthetic Data Generation Example")

    # Initialize the generator
    generator = SyntheticDataGenerator(resolution=(640, 480))

    # Generate a dataset
    dataset_info = generator.generate_dataset(
        num_samples=100,
        environment_config="basic_office.json",
        output_dir="~/isaac_workspace/datasets/basic_office_dataset"
    )

    print(f"Dataset generated successfully: {dataset_info}")


if __name__ == "__main__":
    main()