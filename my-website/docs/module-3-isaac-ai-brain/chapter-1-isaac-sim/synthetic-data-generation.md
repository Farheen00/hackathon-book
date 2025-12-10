---
sidebar_position: 2
title: Synthetic Data Generation
description: Generating synthetic datasets for AI model training in Isaac Sim
id: ch3-s2-synthetic-data
---

# Synthetic Data Generation

This section covers generating synthetic datasets for AI model training in Isaac Sim. You'll learn how to configure synthetic data generation that produces labeled datasets suitable for training perception models.

## Key Concepts

- Synthetic data pipeline configuration
- Sensor simulation and data capture
- Data labeling and annotation
- Domain randomization techniques

## Diagram Descriptions

1. **Synthetic Data Pipeline**: A flow diagram showing the process from Isaac Sim environment to synthetic dataset generation with sensor simulation and labeling.
2. **Domain Randomization**: A visual representation of how domain randomization parameters affect the generated synthetic data to improve domain transfer.

## Content

Synthetic data generation in Isaac Sim involves configuring sensors to capture data from simulation environments and automatically generating labels for that data. This approach enables training of AI models without requiring real-world data collection.

### Sensor Configuration

Isaac Sim supports various sensor types for synthetic data generation:

- RGB cameras for visual data
- Depth sensors for 3D information
- LiDAR for point cloud data
- Segmentation sensors for semantic labeling

### Data Labeling

Synthetic data generation automatically creates ground truth labels for training models:

- Semantic segmentation masks
- Instance segmentation masks
- Depth maps
- Object bounding boxes

## Example

Here's an example synthetic data generation script:

```python
# Example synthetic data generation script
import omni
from omni.isaac.synthetic_utils import SyntheticDataHelper

# Initialize synthetic data helper
synthetic_data = SyntheticDataHelper()
synthetic_data.set_resolution(640, 480)

# Configure sensor outputs
synthetic_data.enable_rgb_output()
synthetic_data.enable_depth_output()
synthetic_data.enable_segmentation_output()

# Generate dataset
dataset = synthetic_data.generate_dataset(
    num_samples=1000,
    environment_config="basic_office.json",
    output_dir="~/isaac_workspace/datasets/basic_office_dataset"
)
```