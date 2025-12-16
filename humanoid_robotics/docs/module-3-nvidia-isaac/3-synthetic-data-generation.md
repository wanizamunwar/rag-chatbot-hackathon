---
title: Synthetic Data Generation
---

# 🏭 Synthetic Data Generation

One of the most transformative features of modern simulators like Isaac Sim is the ability to generate massive, perfectly labeled datasets for training AI models. This process, known as **Synthetic Data Generation (SDG)**, can drastically reduce the cost and time associated with creating high-quality training data.

### The Data Bottleneck

In supervised learning, the performance of a model is fundamentally limited by the quality and quantity of its training data. Manually collecting and labeling data (e.g., drawing bounding boxes around thousands of objects in images) is:
-   **Slow and expensive:** It requires immense human effort.
-   **Prone to error:** Human labelers make mistakes.
-   **Difficult for some data types:** It is nearly impossible to get pixel-perfect labels for tasks like semantic segmentation or to capture accurate depth data for every pixel in a scene.

### How SDG Solves This

Because the simulator has perfect, ground-truth knowledge of the entire scene, it can generate perfect labels automatically.

With Isaac Sim's SDG tools, you can output a variety of data types, including:
-   **RGB Images:** The standard color image from a camera.
-   **Depth Maps:** A grayscale image where each pixel's intensity represents its distance from the camera.
-   **Semantic Segmentation:** An image where each pixel is colored according to the *class* of object it belongs to (e.g., all chairs are blue, all tables are green).
-   **Instance Segmentation:** An image where each pixel is colored according to the specific *instance* of an object (e.g., chair #1 is blue, chair #2 is green).
-   **Bounding Boxes:** 2D or 3D coordinates defining a box around each object of interest.

```python
# Conceptual Python script for controlling a data generation scenario
# This script would be run within the Isaac Sim environment

from omni.isaac.synthetic_utils import SyntheticDataHelper

sd_helper = SyntheticDataHelper()

# Example of how you might enable specific data types
gt = {
    "rgb": True,
    "depth": True,
    "semantic_segmentation": True,
    "instance_segmentation": True,
    "bounding_box_2d_tight": True,
}
sd_helper.initialize(gt)

# Your simulation logic here...
# Move the camera, change lighting, move objects, etc.

# Capture one frame of synthetic data
sd_helper.get_groundtruth(["rgb", "depth", "semantic_segmentation"])
```

By scripting these data generation scenarios, you can create millions of diverse, perfectly labeled images in a fraction of the time it would take to collect and label real-world data, giving you a massive head start in training robust perception models.
