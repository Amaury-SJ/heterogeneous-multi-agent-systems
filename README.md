[![Read the Docs](https://img.shields.io/readthedocs/hmas?logo=readthedocs&logoColor=8CA1AF)](https://hmas.readthedocs.io)
[![HAL](https://img.shields.io/badge/HAL-reference-B03532?logo=HAL&logoColor=B03532)](https://hal.science/hal-04311426)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=22314E)](https://docs.ros.org/en/humble/)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-A42E2B.svg?logo=gnu&logoColor=A42E2B)](https://www.gnu.org/licenses/gpl-3.0)
[![Cite](https://img.shields.io/badge/Cite-CFF-yellow?logo=academia)](./CITATION.cff)

# Heterogeneous Multi Agent Systems

Codes and documentation for a Heterogeneous Multi-Agent System (HMAS), developed as part of a PhD thesis.

The project, key concepts, and source code are **documented at the following link**: [https://hmas.readthedocs.io/](https://hmas.readthedocs.io/)

**Summary**: This project aims to design a generic, decentralized, and interoperable system via the ROS 2 middleware, enabling autonomous missions in complex real-world environments, with interaction and collaboration between robots and human operators. Two original approaches to real-time 3D localization are proposed: a RTK GNSS solution for outdoor use and the NAPS nomadic system for indoor use, both offering robust centimeter-level accuracy and suitable for multi-agent systems. A role-based trust model is also introduced to monitor agent behaviors, detect anomalies, and enhance system resilience. All these contributions, integrated into ROS 2, were tested on a real HMAS combining ground and aerial mobile robots.

**Keywords**: Multi-Agent, Networked mobile robots, Cyber-Physical Systems, ROS 2 middleware, Indoor-Outdoor 3D Localization, Embedded Systems.

## Installation and get started

If you are familiar with ROS 2, you can directly copy the ROS 2 packages contained in the `codes/src` folder and paste them into the `src` directory of your own ROS 2 workspace.

## Citation

If you use this project in your research, please cite it as follows 

> Your Name (2025). Project Name (Version 1.0.0) [Computer software]. Available at https://github.com/your-username/project-name

For more information, see the [CITATION.cff](CITATION.cff) file in the repository.

## License

This project is licensed under the **GNU General Public License v3.0 or later (GPL-3.0-or-later)**. You can freely use, modify, and distribute the code, subject to the terms of this license. For more details, see the [LICENSE](./LICENSE) file.