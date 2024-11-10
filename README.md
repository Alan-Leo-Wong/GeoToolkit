# GeoToolKit

GeoToolKit is an advanced C++ library designed to facilitate both my research and educational activities in geometric processing. It seamlessly integrates renowned geometric libraries such as [CGAL](https://github.com/CGAL/cgal), [geogram](https://github.com/BrunoLevy/geogram), and [libigl](https://github.com/libigl/libigl), providing a rich set of functionalities for implementing classical and contemporary geometric algorithms and data structures. This toolkit is specifically tailored for researchers and educators, helping to introduce fundamental and advanced concepts in geometric processing to both lower-level undergraduates and graduate students.

## Key Features

- **Half-edge Data Structure**: Essential for representing mesh edges, facilitating efficient topological manipulations.
- **Voronoi Diagrams**: Robust implementation using CGAL and geogram, ideal for teaching and research in computational geometry.
- **Power Diagrams**: Extend Voronoi diagrams with weights using CGAL and geogram, useful in a variety of geometric algorithms.
- **Apollonius Diagrams (also known as Additively Weighted Voronoi Diagrams)**: Implemented via CGAL, these diagrams are crucial for advanced studies and applications in geometric optimization.
- **Visualization**: Currently implemented with libigl, with a planned upgrade to [polyscope](https://github.com/nmwsharp/polyscope) to improve user interaction and visual engagement.

## Upcoming Features

GeoToolKit is committed to continuous growth and expansion, with plans to integrate a variety of advanced geometric functionalities:
- **Geodesics**: For computing shortest paths on surfaces.
- **Implicit Surface Reconstruction**: To create meshes from point clouds.
- **Surface Parameterization**: Techniques to unfold 3D surfaces onto 2D domains.
- **Mesh Repair**: Algorithms to correct and enhance mesh quality.
- **Discrete Differential Geometry Operators**: To support advanced analysis and operations on discrete surfaces.
- **Remeshing Techniques**: For optimizing mesh quality and structure for various applications.
- ...

## Prerequisites

To utilize GeoToolKit, ensure you have the following:
- CMake (version 3.20 or higher recommended)
- A C++ compiler supporting C++17 or higher

## Installation

1. **Clone the Repository**
   
   ```bash
   git clone https://github.com/Alan-Leo-Wong/GeoToolkit.git
   cd GeoToolKit
   ```
   
1. Build the Project
   
   ```bash
   mkdir build && cd build
   cmake ..
   cmake --build . -j your-core-num

## Contributing

We encourage contributions from the community to help expand and refine GeoToolKit. If you have enhancements or additional features to propose, please fork the repository, make your changes, and submit a pull request.
