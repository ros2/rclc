# The rclc repository
This repository provides the rclc package, which complements the [ROS Client Support Library (rcl)](https://github.com/ros2/rcl/) to make up a complete ROS 2 client library for the C programming language. That is, rclc does not add a new layer of types on top of rcl (like rclcpp and rclpy do) but only provides convenience functions that ease the programming with the rcl types. New types are introduced only for concepts that are missing in rcl, most important an Executor.

In detail, this repository contains two packages:

- [rclc](rclc/) provides the mentioned convenience functions for creating instances of publishers, subscriptions, nodes, etc. with the corresponding [rcl types](https://github.com/ros2/rcl/tree/master/rcl/include/rcl). Furthermore, it provides the rclc Executor for C, analogously to rclcpp's [Executor class](https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/executor.hpp) for C++. A key feature compared to the rclcpp Executor is that it includes features for implementing deterministic timing behavior.
- [rclc_examples](rclc_examples/) provides small examples for the use of the convenience functions and the rclc Executor.

Technical information on the interfaces and the usage of these packages is given in the README.md files in the corresponding subfolders.

## Purpose of the project

The software is not ready for production use. It has neither been developed nor tested for a specific use case. However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs. Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust it according to any applicable safety standards (e.g. ISO 26262).

## Requirements, how to build, test and install

Source your ROS2 `distribution` with `source /opt/ros/distribution/setup.bash`. Clone the repository into a ROS2 workspace (e.g. `~/ros2_ws/`) and build the packages using `colcon build` from the [Colcon Command Line Tools](https://colcon.readthedocs.io/en/released/). To test the RCLC package run `colcon test` or if you have multiple repositories in this workspace `colcon test --packages-select rclc`. For correct installation of the `rclc`-package do a `source ~/ros2_ws/install/local_setup.bash`. Then you are ready to run the examples in the `rclc_examples` package.

## License

rclc is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open source components included in rclc, see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Quality assurance

*   Coding style:
    *   The [uncrustify](https://github.com/uncrustify/uncrustify) tool is used to check the coding style.
*   Linters:
    *   The [cpplint](https://github.com/google/styleguide/tree/gh-pages/cpplint) tool is used to detect common flaws and problems in C/C++ code.
    * The [cppcheck](http://cppcheck.sourceforge.net/) tool is used for code analysis.
    *   The CMakeLists.txt is checked with [lint_cmake](https://pypi.org/project/cmakelint/) and the package.xml with [xmllint](http://xmlsoft.org/xmllint.html)
*   Unit tests:
    *   Unit tests based on [gtest](https://github.com/google/googletest) are located in the [rclc/test](rclc/test) folder.

## Known issues/limitations

Please notice the following issues/limitations:

*   rclc package support the communication types subscriptions and timers (services, clients and guard conditions are not supported yet)
