# The rclc repository
This repository provides the rclc-package. It is written in C programming language and it uses the ROS Client Library (RCL). It contains an RCLC-Executor and several convenience functions to simplify the configuration of RCL-objects.

In detail, this repository contains two ROS2 packages:

- [rclc](rclc/) provides the RCLC-Executor, which provides an Executor for C applications like the [rclcpp Executor](https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/executor.hpp) for C++ but also provides new features for deterministic timing behavior. Additionally, convenience functions are provided to simplify the creation of RCL-objects.
- [rclc_examples](rclc_examples/) provides small examples for the use of the RCLC-Executor and the convenience functions.

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
