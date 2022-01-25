^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclc
^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.6 (2022-01-25)
------------------
* [backport galactic, foxy] data_available optimization (#212)
* Fix data_available reset for timer (#215) (#216)
* Executor ignore canceled timers (#220) (#221)
* Resolved error in unit test see issue #230 (#231) (#232)
* Updated documentation README.md (#229)

2.0.5 (2021-11-08)
------------------
* Fix printf in executor spin
* Fix init options handling

2.0.4 (2021-08-19)
------------------
* Refactoring: remove callback_type
* Improvement: Checking for valid ROS context in spin_some
* Bug fix: Ignoring unsuccessful SERVICE_TAKE
* Bug fix: Updated ci workflow dependency on galactic
* Improvement: Updated codecov configuration to ignore unit tests

2.0.3 (2021-07-26)
------------------
* Updated codecov to ignore test folders
* Updated bloom release status table

2.0.2 (2021-07-17)
------------------
* Added rclc_parameter package
* Added quality of service entity creation API
* Addded executor_prepare API
* Added support for removing subscription from executor
* Added support for subscription with context
* Updated compatability function for sleep
* Removed duplicate NOTICE files

2.0.1 (2021-05-28)
------------------
* added quality declaration

2.0.0 (2021-04-23)
------------------
* added codecov support
* new API of rcl_lifecycle in Rolling required major version bump

1.0.1 (2021-03-29)
------------------
* Windows port
* Compatibility sleep function (Windows, POSIX-OS)
* Fixed RCL lifecycle API change for Rolling

1.0.0 (2021-03-04)
------------------
* service callbacks with context
* fixed minor issues unit tests
* upgraded setup_ros action (ci jobs)
* removed Eloquent from ci jobs

0.1.7 (2021-01-20)
------------------
* Corrected corrupted changelog

0.1.6 (2021-01-20)
------------------
* Fixed issues due to Github Action timing delays on cloud build
* Fixed missing package dependency in Github Action for Eloquent

0.1.5 (2020-12-11)
------------------
* Added support for services,clients and guard_conditions to rclc executor
* Added table for bloom release status in README file

0.1.4 (2020-11-25)
------------------
* Fixed error in bloom release

0.1.3 (2020-11-23)
------------------
* Added rclc_lifecycle package
* Change maintainer information
* Minor fixes, updated unit tests

0.1.2 (2020-05-19)
------------------
* Fixed compiler errors for bloom release

0.1.1 (2020-05-14)
------------------
* Initial release