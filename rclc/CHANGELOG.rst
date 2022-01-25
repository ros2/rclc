^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclc
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2022-01-25)
------------------
* Update codecov to ignore rclc_examples and all test folders (backport #145) (#150)
* Updated table of bloom releases (removed dashing, inserted galactic) (backport #147) (#152)
* Refactor #116 remove callback_type (#154)
* Fix codecov to ignore unit tests and rclc_examples package (backport #155) (#162)
* Feature request: check for valid ros context in spin_some (#165) (#167)
* Ignoring unsuccessful SERVICE_TAKE (#175) (#177)
* Backport windows port of PR #144 (#182)
* Fix: printf in executor spin (#195) (#197)
* Fix init options handling (#202) (#204)
* [backport galactic, foxy] data_available optimization (backport #212) (#213)
* Fix data_available reset for timer (#215)
* Executor ignore canceled timers (#220) (#222)
* Resolved error in unit test see issue #230 (#231) (#233)
* Updated documentation README.md (#234)

1.0.2 (2021-07-17)
------------------
* Bumped version (tag with version 1.0.1 already exists)

1.0.1 (2021-07-17)
------------------
* Added quality of service entity creation API
* Added executor prepare API
* Added support to add subscription with context to executor
* Added support for removing subscription from executor
* Updated compatability function for sleep
* Added quality declaration statement
* Used executor allocator in spin

1.0.0 (2021-03-04)
------------------
* Service callbacks with context
* Fixed minor issues unit tests
* Upgraded setup_ros action (ci jobs)
* Removed Eloquent from ci jobs

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