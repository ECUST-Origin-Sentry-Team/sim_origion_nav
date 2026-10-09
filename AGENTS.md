# AGENTS Guide

This repository is a ROS 2 colcon workspace rooted here.
Use this file as the default playbook for coding agents working anywhere in the repo.

## Rule Files And Precedence
- Checked for `.cursor/rules/**`: not present.
- Checked for `.cursorrules`: not present.
- Checked for `.github/copilot-instructions.md`: not present.
- If any of those files are added later, treat them as higher-priority instructions than this file.
- No nested `AGENTS.md` was found under `src/` during this review.

## Workspace Shape
- Source packages live under `src/`.
- Generated artifacts live under `build/`, `install/`, and `log/`; do not edit them.
- The workspace is mostly `ament_cmake` packages, with these `ament_python` packages:
  - `dec_tree`
  - `rm_serial`
  - `simulated_obstacle_gui`
- Vendored or third-party code exists, especially under:
  - `src/localization/Adaptive-LIO/thirdparty/`
  - `src/localization/lightning-lm/thirdparty/`
- Avoid changing vendored code unless the task explicitly requires it.

## Environment Setup
1. Ensure ROS is sourced:
   - `export ROS_DISTRO=${ROS_DISTRO:-humble}`
   - `source /opt/ros/$ROS_DISTRO/setup.bash`
2. Install missing dependencies when needed:
   - `rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y`
3. Build and source the overlay before running nodes or Python imports:
   - `colcon build --symlink-install`
   - `source install/setup.bash`

## Build Commands
- List packages: `colcon list`
- Build the full workspace: `colcon build --symlink-install`
- Build one package: `colcon build --symlink-install --packages-select <pkg>`
- Build a package and its dependencies: `colcon build --symlink-install --packages-up-to <pkg>`
- Rebuild one package cleanly: `rm -rf build/<pkg> install/<pkg> && colcon build --symlink-install --packages-select <pkg>`
- Debug build: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug`
- RelWithDebInfo build: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
- Export compile commands: `colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

## Test Commands
- Run all tests: `colcon test --event-handlers console_direct+ --return-code-on-test-failure`
- Print all test results: `colcon test-result --all --verbose`
- Run one package test suite: `colcon test --packages-select <pkg> --event-handlers console_direct+ --return-code-on-test-failure`
- Print one package test results: `colcon test-result --verbose --test-result-base build/<pkg>`
- Reprint failures from the last run: `colcon test-result --all --verbose`

## Running A Single Test
### For `ament_cmake` / CTest-backed packages
- List registered tests: `ctest -N --test-dir build/<pkg>`
- Run tests matching a regex: `colcon test --packages-select <pkg> --ctest-args -R <regex> --output-on-failure`
- Run one exact test target: `colcon test --packages-select <pkg> --ctest-args -R '^<exact_name>$' --output-on-failure`
- Rerun only failures: `colcon test --packages-select <pkg> --ctest-args --rerun-failed --output-on-failure`

### For gtest binaries
- Preferred path: run through CTest first.
- Example: `colcon test --packages-select adaptive_lio --ctest-args -R test_scantext --output-on-failure`
- If needed, run the binary directly after building:
  - `./build/<pkg>/<gtest_binary> --gtest_filter=<Suite>.<Case>`
- Known working repo example:
  - `./build/adaptive_lio/test_scantext --gtest_filter=ScanContextTest.DescriptorGeneration`

### For `ament_python` packages
- Run the package suite through colcon:
  - `colcon test --packages-select <pkg> --event-handlers console_direct+ --return-code-on-test-failure`
- Run one test file directly:
  - `python3 -m pytest -q src/<path_to_pkg>/test/test_<name>.py`
- Run a subset by expression:
  - `python3 -m pytest -q src/<path_to_pkg>/test/test_<name>.py -k <expr>`
- Known working repo examples:
  - `python3 -m pytest -q src/decision/dec_tree/test/test_flake8.py`
  - `python3 -m pytest -q src/serial/rm_serial/test/test_pep257.py`

## Lint And Formatting
- Many CMake packages register lint under `if(BUILD_TESTING)` using `ament_lint_auto_find_test_dependencies()`.
- For those packages, lint usually runs as part of `colcon test`.
- Python packages in this repo explicitly depend on:
  - `ament_flake8`
  - `ament_pep257`
  - `ament_copyright`
  - `python3-pytest`
- Useful package-scoped lint flow: `colcon test --packages-select <pkg> --event-handlers console_direct+ --return-code-on-test-failure` then `colcon test-result --verbose --test-result-base build/<pkg>`
- Pre-commit configs exist in:
  - `src/localization/pcd2pgm/.pre-commit-config.yaml`
  - `src/navigation/pb_omni_pid_pursuit_controller/.pre-commit-config.yaml`
- Run pre-commit from the workspace root when relevant: `pre-commit run -a`
- C++ formatting configs exist in:
  - `src/localization/lightning-lm/.clang-format`
  - `src/lidar/livox_ros_driver2/.clang-format`
  - `src/navigation/pb_omni_pid_pursuit_controller/.clang-format`
  - `src/localization/pcd2pgm/.clang-format`
- C++ tidy configs exist in:
  - `src/lidar/livox_ros_driver2/.clang-tidy`
  - `src/navigation/pb_omni_pid_pursuit_controller/.clang-tidy`
  - `src/localization/pcd2pgm/.clang-tidy`
- Format only touched files or hunks; avoid workspace-wide formatting sweeps.

## Language And Build Conventions
- Respect package-local configuration before applying generic ROS guidance.
- Do not impose one global C++ standard across the workspace.
- Observed standards include C++17 in `src/localization/Adaptive-LIO` and `src/localization/lightning-lm`, plus C++14 in `src/lidar/livox_ros_driver2`, `src/localization/Point-LIO-grid_map_ros2`, and `src/localization/pcd2pgm`.
- Keep `package.xml` and `CMakeLists.txt` dependencies in sync.
- Prefer target-scoped CMake such as `ament_target_dependencies`, `target_link_libraries`, and `target_include_directories`.
- Guard tests and lint in CMake with `if(BUILD_TESTING)`.
- Avoid broad global compile flags unless the package already relies on them.

## C++ Style Guidelines
- Follow the nearest `.clang-format` and `.clang-tidy` when present.
- The strictest observed naming rules use:
  - classes, enums, unions: `CamelCase`
  - methods/functions: `camelBack`
  - variables: `lower_case`
  - private/protected members: `lower_case_`
  - constants and enum constants: `UPPER_CASE`
- Keep include order stable: standard library, third-party, ROS/ament, then project headers.
- Keep headers lean; prefer forward declarations where practical.
- Prefer `nullptr`, `override`, `std::make_unique`, and `std::make_shared`.
- Pass heavy inputs by `const &`; use references for required non-null parameters.
- Use `std::unique_ptr` for sole ownership and `std::shared_ptr` only for shared lifetime.
- Match the surrounding brace and pointer style instead of reformatting entire files.
- Keep column width near the local formatter default; one observed config uses `ColumnLimit: 100`.

## Python Style Guidelines
- Follow PEP 8 and satisfy `ament_flake8` and `ament_pep257`.
- Keep imports grouped as: standard library, third-party, ROS packages, local modules.
- Prefer one import per line for ROS message types when that matches the file.
- Naming:
  - modules, functions, variables: `snake_case`
  - classes: `PascalCase`
  - constants: `UPPER_CASE`
- Add type hints in new or modified code when they improve clarity.
- Prefer ROS logging over `print()` in runtime nodes.
- Keep launch files and node entry points small and explicit.

## Error Handling And Logging
- Validate configuration early and fail fast on invalid setup.
- Log actionable context with `RCLCPP_INFO/WARN/ERROR` or the Python node logger.
- Do not silently swallow exceptions, failed I/O, or invalid message data.
- For hardware-facing code, surface connection failures clearly and preserve enough context to debug them.
- When returning status, prefer explicit success or failure paths over hidden side effects.

## Agent Workflow Expectations
- Keep diffs tightly scoped to the requested task.
- Do not refactor unrelated code just to match personal style.
- Do not edit `build/`, `install/`, or `log/` outputs.
- Avoid changing third-party trees unless explicitly asked.
- For normal package edits, run at minimum: `colcon build --symlink-install --packages-select <pkg>`, `colcon test --packages-select <pkg> --event-handlers console_direct+ --return-code-on-test-failure`, and `colcon test-result --verbose --test-result-base build/<pkg>`.
- If interfaces, messages, or shared libraries change, prefer `--packages-up-to <pkg>` and retest dependents.

## Quick Diagnostics
- Confirm package discovery: `colcon list | grep <pkg>`
- Inspect CTest registration: `ctest -N --test-dir build/<pkg>`
- Re-run a known failing test with logs: `colcon test --packages-select <pkg> --ctest-args -R <failed_name> --output-on-failure`
- If Python imports fail, re-source both environments:
  - `source /opt/ros/$ROS_DISTRO/setup.bash`
  - `source install/setup.bash`
