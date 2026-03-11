# AGENTS.md

## Cursor Cloud specific instructions

### Codebase overview

This is a **partial checkout** of the QCraft autonomous driving monorepo, containing 4 C++ modules:

| Directory | Module | Purpose |
|---|---|---|
| `planner/` | PlannerModule | Motion planning, trajectory optimization, routing, decision-making |
| `prediction/` | PredictionModule | Object trajectory prediction, ML inference |
| `control/` | VehicleControlModule | Vehicle control (MPC controllers, steering, parking) |
| `ml_planner/` | MlPlannerModule | ML-based motion planning (CaptainNet) |

### Build system

- **Primary**: Bazel (`BUILD` files in every directory), but `WORKSPACE` file and external deps (`//onboard/`, `//offboard/`, `//common/`) are **not in this workspace**. Full Bazel builds require the complete monorepo.
- **Secondary**: CMake (`CMakeLists.txt` files), also depends on parent-level CMake config not present.
- Proto files (61 total) use `proto2` syntax with `qcraft` package; most import external protos from `onboard/`.

### What works in this partial checkout

- **Linting**: `cpplint --recursive --filter=-legal/copyright,-build/header_guard,-build/include_subdir <dir>/` — the header guard and copyright filters are needed because files expect to live under `onboard/` in the full monorepo.
- **Formatting**: `clang-format` works on all `.cc`/`.h` files.
- **Syntax checks**: `g++ -std=c++17 -fsyntax-only` works for files whose includes resolve locally. Create symlinks (`ln -sf ../planner onboard/planner`, etc.) to resolve `onboard/` prefixed includes for standalone files.
- **Protobuf**: `protoc` can compile self-contained `.proto` files (those without external imports). ~22 of 61 proto files are self-contained.

### What does NOT work

- Full Bazel build/test — requires the complete monorepo with `WORKSPACE`, `//onboard/`, `//offboard/`, `//common/` targets.
- Running the application — this is an embedded autonomous driving stack that runs on vehicles, not a standalone service.
- Most C++ files cannot be compiled in isolation because they depend on external headers (`//onboard/lite`, `//onboard/maps`, `//onboard/math`, etc.).

### Installed tools

- `g++` 13.3 (C++17), `cmake` 3.28, `bazel` 9.0 (via bazelisk)
- `cpplint` 2.0.2 (Python), `clang-format` 18.1, `protoc` 3.21
- `libgflags-dev`, `libgoogle-glog-dev`, `libprotobuf-dev`, `libgtest-dev`, `libabsl-dev`
