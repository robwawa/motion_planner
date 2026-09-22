# Frozen benchmark reference

These files preserve the working-tree implementation **before** the acceleration
changes. They are not a second production backend. Only the namespace and local
include names were changed so regression tests can link both implementations.
`reference_main.cpp` provides the `pct_map_cleaner_baseline` executable when
`BUILD_TESTING=ON`. It ignores the new `sor.search_backend` key, as the old CLI did.

The original source/configuration/input hashes and build environment are in
`../../reports/baseline_manifest.json`. In particular, this reference was not
obtained from Git HEAD: the original cleaner C++ files were untracked and the
configuration already had local changes.

Build with the same compiler, optimization flags and FLANN/PCL libraries as the
current executable. Reset FLANN's random seed before each in-process reference
comparison; production and benchmark subprocesses retain the original RNG behavior.
