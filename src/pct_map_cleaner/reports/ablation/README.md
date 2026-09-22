# Incremental benchmark versions

These patches record the intermediate versions measured in this experiment.
They are evidence/reproduction inputs, not alternate production implementations.
Apply them in order to the original pre-optimization source tree:

1. `pre.patch`: remove the duplicate pre-sampling hash table, add detailed timers
   and backend/configuration declarations (search is still the original implementation).
2. `ror.patch`: additionally stop ROR as soon as enough neighbors are found.
3. `completion.patch`: additionally reuse completion buffers and offset markers.

The final production source additionally implements batched SOR and the optional
single-tree backend. All measurements use the `legacy` backend except the named
`single_exact` variant. Build flags come from `CMakeLists.txt.baseline`.

To reconstruct an original source tree from `tests/reference`, remove the first
provenance comment line, replace `reference_pct_map_cleaner` with `pct_map_cleaner`,
rename `reference_cleaner.hpp` to `include/pct_map_cleaner.hpp`, rename
`reference_cleaner.cpp` to `src/pct_map_cleaner.cpp`, rename `reference_main.cpp`
to `src/main.cpp`, and update the two local header includes to `pct_map_cleaner.hpp`.
Copy `CMakeLists.txt.baseline` as `CMakeLists.txt`. Run `patch -p1` inside that new
tree for each successive patch, and build each stage in an independent build
folder. Always pass an explicit configuration and input/output override to the CLI.

`manifest.json` records source and executable hashes of the measured variants.
The main `baseline_manifest.json` records hashes before any edits.
