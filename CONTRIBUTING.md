# Contributing

Contributions are welcome when they preserve the project’s bounded-work, explicit-state, and evidence-first design.

## Before opening a change

1. Read the [design record](docs/design/TeleoperationCommandInterpolatorDesign.md), especially the safety boundary and concurrency contracts.
2. Keep public interfaces in the `teleoperation` namespace and follow the existing Allman formatting.
3. Do not introduce heap allocation, blocking synchronization, text formatting, or unbounded loops into command ingress or target update paths without a documented design review.
4. Treat one producer and one consumer per SPSC ring as an invariant, not a suggestion.
5. Add a focused regression test for every behavioral change or corrected defect.

## Verification

Build in Release mode and run all tests:

```sh
cmake -S . -B out/build/release -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build out/build/release
ctest --test-dir out/build/release --output-on-failure
```

Changes to package metadata must also build the consumer under `examples/consumer`. Documentation changes must regenerate affected diagrams, HTML, and PDF editions, followed by a visual inspection of every PDF page.

## Safety-related changes

State transitions, fault policy, timestamp arithmetic, trajectory validation, ring-buffer ownership, and shutdown ordering are safety-significant within the teaching model. Explain the intended invariant, enumerate new failure modes, and include both positive and negative tests. A passing test suite does not make the software suitable for physical hardware.

## Commit scope

Prefer small commits that contain one coherent behavior or documentation change. Do not commit local build trees, generated temporary files, private notes, credentials, hardware configuration, or proprietary source material.
