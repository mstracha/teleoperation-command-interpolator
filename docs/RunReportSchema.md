# Run report schema

`RunReportWriter` creates a new, write-once directory for each completed
real-time harness run. Reporting happens after the three harness threads have
stopped, so filesystem activity never enters the nominal 500 Hz control path.

Every report uses schema version 3 and contains four UTF-8 CSV files:

- `summary.csv` has one data row containing the outcome, configuration,
  requested and actual controller wait strategies, optional logical-processor
  affinity result, lateness percentiles, aggregate recorded-event counts, and
  telemetry drop counts.
- `input_telemetry.csv` has one row for every recorded command-receipt event.
- `control_telemetry.csv` has one row for every recorded controller tick.
- `controller_timing.csv` has one row for every executed controller tick. It
  records the scheduled offset, actual start offset, nonnegative lateness, and
  `updateTarget` execution duration.

Times ending in `_ns` are integer nanoseconds. Sender and controller times
remain in separate clock domains and must not be subtracted from one another.
Joint positions are radians. Empty failure-index cells mean that no command or
joint was associated with that controller sample. Non-finite numeric test data
is written as `nan`, `inf`, or `-inf`.

Counts prefixed with `recorded_` are derived from retained telemetry. If a
telemetry drop count is nonzero, those aggregates are incomplete; the drop
counts make that loss explicit to downstream analysis.

Controller timing offsets share the harness's monotonic clock and are relative
to the scheduled controller start. The p50, p95, and p99 summary values use the
nearest-rank definition. These measurements characterize an observed run; they
do not establish a hard real-time guarantee.

The writer refuses an output path that already exists. Callers should assign a
unique directory to every run rather than reuse or overwrite prior evidence.
The files are first written to a sibling directory ending in `.tmp`; the
completed directory is then renamed into place so readers cannot mistake a
partial report for a complete one.
