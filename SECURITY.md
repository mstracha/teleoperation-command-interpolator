# Security Policy

## Supported version

Security fixes are evaluated against the current `main` branch and the most recent tagged release.

## Reporting a vulnerability

Please do not open a public issue for a vulnerability that could affect physical safety, memory safety, concurrency correctness, report integrity, or dependency execution. Contact the maintainer privately through the contact method listed on Mark Strachan’s GitHub profile and include:

- the affected revision and platform;
- a minimal reproducer or scenario;
- expected and observed behavior;
- the possible safety or security impact;
- whether the issue is deterministic;
- any suggested disclosure constraints.

Do not include credentials, proprietary robot data, or unsafe hardware-operating instructions.

## Scope and safety boundary

This repository is an educational demonstration, not a certified controller. Reports about missing hardware interlocks, safety PLCs, plant feedback, authentication, transport security, or real-time operating-system guarantees are valuable design feedback but may describe capabilities intentionally outside the current scope. Such limitations remain important and will not be presented as solved by software-only tests.
