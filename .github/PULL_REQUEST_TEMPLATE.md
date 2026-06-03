<!-- Thanks for contributing! See docs/contributing.md. -->

## What and why

Describe the change and the problem it solves. Link related issues / RFCs.

## Type

- [ ] Bug fix
- [ ] Feature
- [ ] Safety filter / scorer / fallback
- [ ] Model (Model Zoo)
- [ ] Benchmark
- [ ] Docs / CI

## Checklist

- [ ] `colcon build` and `colcon test` pass locally
- [ ] New code has unit tests
- [ ] Behavior/safety changes covered by integration tests
- [ ] ament lints pass (copyright / cpplint / uncrustify / lint_cmake / xmllint)
- [ ] Docs / README updated
- [ ] Runtime packages did not gain heavy training (Python) dependencies

## For model submissions

- [ ] Manifest (all section-5.4 fields) + completed model card
- [ ] Benchmark passes; safety-first score and latency p95/p99 reported
- [ ] Failure cases and limitations documented
- [ ] Licenses (model / data / code) stated separately

## Safety impact

Does this change anything that can move the robot? How is it gated and tested?
