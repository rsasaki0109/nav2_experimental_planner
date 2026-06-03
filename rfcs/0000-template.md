# RFC 0000: <title>

- Status: draft <!-- draft | proposed | accepted | rejected | superseded -->
- Author(s): <name>
- Created: <YYYY-MM-DD>
- Tracking issue: <link>

> Copy this file to `rfcs/NNNN-short-title.md` (next free number) and fill it in.
> RFCs are required for changes to plugin contracts, message schemas, or the
> safety architecture (docs/contributing.md).

## Summary

One paragraph: what is being proposed.

## Motivation

What problem does this solve? Who benefits (AMR / delivery / warehouse / service)?
Why is the current design insufficient?

## Design

The proposed design in detail. Cover affected layers (Nav2 integration,
observation, generative model, safety/selection, training/benchmark) and any new
or changed plugin interfaces and message schemas.

## Safety considerations

How does this interact with the deterministic safety layer (docs/safety.md)?
New failure modes? Fallback behavior? Does the learned path remain non-authoritative?

## Benchmark / evaluation

How will the change be validated? Which scenarios and metrics
(docs/benchmarking.md)? Any regression thresholds for CI?

## Alternatives

Other designs considered and why they were not chosen.

## Backwards compatibility

Impact on existing plugins, params, message schemas, and supported ROS 2 distros.

## Unresolved questions

Open issues to resolve before acceptance.
