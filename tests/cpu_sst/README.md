# 68000 SingleStepTests harness

`openmac_sst` runs the [SingleStepTests/680x0](https://github.com/SingleStepTests/680x0)
per-opcode JSON suites (hardware-derived) against the M68000 core.

```
openmac_sst <file.json.gz | directory> [--cycles] [--max-show N] [--filter substr]
```

Fetch the suite with `tests/fetch_sst.ps1`, then point the harness at
`tests/data/680x0/68000/v1`.

## Status

- **State: 100%** — all 124 opcode files pass full register/memory/SR/PC
  comparison. Three suite entries are waived in `main.cpp` with rationale
  (they contradict hardware semantics or their own sibling tests).
- **Cycles: ~99%** — all mainstream execution paths are cycle-exact,
  including data-dependent MULU/MULS/DIVU/DIVS timing. The remaining
  deltas are pre-fault cycle counts inside **address-error** paths (mostly
  MOVE with two memory operands), where exact values depend on per-
  instruction prefetch microcode this core intentionally does not model.
  Faulting instructions are within a few cycles of hardware; the 50-cycle
  exception entry itself is exact. Not gated in CI (`--cycles` off).

The exit code is 0 only when every non-waived test passes state comparison,
which is what CI gates on.
