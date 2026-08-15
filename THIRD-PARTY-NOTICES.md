# Third-party notices

OpenMac is MIT licensed (see `LICENSE`). The following third-party code is
included or fetched under its own license.

## MAME Am29000 CPU core (BSD-3-Clause)

`core/src/machine/iifx/am29000.cpp`, `am29000.hpp` and `am29000_ops.inc` are
adapted from the portable Am29000 CPU core in MAME, copyright Philip Bennett,
and remain under the BSD-3-Clause license reproduced below. OpenMac's copy
removes MAME's device framework, adds Harvard instruction/data callbacks and
the corrections described in the wiki (data-width load and store semantics,
channel-register restart of trapped accesses, timer and interrupt-return
behavior); those changes are contributed under the same license.

```
Copyright (c) Philip Bennett
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```

## Build- and test-time dependencies (fetched by CMake, not vendored)

- doctest — MIT license (unit tests)
- nlohmann/json — MIT license (test tooling)
- miniz — MIT license (test tooling)
- SDL3 — zlib license (optional developer shell)
- Dear ImGui — MIT license (optional developer shell)

The 68000 core is validated against the SingleStepTests 680x0 test suite,
which is downloaded at test time and not redistributed here.
