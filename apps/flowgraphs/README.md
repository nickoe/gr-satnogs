# SatNOGS flowgraphs
This directory contains all the available flowgraphs that can be executed from
the `satnogs-client`.

## Contribution guide
Flowgraphs are placed inside the `apps/flowgraphs` directory.
If a decoding flowgraph targets only specific satellite/mission, the
flowgraph should be placed inside the `apps/flowgraphs/satellites` directory.

Each flowgraph should have a representative name (eg `voyager_decoder`) and
both the GNU Radio file (`.grc`) and the generated pythons script should be available.
Python auto-generared flowgraph scripts should have the `satnogs_` prefix.
This can be performed by setting properly the `ID` field of the `Options` block
of the flowgraph.
For example the `voyager_decoder.grc` should generate a python executable named
`satnogs_voyager_decoder.py`

**NOTE:**: Custom python GNU Radio scripts are not allowed.
Each flowgraph should be able to be generated from the corresponding `.grc` file.

All generated python scripts should be installed using the `CMake` build system.
To do so, edit properly the `apps/CMakeLists.txt` or `apps/flowgraphs/satellites/CMakeLists.txt` file.

In the `apps/flowgraphs` directory, the is an example flowgraph called `example_flowgraph.grc`
that can be used as a base.

### Execution arguements interface
The `stanogs-client` and the `gr-satnogs` communicate through a set of
predefined command line arguments.
Depending the decoding flowgraph, additional arguments may exist or missing.
However, there is a set of mandatory arguments.

* `--antenna`: The name of the antenna to use
* `--dev-args`: SDR device specific arguments
* `--bb-gain`: Baseband gain
* `--if-gain`: Intermediate frequency gain
* `--rf-gain`: RF gain
* `--ppm`: The PPM correction
* `--rx-freq`: RX frequency
* `--rx-sdr-device`: The RX SDR device identification name (e.g uhd, airspy, etc)
* `--rigctl-port`: The `rigctld` port
* `--doppler-correction-per-sec`: Number of Doppler corrections per second
* `--lo-offset`: Offset from the desired center frequency. The flowgraph should
tune the SDR with this offset from the center frequency and digitally compensate it.
This eliminate the problem of the DC leakage, expressed in the majority of the
SDR devices.  