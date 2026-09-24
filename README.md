# AIS decoder

A from-scratch decoder for AIS (Automatic Identification System) messages — the
NMEA 0183 `!AIVDM` sentences that vessels broadcast over VHF with their identity,
position, course and speed. It un-armours the 6-bit payload, extracts bit fields
at arbitrary offsets with sign extension, validates the NMEA checksum, and
currently decodes Type 1/2/3 position reports in full.

A learning project, written by hand rather than with a library. Build with:

```sh
g++ -std=c++23 -Wall -Wextra -Wshadow -Wconversion -o ais main.cpp
```
