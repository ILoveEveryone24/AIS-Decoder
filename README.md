# AIS decoder

A for AIS (Automatic Identification System) messages — the
NMEA 0183 `!AIVDM` sentences that vessels broadcast over VHF with their identity,
position, course and speed. It un-armours the 6-bit payload, extracts bit fields
at arbitrary offsets with sign extension, validates the NMEA checksum, and
currently decodes Type 1/2/3 position reports in full.
