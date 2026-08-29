# Reproduces the pcbnew used to generate the synth_1L / synth_2L DSN files.
# Reference DSN header host_version: "9.0.8-9.0.8~ubuntu22.04.1".
# The KiCad stable PPA only keeps the latest 9.0.x (9.0.8 has been superseded by
# 9.0.9). 9.0.9 is verified to emit byte-identical DSN/ORP to the 9.0.8
# reference (see README.md), so we install whatever 9.0.x the PPA serves.
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
# gnupg is required for add-apt-repository's key import (absent in minimal base).
RUN apt-get update -qq \
 && apt-get install -y -qq --no-install-recommends \
        software-properties-common ca-certificates gnupg \
 && add-apt-repository -y ppa:kicad/kicad-9.0-releases \
 && apt-get update -qq \
 && apt-get install -y -qq --no-install-recommends kicad \
 && rm -rf /var/lib/apt/lists/*
# pcbnew lives in the kicad package at /usr/lib/python3/dist-packages/pcbnew.py
# sanity: python3 -c "import pcbnew; print(pcbnew.GetBuildVersion())"
