# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause
# Modified by Amazon.com, Inc. or its affiliates.

import logging

logger = logging.getLogger(__name__)

# Note: previously this module shimmed a patched `aioice` from
# `external_lib/aioice` onto sys.path to share ICE credentials across
# Connection instances (needed for older aiortc 1.9.x + Go2 firmware).
# With aiortc >= 1.14, the upstream aioice is sufficient; the shim is
# disabled to avoid ABI mismatches with the newer aiortc.
