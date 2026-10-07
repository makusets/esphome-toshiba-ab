#!/usr/bin/env python3
"""Verify decoded climate states through checksummed master bus frames."""
from run_master_status_tests import LOG_STUB
from run_esp_address_tests import run_test
if __name__ == '__main__':
    run_test('status_decoding_test.cpp', LOG_STUB)
