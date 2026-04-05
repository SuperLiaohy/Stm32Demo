#!/bin/bash
echo "=== Cleaning up old OpenOCD processes ==="
pkill -f openocd || true
sleep 1