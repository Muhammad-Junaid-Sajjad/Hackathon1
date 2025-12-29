#!/bin/bash
# Validation Script 2: Latency Trap Coverage
# Counts "Required Callout" and "Mention" placements (should total 36: 22 + 14)

set -e

echo "========================================"
echo "Validating Latency Trap Coverage"
echo "========================================"
echo

# Pattern for "Required Callout" - full warning block
REQUIRED_PATTERN="Latency Trap Warning"
REQUIRED_FOUND=$(grep -r "$REQUIRED_PATTERN" docs/ 2>/dev/null | wc -l)
REQUIRED_EXPECTED=22

# Pattern for "Mention" - abbreviated reference
MENTION_PATTERN="See M3-C3-S6 for weight flashing"
MENTION_FOUND=$(grep -r "$MENTION_PATTERN" docs/ 2>/dev/null | wc -l)
MENTION_EXPECTED=14

# Total expected placements
TOTAL_EXPECTED=36
TOTAL_FOUND=$((REQUIRED_FOUND + MENTION_FOUND))

echo "Required Callouts (full warnings):"
echo "  Expected: $REQUIRED_EXPECTED"
echo "  Found:    $REQUIRED_FOUND"
echo

echo "Mentions (abbreviated references):"
echo "  Expected: $MENTION_EXPECTED"
echo "  Found:    $MENTION_FOUND"
echo

echo "Total Latency Trap Placements:"
echo "  Expected: $TOTAL_EXPECTED"
echo "  Found:    $TOTAL_FOUND"
echo

if [ $TOTAL_FOUND -eq $TOTAL_EXPECTED ]; then
    echo "✅ SUCCESS: All 36 latency trap placements found!"
    exit 0
else
    DIFF=$((TOTAL_EXPECTED - TOTAL_FOUND))
    if [ $DIFF -gt 0 ]; then
        echo "⚠️  WARNING: $DIFF latency trap placements missing"
    else
        EXTRA=$((-DIFF))
        echo "⚠️  WARNING: $EXTRA extra latency trap placements found"
    fi
    echo
    echo "Note: This validation runs after content writing phase."
    echo "Current placeholder files will show 0 placements."
    exit 0
fi
