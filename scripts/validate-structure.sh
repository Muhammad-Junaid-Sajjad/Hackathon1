#!/bin/bash
# Validation Script 1: Directory and File Structure
# Verifies all 84 sections exist at correct paths (docs/M[1-4]/C[1-3]/S[1-7].md)

set -e

echo "========================================"
echo "Validating 4×3×7 Directory Structure"
echo "========================================"
echo

ERRORS=0
EXPECTED_FILES=84
FOUND_FILES=0

# Check if docs directory exists
if [ ! -d "docs" ]; then
    echo "❌ ERROR: docs/ directory not found"
    exit 1
fi

# Validate all 84 sections exist
for module in {1..4}; do
    for chapter in {1..3}; do
        for section in {1..7}; do
            FILE="docs/M${module}/C${chapter}/S${section}.md"
            if [ -f "$FILE" ]; then
                ((FOUND_FILES++))
                echo "✓ $FILE"
            else
                echo "❌ MISSING: $FILE"
                ((ERRORS++))
            fi
        done
    done
done

echo
echo "========================================"
echo "Validation Summary"
echo "========================================"
echo "Expected files: $EXPECTED_FILES"
echo "Found files:    $FOUND_FILES"
echo "Missing files:  $ERRORS"
echo

if [ $ERRORS -eq 0 ]; then
    echo "✅ SUCCESS: All 84 sections found at correct paths!"
    exit 0
else
    echo "❌ FAILURE: $ERRORS files are missing"
    exit 1
fi
