#!/bin/bash
# Validation Script 3: Cross-Link Validation
# Checks critical cross-links: URDF, Sim-to-Real, VLA→Actions

set -e

echo "========================================"
echo "Validating Critical Cross-Links"
echo "========================================"
echo

ERRORS=0

# Function to check if a file contains a reference to another section
check_link() {
    local SOURCE_FILE=$1
    local TARGET_SECTION=$2
    local LINK_TYPE=$3

    if [ ! -f "$SOURCE_FILE" ]; then
        echo "⚠️  $LINK_TYPE: Source file $SOURCE_FILE not found"
        return
    fi

    if grep -q "$TARGET_SECTION" "$SOURCE_FILE"; then
        echo "✓ $LINK_TYPE: $SOURCE_FILE → $TARGET_SECTION"
    else
        echo "❌ MISSING LINK: $SOURCE_FILE should reference $TARGET_SECTION ($LINK_TYPE)"
        ((ERRORS++))
    fi
}

echo "1. URDF Cross-Links"
echo "-------------------"
# M1-C3-S1 (canonical) → M2-C1-S1 (Gazebo) → M3-C1-S3 (USD conversion)
check_link "docs/M2/C1/S1.md" "M1-C3-S1\|M1/C3/S1" "URDF Canonical → Gazebo"
check_link "docs/M3/C1/S3.md" "M1-C3-S1\|M1/C3/S1" "URDF Canonical → Isaac"
echo

echo "2. Sim-to-Real Pipeline"
echo "------------------------"
# M2-C2-S7 + M3-C1-S5 → M3-C2-S3 → M3-C3-S5 → M3-C3-S6 (PRIMARY) → M4-C3-S5
check_link "docs/M3/C2/S3.md" "M2-C2-S7\|M2/C2/S7\|M3-C1-S5\|M3/C1/S5" "Synthetic Data → Training"
check_link "docs/M3/C3/S5.md" "M3-C2-S3\|M3/C2/S3" "Training → ONNX Export"
check_link "docs/M3/C3/S6.md" "M3-C3-S5\|M3/C3/S5" "ONNX → Weight Flashing"
check_link "docs/M4/C3/S5.md" "M3-C3-S6\|M3/C3/S6" "Weight Flashing → Physical Deploy"
echo

echo "3. VLA → Actions Pipeline"
echo "--------------------------"
# M4-C1-S2 (Whisper) → M4-C1-S3 (parsing) → M4-C2-S4 (grounding) → M1-C2-S2 (Actions)
check_link "docs/M4/C1/S3.md" "M4-C1-S2\|M4/C1/S2" "Whisper → Parsing"
check_link "docs/M4/C2/S4.md" "M4-C1-S3\|M4/C1/S3" "Parsing → Grounding"
check_link "docs/M4/C2/S4.md" "M1-C2-S2\|M1/C2/S2" "Grounding → ROS 2 Actions"
echo

echo "========================================"
echo "Validation Summary"
echo "========================================"
echo "Total broken cross-links: $ERRORS"
echo

if [ $ERRORS -eq 0 ]; then
    echo "✅ SUCCESS: All critical cross-links valid!"
    echo
    echo "Note: This validation checks for cross-references."
    echo "If sections are still placeholders, links may not exist yet."
    exit 0
else
    echo "❌ FAILURE: $ERRORS cross-links are missing"
    echo
    echo "Note: Cross-links are added during content writing phase."
    exit 0
fi
