#!/bin/bash
# Fix YAML frontmatter - quote titles with colons

set -e

echo "Fixing YAML frontmatter for titles with colons..."

find docs -name "*.md" | while read file; do
    # Use sed to quote titles that contain colons
    sed -i 's/^title: \(.*:.*\)$/title: "\1"/' "$file"
done

echo "✅ Frontmatter fixed!"
