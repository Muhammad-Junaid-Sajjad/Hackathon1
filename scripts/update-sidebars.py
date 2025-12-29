#!/usr/bin/env python3
"""Update sidebars.ts to use lowercase document IDs"""

# Read the current sidebars.ts
with open('sidebars.ts', 'r') as f:
    content = f.read()

# Replace uppercase path references with lowercase IDs
for m in range(1, 5):
    for c in range(1, 4):
        for s in range(1, 8):
            uppercase_ref = f"'M{m}/C{c}/S{s}'"
            lowercase_ref = f"'M{m}/C{c}/m{m}-c{c}-s{s}'"
            content = content.replace(uppercase_ref, lowercase_ref)

# Write back
with open('sidebars.ts', 'w') as f:
    f.write(content)

print("✓ Updated sidebars.ts with lowercase document IDs")
