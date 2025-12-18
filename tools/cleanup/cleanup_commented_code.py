#!/usr/bin/env python3
"""
Automated script to clean up commented-out code blocks.
"""

import re
from pathlib import Path


def cleanup_commented_code() -> None:
    """Clean up commented-out code blocks."""

    cleanup_patterns = [
        # Commented-out subprocess calls
        (r"^\s*#.*subprocess\.run.*$\n^\s*#.*check=True.*$", "", re.MULTILINE),
        # Commented-out variable assignments
        (r"^\s*#.*[a-zA-Z_][a-zA-Z0-9_]*\s*=.*$", "", re.MULTILINE),
        # Commented-out function calls
        (r"^\s*#.*[a-zA-Z_][a-zA-Z0-9_]*\(.*\)$", "", re.MULTILINE),
        # Old debug comments that are no longer relevant
        (r"^\s*# Debug:.*$", "", re.MULTILINE),
        (r"^\s*# TODO: Remove debug.*$", "", re.MULTILINE),
        # Commented-out imports (but keep legitimate commented imports)
        (
            r"^\s*# (import|from) .*(not available|unavailable|disabled).*$",
            "",
            re.MULTILINE,
        ),
    ]

    src_dir = Path("src")
    files_processed = 0
    lines_removed = 0

    for py_file in src_dir.rglob("*.py"):
        if "test_" in py_file.name or "__pycache__" in str(py_file):
            continue

        try:
            content = py_file.read_text()
            original_lines = len(content.split("\n"))

            # Apply cleanup patterns
            for pattern, replacement, flags in cleanup_patterns:
                content = re.sub(pattern, replacement, content, flags=flags)

            # Remove multiple consecutive empty comment lines
            content = re.sub(r"(\n\s*#\s*\n){3,}", "\n    #\n", content)

            # Count lines removed
            new_lines = len(content.split("\n"))
            lines_removed_in_file = original_lines - new_lines

            if lines_removed_in_file > 0:
                py_file.write_text(content)
                files_processed += 1
                lines_removed += lines_removed_in_file
                print(
                    f"[PASS] Cleaned {py_file.name} ({lines_removed_in_file} lines removed)"
                )

        except Exception as e:
            print(f"[FAIL] Error processing {py_file}: {e}")

    print(
        f"\n[GRAPH] SUMMARY: {files_processed} files processed, {lines_removed} lines removed"
    )


if __name__ == "__main__":
    cleanup_commented_code()
