#!/usr/bin/env python3
"""
Script to remove all emojis from Python files in the project.
"""

import os
import re
import sys

# Emoji replacement mappings
EMOJI_REPLACEMENTS = {
    # Test and status emojis
    "[IGNITE]": "[START]",
    "[PASS]": "[PASS]",
    "[FAIL]": "[FAIL]",
    "[TOOL]": "[FIX]",
    "[CONNECT]": "[BRIDGE]",
    "[PLUG]": "[CONNECT]",
    "[CLOCK]": "[TIMER]",
    "[GRAPH]": "[DATA]",
    "[SWEEP]": "[CLEANUP]",
    "[ANTENNA]": "[PUBLISH]",
    "[REFRESH]": "[RETRY]",
    "[OBJECTIVE]": "[TARGET]",
    "[ACHIEVEMENT]": "[SUCCESS]",
    "[MAGNIFY]": "[SEARCH]",
    "[LIGHTNING]": "[PERFORMANCE]",
    "[CONSTRUCTION]": "[BUILD]",
    "[CLIPBOARD]": "[REPORT]",
    "[WATCH]": "[TIME]",
    "[LAB]": "[TEST]",
    "[OBJECTIVE]": "[GOAL]",
    "[ACHIEVEMENT]": "[WINNER]",
    "[IGNITE]": "[LAUNCH]",
    "[SHINE]": "[SPARKLE]",
    "[GRAPH]": "[ANALYTICS]",
    "[REFRESH]": "[CYCLE]",
    "[OBJECTIVE]": "[AIM]",
    "[ACHIEVEMENT]": "[CHAMPION]",
    "[PARTY]": "[CELEBRATE]",
    "[FLAG]": "[FINISH]",
    "[IGNITE]": "[BOOST]",
    "[SHINE]": "[MAGIC]",
    "[LAB]": "[SCIENCE]",
    "[TOOL]": "[TOOL]",
    "[NETWORK]": "[NETWORK]",
    "[EXPERIMENT]": "[EXPERIMENT]",
    "[ANTENNA]": "[ANTENNA]",
    "[CONNECT]": "[CONNECT]",
    "[PLUG]": "[PLUG]",
    "[CLOCK]": "[CLOCK]",
    "[GRAPH]": "[CHART]",
    "[SWEEP]": "[SWEEP]",
    "[MAGNIFY]": "[MAGNIFY]",
    "[LIGHTNING]": "[LIGHTNING]",
    "[CONSTRUCTION]": "[CONSTRUCTION]",
    "[OBJECTIVE]": "[BULLSEYE]",
    "[CLIPBOARD]": "[CLIPBOARD]",
    "[WATCH]": "[WATCH]",
    "[LAB]": "[LAB]",
    "[OBJECTIVE]": "[FOCUS]",
    "[ACHIEVEMENT]": "[TROPHY]",
    "[IGNITE]": "[ROCKET]",
    "[SHINE]": "[STAR]",
    "[GRAPH]": "[GRAPH]",
    "[REFRESH]": "[REFRESH]",
    "[OBJECTIVE]": "[OBJECTIVE]",
    "[ACHIEVEMENT]": "[ACHIEVEMENT]",
    "[PARTY]": "[PARTY]",
    "[FLAG]": "[FLAG]",
    "[IGNITE]": "[IGNITE]",
    "[SHINE]": "[SHINE]",
}


def remove_emojis_from_file(filepath):
    """Remove emojis from a single file."""
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            content = f.read()

        original_content = content

        # Replace all emojis with text equivalents
        for emoji, replacement in EMOJI_REPLACEMENTS.items():
            content = content.replace(emoji, replacement)

        # Also remove any remaining emoji characters (Unicode range)
        # This catches any emojis not in our mapping
        emoji_pattern = re.compile(
            "["
            "\U0001F600-\U0001F64F"  # emoticons
            "\U0001F300-\U0001F5FF"  # symbols & pictographs
            "\U0001F680-\U0001F6FF"  # transport & map symbols
            "\U0001F1E0-\U0001F1FF"  # flags (iOS)
            "\U00002500-\U00002BEF"  # chinese char
            "\U00002702-\U000027B0"
            "\U00002702-\U000027B0"
            "\U000024C2-\U0001F251"
            "\U0001f926-\U0001f937"
            "\U00010000-\U0010ffff"
            "\u2640-\u2642"
            "\u2600-\u2B55"
            "\u200d"
            "\u23cf"
            "\u23e9"
            "\u231a"
            "\ufe0f"  # dingbats
            "\u3030"
            "]+",
            flags=re.UNICODE,
        )
        content = emoji_pattern.sub("", content)

        if content != original_content:
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(content)
            return True

    except Exception as e:
        print(f"Error processing {filepath}: {e}")
        return False

    return False


def main():
    """Main function to remove emojis from all Python files."""
    project_root = "/home/ubuntu/urc-machiato-2026"

    # Find all Python files
    python_files = []
    for root, dirs, files in os.walk(project_root):
        # Skip certain directories
        dirs[:] = [
            d
            for d in dirs
            if not d.startswith(".")
            and d not in ["__pycache__", ".git", "build", "install"]
        ]

        for file in files:
            if file.endswith(".py"):
                python_files.append(os.path.join(root, file))

    print(f"Found {len(python_files)} Python files to process")

    modified_count = 0
    for filepath in python_files:
        if remove_emojis_from_file(filepath):
            modified_count += 1
            print(f"Modified: {filepath}")

    print(f"\nCompleted! Modified {modified_count} files")


if __name__ == "__main__":
    main()
