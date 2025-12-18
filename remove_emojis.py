#!/usr/bin/env python3
"""
Script to remove emojis from the codebase and replace with appropriate text.
"""

import os
import re
from pathlib import Path

def remove_emojis_from_codebase():
    """Remove emojis from Python files and replace with meaningful text."""
    
    # Emoji to text replacements
    emoji_replacements = {
        "🚨": "[ALERT]",
        "✅": "[SUCCESS]",
        "❌": "[ERROR]",
        "🧪": "[TEST]",
        "🎯": "[TARGET]",
        "📊": "[STATUS]",
        "🏆": "[ACHIEVEMENT]",
        "🚀": "[START]",
        "🔧": "[FIX]",
        "⚡": "[PERFORMANCE]",
        "🌐": "[NETWORK]",
        "🛠️": "[TOOLS]",
        "📏": "[FORMAT]",
        "📦": "[PACKAGE]",
        "🐛": "[BUG]",
        "🔷": "[TYPE]",
        "🔍": "[SEARCH]",
        "🧹": "[CLEAN]",
        "📈": "[IMPROVE]",
        "🎉": "[CELEBRATE]",
        "🏗️": "[BUILD]",
        "🔒": "[SECURE]",
        "⚙️": "[CONFIG]",
        "🤖": "[AUTO]",
        "🛡️": "[PROTECT]",
        "📝": "[NOTE]",
        "📚": "[INFO]",
        "⚠️": "[WARNING]",
        "💥": "[FAIL]",
        "✨": "[COMPLETE]",
        "🍰": "[DONE]",
        "🔄": "[UPDATE]",
        "🛑": "[STOP]",
        "🔥": "[HOT]",
        "💡": "[IDEA]",
        "🎨": "[STYLE]",
        "📋": "[LIST]",
        "🔗": "[CONNECT]",
        "🤖": "[ROBOT]",
        "🛡️": "[SHIELD]",
        "📝": "[WRITE]",
        "📚": "[BOOK]",
        "⚠️": "[WARN]",
        "💥": "[EXPLODE]",
        "✨": "[SPARKLE]",
        "🍰": "[CAKE]",
        "🔄": "[REFRESH]",
        "🛑": "[BLOCK]",
        "🔥": "[FIRE]",
        "💡": "[LIGHT]",
        "🎨": "[ART]",
        "📋": "[CLIPBOARD]",
        "🔗": "[LINK]",
        "🎪": "[CIRCUS]",
        "🎭": "[DRAMA]",
        "🎬": "[ACTION]",
        "🎤": "[VOICE]",
        "🎵": "[SOUND]",
        "🎶": "[MUSIC]",
        "🎼": "[SCORE]",
        "🎹": "[KEYBOARD]",
        "🎸": "[GUITAR]",
        "🎺": "[BRASS]",
        "🎷": "[WOODWIND]",
        "🥁": "[DRUMS]",
        "🎧": "[HEADPHONES]",
        "📻": "[RADIO]",
        "📱": "[PHONE]",
        "💻": "[COMPUTER]",
        "🖥️": "[SCREEN]",
        "🖨️": "[PRINTER]",
        "⌨️": "[KEYBOARD]",
        "🖱️": "[MOUSE]",
        "🖲️": "[TRACKBALL]",
        "💽": "[DISK]",
        "💾": "[FLOPPY]",
        "💿": "[CD]",
        "📀": "[DVD]",
        "📺": "[TV]",
        "📷": "[CAMERA]",
        "📹": "[VIDEO]",
        "🎥": "[MOVIE]",
        "📽️": "[PROJECTOR]",
        "🎞️": "[FILM]",
        "📞": "[TELEPHONE]",
        "☎️": "[PHONE]",
        "📟": "[PAGER]",
        "📠": "[FAX]",
        "📡": "[SATELLITE]",
        "🛰️": "[SATELLITE]",
        "💰": "[MONEY]",
        "💲": "[DOLLAR]",
        "💳": "[CARD]",
        "💎": "[GEM]",
        "🔪": "[KNIFE]",
        "💈": "[BARBER]",
        "🛀": "[BATH]",
        "🛌": "[BED]",
        "🚪": "[DOOR]",
        "🚬": "[SMOKE]",
        "🚭": "[NO_SMOKING]",
        "🚰": "[WATER]",
        "🚿": "[SHOWER]",
        "🛁": "[BATHTUB]",
        "🚽": "[TOILET]",
        "🚾": "[WC]",
        "🛂": "[PASSPORT]",
        "🛃": "[CUSTOMS]",
        "🛄": "[BAGGAGE]",
        "🛅": "[LOCKER]",
        "🚹": "[MENS]",
        "🚺": "[WOMENS]",
        "🚻": "[RESTROOM]",
        "🚼": "[BABY]",
        "🚾": "[WC]",
        "🛂": "[PASSPORT]",
        "🛃": "[CUSTOMS]",
        "🛄": "[BAGGAGE]",
        "🛅": "[LOCKER]",
        "🚹": "[MENS]",
        "🚺": "[WOMENS]",
        "🚻": "[RESTROOM]",
        "🚼": "[BABY]",
        "🚪": "[DOOR]",
        "🚬": "[SMOKE]",
        "🚭": "[NO_SMOKING]",
        "🚰": "[WATER]",
        "🚿": "[SHOWER]",
        "🛁": "[BATHTUB]",
        "🚽": "[TOILET]",
        "🚾": "[WC]",
        "🛀": "[BATH]",
        "🛌": "[BED]",
        "💈": "[BARBER]",
        "💰": "[MONEY]",
        "💲": "[DOLLAR]",
        "💳": "[CARD]",
        "💎": "[GEM]",
        "🔪": "[KNIFE]",
        "📞": "[TELEPHONE]",
        "☎️": "[PHONE]",
        "📟": "[PAGER]",
        "📠": "[FAX]",
        "📡": "[SATELLITE]",
        "🛰️": "[SATELLITE]",
        "📺": "[TV]",
        "📷": "[CAMERA]",
        "📹": "[VIDEO]",
        "🎥": "[MOVIE]",
        "📽️": "[PROJECTOR]",
        "🎞️": "[FILM]",
        "💻": "[COMPUTER]",
        "🖥️": "[SCREEN]",
        "🖨️": "[PRINTER]",
        "⌨️": "[KEYBOARD]",
        "🖱️": "[MOUSE]",
        "🖲️": "[TRACKBALL]",
        "💽": "[DISK]",
        "💾": "[FLOPPY]",
        "💿": "[CD]",
        "📀": "[DVD]",
        "🎧": "[HEADPHONES]",
        "📻": "[RADIO]",
        "📱": "[PHONE]",
        "📞": "[TELEPHONE]",
        "☎️": "[PHONE]",
        "📟": "[PAGER]",
        "📠": "[FAX]",
        "📡": "[SATELLITE]",
        "🛰️": "[SATELLITE]",
        "📺": "[TV]",
        "📷": "[CAMERA]",
        "📹": "[VIDEO]",
        "🎥": "[MOVIE]",
        "📽️": "[PROJECTOR]",
        "🎞️": "[FILM]",
        "💻": "[COMPUTER]",
        "🖥️": "[SCREEN]",
        "🖨️": "[PRINTER]",
        "⌨️": "[KEYBOARD]",
        "🖱️": "[MOUSE]",
        "🖲️": "[TRACKBALL]",
        "💽": "[DISK]",
        "💾": "[FLOPPY]",
        "💿": "[CD]",
        "📀": "[DVD]",
        "🎧": "[HEADPHONES]",
        "📻": "[RADIO]",
        "📱": "[PHONE]",
    }
    
    src_dir = Path('src')
    files_processed = 0
    emojis_removed = 0
    
    for py_file in src_dir.rglob('*.py'):
        if '__pycache__' in str(py_file):
            continue
            
        try:
            content = py_file.read_text()
            original_content = content
            
            # Replace emojis with text
            for emoji, replacement in emoji_replacements.items():
                content = content.replace(emoji, replacement)
            
            if content != original_content:
                py_file.write_text(content)
                files_processed += 1
                
                # Count emojis removed in this file
                removed_in_file = sum(1 for emoji in emoji_replacements.keys() 
                                    if emoji in original_content)
                emojis_removed += removed_in_file
                
                print(f"✅ Cleaned {py_file.name} ({removed_in_file} emojis removed)")
                
        except Exception as e:
            print(f"❌ Error processing {py_file}: {e}")
    
    print(f"\n📊 SUMMARY: {files_processed} files processed, {emojis_removed} emojis removed")

if __name__ == "__main__":
    remove_emojis_from_codebase()
