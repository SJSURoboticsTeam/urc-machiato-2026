#!/usr/bin/env python3
"""
Automated script to clean up debug print statements and replace with proper logging.
"""

import os
import re
from pathlib import Path

def cleanup_print_statements():
    """Clean up debug print statements in Python files."""
    
    # Patterns to replace
    replacements = [
        # Simple print statements with logger.info
        (r'^\s*print\((f?".*?"(?:\s*%\s*\w+)?)\)\s*$', r'        self.get_logger().info(\1)'),
        
        # Print statements in methods (need proper indentation)
        (r'^\s*print\((f?".*?"(?:\s*%\s*\w+)?)\)\s*$', r'        logger.info(\1)'),
        
        # Print statements with variables
        (r'^\s*print\((f?".*?"),\s*(\w+)\)\s*$', r'        self.get_logger().info(\1, \2)'),
        
        # Emergency/error prints
        (r'^\s*print\("🚨(.*?)"\)\s*$', r'        self.get_logger().error("\1")'),
        
        # Success prints  
        (r'^\s*print\("✅(.*?)"\)\s*$', r'        self.get_logger().info("\1")'),
        
        # Info prints
        (r'^\s*print\("(.*?)..."\)\s*$', r'        self.get_logger().info("\1...")'),
    ]
    
    src_dir = Path('src')
    files_processed = 0
    prints_removed = 0
    
    for py_file in src_dir.rglob('*.py'):
        if 'test_' in py_file.name:
            continue  # Skip test files for now
            
        try:
            content = py_file.read_text()
            original_content = content
            
            # Apply replacements
            for pattern, replacement in replacements:
                content = re.sub(pattern, replacement, content, flags=re.MULTILINE)
            
            # Remove simple print statements entirely if they're just debug
            content = re.sub(r'^\s*print\([^)]*\)\s*$', '', content, flags=re.MULTILINE)
            
            if content != original_content:
                py_file.write_text(content)
                files_processed += 1
                
                # Count removed prints
                removed_in_file = original_content.count('print(') - content.count('print(')
                prints_removed += removed_in_file
                
                print(f"✅ Cleaned {py_file.name} ({removed_in_file} prints removed)")
                
        except Exception as e:
            print(f"❌ Error processing {py_file}: {e}")
    
    print(f"\n📊 SUMMARY: {files_processed} files processed, {prints_removed} print statements removed")

if __name__ == "__main__":
    cleanup_print_statements()
