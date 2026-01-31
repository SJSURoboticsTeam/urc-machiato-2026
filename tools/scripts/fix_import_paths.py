# URC 2026 Testing Infrastructure Implementation Plan
## Comprehensive Ready-to-Execute Plan for Testing Improvements

---

## 📋 EXECUTIVE SUMMARY

This document provides a complete implementation plan for addressing all identified testing infrastructure issues in the URC 2026 robotics platform. The plan covers 5 critical areas with specific code implementations, hardware requirements, and step-by-step procedures.

**Key Issues Addressed:**
1. Import path inconsistencies in Python modules
2. Missing HIL testing infrastructure
3. Inadequate WebSocket network testing
4. Performance degradation testing gaps
5. Integration and CI/CD pipeline deficiencies

**Implementation Timeline:** 6 weeks total
**Resource Requirements:** 2-3 developers, $2,500 hardware budget
**Risk Level:** Medium (mitigated with comprehensive testing)

---

## 🎯 IMPLEMENTATION OVERVIEW

### Phase 1: Import Path Fixes (Week 1)
### Phase 2: HIL Testing Infrastructure (Weeks 2-3)  
### Phase 3: WebSocket Network Testing (Week 3)
### Phase 4: Performance Testing Framework (Week 4)
### Phase 5: Integration & CI/CD Pipeline (Weeks 5-6)

---

## 📁 PHASE 1: IMPORT PATH FIXES

### 1.1 Current Issues Identified

**Problem Files:**
- `src/autonomy/control/integrated_critical_systems.py:30-38` - Relative import inconsistencies
- Multiple files using inconsistent relative import patterns
- Missing proper package structure configuration

### 1.2 Implementation Steps

#### Step 1.1: Create Import Path Standardization Script
```bash
# File: tools/scripts/fix_import_paths.py
```

**Implementation:**
<tool_call>write
<arg_key>content</arg_key>
<arg_value>#!/usr/bin/env python3
"""
Import Path Standardization Script

Automatically fixes inconsistent import paths across the codebase.
Standardizes relative imports and ensures proper package structure.
"""

import os
import re
import sys
from pathlib import Path
from typing import List, Dict, Tuple

class ImportPathFixer:
    """Fixes import path inconsistencies across the codebase."""
    
    def __init__(self, project_root: str):
        self.project_root = Path(project_root)
        self.src_root = self.project_root / "src"
        self.fixes_applied = []
        
    def scan_and_fix(self) -> Dict[str, Any]:
        """Scan all Python files and fix import issues."""
        results = {
            'files_scanned': 0,
            'files_fixed': 0,
            'fixes_applied': []
        }
        
        # Find all Python files in src/
        python_files = list(self.src_root.rglob("*.py"))
        
        for file_path in python_files:
            results['files_scanned'] += 1
            fixes = self._fix_file_imports(file_path)
            
            if fixes:
                results['files_fixed'] += 1
                results['fixes_applied'].extend(fixes)
        
        return results
    
    def _fix_file_imports(self, file_path: Path) -> List[str]:
        """Fix imports in a single file."""
        fixes = []
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            original_content = content
            
            # Fix relative imports that should be absolute
            content = self._fix_relative_imports(content, file_path)
            
            # Fix inconsistent import patterns
            content = self._fix_import_patterns(content)
            
            # Write back if changed
            if content != original_content:
                with open(file_path, 'w', encoding='utf-8') as f:
                    f.write(content)
                fixes.append(f"Fixed imports in {file_path}")
                
        except Exception as e:
            print(f"Error processing {file_path}: {e}")
        
        return fixes
    
    def _fix_relative_imports(self, content: str, file_path: Path) -> str:
        """Fix relative import patterns."""
        
        # Calculate relative path from src root
        rel_path = file_path.relative_to(self.src_root)
        path_depth = len(rel_path.parts) - 1
        
        # Pattern to find relative imports
        relative_import_pattern = r'from\s+(\.\.+)'
        
        def replace_relative_import(match):
            dots = match.group(1)
            levels = len(dots)
            
            # Calculate the correct import path
            if levels == 1:  # Single dot (same level)
                return f"from ."
            elif levels == 2:  # Double dot (parent level)
                return f"from .."
            else:
                # For deeper levels, calculate proper path
                parent_parts = rel_path.parts[:-levels+1]
                if parent_parts:
                    return f"from {'.'.join(parent_parts)}"
                else:
                    return match.group(0)  # Keep original if can't resolve
        
        return re.sub(relative_import_pattern, replace_relative_import, content)
    
    def _fix_import_patterns(self, content: str) -> str:
        """Fix inconsistent import patterns."""
        
        # Fix specific patterns found in the codebase
        patterns = [
            # Fix bridges imports
            (r'from\s+\.\.bridges\.', 'from bridges.'),
            (r'from\s+\.\.control\.', 'from control.'),
            (r'from\s+\.\.perception\.', 'from perception.'),
            (r'from\s+\.\.\.core\.', 'from core.'),
            
            # Fix multiline imports
            (r'from\s+\.\.bridges\.can_bridge\s+import',
             'from bridges.can_bridge import'),
        ]
        
        for pattern, replacement in patterns:
            content = re.sub(pattern, replacement, content)
        
        return content

def main():
    """Main execution function."""
    project_root = Path(__file__).parent.parent.parent
    fixer = ImportPathFixer(str(project_root))
    
    print("🔧 Fixing import paths across the codebase...")
    results = fixer.scan_and_fix()
    
    print(f"\n✅ Import path fixing complete:")
    print(f"   Files scanned: {results['files_scanned']}")
    print(f"   Files fixed: {results['files_fixed']}")
    print(f"   Fixes applied: {len(results['fixes_applied'])}")
    
    if results['fixes_applied']:
        print("\n📝 Fixes applied:")
        for fix in results['fixes_applied']:
            print(f"   - {fix}")

if __name__ == "__main__":
    main()