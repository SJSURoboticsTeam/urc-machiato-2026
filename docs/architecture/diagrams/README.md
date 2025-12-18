# PlantUML Diagrams - URC 2026 Mars Rover

This directory contains comprehensive PlantUML diagrams documenting the system architecture of the URC 2026 Mars Rover Autonomy System.

## Directory Structure

```
diagrams/
├── critical/           # Phase 1: Critical safety and operation diagrams
│   ├── 01_core_system_state_machine.puml
│   ├── 02_mission_state_machines.puml
│   └── 03_safety_system_architecture.puml
├── high/              # Phase 2: Core architecture and communication
│   ├── 04_system_component_architecture.puml
│   ├── 05_ros2_communication_architecture.puml
│   └── 06_mission_behavior_classes.puml
├── medium/            # Phase 3: Detailed design and interfaces
│   ├── 07_ros2_interface_definitions.puml
│   ├── 08_frontend_component_architecture.puml
│   ├── 09_hardware_interface_architecture.puml
│   └── 10_configuration_management.puml
└── low/               # Phase 4: Supporting systems (future)
    └── (additional diagrams)
```

## Diagram Standards

### File Naming Convention
- `NN_descriptive_name.puml`
- NN = priority number (01-99)
- Use snake_case for descriptive names

### Metadata Requirements
Each diagram must include:
```plantuml
note as N#
  **Created:** $(date)
  **Source:** path/to/source/files
  **Stakeholders:** Team1, Team2, Team3
end note
```

### Color Coding Standards
- 🔵 Light Blue: User Interface Layer
- 🟢 Light Green: Autonomy/Core Systems
- 🟡 Yellow: Processing/Perception
- 🟠 Orange: Hardware Interfaces
- 🔴 Red: Safety/Emergency Systems
- 🟣 Purple: Services/Configuration
- ⚫ Gray: External Systems

## Maintenance Workflow

### When Code Changes
1. Identify affected diagrams
2. Update PlantUML source files
3. Regenerate documentation
4. Review with stakeholders
5. Commit with related code changes

### Adding New Diagrams
1. Determine priority level (critical/high/medium/low)
2. Follow naming convention
3. Add to appropriate directory
4. Update `diagrams.rst` index
5. Test documentation build

## Generation

### Prerequisites
```bash
# Install PlantUML (Ubuntu/Debian)
sudo apt-get install plantuml

# Or download from: https://plantuml.com/download
```

### Build Documentation
```bash
cd docs
make html
```

### View Diagrams
```bash
# Open generated HTML
xdg-open _build/html/architecture/diagrams.html
```

## Tools

### Recommended Editors
- **VS Code** with PlantUML extension
- **IntelliJ IDEA** with PlantUML plugin
- **PlantUML Web Server** for quick previews

### Online Tools
- **PlantUML Web Server**: https://www.plantuml.com/plantuml
- **Live Preview**: Real-time diagram editing

## Quality Assurance

### Review Checklist
- [ ] Diagram matches current codebase
- [ ] All major components included
- [ ] Relationships accurately represented
- [ ] Consistent styling and colors
- [ ] Readable at different zoom levels
- [ ] Source references are current
- [ ] Stakeholders identified

### Performance
- Keep diagram complexity manageable
- Use subgraphs for large diagrams
- Consider splitting very large diagrams
- Optimize for web viewing

## Troubleshooting

### Common Issues
1. **Syntax Errors**: Use PlantUML validator
2. **Missing Dependencies**: Check Sphinx PlantUML extension
3. **Large Diagrams**: Split into multiple diagrams
4. **Color Issues**: Use standard color palette

### Getting Help
- PlantUML Guide: https://plantuml.com/guide
- Sphinx PlantUML: https://sphinxcontrib-plantuml.readthedocs.io/
- Team Documentation Lead

---

**Maintainers**: Architecture Team, Documentation Team
**Last Updated**: $(date)
**Version**: 1.0


