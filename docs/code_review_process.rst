.. _code_review_process:

==================
Code Review Process
==================

This document outlines the code review process and quality standards for the URC 2026 project. All code changes must pass through this process before merging.

Code Review Requirements
========================

**All Changes Require Review**
- No code can be merged without review and approval
- Self-review is not sufficient (different reviewer required)
- Emergency hotfixes need post-review validation

**Review Timeline**
- **Response Time**: Within 24 hours during development phase
- **Review Completion**: Within 48 hours for non-critical changes
- **Critical Path**: 4 hours for blocking issues

**Review Assignment**
- Use GitHub pull request assignments
- Rotate reviewers to distribute knowledge
- Domain experts for complex changes (hardware, autonomy, etc.)

Pre-Commit Checklist
====================

Before submitting for review, ensure:

**Code Quality**
- [ ] Black formatting applied (``black .``)
- [ ] Import sorting correct (``isort .``)
- [ ] Linting passes (``flake8 .``)
- [ ] Type checking passes (``mypy .``)
- [ ] Tests pass (``pytest``)

**Documentation**
- [ ] Docstrings added/updated for public APIs
- [ ] Code is self-documenting where possible
- [ ] READMEs updated for new features
- [ ] Breaking changes documented

**Testing**
- [ ] Unit tests added for new functionality
- [ ] Integration tests for system interactions
- [ ] Edge cases and error conditions tested
- [ ] Test coverage maintained >90%

**Architecture**
- [ ] Follows established patterns (see ``docs/quality_standards.rst``)
- [ ] Clean separation of concerns
- [ ] No circular dependencies
- [ ] Proper error handling

Code Review Checklist
====================

**Required for All Reviews**

**Code Quality & Standards**
- [ ] Follows established coding standards (PEP 8, type hints, etc.)
- [ ] Proper error handling with specific exceptions
- [ ] Logging appropriate and structured
- [ ] No hardcoded values (use parameters/config)
- [ ] Security considerations addressed

**Architecture & Design**
- [ ] Design follows established patterns
- [ ] Clean separation between layers (presentation/application/infrastructure)
- [ ] Dependency injection used appropriately
- [ ] SOLID principles followed
- [ ] Performance considerations included

**Testing & Quality**
- [ ] Unit tests comprehensive and meaningful
- [ ] Integration tests verify system behavior
- [ ] Error conditions properly tested
- [ ] Test coverage maintained or improved
- [ ] Tests are maintainable and readable

**Documentation & Maintainability**
- [ ] Code is self-documenting where possible
- [ ] Public APIs have complete docstrings
- [ ] Complex logic has explanatory comments
- [ ] Breaking changes are documented
- [ ] READMEs updated for user-facing changes

**ROS2-Specific Requirements**
- [ ] Proper QoS settings for real-time requirements
- [ ] Lifecycle management implemented correctly
- [ ] Message/service definitions follow standards
- [ ] Thread safety considered for multi-threaded code
- [ ] Resource cleanup in shutdown/error paths

**Hardware/Safety Critical Code**
- [ ] Fail-safe behavior implemented
- [ ] Emergency stop integration verified
- [ ] Hardware limits and validation included
- [ ] Watchdog timers and health monitoring
- [ ] Recovery strategies for hardware failures

Review Process Flow
===================

**1. Author Preparation**
- Complete pre-commit checklist
- Write clear PR description with context
- Reference related issues/design docs
- Mark as draft if work-in-progress

**2. Automated Checks**
- CI/CD pipeline runs quality checks
- Tests execute automatically
- Linting and formatting verified
- Coverage reports generated

**3. Reviewer Assessment**
- Review code against checklist
- Test changes locally if needed
- Verify integration with existing code
- Check for edge cases and error paths

**4. Feedback & Iteration**
- Specific, actionable feedback provided
- Suggestions include code examples
- Discussions resolve design questions
- Author addresses feedback with commits

**5. Approval & Merge**
- All checklist items satisfied
- CI/CD checks pass
- At least one approving review
- Author or reviewer merges (squash/rebase as appropriate)

PR Template
===========

Use this template for all pull requests:

**Title Format:**
```
[type]: Brief description of change

Types: feat, fix, docs, refactor, test, ci, chore
```

**Description Template:**

.. code-block:: markdown

    ## Description
    [Brief description of what this PR does]

    ## Changes Made
    - [List of key changes]
    - [Architecture decisions made]
    - [Breaking changes, if any]

    ## Testing
    - [How changes were tested]
    - [Test coverage impact]
    - [Manual testing performed]

    ## Checklist
    - [x] Pre-commit checks pass
    - [x] Code review checklist complete
    - [x] Documentation updated
    - [x] Tests added/updated

    ## Related Issues
    - Closes #123
    - Related to #456

    ## Additional Notes
    [Any additional context, concerns, or follow-up work needed]

Reviewer Guidelines
===================

**For Reviewers:**

**Be Constructive**
- Focus on code quality and maintainability
- Explain reasoning for suggestions
- Offer alternatives with trade-offs
- Recognize good patterns and encourage them

**Be Specific**
- Point to exact lines and provide examples
- Explain why changes improve code quality
- Reference coding standards and best practices
- Suggest concrete improvements

**Be Timely**
- Review within agreed timelines
- Communicate delays and availability
- Prioritize critical path items
- Don't block unnecessarily

**Be Collaborative**
- Ask questions to understand intent
- Discuss design decisions openly
- Help author learn and improve
- Share knowledge and context

**Approval Criteria**
- Code meets quality standards
- Tests are adequate and passing
- No critical bugs or security issues
- Documentation is complete
- Design decisions are sound

Quality Gates
=============

**Automated Gates (Must Pass)**
- Code formatting (black)
- Import sorting (isort)
- Linting (flake8)
- Type checking (mypy)
- Unit tests (pytest)
- Integration tests (pytest)
- Coverage requirements (>90%)

**Manual Gates (Reviewer Discretion)**
- Code review checklist completion
- Design quality assessment
- Performance impact evaluation
- Security review for sensitive code
- Documentation adequacy

**Override Conditions**
- **Never**: Automated quality gates
- **Rarely**: Code review checklist (requires lead approval)
- **Sometimes**: Coverage requirements (for legacy code or test infrastructure)

Continuous Improvement
======================

**Monthly Review Process**
- Review code review metrics and feedback
- Update standards based on lessons learned
- Improve tooling and automation
- Train team on new patterns and practices

**Metrics to Track**
- Average review time
- Rejection rates and reasons
- Bug rates post-release
- Time to merge approved PRs
- Coverage trends over time

**Team Learning**
- Regular code review discussions
- Share lessons learned from reviews
- Highlight exemplary code patterns
- Discuss architectural decisions

Common Issues & Solutions
=========================

**"This code is too complex"**
- Break into smaller functions/methods
- Add explanatory comments for complex logic
- Consider extracting helper functions
- Use established design patterns

**"Missing test coverage"**
- Identify what behavior isn't tested
- Add unit tests for new functionality
- Test edge cases and error conditions
- Mock external dependencies appropriately

**"Inconsistent with existing code"**
- Reference established patterns in codebase
- Follow existing naming conventions
- Match error handling styles
- Use consistent logging approaches

**"Performance concerns"**
- Profile code to identify bottlenecks
- Consider algorithmic complexity
- Optimize critical paths only
- Document performance requirements

**"Security considerations"**
- Validate all inputs
- Avoid hardcoded secrets
- Use safe defaults
- Consider attack vectors

Tools & Resources
=================

**Automated Tools**
- ``black`` - Code formatting
- ``isort`` - Import sorting
- ``flake8`` - Linting
- ``mypy`` - Type checking
- ``pytest`` - Testing framework
- ``coverage`` - Coverage reporting

**Manual Resources**
- `docs/quality_standards.rst` - Detailed coding standards
- `docs/architecture/` - System architecture documentation
- `.project_structure.md` - Codebase navigation guide
- GitHub issues/PRs - Historical decisions and discussions

**Getting Help**
- Ask in team chat for review help
- Reference approved PRs for examples
- Consult architecture docs for design guidance
- Reach out to experienced team members

Remember: **Code reviews are about improving code quality and sharing knowledge, not finding faults!** 🚀

