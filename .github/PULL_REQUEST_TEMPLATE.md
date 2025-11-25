# Pull Request

## Description

<!-- Provide a brief description of the changes in this PR -->

## Type of Change

<!-- Mark the relevant option(s) with an 'x' -->

- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update
- [ ] Performance improvement
- [ ] Code refactoring
- [ ] Dependency update

## Testing

<!-- Describe the tests you ran and how to reproduce them -->

- [ ] Tests pass locally (`make test` or `cargo test`)
- [ ] Coverage maintained or improved
- [ ] Tested on relevant platforms (specify):
  - [ ] x86_64 Linux
  - [ ] aarch64 Linux (Maivin/Raivin)
  - [ ] Virtual CAN (vcan0)
  - [ ] Hardware radar sensor

## Code Quality Checklist

- [ ] Code follows project style guidelines (`cargo fmt --all -- --check`)
- [ ] No new warnings (`cargo clippy --all-targets --all-features -- -D warnings`)
- [ ] Self-reviewed code and addressed all concerns
- [ ] Added/updated comments for complex logic
- [ ] No hardcoded values or secrets
- [ ] Error handling is appropriate and robust

## Documentation

- [ ] README.md updated (if public API or features changed)
- [ ] ARCHITECTURE.md updated (if design changed)
- [ ] API documentation added/updated (rustdoc)
- [ ] CHANGELOG.md updated under `[Unreleased]` section
- [ ] Examples updated or added (if applicable)

## Developer Certificate of Origin (DCO)

- [ ] All commits are signed off with `git commit -s`
- [ ] I certify that I have the right to submit this work under Apache-2.0

By contributing, you agree that your contributions will be licensed under the Apache-2.0 license and that you have the right to submit the work under this license.

## Related Issues

<!-- Link to related issues using #issue_number or "Fixes #123" -->

Fixes #

## Additional Context

<!-- Add any other context, screenshots, or information that reviewers should know -->

## Reviewer Notes

<!-- Optional: Any specific areas you'd like reviewers to focus on -->

---

**For Maintainers:**
- [ ] Reviewed for security implications
- [ ] Checked license compliance (if dependencies changed)
- [ ] Verified EdgeFirst integration compatibility
- [ ] Performance impact assessed (if applicable)
