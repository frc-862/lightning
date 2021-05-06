# GitHub Workflows

## Build

CI pipeline runs on all pushes/PRs.

## Deploy

Needs to be run manually.

Takes a version number (like 0.1.2) and a release title (like "Initial Release").

### Generate Release

Creates a release with the provided title and tags it with the version number.

### Create Package

Creates a package with the given version number.

This package is added to the GitHub Package Registry.
