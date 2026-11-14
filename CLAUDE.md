@AGENTS.md

## This fork

- Start with `FORK.md`. Patches are applied by the maintainer from `~/DJI-Craft/` with `git apply`; one command block at a time.
- Commit messages must pass commitizen (`type(scope): subject`), as CI checks them.

## Claude Code

- Do NOT include the Claude session link in commit messages.
- DO end every commit message with the "Co-Authored-By: Claude …" trailer.
- Before a release, follow `docs/RELEASE.md`; update it when packaging,
  versioning or the release flow changes.
