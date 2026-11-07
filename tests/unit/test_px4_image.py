"""docker/px4-sitl: every patch in the directory is copied into the image build and applied."""

import re

import pytest

from framework.config import REPO_ROOT

pytestmark = pytest.mark.unit

IMAGE_DIR = REPO_ROOT / "docker" / "px4-sitl"


def test_every_patch_is_copied_and_applied_by_the_dockerfile():
    dockerfile = (IMAGE_DIR / "Dockerfile").read_text()
    patches = sorted(p.name for p in IMAGE_DIR.glob("*.patch"))
    assert patches
    copy = re.search(r"^COPY ((?:\S+\.patch )+)/tmp/", dockerfile, re.MULTILINE)
    apply = re.search(r"git apply ((?:/tmp/\S+\.patch ?)+)", dockerfile)
    assert copy and apply
    assert sorted(copy.group(1).split()) == patches
    assert sorted(p.removeprefix("/tmp/") for p in apply.group(1).split()) == patches


def test_the_range_finder_patch_changes_only_the_stuck_threshold():
    changed = [line for line in (IMAGE_DIR / "rng_stuck_detector.patch").read_text().splitlines()
               if line[:1] in "+-" and not line.startswith(("+++", "---"))]
    assert len(changed) == 2
    assert "_stuck_threshold{0.1f}" in changed[0] and "_stuck_threshold{0.f}" in changed[1]
