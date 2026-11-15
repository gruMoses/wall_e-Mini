"""Tests for the walle-ups retirement HOLD in bin/auto_deploy.sh.

The script is sourced by bash (it stops after its function definitions when
sourced) and walle_ups_hold is run in a temporary git repository, with a fake
`systemctl` (SYSTEMCTL) and a temporary unit directory (UNIT_DIR). No robot,
no real systemd. Reference: docs/x1202_power_watcher_design.md, section 7.
"""

import os
import shutil
import subprocess
import textwrap

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SCRIPT = os.path.join(_REPO_ROOT, "bin", "auto_deploy.sh")

pytestmark = pytest.mark.skipif(
    shutil.which("bash") is None or shutil.which("git") is None,
    reason="needs bash and git",
)

HOLD, NO_HOLD = 0, 1


def _git(repo, *args):
    subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@t", "-c",
         "commit.gpgsign=false", *args],
        cwd=repo, check=True, capture_output=True,
    )


def _rev(repo, ref="HEAD"):
    return subprocess.run(
        ["git", "rev-parse", ref], cwd=repo, check=True, capture_output=True, text=True,
    ).stdout.strip()


@pytest.fixture
def repo(tmp_path):
    """Commit A has bin/walle_ups.py and bin/other.py."""
    r = tmp_path / "repo"
    (r / "bin").mkdir(parents=True)
    (r / "bin" / "walle_ups.py").write_text("print('old')\n")
    (r / "bin" / "other.py").write_text("print('other')\n")
    (r / "README.md").write_text("x\n")
    _git(r, "init", "-q")
    _git(r, "add", "-A")
    _git(r, "commit", "-q", "-m", "A")
    return r


def _commit_change(repo, *, delete=None, touch=None, rename=None):
    for src, dst in rename or ():
        (repo / dst).parent.mkdir(parents=True, exist_ok=True)
        _git(repo, "mv", src, dst)
    for rel in delete or ():
        (repo / rel).unlink()
    for rel in touch or ():
        (repo / rel).write_text("changed\n")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "B")
    return _rev(repo)


def _fake_systemctl(tmp_path, *, output="", rc=None, stderr=""):
    """Like the real `systemctl list-unit-files PATTERN --no-legend` on the
    Pi (systemd 252, measured): no match prints nothing and exits 1; matches
    print one line each and exit 0. `rc` and `stderr` override."""
    if rc is None:
        rc = 0 if output.strip() else 1
    out = tmp_path / "systemctl_output"
    out.write_text(output)
    err = tmp_path / "systemctl_stderr"
    err.write_text(stderr)
    path = tmp_path / "fake_systemctl"
    path.write_text(textwrap.dedent(f"""\
        #!/usr/bin/env bash
        echo "$@" >> "{tmp_path}/systemctl_args"
        cat "{out}"
        cat "{err}" >&2
        exit {rc}
    """))
    path.chmod(0o755)
    return str(path)


def _run_hold(repo, local, remote, systemctl, unit_dir):
    # $0 must NOT be the script (the source guard compares BASH_SOURCE with
    # $0); AUTO_DEPLOY_FUNCTIONS_ONLY is a second stop before the deploy body.
    env = dict(os.environ, SYSTEMCTL=systemctl, UNIT_DIR=str(unit_dir),
               AUTO_DEPLOY_FUNCTIONS_ONLY="1")
    proc = subprocess.run(
        ["bash", "-c",
         'source "$1" || exit 99; type walle_ups_hold >/dev/null || exit 98; '
         'walle_ups_hold "$2" "$3"',
         "bash", _SCRIPT, local, remote],
        cwd=repo, env=env, capture_output=True, text=True,
    )
    assert proc.returncode in (HOLD, NO_HOLD), (proc.returncode, proc.stderr)
    return proc.returncode


@pytest.fixture
def unit_dir(tmp_path):
    d = tmp_path / "etc_systemd_system"
    d.mkdir()
    return d


UNITS = ("walle-ups.service                 enabled enabled\n"
         "walle-ups-wakecheck.service       enabled enabled\n")


def test_deletion_with_units_listed_holds(repo, tmp_path, unit_dir):
    local = _rev(repo)
    remote = _commit_change(repo, delete=["bin/walle_ups.py"])
    rc = _run_hold(repo, local, remote, _fake_systemctl(tmp_path, output=UNITS), unit_dir)
    assert rc == HOLD
    args = (tmp_path / "systemctl_args").read_text()
    assert "list-unit-files walle-ups* --no-legend" in args
    assert "is-enabled" not in args


def test_deletion_with_unit_file_in_directory_holds(repo, tmp_path, unit_dir):
    (unit_dir / "walle-ups.service").write_text("[Unit]\n")
    local = _rev(repo)
    remote = _commit_change(repo, delete=["bin/walle_ups.py"])
    assert _run_hold(repo, local, remote, _fake_systemctl(tmp_path), unit_dir) == HOLD


def test_deletion_with_dangling_unit_symlink_holds(repo, tmp_path, unit_dir):
    os.symlink("/nonexistent/walle-ups.service", unit_dir / "walle-ups.service")
    local = _rev(repo)
    remote = _commit_change(repo, delete=["bin/walle_ups.py"])
    assert _run_hold(repo, local, remote, _fake_systemctl(tmp_path), unit_dir) == HOLD


@pytest.mark.parametrize("output, rc, stderr, expected", [
    ("", 1, "", NO_HOLD),          # real systemd 252: no match
    ("", 0, "", NO_HOLD),          # an older systemd: no match, status 0
    (UNITS, 0, "", HOLD),          # real: units listed
    (UNITS, 1, "", HOLD),          # any stdout is "installed"
    ("", 1, "Failed to connect to bus\n", HOLD),   # stderr = doubt
    ("", 0, "warning\n", HOLD),
    ("", 2, "", HOLD),             # status above 1 = doubt
    ("", 127, "", HOLD),
])
def test_list_unit_files_statuses(repo, tmp_path, unit_dir, output, rc, stderr, expected):
    local = _rev(repo)
    remote = _commit_change(repo, delete=["bin/walle_ups.py"])
    systemctl = _fake_systemctl(tmp_path, output=output, rc=rc, stderr=stderr)
    assert _run_hold(repo, local, remote, systemctl, unit_dir) == expected


def test_rename_holds(repo, tmp_path, unit_dir):
    local = _rev(repo)
    remote = _commit_change(repo, rename=[("bin/walle_ups.py", "attic/walle_ups.py")])
    rc = _run_hold(repo, local, remote, _fake_systemctl(tmp_path, output=UNITS), unit_dir)
    assert rc == HOLD


def test_rename_with_nothing_installed_does_not_hold(repo, tmp_path, unit_dir):
    local = _rev(repo)
    remote = _commit_change(repo, rename=[("bin/walle_ups.py", "attic/walle_ups.py")])
    assert _run_hold(repo, local, remote, _fake_systemctl(tmp_path), unit_dir) == NO_HOLD


@pytest.mark.parametrize("leftover", ["walle-ups.service.bak", "walle-ups.service.d",
                                      "walle-ups-old.service"])
def test_leftover_with_other_name_does_not_hold(repo, tmp_path, unit_dir, leftover):
    if leftover.endswith(".d"):
        (unit_dir / leftover).mkdir()
        (unit_dir / leftover / "override.conf").write_text("[Service]\n")
    else:
        (unit_dir / leftover).write_text("[Unit]\n")
    local = _rev(repo)
    remote = _commit_change(repo, delete=["bin/walle_ups.py"])
    assert _run_hold(repo, local, remote, _fake_systemctl(tmp_path), unit_dir) == NO_HOLD


def test_wakecheck_unit_file_holds(repo, tmp_path, unit_dir):
    (unit_dir / "walle-ups-wakecheck.service").write_text("[Unit]\n")
    local = _rev(repo)
    remote = _commit_change(repo, delete=["bin/walle_ups.py"])
    assert _run_hold(repo, local, remote, _fake_systemctl(tmp_path), unit_dir) == HOLD


def test_deletion_with_systemctl_missing_holds(repo, tmp_path, unit_dir):
    local = _rev(repo)
    remote = _commit_change(repo, delete=["bin/walle_ups.py"])
    rc = _run_hold(repo, local, remote, str(tmp_path / "no_such_systemctl"), unit_dir)
    assert rc == HOLD


def test_deletion_with_nothing_installed_does_not_hold(repo, tmp_path, unit_dir):
    (unit_dir / "walle-power.service").write_text("[Unit]\n")  # not walle-ups*
    local = _rev(repo)
    remote = _commit_change(repo, delete=["bin/walle_ups.py"])
    assert _run_hold(repo, local, remote, _fake_systemctl(tmp_path), unit_dir) == NO_HOLD


def test_deletion_with_whitespace_only_output_does_not_hold(repo, tmp_path, unit_dir):
    local = _rev(repo)
    remote = _commit_change(repo, delete=["bin/walle_ups.py"])
    rc = _run_hold(repo, local, remote, _fake_systemctl(tmp_path, output="\n  \n"), unit_dir)
    assert rc == NO_HOLD


def test_no_deletion_does_not_hold(repo, tmp_path, unit_dir):
    local = _rev(repo)
    remote = _commit_change(repo, touch=["bin/walle_ups.py"])  # modified, not deleted
    rc = _run_hold(repo, local, remote, _fake_systemctl(tmp_path, output=UNITS), unit_dir)
    assert rc == NO_HOLD


def test_deletion_of_a_different_file_does_not_hold(repo, tmp_path, unit_dir):
    local = _rev(repo)
    remote = _commit_change(repo, delete=["bin/other.py"])
    rc = _run_hold(repo, local, remote, _fake_systemctl(tmp_path, output=UNITS), unit_dir)
    assert rc == NO_HOLD


def test_direction_local_to_remote(repo, tmp_path, unit_dir):
    # A change that brings the file BACK (remote has it, local does not) is
    # an addition from local to remote: no hold. Proves the diff direction.
    with_file = _rev(repo)
    without_file = _commit_change(repo, delete=["bin/walle_ups.py"])
    systemctl = _fake_systemctl(tmp_path, output=UNITS)
    assert _run_hold(repo, with_file, without_file, systemctl, unit_dir) == HOLD
    assert _run_hold(repo, without_file, with_file, systemctl, unit_dir) == NO_HOLD


def test_git_error_holds(repo, tmp_path, unit_dir):
    systemctl = _fake_systemctl(tmp_path)
    assert _run_hold(repo, _rev(repo), "0" * 40, systemctl, unit_dir) == HOLD
    assert _run_hold(repo, "0" * 40, _rev(repo), systemctl, unit_dir) == HOLD
    assert _run_hold(repo, _rev(repo), "no-such-ref", systemctl, unit_dir) == HOLD


def test_functions_only_on_a_real_run_logs_and_does_not_deploy(tmp_path):
    proc = subprocess.run(
        ["bash", _SCRIPT], cwd=tmp_path, capture_output=True, text=True,
        env=dict(os.environ, AUTO_DEPLOY_FUNCTIONS_ONLY="1"),
    )
    assert proc.returncode == 0
    assert "functions-only: no deploy" in proc.stderr


def test_script_wiring():
    """The HOLD sits before the pull and the docs-only decision, logs the
    exact line, and the smoke test compiles bin."""
    text = open(_SCRIPT).read()
    main = text[text.index('exec 9>"$LOCK"'):]
    hold = main.index('walle_ups_hold "$LOCAL" "$REMOTE"')
    assert hold < main.index("code_changed=")
    assert hold < main.index("git pull")
    assert hold > main.index("git fetch")
    assert 'log "HOLD walle-ups still installed"' in main
    assert "--diff-filter=D" not in text  # trees, not the diff (renames)
    assert "is-enabled" not in text.replace("`systemctl is-enabled` is not used", "")
    assert "compileall -q pi_app config.py bin" in main
    assert subprocess.run(["bash", "-n", _SCRIPT]).returncode == 0
