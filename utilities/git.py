import pathlib
import subprocess


def describe() -> str:
    robot_dir = pathlib.Path(__file__).parents[1]
    # would use a hidden file here, but deploy skips dotfiles
    fname = robot_dir / "gitid"

    if (robot_dir / ".git").is_dir():
        commit_hash = subprocess.check_output(
            ("git", "describe", "--broken", "--always", "--tags"), text=True
        ).strip()
        branch = subprocess.check_output(
            ("git", "branch", "--show-current"), text=True
        ).strip()
        desc = f"{commit_hash} ({branch})" if branch else commit_hash

        try:
            with open(fname, "r") as f:
                needs_write = f.read() != desc
        except FileNotFoundError:
            needs_write = True

        if needs_write:
            with open(fname, "w") as f:
                f.write(desc)

        return desc

    with open(fname) as f:
        return f.read()
