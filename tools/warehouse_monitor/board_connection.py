"""SSH paths are interpreted on the board, independently of the desktop checkout."""
import hashlib
import os
import shlex


DEFAULT_ROOT = "~/orange"


def settings(env_file):
    values = {"ORANGE_SSH_HOST": "orange", "ORANGE_REMOTE_ROOT": DEFAULT_ROOT}
    if env_file.is_file():
        for line in env_file.read_text().splitlines():
            name, sep, raw = line.partition("=")
            name = name.strip()
            if sep and name in values:
                fields = shlex.split(raw, comments=True)
                if len(fields) > 1:
                    raise ValueError(f"{name} 含空格时请加引号")
                if fields and fields[0]:
                    values[name] = fields[0]
    return {name: os.environ.get(name) or value for name, value in values.items()}


def remote_path(path):
    """Return a shell word. Only the remote login user's HOME is expanded."""
    if path.startswith("/"):
        return shlex.quote(path)
    if path == "~":
        return '"$HOME"'
    if path.startswith("~/"):
        path = path[2:]
    return '"$HOME"/' + shlex.quote(path)


def in_checkout(root, command):
    return "cd " + remote_path(root) + " && " + command


def fingerprint(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
