"""Archive a stopped task's tmpfs logs under their original names (Linux/Noetic)."""
import argparse
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import tempfile
import time


def writer(root):
    prefix = str(root) + '/'
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            for fd in (proc / 'fd').iterdir():
                try:
                    if not os.readlink(fd).startswith(prefix):
                        continue
                    info = (proc / 'fdinfo' / fd.name).read_text()
                    flags = next(line.split()[1] for line in info.splitlines() if line.startswith('flags:'))
                    if int(flags, 8) & os.O_ACCMODE:
                        return proc.name
                except FileNotFoundError:
                    continue
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return None


def copy_logs(source, destination):
    if not source.is_dir():
        return
    destination.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.log-archive-', dir=destination) as work:
        copied = Path(work) / 'logs'
        shutil.copytree(source, copied, symlinks=True)
        # roslaunch's "latest" can be an absolute link into the volatile run directory.
        for link in copied.rglob('*'):
            if link.is_symlink():
                target = os.readlink(link)
                if os.path.isabs(target):
                    inside = Path(target).relative_to(source)
                    link.unlink()
                    link.symlink_to(os.path.relpath(copied / inside, link.parent))
        for entry in copied.iterdir():
            final = destination / entry.name
            if os.path.lexists(final):
                # A retry may encounter files already installed before a second destination failed.
                if (stat.S_IFMT(entry.lstat().st_mode) != stat.S_IFMT(final.lstat().st_mode)
                        or subprocess.run(['diff', '-qr', '--no-dereference', '--', str(entry), str(final)],
                                          stdout=subprocess.DEVNULL).returncode):
                    raise FileExistsError('归档目标已有不同内容：' + str(final))
            else:
                entry.rename(final)
        subprocess.run(['sync', '-f', str(destination)], check=True)


def archive(root, task, fusion):
    root = root.resolve(strict=True)
    if (root.parent != Path('/dev/shm') or not root.name.startswith(('nav_test_logs.', 'fusion_logs.'))
            or any(p.is_symlink() or p.name not in ('task', 'fusion') for p in root.iterdir())
            or ((root / 'task').exists() and task is None)):
        raise ValueError('暂存目录不属于本次任务')
    # A tee attached to roslaunch can outlive the launcher briefly while it drains stdout.
    for _ in range(21):
        pid = writer(root)
        if pid is None:
            break
        time.sleep(.1)
    else:
        raise RuntimeError('日志仍由 PID ' + pid + ' 写入')
    if task is not None:
        copy_logs(root / 'task', task)
    copy_logs(root / 'fusion', fusion)
    shutil.rmtree(root)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--task', type=Path)
    parser.add_argument('--fusion', required=True, type=Path)
    args = parser.parse_args()
    try:
        archive(args.root, args.task, args.fusion)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        sys.exit('日志归档失败，暂存目录保留在 %s (%s)' % (args.root, error))
