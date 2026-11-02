# Is this Python good enough to build the project on?
#
# Run by install.bat BEFORE anything else, with whatever Python it found.
# Plain ASCII and old syntax on purpose: it must run even on a Python too
# old for the rest of the project, and only say "no" (exit code 1).
#
# Checks exactly the things that broke real installs:
#   - 3.12 (every package we need has a ready wheel for it);
#   - a released build, not alpha/beta: 3.12.0a5 shipped a pip that
#     crashed on "InvalidVersion: '0.dev0'";
#   - 64-bit: numpy, mediapipe, opencv have no 32-bit Windows wheels;
#   - tkinter present: the control window is tkinter.
import sys

if "--help" in sys.argv or "-h" in sys.argv:
    print("usage: python_ok.py [--where]  - exit 0 if this Python fits")
    sys.exit(0)

ok = (sys.version_info[:2] == (3, 12)
      and sys.version_info.releaselevel == "final"
      and sys.maxsize > 2 ** 32)
if ok:
    try:
        import tkinter  # noqa: F401
    except Exception:
        ok = False
if ok and "--where" in sys.argv:
    # install.bat asks for the full path this way: no parentheses in its
    # command line, which a "for /f" block in cmd.exe can choke on.
    sys.stdout.write(sys.executable + "\n")
sys.exit(0 if ok else 1)
