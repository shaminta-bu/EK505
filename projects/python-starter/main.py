import os
import sys
from pathlib import Path

workspace_root = Path(__file__).resolve().parents[2]

print("EK 505 Python starter is running.")
print(f"Python executable: {sys.executable}")
print(f"Workspace root: {workspace_root}")

pythonpath = os.getenv("PYTHONPATH")
if pythonpath:
    print(f"PYTHONPATH: {pythonpath}")
else:
    print("PYTHONPATH is not set in this shell.")
