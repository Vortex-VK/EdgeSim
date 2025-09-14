import os
from pathlib import Path

def collect_py_files(start_dir: str, out_file: str = "all_code.txt") -> None:
    """
    Recursively walk start_dir, collect contents of all .py files,
    and write them into out_file with separators.
    """
    start_path = Path(start_dir).resolve()
    out_path = Path(out_file).resolve()

    with out_path.open("w", encoding="utf-8") as out:
        for root, _, files in os.walk(start_path):
            for fname in files:
                if fname.endswith(".py") or fname.endswith(".toml"):
                    fpath = Path(root) / fname
                    try:
                        text = fpath.read_text(encoding="utf-8")
                    except Exception as e:
                        print(f"Skipping {fpath}: {e}")
                        continue
                    out.write(f"\n\n# ===== {fpath.relative_to(start_path)} =====\n")
                    out.write(text)
    print(f"[OK] Collected .py files from {start_path} into {out_path}")

if __name__ == "__main__":
    # Change '.' to the starter folder path if needed
    collect_py_files(".")
