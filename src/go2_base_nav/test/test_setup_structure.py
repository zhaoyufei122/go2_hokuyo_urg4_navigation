import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_data_file_entries_are_destination_source_pairs():
    tree = ast.parse((PACKAGE_ROOT / "setup.py").read_text())
    setup_call = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "setup"
    )
    data_files = next(
        keyword.value
        for keyword in setup_call.keywords
        if keyword.arg == "data_files"
    )

    assert isinstance(data_files, ast.List)
    assert all(
        isinstance(entry, ast.Tuple) and len(entry.elts) == 2
        for entry in data_files.elts
    )
