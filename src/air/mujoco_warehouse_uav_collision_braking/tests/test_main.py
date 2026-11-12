"""CLI default-path contract tests."""

import unittest
from pathlib import Path
from unittest.mock import patch

from warehouse_braking.main import main


class MainTests(unittest.TestCase):
    def test_default_collection_and_training_share_dataset_directory(self):
        with (
            patch("warehouse_braking.dataset.collect", return_value={"splits": {}}) as collect,
            patch("warehouse_braking.train_model.train", return_value={"ok": True}) as train,
        ):
            main(["--collect"])
            main(["--train"])

        self.assertEqual(collect.call_args.args[0], Path("dataset"))
        self.assertEqual(train.call_args.args[0], Path("dataset"))

    def test_explicit_collection_output_is_preserved(self):
        with patch("warehouse_braking.dataset.collect", return_value={"splits": {}}) as collect:
            main(["--collect", "--output", "custom-data"])

        self.assertEqual(collect.call_args.args[0], Path("custom-data"))

    def test_benchmark_default_output_is_not_changed_by_collection_flag(self):
        with patch("warehouse_braking.benchmark.benchmark", return_value={"ok": True}) as benchmark:
            main(["--collect", "--benchmark"])

        self.assertEqual(benchmark.call_args.args[1], Path("results/benchmark.json"))


if __name__ == "__main__":
    unittest.main()
