import ast
import os
import unittest


MODULE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(MODULE_DIR, "launch", "survey.launch.py")


class LaunchDescriptionTests(unittest.TestCase):
    def test_nodes_use_the_installed_package_executable(self):
        with open(LAUNCH_FILE, encoding="utf-8") as launch_file:
            tree = ast.parse(launch_file.read())

        nodes = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "Node"
        ]

        self.assertEqual(len(nodes), 2)
        for node in nodes:
            keywords = {keyword.arg: keyword.value for keyword in node.keywords}
            self.assertEqual(
                ast.literal_eval(keywords["package"]),
                "drone_lawnmower_survey",
            )
            self.assertEqual(ast.literal_eval(keywords["executable"]), "survey")


if __name__ == "__main__":
    unittest.main(verbosity=2)
