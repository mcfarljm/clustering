# Copyright 2020 D-Wave Systems Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import unittest
import subprocess


class IntegrationTests(unittest.TestCase):

    def test_clustering(self):
        output = subprocess.check_output([sys.executable, "clustering.py"])
        output = str(output).upper()
        if os.getenv('DEBUG_OUTPUT'):
            print("Example output\n" + output)

        with self.subTest(msg="Verify if output contains 'Your plots are saved to"):
            self.assertIn("Your plots are saved to".upper(), output)
        with self.subTest(msg="Verify if error string contains in output"):
            self.assertNotIn("ERROR", output)
        with self.subTest(msg="Verify if warning string contains in output"):
            self.assertNotIn("WARNING", output)

if __name__ == '__main__':
    unittest.main()
