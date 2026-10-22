import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from rby1_tray import cli


class SourceFreezeTests(unittest.TestCase):
    def test_cached_process_rejects_changed_sources_before_next_episode(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'run'
            with patch.object(cli, 'source_hashes', return_value={'session.py': 'changed'}):
                with self.assertRaisesRegex(RuntimeError, 'process startup'):
                    cli.run_episode({}, output)
            self.assertFalse(output.exists())

    def test_source_change_during_run_prevents_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            with patch.object(cli, 'source_hashes', return_value={'session.py': 'changed'}):
                with self.assertRaisesRegex(RuntimeError, 'during the experiment'):
                    cli.write_manifest(output, cli._LOADED_SOURCES)
            self.assertFalse((output / 'manifest.json').exists())
