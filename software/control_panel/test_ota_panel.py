"""Offline OTA input and HTTP failure checks; no robot connection or GUI."""
import ast
from email import policy
from email.parser import BytesParser
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import Mock
import zlib

ROOT = Path(__file__).resolve().parents[2] / 'firmware/argos_f446re'
tree = ast.parse(Path(__file__).with_name('argos_control_panel.py').read_text(encoding='utf-8-sig'))
names = {'validate_ota_image', '_wireless_update_worker'}
functions = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name in names]


class OtaPanelTests(unittest.TestCase):
    def setUp(self):
        if not (ROOT / 'Debug/ARGOS_OTA.bin').is_file() or not (ROOT / 'Debug/ARGOS_BOOTSTRAP.bin').is_file():
            self.skipTest('Build the published STM32 OTA and bootstrap images before OTA tests.')
        self.image = (ROOT / 'Debug/ARGOS_OTA.bin').read_bytes()
        self.connection = Mock()
        self.factory = Mock(return_value=self.connection)
        self.env = {'time': time, 'WIFI_UPDATE_PORT': 3335,
                    'http': SimpleNamespace(client=SimpleNamespace(HTTPConnection=self.factory))}
        exec(compile(ast.Module(body=functions, type_ignores=[]), '<panel>', 'exec'), self.env)

    def test_current_ota_accepted(self):
        self.env['validate_ota_image'](self.image)

    def test_bootstrap_rejected(self):
        with self.assertRaises(ValueError):
            self.env['validate_ota_image']((ROOT / 'Debug/ARGOS_BOOTSTRAP.bin').read_bytes())

    def test_invalid_vectors_and_sizes_rejected(self):
        mutations = [b'', b'1234567', bytes(480 * 1024 + 1)]
        for stack, reset in [(0x10000000, 0x08008009), (0x20020000, 0x08000009),
                             (0x20020000, 0x08008008), (0x20020000, 0x0807ffff)]:
            mutations.append(stack.to_bytes(4, 'little') + reset.to_bytes(4, 'little') + self.image[8:])
        for image in mutations:
            with self.subTest(size=len(image), vector=image[:8].hex()), self.assertRaises(ValueError):
                self.env['validate_ota_image'](image)

    def run_worker(self, status=200, detail=b'UPDATE PASS: verified', error=None):
        response = Mock(status=status, reason='test')
        response.read.return_value = detail
        self.connection.getresponse.return_value = response
        self.connection.request.side_effect = error
        panel = SimpleNamespace(after=lambda delay, callback: callback(),
                                _finish_wireless_update=Mock())
        self.env['_wireless_update_worker'](panel, 'offline.invalid', self.image,
                                          zlib.crc32(self.image), 123)
        self.connection.close.assert_called_once()
        return panel._finish_wireless_update.call_args.args

    def test_multipart_preserves_exact_binary_and_crc_headers(self):
        self.assertTrue(self.run_worker()[0])
        args, kwargs = self.connection.request.call_args
        self.assertEqual(args, ('POST', '/stm32'))
        headers = kwargs['headers']
        message = BytesParser(policy=policy.default).parsebytes(
            ('Content-Type: ' + headers['Content-Type'] + '\r\nMIME-Version: 1.0\r\n\r\n').encode()
            + kwargs['body'])
        parts = list(message.iter_parts())
        self.assertEqual(len(parts), 1)
        self.assertEqual(parts[0].get_payload(decode=True), self.image)
        self.assertEqual(headers['X-ARGOS-Size'], str(len(self.image)))
        self.assertEqual(headers['X-ARGOS-CRC32'], str(zlib.crc32(self.image)))

    def test_http_rejection_reported(self):
        ok, detail = self.run_worker(400, b'CACHE FAIL: upload size/CRC mismatch')
        self.assertFalse(ok)
        self.assertIn('CACHE FAIL', detail)

    def test_unexpected_success_body_rejected(self):
        self.assertFalse(self.run_worker(200, b'unexpected response')[0])

    def test_timeout_reported_and_connection_closed(self):
        ok, detail = self.run_worker(error=TimeoutError('simulated timeout'))
        self.assertFalse(ok)
        self.assertIn('simulated timeout', detail)


if __name__ == '__main__':
    unittest.main(verbosity=2)
