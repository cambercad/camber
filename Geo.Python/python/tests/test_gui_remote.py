"""Control a live pyglet GUI from a separate process, without CAD imports."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from tests.test_gui_testing import load

gui, testing, remote = load("gui"), load("gui_testing"), load("gui_remote")


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class RemoteGuiTests(unittest.TestCase):
    def test_separate_process_input_inspection_screenshot_and_errors(self):
        import pyglet
        window = pyglet.window.Window(420,240,visible=False)
        ui = gui.UI(window)
        state = {"text":"Initial", "clicks":0}
        gui_thread = threading.get_ident()

        def draw():
            self.assertEqual(gui_thread,threading.get_ident())
            window.clear()
            ui.new_frame()
            ui.SetNextWindowPos((0,0),ui.ImGuiCond_Always)
            ui.SetNextWindowSize((420,240),ui.ImGuiCond_Always)
            ui.Begin("Form",ui.ImGuiWindowFlags_NoDecoration)
            _,state["text"] = ui.InputText("Name",state["text"],width=200)
            if ui.Button("Apply"):
                state["clicks"] += 1
            ui.End()
            ui.render()

        try:
            driver = testing.GuiDriver(ui,draw)
            with remote.GuiControlServer(driver) as server, tempfile.TemporaryDirectory() as folder:
                endpoint=Path(folder)/"connection.json"
                endpoint.write_text(json.dumps(server.connection))
                script = fr"""
import importlib.util, json, socket, struct
spec=importlib.util.spec_from_file_location('remote', {str(Path(__file__).parents[1]/'camber'/'gui_remote.py')!r})
remote=importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote)
client=remote.GuiControlClient.from_file({str(endpoint)!r})
assert [c['label'] for c in client.controls()] == ['Name','Apply']
client.click('Name',panel='Form')
client.text('Measured bearing seat',replace=True)
client.key('TAB')
client.key('RETURN')
client.click('Apply',panel='Form')
for command, arguments in [('exec',{{'code':'raise Exception()'}}),('click',{{'label':'Missing'}}),('screenshot',{{'path':'forbidden.png'}})]:
    try: client.command(command,**arguments)
    except RuntimeError: pass
    else: raise AssertionError(command)
bad=remote.GuiControlClient(**{{**json.loads(open({str(endpoint)!r}).read()),'token':'wrong'}})
try: bad.controls()
except RuntimeError: pass
else: raise AssertionError('authentication')
# A malformed request must not terminate the listener.
with socket.create_connection(client.address) as connection:
    connection.sendall(b'not json\n')
    assert 'error' in json.loads(connection.makefile('rb').readline())
png=client.screenshot({str(Path(folder)/'gui.png')!r})
assert png[:8] == b'\x89PNG\r\n\x1a\n'
assert struct.unpack('>II',png[16:24]) == (420,240)
assert client.controls()
print('PASS')
"""
                child=subprocess.Popen([sys.executable,"-c",script],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
                try:
                    deadline=time.monotonic()+30
                    while child.poll() is None and time.monotonic()<deadline:
                        window.dispatch_events()
                        pyglet.clock.tick()
                        time.sleep(.005)
                    if child.poll() is None:
                        child.kill()
                    stdout,stderr=child.communicate(timeout=5)
                    self.assertEqual(0,child.returncode,stderr)
                    self.assertIn('PASS',stdout)
                    self.assertEqual('Measured bearing seat',state['text'])
                    self.assertEqual(2,state['clicks'])
                finally:
                    if child.poll() is None:
                        child.kill()
                        child.wait()
            # Closing is idempotent and removes the scheduled callback.
            server.close()
            self.assertTrue(server._closed.is_set())
            self.assertFalse(server._thread.is_alive())
        finally:
            ui.shutdown()
            window.close()

    def test_only_local_clients_are_supported(self):
        with self.assertRaises(ValueError):
            remote.GuiControlClient(host="example.com",port=1234,token="token")

    def test_close_cancels_queued_input_without_running_it(self):
        from unittest.mock import Mock
        driver = Mock()
        server = remote.GuiControlServer(driver)
        errors = []
        def send():
            try:
                remote.GuiControlClient(**server.connection).click("Apply")
            except RuntimeError as exc:
                errors.append(str(exc))
        client = threading.Thread(target=send)
        try:
            client.start()
            deadline = time.monotonic()+5
            while server._pending.empty() and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertFalse(server._pending.empty())
            server.close()
            client.join(timeout=5)
            self.assertFalse(client.is_alive())
            self.assertEqual(1, len(errors))
            driver.click.assert_not_called()
        finally:
            server.close()
            client.join(timeout=5)

