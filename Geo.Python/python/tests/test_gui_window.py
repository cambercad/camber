"""Window chrome tests independent of CAD models and native geometry."""
import importlib.util
import os
from pathlib import Path
import sys
import unittest


def load(name, file):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).resolve().parents[1]/"camber"/file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


chrome = load("standalone_window_chrome", "gui_window.py")


class HitTestingTests(unittest.TestCase):
    def setUp(self):
        self.frame = chrome.WindowChrome.__new__(chrome.WindowChrome)
        self.frame.drag_rect = None

    def test_edges_corners_caption_and_buttons_are_distinct(self):
        for point, code in [((0,0),13),((799,0),14),((0,599),16),((799,599),17),
                            ((400,0),12),((400,599),15),((0,300),10),((799,300),11),
                            ((300,20),2),((730,20),1),((400,300),1)]:
            self.assertEqual(code, self.frame.hit_test(*point,800,600))

    def test_integrated_toolbar_drags_only_document_title(self):
        self.frame.drag_rect = (300,0,250,36)
        self.assertEqual(1,self.frame.hit_test(80,20,800,600))
        self.assertEqual(2,self.frame.hit_test(350,20,800,600))
        self.assertEqual(1,self.frame.hit_test(570,20,800,600))
        self.assertEqual(1,self.frame.hit_test(0,300,800,600,True))
        self.assertEqual(2,self.frame.hit_test(350,0,800,600,True))


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1" and sys.platform == "win32", "requires Windows OpenGL")
class NativeWindowTests(unittest.TestCase):
    def test_native_hit_testing_work_area_restore_and_draw(self):
        import ctypes as c
        from ctypes import wintypes as w
        import pyglet
        gui = load("standalone_window_gui", "gui.py")
        window = chrome.chrome_window_class(pyglet.window.Window)(width=800,height=600,visible=False,resizable=True,style="borderless")
        try:
            ui = gui.UI(window)
            ui.enable_control_inspection()
            frame = chrome.WindowChrome(window,ui,title="Control gallery")
            native = frame._native
            user = native.user
            user.SendMessageW.argtypes = [w.HWND,w.UINT,c.c_size_t,c.c_ssize_t]
            user.SendMessageW.restype = c.c_ssize_t
            user.ClientToScreen.argtypes = [w.HWND,c.POINTER(w.POINT)]
            for xy, expected in [((300,20),2),((730,20),1),((1,300),10)]:
                p = w.POINT(*xy)
                user.ClientToScreen(native.hwnd,c.byref(p))
                packed = (p.x & 65535) | ((p.y & 65535)<<16)
                self.assertEqual(expected,user.SendMessageW(native.hwnd,0x84,0,packed))
                self.assertEqual(-1 if expected != 1 else 1,user.SendMessageW(native.child,0x84,0,packed))
            before = window.get_size()
            proposed = w.POINT(1200,900)
            self.assertEqual(1,user.SendMessageW(native.hwnd,0x02E4,144,c.addressof(proposed)))
            self.assertEqual((1200,900),(proposed.x,proposed.y))
            native.toggle_maximize()
            window.dispatch_events()
            self.assertTrue(frame.maximized)
            class MonitorInfo(c.Structure):
                _fields_ = [("size",w.DWORD),("monitor",w.RECT),("work",w.RECT),("flags",w.DWORD)]

            monitor = MonitorInfo()
            monitor.size = c.sizeof(monitor)
            user.GetMonitorInfoW.argtypes = [w.HANDLE,c.POINTER(MonitorInfo)]
            self.assertTrue(user.GetMonitorInfoW(user.MonitorFromWindow(native.hwnd,2),c.byref(monitor)))
            self.assertEqual((monitor.work.right-monitor.work.left,monitor.work.bottom-monitor.work.top),window.get_size())
            native.toggle_maximize()
            window.dispatch_events()
            self.assertFalse(frame.maximized)
            self.assertEqual(before,window.get_size())
            window.set_size(720,480)
            window.dispatch_events()
            self.assertEqual((720,480),window.get_size())
            for theme in ("light","dark"):
                ui.set_theme(theme)
                ui.new_frame()
                frame.draw()
                ui.render()
                self.assertEqual(3,len([item for item in ui.controls if "window_" in item.label]))
            driver_module = load("standalone_window_driver", "gui_testing.py")

            def draw():
                window.clear()
                ui.new_frame()
                frame.draw()
                ui.render()

            driver = driver_module.GuiDriver(ui,draw)
            driver.click("##window_Maximize")
            self.assertTrue(frame.maximized)
            driver.click("##window_Restore")
            self.assertFalse(frame.maximized)
            close_requests = []

            def guarded_close():
                close_requests.append(True)
                return pyglet.event.EVENT_HANDLED

            window.push_handlers(on_close=guarded_close)
            driver.click("##window_Close")
            driver.frame()
            self.assertEqual([True],close_requests)
            self.assertIsNotNone(window._hwnd)
        finally:
            window.close()


if __name__ == "__main__":
    unittest.main()
