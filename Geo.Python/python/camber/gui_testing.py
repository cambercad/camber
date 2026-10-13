"""In-process tests for the reusable pyglet GUI; no desktop automation required.

Pass the application's normal frame callback. Controls are addressed by their
labels and optional panel names; mouse coordinates use logical pixels, top-down.
The driver sends real window events rather than changing control/model state.
"""


class GuiDriver:
    def __init__(self, ui, draw, *, capture=None):
        self.ui = ui
        self.window = ui.window
        self.draw = draw
        self.capture = capture
        ui.enable_control_inspection()
        self.frame()

    def frame(self):
        self.window.switch_to()
        self.window.dispatch_pending_events()
        self.draw()
        # Immediate-mode applications may apply queued actions after drawing.
        # Present their resulting controls before the next simulated input.
        self.draw()

    def _event(self, name, *args):
        self.window.dispatch_event(name, *args)
        self.frame()

    def control(self, label, *, panel=None):
        items = [item for item in self.ui.controls
                 if (item.label == label or item.label.split("##", 1)[0] == label)
                 and (panel is None or item.panel == panel)]
        exact = [item for item in items if item.label == label]
        if exact:
            items = exact
        if len(items) != 1:
            raise LookupError(f"Expected one visible control {label!r} in {panel!r}; found {len(items)}")
        return items[0]

    def move(self, x, y):
        self._event("on_mouse_motion", x, self.window.height-y, 0, 0)

    def click(self, label=None, *, panel=None, x=None, y=None, count=1, modifiers=0):
        from pyglet.window import mouse
        if count < 1:
            raise ValueError("Click count must be positive")
        if label is not None:
            item = self.control(label, panel=panel)
            if not item.enabled:
                raise ValueError(f"Control {label!r} is disabled")
            left, top, width, height = item.rect
            x, y = left+width/2, top+height/2
        if x is None or y is None:
            raise ValueError("Supply a control label or both x and y")
        if not (0 <= x < self.window.width and 0 <= y < self.window.height):
            raise ValueError("Click must be inside the window")
        self.move(x, y)
        if label is not None:
            # Hovering draws a fresh frame; use its layout if controls moved.
            item = self.control(label, panel=panel)
            if not item.enabled:
                raise ValueError(f"Control {label!r} is disabled")
            left, top, width, height = item.rect
            x, y = left+width/2, top+height/2
            self.move(x, y)
        for _ in range(count):
            self._event("on_mouse_press", x, self.window.height-y, mouse.LEFT, modifiers)
            self._event("on_mouse_release", x, self.window.height-y, mouse.LEFT, modifiers)

    def key(self, symbol, *, modifiers=0):
        from pyglet.window import key
        if isinstance(symbol, str):
            symbol = getattr(key, symbol.upper())
        self.window.dispatch_event("on_key_press", symbol, modifiers)
        self.window.dispatch_event("on_key_release", symbol, 0)
        self.frame()

    def text(self, text, *, replace=False):
        from pyglet.window import key
        if replace:
            self.key("A", modifiers=key.MOD_CTRL)
        self._event("on_text", text)

    def scroll(self, x, y, amount):
        self.move(x, y)
        self._event("on_mouse_scroll", x, self.window.height-y, 0, amount)

    def drag(self, start, end, *, steps=8, button="left", modifiers=0):
        from pyglet.window import mouse
        if steps < 1:
            raise ValueError("Drag requires at least one step")
        buttons = {"left":mouse.LEFT,"middle":mouse.MIDDLE,"right":mouse.RIGHT}
        if button not in buttons:
            raise ValueError("Drag button must be left, middle or right")
        pressed = buttons[button]
        x, y = start
        self.move(x, y)
        self._event("on_mouse_press", x, self.window.height-y, pressed, modifiers)
        for index in range(1, steps+1):
            nx = start[0]+(end[0]-start[0])*index/steps
            ny = start[1]+(end[1]-start[1])*index/steps
            self._event("on_mouse_drag", nx, self.window.height-ny, nx-x, y-ny, pressed, modifiers)
            x, y = nx, ny
        self._event("on_mouse_release", x, self.window.height-y, pressed, modifiers)

    def screenshot(self, path=None):
        """Return a pyglet image and optionally save it; capture only this GUI."""
        import pyglet
        self.frame()
        image = self.capture() if self.capture else pyglet.graphics.framebuffer.get_screenshot()
        if path is not None:
            image.save(str(path))
        return image
