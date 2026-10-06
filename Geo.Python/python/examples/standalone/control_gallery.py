"""Standalone gallery for the reusable, pyglet-only dock UI.

Run with ``python Geo.Python/python/examples/standalone/control_gallery.py``.
"""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import pyglet

from camber.gui import UI


window = pyglet.window.Window(1280, 760, caption="Control Gallery", resizable=True)
ui = UI(window)
ui.insets = (0, 72, 0, 28)
ui.dock_panel("Tests", "left", fraction=.165)
ui.dock_panel("Output", "bottom", fraction=.31)
selected = ["Text"]
plain = ["Type something here"]
secret = ["password"]
message = ["Control gallery ready."]
filter_text = [""]
output = ["Control gallery ready.", "Dock panels can be moved by their tabs.",
          "Drag a divider to resize adjacent panes."]
focus_first = [False]
closed = [False]
checked = [True]
switched = [False]
radio = ["Balanced"]
choice = ["Medium"]
list_selection = [1]
tree_selection = ["Dashboard"]
notes = ["A compact editor with wrapping and multiple lines."]
level = [0.46]
view_mode = ["Controls"]
grouped = [True]
grid_filter = [""]
grid_selection = [5]

grid_columns = ("Name", "Postal Zip", "Address", "Country", "Region", "Status")
grid_rows = [line.split("|") + [True] for line in """Rajah Reeves|1665|5480 Consectetuer Ave|Vietnam|Jönköpings län
Ezra Guerra|88696|9261 Mi Avenue|Vietnam|Rheinland-Pfalz
Haviva Hill|26021|Ap #136-3993 Nulla Ave|Vietnam|Iowa
Avram Martin|8245|Ap #955-8822 Eu Avenue|Indonesia|Kahramanmaraş
Renee Kirby|186015|701-3259 Nullam Ave|Indonesia|Istanbul
Dante Donovan|IR9 1GE|899-4451 Erat Ave|Indonesia|North Region
Sebastian Ewing|5613|457-3368 Vitae St.|Indonesia|Agder
Jin Rodgers|11173|302-9501 Rd.|Indonesia|Luhansk oblast
Renee Irwin|346826|965-9337 Dui St.|Canada|North Kalimantan
Melissa Moody|92876|7425 Ac St.|Canada|Castilla-La Mancha
Valentine Howard|78-821|P.O. Box 996, 5621 Amet Rd.|Canada|West Region
Richard Pollard|E8R 6KB|Ap #691-9570 Consequat Ave|Canada|Jharkhand
Harrison Lancaster|27369|103-7177 Ac Rd.|Poland|Huadong
Farner Sanders|673606|P.O. Box 319, 5929 Purus St.|Poland|Carinthia
Caesar Velasquez|947697|569-5148 Amet Street|Poland|Maine
Lev Goodman|4161|936-4723 Libero St.|Poland|Diyarbakir
Giacomo Mullins|414135|P.O. Box 898, 6875 Lorem St.|Poland|Overijssel
Germane Sullivan|48-52|P.O. Box 491, 9104 Est. Rd.|Poland|La Guajira
Whoopi Newton|64-750|212-8767 Eros Street|Austria|Steiermark
Oren Walker|3162|501-8165 Quis St.|Austria|Tyrol
Mara Calder|56241|790-2214 Porta Road|Austria|Vienna
Leah Palmer|90418|841-1198 Dignissim St.|Austria|Salzburg
Nolan Fletcher|17392|299-6453 Arcu Lane|Austria|Linz
Sonia Patel|84290|721-1307 Viverra Ave|Austria|Graz
Emil Becker|17504|637-5589 Dolor Road|Austria|Innsbruck""".splitlines()]

sections = {
    "Input": ("Buttons", "Text", "Text area", "Checkboxes", "Switches",
              "Radio buttons", "Choices", "Lists", "Sliders"),
    "Display": ("Progress", "Tabs", "Tree views", "Data grid"),
    "Workspace": ("Docking", "Scrolling"),
}


def panel(title, rect, background=None, scroll=True):
    ui.SetNextWindowPos(rect[:2], ui.ImGuiCond_Always)
    ui.SetNextWindowSize(rect[2:], ui.ImGuiCond_Always)
    if background:
        ui.PushStyleColor(ui.ImGuiCol_WindowBg, tuple(c/255 for c in background))
    flags = (ui.ImGuiWindowFlags_NoTitleBar | ui.ImGuiWindowFlags_NoMove |
             ui.ImGuiWindowFlags_NoResize | ui.ImGuiWindowFlags_NoCollapse)
    if not scroll:
        flags |= ui.ImGuiWindowFlags_NoScrollbar
    visible = ui.Begin(title, flags)
    if background:
        ui.PopStyleColor()
    return visible


@window.event
def on_draw():
    gl = pyglet.gl
    gl.glClearColor(*(channel/255 for channel in ui.theme["window"]))
    window.clear()
    ui.new_frame()

    if panel("##title", (0, 0, window.width, 36), ui.theme["content"], scroll=False):
        ui.Text("Control Gallery")
        ui.SameLine()
        if ui.TabButton("Controls", view_mode[0] == "Controls", 96, 24):
            view_mode[0] = "Controls"
        ui.SameLine()
        if ui.TabButton("Data Grid", view_mode[0] == "Grid", 96, 24):
            view_mode[0] = "Grid"
        ui.SameLine()
        if ui.Button("Dark mode" if ui.theme_name == "light" else "Light mode", 104, 24):
            ui.set_theme("dark" if ui.theme_name == "light" else "light")
    ui.End()
    if panel("##toolbar", (0, 36, window.width, 36), ui.theme["title"], scroll=False):
        if view_mode[0] == "Controls":
            for index, label in enumerate(("Buttons", "Text", "Checkboxes", "Choices", "Sliders")):
                if index:
                    ui.SameLine()
                if ui.TabButton(label, selected[0] == label, 104, 24):
                    selected[0] = label
                    message[0] = label + " selected."
                    output.append(message[0])
        else:
            if ui.TabButton("Default", not grouped[0], 100, 24):
                grouped[0] = False
            ui.SameLine()
            if ui.TabButton("With grouping and filtering", grouped[0], 220, 24):
                grouped[0] = True
            ui.SameLine()
            _, grid_filter[0] = ui.InputText("Grid filter", grid_filter[0], 200, "Filter rows")
    ui.End()

    controls_visible = view_mode[0] == "Controls"
    ui.set_dock_visible("Tests", controls_visible)
    ui.set_dock_visible("Output", controls_visible)

    if not controls_visible:
        x, y, w, h = ui.viewport_rect
        left_width = round(w*.64)
        if panel("##grid", (x, y, left_width, h)):
            visible_rows = [row for row in grid_rows if not grid_filter[0] or
                            grid_filter[0].lower() in " ".join(map(str,row)).lower()]
            active = visible_rows.index(grid_rows[grid_selection[0]]) if grid_rows[grid_selection[0]] in visible_rows else -1
            active = ui.DataGrid(grid_columns, visible_rows, active,
                                 widths=(120,80,180,70,120,55), group_by=3 if grouped[0] else None)
            if active >= 0:
                grid_selection[0] = grid_rows.index(visible_rows[active])
        ui.End()
        if panel("##details", (x+left_width+4, y, w-left_width-4, h)):
            ui.Text("Selected Row")
            ui.Separator()
            row = grid_rows[grid_selection[0]]
            for index, label in enumerate(grid_columns[:5]):
                ui.Text(label)
                ui.SameLine(110)
                _, row[index] = ui.InputText(label+"##value", row[index], min(220,w-left_width-130))
        ui.End()
        bar = ui.GetForegroundDrawList()
        bar.AddRectFilled((0, window.height-28), (window.width, window.height), ui.theme["title"])
        bar.AddTextAligned((10, window.height-14), ui.theme["muted"],
                           "Control Gallery  ·  Scroll the grid  ·  Select a row")
        ui.render()
        return

    if ui.Begin("Tests"):
        ui.Text("Unit tests")
        _, filter_text[0] = ui.InputText("Filter", filter_text[0], 168, "Filter controls")
        ui.PushStyleVar(ui.ImGuiStyleVar_ItemSpacing, (8, 2))
        for section, entries in sections.items():
            matches = [entry for entry in entries if filter_text[0].lower() in entry.lower()]
            if not matches:
                continue
            ui.Separator()
            ui.SectionHeader(section.upper())
            for entry in matches:
                if ui.Selectable(entry, selected[0] == entry):
                    if entry == "Data grid":
                        view_mode[0] = "Grid"
                    else:
                        selected[0] = entry
                    message[0] = entry + " selected."
                    output.append(message[0])
        ui.PopStyleVar()
    ui.End()

    if ui.Begin("Output"):
        ui.Text("Output")
        ui.Separator()
        for line in output:
            ui.Text(line)
    ui.End()

    x, y, w, h = ui.viewport_rect
    if panel("##controls", (x, y+4, min(210, w), min(166, h-4))):
        if focus_first[0]:
            ui.SetKeyboardFocusHere()
            focus_first[0] = False
        changed, plain[0] = ui.InputText("Name", plain[0], 190)
        if changed:
            message[0] = "Text changed: " + plain[0]
            output.append(message[0])
        _, secret[0] = ui.InputText("Second field", secret[0], 190)
        if ui.Button("Select all text on focus", 190):
            focus_first[0] = True
        ui.PushStyleColor(ui.ImGuiCol_Text, (0.25, 0.75, 0.35, 1))
        ui.Text("Different coloured text")
        ui.PopStyleColor()
        ui.PushStyleColor(ui.ImGuiCol_Text, (0.95, 0.30, 0.30, 1))
        ui.Text("200456698")
        ui.PopStyleColor()
    ui.End()

    if w > 460:
        if panel("##document", (x+210, y+4, w-210, min(166, h-4))):
            ui.Text("In olden times when wishing still helped one, there lived a king whose daughters")
            ui.Text("were all beautiful. The youngest was so beautiful that even the sun,")
            ui.Text("which had seen so much, was astonished whenever it shone on her face.")
            ui.Text("The controls here exercise typography, selection, editing, and docking.")
            ui.Text("Each panel has its own clipping region and scroll position.")
            ui.Text("Pyglet draws the text and batched control geometry.")
            ui.Text("Long documents reveal the scrollbar on the right edge.")
            ui.Text("Wheel and track dragging both move the content.")
            ui.Text("The content remains clipped to its panel during scrolling.")
            ui.Separator()
            ui.Text("Drag a dock tab to an edge; drag a divider to resize the pane.")
        ui.End()

    if h > 220:
        if panel("##preview", (x, y+174, w, h-174)):
            ui.BeginTabBar("workspace")
            if ui.BeginTabItem("Preview"):
                ui.Text("Control gallery / " + selected[0])
                ui.Separator()
                if selected[0] == "Checkboxes":
                    changed, checked[0] = ui.Checkbox("Enable option", checked[0])
                    if changed:
                        message[0] = "Option enabled." if checked[0] else "Option disabled."
                        output.append(message[0])
                elif selected[0] == "Switches":
                    changed, switched[0] = ui.Switch("Notifications", switched[0])
                    if changed:
                        output.append("Notifications " + ("on" if switched[0] else "off"))
                elif selected[0] == "Radio buttons":
                    ui.Text("Performance profile")
                    for option in ("Quiet", "Balanced", "Fast"):
                        if ui.RadioButton(option, radio[0] == option):
                            radio[0] = option
                            output.append("Profile: " + option)
                elif selected[0] == "Choices":
                    ui.Text("Display size")
                    changed, choice[0] = ui.ComboBox("Display size", choice[0],
                                                     ("Small", "Medium", "Large", "Extra large"), 200)
                    if changed:
                        output.append("Display size: " + choice[0])
                elif selected[0] == "Lists":
                    ui.Text("Choose a material")
                    list_selection[0] = ui.ListBox("Materials", ("Aluminium", "Steel", "Titanium", "Copper"),
                                                   list_selection[0], 240)
                elif selected[0] == "Tree views":
                    _, expanded = ui.TreeNode("UI flows", default_open=True)
                    if expanded:
                        _, opened = ui.TreeNode("Main flow", 1, default_open=True)
                        if opened:
                            for item in ("Customer", "Dashboard", "Orders", "Settings"):
                                clicked, _ = ui.TreeNode(item, 2, tree_selection[0] == item, leaf=True)
                                if clicked:
                                    tree_selection[0] = item
                    ui.Text("Selected: " + tree_selection[0])
                elif selected[0] == "Sliders":
                    changed, level[0] = ui.SliderFloat("Level", level[0])
                    if changed:
                        message[0] = "Level: {:.2f}".format(level[0])
                elif selected[0] == "Progress":
                    ui.Text("Progress")
                    ui.ProgressBar(level[0], 300)
                elif selected[0] == "Docking":
                    ui.Text("Drag Tests or Output by its tab to dock at another edge.")
                    ui.Text("Drag the divider to change the pane size.")
                elif selected[0] == "Scrolling":
                    ui.Text("The document and Tests pane scroll independently.")
                    ui.Text("Wheel, drag the thumb, or click the track to navigate.")
                elif selected[0] == "Tabs":
                    ui.Text("Preview, Layout, and Details are interactive tabs.")
                elif selected[0] == "Text":
                    ui.Text("Click a field above to edit. Tab moves between fields.")
                    ui.Text("Ctrl+A, C, V, X, Z, and Y use familiar editing shortcuts.")
                elif selected[0] == "Text area":
                    ui.Text("Notes")
                    _, notes[0] = ui.InputTextMultiline("Notes", notes[0], 340, 108,
                                                         "Write a few lines...")
                else:
                    if ui.Button("Primary action", 140, 30, primary=True):
                        message[0] = "Button clicked."
                        output.append(message[0])
                    ui.SameLine()
                    if ui.Button("Secondary", 120, 30):
                        output.append("Secondary clicked.")
                    ui.Button("Unavailable", 120, 30, enabled=False)
                ui.EndTabItem()
            if ui.BeginTabItem("Layout"):
                ui.Text("Dock panels around the central workspace.")
                ui.Text("The available canvas follows the dock tree as it changes.")
                ui.EndTabItem()
            if ui.BeginTabItem("Details"):
                ui.Text("Renderer: pyglet")
                ui.Text("Controls: custom drawn")
                ui.Text("Theme: " + ui.theme_name)
                ui.EndTabItem()
            ui.EndTabBar()
        ui.End()

    bar = ui.GetForegroundDrawList()
    bar.AddRectFilled((0, window.height-28), (window.width, window.height), ui.theme["title"])
    bar.AddTextAligned((10, window.height-14), ui.theme["muted"],
                       "Control Gallery  ·  Drag tabs to dock  ·  Drag dividers to resize")
    del output[:-200]
    ui.render()


@window.event
def on_close():
    if not closed[0]:
        closed[0] = True
        ui.shutdown()


if __name__ == "__main__":
    pyglet.app.run()
