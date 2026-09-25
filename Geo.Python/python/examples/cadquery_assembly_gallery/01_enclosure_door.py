"""CadQuery's 20x20 V-slot door tutorial: currently blocked, not approximated.

The source imports a vendor DXF profile, tags selected faces/edges, sweeps the
handle with a round transition and mates those tagged entities. Replacing the
V-slot by plain bars would misrepresent the tutorial and its constraints.
"""

def build():
    raise NotImplementedError(
        "door tutorial needs DXF wire import, persistent face/edge tags with "
        "CadQuery end() semantics, and round-transition sweep"
    )


if __name__ == "__main__":
    build()
