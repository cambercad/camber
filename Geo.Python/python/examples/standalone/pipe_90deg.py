"""Compatibility entry point. The reusable implementation is in pipe.py."""

from pipe import create_pipe


pipe = create_pipe(part_name="pipe")
print(pipe)
pipe.show(title="camber 90deg pipe")
