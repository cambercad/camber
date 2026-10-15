"""Snapshot undo/redo transactions. Standard library only; no GUI or CAD dependencies."""
from copy import deepcopy


class History:
    """Move a revision only after its restore callback explicitly succeeds.

    Record a completed edit with record(before, after). Restore callbacks return
    True on success or False after retaining/restoring the current application state.
    Exceptions leave the history unchanged. Stored mutable snapshots are isolated.
    """
    def __init__(self):
        self._undo = []
        self._redo = []
        self._busy = False

    @property
    def can_undo(self):
        return bool(self._undo)

    @property
    def can_redo(self):
        return bool(self._redo)

    @property
    def undo_stack(self):
        return deepcopy(tuple(self._undo))

    @property
    def redo_stack(self):
        return deepcopy(tuple(self._redo))

    def _check_idle(self):
        if self._busy:
            raise RuntimeError("History cannot be changed while restoring a revision")

    def record(self, before, after):
        self._check_idle()
        if before != after:
            self._undo.append(deepcopy(before))
            self._redo.clear()

    def step(self, direction, current, restore, *, fallback=None):
        self._check_idle()
        if direction not in ("undo", "redo"):
            raise ValueError("History direction must be undo or redo")
        source, destination = (self._undo, self._redo) if direction == "undo" else (self._redo, self._undo)
        if not source and fallback is None:
            return False
        previous = deepcopy(current)
        target = deepcopy(source[-1] if source else fallback)
        self._busy = True
        try:
            result = restore(target)
            if type(result) is not bool:
                raise TypeError("History restore callbacks must return True or False")
            if not result:
                return False
        finally:
            self._busy = False
        if source:
            source.pop()
        destination.append(previous)
        return True

    def transformed(self, *, transform=lambda value: value):
        """Copy history, optionally rewriting revisions for Save As."""
        self._check_idle()
        result = History()
        self._busy = True
        try:
            result._undo = [deepcopy(transform(deepcopy(value))) for value in self._undo]
            result._redo = [deepcopy(transform(deepcopy(value))) for value in self._redo]
            return result
        finally:
            self._busy = False
