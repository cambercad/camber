"""Optional Camber API parameters are explicit at call sites."""
import inspect
import unittest

import camber
from camber import api, geom, inspection


_POSITIONAL_DEFAULTS = {
    (api.Frame, "__init__"): {"origin"},
    (api.Part, "sketch"): {"plane"},
    (api.Part, "sketch_interactive"): {"plane"},
    (geom, "text_outlines"): {"origin"},
    (inspection, "section"): {"plane"},
}


class ApiKeywordArgumentTests(unittest.TestCase):
    def test_public_optional_parameters_are_keyword_only(self):
        violations = []
        for owner in (api, geom, inspection):
            for name, value in vars(owner).items():
                if name.startswith("_"):
                    continue
                if inspect.isclass(value) and value.__module__ == owner.__name__:
                    for member_name, member in vars(value).items():
                        if member_name.startswith("_") and member_name != "__init__":
                            continue
                        if not (inspect.isfunction(member) or isinstance(member, (staticmethod, classmethod))):
                            continue
                        function = member.__func__ if isinstance(member, (staticmethod, classmethod)) else member
                        allowed = _POSITIONAL_DEFAULTS.get((value, member_name), set())
                        violations.extend(self._optional_positional(function, value.__name__ + "." + member_name, allowed))
                elif inspect.isfunction(value) and value.__module__ == owner.__name__:
                    allowed = _POSITIONAL_DEFAULTS.get((owner, name), set())
                    violations.extend(self._optional_positional(value, owner.__name__ + "." + name, allowed))

        self.assertEqual([], violations, "Optional public parameters must be keyword-only: " + "; ".join(violations))

    def test_package_show_uses_keyword_only_options(self):
        parameters = inspect.signature(camber.show).parameters
        self.assertEqual(inspect.Parameter.KEYWORD_ONLY, parameters["title"].kind)
        self.assertEqual(inspect.Parameter.KEYWORD_ONLY, parameters["colors"].kind)
        self.assertEqual(inspect.Parameter.KEYWORD_ONLY, parameters["checker"].kind)

    @staticmethod
    def _optional_positional(function, label, allowed):
        return [
            f"{label}.{parameter.name}"
            for parameter in inspect.signature(function).parameters.values()
            if parameter.kind in (inspect.Parameter.POSITIONAL_ONLY, inspect.Parameter.POSITIONAL_OR_KEYWORD)
            and parameter.default is not inspect.Parameter.empty
            and parameter.name not in allowed
        ]


if __name__ == "__main__":
    unittest.main()
