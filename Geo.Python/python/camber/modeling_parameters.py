"""Small, explicit registration interface for parametric GUI components."""
import inspect
import math
from dataclasses import fields, is_dataclass, MISSING


GENERATORS = {}


def named_parameter(value: float, *, unit: str = "mm", description: str = "") -> float:
    """Define a finite numeric document parameter; expressions can reference earlier parameters."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("Parameter must be a finite number")
    return value


def register_generator(function, parameter_type, *, name=None):
    """Expose a dataclass-based builder as an editable assembly command.

    The returned callable accepts part/name and the dataclass fields as keywords.
    Assign it to a module-level name so saved documents can import it normally.
    """
    if not is_dataclass(parameter_type):
        raise TypeError("Generator parameters must be a dataclass")
    call_name = name or function.__name__ + "_component"
    if not call_name.isidentifier() or not call_name.endswith("_component"):
        raise ValueError("Generator name must be a Python identifier ending in _component")
    if call_name in GENERATORS:
        raise ValueError("Generator already registered: " + call_name)
    parameters = [inspect.Parameter("part", inspect.Parameter.KEYWORD_ONLY, annotation="Part"),
                  inspect.Parameter("name", inspect.Parameter.KEYWORD_ONLY, default=call_name, annotation=str)]
    for field in fields(parameter_type):
        if not field.init:
            continue
        default = field.default if field.default is not MISSING else (
            field.default_factory() if field.default_factory is not MISSING else inspect.Parameter.empty)
        parameters.append(inspect.Parameter(field.name, inspect.Parameter.KEYWORD_ONLY,
                                            default=default, annotation=field.type))
    signature = inspect.Signature(parameters, return_annotation="Assembly")

    def component(**values):
        bound = signature.bind(**values)
        bound.apply_defaults()
        arguments = dict(bound.arguments)
        part, result_name = arguments.pop("part"), arguments.pop("name")
        return function(parameter_type(**arguments), part=part, name=result_name).assembly

    component.__name__ = component.__qualname__ = call_name
    component.__module__ = function.__module__
    component.__doc__ = inspect.getdoc(function) or "Create a reusable parametric component."
    component.__signature__ = signature
    component.__api_group__ = "Components"
    component.parameter_type = parameter_type
    GENERATORS[call_name] = component
    from .modeling_commands import operations
    operations.cache_clear()
    return component
