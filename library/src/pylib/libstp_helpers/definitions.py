from typing import TypeVar, Callable, Union

from libstp.logging import debug, info

from libstp_helpers import get_table
import os

T = TypeVar('T')


def define_definition(caller_file, default_definition: T) -> T:
    table = get_table()
    definitions_folder = os.path.join(os.path.dirname(caller_file), "definitions")
    if not os.path.exists(definitions_folder):
        raise Exception(f"Could not find definitions folder at {definitions_folder}")

    for definition in os.listdir(definitions_folder):
        if not definition.endswith(".py"):
            continue

        debug(f"Checking definition {definition}")
        with open(os.path.join(definitions_folder, definition), "r") as file:
            content = file.read()

        local_variables = {}
        if table.name in content:
            exec(content, {}, local_variables)

        if local_variables.get("__apply_for_side__", None) == table:
            apply_function: Union[Callable, None] = local_variables.get("__apply__", None)
            if apply_function is None:
                raise Exception(f"Could not find __apply__ function in {definition}")
            result = apply_function(default_definition)
            if result is None:
                raise Exception(f"Could not apply {definition} to {default_definition}")
            info(f"Applied definition {definition}")
            return result
    info(f"Applied default definition")
    return default_definition
