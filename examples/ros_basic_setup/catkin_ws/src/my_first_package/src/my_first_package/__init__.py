# This allows us to import with "from my_first_package import RESULT_DIR"
# Instead of "from my_first_package.results import RESULT_DIR"
# I do this quite a lot, as I think it's cleaner.
# If you want more info: https://stackoverflow.com/a/35710527/18189817

from .example_process import ExampleProcess

__all__ = ("ExampleProcess",)
