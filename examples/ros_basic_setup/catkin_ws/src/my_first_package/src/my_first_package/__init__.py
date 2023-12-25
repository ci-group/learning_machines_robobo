from .message import get_message, get_random_value

# This allows us to import with "from my_fist_package import get_message"
# Instead of "from my_first_package.message import get_message"
# I do this quite a lot, as I think it's cleaner.
# If you want more info: https://stackoverflow.com/a/35710527/18189817
__all__ = ("get_message", "get_random_value")
