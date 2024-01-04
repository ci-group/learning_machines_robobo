import functools

from coppelia_sim import simConst

from typing import Callable, List, Tuple


# Sadly, python3.8 does not have match statements jet.
def error_code_to_message(ret: int) -> str:
    if ret == simConst.simx_return_ok:
        raise ValueError("Error return code is of type OK, cannot be treated as error.")

    errors: List[str] = []
    if ret & simConst.simx_return_novalue_flag:  # 1
        errors.append("There is no command reply in the input buffer.")
    if ret & simConst.simx_return_timeout_flag:  # 2
        errors.append(
            "The function timed out (probably the network is down or too slow)"
        )
    if ret & simConst.simx_return_illegal_opmode_flag:  # 4
        errors.append(
            "The specified operation mode is not supported for the given function"
        )
    if ret & simConst.simx_return_remote_error_flag:  # 8
        errors.append(
            "The function caused an error on the server side (e.g. an invalid handle was specified)"
        )
    if ret & simConst.simx_return_split_progress_flag:  # 16
        errors.append(
            "The communication thread is still processing previous split command of the same type"
        )
    if ret & simConst.simx_return_local_error_flag:  # 32
        errors.append("The function caused an error on the client side")
    if ret & simConst.simx_return_initialize_error_flag:  # 64
        errors.append("simxStart was not yet called")
    return "\n and ".join(errors)


class CoppeliaSimApiError(Exception):
    ret_code: int

    def __init__(self, ret_code: int):
        self.ret_code = ret_code
        super().__init__(error_code_to_message(ret_code))


# Sadly, there are no good ways to get generic varargs typed nicely in 3.8
def simx_function(f: Callable[..., Tuple]) -> Callable:
    @functools.wraps(f)
    def inner(*args, **kwargs):
        res = f(*args, **kwargs)
        if not isinstance(res, Tuple):
            return res

        if not len(res) >= 2:
            raise ValueError("Could not deconstruct CoppeliaSimAPI return value")

        ret_code = res[0]
        if not ret_code == simConst.simx_return_ok:
            raise CoppeliaSimApiError(ret_code)

        ret = res[1:]
        if len(ret) == 1:
            return ret[0]
        return ret

    return inner
