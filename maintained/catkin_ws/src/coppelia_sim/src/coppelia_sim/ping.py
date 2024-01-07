import time
from datetime import datetime

from .error import CoppeliaSimApiError
from .sim import simxGetPingTime


class ApiTimeout(Exception):
    pass


def ping(client_id: int, timeout_seconds: int = 120) -> int:
    """Wait for CoppeliaSim to respond
    Performs exponential backoff with harmonic prevention

    Arguments:
    client_id: the client to ping
    timeout_seconds: the time to wait maximally before raising ApiTimeout
    """
    start = datetime.now()
    wait = 0.02
    while (datetime.now() - start).total_seconds() < timeout_seconds:
        try:
            return simxGetPingTime(client_id)
        except CoppeliaSimApiError:
            time.sleep(wait)
            wait = (2 * wait) - 0.01
    else:
        raise ApiTimeout()
