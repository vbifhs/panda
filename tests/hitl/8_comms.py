import time
import pytest
import random

from panda import Panda
from panda.python.spi import SpiDevice

pytestmark = [
  pytest.mark.test_panda_types((Panda.HW_TYPE_TRES, ))
]

def test_recovers_after_junk(p):
  # all good
  p.health()

  dev = SpiDevice()
  with dev.acquire() as spi:
    for _ in range(2000):
      print(_)
      dat = []
      spi.xfer([0x0, ]*100)

    # ensure we're all good
    assert p.health()['fault_status'] == 0
