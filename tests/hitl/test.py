#!/usr/bin/env python3
import time
from panda_jungle import PandaJungle
from panda import Panda
from panda.tests.hitl.helpers import time_many_sends, clear_can_buffers

p = Panda()
pj = PandaJungle()
def test(p_send, p_recv):
  p_send.can_send_many([(0x1ba, 0, b"message", 0)] * 2)
  time.sleep(0.05)
  p_send.can_recv()

  for bus in (0, 1, 2):
    #for speed in (10, 20, 50, 100, 125, 250, 500, 1000):
    for speed in (1000, ):
      p_send.set_can_speed_kbps(bus, speed)
      p_recv.set_can_speed_kbps(bus, speed)
      clear_can_buffers(p_send)
      clear_can_buffers(p_recv)

  for bus in (0, 1, 2):
    #for speed in (10, 20, 50, 100, 125, 250, 500, 1000):
    for speed in (1000, ):
      comp_kbps = time_many_sends(p_send, bus, p_recv, two_pandas=True)

      saturation_pct = (comp_kbps / speed) * 100.0
      #assert 80 < saturation_pct < 100

      print("two pandas bus {}, 100 messages at speed {:4d}, comp speed is {:7.2f}, {:6.2f}%".format(bus, speed, comp_kbps, saturation_pct))

# Run tests in both directions
p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
print("\n", "abc****************8")
for _ in range(10):
  test(p, pj)
print("\n", "abc****************8")
#test(pj, p)



