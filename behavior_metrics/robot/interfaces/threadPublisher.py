import threading
import time
from datetime import datetime
from utils.logger import logger

time_cycle = 100 # con 100 ms se publican los comandos a de control a 20 hzs

class ThreadPublisher(threading.Thread):

    def __init__(self, pub, kill_event, time_cycle: float = time_cycle):
        super().__init__()
        self.pub = pub
        self.kill_event = kill_event
        self.time_cycle = time_cycle
        # threading.Thread.__init__(self, args=kill_event)

    def run(self):
      
        while not self.kill_event.is_set():
            start_time = datetime.now()
            
            try:
                self.pub.publish()
            except Exception as e:
                print(f"Error in ThreadPublisher: {e}")
                break

            finish_time = datetime.now()
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            if ms < self.time_cycle:
                time.sleep((self.time_cycle - ms) / 1000.0)
            # print(f"ThreadPublisher cycle time: {self.time_cycle - ms} ms")
    # def run(self):
    #     next_call = time.perf_counter()
    #     while not self.kill_event.is_set():
    #         next_call += self.time_cycle
            
    #         try:
    #             self.pub.publish()
    #         except Exception as e:
    #             logger.error(f"Error in ThreadPublisher: {e}")
    #             break
            
    #         time.sleep(max(0.0, next_call - time.perf_counter()))