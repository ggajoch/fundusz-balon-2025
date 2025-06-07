import time

class SerialMock(object):
    def __init__(self, path, wait=3):
        self.data = None
        self.current_line = 0
        self.wait = wait

        self.count = 0

        with open(path, 'r') as f:
            self.data = [line.rstrip('\r') for line in f]

    def flush(self):
        pass

    def readline(self):
        time.sleep(self.wait)
        ret = self.data[self.current_line]
        self.current_line += 1

        self.count +=1 
        return "{};{};{}".format(int(time.time()), self.count, ret)



