from utils.GyroQueue import GyroQueue

if __name__ == "__main__":
    q = GyroQueue(maxsize=1)
    q.put(1)
    q.put(2)
    q1 = q.get()
    q.queue.clear()
    print("Size = ", q.qsize())
    print("q1 = ", q1) # q1 = 2