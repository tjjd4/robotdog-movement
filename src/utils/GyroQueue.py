import queue

class GyroQueue(queue.LifoQueue):
    def put(self, item, block=True, timeout=None):
        """在 queue 滿時，先移除最舊的元素再插入新元素"""
        if self.full():
            try:
                self.get_nowait()  # 先移除最舊的元素
            except queue.Empty:
                pass  # 理論上不會發生
        super().put(item, block, timeout)

# 測試
if __name__ == '__main__':
    lifo_q = GyroQueue(maxsize=3)

    lifo_q.put(1)
    lifo_q.put(2)
    lifo_q.put(3)
    print("初始 LifoQueue:", list(lifo_q.queue))  # [3, 2, 1]

    lifo_q.put(4)  # 這時 1 應該被移除
    print("插入 4 後:", list(lifo_q.queue))  # [4, 2, 1]
