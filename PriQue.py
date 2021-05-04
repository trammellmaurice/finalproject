class PriQue:

    def __init__(self):
        self.heap = [0]

    def pop(self):
        ret = self.heap[1]
        self.heap[1] = self.heap[-1]
        del self.heap[-1]
        self.bub_down(1)
        return ret[1]

    def push(self, p, e):
        index = len(self.heap)
        self.heap.append((p,e))
        self.bub_up(index)

    def swap(self, x, y):
        t = self.heap[x]
        self.heap[x] = self.heap[y]
        self.heap[y] = t

    def bub_down(self, i):
        if self.has_rc(i):
            m = i*2 + 1
            if self.heap[i*2][0] < self.heap[m][0]:
                m = i*2
            if self.heap[m][0] < self.heap[i][0]:
                self.swap(i,m)
                self.bub_down(m)
        elif self.has_lc(i):
            if self.heap[i*2][0] < self.heap[i][0]:
                self.swap(i,i*2)
                self.bub_down(i*2)

    def bub_up(self, i):
        if i == 1:
            return
        elif self.heap[i][0] < self.heap[int(i/2)][0]:
                self.swap(i,int(i/2))
                self.bub_up(int(i/2))

    def has_lc(self,i):
        return i*2 < len(self.heap)

    def has_rc(self,i):
        return i*2 + 1 < len(self.heap)

class MyStack:

    def __init__(self):
        self.stack = {}

    def push(self, e, c):
        if e in self.stack:
            d = self.stack[e]
            if c < d:
                self.stack[e] = c
        else:
            self.stack[e] = c

    def pop(self):
        return self.stack.popitem()[0]

    def __len__(self):
        return len(self.stack)







