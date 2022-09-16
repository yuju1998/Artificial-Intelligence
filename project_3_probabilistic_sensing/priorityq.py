import heapq

class PriorityQueue: 

	def __init__(self):
		self._data = []
		self._index = 0

	def isEmpty(self):
		return len(self._data) == 0

	def push(self, item, priority): 
		heapq.heappush(self._data, (priority, self._index, item))

	def pop(self): 
		return heapq.heappop(self._data)[-1]


