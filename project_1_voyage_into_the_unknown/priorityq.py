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


q = PriorityQueue()
q.push(("firstadded",1),23)
q.push(("secondadded",2),12)
q.push(("thirdadded",3),40)
q.push(("fourthadded",4),23)


top = q.pop()
#print(top)
#print(q._data)

#if (q.isEmpty()):
#	print(q.isEmpty())
#else: 
#	print(q.isEmpty())
