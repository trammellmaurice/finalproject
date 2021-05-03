import json

f = open("balloons.txt", "r")
j = f.read()

data = json.loads(j)
print(data)
