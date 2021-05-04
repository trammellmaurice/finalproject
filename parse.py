import json

## TODO argparse for start and end states

f = open("balloons.txt", "r")
j = f.read()

data = json.loads(j)
print(data)
