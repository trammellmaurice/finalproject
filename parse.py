import json
import argparse

# user interface
#PARSING AND CREATING ADJANCEY MATRIX
parser = argparse.ArgumentParser()
parser.add_argument("start_file_name")
parser.add_argument("end_file_name")
args = parser.parse_args()

fstart = open(args.start_file_name, "r")
fend = open(args.start_file_name, "r")

jstart = fstart.read()
data = json.loads(jstart)


jend = fstart.read()
data = json.loads(jend)

print(data)
