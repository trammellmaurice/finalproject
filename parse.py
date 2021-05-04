import json
import argparse

# user interface
#PARSING AND CREATING ADJANCEY MATRIX
parser = argparse.ArgumentParser()
parser.add_argument("start_file_name")
parser.add_argument("end_file_name")
args = parser.parse_args()

fstart = open(args.start_file_name,)
fend = open(args.end_file_name,)
# fend = open(args.start_file_name,)

data_start = json.load(fstart)
data_end = json.load(fend)
