import csv
import argparse
import sys

eps = 1e-9

def toFloat(str):
    try:
        return float(str)
    except ValueError:
        return str


parser = argparse.ArgumentParser(
    description='Compare and Output some chart.'
)
parser.add_argument('result', help='Input the file position of compare tool output file (last record only).')
parser.add_argument('threshold', help='Threshold of Error')

args = parser.parse_args()

error_threshold = toFloat(args.threshold);

try:
    with open(args.result, 'rt') as resultFile:

        resultData = csv.reader(resultFile, delimiter=',', quotechar='\n')

        acc_error = 0;
        nums_of_items = 0;
        error_list = [];

        for num in next(resultData):
            acc_error += abs(toFloat(num))
            nums_of_items += 1
            error_list.append(abs(toFloat(num)))

        acc_error /= nums_of_items

        if acc_error <= error_threshold:
            print("");
            print("==============================================")
            print("");
            print("=  Accumaltive Error: {} <= {}, Test Passed  =".format(acc_error, error_threshold))
            print("");
            print("= SBII X-axis Error: {} =".format(error_list[5]))
            print("");
            print("= SBII Y-axis Error: {} =".format(error_list[6]))
            print("");
            print("= SBII Z-axis Error: {} =".format(error_list[7]))
            print("");
            print("= VBII X-axis Error: {} =".format(error_list[8]))
            print("");
            print("= VBII Y-axis Error: {} =".format(error_list[9]))
            print("");
            print("= VBII Z-axis Error: {} =".format(error_list[10]))
            print("");
            print("==============================================")
            print("");
            sys.exit(0);
        else:
            print("");
            print("==============================================")
            print("");
            print("= Accumaltive Error: {} > {}, Test Failed =".format(acc_error, error_threshold))
            print("");
            print("= SBII X-axis Error: {} =".format(error_list[5]))
            print("");
            print("= SBII Y-axis Error: {} =".format(error_list[6]))
            print("");
            print("= SBII Z-axis Error: {} =".format(error_list[7]))
            print("");
            print("= VBII X-axis Error: {} =".format(error_list[8]))
            print("");
            print("= VBII Y-axis Error: {} =".format(error_list[9]))
            print("");
            print("= VBII Z-axis Error: {} =".format(error_list[10]))
            print("");
            print("==============================================")
            print("");
            sys.exit(255);


except IOError as e:
    print(e)
