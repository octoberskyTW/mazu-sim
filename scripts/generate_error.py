import csv
import argparse
#import matplotlib.pyplot as plt

eps = 1e-9

def toFloat(str):
    try:
        return float(str)
    except ValueError:
        return str


parser = argparse.ArgumentParser(
    description='Compare and Output some chart.'
)
parser.add_argument('golden', help='Input the file position of golden file.')
parser.add_argument('target', help='Input the file position of target file.')

parser.add_argument('-l', '--last',
    help='Compare the last data between golden and target file.',
    action='store_true',
)

args = parser.parse_args()

try:
    with open(args.golden, "rt", encoding="utf-8") as goldenFile, \
         open(args.target, "rt", encoding="utf-8") as targetFile, \
         open('result.csv','w') as outputFile:

        goldenData = csv.reader(goldenFile, delimiter=',', quotechar='\n')
        targetData = csv.reader(targetFile, delimiter=',', quotechar='\n')
        writer = csv.writer(outputFile, delimiter=',', quotechar='\n')

        goldenArray = [];
        targetArray = [];
        outputArray = [];
        #Eat all shitz in
        for row in goldenData:
            goldenArray.append([]);
            for num in row:
                goldenArray[-1].append(toFloat(num))

        for row in targetData:
            targetArray.append([]);
            for num in row:
                targetArray[-1].append(toFloat(num))

        if(args.last): #only last
            for idx, num in enumerate(goldenArray[-1]):
                if num < eps and num > -eps:
                    outputArray.append((num - targetArray[-1][idx])/(num + 1))
                else:
                    try:
                        outputArray.append((num - targetArray[-1][idx])/num)
                    except TypeError:
                        outputArray.append(goldenNum)
            writer.writerow(outputArray);
        else: #All to All
            for idxRow, goldenRow in enumerate(goldenArray):
                outputArray.append([]);
                for idxNum, goldenNum in enumerate(goldenRow):
                    if goldenNum < eps and goldenNum > -eps:
                        outputArray[-1].append(
                            (goldenNum - targetArray[idxRow][idxNum])/(goldenNum + 1)
                        )
                    else:
                        try:
                            outputArray[-1].append(
                                (goldenNum - targetArray[idxRow][idxNum])/goldenNum
                            )
                        except TypeError:
                            outputArray[-1].append(goldenNum)
            writer.writerows(outputArray)

except IOError as e:
    print(e)
