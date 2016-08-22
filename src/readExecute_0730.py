import os
import time
import datetime
import random
import sys

HourMinute = sys.argv[1]
TargetFile = sys.argv[2]


d = datetime.datetime.today()

DirName1 = str("{0:04d}".format(d.year)) + str("{0:02d}".format(d.month)) + str("{0:02d}".format(d.day))
DirName2 = str("{0:02d}".format(d.hour)) + str("{0:02d}".format(d.minute))

cmd_Mkdir = "mkdir ../data/" + DirName1 + '/' + DirName2 + '/' + HourMinute + '/' + TargetFile

print cmd_Mkdir
#os.system(cmd_Mkdir)

#Time_loop = "200"
#waitTime  = 1
#Prs_Bnd   = 0

#SrcFile = 'data/' + DirName1 + '/Out_pdt_0731_1242.dat'
SrcFile = '../data/' + DirName1 + '/' + HourMinute + '/' + TargetFile

#print SrcFile

for line in open(SrcFile, 'r'):
    itemList = line[:-1].split('\t')
    #print itemList
    cmd = "./hopper2 " + str(itemList[0]) + " " + str(itemList[1]) + " " + str(itemList[2]) + " " + DirName2    
    print cmd
    #os.system(cmd)
    #time.sleep(waitTime)


