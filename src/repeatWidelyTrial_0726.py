import os
import time
import datetime
import random

d = datetime.datetime.today()

DirName1 = str("{0:04d}".format(d.year)) + str("{0:02d}".format(d.month)) + str("{0:02d}".format(d.day))
DirName2 = str("{0:02d}".format(d.hour)) + str("{0:02d}".format(d.minute))

cmd_Mkdir = "mkdir ../data/" + DirName1 + '/' + DirName2

print cmd_Mkdir

os.system(cmd_Mkdir)

#Prs_Sld_max = 0.2
#Prs_Elb_max = 0.2
#Prs_Wst_max = 0.1
#Prs_Sld_min = 0.1
#Prs_Elb_min = 0.1
#Prs_Wst_min = 0.05

#Prs_Sld_max = + 5.0
#Prs_Elb_max = + 5.0
#Prs_Wst_max = + 5.0
#Prs_Sld_min = - 5.0
#Prs_Elb_min = - 5.0
#Prs_Wst_min = - 5.0

Prs_Sld_max = + 10.0
Prs_Elb_max = + 10.0
Prs_Wst_max = + 10.0
Prs_Sld_min = - 10.0
Prs_Elb_min = - 10.0
Prs_Wst_min = - 10.0

#Prs_Bnd = 0.0
#Time_loop = "200"
#waitTime  = 2

N = 1000
#N = 500
#N = 200
#N = 100
#N = 50
#N = 10

for i in range(0,N):
    Prs_Sld = (Prs_Sld_max - Prs_Sld_min)* random.random() + Prs_Sld_min
    Prs_Elb = (Prs_Elb_max - Prs_Elb_min)* random.random() + Prs_Elb_min
    Prs_Wst = (Prs_Wst_max - Prs_Wst_min)* random.random() + Prs_Wst_min

    cmd = "./hopper2 " + str(Prs_Sld) + "\t" + str(Prs_Elb) + "\t" + str(Prs_Wst) + "\t" + DirName2    
    #print cmd
    os.system(cmd)
    #time.sleep(waitTime)
    #ratio = 100* i/ N
    #print ratio


