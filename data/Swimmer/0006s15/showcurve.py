# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import numpy as np
import matplotlib.pyplot as plt
import brewer2mpl
import matplotlib as mpl
from scipy import signal

bmap = brewer2mpl.get_map('Set2','qualitative', 7)
colors = bmap.mpl_colors
#plot preprocessing
params = {
'axes.labelsize': 22,
'font.size': 20,
'legend.fontsize': 20,
'xtick.labelsize': 20,
'ytick.labelsize': 20,
'text.usetex': True ,
'figure.figsize': [7, 5.5], # instead of 4.5, 4.5
'font.weight': 'bold',
'axes.labelweight': 'bold',
'ps.useafm' : True,
'pdf.use14corefonts':True,
'pdf.fonttype': 42,
'ps.fonttype': 42
 }
mpl.rcParams.update(params)

#timefactor=7.57
def multicost(timefactor=600):
    y=np.loadtxt('cost.txt')
    cm = np.mean(y,axis=0)
    cs = np.std(y,axis=0)
    x=np.linspace(1,y.shape[1],y.shape[1])*timefactor
    plt.plot(x, cm/cm[-1], color='dodgerblue', linewidth=1)
    plt.fill_between(x,(cm-cs)/cm[-1],(cm+cs)/cm[-1],alpha=0.5,color='orange')
    plt.xlabel('Num of rollouts', fontsize=20)
    plt.ylabel('Averaged episodic cost fraction', fontsize=20)
    plt.legend(['Mean', 'Standard deviation'])
    plt.tight_layout()
    plt.grid(color='.910', linewidth=1.5)

#timefactor=6.41
def latexplot(timefactor=600,filtered=False):
    #plot
    if filtered == True:
        b, a = signal.butter(8  , 0.025)
        with open('cost.txt') as f:
            r=f.readlines()
        y=np.array(r[0].strip().split()).astype(np.float)
        x=np.linspace(1,y.shape[0],y.shape[0])*timefactor
        plt.plot(x, y/y[-1], color=colors[1], alpha=0.9)
        plt.plot(x, signal.filtfilt(b, a, y/y[-1]), color=colors[2], linewidth=3)
        plt.grid(axis='y', color='.910', linestyle='-', linewidth=1.5)
        plt.grid(axis='x', color='.910', linestyle='-', linewidth=1.5)
        
        plt.xlabel('Num of rollouts', fontsize=20)
        plt.ylabel('Episodic cost fraction', fontsize=20)
        plt.legend(['Original','Filtered'])
    else:
        with open('cost.txt') as f:
            r=f.readlines()
        y=np.array(r[0].strip().split()).astype(np.float)
        x=np.linspace(1,y.shape[0],y.shape[0])*timefactor
        plt.plot(x, y/y[-1], color=colors[2], linewidth=3)
        plt.grid(axis='y', color='.910', linestyle='-', linewidth=1.5)
        plt.grid(axis='x', color='.910', linestyle='-', linewidth=1.5)
        
        plt.xlabel('Num of rollouts', fontsize=20)
        plt.ylabel('Episodic cost fraction', fontsize=20)
        plt.legend(['Original'])
    
    plt.tight_layout()    
    
def showcurve(filename='cost.txt'):
    if filename == 'result.txt':
        with open(filename) as f:
            r=f.readlines()
        y=np.array(r[1].strip().split()).astype(np.float)
    else:
        with open(filename) as f:
            r=f.readlines()
        y=np.array(r[0].strip().split()).astype(np.float)
    x=np.linspace(1,y.shape[0],y.shape[0])
    
    plt.figure(figsize=(12,9))
    plt.plot(x,y)
    plt.show()
    print("NUM = {value1}".format(value1=y.shape[0]))
    print("MAX = {value1}  MIN = {value2}\nMEAN = {value3}  VAR = {value4}".format(value1=np.max(y),value2=np.min(y),value3=np.mean(y),value4=np.var(y)))

def normcost(timefactor=6.41):
    with open('cost.txt') as f:
        r=f.readlines()
    y=np.array(r[0].strip().split()).astype(np.float)
    x=np.linspace(1,y.shape[0],y.shape[0])*timefactor
    plt.figure(figsize=(12,9))
    plt.plot(x,y/y[-1])
    plt.vlines(x[-1],1,y[0]/y[-1],colors='r',linestyles='--')
    plt.text(x[-1],1,'9489s')
    plt.xlabel('time/s')
    plt.ylabel('cost / cost_norm')
    plt.show()
    print("NUM = {value1}".format(value1=y.shape[0]))
    print("MAX = {value1}  MIN = {value2}".format(value1=np.max(y),value2=np.min(y)))

def noiseresist(nstart=0,nend=100,noisemax=100):
    y=np.array(np.loadtxt('finalcost.txt'))
    cost=np.mean(y,axis=1)
    cstd=np.std(y,axis=1)
    step=noisemax/int(cost.shape[0]-1)
    sind=int(nstart/step)
    eind=int(nend/step)+1
    plt.figure(figsize=(12,9))
    f5,=plt.plot(np.arange(sind,(eind-1)*step+1,step),cost[sind:eind]/cost[0],'orange')
    plt.fill_between(np.arange(sind,(eind-1)*step+1,step),(cost[sind:eind]-cstd[sind:eind])/cost[0],(cost[sind:eind]+cstd[sind:eind])/cost[0],alpha=0.3,color='orange')
    plt.xlabel('noise/% of umax(std of noise)')
    plt.ylabel('cost / cost_norm')
    plt.grid(True)
    plt.show()  
    print('averaged by {value1} rollouts'.format(value1=y.shape[1]))
    
def perfcheck(nstart=0,nend=100,type='error',noisemax=100):
    if type=='cost':
        y=np.array(np.loadtxt('perfcheck.txt'))
        perf=np.mean(y,axis=1)
        cstd=np.std(y,axis=1)
        step=noisemax/int(perf.shape[0]-1)
        sind=int(nstart/step)
        eind=int(nend/step)+1
        plt.figure(figsize=(12,9))
        f5,=plt.plot(np.arange(sind,(eind-1)*step+1,step),perf[sind:eind],'orange')
        plt.fill_between(np.arange(sind,(eind-1)*step+1,step),(perf[sind:eind]-cstd[sind:eind]),(perf[sind:eind]+cstd[sind:eind]),alpha=0.3,color='orange')
        plt.xlabel('Std dev of perturbed noise (Percent of max. control)')
        plt.ylabel('cost per step')
        plt.grid(True)
        plt.show()  
        print('averaged by {value1} rollouts'.format(value1=y.shape[1]))

    if type=='error':
        y=np.array(np.loadtxt('perfcheck.txt'))
        perf=np.mean(y,axis=1)
        cstd=np.std(y,axis=1)
        step=noisemax/int(perf.shape[0]-1)
        sind=int(nstart/step)
        eind=int(nend/step)+1
        f5,=plt.plot(np.arange(sind,(eind-1)*step+1,step),perf[sind:eind],'orange')
        plt.fill_between(np.arange(sind,(eind-1)*step+1,step),(perf[sind:eind]-cstd[sind:eind]),(perf[sind:eind]+cstd[sind:eind]),alpha=0.3,color='orange')
        plt.xlabel('Std dev of perturbed noise (Percent of max. control)')
        plt.ylabel('L2-norm of terminal state error')
        plt.grid(True)
        plt.show()  
        print('averaged by {value1} rollouts'.format(value1=y.shape[1]))
        
def clopcompare():    
    pointnum=9
    testnum=400
    y=np.array(np.loadtxt('clopdata.txt'))
    clerr1=[0 for i in range(int(y.shape[0]/2))]
    operr1=[0 for i in range(int(y.shape[0]/2))]
    
    # calculate error value and get the average by each test
    for i in range(int(y.shape[0]/2)):
        clerr1[i]=abs(y[2*i])
        operr1[i]=abs(y[2*i+1])
    with open('clopbar.txt', 'wt+') as f:
        for k in range(pointnum):
            print(np.mean(clerr1[testnum*k:testnum*(k+1)]), np.std(clerr1[testnum*k:testnum*(k+1)]), np.mean(operr1[testnum*k:testnum*(k+1)]), np.std(operr1[testnum*k:testnum*(k+1)]), k*5, file=f)
    
    # plot performance compare data and success rate
    sind=0
    eind=9
    perfdata=np.transpose(np.loadtxt('clopbar.txt'))
    f5,=plt.plot(perfdata[4][sind:eind],perfdata[0,sind:eind],'orange', linewidth=3)
    f6,=plt.plot(perfdata[4][sind:eind],perfdata[2,sind:eind],'dodgerblue', linewidth=3)
    plt.fill_between(perfdata[4][sind:eind],perfdata[0][sind:eind]-perfdata[1][sind:eind],perfdata[0][sind:eind]+perfdata[1][sind:eind],alpha=0.3,color='orange')
    plt.fill_between(perfdata[4][sind:eind],perfdata[2][sind:eind]-perfdata[3][sind:eind],perfdata[2][sind:eind]+perfdata[3][sind:eind],alpha=0.3,color='dodgerblue')
    plt.xlabel('Std dev of perturbed noise(Percent of max. control)')
    plt.ylabel('Episodic cost')
    plt.legend(handles=[f5,f6],labels=['Closed-loop','Open-loop'],loc='upper left')
    plt.grid(color='.910', linewidth=1.5)
    plt.show()  
 
def mclopcompare():                   
    nstart=0
    nend=100
    pointnum=6
    testnum=200
    y=np.array(np.loadtxt('clopdata.txt'))
    clerr1=[0 for i in range(int(y.shape[0]/2))]
    operr1=[0 for i in range(int(y.shape[0]/2))]
    
    # calculate error value and get the average by each test
    for i in range(int(y.shape[0]/2)):
        clerr1[i]=abs(y[2*i])
        operr1[i]=abs(y[2*i+1])
    with open('clopbar.txt', 'wt+') as f:
        for k in range(pointnum):
            print(np.mean(clerr1[testnum*k:testnum*(k+1)]), np.std(clerr1[testnum*k:testnum*(k+1)]), np.mean(operr1[testnum*k:testnum*(k+1)]), np.std(operr1[testnum*k:testnum*(k+1)]), k*0.003/0.1*100, file=f)
    
    # plot performance compare data and success rate
    sind=int(nstart/100*(pointnum-1))
    eind=int(nend/100*(pointnum-1))+1
    perfdata=np.transpose(np.loadtxt('clopbar.txt'))
    f5,=plt.plot(perfdata[4][sind:eind],perfdata[0][sind:eind],'orange', linewidth=3)
    f6,=plt.plot(perfdata[4][sind:eind],perfdata[2][sind:eind],'dodgerblue', linewidth=3)
    plt.fill_between(perfdata[4][sind:eind],perfdata[0][sind:eind]-perfdata[1][sind:eind],perfdata[0][sind:eind]+perfdata[1][sind:eind],alpha=0.3,color='orange')
    plt.fill_between(perfdata[4][sind:eind],perfdata[2][sind:eind]-perfdata[3][sind:eind],perfdata[2][sind:eind]+perfdata[3][sind:eind],alpha=0.3,color='dodgerblue')
    plt.xlabel('Std dev of measurement noise(Percent of max. measurement)')
    plt.ylabel('Episodic cost')
    plt.legend(handles=[f5,f6,],labels=['LQG','LQR'],loc='upper left')
    plt.grid(color='.910', linewidth=1.5)
    plt.show()  
       
#def clopcompare():       
#    nstart=0
#    nend=30
#    pointnum=17
#    testnum=500
#    y=np.array(np.loadtxt('clopdata.txt'))
#    clerr1=[0 for i in range(int(y.shape[0]/2))]
#    operr1=[0 for i in range(int(y.shape[0]/2))]
#    
#    # calculate error value and get the average by each test
#    for i in range(int(y.shape[0]/2)):
#        clerr1[i]=abs(y[2*i])
#        operr1[i]=abs(y[2*i+1])
#    with open('clopbar.txt', 'wt+') as f:
#        for k in range(pointnum):
#            print(np.mean(clerr1[testnum*k:testnum*(k+1)]), np.std(clerr1[testnum*k:testnum*(k+1)]), np.mean(operr1[testnum*k:testnum*(k+1)]), np.std(operr1[testnum*k:testnum*(k+1)]), k*5, file=f)
#    
#    # plot performance compare data and success rate
#    sind=int(nstart/5)
#    eind=int(nend/5)+1
#    perfdata=np.transpose(np.loadtxt('clopbar.txt'))
#    f5,=plt.plot(perfdata[4][sind:eind],perfdata[0][sind:eind],'orange', linewidth=3)
#    f6,=plt.plot(perfdata[4][sind:eind],perfdata[2][sind:eind],'dodgerblue', linewidth=3)
#    plt.fill_between(perfdata[4][sind:eind],perfdata[0][sind:eind]-perfdata[1][sind:eind],perfdata[0][sind:eind]+perfdata[1][sind:eind],alpha=0.3,color='orange')
#    plt.fill_between(perfdata[4][sind:eind],perfdata[2][sind:eind]-perfdata[3][sind:eind],perfdata[2][sind:eind]+perfdata[3][sind:eind],alpha=0.3,color='dodgerblue')
#    plt.xlabel('Std dev of perturbed noise(Percent of max. control)', fontsize=18)
#    plt.ylabel('D2C averaged cost', fontsize=18)
#    plt.legend(handles=[f5,f6,],labels=['Closed-loop cost','Open-loop cost'],loc='upper left')
#    plt.grid(axis='y', color='.910', linestyle='-', linewidth=1.5)
#    plt.grid(axis='x', color='.910', linestyle='-', linewidth=1.5)
#    plt.show()  
    
def errcompare(nstart=0,nend=100, testnum=200, by='state'):
    pointnum=10 # total number of noise level checked
    singlecheck=4 # get error result under selected noise level 4 means 40%
    y=np.array(np.loadtxt('errdata.txt'))
    clerr1=[[0 for i in range(y.shape[1])] for i in range(int(y.shape[0]/3))]
    operr1=[[0 for i in range(y.shape[1])] for i in range(int(y.shape[0]/3))]
    clerr2=[0 for i in range(testnum*pointnum)]
    operr2=[0 for i in range(testnum*pointnum)]
    clerr=[0 for i in range(y.shape[1])]
    operr=[0 for i in range(y.shape[1])]
    
    # calculate error value and get the average by each test
    for i in range(int(y.shape[0]/3)):
        clerr1[i][:]=abs((y[3*i+1][:]-y[3*i][:])/y[3*i][:])
        operr1[i][:]=abs((y[3*i+2][:]-y[3*i][:])/y[3*i][:])
    ns=int(y.shape[0]/testnum/3/pointnum)
    for j in range(testnum*pointnum):
        clerr2[j]=np.mean(clerr1[ns*j:ns*(j+1)][:])
        operr2[j]=np.mean(operr1[ns*j:ns*(j+1)][:])
    with open('errorbar.txt', 'wt+') as f:
        print(0, 0, 0, 0, 0, file=f)
        for k in range(pointnum):
            print(np.mean(clerr2[testnum*k:testnum*(k+1)]), np.std(clerr2[testnum*k:testnum*(k+1)]), np.mean(operr2[testnum*k:testnum*(k+1)]), np.std(operr2[testnum*k:testnum*(k+1)]), (k+1)*10, file=f)
    
    # plot performance compare data and success rate
    sind=int(nstart/10)
    eind=int(nend/10)+1
    perfdata=np.transpose(np.loadtxt('errorbar.txt'))
    plt.figure(figsize=(12,9))
    f5,=plt.plot(perfdata[4][sind:eind],perfdata[0][sind:eind],'orange')
    f6,=plt.plot(perfdata[4][sind:eind],perfdata[2][sind:eind],'dodgerblue')
    plt.fill_between(perfdata[4][sind:eind],perfdata[0][sind:eind]-perfdata[1][sind:eind],perfdata[0][sind:eind]+perfdata[1][sind:eind],alpha=0.3,color='orange')
    plt.fill_between(perfdata[4][sind:eind],perfdata[2][sind:eind]-perfdata[3][sind:eind],perfdata[2][sind:eind]+perfdata[3][sind:eind],alpha=0.3,color='dodgerblue')
#    sucrate=np.insert(np.loadtxt('sucrate.txt'),0,1)
#    for a,b,c in zip(perfdata[4][:],perfdata[0][:],sucrate):
#        plt.text(a, b+0.05, '%.3f' % c, ha='center', va= 'bottom',fontsize=10)
    plt.xlabel('noise/%')
    plt.ylabel('error')
    plt.legend(handles=[f5,f6,],labels=['closed-loop error','open-loop error'],loc='upper left')
    plt.grid(True)
    plt.show()
    
    # check error by step or state to find the exact step or state that behaves bad, plot
    # valid only when testnum=1
    if testnum == 1:
        if by == 'step':
            clerr=np.mean(clerr1,axis=0)
            operr=np.mean(operr1,axis=0)
            x=np.linspace(1,y.shape[1],y.shape[1])
        elif by == 'state':
            clerr=np.mean(clerr1,axis=1)
            operr=np.mean(operr1,axis=1)
            x=np.linspace(1,int(y.shape[0]/3),int(y.shape[0]/3))
        plt.figure(figsize=(20,16))
        f1,=plt.plot(x,clerr)
        f2,=plt.plot(x,operr)
        plt.ylabel('error')
        if by == 'step':
            plt.xlabel('timestep')
        elif by == 'state':
            plt.xlabel('state')
        plt.legend(handles=[f1,f2,],labels=['closed-loop error','open-loop error'],loc='upper right')
        plt.show()
    if pointnum == 1 and singlecheck > 0:
        mcl=np.mean(clerr2[testnum*(singlecheck-1):testnum*(singlecheck)])
        mop=np.mean(operr2[testnum*(singlecheck-1):testnum*(singlecheck)])
        stdcl=np.std(clerr2[testnum*(singlecheck-1):testnum*(singlecheck)])
        stdop=np.std(operr2[testnum*(singlecheck-1):testnum*(singlecheck)])
        x=np.linspace(1,testnum,testnum) 
        plt.figure(figsize=(20,16))
        f3,=plt.plot(x,clerr2[testnum*singlecheck:testnum*(singlecheck+1)])
        f4,=plt.plot(x,operr2[testnum*singlecheck:testnum*(singlecheck+1)])
        plt.ylabel('error')  
        plt.legend(handles=[f3,f4,],labels=['closed-loop error','open-loop error'],loc='upper right')
        plt.show() 
        print(testnum)
        print("closed-loop error = {value1}  std = {value2}".format(value1=mcl, value2=stdcl))
        print("open-loop error = {value1}  std = {value2}".format(value1=mop, value2=stdop))
        
def sysidcheck():
    y=np.array(np.loadtxt('sysidcheck.txt'))
    syserr1=[[0 for i in range(y.shape[1])] for i in range(int(y.shape[0]/2))]
    syserr=[0 for i in range(int(y.shape[0]/2))]
    for i in range(int(y.shape[0]/2)):
        for j in range(int(y.shape[1])):
            if y[2*i][j] != 0:
                syserr1[i][j]=abs((y[2*i+1][j]-y[2*i][j])/y[2*i][j]) 
    syserr=np.mean(syserr1,axis=1)
    
    x=np.linspace(1,int(y.shape[0]/2),int(y.shape[0]/2))
    plt.figure(figsize=(20,16))
    f1,=plt.plot(x,syserr)
    plt.ylabel('error')
    plt.xlabel('state')
    plt.show()
    print("total error = {value1}".format(value1=np.mean(syserr)))

