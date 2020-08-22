# -*- coding: utf-8 -*-
"""
Created on Sat Aphril  27 09:00:44 2020

@author: www.haroldmurcia.com

"""
import matplotlib.pyplot as plt
#from sklearn import datasets
import pandas as pd
import torch
import sys, os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.legend import Legend
from datetime import datetime

import os
path_src  = os.path.abspath('')
path_main = os.path.abspath('..')
path_data = os.path.join(path_main,'data')

PI=torch.acos(torch.Tensor([-1]))


class analisys(object):
    def __init__(self,data):
        print("Data ok (y)")
        self.df   = data
        #drop_rows = self.df.loc[(self.df["Range"]==0) | (self.df["Range"]==60) | (self.df["I"]==0)]
        #self.df   = self.df.drop(drop_rows.index)
        #print(self.df.head())
        #self.df.I = self.normalization(self.df.I,1024)
        #self.df["classification"] =""
        #self.insights()
        # split
        #pos_g  = self.df.Z<=0.3
        #self.df.classification[pos_g]=0
        #pos_v  = self.df.Z>0.3
        #
        #self.df.classification[pos_v]=1
        # Intensity correction I=I*Range^2
        #self.df.I = self.df.I*self.df.Range*self.df.Range
        #self.df.I = self.normalization(self.df.I,1024)
        self.charts(self.df)
        #   sys.exit(0)

    def insights(self):
        Total        =self.df.shape[0]
        first_echoes =(self.df['Echoe_position']==1).sum()
        middle_echoes=(self.df['Echoe_position']==2).sum()
        last_echoes  =(self.df['Echoe_position']==3).sum()
        #
        print("Total samples: "+str(Total))
        print("Firs echoes: "  +str(first_echoes))
        print("Middle echoes: "+str(middle_echoes))
        print("Last echoes: "  +str(last_echoes))
        print("First+middle+last: "+str(last_echoes+middle_echoes+first_echoes))

    def normalization(self,df,N):
        data_max=max(df)
        data_min=min(df)
        df=N*(df-data_min)/(data_max-data_min)
        return df

    def intensity_hist(self,df,N):
        plt.figure()
        plt.hist(df.I[df.classification==0],N)
        plt.hist(df.I[df.classification==1],N)
        plt.show()
        plt.close()
        ##
        ##
        fig, axs = plt.subplots(3)
        axs[0].scatter(df.X, df.Y, c=df.I, s=0.1)
        axs[1].scatter(df.X, df.Z, c=df.I, s=0.1)
        axs[2].scatter(df.Y, df.Z, c=df.I, s=0.1)
        plt.show()
        plt.close()
        ##
        fig = plt.figure()
        ax  = fig.add_subplot(111, projection='3d')
        SA_tensor_g =  torch.tensor(df.Sa[df.classification==0].values)
        I_tensor_g  =  torch.tensor(df.I[df.classification==0].values)
        Range_tensor_g  =  torch.tensor(df.Range[df.classification==0].values)
        SA_tensor_v =  torch.tensor(df.Sa[df.classification==1].values)
        I_tensor_v  =  torch.tensor(df.I[df.classification==1].values)
        Range_tensor_v  =  torch.tensor(df.Range[df.classification==1].values)
        ax.scatter(Range_tensor_g,torch.cos(SA_tensor_g*PI/180.0),I_tensor_g ,s=0.1)
        ax.scatter(Range_tensor_v,torch.cos(SA_tensor_v*PI/180.0), I_tensor_v,s=0.1)
        plt.show()
        plt.close()

    def charts(self,df):
        fig, axs = plt.subplots(6)
        axs[0].scatter(df.X, df.Y, c=df.Z, s=0.1)
        axs[1].scatter(df.X, df.Z, c=df.I, s=0.1)
        axs[2].scatter(df.X, df.Y, c=df.Echoe_position, s=0.1)
        axs[3].scatter(df.X, df.Z, c=df.Echoe_position, s=0.1)
        colo=df.R/df.Nr
        axs[4].scatter(df.X, df.Y, c=colo, s=0.1)
        axs[5].scatter(df.X, df.Z, c=colo, s=0.1)
        plt.show()
        plt.close()


if __name__ == "__main__":
    path_txt = os.path.join(path_data,'reco_move_output.txt')
    data     = pd.read_csv(path_txt,sep='\t',header=0)
    data     = data.loc[(data.Range)<10]
    L=len(data)
    print("Length data",L)
    data     = data.sample(n=100, random_state=1)
    result   = analisys(data)
