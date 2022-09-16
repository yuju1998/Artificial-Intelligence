import os
import torch
from torch import float32, nn
import torch.nn.functional as F

from torch.utils.data import Dataset, DataLoader, random_split
from torchvision import datasets, transforms
from torchvision.transforms import ToTensor
import numpy as np
import pandas as pd

# data

# transfer string data to np array
def to_nparray(data, idx, dim):
    temp_list = list(data[idx]) # items in this list are str type
    data_list = []
    for item in temp_list:
        try:
            data_list.append(float(item))
        except:
            pass
    data_array = np.array(data_list).reshape((dim, dim))
    data_array[1][1] = 0.0
    norm = np.linalg.norm(data_array)
    normal_array = data_array/norm
    return normal_array

class RepeatedForwardDataset(Dataset):
    def __init__(self, csv_file, dim):
        super(RepeatedForwardDataset, self).__init__()
        self.dataset = pd.read_csv(csv_file)
        self.gridworld = self.dataset["local_grid"]
        self.dim = dim
        #print(self.dataset)
    def __len__(self):
        return len(self.dataset)

    def __getitem__(self, idx):

        gridworld = to_nparray(self.gridworld, idx, self.dim)
        gridworld = torch.tensor(gridworld)
        label = self.dataset["next_move"][idx]
        gridend = self.dataset["at_goal"][idx]
        # label equals to next move (agent's action)

        return gridworld, label, gridend

class ConvBlock(nn.Module):
    def __init__(self, in_size, out_size):
        super(ConvBlock, self).__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_size, out_size, kernel_size=(1, 1), bias=False), 
            nn.ReLU(), 
            nn.BatchNorm2d(out_size), 
            nn.MaxPool2d(kernel_size=(2, 2)), 
            nn.Dropout2d(p=0.5)
        )
    def forward(self, x):
        x = self.block(x)
        return x

class NN_repeatedForward(nn.Module):
    def __init__(self, num_classes = 5):
        """
        Args:
            num_classes: number of classes
        """
        super(NN_repeatedForward, self).__init__()
        #self.flatten = nn.Flatten()

        # pass gridworld here
        # gridworld only has one channel
        self.conv1 = ConvBlock(1, 8)
        self.pool = nn.MaxPool2d(2, 2)
        self.dropout = nn.Dropout2d(p=0.25)
        self.conv2 = ConvBlock(8, 32)
        self.fc1 = nn.Linear(32, 64)
        self.fc2 = nn.Linear(64, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, num_classes)

    def forward(self, x):
        x = self.conv1(x)
        x = self.conv2(x)
        x = torch.flatten(x, 1) # flatten all dimensions except batch
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        x = self.dropout(x)
        x = F.relu(self.fc3(x))

        x = self.fc4(x)
        return x

