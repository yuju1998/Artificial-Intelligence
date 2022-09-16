import os
import torch
from torch import float32, nn
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
    return data_array

def min_max_normalize(data):
    for column in data.columns:
            max = data[column].max()
            min = data[column].min()
            if max != min:
                data[column] = (data[column] - min) / (max - min)
            elif max == min != 0:
                data[column] = data[column] / max
            else:
                pass
    return data

class RepeatedForwardDataset(Dataset):
    #TODO: how to determine data's state -> train/val/test?
    def __init__(self, csv_file, dim=3):
        super(RepeatedForwardDataset, self).__init__()
        self.dataset = pd.read_csv(csv_file)
        self.local_grid = self.dataset["local_grid"]
        self.dim = dim

    def __len__(self):
        return len(self.dataset)

    def __getitem__(self, idx):
        # return local_grid as np array to feed into maze environment
        local_grid = to_nparray(self.local_grid, idx, self.dim)

        # label equals to next move (agent's action)
        direction = self.dataset["next_direction"][idx]
        #print(label)

        return local_grid, direction



class ConvBlock(nn.Module):
    def __init__(self, in_size, out_size):
        super(ConvBlock, self).__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_size, out_size, kernel_size=(1, 1)), 
            nn.ReLU(), 
            nn.BatchNorm2d(out_size), 
            nn.MaxPool2d(kernel_size=(2, 2)), 
            nn.Dropout2d(p=0.25)
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
        self.conv1 = ConvBlock(1, 16)
        self.conv2 = ConvBlock(16, 32)
        self.conv3 = ConvBlock(32, 64)
        
        #new_dim = (((dim-2)/2 - 2)/2 - 2)/2
        self.ln1 = nn.Linear(64, 16)
        self.relu = nn.ReLU()
        self.batchnorm = nn.BatchNorm1d(16)
        self.dropout = nn.Dropout2d(p=0.1)
        self.ln2 = nn.Linear(16, 8)

        # pass knowledge here
        self.ln3 = nn.Linear(9, 16) # knowledge contains 9 features(columns)
        self.ln4 = nn.Linear(16, 16)
        self.ln5 = nn.Linear(16, 8)

        # pass concated all features here
        self.last_linear = nn.Linear(16, num_classes)
    
    def features(self, gridworld, knowledge):
        gridworld = self.conv1(gridworld)
        gridworld = self.conv2(gridworld)
        gridworld = self.conv3(gridworld)
        gridworld = gridworld.reshape(gridworld.shape[0], -1)
        gridworld = self.ln1(gridworld)
        gridworld = self.relu(gridworld)
        gridworld = self.batchnorm(gridworld)
        gridworld = self.dropout(gridworld)
        gridworld = self.ln2(gridworld)
        gridworld = self.relu(gridworld)

        knowledge = self.ln3(knowledge)
        knowledge = self.relu(knowledge)
        knowledge = self.batchnorm(knowledge)
        knowledge = self.dropout(knowledge)

        knowledge = self.ln4(knowledge)
        knowledge = self.relu(knowledge)
        knowledge = self.batchnorm(knowledge)
        knowledge = self.dropout(knowledge)

        knowledge = self.ln5(knowledge)
        knowledge = self.relu(knowledge)

        x = torch.cat((gridworld, knowledge), dim=1)
        x = self.relu(x)
        
        return x
    
    def logits(self, features):
        y = self.last_linear(features)

        return y

    def forward(self, gridworld, knowledge):
        feature = self.features(gridworld, knowledge)
        output = self.logits(feature)

        return output, feature

