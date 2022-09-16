import sys
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.nn import init
import torch.optim as optim
from torch.optim import lr_scheduler
import numpy as np
import os 
import errno
from torch.utils.data import Dataset, DataLoader, dataloader
from torchvision import transforms, utils, models, datasets
import pandas as pd
import time
import torch.backends.cudnn as cudnn

import os.path as osp
from NN_repeatedForward import NN_repeatedForward, RepeatedForwardDataset

### device
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f'Using {device} device')

import argparse
parser = argparse.ArgumentParser("Repeated Forward")
parser.add_argument('--seed', type=int, default=5)
parser.add_argument('--gpu', type=str, default='0')
parser.add_argument('--use-cpu', action='store_true')
parser.add_argument('--num_classes', type=int, default=5)
parser.add_argument('--dim', type=int, default=3)
parser.add_argument('--batch_size', type=int, default=15)
args = parser.parse_args()

csv_pth = "C:/Users/Ruby/Desktop/520_project4/repeatedforward_data_101"

def split_train_val(csv_filename):
    data = pd.read_csv(csv_filename + ".csv")
    #data = data.sample(frac=1).reset_index(drop=True)
    n = int(len(data)*0.7)
    train_data = data.iloc[0:n-1, ]
    print(n)
    print(train_data)
    val_data = data.iloc[n:, ]
    train_data.to_csv(csv_filename + "_train.csv")
    val_data.to_csv(csv_filename + "_val.csv")

### split data
split_train_val(csv_pth)

### load data
#TODO: does gridworld data needs to be transformed?
repeated_dataset = {x: RepeatedForwardDataset(csv_pth+"_{}.csv".format(x), dim=args.dim)for x in ['train', 'val']}
dataloaders = {x: DataLoader(repeated_dataset[x], batch_size=args.batch_size, shuffle=False) for x in ['train', 'val']}
dataset_sizes = {x: len(repeated_dataset[x]) for x in ['train', 'val']}

"""
### initialize NN model
def NNAgent_repeatedForward(num_classes, dim):
    model = NN_repeatedForward(num_classes, dim)
    num_features = model.last_linear.in_features
    model.last_linear = COCOLoss(num_classes, num_features)
    # use COCOLoss to optimize performance
    return model
"""

"""
refer to Pytorch tutorial:
    https://pytorch.org/tutorials/beginner/blitz/cifar10_tutorial.html
"""
def mkdir_if_missing(directory):
    if not osp.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
class Logger(object):
    """
    Write console output to external text file.
    
    Code imported from https://github.com/Cysu/open-reid/blob/master/reid/utils/logging.py.
    """
    def __init__(self, fpath=None):
        self.console = sys.stdout
        self.file = None
        if fpath is not None:
            mkdir_if_missing(os.path.dirname(fpath))
            self.file = open(fpath, 'w')

    def __del__(self):
        self.close()

    def __enter__(self):
        pass

    def __exit__(self, *args):
        self.close()

    def write(self, msg):
        self.console.write(msg)
        if self.file is not None:
            self.file.write(msg)

    def flush(self):
        self.console.flush()
        if self.file is not None:
            self.file.flush()
            os.fsync(self.file.fileno())

    def close(self):
        self.console.close()
        if self.file is not None:
            self.file.close()

def train(model, criterion, optimizer, scheduler, num_epochs = 5):
    
    start = time.time()
    best_acc = 0.0

    for param in model.parameters():
        param.requires_grad = True

    for epoch in range(num_epochs):
        print("Epoch {}/{}".format(epoch, num_epochs-1))
        print("-"*10)

        for state in ['train', 'val']:
            if state == 'train':
                # tell that model is training
                model.train()
            else:
                # tell that model is doint validation
                model.eval()
            total_loss = 0.0
            corrects = 0

            for idx, input_data in enumerate(dataloaders[state]):
                gridworld, knowledge, labels = input_data
                gridworld = torch.unsqueeze(gridworld, 1)

                if torch.cuda.is_available():
                    gridworld, knowledge, labels = gridworld.to(device, dtype=torch.float), knowledge.to(device, dtype=torch.float), labels.to(device, dtype=torch.long)
                    model.cuda()
                
                # sets gradient calculation to ON when state == train
                with torch.set_grad_enabled(state == 'train'):                    
                    # set zero to the parameter gradients
                    optimizer.zero_grad()
                    logits, _ = model(gridworld, knowledge)
                    loss = criterion(logits, labels)
                    _, preds = torch.max(logits, 1)
                if state == 'train':
                    loss.backward()
                    optimizer.step()
                    scheduler.step()
                print("prediction:{}, label:{}".format(preds, labels.data))
                total_loss += loss.item()
                corrects += torch.sum(preds==labels.data)
                print('#{} batch loss: {}'.format(idx, loss.item()), 
                      '#{} batch corrects: {}'.format(idx, torch.sum(preds == labels.data)),
                      '#{} batch accuracy: {}%'.format(idx, torch.sum(preds == labels.data)*100. /args.batch_size))
        
        epoch_loss = total_loss/dataset_sizes[state]
        epoch_acc = corrects.double() / dataset_sizes[state]

        print('{} Loss: {} Acc: {}'.format(state, epoch_loss, epoch_acc))

        if state == 'val' and epoch_acc > best_acc:
            best_acc = epoch_acc
        temp_model_path = "./repeatedForwardAgent_{}.pth".format(epoch)
        # save models generated in each epoch
        torch.save(model.state_dict(), temp_model_path)
    
    print("-" * 10)
    time_elapsed = time.time() - start
    print("training time: {}m {}s".format(time_elapsed//60, time_elapsed%60))
    print("best validation accuracy: {}".format(best_acc))

    return model

def main():
    torch.manual_seed(args.seed)
    os.environ['CUDA_VISIBLE_DEVICES'] = args.gpu
    if torch.cuda.is_available():
        use_gpu = True
    else:
        use_gpu = False

    if use_gpu:
        print("Currently using GPU")
        cudnn.benchmark = True
        torch.cuda.manual_seed_all(args.seed)
    else:
        print("Currently using CPU")
        
    model = NN_repeatedForward(num_classes=args.num_classes)
    model.cuda()
    criterion = nn.CrossEntropyLoss()
    sys.stdout = Logger(osp.join("./log_train_repeatedForward.txt"))

    # optimize
    params_to_update = model.parameters()
    optimizer = optim.SGD(params_to_update, lr=1e-06, momentum=0.9)
    exp_lr_scheduler = lr_scheduler.StepLR(optimizer, step_size=5, gamma=0.1)

    model_train = train(model=model, 
                        criterion=criterion, 
                        optimizer=optimizer, 
                        scheduler=exp_lr_scheduler, 
                        num_epochs=5)
    final_model_path = "./Final_repeatedForwardAgent.pth"
    torch.save(model_train.state_dict(), final_model_path)

if __name__=='__main__':
    main()
