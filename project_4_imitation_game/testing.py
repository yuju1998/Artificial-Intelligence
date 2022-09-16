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
from NN_repeatedForward_test import NN_repeatedForward, RepeatedForwardDataset


### csv path
csv_pth = "./test_repeated_forward/repeatedforward_trial"
best_model = "Final_repeatedForwardAgent.pth"


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
parser.add_argument('--batch_size', type=int, default=40)
args = parser.parse_args()



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

def test(model, model_pth, dataloaders, dataset_sizes):

    start = time.time()
    corrects = 0

    for idx, input_data in enumerate(dataloaders):
        
        model.eval()

        gridworld, labels, _ = input_data
        gridworld = torch.unsqueeze(gridworld, 1)
        if torch.cuda.is_available():
            gridworld, labels = gridworld.to(device, dtype=torch.float), labels.to(device, dtype=torch.long)
            model.to(device)

        model.load_state_dict(torch.load(model_pth))

        with torch.no_grad():
            logits = model(gridworld)
            preds = torch.argmax(logits, 1)
        
        print("#{} batch prediction:{}".format(idx, preds))
        print("#{} batch label:{}".format(idx, labels.data))
        corrects += torch.sum(preds==labels.data)
        print('#{} batch corrects: {}'.format(idx, torch.sum(preds == labels.data)),
              '#{} batch accuracy: {}%'.format(idx, torch.sum(preds == labels.data)*100. /args.batch_size))
        
    
    test_acc = corrects.double() / dataset_sizes
    time_elapse = time.time() - start
    print("-" * 10)
    print("Testing Acc: {}".format(test_acc))
    print("Testing time: {}m {}s".format(time_elapse//60, time_elapse%60))

def main():
    num_trials = 30
    for trial in range(num_trials):

        repeated_dataset = RepeatedForwardDataset(csv_pth+"{}.csv".format(trial), dim=args.dim)
        dataloaders = DataLoader(repeated_dataset, batch_size=args.batch_size, shuffle=False) # need to keep track of NN agent's step
        dataset_sizes = len(repeated_dataset)

        os.environ['CUDA_VISIBLE_DEVICES'] = args.gpu
        if torch.cuda.is_available():
            use_gpu = True
        else:
            use_gpu = False

        if use_gpu:
            print("Currently using GPU")
            cudnn.benchmark = True
        else:
            print("Currently using CPU")

        model = NN_repeatedForward(num_classes=args.num_classes)
        sys.stdout = Logger(osp.join("./test_log/log_test_repeatedForward_trial{}.txt".format(trial)))
        test(model, model_pth=best_model, dataloaders=dataloaders, dataset_sizes=dataset_sizes)



if __name__ == '__main__':
    main()

