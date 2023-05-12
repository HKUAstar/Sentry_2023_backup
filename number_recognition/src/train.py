import torch
import torchvision
import copy
import os
from torch import nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader

os.environ['CUDA_LAUNCH_BLOCKING'] = '1'

torch.device('cpu'), torch.cuda.device('cuda'), torch.cuda.device('cuda:1')
torch.cuda.device_count()
    
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    
def try_gpu(i=0):
    if torch.cuda.device_count() >= i + 1:
        return torch.device(f'cuda:{i}')
    return torch.device('cpu')

def try_all_gpus():
    devices = [torch.device(f'cuda:{i}') for i in range(torch.cuda.device_count())]
    return devices if devices else [torch.device('cpu')]

class Accumulator:
    def __init__(self, n):
        self.data = [0.0] * n
    def add(self, *args):
        self.data = [a + float(b) for a, b in zip(self.data, args)]
    def reset(self):
        self.data = [0.0] * len(self.data)
    def __getitem__(self, idx):
        return self.data[idx]

THRESHOLD = 100
def process(img):
    visited =  [[0 for i in range(32) ] for j in range(32)]
    for i in range(32):
        x, y = i, 0
        l = [(x, y)]
        pf, pe = 0, 1
        while (pf < pe):
            ux, uy = l[pf]
            pf += 1
            if visited[ux][uy] == 1:
                continue
            visited[ux][uy] = 1
            if img[0][ux][uy] < THRESHOLD:
                continue
            img[0][ux][uy] = 0
#            print(img[0][ux][uy])
            for wx, wy in [(0, -1), (0, 1), (1, 0), (-1, 0)]:
                nx, ny = ux + wx, uy + wy
                if 0 <= nx < 32 and 0 <= ny < 32:
                    if visited[nx][ny] == 0:
                        l.append((nx, ny))
                        pe += 1
        x, y = i, 31
        l = [(x, y)]
        pf, pe = 0, 1
        while (pf < pe):
            ux, uy = l[pf]
            pf += 1
            if visited[ux][uy] == 1:
                continue
            visited[ux][uy] = 1
            if img[0][ux][uy] < THRESHOLD:
                continue
            img[0][ux][uy] = 0
            for wx, wy in [(0, -1), (0, 1), (1, 0), (-1, 0)]:
                nx, ny = ux + wx, uy + wy
                if 0 <= nx < 32 and 0 <= ny < 32:
                    if visited[nx][ny] == 0:
                        l.append((nx, ny))
                        pe += 1
    return img

class MyData(Dataset):

    def __init__(self, root_dir, imgs_dir, labels_dir):
        super().__init__()
        self.imgs_path = os.path.join(root_dir, imgs_dir)
        self.labels_path = os.path.join(root_dir, labels_dir)
        
        self.imgs_name = os.listdir(self.imgs_path)
        

    def __getitem__(self, idx):
        item_img_name = self.imgs_name[idx]
        item_img_path = os.path.join(self.imgs_path, item_img_name)
        img = torchvision.io.read_image(item_img_path).reshape(1, 32, 32).float()

        item_label_name = item_img_name.rpartition('.')[0] + '.txt'
        item_label_path = os.path.join(self.labels_path, item_label_name)
        with open(item_label_path, "r") as f:
            id = int(f.read())-1
            label = torch.zeros(NUM)
            label[id] = 1
        return img, label

    def __len__(self):
        return len(self.imgs_name)

root_dir = "."
train_imgs_dir = "imgs/train"
train_labels_dir = "labels/train"
test_imgs_dir = "imgs/test"
test_labels_dir = "labels/test"


# train_dataset_origin = MyData(root_dir, train_imgs_dir, train_labels_dir)
# train_dataset_noisy = MyData(root_dir, "imgs_noise/train", train_labels_dir)
# train_dataset_light = MyData(root_dir, "img_light/train", train_labels_dir)

# test_dataset_origin = MyData(root_dir, test_imgs_dir, test_labels_dir)
# test_dataset_astar = MyData(root_dir, "astarset/imgs", "astarset/labels")

# train_dataset = train_dataset_origin + train_dataset_noisy + train_dataset_light
# test_dataset = test_dataset_astar

CHANNEL1 = 10
CHANNEL2 = 20
NUM = 8
class Net(torch.nn.Module):

    def __init__(self):
        super().__init__()
        self.conv1 = nn.Sequential(nn.Conv2d(1, CHANNEL1, kernel_size=5),
                                   nn.BatchNorm2d(CHANNEL1), 
                                   nn.Dropout(), 
                                   nn.ReLU(), 
                                   nn.MaxPool2d(2, 2),
                                   nn.ReLU(),
                                   )
        self.conv2 = nn.Sequential(nn.Conv2d(CHANNEL1, CHANNEL2, kernel_size=5), 
                                   nn.BatchNorm2d(CHANNEL2), 
                                   nn.Dropout(), 
                                   nn.ReLU(),
                                   nn.MaxPool2d(2, 2),
                                   nn.ReLU(), 
                                   )
        
        # self.conv1 = nn.Conv2d(1, CHANNEL1, kernel_size=5)
        # self.conv2 = nn.Conv2d(CHANNEL1, CHANNEL2, kernel_size=5)
        # self.drop = nn.Dropout()
        # self.pool = nn.MaxPool2d(2, 2)

        self.dense1 = nn.Linear(CHANNEL2 * 5 * 5, 128)
        self.dense2 = nn.Linear(128, 32)
        self.dense3 = nn.Linear(32, 8)

    def forward(self, x):
        x = self.conv1(x)
        x = self.conv2(x)
        x = x.view(-1, CHANNEL2 * 5 * 5)
        x = F.relu(self.dense1(x))
        x = F.dropout(x, training=self.training)
        x = F.relu(self.dense2(x))
        x = self.dense3(x)
        return x

def accuracy(y_hat, y):
    y_hat = torch.argmax(y_hat, dim=1)
    y = torch.argmax(y, dim=1)
    tot = 0
    for i in range(len(y)):
        if y_hat[i] == y[i]:
            tot += 1
    return tot

def evaluate_accuracy(Model, data_iter):
    Model.eval()
    metric = Accumulator(2)
    for X, y in data_iter:
        X, y = X.to(device), y.to(device)
        metric.add(accuracy(Model(X), y), y.shape[0])
    return metric[0] / metric[1]

def train_epoch(Model, train_iter, loss, optimizer):
    Model.train()
    metric = Accumulator(3)
    for X, y in train_iter:
        X, y = X.to(device), y.to(device)
        optimizer.zero_grad()
        y_hat = Model(X)
        l = loss(y_hat, y)
        l.backward()
        optimizer.step()
        metric.add(l.item() * y.shape[0], accuracy(y_hat, y), y.shape[0])
    return metric[0] / metric[2], metric[1] / metric[2]

def Train_Model(Model, loss, num_epochs, optimizer, train_dataloader, test_dataloader):
    max_test_acc = 0
    Best_Model = Net()
    for epoch in range(num_epochs):
        train_metrics = train_epoch(Model, train_dataloader, loss, optimizer)
        test_acc = evaluate_accuracy(Model, test_dataloader)
        
        if(test_acc > max_test_acc):
            max_test_acc = test_acc
            Best_Model = copy.deepcopy(Model)
        print(f"{epoch+1}/{num_epochs} loss: {train_metrics[0]} Correct rate: {test_acc} Max rate: {max_test_acc}")
    return Best_Model, max_test_acc
    
def Predict(Model, img):
    Model.eval()
    y_hat = Model(img)
    return y_hat

def write_data(root, img, label, id):
    torchvision.io.write_jpeg(img, os.path.join(root, "imgs", f"{id}.jpg"))
    with open (os.path.join(root, "labels", f"{id}.txt"), "w") as f:
        f.write(f"{id}")

BATCH_SIZE = 16
num_epochs = 500

# if __name__ == '__main__':
    
#     train_dataloader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True, num_workers=8)
#     test_dataloader = DataLoader(test_dataset, batch_size=BATCH_SIZE, shuffle=False, num_workers=8)
#     print("Data Loading finished")
#     # for id, (img, label) in enumerate(train_dataloader):
#     #     for i in range(img.shape[0]):
#     #         write_data("./data_clear", img[i].type(torch.uint8), label[i]+1, id*16+i)
#             #print(img[i].type(torch.uint8))
#             #exit(0)
#     Model = Net()
#     loss = nn.CrossEntropyLoss()
#     optimizer = torch.optim.SGD(Model.parameters(), lr=0.003)

#     Model.to(device)
#     Model, acc = Train_Model(Model, loss, num_epochs, optimizer, train_dataloader, test_dataloader)
#     torch.save(Model, "./Model.pt")

#     # Model = torch.load("./Model9041.pt")
#     # Model.to(device)
#     print(evaluate_accuracy(Model, test_dataloader))

#     id = 0
#     for x, y in test_dataset:
#         x, y = x.to(device), y.to(device)
#         y_hat = Predict(Model, x)

#         label = torch.argmax(y) + 1
#         label_hat = torch.argmax(y_hat) + 1

#         if (label_hat != label):
#             x = x.to('cpu').type(torch.uint8)
#             torchvision.io.write_jpeg(x, os.path.join("./img_wrong", f"{id}.jpg"))
#             with open ("label.txt", "a") as f:
#                 f.write(f"{id}:   Predict: {label_hat}  True: {label}\n")
#             id += 1
