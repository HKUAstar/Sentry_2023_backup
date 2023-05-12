import time
import copy
import torch
from torch.utils.data import DataLoader, Dataset
from torchvision.io import read_image
import torch.nn.functional as F
import torch.nn as nn
import os
import sys

os.environ['CUDA_LAUNCH_BLOCKING'] = '1'
BATCH_SIZE = 16
EPOCH = 300
NUM = 8
SAVE_PATH = './model.pt'
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

class NumberDataset(Dataset):
	def __init__(self, img_path, label_path):
		super().__init__()
		self.data = []
		self.label = []
		for img_name in os.listdir(img_path):
			img = read_image(img_path + img_name)
			self.data.append(img)
			name = img_name.rpartition('.')[0]
			with open(label_path+name+".txt") as file:
				y = int(file.readline().strip('\n'))
				lb = torch.zeros(NUM)
				lb[y - 1] = 1
				self.label.append(lb)

	def __len__(self):
		return len(self.data)

	def __getitem__(self, item):
		return self.data[item], self.label[item]


CHANEL1 = 10
CHANEL2 = 20
class CNN(nn.Module):
	def __init__(self):
		super().__init__()
		self.conv1 = nn.Conv2d(1, CHANEL1, kernel_size=5)
		self.conv2 = nn.Conv2d(CHANEL1, CHANEL2, kernel_size=5)
		self.drop = nn.Dropout()
		self.pool = nn.MaxPool2d(2, 2)
		self.dense1 = nn.Linear(CHANEL2*5*5, 128)
		self.dense2 = nn.Linear(128, NUM)
		# self.dense3 = nn.Linear(128, NUM)

	def forward(self, x):
		# print(type(x), x.shape)
		x = self.pool(F.relu(self.conv1(x)))
		x = self.pool(F.relu(self.drop(self.conv2(x))))
		x = x.view(-1, CHANEL2*5*5)
		x = F.relu(self.dense1(x))
		x = F.dropout(x, training=self.training)
		x = self.dense2(x)
		return x


def train(train_Loader, valid_Loader, model, loss_fn, optimizer):
	model.train()
	size = len(train_Loader.dataset)
	max_accuracy = 0
	best = None
	for epoch in range(EPOCH):
		epoch_loss = 0
		for batch, (x, y) in enumerate(train_Loader):
			x, y = x.to(device), y.to(device)

			optimizer.zero_grad()
			pred = model(x.float())
			# print(pred.shape, y.shape)
			loss = loss_fn(pred, y)

			loss.backward()
			optimizer.step()
			loss, current = loss.item(), min(batch*BATCH_SIZE, size)

			print(f'loss: {loss:.6f}, [{current}/{size}]    ', end = '\r')
			epoch_loss += loss
		accuracy = inference(valid_Loader, model)
		if accuracy > max_accuracy:
			max_accuracy = accuracy
			best = copy.deepcopy(model.state_dict())
		print(f'Epoch [{epoch}/{EPOCH}]: Loss={epoch_loss:.6f}      Accuracy: {accuracy}     Max: {max_accuracy}')
		model.train()

	torch.save(best, SAVE_PATH)

def inference(dataLoader, model):
	model.eval()
	tot = 0
	err = 0
	for img, label in dataLoader:
		img, label = img.to(device), label.to(device)
		for i in range(len(img)):
			tot += 1
			y = model.forward(img[i].float())
			y = y.argmax()+1
			num = label[i].argmax()+1
			if y != num:
				err += 1
	return (tot-err)/tot

if __name__ == '__main__':
	arg = sys.argv
	if len(arg) > 1:
		EPOCH = int(arg[1])
		# print(123123)
	timer = time.time()
	train_data = NumberDataset('./imgs/train/', './labels/train/')
	# valid_data = NumberDataset('./imgs/valid/', './valid/labels/')
	test_data = NumberDataset('./imgs/test/', './labels/test/')
	train_dataLoader = DataLoader(train_data, batch_size=BATCH_SIZE, shuffle=True, num_workers=8)
	# valid_dataLoader = DataLoader(valid_data, batch_size=BATCH_SIZE, shuffle=True, num_workers=8)
	test_dataLoader = DataLoader(test_data, batch_size=BATCH_SIZE, shuffle=False, num_workers=8)
	print(f'Data loaded in {time.time()-timer}s')

	model = CNN()
	cost = nn.CrossEntropyLoss()
	# optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
	optimizer = torch.optim.SGD(model.parameters(), lr=0.003)
	model = model.to(device)
	train(train_dataLoader, test_dataLoader, model, cost, optimizer)
	best = CNN()
	best.load_state_dict(torch.load(SAVE_PATH))
	best.to(device)
	timer = time.time()
	print(inference(test_dataLoader, best))
	print(f"Inference finished in {time.time()-timer}s")
