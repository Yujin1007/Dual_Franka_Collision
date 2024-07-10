import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import numpy as np
import pandas as pd
import argparse

def main(dataset_path, model_path, batch_size):
    # Check for GPU availability
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    chunk_size = 10 ** 6
    chunks = pd.read_csv(dataset_path, header=None, chunksize=chunk_size)
    random_state = 2
    # Initialize empty lists to store chunks
    X_chunks = []
    Y_chunks = []

    # Read the chunks
    for chunk in chunks:
        # Splitting into features and target
        X_chunk = chunk.iloc[:, :14].values
        Y_chunk = chunk.iloc[:, 14].values.reshape(-1, 1)

        X_chunks.append(X_chunk)
        Y_chunks.append(Y_chunk)

    # Concatenate chunks
    X = np.concatenate(X_chunks, axis=0)
    Y = np.concatenate(Y_chunks, axis=0)

    # Separate samples based on the label
    X_label0 = X[Y[:, 0] == 0]
    X_label1 = X[Y[:, 0] == 1]

    # Randomly select samples from label 0 to match the number of samples in label 1
    np.random.seed(random_state)
    np.random.shuffle(X_label0)
    X_label0_balanced = X_label0[:len(X_label1)]

    # Combine balanced samples with label 1 samples
    X_balanced = np.concatenate([X_label0_balanced, X_label1], axis=0)
    Y_balanced = np.concatenate([np.zeros((len(X_label1), 1)), np.ones((len(X_label1), 1))], axis=0)

    # Splitting into train and test sets

    # X_train, X_test, Y_train, Y_test = train_test_split(X_balanced, Y_balanced, test_size=0.2, random_state=2) # light datasets
    X_train, X_test, Y_train, Y_test = train_test_split(X_balanced, Y_balanced, test_size=0.01, random_state=2) # heavy datasets

    # Convert data to PyTorch tensors
    train_dataset = TensorDataset(torch.tensor(X_train, dtype=torch.float32),
                                  torch.tensor(Y_train, dtype=torch.float32))
    test_dataset = TensorDataset(torch.tensor(X_test, dtype=torch.float32), torch.tensor(Y_test, dtype=torch.float32))

    # Create DataLoaders
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=len(test_dataset), shuffle=False)

    # Define the model
    class NeuralNetwork(nn.Module):
        def __init__(self):
            super(NeuralNetwork, self).__init__()

            # best_model_dual_panda_deep2_0419
            self.fc1 = nn.Linear(14, 256)
            self.fc2 = nn.Linear(256, 256)
            self.fc3 = nn.Linear(256, 256)
            self.fc4 = nn.Linear(256, 128)
            self.fc5 = nn.Linear(128, 1)

            # best_model_dual_panda_dropout_0419
            # self.fc1 = nn.Linear(14, 1024)
            # self.fc2 = nn.Linear(1024, 1024)
            # self.fc3 = nn.Linear(1024, 64)
            # self.fc4 = nn.Linear(64, 1)
            # self.dropout = nn.Dropout(0.2)  # 드롭아웃 추가

            # best_model_dual_panda_deep_0419, best_model_dual_panda_deep_0419_batch10000
            # self.fc1 = nn.Linear(14, 256)
            # self.fc2 = nn.Linear(256, 256)
            # self.fc3 = nn.Linear(256, 128)
            # self.fc4 = nn.Linear(128, 1)

            # best_model_dual_panda_light_0417
            # self.fc1 = nn.Linear(14, 256)
            # self.fc2 = nn.Linear(256, 64)
            # self.fc3 = nn.Linear(64, 1)

            # original network (no saved param. train (x))
            # self.fc1 = nn.Linear(14, 32)
            # self.fc2 = nn.Linear(32, 32)
            # self.fc3 = nn.Linear(32, 1)

        def forward(self, x):
            x = torch.relu(self.fc1(x))
            x = torch.relu(self.fc2(x))
            x = torch.relu(self.fc3(x))
            x = torch.relu(self.fc4(x))
            x = torch.sigmoid(self.fc5(x))

            # x = torch.relu(self.fc1(x))
            # x = self.dropout(x)
            # x = torch.relu(self.fc2(x))
            # x = self.dropout(x)
            # x = torch.relu(self.fc3(x))
            # x = self.dropout(x)
            # x = torch.sigmoid(self.fc4(x))
            return x

    class ConvNeuralNetwork(nn.Module):
        def __init__(self):
            # conv
            super(ConvNeuralNetwork, self).__init__()
            self.conv1 = nn.Conv2d(1, 256, kernel_size=2)  # 1 input channel, 256 output channels, 2x2 kernel
            self.fc1 = nn.Linear(256 * 6 * 1,
                                 256)  # 32 output channels from conv1, reshaped to (32*6*1), 32 output neurons
            self.fc2 = nn.Linear(256, 128)
            self.fc3 = nn.Linear(128, 1)




        def forward(self, x):
            x = x.view(-1, 1, 7, 2)  # reshape input from (batch_size, 14) to (batch_size, 1, 7, 2)
            x = torch.relu(self.conv1(x))
            x = x.view(-1, 256 * 6 * 1)  # flatten the output of conv1
            x = torch.relu(self.fc1(x))
            x = torch.relu(self.fc2(x))
            x = torch.sigmoid(self.fc3(x))

            return x

    # model = NeuralNetwork().to(device)
    model = ConvNeuralNetwork().to(device)
    # Define loss function and optimizer
    criterion = nn.BCELoss()
    # optimizer = optim.Adam(model.parameters())
    #drop out 적용하고부터 weight decay 도 사용
    optimizer = optim.Adam(model.parameters(), weight_decay=1e-5)

    # Training the model
    num_epochs = 3000
    max_accuracy = 0
    min_loss = 100000

    for epoch in range(num_epochs):
        for inputs, labels in train_loader:
            inputs, labels = inputs.to(device), labels.to(device)

            # Forward pass
            outputs = model(inputs)
            loss = criterion(outputs, labels)

            # Backward and optimize
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        # if (epoch+1) % 10 == 0: # light
        if (epoch+1) % 1 == 0: # heavy
            print(f'Epoch [{epoch+1}/{num_epochs}], Train Loss: {loss.item():.4f}')
            for inputs, labels in test_loader:
                inputs, labels = inputs.to(device), labels.to(device)

                # Forward pass
                outputs = model(inputs)
                outputs = (outputs > 0.5).float()

                accuracy = torch.sum(outputs == labels).item()
            print("test 정확도 : ", accuracy, "/", len(outputs))
            if accuracy > max_accuracy and loss.item() < min_loss:
                # Save the trained model
                model_name = model_path + ".pth"
                torch.save(model.state_dict(), model_name)
                weight_prefix = "weight/dual_arm/"+model_path
                # for i, param in enumerate(model.parameters()):
                #     np.savetxt(f'{weight_prefix}[{i}].txt', param.detach().cpu().numpy())
                for i, param in enumerate(model.parameters()):
                    if len(param.shape) > 1:  # Check if the parameter is not a bias parameter
                        weights = param.detach().cpu().numpy().flatten()
                    else:
                        weights = param.detach().cpu().numpy()
                    np.savetxt(f'{weight_prefix}[{i}].txt', weights)
                print("---------save new model ---------\n")
            if accuracy > max_accuracy:
                max_accuracy = accuracy
            if loss.item() < min_loss:
                min_loss = loss.item()

    # Save the trained model
    model_name = model_path + "_final.pth"
    torch.save(model.state_dict(), model_name)

    # Load the model
    model.load_state_dict(torch.load(model_name))

    for inputs, labels in test_loader:
        inputs, labels = inputs.to(device), labels.to(device)

        # Forward pass
        outputs = model(inputs)
        outputs = (outputs > 0.5).float()

        accuracy = torch.sum(outputs == labels).item()
    print("정확도 : ", accuracy,"/",len(outputs))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train a neural network model.")
    parser.add_argument("dataset_path", type=str, help="Path to the dataset file", default="save_data/dual_panda/panda_light_0409.txt")
    parser.add_argument("model_path", type=str, help="Path to save the trained model", default="test_0416")
    parser.add_argument("batch_size", type=int, help="batch size", default="126")

    args = parser.parse_args()

    main(args.dataset_path, args.model_path, args.batch_size)
