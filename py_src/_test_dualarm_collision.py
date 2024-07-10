import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import numpy as np
import pandas as pd
import argparse

def main(dataset_path, model_path):
    # Check for GPU availability
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Read the dataset in chunks
    chunk_size = 10**6  # Define the chunk size
    # batch_size = 10000
    chunks = pd.read_csv(dataset_path, header=None, chunksize=chunk_size)

    # Initialize empty lists to store chunks
    X_chunks = []
    Y_chunks = []

    # Read the chunks
    for chunk in chunks:
        # Splitting into features and target
        X_chunk = chunk.iloc[:, :14].values
        Y_chunk = chunk.iloc[:, 14].values.reshape(-1, 1)

        # Standardize the features
        # scaler = StandardScaler()
        # X_chunk = scaler.fit_transform(X_chunk)

        X_chunks.append(X_chunk)
        Y_chunks.append(Y_chunk)

    # Concatenate chunks
    X = np.concatenate(X_chunks, axis=0)
    Y = np.concatenate(Y_chunks, axis=0)

    # Convert data to PyTorch tensors
    test_dataset = TensorDataset(torch.tensor(X, dtype=torch.float32), torch.tensor(Y, dtype=torch.float32))

    # Create DataLoaders
    test_loader = DataLoader(test_dataset, batch_size=len(test_dataset), shuffle=False)

    # Define the model
    class NeuralNetwork(nn.Module):
        def __init__(self):
            super(NeuralNetwork, self).__init__()
            self.fc1 = nn.Linear(14, 256)
            self.fc2 = nn.Linear(256, 256)
            self.fc3 = nn.Linear(256, 128)
            self.fc4 = nn.Linear(128, 1)

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
            print("hidden 1 0~3\n", x[0][9])
            x = torch.relu(self.fc2(x))
            print("hidden 2 0~3", x[0][1])
            x = torch.relu(self.fc3(x))
            print("hidden 3 0~3", x[0][0])
            x = torch.sigmoid(self.fc4(x))
            # print("output : ", x[0])
            return x

    model = NeuralNetwork().to(device)

    model_name = model_path + ".pth"

    model.load_state_dict(torch.load(model_name))
    # Save weights
    weight_prefix = "weight/dual_arm/"+model_path
    for i, param in enumerate(model.parameters()):
        np.savetxt(f'{weight_prefix}[{i}].txt', param.detach().cpu().numpy())

    for inputs, labels in test_loader:
        inputs, labels = inputs.to(device), labels.to(device)

        # Forward pass
        outputs = model(inputs)
        outputs = (outputs > 0.5).float()

        accuracy = torch.sum(outputs == labels).item()
    print("정확도 : ", accuracy, "/", len(outputs))


    for inputs, labels in test_loader:
        inputs, labels = inputs.to(device), labels.to(device)

        # Forward pass
        outputs = model(inputs)
        outputs = (outputs > 0.5).float()

        accuracy = torch.sum(outputs == labels).item()
    print("정확도 : ", accuracy,"/",len(outputs))


    # Save weights
    # weight_prefix = "weight/dual_arm/"+model_path
    # for i, param in enumerate(model.parameters()):
    #     # np.savetxt(f'{weight_prefix}[{i}].txt', param.detach().cpu().numpy())
    #     pd.DataFrame(param.detach().cpu().numpy()).to_csv(f'{weight_prefix}[{i}].csv', header=False, index=False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train a neural network model.")
    parser.add_argument("dataset_path", type=str, help="Path to the dataset file", default="save_data/dual_panda/panda_light_0409.txt")
    parser.add_argument("model_path", type=str, help="Path to save the trained model", default="test_0416")

    args = parser.parse_args()

    main(args.dataset_path, args.model_path)