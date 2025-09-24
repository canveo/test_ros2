import torch
import torch.nn as nn
import torch.nn.functional as F

class PilotNet(nn.Module):
    def __init__(self, image_shape, num_labels, dropout_rate=0.3):
        super().__init__()
        C, H, W = image_shape

        self.ln_1 = nn.BatchNorm2d(3, eps=1e-3)
        self.cn_1 = nn.Conv2d(3, 24, kernel_size=5, stride=2)
        self.cn_2 = nn.Conv2d(24, 36, kernel_size=5, stride=2)
        self.cn_3 = nn.Conv2d(36, 48, kernel_size=5, stride=2)
        self.cn_4 = nn.Conv2d(48, 64, kernel_size=3, stride=1)
        self.cn_5 = nn.Conv2d(64, 64, kernel_size=3, stride=1)

        # Para (3×66×200) → tras las 5 convs queda (64×1×18) → flatten_dim=64*1*18=1152
        flatten_dim = 64 * 1 * 18

        self.flatten = nn.Flatten()
        self.fc_1 = nn.Linear(flatten_dim, 1164)
        self.dropout1 = nn.Dropout(dropout_rate)
        self.fc_2 = nn.Linear(1164, 100)
        self.dropout2 = nn.Dropout(dropout_rate)
        self.fc_3 = nn.Linear(100, 50)
        self.dropout3 = nn.Dropout(dropout_rate)
        self.fc_4 = nn.Linear(50, 10)
        self.dropout4 = nn.Dropout(dropout_rate)
        self.fc_5 = nn.Linear(10, num_labels)

    def forward(self, x):
        # <-- Se asume: x ya es (N, C, H, W) -->
        x = self.ln_1(x)
        x = F.relu(self.cn_1(x))
        x = F.relu(self.cn_2(x))
        x = F.relu(self.cn_3(x))
        x = F.relu(self.cn_4(x))
        x = F.relu(self.cn_5(x))

        x = self.flatten(x)
        x = F.relu(self.fc_1(x)); x = self.dropout1(x)
        x = F.relu(self.fc_2(x)); x = self.dropout2(x)
        x = F.relu(self.fc_3(x)); x = self.dropout3(x)
        x = F.relu(self.fc_4(x)); x = self.dropout4(x)
        x = self.fc_5(x)
        return x