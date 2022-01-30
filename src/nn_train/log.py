# 1
class MyModel(nn.Module):
    def __init__(self, dev, input_size = 48, output_size = 6):
        super().__init__()
        self.dev = dev
        self.linear_1 = nn.Linear(in_features=input_size, out_features=50)
        self.linear_2 = nn.Linear(50, 100)
        self.linear_3 = nn.Linear(100, 50)
        # self.linear_4 = nn.Linear(600, 400)
        # self.linear_5 = nn.Linear(400, 200)
        self.linear_6 = nn.Linear(50, output_size)

    def forward(self, x):
        x = self.linear_1(x)
        x = torch.tanh(x)
        x = self.linear_2(x)
        x = torch.tanh(x)
        x = self.linear_3(x)
        x = torch.tanh(x)
        # x = self.linear_4(x)
        # x = torch.tanh(x)
        # x = self.linear_5(x)
        # x = torch.tanh(x)
        return self.linear_6(x)

# 2
class MyModel(nn.Module):
    def __init__(self, dev, input_size = 48, output_size = 6):
        super().__init__()
        self.dev = dev
        self.linear_1 = nn.Linear(in_features=input_size, out_features=100)
        self.linear_2 = nn.Linear(100, 200)
        self.linear_3 = nn.Linear(200, 100)
        # self.linear_4 = nn.Linear(600, 400)
        # self.linear_5 = nn.Linear(400, 200)
        self.linear_6 = nn.Linear(100, output_size)

    def forward(self, x):
        x = self.linear_1(x)
        x = torch.tanh(x)
        x = self.linear_2(x)
        x = torch.tanh(x)
        x = self.linear_3(x)
        x = torch.tanh(x)
        # x = self.linear_4(x)
        # x = torch.tanh(x)
        # x = self.linear_5(x)
        # x = torch.tanh(x)
        return self.linear_6(x)

# 3

class MyModel(nn.Module):
    def __init__(self, dev, input_size = 48, output_size = 6):
        super().__init__()
        self.dev = dev
        self.linear_1 = nn.Linear(in_features=input_size, out_features=500)
        self.linear_2 = nn.Linear(500, 500)
        self.linear_3 = nn.Linear(500, 500)
        # self.linear_4 = nn.Linear(600, 400)
        # self.linear_5 = nn.Linear(400, 200)
        self.linear_6 = nn.Linear(200, output_size)

    def forward(self, x):
        x = self.linear_1(x)
        x = torch.tanh(x)
        x = self.linear_2(x)
        x = torch.tanh(x)
        # x = self.linear_3(x)
        # x = torch.tanh(x)
        # x = self.linear_4(x)
        # x = torch.tanh(x)
        # x = self.linear_5(x)
        # x = torch.tanh(x)
        return self.linear_6(x)


# 4
class MyModel(nn.Module):
    def __init__(self, dev, input_size = 48, output_size = 6):
        super().__init__()
        self.dev = dev
        self.linear_1 = nn.Linear(in_features=input_size, out_features=1000)
        self.linear_2 = nn.Linear(1000, 1000)
        # self.linear_3 = nn.Linear(500, 500)
        # self.linear_4 = nn.Linear(600, 400)
        # self.linear_5 = nn.Linear(400, 200)
        self.linear_6 = nn.Linear(1000, output_size)

    def forward(self, x):
        x = self.linear_1(x)
        x = torch.tanh(x)
        x = self.linear_2(x)
        x = torch.tanh(x)
        # x = self.linear_3(x)
        # x = torch.tanh(x)
        # x = self.linear_4(x)
        # x = torch.tanh(x)
        # x = self.linear_5(x)
        # x = torch.tanh(x)
        return self.linear_6(x)


        # 5
class MyModel(nn.Module):
    def __init__(self, dev, input_size = 48, output_size = 6):
        super().__init__()
        self.dev = dev
        self.linear_1 = nn.Linear(in_features=input_size, out_features=500)
        self.linear_2 = nn.Linear(500, 500)
        # self.linear_3 = nn.Linear(500, 500)
        # self.linear_4 = nn.Linear(600, 400)
        # self.linear_5 = nn.Linear(400, 200)
        self.linear_6 = nn.Linear(500, output_size)

    def forward(self, x):
        x = self.linear_1(x)
        x = torch.tanh(x)
        x = self.linear_2(x)
        x = torch.tanh(x)
        # x = self.linear_3(x)
        # x = torch.tanh(x)
        # x = self.linear_4(x)
        # x = torch.tanh(x)
        # x = self.linear_5(x)
        # x = torch.tanh(x)
        return self.linear_6(x)

        # 6
class MyModel(nn.Module):
    def __init__(self, dev, input_size = 48, output_size = 6):
        super().__init__()
        self.dev = dev
        self.linear_1 = nn.Linear(in_features=input_size, out_features=2000)
        self.linear_2 = nn.Linear(2000, 2000)
        # self.linear_3 = nn.Linear(500, 500)
        # self.linear_4 = nn.Linear(600, 400)
        # self.linear_5 = nn.Linear(400, 200)
        self.linear_6 = nn.Linear(2000, output_size)

    def forward(self, x):
        x = self.linear_1(x)
        x = torch.tanh(x)
        x = self.linear_2(x)
        x = torch.tanh(x)
        # x = self.linear_3(x)
        # x = torch.tanh(x)
        # x = self.linear_4(x)
        # x = torch.tanh(x)
        # x = self.linear_5(x)
        # x = torch.tanh(x)
        return self.linear_6(x)

        # 7
class MyModel(nn.Module):
    def __init__(self, dev, input_size = 48, output_size = 6):
        super().__init__()
        self.dev = dev
        self.linear_1 = nn.Linear(in_features=input_size, out_features=200)
        self.linear_2 = nn.Linear(200, 500)
        self.linear_3 = nn.Linear(500, 200)
        # self.linear_4 = nn.Linear(600, 400)
        # self.linear_5 = nn.Linear(400, 200)
        self.linear_6 = nn.Linear(200, output_size)

    def forward(self, x):
        x = self.linear_1(x)
        x = torch.tanh(x)
        x = self.linear_2(x)
        x = torch.tanh(x)
        x = self.linear_3(x)
        x = torch.tanh(x)
        # x = self.linear_4(x)
        # x = torch.tanh(x)
        # x = self.linear_5(x)
        # x = torch.tanh(x)
        return self.linear_6(x)        