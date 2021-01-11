import torch

class NN(torch.nn.Module):
    def __init__(self, num_input, num_hidden, num_output):
        super().__init__()
        self.linear1 = torch.nn.Linear(num_input, num_hidden)
        self.linear2 = torch.nn.Linear(num_hidden, num_output)

    def forward(self, x):
        h = self.linear1(x)
        a = torch.tanh(h)
        y = self.linear2(a)
        return y

"""
Gene encoding:
num_input = x, num_hidden = h, num_output = y
linear 1: w1 (h, x); b1 (h)
linear 2: w2 (y, h); b2 (y)
gene: 
    w1: [0 : x*h]
    b1: [x*h+1 : (x+1)*h]
    w2: [(x+1)*h+1 : (x+1+y)*h]
    b2: [(x+1+y)*h+1 : (x+y+1)*h+y]

total length: (x+y+1)*h+y
"""


class Controller:
    def __init__(self, gene=None):
        self.gene = gene
        self.nn = NN(6, 10, 2)
        for i in range(10):
            for j in range(6):
                self.nn.linear1.weight[i][j] = gene[6 * i + j]


    def construct(self):
        pass

if __name__ == '__main__':
    gene = [0.5] * 92
    n = NN(6, 10, 2)
    c = Controller(gene)
    print(n.linear1.weight)
    # print(c.nn.linear2.bias)
