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
    w1: [0 : x*h-1]
    b1: [x*h : (x+1)*h-1]
    w2: [(x+1)*h : (x+1+y)*h-1]
    b2: [(x+1+y)*h : (x+y+1)*h+y-1]

total length: (x+y+1)*h+y
"""


class Controller:
    def __init__(self, gene=None):
        x = 6
        h = 10
        y = 2
        self.gene = gene
        self.nn = NN(x, h, y)
        state_dict = self.nn.state_dict()
        w1 = [[gene[6 * i + j] for j in range(6)] for i in range(10)]
        b1 = [gene[60 + i] for i in range(10)]
        w2 = [[gene[70 + 10 * i + j] for j in range(10)] for i in range(2)]
        b2 = [gene[90 + i] for i in range(2)]
        state_dict['linear1.weight'] = torch.tensor(w1)
        state_dict['linear1.bias'] = torch.tensor(b1)
        state_dict['linear2.weight'] = torch.tensor(w2)
        state_dict['linear2.bias'] = torch.tensor(b2)
        self.nn.load_state_dict(state_dict, strict=False)
        # self.nn.linear1.weight = torch.tensor(w1)


    def construct(self):
        pass

if __name__ == '__main__':
    gene = [0.1] * 60 + [0.2] * 10 + [0.3] * 20 + [0.4] * 2
    c = Controller(gene)
    # print(c.nn.linear2.bias)
