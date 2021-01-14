import torch
import numpy as np

class NN(torch.nn.Module):
    def __init__(self, num_input, num_hidden, num_output):
        super().__init__()
        self.linear1 = torch.nn.Linear(num_input, num_hidden)
        self.linear2 = torch.nn.Linear(num_hidden, num_output)

    def forward(self, x):
        x = np.array(x, dtype=np.float32)
        x = torch.tensor(x,)
        h = torch.tanh(self.linear1(x))
        y = torch.tanh(self.linear2(h))
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
    def __init__(self, gene):
        x = 8
        h = 2
        y = 2
        self.nn = NN(x, h, y)
        state_dict = self.nn.state_dict()
        w1 = [[gene[x * i + j] for j in range(x)] for i in range(h)]
        b1 = [gene[x * h + i] for i in range(h)]
        w2 = [[gene[(x + 1) * h + h * i + j] for j in range(h)] for i in range(y)]
        b2 = [gene[(x + 1 + y) * h + i] for i in range(y)]
        state_dict['linear1.weight'] = torch.tensor(w1)
        state_dict['linear1.bias'] = torch.tensor(b1)
        state_dict['linear2.weight'] = torch.tensor(w2)
        state_dict['linear2.bias'] = torch.tensor(b2)
        self.nn.load_state_dict(state_dict, strict=False)

    def act(self, inputs):
        assert len(inputs) == self.nn.linear1.weight.shape[1]
        inputs = [inputs[i] * 5 if inputs[i] is not False else 1 for i in range(len(inputs))]

        print(inputs)
        outputs = self.nn.forward(inputs)
        left = outputs[0] * 50
        right = outputs[1] * 50
        return left, right

if __name__ == '__main__':
    gene = [0.1] * 80 + [0.2] * 10 + [0.3] * 20 + [0.4] * 2
    c = Controller(gene)
    print(c.act([0] * 8))
    # print(c.nn.linear2.bias)
