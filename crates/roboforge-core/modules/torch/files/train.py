"""Small PyTorch training loop to confirm the environment and GPU work."""
import torch
from torch import nn


def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f'torch {torch.__version__} on {device}')
    torch.manual_seed(0)
    x = torch.linspace(-3, 3, 1024, device=device).unsqueeze(1)
    y = torch.sin(2 * x) + 0.1 * torch.randn_like(x)

    model = nn.Sequential(nn.Linear(1, 64), nn.Tanh(), nn.Linear(64, 64), nn.Tanh(), nn.Linear(64, 1)).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=1e-2)
    for step in range(501):
        loss = nn.functional.mse_loss(model(x), y)
        opt.zero_grad()
        loss.backward()
        opt.step()
        if step % 100 == 0:
            print(f'step {step:4d}  loss {loss.item():.5f}')


if __name__ == '__main__':
    main()
