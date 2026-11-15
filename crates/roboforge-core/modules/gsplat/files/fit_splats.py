"""Fit 3D Gaussians to one target image with gsplat (needs a CUDA GPU)."""
import math
from pathlib import Path

import torch
from gsplat import rasterization


def target_image(h, w, device):
    ys, xs = torch.meshgrid(torch.linspace(0, 1, h, device=device), torch.linspace(0, 1, w, device=device), indexing='ij')
    return torch.stack([xs, ys, 0.5 + 0.5 * torch.sin(12 * xs) * torch.cos(12 * ys)], dim=-1)


def main():
    if not torch.cuda.is_available():
        raise SystemExit('gsplat rasterisation needs CUDA')
    device = torch.device('cuda')
    torch.manual_seed(0)
    h = w = 256
    n = 4000
    gt = target_image(h, w, device)

    means = torch.nn.Parameter(torch.rand(n, 3, device=device) * 2 - 1)
    quats = torch.nn.Parameter(torch.randn(n, 4, device=device))
    log_scales = torch.nn.Parameter(torch.full((n, 3), math.log(0.05), device=device))
    logit_opac = torch.nn.Parameter(torch.zeros(n, device=device))
    colors = torch.nn.Parameter(torch.rand(n, 3, device=device))

    focal = 0.5 * w / math.tan(0.5 * math.radians(60))
    K = torch.tensor([[focal, 0, w / 2], [0, focal, h / 2], [0, 0, 1]], device=device)[None]
    viewmat = torch.eye(4, device=device)
    viewmat[2, 3] = 3.0  # camera 3 units back along z
    viewmat = viewmat[None]

    opt = torch.optim.Adam([means, quats, log_scales, logit_opac, colors], lr=1e-2)
    for step in range(1001):
        render, _, _ = rasterization(
            means, quats / quats.norm(dim=-1, keepdim=True), log_scales.exp(),
            torch.sigmoid(logit_opac), torch.sigmoid(colors), viewmat, K, w, h,
        )
        loss = torch.nn.functional.mse_loss(render[0], gt)
        opt.zero_grad()
        loss.backward()
        opt.step()
        if step % 100 == 0:
            print(f'step {step:4d}  mse {loss.item():.5f}')

    out = Path('outputs')
    out.mkdir(exist_ok=True)
    try:
        from torchvision.utils import save_image
        save_image(render[0].permute(2, 0, 1).clamp(0, 1).detach().cpu(), out / 'splats.png')
        print(f'wrote {out / "splats.png"}')
    except ImportError:
        pass


if __name__ == '__main__':
    main()
